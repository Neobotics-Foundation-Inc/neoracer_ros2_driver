#!/usr/bin/env python3
"""
Trace a taped lane track from one photo into a lane map (JSON).

    python3 trace_track_photo.py PHOTO OUT.json --corners FL FR NR NL
            [--tile 0.6096] [--tiles-across 8.075] [--aspect 1.061] [--name NAME]

The track is tape on a rectangular mat of square foam tiles: white tape
bounds the roads, a dashed yellow line runs down the middle of each two-way
road, and red tape marks the stop lines. The photo needs the mat's four
corners in pixels (far-left, far-right, near-right, near-left, as x,y; a
corner outside the frame is the crossing of the two edges that meet there).

The mat is straightened into a top view, the tape picked out by colour and
thinned to its centre line, and the roads and their lanes built from it:

- each dashed yellow line is a road's centre; a closed one is a loop, an open
  one a road whose ends meet another road at a T junction (where its centre,
  run on along its end, meets the other's);
- each road has two lanes, one each way, half a lane's width to the right of
  the centre for the way it goes (right-hand traffic). A lane is as wide as
  from the yellow line to the nearest white line;
- a lane into a junction ends at its stop line (or, with none, where the
  junction's other road begins); a lane out of it starts as far out as the
  lane into it from the same road ends;
- through a junction, each lane in joins each lane out but its own road's
  other way (no U-turns) by a smooth curve (a cubic Bezier, its ends along
  the two lanes).

Sizes come from the tiles: --tiles-across is the mat's width in tiles,
--tile one tile's side in metres, --aspect the mat's length over its width.
All three are measured from the photo but the tile's side, which is ASSUMED
until someone measures it; the map records which values were assumed.

The map's frame: metres, x along the mat's far edge (left to right as the
photo looks at it), y from the near edge to the far edge, origin at the
near-left corner, z up.
"""

import argparse
import json
import math

import cv2
import numpy as np
from skimage.morphology import skeletonize

R = 0.005            # m per pixel of the top view
SAMPLE = 0.02        # m between the points of every polyline written
SMOOTH = 0.12        # m: the window the traced lines are smoothed over
SPUR = 0.08          # m: thinning's side branches shorter than this are dropped
# m: the farthest apart two dashes of one line may lie (a stop line's tape over
# the line, a dash missing); two lines' dashes lie further apart than this
CHAIN = 0.35
TRIM_DEFAULT = 0.45  # m: a lane's end from a junction's centre, with no stop line on it


def rectify(photo, corners, width, length):
    """Return the metric top view, its valid-pixel mask and the homography."""
    nx, ny = int(round(width / R)), int(round(length / R))
    src = np.float32(corners)
    dst = np.float32([[0, 0], [nx, 0], [nx, ny], [0, ny]])
    hom = cv2.getPerspectiveTransform(src, dst)
    top = cv2.warpPerspective(photo, hom, (nx, ny), flags=cv2.INTER_CUBIC)
    valid = cv2.warpPerspective(np.full(photo.shape[:2], 255, np.uint8), hom, (nx, ny)) > 0
    return top, valid, hom


def masks(top, valid):
    """White, yellow and red tape, by colour."""
    hsv = cv2.cvtColor(top, cv2.COLOR_BGR2HSV)
    h, s, v = (hsv[..., i].astype(int) for i in range(3))
    yellow = (h >= 18) & (h <= 38) & (s > 90) & (v > 110) & valid
    red = ((h <= 12) | (h >= 165)) & (s > 110) & (v > 110) & valid
    white = (s < 45) & (v > 150) & valid & ~yellow & ~red

    def clean(m, k):
        m = cv2.morphologyEx(m.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        disc = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        return cv2.morphologyEx(m, cv2.MORPH_CLOSE, disc) > 0
    return clean(white, 5), clean(yellow, 3), clean(red, 5)


def chain_dashes(yellow, length):
    """Link a dashed line's dashes in order: [(points, closed)], dash length, dash width.

    The yellow is thinned to its middle and sampled every SAMPLE m (where the
    photo is far and blurred, dashes run together into one strip, so each blob
    gives points along its length, not one centre). The points within CHAIN m
    of each other are joined by their minimum spanning tree: along a dashed
    line that is a path, with short side branches where a dash split in two.
    Each tree's longest path is its line in order; a line whose ends lie
    within CHAIN m of each other is a loop. The dash sizes come from the blobs
    short enough to be one dash.
    """
    n, lab, stats, _ = cv2.connectedComponentsWithStats(yellow.astype(np.uint8))
    sizes = []
    for k in range(1, n):
        ys, xs = np.nonzero(lab == k)
        pts = np.column_stack([(xs + 0.5) * R, length - (ys + 0.5) * R]).astype(np.float32)
        _, (w, h), _ = cv2.minAreaRect(pts)
        if 0.02 < max(w, h) < 0.1:
            sizes.append((max(w, h), min(w, h)))
    ys, xs = np.nonzero(skeletonize(yellow))
    c = np.column_stack([(xs + 0.5) * R, length - (ys + 0.5) * R])
    _, first = np.unique(np.round(c / SAMPLE).astype(int), axis=0, return_index=True)
    c = c[np.sort(first)]
    from scipy.sparse.csgraph import connected_components, dijkstra, minimum_spanning_tree
    from scipy.spatial import cKDTree
    near = cKDTree(c).sparse_distance_matrix(cKDTree(c), CHAIN).tocsr()
    tree = minimum_spanning_tree(near)
    tree = tree + tree.T
    count, label = connected_components(tree, directed=False)
    lines, gaps = [], []
    for k in range(count):
        members = np.nonzero(label == k)[0]
        if len(members) < 4:
            continue
        d = dijkstra(tree, indices=members[0])
        a = members[np.argmax(np.where(np.isfinite(d[members]), d[members], -1))]
        d, pred = dijkstra(tree, indices=a, return_predecessors=True)
        b = members[np.argmax(np.where(np.isfinite(d[members]), d[members], -1))]
        path = [b]
        while path[-1] != a:
            path.append(pred[path[-1]])
        path = path[::-1]
        closed = float(np.hypot(*(c[path[0]] - c[path[-1]]))) <= CHAIN
        lines.append((c[path], closed))
        q = c[path + ([path[0]] if closed else [])]
        gaps.extend(np.hypot(*np.diff(q, axis=0).T))
    sizes = np.array(sizes)
    return lines, float(np.median(sizes[:, 0])), float(np.median(sizes[:, 1]))


NEIGH = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


def trace(mask):
    """Thin a mask to its centre lines: a list of (pixel points, closed), longest first."""
    sk = skeletonize(mask)
    pts = set(zip(*np.nonzero(sk)))

    def nb(p):
        return [(p[0] + a, p[1] + b) for a, b in NEIGH if (p[0] + a, p[1] + b) in pts]

    # drop short side branches, from their free end to where they join
    spur = int(SPUR / R)
    changed = True
    while changed:
        changed = False
        for e in [p for p in pts if len(nb(p)) == 1]:
            path, cur, prev = [e], e, None
            while True:
                nxt = [q for q in nb(cur) if q != prev and q not in path]
                if len(nxt) != 1 or len(nb(nxt[0])) > 2:
                    break
                prev, cur = cur, nxt[0]
                path.append(cur)
            if len(path) < spur and any(len(nb(q)) > 2 for q in nb(path[-1]) + [path[-1]]):
                pts.difference_update(path)
                changed = True
    lines = []
    seen = set()
    for start in sorted(pts):
        if start in seen:
            continue
        # this component; walk from an end, or anywhere on a loop
        comp, stack = {start}, [start]
        while stack:
            for q in nb(stack.pop()):
                if q not in comp:
                    comp.add(q)
                    stack.append(q)
        seen |= comp
        ends = [p for p in comp if len(nb(p)) == 1]
        cur = ends[0] if ends else start
        path, used = [cur], {cur}
        while True:
            nxt = [q for q in nb(cur) if q not in used]
            if not nxt:
                break
            # 4-neighbours first: a thinned line's steps
            nxt.sort(key=lambda q: abs(q[0] - cur[0]) + abs(q[1] - cur[1]))
            cur = nxt[0]
            path.append(cur)
            used.add(cur)
        closed = not ends and len(path) > 20
        if len(path) * R > 0.2:
            lines.append((path, closed))
    lines.sort(key=lambda line: -len(line[0]))
    return lines


def to_map(path, length):
    """Pixel (row, column) points into the map frame (x right, y from the near edge)."""
    return np.array([[(c + 0.5) * R, length - (r + 0.5) * R] for r, c in path])


def resample(points, closed, step=SAMPLE):
    """Points every `step` m along a polyline (a loop wraps)."""
    p = np.vstack([points, points[:1]]) if closed else points
    seg = np.hypot(*np.diff(p, axis=0).T)
    s = np.concatenate([[0], np.cumsum(seg)])
    n = max(int(s[-1] / step), 2)
    t = np.linspace(0, s[-1], n, endpoint=not closed)
    return np.column_stack([np.interp(t, s, p[:, 0]), np.interp(t, s, p[:, 1])])


def smooth(points, closed, window=SMOOTH):
    """Average over `window` m (a loop wraps; an open line keeps its ends)."""
    k = max(int(window / SAMPLE) // 2, 1)
    if closed:
        ext = np.vstack([points[-k:], points, points[:k]])
        out = np.array([ext[i:i + 2 * k + 1].mean(0) for i in range(len(points))])
    else:
        out = points.copy()
        for i in range(len(points)):
            a, b = max(0, i - k), min(len(points), i + k + 1)
            w = min(i - a, b - 1 - i)
            out[i] = points[i - w:i + w + 1].mean(0)
    return out


def polyline(points, closed):
    return smooth(resample(points, closed), closed)


def tangents(p, closed):
    d = np.roll(p, -1, 0) - np.roll(p, 1, 0) if closed else np.gradient(p, axis=0)
    return d / np.hypot(d[:, 0], d[:, 1])[:, None]


def length_of(p, closed=False):
    q = np.vstack([p, p[:1]]) if closed else p
    return float(np.hypot(*np.diff(q, axis=0).T).sum())


def nearest(p, pts):
    d = np.hypot(pts[:, 0] - p[0], pts[:, 1] - p[1])
    i = int(np.argmin(d))
    return i, float(d[i])


def ray_hit(origin, direction, loop):
    """Where a ray first crosses a closed polyline: (point, index of the segment)."""
    best = None
    q = np.vstack([loop, loop[:1]])
    for i in range(len(loop)):
        a, b = q[i], q[i + 1]
        m = np.array([[direction[0], a[0] - b[0]], [direction[1], a[1] - b[1]]])
        if abs(np.linalg.det(m)) < 1e-12:
            continue
        t, u = np.linalg.solve(m, a - origin)
        if t > 0 and 0 <= u <= 1 and (best is None or t < best[0]):
            best = (t, a + u * (b - a), i)
    if best is None:
        raise ValueError("an open road's end does not run into the loop")
    return best[1], best[2]


def offset(p, closed, d):
    """Move a polyline d m to the right of its way."""
    t = tangents(p, closed)
    return p + d * np.column_stack([t[:, 1], -t[:, 0]])


def bezier(p0, t0, p3, t3, step=SAMPLE):
    """Return a cubic Bezier from p0 leaving along t0 to p3 arriving along t3."""
    chord = float(np.hypot(*(p3 - p0)))
    h = 0.4 * chord
    p1, p2 = p0 + h * t0, p3 - h * t3
    n = max(int(chord / step * 1.3), 4)
    u = np.linspace(0, 1, n)[:, None]
    return (1 - u) ** 3 * p0 + 3 * (1 - u) ** 2 * u * p1 + 3 * (1 - u) * u ** 2 * p2 + u ** 3 * p3


def segment_hit(a, b, p, q):
    """Where segment ab crosses the polyline p (open): (point, index) or None."""
    for i in range(len(p) - 1):
        c, d = p[i], p[i + 1]
        m = np.array([[b[0] - a[0], c[0] - d[0]], [b[1] - a[1], c[1] - d[1]]])
        if abs(np.linalg.det(m)) < 1e-12:
            continue
        t, u = np.linalg.solve(m, c - a)
        if -0.15 <= t <= 1.15 and 0 <= u <= 1:
            return c + u * (d - c), i
    return None


def build(args):
    photo = cv2.imread(args.photo)
    if photo is None:
        raise SystemExit('cannot read ' + args.photo)
    width = args.tiles_across * args.tile
    length = args.aspect * width
    top, valid, hom = rectify(photo, args.corners, width, length)
    white, yellow, red = masks(top, valid)
    # colours, as the photo shows them: each tape's median, and the mat's away from tape
    taped = cv2.dilate((white | yellow | red).astype(np.uint8), np.ones((15, 15))).astype(bool)
    bare = valid & ~taped
    look = {}
    for name, where in (('mat', bare), ('white', white), ('yellow', yellow), ('red', red)):
        b, g, r = np.median(top[where], axis=0)
        look[name] = '#%02x%02x%02x' % (int(r), int(g), int(b))

    # white tape: thinned to its middle; its width is its area over that length
    white_lines = [(polyline(to_map(p, length), c), c) for p, c in trace(white)]
    white_len = sum(length_of(p, c) for p, c in white_lines)
    tape_white = float(white.sum() * R * R / white_len)
    # yellow: the dashes' centres, linked in order
    chains, dash_length, tape_yellow = chain_dashes(yellow, length)
    yellow_lines = [(polyline(p, c), c) for p, c in chains]
    # a dash and the gap after it: a dash over the share of the line the yellow covers
    yellow_len = sum(length_of(p, c) for p, c in yellow_lines)
    covered = float(yellow.sum() * R * R / tape_yellow) / yellow_len
    dash_period = dash_length / covered

    loops = [p for p, c in yellow_lines if c]
    opens = [p for p, c in yellow_lines if not c]
    if len(loops) != 1:
        raise SystemExit('expected one closed yellow loop, found %d' % len(loops))
    loop = loops[0]
    # the loop runs anticlockwise seen from above
    x, y = loop[:, 0], loop[:, 1]
    area = 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))
    if area < 0:
        loop = loop[::-1].copy()

    # lane width: yellow to the nearest white, along the loop
    whites = np.vstack([p for p, _ in white_lines])
    gaps = [nearest(q, whites)[1] for q in loop[::5]]
    lane_width = float(np.median(gaps)) - tape_white / 2

    # junctions: each open road's ends run on into the loop
    junctions = []
    connector_roads = []
    for road in opens:
        ends = []
        for end, inner in ((road[0], road[3]), (road[-1], road[-4])):
            way = (end - inner) / np.hypot(*(end - inner))
            hit, seg = ray_hit(end, way, loop)
            ends.append((hit, seg))
        n0 = max(int(np.hypot(*(road[0] - ends[0][0])) / SAMPLE), 2)
        n1 = max(int(np.hypot(*(road[-1] - ends[1][0])) / SAMPLE), 2)
        ext0 = np.linspace(ends[0][0], road[0], n0, endpoint=False)
        ext1 = np.linspace(road[-1], ends[1][0], n1)[1:]
        connector_roads.append(np.vstack([ext0, road, ext1]))
        for hit, seg in ends:
            junctions.append({'at': hit, 'seg': seg})
    if len(junctions) < 2:
        raise SystemExit('expected an open road with both ends on the loop')

    # the loop cut at the junctions into roads, each from one junction to the next
    for j in junctions:
        j['i'] = nearest(j['at'], loop)[0]
    junctions.sort(key=lambda j: j['i'])
    roads = []
    for k, j in enumerate(junctions):
        nxt = junctions[(k + 1) % len(junctions)]
        if nxt['i'] > j['i']:
            idx = list(range(j['i'], nxt['i'] + 1))
        else:
            idx = list(range(j['i'], len(loop))) + list(range(0, nxt['i'] + 1))
        roads.append({'id': 'loop%d' % (k + 1), 'points': loop[idx],
                      'from': k, 'to': (k + 1) % len(junctions)})
    for k, c in enumerate(connector_roads):
        a = min(range(len(junctions)), key=lambda n: np.hypot(*(junctions[n]['at'] - c[0])))
        b = min(range(len(junctions)), key=lambda n: np.hypot(*(junctions[n]['at'] - c[-1])))
        roads.append({'id': 'link%d' % (k + 1), 'points': c, 'from': a, 'to': b})

    # stop lines
    n_red, lab = cv2.connectedComponents(red.astype(np.uint8))
    stops = []
    for k in range(1, n_red):
        ys, xs = np.nonzero(lab == k)
        if len(xs) * R * R < 0.002:
            continue
        pts = np.column_stack([(xs + 0.5) * R, length - (ys + 0.5) * R]).astype(np.float32)
        (cx, cy), (w, h), ang = cv2.minAreaRect(pts)
        long_side, short_side = max(w, h), min(w, h)
        a = math.radians(ang if w >= h else ang + 90)
        d = np.array([math.cos(a), math.sin(a)]) * long_side / 2
        stops.append({'a': [cx - d[0], cy - d[1]], 'b': [cx + d[0], cy + d[1]],
                      'width': short_side})

    # lanes: two per road, half a lane to the right of the centre for the way each goes
    half = lane_width / 2
    lanes = []
    for road in roads:
        p = road['points']
        ways = (('fwd', offset(p, False, half), road['from'], road['to']),
                ('back', offset(p[::-1].copy(), False, half), road['to'], road['from']))
        for way, pts, j0, j1 in ways:
            lanes.append({'id': road['id'] + '_' + way, 'road': road['id'],
                          'from': j0, 'to': j1, 'raw': pts})

    # trim each lane at its junctions: into one at its stop line, out of one as far
    # out as the same road's lane in ends
    stop_used = [None] * len(stops)
    for lane in lanes:
        pts = lane['raw']
        end_cut = None
        for k, st in enumerate(stops):
            hit = segment_hit(np.array(st['a']), np.array(st['b']), pts, None)
            if hit is not None and np.hypot(*(hit[0] - junctions[lane['to']]['at'])) < 1.2:
                end_cut = hit
                stop_used[k] = lane['id']
                lane['stop'] = k
        jt = junctions[lane['to']]['at']
        if end_cut is not None:
            lane['end_radius'] = float(np.hypot(*(end_cut[0] - jt)))
        else:
            lane['end_radius'] = TRIM_DEFAULT
    for lane in lanes:
        twin = next(o for o in lanes if o['road'] == lane['road'] and o is not lane)
        lane['start_radius'] = twin['end_radius']
    for lane in lanes:
        pts = lane['raw']
        j0, j1 = junctions[lane['from']]['at'], junctions[lane['to']]['at']
        keep = [i for i, q in enumerate(pts)
                if np.hypot(*(q - j0)) >= lane['start_radius']
                and np.hypot(*(q - j1)) >= lane['end_radius']]
        # one run: from the first point clear of the start to the first point into the end
        a = keep[0]
        b = a
        while b + 1 < len(pts) and (b + 1) in keep:
            b += 1
        lane['points'] = resample(pts[a:b + 1], False)

    # through each junction: every lane in to every lane out but its own road's other way
    connections = []
    for j in range(len(junctions)):
        ins = [lane for lane in lanes if lane['to'] == j]
        outs = [lane for lane in lanes if lane['from'] == j]
        for li in ins:
            for lo in outs:
                if lo['road'] == li['road']:
                    continue
                p0, p3 = li['points'][-1], lo['points'][0]
                t0 = tangents(li['points'], False)[-1]
                t3 = tangents(lo['points'], False)[0]
                turn = math.degrees(math.atan2(t0[0] * t3[1] - t0[1] * t3[0], t0 @ t3))
                kind = 'straight' if abs(turn) < 30 else ('left' if turn > 0 else 'right')
                cid = 'j%d_%s_to_%s' % (j + 1, li['id'], lo['id'])
                connections.append({'id': cid, 'junction': j, 'in': li['id'], 'out': lo['id'],
                                    'turn': kind, 'points': bezier(p0, t0, p3, t3)})

    out_lanes = []
    for lane in lanes:
        out_lanes.append({'id': lane['id'], 'road': lane['road'], 'kind': 'lane',
                          'stop_line': lane.get('stop'),
                          'next': [c['id'] for c in connections if c['in'] == lane['id']],
                          'points': np.round(lane['points'], 4).tolist()})
    for c in connections:
        out_lanes.append({'id': c['id'], 'road': None, 'kind': 'junction', 'turn': c['turn'],
                          'stop_line': None, 'next': [c['out']],
                          'points': np.round(c['points'], 4).tolist()})
    for lane in out_lanes:
        lane['length'] = round(length_of(np.array(lane['points'])), 4)

    m = {
        'name': args.name,
        'units': 'm',
        'frame': 'x along the far edge (left to right in the photo), y from the near edge to '
                 'the far edge, origin at the near-left corner of the mat, z up',
        'source': {
            'photo': args.photo.replace('\\', '/').split('/')[-1],
            'corners_px': args.corners,
            'tile_m': args.tile,
            'tiles_across': args.tiles_across,
            'aspect': args.aspect,
            'assumed': ['tile_m'] + args.assumed,
            'note': 'Traced from one photo. Every size scales with tile_m, which is assumed (a '
                    '24 in foam tile) until measured; the shape and proportions are measured '
                    'from the photo.',
        },
        'mat': {'width': round(width, 4), 'length': round(length, 4)},
        'traffic': 'right-hand, two-way roads, three-way stops',
        'tape': {'white_width': round(tape_white, 4), 'yellow_width': tape_yellow,
                 'dash_length': round(dash_length, 4), 'dash_period': round(dash_period, 4),
                 'red_width': round(float(np.median([s['width'] for s in stops])), 4)},
        'lane_width': round(lane_width, 4),
        'look': look,
        'white_lines': [{'closed': bool(c), 'points': np.round(p, 4).tolist()}
                        for p, c in white_lines],
        'yellow_lines': [{'closed': bool(c), 'points': np.round(p, 4).tolist()}
                         for p, c in yellow_lines],
        'stop_lines': [{'a': np.round(s['a'], 4).tolist(), 'b': np.round(s['b'], 4).tolist(),
                        'width': round(float(s['width']), 4), 'lane': stop_used[k]}
                       for k, s in enumerate(stops)],
        'junctions': [{'id': k, 'at': np.round(j['at'], 4).tolist()}
                      for k, j in enumerate(junctions)],
        'roads': [{'id': r['id'], 'from': r['from'], 'to': r['to'],
                   'centre': np.round(r['points'], 4).tolist()} for r in roads],
        'lanes': out_lanes,
    }
    return m, top, hom, length


def scaled(m, f, why):
    """Return the map with every length times f (the shape and proportions stay)."""
    def pts(p):
        return [[round(v * f, 4) for v in q] for q in p]
    out = json.loads(json.dumps(m))
    out['mat'] = {k: round(v * f, 4) for k, v in m['mat'].items()}
    out['tape'] = {k: round(v * f, 4) for k, v in m['tape'].items()}
    out['lane_width'] = round(m['lane_width'] * f, 4)
    for key in ('white_lines', 'yellow_lines'):
        for line in out[key]:
            line['points'] = pts(line['points'])
    for s in out['stop_lines']:
        s['a'], s['b'], s['width'] = pts([s['a']])[0], pts([s['b']])[0], round(s['width'] * f, 4)
    for j in out['junctions']:
        j['at'] = pts([j['at']])[0]
    for r in out['roads']:
        r['centre'] = pts(r['centre'])
    for lane in out['lanes']:
        lane['points'] = pts(lane['points'])
        lane['length'] = round(lane['length'] * f, 4)
    src = out['source']
    src['tile_m'] = round(src['tile_m'] * f, 4)
    src['assumed'] = ['mat_length']
    src['scale'] = why
    src['note'] = ('Traced from one photo: the shape and proportions are measured from it; '
                   'the scale is set by mat_length, which is assumed until someone measures '
                   'the track.')
    return out


def overlay(m, top, hom, length, photo_path, out_prefix):
    """Draw the map on the top view and back onto the photo, to check it by eye."""
    def px(p):
        return (int(round(p[0] / R)), int(round((length - p[1]) / R)))

    def poly(img, points, closed, colour, width):
        cv2.polylines(img, [np.array(points)], closed, colour, width)

    vis = (top * 0.45).astype(np.uint8)
    for w in m['white_lines']:
        poly(vis, [px(p) for p in w['points']], w['closed'], (255, 255, 255), 2)
    for y in m['yellow_lines']:
        poly(vis, [px(p) for p in y['points']], y['closed'], (0, 220, 255), 2)
    for s in m['stop_lines']:
        cv2.line(vis, px(s['a']), px(s['b']), (0, 0, 255), 4)
    colours = [(80, 255, 80), (255, 140, 40), (255, 80, 255), (40, 200, 255), (200, 200, 80),
               (120, 120, 255)]
    lanes = [x for x in m['lanes'] if x['kind'] == 'lane']
    for k, lane in enumerate(lanes):
        colour = colours[k % len(colours)]
        pts = [px(p) for p in lane['points']]
        poly(vis, pts, False, colour, 2)
        mid, nxt = pts[len(pts) // 2], pts[min(len(pts) // 2 + 6, len(pts) - 1)]
        cv2.arrowedLine(vis, mid, nxt, colour, 3, tipLength=0.6)
    for lane in m['lanes']:
        if lane['kind'] == 'junction':
            poly(vis, [px(p) for p in lane['points']], False, (180, 180, 180), 1)
    for j in m['junctions']:
        cv2.circle(vis, px(j['at']), 6, (0, 255, 255), -1)
    cv2.imwrite(out_prefix + '_top.png', vis)
    inverse = np.linalg.inv(hom)
    back = cv2.imread(photo_path)

    def ph(p):
        q = inverse @ np.array([p[0] / R, (length - p[1]) / R, 1.0])
        return (int(q[0] / q[2]), int(q[1] / q[2]))

    for lane in lanes:
        poly(back, [ph(p) for p in lane['points']], False, (255, 0, 255), 8)
    for y in m['yellow_lines']:
        poly(back, [ph(p) for p in y['points']], y['closed'], (255, 120, 0), 5)
    small = cv2.resize(back, (back.shape[1] // 3, back.shape[0] // 3))
    cv2.imwrite(out_prefix + '_photo.jpg', small)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('photo')
    ap.add_argument('out')
    ap.add_argument('--corners', nargs=4, required=True, metavar='X,Y',
                    type=lambda t: [float(v) for v in t.split(',')])
    ap.add_argument('--tile', type=float, default=0.6096,
                    help="a tile's side, m (assumed: 24 in)")
    ap.add_argument('--tiles-across', type=float, required=True,
                    help="the mat's width in tiles")
    ap.add_argument('--aspect', type=float, required=True,
                    help="the mat's length over its width")
    ap.add_argument('--assumed', nargs='*', default=[],
                    help='other values that are assumed, not measured')
    ap.add_argument('--mat-length', type=float,
                    help="scale the map so the mat's longer side is this many metres "
                         '(overrides --tile)')
    ap.add_argument('--scale-note', default='', help='where --mat-length comes from')
    ap.add_argument('--name', default='track')
    ap.add_argument('--overlay', help='write check images with this path prefix')
    args = ap.parse_args()
    m, top, hom, length = build(args)
    if args.overlay:
        overlay(m, top, hom, length, args.photo, args.overlay)
    if args.mat_length is not None:
        m = scaled(m, args.mat_length / max(m['mat'].values()), args.scale_note)
    elif args.scale_note:
        m['source']['scale'] = args.scale_note
    with open(args.out, 'w') as f:
        json.dump(m, f, indent=1)
    lanes = [lane for lane in m['lanes'] if lane['kind'] == 'lane']
    print('%s: mat %.2f x %.2f m, lane %.3f m, white tape %.3f m, dash %.3f m every %.3f m'
          % (m['name'], m['mat']['width'], m['mat']['length'], m['lane_width'],
             m['tape']['white_width'], m['tape']['dash_length'], m['tape']['dash_period']))
    print('%d white lines, %d yellow lines, %d stop lines, %d junctions, %d roads, %d lanes, '
          '%d junction paths'
          % (len(m['white_lines']), len(m['yellow_lines']), len(m['stop_lines']),
             len(m['junctions']), len(m['roads']), len(lanes), len(m['lanes']) - len(lanes)))
    for lane in lanes:
        print('  %-12s %.2f m, stop line %s, next %s'
              % (lane['id'], lane['length'], lane['stop_line'], lane['next']))


if __name__ == '__main__':
    main()
