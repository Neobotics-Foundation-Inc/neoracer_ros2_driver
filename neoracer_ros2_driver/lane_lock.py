"""
Lane lock: where a car is on a lane map, from its odometry alone.

A car's odometry (x, y and heading from the wheel encoder and the IMU) drifts:
the heading slowly, as the gyro's bias wanders, and so the position more and
more. Two parts of it stay good: the distance the car has driven (the
encoder) and how its heading changes over a few metres (the gyro, over a
short time). On a known lane map those are enough to tell where the car is:
how far along which lane, so it is always drawn on its lane, never beside it.

LaneLock is a particle filter over the lanes. Each particle is a guess: a
lane, how far along it (s, m), the gyro's bias (the odometry's heading less
the lane's, rad) and the scale from the odometry's metres to the map's. As
the car drives, every particle moves as far along its lane (and on into the
next, at a junction one of the ways on, at random) and is weighed by how well
the odometry's heading, less its bias, matches its lane's direction there.
Guesses on the wrong lane, or the right lane at the wrong place, stop
matching at the next bend or turn and die out; the bias and the scale follow
the car's drift slowly. Without a known start the particles begin spread over
every lane and find the car after a few bends (each bias set so the first
heading fits); with one, around it.

Pure Python (the viewer runs under the Python 3.8 AutoDRIVE's devkit pins).
"""

import bisect
import json
import math
import random


def wrap(a):
    """Return an angle in -pi..pi."""
    return math.atan2(math.sin(a), math.cos(a))


class Lane:
    """One directed lane of a map: a polyline, its length along it and what follows it."""

    def __init__(self, index, data):
        self.index = index
        self.id = data['id']
        self.kind = data['kind']
        self.points = [tuple(p) for p in data['points']]
        self.cum = [0.0]
        self.heads = []
        for (x0, y0), (x1, y1) in zip(self.points, self.points[1:]):
            self.cum.append(self.cum[-1] + math.hypot(x1 - x0, y1 - y0))
            self.heads.append(math.atan2(y1 - y0, x1 - x0))
        self.length = self.cum[-1]
        self.next_ids = list(data['next'])
        self.next = []
        self.prev = []

    def _at(self, s):
        """Return the segment holding s (clamped to the lane) and how far along it."""
        s = min(max(s, 0.0), self.length)
        i = min(max(bisect.bisect_right(self.cum, s) - 1, 0), len(self.heads) - 1)
        return i, s - self.cum[i]

    def heading(self, s):
        """Return the lane's direction at s (rad, anticlockwise from the map's x)."""
        return self.heads[self._at(s)[0]]

    def pose(self, s):
        """(x, y, heading) on the lane's centre at s."""
        i, d = self._at(s)
        (x0, y0), (x1, y1) = self.points[i], self.points[i + 1]
        seg = self.cum[i + 1] - self.cum[i]
        f = d / seg if seg > 0 else 0.0
        return x0 + f * (x1 - x0), y0 + f * (y1 - y0), self.heads[i]

    def nearest(self, x, y):
        """(s, distance) of the lane's point nearest to (x, y)."""
        best = (0.0, float('inf'))
        for i, ((x0, y0), (x1, y1)) in enumerate(zip(self.points, self.points[1:])):
            dx, dy = x1 - x0, y1 - y0
            seg2 = dx * dx + dy * dy
            f = 0.0 if seg2 == 0 else min(max(((x - x0) * dx + (y - y0) * dy) / seg2, 0.0), 1.0)
            d = math.hypot(x - (x0 + f * dx), y - (y0 + f * dy))
            if d < best[1]:
                best = (self.cum[i] + f * math.sqrt(seg2), d)
        return best


class LaneMap:
    """A lane map (scripts/lanes/*.json): its directed lanes, joined end to start."""

    def __init__(self, data):
        self.data = data
        self.lanes = [Lane(i, lane) for i, lane in enumerate(data['lanes'])]
        self.by_id = {lane.id: lane for lane in self.lanes}
        for lane in self.lanes:
            for nid in lane.next_ids:
                if nid not in self.by_id:
                    raise ValueError('lane %s leads to %s, which the map does not have'
                                     % (lane.id, nid))
                lane.next.append(self.by_id[nid])
                self.by_id[nid].prev.append(lane)
        self.total = sum(lane.length for lane in self.lanes)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls(json.load(f))

    def nearest(self, x, y, heading=None):
        """Return the lane, s and distance nearest to (x, y).

        Only lanes running within 60 degrees of `heading` count, if given.
        """
        best = None
        for lane in self.lanes:
            s, d = lane.nearest(x, y)
            if heading is not None and abs(wrap(lane.heading(s) - heading)) > math.radians(60):
                continue
            if best is None or d < best[2]:
                best = (lane, s, d)
        if best is None:
            raise ValueError('no lane runs that way near (%.2f, %.2f)' % (x, y))
        return best


# The filter's settings. Lengths are the map's metres.
# Particles while locked, and while searching the whole map (an unknown start,
# a lost lock: the car could be anywhere on any lane, at any scale).
PARTICLES = 2000
SEARCH_PARTICLES = 6000
# m the odometry drives between updates (a car standing still is not reweighed)
STEP = 0.05
# m: the odometry's heading is averaged over this much of its travel, the
# lane's taken half of it back (weaving in the lane and vibration average out)
WINDOW = 1.0
# rad: that heading off its lane's, less the bias, to start with; then
# learned from the best guess's own misfit (how much this car weaves and
# shakes), within SD_RANGE
HEADING_SD = 0.14
SD_RANGE = (0.08, 0.20)
# rad: a best guess misfitting this much for MISFIT_AFTER m is lost, however
# sure it is (a wrong lock widens the misfit; capped, SD cannot widen to fit it).
# A right lock misfits 0.06 to 0.17 (a car shaking hard); a wrong one, 0.4 up.
MISFIT_LOST = 0.30
MISFIT_AFTER = 4.0
# A search of the whole map locks only once the car has turned this much (rad):
# after one bend, many places fit (this track has several right-angle bends);
# the bends after it, and how far apart they lie, tell them apart.
SEARCH_TURN = math.pi
# ... and only once its best guess has fit the car's heading this well (rad of
# misfit), each new guess starting at MISFIT_LOST, so it proves itself over a
# metre or two first (a lookalike stretch fits for a bend, then drifts off)
PROBATION = 0.20
OUTLIER = 0.05        # the share of headings that may be anything (a bump, a swerve)
BIAS_DRIFT = 0.004    # rad per sqrt(s): how fast the gyro's bias may wander
MOVE_SD = 0.03        # of the distance moved: wheel slip
SCALE_SD = 0.002      # per sqrt(m): how fast the odometry-to-map scale may wander
SCALE_RANGE = (0.8, 1.25)   # the scale's prior, times the nominal one
ROUGHEN = (0.02, 0.005, 0.004)   # m, rad and scale: the nudge each resampled copy gets
START_SD = 0.3        # m of spread around a known start
CLUSTER = 1.0         # m: particles this near the best guess, running its way, back it
CONFIDENT = 0.6       # the share of the weight that makes the guess a lock
# A lock lost for this many metres of driving searches the whole map again, as
# an unknown start does (more than a search takes to find the car, so it never
# cuts one short). A lock is not dropped on every dip: at a junction the guesses
# split between the ways on, and any lane pointing the car's way fits it until
# the next turn, so starting guesses again at every dip draws the lock away.
LOST_AFTER = 30.0


class LaneLock:
    """Where one car is on a lane map, from its odometry poses as they come."""

    def __init__(self, lane_map, scale=1.0, start=None, particles=PARTICLES,
                 search=SEARCH_PARTICLES, seed=None):
        self.map = lane_map
        self.locked_n = particles     # particles while locked
        self.search = search          # and while searching the whole map
        self.n = particles
        self.scale = scale
        self.rng = random.Random(seed)
        self.start = start            # (lane id, s) or None: anywhere
        self.lane = []                # each particle's lane
        self.s = []
        self.bias = []
        self.k = []
        self.w = []
        self.last = None              # the odometry pose at the last update: (x, y, yaw, t)
        self.estimate = None
        self.driven = 0.0             # m of odometry since the start
        self.recent = []              # (odometry m driven, heading) over the last WINDOW m
        self.sd = HEADING_SD          # the heading misfit expected (learned, capped)
        self.misfit = HEADING_SD      # the best guess's misfit, uncapped
        self.lost_at = None           # m driven when the lock was lost, or None
        self.misfit_at = None         # m driven when the misfit grew past MISFIT_LOST, or None
        self.searching = start is None    # searching the whole map, not yet locked
        self.turned = 0.0             # rad the car has turned since the search began
        self.seen = None              # the last averaged odometry heading

    def _spawn(self, yaw):
        """Spread the particles out, each bias set so the first heading fits."""
        rng = self.rng
        self.n = self.search if self.start is None else self.locked_n
        self.searching = self.start is None
        self.turned = 0.0
        self.lane, self.s, self.bias, self.k = [], [], [], []
        for _ in range(self.n):
            if self.start is None:
                lane, s = self._anywhere()
            else:
                lane = self.map.by_id[self.start[0]]
                lane, s = self._along(lane, self.start[1] + rng.gauss(0, START_SD))
            self.lane.append(lane)
            self.s.append(s)
            self.bias.append(wrap(yaw - lane.heading(s)))
            self.k.append(self.scale * rng.uniform(*SCALE_RANGE))
        self.w = [1.0 / self.n] * self.n

    def _anywhere(self):
        """Return a lane and s anywhere on the map, every metre of lane as likely."""
        pick = self.rng.uniform(0, self.map.total)
        for lane in self.map.lanes:
            if pick <= lane.length:
                return lane, pick
            pick -= lane.length
        return lane, lane.length

    def _along(self, lane, s):
        """Carry a lane and s on into the lanes after it (or before it), at random."""
        while s > lane.length and lane.next:
            s -= lane.length
            lane = lane.next[self.rng.randrange(len(lane.next))]
        while s < 0 and lane.prev:
            lane = lane.prev[self.rng.randrange(len(lane.prev))]
            s += lane.length
        return lane, min(max(s, 0.0), lane.length)

    def rebase(self, x, y, yaw, t):
        """Take the odometry's next poses from here, the guesses kept.

        For odometry that starts again elsewhere while the car stands where
        it was (its driver restarted). A car moved meanwhile misfits, and the
        lock searches again.
        """
        self.last = (x, y, yaw, t)
        self.recent = []

    def update(self, x, y, yaw, t):
        """Take one odometry pose (m, rad, s on the car's clock); return the estimate.

        The estimate is None until the car has moved; then a dict: lane, s,
        x, y, heading (the lane's, on the map), confidence (the share of the
        weight near it), locked (confidence of CONFIDENT or more, and after a
        search of the whole map, SEARCH_TURN of turning since it began and a
        misfit within PROBATION), scale and bias.
        """
        if self.last is None:
            self.last = (x, y, yaw, t)
            self._spawn(yaw)
            return None
        x0, y0, _, t0 = self.last
        d = math.hypot(x - x0, y - y0)
        if d < STEP:
            return self.estimate
        # forward or back, by the heading the move was made at
        sign = 1.0 if math.cos(math.atan2(y - y0, x - x0) - yaw) >= 0 else -1.0
        dt = max(t - t0, 0.0)
        self.last = (x, y, yaw, t)
        self.driven += d
        self.recent = [(m, h) for m, h in self.recent if self.driven - m <= WINDOW]
        self.recent.append((self.driven, yaw))
        seen = math.atan2(sum(math.sin(h) for _, h in self.recent),
                          sum(math.cos(h) for _, h in self.recent))
        back = 0.5 * min(WINDOW, self.driven)
        if self.seen is not None:
            self.turned += abs(wrap(seen - self.seen))
        self.seen = seen
        rng = self.rng
        sd_bias = BIAS_DRIFT * math.sqrt(dt)
        sd_k = SCALE_SD * math.sqrt(d)
        lo, hi = self.scale * SCALE_RANGE[0], self.scale * SCALE_RANGE[1]
        norm = 2 * self.sd * self.sd
        total = 0.0
        for i in range(self.n):
            k = min(max(self.k[i] + rng.gauss(0, sd_k), lo), hi)
            self.k[i] = k
            ds = sign * k * d * (1 + rng.gauss(0, MOVE_SD))
            lane, s = self._along(self.lane[i], self.s[i] + ds)
            self.lane[i], self.s[i] = lane, s
            b = self.bias[i] + rng.gauss(0, sd_bias)
            self.bias[i] = b
            e = wrap(seen - b - lane.heading(s - k * back))
            w = self.w[i] * (OUTLIER + (1 - OUTLIER) * math.exp(-e * e / norm))
            self.w[i] = w
            total += w
        # total > 0: every weight keeps OUTLIER of its last
        self.w = [w / total for w in self.w]
        before = self.estimate
        est = self._estimate()
        # the best guess's misfit; a guess that is not the last one moved on starts again
        moved = 2 * est['scale'] * d + 0.5
        if before is not None and \
                math.hypot(est['x'] - before['x'], est['y'] - before['y']) > moved:
            self.misfit = max(self.misfit, MISFIT_LOST)
        lane = self.map.by_id[est['lane']]
        e = wrap(seen - est['bias'] - lane.heading(est['s'] - est['scale'] * back))
        self.misfit = math.sqrt(0.94 * self.misfit ** 2 + 0.06 * e * e)
        est['locked'] = est['confidence'] >= CONFIDENT and (not self.searching or (
            self.turned >= SEARCH_TURN and self.misfit <= PROBATION))
        self.estimate = est
        lost = False
        if est['locked']:
            self.searching = False
            self.lost_at = None
            self.sd = min(max(self.misfit, SD_RANGE[0]), SD_RANGE[1])
            if self.misfit <= MISFIT_LOST:
                self.misfit_at = None
            elif self.misfit_at is None:
                self.misfit_at = self.driven
            lost = self.misfit_at is not None and self.driven - self.misfit_at > MISFIT_AFTER
        else:
            if self.lost_at is None:
                self.lost_at = self.driven
            lost = self.driven - self.lost_at > LOST_AFTER
        if lost:
            # search the whole map again, as an unknown start does
            self.start = None
            self.lost_at = self.misfit_at = None
            self.sd = HEADING_SD
            self.misfit = MISFIT_LOST
            self._spawn(seen)
            return est
        if self.n > self.locked_n and est['locked']:
            self._resample(self.locked_n)
        elif 1.0 / sum(w * w for w in self.w) < self.n / 2:
            self._resample(self.n)
        return est

    def _resample(self, n):
        """Resample into n particles (low variance), each copy nudged off its original."""
        rng = self.rng
        lo, hi = self.scale * SCALE_RANGE[0], self.scale * SCALE_RANGE[1]
        step = 1.0 / n
        u = rng.uniform(0, step)
        c, i = self.w[0], 0
        lane, s, bias, k = [], [], [], []
        for m in range(n):
            while u > c and i < self.n - 1:
                i += 1
                c += self.w[i]
            u += step
            l2, s2 = self._along(self.lane[i], self.s[i] + rng.gauss(0, ROUGHEN[0]))
            lane.append(l2)
            s.append(s2)
            bias.append(self.bias[i] + rng.gauss(0, ROUGHEN[1]))
            k.append(min(max(self.k[i] + rng.gauss(0, ROUGHEN[2]), lo), hi))
        self.lane, self.s, self.bias, self.k = lane, s, bias, k
        self.n = n
        self.w = [step] * n

    def _estimate(self):
        """Return the best guess: the heaviest half-metre of a lane, and the weight near it."""
        bins = {}
        for i in range(self.n):
            key = (self.lane[i].index, int(self.s[i] / 0.5))
            bins[key] = bins.get(key, 0.0) + self.w[i]
        (li, b), _ = max(bins.items(), key=lambda kv: kv[1])
        lane = self.map.lanes[li]
        mid = (b + 0.5) * 0.5
        near = [i for i in range(self.n)
                if self.lane[i] is lane and abs(self.s[i] - mid) <= 0.5]
        wsum = sum(self.w[i] for i in near)
        s = sum(self.w[i] * self.s[i] for i in near) / wsum
        x, y, heading = lane.pose(s)
        conf = wk = bs = bc = 0.0
        for i in range(self.n):
            px, py, ph = self.lane[i].pose(self.s[i])
            if math.hypot(px - x, py - y) <= CLUSTER and abs(wrap(ph - heading)) < math.pi / 3:
                conf += self.w[i]
                wk += self.w[i] * self.k[i]
                bs += self.w[i] * math.sin(self.bias[i])
                bc += self.w[i] * math.cos(self.bias[i])
        # the stretch's own particles are near it, so conf > 0
        return {'lane': lane.id, 's': s, 'x': x, 'y': y, 'heading': heading, 'confidence': conf,
                'scale': wk / conf, 'bias': math.atan2(bs, bc)}
