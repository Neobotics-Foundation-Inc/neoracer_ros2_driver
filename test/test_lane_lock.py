"""Tests for the lane lock (neoracer_ros2_driver/lane_lock.py) on the Purdue testbed's lane map.

A simulated car drives routes through the map; its odometry carries what a
real one does: a gyro bias that grows and wanders, vibration on the heading,
the car weaving in its lane, and an encoder that reads distance a few percent
off. The lock is held to where the car really is.
"""

import json
import math
import os
import random
import statistics
import sys

from neoracer_ros2_driver.lane_lock import LaneLock, LaneMap, wrap
import pytest

LANES = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                     'scripts', 'lanes')
MAP = os.path.join(LANES, 'purdue_testbed.json')
# Tolerances are in lane widths (LW), as the lane lock's settings are.
with open(MAP) as f:
    LW = json.load(f)['lane_width']
sys.path.insert(0, LANES)
from lane_sim import drive, route  # noqa: E402,I100


@pytest.fixture(scope='module')
def lanes():
    return LaneMap.load(MAP)


def right_lane(est, lane, s):
    """Tell whether an estimate is on the car's lane.

    Where a junction path and the lane it joins meet, they overlap: either
    counts there, within a metre and running the car's way.
    """
    if est['lane'] == lane.id:
        return True
    if lane.kind != 'junction' and not est['lane'].startswith('j'):
        return False
    x, y, h = lane.pose(s)
    near = math.hypot(est['x'] - x, est['y'] - y) <= LW
    return near and abs(wrap(est['heading'] - h)) < math.pi / 3


def run(lane_map, samples, **kw):
    lock = LaneLock(lane_map, seed=7, **kw)
    out = []
    for t, x, y, yaw, lane, s in samples:
        out.append((lock.update(x, y, yaw, t), lane, s))
    return lock, out


def score(out, after=0):
    """(share locked, share of locked updates on the right lane, position errors in m)."""
    rows = out[after:]
    locked = [(e, l, s) for e, l, s in rows if e is not None and e['locked']]
    right = [r for r in locked if right_lane(*r)]
    errors = []
    for e, l, s in locked:
        x, y, _ = l.pose(s)
        errors.append(math.hypot(e['x'] - x, e['y'] - y))
    return len(locked) / max(len(rows), 1), len(right) / max(len(locked), 1), errors


def found_at(out, hold=30):
    """Return the update from which the lock holds the car's place (1 m) for `hold` updates."""
    good = []
    for e, lane, s in out:
        x, y, _ = lane.pose(s)
        good.append(e is not None and e['locked'] and right_lane(e, lane, s)
                    and math.hypot(e['x'] - x, e['y'] - y) < LW)
    return next((i for i in range(len(good) - hold) if all(good[i:i + hold])), None)


def p95(xs):
    return sorted(xs)[int(0.95 * (len(xs) - 1))]


def test_the_map_joins_up(lanes):
    for lane in lanes.lanes:
        assert lane.length > 0.5 * LW
        assert lane.next, lane.id + ' leads nowhere'
        for nxt in lane.next:
            (x0, y0), (x1, y1) = lane.points[-1], nxt.points[0]
            assert math.hypot(x1 - x0, y1 - y0) < 0.01, (lane.id, nxt.id)
    kinds = {lanes.data['lanes'][lane.index].get('turn') for lane in lanes.lanes
             if lane.kind == 'junction'}
    assert kinds == {'left', 'right', 'straight'}


def test_a_known_start_holds_the_lane_for_two_laps(lanes):
    driven = route(lanes, 'loop2_fwd', [], 9)
    samples = drive(lanes, driven, start_s=1.0)
    lock, out = run(lanes, samples, start=('loop2_fwd', 1.0))
    locked, right, errors = score(out, after=20)
    assert locked > 0.95
    assert right > 0.98
    assert statistics.median(errors) < 0.15 * LW and p95(errors) < 0.4 * LW


def test_an_unknown_start_finds_the_car_within_a_lap(lanes):
    driven = route(lanes, 'loop2_fwd', [], 9)
    samples = drive(lanes, driven, start_s=6.0, seed=3)
    lock, out = run(lanes, samples)
    first = found_at(out)
    assert first is not None and first * 0.1 < 35 * LW, \
        'found after %s m' % (first and first * 0.1)
    locked, right, _ = score(out, after=first)
    assert right > 0.97


def test_turning_into_the_link_follows_the_car(lanes):
    driven = route(lanes, 'loop1_back', ['link1_back', 'loop2_fwd'], 8)
    samples = drive(lanes, driven, seed=5)
    lock, out = run(lanes, samples, start=('loop1_back', 0.0))
    on_link = [i for i, (_, l, _) in enumerate(out) if l.id == 'link1_back']
    # 1.5 m into the link, the estimate is on it
    settled = [out[i] for i in on_link if out[i][2] > 1.5]
    assert settled and all(e['lane'] == 'link1_back' for e, _, _ in settled)
    locked, right, _ = score(out, after=20)
    assert right > 0.97


def test_clockwise_runs_on_the_other_lanes(lanes):
    driven = route(lanes, 'loop2_back', [], 9)
    samples = drive(lanes, driven, seed=11)
    lock, out = run(lanes, samples, start=('loop2_back', 0.0))
    locked, right, _ = score(out, after=20)
    assert locked > 0.95 and right > 0.98


def test_strong_drift_and_vibration_still_hold(lanes):
    # a gyro drifting 10 deg a minute, 5 deg of shake, the car weaving 4.5 cm either side
    driven = route(lanes, 'loop2_fwd', [], 9)
    samples = drive(lanes, driven, bias_rate=math.radians(10) / 60, bias_walk=0.008,
                    vibration=math.radians(5), wobble=0.045, seed=13)
    lock, out = run(lanes, samples, start=('loop2_fwd', 0.0))
    locked, right, errors = score(out, after=20)
    assert locked > 0.9 and right > 0.97
    assert p95(errors) < 0.5 * LW


def test_an_encoder_scale_off_by_fifteen_percent_is_learned(lanes):
    driven = route(lanes, 'loop2_fwd', [], 9)
    samples = drive(lanes, driven, scale=1.15, seed=17)
    lock, out = run(lanes, samples, start=('loop2_fwd', 0.0))
    locked, right, _ = score(out, after=100)
    assert right > 0.97
    assert out[-1][0]['scale'] == pytest.approx(1.15, abs=0.06)


def test_standing_still_keeps_the_estimate(lanes):
    driven = route(lanes, 'loop2_fwd', [], 3)
    samples = drive(lanes, driven, seed=19)[:150]
    lock, out = run(lanes, samples, start=('loop2_fwd', 0.0))
    before = out[-1][0]
    t, x, y, yaw = samples[-1][:4]
    rng = random.Random(1)
    for k in range(300):
        est = lock.update(x + rng.gauss(0, 0.004), y + rng.gauss(0, 0.004),
                          yaw + rng.gauss(0, math.radians(1)), t + 0.1 * (k + 1))
    assert est['lane'] == before['lane'] and abs(est['s'] - before['s']) < 0.15 * LW
