#!/usr/bin/env python3
"""
Benchmark the lane lock on a lane map over many simulated drives.

    python3 scripts/lanes/bench_lane_lock.py [MAP.json] [--seeds 6]

Each scenario is driven with several random seeds (the simulated car's
odometry noise, and the filter's own): the car weaves in its lane, its gyro
bias grows and wanders, its heading shakes, its encoder may read distance
off. For each scenario: the worst seed's share of locked updates on the right
lane, the along-track error (median, 95th percentile, worst), and for an
unknown start how far the car drove before the lock found it and held.
"""

import argparse
import math
import os
import statistics
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, 'test'))
from neoracer_ros2_driver.lane_lock import LaneMap  # noqa: E402
import lane_sim  # noqa: E402,I100
import test_lane_lock as sim  # noqa: E402,I100


def scenarios(lanes):
    heavy = {'bias_rate': math.radians(15) / 60, 'bias_walk': 0.01,
             'vibration': math.radians(6), 'wobble': 0.12}
    tour = ['link1_fwd', 'loop2_fwd', 'link1_fwd', 'loop1_back', 'loop2_back', 'link1_back',
            'loop1_fwd']
    return {
        'anticlockwise': (lane_sim.route(lanes, 'loop2_fwd', [], 9), {},
                          {'start': ('loop2_fwd', 0.0)}),
        'clockwise': (lane_sim.route(lanes, 'loop2_back', [], 9), {},
                      {'start': ('loop2_back', 0.0)}),
        'link turns': (lane_sim.route(lanes, 'loop1_back', tour, 17), {},
                       {'start': ('loop1_back', 0.0)}),
        'heavy drift': (lane_sim.route(lanes, 'loop2_fwd', [], 9), heavy,
                        {'start': ('loop2_fwd', 0.0)}),
        'encoder +15%': (lane_sim.route(lanes, 'loop2_fwd', [], 9), {'scale': 1.15},
                         {'start': ('loop2_fwd', 0.0)}),
        'unknown start': (lane_sim.route(lanes, 'loop2_fwd', [], 9), {'start_s': 6.0}, {}),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map', nargs='?', default=sim.MAP)
    ap.add_argument('--seeds', type=int, default=6)
    args = ap.parse_args()
    lanes = LaneMap.load(args.map)
    print('%-14s %10s %8s %8s %8s %12s'
          % ('scenario', 'right lane', 'median', 'p95', 'worst', 'found after'))
    for name, (driven, drive_kw, lock_kw) in scenarios(lanes).items():
        rights, meds, p95s, worsts, finds = [], [], [], [], []
        for seed in range(1, args.seeds + 1):
            samples = lane_sim.drive(lanes, driven, seed=seed, **drive_kw)
            lock = sim.LaneLock(lanes, seed=100 + seed, **lock_kw)
            out = [(lock.update(x, y, yaw, t), lane, s) for t, x, y, yaw, lane, s in samples]
            after = 20
            if 'start' not in lock_kw:
                at = sim.found_at(out)
                finds.append(None if at is None else at * 0.1)
                after = len(out) if at is None else at
            _, right, errors = sim.score(out, after)
            errors.sort()
            rights.append(right)
            if errors:
                meds.append(statistics.median(errors))
                p95s.append(errors[int(0.95 * (len(errors) - 1))])
                worsts.append(errors[-1])
        found = ''
        if finds:
            ok = [f for f in finds if f is not None]
            found = '%d/%d, <= %.0f m' % (len(ok), len(finds), max(ok)) if ok else 'never'
        print('%-14s %10.3f %7.2fm %7.2fm %7.2fm %12s'
              % (name, min(rights), max(meds), max(p95s), max(worsts), found))


if __name__ == '__main__':
    main()
