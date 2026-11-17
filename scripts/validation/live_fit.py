#!/usr/bin/env python3
"""
Check a localized pose against the live scan: how well it fits, and where it fits best.

    ./live_fit.py MAP                       # car parked, autonomy running on MAP
    ./live_fit.py MAP --span-m 1.0 --span-deg 30

Takes a few scans and the current /autonomy/pose, scores the scan against the
saved map at that pose (scan_fit.py), then searches poses around it on a grid
for the best fit. A reported pose far from the best one is a pose the
localizer has not corrected. The car must be still while it runs.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import time

import numpy as np
import scan_fit as sf

SCANS = 5
STEP_M = 0.025
STEP_DEG = 1.0
SEARCH_STRIDE = 4  # every Nth point during the search; the result is rescored with all


@dataclass
class Score:
    pose: sf.Pose
    median_m: float
    inlier: float


def score(grid: sf.GridMap, points: np.ndarray, pose: sf.Pose, base_laser: sf.Pose) -> Score:
    x, y = sf.place(points, sf.compose(pose, base_laser))
    d = grid.lookup(x, y)
    return Score(pose, float(np.median(d)), float(np.mean(d <= sf.FIT_INLIER_M)))


def best_nearby(
    grid: sf.GridMap,
    points: np.ndarray,
    pose: sf.Pose,
    base_laser: sf.Pose,
    span_m: float,
    span_deg: float,
) -> Score:
    """Return the pose within span_m and span_deg of `pose` whose scan median is lowest."""
    sample = points[::SEARCH_STRIDE]
    best_median, best = math.inf, pose
    offsets = np.arange(-span_m, span_m + 1e-9, STEP_M)
    turns = np.radians(np.arange(-span_deg, span_deg + 1e-9, STEP_DEG))
    for dth in turns:
        for dx in offsets:
            for dy in offsets:
                cand = (pose[0] + float(dx), pose[1] + float(dy), pose[2] + float(dth))
                x, y = sf.place(sample, sf.compose(cand, base_laser))
                median = float(np.median(grid.lookup(x, y)))
                if median < best_median:
                    best_median, best = median, cand
    return score(grid, points, best, base_laser)


def laser_offset() -> sf.Pose:
    """base_link -> laser from frames.yaml with the car's override."""
    from ament_index_python.packages import get_package_share_directory
    from neoracer_ros2_driver.frames import load_frames

    f = load_frames(Path(get_package_share_directory('neoracer_ros2_driver')) / 'config')
    return f['laser_x_mm'] / 1e3, f['laser_y_mm'] / 1e3, math.radians(f['laser_yaw_deg'])


def read_live(timeout: float = 5.0) -> tuple[np.ndarray, sf.Pose, int]:
    """Return the points of SCANS scans, the latest /autonomy/pose, and the scan count."""
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = rclpy.create_node('live_fit')
    scans: list = []
    poses: list = []
    sensor = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(LaserScan, '/scan', scans.append, sensor)
    node.create_subscription(PoseWithCovarianceStamped, '/autonomy/pose', poses.append, 10)
    end = time.monotonic() + timeout
    try:
        while time.monotonic() < end and (len(scans) < SCANS or not poses):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    if not scans or not poses:
        raise SystemExit('live_fit: no /scan or /autonomy/pose; is autonomy running?')
    p = poses[-1].pose.pose
    q = p.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    points = np.vstack(
        [
            sf.scan_points(np.asarray(m.ranges, float), m.angle_min, m.angle_increment)
            for m in scans
        ]
    )
    return points, (p.position.x, p.position.y, yaw), len(scans)


def describe(label: str, s: Score) -> str:
    return (
        f'{label}: x {s.pose[0]:.3f} m, y {s.pose[1]:.3f} m, heading '
        f'{math.degrees(s.pose[2]):.1f} deg; median {s.median_m * 1e3:.0f} mm, '
        f'{s.inlier:.1%} within {sf.FIT_INLIER_M * 1e3:.0f} mm'
    )


def main(argv: list[str] | None = None) -> int:
    from neoracer_ros2_driver.maps import map_stem

    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('map', help='map name under the map root')
    ap.add_argument('--span-m', type=float, default=0.5, help='search half-width (m)')
    ap.add_argument('--span-deg', type=float, default=20.0, help='search half-width (deg)')
    args = ap.parse_args(argv)
    grid = sf.load_grid(map_stem(args.map))
    points, pose, n = read_live()
    base_laser = laser_offset()
    here = score(grid, points, pose, base_laser)
    best = best_nearby(grid, points, pose, base_laser, args.span_m, args.span_deg)
    print(f'{n} scans, {len(points)} points.')
    print(describe('reported', here))
    print(describe('best fit', best))
    print(
        f'offset {math.hypot(best.pose[0] - pose[0], best.pose[1] - pose[1]) * 100:.1f} cm, '
        f'{math.degrees(best.pose[2] - pose[2]):.1f} deg'
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
