#!/usr/bin/env python3
"""
Lidar yaw calibration from the direction of travel, by scan matching.

The `lidar` step of `racecar calibrate`. From each of three placements the
car drives itself while the handset's mode switch is DOWN (autonomy); flipping
it UP (manual) stops the car. It drives forward 2.5 s at 0.5 m/s, pauses, then
drives back, with lidar scans at rest before, between, and after. base_link
moves without sideslip, so its chord points the way /odom/integrated says it
went (the gyro heading along the path). The scan match gives the laser's
motion in the laser's own frame, and the one laser yaw that lines the two up
is the measurement. Each forward leg is one estimate; the reverse leg
returns the car and is reported, not counted, since reversing with front
steering wanders. The mean is written to the car profile's params/frames.yaml
(laser_yaw_deg) when its standard error is at most MAX_YAW_SE_DEG; otherwise
another placement is asked for.

The speeds are in m/s and go out as /drive values through drive_speed.py,
using this car's max_speed_mps and throttle caps; the ESP32's speed loop has
no floor. The driving method needs calibrate_odom.py and calibrate_dynamics.py
from the same folder; until they are in this driver it says so, and --wall
works on its own.

--check drives one placement and compares its forward leg with the
configured yaw. --analyze BAG ... recomputes from recorded placements and
writes the result the same way, without driving.

--wall uses the older method: the car squared by hand to a flat wall ahead,
and a line fit that should be perpendicular to x in base_link (--wall
--check reports without writing; --wall-distance-mm takes a tape distance
from base_link to the wall and reports the lidar x error it implies).
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
from neoracer_ros2_driver.frames import load_frames
import rclpy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibrate_common import (  # noqa: E402, I100
    backup_reminder,
    CalibratorNode,
    Criterion,
    finish_check,
    install_config_dir,
    read_line,
    source_config_dir,
    spin_until_done,
    update_local_values,
)
from drive_speed import drive_limits, drive_segments  # noqa: E402, I100
from scan_match import median_ranges, scan_points  # noqa: E402, I100

TOOL = 'calibrate_lidar.py'
HERE = Path(__file__).resolve().parent
# The driving method reuses these tools' bag reader, driver and scan matching.
DRIVE_TOOLS = ('calibrate_odom', 'calibrate_dynamics')
SCAN_SECONDS = 5.0
# Wall points: within this bearing of straight ahead, seen from the lidar.
SECTOR_HALF_DEG = 30.0
MIN_WALL_POINTS = 50
# Beyond this the car is not squared to the wall, or the mount is not
# the configured one; refuse rather than write a large correction.
MAX_CORRECTION_DEG = 10.0
# The fit is planar; a tilted or upside-down mount needs the full transform.
MAX_TILT_DEG = 5.0
# --check limits: yaw to 0.5 deg; RMS residual above 10 mm means a cluttered
# or curved wall.
PASS_ANGLE_DEG = 0.5
# Hand squaring at the wheel centers is good to about 1 deg (0.32 deg per mm),
# so up to WARN_ANGLE_DEG warns; beyond it the mount moved or the car is badly
# placed.
WARN_ANGLE_DEG = 2.0
PASS_RMS_MM = 10.0


@dataclass(frozen=True)
class WallFit:
    """Line fit to the wall, in base_link."""

    angle_deg: float  # wall normal from +x; 0 when perpendicular to x
    distance_mm: float  # base_link origin to the wall, along the normal
    rms_mm: float
    max_mm: float
    points: int


def to_base(points: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
    """Transform laser-frame points into base_link (planar; roll and pitch are 0)."""
    c, s = math.cos(yaw), math.sin(yaw)
    rot = np.array([[c, -s], [s, c]])
    return points @ rot.T + np.array([x, y])


def wall_sector(points: np.ndarray, x: float, y: float) -> np.ndarray:
    """Return the base_link points ahead of the lidar, within SECTOR_HALF_DEG."""
    bearing = np.degrees(np.arctan2(points[:, 1] - y, points[:, 0] - x))
    return points[np.abs(bearing) <= SECTOR_HALF_DEG]


def fit_wall(points: np.ndarray) -> WallFit | None:
    """Total-least-squares line fit; None with fewer than MIN_WALL_POINTS."""
    if len(points) < MIN_WALL_POINTS:
        return None
    centroid = points.mean(axis=0)
    _, vecs = np.linalg.eigh(np.cov((points - centroid).T))
    normal = vecs[:, 0]
    if normal[0] < 0:
        normal = -normal
    residuals = (points - centroid) @ normal
    return WallFit(
        angle_deg=math.degrees(math.atan2(normal[1], normal[0])),
        distance_mm=float(centroid @ normal) * 1e3,
        rms_mm=float(np.sqrt(np.mean(residuals**2))) * 1e3,
        max_mm=float(np.max(np.abs(residuals))) * 1e3,
        points=len(points),
    )


def corrected_yaw_deg(current_deg: float, fit: WallFit) -> float:
    """
    Return the laser yaw that makes the wall perpendicular to x.

    Points placed with a yaw too large by e show the wall rotated by +e.
    """
    return current_deg - fit.angle_deg


def check_criteria(fit: WallFit, yaw_deg: float) -> list[Criterion]:
    """Return the --check result for a wall fit made with the configured yaw."""
    angle = abs(fit.angle_deg)
    if angle <= PASS_ANGLE_DEG:
        status, advice = 'PASS', ''
    elif angle <= WARN_ANGLE_DEG:
        status = 'WARN'
        advice = 'within hand-squaring precision (1 mm at the wheel centers is 0.32 deg)'
    else:
        status = 'FAIL'
        advice = 'beyond hand-squaring error: check the lidar mount, re-square, recalibrate'
    return [
        Criterion(
            'wall angle',
            f'{fit.angle_deg:+.2f} deg (fits {corrected_yaw_deg(yaw_deg, fit):.2f})',
            f'within {PASS_ANGLE_DEG} deg (warn to {WARN_ANGLE_DEG})',
            status,
            advice,
        ),
        Criterion(
            'wall straightness',
            f'{fit.rms_mm:.1f} mm RMS, {fit.points} points',
            f'under {PASS_RMS_MM:.0f} mm',
            'PASS' if fit.rms_mm <= PASS_RMS_MM else 'FAIL',
            'wall cluttered or not flat; use a clear flat wall',
        ),
        Criterion('wall distance', f'{fit.distance_mm:.1f} mm from base_link', '', 'INFO'),
    ]


def config_dir() -> Path:
    """Return the config dir the running stack loaded: installed share, else source."""
    d = install_config_dir() or source_config_dir()
    if d is None:
        raise FileNotFoundError('no config directory found')
    return d


class LidarYawCalibrator(CalibratorNode):
    """Collect scans of a wall ahead, fit it, and correct laser_yaw_deg."""

    def __init__(
        self, check: bool, wall_distance_mm: float | None, report: str | None = None
    ) -> None:
        super().__init__('lidar_yaw_calibrator', LaserScan, '/scan')
        self.check = check
        self.wall_distance_mm = wall_distance_mm
        self.report = report
        self.meta: tuple[float, float, float, float] | None = None

    def extract(self, msg: Any) -> list[float]:
        self.meta = (msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max)
        return list(msg.ranges)

    def run(self) -> int:
        log = self.get_logger()
        if not self.wait_for_first_message():
            return 1
        frames = load_frames(config_dir())
        lx, ly = frames['laser_x_mm'] * 1e-3, frames['laser_y_mm'] * 1e-3
        yaw_deg = frames['laser_yaw_deg']
        if max(abs(frames['laser_roll_deg']), abs(frames['laser_pitch_deg'])) > MAX_TILT_DEG:
            log.error(f'Lidar roll or pitch over {MAX_TILT_DEG:.0f} deg; this tool assumes level.')
            return 1

        log.info('Lidar yaw calibration')
        log.info('Place the car on the ground, facing a flat, clear wall.')
        log.info('Ensure the front bumper is 0.5 to 2 meters away from the wall.')
        log.info('Use a tape measure to measure from the wall to the center of each front wheel.')
        log.info('Turn the car until the two distances are equal.')
        read_line(f'Press Enter, then keep the car still for {SCAN_SECONDS:.0f} seconds...')
        scans = self.collect(SCAN_SECONDS, 'Scan')
        ranges = median_ranges(scans)
        if ranges is None or self.meta is None:
            log.error('No scans collected; nothing written.')
            return 1

        points = to_base(scan_points(ranges, *self.meta), lx, ly, math.radians(yaw_deg))
        fit = fit_wall(wall_sector(points, lx, ly))
        if fit is None:
            log.error(f'Fewer than {MIN_WALL_POINTS} wall points ahead; nothing written.')
            return 1

        new_yaw = corrected_yaw_deg(yaw_deg, fit)
        log.info(f'Wall points: {fit.points}; RMS {fit.rms_mm:.1f} mm, max {fit.max_mm:.1f} mm')
        log.info(f'Wall distance from base_link: {fit.distance_mm:.1f} mm')
        log.info(f'Wall angle error: {fit.angle_deg:+.2f} deg')
        log.info(f'laser_yaw_deg: {yaw_deg:.2f} configured, {new_yaw:.2f} fitted')
        if self.wall_distance_mm is not None:
            dx = fit.distance_mm - self.wall_distance_mm
            log.info(
                f'Taped wall distance {self.wall_distance_mm:.1f} mm: lidar x error '
                f'{dx:+.1f} mm (laser_x_mm {frames["laser_x_mm"] - dx:.1f} would match)'
            )
        if fit.rms_mm > PASS_RMS_MM:
            log.warning(f'RMS above {PASS_RMS_MM:.0f} mm: the wall is cluttered or not flat.')

        if self.check:
            return finish_check(log, check_criteria(fit, yaw_deg), self.report)
        if abs(fit.angle_deg) > MAX_CORRECTION_DEG:
            log.error(
                f'Correction over {MAX_CORRECTION_DEG:.0f} deg: the car is not squared to '
                'the wall, or the lidar mount differs from frames.yaml. Nothing written.'
            )
            return 1
        written = update_local_values('frames', {'laser_yaw_deg': round(new_yaw, 3)}, TOOL)
        log.info(backup_reminder(written))
        return 0


# ---------------------------------------------------------------------------
# Yaw from the direction of travel (the default method)
# ---------------------------------------------------------------------------

# m/s, sent through drive_speed.py. 2.5 s at 0.5 m/s (the dev car holds about
# 0.54) gives a leg of about 1.3 m, the length upstream's precision assumes.
DRIVE_SPEED_MPS = 0.5
LEG_SECONDS = 2.5
# Forward, pause, reverse: the mode switch stays DOWN through the pause.
SEGMENTS_MPS = [
    (0.0, 0.0, 2.0),
    (DRIVE_SPEED_MPS, 0.0, LEG_SECONDS),
    (0.0, 0.0, 2.0),
    (-DRIVE_SPEED_MPS, 0.0, LEG_SECONDS),
    (0.0, 0.0, 3.0),
]
PLACEMENTS = 3
MAX_PLACEMENTS = 6
# One forward leg scatters about 0.14 deg; three placements give about 0.08.
MAX_YAW_SE_DEG = 0.15
CHECK_PASS_DEG, CHECK_WARN_DEG = 0.5, 1.0
# A leg shorter than this did not drive (no reverse from rest, or the mode
# switch flipped UP).
MIN_LEG_M = 0.5
SEARCH_DEG = 15.0
PLACE = 'Place the car on the floor pointing along the clear area.'
PLACE_AGAIN = 'Use a different spot or direction from the previous placements.'
MOTION = 'The car drives forward about 1.3 meters at 0.5 m/s, stops, then drives back.'
STOP = (
    'The car drives itself only with the mode switch DOWN (autonomy). '
    'Flip the mode switch UP to stop.'
)


def missing_drive_tools(folder: Path = HERE) -> list[str]:
    """Return the tools the driving method needs that are not in `folder`."""
    return [f'{name}.py' for name in DRIVE_TOOLS if not (folder / f'{name}.py').is_file()]


def yaw_from_leg(
    laser_motion_pose: Any, heading: float, laser_xy: tuple, yaw0: float, direction: float
) -> float:
    """
    Return the laser yaw (rad) that puts base_link's chord along `direction`.

    `laser_motion_pose` is the scan match (laser frame); `heading` its turn,
    resolved to the true multiple of 2 pi. `direction` is the chord direction
    in the start base_link frame from /odom/integrated, which follows the
    heading along the whole path (half the turn only holds for a constant
    arc). Compared modulo pi, so it holds for a reverse leg too.
    """
    from scan_match import base_motion
    from scipy.optimize import brentq

    m = (laser_motion_pose[0], laser_motion_pose[1], heading)

    def sideslip(yaw: float) -> float:
        b = base_motion(m, (laser_xy[0], laser_xy[1], yaw))
        return math.remainder(math.atan2(b[1], b[0]) - direction, math.pi)

    span = math.radians(SEARCH_DEG)
    return float(brentq(sideslip, yaw0 - span, yaw0 + span))


@dataclass(frozen=True)
class Leg:
    direction: str  # forward or reverse
    yaw_deg: float
    chord_m: float
    heading_deg: float
    rms_m: float
    inliers: float
    problem: str = ''

    @property
    def counted(self) -> bool:
        """Forward legs only: reversing with front steering wanders (4x the scatter)."""
        return self.direction == 'forward' and not self.problem


def measure_legs(bag: Any, laser: Any) -> list[Leg]:
    """Return the forward and reverse legs of one placement, from its three rest scans."""
    from calibrate_dynamics import command_edges, rest_scan
    from calibrate_odom import best_match, MATCH_MAX_RMS_M, MATCH_MIN_INLIERS, resolve_turns
    from scan_match import base_motion, compose, invert, laser_motion

    edges = command_edges(bag)
    starts = [e for e in edges if e[1] != 0.0]
    legs = []
    for n, launch in enumerate(starts[:2]):
        direction = 'forward' if launch[1] > 0 else 'reverse'
        after = starts[n + 1][0] if n + 1 < len(starts) else bag.odom_t[-1] + 0.1
        a = rest_scan(bag, launch[0] - 1.5, launch[0] - 0.1)
        b = rest_scan(bag, after - 1.6, after - 0.1)
        i0 = int(np.searchsorted(bag.odom_t, launch[0] - 0.1))
        i1 = min(int(np.searchsorted(bag.odom_t, after - 0.1)), len(bag.odom_t) - 1)
        rel = compose(invert(tuple(bag.odom[i0])), tuple(bag.odom[i1]))
        heading = float(bag.odom[i1, 2] - bag.odom[i0, 2])
        odom_m = math.hypot(rel[0], rel[1])
        if odom_m < MIN_LEG_M or len(a) < 50 or len(b) < 50:
            what = 'did not reverse' if direction == 'reverse' else 'did not drive forward'
            legs.append(Leg(direction, math.nan, odom_m, 0.0, math.nan, 0.0, f'The car {what}.'))
            continue
        guesses = [
            laser_motion((rel[0], rel[1], heading), laser),
            laser_motion((0.0, 0.0, heading), laser),
        ]
        match = best_match(a, b, guesses)
        true = resolve_turns(match.pose[2], heading)
        chord = math.hypot(*base_motion(match.pose, laser)[:2])
        problem = ''
        if match.rms_m > MATCH_MAX_RMS_M or match.inliers < MATCH_MIN_INLIERS:
            problem = f'Scan match too poor ({match.rms_m * 1e3:.1f} mm, {match.inliers:.0%}).'
        elif true is None or abs(math.degrees(true - heading)) > 10.0:
            problem = 'The lidar and the odometry disagree on the heading.'
        elif abs(chord / odom_m - 1.0) > 0.15:
            problem = f'The lidar measured {chord:.2f} m and the odometry {odom_m:.2f} m.'
        yaw = math.nan
        if not problem:
            try:
                direction_rad = math.atan2(rel[1], rel[0])
                yaw = math.degrees(
                    yaw_from_leg(match.pose, true, laser[:2], laser[2], direction_rad)
                )
            except ValueError:
                problem = f'No lidar yaw within {SEARCH_DEG:g} deg of the configured one fits.'
        legs.append(
            Leg(
                direction,
                yaw,
                chord,
                math.degrees(true if true is not None else heading),
                match.rms_m,
                match.inliers,
                problem,
            )
        )
    return legs


def yaw_summary(estimates: list[float]) -> tuple[float, float]:
    """Return (mean, standard error) in degrees; the error is inf with fewer than 2."""
    if not estimates:
        return math.nan, math.inf
    mean = float(np.mean(estimates))
    if len(estimates) < 2:
        return mean, math.inf
    return mean, float(np.std(estimates, ddof=1) / math.sqrt(len(estimates)))


def offset_criterion(measured_deg: float, configured_deg: float) -> Criterion:
    error = math.remainder(measured_deg - configured_deg, 360.0)
    status = (
        'PASS'
        if abs(error) <= CHECK_PASS_DEG
        else 'WARN' if abs(error) <= CHECK_WARN_DEG else 'FAIL'
    )
    return Criterion(
        'lidar yaw',
        f'{measured_deg:.2f} deg ({error:+.2f} from {configured_deg:.2f})',
        f'within {CHECK_PASS_DEG} deg (warn to {CHECK_WARN_DEG})',
        status,
        'recalibrate the lidar yaw: racecar calibrate lidar',
    )


class DriveYawCalibrator:
    def __init__(self, check: bool, report: str | None) -> None:
        from calibrate_dynamics import Driver, laser_pose

        self.check = check
        self.report = report
        self.driver = Driver()
        self.log = self.driver.log
        self.laser = laser_pose()

    def placement(self) -> list[Leg] | None:
        """Drive one placement; return its good legs, or None to offer it again."""
        from calibrate_dynamics import read_bag

        segments = drive_segments(SEGMENTS_MPS, drive_limits())
        bag = self.driver.drive('yaw', segments, prefix='lidar')
        if bag is None:
            return None
        self.log.info('Analyzing the run...')
        legs = measure_legs(read_bag(bag), self.laser)
        good = []
        for leg in legs:
            if leg.problem:
                self.log.error(f'{leg.direction.capitalize()}: {leg.problem}')
                continue
            note = '' if leg.counted else ' Not counted: the reverse leg returns the car.'
            self.log.info(
                f'{leg.direction.capitalize()}: {leg.chord_m:.2f} m, turned '
                f'{leg.heading_deg:+.1f} deg: lidar yaw {leg.yaw_deg:.2f} deg '
                f'(scan match {leg.rms_m * 1e3:.1f} mm, {leg.inliers:.0%}).{note}'
            )
            if leg.counted:
                good.append(leg)
        if any('reverse' in leg.problem for leg in legs):
            self.log.error(
                'The car did not reverse from rest. The ESC may need a brake before '
                'reverse; check its reverse mode.'
            )
        if not good:
            self.log.error('The forward leg gave no estimate. Run it again.')
            return None
        return good

    def run(self) -> int:
        try:
            return self._check() if self.check else self._calibrate()
        finally:
            self.driver.close()

    def _show(self, i: int, count: int) -> None:
        self.log.info(f'Placement {i + 1} of {count}')
        self.log.info(PLACE)
        if i:
            self.log.info(PLACE_AGAIN)
        self.log.info(MOTION)
        self.log.info(STOP)

    def _calibrate(self) -> int:
        from calibrate_common import run_sequence

        log = self.log
        found = run_sequence(
            PLACEMENTS, lambda i: self._show(i, PLACEMENTS), lambda i: self.placement()
        )
        estimates = [leg.yaw_deg for i in sorted(found) for leg in found[i]]
        mean, se = yaw_summary(estimates)
        count = PLACEMENTS
        while se > MAX_YAW_SE_DEG and count < MAX_PLACEMENTS and estimates:
            log.info(
                f'The estimates scatter too much (standard error {se:.2f} deg, '
                f'needs {MAX_YAW_SE_DEG}). One more placement is needed.'
            )
            self._show(count, count + 1)
            if read_line('Press Enter to run it, or s to stop: ').strip().lower() == 's':
                break
            legs = self.placement()
            if legs:
                estimates += [leg.yaw_deg for leg in legs]
                mean, se = yaw_summary(estimates)
                count += 1
        return write_yaw(estimates, math.degrees(self.laser[2]), log)

    def _check(self) -> int:
        from calibrate_common import run_sequence
        from calibrate_odom import match_criterion
        from scan_match import MatchResult

        log = self.log
        found = run_sequence(1, lambda i: self._show(i, 1), lambda i: self.placement())
        legs = found.get(0, [])
        criteria = []
        if not legs:
            criteria.append(Criterion('lidar yaw', 'no result', '', 'FAIL', 'see above'))
        else:
            for leg in legs:
                criteria.append(
                    match_criterion(
                        f'{leg.direction} scan match',
                        MatchResult((0.0, 0.0, 0.0), leg.rms_m, leg.inliers, 0),
                    )
                )
            mean, _ = yaw_summary([leg.yaw_deg for leg in legs])
            criteria.append(offset_criterion(mean, math.degrees(self.laser[2])))
        return finish_check(log, criteria, self.report)


def analyze_bags(names: list[str]) -> int:
    """Recompute the yaw from recorded placements and write it when precise enough."""
    import logging

    from calibrate_dynamics import laser_pose, read_bag
    from neoracer_log import load_config, resolve_log_root

    logging.basicConfig(format='%(message)s', level=logging.INFO)
    log = logging.getLogger(TOOL)
    laser = laser_pose()
    estimates = []
    for name in names:
        bag = Path(name).expanduser()
        if not bag.exists():
            bag = Path(resolve_log_root(load_config().get('DIR'))) / name
        for leg in measure_legs(read_bag(bag), laser):
            state = leg.problem or ('counted' if leg.counted else 'not counted')
            log.info(f'{bag.name} {leg.direction}: {leg.yaw_deg:.3f} deg ({state})')
            if leg.counted:
                estimates.append(leg.yaw_deg)
    return write_yaw(estimates, math.degrees(laser[2]), log)


def write_yaw(estimates: list[float], configured: float, log: Any) -> int:
    """Report the mean and write it as laser_yaw_deg when its standard error allows."""
    if not estimates:
        log.info('Nothing measured.')
        return 0
    mean, se = yaw_summary(estimates)
    log.info(
        f'Lidar yaw {mean:.3f} deg from {len(estimates)} estimates (standard error '
        f'{se:.2f} deg, spread {min(estimates):.2f} to {max(estimates):.2f}); '
        f'configured {configured:.3f} deg.'
    )
    if se > MAX_YAW_SE_DEG:
        log.error(f'The standard error is above {MAX_YAW_SE_DEG} deg. Nothing written.')
        return 1
    written = update_local_values('frames', {'laser_yaw_deg': round(mean, 3)}, TOOL)
    log.info(f'laser_yaw_deg: {configured:.3f} -> {mean:.3f}')
    log.info(backup_reminder(written))
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description='Lidar yaw calibration from the direction of travel (or --wall).'
    )
    parser.add_argument(
        '--analyze', nargs='+', metavar='BAG', help='recompute from recorded placements'
    )
    parser.add_argument('--check', action='store_true', help='report only; exit 1 on FAIL')
    parser.add_argument('--wall', action='store_true', help='use the hand-squared wall method')
    parser.add_argument(
        '--wall-distance-mm',
        type=float,
        help='with --wall: taped distance from base_link to the wall',
    )
    parser.add_argument('--report', help='with --check: write the result as JSON to this file')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    missing = missing_drive_tools()
    if not args.wall and missing:
        print(
            f'The driving method needs {" and ".join(missing)}, which are not in this '
            'driver yet. Use --wall meanwhile: the car stays still, facing a flat wall.',
            file=sys.stderr,
        )
        return 1
    if args.analyze:
        return analyze_bags(args.analyze)
    if not args.wall:
        return DriveYawCalibrator(args.check, args.report).run()
    rclpy.init()
    return spin_until_done(LidarYawCalibrator(args.check, args.wall_distance_mm, args.report))


if __name__ == '__main__':
    sys.exit(main())
