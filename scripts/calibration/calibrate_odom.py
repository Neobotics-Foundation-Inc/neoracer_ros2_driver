#!/usr/bin/env python3
"""
Odometry scale calibration for the controller's /odom/integrated, referenced to the lidar.

Each run is bracketed by two lidar scans at rest. Scan matching
(scan_match.py) gives base_link's true motion between them, so the car may
stop anywhere. A person drives each run with the handset, mode switch UP
(manual); this tool commands nothing.

Spin: the car drives circles and stops near its start. The scan match gives
the heading change modulo 360 deg; the gyro integral picks the whole-turn
count. odometry.yaw_rate_scale = current scale x lidar / gyro. A scale far
from 1 is not written: the ESP32 sends the gyro in rad/s, so it points at the
axis settings or at a run that did not go as driven.

Drive: the car drives straight toward a wall and stops short of it.
odometry.speed_scale = base x lidar distance / /odom/integrated distance.
The base is the current scale when /odom/integrated comes from the
integrated source, and 1 when it comes from the firmware pose, which the
driver's scale does not touch.

Both scales act only on the integrated source (odometry.source: integrated,
neoracer_ros2_driver/odometry.py). The firmware's own `odom scale` is not
changed here. Both runs also compare /odom/integrated with the lidar, so a
fault in the integration shows instead of hiding in a scale. Writes the car
profile's params/odom.yaml for controller_node, keeping the other step's
value. --check judges /odom/integrated against the lidar with the scales in
use.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
import threading
import time
from typing import Any

from nav_msgs.msg import Odometry
import numpy as np
from neoracer_ros2_driver.frames import load_frames
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Imu, LaserScan

sys.path.insert(0, str(Path(__file__).resolve().parent))

from calibrate_common import (  # noqa: E402, I100
    backup_reminder,
    CalibratorNode,
    Criterion,
    finish_check,
    install_config_dir,
    node_params,
    read_line,
    run_sequence,
    source_config_dir,
    spin_until_done,
    write_local_yaml,
)
from scan_match import (  # noqa: E402, I100
    base_motion,
    compose,
    invert,
    laser_motion,
    match_scans,
    MatchResult,
    median_ranges,
    Pose,
    scan_points,
)

CAL_NAME = 'odom'
NODE = 'controller_node'
TOOL = 'calibrate_odom.py'
REST_SECONDS = 2.0
STILL_SECONDS = 0.5
STILL_MAX_RATE = math.radians(2.0)
SETTLE_TIMEOUT = 20.0
SCALE_LIMITS = (0.8, 1.25)
SPIN_TURNS = 3
CHECK_TURNS = 2
# The gyro picks the whole-turn count; beyond this from the lidar it cannot.
UNWRAP_LIMIT_DEG = 90.0
# The NeoRacer is a 1:12 car and a room rarely has more than 2 m clear in a
# line; at 1.5 m a 1.5 cm scan-match error is 1%, inside the 2% pass limit.
MIN_DRIVE_M = 1.5
# Scan match quality: a moved scene or a featureless corridor fails these.
MATCH_MAX_RMS_M = 0.03
# A 2.3 m drive or an 80 deg turn changes what the lidar sees: 40 to 45% of the
# points matching is normal there.
MATCH_MIN_INLIERS = 0.3
SPIN_PASS, SPIN_WARN = 0.01, 0.03
DRIVE_PASS, DRIVE_WARN = 0.02, 0.05
SCALE_KEYS = ('odometry.yaw_rate_scale', 'odometry.speed_scale')
# The gyro the integrator reads: /imu/fused after the profile's axis remap.
GYRO_TOPIC = '/imu/fused'
PARAM_TIMEOUT = 3.0

HANDS_OFF = f'Press Enter, then keep your hands off the car for {REST_SECONDS:.0f} seconds...'
STOPPED = 'Press Enter when the car has stopped, then keep your hands off it...'
SPIN_PLACE = 'Place the car on the floor in the middle of the clear space.'
DRIVE_PLACE = 'Place the car on the floor facing a wall or furniture 2.5 to 4 meters away.'
HOW_TO_DRIVE = [
    'Drive with the handset, mode switch UP (manual).',
    'Keep the speed switch in its slow position.',
]


def spin_lines(turns: int) -> list[str]:
    return [
        f'Drive the car in a circle (fully right or fully left) {turns} times.',
        'Stop the car close to where it started. The exact position does not matter.',
    ]


DRIVE_LINES = [
    'Drive straight toward the wall at a steady speed.',
    'Stop at least 0.5 meters before the wall. The exact position does not matter.',
]


def log_lines(log: Any, lines: list[str]) -> None:
    for line in lines:
        log.info(line)


def integrate_rate(samples: list[tuple[float, float]], bias: float = 0.0) -> float:
    """Integrate (time s, rate rad/s) samples, minus `bias`, by the trapezoid rule."""
    if len(samples) < 2:
        return 0.0
    data = np.asarray(samples, float)
    rate = data[:, 1] - bias
    return float(np.sum(0.5 * (rate[1:] + rate[:-1]) * np.diff(data[:, 0])))


def resolve_turns(lidar_rad: float, gyro_rad: float) -> float | None:
    """
    Return the true heading change: the lidar's, plus the whole turns the gyro counted.

    None when the gyro is more than UNWRAP_LIMIT_DEG from every candidate.
    """
    turn = 2 * math.pi
    true = lidar_rad + turn * round((gyro_rad - lidar_rad) / turn)
    return true if abs(true - gyro_rad) <= math.radians(UNWRAP_LIMIT_DEG) else None


def scale_ok(scale: float) -> bool:
    return SCALE_LIMITS[0] <= scale <= SCALE_LIMITS[1]


def graded(error: float, passing: float, warning: float) -> str:
    return 'PASS' if abs(error) <= passing else 'WARN' if abs(error) <= warning else 'FAIL'


def spin_criterion(odom_deg: float, lidar_deg: float) -> Criterion:
    error = odom_deg / lidar_deg - 1.0
    return Criterion(
        'spin heading',
        f'{odom_deg:+.1f} deg ({error:+.2%})',
        f'lidar {lidar_deg:+.1f} within {SPIN_PASS:.0%} (warn to {SPIN_WARN:.0%})',
        graded(error, SPIN_PASS, SPIN_WARN),
        'recalibrate the yaw scale: racecar calibrate odom',
    )


def drive_criterion(odom_m: float, lidar_m: float) -> Criterion:
    error = odom_m / lidar_m - 1.0
    return Criterion(
        'drive distance',
        f'{odom_m:.3f} m ({error:+.2%})',
        f'lidar {lidar_m:.3f} m within {DRIVE_PASS:.0%} (warn to {DRIVE_WARN:.0%})',
        graded(error, DRIVE_PASS, DRIVE_WARN),
        'recalibrate the speed scale: racecar calibrate odom',
    )


def match_criterion(name: str, match: MatchResult) -> Criterion:
    ok = match.rms_m <= MATCH_MAX_RMS_M and match.inliers >= MATCH_MIN_INLIERS
    return Criterion(
        name,
        f'{match.rms_m * 1e3:.1f} mm RMS, {match.inliers:.0%} matched',
        f'under {MATCH_MAX_RMS_M * 1e3:.0f} mm, at least {MATCH_MIN_INLIERS:.0%}',
        'PASS' if ok else 'FAIL',
        'keep people and objects still during the run, and use a room, not a corridor',
    )


def scales_criterion(
    file_scales: tuple[float, float], live: tuple[float, float] | None
) -> Criterion:
    """Compare the scales in params/odom.yaml with the ones controller_node is running."""
    yaw, speed = file_scales
    if live is None:
        return Criterion(
            'scales in use',
            f'file: yaw {yaw:g}, speed {speed:g}',
            'controller_node not reachable',
            'INFO',
        )
    if not all(math.isclose(a, b, rel_tol=1e-6) for a, b in zip(file_scales, live, strict=True)):
        return Criterion(
            'scales in use',
            f'controller_node: yaw {live[0]:g}, speed {live[1]:g}; '
            f'file: yaw {yaw:g}, speed {speed:g}',
            'controller_node matches the file',
            'FAIL',
            'restart teleop to load the file: racecar service restart teleop',
        )
    return Criterion('scales in use', f'yaw {yaw:g}, speed {speed:g}', '', 'INFO')


def source_note(source: str) -> str:
    """Say which pose /odom/integrated carries and where the scales act."""
    if source == 'integrated':
        return (
            '/odom/integrated comes from the integrated source, '
            'which applies the scales written here.'
        )
    return (
        f'/odom/integrated comes from the {source} pose. The scales written here act '
        'only on the integrated source (odometry.source: integrated).'
    )


def parameter_number(value: Any) -> float | None:
    """Return a double or integer ROS parameter value as a float, else None."""
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)
    if value.type == ParameterType.PARAMETER_INTEGER:
        return float(value.integer_value)
    return None


def odom_motion(poses: list[list[float]]) -> Pose:
    """Return base_link's motion from the first to the last [t, x, y, yaw, v] pose."""
    start, end = poses[0], poses[-1]
    heading = float(np.unwrap([p[3] for p in poses])[-1] - start[3])
    rel = compose(invert((start[1], start[2], start[3])), (end[1], end[2], end[3]))
    return rel[0], rel[1], heading


@dataclass(frozen=True)
class Run:
    """One run between two rest scans: the lidar's account and the odometry's."""

    lidar: Pose  # base_link motion from the scan match; heading modulo 2 pi
    match: MatchResult
    odom: Pose  # from /odom/integrated; heading unwrapped
    gyro_rad: float  # yaw from the gyro, current scale, bias from the first rest


# Heading offsets tried around the gyro's guess. ICP converges from about
# 10 deg; a gyro scale 3% off is 32 deg after 3 turns.
GUESS_OFFSETS_DEG = tuple(range(-45, 50, 5))


# A match whose distance differs from the odometry's by more than this is a
# wrong alignment (a repeated or symmetric scene), not an odometry error: the
# check limits (2%, 5%) sit far inside it.
CONSISTENT_FRACTION, CONSISTENT_MIN_M = 0.3, 0.4


def consistent(result: MatchResult, expected: Pose) -> bool:
    """Return whether a match's translation agrees with the expected (odometry) one."""
    gap = math.hypot(result.pose[0] - expected[0], result.pose[1] - expected[1])
    return gap <= max(CONSISTENT_MIN_M, CONSISTENT_FRACTION * math.hypot(*expected[:2]))


def best_match(reference: np.ndarray, scan: np.ndarray, guesses: list[Pose]) -> MatchResult:
    """
    Return the best-quality match over the guesses, each fanned out in heading.

    guesses[0] is the odometry's motion; matches far from its translation are
    passed over while any consistent one exists.
    """
    expected = guesses[0]
    results = []
    for x, y, th in guesses:
        for offset in GUESS_OFFSETS_DEG:
            result = match_scans(reference, scan, (x, y, th + math.radians(offset)))
            results.append(result)
            good = result.inliers >= 0.95 and result.rms_m <= MATCH_MAX_RMS_M / 3
            if good and consistent(result, expected):
                return result
    pool = [r for r in results if consistent(r, expected)] or results
    return max(pool, key=lambda r: (r.inliers, -r.rms_m))


class OdomCalibrator(CalibratorNode):
    """/odom/integrated, the gyro the odometry uses, and /scan at rest."""

    def __init__(self, check: bool, report: str | None) -> None:
        super().__init__('odom_calibrator', Odometry, '/odom/integrated')
        self.check = check
        self.report = report
        params = node_params(CAL_NAME, NODE)
        self.source = str(params.get('odometry.source', 'firmware'))
        self.axis = int(params.get('odometry.yaw_rate_axis', 2))
        self.sign = float(params.get('odometry.yaw_rate_sign', 1.0))
        self.yaw_scale = float(params.get('odometry.yaw_rate_scale', 1.0))
        self.speed_scale = float(params.get('odometry.speed_scale', 1.0))
        config_dir = install_config_dir() or source_config_dir()
        if config_dir is None:
            raise RuntimeError('No config folder for neoracer_ros2_driver: run racecar build.')
        frames = load_frames(config_dir)
        self.laser: Pose = (
            frames['laser_x_mm'] * 1e-3,
            frames['laser_y_mm'] * 1e-3,
            math.radians(frames['laser_yaw_deg']),
        )
        self._gyro: list[tuple[float, float]] = []
        self._gyro_on = False
        self._last_rate = 0.0
        self._last_speed: float | None = None
        self._scans: list[list[float]] = []
        self._scan_on = False
        self._scan_meta: tuple[float, float, float, float] | None = None
        self._aux_lock = threading.Lock()
        self.create_subscription(Imu, GYRO_TOPIC, self._gyro_cb, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self._scan_cb, qos_profile_sensor_data)

    def extract(self, msg: Any) -> list[float]:
        q = msg.pose.pose.orientation
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return [
            stamp,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            2 * math.atan2(q.z, q.w),
            msg.twist.twist.linear.x,
        ]

    def _callback(self, msg: Any) -> None:
        self._last_speed = msg.twist.twist.linear.x
        super()._callback(msg)

    def _gyro_cb(self, msg: Any) -> None:
        rate = self.sign * float(getattr(msg.angular_velocity, 'xyz'[self.axis]))
        self._last_rate = rate
        if self._gyro_on:
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            with self._aux_lock:
                self._gyro.append((stamp, rate))

    def _scan_cb(self, msg: Any) -> None:
        if self._scan_on:
            with self._aux_lock:
                self._scan_meta = (
                    msg.angle_min,
                    msg.angle_increment,
                    msg.range_min,
                    msg.range_max,
                )
                self._scans.append(list(msg.ranges))

    def _rest_scan(self) -> np.ndarray | None:
        """Return the median scan over REST_SECONDS as laser-frame points."""
        with self._aux_lock:
            self._scans = []
        self._scan_on = True
        time.sleep(REST_SECONDS)
        self._scan_on = False
        with self._aux_lock:
            ranges, meta = median_ranges(self._scans), self._scan_meta
        if ranges is None or meta is None:
            return None
        return scan_points(ranges, *meta)

    def _wait_still(self) -> bool:
        """Wait until the wheels and the gyro have been still for STILL_SECONDS."""
        deadline = time.monotonic() + SETTLE_TIMEOUT
        still_since: float | None = None
        while (now := time.monotonic()) < deadline:
            if self._last_speed == 0.0 and abs(self._last_rate) < STILL_MAX_RATE:
                still_since = still_since or now
                if now - still_since >= STILL_SECONDS:
                    return True
            else:
                still_since = None
            time.sleep(0.05)
        return False

    def measure(self, place: str, drive: list[str]) -> Run | None:
        """Run one rest scan, drive, rest scan sequence; return both accounts of it."""
        log = self.get_logger()
        log.info(place)
        read_line(HANDS_OFF)
        with self._lock:
            self._samples = []
        with self._aux_lock:
            self._gyro = []
        self._collecting = True
        self._gyro_on = True
        start_scan = self._rest_scan()
        with self._aux_lock:
            rest = [r for _, r in self._gyro]
        bias = float(np.mean(rest)) if rest else 0.0
        log.info('Done. You can drive now.')
        log_lines(log, HOW_TO_DRIVE + drive)
        read_line(STOPPED)
        if not self._wait_still():
            self._collecting = self._gyro_on = False
            log.error('The car did not stop. Nothing measured.')
            return None
        end_scan = self._rest_scan()
        self._collecting = self._gyro_on = False
        log.info('Done. You can move the car now.')
        with self._lock:
            poses = list(self._samples)
        with self._aux_lock:
            gyro = list(self._gyro)
        if start_scan is None or end_scan is None or len(start_scan) < 50 or len(end_scan) < 50:
            log.error('No lidar scans received. Check that teleop is running.')
            return None
        if len(poses) < 2:
            log.error('No odometry received. Check that teleop is running.')
            return None
        odom = odom_motion(poses)
        gyro_rad = integrate_rate(gyro, bias) * self.yaw_scale
        # The gyro guesses the heading: it is what the scale corrects, and
        # /odom/integrated is what is under test.
        guesses = [
            laser_motion((odom[0], odom[1], gyro_rad), self.laser),
            laser_motion((0.0, 0.0, gyro_rad), self.laser),
        ]
        match = best_match(start_scan, end_scan, guesses)
        return Run(base_motion(match.pose, self.laser), match, odom, gyro_rad)

    def spin(self, turns: int) -> tuple[Run, float] | None:
        """Return the spin run and its true heading change (rad), or None."""
        run = self.measure(SPIN_PLACE, spin_lines(turns))
        if run is None:
            return None
        true = resolve_turns(run.lidar[2], run.gyro_rad)
        if true is None:
            self.get_logger().error(
                f'The gyro measured {math.degrees(run.gyro_rad):+.1f} deg, more than '
                f'{UNWRAP_LIMIT_DEG:.0f} deg from the lidar. Nothing measured.'
            )
            return None
        return run, true

    def drive(self) -> Run | None:
        return self.measure(DRIVE_PLACE, DRIVE_LINES)

    def _match_ok(self, run: Run) -> bool:
        c = match_criterion('scan match', run.match)
        if c.status != 'PASS':
            self.get_logger().error(f'Scan match too poor ({c.measured}); {c.advice}.')
            return False
        return True

    def speed_base(self) -> float:
        """Return the speed scale already in /odom/integrated: 1 for the firmware pose."""
        return self.speed_scale if self.source == 'integrated' else 1.0

    def run(self) -> int:
        log = self.get_logger()
        if not self.wait_for_first_message():
            return 1
        log.info(source_note(self.source))
        if self.check:
            return self._run_check()
        titles = ('Step 1 of 2: gyro scale', 'Step 2 of 2: speed scale')
        steps = run_sequence(
            2,
            lambda i: log.info(titles[i]),
            lambda i: self._gyro_step() if i == 0 else self._speed_step(),
        )
        params = dict(steps.values())

        if not params:
            log.info('Nothing calibrated.')
            return 0
        written = write_local_yaml(CAL_NAME, NODE, params, TOOL, merge=True)
        for key, value in params.items():
            log.info(f'{key}: {value}')
        log.info(backup_reminder(written))
        return 0

    def _gyro_step(self) -> tuple[str, float] | None:
        """Run the spin; return the new yaw scale, or None to offer the step again."""
        log = self.get_logger()
        spun = self.spin(SPIN_TURNS)
        if spun is None or not self._match_ok(spun[0]):
            return None
        run, true = spun
        scale = true / run.gyro_rad
        log.info(
            f'Lidar {math.degrees(true):+.1f} deg, gyro {math.degrees(run.gyro_rad):+.1f} '
            f'deg, odometry {math.degrees(run.odom[2]):+.1f} deg'
        )
        if not scale_ok(scale):
            log.error(
                f'Gyro scale {scale:.3f} is outside 0.8 to 1.25. The ESP32 sends the gyro '
                'in rad/s, so check imu.accel_gyro_axis_order and _sign and '
                'odometry.yaw_rate_axis, and that the car turned as many times as asked.'
            )
            return None
        self._warn_odom_mismatch(run.odom[2], run.gyro_rad, 'heading')
        return 'odometry.yaw_rate_scale', round(self.yaw_scale * scale, 5)

    def _speed_step(self) -> tuple[str, float] | None:
        """Run the straight drive; return the new speed scale, or None to offer it again."""
        log = self.get_logger()
        run = self.drive()
        if run is None or not self._match_ok(run):
            return None
        lidar_m = math.hypot(run.lidar[0], run.lidar[1])
        odom_m = math.hypot(run.odom[0], run.odom[1])
        log.info(f'Lidar {lidar_m:.3f} m, odometry {odom_m:.3f} m')
        if lidar_m < MIN_DRIVE_M:
            log.error(
                f'The car drove {lidar_m:.2f} meters. Drive at least {MIN_DRIVE_M:g} meters.'
            )
            return None
        ratio = lidar_m / odom_m if odom_m > 0 else math.inf
        if not scale_ok(ratio):
            log.error(
                f'Odometry is off by more than 20% ({odom_m:.3f} m against {lidar_m:.3f} m).'
            )
            return None
        return 'odometry.speed_scale', round(self.speed_base() * ratio, 5)

    def _warn_odom_mismatch(self, odom_rad: float, gyro_rad: float, what: str) -> None:
        """Warn when /odom/integrated disagrees with the gyro it should follow."""
        if gyro_rad and abs(odom_rad / gyro_rad - 1.0) > SPIN_PASS:
            cause = (
                "a fault in the controller's integration, not the scale"
                if self.source == 'integrated'
                else "the firmware fuses its own heading, so this is not the driver's scale"
            )
            self.get_logger().warning(
                f'Odometry {what} {math.degrees(odom_rad):+.1f} deg differs from the gyro '
                f'{math.degrees(gyro_rad):+.1f} deg: {cause}.'
            )

    def _run_check(self) -> int:
        log = self.get_logger()
        log.info(source_note(self.source))
        titles = (f'Step 1 of 2: spin, {CHECK_TURNS} circles', 'Step 2 of 2: straight drive')
        steps = run_sequence(
            2,
            lambda i: log.info(titles[i]),
            lambda i: self._check_spin() if i == 0 else self._check_drive(),
        )
        criteria = [c for i in sorted(steps) for c in steps[i]]
        file_scales = (self.yaw_scale, self.speed_scale)
        criteria.append(scales_criterion(file_scales, self.live_scales()))
        return finish_check(log, criteria, self.report)

    def _check_spin(self) -> list[Criterion] | None:
        spun = self.spin(CHECK_TURNS)
        if spun is None:
            return None
        run, true = spun
        return [
            match_criterion('spin scan match', run.match),
            spin_criterion(math.degrees(run.odom[2]), math.degrees(true)),
            Criterion('gyro', f'{math.degrees(run.gyro_rad):+.1f} deg', '', 'INFO'),
        ]

    def _check_drive(self) -> list[Criterion] | None:
        run = self.drive()
        if run is None:
            return None
        lidar_m = math.hypot(run.lidar[0], run.lidar[1])
        odom_m = math.hypot(run.odom[0], run.odom[1])
        if abs(lidar_m - odom_m) > max(CONSISTENT_MIN_M, CONSISTENT_FRACTION * odom_m):
            return [
                Criterion(
                    'drive distance',
                    f'lidar {lidar_m:.3f} m, odometry {odom_m:.3f} m',
                    f'within {CONSISTENT_FRACTION:.0%} of each other',
                    'FAIL',
                    'the scan match found no alignment near the odometry: run the check again '
                    'with walls or furniture in view from both ends',
                )
            ]
        return [
            match_criterion('drive scan match', run.match),
            drive_criterion(
                math.hypot(run.odom[0], run.odom[1]), math.hypot(run.lidar[0], run.lidar[1])
            ),
        ]

    def live_scales(self) -> tuple[float, float] | None:
        """Return controller_node's (yaw, speed) scales, or None when it does not answer."""
        client = self.create_client(GetParameters, f'/{NODE}/get_parameters')
        if not client.wait_for_service(timeout_sec=PARAM_TIMEOUT):
            return None
        future = client.call_async(GetParameters.Request(names=list(SCALE_KEYS)))
        deadline = time.monotonic() + PARAM_TIMEOUT
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        values = future.result().values if future.done() and future.result() else []
        if len(values) != len(SCALE_KEYS):
            return None
        # The controller declares its floats dynamically typed, so a hand-written
        # 1 in the YAML arrives as an integer.
        live = [parameter_number(v) for v in values]
        if live[0] is None or live[1] is None:
            return None
        return live[0], live[1]


def main() -> int:
    parser = argparse.ArgumentParser(description='Odometry scale calibration against the lidar.')
    parser.add_argument('--check', action='store_true', help='check the scales in use')
    parser.add_argument('--report', help='with --check: write the result as JSON to this file')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    rclpy.init()
    return spin_until_done(OdomCalibrator(args.check, args.report))


if __name__ == '__main__':
    sys.exit(main())
