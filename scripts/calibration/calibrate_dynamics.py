#!/usr/bin/env python3
"""
Vehicle dynamics for the simulator: steering angle, servo time, speed, stopping.

The `dynamics` step of `racecar calibrate`. The car drives five fixed runs by
itself while the handset's mode switch is DOWN (autonomy); flipping it UP
(manual) stops the car. Each run is recorded as a bag and analyzed. Results
go to the car profile's params/dynamics.yaml, which no node reads: it records
this car's measured values for the simulator. There is no --check.

Runs (speeds in m/s, the ESP32's speed target, sent as /drive values through
drive_speed.py; steering as on /drive, +1 is full left):

    straight025, straight05, straight10   2 s still, 0.62 / 0.75 / 1.0 m/s
                                           straight, then 0 and 3 s still
    circle_right, circle_left              2 s still, 0.62 m/s straight for
                                           0.8 s, full lock for 13 s, then 0
                                           and 3 s still

The run names and result keys are upstream's, so the simulator reads the same
file. Upstream's car reached 0.62, 0.75 and 1.0 m/s at its /drive speeds
0.25, 0.5 and 1.0; those speeds are what the NeoRacer drives, so a key's
suffix names the run, not a /drive value. Full lock is 1.0 on /drive, which
the throttle node's max_steering 0.625 makes 18.75 deg at the ESP32.

Straight runs give the steady speed, the time to reach it, and the stop
distance and time after the command goes to 0, from /odom/integrated; the
lidar's distance between the rest scans checks the encoder over the run.
Circle runs chain lidar scan matches (each scan deskewed with the odometry)
into a trajectory and fit a circle: the rear-axle radius, the Ackermann inner
and outer wheel angles, the outer front tire track diameter, and the
encoder's over-read against the lidar path. The step from straight to full
lock gives the center-to-lock steering time from the yaw rate, an upper
bound for the servo since it includes tire lag; lock to lock is the sum of
both sides. The side is the way the gyro says the car turned.

A run starts only after the mode switch has been seen UP and then DOWN since
the run began, so a switch left DOWN from an earlier run does not start the
car. The car starts about 5 seconds after that flip: a 3 second countdown,
then the run's 2 seconds at rest. From the flip to the last move the switch
must stay DOWN and /joy must keep arriving, or the run stops. Without the
flip within ARM_TIMEOUT_SEC the car does not drive, nothing is written, and
the tool exits with 1. Before the first run the tool reads the throttle and
controller limits from the running nodes. When they differ from the files
the speeds are converted with, nothing drives and the tool exits with 1.

--analyze BAG re-analyzes recorded runs without driving.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Any

import numpy as np
from neoracer_ros2_driver.car_profile import param_path
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

from calibrate_common import (  # noqa: E402, I100
    action_lines,
    backup_reminder,
    DEADMAN,
    install_config_dir,
    log_block,
    node_params,
    result_lines,
    run_sequence,
    source_config_dir,
    status_line,
    step_header,
    update_local_values,
)
from drive_speed import drive_limits, drive_segments  # noqa: E402, I100
from neoracer_log import bag_storage, build_bag_name, build_record_argv  # noqa: E402, I100
from neoracer_log import choose_storage, load_config, resolve_log_root  # noqa: E402
from scan_match import (  # noqa: E402
    base_motion,
    compose,
    invert,
    laser_motion,
    match_scans,
    Pose,
    transform,
)

TOOL = 'calibrate_dynamics.py'
# /imu/fused is REP-103 on the NeoRacer: gyro z is the yaw rate, left positive.
TOPICS = ['/scan', '/imu/fused', '/odom/integrated', '/motor', '/drive']
RATE_HZ = 50.0
RB_BUTTON = 5  # /joy button 5: set while the mode switch is DOWN (autonomy)
COUNTDOWN_SEC = 3.0
# How long a run waits for the mode switch to go UP and then DOWN.
ARM_TIMEOUT_SEC = 60.0
# /joy older than this reads as the switch UP. The controller publishes /joy
# at about 10 Hz; the mux uses the same 0.5 s (joystick_timeout_sec).
JOY_TIMEOUT_SEC = 0.5
# How long to wait for a running node's parameters.
PARAM_TIMEOUT_SEC = 3.0
# The limits that turn a run's m/s and full lock into the ESP32's command,
# read from the running nodes and compared with the files.
LIMIT_PARAMS = {
    'throttle_node': ('max_speed_forward', 'max_speed_backward', 'max_steering'),
    'controller_node': ('max_speed_mps', 'max_steering_angle_deg'),
}
LIMITS_ADVICE = (
    'The car did not drive. Restart teleop so the nodes load the files: '
    'racecar service restart teleop'
)
TIMEOUT_ADVICE = 'Nothing was written. Run the step again with the handset on.'
# A circle run whose gyro curvature is below this did not turn: full lock
# gives about 1.0 to 1.5 per meter.
MIN_TURN_CURVATURE = 0.3
# The straight run's lidar check needs a scan match this good, as
# calibrate_odom.py's MATCH_MAX_RMS_M and MATCH_MIN_INLIERS.
LIDAR_MAX_RMS_M = 0.03
LIDAR_MIN_INLIERS = 0.3
# The straight runs' speeds (m/s) by the suffix their result keys carry, and
# the circles' speed: upstream's car at /drive 0.25, 0.5 and 1.0.
STRAIGHT_MPS = {'025': 0.62, '05': 0.75, '10': 1.0}
CIRCLE_MPS = 0.62
# A commanded speed this close (m/s) to a straight run's names that run.
TAG_TOLERANCE_MPS = 0.05
# (m/s, steering, seconds); steering +1 is full left.
RUNS: dict[str, list[tuple[float, float, float]]] = {
    'straight025': [(0.0, 0.0, 2.0), (STRAIGHT_MPS['025'], 0.0, 3.0), (0.0, 0.0, 3.0)],
    'straight05': [(0.0, 0.0, 2.0), (STRAIGHT_MPS['05'], 0.0, 3.0), (0.0, 0.0, 3.0)],
    'straight10': [(0.0, 0.0, 2.0), (STRAIGHT_MPS['10'], 0.0, 2.5), (0.0, 0.0, 3.0)],
    # A short straight before the lock gives the center-to-lock steering step.
    'circle_right': [
        (0.0, 0.0, 2.0),
        (CIRCLE_MPS, 0.0, 0.8),
        (CIRCLE_MPS, -1.0, 13.0),
        (0.0, 0.0, 3.0),
    ],
    'circle_left': [
        (0.0, 0.0, 2.0),
        (CIRCLE_MPS, 0.0, 0.8),
        (CIRCLE_MPS, 1.0, 13.0),
        (0.0, 0.0, 3.0),
    ],
}
TITLES = {
    'straight025': 'straight at 0.62 m/s',
    'straight05': 'straight at 0.75 m/s',
    'straight10': 'straight at 1.0 m/s',
    'circle_right': 'right-turn circles at full lock, at 0.62 m/s',
    'circle_left': 'left-turn circles at full lock, at 0.62 m/s',
}
STRAIGHT_PLACE = 'Place the car at the start of the straight path, pointing along it.'
# At full lock the car sweeps a circle 1.8 to 2.3 m across: 1.8 to 2.0 m at
# the hand-driven full-lock radii (0.69 m left, 0.77 m right), 2.3 m if the
# 18.75 deg command is the inner wheel's angle. In the widest case the body
# reaches 0.7 m behind the rear axle's start, 1.6 m ahead of it, 2.1 m to the
# inside of the turn and 0.2 m to the outside, with the 0.8 s lead-in.
CIRCLE_PLACE = {
    'circle_right': (
        'Place the car with its back about 1 meter from one end of the clear area, '
        'pointing away from that end. Its left side is about 0.5 meters from the left '
        'edge of the area. The car turns right.'
    ),
    'circle_left': (
        'Place the car with its back about 1 meter from one end of the clear area, '
        'pointing away from that end. Its right side is about 0.5 meters from the right '
        'edge of the area. The car turns left.'
    ),
}


def sentences(text: str) -> list[str]:
    """Split a placement text into one sentence per line, the words unchanged."""
    return re.split(r'(?<=\.) ', text)


# Wheelbase and track of the NeoRacer (neoracer-mujoco spec, CAD), m.
# Per car: odometry.wheelbase_m in odom.yaml / the profile's params/odom.yaml (measured).
WHEELBASE_M = float(node_params('odom', 'controller_node').get('odometry.wheelbase_m', 0.28764))
TRACK_M = 0.224
# Circle fits skip this long after each steering change.
SETTLE_SEC = 2.0


# ---------------------------------------------------------------------------
# Geometry and timing (pure)
# ---------------------------------------------------------------------------


def fit_circle(xy: np.ndarray) -> tuple[float, float, float, float]:
    """Return (cx, cy, radius, RMS residual) of an algebraic circle fit."""
    x, y = xy[:, 0], xy[:, 1]
    design = np.column_stack((x, y, np.ones(len(x))))
    (d, e, f), *_ = np.linalg.lstsq(design, -(x * x + y * y), rcond=None)
    cx, cy = -d / 2.0, -e / 2.0
    r = math.sqrt(cx * cx + cy * cy - f)
    resid = np.hypot(x - cx, y - cy) - r
    return float(cx), float(cy), r, float(np.sqrt(np.mean(resid**2)))


@dataclass(frozen=True)
class Steering:
    """Front wheel angles (deg) for a rear-axle-midpoint turning radius (m)."""

    radius_m: float
    inner_deg: float
    outer_deg: float
    bicycle_deg: float
    outer_front_track_diameter_m: float


def ackermann(radius_m: float, wheelbase: float = WHEELBASE_M, track: float = TRACK_M) -> Steering:
    """Return wheel angles that roll without slip about a center `radius_m` from base_link."""
    inner = math.degrees(math.atan(wheelbase / (radius_m - track / 2)))
    outer = math.degrees(math.atan(wheelbase / (radius_m + track / 2)))
    return Steering(
        radius_m,
        inner,
        outer,
        math.degrees(math.atan(wheelbase / radius_m)),
        2 * math.hypot(radius_m + track / 2, wheelbase),
    )


def transition_times(
    t: np.ndarray, kappa: np.ndarray, t_switch: float, before: float, after: float
) -> tuple[float, float]:
    """
    Return (10-90% time, command-to-95% time) of a curvature step at t_switch.

    `before` and `after` are the steady curvatures on each side.
    """
    span = after - before
    frac = (kappa - before) / span
    later = t >= t_switch

    def first(level: float) -> float:
        hit = np.nonzero(later & (frac >= level))[0]
        return float(t[hit[0]]) if hit.size else math.nan

    t10, t90, t95 = first(0.1), first(0.9), first(0.95)
    return t90 - t10, t95 - t_switch


@dataclass(frozen=True)
class StraightResult:
    steady_mps: float
    rise_90_s: float  # command on to 90% of steady
    stop_m: float  # command 0 to standstill, along the path
    stop_s: float


def straight_metrics(
    t: np.ndarray, v: np.ndarray, s: np.ndarray, t_on: float, t_off: float
) -> StraightResult:
    """Speed `v` and path length `s` (both from /odom/integrated) at times `t`."""
    hold = (t >= t_off - 1.0) & (t < t_off)
    steady = float(np.mean(v[hold]))
    rise = np.nonzero((t >= t_on) & (v >= 0.9 * steady))[0]
    after = np.nonzero(t >= t_off)[0]
    moving = after[np.abs(v[after]) > 0.0]
    i_off = after[0]
    i_stop = moving[-1] + 1 if moving.size else i_off
    i_stop = min(i_stop, len(t) - 1)
    return StraightResult(
        steady,
        float(t[rise[0]] - t_on) if rise.size else math.nan,
        float(s[i_stop] - s[i_off]),
        float(t[i_stop] - t_off),
    )


# ---------------------------------------------------------------------------
# Lidar scan chaining
# ---------------------------------------------------------------------------


def interp_pose(t: np.ndarray, poses: np.ndarray, at: np.ndarray) -> np.ndarray:
    """Interpolate (x, y, unwrapped theta) poses at times `at`."""
    return np.column_stack([np.interp(at, t, poses[:, k]) for k in range(3)])


def deskew(
    points: np.ndarray,
    beam_times: np.ndarray,
    t_ref: float,
    odom_t: np.ndarray,
    odom_poses: np.ndarray,
    laser: Pose,
) -> np.ndarray:
    """Move each point from the laser frame at its beam time to the laser frame at t_ref."""
    ref = tuple(interp_pose(odom_t, odom_poses, np.array([t_ref]))[0])
    at = interp_pose(odom_t, odom_poses, beam_times)
    to_ref = compose(invert(laser), invert(ref))
    out = np.empty_like(points)
    for i, (p, pose) in enumerate(zip(points, at, strict=True)):
        m = compose(compose(to_ref, tuple(pose)), laser)
        out[i] = transform(p[None, :], m)[0]
    return out


def chain_scans(
    scans: list[tuple[float, np.ndarray, np.ndarray]],
    odom_t: np.ndarray,
    odom_poses: np.ndarray,
    laser: Pose,
) -> tuple[np.ndarray, np.ndarray, float]:
    """
    Return (times, base_link poses, mean match RMS) from consecutive scan matches.

    Each scan is (reference time, laser-frame points, beam times); poses start
    at the first scan, in its base_link frame.
    """
    times, poses, rms = [scans[0][0]], [(0.0, 0.0, 0.0)], []
    prev_t = scans[0][0]
    prev_pts = deskew(scans[0][1], scans[0][2], prev_t, odom_t, odom_poses, laser)
    pose = (0.0, 0.0, 0.0)
    for t_ref, pts, beams in scans[1:]:
        cur = deskew(pts, beams, t_ref, odom_t, odom_poses, laser)
        a, b = interp_pose(odom_t, odom_poses, np.array([prev_t, t_ref]))
        guess = laser_motion(compose(invert(tuple(a)), tuple(b)), laser)
        match = match_scans(prev_pts, cur, guess)
        rms.append(match.rms_m)
        pose = compose(pose, base_motion(match.pose, laser))
        times.append(t_ref)
        poses.append(pose)
        prev_t, prev_pts = t_ref, cur
    return np.array(times), np.array(poses), float(np.mean(rms)) if rms else math.nan


# ---------------------------------------------------------------------------
# Bag reading
# ---------------------------------------------------------------------------


@dataclass
class Bag:
    odom_t: np.ndarray  # s
    odom: np.ndarray  # x, y, unwrapped theta
    speed: np.ndarray  # m/s, /odom/integrated twist
    path_m: np.ndarray  # cumulative distance along /odom/integrated
    gyro_t: np.ndarray
    yaw_rate: np.ndarray  # rad/s about base_link z, as the odometry reads it
    motor_t: np.ndarray  # bag receive time, s
    motor: np.ndarray  # normalized speed, steering
    scans: list[tuple[float, np.ndarray, np.ndarray]]  # stamp, ranges, angles


def read_bag(path: Path, axis: int = 2, sign: float = 1.0) -> Bag:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    info = rosbag2_py.Info().read_metadata(str(path), bag_storage(path))
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id=info.storage_identifier),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    classes = {n: get_message(types[n]) for n in TOPICS if n in types}
    odom, gyro, motor, scans = [], [], [], []
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in classes:
            continue
        msg = deserialize_message(data, classes[topic])
        if topic == '/odom/integrated':
            q = msg.pose.pose.orientation
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            odom.append(
                (
                    st,
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    2 * math.atan2(q.z, q.w),
                    msg.twist.twist.linear.x,
                )
            )
        elif topic == '/imu/fused':
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            gyro.append((st, sign * getattr(msg.angular_velocity, 'xyz'[axis])))
        elif topic == '/motor':
            motor.append((t_ns * 1e-9, msg.drive.speed, msg.drive.steering_angle))
        elif topic == '/scan':
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            ranges = np.asarray(msg.ranges, float)
            angles = msg.angle_min + msg.angle_increment * np.arange(ranges.size)
            ok = np.isfinite(ranges) & (ranges >= msg.range_min) & (ranges <= msg.range_max)
            beams = st + msg.time_increment * np.arange(ranges.size)
            scans.append((st + msg.scan_time, ranges[ok], angles[ok], beams[ok]))
    o = np.array(odom)
    xy = o[:, 1:3]
    path_m = np.concatenate(([0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))))
    g = np.array(gyro)
    m = np.array(motor)
    pts = [(t_ref, np.column_stack((r * np.cos(a), r * np.sin(a))), b) for t_ref, r, a, b in scans]
    return Bag(
        odom_t=o[:, 0],
        odom=np.column_stack((xy, np.unwrap(o[:, 3]))),
        speed=o[:, 4],
        path_m=path_m,
        gyro_t=g[:, 0],
        yaw_rate=g[:, 1],
        motor_t=m[:, 0],
        motor=m[:, 1:3],
        scans=pts,
    )


def command_edges(bag: Bag) -> list[tuple[float, float, float]]:
    """Return (time, speed, steering) at each change of the /motor command."""
    edges = []
    last = None
    for t, (sp, st) in zip(bag.motor_t, bag.motor, strict=True):
        key = (round(float(sp), 3), round(float(st), 3))
        if key != last:
            edges.append((float(t), *key))
            last = key
    return edges


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------


def laser_pose() -> Pose:
    from neoracer_ros2_driver.frames import load_frames

    config_dir = install_config_dir() or source_config_dir()
    if config_dir is None:
        raise FileNotFoundError('no config directory found')
    f = load_frames(config_dir)
    return f['laser_x_mm'] * 1e-3, f['laser_y_mm'] * 1e-3, math.radians(f['laser_yaw_deg'])


# Scans stacked per rest: a few fill gaps between beams; more only slow the match.
REST_SCANS = 3


def rest_scan(bag: Bag, t0: float, t1: float) -> np.ndarray:
    """Return the points of the last REST_SCANS scans between t0 and t1."""
    pts = [p for t, p, _ in bag.scans if t0 <= t <= t1][-REST_SCANS:]
    return np.vstack(pts) if pts else np.empty((0, 2))


def speed_tag(mps: float) -> str:
    """
    Return the key suffix of the straight run commanded at `mps` m/s: 0.62 -> '025'.

    Raises ValueError when `mps` is not one of the straight runs' speeds.
    """
    tag, run_mps = min(STRAIGHT_MPS.items(), key=lambda item: abs(item[1] - abs(mps)))
    if abs(run_mps - abs(mps)) > TAG_TOLERANCE_MPS:
        speeds = ', '.join(f'{v:g}' for v in STRAIGHT_MPS.values())
        raise ValueError(f'{mps:.2f} m/s is not one of the straight runs ({speeds} m/s)')
    return tag


def motor_mps(motor_speed: float) -> float:
    """Return the m/s the controller sent the ESP32 for a /motor speed."""
    return motor_speed * drive_limits().max_speed_mps


def analyze_straight(bag: Bag) -> tuple[dict[str, float], list[str]]:
    """Return (values for params/dynamics.yaml, report lines) for a straight run."""
    edges = command_edges(bag)
    on = next(e for e in edges if e[1] != 0.0)
    off = next(e for e in edges if e[0] > on[0] and e[1] == 0.0)
    r = straight_metrics(bag.odom_t, bag.speed, bag.path_m, on[0], off[0])
    command = motor_mps(on[1])
    tag = speed_tag(command)
    values = {
        f'steady_speed_mps_{tag}': round(r.steady_mps, 3),
        f'rise_90_s_{tag}': round(r.rise_90_s, 2),
        f'stop_distance_mm_{tag}': round(r.stop_m * 1e3),
        f'stop_time_s_{tag}': round(r.stop_s, 2),
    }
    lines = [
        f'Command {command:.2f} m/s: steady {r.steady_mps:.3f} m/s, '
        f'90% of it {r.rise_90_s:.2f} s after the command.',
        f'Stop after the command goes to 0: {r.stop_m * 1e3:.0f} mm in {r.stop_s:.2f} s.',
    ]
    laser = laser_pose()
    start = rest_scan(bag, bag.odom_t[0], on[0] - 0.2)
    end = rest_scan(bag, bag.odom_t[-1] - 2.0, bag.odom_t[-1])
    if len(start) and len(end):
        a, b = tuple(bag.odom[0]), tuple(bag.odom[-1])
        match = match_scans(start, end, laser_motion(compose(invert(a), b), laser))
        lidar_m = math.hypot(*base_motion(match.pose, laser)[:2])
        odom_m = float(bag.path_m[-1] - bag.path_m[0])
        if match.rms_m > LIDAR_MAX_RMS_M or match.inliers < LIDAR_MIN_INLIERS or lidar_m <= 0:
            lines.append(
                f'Run distance: odometry {odom_m:.3f} m. The scans at rest did not match '
                f'({match.rms_m * 1e3:.1f} mm, {match.inliers:.0%}), so there is no lidar check.'
            )
        else:
            lines.append(
                f'Run distance: odometry {odom_m:.3f} m, lidar {lidar_m:.3f} m '
                f'({odom_m / lidar_m - 1:+.2%}; scan match {match.rms_m * 1e3:.1f} mm, '
                f'{match.inliers:.0%}).'
            )
    return values, lines


def turn_side(command_steer: float, turning_curvature: float) -> tuple[str, str]:
    """
    Return (side, warning) for a circle run.

    The side is the way the car turned: positive curvature (gyro z over speed)
    is left. The command's sign names it only when the gyro gave nothing.
    """
    commanded = 'left' if command_steer > 0 else 'right'
    if not math.isfinite(turning_curvature) or turning_curvature == 0.0:
        return commanded, ''
    turned = 'left' if turning_curvature > 0 else 'right'
    if turned == commanded:
        return turned, ''
    return turned, (
        f'The car turned {turned} on a full {commanded} command. The values are stored '
        f'as {turned}; check the steering sign from /drive to the ESP32.'
    )


def analyze_circle(bag: Bag) -> tuple[dict[str, float], list[str]]:
    """Return (values for params/dynamics.yaml, report lines) for one circle run."""
    edges = command_edges(bag)
    lead = next(e for e in edges if e[1] != 0.0)
    lock = next(e for e in edges if e[0] > lead[0] and e[2] != 0.0)
    t_end = next(e[0] for e in edges if e[0] > lock[0] and e[1] == 0.0)

    def curvature(t0: float, t1: float) -> tuple[np.ndarray, np.ndarray]:
        g = (bag.gyro_t >= t0) & (bag.gyro_t <= t1)
        v = np.interp(bag.gyro_t[g], bag.odom_t, bag.speed)
        return bag.gyro_t[g], bag.yaw_rate[g] / np.where(np.abs(v) > 0.1, v, np.nan)

    straight = float(np.nanmedian(curvature(lock[0] - 0.4, lock[0])[1]))
    turning = float(np.nanmedian(curvature(lock[0] + SETTLE_SEC, t_end)[1]))
    if math.isfinite(turning) and abs(turning) < MIN_TURN_CURVATURE:
        raise ValueError(
            f'the gyro shows no turn at full lock ({turning:+.2f} per meter), so nothing '
            'is stored for this run'
        )
    side, warning = turn_side(lock[2], turning)
    laser = laser_pose()
    scans = [s for s in bag.scans if lock[0] - 0.5 <= s[0] <= t_end]
    # The driver does not document beam time order against beam index; the
    # order that matches better wins.
    flipped = [(t, p, b[::-1]) for t, p, b in scans]
    trial = [chain_scans(c[:15], bag.odom_t, bag.odom, laser)[2] for c in (scans, flipped)]
    if trial[1] < trial[0]:
        scans = flipped
    times, poses, rms = chain_scans(scans, bag.odom_t, bag.odom, laser)
    keep = (times >= lock[0] + SETTLE_SEC) & (times <= t_end)
    _, _, radius, resid = fit_circle(poses[keep, :2])
    laps = abs(poses[keep, 2][-1] - poses[keep, 2][0]) / (2 * math.pi)
    lidar_path = float(np.sum(np.hypot(*np.diff(poses[keep, :2], axis=0).T)))
    in_odom = (bag.odom_t >= times[keep][0]) & (bag.odom_t <= times[keep][-1])
    enc_path = float(bag.path_m[in_odom][-1] - bag.path_m[in_odom][0])
    sa = ackermann(radius)
    values: dict[str, float] = {
        f'turn_radius_mm_{side}': round(radius * 1e3),
        f'steer_inner_deg_{side}': round(sa.inner_deg, 1),
        f'steer_outer_deg_{side}': round(sa.outer_deg, 1),
        f'outer_front_track_diameter_mm_{side}': round(sa.outer_front_track_diameter_m * 1e3),
        f'encoder_turn_overread_{side}': round(enc_path / lidar_path - 1, 4),
    }
    lines = [warning] if warning else []
    lines += [
        f'Scan chain: {len(times)} scans, mean match {rms * 1e3:.1f} mm.',
        f'Full {side} lock at {motor_mps(lock[1]):.2f} m/s: rear-axle radius '
        f'{radius * 1e3:.0f} mm (fit RMS {resid * 1e3:.1f} mm, {laps:.2f} laps).',
        f'Wheel angles: inner {sa.inner_deg:.1f} deg, outer {sa.outer_deg:.1f} deg.',
        f'Outer front tire track diameter: {sa.outer_front_track_diameter_m * 1e3:.0f} mm.',
        f'Path: lidar {lidar_path:.3f} m, encoder {enc_path:.3f} m '
        f'({enc_path / lidar_path - 1:+.2%}).',
    ]
    t, kappa = curvature(lock[0] - 0.3, lock[0] + 1.5)
    rise, total = transition_times(t, kappa, lock[0], straight, turning)
    values[f'servo_center_to_lock_s_{side}'] = round(total, 3)
    lines.append(
        f'Center to full {side} lock from the yaw rate: 10-90% in {rise:.3f} s, 95% '
        f'{total:.3f} s after the command (an upper bound for the servo).'
    )
    return values, lines


def lock_to_lock(values: dict[str, float]) -> dict[str, float]:
    """Return servo_lock_to_lock_s from both center-to-lock times, when both exist."""
    right = values.get('servo_center_to_lock_s_right')
    left = values.get('servo_center_to_lock_s_left')
    if right is None or left is None:
        return {}
    return {'servo_lock_to_lock_s': round(right + left, 3)}


def analyze(bag_path: Path) -> tuple[dict[str, float], list[str]]:
    bag = read_bag(bag_path)
    return analyze_circle(bag) if '_dyn_circle_' in bag_path.name else analyze_straight(bag)


def analysis_error(exc: Exception) -> str:
    """Return why a run could not be analyzed; a missing /motor edge gives an empty message."""
    return str(exc) or 'the bag has no complete command on /motor'


# ---------------------------------------------------------------------------
# Limits in use
# ---------------------------------------------------------------------------


def file_limits() -> dict[str, dict[str, float]]:
    """Return the LIMIT_PARAMS values the files give: the config YAMLs plus the car profile."""
    limits = drive_limits()
    throttle = node_params('throttle', 'throttle_node')
    controller = node_params('controller', 'controller_node')
    return {
        'throttle_node': {
            'max_speed_forward': limits.max_forward,
            'max_speed_backward': limits.max_backward,
            # The nodes' own defaults when a file does not set the value.
            'max_steering': float(throttle.get('max_steering', 0.625)),
        },
        'controller_node': {
            'max_speed_mps': limits.max_speed_mps,
            'max_steering_angle_deg': float(controller.get('max_steering_angle_deg', 30.0)),
        },
    }


def limit_problems(
    live: dict[str, dict[str, float | None] | None], files: dict[str, dict[str, float]]
) -> list[str]:
    """Return a line for each node that did not answer or runs a limit the files do not give."""
    problems = []
    for node, expected in files.items():
        running = live.get(node)
        if running is None:
            problems.append(f'{node} did not answer with its parameters. Is teleop running?')
            continue
        for name, value in expected.items():
            now = running.get(name)
            if now is None or not math.isclose(now, value, rel_tol=1e-6, abs_tol=1e-9):
                shown = 'no number' if now is None else f'{now:g}'
                problems.append(f'{node} runs {name} {shown}; the files give {value:g}.')
    return problems


def parameter_float(value: Any) -> float | None:
    """Return a double or integer ROS parameter value as a float, else None."""
    from rcl_interfaces.msg import ParameterType

    if value.type == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)
    if value.type == ParameterType.PARAMETER_INTEGER:
        return float(value.integer_value)
    return None


# ---------------------------------------------------------------------------
# Driving
# ---------------------------------------------------------------------------


@dataclass
class Arming:
    """
    The mode switch flip that starts a run: UP, then DOWN, both seen after `reset`.

    A switch left DOWN reads as button 5 set from the first message, so a
    run needs it seen UP first. With no /joy message at all it never arms.
    """

    seen_up: bool = False
    armed: bool = False

    def reset(self) -> None:
        self.seen_up = self.armed = False

    def update(self, down: bool) -> bool:
        """Take one /joy reading of button 5; return whether the run may start."""
        if not down:
            self.seen_up = True
        elif self.seen_up:
            self.armed = True
        return self.armed


class Driver:
    """Publish /drive while the mode switch is DOWN; record the topics the analysis needs."""

    def __init__(self) -> None:
        import rclpy
        from ackermann_msgs.msg import AckermannDriveStamped
        from sensor_msgs.msg import Joy

        rclpy.init()
        self.rclpy = rclpy
        self.msg_type = AckermannDriveStamped
        self.node = rclpy.create_node('dynamics_calibrator')
        self.log = self.node.get_logger()
        self.pub = self.node.create_publisher(AckermannDriveStamped, '/drive', 10)
        self.rb = False
        self.joy_time = -math.inf
        self.arming = Arming()
        self.timed_out = False
        self.node.create_subscription(Joy, '/joy', self._joy, 10)

    def _joy(self, msg: Any) -> None:
        self._button(len(msg.buttons) > RB_BUTTON and bool(msg.buttons[RB_BUTTON]))

    def _button(self, down: bool) -> None:
        self.rb = down
        self.joy_time = time.monotonic()
        self.arming.update(down)

    def switch_down(self) -> bool:
        """Whether the last /joy had the mode switch DOWN and is at most JOY_TIMEOUT_SEC old."""
        return self.rb and time.monotonic() - self.joy_time <= JOY_TIMEOUT_SEC

    def live_limits(self) -> dict[str, dict[str, float | None] | None]:
        """Return the running nodes' LIMIT_PARAMS; None for a node that does not answer."""
        from rcl_interfaces.srv import GetParameters

        live: dict[str, dict[str, float | None] | None] = {}
        for node, names in LIMIT_PARAMS.items():
            client = self.node.create_client(GetParameters, f'/{node}/get_parameters')
            values: list[Any] = []
            if client.wait_for_service(timeout_sec=PARAM_TIMEOUT_SEC):
                future = client.call_async(GetParameters.Request(names=list(names)))
                self.rclpy.spin_until_future_complete(
                    self.node, future, timeout_sec=PARAM_TIMEOUT_SEC
                )
                result = future.result() if future.done() else None
                values = list(result.values) if result is not None else []
            self.node.destroy_client(client)
            live[node] = (
                {n: parameter_float(v) for n, v in zip(names, values, strict=True)}
                if len(values) == len(names)
                else None
            )
        return live

    def _send(self, speed: float, steer: float) -> None:
        msg = self.msg_type()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.drive.speed = float(speed)
        msg.drive.steering_angle = float(steer)
        self.pub.publish(msg)

    def _hold(self, seconds: float, speed: float, steer: float, need_rb: bool) -> bool:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self._send(speed, steer)
            self.rclpy.spin_once(self.node, timeout_sec=1.0 / RATE_HZ)
            if need_rb and not self.switch_down():
                return False
        return True

    def _wait_for_flip(self, lead: float) -> bool:
        """
        Hold still until the mode switch goes UP then DOWN; False after ARM_TIMEOUT_SEC.

        `lead` is the plan's time at rest before its first move, which follows
        the countdown.
        """
        self.arming.reset()
        end = time.monotonic() + ARM_TIMEOUT_SEC
        # One instruction at a time: a switch left DOWN is asked UP first, and
        # the flip DOWN is asked for only once the switch has been seen UP.
        number = 2
        if self.rb:
            log_block(self.log, action_lines(number, ['The mode switch is DOWN. Flip it UP now.']))
            number += 1
            while not self.arming.seen_up:
                if time.monotonic() >= end:
                    return False
                self._hold(0.1, 0.0, 0.0, need_rb=False)
        log_block(
            self.log,
            action_lines(
                number,
                [
                    'Flip the mode switch DOWN (autonomy) to start the run. The car starts '
                    f'driving {COUNTDOWN_SEC + lead:.0f} seconds after you flip it.',
                    'Flip it UP at any time to stop the car.',
                ],
            ),
        )
        while not self.arming.armed:
            if time.monotonic() >= end:
                return False
            self._hold(0.1, 0.0, 0.0, need_rb=False)
        return True

    def _count_down(self, seconds: float, until_move: float, steer: float = 0.0) -> bool:
        """
        Hold still with the switch guarded, printing the whole seconds left until the car moves.

        `until_move` is the time left until the first move when this hold starts.
        `steer` is held with the car still, so a plan that sets its steering
        before the move turns the wheels before the car rolls, as upstream does.
        Return False when the switch goes UP or /joy stops arriving.
        """
        # Whole-second holds, one per number shown; each hold still checks
        # the switch at RATE_HZ.
        done = 0.0
        while done < seconds - 1e-6:
            left = math.ceil(until_move - done - 1e-6)
            if left > 0:
                self.log.info(status_line(f'Starting in {left}...'))
            chunk = min(1.0, seconds - done)
            if not self._hold(chunk, 0.0, steer, need_rb=True):
                return False
            done += chunk
        return True

    def drive(
        self,
        run: str,
        segments: list[tuple[float, float, float]] | None = None,
        prefix: str = 'dyn',
    ) -> Path | None:
        """
        Drive one run while the mode switch is DOWN; return its bag, or None when stopped.

        `segments` are /drive values (speed, steering, seconds); they default
        to RUNS[run] in m/s, converted with this car's limits. The bag is named
        <time>_<prefix>_<run>. After a run that waited ARM_TIMEOUT_SEC for the
        mode switch, `timed_out` is set and the car has not moved.
        """
        plan = segments or drive_segments(RUNS[run], drive_limits())
        cfg = load_config()
        storage, _ = choose_storage(None, cfg.get('STORAGE'))
        root = Path(resolve_log_root(cfg.get('DIR')))
        bag_dir = root / build_bag_name(f'{prefix}_{run}', time.time())
        recorder = subprocess.Popen(
            build_record_argv(bag_dir, TOPICS, storage=storage or 'sqlite3'),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        ok = False
        self.timed_out = False
        moving = [k for k, seg in enumerate(plan) if seg[0] != 0.0]
        first_move = moving[0] if moving else len(plan)
        last_move = moving[-1] if moving else -1
        lead = sum(seg[2] for seg in plan[:first_move])
        try:
            self._hold(2.0, 0.0, 0.0, need_rb=False)
            if not self._wait_for_flip(lead):
                self.timed_out = True
            else:
                ok = self._count_down(COUNTDOWN_SEC, COUNTDOWN_SEC + lead)
            for k, (speed, steer, seconds) in enumerate(plan if ok else []):
                # The switch stays DOWN from the flip to the last move: through
                # the rest before the first move and the pauses between moves.
                # The rest at the end is still.
                if k == first_move:
                    self.log.info(status_line('Driving.'))
                if k == last_move + 1:
                    self.log.info(status_line('You can flip the mode switch UP now.'))
                    self.log.info(
                        status_line(f'Keep your hands off the car for {seconds:.0f} seconds.')
                    )
                if k < first_move:
                    until_move = sum(seg[2] for seg in plan[k:first_move])
                    held = self._count_down(seconds, until_move, steer)
                else:
                    held = self._hold(seconds, speed, steer, need_rb=k <= last_move)
                if not held:
                    ok = False
                    break
            if not ok:
                self._send(0.0, 0.0)
        finally:
            recorder.send_signal(signal.SIGINT)
            recorder.wait(timeout=15)
        if ok:
            self.log.info(status_line('Done.'))
            return bag_dir
        if self.timed_out:
            self.log.error(
                f'The mode switch did not go UP and then DOWN within {ARM_TIMEOUT_SEC:.0f} '
                'seconds, so the car did not drive. Check that /joy button 5 follows the '
                'switch: ros2 topic echo /joy'
            )
        else:
            self.log.error(
                'Stopped: the mode switch went UP before the run ended, or the handset '
                'signal was lost. The car stopped, and this run is not measured.'
            )
        return None

    def close(self) -> None:
        self.node.destroy_node()
        self.rclpy.try_shutdown()


def write_values(values: dict[str, float], log: Any) -> int:
    # The two circle runs may come from different sessions.
    local = param_path('dynamics')
    stored = (yaml.safe_load(local.read_text()) or {}) if local.exists() else {}
    values = {**values, **lock_to_lock({**stored, **values})}
    written = update_local_values('dynamics', values, TOOL)
    log_block(log, result_lines([f'{key}: {value}' for key, value in sorted(values.items())]))
    log.info(status_line(backup_reminder(written)))
    return 0


class ArmTimeout(Exception):
    """A run waited ARM_TIMEOUT_SEC for the mode switch and the car did not drive."""


def session_plans() -> dict[str, list[tuple[float, float, float]]]:
    """
    Return every run's segments as /drive values, converted with this car's limits.

    Raises ValueError, before any node starts, when the limits cannot give a run's speed.
    """
    limits = drive_limits()
    return {run: drive_segments(segments, limits) for run, segments in RUNS.items()}


def run_session(plans: dict[str, list[tuple[float, float, float]]]) -> int:
    driver = Driver()
    log = driver.log
    values: dict[str, float] = {}
    runs = list(RUNS)

    def show(i: int) -> None:
        log_block(log, [step_header(f'Run {i + 1} of {len(runs)}: {TITLES[runs[i]]}')])
        place = sentences(CIRCLE_PLACE.get(runs[i], STRAIGHT_PLACE))
        log_block(log, action_lines(1, [*place, *DEADMAN.split('\n')]))

    def run_one(i: int) -> dict[str, float] | None:
        bag = driver.drive(runs[i], plans[runs[i]])
        if bag is None:
            if driver.timed_out:
                raise ArmTimeout
            return None
        log.info(status_line('Analyzing the run...'))
        try:
            got, lines = analyze(bag)
        except (ValueError, StopIteration) as exc:
            log.error(f'The run could not be analyzed: {analysis_error(exc)}. Its bag is {bag}.')
            return None
        log_block(log, result_lines(lines))
        return got

    try:
        problems = limit_problems(driver.live_limits(), file_limits())
        if problems:
            for problem in problems:
                log.error(problem)
            log.error(LIMITS_ADVICE)
            return 1
        for _, got in sorted(run_sequence(len(runs), show, run_one).items()):
            values.update(got)
        if not values:
            log.info(status_line('Nothing measured.'))
            return 0
        return write_values(values, log)
    except ArmTimeout:
        log.error(TIMEOUT_ADVICE)
        return 1
    finally:
        driver.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument(
        '--analyze', nargs='+', metavar='BAG', help='analyze recorded runs instead of driving'
    )
    args = parser.parse_args(argv)
    if not args.analyze:
        try:
            plans = session_plans()
        except ValueError as exc:
            print(f'The dynamics step cannot run: {exc}.', file=sys.stderr)
            return 1
        return run_session(plans)
    import logging

    logging.basicConfig(format='%(message)s', level=logging.INFO)
    log = logging.getLogger(TOOL)
    values: dict[str, float] = {}
    for name in args.analyze:
        bag = Path(name).expanduser()
        if not bag.exists():
            bag = Path(resolve_log_root(load_config().get('DIR'))) / name
        log_block(log, [step_header(bag.name)])
        try:
            got, lines = analyze(bag)
        except (ValueError, StopIteration) as exc:
            log.error(f'Not analyzed: {analysis_error(exc)}.')
            continue
        log_block(log, result_lines(lines))
        values.update(got)
    if not values:
        log.info(status_line('Nothing measured.'))
        return 1
    return write_values(values, log)


if __name__ == '__main__':
    sys.exit(main())
