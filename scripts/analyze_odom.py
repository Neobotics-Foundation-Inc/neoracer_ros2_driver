#!/usr/bin/env python3
"""
Judge odometry test runs from bags: circle, straight, turn, and CPU cost.

Record a run with only the topics it needs, for example:

    racecar log start circle --topics /odom/integrated /odom /imu/fused /motor
    ... drive ...
    racecar log stop

then analyze it (no bag argument: the newest bag):

    analyze_odom.py circle [BAG]                full circle back to a floor mark
    analyze_odom.py straight [BAG] --tape-m 3   taped straight run, either way
    analyze_odom.py turn [BAG] --radius-m 0.74  laps at fixed steering, taped radius
    analyze_odom.py cpu --baseline PCT          live controller_node CPU, not from a bag

The run is the span from the first to the last sample with the wheels turning
(`/odom/integrated` speed not 0). Each command prints its criteria (PASS,
WARN, FAIL, INFO) like `racecar calibrate check` and exits 1 on a FAIL.

On the NeoRacer, `/odom/integrated` carries the source in use
(`odometry.source`), the ESP32's own pose by default. When the bag also holds
`/odom`, the other source's account is shown beside it as INFO: the firmware
pose from `/odom`, or the integrated estimate, replayed from `/odom` and
`/imu/fused` with the profile's scales as calibrate_odom.py does. Only the
source in use is graded.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
import os
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent / 'calibration'))

from calibrate_common import (  # noqa: E402, I100
    criteria_lines,
    Criterion,
    node_params,
    overall,
    paint,
    step_header,
    use_color,
)
from neoracer_log import bag_storage, load_config, resolve_log_root  # noqa: E402

# Acceptance limits.
CIRCLE_YAW_TOLERANCE_DEG = 5.0
STRAIGHT_TOLERANCE = 0.05
TURN_BIAS_LIMIT = 0.03  # encoder over-read in turns; above this, turns need a correction
CPU_ADDED_LIMIT = 10.0  # percent of one core added to controller_node

# /odom/integrated is required; /motor gives the steering, and /odom with
# /imu/fused give the other source's account.
TOPICS = ('/odom/integrated', '/motor', '/odom', '/imu/fused')
# On the firmware source each /odom/integrated sample repeats the position of
# one /odom sample, the one from the same `s` frame.
SAME_POSE_M = 1e-6
# Share of the moving samples that must repeat an /odom position. A few /odom
# messages the recorder dropped leave some samples without their twin.
SAME_POSE_SHARE = 0.9


@dataclass
class Run:
    """Samples of `/odom/integrated` (and `/motor` steering), times in s."""

    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    theta: np.ndarray  # unwrapped, rad
    v: np.ndarray
    omega: np.ndarray
    steer_t: np.ndarray
    steer: np.ndarray

    def window(self) -> Run:
        """Return the span from the first to the last sample with the wheels turning."""
        moving = np.flatnonzero(self.v != 0.0)
        if moving.size == 0:
            raise ValueError('the wheels never turned in this bag')
        lo, hi = moving[0], moving[-1] + 1
        t0, t1 = self.t[lo], self.t[hi - 1]
        keep = (self.steer_t >= t0) & (self.steer_t <= t1)
        return Run(
            self.t[lo:hi],
            self.x[lo:hi],
            self.y[lo:hi],
            self.theta[lo:hi],
            self.v[lo:hi],
            self.omega[lo:hi],
            self.steer_t[keep],
            self.steer[keep],
        )

    def path_length(self) -> float:
        return float(np.sum(np.hypot(np.diff(self.x), np.diff(self.y))))


def run_from_arrays(
    t: list[float],
    x: list[float],
    y: list[float],
    yaw: list[float],
    v: list[float],
    omega: list[float],
    steer_t: list[float] | None = None,
    steer: list[float] | None = None,
) -> Run:
    return Run(
        np.asarray(t, float),
        np.asarray(x, float),
        np.asarray(y, float),
        np.unwrap(np.asarray(yaw, float)),
        np.asarray(v, float),
        np.asarray(omega, float),
        np.asarray(steer_t or [], float),
        np.asarray(steer or [], float),
    )


@dataclass
class BagData:
    """One bag: the source in use, and the samples for the other source's account."""

    run: Run
    firmware: list[list[float]]  # /odom [t, x, y, yaw, v, omega]
    gyro: list[tuple[float, float, float, float]]  # /imu/fused (t, x, y, z) rates


def read_topics(bag: Path) -> BagData:
    """Read `/odom/integrated`, `/motor`, `/odom` and `/imu/fused` from a bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    info = rosbag2_py.Info().read_metadata(str(bag), bag_storage(bag))
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=info.storage_identifier),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    types = {m.name: m.type for m in reader.get_all_topics_and_types()}
    missing = [topic for topic in TOPICS[:1] if topic not in types]
    if missing:
        raise ValueError(f'{bag} has no {", ".join(missing)}; record it with racecar log')
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in TOPICS if t in types]))
    classes = {name: get_message(types[name]) for name in TOPICS if name in types}

    cols: dict[str, list[float]] = {k: [] for k in ('t', 'x', 'y', 'yaw', 'v', 'w', 'st', 's')}
    firmware: list[list[float]] = []
    gyro: list[tuple[float, float, float, float]] = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, classes[topic])
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if topic == '/odom/integrated':
            q = msg.pose.pose.orientation
            cols['t'].append(stamp)
            cols['x'].append(msg.pose.pose.position.x)
            cols['y'].append(msg.pose.pose.position.y)
            cols['yaw'].append(2.0 * math.atan2(q.z, q.w))
            cols['v'].append(msg.twist.twist.linear.x)
            cols['w'].append(msg.twist.twist.angular.z)
        elif topic == '/odom':
            q = msg.pose.pose.orientation
            firmware.append(
                [
                    stamp,
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    2.0 * math.atan2(q.z, q.w),
                    msg.twist.twist.linear.x,
                    msg.twist.twist.angular.z,
                ]
            )
        elif topic == '/imu/fused':
            w = msg.angular_velocity
            gyro.append((stamp, w.x, w.y, w.z))
        else:
            cols['st'].append(stamp)
            cols['s'].append(msg.drive.steering_angle)
    order = np.argsort(cols['t'])
    pick = lambda k: [cols[k][i] for i in order]  # noqa: E731
    run = run_from_arrays(
        pick('t'), pick('x'), pick('y'), pick('yaw'), pick('v'), pick('w'), cols['st'], cols['s']
    )
    return BagData(run, sorted(firmware), sorted(gyro))


def read_bag(bag: Path) -> Run:
    """Read `/odom/integrated` and `/motor` from a bag (any storage the bag declares)."""
    return read_topics(bag).run


# ---------------------------------------------------------------------------
# The other source
# ---------------------------------------------------------------------------


def firmware_run(rows: list[list[float]]) -> Run:
    """Return the ESP32's own pose from /odom [t, x, y, yaw, v, omega] rows as a Run."""
    t, x, y, yaw, v, w = (list(c) for c in zip(*rows, strict=True))
    return run_from_arrays(t, x, y, yaw, v, w)


def source_in_use(run: Run, firmware: Run) -> str:
    """
    Name the source /odom/integrated carried: 'firmware' when it repeats /odom's positions.

    The firmware source republishes each `s` frame's pose, stamped with that
    frame's /imu/fused stamp, and /odom carries the same pose stamped a moment
    later (controller.py publishes the IMU first). So each moving sample
    equals one of the two /odom samples around its stamp. Comparing at one
    interpolated time does not work: the two stamps differ by 0.25 to 6 ms on
    the car, a few mm while driving. The integrated source's positions are
    its own and match none.
    """
    moving = run.v != 0.0
    if not moving.any() or firmware.t.size < 2:
        return 'integrated'
    t, x, y = run.t[moving], run.x[moving], run.y[moving]
    after = np.clip(np.searchsorted(firmware.t, t), 0, firmware.t.size - 1)
    before = np.clip(after - 1, 0, firmware.t.size - 1)
    gap = np.minimum(
        np.hypot(x - firmware.x[after], y - firmware.y[after]),
        np.hypot(x - firmware.x[before], y - firmware.y[before]),
    )
    share = float(np.mean(gap <= SAME_POSE_M))
    return 'firmware' if share >= SAME_POSE_SHARE else 'integrated'


def rest_bias(firmware: list[list[float]], rates: list[tuple[float, float]]) -> float:
    """Return the mean gyro rate before the wheels first turn: the replay's starting bias."""
    moving = [row[0] for row in firmware if row[4] != 0.0]
    start = moving[0] if moving else math.inf
    still = [rate for t, rate in rates if t < start]
    return float(np.mean(still)) if still else 0.0


def integrated_run(bag: BagData, params: dict | None = None) -> Run | None:
    """
    Replay the integrated estimate from /odom and /imu/fused, as calibrate_odom.py does.

    Uses the profile's odometry.* values for controller_node; None without
    /odom or /imu/fused samples.
    """
    if len(bag.firmware) < 2 or len(bag.gyro) < 2:
        return None
    from calibrate_odom import CAL_NAME, integrated_track, integration_from_params, NODE

    params = node_params(CAL_NAME, NODE) if params is None else params
    axis = int(params.get('odometry.yaw_rate_axis', 2))
    sign = float(params.get('odometry.yaw_rate_sign', 1.0))
    rates = [(g[0], sign * g[1 + axis]) for g in bag.gyro]
    rows = [row[:5] for row in bag.firmware]
    cfg = integration_from_params(params)
    track = integrated_track(rows, rates, rest_bias(bag.firmware, rates), cfg)
    if not track:
        return None
    t, x, y, heading, v, w = (list(c) for c in zip(*track, strict=True))
    return run_from_arrays(t, x, y, heading, v, w)


def other_source(bag: BagData, params: dict | None = None) -> tuple[str, str, Run | None]:
    """
    Return (in use, other, other's run): the source /odom/integrated carried and the other one.

    The other run is None when the bag lacks what it needs; the source in use
    reads 'unknown' without /odom.
    """
    if len(bag.firmware) < 2:
        return 'unknown', '', None
    in_use = source_in_use(bag.run, firmware_run(bag.firmware))
    if in_use == 'firmware':
        return in_use, 'integrated', integrated_run(bag, params)
    return in_use, 'firmware', firmware_run(bag.firmware)


def as_info(criterion: Criterion, name: str) -> Criterion:
    """Return the other source's account of a criterion: same numbers, not graded."""
    return Criterion(name, criterion.measured, criterion.limit, 'INFO')


# ---------------------------------------------------------------------------
# Run types
# ---------------------------------------------------------------------------


def circle_criteria(run: Run) -> list[Criterion]:
    """Judge one full circle back to a floor mark: the integrated yaw should be 360 deg."""
    run = run.window()
    turned = math.degrees(run.theta[-1] - run.theta[0])
    error = abs(turned) - 360.0
    closure = math.hypot(run.x[-1] - run.x[0], run.y[-1] - run.y[0])
    return [
        Criterion(
            'heading closure',
            f'{turned:+.1f} deg ({error:+.1f})',
            f'360 within {CIRCLE_YAW_TOLERANCE_DEG:g} deg',
            'PASS' if abs(error) <= CIRCLE_YAW_TOLERANCE_DEG else 'FAIL',
            'check the gyro scale (calibrate_odom spin) and that the car stopped on the mark',
        ),
        Criterion(
            'position closure',
            f'{closure * 1000:.0f} mm over {run.path_length():.2f} m',
            'same mark start and end',
            'INFO',
        ),
        Criterion('duration', f'{run.t[-1] - run.t[0]:.1f} s', '', 'INFO'),
    ]


def straight_criteria(run: Run, tape_m: float) -> list[Criterion]:
    """Judge a straight run over a taped distance, forward or reverse."""
    run = run.window()
    dx, dy = run.x[-1] - run.x[0], run.y[-1] - run.y[0]
    measured = math.hypot(dx, dy)
    direction = 'forward' if float(np.mean(run.v)) > 0 else 'reverse'
    error = measured / tape_m - 1.0
    heading = math.degrees(run.theta[-1] - run.theta[0])
    return [
        Criterion(
            f'distance, {direction}',
            f'{measured:.3f} m ({error:+.1%})',
            f'{tape_m:g} m within {STRAIGHT_TOLERANCE:.0%}',
            'PASS' if abs(error) <= STRAIGHT_TOLERANCE else 'FAIL',
            'encoder scale off: calibrate_odom sets odometry.speed_scale',
        ),
        Criterion(
            'heading change',
            f'{heading:+.2f} deg ({heading / max(measured, 1e-6):+.2f} deg/m)',
            'steering trim input',
            'INFO',
        ),
    ]


def turn_criteria(run: Run, radius_m: float) -> list[Criterion]:
    """
    Judge laps at fixed steering around a taped radius (rear axle midpoint).

    Laps come from the gyro (|heading change| / 360 deg), true distance is laps
    x 2 pi R, and the encoder distance is the integrated speed; their ratio is
    the encoder's over-read in turns.
    """
    run = run.window()
    laps = abs(run.theta[-1] - run.theta[0]) / (2.0 * math.pi)
    speeds = np.abs(run.v)
    encoder = float(np.sum(0.5 * (speeds[1:] + speeds[:-1]) * np.diff(run.t)))
    true = laps * 2.0 * math.pi * radius_m
    bias = encoder / true - 1.0 if true > 0 else float('nan')
    kappa = float(np.mean(run.omega) / np.mean(run.v)) if np.mean(run.v) else float('nan')
    side = 'left' if np.mean(run.omega) > 0 else 'right'
    out = [
        Criterion(
            f'turning bias, {side}',
            f'{bias:+.1%} over {laps:.2f} laps',
            f'under {TURN_BIAS_LIMIT:.0%} needs no correction',
            'PASS' if abs(bias) <= TURN_BIAS_LIMIT else 'WARN',
            'above 3%: the encoder over-reads in turns, so distance runs long when turning',
        ),
        Criterion(
            'curvature',
            f'{kappa:+.3f} 1/m (radius {1 / abs(kappa):.3f} m)' if kappa else 'n/a',
            f'taped radius {radius_m:g} m',
            'INFO',
        ),
    ]
    if run.steer.size:
        out.append(Criterion('steering command', f'{float(np.mean(run.steer)):+.3f}', '', 'INFO'))
    return out


# ---------------------------------------------------------------------------
# CPU (live)
# ---------------------------------------------------------------------------

# The integrated source runs in controller_node (neoracer_ros2_driver/odometry.py).
CONTROLLER_PATH = '/lib/neoracer_ros2_driver/controller'


def controller_pid() -> int | None:
    for entry in Path('/proc').iterdir():
        if entry.name.isdigit():
            try:
                argv = (entry / 'cmdline').read_bytes().split(b'\0')
            except OSError:
                continue
            if len(argv) > 1 and argv[1].decode(errors='replace').endswith(CONTROLLER_PATH):
                return int(entry.name)
    return None


def cpu_seconds(pid: int) -> float:
    fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
    ticks = int(fields[11]) + int(fields[12])  # utime + stime
    return ticks / os.sysconf('SC_CLK_TCK')


def cpu_criteria(percent: float, baseline: float | None) -> list[Criterion]:
    if baseline is None:
        return [Criterion('controller CPU', f'{percent:.1f}% of one core', 'no baseline', 'INFO')]
    added = percent - baseline
    return [
        Criterion(
            'odometry CPU',
            f'{percent:.1f}% ({added:+.1f} on {baseline:g})',
            f'under {CPU_ADDED_LIMIT:g}% added',
            'PASS' if added <= CPU_ADDED_LIMIT else 'FAIL',
            'measure the baseline on the same boot with odometry.enable: false',
        )
    ]


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------


def resolve_bag(arg: str | None) -> Path:
    root = resolve_log_root(load_config().get('DIR'))
    if arg:
        bag = Path(arg).expanduser()
        return bag if bag.exists() else root / arg
    bags = [p for p in root.iterdir() if p.is_dir()] if root.is_dir() else []
    if not bags:
        raise FileNotFoundError(f'no bags in {root}')
    return max(bags, key=lambda p: p.stat().st_mtime)


def report(title: str, criteria: list[Criterion]) -> int:
    color = use_color()
    print(step_header(title, color))
    print('\n'.join(criteria_lines(criteria, color=color)))
    verdict = overall(criteria)
    print(f'Result: {paint(verdict, verdict, color, bold=True)}')
    return 1 if verdict == 'FAIL' else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='cmd', required=True)
    sub.add_parser('circle', help='full circle back to a floor mark').add_argument(
        'bag', nargs='?'
    )
    straight = sub.add_parser('straight', help='taped straight run')
    straight.add_argument('bag', nargs='?')
    straight.add_argument('--tape-m', type=float, required=True, help='taped distance, m')
    turn = sub.add_parser('turn', help='laps at fixed steering around a taped radius')
    turn.add_argument('bag', nargs='?')
    turn.add_argument('--radius-m', type=float, required=True, help='rear-axle circle radius, m')
    cpu = sub.add_parser('cpu', help='sample controller_node CPU live')
    cpu.add_argument('--seconds', type=float, default=30.0)
    cpu.add_argument(
        '--baseline', type=float, help='controller_node %% without odometry, same boot'
    )
    args = parser.parse_args(argv)

    if args.cmd == 'cpu':
        pid = controller_pid()
        if pid is None:
            print('controller_node is not running', file=sys.stderr)
            return 1
        start = cpu_seconds(pid)
        time.sleep(args.seconds)
        percent = (cpu_seconds(pid) - start) / args.seconds * 100.0
        return report(
            f'controller_node (pid {pid}) over {args.seconds:g} s',
            cpu_criteria(percent, args.baseline),
        )

    def judge(run: Run) -> list[Criterion]:
        if args.cmd == 'circle':
            return circle_criteria(run)
        if args.cmd == 'straight':
            return straight_criteria(run, args.tape_m)
        return turn_criteria(run, args.radius_m)

    try:
        bag = resolve_bag(args.bag)
        data = read_topics(bag)
        criteria = judge(data.run)
        in_use, other, other_run = other_source(data)
        criteria.insert(0, Criterion('odometry source', in_use, '/odom/integrated', 'INFO'))
        if other_run is not None:
            # The other source's account of the main criterion, beside the graded one.
            what = {'circle': 'heading', 'straight': 'distance', 'turn': 'bias'}[args.cmd]
            criteria.insert(2, as_info(judge(other_run)[0], f'{other} {what}'))
    except (FileNotFoundError, ValueError, RuntimeError) as exc:
        print(exc, file=sys.stderr)
        return 1
    return report(f'{args.cmd}: {bag}', criteria)


if __name__ == '__main__':
    sys.exit(main())
