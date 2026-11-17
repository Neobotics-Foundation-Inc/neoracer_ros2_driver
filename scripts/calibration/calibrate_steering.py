#!/usr/bin/env python3
"""
Steering trim and command-to-curvature table, referenced to the lidar.

The `steering` step of `racecar calibrate`. The car drives one short run per
steering command by itself while the handset's mode switch is DOWN
(autonomy); flipping it UP (manual) stops the car. In each run the wheels are
set with the car still, the car drives at 0.62 m/s and stops, and the
steering is never changed, so the car moves on a single arc. Two lidar scans
at rest, before and after, give the arc's chord and heading change;
curvature = 2 sin(heading / 2) / chord. No encoder distance is used.

The trim is the drift at command 0 divided by the slope near center (from
+/-0.25; at +/-0.5 the curvature already flattens toward full lock and
differs by side). The trim is measured in /drive steering units and applied
in the ESP32 firmware in degrees: one /drive unit is max_steering times
max_steering_angle_deg on this car (0.625 x 30 = 18.75 deg). The runs measure
with the board's trim in use, so the new trim is the board's present trim
plus the correction. A change smaller than MIN_TRIM_STEP_DEG is not written.
The trim is written through controller_node's board.steering_trim_deg, which
sends `trim set` and confirms it with `trim get`; the firmware saves it and
applies it to manual driving too. The board's trim before the write is kept
in the car profile's params/steering.yaml (trim_deg, then previous_trim_deg),
whose earlier copy goes to the profile's history. A non-zero driver
steering_trim_deg is folded into the board's trim and set to 0 in the profile.
The curved runs also report the encoder's over-read in turns as a gain for
odometry.turn_gain, which is not written.

--check drives straight at command 0: heading drift within 1 deg/m passes; up
to 5 deg/m warns, naming servo and linkage play (up to 3 deg/m between runs);
beyond is a mechanical fault. The board's trim must match the profile copy;
a board swap or a full flash shows as a mismatch.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import math
from pathlib import Path
import sys
from typing import Any

import numpy as np
from neoracer_ros2_driver import car_profile
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

from calibrate_common import (  # noqa: E402, I100
    action_lines,
    Criterion,
    DEADMAN,
    finish_check,
    log_block,
    node_params,
    result_lines,
    run_sequence,
    status_line,
    step_header,
    update_local_values,
    write_local_yaml,
)
from calibrate_dynamics import (  # noqa: E402
    analysis_error,
    ArmTimeout,
    Bag,
    command_edges,
    Driver,
    file_limits,
    laser_pose,
    limit_problems,
    LIMITS_ADVICE,
    PARAM_TIMEOUT_SEC,
    parameter_float,
    read_bag,
    rest_scan,
    TIMEOUT_ADVICE,
    WHEELBASE_M,
)
from calibrate_odom import best_match, match_criterion, resolve_turns  # noqa: E402
from drive_speed import drive_limits, drive_segments  # noqa: E402
from scan_match import (  # noqa: E402
    base_motion,
    compose,
    invert,
    laser_motion,
    MatchResult,
)

CAL_NAME = 'steering'  # the profile copy, params/steering.yaml; no node loads it
NODE = 'controller_node'
BOARD_PARAM = 'board.steering_trim_deg'
DRIVER_TRIM_PARAM = 'steering_trim_deg'
TOOL = 'calibrate_steering.py'
# Upstream drives at a /drive speed of 0.25, about 0.62 m/s on its indoor
# firmware, as in calibrate_dynamics.py.
DRIVE_MPS = 0.62
COMMANDS = (0.0, 0.25, -0.25, 0.5, -0.5, 1.0, -1.0)
# Commands used for the trim fit; full lock is past where curvature is linear.
# The trim comes from the drift at 0 and the slope from +/-CENTER_COMMAND: past
# that the curvature flattens toward full lock and differs by side.
CENTER_COMMAND = 0.25
# Smallest board trim change worth writing. The servo pulse moves in 1 us
# steps, about 0.06 deg; 0.1 deg moves the straight drift by about 0.35 deg/m
# on this wheelbase, a third of the 1 deg/m pass.
MIN_TRIM_STEP_DEG = 0.1
# Fit of the encoder over-read in turns, k x (1/cos(delta) - 1): only runs with
# enough steering to carry it.
TURN_FIT_MIN_X = 0.05
# The board's trim range; a trim past it means the servo horn or the linkage moved.
TRIM_LIMIT_DEG = 5.0
# The board prints its trim with two decimals.
TRIM_TOL_DEG = 0.01
# Straight drift at a fixed trim varied from -2.7 to +0.4 deg/m between runs on
# the reference car (servo and linkage play). Drift past DRIFT_PASS still
# warns, naming that play; past DRIFT_WARN it is beyond play.
DRIFT_PASS, DRIFT_WARN = 1.0, 5.0  # deg/m
HARDWARE_PLAY_DEG_PER_M = 3.0
PARAM_TIMEOUT = PARAM_TIMEOUT_SEC

PLACE = 'Place the car at the middle of one edge of the clear area, pointing across it.'


def segments(command: float) -> list[tuple[float, float, float]]:
    """Return a run in m/s: wheels set while still, one drive, stop; the steering never changes."""
    # Long enough to measure, short enough that the turn stays under about
    # 100 deg so the two rest scans still overlap.
    drive_sec = 3.0 if command == 0.0 else 2.0 if abs(command) <= 0.25 else 1.0
    return [(0.0, command, 2.0), (DRIVE_MPS, command, drive_sec), (0.0, command, 3.0)]


def session_plans() -> dict[float, list[tuple[float, float, float]]]:
    """
    Return every run's segments as /drive values, converted with this car's limits.

    Raises ValueError, before any node starts, when the limits cannot give the speed.
    """
    limits = drive_limits()
    return {command: drive_segments(segments(command), limits) for command in COMMANDS}


def run_name(command: float) -> str:
    """Return a bag-safe name: 0.25 -> 'l025', -0.5 -> 'r05', 0 -> 'c0'."""
    if command == 0.0:
        return 'c0'
    tag = f'{abs(command):.2f}'.replace('.', '').rstrip('0').ljust(2, '0')
    return ('l' if command > 0 else 'r') + tag


def describe(command: float) -> str:
    side = 'straight' if command == 0.0 else 'left' if command > 0 else 'right'
    return f'steering {command:+g} ({side})'


def arc_curvature(heading_rad: float, chord_m: float) -> float:
    """Return the curvature (1/m, + left) of an arc with this heading change and chord."""
    if chord_m <= 0.0:
        return math.nan
    return 2.0 * math.sin(heading_rad / 2.0) / chord_m


def degrees_per_command(limits: dict[str, dict[str, float]]) -> float:
    """
    Return the degrees on the board's steering command for one /drive steering unit.

    The throttle node scales /drive by max_steering onto /motor, and the
    controller sends /motor times max_steering_angle_deg to the board, which
    adds its trim in the same degrees.
    """
    return (
        limits['throttle_node']['max_steering']
        * limits['controller_node']['max_steering_angle_deg']
    )


def new_board_trim(board_deg: float, driver_deg: float, correction_deg: float) -> float:
    """
    Return the board trim that drives straight, the driver's trim folded in.

    The runs measure with both trims in use; the driver's goes to 0 afterwards.
    """
    return board_deg + driver_deg + correction_deg


@dataclass(frozen=True)
class Arc:
    command: float
    curvature: float  # 1/m, + left
    heading_deg: float
    chord_m: float
    match: MatchResult
    odom_heading_deg: float = math.nan  # /odom/integrated over the same run
    odom_chord_m: float = math.nan
    encoder_m: float = math.nan  # /odom/integrated path length over the run
    lidar_arc_m: float = math.nan  # arc length from the lidar chord and heading


@dataclass(frozen=True)
class TrimFit:
    trim_delta: float  # command that drives straight, in the current trim's terms
    drift_deg_per_m: float  # curvature at command 0, deg/m
    gain_left: float  # curvature per unit command near center, 1/m
    gain_right: float


def fit_trim(arcs: list[Arc]) -> TrimFit | None:
    """
    Return the trim from the drift at 0 and the center slope; None without them.

    Needs the run at 0 and at least one of +/-CENTER_COMMAND.
    """
    by = {a.command: a.curvature for a in arcs}
    if 0.0 not in by:
        return None
    k0 = by[0.0]
    left = (by[CENTER_COMMAND] - k0) / CENTER_COMMAND if CENTER_COMMAND in by else math.nan
    right = (k0 - by[-CENTER_COMMAND]) / CENTER_COMMAND if -CENTER_COMMAND in by else math.nan
    slopes = [g for g in (left, right) if math.isfinite(g) and g > 0]
    if not slopes:
        return None
    return TrimFit(
        trim_delta=float(-k0 / np.mean(slopes)),
        drift_deg_per_m=math.degrees(k0),
        gain_left=float(left),
        gain_right=float(right),
    )


def measure_arc(bag: Bag, laser: Any, command: float) -> Arc | None:
    """
    Return the arc of one constant-steering run, from its two rest scans.

    `command` is the steering sent on /drive; the board adds its own trim.
    """
    edges = command_edges(bag)
    launch = next((e for e in edges if e[1] != 0.0), None)
    if launch is None:
        return None
    start = rest_scan(bag, launch[0] - 1.5, launch[0] - 0.1)
    end = rest_scan(bag, bag.odom_t[-1] - 2.0, bag.odom_t[-1])
    if len(start) < 50 or len(end) < 50:
        return None
    i0 = int(np.searchsorted(bag.odom_t, launch[0] - 0.1))
    a, b = tuple(bag.odom[i0]), tuple(bag.odom[-1])
    rel = compose(invert(a), b)
    heading = float(bag.odom[-1, 2] - bag.odom[i0, 2])
    guesses = [
        laser_motion((rel[0], rel[1], heading), laser),
        laser_motion((0, 0, heading), laser),
    ]
    match = best_match(start, end, guesses)
    motion = base_motion(match.pose, laser)
    true = resolve_turns(motion[2], heading)
    if true is None:
        return None
    chord = math.hypot(motion[0], motion[1])
    half = true / 2.0
    arc_m = chord * (half / math.sin(half)) if abs(half) > 1e-6 else chord
    return Arc(
        command,
        arc_curvature(true, chord),
        math.degrees(true),
        chord,
        match,
        math.degrees(heading),
        math.hypot(rel[0], rel[1]),
        float(bag.path_m[-1] - bag.path_m[i0]),
        abs(arc_m),
    )


@dataclass(frozen=True)
class TurnFit:
    gain: float  # k for odometry.turn_gain
    straight_ratio: float  # encoder / lidar on the straight runs (the session's baseline)
    runs: int


def fit_turn_gain(arcs: list[Arc], current_gain: float, wheelbase: float) -> TurnFit | None:
    """
    Fit k in encoder / lidar = baseline (1 + k (1/cos(delta) - 1)) over the curved runs.

    The baseline is the straight runs of the same session, so a speed_scale
    error cancels. /odom/integrated already divides by the current gain's
    factor, which is put back first. None without a straight run and two
    curved runs that carry the fit.
    """
    straight = [
        a.encoder_m / a.lidar_arc_m for a in arcs if a.command == 0.0 and a.lidar_arc_m > 0
    ]
    if not straight:
        return None
    base = float(np.mean(straight))
    xs, ys = [], []
    for a in arcs:
        if a.command == 0.0 or not a.lidar_arc_m > 0:
            continue
        x = 1.0 / math.cos(math.atan(wheelbase * abs(a.curvature))) - 1.0
        if x < TURN_FIT_MIN_X:
            continue
        ratio = (a.encoder_m / a.lidar_arc_m) / base * (1.0 + current_gain * x)
        xs.append(x)
        ys.append(ratio - 1.0)
    if len(xs) < 2:
        return None
    xa, ya = np.array(xs), np.array(ys)
    return TurnFit(float(xa @ ya / (xa @ xa)), base, len(xs))


# The odometry heading is calibrated to well under 1%; a scan match further
# off than this has locked onto the wrong alignment.
MAX_HEADING_DISAGREEMENT_DEG = 10.0
# The encoder reads up to about 4% high in turns; beyond this the match is wrong.
MAX_CHORD_DISAGREEMENT = 0.15


def arc_problem(arc: Arc) -> str:
    """Return why an arc's measurement cannot be trusted, or '' when it can."""
    gate = match_criterion('scan match', arc.match)
    if gate.status != 'PASS':
        return f'Scan match too poor ({gate.measured}).'
    gap = abs(arc.heading_deg - arc.odom_heading_deg)
    if gap > MAX_HEADING_DISAGREEMENT_DEG:
        return (
            f'The lidar measured {arc.heading_deg:+.1f} deg and the odometry '
            f'{arc.odom_heading_deg:+.1f} deg.'
        )
    if abs(arc.chord_m / arc.odom_chord_m - 1.0) > MAX_CHORD_DISAGREEMENT:
        return (
            f'The lidar measured a {arc.chord_m:.2f} m chord and the odometry '
            f'{arc.odom_chord_m:.2f} m.'
        )
    return ''


def drift_criterion(drift: float) -> Criterion:
    status = 'PASS' if abs(drift) <= DRIFT_PASS else 'WARN' if abs(drift) <= DRIFT_WARN else 'FAIL'
    return Criterion(
        'straight drift',
        f'{drift:+.2f} deg/m ({"left" if drift > 0 else "right"})',
        f'within {DRIFT_PASS} deg/m (warn to {DRIFT_WARN})',
        status,
        (
            f'servo and linkage play alone moves the drift by up to {HARDWARE_PLAY_DEG_PER_M:g} '
            'deg/m between runs; recalibrate only if it stays: racecar calibrate steering'
            if status == 'WARN'
            else 'beyond servo and linkage play: check the servo horn and the steering '
            'linkage, then racecar calibrate steering'
        ),
    )


def trim_criterion(copy_deg: float | None, board_deg: float | None) -> Criterion:
    """Compare the board's trim, read live, with the car profile's copy."""
    if board_deg is None:
        shown = 'no profile copy' if copy_deg is None else f'profile copy {copy_deg:+.2f} deg'
        return Criterion('board trim', shown, f'{NODE} not reachable', 'INFO')
    if not math.isfinite(board_deg):
        return Criterion(
            'board trim',
            'unknown',
            "the controller reads the board's trim",
            'FAIL',
            "the controller did not read the board's trim at start: check the ESP32 "
            'link, then racecar service restart teleop',
        )
    if copy_deg is None:
        return Criterion('board trim', f'{board_deg:+.2f} deg; no profile copy yet', '', 'INFO')
    if abs(board_deg - copy_deg) > TRIM_TOL_DEG:
        return Criterion(
            'board trim',
            f'board {board_deg:+.2f} deg; profile copy {copy_deg:+.2f} deg',
            'the board matches the profile copy',
            'FAIL',
            'a new board or a full flash: set it back with ros2 param set '
            f'/{NODE} {BOARD_PARAM} {copy_deg:.2f}, or recalibrate: racecar calibrate steering',
        )
    return Criterion('board trim', f'{board_deg:+.2f} deg, as the profile copy', '', 'INFO')


def driver_trim_criterion(driver_deg: float) -> Criterion | None:
    """Warn when the driver adds its own trim on top of the board's; None when it is 0."""
    if abs(driver_deg) < 1e-9:
        return None
    return Criterion(
        'driver trim',
        f'{DRIVER_TRIM_PARAM} {driver_deg:+.2f} deg',
        '0, with the trim on the board',
        'WARN',
        "it adds to the board's trim; racecar calibrate steering folds it into the board",
    )


def profile_copy() -> float | None:
    """Return the trim_deg of the profile's params/steering.yaml, or None."""
    try:
        data = yaml.safe_load(car_profile.param_path(CAL_NAME).read_text()) or {}
    except (OSError, yaml.YAMLError):
        return None
    value = data.get('trim_deg') if isinstance(data, dict) else None
    return float(value) if isinstance(value, (int, float)) else None


def table_lines(arcs: list[Arc]) -> list[str]:
    lines = ['Command  Curvature   Radius    Wheel angle (bicycle)']
    for a in sorted(arcs, key=lambda a: a.command):
        radius = 1.0 / a.curvature if a.curvature else math.inf
        angle = math.degrees(math.atan(WHEELBASE_M * a.curvature))
        lines.append(
            f'{a.command:+6.2f}  {a.curvature:+7.3f}/m  {radius:+7.3f} m  {angle:+6.1f} deg'
        )
    return lines


class SteeringCalibrator:
    def __init__(self, check: bool, report: str | None) -> None:
        # Before the node starts: limits that cannot give the speed raise here.
        self.plans = session_plans()
        self.deg_per_command = degrees_per_command(file_limits())
        self.check = check
        self.report = report
        self.driver = Driver()
        self.log = self.driver.log
        self.laser = laser_pose()

    def measure(self, command: float) -> Arc | None:
        bag = self.driver.drive(run_name(command), self.plans[command], prefix='steer')
        if bag is None:
            # No flip within the wait ends the step; a stop offers the run again.
            if self.driver.timed_out:
                raise ArmTimeout
            return None
        self.log.info(status_line('Analyzing the run...'))
        try:
            arc = measure_arc(read_bag(bag), self.laser, command)
        except (ValueError, StopIteration) as exc:
            self.log.error(
                f'The run could not be analyzed: {analysis_error(exc)}. Its bag is {bag}.'
            )
            return None
        if arc is None:
            self.log.error('The lidar could not measure this run. Run it again.')
            return None
        log_block(
            self.log,
            result_lines(
                [
                    f'Turned {arc.heading_deg:+.1f} deg over a {arc.chord_m:.3f} m chord: '
                    f'curvature {arc.curvature:+.3f}/m (scan match '
                    f'{arc.match.rms_m * 1e3:.1f} mm, {arc.match.inliers:.0%}).'
                ]
            ),
        )
        return arc

    def live_values(self) -> dict[str, float | None] | None:
        """Return the board's and the driver's trims from controller_node; None if it is silent."""
        from rcl_interfaces.srv import GetParameters

        node = self.driver.node
        names = [BOARD_PARAM, DRIVER_TRIM_PARAM]
        client = node.create_client(GetParameters, f'/{NODE}/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=PARAM_TIMEOUT):
                return None
            future = client.call_async(GetParameters.Request(names=names))
            self.driver.rclpy.spin_until_future_complete(node, future, timeout_sec=PARAM_TIMEOUT)
            result = future.result() if future.done() else None
        finally:
            node.destroy_client(client)
        if result is None or len(result.values) != len(names):
            return None
        return {n: parameter_float(v) for n, v in zip(names, result.values, strict=True)}

    def write_board(self, trim_deg: float) -> tuple[bool, str]:
        """Set board.steering_trim_deg; return (accepted, the controller's reason)."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters

        node = self.driver.node
        client = node.create_client(SetParameters, f'/{NODE}/set_parameters')
        try:
            if not client.wait_for_service(timeout_sec=PARAM_TIMEOUT):
                return False, f'{NODE} did not answer'
            value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=trim_deg)
            request = SetParameters.Request(parameters=[Parameter(name=BOARD_PARAM, value=value)])
            future = client.call_async(request)
            self.driver.rclpy.spin_until_future_complete(node, future, timeout_sec=PARAM_TIMEOUT)
            result = future.result() if future.done() else None
        finally:
            node.destroy_client(client)
        if result is None or not result.results:
            return False, f'{NODE} did not answer'
        outcome = result.results[0]
        return bool(outcome.successful), str(outcome.reason)

    def run(self) -> int:
        try:
            problems = limit_problems(self.driver.live_limits(), file_limits())
            if problems:
                for problem in problems:
                    self.log.error(problem)
                self.log.error(LIMITS_ADVICE)
                return 1
            return self._check() if self.check else self._calibrate()
        except ArmTimeout:
            self.log.error(TIMEOUT_ADVICE)
            return 1
        finally:
            self.driver.close()

    def _show(self, title: str) -> None:
        log_block(self.log, [step_header(title)])
        log_block(self.log, action_lines(1, [PLACE, *DEADMAN.split('\n')]))

    def _board_now(self) -> tuple[float, float] | None:
        """Return the board's and the driver's trims, after saying why when they are missing."""
        live = self.live_values()
        if live is None:
            self.log.error(f'{NODE} did not answer with its parameters. Is teleop running?')
            return None
        board = live[BOARD_PARAM]
        if board is None or not math.isfinite(board):
            self.log.error(
                f"The controller has not read the board's trim ({BOARD_PARAM} is unknown). "
                'Check the ESP32 link, then racecar service restart teleop.'
            )
            return None
        return board, live[DRIVER_TRIM_PARAM] or 0.0

    def _keep_copy(self, board: float) -> list[Path]:
        """Save the board's present trim in the profile before it changes, when not saved yet."""
        copy = profile_copy()
        if copy is not None and abs(copy - board) <= TRIM_TOL_DEG:
            return []
        self.log.info(
            status_line(
                f"Saving the board's present trim, {board:+.2f} deg, in the car profile "
                'before changing it.'
            )
        )
        return update_local_values(CAL_NAME, {'trim_deg': round(board, 2)}, TOOL)

    def _calibrate(self) -> int:
        log = self.log
        now = self._board_now()
        if now is None:
            return 1
        board, driver_trim = now
        log.info(status_line(f"The board's steering trim is {board:+.2f} deg now."))

        def show(i: int) -> None:
            self._show(f'Run {i + 1} of {len(COMMANDS)}: {describe(COMMANDS[i])}')

        def run_one(i: int) -> Arc | None:
            arc = self.measure(COMMANDS[i])
            if arc is None:
                return None
            problem = arc_problem(arc)
            if problem:
                log.error(f'{problem} Run it again.')
                return None
            return arc

        arcs = list(run_sequence(len(COMMANDS), show, run_one).values())
        if not arcs:
            log.info(status_line('Nothing measured.'))
            return 0
        fit = fit_trim(arcs)
        rows = table_lines(arcs)
        if fit is None:
            log_block(log, result_lines(rows))
            log.error(
                f'The trim needs the run at steering 0 and one at +/-{CENTER_COMMAND:g}. '
                'Nothing written.'
            )
            return 1
        correction = fit.trim_delta * self.deg_per_command
        rows += [
            f'Drift at steering 0: {fit.drift_deg_per_m:+.2f} deg/m. Gain near center: left '
            f'{fit.gain_left:.2f}/m, right {fit.gain_right:.2f}/m per unit command.',
            f'Correction: {fit.trim_delta:+.4f} on /drive, {correction:+.2f} deg on the board '
            f'({self.deg_per_command:g} deg per unit).',
        ]
        turn = self._turn_gain_line(arcs)
        if turn:
            rows.append(turn)
        log_block(log, result_lines(rows))
        new = new_board_trim(board, driver_trim, correction)
        if not math.isfinite(new):
            log.error(f'The trim came out as {new}, not a number of degrees. Nothing written.')
            return 1
        if abs(new - board) < MIN_TRIM_STEP_DEG and abs(driver_trim) < 1e-9:
            log.info(
                status_line(
                    f'The trim would change by {new - board:+.2f} deg, below '
                    f'{MIN_TRIM_STEP_DEG} deg. The board keeps {board:+.2f} deg.'
                )
            )
            for path in self._keep_copy(board):
                log.info(status_line(f'Wrote {path}'))
            return 0
        if abs(new) > TRIM_LIMIT_DEG:
            log.error(
                f'Trim {new:+.2f} deg is beyond +/-{TRIM_LIMIT_DEG:g} deg: check the servo horn '
                'and the steering linkage. Nothing written.'
            )
            return 1
        return self._write(board, driver_trim, round(new, 2))

    def _write(self, board: float, driver_trim: float, new: float) -> int:
        log = self.log
        # The runs measured with the trims read before them; a change since
        # then makes the correction wrong.
        now = self._board_now()
        if now is None:
            return 1
        if abs(now[0] - board) > TRIM_TOL_DEG or abs(now[1] - driver_trim) > 1e-9:
            log.error(
                f'The trims changed during the runs: board {board:+.2f} -> {now[0]:+.2f} deg, '
                f'driver {driver_trim:+.2f} -> {now[1]:+.2f} deg. Nothing written. Run the '
                'step again.'
            )
            return 1
        written = self._keep_copy(board)
        accepted, reason = self.write_board(new)
        if not accepted:
            log.error(
                f'The board did not take the trim: {reason}. The board keeps {board:+.2f} deg.'
            )
            return 1
        # The controller accepts the write only after the board reads it back,
        # so a parameter read that does not answer still records the new trim.
        back = self.live_values()
        read = back.get(BOARD_PARAM) if back else None
        if read is not None and abs(read - new) > TRIM_TOL_DEG:
            log.error(
                f'The controller reads {read:+.2f} deg after writing {new:+.2f} deg. The '
                f'profile copy keeps {board:+.2f} deg. Check: racecar calibrate status'
            )
            return 1
        written += update_local_values(
            CAL_NAME, {'trim_deg': new, 'previous_trim_deg': round(board, 2)}, TOOL
        )
        lines = [
            f'{BOARD_PARAM}: {board:+.2f} -> {new:+.2f} deg, saved on the board. It also '
            'trims manual driving.'
        ]
        if abs(driver_trim) >= 1e-9:
            written += write_local_yaml('controller', NODE, {DRIVER_TRIM_PARAM: 0.0}, TOOL, True)
            lines.append(
                f'The driver trim {driver_trim:+.2f} deg is now on the board. Restart teleop so '
                f'{DRIVER_TRIM_PARAM} loads 0: racecar service restart teleop'
            )
        # The copy kept before the write and the one after it are one file.
        lines += [f'Wrote {path}' for path in dict.fromkeys(written)]
        log_block(log, result_lines(lines))
        return 0

    def _turn_gain_line(self, arcs: list[Arc]) -> str:
        """Return the encoder over-read in turns as a gain for odometry.turn_gain; not written."""
        odom = node_params('odom', NODE)
        current = float(odom.get('odometry.turn_gain', 0.0))
        wheelbase = float(odom.get('odometry.wheelbase_m', WHEELBASE_M))
        fit = fit_turn_gain(arcs, current, wheelbase)
        if fit is None:
            return ''
        return (
            f'Encoder over-read in turns: gain {fit.gain:.3f} from {fit.runs} runs; '
            f'straight runs read {fit.straight_ratio - 1:+.2%}. '
            f'odometry.turn_gain stays at {current:.3f}.'
        )

    def _check(self) -> int:
        log = self.log
        criteria = []
        self._show(f'Straight drive at {describe(0.0)}')
        arc = self.measure(0.0)
        if arc is None:
            criteria.append(Criterion('straight drift', 'no result', '', 'FAIL', 'see above'))
        else:
            criteria.append(match_criterion('scan match', arc.match))
            gap = abs(arc.heading_deg - arc.odom_heading_deg)
            criteria.append(
                Criterion(
                    'lidar vs odometry',
                    f'heading {arc.heading_deg:+.1f} vs {arc.odom_heading_deg:+.1f} deg',
                    f'within {MAX_HEADING_DISAGREEMENT_DEG:.0f} deg',
                    'PASS' if gap <= MAX_HEADING_DISAGREEMENT_DEG else 'FAIL',
                    'the scan match is wrong: run the check again',
                )
            )
            criteria.append(drift_criterion(math.degrees(arc.curvature)))
        live = self.live_values()
        criteria.append(trim_criterion(profile_copy(), live.get(BOARD_PARAM) if live else None))
        driver = driver_trim_criterion((live or {}).get(DRIVER_TRIM_PARAM) or 0.0)
        if driver is not None:
            criteria.append(driver)
        return finish_check(log, criteria, self.report)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('--check', action='store_true', help='check the trim in use')
    parser.add_argument('--report', help='with --check: write the result as JSON to this file')
    args = parser.parse_args(argv)
    try:
        calibrator = SteeringCalibrator(args.check, args.report)
    except ValueError as exc:
        print(f'The steering step cannot run: {exc}.', file=sys.stderr)
        return 1
    return calibrator.run()


if __name__ == '__main__':
    sys.exit(main())
