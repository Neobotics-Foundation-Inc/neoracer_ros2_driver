#!/usr/bin/env python3
"""
Guided whole-car calibration behind `racecar calibrate`.

Runs the calibrate_*.py tools in this folder in a fixed order, each with its
own prompts, then shows old and new values and offers to restart teleop so
the nodes load them. `racecar calibrate status` lists each calibration's state without ROS.
`racecar calibrate check` runs each tool's --check against the live sensors,
the last step of car prep: calibrate, restart teleop, check.

The tools write the car profile (params/<name>.yaml under ~/.config/racecar,
neoracer_ros2_driver.car_profile), outside the driver folder; launch reads
it, so no rebuild is needed. The steering trim is set in the ESP32 firmware
(`trim set`), which also trims manual driving; the profile keeps a copy.

The NeoRacer runs four steps: lidar, odom, steering and dynamics. Its IMU is
fused and bias-corrected on the ESP32, so upstream's imu, mag and realsense
steps are not here. A step whose tool is not installed reports so instead of
failing.

Where a step drives the car by itself, upstream holds RB on the gamepad. On
the NeoRacer that is the handset's mode switch: down (autonomy) lets the car
drive, up (manual) stops it. A switch stays where it is put, so the setup text
of every such step says how to stop.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import shlex
import subprocess
import sys
import tempfile
from typing import Any

from neoracer_ros2_driver import car_profile
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from calibrate_common import (  # noqa: E402, I100
    criteria_lines,
    Criterion,
    paint,
    read_line,
    SESSION_ENV,
    source_config_dir,
    use_color,
)

TELEOP_UNIT = 'neoracer-teleop'
RUN_PROMPT = 'Press Enter to run, s to skip, and q to stop: '
RUN_PROMPT_BACK = 'Press Enter to run, s to skip, b to go back a step, and q to stop: '
LIDAR_SETUP = (
    'Use a clear straight path of about 2 meters with walls or furniture nearby.\n'
    'Turn the handset on. The car drives itself only with the mode switch DOWN (autonomy).\n'
    'Flip the mode switch UP to stop.'
)
STEERING_SETUP = (
    'Use a clear area of about 3 by 3 meters surrounded by walls or furniture.\n'
    'Turn the handset on. The car drives itself only with the mode switch DOWN (autonomy).\n'
    'Flip the mode switch UP to stop.'
)
DYNAMICS_SETUP = (
    'Use a clear straight path of about 4 meters with walls or furniture nearby.\n'
    'Use a clear area of about 2.3 by 1.6 meters for driving circles.\n'
    'Turn the handset on. The car drives itself only with the mode switch DOWN (autonomy).\n'
    'Flip the mode switch UP to stop.'
)
# calibrate_dynamics.py writes these; per speed (0.25, 0.5, 1.0) and per side.
DYNAMICS_KEYS = tuple(
    [
        f'{name}_{tag}'
        for tag in ('025', '05', '10')
        for name in ('steady_speed_mps', 'rise_90_s', 'stop_distance_mm', 'stop_time_s')
    ]
    + [
        f'{name}_{side}'
        for side in ('right', 'left')
        for name in (
            'turn_radius_mm',
            'steer_inner_deg',
            'steer_outer_deg',
            'outer_front_track_diameter_mm',
            'encoder_turn_overread',
            'servo_center_to_lock_s',
        )
    ]
    + ['servo_lock_to_lock_s']
)
ODOM_SETUP = (
    'Use a room with walls or furniture nearby.\n'
    'Clear about 2 meters around the car for driving circles.\n'
    'Clear a straight path of 3 to 5 meters that ends at a wall or furniture.'
)


@dataclass(frozen=True)
class Step:
    """One calibration in the sequence."""

    key: str
    label: str
    tool: str
    position: str
    local_name: str
    keys: tuple[str, ...]
    check: str = ''  # how to place the car for the tool's --check; empty: no check
    optional: bool = False  # runs only when named
    note: str = ''  # where the result is applied, when not only in the profile
    requires: tuple[str, ...] = ()  # other tools in this folder the tool imports

    @property
    def missing(self) -> list[str]:
        """Return the step's tool when absent, else the absent tools it imports."""
        if not (HERE / self.tool).is_file():
            return [self.tool]
        return [name for name in self.requires if not (HERE / name).is_file()]

    @property
    def installed(self) -> bool:
        """Whether the step's tool, and every tool it imports, is in this driver."""
        return not self.missing

    @property
    def missing_text(self) -> str:
        """Say which tools are not in this driver yet."""
        names = self.missing
        if len(names) == 1:
            return f'{names[0]} is not in this driver yet'
        return f'{", ".join(names[:-1])} and {names[-1]} are not in this driver yet'


# Order: the lidar yaw first, since the other steps measure against the lidar.
STEPS = (
    Step(
        'lidar',
        'Lidar yaw',
        'calibrate_lidar.py',
        LIDAR_SETUP,
        'frames',
        ('laser_yaw_deg',),
        check=LIDAR_SETUP,
        # The driving method reuses their bag reader, driver and scan match.
        requires=('calibrate_odom.py', 'calibrate_dynamics.py'),
    ),
    Step(
        'odom',
        'Odometry yaw and speed scale',
        'calibrate_odom.py',
        ODOM_SETUP,
        'odom',
        ('odometry.yaw_rate_scale', 'odometry.speed_scale', 'odometry.turn_gain'),
        check=ODOM_SETUP,
    ),
    # The trim is set in the ESP32 firmware with `trim set`; params/steering.yaml
    # keeps a copy, so a new board or a full flash can be set back.
    Step(
        'steering',
        'Steering trim, set in the ESP32 firmware',
        'calibrate_steering.py',
        STEERING_SETUP,
        'steering',
        ('trim_deg',),
        check=STEERING_SETUP,
        note='applied in the ESP32 firmware (trim set); the profile keeps a copy',
    ),
    # Measured for the simulator; nothing reads it back, so there is no check.
    Step(
        'dynamics',
        'Vehicle dynamics for the simulator',
        'calibrate_dynamics.py',
        DYNAMICS_SETUP,
        'dynamics',
        DYNAMICS_KEYS,
        optional=True,
    ),
)
STEP_KEYS = [s.key for s in STEPS]
OPTIONAL_KEYS = [s.key for s in STEPS if s.optional]


def select_steps(names: list[str]) -> list[Step]:
    """Return the named steps in table order; with no names, every step but the optional ones."""
    if names:
        return [s for s in STEPS if s.key in names]
    return [s for s in STEPS if not s.optional]


def setup_text(text: str) -> str:
    """Return a step's setup lines under a `Setup:` label, one instruction per line."""
    return 'Setup: ' + text.replace('\n', '\n       ')


def read_values(path: Path) -> dict[str, Any]:
    """Return a local YAML's values: a ROS parameter file's parameters, or a plain mapping."""
    try:
        data = yaml.safe_load(path.read_text()) or {}
    except (OSError, yaml.YAMLError):
        return {}
    if not isinstance(data, dict):
        return {}
    if len(data) == 1:
        (inner,) = data.values()
        if isinstance(inner, dict) and 'ros__parameters' in inner:
            return dict(inner['ros__parameters'] or {})
    return data


def step_values(step: Step) -> dict[str, Any]:
    values = read_values(car_profile.param_path(step.local_name))
    return {k: values[k] for k in step.keys if k in values}


def status_verdict(step: Step, state: str) -> str:
    """Return the color class of a status row: PASS, WARN, or FAIL."""
    if state == 'calibrated':
        return 'PASS'
    if state == 'not installed':
        return 'WARN'
    # Uncalibrated sensors run on zero bias and unit scale; dynamics is optional.
    return 'FAIL' if step.check else 'WARN'


def status_lines(config_dir: Path, color: bool = False) -> list[str]:
    """Return `racecar calibrate status`: one block per step, blank lines between."""
    lines: list[str] = [f'Car profile: {car_profile.profile_dir()}']
    for step, (key, state, detail) in zip(STEPS, status_rows(config_dir), strict=True):
        if lines:
            lines.append('')
        header = f'{key}: {step.label}'
        lines.append(f'\033[1m{header}\033[0m' if color else header)
        verdict = status_verdict(step, state)
        lines.append(
            f'  {paint(state, verdict, color, bold=True)}  {paint(detail, verdict, color)}'
        )
        if step.note:
            lines.append(f'  {step.note}')
        values = step_values(step)
        width = max((len(k) for k in values), default=0)
        lines += [f'  {k:<{width}}  {fmt(values[k], 7)}' for k in step.keys if k in values]
    return lines


def status_rows(config_dir: Path) -> list[tuple[str, str, str]]:
    """Return (step, state, detail) for each step."""
    rows = []
    for step in STEPS:
        local = car_profile.param_path(step.local_name)
        values = step_values(step)
        if values:
            stamp = datetime.fromtimestamp(local.stat().st_mtime).strftime('%Y-%m-%d %H:%M')
            rows.append((step.key, 'calibrated', stamp))
        elif not step.installed:
            rows.append((step.key, 'not installed', step.missing_text))
        elif (config_dir / f'{step.local_name}.yaml').exists():
            rows.append((step.key, 'default', 'committed defaults; not calibrated on this car'))
        else:
            rows.append((step.key, 'default', 'not measured on this car'))
    return rows


def fmt(value: Any, digits: int = 4) -> str:
    if isinstance(value, list):
        return '[' + ', '.join(fmt(v, digits) for v in value) + ']'
    return f'{value:.{digits}g}' if isinstance(value, float) else str(value)


def systemctl() -> list[str]:
    return shlex.split(os.environ.get('RACECAR_SYSTEMCTL', 'systemctl'))


def teleop_active() -> bool:
    result = subprocess.run(
        [*systemctl(), 'is-active', TELEOP_UNIT], capture_output=True, text=True, check=False
    )
    return result.stdout.strip() == 'active'


def ask(prompt: str) -> str:
    """Prompt for one line (calibrate_common.read_line); end of input reads as q."""
    try:
        return read_line(prompt).strip().lower()
    except (EOFError, OSError):
        return 'q'


def walk_steps(steps: list[Step], header: Any, run: Any) -> list[Any]:
    """
    Prompt for each step in order: Enter runs it, s skips, b goes back a step, q stops.

    `header(i)` prints the step's banner and `run(i)` returns its result entry.
    A skipped step gives (step, 'skipped') and the steps left by q give
    (step, 'not run'). Returns one entry per step reached, in order.
    """
    entries: dict[int, Any] = {}
    i = 0
    while i < len(steps):
        header(i)
        answer = ask(RUN_PROMPT_BACK if i > 0 else RUN_PROMPT)
        if answer == 'b':
            i = max(i - 1, 0)
            continue
        if answer == 'q':
            for k in range(i, len(steps)):
                entries[k] = (steps[k], 'not run')
            break
        entries[i] = (steps[i], 'skipped') if answer == 's' else run(i)
        i += 1
    return [entries[k] for k in sorted(entries)]


def run_steps(steps: list[Step]) -> int:
    """Run the steps in order; return 0 when every attempted step succeeded."""
    # A step run again keeps its values from before its first run, for the summary.
    first_before: dict[int, dict[str, Any]] = {}

    def header(i: int) -> None:
        print(f'\n=== {i + 1}/{len(steps)} {steps[i].label} ===')
        print(setup_text(steps[i].position))

    def run(i: int) -> tuple[Step, str, dict[str, Any], dict[str, Any]]:
        step = steps[i]
        before = first_before.setdefault(i, step_values(step))
        if not step.installed:
            print(f'{step.missing_text}; nothing to run.')
            return step, 'not installed', before, before
        rc = subprocess.run(
            [sys.executable, str(HERE / step.tool)],
            check=False,
            env={**os.environ, SESSION_ENV: '1'},
        ).returncode
        return step, 'done' if rc == 0 else 'failed', before, step_values(step)

    results = [e if len(e) == 4 else (*e, {}, {}) for e in walk_steps(steps, header, run)]

    print('\n=== Summary ===')
    for step, outcome, before, after in results:
        print(f'{step.key:10s} {outcome}')
        for key in step.keys if outcome == 'done' else ():
            old = fmt(before[key]) if key in before else 'default'
            print(f'    {key}: {old} -> {fmt(after.get(key, "?"))}')

    if any(outcome == 'done' for _, outcome, *_ in results):
        restart = [*shlex.split(os.environ.get('RACECAR_SUDO', 'sudo')), *systemctl()]
        restart += ['restart', TELEOP_UNIT]
        print('Restarting teleop uses sudo, which may ask for your password.')
        if ask('Restart teleop now so the nodes load the new values? [y/N] ') == 'y':
            subprocess.run(restart, check=False)
        else:
            print(f'Apply later: {shlex.join(restart)}')
        print('Then verify: racecar calibrate check')
    failed = [step.key for step, outcome, *_ in results if outcome == 'failed']
    return 1 if failed else 0


def read_report(path: Path) -> tuple[str, list[Criterion]] | None:
    """Return (overall, criteria) from a tool's --report file, or None if absent."""
    try:
        data = json.loads(path.read_text())
        return data['overall'], [Criterion(**c) for c in data['criteria']]
    except (OSError, ValueError, KeyError, TypeError):
        return None


def summary_lines(
    results: list[tuple[Step, str, list[Criterion]]], color: bool = False
) -> list[str]:
    """Render the end-of-run table: each step's verdict, then each criterion."""
    lines = ['', '=== Check summary ===']
    for step, verdict, criteria in results:
        mark = paint(f'{verdict:7s}', verdict, color, bold=True)
        lines.append(f'{step.key:10s} {mark} {step.label}')
        lines += criteria_lines(criteria, indent='    ', color=color)
    counts = {
        v: sum(1 for _, verdict, _ in results if verdict == v) for v in ('PASS', 'WARN', 'FAIL')
    }
    skipped = sum(1 for _, verdict, _ in results if verdict == 'skipped')
    not_run = sum(1 for _, verdict, _ in results if verdict == 'not run')
    missing = sum(1 for _, verdict, _ in results if verdict == 'not installed')
    tallies = '  '.join(
        paint(f'{counts[v]} {v.lower()}', v, color and counts[v] > 0)
        for v in ('PASS', 'WARN', 'FAIL')
    )
    worst = 'FAIL' if counts['FAIL'] else 'WARN' if counts['WARN'] else 'PASS'
    lines.append(
        f'RESULT {paint(worst, worst, color, bold=True)}  {tallies}  {skipped} skipped'
        + (f'  {not_run} not run' if not_run else '')
        + (f'  {missing} not installed' if missing else '')
    )
    return lines


def run_checks(steps: list[Step]) -> int:
    """Run each step's --check; print a per-criterion summary; return 1 when any failed."""
    with tempfile.TemporaryDirectory() as tmp:

        def header(i: int) -> None:
            print(f'\n=== Check {i + 1}/{len(steps)}: {steps[i].label} ===')
            print(setup_text(steps[i].check))

        def run(i: int) -> tuple[Step, str, list[Criterion]]:
            step = steps[i]
            if not step.installed:
                print(f'{step.missing_text}; nothing to check.')
                return step, 'not installed', []
            report = Path(tmp) / f'{step.key}.json'
            report.unlink(missing_ok=True)
            cmd = [sys.executable, str(HERE / step.tool), '--check', '--report', str(report)]
            rc = subprocess.run(cmd, check=False).returncode
            parsed = read_report(report)
            if parsed is None:
                criteria = [
                    Criterion('check', f'exit {rc}', 'a result', 'FAIL', 'see output above')
                ]
                return step, 'FAIL', criteria
            verdict, criteria = parsed
            return step, verdict, criteria

        results = [e if len(e) == 3 else (*e, []) for e in walk_steps(steps, header, run)]
    print('\n'.join(summary_lines(results, use_color())))
    return 1 if any(verdict == 'FAIL' for _, verdict, _ in results) else 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        prog='racecar calibrate',
        description='Calibrate the car: lidar yaw, odometry, steering trim, dynamics. '
        '`status` lists what is calibrated.',
    )
    parser.add_argument(
        'steps',
        nargs='*',
        metavar='step',
        help=f'status; check [steps]; or steps to run in order ({", ".join(STEP_KEYS)}); '
        f'default: all but {", ".join(OPTIONAL_KEYS)}',
    )
    args = parser.parse_args(argv)

    config_dir = source_config_dir(Path(__file__))
    if config_dir is None:
        print('No package config directory found.', file=sys.stderr)
        return 1

    if args.steps == ['status']:
        print('\n'.join(status_lines(config_dir, color=use_color())))
        return 0

    checking = bool(args.steps) and args.steps[0] == 'check'
    if checking:
        args.steps = args.steps[1:]
    unknown = [s for s in args.steps if s not in STEP_KEYS]
    if unknown:
        parser.print_usage(sys.stderr)
        print(f'unknown step: {", ".join(unknown)}', file=sys.stderr)
        return 2
    chosen = select_steps(args.steps)
    if checking:
        chosen = [s for s in chosen if s.check]
        if not chosen:
            return 2
    if not teleop_active():
        print(
            f'{TELEOP_UNIT} is not running. Start it with racecar service start teleop, '
            'then run racecar calibrate again.'
        )
        return 1
    return run_checks(chosen) if checking else run_steps(chosen)


if __name__ == '__main__':
    sys.exit(main())
