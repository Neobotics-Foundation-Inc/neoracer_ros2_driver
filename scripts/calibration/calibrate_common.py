#!/usr/bin/env python3
"""
Shared plumbing for the calibrate_*.py tools.

Results go to the car profile (params/<name>.yaml under ~/.config/racecar,
neoracer_ros2_driver.car_profile), outside the driver folder. Launch loads
them after the tracked zero-default YAMLs, so the car's values win, and reads
them at launch, so a teleop restart applies them. The steering trim is the
exception: it lives in the ESP32 firmware (`trim set`), and the profile keeps
a copy.

The NeoRacer has no IMU, magnetometer or RealSense calibration step: the ESP32
fuses and bias-corrects the IMU on board, so upstream's IMU bias tools and
their shared code are not here.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import sys
import termios
import threading
import time
from typing import Any

import numpy as np
from neoracer_ros2_driver import car_profile
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

PACKAGE = 'neoracer_ros2_driver'
# Operator-facing output: the message only, without the ROS severity, time and node.
# rcutils reads this at rclpy.init(), which every tool calls after importing this.
os.environ.setdefault('RCUTILS_CONSOLE_OUTPUT_FORMAT', '{message}')
# Set by calibrate.py for the tools it runs; it backs up and restarts teleop itself.
SESSION_ENV = 'RACECAR_CALIBRATE_SESSION'
FIRST_MESSAGE_TIMEOUT = 15.0
PROGRESS_INTERVAL = 5.0


def install_config_dir() -> Path | None:
    """Return the installed share config dir, or None when not installed."""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory(PACKAGE)) / 'config'
    except (ImportError, KeyError):
        return None
    return share if share.is_dir() else None


def source_config_dir(script: Path | None = None) -> Path | None:
    """
    Return the package's source config dir, or None.

    Checks the package root above the resolved script first (source checkout
    or symlink install), then <ws>/src/<package>/config derived from the
    install prefix.
    """
    for here in (script or Path(__file__)).resolve().parents:
        if (here / 'setup.py').is_file() and (here / 'config').is_dir():
            return here / 'config'
    share = install_config_dir()
    if share is not None and '/install/' in str(share):
        ws = Path(str(share).split('/install/')[0])
        candidate = ws / 'src' / PACKAGE / 'config'
        if candidate.is_dir():
            return candidate
    return None


def write_local_yaml(
    name: str,
    node: str,
    params: dict[str, Any],
    tool: str,
    merge: bool = False,
) -> list[Path]:
    """
    Write `params` for `node` to the profile's params/<name>.yaml.

    With `merge`, parameters already in the file for `node` are kept unless
    `params` sets them, so a tool that calibrates in steps can write each step.
    The file it replaces is kept under the profile's history/.
    """
    stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    header = f'# Per-car calibration from {tool}, {stamp}.\n'
    return [car_profile.write_params(name, node, params, header, merge)]


def update_local_values(name: str, values: dict[str, float], tool: str) -> list[Path]:
    """
    Set `values` in the profile's params/<name>.yaml, keeping its other keys.

    For plain key: value files such as frames.yaml, which several tools write
    into (not ROS parameter files).
    """
    stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    header = f'# Per-car values; last written by {tool}, {stamp}.\n'
    return [car_profile.write_values(name, values, header)]


def backup_reminder(paths: list[Path]) -> str:
    """
    Return the post-write message: where the file went and how to apply it.

    Under `racecar calibrate` (SESSION_ENV set) only the paths: the session
    offers the teleop restart itself.
    """
    lines = [f'Wrote {p}' for p in paths]
    if paths and not os.environ.get(SESSION_ENV):
        lines.append('Apply it: racecar service restart teleop (no rebuild needed).')
    return '\n'.join(lines)


def node_params(cal_name: str, node: str, config_dir: Path | None = None) -> dict[str, Any]:
    """
    Return `node`'s parameters from `<cal_name>.yaml` with the profile override on top.

    The defaults come from the config the running stack loaded: the installed
    share directory, else the source tree.
    """
    config_dir = config_dir or install_config_dir() or source_config_dir()
    return car_profile.node_params(cal_name, node, config_dir)


# ---------------------------------------------------------------------------
# Check results (--check), shared by every tool and read by calibrate.py
# ---------------------------------------------------------------------------

STATUS_ORDER = ('INFO', 'PASS', 'WARN', 'FAIL')


@dataclass(frozen=True)
class Criterion:
    """One line of a --check result. status: PASS, WARN, FAIL, or INFO (no verdict)."""

    name: str
    measured: str
    limit: str
    status: str
    advice: str = ''


def overall(criteria: list[Criterion]) -> str:
    """Return the worst status among the criteria (INFO lines do not count)."""
    judged = [c.status for c in criteria if c.status != 'INFO']
    return max(judged, key=STATUS_ORDER.index) if judged else 'PASS'


# Same colors and opt-outs as `racecar status` (scripts/diagnose.py).
ANSI = {'PASS': '32', 'WARN': '33', 'FAIL': '31'}


def use_color() -> bool:
    """Color only an interactive terminal; NO_COLOR (no-color.org) opts out."""
    return (
        sys.stdout.isatty()
        and 'NO_COLOR' not in os.environ
        and os.environ.get('TERM', '') != 'dumb'
    )


def paint(text: str, status: str, color: bool, bold: bool = False) -> str:
    code = ANSI.get(status)
    if not color or code is None:
        return text
    return f'\033[{"1;" if bold else ""}{code}m{text}\033[0m'


def criteria_lines(
    criteria: list[Criterion], indent: str = '  ', color: bool = False
) -> list[str]:
    """Render criteria as aligned table lines, advice on its own line below a WARN or FAIL."""
    lines = []
    for c in criteria:
        status = paint(f'{c.status:5s}', c.status, color)
        lines.append(f'{indent}{status} {c.name:18s} {c.measured:28s} {c.limit}'.rstrip())
        if c.advice and c.status in ('WARN', 'FAIL'):
            lines.append(f'{indent}      -> {c.advice}')
    return lines


def finish_check(log: Any, criteria: list[Criterion], report: str | None) -> int:
    """Log the criteria table, write the JSON report, and return the exit code (1 on FAIL)."""
    import json

    for line in criteria_lines(criteria):
        log.info(line)
    verdict = overall(criteria)
    log.info(f'Check: {verdict}')
    if report:
        Path(report).write_text(
            json.dumps({'overall': verdict, 'criteria': [c.__dict__ for c in criteria]})
        )
    return 1 if verdict == 'FAIL' else 0


def read_line(prompt: str = '') -> str:
    """
    input() for these tools: print the prompt, read one line a byte at a time.

    input() reads a piped stdin in blocks and would swallow the lines meant for
    the next prompt or the next tool, so a scripted run (one line per prompt)
    stops early. Raises EOFError at end of input, as input() does.

    On a terminal, keys pressed before the prompt appears are discarded, so an
    extra Enter during a recording cannot answer the next prompt.
    """
    if os.isatty(0):
        termios.tcflush(0, termios.TCIFLUSH)
    print(prompt, end='', flush=True)
    line = bytearray()
    while True:
        ch = os.read(0, 1)
        if not ch:
            if line:
                return line.decode(errors='replace')
            raise EOFError
        if ch == b'\n':
            return line.decode(errors='replace')
        line += ch


STEP_PROMPT = 'Press Enter to run this step, or s to skip it: '
STEP_PROMPT_BACK = 'Press Enter to run this step, s to skip it, or b to redo the previous step: '


def step_prompt(index: int) -> str:
    """Return the run prompt for step `index`; b is offered from the second step on."""
    return STEP_PROMPT_BACK if index > 0 else STEP_PROMPT


def run_sequence(
    count: int,
    show: Callable[[int], None],
    run: Callable[[int], Any],
    prompt: Callable[[int], str] = step_prompt,
    allow_skip: bool = True,
) -> dict[int, Any]:
    """
    Run steps 0..count-1 in order; return {index: result} for the steps that ran.

    At each step's prompt Enter runs it, s skips it (when allowed), and b goes
    back to the previous step and discards its result. A step whose run
    returns None (stopped, or a failed measurement) is offered again.
    """
    results: dict[int, Any] = {}
    i = 0
    while i < count:
        show(i)
        answer = read_line(prompt(i)).strip().lower()
        if answer == 'b':
            if i > 0:
                i -= 1
                results.pop(i, None)
            continue
        if answer == 's' and allow_skip:
            results.pop(i, None)
            i += 1
            continue
        result = run(i)
        if result is None:
            continue
        results[i] = result
        i += 1
    return results


def mean_vector(samples: list[list[float]]) -> list[float]:
    """Return the per-axis mean of a list of 3-vectors."""
    return [float(v) for v in np.asarray(samples, dtype=float).mean(axis=0)]


class CalibratorNode(Node):
    """
    Base node: one sensor subscription, a first-message gate, timed collection.

    Subclasses implement `extract(msg)` (one sample per message) and `run()`,
    which returns the process exit code. `run()` executes on a worker thread
    while `spin_until_done` spins the node on the main thread.
    """

    def __init__(self, node_name: str, msg_type: Any, topic: str) -> None:
        super().__init__(node_name)
        self.topic = topic
        self._first = threading.Event()
        self._lock = threading.Lock()
        self._collecting = False
        self._samples: list[list[float]] = []
        self.create_subscription(msg_type, topic, self._callback, qos_profile_sensor_data)

    def extract(self, msg: Any) -> list[float]:
        raise NotImplementedError

    def run(self) -> int:
        raise NotImplementedError

    def _callback(self, msg: Any) -> None:
        self._first.set()
        if self._collecting:
            sample = self.extract(msg)
            with self._lock:
                self._samples.append(sample)

    def wait_for_first_message(self, timeout: float = FIRST_MESSAGE_TIMEOUT) -> bool:
        self.get_logger().info(f'Waiting for {self.topic}...')
        if self._first.wait(timeout):
            return True
        self.get_logger().error(f'No messages on {self.topic} after {timeout:.0f} seconds.')
        self.get_logger().error(f'Check: ros2 topic hz {self.topic}')
        return False

    def collect(self, duration: float, description: str) -> list[list[float]]:
        """Record samples for `duration` seconds and return them."""
        with self._lock:
            self._samples = []
        self.get_logger().info(f'{description}: collecting for {duration:.0f} seconds')
        self._collecting = True
        start = time.monotonic()
        next_report = start + PROGRESS_INTERVAL
        while (now := time.monotonic()) - start < duration:
            if now >= next_report:
                with self._lock:
                    n = len(self._samples)
                remaining = duration - (now - start)
                self.get_logger().info(f'{description}: {remaining:.0f} seconds left, {n} samples')
                next_report += PROGRESS_INTERVAL
            time.sleep(0.1)
        self._collecting = False
        with self._lock:
            samples = list(self._samples)
        self.get_logger().info(f'{description}: {len(samples)} samples')
        return samples


def spin_until_done(node: CalibratorNode) -> int:
    """Run `node.run()` on a worker thread, spin on this one, return its exit code."""
    result = [1]

    def work() -> None:
        try:
            result[0] = node.run()
        except EOFError:
            node.get_logger().error('Input closed; calibration aborted.')
        except Exception as exc:  # noqa: BLE001
            node.get_logger().error(f'Calibration failed: {exc}')

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    worker = threading.Thread(target=work, daemon=True)
    worker.start()
    try:
        while worker.is_alive() and rclpy.ok():
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        node.get_logger().info('Calibration interrupted.')
        result[0] = 1
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()
    return result[0]
