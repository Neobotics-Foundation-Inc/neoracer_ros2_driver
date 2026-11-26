#!/usr/bin/env python3
"""
One-terminal SLAM validation session on the car.

    slam_session.py map NAME [--camera off] [--inference on]
    slam_session.py localize MAP [--localizer amcl|slam_toolbox] [--pose x,y,yaw] [--inference on]
    slam_session.py analyze [SESSION_DIR]        # rewrite report.md; default the newest
    slam_session.py replay SESSION_DIR TAG [--localizer amcl|slam_toolbox] [--autonomy]
                           [--set key=value ...]

A session stops the teleop service, then starts teleop (inference off unless
--inference on), the
autonomy launch, foxglove_bridge (Lichtblick: ws://<car>:8765), a bag of the
SLAM topics, and the load recorder, each logging into the session folder.
Keys while driving: m mark, e event note, g global search (AMCL), s save the
map (mapping), t timed save (mapping), q quit. On quit everything stops in
order, a mapping run is saved under the map root, and report.md and map.png
are written to
slam_eval/sessions/<time>-<mode>-<name>/ under the data root. Driving stays on the
handset; the session publishes no drive commands.

`replay` runs a session's bag through a fresh mapper (mapping runs) or
localizer (localization runs; --localizer picks AMCL or slam_toolbox on the
same bag) with the config file plus the --set overrides, on its own ROS
domain so a running car is not disturbed, and writes report.md (and a map for
mapping) to <session>/replay/<TAG>/. The bag's map -> odom transforms are
dropped. Reports score the scan-to-map fit (scan_fit.py).
"""

from __future__ import annotations

import argparse
from collections.abc import Sequence
import csv
from dataclasses import dataclass, field
from datetime import datetime
import math
import os
from pathlib import Path
import re
import select
import shutil
import signal
import subprocess
import sys
import threading
import time
from typing import Any

from neoracer_ros2_driver import car_profile
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
REPO = HERE.parent.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent))

import neoracer_log  # noqa: E402, I100
import scan_fit  # noqa: E402
import slam_eval  # noqa: E402
import sysinfo  # noqa: E402

SESSIONS = slam_eval.EVAL_DIR / 'sessions'
TELEOP_UNIT = 'neoracer-teleop.service'
FOXGLOVE_PARAMS = HERE / 'foxglove_validation.yaml'
FOXGLOVE_PORT = 8765
BAG_TOPICS = ('/scan', '/tf', '/tf_static', '/odom/integrated', '/pose', '/amcl_pose', '/map')
READY_TIMEOUT = 90.0
LOCALIZER_TIMEOUT = 60.0
SERVICE_TIMEOUT = 120.0
# Storage: warn below either limit; refuse to start below the last.
DISK_WARN_USED = 0.80
DISK_WARN_FREE_GB = 10.0
DISK_REFUSE_FREE_GB = 3.0
DISK_CHECK_SEC = 30.0
# Seconds to wait after SIGINT, then SIGTERM, before SIGKILL. On upstream's car
# slam_toolbox ignores both and is killed about 15 s after SIGINT.
STOP_WAIT = (12.0, 5.0)
LOOP_PATTERN = re.compile(r'loop clos', re.IGNORECASE)
REPLAY_DOMAIN = 77
REPLAY_TOPICS = ('/scan', '/tf', '/tf_static', '/odom/integrated')
REPLAY_SETTLE_SEC = 6.0


# --- pure helpers --------------------------------------------------------------


def session_dir_name(mode: str, name: str, now: datetime) -> str:
    return f'{now:%Y%m%d-%H%M%S}-{mode}-{name}'


def teleop_argv(camera: bool, inference: bool = False) -> list[str]:
    argv = ['bash', str(REPO / 'scripts' / 'launch_teleop.sh')]
    if not inference:
        argv.append('inference_enable:=false')
    if not camera:
        argv.append('camera_enable:=false')
    return argv


def autonomy_argv(
    mode: str, map_name: str, localizer: str, pose: str, session: str = ''
) -> list[str]:
    argv = [
        'ros2', 'launch', 'neoracer_ros2_driver', 'autonomy.launch.py',
        f'mode:={mode}', f'map:={map_name}', f'session:={session}',
    ]  # fmt: skip
    if mode == 'localization':
        argv.append(f'localizer:={localizer}')
        if pose:
            argv.append(f'pose:={pose}')
    return argv


def foxglove_argv() -> list[str]:
    return [
        'ros2', 'run', 'foxglove_bridge', 'foxglove_bridge',
        '--ros-args', '--params-file', str(FOXGLOVE_PARAMS),
    ]  # fmt: skip


def bag_argv(out: Path) -> list[str]:
    return ['ros2', 'bag', 'record', '--output', str(out), '--storage', 'mcap', *BAG_TOPICS]


def disk_status(total: int, used: int, free: int) -> tuple[str, str]:
    """Return (level, text): level 'ok', 'warn', or 'refuse'."""
    free_gb = free / 1e9
    text = f'{free_gb:.1f} GB free of {total / 1e9:.0f} GB ({used / total:.0%} used)'
    if free_gb < DISK_REFUSE_FREE_GB:
        return 'refuse', text
    if used / total >= DISK_WARN_USED or free_gb < DISK_WARN_FREE_GB:
        return 'warn', text
    return 'ok', text


def largest_entries(root: Path, count: int = 5) -> list[tuple[int, Path]]:
    """Return the largest direct children of `root` by total size, largest first."""
    sizes = []
    for child in root.iterdir() if root.is_dir() else []:
        if child.is_symlink():
            continue
        if child.is_file():
            sizes.append((child.stat().st_size, child))
            continue
        total = 0
        for path in child.rglob('*'):
            try:
                if path.is_file() and not path.is_symlink():
                    total += path.stat().st_size
            except OSError:
                continue
        sizes.append((total, child))
    return sorted(sizes, reverse=True)[:count]


def max_gap(times: Sequence[float], t0: float, t1: float) -> float:
    """Longest interval between consecutive times that overlaps [t0, t1]; 0 without two."""
    gap = 0.0
    for a, b in zip(times, times[1:], strict=False):
        if b >= t0 and a <= t1:
            gap = max(gap, b - a)
    return gap


@dataclass
class Drift:
    path_m: float
    final: tuple[float, float, float]  # map -> odom change at the end: x, y, yaw (rad)
    largest_jump_m: float
    largest_jump_deg: float
    largest_jump_t: float  # seconds from the first map -> odom sample
    max_offset_m: float  # largest map -> odom change in translation during the run
    max_yaw_deg: float  # largest map -> odom change in rotation during the run
    yaw_sd_deg: float  # spread of the map -> odom rotation over the run


def drift_summary(
    map_odom: Sequence[tuple[float, float, float, float]],
    odom: Sequence[tuple[float, float, float]],
) -> Drift | None:
    """
    Summarize how far SLAM corrected the odometry.

    `map_odom` is (t, x, y, yaw) of the map -> odom transform, `odom` is
    (t, x, y) of /odom/integrated. A jump between consecutive transforms is a
    correction applied at once, as a loop closure does. Offsets, rotations
    and their spread are measured from map -> odom at its last sample before
    the car first moved (first_moved()), or at its first sample if the car
    never moved. Earlier samples count as no change. A localization run
    starts the transform at the offset between the start pose and the
    odometry pose, which the ESP32 keeps through the session's teleop
    restart. That offset is there before the car moves, so it is not drift,
    and neither is the localizer settling while the car stands still. In a
    mapping run the transform stays at the identity until the car moves, so
    its numbers are upstream's.
    """
    if not map_odom:
        return None
    path = sum(math.dist(a[1:], b[1:]) for a, b in zip(odom, odom[1:], strict=False))
    jump_m, jump_deg, jump_t = 0.0, 0.0, 0.0
    for a, b in zip(map_odom, map_odom[1:], strict=False):
        d = math.dist(a[1:3], b[1:3])
        r = abs(math.degrees(math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))))
        if d > jump_m or (d == jump_m and r > jump_deg):
            jump_m, jump_deg, jump_t = d, r, b[0] - map_odom[0][0]
    moved = first_moved(odom)
    base = map_odom[0]
    for m in map_odom:
        if moved is None or m[0] >= moved:
            break
        base = m
    run = []
    for m in map_odom:
        if m[0] < base[0]:
            run.append((0.0, 0.0, 0.0))
            continue
        turn = math.atan2(math.sin(m[3] - base[3]), math.cos(m[3] - base[3]))
        run.append((m[1] - base[1], m[2] - base[2], turn))
    yaws = [math.degrees(r[2]) for r in run]
    return Drift(
        path,
        run[-1],
        jump_m,
        jump_deg,
        jump_t,
        max(math.hypot(r[0], r[1]) for r in run),
        max(abs(y) for y in yaws),
        float(np.std(yaws)),
    )


def drift_lines(drift: Drift) -> list[str]:
    x, y, yaw = drift.final
    end = math.hypot(x, y)
    share = f' ({end / drift.path_m:.2%} of the path)' if drift.path_m else ''
    return [
        f"Odometry path {drift.path_m:.1f} m. Change in map -> odom from the car's first move "
        f'to the end: {end:.3f} m, {math.degrees(yaw):+.1f} deg{share}.',
        f'During the run: largest offset {drift.max_offset_m:.3f} m, largest rotation '
        f'{drift.max_yaw_deg:.1f} deg, rotation spread (SD) {drift.yaw_sd_deg:.2f} deg. '
        f'Largest single correction {drift.largest_jump_m * 1e3:.0f} mm, '
        f'{drift.largest_jump_deg:.1f} deg at {drift.largest_jump_t:.0f} s.',
    ]


def loop_lines(text: str) -> list[str]:
    return [line.strip() for line in text.splitlines() if LOOP_PATTERN.search(line)]


def read_load_csv(path: Path) -> list[slam_eval.Sample]:
    fixed = set(slam_eval.LOAD_COLUMNS)

    def num(v: str | None) -> float | None:
        return None if v in (None, '', 'None') else float(v)

    def whole(v: str | None) -> int | None:
        n = num(v)
        return None if n is None else int(n)

    samples = []
    with path.open() as f:
        for r in csv.DictReader(f):
            samples.append(
                slam_eval.Sample(
                    float(r['t_s']),
                    float(r['cpu_busy_pct']),
                    num(r['temp_c']),
                    whole(r['throttled']),
                    whole(r['arm_mhz']),
                    {k: float(v) for k, v in r.items() if k not in fixed},
                    gpu_pct=num(r.get('gpu_pct')),
                    power_mw=whole(r.get('power_mw')),
                    mem_used_mib=whole(r.get('mem_used_mib')),
                    mem_avail_mib=whole(r.get('mem_avail_mib')),
                    swap_used_mib=whole(r.get('swap_used_mib')),
                )
            )
    return samples


# --- processes -----------------------------------------------------------------


@dataclass
class Child:
    """A process in its own session, so the terminal's Ctrl-C does not reach it."""

    name: str
    argv: list[str]
    log: Path
    env: dict[str, str] = field(default_factory=dict)
    proc: subprocess.Popen[bytes] | None = None

    def start(self) -> None:
        self.log.parent.mkdir(parents=True, exist_ok=True)
        out = self.log.open('ab')
        self.proc = subprocess.Popen(
            self.argv,
            stdout=out,
            stderr=subprocess.STDOUT,
            stdin=subprocess.DEVNULL,
            start_new_session=True,
            env={**os.environ, **self.env},
        )
        out.close()

    def running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def stop(self) -> str:
        """SIGINT, then SIGTERM, then SIGKILL to the process group; return how it ended."""
        if self.proc is None:
            return 'not started'
        if self.proc.poll() is not None:
            return f'had exited ({self.proc.returncode})'
        for sig, wait in ((signal.SIGINT, STOP_WAIT[0]), (signal.SIGTERM, STOP_WAIT[1])):
            try:
                os.killpg(self.proc.pid, sig)
            except ProcessLookupError:
                return 'stopped'
            try:
                self.proc.wait(timeout=wait)
                return f'stopped by {sig.name}'
            except subprocess.TimeoutExpired:
                continue
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.proc.wait()
        return 'killed'


# --- session -------------------------------------------------------------------


class Terminal:
    """Single-key input on a terminal; plain lines otherwise."""

    def __init__(self) -> None:
        self.tty = sys.stdin.isatty()
        self._saved: Any = None

    def __enter__(self) -> Terminal:
        if self.tty:
            import termios
            import tty

            self._saved = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, *exc: object) -> None:
        self.restore()

    def restore(self) -> None:
        if self.tty and self._saved is not None:
            import termios

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._saved)

    def key(self, timeout: float) -> str:
        ready, _, _ = select.select([sys.stdin], [], [], timeout)
        if not ready:
            return ''
        return sys.stdin.read(1) if self.tty else sys.stdin.readline().strip()[:1]

    def line(self, prompt: str) -> str:
        self.restore()
        try:
            return input(prompt).strip()
        finally:
            if self.tty:
                import tty

                tty.setcbreak(sys.stdin.fileno())


def ask(prompt: str) -> str:
    try:
        return input(prompt).strip().lower()
    except EOFError:
        return ''


def disk_line(path: Path | None = None) -> tuple[str, str]:
    u = shutil.disk_usage(path or Path.home())
    return disk_status(u.total, u.used, u.free)


def print_storage_advice() -> None:
    print('Largest folders to clear (check each before deleting):')
    for root in (Path.home() / 'logs', SESSIONS, slam_eval.EVAL_DIR.parent / 'maps'):
        for size, path in largest_entries(root, 3):
            print(f'  {size / 1e9:6.2f} GB  {path}')


class Session:
    def __init__(self, args: argparse.Namespace) -> None:
        self.mode = 'mapping' if args.cmd == 'map' else 'localization'
        self.name = args.name if self.mode == 'mapping' else args.map
        self.localizer = getattr(args, 'localizer', 'amcl')
        self.pose = getattr(args, 'pose', '') or ''
        self.camera = args.camera == 'on'
        self.inference = args.inference == 'on'
        self.tag = 'mapping' if self.mode == 'mapping' else self.localizer
        self.dir = SESSIONS / session_dir_name(
            'map' if self.mode == 'mapping' else f'loc-{self.localizer}', self.name, datetime.now()
        )
        self.children: list[Child] = []
        self.recorder: slam_eval.LoadRecorder | None = None
        self.stop_event = threading.Event()
        self.pose_times: list[float] = []
        self.map_size = ''
        self.status_rows: list[tuple[float, str, str, str, str, str]] = []
        self.marks = 0
        self.saving = ''
        self.messages: list[str] = []
        self.node: Any = None
        self.buffer: Any = None
        self.stalls = 0
        self.disk_warned = False

    # -- setup --

    def preflight(self) -> bool:
        from neoracer_ros2_driver.maps import map_stem, valid_name

        level, text = disk_line()
        print(f'Storage: {text}')
        if level == 'refuse':
            print('Too little space to record a session. Clear space first.')
            print_storage_advice()
            return False
        if level == 'warn':
            print('Storage is getting full. Plan to clear old logs and sessions soon.')
            print_storage_advice()
        if not valid_name(self.name):
            print(f'Map names use letters, digits, - and _: {self.name!r}')
            return False
        stem = map_stem(self.name)
        if self.mode == 'mapping' and stem.parent.exists():
            print(f'A map named {self.name} exists in {stem.parent}. Choose another name.')
            return False
        if self.mode == 'localization' and not Path(str(stem) + '.yaml').exists():
            print(f'No saved map {self.name} in {stem.parent}.')
            return False
        if self.mode == 'localization' and not self.pose:
            self.pose = start_mark_pose(stem)
            if self.pose:
                print(f'Start pose from the map: {self.pose} (the start mark).')
        # Humble ships MCAP as a separate package; without it the bag would not start.
        if not neoracer_log.storage_installed('mcap'):
            print(
                'The MCAP bag plugin is not installed. Run scripts/setup_ros2.sh, or '
                'sudo apt install ros-humble-rosbag2-storage-mcap.'
            )
            return False
        running = subprocess.run(
            ['pgrep', '-f', 'teleop.launch.py'], capture_output=True, text=True, check=False
        ).stdout.split()
        if running and not self.teleop_service_active():
            print('A teleop launch is already running in another terminal. Stop it first.')
            return False
        return True

    @staticmethod
    def teleop_service_active() -> bool:
        r = subprocess.run(['systemctl', 'is-active', TELEOP_UNIT], capture_output=True, text=True)
        return r.stdout.strip() == 'active'

    def stop_teleop_service(self) -> bool:
        if not self.teleop_service_active():
            return True
        print('Stopping the teleop service.')
        r = subprocess.run(['sudo', '-n', 'systemctl', 'stop', TELEOP_UNIT], check=False)
        return r.returncode == 0

    def write_meta(self, **extra: Any) -> None:
        path = self.dir / 'session.yaml'
        meta = yaml.safe_load(path.read_text()) if path.exists() else {}
        meta.update(extra)
        path.write_text(yaml.safe_dump(meta, sort_keys=False))

    def git_state(self) -> dict[str, Any]:
        def git(*a: str) -> str:
            return subprocess.run(
                ['git', '-C', str(REPO), *a], capture_output=True, text=True
            ).stdout.strip()

        return {
            'commit': git('rev-parse', '--short', 'HEAD'),
            'dirty': bool(git('status', '--porcelain')),
        }

    def save_calibration(self) -> None:
        out = self.dir / 'calibration'
        out.mkdir(parents=True, exist_ok=True)
        for f in sorted(car_profile.params_dir().glob('*.yaml')):
            shutil.copy2(f, out / f.name)

    def start_ros(self) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from tf2_ros import Buffer, TransformListener
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from diagnostic_msgs.msg import DiagnosticStatus
        from nav_msgs.msg import OccupancyGrid
        from sensor_msgs.msg import LaserScan

        rclpy.init()
        self.node = rclpy.create_node('slam_session')
        self.buffer = Buffer()
        TransformListener(self.buffer, self.node)
        self.scan_seen = False

        def on_scan(_: Any) -> None:
            self.scan_seen = True

        def on_pose(_: Any) -> None:
            self.pose_times.append(time.monotonic())

        def on_map(m: Any) -> None:
            res = m.info.resolution
            self.map_size = f'{m.info.width * res:.1f} x {m.info.height * res:.1f} m'

        sensor = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.node.create_subscription(LaserScan, '/scan', on_scan, sensor)
        self.node.create_subscription(PoseWithCovarianceStamped, '/pose', on_pose, 50)
        self.node.create_subscription(OccupancyGrid, '/map', on_map, latched)

        def on_status(m: Any) -> None:
            v = {kv.key: kv.value for kv in m.values}
            self.status_rows.append(
                (
                    time.monotonic(),
                    v.get('mode', ''),
                    v.get('localized', ''),
                    v.get('pose_sd_m', ''),
                    v.get('last_autosave', ''),
                    v.get('pose_sd_deg', ''),
                )
            )

        self.node.create_subscription(DiagnosticStatus, '/autonomy/status', on_status, latched)
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node)
        threading.Thread(target=self.executor.spin, daemon=True).start()

    def wait_for(self, what: str, ready: Any, timeout: float) -> bool:
        end = time.monotonic() + timeout
        print(f'Waiting for {what}...', end='', flush=True)
        while time.monotonic() < end:
            if ready():
                print(' ready.')
                return True
            if any(c.proc is not None and not c.running() for c in self.children):
                break
            time.sleep(0.5)
        print(' not ready.')
        return False

    def can_transform(self, target: str, source: str) -> bool:
        from rclpy.time import Time

        return bool(self.buffer.can_transform(target, source, Time()))

    def start_children(self, *names: str) -> None:
        logs = self.dir / 'logs'
        table = {
            'teleop': Child(
                'teleop',
                teleop_argv(self.camera, self.inference),
                logs / 'teleop.log',
            ),
            'autonomy': Child(
                'autonomy',
                autonomy_argv(self.mode, self.name, self.localizer, self.pose, self.dir.name),
                logs / 'autonomy.log',
                {'ROS_LOG_DIR': str(logs / 'autonomy')},
            ),
            'foxglove': Child('foxglove', foxglove_argv(), logs / 'foxglove.log'),
            'bag': Child('bag', bag_argv(self.dir / 'bag'), logs / 'bag.log'),
        }
        for n in names:
            table[n].start()
            self.children.append(table[n])

    def record_load(self) -> None:
        self.recorder = slam_eval.LoadRecorder()
        while not self.stop_event.wait(1.0):
            self.recorder.sample()

    # -- actions --

    def call(self, srv_type: Any, name: str, request: Any) -> Any:
        client = self.node.create_client(srv_type, name)
        try:
            if not client.wait_for_service(timeout_sec=5.0):
                return None
            future = client.call_async(request)
            end = time.monotonic() + SERVICE_TIMEOUT
            while not future.done() and time.monotonic() < end:
                time.sleep(0.05)
            return future.result() if future.done() else None
        finally:
            self.node.destroy_client(client)

    def serialize(self, stem: Path) -> bool:
        from slam_toolbox.srv import SerializePoseGraph

        result = self.call(
            SerializePoseGraph,
            '/slam_toolbox/serialize_map',
            SerializePoseGraph.Request(filename=str(stem)),
        )
        return result is not None and result.result == SerializePoseGraph.Response.RESULT_SUCCESS

    def save_map(self) -> bool:
        """Save through /autonomy/save_map, the interface the CLI and library use."""
        from std_srvs.srv import Trigger

        self.saving = 'saving'
        t0 = time.monotonic()
        result = self.call(Trigger, '/autonomy/save_map', Trigger.Request())
        self.saving = ''
        ok = bool(result and result.success)
        detail = result.message if result else 'no reply from autonomy_node'
        self.say(
            f'Map save {"done" if ok else "failed"} ({time.monotonic() - t0:.1f} s): {detail}'
        )
        return ok

    def timed_save(self) -> None:
        self.stalls += 1
        out = self.dir / 'stall'
        out.mkdir(exist_ok=True)
        self.saving = 'timed save'
        t0 = time.monotonic()
        ok = self.serialize(out / f'stall_{self.stalls}')
        t1 = time.monotonic()
        time.sleep(1.5)
        times = list(self.pose_times)
        moving = sum(1 for t in times if t0 - 1.0 <= t <= t1 + 1.0) >= 2
        gap = max_gap(times, t0, t1) if moving else math.nan
        size = sum(p.stat().st_size for p in out.glob(f'stall_{self.stalls}.*'))
        self.saving = ''
        path = self.dir / 'stalls.csv'
        new = not path.exists()
        with path.open('a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['n', 'elapsed_s', 'ok', 'call_s', 'max_pose_gap_s', 'size_mb'])
            w.writerow(
                [self.stalls, f'{t0 - self.started:.0f}', ok, f'{t1 - t0:.2f}',
                 '' if math.isnan(gap) else f'{gap:.2f}', f'{size / 1e6:.1f}']
            )  # fmt: skip
        gap_text = (
            'no /pose during the save (drive while pressing t)'
            if math.isnan(gap)
            else f'longest gap in /pose {gap:.2f} s'
        )
        self.say(
            f'Timed save {self.stalls}: call {t1 - t0:.2f} s, {gap_text}, {size / 1e6:.1f} MB.'
        )

    def mark(self, label: str) -> None:
        poses = slam_eval.sample_map_pose(self.buffer)
        if not poses:
            self.say('No map -> base_link transform; mark not recorded.')
            return
        self.marks += 1
        self.say(slam_eval.append_mark(self.dir / 'marks.csv', label, self.tag, poses))

    def event(self, text: str) -> None:
        """Append a timed note to events.csv (same clock as status.csv)."""
        path = self.dir / 'events.csv'
        new = not path.exists()
        with path.open('a', newline='') as f:
            w = csv.writer(f)
            if new:
                w.writerow(['t_s', 'event'])
            w.writerow([f'{time.monotonic() - self.started:.1f}', text])
        self.say(f'Event at {time.monotonic() - self.started:.0f} s: {text}')

    def global_search(self) -> None:
        from std_srvs.srv import Trigger

        self.event('global search requested')
        result = self.call(Trigger, '/autonomy/global_localization', Trigger.Request())
        self.say(f'Global search: {result.message if result else "no reply from autonomy_node"}')

    def say(self, text: str) -> None:
        self.messages.append(text)

    def in_background(self, fn: Any, *a: Any) -> None:
        if self.saving:
            self.say(f'Busy ({self.saving}); try again when it finishes.')
            return
        threading.Thread(target=fn, args=a, daemon=True).start()

    # -- main loop --

    def status(self) -> str:
        elapsed = time.monotonic() - self.started
        samples = self.recorder.samples if self.recorder else []
        s = samples[-1] if samples else None
        parts = [f'{int(elapsed // 60):02d}:{int(elapsed % 60):02d}']
        if s:
            parts.append(f'cpu {s.cpu_busy:3.0f}%')
            if s.temp_c is not None:
                parts.append(f'{s.temp_c:.1f}C')
            # A run whose swap grows, or that throttles, is discarded, so both show here.
            alarms = ['THROTTLED'] if s.throttled else []
            start = samples[0].swap_used_mib
            if s.swap_used_mib and start is not None and s.swap_used_mib > start:
                alarms.append(f'SWAP +{s.swap_used_mib - start} MiB')
            parts.append(' '.join(alarms) or 'ok')
        if self.map_size:
            parts.append(f'map {self.map_size}')
        if self.mode == 'localization' and self.status_rows:
            _, _, localized, sd_m, _, sd_deg = self.status_rows[-1]
            parts.append(f'localized {"yes" if localized == "true" else "NO"}')
            if sd_m:
                parts.append(f'spread {sd_m} m {sd_deg} deg')
        parts.append(f'marks {self.marks}')
        if self.saving:
            parts.append(self.saving + '...')
        dead = [c.name for c in self.children if not c.running()]
        if dead:
            parts.append('STOPPED: ' + ','.join(dead))
        keys = (
            'm mark, e event, s save, t timed save, q quit'
            if self.mode == 'mapping'
            else 'm mark, e event, g global search, q quit'
        )
        return '  '.join(parts) + f'  [{keys}]'

    def loop(self, term: Terminal) -> None:
        last_disk = time.monotonic()
        while True:
            for text in self.messages:
                print('\r\033[K' + text)
            self.messages.clear()
            print('\r\033[K' + self.status(), end='', flush=True)
            k = term.key(1.0).lower()
            if time.monotonic() - last_disk > DISK_CHECK_SEC:
                last_disk = time.monotonic()
                level, text = disk_line()
                if level != 'ok' and not self.disk_warned:
                    self.disk_warned = True
                    self.say(f'Storage warning: {text}. Clear space after this session.')
            if k == 'q':
                print()
                return
            if k == 'm':
                print()
                label = term.line('Mark label (letters and digits, Enter cancels): ')
                if re.fullmatch(r'[A-Za-z0-9_-]+', label):
                    self.in_background(self.mark, label)
            elif k == 'e':
                print()
                note = term.line('Event (Enter cancels): ').strip()
                if note:
                    self.event(note.replace(',', ';'))
            elif k == 'g' and self.mode == 'localization':
                self.in_background(self.global_search)
            elif k == 's' and self.mode == 'mapping':
                self.in_background(self.save_map)
            elif k == 't' and self.mode == 'mapping':
                self.in_background(self.timed_save)

    def run(self) -> int:
        if not self.preflight():
            return 1
        self.dir.mkdir(parents=True)
        self.write_meta(
            mode=self.mode,
            map=self.name,
            localizer=self.localizer if self.mode == 'localization' else None,
            pose=self.pose or None,
            camera=self.camera,
            inference=self.inference,
            power_mode=sysinfo.read_power_mode()[0],
            started=f'{datetime.now():%Y-%m-%d %H:%M:%S}',
            git=self.git_state(),
            disk_start=disk_line()[1],
        )
        self.save_calibration()
        print(f'Session folder: {self.dir}')
        print('Place the car on the start mark (cross O), pointing along the same arm as before.')
        if ask('Press Enter when the car is in place, or q to cancel: ') == 'q':
            return 1
        if not self.stop_teleop_service():
            print('Could not stop the teleop service.')
            return 1
        self.started = time.monotonic()
        code = 0
        try:
            self.start_children('teleop')
            self.start_ros()
            if not self.wait_for(
                'teleop (lidar and odometry)',
                lambda: self.scan_seen and self.can_transform('odom', 'laser'),
                READY_TIMEOUT,
            ):
                print('Teleop did not come up; see logs/teleop.log.')
                code = 1
                return code
            latest = Path.home() / 'logs' / 'latest'
            self.write_meta(teleop_log_dir=str(latest.resolve()) if latest.exists() else None)
            self.start_children('autonomy', 'foxglove', 'bag')
            threading.Thread(target=self.record_load, daemon=True).start()
            if not self.wait_for(
                f'{self.tag} (map -> odom)',
                lambda: self.can_transform('map', 'odom'),
                LOCALIZER_TIMEOUT,
            ):
                print('The SLAM launch did not come up; see logs/autonomy.log.')
                code = 1
                return code
            print(f'Lichtblick: connect to ws://{hostname_ip()}:{FOXGLOVE_PORT}')
            if self.mode == 'localization':
                print('Drive at least 0.5 m before the first mark so the localizer updates.')
            # With the mode switch UP the ESP32 drives from the sticks, so the
            # Jetson's mapping caps do not apply. The firmware caps the sticks at
            # 15 percent only with the speed switch UP, so every session keeps it UP.
            print('Drive with the handset, mode switch UP (manual).')
            print('Keep the speed switch UP (slow).')
            print('Keys are read without Enter.')
            with Terminal() as term:
                try:
                    self.loop(term)
                except KeyboardInterrupt:
                    print()
            if self.mode == 'mapping':
                print('Saving the map...')
                while self.saving:
                    time.sleep(0.2)
                self.save_map()
                for text in self.messages:
                    print(text)
                self.messages.clear()
        finally:
            self.shutdown()
        analyze(self.dir)
        print(f'Report: {self.dir / "report.md"}')
        level, text = disk_line()
        print(f'Storage: {text}')
        if level != 'ok':
            print('Storage is getting full. Clear space before the next session.')
            print_storage_advice()
        if ask('Restart the teleop service now? [y/N] ') == 'y':
            subprocess.run(['sudo', '-n', 'systemctl', 'start', TELEOP_UNIT], check=False)
        return code

    def shutdown(self) -> None:
        print('Stopping...')
        self.stop_event.set()
        if self.recorder:
            self.recorder.write_csv(self.dir / 'load.csv')
        if self.status_rows:
            with (self.dir / 'status.csv').open('w', newline='') as f:
                w = csv.writer(f)
                w.writerow(
                    ['t_s', 'mode', 'localized', 'pose_sd_m', 'last_autosave', 'pose_sd_deg']
                )
                for t, *rest in self.status_rows:
                    w.writerow([f'{t - self.started:.1f}', *rest])
        ended: dict[str, str] = {}
        for n in ('bag', 'autonomy', 'foxglove'):
            for c in self.children:
                if c.name == n:
                    print(f'  {n}...', end='', flush=True)
                    ended[n] = c.stop()
                    print(f' {ended[n]}')
        if self.node is not None:
            import rclpy

            self.executor.shutdown(timeout_sec=2.0)
            self.node.destroy_node()
            rclpy.try_shutdown()
        for c in self.children:
            if c.name == 'teleop':
                print('  teleop...', end='', flush=True)
                ended['teleop'] = c.stop()
                print(f' {ended["teleop"]}')
        self.write_meta(
            ended=f'{datetime.now():%Y-%m-%d %H:%M:%S}',
            duration_s=round(time.monotonic() - self.started) if hasattr(self, 'started') else 0,
            stops=ended,
            disk_end=disk_line()[1],
        )


def start_mark_pose(stem: Path) -> str:
    """
    Return the map's start pose from its meta.yaml as `x,y,yaw`, or ''.

    The session asks for the car on the start mark, which is where the map was
    begun. The ESP32 keeps its odometry pose through the session's teleop
    restart, so the map origin is the odometry origin of the mapping run and
    not the start mark. Without a start pose the localizer starts at the map
    origin.
    """
    from neoracer_ros2_driver import autonomy as au

    pose = au.pose_of(au.read_record(stem.parent / au.META_FILE).get('start_pose'))
    return ','.join(str(v) for v in pose) if pose else ''


def hostname_ip() -> str:
    out = subprocess.run(['hostname', '-I'], capture_output=True, text=True).stdout.split()
    return out[0] if out else 'neoracer.local'


# --- analysis ------------------------------------------------------------------


def read_bag(
    bag: Path,
) -> tuple[list[tuple[float, float, float, float]], list[tuple[float, float, float]]]:
    """Return map -> odom (t, x, y, yaw) and /odom/integrated (t, x, y) from a bag."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/tf', '/odom/integrated']))
    map_odom, odom = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        ts = t / 1e9
        if topic == '/tf':
            for tr in deserialize_message(data, TFMessage).transforms:
                if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                    p = tr.transform.translation
                    map_odom.append((ts, p.x, p.y, slam_eval.yaw_of(tr.transform.rotation)))
        else:
            p = deserialize_message(data, Odometry).pose.pose.position
            odom.append((ts, p.x, p.y))
    return map_odom, odom


def render_map(stem: Path, out: Path) -> None:
    from PIL import Image

    meta = yaml.safe_load(Path(str(stem) + '.yaml').read_text())
    image = Image.open(stem.parent / meta['image'])
    image.resize((image.width * 4, image.height * 4), Image.NEAREST).save(out)


def analyze(session: Path) -> Path:
    """Write report.md (and map.png for a mapping run) into a session folder."""
    from neoracer_ros2_driver.maps import map_stem

    meta = yaml.safe_load((session / 'session.yaml').read_text()) or {}
    lines = [f'# Session {session.name}', '']
    lines.append(
        f"Mode {meta.get('mode')}, map {meta.get('map')}, localizer {meta.get('localizer')}, "
        f"camera {'on' if meta.get('camera') else 'off'}, {meta.get('duration_s', '?')} s, "
        f"commit {meta.get('git', {}).get('commit')}"
        f"{' (dirty)' if meta.get('git', {}).get('dirty') else ''}."
    )
    lines.append(f"Stops: {meta.get('stops')}")

    power_mode = meta.get('power_mode') or 'not read'
    lines += ['', '## Load', '', f'Jetson power mode {power_mode}.', '```']
    load = session / 'load.csv'
    lines += slam_eval.summarize(read_load_csv(load)) if load.exists() else ['No load record.']
    lines.append('```')

    stem = map_stem(str(meta.get('map', '')))
    lines += ['', '## Map', '']
    if Path(str(stem) + '.yaml').exists():
        from PIL import Image

        m = yaml.safe_load(Path(str(stem) + '.yaml').read_text())
        image = Image.open(stem.parent / m['image'])
        occupied, free, unknown = slam_eval.cell_counts(image.getdata())
        res = float(m['resolution'])
        area = free * res * res
        lines.append(
            f'{stem}: {image.width * res:.2f} x {image.height * res:.2f} m, occupied {occupied}, '
            f'free {free} ({area:.1f} m^2), unknown {unknown}; '
            f'occupied per free m^2 {occupied / max(area, 1e-9):.1f}.'
        )
        if meta.get('mode') == 'mapping':
            render_map(stem, session / 'map.png')
            lines.append('Rendered: map.png (4 px per cell).')
    else:
        lines.append(f'No saved map at {stem}.')

    lines += ['', '## Odometry against SLAM', '']
    bag = session / 'bag'
    drift = None
    if bag.exists():
        try:
            drift = drift_summary(*read_bag(bag))
        except Exception as e:  # noqa: BLE001  a damaged bag still gets a report
            lines.append(f'Bag not readable: {e}')
    if drift:
        lines += drift_lines(drift)
    elif bag.exists():
        lines.append('No map -> odom transforms in the bag.')

    if drift and Path(str(stem) + '.yaml').exists():
        lines += ['', '## Scan-to-map fit', '', fit_line(stem, bag)]

    lines += ['', '## Loop closures', '']
    # The launch's own output holds every node's screen output, including
    # lines the karto core prints outside the ROS logger.
    log = session / 'logs' / 'autonomy.log'
    found = loop_lines(log.read_text(errors='replace')) if log.exists() else []
    lines.append(
        f'{len(found)} log lines mention a loop.' if found else 'No loop lines in the log.'
    )
    lines += [f'    {line[:160]}' for line in found[:10]]

    status = session / 'status.csv'
    if status.exists():
        lines += ['', '## Autonomy status', ''] + status_lines(status)

    events = session / 'events.csv'
    if events.exists():
        lines += ['', '## Events', '', '```', events.read_text().rstrip(), '```']

    stalls = session / 'stalls.csv'
    if stalls.exists():
        lines += ['', '## Timed saves', '', '```', stalls.read_text().rstrip(), '```']

    marks = session / 'marks.csv'
    if marks.exists():
        taped = slam_eval.EVAL_DIR / 'taped.txt'
        rows = slam_eval.read_taped(taped) if taped.exists() else []
        point = slam_eval.read_point(taped) if taped.exists() else (0.0, 0.0)
        lines += ['', '## Marks', '', '```']
        lines += slam_eval.report_lines(slam_eval.read_marks(marks), rows, point)
        lines.append('```')

    lines += ['', '## Storage', '', f"Start {meta.get('disk_start')}; end {meta.get('disk_end')}."]
    out = session / 'report.md'
    out.write_text('\n'.join(lines) + '\n')
    return out


def map_lines(stem: Path) -> list[str]:
    """Size and cell counts of a saved map; fewer occupied cells per free m^2 is thinner walls."""
    from PIL import Image

    m = yaml.safe_load(Path(str(stem) + '.yaml').read_text())
    image = Image.open(stem.parent / m['image'])
    occupied, free, unknown = slam_eval.cell_counts(image.getdata())
    res = float(m['resolution'])
    area = free * res * res
    return [
        f'{image.width * res:.2f} x {image.height * res:.2f} m, occupied {occupied}, '
        f'free {free} ({area:.1f} m^2), unknown {unknown}; '
        f'occupied per free m^2 {occupied / max(area, 1e-9):.1f}.'
    ]


def parse_overrides(items: Sequence[str]) -> dict[str, Any]:
    """Parse key=value pairs; values are YAML (numbers, booleans, lists)."""
    out = {}
    for item in items:
        key, sep, value = item.partition('=')
        if not sep or not key.strip():
            raise ValueError(f'expected key=value: {item!r}')
        out[key.strip()] = yaml.safe_load(value)
    return out


REPLAY_KINDS = ('mapping', 'amcl', 'slam_toolbox')


def play_argv(bag: Path) -> list[str]:
    return ['ros2', 'bag', 'play', str(bag), '--clock', '100', '--topics', *REPLAY_TOPICS,
            '--remap', '/tf:=/tf_bag']  # fmt: skip


def _ros_node(package: str, executable: str, name: str, files: Sequence[Path]) -> list[str]:
    argv = ['ros2', 'run', package, executable, '--ros-args', '-r', f'__node:={name}']
    for f in files:
        argv += ['--params-file', str(f)]
    return argv


def replay_specs(kind: str, overrides: Path) -> list[tuple[str, list[str]]]:
    """Return (name, command line) for each node of a replay kind."""
    config = REPO / 'config'
    if kind == 'amcl':
        files = [config / 'amcl.yaml', overrides]
        return [
            ('map_server', _ros_node('nav2_map_server', 'map_server', 'map_server', files)),
            ('amcl', _ros_node('nav2_amcl', 'amcl', 'amcl', files)),
            (
                'lifecycle_manager',
                _ros_node(
                    'nav2_lifecycle_manager',
                    'lifecycle_manager',
                    'lifecycle_manager_localization',
                    files,
                ),
            ),
        ]
    yaml_name = 'slam_mapping.yaml' if kind == 'mapping' else 'slam_localization.yaml'
    exe = 'async_slam_toolbox_node' if kind == 'mapping' else 'localization_slam_toolbox_node'
    files = [config / yaml_name, overrides]
    # Humble's slam_toolbox is a plain node, as in the autonomy launch, so no
    # lifecycle manager starts it.
    return [('slam_toolbox', _ros_node('slam_toolbox', exe, 'slam_toolbox', files))]


def replay_overrides(
    kind: str, overrides: dict[str, Any], stem: Path | None, pose: tuple[float, float, float]
) -> dict[str, Any]:
    """Parameter file for a replay: sim time everywhere, the map and start pose, the overrides."""
    sim = {'use_sim_time': True}
    if kind == 'amcl':
        x, y, yaw = pose
        return {
            'amcl': {
                'ros__parameters': {
                    'set_initial_pose': True,
                    'initial_pose': {'x': x, 'y': y, 'z': 0.0, 'yaw': yaw},
                    **overrides,
                    **sim,
                }
            },
            'map_server': {'ros__parameters': {'yaml_filename': f'{stem}.yaml', **sim}},
            'lifecycle_manager_localization': {'ros__parameters': sim},
        }
    slam = {**overrides, **sim}
    if kind == 'slam_toolbox':
        slam = {'map_file_name': str(stem), 'map_start_pose': list(pose), **slam}
    return {'slam_toolbox': {'ros__parameters': slam}}


def fit_line(stem: Path, bag: Path, map_odom: Any = None) -> str:
    """Scan-to-map fit of a bag's scans; map -> odom from the bag unless given."""
    data = scan_fit.read_bag(bag)
    track = data['map_odom'] if map_odom is None else scan_fit.as_track(map_odom)
    if data['base_laser'] is None:
        return 'No base_link -> laser transform in the bag.'
    fit = scan_fit.fit_scans(
        scan_fit.load_grid(stem), data['scans'], track, data['odom_base'], data['base_laser']
    )
    return fit.line() if fit else 'No scans to score.'


def autonomy_replay(
    kind: str, map_name: str, localizer: str, pose_text: str, overrides: Path
) -> tuple[str, list[str], dict[str, Any]]:
    """Return the autonomy_node child and its parameter section for a replay."""
    section = {
        'mode': 'mapping' if kind == 'mapping' else 'localization',
        'map': map_name,
        'localizer': localizer,
        'pose': pose_text,
        'use_sim_time': True,
    }
    argv = _ros_node(
        'neoracer_ros2_driver',
        'autonomy_node',
        'autonomy_node',
        [REPO / 'config' / 'autonomy.yaml', overrides],
    )
    return 'autonomy_node', argv, section


def write_rows(path: Path, header: list[str], rows: Sequence[Sequence[Any]]) -> None:
    with path.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def localized_fit_line(
    stem: Path, bag: Path, map_odom: Any, status: Sequence[tuple[float, bool]]
) -> str:
    """How long autonomy_node's localized disagreed with the scan fit at the replayed pose."""
    data = scan_fit.read_bag(bag, every=1)
    shares = scan_fit.scan_shares(
        scan_fit.load_grid(stem),
        data['scans'],
        scan_fit.as_track(map_odom),
        data['odom_base'],
        data['base_laser'],
    )
    wrong, missed, span = scan_fit.localized_against_fit(status, shares)
    return (
        f'Localized while the 2 s scan fit was under 50%: {wrong:.1f} s; not localized '
        f'while it was 65% or more: {missed:.1f} s (of {span:.0f} s judged).'
    )


def first_moved(
    odom: Sequence[tuple[float, float, float]], distance: float = 0.05
) -> float | None:
    """Time the odometry first moves `distance` from where it started."""
    if not odom:
        return None
    x0, y0 = odom[0][1], odom[0][2]
    return next((t for t, x, y in odom if math.hypot(x - x0, y - y0) >= distance), None)


def replay(
    session: Path,
    tag: str,
    overrides: dict[str, Any],
    localizer: str | None,
    with_autonomy: bool = False,
) -> int:
    """Run a session's bag through a fresh localizer or mapper with overrides; write a report."""
    from neoracer_ros2_driver.maps import map_stem, parse_pose

    bag = session / 'bag'
    if not bag.exists():
        print(f'No bag in {session}.')
        return 1
    if not re.fullmatch(r'[A-Za-z0-9_-]+', tag):
        print(f'Tags use letters, digits, - and _: {tag!r}')
        return 2
    meta = yaml.safe_load((session / 'session.yaml').read_text()) or {}
    if meta.get('mode') == 'mapping':
        kind, stem = 'mapping', None
    else:
        kind = localizer or meta.get('localizer') or 'amcl'
        stem = map_stem(str(meta.get('map')))
    if kind not in REPLAY_KINDS:
        print(f'Localizer must be amcl or slam_toolbox: {kind!r}')
        return 2
    out = session / 'replay' / tag
    if out.exists():
        print(f'{out} exists. Choose another tag.')
        return 1
    out.mkdir(parents=True)
    pose = parse_pose(meta.get('pose') or '') or (0.0, 0.0, 0.0)
    params = out / 'overrides.yaml'
    sections = replay_overrides(kind, overrides, stem, pose)
    autonomy = None
    if with_autonomy:
        autonomy = autonomy_replay(
            kind, str(meta.get('map') or ''), kind, str(meta.get('pose') or ''), params
        )
        sections['autonomy_node'] = {'ros__parameters': autonomy[2]}
    params.write_text(yaml.safe_dump(sections, sort_keys=False))
    # Every process of the replay, this one included, on its own domain.
    os.environ['ROS_DOMAIN_ID'] = str(REPLAY_DOMAIN)
    logs = out / 'logs'
    env = {'ROS_DOMAIN_ID': str(REPLAY_DOMAIN), 'ROS_LOG_DIR': str(logs)}
    children = [Child(n, argv, logs / f'{n}.log', env) for n, argv in replay_specs(kind, params)]
    if autonomy is not None:
        # Its map writes (last_pose.yaml, autosaves) stay inside the replay folder.
        children.append(
            Child(
                autonomy[0],
                autonomy[1],
                logs / 'autonomy_node.log',
                {**env, 'RACECAR_MAP_ROOT': str(out / 'maps')},
            )
        )
    play = Child('bag', play_argv(bag), logs / 'bag.log', env)
    rec, code = run_replay(children, play, out, save_map=kind == 'mapping')
    write_rows(out / 'map_odom.csv', ['t', 'x', 'y', 'yaw'], sorted(rec['map_odom']))
    if rec['status']:
        write_rows(out / 'status.csv', ['t', 'localized', 'scan_fit'], rec['status'])

    lines = [f'# Replay {tag} of {session.name}', '', f'Kind: {kind}. Overrides:', '', '```']
    lines += [f'{k}: {v}' for k, v in overrides.items()] or ['(none)']
    lines += ['```', '', '## Odometry against SLAM', '']
    drift = drift_summary(sorted(rec['map_odom']), sorted(rec['odom']))
    lines += drift_lines(drift) if drift else ['No map -> odom transforms.']
    if rec['amcl_sd']:
        sd = np.array(rec['amcl_sd'])
        lines.append(
            f'AMCL pose SD: median {np.median(sd[:, 0]):.2f} m, {np.median(sd[:, 1]):.1f} deg; '
            f'p90 {np.percentile(sd[:, 0], 90):.2f} m, {np.percentile(sd[:, 1], 90):.1f} deg.'
        )
    if with_autonomy:
        moved = first_moved(sorted(rec['odom']))
        # A status received before the replay clock starts reads time 0.
        status = sorted((t, ok) for t, ok, _ in rec['status'] if t > 0)
        t0 = status[0][0] if status else None
        first = next((t for t, ok in status if ok), None)
        lines += ['', '## autonomy_node', '']
        if t0 is None:
            lines.append('No /autonomy/status received.')
        else:
            loc = 'never' if first is None else f'{first - t0:.1f} s'
            mov = 'never' if moved is None else f'{moved - t0:.1f} s'
            lines.append(
                f'From its first status: localized at {loc}; the car first moved at {mov}.'
            )
            if stem is not None and Path(f'{stem}.yaml').exists() and rec['map_odom']:
                lines.append(localized_fit_line(stem, bag, rec['map_odom'], status))
    fit_stem = out / 'map' if kind == 'mapping' else stem
    lines += ['', '## Scan-to-map fit', '']
    if fit_stem is not None and Path(f'{fit_stem}.yaml').exists() and rec['map_odom']:
        lines.append(fit_line(fit_stem, bag, rec['map_odom']))
    else:
        lines.append('No map or no map -> odom to score against.')
    if kind == 'mapping' and (out / 'map.yaml').exists():
        lines += ['', '## Map', ''] + map_lines(out / 'map')
        render_map(out / 'map', out / 'map.png')
    (out / 'report.md').write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))
    return code


def run_replay(
    children: list[Child], play: Child, out: Path, save_map: bool
) -> tuple[dict[str, list[Any]], int]:
    """
    Start the nodes, play the bag, and collect map -> odom, odometry, and AMCL spread.

    Times are the replay clock, which is the bag's receive time, the clock the
    scan fit reads the bag with.
    """
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from diagnostic_msgs.msg import DiagnosticStatus
    from neoracer_ros2_driver.autonomy import covariance_sd
    from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from nav_msgs.msg import Odometry
    from tf2_msgs.msg import TFMessage

    rec: dict[str, list[Any]] = {'map_odom': [], 'odom': [], 'amcl_sd': [], 'status': []}
    rclpy.init()
    node = rclpy.create_node(
        'slam_replay', parameter_overrides=[Parameter('use_sim_time', value=True)]
    )
    tf_pub = node.create_publisher(TFMessage, '/tf', 100)

    def now() -> float:
        return node.get_clock().now().nanoseconds * 1e-9

    def on_bag_tf(msg: Any) -> None:
        keep = [
            t for t in msg.transforms if (t.header.frame_id, t.child_frame_id) != ('map', 'odom')
        ]
        if keep:
            tf_pub.publish(TFMessage(transforms=keep))

    def on_tf(msg: Any) -> None:
        for t in msg.transforms:
            if (t.header.frame_id, t.child_frame_id) == ('map', 'odom'):
                tr = t.transform.translation
                rec['map_odom'].append((now(), tr.x, tr.y, slam_eval.yaw_of(t.transform.rotation)))

    def on_odom(msg: Any) -> None:
        p = msg.pose.pose.position
        rec['odom'].append((now(), p.x, p.y))

    def on_amcl(msg: Any) -> None:
        # AMCL can publish slightly negative variances; covariance_sd clamps them.
        rec['amcl_sd'].append(covariance_sd(list(msg.pose.covariance)))

    reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    node.create_subscription(TFMessage, '/tf_bag', on_bag_tf, reliable)
    node.create_subscription(TFMessage, '/tf', on_tf, reliable)
    node.create_subscription(Odometry, '/odom/integrated', on_odom, qos_profile_sensor_data)
    node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', on_amcl, 10)

    def on_status(msg: Any) -> None:
        values = {kv.key: kv.value for kv in msg.values}
        if values.get('mode') != 'off':
            rec['status'].append(
                (now(), values.get('localized') == 'true', values.get('scan_fit', ''))
            )

    kept = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    node.create_subscription(DiagnosticStatus, '/autonomy/status', on_status, kept)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    code = 0
    try:
        for c in children:
            c.start()
        time.sleep(REPLAY_SETTLE_SEC)
        if not all(c.running() for c in children):
            print('A replay node did not start; see logs/.')
            return rec, 1
        print(f'Replaying ({out.name})...')
        play.start()
        assert play.proc is not None
        play.proc.wait()
        time.sleep(3.0)
        if save_map and not slam_eval.run_map_saver(out / 'map', out / 'logs' / 'map_saver.log'):
            print('map_saver_cli failed; see logs/map_saver.log.')
            code = 1
    finally:
        for c in children:
            c.stop()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
    return rec, code


def status_lines(path: Path) -> list[str]:
    """Share of the run localized, first time localized, and the autosaves seen."""
    with path.open() as f:
        rows = [r for r in csv.DictReader(f) if r['mode'] != 'off']
    if not rows:
        return ['No status messages.']
    t = [float(r['t_s']) for r in rows]
    loc = [r['localized'] == 'true' for r in rows]
    span = t[-1] - t[0]
    on = sum(b - a for a, b, ok in zip(t, t[1:], loc, strict=False) if ok)
    first = next((ti for ti, ok in zip(t, loc, strict=False) if ok), None)
    drops = sum(1 for a, b in zip(loc, loc[1:], strict=False) if a and not b)
    autosaves = sorted({r['last_autosave'] for r in rows if r['last_autosave']})
    return [
        f'Localized {on / span:.0%} of {span:.0f} s; first at '
        + (f'{first:.0f} s' if first is not None else 'never')
        + f'; dropped {drops} time(s). Autosaves seen: {len(autosaves)}'
        + (f' ({autosaves[0]} to {autosaves[-1]}).' if autosaves else '.')
    ]


def newest_session() -> Path | None:
    dirs = sorted(p for p in SESSIONS.glob('*') if (p / 'session.yaml').exists())
    return dirs[-1] if dirs else None


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    sub = parser.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('map', help='mapping run, saved under the map root as NAME')
    p.add_argument('name')
    p.add_argument('--camera', choices=('on', 'off'), default='on')
    p.add_argument('--inference', choices=('on', 'off'), default='off')
    p = sub.add_parser('localize', help='localization run on a saved map')
    p.add_argument('map')
    p.add_argument('--localizer', choices=('amcl', 'slam_toolbox'), default='amcl')
    p.add_argument('--pose', default='', help='start pose x,y,yaw in the map frame')
    p.add_argument('--camera', choices=('on', 'off'), default='on')
    p.add_argument('--inference', choices=('on', 'off'), default='off')
    p = sub.add_parser('analyze', help='rewrite report.md for a session')
    p.add_argument('session', nargs='?', type=Path)
    p = sub.add_parser('replay', help='replay a mapping bag with slam_toolbox overrides')
    p.add_argument('session', type=Path)
    p.add_argument('tag')
    p.add_argument('--set', action='append', default=[], metavar='KEY=VALUE')
    p.add_argument(
        '--localizer',
        choices=('amcl', 'slam_toolbox'),
        help='localization bags: default the localizer of the run',
    )
    p.add_argument(
        '--autonomy',
        action='store_true',
        help='also run autonomy_node (its requests change the localizer; off for tuning)',
    )
    args = parser.parse_args(argv)

    if args.cmd == 'replay':
        return replay(
            args.session, args.tag, parse_overrides(args.set), args.localizer, args.autonomy
        )

    if args.cmd == 'analyze':
        session = args.session or newest_session()
        if session is None:
            print('No sessions yet.')
            return 1
        print(analyze(session))
        return 0
    return Session(args).run()


if __name__ == '__main__':
    sys.exit(main())
