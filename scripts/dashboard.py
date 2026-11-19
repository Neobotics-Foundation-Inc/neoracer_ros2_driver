#!/usr/bin/env python3
"""NeoRacer web dashboard on port 8080: the apps page and the node and topic monitor."""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import logging
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
from types import FrameType
from typing import Any
from urllib.parse import parse_qs, urlparse

from diagnostic_msgs.msg import DiagnosticStatus
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import BatteryState

# The shared web assets and the host readings live beside this file. It is run
# by path, so its folder goes on the path before they are imported.
sys.path.insert(0, str(Path(__file__).resolve().parent))

import features  # noqa: E402, I100  sibling scripts, after the path insert
import overview  # noqa: E402
import sysinfo  # noqa: E402
import webui  # noqa: E402

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

PORT = 8080

# Home page apps. Core apps start at boot. The lab dashboards start when
# opened and stop with Clear apps, through `sudo -n systemctl` and the
# sudoers drop-in /etc/sudoers.d/neoracer. `exclusive` apps publish /drive,
# so one runs at a time. `needs` names what an app reads: the camera, or
# inference, which turns the camera on too.
# fmt: off
APPS: list[dict[str, Any]] = [
    {'id': 'jupyter', 'name': 'JupyterLab', 'icon': 'notebook', 'port': 8888,
     'group': 'core', 'unit': 'neoracer-jupyter', 'exclusive': False, 'startable': False,
     'desc': 'Notebooks and code'},
    {'id': 'mapview', 'name': 'Map Viewer', 'icon': 'map', 'port': 8088,
     'group': 'core', 'unit': 'neoracer-mapview', 'exclusive': False, 'startable': False,
     'desc': 'Build and load maps'},
    {'id': 'wallfollow', 'name': 'Wall Follow', 'icon': 'wall', 'port': 8081,
     'group': 'apps', 'unit': 'neoracer-wallfollow', 'exclusive': True, 'startable': True,
     'desc': 'Follow walls with the lidar.'},
    {'id': 'camlabel', 'name': 'Camera Label', 'icon': 'tag', 'port': 8082,
     'group': 'apps', 'unit': 'neoracer-camlabel', 'exclusive': False, 'startable': True,
     'desc': 'Capture camera frames and label them for training.', 'needs': 'camera'},
    {'id': 'pursuit', 'name': 'Pure Pursuit', 'icon': 'target', 'port': 8083,
     'group': 'apps', 'unit': 'neoracer-pursuit', 'exclusive': True, 'startable': True,
     'desc': 'Follow a detected target with the camera.', 'needs': 'inference'},
    {'id': 'eps', 'name': 'Policy Search', 'icon': 'refresh', 'port': 8084,
     'group': 'apps', 'unit': 'neoracer-eps', 'exclusive': True, 'startable': True,
     'desc': 'Tune a gap follower from the result of each run.'},
    {'id': 'smartfollow', 'name': 'Smart Follow', 'icon': 'merge', 'port': 8085,
     'group': 'apps', 'unit': 'neoracer-smartfollow', 'exclusive': True, 'startable': True,
     'desc': 'Follow a detected car, or the walls when none is in view.', 'needs': 'inference'},
    {'id': 'linefollow', 'name': 'Line Follow', 'icon': 'line', 'port': 8086,
     'group': 'apps', 'unit': 'neoracer-linefollow', 'exclusive': True, 'startable': True,
     'desc': 'Follow a colored line with the camera.', 'needs': 'camera'},
    {'id': 'webteleop', 'name': 'Web Teleop', 'icon': 'gamepad', 'port': 8087,
     'group': 'apps', 'unit': 'neoracer-webteleop', 'exclusive': True, 'startable': True,
     'desc': 'Drive from the browser with camera and lidar.'},
]

# Features: parts of the car that cost CPU and can be turned off. The camera
# and detections are teleop subsystems (scripts/features.py); changing one
# restarts teleop. Autonomy is its own unit, started from the map viewer.
FEATURES: list[dict[str, Any]] = [
    {'id': 'camera', 'name': 'Camera', 'icon': 'camera',
     'desc': 'Turn the USB camera on or off.'},
    {'id': 'detections', 'name': 'Detections', 'icon': 'target',
     'desc': 'Turn the YOLO detection node on or off.'},
    {'id': 'autonomy', 'name': 'Autonomy', 'icon': 'radar',
     'desc': 'Mapping or localization.'},
]
# fmt: on
TELEOP_UNIT = 'neoracer-teleop.service'
AUTONOMY_UNIT = 'neoracer-autonomy.service'
AUTONOMY_MODE_FILE = Path.home() / '.config' / 'racecar' / 'autonomy.conf'
MAP_TOOL = [sys.executable, str(Path(__file__).resolve().parent / 'neoracer_map.py')]
MAP_TOOL_TIMEOUT_SEC = 180.0
# Time for the sensors to come up after a teleop restart before autonomy resumes.
TELEOP_SETTLE_SEC = 10.0
READY_TIMEOUT_SEC = 1.0
SYSTEMCTL_TIMEOUT_SEC = 30.0
RATE_WINDOW_SEC = 3.0
MAPVIEW_PORT = 8088
# /autonomy/status arrives at 1 Hz; older than this, the node has stopped.
AUTONOMY_STALE_SEC = 5.0
SYSTEM_HEALTH_REFRESH_SEC = 60.0
# Rates, nodes, and app states are measured only while a page polls: the rate
# subscriptions wake the process for every message, a large share of a core.
# They stop VIEW_IDLE_SEC after the last request.
VIEW_IDLE_SEC = 15.0
MONITOR_PERIOD_SEC = 3.0

# `supervised`: the watchdog restarts this node. A card whose topic has no
# publisher reads not detected either way; a subsystem teleop was launched
# without reads Off.
MONITORED: dict[str, dict[str, Any]] = {
    'controller': {'topic': '/imu/fused', 'label': 'ESP32 (IMU/odom/joy)', 'supervised': True},
    'throttle': {'topic': '/motor', 'label': 'Throttle (clamping)', 'supervised': True},
    'mux': {'topic': '/mux_out', 'label': 'Mux (arbitrator)', 'supervised': True},
    'gamepad': {'topic': '/gamepad_drive', 'label': 'Gamepad', 'supervised': True},
    'odom': {'topic': '/odom', 'label': 'Odometry', 'supervised': False},
    'joy': {'topic': '/joy', 'label': 'FlySky RC (/joy)', 'supervised': False},
    'lidar': {'topic': '/scan', 'label': 'Lakibeam lidar', 'supervised': True},
    'camera': {'topic': '/camera/color', 'label': 'USB camera', 'supervised': True},
    # Off at boot (scripts/features.py). The watchdog does not restart it, and
    # with it turned on, a car without `racecar setup ml` shows it as not
    # detected.
    'inference': {'topic': '/edgetpu/inference', 'label': 'YOLO inference', 'supervised': False},
}

# Subscribed to and counted by the dashboard, without deserializing a message.
RATE_TOPICS = [
    '/motor',
    '/mux_out',
    '/imu/fused',
    '/odom',
    '/joy',
    '/scan',
    '/camera/color',
    '/edgetpu/inference',
]

log = logging.getLogger('dashboard')

# ---------------------------------------------------------------------------
# Status collection (cached, background-refreshed)
# ---------------------------------------------------------------------------

_status_lock = threading.Lock()
_latest_status: dict[str, Any] = {
    'timestamp': '',
    'nodes': {},
    'node_list': [],
    'topic_list': [],
    'rates': {},
    'system_health': {},
    'watchdog_log': [],
    'log_dir': str(Path.home() / 'logs' / 'latest'),
    'autonomy': {},
    'apps': [],
}
_monitor_running = True


# ---------------------------------------------------------------------------
# Rate measurement via rclpy subscriptions
# ---------------------------------------------------------------------------


class _RateSampler(Node):
    """Holds one BEST_EFFORT subscription per RATE_TOPICS entry and tracks arrivals."""

    def __init__(self, topics: list[str], window_sec: float = RATE_WINDOW_SEC) -> None:
        super().__init__('racecar_dashboard')
        self._window = window_sec
        self._stamps: dict[str, deque[float]] = {t: deque() for t in topics}
        self._lock = threading.Lock()
        # Subscriptions need a concrete message type, so attach_subscriptions()
        # creates each one once the topic and its type appear on the graph.
        self._qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._topics = list(topics)
        self._subs: dict[str, Any] = {}
        self.attached_at: float | None = None
        # 1 Hz and latched: the autonomy card costs one small message a second.
        self._autonomy: tuple[dict[str, str], float] | None = None
        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._autonomy_sub = self.create_subscription(
            DiagnosticStatus, '/autonomy/status', self._record_autonomy, latched
        )
        # The pack voltage as (volts, monotonic stamp), kept while no page is
        # open so the history has no gaps. None until the first /battery
        # message; pre-V1.1 firmware never sends one.
        self._battery: tuple[float, float] | None = None
        self._battery_sub = self.create_subscription(
            BatteryState, '/battery', self._record_battery, self._qos
        )

    def _record(self, topic: str) -> None:
        now = time.monotonic()
        with self._lock:
            dq = self._stamps[topic]
            dq.append(now)
            cutoff = now - self._window
            while dq and dq[0] < cutoff:
                dq.popleft()

    def _record_battery(self, msg: BatteryState) -> None:
        self._battery = (float(msg.voltage), time.monotonic())
        _battery_history.add(time.time(), float(msg.voltage))

    def battery(self) -> tuple[float, float] | None:
        """Return the latest (volts, age in seconds), or None before the first reading."""
        if self._battery is None:
            return None
        volts, stamp = self._battery
        return volts, time.monotonic() - stamp

    def _record_autonomy(self, msg: DiagnosticStatus) -> None:
        values = {kv.key: kv.value for kv in msg.values}
        with self._lock:
            self._autonomy = (values, time.monotonic())

    def autonomy(self) -> tuple[dict[str, str] | None, float | None]:
        """Return the last /autonomy/status values and their age in seconds."""
        with self._lock:
            if self._autonomy is None:
                return None, None
            values, at = self._autonomy
            return dict(values), time.monotonic() - at

    def attach_subscriptions(self) -> None:
        """Resolve each topic's type and create a subscription. Re-runnable; idempotent."""
        names_types = dict(self.get_topic_names_and_types())
        for topic in self._topics:
            if topic in self._subs:
                continue
            types = names_types.get(topic)
            if not types:
                continue
            try:
                msg_module, msg_name = types[0].rsplit('/', 1)
                pkg = msg_module.split('/')[0]
                module = __import__(f'{pkg}.msg', fromlist=[msg_name])
                msg_cls = getattr(module, msg_name)
            except (ValueError, ImportError, AttributeError) as exc:
                log.debug('Skipping %s: %s', topic, exc)
                continue
            self._subs[topic] = self.create_subscription(
                msg_cls, topic, self._arrival_callback(topic), self._qos, raw=True
            )
            if self.attached_at is None:
                self.attached_at = time.monotonic()

    def detach_subscriptions(self) -> None:
        """Drop the rate subscriptions and their counts until the next attach."""
        for sub in self._subs.values():
            self.destroy_subscription(sub)
        self._subs.clear()
        with self._lock:
            for dq in self._stamps.values():
                dq.clear()
        self.attached_at = None

    def warming(self) -> bool:
        """Return True until the rate window has filled since the subscriptions opened."""
        return self.attached_at is None or time.monotonic() - self.attached_at < self._window

    def _arrival_callback(self, topic: str) -> Callable[[Any], None]:
        return lambda _msg: self._record(topic)

    def measure_hz(self, topic: str) -> float | None:
        """Return the arrival rate (Hz) over the window, or None when no data."""
        with self._lock:
            dq = self._stamps.get(topic)
            if dq is None:
                return None
            now = time.monotonic()
            cutoff = now - self._window
            while dq and dq[0] < cutoff:
                dq.popleft()
            if len(dq) < 2:
                return None
            return len(dq) / self._window

    def topic_list(self) -> list[tuple[str, list[str]]]:
        return sorted(self.get_topic_names_and_types(), key=lambda x: x[0])

    def node_list(self) -> list[str]:
        return sorted(f'/{n}' if not n.startswith('/') else n for n in self.get_node_names())


_sampler: _RateSampler | None = None
# The System Health page's sensor panels (overview.Overview once main() runs),
# and its history charts: the battery voltage from /battery, the junction
# temperature and memory in use from the monitor loop.
_overview: Any = None
_battery_history = overview.History()
_system_history = overview.History()


def _measure_hz(topic: str) -> float | None:
    """Return arrival rate (Hz) for a topic from the rclpy sampler, or None."""
    sampler = _sampler
    if sampler is None:
        return None
    return sampler.measure_hz(topic)


def _count_publishers(topic: str) -> int:
    sampler = _sampler
    if sampler is None:
        return 0
    try:
        return sampler.count_publishers(topic)
    except Exception:  # noqa: BLE001  a graph query during shutdown
        return 0


def _get_topic_list() -> list[str]:
    sampler = _sampler
    if sampler is None:
        return []
    return [name for name, _types in sampler.topic_list()]


def _get_node_list() -> list[str]:
    sampler = _sampler
    if sampler is None:
        return []
    return sampler.node_list()


WATCHDOG_LOG = Path.home() / 'logs' / 'latest' / 'watchdog.log'
# Byte offset of the last Clear, beside the log of this teleop session.
WATCHDOG_CLEARED = WATCHDOG_LOG.with_name('watchdog.cleared')


def _cleared_offset(size: int) -> int:
    try:
        offset = int(WATCHDOG_CLEARED.read_text().strip() or 0)
    except (OSError, ValueError):
        return 0
    # A shorter log is a new file, so an old mark does not apply.
    return offset if offset <= size else 0


def _read_watchdog_tail(n: int = 10, max_bytes: int = 4096) -> list[str]:
    """Return the last n lines of the watchdog log after the last Clear."""
    try:
        with open(WATCHDOG_LOG, 'rb') as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(_cleared_offset(size), size - max_bytes))
            tail = f.read()
    except OSError:
        return []
    return tail.decode(errors='replace').splitlines()[-n:]


def clear_watchdog_log() -> tuple[bool, str]:
    """Hide the lines logged so far; the file itself is kept."""
    try:
        size = WATCHDOG_LOG.stat().st_size
    except OSError:
        return True, 'Log is empty.'
    try:
        WATCHDOG_CLEARED.write_text(f'{size}\n')
    except OSError as exc:
        return False, str(exc)
    return True, 'Log cleared.'


# Junction temperature bands (C), the same as `racecar status` and the System
# Health page. The Orin throttles near 95 C, so warm and hot warn well before.
TEMP_OK_C = 70.0
TEMP_HIGH_C = 85.0
# The data root reads low below this much free space.
DATA_WARN_FREE_GB = 10


def health(label: str, status: str, detail: str, word: str = '') -> dict[str, str]:
    """
    Return one health card: its status (card color), detail, and problem word.

    The page writes the word only for a problem, as the System Health page
    does, and keeps the detail for the card's lines and its tooltip.
    """
    return {'label': label, 'status': status, 'detail': detail, 'word': word}


def temperature_health(celsius: float | None) -> dict[str, str]:
    """Map the junction temperature to a health card; only HOT is a fault."""
    label = 'Junction temperature'
    if celsius is None:
        return health(label, 'stale', 'No reading', 'No reading')
    if celsius < TEMP_OK_C:
        return health(label, 'healthy', f'{celsius:.1f} C')
    if celsius < TEMP_HIGH_C:
        return health(label, 'stale', f'{celsius:.1f} C', 'Warm')
    return health(label, 'dead', f'{celsius:.1f} C', 'HOT')


def data_root_health() -> dict[str, str]:
    """Report where the car's data goes and its free space; missing or read-only is a fault."""
    from neoracer_ros2_driver import data_root

    label = 'Data root'
    root = data_root.current()[0]
    try:
        free = shutil.disk_usage(root).free / 1e9 if root.is_dir() else None
    except OSError:
        free = None
    if free is None:
        return health(label, 'dead', f'{root} missing', 'Missing')
    detail = f'{root}, {free:.0f} GB free'
    if data_root.session_problem():
        return health(label, 'dead', f'{detail}, not writable', 'Not writable')
    if free < DATA_WARN_FREE_GB:
        return health(label, 'stale', detail, 'Low space')
    return health(label, 'healthy', detail)


def throttle_health(flags: int | None, active: list[str]) -> dict[str, str]:
    """
    Report the SoC's over-current throttling since boot.

    The counters only grow until a reboot, so a count is a past event and
    reads Check, as in `racecar status`.
    """
    label = 'Throttling'
    if flags is None:
        return health(label, 'stale', 'No over-current counters', 'No reading')
    if not flags:
        return health(label, 'healthy', 'None since boot')
    return health(label, 'stale', '; '.join(active), 'Has occurred')


def _collect_system_health() -> dict[str, dict[str, str]]:
    """Slow-refresh diagnostics: the data root, the junction temperature, throttling."""
    return {
        'data': data_root_health(),
        'temperature': temperature_health(sysinfo.read_soc_temp()),
        'throttling': throttle_health(*sysinfo.read_throttled()),
    }


# 3S LiPo bands: full 12.6 V, low warning 11.1 V, cutoff 10.8 V.
BATTERY_V_LOW = 11.1
BATTERY_V_MIN = 10.8
BATTERY_V_MAX = 12.6
BATTERY_STALE_SEC = 15.0


def _battery_host() -> dict[str, Any] | None:
    """Battery reading for /api/host and the apps page: volts, percent, age and state, or None."""
    sampler = _sampler
    reading = sampler.battery() if sampler is not None else None
    if reading is None:
        return None
    volts, age = reading
    pct = min(1.0, max(0.0, (volts - BATTERY_V_MIN) / (BATTERY_V_MAX - BATTERY_V_MIN)))
    if age > BATTERY_STALE_SEC:
        state = 'stale'
    elif volts < BATTERY_V_MIN:
        state = 'charge'
    elif volts < BATTERY_V_LOW:
        state = 'low'
    else:
        state = 'ok'
    return {
        'volts': round(volts, 2),
        'pct': round(pct * 100),
        'age_s': round(age, 1),
        'state': state,
    }


# Battery state -> (card status, problem word).
BATTERY_VIEW = {
    'ok': ('healthy', ''),
    'low': ('stale', 'Low'),
    'charge': ('dead', 'CHARGE NOW'),
    'stale': ('stale', 'No reading'),
}


def _battery_card() -> dict[str, str] | None:
    """Battery card for system_health, or None before the first reading."""
    battery = _battery_host()
    if battery is None:
        return None
    status, word = BATTERY_VIEW[battery['state']]
    if battery['state'] == 'stale':
        detail = f'No reading for {battery["age_s"]:.0f} s'
    else:
        detail = f'{battery["volts"]:.1f} V ({battery["pct"]}%)'
    return health('Battery (3S pack)', status, detail, word)


# Watchdog states that change a card; healthy and starting leave the
# topic-based status alone. A node launched off reads Off, as on the System
# Health page.
WATCHDOG_VIEW = {
    'gave_up': ('dead', 'watchdog gave up; replug the device or restart teleop'),
    'no_device': ('stale', 'device not connected'),
    'restarting': ('stale', 'watchdog restarting'),
    'disabled': ('off', 'turned off'),
}


def watchdog_view(status: str, watched: dict[str, Any] | None) -> tuple[str, str]:
    """Return (card status, detail) after applying the watchdog's state for a node."""
    if not watched or watched.get('state') not in WATCHDOG_VIEW:
        return status, ''
    view_status, detail = WATCHDOG_VIEW[watched['state']]
    if watched['state'] == 'restarting':
        detail += f" ({watched.get('restarts', 0)} so far)"
    return view_status, detail


def autonomy_view(values: dict[str, str] | None, age: float | None) -> dict[str, Any]:
    """
    Card for the autonomy service: status (card color), headline, and detail lines.

    healthy: localized; stale: running but not localized; dead: status stopped
    arriving without the node saying it stopped; off: stopped.
    """
    if values is None or values.get('mode') in (None, '', 'off'):
        return {
            'status': 'off',
            'headline': 'Stopped',
            'lines': ['Start mapping or load a map in the map viewer.'],
            'port': MAPVIEW_PORT,
        }
    mode, name = values.get('mode', ''), values.get('map') or '(no map)'
    headline = f'{mode.capitalize()} on {name}'
    if mode == 'localization' and values.get('localizer'):
        headline += f" ({values['localizer']})"
    if age is not None and age > AUTONOMY_STALE_SEC:
        return {
            'status': 'dead',
            'headline': headline,
            'lines': [f'No status for {age:.0f} s. The autonomy service may have stopped.'],
            'port': MAPVIEW_PORT,
        }
    localized = values.get('localized') == 'true'
    lines = ['Localized' if localized else 'Not localized']
    if mode == 'localization' and values.get('scan_fit'):
        lines.append(f"Scan fit {float(values['scan_fit']):.0%}")
    if mode == 'localization' and values.get('pose_sd_m'):
        sd = f"{float(values['pose_sd_m']):.2f} m, {float(values.get('pose_sd_deg') or 0):.0f} deg"
        lines.append(f'Spread {sd}')
    if mode == 'mapping':
        lines.append(f"Saved {values.get('last_save') or 'not yet'}")
    return {
        'status': 'healthy' if localized else 'stale',
        'headline': headline,
        'lines': lines,
        'port': MAPVIEW_PORT,
    }


def unit_states(units: list[str], run: Callable[..., Any] = subprocess.run) -> dict[str, str]:
    """Return systemctl is-active for each unit: active, inactive, failed, activating, ..."""
    try:
        out = run(['systemctl', 'is-active', *units], capture_output=True, text=True, timeout=5)
    except (OSError, subprocess.TimeoutExpired):
        return dict.fromkeys(units, 'unknown')
    lines = out.stdout.split()
    return {u: (lines[i] if i < len(lines) else 'unknown') for i, u in enumerate(units)}


def app_list(states: dict[str, str]) -> list[dict[str, Any]]:
    return [
        {**app, 'state': states.get(app['unit'], 'unknown') if app['unit'] else 'unavailable'}
        for app in APPS
    ]


def _app_units() -> list[str]:
    return [a['unit'] for a in APPS if a['unit']]


def open_app(target: str, run: Callable[..., Any] = subprocess.run) -> tuple[bool, str]:
    """
    Start `target` if it is not running.

    An exclusive app first stops the other exclusive one and a running
    autonomy run (a mapping run is saved first); the page asks the user before
    calling this.
    """
    app = next((a for a in APPS if a['id'] == target), None)
    if app is None:
        return False, f'Unknown app: {target}'
    if app['unit'] is None:
        return False, f'{app["name"]} is not available.'
    if app['exclusive']:
        live = ('active', 'activating')
        if (
            unit_states([app['unit']], run)[app['unit']] not in live
            and unit_states([AUTONOMY_UNIT], run)[AUTONOMY_UNIT] in live
        ):
            mapping = last_run().get('mode') == 'mapping'
            message = autonomy_action('save-stop' if mapping else 'stop', run)
            if unit_states([AUTONOMY_UNIT], run)[AUTONOMY_UNIT] in live:
                return False, f'Autonomy is still running. {message}'
        return activate(target, run)
    if unit_states([app['unit']], run)[app['unit']] in ('active', 'activating'):
        return True, f'{app["name"]} is running.'
    if not app['startable']:
        return False, f'{app["name"]} is stopped. Run: racecar service start {target}'
    error = _systemctl('start', app['unit'], run)
    if error:
        return False, error
    return True, f'{app["name"]} started.'


def clear_apps(run: Callable[..., Any] = subprocess.run) -> tuple[bool, str]:
    """Stop every running lab dashboard to free CPU; core apps and the dashboard keep running."""
    units = [a['unit'] for a in APPS if a['group'] == 'apps']
    states = unit_states(units, run)
    stopped, errors = 0, []
    for unit in units:
        if states.get(unit) in ('active', 'activating'):
            error = _systemctl('stop', unit, run)
            if error:
                errors.append(error)
            else:
                stopped += 1
    if errors:
        return False, ' '.join(errors)
    return True, 'Lab dashboards stopped.' if stopped else 'No lab dashboards running.'


def unit_file(unit: str) -> str:
    """Return the unit's full name, the form the sudoers file lists."""
    return unit if unit.endswith('.service') else f'{unit}.service'


def systemctl_argv(verb: str, unit: str) -> list[str]:
    """
    Return the command that starts, stops or restarts one unit.

    /etc/sudoers.d/neoracer lets the racecar user run exactly this for each
    neoracer-* unit by its full name, with no password. -n makes sudo fail at
    once when the file is missing, instead of waiting for a password.
    """
    return ['sudo', '-n', 'systemctl', verb, unit_file(unit)]


def _systemctl(verb: str, unit: str, run: Callable[..., Any]) -> str:
    """Run one unit's systemctl command through sudo; return '' or a short error."""
    argv = systemctl_argv(verb, unit)
    try:
        r = run(argv, capture_output=True, text=True, timeout=SYSTEMCTL_TIMEOUT_SEC)
    except (OSError, subprocess.TimeoutExpired) as exc:
        return str(exc)
    if r.returncode == 0:
        return ''
    stderr = (r.stderr or '').strip()
    if 'password is required' in stderr or 'not allowed to execute' in stderr:
        return f'No permission to {verb} {argv[-1]}. Run: racecar service install'
    return stderr.splitlines()[-1] if stderr else f'{verb} failed'


def app_ready(port: int, timeout: float = READY_TIMEOUT_SEC) -> bool:
    """Return True once the app's web server answers on localhost."""
    conn = http.client.HTTPConnection('127.0.0.1', port, timeout=timeout)
    try:
        conn.request('GET', '/')
        conn.getresponse().read(0)
        return True
    except (OSError, http.client.HTTPException):
        return False
    finally:
        conn.close()


def activate(target: str, run: Callable[..., Any] = subprocess.run) -> tuple[bool, str]:
    """Make `target` the one running exclusive app, or stop them all for 'none'."""
    labs = {a['id']: a['unit'] for a in APPS if a['exclusive']}
    if target != 'none' and target not in labs:
        return False, f'Unknown dashboard: {target}'
    states = unit_states(list(labs.values()), run)

    for lab_id, unit in labs.items():
        if lab_id != target and states.get(unit) in ('active', 'activating'):
            error = _systemctl('stop', unit, run)
            if error:
                return False, error
    if target == 'none':
        return True, 'Lab dashboards stopped.'
    if states.get(labs[target]) not in ('active', 'activating'):
        error = _systemctl('start', labs[target], run)
        if error:
            return False, error
    name = next(a['name'] for a in APPS if a['id'] == target)
    return True, f'{name} active.'


def last_run(path: Path | None = None) -> dict[str, str]:
    """Return the mode and map of the last autonomy run from its mode file, or {}."""
    try:
        lines = (path or AUTONOMY_MODE_FILE).read_text().splitlines()
    except OSError:
        return {}
    values = dict(line.split('=', 1) for line in lines if '=' in line)
    mode, name = values.get('AUTONOMY_MODE', ''), values.get('AUTONOMY_MAP', '')
    if mode not in ('mapping', 'localization') or not name:
        return {}
    return {'mode': mode, 'map': name}


def feature_list(
    states: dict[str, str],
    wanted: dict[str, bool],
    running: dict[str, bool],
    restarting: bool,
    last: dict[str, str],
) -> list[dict[str, Any]]:
    """
    Return one tile per feature with its state: active, activating, or inactive.

    `restarting` is true while a camera or detections change from the apps
    page restarts teleop. Starting or stopping autonomy leaves teleop running,
    so the camera and detections keep their own state meanwhile.
    """
    teleop = states.get(TELEOP_UNIT, 'unknown')
    out = []
    for item in FEATURES:
        if item['id'] == 'autonomy':
            state = states.get(AUTONOMY_UNIT, 'unknown')
            out.append({**item, 'state': state, 'last': last})
            continue
        on = teleop == 'active' and running[item['id']]
        pending = (
            restarting
            or teleop in ('activating', 'reloading')
            or (teleop == 'active' and running[item['id']] != wanted[item['id']])
        )
        state = 'activating' if pending else ('active' if on else 'inactive')
        out.append({**item, 'state': state, 'wanted': wanted[item['id']]})
    return out


def map_tool(*argv: str, run: Callable[..., Any] = subprocess.run) -> tuple[int, str]:
    """Run `racecar map ARGV`; return its exit code and last output line."""
    try:
        r = run(
            [*MAP_TOOL, *argv],
            capture_output=True,
            text=True,
            timeout=MAP_TOOL_TIMEOUT_SEC,
            stdin=subprocess.DEVNULL,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return 1, str(exc)
    lines = [line for line in (r.stdout + r.stderr).splitlines() if line.strip()]
    return r.returncode, lines[-1] if lines else ''


def describe(values: dict[str, bool]) -> str:
    names = {f['id']: f['name'] for f in FEATURES}
    return ' '.join(f'{names[k]} {"on" if v else "off"}.' for k, v in values.items())


def apply_features(
    values: dict[str, bool],
    run: Callable[..., Any] = subprocess.run,
    sleep: Callable[[float], None] = time.sleep,
) -> str:
    """
    Restart teleop with `values`; a running autonomy run is stopped first and resumed after.

    Stopping it first lets a localization run start again from the car's last
    pose rather than from where that run began. The run stays stopped when a
    driving app started meanwhile: the apps page turns on what an app needs
    while the app opens, and `racecar map resume` would close that app again.
    """
    features.write(values)
    resume = unit_states([AUTONOMY_UNIT], run)[AUTONOMY_UNIT] in ('active', 'activating')
    if resume:
        code, line = map_tool('stop', run=run)
        if code != 0:
            return f'Could not stop autonomy: {line}'
    error = _systemctl('restart', TELEOP_UNIT, run)
    if error:
        return error
    message = describe(values)
    if resume:
        sleep(TELEOP_SETTLE_SEC)
        driving = [a for a in APPS if a['exclusive']]
        states = unit_states([a['unit'] for a in driving], run)
        opened = [a['name'] for a in driving if states[a['unit']] in ('active', 'activating')]
        if opened:
            return f'{message} Autonomy stays stopped, because {opened[0]} is open.'
        code, line = map_tool('resume', run=run)
        message += f' {line}' if code == 0 else f' Autonomy did not resume: {line}'
    return message


def autonomy_action(action: str, run: Callable[..., Any] = subprocess.run) -> str:
    if action == 'resume':
        return map_tool('resume', run=run)[1]
    if action == 'save-stop':
        code, line = map_tool('save', run=run)
        if code != 0:
            return f'Not stopped. {line}'
    if action in ('stop', 'save-stop'):
        return map_tool('stop', run=run)[1]
    raise ValueError(f'unknown autonomy action {action!r}')


class FeatureJob:
    """One feature change at a time, in a thread; its message is shown on the page."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.busy = False
        self.teleop = False
        self.message = ''

    def start(self, label: str, work: Callable[[], str], teleop: bool = False) -> bool:
        """Start `work` unless a job runs; `teleop` marks a job that restarts teleop."""
        with self._lock:
            if self.busy:
                return False
            self.busy, self.teleop, self.message = True, teleop, label
        threading.Thread(target=self._run, args=(work,), daemon=True).start()
        return True

    def _run(self, work: Callable[[], str]) -> None:
        try:
            message = work()
        except Exception as exc:  # noqa: BLE001  shown to the user, not raised in a thread
            log.exception('feature change failed')
            message = f'Failed: {exc}'
        log.info('features: %s', message)
        with self._lock:
            self.busy, self.message = False, message

    def state(self) -> dict[str, Any]:
        with self._lock:
            return {
                'busy': self.busy,
                'teleop': self.busy and self.teleop,
                'message': self.message,
            }


_features_job = FeatureJob()


def features_view() -> dict[str, Any]:
    job = _features_job.state()
    items = feature_list(
        unit_states([TELEOP_UNIT, AUTONOMY_UNIT]),
        features.read(),
        features.running(),
        job['teleop'],
        last_run(),
    )
    return {'items': items, 'job': job}


def change_feature(name: str, on: bool, action: str = '') -> tuple[bool, str]:
    """Start a feature change in the background; return whether it started."""
    if name == 'autonomy':
        if action not in ('resume', 'stop', 'save-stop'):
            return False, 'Unknown action.'
        label = 'Starting autonomy...' if action == 'resume' else 'Stopping autonomy...'
        work: Callable[[], str] = lambda: autonomy_action(action)  # noqa: E731
    else:
        try:
            values = features.change(features.read(), name, on)
        except ValueError as exc:
            return False, str(exc)
        label = 'Restarting teleop...'
        work = lambda: apply_features(values)  # noqa: E731
    if not _features_job.start(label, work, teleop=name != 'autonomy'):
        return False, 'Another change is running.'
    return True, label


def _refresh_apps() -> None:
    apps = app_list(unit_states(_app_units()))
    with _status_lock:
        _latest_status['apps'] = apps


_last_view = 0.0
_wake = threading.Event()


def note_view() -> bool:
    """Record a page request; return True when monitoring was idle until now."""
    global _last_view
    idle = not viewing()
    _last_view = time.monotonic()
    if idle:
        _wake.set()
    return idle


def viewing() -> bool:
    return time.monotonic() - _last_view < VIEW_IDLE_SEC


def node_status(
    present: bool, cfg: dict[str, Any], launched: bool, watched: dict[str, Any] | None
) -> tuple[str, str]:
    """
    Return (card status, detail) for one monitored node.

    Present is judged by publisher count, not topic existence: the sampler's
    own subscriptions create the topics, so existence always reads true and a
    dead node never went red.
    """
    if present:
        status = 'healthy'
    elif not launched:
        status = 'off'
    elif cfg['supervised']:
        status = 'dead'
    else:
        status = 'unsupervised'
    return watchdog_view(status, watched)


def _monitor_loop() -> None:
    """Background thread that refreshes the cached status snapshot while a page polls."""
    # Seed empty and let the first pass collect system health inside the try
    # block; an unguarded call here once killed the whole thread on boot.
    last_system_health = 0.0
    system_health: dict[str, dict[str, str]] = {}
    while _monitor_running:
        try:
            overview.sample_system(
                _system_history, time.time(), sysinfo.read_soc_temp(), sysinfo.read_memory()
            )
            if not viewing():
                if _sampler is not None and _sampler.attached_at is not None:
                    _sampler.detach_subscriptions()
                    log.info('No page open; rate sampling stopped')
                _wake.wait(MONITOR_PERIOD_SEC)
                _wake.clear()
                continue
            # Publishers can appear after the dashboard starts, so retry the
            # topic-to-type lookup each tick.
            if _sampler is not None:
                _sampler.attach_subscriptions()

            topics = _get_topic_list()
            nodes = _get_node_list()

            watchdog = sysinfo.read_watchdog_state()
            watched = (
                (watchdog or {}).get('nodes', {}) if not (watchdog or {}).get('stale') else {}
            )
            # By teleop subsystem, which is the card name here (the camera and
            # inference cards).
            launched = {
                features.SUBSYSTEMS[feature]: on for feature, on in features.running().items()
            }
            node_cards = {}
            for name, cfg in MONITORED.items():
                present = _count_publishers(cfg['topic']) > 0
                status, detail = node_status(
                    present, cfg, launched.get(name, True), watched.get(name)
                )
                node_cards[name] = {
                    'label': cfg['label'],
                    'topic': cfg['topic'],
                    'alive': present,
                    'supervised': cfg['supervised'],
                    'status': status,
                    'detail': detail,
                }

            rates: dict[str, dict[str, Any]] = {}
            warming = _sampler is not None and _sampler.warming()
            # A topic whose node was launched off reads Off, as its card does.
            off = {c['topic'] for c in node_cards.values() if c['status'] == 'off'}
            for topic in RATE_TOPICS:
                hz = _measure_hz(topic)
                if topic in off and hz is None:
                    rates[topic] = {'hz': None, 'stale': False, 'off': True}
                elif warming and hz is None:
                    rates[topic] = {'hz': None, 'stale': False, 'warming': True}
                else:
                    rates[topic] = {'hz': hz, 'stale': hz is None or hz < 0.5}

            now = time.monotonic()
            if now - last_system_health >= SYSTEM_HEALTH_REFRESH_SEC:
                system_health = _collect_system_health()
                last_system_health = now
            # The battery changes faster than the 60 s health refresh; merge it
            # in every tick.
            health_now = dict(system_health)
            battery = _battery_card()
            if battery is not None:
                health_now['battery'] = battery

            snapshot = {
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'nodes': node_cards,
                'node_list': nodes,
                'topic_list': topics,
                'rates': rates,
                'system_health': health_now,
                'watchdog_log': _read_watchdog_tail(),
                'log_dir': str(Path.home() / 'logs' / 'latest'),
                'autonomy': autonomy_view(*(_sampler.autonomy() if _sampler else (None, None))),
                'apps': app_list(unit_states(_app_units())),
                'battery': _battery_host(),
                'host': socket.gethostname(),
            }

            with _status_lock:
                _latest_status.update(snapshot)

        except Exception:  # noqa: BLE001
            log.exception('Error in monitor loop')

        _wake.wait(MONITOR_PERIOD_SEC)
        _wake.clear()


def get_status() -> dict[str, Any]:
    """Return the most recent status snapshot (non-blocking)."""
    with _status_lock:
        return dict(_latest_status)


# ---------------------------------------------------------------------------
# HTTP handler; the pages are scripts/dashboard.html, dashboard_launch.html
# and health.html
# ---------------------------------------------------------------------------

_HTML_PATH = Path(__file__).resolve().parent / 'dashboard.html'
_LAUNCH_PATH = Path(__file__).resolve().parent / 'dashboard_launch.html'
# The System Health page: the Jetson's live readings, the sensor panels and
# the battery, on the shared /ui/ styles.
_HEALTH_PATH = Path(__file__).resolve().parent / 'health.html'
_host_poller = sysinfo.HostPoller()
_host_lock = threading.Lock()


def _read_page(path: Path) -> str:
    """Read a page on each request, so an edited page shows on the next reload."""
    try:
        return path.read_text(encoding='utf-8')
    except OSError as exc:
        return f'<!DOCTYPE html><body><pre>{path.name} unreadable: {exc}</pre>'


def _load_dashboard_html() -> str:
    return _read_page(_HTML_PATH)


DASHBOARD_HTML = _load_dashboard_html()


class DashboardHandler(BaseHTTPRequestHandler):
    """
    Serve the pages, their JSON, the camera's latest frame, and /ui/ (shared files).

    GET: /, /launch and /health are HTML; /api/status, /api/ready, /api/host,
    /api/overview and /api/history are JSON; /api/overview/color.jpg is the
    camera frame. POST: /api/open, /api/feature, /api/clear-apps and
    /api/clear-log.
    """

    def do_GET(self) -> None:  # noqa: N802
        asset = webui.static_file(self.path)
        # Routes compare the path without its query, so /health?theme=light works.
        route = urlparse(self.path).path
        query = parse_qs(urlparse(self.path).query)
        if asset is not None:
            self._send(asset[0], asset[1], webui.CACHE_CONTROL)
        elif route == '/':
            self._send(_load_dashboard_html().encode(), 'text/html; charset=utf-8', 'no-store')
        elif route == '/launch':
            self._send(_read_page(_LAUNCH_PATH).encode(), 'text/html; charset=utf-8', 'no-store')
        elif route == '/api/status':
            if note_view():
                # The cached snapshot may be minutes old; refresh what is cheap.
                _refresh_apps()
            if query.get('refresh') == ['1']:
                _refresh_apps()
            self._send_json(200, {**get_status(), 'features': features_view()})
        elif route == '/api/ready':
            app = next((a for a in APPS if a['id'] == query.get('app', [''])[0]), None)
            if app is None:
                self._send_json(404, {'ready': False})
            else:
                self._send_json(200, {'ready': app_ready(app['port']), 'port': app['port']})
        elif route == '/health':
            self._serve_health()
        elif route == '/api/host':
            self._serve_host()
        elif route == '/api/overview':
            self._serve_overview()
        elif route == '/api/overview/color.jpg':
            self._serve_frame('color')
        elif route == '/api/history':
            data = overview.history_state(_battery_history, _system_history, time.time())
            self._send_json(200, data)
        else:
            self.send_error(404)

    def _send(self, body: bytes, content_type: str, cache: str | None = None) -> None:
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        if cache:
            self.send_header('Cache-Control', cache)
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, code: int, data: Any) -> None:
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _serve_health(self) -> None:
        try:
            body = _HEALTH_PATH.read_bytes()
        except OSError:
            self.send_error(404)
            return
        self._send(body, 'text/html; charset=utf-8', 'no-store')

    def _serve_host(self) -> None:
        with _host_lock:
            data = dict(_host_poller.poll())
        data['battery'] = _battery_host()
        self._send_json(200, data)

    def _serve_overview(self) -> None:
        feeds = _overview
        if feeds is None:
            self.send_error(503)
            return
        feeds.touch()
        self._send_json(200, feeds.state())

    def _serve_frame(self, name: str) -> None:
        feeds = _overview
        if feeds is not None:
            feeds.touch_cameras()
        frame = feeds.jpeg(name) if feeds is not None else None
        if frame is None:
            self.send_error(404)
            return
        self._send(frame, 'image/jpeg', 'no-store')

    def do_POST(self) -> None:  # noqa: N802
        route = urlparse(self.path).path
        if route == '/api/clear-apps':
            ok, message = clear_apps()
            _refresh_apps()
            log.info('clear apps: %s', message)
            self._send_json(200 if ok else 400, {'ok': ok, 'message': message})
            return
        if route == '/api/clear-log':
            ok, message = clear_watchdog_log()
            self._send_json(200 if ok else 400, {'ok': ok, 'message': message})
            return
        if route not in ('/api/open', '/api/feature'):
            self.send_error(404)
            return
        try:
            length = int(self.headers.get('Content-Length') or 0)
            body = json.loads(self.rfile.read(length) or b'{}')
            target = str(body.get('app', ''))
            action = str(body.get('action', ''))
            feature, on = str(body.get('feature', '')), bool(body.get('on'))
        except (ValueError, AttributeError):
            target = action = feature = ''
            on = False
        if route == '/api/feature':
            ok, message = change_feature(feature, on, action)
            self._send_json(200 if ok else 400, {'ok': ok, 'message': message})
            return
        ok, message = open_app(target)
        _refresh_apps()
        log.info('open %s: %s', target, message)
        self._send_json(200 if ok else 400, {'ok': ok, 'message': message})

    def log_message(self, fmt: str, *args: Any) -> None:
        """Suppress default per-request logging."""


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> None:
    global _monitor_running, _sampler, _overview

    logdir = Path.home() / 'logs' / 'latest'
    handlers: list[logging.Handler] = [logging.StreamHandler(sys.stderr)]
    try:
        if logdir.exists():
            handlers.append(logging.FileHandler(logdir / 'dashboard.log'))
    except OSError:
        pass

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
        handlers=handlers,
    )

    rclpy.init()
    sampler = _RateSampler(RATE_TOPICS)
    _sampler = sampler
    _overview = overview.Overview(sampler)

    def _spin_sampler() -> None:
        try:
            rclpy.spin(sampler)
        except (KeyboardInterrupt, SystemExit):
            pass
        except Exception:  # noqa: BLE001
            log.exception('rclpy.spin terminated')

    spinner = threading.Thread(target=_spin_sampler, daemon=True)
    spinner.start()
    log.info('rclpy sampler spinning')

    monitor = threading.Thread(target=_monitor_loop, daemon=True)
    monitor.start()
    log.info('Background monitor started')

    # Threads, so a phone slow to take a camera frame holds up no other request.
    server = ThreadingHTTPServer(('0.0.0.0', PORT), DashboardHandler)
    server.daemon_threads = True
    log.info('Dashboard listening on http://0.0.0.0:%d', PORT)

    def _shutdown(signum: int, _frame: FrameType | None) -> None:
        global _monitor_running
        log.info('Received signal %d, shutting down', signum)
        _monitor_running = False
        threading.Thread(target=server.shutdown).start()

    signal.signal(signal.SIGTERM, _shutdown)
    signal.signal(signal.SIGINT, _shutdown)

    try:
        server.serve_forever()
    finally:
        _monitor_running = False
        server.server_close()
        monitor.join(timeout=5)
        # rclpy shutdown wakes the spin thread; destroying the node while it
        # waits can block or abort the interpreter at exit.
        rclpy.try_shutdown()
        spinner.join(timeout=2)
        log.info('Dashboard stopped')


if __name__ == '__main__':
    main()
