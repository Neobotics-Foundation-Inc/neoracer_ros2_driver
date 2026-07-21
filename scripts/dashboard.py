#!/usr/bin/env python3
"""Neoracer web dashboard: live node/topic monitor on port 8080 (stdlib HTTP + rclpy)."""

from collections import deque
from http.server import BaseHTTPRequestHandler, HTTPServer
import json
import logging
from pathlib import Path
import signal
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

PORT = 8080
CACHE_TTL = 2.0
RATE_WINDOW_SEC = 3.0

# `supervised`: True if the watchdog will auto-restart this node. False means
# the card may go red without any recovery - surface that to the operator so
# the asymmetry isn't silent.
MONITORED = {
    'controller': {'topic': '/imu', 'label': 'ESP32 (IMU/odom/joy)', 'supervised': True},
    'throttle': {'topic': '/motor', 'label': 'Throttle (clamping)', 'supervised': True},
    'mux': {'topic': '/mux_out', 'label': 'Mux (arbitrator)', 'supervised': True},
    'gamepad': {'topic': '/gamepad_drive', 'label': 'Gamepad', 'supervised': True},
    'odom': {'topic': '/odom', 'label': 'Odometry', 'supervised': False},
    'joy': {'topic': '/joy', 'label': 'FlySky RC (/joy)', 'supervised': False},
    'lidar': {'topic': '/scan', 'label': 'Lakibeam lidar', 'supervised': True},
    'camera': {'topic': '/camera', 'label': 'USB camera', 'supervised': True},
}

RATE_TOPICS = [
    '/motor',
    '/mux_out',
    '/imu',
    '/odom',
    '/joy',
    '/scan',
    '/camera',
]

log = logging.getLogger('dashboard')

# ---------------------------------------------------------------------------
# Status collection (cached, background-refreshed)
# ---------------------------------------------------------------------------

_status_lock = threading.Lock()
_latest_status: dict = {
    'timestamp': '',
    'nodes': {},
    'node_list': [],
    'topic_list': [],
    'rates': {},
    'system_health': {},
    'watchdog_log': [],
    'log_dir': str(Path.home() / 'logs' / 'latest'),
}
_monitor_running = True

# Refresh slow-changing system diagnostics (RTC, under-voltage) every minute,
# not every monitor tick.
SYSTEM_HEALTH_REFRESH_SEC = 60.0


# ---------------------------------------------------------------------------
# Rate measurement via rclpy subscriptions (replaces `ros2 topic hz` subprocs)
# ---------------------------------------------------------------------------


class _RateSampler(Node):
    """Holds one BEST_EFFORT subscription per RATE_TOPICS entry and tracks arrivals."""

    def __init__(self, topics, window_sec: float = RATE_WINDOW_SEC):
        super().__init__('racecar_dashboard')
        self._window = window_sec
        self._stamps: dict = {t: deque() for t in topics}
        self._lock = threading.Lock()
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        # Subscribe with a generic message type via the type name lookup at
        # spin time. Workaround: rclpy needs a concrete msg type, so peek at
        # the topic list once after spin starts. See _attach_subscriptions.
        self._qos = qos
        self._topics = list(topics)
        self._subs: dict = {}

    def _record(self, topic: str):
        now = time.monotonic()
        with self._lock:
            dq = self._stamps[topic]
            dq.append(now)
            cutoff = now - self._window
            while dq and dq[0] < cutoff:
                dq.popleft()

    def attach_subscriptions(self):
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
                msg_cls, topic, lambda _msg, t=topic: self._record(t), self._qos,
            )

    def measure_hz(self, topic: str):
        """Return arrival rate (Hz) over the window, or None if not subscribed/no data."""
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

    def topic_list(self):
        return sorted(self.get_topic_names_and_types(), key=lambda x: x[0])

    def node_list(self):
        return sorted(f'/{n}' if not n.startswith('/') else n for n in self.get_node_names())


_sampler: _RateSampler = None
_sampler_lock = threading.Lock()


def _measure_hz(topic: str):
    """Return arrival rate (Hz) for a topic from the rclpy sampler, or None."""
    sampler = _sampler
    if sampler is None:
        return None
    return sampler.measure_hz(topic)


def _get_topic_list():
    sampler = _sampler
    if sampler is None:
        return []
    return [name for name, _types in sampler.topic_list()]


def _get_node_list():
    sampler = _sampler
    if sampler is None:
        return []
    return sampler.node_list()


def _read_watchdog_tail(n: int = 10, max_bytes: int = 4096):
    """Return the last n lines of the watchdog log without reading the whole file."""
    logfile = Path.home() / 'logs' / 'latest' / 'watchdog.log'
    try:
        with open(logfile, 'rb') as f:
            f.seek(0, 2)
            size = f.tell()
            f.seek(max(0, size - max_bytes))
            tail = f.read()
    except OSError:
        return []
    return tail.decode(errors='replace').splitlines()[-n:]


# Jetson Orin Nano thermal thresholds (C). The SoC soft-throttles around 95 C,
# so 'warm' and 'hot' bands give early warning well before throttling.
TEMP_OK_C = 70.0
TEMP_HIGH_C = 85.0


def _read_max_temp_c():
    """Highest Jetson thermal-zone temperature in C, or None on failure."""
    hottest = None
    for tz in Path('/sys/class/thermal').glob('thermal_zone*'):
        # The cv*-thermal zones on a powered-down CV engine return EAGAIN,
        # which Python's buffered reader surfaces as TypeError (raw read
        # yields None), not OSError.
        try:
            celsius = int((tz / 'temp').read_text().strip()) / 1000.0
        except (OSError, ValueError, TypeError):
            continue
        if celsius <= 0:
            continue
        if hottest is None or celsius > hottest:
            hottest = celsius
    return hottest


def _classify_temp(celsius):
    """Map the hottest thermal zone to (status, label) for dashboard rendering."""
    if celsius is None:
        return ('dead', 'NO READING')
    if celsius < TEMP_OK_C:
        return ('healthy', f'{celsius:.1f} C')
    if celsius < TEMP_HIGH_C:
        return ('stale', f'{celsius:.1f} C - warm')
    return ('dead', f'{celsius:.1f} C - HOT')


def _collect_system_health():
    """Slow-refresh diagnostics: Jetson thermals."""
    temp = _read_max_temp_c()
    temp_status, temp_label = _classify_temp(temp)
    return {
        'temperature': {
            'label': 'Jetson temp (max zone)',
            'status': temp_status,
            'detail': temp_label,
        },
    }


def _monitor_loop() -> None:
    """Background thread that continuously refreshes the cached status snapshot."""
    global _monitor_running
    # Seed empty and let the first loop pass populate system health inside the
    # try block; an unguarded call here once killed the whole thread on boot.
    last_system_health = 0.0
    system_health = {}
    while _monitor_running:
        try:
            # Late-binding subscription attach: new publishers may show up after
            # dashboard start, so retry the topic->type lookup each tick.
            if _sampler is not None:
                _sampler.attach_subscriptions()

            topics = _get_topic_list()
            nodes = _get_node_list()

            node_status = {}
            for name, cfg in MONITORED.items():
                present = cfg['topic'] in topics
                if present:
                    status = 'healthy'
                elif cfg.get('supervised', True):
                    status = 'dead'
                else:
                    status = 'unsupervised'
                node_status[name] = {
                    'label': cfg['label'],
                    'topic': cfg['topic'],
                    'alive': present,
                    'supervised': cfg.get('supervised', True),
                    'status': status,
                }

            rates = {}
            for topic in RATE_TOPICS:
                hz = _measure_hz(topic)
                rates[topic] = {'hz': hz, 'stale': hz is None or hz < 0.5}

            now = time.monotonic()
            if now - last_system_health >= SYSTEM_HEALTH_REFRESH_SEC:
                system_health = _collect_system_health()
                last_system_health = now

            status = {
                'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
                'nodes': node_status,
                'node_list': nodes,
                'topic_list': topics,
                'rates': rates,
                'system_health': system_health,
                'watchdog_log': _read_watchdog_tail(),
                'log_dir': str(Path.home() / 'logs' / 'latest'),
            }

            with _status_lock:
                _latest_status.update(status)

        except Exception:  # noqa: BLE001
            log.exception('Error in monitor loop')

        for _ in range(30):
            if not _monitor_running:
                break
            time.sleep(0.1)


def get_status() -> dict:
    """Return the most recent status snapshot (non-blocking)."""
    with _status_lock:
        return dict(_latest_status)


# ---------------------------------------------------------------------------
# HTTP handler - HTML lives in scripts/dashboard.html so flake8 stays happy.
# ---------------------------------------------------------------------------

_HTML_PATH = Path(__file__).resolve().parent / 'dashboard.html'


def _load_dashboard_html() -> str:
    """Read the HTML template from disk. Cached at module import time."""
    try:
        return _HTML_PATH.read_text(encoding='utf-8')
    except OSError as exc:
        return f'<!DOCTYPE html><body><pre>dashboard.html unreadable: {exc}</pre>'


DASHBOARD_HTML = _load_dashboard_html()


class DashboardHandler(BaseHTTPRequestHandler):
    """Serve GET / (HTML) and GET /api/status (JSON snapshot)."""

    def do_GET(self):
        if self.path == '/':
            self._serve_html()
        elif self.path == '/api/status':
            self._serve_status()
        else:
            self.send_error(404)

    def _serve_html(self):
        content = DASHBOARD_HTML.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html; charset=utf-8')
        self.send_header('Content-Length', str(len(content)))
        self.end_headers()
        self.wfile.write(content)

    def _serve_status(self):
        data = get_status()
        body = json.dumps(data).encode()
        self.send_response(200)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Content-Length', str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        """Suppress default per-request logging."""
        pass


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> None:
    global _monitor_running, _sampler

    logdir = Path.home() / 'logs' / 'latest'
    handlers = [logging.StreamHandler(sys.stderr)]
    try:
        if logdir.exists():
            fh = logging.FileHandler(logdir / 'dashboard.log')
            handlers.append(fh)
    except OSError:
        pass

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
        handlers=handlers,
    )

    rclpy.init()
    with _sampler_lock:
        _sampler = _RateSampler(RATE_TOPICS)

    def _spin_sampler():
        try:
            rclpy.spin(_sampler)
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

    server = HTTPServer(('0.0.0.0', PORT), DashboardHandler)
    log.info('Dashboard listening on http://0.0.0.0:%d', PORT)

    def _shutdown(signum, _frame):
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
        try:
            _sampler.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        rclpy.try_shutdown()
        log.info('Dashboard stopped')


if __name__ == '__main__':
    main()
