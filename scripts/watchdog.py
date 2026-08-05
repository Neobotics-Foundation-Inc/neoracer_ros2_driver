#!/usr/bin/env python3
"""Neoracer node watchdog: monitor + restart the control pipeline and sensors."""

from datetime import datetime
import logging
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

POLL_INTERVAL = 5            # seconds between health checks
RESTART_COOLDOWN = 30        # minimum seconds between restarts of the same node
STARTUP_GRACE = 15           # seconds to wait before first check (systemd matches)
SHM_CLEANUP_INTERVAL = 60    # seconds between FastRTPS shm orphan sweeps
PGREP_FAIL_THRESHOLD = 5     # consecutive pgrep failures before assuming "not running"

PACKAGE = 'neoracer_ros2_driver'

# Executable-path substrings used by process_check. Specific enough to
# distinguish the node binary from `ros2 launch` wrappers in pgrep -f.
DRIVER_LIB = '/install/neoracer_ros2_driver/lib/neoracer_ros2_driver'
LAKIBEAM_LIB = '/install/lakibeam1/lib/lakibeam1/lakibeam1_scan_node'


def _is_running(path_substring: str):
    """
    Return a process_check callable that pgreps for the given path substring.

    Consecutive pgrep exceptions return True (conservative - don't restart blindly),
    but after PGREP_FAIL_THRESHOLD failures we flip to pessimistic False so a
    broken pgrep doesn't mask a real outage forever.
    """
    state = {'fails': 0}

    def check() -> bool:
        try:
            r = subprocess.run(
                ['pgrep', '-f', path_substring],
                capture_output=True, timeout=3,
            )
            state['fails'] = 0
            return r.returncode == 0
        except (subprocess.TimeoutExpired, OSError) as exc:
            state['fails'] += 1
            if state['fails'] >= PGREP_FAIL_THRESHOLD:
                log.error('pgrep(%s) failed %d times in a row: %s - treating as down',
                          path_substring, state['fails'], exc)
                return False
            log.warning('pgrep(%s) failed (%d/%d): %s',
                        path_substring, state['fails'], PGREP_FAIL_THRESHOLD, exc)
            return True
    return check


NODES = {
    # ----- Control pipeline (safety-critical) -----
    'controller': {
        # ESP32 bridge: publishes /imu/fused (~170 Hz), /odom, /joy; drives /motor.
        # Monitoring /imu/fused freshness catches a stalled or unplugged serial link.
        'topic': '/imu/fused',
        'launch': 'controller.launch.py',
        'device_check': lambda: os.path.exists('/dev/osrbot_base'),
        'device_label': '/dev/osrbot_base (ESP32)',
        'kill_pattern': f'{DRIVER_LIB}/controller',
        'process_check': _is_running(f'{DRIVER_LIB}/controller'),
        'freshness_sec': 2.0,
    },
    'throttle': {
        'topic': '/motor',  # downstream of throttle, alive iff throttle alive
        'launch': 'throttle.launch.py',
        'device_check': lambda: True,  # software node
        'device_label': 'throttle_node (software)',
        'kill_pattern': f'{DRIVER_LIB}/throttle_node',
        'process_check': _is_running(f'{DRIVER_LIB}/throttle_node'),
    },
    'mux': {
        'topic': '/mux_out',
        'launch': 'mux.launch.py',
        'device_check': lambda: True,
        'device_label': 'mux_node (software)',
        'kill_pattern': f'{DRIVER_LIB}/mux_node',
        'process_check': _is_running(f'{DRIVER_LIB}/mux_node'),
    },
    'gamepad': {
        'topic': '/gamepad_drive',
        'launch': 'gamepad.launch.py',
        'device_check': lambda: True,  # software node (/joy comes from controller)
        'device_label': 'gamepad_node (software)',
        'kill_pattern': f'{DRIVER_LIB}/gamepad_node',
        'process_check': _is_running(f'{DRIVER_LIB}/gamepad_node'),
    },

    # ----- Sensors -----
    'lidar': {
        'topic': '/scan',
        'launch': 'lidar.launch.py',
        'device_check': lambda: True,  # network sensor (UDP); no /dev node to check
        'device_label': 'Lakibeam (UDP 192.168.8.2)',
        'kill_pattern': LAKIBEAM_LIB,
        'process_check': _is_running(LAKIBEAM_LIB),
        # The lidar can silently desync (UDP frames dropped) with the process +
        # advertisement both alive. 5s is well above the 30 Hz scan period and
        # well below the restart cooldown.
        'freshness_sec': 5.0,
    },
    'camera': {
        'topic': '/camera/color',
        'launch': 'camera.launch.py',
        'device_check': lambda: os.path.exists('/dev/osrbot_usb_cam'),
        'device_label': '/dev/osrbot_usb_cam (USB camera)',
        'kill_pattern': f'{DRIVER_LIB}/camera',
        'process_check': _is_running(f'{DRIVER_LIB}/camera'),
    },
    # led_matrix (display) is intentionally out of scope, like the original
    # dotmatrix node: it only subscribes, and a USB-serial re-init on restart is
    # riskier than the non-critical display it would recover.
}


# ---------------------------------------------------------------------------
# Globals
# ---------------------------------------------------------------------------

_running = True
_child_procs: dict = {}
_last_restart: dict = {}

log = logging.getLogger('watchdog')


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _clean_fastrtps_orphans() -> int:
    """Remove 0-byte /dev/shm/fastrtps_port* segments and stranded lock files."""
    shm = Path('/dev/shm')
    removed = 0
    for port in shm.glob('fastrtps_port*'):
        if port.name.endswith('_el'):
            continue
        try:
            if port.stat().st_size == 0:
                (shm / f'{port.name}_el').unlink(missing_ok=True)
                (shm / f'sem.{port.name}_mutex').unlink(missing_ok=True)
                port.unlink(missing_ok=True)
                removed += 1
        except (OSError, FileNotFoundError):
            pass
    for el in shm.glob('fastrtps_port*_el'):
        data = shm / el.name[:-len('_el')]
        if not data.exists():
            try:
                (shm / f'sem.{data.name}_mutex').unlink(missing_ok=True)
                el.unlink(missing_ok=True)
                removed += 1
            except (OSError, FileNotFoundError):
                pass
    return removed


class _FreshnessMonitor:
    """
    Track last-arrival monotonic time for topics that need a freshness check.

    Subscriptions are BEST_EFFORT (publishers like the lidar use BEST_EFFORT too)
    and re-attached every poll so a topic that comes up after a restart wires in.
    """

    _QOS = QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )

    def __init__(self, node: Node, topics):
        self._node = node
        self._topics = list(topics)
        self._lock = threading.Lock()
        self._last: dict = {}
        self._subs: dict = {}

    def attach(self):
        names_types = dict(self._node.get_topic_names_and_types())
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
                log.debug('freshness: cannot subscribe to %s: %s', topic, exc)
                continue
            self._subs[topic] = self._node.create_subscription(
                msg_cls, topic, lambda _msg, t=topic: self._mark(t), self._QOS,
            )

    def _mark(self, topic: str):
        with self._lock:
            self._last[topic] = time.monotonic()

    def reset(self, topic: str):
        """Forget the last-seen time for a topic (call after a restart kick)."""
        with self._lock:
            self._last.pop(topic, None)
        sub = self._subs.pop(topic, None)
        if sub is not None:
            try:
                self._node.destroy_subscription(sub)
            except Exception:  # noqa: BLE001
                pass

    def age(self, topic: str):
        """Seconds since last message on topic, or None if never seen."""
        with self._lock:
            ts = self._last.get(topic)
        if ts is None:
            return None
        return time.monotonic() - ts


def _get_active_topics(node=None) -> set:
    """
    Return the set of currently advertised ROS2 topics.

    Prefers `node.get_topic_names_and_types()` (fast, in-process). Falls back to
    `ros2 topic list` subprocess only if no node is given - kept so module-level
    helpers stay importable without rclpy.init.
    """
    if node is not None:
        try:
            return {name for name, _types in node.get_topic_names_and_types()}
        except Exception as exc:  # noqa: BLE001
            log.warning('node.get_topic_names_and_types failed: %s', exc)
            return set()
    try:
        result = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True, text=True, timeout=10,
        )
        if result.returncode == 0:
            return set(result.stdout.strip().splitlines())
    except (subprocess.TimeoutExpired, FileNotFoundError):
        log.warning('Failed to query ros2 topic list')
    return set()


def _log_dir() -> Path:
    """Resolve ~/logs/latest to the real session directory."""
    latest = Path.home() / 'logs' / 'latest'
    if latest.is_symlink() or latest.is_dir():
        return latest.resolve()
    fallback = Path.home() / 'logs'
    fallback.mkdir(parents=True, exist_ok=True)
    return fallback


def _restart_node(name: str, cfg: dict) -> None:
    """Launch an individual node's launch file as a subprocess."""
    now = time.time()
    last = _last_restart.get(name, 0)
    if now - last < RESTART_COOLDOWN:
        remaining = int(RESTART_COOLDOWN - (now - last))
        log.info('%s: cooldown active, retry in %ds', name, remaining)
        return

    # Kill any previous child we started for this node.
    old = _child_procs.get(name)
    if old:
        old_proc, old_fh = old
        if old_proc.poll() is None:
            log.info('%s: terminating stale child PID %d', name, old_proc.pid)
            old_proc.terminate()
            try:
                old_proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                old_proc.kill()
        try:
            old_fh.close()
        except OSError:
            pass

    # Kill any stale system-wide processes (e.g. from teleop.launch) that might
    # still hold the device open. SIGTERM first, then SIGKILL after 2 s.
    kill_pat = cfg.get('kill_pattern')
    if kill_pat:
        try:
            r = subprocess.run(
                ['pkill', '-f', kill_pat], capture_output=True, timeout=5,
            )
            if r.returncode == 0:
                log.info('%s: sent SIGTERM to processes matching "%s"',
                         name, kill_pat)
                time.sleep(2)
                r2 = subprocess.run(
                    ['pkill', '-9', '-f', kill_pat],
                    capture_output=True, timeout=5,
                )
                if r2.returncode == 0:
                    log.info('%s: sent SIGKILL to surviving processes', name)
                time.sleep(1)
        except subprocess.TimeoutExpired:
            pass

    delay = cfg.get('restart_delay', 0)
    if delay > 0:
        log.info('%s: waiting %ds before restart (USB settle)', name, delay)
        for _ in range(delay * 10):
            if not _running:
                return
            time.sleep(0.1)

    ts = datetime.now().strftime('%H%M%S')
    restart_log = _log_dir() / f'restart_{name}_{ts}.log'
    log.info('%s: restarting via %s - log: %s', name, cfg['launch'], restart_log)

    log_fh = open(restart_log, 'w')  # noqa: SIM115
    env = os.environ.copy()
    env['ROS_LOG_DIR'] = str(_log_dir())

    proc = subprocess.Popen(
        ['ros2', 'launch', PACKAGE, cfg['launch']],
        stdout=log_fh,
        stderr=subprocess.STDOUT,
        env=env,
    )
    _child_procs[name] = (proc, log_fh)
    _last_restart[name] = now
    log.info('%s: launched PID %d', name, proc.pid)


def _cleanup_children() -> None:
    """Terminate all child processes we spawned and close their log handles."""
    for name, (proc, fh) in _child_procs.items():
        if proc.poll() is None:
            log.info('Stopping child %s (PID %d)', name, proc.pid)
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        try:
            fh.close()
        except OSError:
            pass


def _signal_handler(signum, _frame):
    global _running
    log.info('Received signal %d, shutting down', signum)
    _running = False


# ---------------------------------------------------------------------------
# Main loop
# ---------------------------------------------------------------------------

def main() -> None:
    global _running

    # Set up logging to both file and stderr (journald).
    logdir = _log_dir()
    handlers = [logging.StreamHandler(sys.stderr)]
    try:
        fh = logging.FileHandler(logdir / 'watchdog.log')
        handlers.append(fh)
    except OSError as exc:
        print(f'Warning: cannot open watchdog.log: {exc}', file=sys.stderr)

    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S',
        handlers=handlers,
    )

    signal.signal(signal.SIGTERM, _signal_handler)
    signal.signal(signal.SIGINT, _signal_handler)

    log.info('Watchdog started - monitoring: %s', ', '.join(NODES.keys()))
    log.info('Log directory: %s', logdir)

    rclpy.init()
    intro_node = Node('racecar_watchdog')

    fresh_topics = [cfg['topic'] for cfg in NODES.values() if 'freshness_sec' in cfg]
    freshness = _FreshnessMonitor(intro_node, fresh_topics)

    def _spin():
        try:
            rclpy.spin(intro_node)
        except (KeyboardInterrupt, SystemExit):
            pass
        except Exception:  # noqa: BLE001
            log.exception('rclpy.spin terminated')

    spinner = threading.Thread(target=_spin, daemon=True)
    spinner.start()

    startup_removed = _clean_fastrtps_orphans()
    if startup_removed:
        log.info('Cleaned %d FastRTPS SHM orphan(s) at startup', startup_removed)

    last_shm_cleanup = time.monotonic()
    while _running:
        topics = _get_active_topics(intro_node)
        freshness.attach()

        if time.monotonic() - last_shm_cleanup >= SHM_CLEANUP_INTERVAL:
            last_shm_cleanup = time.monotonic()
            n = _clean_fastrtps_orphans()
            if n:
                log.info('Cleaned %d FastRTPS SHM orphan(s)', n)

        for name, cfg in NODES.items():
            topic = cfg['topic']
            topic_alive = topic in topics
            proc_check = cfg.get('process_check')
            proc_alive = proc_check() if proc_check is not None else True

            # Freshness only applies once the node has been up long enough to
            # publish - skip while topic isn't advertised or during cooldown
            # after a restart, so we don't false-positive on a still-warming-up node.
            fresh_window = cfg.get('freshness_sec')
            topic_stale = False
            if fresh_window and topic_alive and proc_alive:
                last_restart = _last_restart.get(name, 0)
                if time.time() - last_restart >= fresh_window:
                    age = freshness.age(topic)
                    if age is not None and age > fresh_window:
                        topic_stale = True

            alive = topic_alive and proc_alive and not topic_stale
            failure = (
                'topic+process down' if not topic_alive and not proc_alive else
                'topic not advertised' if not topic_alive else
                'process not running' if not proc_alive else
                f'topic stale ({freshness.age(topic):.1f}s)' if topic_stale else
                None
            )

            child = _child_procs.get(name)
            if child:
                child_proc, child_fh = child
                if child_proc.poll() is not None:
                    log.warning('%s: restarted child PID %d exited with code %s',
                                name, child_proc.pid, child_proc.returncode)
                    try:
                        child_fh.close()
                    except OSError:
                        pass
                    _child_procs.pop(name, None)

            if alive:
                continue

            device_ok = cfg['device_check']()
            if not device_ok:
                log.warning('%s: %s - device %s NOT connected, skipping restart',
                            name, failure, cfg['device_label'])
                continue

            log.warning('%s: %s - device %s connected, attempting restart',
                        name, failure, cfg['device_label'])
            _restart_node(name, cfg)
            # Drop the stale subscription so it re-binds to the new publisher.
            if cfg.get('freshness_sec'):
                freshness.reset(topic)

        # Sleep in short increments so we respond to signals promptly.
        for _ in range(POLL_INTERVAL * 10):
            if not _running:
                break
            time.sleep(0.1)

    _cleanup_children()
    try:
        intro_node.destroy_node()
    except Exception:  # noqa: BLE001
        pass
    rclpy.try_shutdown()
    log.info('Watchdog stopped')


if __name__ == '__main__':
    main()
