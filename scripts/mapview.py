#!/usr/bin/env python3
"""
Web map viewer on port 8088 for a phone or laptop browser.

The live map with the car on it, and the controls of `racecar map`. The page
never drives the car.

ROS subscriptions open on the first browser request and close IDLE_SEC after
the last one, so with no browser connected the viewer costs no ROS traffic.
The map list, thumbnails, and saved maps are read from the map folder without
ROS. Start, save, stop, load, and keep run `racecar map`
(scripts/neoracer_map.py) as a child process, one at a time, and the page shows
its output. Set pose publishes /initialpose with the spread `racecar map load`
uses; Find me calls /autonomy/global_localization.

Run by scripts/neoracer-mapview.service. The page is scripts/mapview.html.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
import gzip
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
from typing import Any
from urllib.parse import parse_qs, urlparse

import map_control as mc
from neoracer_ros2_driver import autonomy as au
from neoracer_ros2_driver.maps import map_root, valid_name
import numpy as np
import sysinfo
import webui
import yaml

# 8084 is the eps lab dashboard on the NeoRacer; the lab dashboards use 8081 to 8087.
PORT = 8088
HERE = Path(__file__).resolve().parent
PAGE = HERE / 'mapview.html'
RACECAR_MAP = HERE / 'neoracer_map.py'
IDLE_SEC = 10.0
SCAN_PERIOD_SEC = 0.2
SCAN_POINTS = 360
# The scan returns the localizers use (config/amcl.yaml, laser_min_range and
# laser_max_range): inside 0.12 m is the car's own outline, and 15 m is the
# LakiBeam1's rating at 10% reflectivity.
SCAN_RANGE_M = (0.12, 15.0)
TRAIL_STEP_M = 0.05
TRAIL_MAX = 5000
THUMB_PX = 160
# Unknown, free, and wall cells; the same colors as the page's map canvas.
THUMB_RGB = {'unknown': (0x13, 0x13, 0x17), 'free': (0x4A, 0x4B, 0x53), 'wall': (0xFF, 0xFF, 0xFF)}
JOB_LINES = 40
FIND_TIMEOUT_SEC = 5.0
# Same spread as the start pose of `racecar map load` (config/autonomy.yaml,
# start_spread_xy_m and start_spread_yaw_deg).
SET_POSE_SD = (0.25, 10.0)
# The junction temperature the System Health page and `racecar status` call hot
# (scripts/diagnose.py, SOC_FAIL_C).
HOT_C = 85.0
ACTIONS = ('start', 'save', 'stop', 'load', 'recover')


# -- maps from disk --


def occupancy_from_pixels(pixels: np.ndarray, meta: dict[str, Any]) -> np.ndarray:
    """Return OccupancyGrid values (100 wall, 0 free, -1 unknown), rows from the top edge."""
    p = pixels / 255.0 if meta.get('negate', 0) else (255.0 - pixels) / 255.0
    out = np.full(pixels.shape, -1, dtype=np.int8)
    out[p > float(meta.get('occupied_thresh', 0.65))] = 100
    out[p < float(meta.get('free_thresh', 0.196))] = 0
    return out


def read_saved(stem: Path) -> tuple[dict[str, Any], np.ndarray] | None:
    """Return (map info, occupancy rows from the bottom edge, as /map) for a saved map."""
    from PIL import Image

    meta_path = Path(f'{stem}.yaml')
    try:
        meta = yaml.safe_load(meta_path.read_text()) or {}
        image = meta_path.parent / str(meta.get('image', f'{stem.name}.pgm'))
        pixels = np.asarray(Image.open(image).convert('L'), dtype=float)
    except (OSError, yaml.YAMLError):
        return None
    grid = np.flipud(occupancy_from_pixels(pixels, meta))
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    info = {
        'width': int(grid.shape[1]),
        'height': int(grid.shape[0]),
        'resolution': float(meta.get('resolution', 0.05)),
        'origin': [float(origin[0]), float(origin[1])],
    }
    return info, grid


def saved_stem(name: str) -> Path | None:
    """Return the stem of NAME's saved map, or of its autosave when never saved."""
    state = au.map_state(map_root(), name)
    if state.saved:
        return state.folder / name
    if state.autosaved:
        return state.folder / au.AUTOSAVE_DIR / name
    return None


def thumbnail_png(stem: Path, max_px: int = THUMB_PX) -> bytes | None:
    from PIL import Image

    saved = read_saved(stem)
    if saved is None:
        return None
    _, grid = saved
    rgb = np.empty(grid.shape + (3,), dtype=np.uint8)
    rgb[:] = THUMB_RGB['unknown']
    rgb[grid == 0] = THUMB_RGB['free']
    rgb[grid == 100] = THUMB_RGB['wall']
    img = Image.fromarray(np.flipud(rgb))
    img.thumbnail((max_px, max_px))
    buf = io.BytesIO()
    img.save(buf, format='PNG')
    return buf.getvalue()


# -- live data --


def scan_to_map(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    pose: tuple[float, float, float],
    laser: tuple[float, float, float],
    points: int = SCAN_POINTS,
) -> list[list[float]]:
    """Place up to `points` scan returns in the map from base_link's pose and the laser mount."""
    c, s = math.cos(pose[2]), math.sin(pose[2])
    lx = pose[0] + c * laser[0] - s * laser[1]
    ly = pose[1] + s * laser[0] + c * laser[1]
    x, y = au.scan_points_in_map(
        ranges, angle_min, angle_increment, (lx, ly, pose[2] + laser[2]), *SCAN_RANGE_M, points
    )
    return [[round(float(a), 2), round(float(b), 2)] for a, b in zip(x, y, strict=True)]


def laser_mount(frames: dict[str, float] | None) -> tuple[float, float, float] | None:
    if frames is None:
        return None
    return (
        frames.get('laser_x_mm', 0.0) / 1000.0,
        frames.get('laser_y_mm', 0.0) / 1000.0,
        math.radians(frames.get('laser_yaw_deg', 0.0)),
    )


@dataclass
class Trail:
    """The car's path as points at least step_m apart; `id` changes when it restarts."""

    step_m: float = TRAIL_STEP_M
    limit: int = TRAIL_MAX
    id: int = 0  # noqa: A003  the trail id the page compares
    first: int = 0  # index of points[0] since the restart
    points: list[list[float]] = field(default_factory=list)

    def add(self, x: float, y: float) -> None:
        last = self.points[-1] if self.points else None
        if last is not None and math.hypot(x - last[0], y - last[1]) < self.step_m:
            return
        self.points.append([round(x, 2), round(y, 2)])
        if len(self.points) > self.limit:
            drop = len(self.points) - self.limit
            del self.points[:drop]
            self.first += drop

    def restart(self) -> None:
        self.id += 1
        self.first = 0
        self.points.clear()

    def since(self, index: int, trail_id: int) -> dict[str, Any]:
        """Points from `index` on, or the whole trail for another id or an index aged out."""
        if trail_id != self.id or index < self.first:
            index = self.first
        start = min(max(index - self.first, 0), len(self.points))
        return {'id': self.id, 'from': self.first + start, 'points': self.points[start:]}


# -- racecar map commands --


def action_argv(req: dict[str, Any]) -> list[str]:
    """Return `racecar map` arguments for a page request; ValueError names what is wrong."""
    action = req.get('action')
    if action not in ACTIONS:
        raise ValueError(f'unknown action {action!r}')
    argv = [str(action)]
    if action in ('start', 'load', 'recover'):
        name = str(req.get('name') or '')
        if not valid_name(name):
            raise ValueError('map names use letters, digits, - and _')
        argv.append(name)
    pose = req.get('pose')
    if pose is not None:
        if action not in ('start', 'load'):
            raise ValueError(f'{action} takes no pose')
        x, y, yaw = (float(v) for v in pose)
        argv.append(f'--pose={x:.3f},{y:.3f},{yaw:.4f}')
    if action == 'start' and req.get('source'):
        source = str(req['source'])
        if not valid_name(source):
            raise ValueError('map names use letters, digits, - and _')
        argv += ['--from', source]
    if action == 'start' and pose is not None and not req.get('source'):
        raise ValueError('a start pose needs a map to extend')
    if req.get('replace'):
        # The CLI moves the old map aside rather than deleting it.
        if action != 'start':
            raise ValueError('only start can replace a map')
        if req.get('source') == argv[1]:
            raise ValueError('a map cannot replace itself while extending it')
        argv.append('--force')
    if action == 'load':
        if req.get('search'):
            if pose is not None:
                raise ValueError('choose a pose or a search, not both')
            argv.append('--global')
        localizer = str(req.get('localizer') or 'amcl')
        if localizer not in mc.LOCALIZERS:
            raise ValueError(f'unknown localizer {localizer!r}')
        argv += ['--localizer', localizer]
    return argv


class Job:
    """One `racecar map` child at a time; its output lines and exit code for the page."""

    def __init__(self, program: Sequence[str]) -> None:
        self._program = list(program)
        self._lock = threading.Lock()
        self.argv: list[str] = []
        self.lines: list[str] = []
        self.code: int | None = None
        self.running = False

    def start(self, argv: list[str]) -> bool:
        with self._lock:
            if self.running:
                return False
            self.argv, self.lines, self.code, self.running = argv, [], None, True
        threading.Thread(target=self._run, args=(argv,), daemon=True).start()
        return True

    def _run(self, argv: list[str]) -> None:
        try:
            proc = subprocess.Popen(
                [*self._program, *argv],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                text=True,
            )
        except OSError as exc:
            with self._lock:
                self.lines, self.code, self.running = [str(exc)], 1, False
            return
        assert proc.stdout is not None
        for line in proc.stdout:
            with self._lock:
                self.lines = (self.lines + [line.rstrip()])[-JOB_LINES:]
        code = proc.wait()
        with self._lock:
            self.code, self.running = code, False
            last = self.lines[-1] if self.lines else ''
        print(f'mapview: racecar map {" ".join(argv)}: exit {code}: {last}', flush=True)

    def state(self) -> dict[str, Any]:
        with self._lock:
            return {
                'action': self.argv[0] if self.argv else '',
                'argv': list(self.argv),
                'running': self.running,
                'code': self.code,
                'lines': list(self.lines),
            }


class CpuMeter:
    """Busy share of all cores between calls, from /proc/stat."""

    def __init__(self) -> None:
        self._last = sysinfo.read_cpu_times()

    def read(self) -> float | None:
        now = sysinfo.read_cpu_times()  # (busy, total)
        last, self._last = self._last, now
        return sysinfo.cpu_load_pct(last, now)


class HostReadings:
    """
    The page's host line: CPU busy share, junction temperature, over-current.

    The Jetson's over-current counters only grow until reboot, so a count
    shows past events too. The page reports the events of this run only: the
    counters that rose after the first reading since restart(). The viewer
    restarts it when a browser connects and when a new autonomy session
    begins.
    """

    def __init__(self) -> None:
        self._cpu = CpuMeter()
        self._before: list[str] | None = None

    def restart(self) -> None:
        """Take the next reading as the start of a run."""
        self._before = None

    def read(self) -> dict[str, Any]:
        flags, names = sysinfo.read_throttled()
        before = self._before
        if before is None:
            before = self._before = names
        # Each name carries its counter's event count, so a name that is not
        # in the first reading is an event since then.
        return {
            'throttled': None if flags is None else f'0x{flags:x}',
            'throttled_this_run': [n for n in names if n not in before],
            'temp_c': sysinfo.read_soc_temp(),
            'hot_c': HOT_C,
            'cpu': self._cpu.read(),
        }


# -- ROS --


class Viewer:
    """rclpy node whose subscriptions exist only while a browser is polling."""

    def __init__(self) -> None:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor

        self._rclpy = rclpy
        self.node = rclpy.create_node('mapview')
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self.node)
        self._spinner = threading.Thread(target=self._spin, daemon=True)
        self._spinner.start()
        self._lock = threading.Lock()
        self._handles: list[Any] = []
        self._publisher: Any = None
        self._client: Any = None
        self.active = False
        self.last_request = 0.0
        self.status: dict[str, str] = {}
        self.pose: tuple[float, float, float] | None = None
        self.scan: list[list[float]] = []
        self._scan_at = 0.0
        self.trail = Trail()
        self._session = ''
        self.map_info: dict[str, Any] | None = None
        self.map_gz = b''
        self.map_version = 0
        self.sys: dict[str, Any] = {}
        self._host = HostReadings()
        self._laser = laser_mount(mc.current_frames())
        threading.Thread(target=self._watch, daemon=True).start()

    def _spin(self) -> None:
        try:
            self._executor.spin()
        except Exception:  # noqa: BLE001  shutdown interrupts the spin
            pass

    def touch(self) -> None:
        with self._lock:
            self.last_request = time.monotonic()
            if not self.active:
                self._open()

    def _open(self) -> None:
        from diagnostic_msgs.msg import DiagnosticStatus
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from nav_msgs.msg import OccupancyGrid
        from rclpy.qos import (
            qos_profile_sensor_data,
            QoSDurabilityPolicy,
            QoSProfile,
            QoSReliabilityPolicy,
        )
        from sensor_msgs.msg import LaserScan
        from std_srvs.srv import Trigger

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        n = self.node
        self._handles = [
            n.create_subscription(DiagnosticStatus, '/autonomy/status', self._on_status, latched),
            n.create_subscription(PoseWithCovarianceStamped, '/autonomy/pose', self._on_pose, 10),
            n.create_subscription(OccupancyGrid, '/map', self._on_map, latched),
            n.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data),
        ]
        self._publisher = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._client = n.create_client(Trigger, '/autonomy/global_localization')
        self._host.restart()
        self.active = True

    def _close(self) -> None:
        for h in self._handles:
            self.node.destroy_subscription(h)
        self._handles = []
        if self._publisher is not None:
            self.node.destroy_publisher(self._publisher)
        if self._client is not None:
            self.node.destroy_client(self._client)
        self._publisher = self._client = None
        self.status, self.pose, self.scan = {}, None, []
        self.map_info, self.map_gz = None, b''
        self.active = False

    def _watch(self) -> None:
        while True:
            time.sleep(1.0)
            with self._lock:
                if self.active and time.monotonic() - self.last_request > IDLE_SEC:
                    self._close()
                active = self.active
            if active:
                info = self._host.read()
                with self._lock:
                    self.sys = info

    def _on_status(self, msg: Any) -> None:
        values = mc.status_dict(msg)
        with self._lock:
            self.status = values
            session = values.get('session', '')
            if session != self._session:
                self._session = session
                self.trail.restart()
                self._host.restart()

    def _on_pose(self, msg: Any) -> None:
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        with self._lock:
            self.pose = (p.x, p.y, yaw)
            self.trail.add(p.x, p.y)

    def _on_scan(self, msg: Any) -> None:
        now = time.monotonic()
        if now - self._scan_at < SCAN_PERIOD_SEC or self.pose is None or self._laser is None:
            return
        self._scan_at = now
        points = scan_to_map(
            msg.ranges, msg.angle_min, msg.angle_increment, self.pose, self._laser
        )
        with self._lock:
            self.scan = points

    def _on_map(self, msg: Any) -> None:
        info = msg.info
        data = np.asarray(msg.data, dtype=np.int8).tobytes()
        with self._lock:
            self.map_info = {
                'width': info.width,
                'height': info.height,
                'resolution': info.resolution,
                'origin': [info.origin.position.x, info.origin.position.y],
            }
            self.map_gz = gzip.compress(data, compresslevel=5)
            self.map_version += 1

    def snapshot(self, trail_from: int, trail_id: int) -> dict[str, Any]:
        with self._lock:
            return {
                'status': dict(self.status),
                'pose': None if self.pose is None else [round(v, 3) for v in self.pose],
                'scan': self.scan,
                'trail': self.trail.since(trail_from, trail_id),
                'map_version': self.map_version if self.map_info else 0,
                'sys': dict(self.sys),
            }

    def map_payload(self) -> tuple[dict[str, Any], bytes] | None:
        with self._lock:
            if self.map_info is None:
                return None
            return {**self.map_info, 'version': self.map_version}, self.map_gz

    def set_pose(self, x: float, y: float, yaw: float) -> None:
        from geometry_msgs.msg import PoseWithCovarianceStamped

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = x, y
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance = au.seed_covariance(*SET_POSE_SD)
        with self._lock:
            publisher = self._publisher
        if publisher is None:
            raise RuntimeError('viewer not connected')
        publisher.publish(msg)

    def clear_trail(self) -> None:
        with self._lock:
            self.trail.restart()

    def find_me(self) -> tuple[bool, str]:
        from std_srvs.srv import Trigger

        with self._lock:
            client = self._client
        if client is None or not client.wait_for_service(timeout_sec=1.0):
            return False, 'autonomy is not running'
        done = threading.Event()
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda _: done.set())
        if not done.wait(FIND_TIMEOUT_SEC):
            return False, 'no reply from autonomy_node'
        result = future.result()
        return bool(result.success), str(result.message)

    def shutdown(self) -> None:
        # rclpy shutdown wakes the executor; destroying the node while it waits can block.
        self._rclpy.try_shutdown()
        self._spinner.join(timeout=2.0)


# -- HTTP --


class Handler(BaseHTTPRequestHandler):
    viewer: Viewer
    job: Job

    def log_message(self, fmt: str, *args: Any) -> None:  # noqa: A002
        return

    def _send(
        self, code: int, body: bytes, kind: str, headers: dict[str, str] | None = None
    ) -> None:
        self.send_response(code)
        self.send_header('Content-Type', kind)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        for k, v in (headers or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def _json(self, data: Any, code: int = 200) -> None:
        self._send(code, json.dumps(data).encode(), 'application/json')

    def _body(self) -> dict[str, Any]:
        length = int(self.headers.get('Content-Length') or 0)
        data = json.loads(self.rfile.read(length) or b'{}')
        if not isinstance(data, dict):
            raise ValueError('expected a JSON object')
        return data

    def do_GET(self) -> None:  # noqa: N802
        asset = webui.static_file(self.path)
        if asset is not None:
            self.send_response(200)
            self.send_header('Content-Type', asset[1])
            self.send_header('Content-Length', str(len(asset[0])))
            self.send_header('Cache-Control', webui.CACHE_CONTROL)
            self.end_headers()
            self.wfile.write(asset[0])
            return
        url = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(url.query).items()}
        if url.path in ('/', '/index.html'):
            self._send(200, PAGE.read_bytes(), 'text/html; charset=utf-8')
        elif url.path == '/api/state':
            self.viewer.touch()
            state = self.viewer.snapshot(int(q.get('trail_from', 0)), int(q.get('trail_id', -1)))
            state['job'] = self.job.state()
            self._json(state)
        elif url.path == '/api/map':
            self.viewer.touch()
            payload = self.viewer.map_payload()
            if payload is None:
                self._send(204, b'', 'application/octet-stream')
            else:
                self._grid(*payload)
        elif url.path == '/api/driving':
            self._json({'apps': mc.running_driving_apps()})
        elif url.path == '/api/maps':
            self._json({'root': str(map_root()), 'maps': mc.map_rows(map_root())})
        elif url.path == '/api/saved':
            self._saved(q.get('name', ''))
        elif url.path == '/api/thumb':
            self._thumb(q.get('name', ''))
        else:
            self._send(404, b'not found', 'text/plain')

    def _grid(self, info: dict[str, Any], gz: bytes) -> None:
        self._send(
            200,
            gz,
            'application/octet-stream',
            {'Content-Encoding': 'gzip', 'X-Map-Info': json.dumps(info)},
        )

    def _saved(self, name: str) -> None:
        stem = saved_stem(name) if valid_name(name) else None
        saved = read_saved(stem) if stem is not None else None
        if saved is None:
            self._send(404, b'no such map', 'text/plain')
            return
        info, grid = saved
        self._grid({**info, 'version': 0, 'name': name}, gzip.compress(grid.tobytes(), 5))

    def _thumb(self, name: str) -> None:
        stem = saved_stem(name) if valid_name(name) else None
        png = thumbnail_png(stem) if stem is not None else None
        if png is None:
            self._send(404, b'no such map', 'text/plain')
        else:
            self._send(200, png, 'image/png')

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        try:
            body = self._body()
            if path == '/api/action':
                argv = action_argv(body)
                if not self.job.start(argv):
                    self._json({'ok': False, 'message': 'another command is running'}, 409)
                    return
                self._json({'ok': True, 'argv': argv})
            elif path == '/api/pose':
                self.viewer.touch()
                x, y, yaw = (float(body[k]) for k in ('x', 'y', 'yaw'))
                self.viewer.set_pose(x, y, yaw)
                self._json({'ok': True, 'message': 'pose sent'})
            elif path == '/api/trail/clear':
                self.viewer.clear_trail()
                self._json({'ok': True})
            elif path == '/api/find':
                self.viewer.touch()
                ok, message = self.viewer.find_me()
                self._json({'ok': ok, 'message': message})
            else:
                self._send(404, b'not found', 'text/plain')
        except (ValueError, KeyError, TypeError, RuntimeError) as exc:
            self._json({'ok': False, 'message': str(exc)}, 400)


def main() -> int:
    import rclpy

    rclpy.init()
    Handler.viewer = Viewer()
    Handler.job = Job([sys.executable, str(RACECAR_MAP)])
    server = ThreadingHTTPServer(('', PORT), Handler)
    server.daemon_threads = True

    def stop(*_: Any) -> None:
        threading.Thread(target=server.shutdown, daemon=True).start()

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print(f'mapview: http://0.0.0.0:{PORT}', flush=True)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        Handler.viewer.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
