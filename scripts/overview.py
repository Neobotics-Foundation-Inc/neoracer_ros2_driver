"""
Live sensor feeds for the 8080 server's System Health page, and its history.

Subscriptions open on the first request and close IDLE_SEC after the last, so
the server costs nothing extra while nobody watches. The camera feed, a 60 Hz
stream, opens only while the page asks for frames and closes CAMERA_IDLE_SEC
after the last one. Callbacks keep only the latest serialized message;
decoding happens when the page asks, at the page's rate rather than the
sensor's.

The NeoRacer's camera node publishes the camera's own MJPG frames on
/camera/color (encoding 'jpeg'), so a frame goes to the page as it arrived,
with no decode and no re-encode. The car has no depth camera, so upstream's
depth preview is not here.

History keeps the battery voltage, the junction temperature and the memory in
use for one battery's run, for the page's history charts.
"""

from __future__ import annotations

from collections import deque
from collections.abc import Callable
import math
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np

IDLE_SEC = 10.0
CAMERA_IDLE_SEC = 3.0
# A camera frame older than this is not served, so a stopped camera shows No data.
FRAME_STALE_SEC = 1.0
CAMERAS = ('color',)
GRAPH_SEC = 30.0
GRAPH_HZ = 10.0
# A raw (not JPEG) frame is scaled to this width and encoded for the page.
PREVIEW_WIDTH = 320
JPEG_QUALITY = 60
SCAN_MAX_M = 8.0
SCAN_STRIDE = 3
# The history charts: one battery's run, in 10 s means.
BATTERY_WINDOW_SEC = 2 * 3600
BATTERY_BUCKET_SEC = 10

# The car's outline in frames.yaml, drawn under the scan.
FOOTPRINT_KEYS = {
    'front': 'footprint_front_x_mm',
    'rear': 'footprint_rear_x_mm',
    'left': 'footprint_left_y_mm',
    'right': 'footprint_right_y_mm',
}

# name: (topic, message type path)
FEEDS = {
    'color': ('/camera/color', 'sensor_msgs/msg/Image'),
    'scan': ('/scan', 'sensor_msgs/msg/LaserScan'),
    'imu': ('/imu/fused', 'sensor_msgs/msg/Imu'),
    'encoder': ('/encoder/speed', 'std_msgs/msg/Float32'),
}


def _msg_class(path: str) -> Any:
    pkg, _, name = path.split('/')
    return getattr(__import__(f'{pkg}.msg', fromlist=[name]), name)


def image_array(msg: Any) -> np.ndarray | None:
    """Return a raw sensor_msgs/Image as an HxWx3 BGR or HxW mono array, or None."""
    h, w = msg.height, msg.width
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if msg.encoding in ('rgb8', 'bgr8'):
        img = buf.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
        return img[:, :, ::-1] if msg.encoding == 'rgb8' else img
    if msg.encoding == 'mono8':
        return buf.reshape(h, msg.step)[:, :w]
    return None


def scan_points(
    ranges: list[float], angle_min: float, angle_inc: float, mount: tuple[float, float, float]
) -> list[list[float]]:
    """Return lidar returns as [x, y] in base_link (x forward, y left), thinned."""
    lx, ly, yaw = mount
    out = []
    for i in range(0, len(ranges), SCAN_STRIDE):
        r = ranges[i]
        if not math.isfinite(r) or r <= 0.0 or r > SCAN_MAX_M:
            continue
        a = angle_min + i * angle_inc + yaw
        out.append([round(lx + r * math.cos(a), 2), round(ly + r * math.sin(a), 2)])
    return out


def _frames(config_dir: Path | None) -> dict[str, float] | None:
    """Return frames.yaml with the car profile's override, or None when it cannot be read."""
    try:
        from neoracer_ros2_driver.frames import load_frames

        if config_dir is None:
            from ament_index_python.packages import get_package_share_directory

            config_dir = Path(get_package_share_directory('neoracer_ros2_driver')) / 'config'
        return load_frames(config_dir)
    except Exception:  # noqa: BLE001  the view still draws, unrotated
        return None


def laser_mount_from_config(config_dir: Path | None) -> tuple[float, float, float]:
    """Return the lidar's (x, y, yaw) in base_link from frames.yaml, or the origin."""
    f = _frames(config_dir)
    if f is None:
        return (0.0, 0.0, 0.0)
    return (
        f.get('laser_x_mm', 0.0) / 1000.0,
        f.get('laser_y_mm', 0.0) / 1000.0,
        math.radians(f.get('laser_yaw_deg', 0.0)),
    )


def footprint_from_config(config_dir: Path | None) -> dict[str, float] | None:
    """Return the car's outline in base_link (front, rear, left, right, in m), or None."""
    f = _frames(config_dir)
    if f is None or any(k not in f for k in FOOTPRINT_KEYS.values()):
        return None
    return {side: round(f[k] / 1000.0, 3) for side, k in FOOTPRINT_KEYS.items()}


class History:
    """Timed samples of a few values over a sliding window (wall-clock seconds)."""

    def __init__(self, window_sec: float = BATTERY_WINDOW_SEC) -> None:
        self.window = window_sec
        self._samples: deque[tuple[float, ...]] = deque()
        self._lock = threading.Lock()

    def add(self, t: float, *values: float) -> None:
        # JSON has no NaN; a sample the page could not read is dropped here.
        if not all(math.isfinite(v) for v in values):
            return
        with self._lock:
            self._samples.append((t, *values))
            while self._samples and self._samples[0][0] < t - self.window:
                self._samples.popleft()

    def latest(self, now: float) -> tuple[tuple[float, ...], float] | None:
        """Return the last values and their age, or None before the first sample."""
        with self._lock:
            if not self._samples:
                return None
            last = self._samples[-1]
        return last[1:], now - last[0]

    def buckets(self, now: float, bucket_sec: float = BATTERY_BUCKET_SEC) -> list[list[float]]:
        """Return [time, mean of each value] per bucket within the window, oldest first."""
        with self._lock:
            samples = [s for s in self._samples if s[0] >= now - self.window]
        acc: dict[int, list[float]] = {}
        for t, *values in samples:
            a = acc.setdefault(int(t // bucket_sec), [0.0] * (len(values) + 1))
            for i, v in enumerate(values):
                a[i] += v
            a[-1] += 1
        return [
            [(k + 0.5) * bucket_sec, *[round(s / acc[k][-1], 3) for s in acc[k][:-1]]]
            for k in sorted(acc)
        ]


def sample_system(
    history: History, now: float, temp_c: float | None, memory: dict[str, int] | None
) -> None:
    """Record the junction temperature (C) and memory in use (%) for the history charts."""
    if temp_c is None or not memory or not memory.get('total'):
        return
    used = 100.0 * (1 - memory['available'] / memory['total'])
    history.add(now, round(temp_c, 1), round(used, 1))


def history_state(battery: History, system: History, now: float) -> dict[str, Any]:
    """Return the /api/history reply: battery volts, then temperature and memory, per bucket."""
    return {
        'now': now,
        'window_sec': battery.window,
        'battery': battery.buckets(now),
        'system': system.buckets(now),
    }


class Overview:
    """On-demand feeds: the latest frame, a thinned scan, and 30 s IMU and speed graphs."""

    def __init__(
        self,
        node: Any,
        deserialize: Callable[[bytes, Any], Any] | None = None,
        config_dir: Path | None = None,
    ) -> None:
        if deserialize is None:
            from rclpy.serialization import deserialize_message

            deserialize = deserialize_message
        self.node = node
        self._deserialize: Callable[[bytes, Any], Any] = deserialize
        self._lock = threading.Lock()
        self._handles: dict[str, Any] = {}
        self.last_frame_request = 0.0
        self._latest: dict[str, tuple[bytes, float]] = {}
        self._jpeg: dict[str, tuple[bytes, float]] = {}
        self.imu: deque[list[float]] = deque(maxlen=int(GRAPH_SEC * GRAPH_HZ))
        self.speed: deque[list[float]] = deque(maxlen=int(GRAPH_SEC * GRAPH_HZ))
        self.active = False
        self.last_request = 0.0
        self.opened = 0.0
        self._mount = laser_mount_from_config(config_dir)
        self._footprint = footprint_from_config(config_dir)
        threading.Thread(target=self._sample, daemon=True).start()

    # -- subscriptions --

    def touch(self) -> None:
        with self._lock:
            self.last_request = time.monotonic()
            if not self.active:
                self._open([n for n in FEEDS if n not in CAMERAS])
                self.active = True
                self.opened = self.last_request

    def touch_cameras(self) -> None:
        with self._lock:
            self.last_request = self.last_frame_request = time.monotonic()
            if not self.active:
                self._open([n for n in FEEDS if n not in CAMERAS])
                self.active = True
                self.opened = self.last_request
            self._open([n for n in CAMERAS if n in FEEDS and n not in self._handles])

    def _open(self, names: list[str]) -> None:
        from rclpy.qos import qos_profile_sensor_data

        for name in names:
            topic, type_path = FEEDS[name]
            self._handles[name] = self.node.create_subscription(
                _msg_class(type_path),
                topic,
                self._store(name),
                qos_profile_sensor_data,
                raw=True,
            )

    def _drop(self, names: list[str]) -> None:
        for name in names:
            h = self._handles.pop(name, None)
            if h is not None:
                self.node.destroy_subscription(h)
            self._latest.pop(name, None)
            self._jpeg.pop(name, None)

    def _close(self) -> None:
        self._drop(list(self._handles))
        self._latest.clear()
        self._jpeg.clear()
        self.imu.clear()
        self.speed.clear()
        self.active = False

    def _store(self, name: str) -> Callable[[bytes], None]:
        def cb(data: bytes) -> None:
            self._latest[name] = (data, time.monotonic())

        return cb

    def _decode(self, name: str) -> tuple[Any, float] | None:
        item = self._latest.get(name)
        if item is None:
            return None
        return self._deserialize(item[0], _msg_class(FEEDS[name][1])), item[1]

    # -- graph sampling --

    def _sample(self) -> None:
        while True:
            time.sleep(1.0 / GRAPH_HZ)
            self.sample_once()

    def sample_once(self) -> None:
        """Close idle feeds, then add one IMU and one speed sample to the graphs."""
        with self._lock:
            if self.active and time.monotonic() - self.last_request > IDLE_SEC:
                self._close()
            elif time.monotonic() - self.last_frame_request > CAMERA_IDLE_SEC:
                self._drop([n for n in CAMERAS if n in self._handles])
            if not self.active:
                return
            now = time.time()
            imu = self._decode('imu')
            if imu is not None:
                g, a = imu[0].angular_velocity, imu[0].linear_acceleration
                row = [g.x, g.y, g.z, a.x, a.y, a.z]
                # JSON has no NaN, and one bad row would break /api/overview.
                if all(math.isfinite(v) for v in row):
                    rates = [round(math.degrees(v), 1) for v in row[:3]]
                    accel = [round(v, 2) for v in row[3:]]
                    self.imu.append([round(now, 2), *rates, *accel])
            enc = self._decode('encoder')
            if enc is not None and math.isfinite(float(enc[0].data)):
                self.speed.append([round(now, 2), round(float(enc[0].data), 3)])

    # -- page data --

    def state(self) -> dict[str, Any]:
        """
        Return scan points, graph histories, the car's outline, and the age of each feed.

        imu rows are [time, roll, pitch and yaw rate in deg/s, x, y and z
        acceleration in m/s^2]; speed rows are [time, m/s]. open_s is how long
        the feeds have been open, so the page can tell a feed that has not
        arrived yet from one that is missing.
        """
        with self._lock:
            now = time.monotonic()
            ages = {k: round(now - v[1], 1) for k, v in self._latest.items()}
            scan = self._decode('scan')
            points = (
                scan_points(
                    list(scan[0].ranges), scan[0].angle_min, scan[0].angle_increment, self._mount
                )
                if scan
                else []
            )
            return {
                'now': time.time(),
                'ages': ages,
                'open_s': round(now - self.opened, 1) if self.active else 0.0,
                'scan': points,
                'footprint': self._footprint,
                'imu': list(self.imu),
                'speed': list(self.speed),
                'graph_sec': GRAPH_SEC,
            }

    def jpeg(self, name: str, max_age: float = 0.25) -> bytes | None:
        """
        Return a JPEG of the latest frame from camera `name`; cached for max_age s.

        A JPEG frame, which is what the NeoRacer's camera node publishes, is
        returned as it arrived. A raw frame (rgb8, bgr8, mono8) is scaled to
        PREVIEW_WIDTH and encoded, as upstream does.
        """
        with self._lock:
            cached = self._jpeg.get(name)
            if cached and time.monotonic() - cached[1] < max_age:
                return cached[0]
            decoded = self._decode(name)
        if decoded is None or time.monotonic() - decoded[1] > FRAME_STALE_SEC:
            return None
        msg = decoded[0]
        if msg.encoding == 'jpeg':
            data = bytes(msg.data)
            if data[:2] != b'\xff\xd8':
                return None
        else:
            img = image_array(msg)
            if img is None:
                return None
            import cv2

            scale = PREVIEW_WIDTH / img.shape[1]
            small = cv2.resize(
                img,
                (PREVIEW_WIDTH, max(1, int(img.shape[0] * scale))),
                interpolation=cv2.INTER_AREA,
            )
            data = cv2.imencode('.jpg', small, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])[
                1
            ].tobytes()
        with self._lock:
            self._jpeg[name] = (data, time.monotonic())
        return data
