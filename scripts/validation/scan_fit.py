"""
Scan-to-map fit: how far lidar points land from the map's walls at a pose.

Each scan is placed in the map with map -> odom (from a localizer),
odom -> base_link (the odometry), and base_link -> laser (the static frame),
all taken at the scan's mid-sweep time. Every point's distance to the
nearest occupied cell of a saved map is read from a distance transform. A
well-localized scan in an unchanged room has most points within a cell or two
of a wall; people, moved furniture, and space the map never saw add a tail,
so the median and the share within FIT_INLIER_M are the measures, not the
mean.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any

import numpy as np
import yaml

FIT_INLIER_M = 0.10
MIN_RANGE_M = 0.3  # inside it: the car's own antennas and cable header
MAX_RANGE_M = 12.0
OUTSIDE_M = 1.0  # distance given to a point that falls outside the map
PGM_OCCUPIED = 0

Pose = tuple[float, float, float]  # x, y, yaw (rad)
Track = Sequence[tuple[float, float, float, float]]  # t, x, y, yaw


@dataclass
class GridMap:
    nearest: np.ndarray  # (2, H, W): row and column of each cell's nearest occupied cell
    origin: tuple[float, float]  # map coordinates of the lower-left corner
    resolution: float

    def lookup(self, x: np.ndarray, y: np.ndarray) -> np.ndarray:
        """Distance from each point to the center of the occupied cell nearest its cell."""
        _, h, w = self.nearest.shape
        col = np.floor((x - self.origin[0]) / self.resolution).astype(int)
        row = h - 1 - np.floor((y - self.origin[1]) / self.resolution).astype(int)
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        out = np.full(x.shape, OUTSIDE_M)
        nr = self.nearest[0, row[inside], col[inside]]
        nc = self.nearest[1, row[inside], col[inside]]
        cx = self.origin[0] + (nc + 0.5) * self.resolution
        cy = self.origin[1] + (h - 1 - nr + 0.5) * self.resolution
        out[inside] = np.hypot(x[inside] - cx, y[inside] - cy)
        return out


def grid_from_pixels(pixels: np.ndarray, origin: Sequence[float], resolution: float) -> GridMap:
    """Build the nearest-wall grid from map_saver pixels (row 0 is the top edge)."""
    from scipy.ndimage import distance_transform_edt

    free_of_walls = pixels != PGM_OCCUPIED
    _, nearest = distance_transform_edt(free_of_walls, return_indices=True)
    return GridMap(nearest, (float(origin[0]), float(origin[1])), float(resolution))


def load_grid(stem: Path) -> GridMap:
    from PIL import Image

    meta = yaml.safe_load(Path(str(stem) + '.yaml').read_text())
    pixels = np.asarray(Image.open(stem.parent / meta['image']).convert('L'))
    return grid_from_pixels(pixels, meta['origin'], meta['resolution'])


def compose(a: Pose, b: Pose) -> Pose:
    """Pose b expressed in a's parent frame."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def pose_at(track: np.ndarray, t: float) -> Pose | None:
    """Interpolate an (N, 4) track of t, x, y, yaw (yaw unwrapped) at time t."""
    if len(track) == 0 or t < track[0, 0] or t > track[-1, 0]:
        return None
    return (
        float(np.interp(t, track[:, 0], track[:, 1])),
        float(np.interp(t, track[:, 0], track[:, 2])),
        float(np.interp(t, track[:, 0], track[:, 3])),
    )


def as_track(samples: Track) -> np.ndarray:
    a = np.array(sorted(samples), dtype=float).reshape(-1, 4)
    if len(a):
        a[:, 3] = np.unwrap(a[:, 3])
    return a


def scan_points(ranges: np.ndarray, angle_min: float, increment: float) -> np.ndarray:
    """Return (N, 2) points in the laser frame for ranges inside the usable band."""
    angles = angle_min + increment * np.arange(len(ranges))
    ok = np.isfinite(ranges) & (ranges >= MIN_RANGE_M) & (ranges <= MAX_RANGE_M)
    r, a = ranges[ok], angles[ok]
    return np.column_stack((r * np.cos(a), r * np.sin(a)))


def place(points: np.ndarray, pose: Pose) -> tuple[np.ndarray, np.ndarray]:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    x = pose[0] + c * points[:, 0] - s * points[:, 1]
    y = pose[1] + s * points[:, 0] + c * points[:, 1]
    return x, y


@dataclass
class Fit:
    scans: int
    median_m: float  # over all points
    inlier: float  # share of points within FIT_INLIER_M of a wall
    scan_median_p90_m: float  # 90th percentile of each scan's median: the bad stretches
    worst_t: float  # seconds from the first scan to the worst scan

    def line(self) -> str:
        return (
            f'{self.scans} scans: median point-to-wall {self.median_m * 1e3:.0f} mm, '
            f'{self.inlier:.1%} of points within {FIT_INLIER_M * 1e3:.0f} mm, per-scan median '
            f'p90 {self.scan_median_p90_m * 1e3:.0f} mm (worst at {self.worst_t:.0f} s).'
        )


def fit_scans(
    grid: GridMap,
    scans: Sequence[tuple[float, np.ndarray]],
    map_odom: np.ndarray,
    odom_base: np.ndarray,
    base_laser: Pose,
) -> Fit | None:
    """Score scans given as (mid-sweep time, laser-frame points) against the grid."""
    all_d, medians, times = [], [], []
    for t, pts in scans:
        mo, ob = pose_at(map_odom, t), pose_at(odom_base, t)
        if mo is None or ob is None or len(pts) == 0:
            continue
        x, y = place(pts, compose(compose(mo, ob), base_laser))
        d = grid.lookup(x, y)
        all_d.append(d)
        medians.append(float(np.median(d)))
        times.append(t)
    if not medians:
        return None
    d = np.concatenate(all_d)
    m = np.array(medians)
    return Fit(
        len(medians),
        float(np.median(d)),
        float(np.mean(d <= FIT_INLIER_M)),
        float(np.percentile(m, 90)),
        times[int(np.argmax(m))] - times[0],
    )


def scan_shares(
    grid: GridMap,
    scans: Sequence[tuple[float, np.ndarray]],
    map_odom: np.ndarray,
    odom_base: np.ndarray,
    base_laser: Pose,
) -> list[tuple[float, float]]:
    """Return (time, share of points within FIT_INLIER_M) for each scan with a pose."""
    out = []
    for t, pts in scans:
        mo, ob = pose_at(map_odom, t), pose_at(odom_base, t)
        if mo is None or ob is None or len(pts) == 0:
            continue
        x, y = place(pts, compose(compose(mo, ob), base_laser))
        out.append((t, float(np.mean(grid.lookup(x, y) <= FIT_INLIER_M))))
    return out


def localized_against_fit(
    status: Sequence[tuple[float, bool]],
    shares: Sequence[tuple[float, float]],
    window_sec: float = 2.0,
    bad: float = 0.5,
    good: float = 0.65,
) -> tuple[float, float, float]:
    """
    Return (wrong, missed, span) seconds of localized against the rolling scan fit.

    wrong: localized while the fit was under `bad`. missed: not localized while it was
    `good` or more, after the first localized status. span: the time judged.

    `status` holds (time, localized) changes or samples; each scan takes the last one at
    or before it, and counts until the next scan.
    """
    if not status or len(shares) < 2:
        return 0.0, 0.0, 0.0
    status = sorted(status)
    first = next((t for t, ok in status if ok), None)
    wrong = missed = span = 0.0
    times = np.array([t for t, _ in shares])
    vals = np.array([v for _, v in shares])
    j = 0
    for i in range(len(shares) - 1):
        t, dt = times[i], times[i + 1] - times[i]
        while j + 1 < len(status) and status[j + 1][0] <= t:
            j += 1
        if status[j][0] > t:
            continue
        win = vals[(times > t - window_sec) & (times <= t)]
        m = float(win.mean())
        loc = status[j][1]
        span += dt
        if loc and m < bad:
            wrong += dt
        elif not loc and m >= good and first is not None and t >= first:
            missed += dt
    return wrong, missed, span


def read_bag(bag: Path, every: int = 3) -> dict[str, Any]:
    """Return scans (every Nth), map -> odom, odom -> base_link, and base_link -> laser."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    def yaw(q: Any) -> float:
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id='mcap'),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/scan', '/tf', '/tf_static']))
    scans, map_odom, odom_base = [], [], []
    base_laser: Pose | None = None
    n = 0
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        t = stamp / 1e9  # receive time: one clock for every topic
        if topic == '/scan':
            n += 1
            if n % every:
                continue
            m = deserialize_message(data, LaserScan)
            pts = scan_points(np.asarray(m.ranges, float), m.angle_min, m.angle_increment)
            scans.append((t - m.scan_time / 2.0, pts))
            continue
        for tr in deserialize_message(data, TFMessage).transforms:
            p, q = tr.transform.translation, tr.transform.rotation
            key = (tr.header.frame_id, tr.child_frame_id)
            if key == ('map', 'odom'):
                map_odom.append((t, p.x, p.y, yaw(q)))
            elif key == ('odom', 'base_link'):
                odom_base.append((t, p.x, p.y, yaw(q)))
            elif key == ('base_link', 'laser'):
                base_laser = (p.x, p.y, yaw(q))
    return {
        'scans': scans,
        'map_odom': as_track(map_odom),
        'odom_base': as_track(odom_base),
        'base_laser': base_laser,
    }
