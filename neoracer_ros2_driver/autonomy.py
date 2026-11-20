"""
autonomy_node logic without ROS: the localized test, status fields, and the map files.

Map directory layout (neoracer_ros2_driver/maps.py gives the root):

    <name>/<name>.posegraph, <name>.data   slam_toolbox serialization
    <name>/<name>.pgm, <name>.yaml         map_server format
    <name>/meta.yaml                       written with each save
    <name>/last_pose.yaml                  written while localized; flagged when a
                                           run stops unlocalized (mark_pose_lost)
    <name>/autosave/                       the same file set, replaced whole
    <name>/mapping_pose.yaml               the car's map pose each second while mapping
    <name>/previous/<time>/                saved files replaced by a later run or recover
    replaced/<name>-<time>/                map folders moved aside by `racecar map start --force`

A mapping run that stops without its stop sequence (slam_toolbox crashed)
resumes from the autosave of the same session, at the newer of the autosave's
pose and mapping_pose.yaml (resume_point).

meta.yaml records `start_pose`, the car's pose when the map was begun (the
fallback start pose for a later load), and `last_change`, the time of the last
scan added to the map. An autosave whose `last_change` is newer than the saved
set's holds work that was never saved (map_state).
"""

from __future__ import annotations

from collections import deque
from collections.abc import Sequence
from dataclasses import dataclass, field
from datetime import datetime
import math
import os
from pathlib import Path
import shutil
import socket
from typing import Any

import yaml

AUTOSAVE_DIR = 'autosave'
META_FILE = 'meta.yaml'
LAST_POSE_FILE = 'last_pose.yaml'
AUTOSAVE_FILE = 'autosave.yaml'
MAPPING_POSE_FILE = 'mapping_pose.yaml'
PREVIOUS_DIR = 'previous'
REPLACED_DIR = 'replaced'
MAP_SUFFIXES = ('.posegraph', '.data', '.pgm', '.yaml')
TIME_FORMAT = '%Y-%m-%d %H:%M:%S'

LEVEL_OK, LEVEL_WARN, LEVEL_ERROR = 0, 1, 2


@dataclass(frozen=True)
class Limits:
    scan_timeout_sec: float = 0.5
    max_xy_sd_m: float = 0.25
    max_yaw_sd_deg: float = 20.0
    # Becoming localized takes this share of the limits; losing it, the limits.
    enter_fraction: float = 0.6


def covariance_sd(cov: list[float] | tuple[float, ...]) -> tuple[float, float]:
    """Return (xy SD in m, yaw SD in deg) from a 6x6 row-major pose covariance."""
    xy = math.sqrt(max(cov[0], 0.0) + max(cov[7], 0.0))
    return xy, math.degrees(math.sqrt(max(cov[35], 0.0)))


def is_localized(
    mode: str,
    has_map_odom: bool,
    scan_age: float | None,
    sd: tuple[float, float] | None,
    poses_seen: int,
    global_pending: bool,
    limits: Limits,
    was_localized: bool = False,
    fit_ok: bool = True,
) -> bool:
    """
    Decide `localized` for /autonomy/status.

    Both modes need map -> odom and a scan within scan_timeout_sec; a stale
    scan means the lidar stopped. That is all mapping needs: slam_toolbox
    publishes map -> odom from its first scan, and its pose only when the car
    moves. Localization also needs the localizer's pose spread under the
    limits, and stays false while a global search has not converged. The
    limits have hysteresis: the spread must fall to enter_fraction of them to
    become localized, so the car does not set off with a spread at the limit,
    which the first motion can push over. Localization also needs the live
    scan to fit the map (fit_ok, from FitGate): a car carried elsewhere keeps
    a tight particle cloud at the old place, which the spread alone passes.
    """
    if not has_map_odom or scan_age is None or scan_age > limits.scan_timeout_sec:
        return False
    if mode == 'mapping':
        return True
    if global_pending or sd is None or not fit_ok:
        return False
    scale = 1.0 if was_localized else limits.enter_fraction
    return sd[0] <= limits.max_xy_sd_m * scale and sd[1] <= limits.max_yaw_sd_deg * scale


FIT_INLIER_M = 0.10


class WallMask:
    """Map cells within FIT_INLIER_M of an occupied cell, for the live scan-to-map fit."""

    def __init__(self, near: Any, origin: tuple[float, float], resolution: float) -> None:
        self.near = near  # bool (H, W), row 0 at the top edge as in the .pgm
        self.origin = origin
        self.resolution = resolution

    @classmethod
    def from_occupied(
        cls,
        occupied: Any,
        origin: Sequence[float],
        resolution: float,
        inlier_m: float = FIT_INLIER_M,
    ) -> WallMask:
        from scipy.ndimage import distance_transform_edt

        dist = distance_transform_edt(~occupied) * resolution
        return cls(dist <= inlier_m, (float(origin[0]), float(origin[1])), float(resolution))

    @classmethod
    def from_grid(
        cls,
        data: Sequence[int],
        width: int,
        height: int,
        origin: Sequence[float],
        resolution: float,
        occupied_at: int = 65,
    ) -> WallMask:
        """Build from nav_msgs/OccupancyGrid fields (row 0 at the bottom; -1 unknown)."""
        import numpy as np

        grid = np.asarray(data, dtype=np.int16).reshape(height, width)
        return cls.from_occupied(np.flipud(grid >= occupied_at), origin, resolution)

    def share(self, x: Any, y: Any) -> float:
        """Return the share of points on a near-wall cell; points off the map miss."""
        import numpy as np

        if len(x) == 0:
            return 0.0
        h, w = self.near.shape
        col = np.floor((x - self.origin[0]) / self.resolution).astype(int)
        row = h - 1 - np.floor((y - self.origin[1]) / self.resolution).astype(int)
        inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
        hits = self.near[row[inside], col[inside]].sum()
        return float(hits) / len(x)


def scan_points_in_map(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    laser_pose: tuple[float, float, float],
    min_range: float,
    max_range: float,
    beams: int,
) -> tuple[Any, Any]:
    """Return map x and y of about `beams` evenly spaced valid returns of one scan."""
    import numpy as np

    r = np.asarray(ranges, dtype=float)
    step = max(1, len(r) // max(beams, 1))
    idx = np.arange(0, len(r), step)
    r = r[idx]
    ok = np.isfinite(r) & (r >= min_range) & (r <= max_range)
    a = angle_min + idx[ok] * angle_increment + laser_pose[2]
    return laser_pose[0] + r[ok] * np.cos(a), laser_pose[1] + r[ok] * np.sin(a)


@dataclass
class FitGate:
    """
    Rolling scan-fit share with hysteresis, for the fit_ok input of is_localized.

    Correct poses score 0.78 to 0.91 on a room map, including people passing
    and 1 m/s driving; a car carried 2 m scores 0.29 to 0.52. Losing takes the
    window mean under drop_share; becoming localized takes enter_share.
    """

    window_sec: float = 2.0
    drop_share: float = 0.5
    enter_share: float = 0.65
    min_scores: int = 3
    scores: deque[tuple[float, float]] = field(default_factory=deque)

    def add(self, t: float, share: float) -> None:
        self.scores.append((t, share))
        while self.scores and self.scores[0][0] < t - self.window_sec:
            self.scores.popleft()

    def mean(self, now: float) -> float | None:
        recent = [s for t, s in self.scores if t >= now - self.window_sec]
        if len(recent) < self.min_scores:
            return None
        return sum(recent) / len(recent)

    def passes(self, now: float, was_localized: bool) -> bool:
        m = self.mean(now)
        if m is None:
            return False
        return m >= (self.drop_share if was_localized else self.enter_share)


SEED_WAIT_SEC = 3.0


def seed_covariance(sd_xy_m: float, sd_yaw_deg: float) -> list[float]:
    """Return a 6x6 row-major pose covariance with the given x, y, and yaw spread."""
    cov = [0.0] * 36
    cov[0] = cov[7] = sd_xy_m**2
    cov[35] = math.radians(sd_yaw_deg) ** 2
    return cov


def shows_seed(sd: tuple[float, float], cov: list[float]) -> bool:
    """Report whether a localizer pose spread reflects the seed (half its spread or more)."""
    return sd[0] >= 0.5 * covariance_sd(cov)[0]


def status_level(localized: bool, has_map_odom: bool, seconds_running: float) -> int:
    if localized:
        return LEVEL_OK
    return LEVEL_ERROR if not has_map_odom and seconds_running > 5.0 else LEVEL_WARN


def write_yaml_atomic(path: Path, data: dict[str, Any]) -> None:
    """Write YAML through a temporary file and a rename, so readers never see half a file."""
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_name(path.name + '.tmp')
    tmp.write_text(yaml.safe_dump(data, sort_keys=False))
    os.replace(tmp, path)


def replace_dir(tmp: Path, final: Path) -> None:
    """
    Move a finished directory over another.

    A crash at any point leaves either the old or the new directory in place
    under `final`, or the new one still under `tmp`.
    """
    old = final.with_name(final.name + '.old')
    if old.exists():
        shutil.rmtree(old)
    if final.exists():
        os.rename(final, old)
    os.rename(tmp, final)
    if old.exists():
        shutil.rmtree(old)


LOCALIZED_AT_STOP = 'localized_at_stop'


def mark_pose_lost(path: Path, now: datetime) -> None:
    """Flag last_pose.yaml when a run stops unlocalized: the car may have moved since."""
    record = read_record(path)
    if pose_of(record) is None:
        return
    record[LOCALIZED_AT_STOP] = False
    record['stopped'] = now.strftime(TIME_FORMAT)
    write_yaml_atomic(path, record)


def pose_values(pose: tuple[float, float, float]) -> dict[str, float]:
    x, y, yaw = pose
    return {'x': round(x, 4), 'y': round(y, 4), 'yaw': round(yaw, 5)}


def pose_record(pose: tuple[float, float, float], now: datetime, **extra: Any) -> dict[str, Any]:
    return {**pose_values(pose), 'time': now.strftime(TIME_FORMAT), **extra}


def change_time(now: datetime) -> str:
    """Return a `last_change` stamp; milliseconds, so string order is time order."""
    return now.isoformat(sep=' ', timespec='milliseconds')


def pose_of(data: Any) -> tuple[float, float, float] | None:
    """Return x, y, yaw from a mapping that holds them; None otherwise."""
    try:
        return float(data['x']), float(data['y']), float(data['yaw'])
    except (KeyError, TypeError, ValueError):
        return None


def read_pose(path: Path) -> tuple[float, float, float] | None:
    """Read x, y, yaw from last_pose.yaml or autosave.yaml; None when missing or malformed."""
    return pose_of(read_record(path))


def read_record(path: Path) -> dict[str, Any]:
    """Read a YAML mapping; {} when missing or malformed."""
    try:
        data = yaml.safe_load(path.read_text())
    except (OSError, yaml.YAMLError):
        return {}
    return data if isinstance(data, dict) else {}


def resume_point(
    folder: Path, name: str, session: str
) -> tuple[Path, tuple[float, float, float]] | None:
    """
    Return the autosaved pose graph stem and the start pose to resume a run, or None.

    Resumes only an autosave of the same session (a new run starts a new map).
    The pose is the newer of the autosave's and mapping_pose.yaml's, so a
    crash loses at most the time since the last autosave and the car resumes
    near where it stopped. Both are in the map frame of that pose graph.
    """
    if not session:
        return None
    autosave = folder / AUTOSAVE_DIR
    record = read_record(autosave / AUTOSAVE_FILE)
    stem = autosave / name
    if record.get('session') != session or not Path(f'{stem}.posegraph').exists():
        return None
    pose = read_pose(autosave / AUTOSAVE_FILE)
    latest = read_record(folder / MAPPING_POSE_FILE)
    if latest.get('session') == session and str(latest.get('time', '')) >= str(
        record.get('time', '')
    ):
        pose = read_pose(folder / MAPPING_POSE_FILE) or pose
    return (stem, pose) if pose is not None else None


def meta_record(
    name: str,
    now: datetime,
    driver_version: str,
    frames: dict[str, float],
    session: str,
    start_pose: tuple[float, float, float] | None = None,
    last_change: str = '',
    extended_from: str = '',
) -> dict[str, Any]:
    return {
        'name': name,
        'saved': now.strftime(TIME_FORMAT),
        'driver_version': driver_version,
        'hostname': socket.gethostname(),
        'session': session,
        'start_pose': pose_values(start_pose) if start_pose else None,
        'last_change': last_change,
        'extended_from': extended_from,
        'frames': frames,
    }


def has_map_files(folder: Path, name: str) -> bool:
    return all((folder / f'{name}{suffix}').is_file() for suffix in MAP_SUFFIXES)


@dataclass(frozen=True)
class MapState:
    """What a map folder holds; read from the files alone."""

    name: str
    folder: Path
    saved: bool
    autosaved: bool
    meta: dict[str, Any]
    autosave_meta: dict[str, Any]
    autosave_record: dict[str, Any]

    @property
    def exists(self) -> bool:
        return self.saved or self.autosaved

    @property
    def unsaved(self) -> bool:
        """Report whether the autosave holds changes the saved set lacks."""
        if not self.autosaved:
            return False
        if not self.saved:
            return True
        auto, saved = self.autosave_meta.get('last_change'), self.meta.get('last_change')
        if auto is not None and saved is not None:
            return str(auto) > str(saved)
        # Saved before last_change was recorded: compare the write times.
        return str(self.autosave_record.get('time', '')) > str(self.meta.get('saved', ''))

    @property
    def autosave_time(self) -> str:
        return str(self.autosave_record.get('time', ''))

    @property
    def start_pose(self) -> tuple[float, float, float] | None:
        return pose_of(self.meta.get('start_pose'))


def map_state(root: Path, name: str) -> MapState:
    folder = root / name
    autosave = folder / AUTOSAVE_DIR
    return MapState(
        name=name,
        folder=folder,
        saved=has_map_files(folder, name),
        autosaved=has_map_files(autosave, name),
        meta=read_record(folder / META_FILE),
        autosave_meta=read_record(autosave / META_FILE),
        autosave_record=read_record(autosave / AUTOSAVE_FILE),
    )


def _free_path(path: Path) -> Path:
    """Return `path`, or `path-2`, `path-3`, ... when it is taken."""
    candidate, n = path, 1
    while candidate.exists():
        n += 1
        candidate = path.with_name(f'{path.name}-{n}')
    return candidate


def archive_map_files(folder: Path, name: str, now: datetime) -> Path | None:
    """
    Move the saved map files and meta.yaml into `previous/<time>/`; nothing is deleted.

    Returns the folder they went to, or None when there was nothing to move.
    last_pose.yaml stays: the map frame does not change when a map is extended.
    """
    files = [folder / f'{name}{suffix}' for suffix in MAP_SUFFIXES] + [folder / META_FILE]
    present = [f for f in files if f.is_file()]
    if not present:
        return None
    dest = _free_path(folder / PREVIOUS_DIR / now.strftime('%Y%m%d-%H%M%S'))
    dest.mkdir(parents=True)
    for f in present:
        os.rename(f, dest / f.name)
    return dest


def replace_folder(root: Path, name: str, now: datetime) -> Path:
    """Move `<root>/<name>/` to `<root>/replaced/<name>-<time>/`; return the new path."""
    dest = _free_path(root / REPLACED_DIR / f'{name}-{now.strftime("%Y%m%d-%H%M%S")}')
    dest.parent.mkdir(parents=True, exist_ok=True)
    os.rename(root / name, dest)
    return dest


def copy_autosave(folder: Path, name: str) -> None:
    """Copy the autosaved map files and meta.yaml into the map folder, each through a rename."""
    autosave = folder / AUTOSAVE_DIR
    for f in [f'{name}{suffix}' for suffix in MAP_SUFFIXES] + [META_FILE]:
        src = autosave / f
        if not src.is_file():
            continue
        tmp = folder / (f + '.tmp')
        shutil.copy2(src, tmp)
        os.replace(tmp, folder / f)


def status_values(**fields: Any) -> list[tuple[str, str]]:
    """Format status fields as DiagnosticStatus key-value strings, in the given order."""
    out = []
    for key, value in fields.items():
        if isinstance(value, bool):
            text = 'true' if value else 'false'
        elif isinstance(value, float):
            text = f'{value:.0f}' if key.endswith('_ms') else f'{value:.3f}'
        else:
            text = '' if value is None else str(value)
        out.append((key, text))
    return out
