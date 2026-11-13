"""Saved-map locations: `<root>/<name>/<name>.{yaml,pgm,posegraph,data}`."""

from __future__ import annotations

import math
import os
from pathlib import Path
import re

from .data_root import current as data_root

ROOT_ENV = 'RACECAR_MAP_ROOT'
NAME_PATTERN = re.compile(r'[A-Za-z0-9_-]+')


def map_root() -> Path:
    """RACECAR_MAP_ROOT, else maps/ under the data root (data_root.py)."""
    env = os.environ.get(ROOT_ENV)
    return Path(env) if env else data_root()[0] / 'maps'


def valid_name(name: str) -> bool:
    return NAME_PATTERN.fullmatch(name) is not None


def map_stem(name: str, root: Path | None = None) -> Path:
    """
    Return the path stem of a saved map: a name under the map root, or a path.

    A name `room` gives `<root>/room/room`; slam_toolbox appends `.posegraph`
    and `.data` to the stem, map_server reads `<stem>.yaml`.
    """
    if valid_name(name):
        return (root or map_root()) / name / name
    path = Path(name).expanduser()
    return path.with_suffix('') if path.suffix in ('.yaml', '.posegraph', '.data') else path


def parse_pose(text: str) -> tuple[float, float, float] | None:
    """Parse `x,y,yaw` (m, m, rad); empty gives None. Raises ValueError otherwise."""
    if not text.strip():
        return None
    parts = [float(p) for p in text.split(',')]
    if len(parts) != 3 or not all(math.isfinite(p) for p in parts):
        raise ValueError(f'pose must be x,y,yaw: {text!r}')
    return parts[0], parts[1], parts[2]
