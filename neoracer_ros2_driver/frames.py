"""
Frame offsets: load config/frames.yaml with the car profile's override, convert units.

frames.yaml keys carry their unit (`_mm` or `_deg`). The URDF takes meters and
radians, under the key name with the unit suffix removed.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import yaml

from .car_profile import param_override

FRAMES_YAML = 'frames.yaml'
_UNITS = {'_mm': 1e-3, '_deg': math.pi / 180.0}


def _read(path: Path) -> dict[str, float]:
    data: Any = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        raise ValueError(f'{path}: expected a mapping of key: value')
    for key, value in data.items():
        if not any(key.endswith(suffix) for suffix in _UNITS):
            raise ValueError(f'{path}: {key} has no unit suffix (_mm or _deg)')
        if isinstance(value, bool) or not isinstance(value, int | float):
            raise ValueError(f'{path}: {key} is not a number: {value!r}')
    return {k: float(v) for k, v in data.items()}


def load_frames(config_dir: str | Path) -> dict[str, float]:
    """
    Return frames.yaml merged with the profile's params/frames.yaml, in file units.

    An override key absent from frames.yaml is an error, so a typo cannot pass
    silently as an unused override.
    """
    config_dir = Path(config_dir)
    values = _read(config_dir / FRAMES_YAML)
    local = param_override('frames')
    if local is not None:
        overrides = _read(local)
        unknown = sorted(set(overrides) - set(values))
        if unknown:
            raise ValueError(f'{local}: keys not in {FRAMES_YAML}: {", ".join(unknown)}')
        values.update(overrides)
    return values


def xacro_mappings(values: dict[str, float]) -> dict[str, str]:
    """Return xacro arguments in meters and radians, keyed without the unit suffix."""
    mappings = {}
    for key, value in values.items():
        suffix = next(s for s in _UNITS if key.endswith(s))
        mappings[key[: -len(suffix)]] = format(value * _UNITS[suffix], '.10g')
    return mappings
