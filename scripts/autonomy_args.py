#!/usr/bin/env python3
"""
Turn the autonomy unit's mode file into autonomy.launch.py arguments, one per line.

launch_autonomy.sh runs this with the variables systemd loads from
~/.config/racecar/autonomy.conf (EnvironmentFile):

    AUTONOMY_MODE        mapping | localization (default mapping)
    AUTONOMY_MAP         map name under the map root
    AUTONOMY_SESSION     run id; a restart with the same id resumes its autosave
    AUTONOMY_FROM        mapping: saved map to extend
    AUTONOMY_POSE        x,y,yaw | global (localization); x,y,yaw (extension)
    AUTONOMY_POSE_SOURCE reported in /autonomy/status
    AUTONOMY_LOCALIZER   amcl | slam_toolbox (localization; default amcl)
    AUTONOMY_FOXGLOVE    1 adds foxglove_bridge

A mapping run whose map holds an autosave of the same session resumes from it
(neoracer_ros2_driver.autonomy.resume_point): after slam_toolbox crashed and
systemd restarted the unit, or after a teleop restart restarted it. Otherwise
a mapping run with AUTONOMY_FROM loads that map's pose graph (its autosave when
it was never saved) at AUTONOMY_POSE. Exits 2 with a message on a bad value,
which fails the unit start.
"""

from __future__ import annotations

from collections.abc import Mapping
import os
from pathlib import Path
import sys

from neoracer_ros2_driver.autonomy import AUTOSAVE_DIR, map_state, resume_point
from neoracer_ros2_driver.maps import map_root, map_stem, valid_name

MODES = ('mapping', 'localization')
LOCALIZERS = ('amcl', 'slam_toolbox')


def source_stem(name: str) -> Path:
    """Return the pose graph stem of a map to extend: its saved set, else its autosave."""
    if not valid_name(name):
        raise ValueError(f'AUTONOMY_FROM must be letters, digits, - and _: {name!r}')
    state = map_state(map_root(), name)
    if state.saved:
        return state.folder / name
    if state.autosaved:
        return state.folder / AUTOSAVE_DIR / name
    raise ValueError(f'AUTONOMY_FROM: no map named {name!r} under {map_root()}')


def launch_args(env: Mapping[str, str]) -> list[str]:
    """Return autonomy.launch.py arguments for a mode file; ValueError on a bad value."""
    mode = env.get('AUTONOMY_MODE', '').strip() or 'mapping'
    name = env.get('AUTONOMY_MAP', '').strip()
    session = env.get('AUTONOMY_SESSION', '').strip()
    pose = env.get('AUTONOMY_POSE', '').strip()
    localizer = env.get('AUTONOMY_LOCALIZER', '').strip() or 'amcl'
    if mode not in MODES:
        raise ValueError(f'AUTONOMY_MODE must be mapping or localization: {mode!r}')
    if not valid_name(name):
        raise ValueError(f'AUTONOMY_MAP must be letters, digits, - and _: {name!r}')
    if localizer not in LOCALIZERS:
        raise ValueError(f'AUTONOMY_LOCALIZER must be amcl or slam_toolbox: {localizer!r}')
    args = [f'mode:={mode}', f'map:={name}', f'session:={session}']
    if mode == 'mapping':
        found = resume_point(map_stem(name).parent, name, session)
        source = env.get('AUTONOMY_FROM', '').strip()
        if found is not None:
            stem, (x, y, yaw) = found
            args += [f'resume:={stem}', f'pose:={x},{y},{yaw}']
        elif source:
            args += [f'extend:={source_stem(source)}', f'pose:={pose or "0,0,0"}']
    else:
        args.append(f'localizer:={localizer}')
        if pose:
            args.append(f'pose:={pose}')
    source = env.get('AUTONOMY_POSE_SOURCE', '').strip()
    if source:
        args.append(f'pose_source:={source}')
    if env.get('AUTONOMY_FOXGLOVE', '').strip() == '1':
        args.append('foxglove:=true')
    return args


def main() -> int:
    try:
        args = launch_args(os.environ)
    except ValueError as exc:
        print(f'autonomy: {exc}', file=sys.stderr)
        return 2
    print('\n'.join(args))
    return 0


if __name__ == '__main__':
    sys.exit(main())
