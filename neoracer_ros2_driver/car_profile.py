"""
The car profile: this car's settings, kept outside the driver folder.

A driver update, re-clone, or delete never touches it, so calibration
survives. Layout under profile_dir():

    car.yaml               car name, profile version
    params/<name>.yaml     ROS parameter overrides for config/<name>.yaml
    dashboards/<name>.yaml lab dashboard tuning
    autonomy.conf          mode file of neoracer-autonomy.service
    features.conf          camera and inference choice for this boot
    networking.env         network settings (racecar setup networking)
    history/<stamp>/       the previous copy of each file a write replaced

Launch reads params/ at launch time, so a new value applies on the next
teleop restart with no rebuild.
"""

from __future__ import annotations

from collections.abc import Iterable
from datetime import datetime
import os
from pathlib import Path
import shutil
import socket
import tarfile
from typing import Any

import yaml

PROFILE_ENV = 'RACECAR_CONFIG_DIR'
PROFILE_VERSION = 1
LEGACY_SUFFIX = '.local.yaml'
# The hand-kept calibration copies that the profile replaces.
LEGACY_BACKUP = 'calibration'


def profile_dir() -> Path:
    """RACECAR_CONFIG_DIR, else $XDG_CONFIG_HOME/racecar, else ~/.config/racecar."""
    env = os.environ.get(PROFILE_ENV)
    if env:
        return Path(env).expanduser()
    xdg = os.environ.get('XDG_CONFIG_HOME')
    base = Path(xdg).expanduser() if xdg else Path.home() / '.config'
    return base / 'racecar'


def params_dir() -> Path:
    return profile_dir() / 'params'


def dashboards_dir() -> Path:
    return profile_dir() / 'dashboards'


def param_path(name: str) -> Path:
    """Where the override for config/<name>.yaml lives; `name` without .yaml."""
    return params_dir() / f'{_stem(name)}.yaml'


def param_override(name: str) -> Path | None:
    """Return the override for config/<name>.yaml, or None."""
    path = param_path(name)
    return path if path.is_file() else None


def param_overrides(yamls: Iterable[str]) -> list[str]:
    """Return the override paths, in order, for the config YAMLs that have one."""
    return [str(p) for p in (param_override(y) for y in yamls) if p is not None]


def dashboard_settings(name: str) -> Path:
    """Return the tuning file of lab dashboard `name` (it may not exist yet)."""
    return dashboards_dir() / f'{name}.yaml'


def _stem(name: str) -> str:
    return name[: -len('.yaml')] if name.endswith('.yaml') else name


def write_text(path: Path, text: str, now: datetime | None = None) -> Path:
    """
    Write `text` to `path` through a temporary file.

    A file in the profile that it replaces is copied to history/<stamp>/
    first, at its path relative to the profile.
    """
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists() and _in_profile(path):
        archive(path, now)
    tmp = path.with_name(f'.{path.name}.tmp')
    tmp.write_text(text)
    tmp.replace(path)
    return path


def _in_profile(path: Path) -> bool:
    return path.resolve().is_relative_to(profile_dir().resolve())


def archive(path: Path, now: datetime | None = None) -> Path:
    """Copy `path` into history/<stamp>/ under its place in the profile; return the copy."""
    root = profile_dir()
    stamp = (now or datetime.now()).strftime('%Y%m%d-%H%M%S')
    rel = path.resolve().relative_to(root.resolve())
    dest = root / 'history' / stamp / rel
    dest.parent.mkdir(parents=True, exist_ok=True)
    if path.is_dir():
        shutil.copytree(path, dest, dirs_exist_ok=True)
    else:
        shutil.copy2(path, dest)
    return dest


def read_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        return {}
    data = yaml.safe_load(path.read_text())
    return data if isinstance(data, dict) else {}


def write_params(
    name: str, node: str, params: dict[str, Any], header: str = '', merge: bool = False
) -> Path:
    """
    Write ROS parameters for `node` to params/<name>.yaml.

    With `merge`, the node's parameters already in the file are kept unless
    `params` sets them.
    """
    path = param_path(name)
    data = read_yaml(path)
    old = (data.get(node) or {}).get('ros__parameters') or {} if merge else {}
    data[node] = {'ros__parameters': {**old, **params}}
    body = yaml.safe_dump(data, default_flow_style=None)
    return write_text(path, header + body)


def write_values(name: str, values: dict[str, Any], header: str = '') -> Path:
    """Set plain key: value entries in params/<name>.yaml, keeping its other keys."""
    path = param_path(name)
    merged = {**read_yaml(path), **values}
    return write_text(path, header + yaml.safe_dump(merged, sort_keys=True))


def node_params(name: str, node: str, config_dir: Path | None) -> dict[str, Any]:
    """Return `node`'s parameters from config/<name>.yaml with the profile override on top."""
    params: dict[str, Any] = {}
    paths = [config_dir / f'{_stem(name)}.yaml'] if config_dir is not None else []
    override = param_override(name)
    if override is not None:
        paths.append(override)
    for path in paths:
        params.update((read_yaml(path).get(node) or {}).get('ros__parameters') or {})
    return params


# -- car.yaml --


def car_info() -> dict[str, Any]:
    """Return car.yaml; the name defaults to the host name."""
    info = read_yaml(profile_dir() / 'car.yaml')
    info.setdefault('name', socket.gethostname())
    return info


def ensure_profile() -> Path:
    """Create the profile with a car.yaml if it has none; return its path."""
    root = profile_dir()
    car = root / 'car.yaml'
    if not car.exists():
        body = {'name': socket.gethostname(), 'profile_version': PROFILE_VERSION}
        write_text(
            car,
            '# This car. Read by racecar profile and racecar status.\n'
            + yaml.safe_dump(body, sort_keys=False),
        )
    return root


# -- moving the old per-car files in --


def legacy_files(config_dir: Path | None, dashboards_root: Path | None) -> dict[Path, Path]:
    """
    Map each old per-car file in the driver tree to its place in the profile.

    config/<name>.local.yaml goes to params/<name>.yaml;
    scripts/dashboards/<dir>/<name>.local.yaml to dashboards/<name>.yaml.
    """
    out: dict[Path, Path] = {}
    if config_dir is not None and config_dir.is_dir():
        for f in sorted(config_dir.glob(f'*{LEGACY_SUFFIX}')):
            out[f] = param_path(f.name[: -len(LEGACY_SUFFIX)])
    if dashboards_root is not None and dashboards_root.is_dir():
        for f in sorted(dashboards_root.glob(f'*/*{LEGACY_SUFFIX}')):
            out[f] = dashboard_settings(f.name[: -len(LEGACY_SUFFIX)])
    return out


def unmigrated(config_dir: Path | None, dashboards_root: Path | None) -> list[Path]:
    """Return old per-car files whose profile copy does not exist: values the driver ignores."""
    return [
        old
        for old, new in legacy_files(config_dir, dashboards_root).items()
        if not new.exists()
    ]


def migrate(
    config_dir: Path | None, dashboards_root: Path | None, now: datetime | None = None
) -> list[str]:
    """
    Move the old per-car files into the profile; return what happened, one line each.

    The in-tree file wins. A profile file it replaces is archived first. A
    hand-kept copy in calibration/ that differs is reported, then the folder
    is moved under history/. Running it again changes nothing.
    """
    ensure_profile()
    lines = []
    backup = profile_dir() / LEGACY_BACKUP
    for old, new in legacy_files(config_dir, dashboards_root).items():
        kept = backup / old.name
        if kept.is_file() and kept.read_bytes() != old.read_bytes():
            lines.append(
                f'{old.name}: the copy in {backup} differs; keeping the one in '
                f'the driver folder. The other is kept under {profile_dir()}/history.'
            )
        if new.exists() and new.read_bytes() == old.read_bytes():
            old.unlink()
            lines.append(f'{old.name}: already in the profile; removed from the driver folder.')
            continue
        write_text(new, old.read_text(), now)
        old.unlink()
        lines.append(f'{old.name} -> {new}')
    if backup.is_dir():
        dest = archive(backup, now)
        shutil.rmtree(backup)
        lines.append(f'Moved {backup} to {dest}.')
    return lines


# -- export and import --


def export(path: Path) -> Path:
    """Write the profile, without history/, to a .tar.gz at `path`."""
    root = profile_dir()
    with tarfile.open(path, 'w:gz') as tar:
        for item in sorted(root.iterdir()) if root.is_dir() else []:
            if item.name != 'history':
                tar.add(item, arcname=item.name)
    return path


def import_profile(path: Path, now: datetime | None = None) -> list[str]:
    """Unpack an exported profile; each file it replaces is archived first."""
    root = profile_dir()
    lines = []
    with tarfile.open(path, 'r:gz') as tar:
        for member in tar.getmembers():
            if not member.isfile():
                continue
            rel = Path(member.name)
            if rel.is_absolute() or '..' in rel.parts or rel.parts[0] == 'history':
                lines.append(f'Skipped {member.name}.')
                continue
            data = tar.extractfile(member)
            if data is None:
                continue
            dest = root / rel
            content = data.read()
            if dest.exists() and dest.read_bytes() == content:
                continue
            dest.parent.mkdir(parents=True, exist_ok=True)
            if dest.exists():
                archive(dest, now)
            tmp = dest.with_name(f'.{dest.name}.tmp')
            tmp.write_bytes(content)
            tmp.chmod(member.mode & 0o777)
            tmp.replace(dest)
            lines.append(f'Imported {rel}.')
    return lines
