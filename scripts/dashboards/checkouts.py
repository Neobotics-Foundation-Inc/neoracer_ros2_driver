#!/usr/bin/env python3
"""
Move a car's old lab dashboard checkouts aside, keeping what the car made in them.

Before the lab dashboards moved into the driver, setup_services.sh cloned each
one's repository to scripts/dashboards/<repo>_dashboard/, and each dashboard
saved its tuning into its own yaml in that checkout. Now each one ships in
scripts/dashboards/<id>/, its tuning lives in the car profile and its logs
under the data root. setup_dashboards.sh runs this before it installs the
units. For every old checkout it:

- copies the values the car tuned into the car profile's
  dashboards/<name>.yaml, on top of the in-tree defaults. A tuned value is a
  number that differs from the checkout's committed yaml. A profile file that
  already exists is left as it is.
- moves the checkout's logs (logs/*.csv), and eps's ledger, to
  dashboards/<name>/ under the data root, never over a file already there.
- moves the checkout to scripts/dashboards/.migrated/.

It also moves camlabel's dataset from ~/data/camlabel to datasets/camlabel
under the data root while the data root holds no dataset yet. Nothing is
deleted, and running it again finds nothing left to move.

    python3 checkouts.py [DASHBOARDS_DIR]
"""

from __future__ import annotations

from datetime import datetime
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
from typing import Any

from neoracer_ros2_driver import car_profile, data_root
import yaml

HERE = Path(__file__).resolve().parent
MIGRATED = '.migrated'
# id: (old repository, the name of its yaml, tuning and log folder)
DASHBOARDS = {
    'wallfollow': ('wallfollow_dashboard', 'wallfollow'),
    'camlabel': ('camlabel_dashboard', 'camlabel'),
    'pursuit': ('pursuit_dashboard', 'pursuit'),
    'eps': ('eps_dashboard', 'eps'),
    'smartfollow': ('smartfollow_dashboard', 'smartfollow'),
    'linefollow': ('linefollow_dashboard', 'linefollow'),
    'webteleop': ('teleop_dashboard', 'teleop'),
}
# What eps kept beside its code: the session ledger, its archives, and the
# mark that the starting point is set.
EPS_FILES = ('ledger.jsonl', 'ledger_*.jsonl', '.init_set')


def _number(v: Any) -> bool:
    return isinstance(v, (int, float))


def flat_numbers(data: Any) -> dict[tuple[str, ...], Any]:
    """Return the numbers and booleans of a yaml mapping, one level of blocks deep."""
    out: dict[tuple[str, ...], Any] = {}
    if not isinstance(data, dict):
        return out
    for k, v in data.items():
        if isinstance(v, dict):
            for k2, v2 in v.items():
                if _number(v2):
                    out[(str(k), str(k2))] = v2
        elif _number(v):
            out[(str(k),)] = v
    return out


def tuned_values(current: Any, shipped: Any) -> dict[tuple[str, ...], Any]:
    """Return the numbers in `current` that differ from, or are missing in, `shipped`."""
    base = flat_numbers(shipped)
    return {k: v for k, v in flat_numbers(current).items() if base.get(k) != v}


def _scalar(v: Any) -> str:
    return 'true' if v is True else 'false' if v is False else str(v)


def apply_values(text: str, values: dict[tuple[str, ...], Any]) -> tuple[str, list[str]]:
    """
    Write `values` into a yaml text in place, so its comments survive.

    Keys are (key,) at the top level or (block, key) one level down, as
    flat_numbers returns them. Returns the new text and the dotted names of
    the keys the text does not have.
    """
    scalar = re.compile(r'^([ \t]*)([A-Za-z_]\w*):([ \t]*)([^#\s][^#]*?)?([ \t]*#.*)?$')
    out, block, done = [], None, set()
    for raw in text.splitlines(keepends=True):
        line = raw.rstrip('\n')
        m = scalar.match(line)
        if not m:
            out.append(raw)
            continue
        indent, key, gap, value, tail = m.groups()
        if not indent:
            block = key if value is None else None
        path = (block, key) if indent and block else (key,)
        if value is not None and path in values:
            out.append(f'{indent}{key}:{gap}{_scalar(values[path])}{tail or ""}\n')
            done.add(path)
            continue
        out.append(raw)
    missing = ['.'.join(k) for k in values if k not in done]
    return ''.join(out), missing


def _committed_yaml(checkout: Path, name: str) -> str | None:
    """Return the checkout's committed <name>.yaml, or None without git history."""
    try:
        res = subprocess.run(
            ['git', '-C', str(checkout), 'show', f'HEAD:{name}.yaml'],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    return res.stdout if res.returncode == 0 else None


def migrate_tuning(dash_dir: Path, ident: str, checkout: Path, now: datetime) -> list[str]:
    """Copy the values the car tuned in an old checkout into the car profile."""
    repo, name = DASHBOARDS[ident]
    old = checkout / f'{name}.yaml'
    if not old.is_file():
        return []
    profile = car_profile.dashboard_settings(name)
    if profile.exists():
        return [f'{ident}: {profile} already exists, so the tuning in {repo} was not copied']
    defaults = (dash_dir / ident / f'{name}.yaml').read_text()
    committed = _committed_yaml(checkout, name)
    baseline = committed if committed is not None else defaults
    tuned = tuned_values(yaml.safe_load(old.read_text()), yaml.safe_load(baseline))
    if not tuned:
        return [f'{ident}: no tuned values in {repo}']
    text, missing = apply_values(defaults, tuned)
    car_profile.write_text(profile, text, now)
    kept = sorted('.'.join(k) for k in tuned if '.'.join(k) not in missing)
    lines = [f'{ident}: tuned values {", ".join(kept)} -> {profile}']
    if committed is None:
        lines.append(f'{ident}: {repo} has no git history; compared with the shipped defaults')
    if missing:
        lines.append(f'{ident}: not in the new {name}.yaml, so not copied: {", ".join(missing)}')
    return lines


def _move_files(files: list[Path], dest: Path, ident: str) -> list[str]:
    lines, moved = [], 0
    for f in files:
        target = dest / f.name
        if target.exists():
            lines.append(f'{ident}: {target} already exists; {f.name} stays with the old checkout')
            continue
        dest.mkdir(parents=True, exist_ok=True)
        shutil.move(str(f), str(target))
        moved += 1
    if moved:
        lines.insert(0, f'{ident}: moved {moved} file(s) to {dest}')
    return lines


def migrate_logs(ident: str, checkout: Path) -> list[str]:
    """Move an old checkout's logs, and eps's ledger, under the data root."""
    _, name = DASHBOARDS[ident]
    files = sorted((checkout / 'logs').glob('*.csv'))
    if ident == 'eps':
        files += sorted(f for pattern in EPS_FILES for f in checkout.glob(pattern))
    dest = data_root.current()[0] / 'dashboards' / name
    return _move_files(files, dest, ident) if files else []


def _has_files(path: Path) -> bool:
    return path.is_dir() and any(p.is_file() for p in path.rglob('*'))


def _remove_empty_tree(path: Path) -> None:
    for d in sorted((p for p in path.rglob('*') if p.is_dir()), reverse=True):
        d.rmdir()
    path.rmdir()


def migrate_camlabel_dataset(home: Path | None = None) -> list[str]:
    """Move camlabel's dataset from ~/data/camlabel to datasets/camlabel under the data root."""
    src = (home or Path.home()) / 'data' / 'camlabel'
    if not _has_files(src):
        return []
    dest = data_root.current()[0] / 'datasets' / 'camlabel'
    if _has_files(dest):
        return [
            f'camlabel: {dest} already holds a dataset, so {src} was left in place; '
            'merge them by hand (image numbers would collide)'
        ]
    if dest.exists():
        # camlabel makes its empty folders at startup.
        _remove_empty_tree(dest)
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(src), str(dest))
    return [f'camlabel: dataset moved from {src} to {dest}']


def set_aside(dash_dir: Path, checkout: Path, now: datetime) -> str:
    """Move an old checkout to .migrated/ and return where it went."""
    dest = dash_dir / MIGRATED / checkout.name
    if dest.exists():
        dest = dest.with_name(f'{checkout.name}-{now:%Y%m%d-%H%M%S}')
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.move(str(checkout), str(dest))
    return f'{checkout.name}: moved to {dest}'


def migrate(
    dash_dir: Path, now: datetime | None = None, home: Path | None = None
) -> tuple[list[str], bool]:
    """Migrate every old checkout under dash_dir; return the report and whether all went well."""
    now = now or datetime.now()
    lines, ok = [], True
    for ident, (repo, _) in DASHBOARDS.items():
        checkout = dash_dir / repo
        if not checkout.is_dir():
            continue
        try:
            lines += migrate_tuning(dash_dir, ident, checkout, now)
            lines += migrate_logs(ident, checkout)
            lines.append(set_aside(dash_dir, checkout, now))
        except Exception as e:
            # One checkout that cannot be moved never stops the others, or
            # the install of the units after them.
            lines.append(f'{ident}: {repo} left in place: {e}')
            ok = False
    try:
        lines += migrate_camlabel_dataset(home)
    except Exception as e:
        lines.append(f'camlabel: dataset left in place: {e}')
        ok = False
    return lines, ok


def main(argv: list[str]) -> int:
    dash_dir = Path(argv[0]) if argv else HERE
    if os.geteuid() == 0:
        print('  run this as the car user, not root: the profile and data are theirs')
        return 1
    lines, ok = migrate(dash_dir)
    for line in lines or ['no old checkouts']:
        print(f'  {line}')
    return 0 if ok else 1


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))
