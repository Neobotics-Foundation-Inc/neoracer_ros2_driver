#!/usr/bin/env python3
"""
Teleop features that can be turned off to free the CPU: the camera and inference.

The choice is kept in ~/.config/racecar/features.conf; the 8080 dashboard
writes it and restarts teleop once its tile lands (F32).
scripts/launch_teleop.sh turns it into `<subsystem>_enable` launch arguments,
for the teleop service and for `racecar teleop`. Arguments given on the
command line win. The file records the boot it was written in and is ignored
after a reboot, so every boot starts from DEFAULTS: the camera on, because it
is a core part of the API, and inference off, because the YOLO model holds the
GPU and costs CPU on every camera frame. Inference needs the camera.

Usage:
    features.py args [LAUNCH_ARG...]   launch arguments for teleop, one per line;
                                       subsystems already set in LAUNCH_ARG are left out
"""

from __future__ import annotations

from pathlib import Path
import sys

# Feature id -> teleop.launch.py subsystem.
SUBSYSTEMS = {'camera': 'camera', 'inference': 'inference'}
DEFAULTS = {'camera': True, 'inference': False}
# Inference runs on the camera image.
NEEDS = {'inference': 'camera'}

BOOT_ID_PATH = Path('/proc/sys/kernel/random/boot_id')
LAUNCH_ARGS_FILE = 'launch_args'


def state_file() -> Path:
    return Path.home() / '.config' / 'racecar' / 'features.conf'


def boot_id() -> str:
    try:
        return BOOT_ID_PATH.read_text().strip()
    except OSError:
        return ''


def read(path: Path | None = None, boot: str | None = None) -> dict[str, bool]:
    """Return the chosen features; DEFAULTS when the file is missing or from another boot."""
    path = path or state_file()
    boot = boot_id() if boot is None else boot
    values = dict(DEFAULTS)
    try:
        lines = path.read_text().splitlines()
    except OSError:
        return values
    fields = dict(line.split('=', 1) for line in lines if '=' in line)
    if not boot or fields.get('BOOT_ID') != boot:
        return values
    for name in values:
        if fields.get(name.upper()) in ('0', '1'):
            values[name] = fields[name.upper()] == '1'
    return resolve(values)


def write(values: dict[str, bool], path: Path | None = None, boot: str | None = None) -> None:
    path = path or state_file()
    boot = boot_id() if boot is None else boot
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f'BOOT_ID={boot}'] + [f'{k.upper()}={int(v)}' for k, v in values.items()]
    tmp = path.with_suffix('.tmp')
    tmp.write_text('\n'.join(lines) + '\n')
    tmp.replace(path)


def resolve(values: dict[str, bool]) -> dict[str, bool]:
    """Turn off a feature whose requirement is off."""
    out = dict(values)
    for name, needed in NEEDS.items():
        if out.get(name) and not out.get(needed):
            out[name] = False
    return out


def change(values: dict[str, bool], name: str, on: bool) -> dict[str, bool]:
    """Set one feature; turning one on also turns on what it needs."""
    if name not in DEFAULTS:
        raise ValueError(f'unknown feature {name!r}')
    out = dict(values)
    out[name] = on
    if on and name in NEEDS:
        out[NEEDS[name]] = True
    return resolve(out)


def launch_args(values: dict[str, bool], given: list[str] | None = None) -> list[str]:
    """Return `<subsystem>_enable:=true|false` for each feature not already in `given`."""
    set_already = {arg.split(':=', 1)[0] for arg in given or [] if ':=' in arg}
    out = []
    for name, subsystem in SUBSYSTEMS.items():
        key = f'{subsystem}_enable'
        if key not in set_already:
            out.append(f'{key}:={"true" if values[name] else "false"}')
    return out


def running(log_dir: Path | None = None) -> dict[str, bool]:
    """Return the features the running teleop was launched with (logs/latest/launch_args)."""
    path = (log_dir or Path.home() / 'logs' / 'latest') / LAUNCH_ARGS_FILE
    try:
        lines = path.read_text().splitlines()
    except OSError:
        lines = []
    given = dict(arg.strip().split(':=', 1) for arg in lines if ':=' in arg)
    return {
        name: given.get(f'{subsystem}_enable', 'true').strip().lower() not in ('false', '0', 'no')
        for name, subsystem in SUBSYSTEMS.items()
    }


def main(argv: list[str]) -> int:
    if not argv or argv[0] != 'args':
        print(__doc__.strip().split('Usage:')[1], file=sys.stderr)
        return 2
    for arg in launch_args(read(), argv[1:]):
        print(arg)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
