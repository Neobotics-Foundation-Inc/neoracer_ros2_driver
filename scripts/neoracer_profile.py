#!/usr/bin/env python3
"""
`racecar profile`: this car's settings, kept outside the driver folder.

    racecar profile               path, car name, and each file with its date
    racecar profile path          the profile folder only (for scripts)
    racecar profile migrate       move config/*.local.yaml and dashboard tuning in
    racecar profile export FILE   write the profile to FILE (.tar.gz)
    racecar profile import FILE   read a profile written by export
    racecar profile check         warn about per-car files the driver ignores

The work is in neoracer_ros2_driver/car_profile.py.
"""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys

from neoracer_ros2_driver import car_profile

HERE = Path(__file__).resolve().parent
PACKAGE_ROOT = HERE.parent


def show() -> int:
    root = car_profile.profile_dir()
    print(f'Car profile: {root}')
    print(f'Car name: {car_profile.car_info()["name"]}')
    if not root.is_dir():
        print('No profile yet. Calibrating the car, or racecar profile migrate, creates it.')
        return 0
    files = sorted(
        p for p in root.rglob('*') if p.is_file() and 'history' not in p.relative_to(root).parts
    )
    for path in files:
        stamp = datetime.fromtimestamp(path.stat().st_mtime).strftime('%Y-%m-%d %H:%M')
        print(f'  {stamp}  {path.relative_to(root)}')
    left = car_profile.unmigrated(PACKAGE_ROOT / 'config', HERE / 'dashboards')
    if left:
        print('Not in the profile, so not used (run racecar profile migrate):')
        for path in left:
            print(f'  {path}')
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog='racecar profile', description=__doc__.split('\n')[1])
    sub = parser.add_subparsers(dest='action')
    sub.add_parser('path', help='print the profile folder')
    sub.add_parser('migrate', help='move the per-car files out of the driver folder')
    e = sub.add_parser('export', help='write the profile to a .tar.gz')
    e.add_argument('file', type=Path)
    i = sub.add_parser('import', help='read a profile written by export')
    i.add_argument('file', type=Path)
    sub.add_parser('check', help='warn about per-car files left in the driver folder')
    args = parser.parse_args(argv)

    if args.action is None:
        return show()
    if args.action == 'check':
        left = car_profile.unmigrated(PACKAGE_ROOT / 'config', HERE / 'dashboards')
        for path in left:
            print(f'WARNING: {path} is not used. Run: racecar profile migrate')
        return 0
    if args.action == 'path':
        print(car_profile.profile_dir())
        return 0
    if args.action == 'migrate':
        lines = car_profile.migrate(PACKAGE_ROOT / 'config', HERE / 'dashboards')
        print('\n'.join(lines) if lines else 'Nothing to move.')
        if lines:
            print('Apply it: racecar service restart teleop.')
        return 0
    if args.action == 'export':
        print(f'Wrote {car_profile.export(args.file)}')
        return 0
    if not args.file.is_file():
        print(f'No such file: {args.file}', file=sys.stderr)
        return 1
    lines = car_profile.import_profile(args.file)
    print('\n'.join(lines) if lines else 'Nothing changed.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
