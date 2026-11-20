#!/usr/bin/env python3
"""
`racecar map`: build, save, and load maps with SLAM on top of teleop.

    racecar map start NAME [--from SAVED [--pose x,y,yaw]] [--force]
    racecar map save
    racecar map stop
    racecar map resume
    racecar map load NAME [--pose x,y,yaw | --global] [--localizer amcl|slam_toolbox]
    racecar map recover NAME
    racecar map status
    racecar map list
    racecar map info NAME
    racecar map foxglove on|off

The work is in scripts/map_control.py. Exit codes: 0 ready, 1 failed, 2
usage.
"""

from __future__ import annotations

import argparse
import sys

import map_control as mc

POSE_HELP = 'x,y,yaw in the map frame: meters, meters, radians (e.g. 1.5,-0.4,1.57)'


def join_pose(argv: list[str]) -> list[str]:
    """Join `--pose -1,2,0` into `--pose=-1,2,0`; argparse reads a leading minus as an option."""
    out: list[str] = []
    i = 0
    while i < len(argv):
        if argv[i] == '--pose' and i + 1 < len(argv):
            out.append(f'--pose={argv[i + 1]}')
            i += 2
        else:
            out.append(argv[i])
            i += 1
    return out


def parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='racecar map',
        description='Build, save, and load maps. Teleop must be running; the car is driven '
        'with the handset as usual.',
    )
    sub = p.add_subparsers(dest='action', required=True, metavar='ACTION')

    s = sub.add_parser('start', help="start mapping NAME; needs the handset's speed switch UP")
    s.add_argument('name')
    s.add_argument('--from', dest='source', metavar='SAVED', help='extend the saved map SAVED')
    s.add_argument('--pose', help=f'with --from: where the car is on SAVED, {POSE_HELP}')
    s.add_argument(
        '--force', action='store_true', help='replace an existing NAME (the old one is kept)'
    )

    sub.add_parser('save', help='save the map being built; returns when the files are written')
    sub.add_parser('stop', help='stop mapping or localization (a mapping run is autosaved)')
    sub.add_parser(
        'resume',
        help='start the last mapping or localization run again; mapping needs the speed switch UP',
    )

    ld = sub.add_parser('load', help='localize on the saved map NAME')
    ld.add_argument('name')
    where = ld.add_mutually_exclusive_group()
    where.add_argument('--pose', help=f'where the car is, {POSE_HELP}')
    where.add_argument(
        '--global', dest='search', action='store_true', help='search the whole map while driving'
    )
    ld.add_argument('--localizer', choices=mc.LOCALIZERS, default='amcl')

    r = sub.add_parser('recover', help="keep NAME's autosave as its saved map")
    r.add_argument('name')
    sub.add_parser('status', help='mode, map, localized, pose')
    sub.add_parser('list', help='saved maps; unsaved ones marked')
    i = sub.add_parser('info', help='details of the map NAME')
    i.add_argument('name')
    f = sub.add_parser('foxglove', help='foxglove_bridge on port 8765 at the next start or load')
    f.add_argument('state', choices=('on', 'off'))
    return p


def run(args: argparse.Namespace) -> int:
    action = args.action
    if action == 'start':
        return mc.start(args.name, args.source, args.pose, args.force)
    if action == 'load':
        return mc.load(args.name, args.pose, args.search, args.localizer)
    if action == 'recover':
        return mc.recover(args.name)
    if action == 'info':
        return mc.info(args.name)
    if action == 'foxglove':
        return mc.set_foxglove(args.state == 'on')
    simple = {
        'save': mc.save,
        'stop': mc.stop,
        'resume': mc.resume,
        'status': mc.status,
        'list': mc.list_maps,
    }
    return simple[action]()


def main(argv: list[str] | None = None) -> int:
    args = parser().parse_args(join_pose(sys.argv[1:] if argv is None else argv))
    try:
        return run(args)
    except mc.MapError as exc:
        print(f'racecar map: {exc}', file=sys.stderr)
        return exc.code
    except KeyboardInterrupt:
        print('racecar map: interrupted', file=sys.stderr)
        return 130


if __name__ == '__main__':
    sys.exit(main())
