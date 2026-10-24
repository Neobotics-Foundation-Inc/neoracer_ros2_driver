#!/usr/bin/env python3
r"""
Simulated NeoRacers on a lane map, sending V2V state as real cars do.

    python3 scripts/lanes/sim_cars.py scripts/lanes/purdue_testbed.json \\
        --car 1:loop2_fwd:2 --car 2:loop2_back:5 [--to 127.0.0.1] [--seconds 600]

Each car (CAR:LANE:S, where it starts) drives its lanes at --speed m/s,
taking a way on at random at each junction, and sends its state ten times a
second to the V2V port of --to, as a car's v2v_node does. Its state carries
odometry with what a real car's has (lane_sim.drive): a gyro bias that grows
and wanders, a shaking heading, the car weaving in its lane, its encoder off
by --encoder. The odometry starts at 0, 0 heading 0 wherever the car is, as
a car's does, so a viewer with the lane map has to find it (v2v_view.py
--lanes). With --truth FILE, each sample's true lane and place are written
as JSON lines, to score what the viewer shows.
"""

import argparse
import json
import math
import os
import random
import secrets
import socket
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
from neoracer_ros2_driver import v2v_lib as v2v  # noqa: E402
from neoracer_ros2_driver.lane_lock import LaneMap  # noqa: E402,I100
import lane_sim  # noqa: E402,I100


def wander(lane_map, start, lanes, rng):
    """Return `lanes` lanes from `start`, a way on at random at each junction."""
    out = [lane_map.by_id[start]]
    while len(out) < lanes:
        out.append(rng.choice(out[-1].next))
    return out


def parse_car(text):
    car, lane, s = text.split(':')
    return int(car), lane, float(s)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map')
    ap.add_argument('--car', action='append', type=parse_car, required=True,
                    metavar='CAR:LANE:S', help='a car and where it starts (repeatable)')
    ap.add_argument('--to', default='127.0.0.1', help="the viewer's address")
    ap.add_argument('--port', type=int, default=4747)
    ap.add_argument('--speed', type=float, default=1.0, help='m/s on the map')
    ap.add_argument('--seconds', type=float, default=600.0)
    ap.add_argument('--drift', type=float, default=3.0, help='gyro drift, degrees a minute')
    ap.add_argument('--shake', type=float, default=2.0, help='heading shake, degrees')
    ap.add_argument('--weave', type=float, default=0.03, help='weaving in the lane, m either side')
    ap.add_argument('--encoder', type=float, default=1.0,
                    help='map metres per metre the encoder reads (1: true)')
    ap.add_argument('--truth', help="write each sample's true place here (JSON lines)")
    ap.add_argument('--seed', type=int, default=1)
    args = ap.parse_args()
    lanes = LaneMap.load(args.map)
    rng = random.Random(args.seed)
    rate = 10.0
    feeds = []
    for car, start, s0 in args.car:
        if start not in lanes.by_id:
            ap.error('car %d: the map has no lane %s' % (car, start))
        driven = wander(lanes, start, int(args.speed * args.seconds / 2) + 2, rng)
        samples = lane_sim.drive(lanes, driven, speed=args.speed, rate=rate, scale=args.encoder,
                                 bias_rate=math.radians(args.drift) / 60,
                                 vibration=math.radians(args.shake), wobble=args.weave,
                                 start_s=s0, seed=rng.randrange(1 << 30))
        feeds.append({'id': car, 'epoch': secrets.randbits(32), 'samples': samples,
                      'turn': rng.uniform(-math.pi, math.pi)})
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    truth = open(args.truth, 'w', buffering=1) if args.truth else None
    t0 = time.monotonic()
    wall0 = time.time()
    n = min(len(f['samples']) for f in feeds)
    print('%d cars, %.0f s at %.1f m/s: sending to %s:%d'
          % (len(feeds), n / rate, args.speed, args.to, args.port), flush=True)
    for i in range(n):
        while time.monotonic() - t0 < i / rate:
            time.sleep(0.002)
        for f in feeds:
            t, x, y, yaw, lane, s = f['samples'][i]
            # the car's odometry frame: its own, turned by where it was switched on
            c, sn = math.cos(f['turn']), math.sin(f['turn'])
            ox, oy = x * c - y * sn, x * sn + y * c
            packet = {'ver': v2v.PROTOCOL_VERSION, 'type': 'bsm', 'id': f['id'],
                      'epoch': f['epoch'], 'seq': i, 't': wall0 + t,
                      'x': round(ox, 4), 'y': round(oy, 4),
                      'yaw': round(lane_sim.wrap(yaw + f['turn']), 4), 'speed': args.speed,
                      'yaw_rate': 0.0, 'ax': 0.0, 'cmd_speed': 0.2, 'cmd_steer': 0.0, 'batt': 0.8}
            sock.sendto(v2v.encode(packet), (args.to, args.port))
            if truth is not None:
                px, py, ph = lane.pose(s)
                truth.write(json.dumps({'wall': time.time(), 'car': f['id'], 'lane': lane.id,
                                        's': round(s, 3), 'x': round(px, 3), 'y': round(py, 3),
                                        'heading': round(ph, 4)}) + '\n')
    print('done: %d samples a car' % n, flush=True)


if __name__ == '__main__':
    main()
