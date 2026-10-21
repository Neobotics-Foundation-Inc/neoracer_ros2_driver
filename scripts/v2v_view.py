#!/usr/bin/env python3

"""
Watch the NeoRacers from a computer on their network.

A bird's-eye view in the browser, and, optionally, one car's pose driving the
NeoRacer in the AutoDRIVE simulator.

Each car sends its state over V2V ten times a second: its pose in its own
odometry frame (x, y and heading from where its driver started, from the
wheel encoder and the IMU), its speed and its battery. This script joins V2V
as an observer: it says hello like a car, so the cars send it their state,
and sends no state itself, so no car lists it as a peer.

    python3 scripts/v2v_view.py --broadcast 10.0.0.255
        bird's-eye view at http://localhost:8470: every car, its heading, the
        path it has driven, its speed and its battery.
    python3 scripts/v2v_view.py --broadcast 10.0.0.255 --autodrive 2
        also car 2 in AutoDRIVE: open the simulator's NeoRacer scene on this
        computer and press Connect. The simulated NeoRacer starts where it
        stands and moves as car 2 moves, through AutoDRIVE's co-simulation
        input; its Bird's Eye camera follows it from above.

--broadcast is the cars' subnet broadcast address (Windows sends
255.255.255.255 on one interface only). --autodrive needs the Socket.IO stack
AutoDRIVE's devkit pins: Python 3.8, python-socketio 4.2.0, python-engineio
3.13.0 and eventlet 0.33.3.

Positions are each car's odometry: each car starts at 0, 0 where its driver
started, and drifts slowly. Cars share no frame, so --place puts a car's
start where it really stands (CAR:X,Y,HEADING in the view's metres and
degrees) to line several cars up. Only one program on a computer can listen
for V2V (UDP 4747): not this and v2v_peer.py together.
"""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import secrets
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from neoracer_ros2_driver import v2v_lib as v2v  # noqa: E402,I100

PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'v2v_view.html')
HELLO_SEC = 1.0
# The path drawn behind a car: a point every TRAIL_STEP m, the last
# TRAIL_KEEP of them (75 m).
TRAIL_STEP = 0.05
TRAIL_KEEP = 1500
# AutoDRIVE's simulator connects to this port on the computer it runs on.
AUTODRIVE_PORT = 4567


def stamp():
    """Return the wall-clock prefix for a printed line."""
    return time.strftime('%H:%M:%S')


def place(origin, x, y, yaw):
    """Return a pose in a car's odometry frame in the view's frame."""
    ox, oy, oyaw = origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    return ox + x * c - y * s, oy + x * s + y * c, oyaw + yaw


class Fleet:
    """The latest state of every car heard, and the path each has driven."""

    def __init__(self, places):
        self.lock = threading.Lock()
        self.cars = {}
        self.places = places

    def update(self, packet, addr, now):
        """Record one state packet; one with no pose keeps the last pose."""
        with self.lock:
            car = self.cars.setdefault(packet['id'], {'trail': [], 'odom': None, 'pose': None})
            car['addr'] = addr
            car['seen'] = now
            car['bsm'] = packet
            if None in (packet['x'], packet['y'], packet['yaw']):
                return
            car['odom'] = (packet['x'], packet['y'], packet['yaw'])
            car['pose'] = place(self.places.get(packet['id'], (0.0, 0.0, 0.0)), *car['odom'])
            car['posed'] = now
            trail = car['trail']
            x, y = car['pose'][0], car['pose'][1]
            if not trail or math.hypot(x - trail[-1][0], y - trail[-1][1]) >= TRAIL_STEP:
                trail.append((round(x, 3), round(y, 3)))
                del trail[:-TRAIL_KEEP]

    def snapshot(self, now):
        """Return every car as JSON-ready data, ages measured from now."""
        with self.lock:
            out = {}
            for pid, car in sorted(self.cars.items()):
                bsm = car['bsm']
                out[str(pid)] = {
                    'addr': car['addr'],
                    'age': round(now - car['seen'], 2),
                    'pose_age': round(now - car['posed'], 2) if car['pose'] else None,
                    'pose': [round(v, 4) for v in car['pose']] if car['pose'] else None,
                    'speed': bsm['speed'],
                    'yaw_rate': bsm['yaw_rate'],
                    'cmd_speed': bsm['cmd_speed'],
                    'cmd_steer': bsm['cmd_steer'],
                    'batt': bsm['batt'],
                    'trail': car['trail'],
                }
            return out


class Projector:
    """One car's odometry as AutoDRIVE co-simulation poses for vehicle V1."""

    def __init__(self, fleet, car_id, at):
        self.fleet = fleet
        self.car_id = car_id
        self.at = at
        # The simulated car's start (x, y, z, yaw in the simulator's frame:
        # x forward along the scene, y left, z up), and the car's pose that
        # maps onto it: its first one heard.
        self.sim_origin = None
        self.car_origin = None
        self.last = None
        self.said = 0.0

    def reply(self, data):
        """Return the command answering one telemetry message from the simulator."""
        if self.sim_origin is None:
            x, y, z = (float(v) for v in data['V1 Position'].split())
            quaternion = (float(v) for v in data['V1 Orientation Quaternion'].split())
            yaw = v2v.yaw_from_quaternion(*quaternion)
            if self.at is not None:
                x, y, yaw = self.at
            self.sim_origin = (x, y, z, yaw)
            print('%s AutoDRIVE: the simulated NeoRacer starts at x %.2f y %.2f heading %.0f deg'
                  % (stamp(), x, y, math.degrees(yaw)))
        with self.fleet.lock:
            car = self.fleet.cars.get(self.car_id)
            odom = car['odom'] if car is not None else None
        command = {'V1 Throttle': '0', 'V1 Steering': '0', 'V1 Reset': 'False', 'V1 CoSim': '0'}
        if odom is not None:
            if self.car_origin is None:
                self.car_origin = odom
                print('%s AutoDRIVE: car %d at x %.2f y %.2f heading %.0f deg maps onto that start'
                      % (stamp(), self.car_id, odom[0], odom[1], math.degrees(odom[2])))
            self.last = self.project(odom)
        if self.last is not None:
            x, y, z, yaw = self.last
            qx, qy, qz, qw = v2v.quaternion_from_yaw(yaw)
            command.update({'V1 CoSim': '1',
                            'V1 PosX': '%.4f' % x, 'V1 PosY': '%.4f' % y, 'V1 PosZ': '%.4f' % z,
                            'V1 RotX': '%.6f' % qx, 'V1 RotY': '%.6f' % qy,
                            'V1 RotZ': '%.6f' % qz, 'V1 RotW': '%.6f' % qw})
            now = time.monotonic()
            if now - self.said >= 5.0:
                self.said = now
                print('%s AutoDRIVE: car %d -> x %.2f y %.2f heading %.0f deg'
                      % (stamp(), self.car_id, x, y, math.degrees(yaw)))
        return command

    def project(self, odom):
        """Map a pose of the car onto the simulator: its move since the car's origin."""
        cx, cy, cyaw = self.car_origin
        c, s = math.cos(-cyaw), math.sin(-cyaw)
        dx, dy = odom[0] - cx, odom[1] - cy
        rx, ry = dx * c - dy * s, dx * s + dy * c
        sx, sy, sz, syaw = self.sim_origin
        c, s = math.cos(syaw), math.sin(syaw)
        return sx + rx * c - ry * s, sy + rx * s + ry * c, sz, syaw + odom[2] - cyaw


class Observer:
    """A V2V observer: says hello so the cars send it their state."""

    def __init__(self, args, fleet):
        self.fleet = fleet
        self.my_id = args.id
        self.port = args.port
        self.targets = args.broadcast or ['255.255.255.255']
        self.key = bytes.fromhex(args.key) if args.key else None
        self.guard = v2v.ReplayGuard()
        self.epoch = secrets.randbits(32)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(('', self.port))

    def hello(self):
        """Say hello to every target once a second."""
        seq = 0
        while True:
            payload = v2v.encode({'ver': v2v.PROTOCOL_VERSION, 'type': 'hello', 'id': self.my_id,
                                  'epoch': self.epoch, 'seq': seq})
            if self.key is not None:
                payload = v2v.sign(payload, self.key)
            for target in self.targets:
                self.sock.sendto(payload, (target, self.port))
            seq += 1
            time.sleep(HELLO_SEC)

    def listen(self):
        """Take in every state packet the cars send."""
        while True:
            try:
                data, (addr, _) = self.sock.recvfrom(v2v.MAX_DATAGRAM)
            except ConnectionResetError:
                # Windows reports a peer's ICMP port-unreachable as a reset on
                # the next receive; the socket itself is fine.
                continue
            if self.key is not None:
                try:
                    data = v2v.verify(data, self.key)
                except ValueError as e:
                    print('%s %-15s rejected: %s' % (stamp(), addr, e))
                    continue
            try:
                packet = v2v.parse_packet(data)
            except ValueError as e:
                print('%s %-15s bad packet: %s' % (stamp(), addr, e))
                continue
            if packet['id'] == self.my_id:
                continue
            if self.key is not None:
                try:
                    self.guard.check(packet)
                except ValueError as e:
                    print('%s %-15s rejected: %s' % (stamp(), addr, e))
                    continue
            if packet['type'] == 'bsm':
                now = time.monotonic()
                new = packet['id'] not in self.fleet.cars
                self.fleet.update(packet, addr, now)
                if new:
                    print('%s %-15s car %d sending state' % (stamp(), addr, packet['id']))


def serve_page(fleet, port):
    """Serve the bird's-eye page and the cars' state on this port."""
    with open(PAGE, 'rb') as f:
        page = f.read()

    class Handler(BaseHTTPRequestHandler):

        def do_GET(self):
            if self.path == '/':
                body, kind = page, 'text/html; charset=utf-8'
            elif self.path == '/state':
                body = json.dumps({'cars': fleet.snapshot(time.monotonic())}).encode('utf-8')
                kind = 'application/json'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *args):
            pass

    ThreadingHTTPServer(('', port), Handler).serve_forever()


def serve_autodrive(projector):
    """Answer AutoDRIVE's simulator on its port: one pose per telemetry message."""
    import eventlet
    import socketio

    sio = socketio.Server()

    @sio.on('connect')
    def connect(sid, environ):
        print('%s AutoDRIVE: simulator connected' % stamp())

    @sio.on('Bridge')
    def bridge(sid, data):
        # The first message after connecting carries no telemetry: answer it
        # idle, the simulated car where it stands.
        reply = projector.reply(data) if data else {
            'V1 Throttle': '0', 'V1 Steering': '0', 'V1 Reset': 'False', 'V1 CoSim': '0'}
        sio.emit('Bridge', data=reply)

    print('%s AutoDRIVE: waiting for the simulator on port %d (press Connect in it)'
          % (stamp(), AUTODRIVE_PORT))
    eventlet.wsgi.server(eventlet.listen(('', AUTODRIVE_PORT)), socketio.WSGIApp(sio),
                         log_output=False)


def parse_place(text):
    """Parse CAR:X,Y,HEADING (metres, degrees) into (car, (x, y, yaw))."""
    car, rest = text.split(':')
    x, y, heading = (float(v) for v in rest.split(','))
    return int(car), (x, y, math.radians(heading))


def main():
    # Output is usually piped or tee'd; a line at a time, not a block at exit.
    sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=4747, help='V2V UDP port')
    parser.add_argument('--id', type=int, default=91, help='observer id (1-101), no car uses it')
    parser.add_argument('--broadcast', action='append', default=[],
                        help='subnet broadcast address for hellos (repeatable)')
    parser.add_argument('--key', help='fleet key (64 hex) when the cars sign')
    parser.add_argument('--http', type=int, default=8470, help="bird's-eye view port")
    parser.add_argument('--place', action='append', default=[], type=parse_place,
                        help="a car's start in the view: CAR:X,Y,HEADING (repeatable)")
    parser.add_argument('--autodrive', type=int, metavar='CAR',
                        help='drive the AutoDRIVE NeoRacer with this car')
    parser.add_argument('--at', type=float, nargs=3, metavar=('X', 'Y', 'HEADING'),
                        help='the simulated car starts here, not where it stands '
                             "(the simulator's frame: x forward, y left, metres; degrees)")
    args = parser.parse_args()
    if not 1 <= args.id <= v2v.MAX_ID:
        parser.error('ids are 1..%d' % v2v.MAX_ID)

    fleet = Fleet(dict(args.place))
    observer = Observer(args, fleet)
    threading.Thread(target=observer.listen, daemon=True).start()
    threading.Thread(target=observer.hello, daemon=True).start()
    threading.Thread(target=serve_page, args=(fleet, args.http), daemon=True).start()
    print("%s bird's-eye view: http://localhost:%d (observer id %d, hellos to %s)"
          % (stamp(), args.http, args.id, ', '.join(observer.targets)))
    if args.autodrive is None:
        while True:
            time.sleep(1.0)
    at = None
    if args.at is not None:
        at = (args.at[0], args.at[1], math.radians(args.at[2]))
    serve_autodrive(Projector(fleet, args.autodrive, at))


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
