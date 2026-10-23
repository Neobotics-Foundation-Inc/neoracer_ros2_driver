#!/usr/bin/env python3

"""
Watch the NeoRacers from a computer on their network.

A bird's-eye view in the browser, and, optionally, one car's pose driving the
NeoRacer in the AutoDRIVE simulator.

Each car sends its state over V2V ten times a second: its pose in its own
odometry frame (x, y and heading from where its driver started, from the
wheel encoder and the IMU), its pose on the map while it is localized on one,
its speed and its battery. This script joins V2V as an observer: it says
hello like a car, so the cars send it their state, and sends no state
itself, so no car lists it as a peer.

    python3 scripts/v2v_view.py --broadcast 10.0.0.255
        bird's-eye view at http://localhost:8470: every car, its heading, the
        path it has driven, its speed and its battery.
    python3 scripts/v2v_view.py --broadcast 10.0.0.255 --map lab.yaml
        the same over a saved SLAM map (racecar mapping save; the .yaml and
        its .pgm), each car localized on that map drawn where it is on it.
    python3 scripts/v2v_view.py --broadcast 10.0.0.255 --autodrive 2
        also car 2 in AutoDRIVE: open the simulator's NeoRacer scene on this
        computer and press Connect. The simulated NeoRacer starts where it
        stands and moves as car 2 moves, through AutoDRIVE's co-simulation
        input, one state packet (a tenth of a second) behind so that it moves
        smoothly every frame; its Bird's Eye camera follows it from above.
    python3 scripts/v2v_view.py --broadcast 10.0.0.255 --lanes scripts/lanes/purdue_testbed.json
        each car locked onto its lane of a taped track (lane_lock): from its
        odometry alone, so the gyro's drift and the car's shaking cannot move
        it off its lane; found as it drives (a few metres and a bend), or
        from --start CAR:LANE:S. With --autodrive, each car is placed on its
        lane in the testbed scene (NeoRacer - Purdue Testbed), built from the
        same map.
    python3 scripts/v2v_view.py --broadcast 10.0.0.255 --autodrive 1 2
        cars 1 and 2 in the two-car scene (NeoRacer - AuE-X930 - Two Cars):
        car 1 drives AutoDRIVE's vehicle 1, car 2 vehicle 2. Each starts where
        its simulated car stands. Cars localized on the same map share it: the
        first one pins the map where its simulated car stands, and the other
        goes where the map puts it, so the two keep their real places.

--broadcast is the cars' subnet broadcast address (Windows sends
255.255.255.255 on one interface only). --autodrive needs the Socket.IO stack
AutoDRIVE's devkit pins: Python 3.8, python-socketio 4.2.0, python-engineio
3.13.0 and eventlet 0.33.3.

A car localized on a map (AMCL or slam_toolbox running on it) is drawn by its
map pose, so cars localized on the same map share one frame. A car that is
not is drawn by its odometry: it starts at 0, 0 where its driver started and
drifts slowly, and --place puts that start where it really stands
(CAR:X,Y,HEADING in the view's metres and degrees). Only one program on a
computer can listen for V2V (UDP 4747): not this and v2v_peer.py together.
"""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
import re
import secrets
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from neoracer_ros2_driver import v2v_lib as v2v  # noqa: E402,I100
from neoracer_ros2_driver.lane_lock import LaneLock, LaneMap  # noqa: E402

# The page, the Playground's top-down NeoRacer sprite, and the sprite's rear
# wing and front bumper as a paint layer the page tints in each car's colour.
FILES = {'/': ('v2v_view.html', 'text/html; charset=utf-8'),
         '/car.png': ('v2v_view_car.png', 'image/png'),
         '/car-paint.png': ('v2v_view_car_paint.png', 'image/png')}
HELLO_SEC = 1.0
# The path drawn behind a car: a point every TRAIL_STEP m, the last
# TRAIL_KEEP of them (75 m).
TRAIL_STEP = 0.05
TRAIL_KEEP = 1500
# AutoDRIVE's simulator connects to this port on the computer it runs on.
AUTODRIVE_PORT = 4567
# AutoDRIVE's NeoRacer reports V1 Position at its "Rear Axle Center"
# transform, this far above the vehicle origin that a co-simulation pose
# places (NeoRacer.prefab).
AUTODRIVE_GPS_HEIGHT = 0.0455
# The NeoRacer scenes hold one car or two. The simulator reads every key of
# each of its vehicles from every answer and stops for good at a missing one,
# and its first message carries no telemetry to count them by, so every answer
# covers two vehicles; a one-car scene ignores vehicle 2's keys.
SCENE_VEHICLES = 2
# The shade nav2's map_saver writes for an unknown cell of a trinary map.
MAP_SAVER_UNKNOWN = 205
# A binary PGM's header: P5, width, height, maxval, with # comments between,
# then one whitespace byte before the pixels.
PGM_HEADER = re.compile(rb'P5(?:\s|#[^\n]*\n)+(\d+)(?:\s|#[^\n]*\n)+(\d+)'
                        rb'(?:\s|#[^\n]*\n)+(\d+)\s')


def idle(vehicle):
    """Return the command that leaves this simulated vehicle where it stands."""
    v = 'V%d ' % vehicle
    return {v + 'Throttle': '0', v + 'Steering': '0', v + 'Reset': 'False', v + 'CoSim': '0'}


def stamp():
    """Return the wall-clock prefix for a printed line."""
    return time.strftime('%H:%M:%S')


def degrees(yaw):
    """Return a heading in degrees, -180 to 180."""
    return math.degrees(math.atan2(math.sin(yaw), math.cos(yaw)))


def drawn(car, now):
    """Return a car's pose one packet behind its latest, for every frame.

    A car sends its pose ten times a second; the simulator draws many more
    frames. Each frame shows the move from the pose before the latest to the
    latest, as far along as the time since the latest came in, over their
    spacing by the car's clock.
    """
    if car['before'] is None:
        return car['sent']
    (x0, y0, yaw0), t0 = car['before']
    x1, y1, yaw1 = car['sent']
    span = car['sent_t'] - t0
    # The car's clock stepped back (a time sync): show the latest.
    f = 1.0 if span <= 0 else min(1.0, (now - car['posed']) / span)
    turn = math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))
    return x0 + f * (x1 - x0), y0 + f * (y1 - y0), yaw0 + f * turn


def read_pgm(path):
    """Return (width, height, maxval, pixels) of a binary (P5) PGM, top row first."""
    with open(path, 'rb') as f:
        data = f.read()
    header = PGM_HEADER.match(data)
    if header is None:
        raise ValueError(f'{path} is not a binary PGM (P5), as map_saver writes')
    width, height, maxval = (int(g) for g in header.groups())
    if maxval > 255:
        raise ValueError(f'{path} has 16-bit pixels; map_saver writes 8-bit')
    pixels = data[header.end():header.end() + width * height]
    if len(pixels) != width * height:
        raise ValueError(f'{path} ends before its {width}x{height} pixels')
    return width, height, maxval, pixels


def read_map(path):
    """Read a saved ROS map (map_server's .yaml and its PGM) for the page.

    Return (meta, cells): meta is the size, resolution (m a cell) and origin
    (x, y, yaw of the bottom-left cell); cells hold one byte a cell, top row
    first: 0 unknown, 1 free, 2 occupied, by the file's thresholds as
    map_server classifies them, except map_saver's unknown shade. map_saver
    writes unknown as 205 (0.196 occupied), under its default free_thresh of
    0.25, so Nav2's loader counts it free; drawn free, the space beyond the
    walls the car never saw would look driveable, so it stays unknown here.
    """
    meta = {}
    with open(path) as f:
        for line in f:
            key, sep, value = line.split('#', 1)[0].partition(':')
            if sep:
                meta[key.strip()] = value.strip().strip('\'"')
    for key in ('image', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'):
        if key not in meta:
            raise ValueError(f'{path} has no {key}: is it a map_server map?')
    mode = meta.get('mode', 'trinary')
    if mode not in ('trinary', 'scale'):
        raise ValueError(f'{path}: mode {mode} maps are not drawn (trinary or scale only)')
    image = os.path.join(os.path.dirname(os.path.abspath(path)), meta['image'])
    width, height, maxval, pixels = read_pgm(image)
    occupied, free = float(meta['occupied_thresh']), float(meta['free_thresh'])
    negate = int(meta['negate'])
    table = bytearray(256)
    for v in range(maxval + 1):
        p = v / maxval if negate else (maxval - v) / maxval
        table[v] = 2 if p > occupied else 1 if p < free else 0
    if mode == 'trinary' and maxval == 255 and not negate:
        table[MAP_SAVER_UNKNOWN] = 0
    origin = [float(v) for v in meta['origin'].strip('[]').split(',')]
    return ({'width': width, 'height': height, 'resolution': float(meta['resolution']),
             'origin': origin, 'name': os.path.basename(path)},
            pixels.translate(bytes(table)))


def sim_start(data, vehicle=1):
    """Return (x, y, z, yaw) of one simulated NeoRacer from a telemetry message.

    Raise ValueError if the message does not carry that vehicle's Position
    (x y z) and Orientation Quaternion (x y z w) as numbers: a scene without
    that many NeoRacers, or a simulator writing decimal commas.
    """
    if not isinstance(data, dict):
        raise ValueError('telemetry is not a JSON object')
    position, orientation = 'V%d Position' % vehicle, 'V%d Orientation Quaternion' % vehicle
    for key, count in ((position, 3), (orientation, 4)):
        if not isinstance(data.get(key), str) or len(data[key].split()) != count:
            raise ValueError('telemetry has no %s of %d numbers: has this scene a NeoRacer as '
                             'vehicle %d?' % (key, count, vehicle))
    try:
        x, y, z = (float(v) for v in data[position].split())
        quaternion = [float(v) for v in data[orientation].split()]
    except ValueError:
        raise ValueError('telemetry %s "%s" is not numbers (decimal commas?)'
                         % (position, data[position])) from None
    return x, y, z, v2v.yaw_from_quaternion(*quaternion)


class Fleet:
    """The latest state of every car heard, and the path each has driven."""

    def __init__(self, places, lanes=None, lane_scale=1.0, starts=None):
        self.lock = threading.Lock()
        self.cars = {}
        self.places = places
        # With a lane map, each car's odometry is locked onto its lanes
        # (lane_lock): from where it is known to start, or found as it drives.
        self.lanes = lanes
        self.lane_scale = lane_scale
        self.starts = {} if starts is None else starts

    def update(self, packet, addr, now):
        """Record one state packet; one with no pose keeps the last pose.

        With a lane map, a car's pose is where its lane lock puts it on its
        lane: frame 'lane' once locked, 'searching' (its best guess) before.
        Without, its map pose while it is localized, else its odometry
        (placed by --place): frame 'map' or 'odom'.
        """
        with self.lock:
            car = self.cars.setdefault(packet['id'], {'trail': [], 'sent': None, 'pose': None,
                                                      'before': None, 'bsm': None,
                                                      'frame': None, 'run': None})
            last = car['bsm']
            if (last is not None and last['epoch'] == packet['epoch']
                    and packet['seq'] <= last['seq']):
                # Late or repeated: the car has already sent newer state.
                return
            car['addr'] = addr
            car['seen'] = now
            car['bsm'] = packet
            if self.lanes is not None:
                if None in (packet['x'], packet['y'], packet['yaw']):
                    return
                odom = (packet['x'], packet['y'], packet['yaw'], packet['t'])
                if 'lock' not in car:
                    car['lock'] = LaneLock(self.lanes, scale=self.lane_scale,
                                           start=self.starts.get(packet['id']), seed=packet['id'])
                elif packet['epoch'] != car['lock_epoch']:
                    # its V2V restarted: its odometry may start again from 0
                    car['lock'].rebase(*odom)
                car['lock_epoch'] = packet['epoch']
                est = car['lock'].update(*odom)
                if est is None:
                    return
                car['lane'] = est
                frame = 'lane' if est['locked'] else 'searching'
                sent = (est['x'], est['y'], est['heading'])
            elif None not in (packet['map_x'], packet['map_y'], packet['map_yaw']):
                frame, sent = 'map', (packet['map_x'], packet['map_y'], packet['map_yaw'])
            elif None not in (packet['x'], packet['y'], packet['yaw']):
                frame, sent = 'odom', (packet['x'], packet['y'], packet['yaw'])
            else:
                return
            if frame != car['frame'] or packet['epoch'] != car['run']:
                # Localized, or lost, or the car's V2V restarted (a new epoch, its
                # odometry perhaps from 0 again): positions in another frame, so a
                # new path, and no move to smooth across from the old one.
                car['frame'], car['before'], car['trail'] = frame, None, []
            else:
                # From where the car is drawn now, not from the pose before: a
                # packet that comes early would otherwise jump the drawn car forward.
                car['before'] = (drawn(car, now), car['sent_t'])
            car['run'] = packet['epoch']
            car['sent'], car['sent_t'] = sent, packet['t']
            car['pose'] = sent if frame != 'odom' else v2v.compose(
                self.places.get(packet['id'], (0.0, 0.0, 0.0)), *sent)
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
                    'frame': car['frame'],
                    'speed': bsm['speed'],
                    'yaw_rate': bsm['yaw_rate'],
                    'cmd_speed': bsm['cmd_speed'],
                    'cmd_steer': bsm['cmd_steer'],
                    'batt': bsm['batt'],
                    # A copy: the page's thread encodes it after the lock is let go.
                    'trail': list(car['trail']),
                }
                if 'lane' in car:
                    est = car['lane']
                    out[str(pid)]['lane'] = {'id': est['lane'],
                                             'confidence': round(est['confidence'], 2),
                                             'scale': round(est['scale'], 3)}
            return out


class MapFrame:
    """Where the map lies in the simulator, shared by every car projected into it.

    The first car to send a pose on the map pins it: that pose onto where its
    simulated car stands, so that car goes on without a jump. A car localized
    on the same map afterwards goes where the map puts it, so cars on one map
    keep their real places to each other in the simulator.
    """

    def __init__(self):
        self.anchor = None  # (car pose on the map, simulator pose)


class Projector:
    """One car's pose (on the map while localized) as co-simulation poses for one vehicle."""

    def __init__(self, fleet, car_id, at, vehicle=1, map_frame=None):
        self.fleet = fleet
        self.car_id = car_id
        self.at = at
        self.vehicle = vehicle
        self.map_frame = MapFrame() if map_frame is None else map_frame
        # The simulated car's start (x, y, z, yaw in the simulator's frame:
        # x forward along the scene, y left, z up).
        self.sim_origin = None
        # (frame, run, car pose, sim pose): a pose of the car and where it
        # maps to in the simulator. In odometry, per run (the car's V2V
        # epoch): the car's first pose pins to the start, and a new run or a
        # map lost pins to where the simulated car stands, so it goes on from
        # there without a jump. On the map, the shared MapFrame's.
        self.anchor = None
        self.last = None
        self.said = 0.0

    def standing(self):
        """Return where the simulated car stands: its last pose sent, else its start."""
        return self.sim_origin if self.last is None else self.last

    def reply(self, data):
        """Return this vehicle's command answering one telemetry message from the simulator.

        Raise ValueError if the first telemetry does not carry this vehicle's
        pose (see sim_start).
        """
        if self.sim_origin is None:
            x, y, z, yaw = sim_start(data, self.vehicle)
            if self.at is not None:
                x, y, yaw = self.at
            self.sim_origin = (x, y, z - AUTODRIVE_GPS_HEIGHT, yaw)
            print('%s AutoDRIVE: vehicle %d starts at x %.2f y %.2f heading %.0f deg'
                  % (stamp(), self.vehicle, x, y, degrees(yaw)))
        with self.fleet.lock:
            car = self.fleet.cars.get(self.car_id)
            pose = frame = run = None
            if car is not None and car['sent'] is not None:
                pose, frame, run = drawn(car, time.monotonic()), car['frame'], car['run']
        command = idle(self.vehicle)
        if pose is not None and frame == 'lane':
            # The testbed scene is built in the lane map's own frame: the car
            # goes where its lane puts it, on the ground its vehicle started on.
            self.last = (pose[0], pose[1], self.sim_origin[2], pose[2])
        elif pose is not None and frame != 'searching':
            key = ('map', None) if frame == 'map' else (frame, run)
            if self.anchor is None or self.anchor[:2] != key:
                if frame == 'map':
                    if self.map_frame.anchor is None:
                        self.map_frame.anchor = (pose, self.standing())
                    held, sim = self.map_frame.anchor
                else:
                    held, sim = pose, self.standing()
                self.anchor = key + (held, sim)
                x, y, _, yaw = self.project(pose)
                print('%s AutoDRIVE: car %d at x %.2f y %.2f heading %.0f deg (%s) -> vehicle %d '
                      'at x %.2f y %.2f heading %.0f deg'
                      % (stamp(), self.car_id, pose[0], pose[1], degrees(pose[2]),
                         'on the map' if frame == 'map' else 'odometry', self.vehicle,
                         x, y, degrees(yaw)))
            self.last = self.project(pose)
        if self.last is not None:
            x, y, z, yaw = self.last
            qx, qy, qz, qw = v2v.quaternion_from_yaw(yaw)
            v = 'V%d ' % self.vehicle
            command.update({v + 'CoSim': '1',
                            v + 'PosX': '%.4f' % x, v + 'PosY': '%.4f' % y, v + 'PosZ': '%.4f' % z,
                            v + 'RotX': '%.6f' % qx, v + 'RotY': '%.6f' % qy,
                            v + 'RotZ': '%.6f' % qz, v + 'RotW': '%.6f' % qw})
            now = time.monotonic()
            if now - self.said >= 5.0:
                self.said = now
                print('%s AutoDRIVE: car %d -> vehicle %d x %.2f y %.2f heading %.0f deg'
                      % (stamp(), self.car_id, self.vehicle, x, y, degrees(yaw)))
        return command

    def project(self, pose):
        """Map a pose of the car onto the simulator: its move since the anchor's."""
        _, _, (cx, cy, cyaw), (sx, sy, sz, syaw) = self.anchor
        c, s = math.cos(-cyaw), math.sin(-cyaw)
        dx, dy = pose[0] - cx, pose[1] - cy
        rx, ry = dx * c - dy * s, dx * s + dy * c
        c, s = math.cos(syaw), math.sin(syaw)
        return sx + rx * c - ry * s, sy + rx * s + ry * c, sz, syaw + pose[2] - cyaw


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


def serve_page(fleet, port, saved_map, lanes_json):
    """Serve the bird's-eye page, the cars' state and the maps (if any) on this port."""
    here = os.path.dirname(os.path.abspath(__file__))
    files = {}
    for path, (name, kind) in FILES.items():
        with open(os.path.join(here, name), 'rb') as f:
            files[path] = (f.read(), kind)
    if saved_map is not None:
        meta, cells = saved_map
        files['/map.json'] = (json.dumps(meta).encode('utf-8'), 'application/json')
        files['/map.raw'] = (cells, 'application/octet-stream')
    if lanes_json is not None:
        files['/lanes.json'] = (lanes_json, 'application/json')

    class Handler(BaseHTTPRequestHandler):

        def do_GET(self):
            if self.path in files:
                body, kind = files[self.path]
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


def answer(data, projectors, refused):
    """Return the answer to one message from the simulator: a command for every vehicle.

    The first message after connecting carries no telemetry: every vehicle
    idles where it stands. Every message is answered, since the simulator
    sends its next telemetry only on an answer. Telemetry a car cannot use
    is reported once by reason (kept in refused) and its vehicle idles.
    """
    reply = {}
    for vehicle in range(1, SCENE_VEHICLES + 1):
        reply.update(idle(vehicle))
    if data:
        for projector in projectors:
            try:
                reply.update(projector.reply(data))
            except ValueError as e:
                if str(e) not in refused:
                    refused.append(str(e))
                    print('%s AutoDRIVE: %s; vehicle %d stays where it is'
                          % (stamp(), e, projector.vehicle))
    return reply


def serve_autodrive(projectors):
    """Answer AutoDRIVE's simulator on its port: one pose per car per telemetry message."""
    import eventlet
    import socketio

    sio = socketio.Server()
    refused = []

    @sio.on('connect')
    def connect(sid, environ):
        print('%s AutoDRIVE: simulator connected' % stamp())

    @sio.on('Bridge')
    def bridge(sid, data):
        sio.emit('Bridge', data=answer(data, projectors, refused), room=sid)

    print('%s AutoDRIVE: waiting for the simulator on port %d (press Connect in it)'
          % (stamp(), AUTODRIVE_PORT))
    eventlet.wsgi.server(eventlet.listen(('', AUTODRIVE_PORT)), socketio.WSGIApp(sio),
                         log_output=False)


def parse_start(text):
    """Parse CAR:LANE:S (a lane map's lane id, metres along it) into (car, (lane, s))."""
    car, lane, s = text.split(':')
    return int(car), (lane, float(s))


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
    parser.add_argument('--map', metavar='YAML',
                        help='a saved map (map_server .yaml with its .pgm) under the cars')
    parser.add_argument('--autodrive', type=int, nargs='+', metavar='CAR',
                        help="drive AutoDRIVE's NeoRacer with this car, or vehicles 1 and 2 of "
                             'the two-car scene with two cars')
    parser.add_argument('--at', type=float, nargs=3, metavar=('X', 'Y', 'HEADING'),
                        help='one simulated car starts here, not where it stands '
                             "(the simulator's frame: x forward, y left, metres; degrees)")
    parser.add_argument('--lanes', metavar='JSON',
                        help='a lane map (scripts/lanes/*.json): each car locked onto its lane '
                             'from its odometry, drawn on the track, placed on it in AutoDRIVE')
    parser.add_argument('--start', action='append', default=[], type=parse_start,
                        help="a car's known start on the lane map: CAR:LANE:S (repeatable); "
                             'without one, the lock finds the car as it drives')
    parser.add_argument('--lane-scale', type=float, default=1.0,
                        help="the lane map's metres per metre of the cars' odometry "
                             '(1 for a map measured true; the lock learns 0.8 to 1.25 of it)')
    args = parser.parse_args()
    if not 1 <= args.id <= v2v.MAX_ID:
        parser.error('ids are 1..%d' % v2v.MAX_ID)
    cars = [c for c, _ in args.place]
    if args.autodrive is not None:
        if len(args.autodrive) > SCENE_VEHICLES or len(set(args.autodrive)) < len(args.autodrive):
            parser.error('--autodrive takes one car or two different cars')
        if args.at is not None and len(args.autodrive) > 1:
            parser.error('--at places one car; with two, each starts where its vehicle stands')
        cars += args.autodrive
    for car in cars:
        if not 1 <= car <= v2v.MAX_ID or car == args.id:
            parser.error("car %d: cars are 1..%d, and not the observer's own id %d"
                         % (car, v2v.MAX_ID, args.id))
    if args.key is not None and not re.fullmatch('[0-9a-fA-F]{64}', args.key):
        parser.error('--key is the fleet key: 64 hex characters (racecar v2v key show)')
    lanes = lanes_json = None
    if args.lanes is not None:
        if args.map is not None or args.at is not None or args.place:
            parser.error('--lanes places the cars on their lanes: not with --map, --at or --place')
        try:
            with open(args.lanes, 'rb') as f:
                lanes_json = f.read()
            lanes = LaneMap(json.loads(lanes_json))
        except (OSError, ValueError, KeyError) as e:
            parser.error('%s is not a lane map: %s' % (args.lanes, e))
        for car, (lane, s) in args.start:
            if lane not in lanes.by_id:
                parser.error('--start %d: the map has no lane %s' % (car, lane))
        print('%s lanes: %s, %d lanes, %.1f m of lane, odometry scale %.2f'
              % (stamp(), lanes.data['name'], len(lanes.lanes), lanes.total, args.lane_scale))
    elif args.start:
        parser.error('--start needs --lanes')

    saved_map = None
    if args.map is not None:
        try:
            saved_map = read_map(args.map)
        except (OSError, ValueError) as e:
            parser.error(str(e))
        meta = saved_map[0]
        print('%s map: %s, %d x %d cells of %.3f m, origin x %.2f y %.2f'
              % (stamp(), meta['name'], meta['width'], meta['height'], meta['resolution'],
                 meta['origin'][0], meta['origin'][1]))
    fleet = Fleet(dict(args.place), lanes, args.lane_scale, dict(args.start))
    observer = Observer(args, fleet)
    threading.Thread(target=observer.listen, daemon=True).start()
    threading.Thread(target=observer.hello, daemon=True).start()
    threading.Thread(target=serve_page, args=(fleet, args.http, saved_map, lanes_json),
                     daemon=True).start()
    print("%s bird's-eye view: http://localhost:%d (observer id %d, hellos to %s)"
          % (stamp(), args.http, args.id, ', '.join(observer.targets)))
    if args.autodrive is None:
        while True:
            time.sleep(1.0)
    at = None
    if args.at is not None:
        at = (args.at[0], args.at[1], math.radians(args.at[2]))
    shared = MapFrame()
    serve_autodrive([Projector(fleet, car, at, vehicle, shared)
                     for vehicle, car in enumerate(args.autodrive, start=1)])


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
