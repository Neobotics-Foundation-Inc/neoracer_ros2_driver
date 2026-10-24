#!/usr/bin/env python3

"""
Several NeoRacers in AutoDRIVE, each in its own ROS domain, as real cars are.

    python3 scripts/autodrive_cars.py --cars 2

Car N is in ROS domain N, the domain `racecar v2v id N` gives a real car, with
the same nodes a car's twin has in one domain: the driver's mux, throttle and
gamepad nodes, and the AutoDRIVE bridge node (autodrive_neoracer's
autodrive_bridge, unchanged). So each car has its own /drive, /odom,
/imu/fused, /scan, /camera/color and /encoder/speed, and a program drives a
car by its domain:

    ROS_DOMAIN_ID=2 python3 lane_follow.py

The simulator makes one connection (Socket.IO, port 4567) and sends every car's
telemetry in one message, its keys numbered by car (V1, V2, ...). This script
is that connection's server: it hands each car's telemetry to that car's
bridge node, renumbered as car 1's (the bridge reads and writes only V1 keys),
and answers the simulator with every car's latest commands, as the bridge
answers with its own. Each bridge node runs in a process of its own, in its
car's domain. A car's bridge still busy with the last frame skips the new one,
as a car's camera drops frames it cannot keep up with.

The simulated car holds the commanded speed itself, as a car's firmware does
(NeoRacerController, AutoDRIVE branch purdue-cosim), so each bridge's own speed
controller is set to pass the speed through (speed_ctrl_kP, kI and kD 0).

Needs the workspace with neoracer_ros2_driver and autodrive_neoracer built
and sourced, and the bridge's Python packages (python-socketio 4.2.0,
python-engineio 3.13.0, gevent, gevent-websocket). One server per port: stop
any other AutoDRIVE bridge first.
"""

import argparse
import multiprocessing
import os
import re
import signal
import subprocess
import sys
import threading
import traceback

KEY = re.compile(r'^V(\d+) (.*)$')
DRIVER_NODES = ('mux_node', 'throttle_node', 'gamepad_node')
PASS_SPEED = ['--ros-args', '-p', 'speed_ctrl_kP:=0.0', '-p', 'speed_ctrl_kI:=0.0',
              '-p', 'speed_ctrl_kD:=0.0']


def bridge_process(car, conn, command):
    """
    Run car `car`'s bridge node in ROS domain `car`.

    Each telemetry received on `conn` (renumbered as car 1's) is published,
    then acknowledged. The bridge's throttle and steering go into `command`
    as each /motor arrives, for the hub's next answer to the simulator, as
    autodrive_bridge answers with its latest.
    """
    import rclpy
    rclpy.init(domain_id=car, args=PASS_SPEED)
    from autodrive_neoracer.autodrive_bridge import AutoDRIVE_Bridge

    class Bridge(AutoDRIVE_Bridge):
        def callback_motor(self, msg):
            super().callback_motor(msg)
            command[:] = (self.throttle_cmd, self.steering_cmd)

    node = Bridge()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    while True:
        try:
            data = conn.recv()
        except EOFError:  # the hub stopped
            return
        try:
            node.publish_telemetry(data)
        except Exception:
            # as autodrive_bridge does: the simulator's first frames lack the LiDAR's ranges
            node.get_logger().error('AutoDRIVE Telemetry Failed:\n' + traceback.format_exc())
        conn.send(True)


def driver_nodes(car):
    """
    Start car `car`'s mux, throttle and gamepad nodes in ROS domain `car`.

    They start as autodrive_neoracer's launch starts them.
    """
    from ament_index_python.packages import get_package_prefix, get_package_share_directory
    lib = os.path.join(get_package_prefix('neoracer_ros2_driver'), 'lib', 'neoracer_ros2_driver')
    config = os.path.join(get_package_share_directory('neoracer_ros2_driver'), 'config')
    env = dict(os.environ, ROS_DOMAIN_ID=str(car))
    return [subprocess.Popen([os.path.join(lib, name), '--ros-args', '-r', '__node:=' + name,
                              '--params-file', os.path.join(config, name.split('_')[0] + '.yaml')],
                             env=env)
            for name in DRIVER_NODES]


def split(data, cars):
    """Each car's telemetry, its keys renumbered as car 1's; keys of no car go to every car."""
    out = {car: {} for car in cars}
    for key, value in data.items():
        m = KEY.match(key)
        if m is None:
            for car in cars:
                out[car][key] = value
        elif int(m.group(1)) in out:
            out[int(m.group(1))]['V1 ' + m.group(2)] = value
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cars', type=int, default=1,
                    help='how many cars, numbered 1 to N (default 1)')
    ap.add_argument('--port', type=int, default=4567, help='the port the simulator connects to')
    args = ap.parse_args()
    if not 1 <= args.cars <= 101:
        sys.exit('--cars must be 1 to 101: a car is a ROS domain, and DDS maps domains 1-101')

    import socketio
    from gevent import pywsgi
    from geventwebsocket.handler import WebSocketHandler

    cars = list(range(1, args.cars + 1))
    spawn = multiprocessing.get_context('spawn')
    links, commands, processes = {}, {}, []
    for car in cars:
        here, there = spawn.Pipe()
        commands[car] = spawn.Array('d', 2)
        p = spawn.Process(target=bridge_process, args=(car, there, commands[car]))
        p.start()
        links[car] = here
        processes.append(p)
        processes.extend(driver_nodes(car))
    busy = {car: False for car in cars}
    seen = {'scene': None}

    def stop(*_):
        for p in processes:
            p.terminate()
        os._exit(0)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    sio = socketio.Server(async_mode='gevent')

    @sio.on('connect')
    def on_connect(sid, environ):
        print('Simulator: Connected', flush=True)

    @sio.on('Bridge')
    def on_bridge(sid, data):
        for car in cars:
            while links[car].poll():
                links[car].recv()
                busy[car] = False
        scene = cars
        if data:
            scene = sorted({int(m.group(1)) for m in map(KEY.match, data) if m})
            if scene != seen['scene']:
                seen['scene'] = scene
                print('Simulator: cars %s; bridged: %s%s' % (
                    scene, cars, '' if set(scene) == set(cars) else
                    ' (cars without a bridge get no commands;'
                    ' bridges without a car get no telemetry)'),
                    flush=True)
            for car, telemetry in split(data, cars).items():
                if car in scene and not busy[car]:
                    links[car].send(telemetry)
                    busy[car] = True
        reply = {}
        for car in scene:
            throttle, steering = commands[car][:] if car in commands else (0.0, 0.0)
            reply['V%d Throttle' % car] = str(throttle)
            reply['V%d Steering' % car] = str(steering)
            reply['V%d Reset' % car] = 'False'
            reply['V%d CoSim' % car] = '0'
        sio.emit('Bridge', data=reply)

    print('Bridging cars %s on port %d, car N in ROS domain N' % (cars, args.port), flush=True)
    pywsgi.WSGIServer(('', args.port), socketio.WSGIApp(sio), handler_class=WebSocketHandler,
                      log=None).serve_forever()


if __name__ == '__main__':
    main()
