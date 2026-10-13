#!/usr/bin/env python3

"""
Command-line harness for V2V: what `racecar v2v status|ping|send|recv|id` runs.

Talks to this car's v2v_node over its ROS topics, so the shell must be in the
car's ROS domain (~/.neoracer/env, sourced by ~/.bashrc). Exit status is 0
when the command did what it says and 1 when the node is down, the peer is
absent, or a ping got no answer.
"""

import argparse
import json
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Int32, String


def num(value, fmt, width):
    """Format a number, or a dash of the same width for a null reading."""
    if value is None:
        return '-'.rjust(width)
    return format(value, fmt).rjust(width)


def table(status):
    """Render one /v2v/peers message as text."""
    ifaces = ' '.join(f'{name}={addr}' for name, addr in status['ifaces']) or 'no interface'
    m = status['msgs']
    r = status['rejected']
    signing = f"ON key {status['signing']}" if status['signing'] else 'off'
    lines = [f"car {status['id']}  state sent {status['seq']}  rx {status['rx']}  "
             f"tx_err {status['tx_err']}  {ifaces}",
             f"  signing {signing}  rejected {status['bad']}: malformed {r['malformed']}  "
             f"unsigned {r['unsigned']}  forged {r['forged']}  replay {r['replay']}  "
             f"other {r['other']}"
             + ("  (forged from a fleet car usually means a different key: compare "
                "`racecar v2v key show` on both)" if r['forged'] else ''),
             f"  messages: in {m['in']}  delivered {m['delivered']}  pending {m['pending']}  "
             f"undelivered {m['undelivered']}  retransmits {m['retx']}  "
             f"heard: {' '.join(status['heard']) or 'none'}"]
    peers = status['peers']
    if not peers:
        lines.append('  no peers')
        return '\n'.join(lines)
    lines.append(f"  {'car':>3} {'age s':>6} {'addr':<15} {'rx':>6} {'lost':>5} "
                 f"{'x':>7} {'y':>7} {'yaw deg':>7} {'m/s':>6} "
                 f"{'cmd v':>6} {'cmd s':>6} {'batt':>5}")
    for pid, p in peers.items():
        yaw = None if p['yaw'] is None else math.degrees(p['yaw'])
        lines.append(
            f"  {pid:>3} {p['age']:6.2f} {p['addr']:<15} {p['rx']:>6} {p['lost']:>5} "
            f"{num(p['x'], '.3f', 7)} {num(p['y'], '.3f', 7)} {num(yaw, '.1f', 7)} "
            f"{num(p['speed'], '.2f', 6)} {num(p['cmd_speed'], '.2f', 6)} "
            f"{num(p['cmd_steer'], '.2f', 6)} {num(p['batt'], '.2f', 5)}")
    return '\n'.join(lines)


class V2VClient(Node):
    """One-shot client for /v2v/peers, /v2v/recv, /v2v/send and /v2v/ping."""

    def __init__(self):
        super().__init__('v2v_tool')
        self.status = None
        self.events = []
        self.create_subscription(String, '/v2v/peers', self._on_peers, 10)
        # Deep queues on the message topics, matching v2v_node's: keep-last
        # drops the oldest sample when one side outruns the other.
        self.create_subscription(String, '/v2v/recv', self._on_recv, 1000)
        self.send_pub = self.create_publisher(String, '/v2v/send', 1000)
        self.ping_pub = self.create_publisher(Int32, '/v2v/ping', 10)

    def _on_peers(self, msg):
        self.status = json.loads(msg.data)

    def _on_recv(self, msg):
        self.events.append(json.loads(msg.data))

    def spin_for(self, seconds):
        """Process callbacks for a fixed time."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def spin_until(self, done, timeout):
        """Process callbacks until done() holds; False on timeout."""
        end = time.monotonic() + timeout
        while not done():
            if time.monotonic() > end:
                return False
            rclpy.spin_once(self, timeout_sec=0.05)
        return True

    def require_node(self):
        """Wait for the first peer table; exit 1 when v2v_node is not up."""
        if not self.spin_until(lambda: self.status is not None, 3.0):
            sys.exit('v2v_node is not running on this car: racecar service start v2v')

    def _discovered(self):
        return (self.send_pub.get_subscription_count() > 0
                and self.ping_pub.get_subscription_count() > 0)

    def require_publishers(self):
        """Wait for the node to discover this client's publishers."""
        if not self.spin_until(self._discovered, 3.0):
            sys.exit('v2v_node did not discover this client; is it running in this domain?')


def cmd_status(client, args):
    client.require_node()
    print(table(client.status))
    if not args.watch:
        return
    while True:
        client.spin_for(1.0)
        print()
        print(table(client.status))


def cmd_ping(client, args):
    client.require_node()
    client.require_publishers()
    if str(args.car) not in client.status['heard']:
        known = ', '.join(client.status['heard']) or 'none'
        sys.exit(f'car {args.car} has not been heard in the last 3 s (heard: {known})')
    for _ in range(args.count):
        client.ping_pub.publish(Int32(data=args.car))
        client.spin_for(args.interval)
    client.spin_for(1.0)
    rtts = [e['rtt_ms'] for e in client.events if e['type'] == 'pong' and e['from'] == args.car]
    loss = 100.0 * (1.0 - len(rtts) / args.count)
    print(f'car {args.car}: {args.count} sent, {len(rtts)} received, {loss:.0f}% loss')
    if not rtts:
        sys.exit(1)
    print(f'rtt min/avg/max {min(rtts):.1f}/{sum(rtts) / len(rtts):.1f}/{max(rtts):.1f} ms')


def cmd_send(client, args):
    client.require_node()
    client.require_publishers()
    expect = {int(pid) for pid in client.status['peers']}
    if not expect:
        sys.exit('no car is sending state; nothing to send to')
    text = ' '.join(args.text)
    client.send_pub.publish(String(data=text))
    reports = {}
    unsent = []

    def settled():
        for event in client.events:
            if event.get('text') != text:
                continue
            if event['type'] in ('delivered', 'undelivered'):
                reports[event['to']] = event
            elif event['type'] == 'unsent':
                unsent.append(event['reason'])
        client.events.clear()
        return bool(unsent) or expect <= set(reports)

    client.spin_until(settled, args.wait)
    if unsent:
        sys.exit(f'not sent: {unsent[0]}')
    ok = True
    for pid in sorted(expect | set(reports)):
        event = reports.get(pid)
        if event is None:
            print(f'car {pid}: no report within {args.wait:g} s')
            ok = False
        elif event['type'] == 'delivered':
            print(f"car {pid}: delivered in {event['ms']:.1f} ms (seq {event['seq']})")
        else:
            print(f"car {pid}: NOT delivered, no ack (seq {event['seq']})")
            ok = False
    if not ok:
        sys.exit(1)


def cmd_recv(client, args):
    client.require_node()
    print('listening on /v2v/recv (Ctrl-C to stop)')
    while True:
        client.spin_for(0.2)
        for event in client.events:
            kind = event['type']
            if kind == 'msg':
                print(f"car {event['from']}: {event['text']}")
            elif kind == 'pong':
                print(f"pong from car {event['from']}: {event['rtt_ms']} ms")
            elif kind == 'delivered':
                print(f"  delivered to car {event['to']} in {event['ms']} ms: {event['text']}")
            elif kind == 'undelivered':
                print(f"  NOT delivered to car {event['to']}: {event['text']}")
            else:
                print(f"  not sent ({event['reason']}): {event['text']}")
        client.events.clear()


def cmd_id(client, args):
    car_id = os.environ.get('NEORACER_CAR_ID')
    domain = os.environ.get('ROS_DOMAIN_ID')
    if car_id is None:
        sys.exit('this car has no id yet: racecar v2v id N')
    print(f'NEORACER_CAR_ID={car_id} ROS_DOMAIN_ID={domain}')


def main():
    # `recv` and `status --watch` are read live, often through a pipe.
    sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(prog='racecar v2v', description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('status', help='peer table (once; --watch to keep printing)')
    p.add_argument('--watch', '-w', action='store_true')
    p.set_defaults(run=cmd_status)
    p = sub.add_parser('ping', help='round trip to one car through its v2v_node')
    p.add_argument('car', type=int)
    p.add_argument('--count', '-c', type=int, default=20)
    p.add_argument('--interval', '-i', type=float, default=0.1)
    p.set_defaults(run=cmd_ping)
    p = sub.add_parser('send', help='text message to every car, with a delivery report per car')
    p.add_argument('text', nargs='+')
    p.add_argument('--wait', type=float, default=12.0,
                   help='seconds to wait for the reports (node gives up at 10)')
    p.set_defaults(run=cmd_send)
    p = sub.add_parser('recv', help='print messages and pongs as they arrive')
    p.set_defaults(run=cmd_recv)
    p = sub.add_parser('id', help="this car's id and ROS domain")
    p.set_defaults(run=cmd_id)
    args = parser.parse_args()

    if args.command == 'id':
        cmd_id(None, args)
        return
    # rclpy's handlers shut the context down under a running spin, which then
    # fails in whichever call it is in. Python's defaults are what a CLI wants:
    # Ctrl-C raises KeyboardInterrupt here, SIGTERM (timeout, kill) just ends.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    client = V2VClient()
    try:
        args.run(client, args)
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
