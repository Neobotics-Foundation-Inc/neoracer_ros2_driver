#!/usr/bin/env python3

"""
Watch the V2V channel from a laptop, or join it as a virtual car.

Run it from a clone of neoracer_ros2_driver: it imports the package's
v2v_lib, which needs nothing beyond the Python standard library. Windows,
macOS, or Linux, on the network the cars are on.

    python3 scripts/v2v_peer.py --watch 90 --broadcast 10.0.0.255
        observe as id 90: the cars send their state to it as to any peer
        that says hello. It sends no state and takes no messages, so no
        car lists it in its peer table.
    python3 scripts/v2v_peer.py --as-car 9 --broadcast 10.0.0.255
        join as car 9: state at 10 Hz, pings answered, messages acked, and
        each typed line sent to every car with a delivery report.
    python3 scripts/v2v_peer.py
        passive: prints what is broadcast, which is the hellos only; the
        state and the messages are unicast between cars.

--broadcast is the broadcast address of a subnet the cars are on, once per
subnet. Without it the script uses 255.255.255.255, which Windows sends on
one interface only.

A testbed uses the same script as a roadside unit, a simulator injecting
vehicles, or the attacker: --as-car with a number a real car holds, and every
car logs the duplicate.
"""

import argparse
import os
import secrets
import socket
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from neoracer_ros2_driver import v2v_lib as v2v  # noqa: E402,I100

STATE_HZ = 10.0
HELLO_HZ = 1.0
RETRY_SEC = 0.1
GIVE_UP_SEC = 10.0
ADDR_TIMEOUT_SEC = 3.0
CAR_TIMEOUT_SEC = 1.0


def stamp():
    """Wall-clock prefix for a printed line."""
    return time.strftime('%H:%M:%S')


def summary(packet):
    """Render one validated packet as a single line."""
    head = f"{packet['type']:<5} car {packet['id']} seq {packet['seq']}"
    if packet['type'] == 'bsm':
        return (f"{head} x {packet['x']} y {packet['y']} yaw {packet['yaw']} "
                f"v {packet['speed']} cmd {packet['cmd_speed']}/{packet['cmd_steer']} "
                f"batt {packet['batt']}")
    if packet['type'] == 'msg':
        return f"{head} to {packet['to']}: {packet['text']}"
    if packet['type'] == 'ack':
        return f"{head} to {packet['to']}"
    if packet['type'] in ('ping', 'pong'):
        return f"{head} t_mono {packet['t_mono']}"
    return head


class Peer:
    """One laptop on the V2V network: passive, observer, or virtual car."""

    def __init__(self, args):
        self.port = args.port
        self.targets = args.broadcast or ['255.255.255.255']
        self.my_id = args.as_car or args.watch
        self.is_car = args.as_car is not None
        self.quiet_bsm = args.quiet_bsm
        self.key = bytes.fromhex(args.key) if args.key else None
        self.epoch = secrets.randbits(32)
        self.lock = threading.Lock()
        self.book = v2v.AddressBook()
        self.cars = {}
        self.guard = v2v.ReplayGuard()
        self.receiver = v2v.MessageReceiver()
        self.sender = v2v.MessageSender(self.my_id or 0, self.epoch, RETRY_SEC, GIVE_UP_SEC)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind(('', self.port))

    def send(self, packet, addr):
        payload = v2v.encode(packet)
        if self.key is not None:
            payload = v2v.sign(payload, self.key)
        self.sock.sendto(payload, (addr, self.port))

    def listen(self):
        """Print what arrives; answer pings, ack messages, apply acks."""
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
                    print(f'{stamp()} {addr:<15} rejected: {e}')
                    continue
            try:
                packet = v2v.parse_packet(data)
            except ValueError as e:
                print(f'{stamp()} {addr:<15} bad packet: {e}')
                continue
            pid = packet['id']
            if pid == self.my_id:
                continue
            # Replay protection only with signing; see v2v_node.
            if self.key is not None:
                try:
                    self.guard.check(packet)
                except ValueError as e:
                    print(f'{stamp()} {addr:<15} rejected: {e}')
                    continue
            now = time.monotonic()
            with self.lock:
                if self.book.heard(pid, addr, now):
                    print(f'{stamp()} {addr:<15} car {pid} heard')
                if packet['type'] == 'bsm':
                    self.cars[pid] = now
            self.handle(packet, addr, now)

    def handle(self, packet, addr, now):
        ptype = packet['type']
        if not (self.quiet_bsm and ptype in ('bsm', 'hello')):
            print(f'{stamp()} {addr:<15} {summary(packet)}')
        if self.my_id is None:
            return
        if ptype == 'ping':
            self.send({'ver': v2v.PROTOCOL_VERSION, 'type': 'pong', 'id': self.my_id,
                       'epoch': self.epoch, 'seq': packet['seq'],
                       't_mono': packet['t_mono']}, addr)
        elif ptype == 'msg' and packet['to'] == self.my_id:
            try:
                ready, acks = self.receiver.accept(packet)
            except ValueError as e:
                print(f'{stamp()} {addr:<15} bad message: {e}')
                return
            for msg in acks:
                self.send(v2v.ack_for(msg, self.my_id, self.epoch), addr)
            for msg in ready:
                print(f"{stamp()} message from car {msg['id']}: {msg['text']}")
        elif ptype == 'ack':
            with self.lock:
                result, msg, latency = self.sender.ack(packet, now)
            if result == 'delivered':
                print(f"{stamp()} delivered to car {packet['id']} in "
                      f"{latency * 1000.0:.1f} ms: {msg['text']}")
            elif result == 'unknown':
                print(f'{stamp()} {addr:<15} ack answers no message of ours')

    def run(self):
        """Hellos, state, and retransmits on their own clocks."""
        state_seq = hello_seq = 0
        next_state = next_hello = time.monotonic()
        while True:
            now = time.monotonic()
            if now >= next_hello:
                for target in self.targets:
                    self.send({'ver': v2v.PROTOCOL_VERSION, 'type': 'hello',
                               'id': self.my_id, 'epoch': self.epoch, 'seq': hello_seq},
                              target)
                hello_seq += 1
                next_hello += 1.0 / HELLO_HZ
            with self.lock:
                live = self.book.live(now, ADDR_TIMEOUT_SEC)
                resend, expired = self.sender.due(now)
            if self.is_car and now >= next_state:
                state = {'ver': v2v.PROTOCOL_VERSION, 'type': 'bsm', 'id': self.my_id,
                         'epoch': self.epoch, 'seq': state_seq, 't': time.time()}
                state.update(dict.fromkeys(v2v.BSM_FIELDS, 0.0), batt=None)
                for addr in live.values():
                    self.send(state, addr)
                state_seq += 1
                next_state += 1.0 / STATE_HZ
            for to, packet in resend:
                self.send(packet, self.book.addr(to))
            for to, packet in expired:
                print(f"{stamp()} NOT delivered to car {to}: {packet['text']}")
            time.sleep(0.02)

    def type_lines(self):
        """Send each typed line to every car; a closed stdin just stops typing."""
        for line in sys.stdin:
            text = line.rstrip('\n')
            if not text:
                continue
            now = time.monotonic()
            with self.lock:
                cars = sorted(pid for pid, last in self.cars.items()
                              if now - last <= CAR_TIMEOUT_SEC)
                out = self.sender.send(text, cars, now, time.time())
            if not cars:
                print(f'{stamp()} no car is sending state; not sent')
            for to, packet in out:
                self.send(packet, self.book.addr(to))


def main():
    # Output is usually piped or tee'd; a line at a time, not a block at exit.
    sys.stdout.reconfigure(line_buffering=True)
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--port', type=int, default=4747)
    who = parser.add_mutually_exclusive_group()
    who.add_argument('--as-car', type=int, help='join as this car id (1-101)')
    who.add_argument('--watch', type=int, help='observe as this id (1-101), no state sent')
    parser.add_argument('--broadcast', action='append', default=[],
                        help='subnet broadcast address for hellos (repeatable)')
    parser.add_argument('--quiet-bsm', action='store_true',
                        help='do not print state packets and hellos')
    parser.add_argument('--key',
                        help='fleet key (64 hex, from `racecar v2v key new`): sign what this '
                             'sends and reject unsigned packets. Without it, this laptop is '
                             'what an outsider is: cars with a key reject everything it sends')
    args = parser.parse_args()
    for value in (args.as_car, args.watch):
        if value is not None and not 1 <= value <= v2v.MAX_ID:
            parser.error(f'ids are 1..{v2v.MAX_ID}')
    if args.key is not None and (len(args.key) != 64 or not all(
            c in '0123456789abcdefABCDEF' for c in args.key)):
        parser.error('--key takes the 64 hex characters `racecar v2v key new` prints')

    peer = Peer(args)
    threading.Thread(target=peer.listen, daemon=True).start()
    if peer.my_id is None:
        print(f'listening on UDP {args.port}: hellos only (Ctrl-C to stop)')
        while True:
            time.sleep(1.0)
    threading.Thread(target=peer.run, daemon=True).start()
    if peer.is_car:
        # Typing runs on its own thread: a closed stdin (a background run)
        # ends the typing, not the car. Only Ctrl-C ends the car.
        threading.Thread(target=peer.type_lines, daemon=True).start()
        print(f'joined as car {peer.my_id} on UDP {args.port}, hellos to {peer.targets}; '
              f'type a line to send it to every car (Ctrl-C to stop)')
    else:
        print(f'watching as id {peer.my_id} on UDP {args.port}, hellos to {peer.targets} '
              f'(Ctrl-C to stop)')
    while True:
        time.sleep(1.0)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
