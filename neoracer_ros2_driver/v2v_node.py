#!/usr/bin/env python3

"""
Car-to-car (V2V) state and messages over UDP.

Every car on the network broadcasts a ``hello`` once a second so the others
find it, then sends its state ten times a second, unicast, to every car it has
heard from in the last few seconds. Unicast is the point: 802.11 acknowledges
and retries a unicast frame, and sends a broadcast frame once, unacknowledged.
The state packet follows the core of the SAE J2735 Basic Safety Message: who,
when, where (in the car's own odom frame), how fast, which way, what is
commanded, how much battery. Text messages are delivered exactly once and in
order: numbered per recipient, acked, retransmitted until acked, and reported
undelivered if the ack never comes. The wire is plain JSON on UDP 4747, so a
testbed can watch, delay, replay, or forge it on purpose. Packet format,
validation, and the delivery bookkeeping live in v2v_lib.

Nothing crosses cars through DDS. Each car keeps its own ROS graph
(ROS_DOMAIN_ID = its car id, written by ``racecar v2v id N``) and this node is
the only bridge:

    /odom, /imu/fused, /mux_out, /battery  ->  state to every car, 10 Hz
    state from car N   ->  /v2v/carN/odom (nav_msgs/Odometry, frame carN/odom)
    every peer         ->  /v2v/peers (std_msgs/String, JSON table, 10 Hz)
    /v2v/send (std_msgs/String)  ->  message to every car sending state
    message from car N ->  /v2v/recv {"type": "msg", "from", "seq", "text"}
    ack from car N     ->  /v2v/recv {"type": "delivered", "to", "seq", "ms", "text"}
    no ack in time     ->  /v2v/recv {"type": "undelivered", "to", "seq", "text"}
    /v2v/ping (std_msgs/Int32, a car id)  ->  ping, answered by that car's node
    pong from car N    ->  /v2v/recv {"type": "pong", "from", "rtt_ms"}

Positions are each car's own odometry, so two cars share no frame until a map
or external positioning gives them one. Ages in the peer table are measured on
the receiving car's clock; the cars' wall clocks are not synchronized.
"""

import functools
import json
import os
import secrets
import socket
import threading
import time

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from neoracer_ros2_driver.v2v_lib import (
    ack_for, AddressBook, broadcast_targets, BSM_FIELDS, encode, key_fingerprint, load_key,
    MAX_DATAGRAM, MAX_ID, MAX_TEXT, MessageReceiver, MessageSender, parse_packet, PeerTable,
    PROTOCOL_VERSION, quaternion_from_yaw, ReplayGuard, sign, verify, yaw_from_quaternion)
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState, Imu
from std_msgs.msg import Int32, String

# A ping unanswered for this long counts as lost and is forgotten.
PING_TIMEOUT_SEC = 5.0

# History kept on /v2v/send and /v2v/recv. Keep-last drops the oldest sample
# when a publisher outruns its subscriber, so the local hop holds a whole
# burst rather than 10, or a message is lost before it ever reaches the wire.
MESSAGE_QUEUE_DEPTH = 1000


class V2VNode(Node):
    """Exchange state and messages with every other car on the network."""

    def __init__(self):
        super().__init__('v2v_node')

        self.car_id = self.declare_parameter('car_id', 0).value
        if not 1 <= self.car_id <= MAX_ID:
            raise RuntimeError(
                f'car_id is {self.car_id}; stamp this car first: '
                f'racecar v2v id N (1..{MAX_ID})')
        self.port = self.declare_parameter('port', 4747).value
        state_rate = self.declare_parameter('state_rate_hz', 10.0).value
        hello_rate = self.declare_parameter('hello_rate_hz', 1.0).value
        peers_rate = self.declare_parameter('peers_rate_hz', 10.0).value
        self.peer_timeout = self.declare_parameter('peer_timeout_sec', 1.0).value
        self.addr_timeout = self.declare_parameter('addr_timeout_sec', 3.0).value
        self.input_timeout = self.declare_parameter('input_timeout_sec', 1.0).value
        self.battery_timeout = self.declare_parameter('battery_timeout_sec', 10.0).value
        msg_retry = self.declare_parameter('msg_retry_sec', 0.1).value
        self.msg_timeout = self.declare_parameter('msg_timeout_sec', 10.0).value
        # The fleet signing key. Present: sign everything sent, reject anything
        # unsigned or wrongly signed. Absent: signing off, as before.
        key_file = os.path.expanduser(
            self.declare_parameter('key_file', '~/.neoracer/v2v.key').value)
        self._key = load_key(key_file)
        # One epoch per start, on every packet: receivers tell a restart from
        # a replay with it, and message streams restart with it.
        self._epoch = secrets.randbits(32)

        # No SO_REUSEADDR: a second instance on this car must fail to bind,
        # not split the incoming packets with the first.
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self._sock.bind(('', self.port))
        self._sock.settimeout(0.5)

        # The address book, peer table, pings and the sender are shared
        # between the receive thread and the executor thread. The receiver
        # is only touched by the receive thread.
        self._lock = threading.Lock()
        self._book = AddressBook()
        self._peers = PeerTable()
        self._sender = MessageSender(
            self.car_id, self._epoch, msg_retry, self.msg_timeout)
        self._receiver = MessageReceiver()
        self._guard = ReplayGuard()
        self._peer_pubs = {}
        self._pings = {}
        self._targets = []
        self._own_addrs = frozenset()
        self._hello_seq = 0
        self._bsm_seq = 0
        self._ping_seq = 0
        self._rx = 0
        self._rx_bad = 0
        self._rejected = dict.fromkeys(
            ('malformed', 'unsigned', 'forged', 'replay', 'other'), 0)
        self._tx_err = 0
        self._msgs_in = 0

        # Latest own reading per topic with its arrival time (monotonic). Each
        # field reads the first of its topics that is fresh, so the same node
        # serves both stacks (`racecar mode`): the fused estimate
        # /odometry/filtered when an EKF runs, and osracer's chassis, which
        # publishes /odometry/filtered, /imu_filter and /battery_state when its
        # own EKF is off.
        self._sources = {
            'odom': ('/odometry/filtered', '/odom'),
            'imu': ('/imu/fused', '/imu_filter'),
            'cmd': ('/mux_out',),
            'batt': ('/battery', '/battery_state'),
        }
        types = {'odom': Odometry, 'imu': Imu, 'cmd': AckermannDriveStamped,
                 'batt': BatteryState}
        self._inputs = {}
        for field, topics in self._sources.items():
            for topic in topics:
                self.create_subscription(
                    types[field], topic, functools.partial(self._on_input, topic),
                    qos_profile_sensor_data)

        self._peers_pub = self.create_publisher(String, '/v2v/peers', 10)
        self._recv_pub = self.create_publisher(String, '/v2v/recv', MESSAGE_QUEUE_DEPTH)
        self.create_subscription(String, '/v2v/send', self._on_send, MESSAGE_QUEUE_DEPTH)
        self.create_subscription(Int32, '/v2v/ping', self._on_ping_request, 10)

        self.create_timer(1.0 / state_rate, self._send_state)
        self.create_timer(1.0 / hello_rate, self._hello)
        self.create_timer(msg_retry / 2.0, self._retransmit)
        self.create_timer(1.0 / peers_rate, self._publish_peers)

        self._stop = threading.Event()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name='v2v-rx', daemon=True)
        self._rx_thread.start()
        self._hello()

        self.get_logger().info(
            f'[INFO] v2v up: car {self.car_id}, UDP {self.port}, state {state_rate:g} Hz '
            f'unicast, hello {hello_rate:g} Hz broadcast, messages retried every '
            f'{msg_retry:g} s for up to {self.msg_timeout:g} s, signing '
            + (f'ON (key {key_fingerprint(self._key)} from {key_file})' if self._key
               else f'off (no {key_file})'))

    # -- own state -> wire -------------------------------------------------

    def _on_input(self, name, msg):
        self._inputs[name] = (msg, time.monotonic())

    def _fresh(self, field, now, timeout):
        """Return the latest message on the first fresh topic of a field, or None."""
        for topic in self._sources[field]:
            entry = self._inputs.get(topic)
            if entry is not None and now - entry[1] <= timeout:
                return entry[0]
        return None

    def _own_state(self, now):
        """Build the bsm fields from the live inputs; a stale input is null."""
        state = dict.fromkeys(BSM_FIELDS)
        odom = self._fresh('odom', now, self.input_timeout)
        if odom is not None:
            pose = odom.pose.pose
            q = pose.orientation
            state['x'] = round(pose.position.x, 4)
            state['y'] = round(pose.position.y, 4)
            state['yaw'] = round(yaw_from_quaternion(q.x, q.y, q.z, q.w), 4)
            state['speed'] = round(odom.twist.twist.linear.x, 3)
        imu = self._fresh('imu', now, self.input_timeout)
        if imu is not None:
            state['yaw_rate'] = round(imu.angular_velocity.z, 4)
            state['ax'] = round(imu.linear_acceleration.x, 3)
        cmd = self._fresh('cmd', now, self.input_timeout)
        if cmd is not None:
            state['cmd_speed'] = round(cmd.drive.speed, 3)
            state['cmd_steer'] = round(cmd.drive.steering_angle, 3)
        # The firmware reports the battery every 2.5 s; it earns a longer window.
        batt = self._fresh('batt', now, self.battery_timeout)
        if batt is not None:
            state['batt'] = round(batt.percentage, 3)
        return state

    def _refresh_targets(self):
        """Re-enumerate the interfaces, so a link that just came up joins at once."""
        self._targets = broadcast_targets(self._sock)
        self._own_addrs = frozenset(addr for _, addr, _ in self._targets)

    def _hello(self):
        self._refresh_targets()
        payload = self._wire({'ver': PROTOCOL_VERSION, 'type': 'hello', 'id': self.car_id,
                              'epoch': self._epoch, 'seq': self._hello_seq})
        self._hello_seq += 1
        for name, _, bcast in self._targets:
            self._send(payload, bcast, name)

    def _send_state(self):
        now = time.monotonic()
        packet = {'ver': PROTOCOL_VERSION, 'type': 'bsm', 'id': self.car_id,
                  'epoch': self._epoch, 'seq': self._bsm_seq, 't': time.time()}
        packet.update(self._own_state(now))
        self._bsm_seq += 1
        payload = self._wire(packet)
        with self._lock:
            peers = self._book.live(now, self.addr_timeout)
        for pid, addr in peers.items():
            self._send(payload, addr, f'car {pid}')

    def _wire(self, packet):
        """Encode a packet for the wire, signed when this car holds a key."""
        payload = encode(packet)
        return payload if self._key is None else sign(payload, self._key)

    def _send(self, payload, addr, where):
        """Send one datagram. A link that drops mid-send is counted, not fatal."""
        try:
            self._sock.sendto(payload, (addr, self.port))
        except OSError as e:
            self._tx_err += 1
            self.get_logger().warning(
                f'send to {addr} ({where}) failed: {e}', throttle_duration_sec=5.0)

    def _event(self, event):
        self._recv_pub.publish(String(data=json.dumps(event)))

    def _on_send(self, msg):
        if len(msg.data) > MAX_TEXT:
            self.get_logger().warning(
                f'/v2v/send text is {len(msg.data)} characters, limit {MAX_TEXT}; not sent')
            self._event({'type': 'unsent', 'text': msg.data, 'reason': 'too long'})
            return
        now = time.monotonic()
        with self._lock:
            recipients = sorted(self._peers.peers)
            out = self._sender.send(msg.data, recipients, now, time.time())
            addrs = {pid: self._book.addr(pid) for pid in recipients}
        if not recipients:
            self.get_logger().warning('/v2v/send: no car is sending state; not sent')
            self._event({'type': 'unsent', 'text': msg.data, 'reason': 'no peers'})
            return
        for to, packet in out:
            self._send(self._wire(packet), addrs[to], f'message to car {to}')

    def _retransmit(self):
        now = time.monotonic()
        with self._lock:
            resend, expired = self._sender.due(now)
            addrs = {to: self._book.addr(to) for to, _ in resend}
        for to, packet in resend:
            self._send(self._wire(packet), addrs[to], f'message to car {to}')
        for to, packet in expired:
            self.get_logger().warning(
                f'message {packet["seq"]} to car {to} not acked in '
                f'{self.msg_timeout:g} s; undelivered')
            self._event({'type': 'undelivered', 'to': to, 'seq': packet['seq'],
                         'text': packet['text']})

    def _on_ping_request(self, msg):
        with self._lock:
            addr = self._book.addr(msg.data)
        if addr is None:
            self.get_logger().warning(f'ping: car {msg.data} has not been heard')
            return
        now = time.monotonic()
        seq = self._ping_seq
        self._ping_seq += 1
        with self._lock:
            self._pings[seq] = (now, msg.data)
        self._send(self._wire({'ver': PROTOCOL_VERSION, 'type': 'ping', 'id': self.car_id,
                               'epoch': self._epoch, 'seq': seq, 't_mono': now}),
                   addr, f'ping car {msg.data}')

    # -- wire -> ROS -------------------------------------------------------

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                data, (addr, _) = self._sock.recvfrom(MAX_DATAGRAM)
            except socket.timeout:
                continue
            self._handle(data, addr, time.monotonic())

    def _reject(self, kind, addr, reason):
        self._rx_bad += 1
        self._rejected[kind] += 1
        self.get_logger().warning(
            f'rejected ({kind}) from {addr}: {reason}', throttle_duration_sec=5.0)

    def _handle(self, data, addr, now):
        if self._key is not None:
            try:
                data = verify(data, self._key)
            except ValueError as e:
                self._reject(str(e), addr, f'{e} packet; this car requires the fleet key')
                return
        try:
            packet = parse_packet(data)
        except ValueError as e:
            self._reject('malformed', addr, str(e))
            return
        pid = packet['id']
        ptype = packet['type']
        if pid == self.car_id:
            # Own hellos loop back on every interface; another address using
            # this id is a second car stamped the same, or a forgery.
            if addr not in self._own_addrs:
                self._reject('other', addr,
                             f'also sends as car {self.car_id}: duplicate id or a spoof')
            return
        if ptype in ('msg', 'ack') and packet['to'] != self.car_id:
            self._reject('other', addr,
                         f'{ptype} for car {packet["to"]} reached car {self.car_id}')
            return
        # Replay protection only with signing: on an open channel anyone can
        # send as any car with a made-up epoch, and the guard would then
        # retire that car's real epoch and reject the real car from then on.
        if self._key is not None:
            try:
                self._guard.check(packet)
            except ValueError as e:
                self._reject('replay', addr, str(e))
                return
        self._rx += 1
        with self._lock:
            new = self._book.heard(pid, addr, now)
        if new:
            self.get_logger().info(f'[INFO] car {pid} heard at {addr}')
        if ptype == 'bsm':
            self._on_bsm(packet, addr, now)
        elif ptype == 'msg':
            self._on_msg(packet, addr)
        elif ptype == 'ack':
            self._on_ack(packet, addr, now)
        elif ptype == 'ping':
            self._send(self._wire({'ver': PROTOCOL_VERSION, 'type': 'pong', 'id': self.car_id,
                                   'epoch': self._epoch, 'seq': packet['seq'],
                                   't_mono': packet['t_mono']}),
                       addr, f'pong car {pid}')
        elif ptype == 'pong':
            self._on_pong(packet, addr, now)
        # A hello carries nothing beyond the address book entry made above.

    def _on_bsm(self, packet, addr, now):
        with self._lock:
            first, _ = self._peers.update_bsm(packet, addr, now)
            pub = self._peer_pubs.get(packet['id'])
        if first:
            self.get_logger().info(f'[INFO] car {packet["id"]} sending state from {addr}')
        # The publisher is created by the executor thread on the next peers
        # tick; until then, and while the peer has no odometry, nothing is
        # published rather than a zero pose.
        if pub is not None and all(packet[f] is not None for f in ('x', 'y', 'yaw', 'speed')):
            pub.publish(self._peer_odom(packet))

    def _on_msg(self, packet, addr):
        try:
            ready, acks = self._receiver.accept(packet)
        except ValueError as e:
            self._reject('other', addr, str(e))
            return
        for msg in acks:
            self._send(self._wire(ack_for(msg, self.car_id, self._epoch)), addr,
                       f'ack car {msg["id"]}')
        for msg in ready:
            self._msgs_in += 1
            self._event({'type': 'msg', 'from': msg['id'], 'seq': msg['seq'],
                         'text': msg['text']})

    def _on_ack(self, packet, addr, now):
        with self._lock:
            result, msg, latency = self._sender.ack(packet, now)
        if result == 'delivered':
            self._event({'type': 'delivered', 'to': packet['id'], 'seq': packet['seq'],
                         'ms': round(latency * 1000.0, 2), 'text': msg['text']})
        elif result == 'unknown':
            self._reject('other', addr, f'ack from car {packet["id"]} answers no message of ours')

    def _on_pong(self, packet, addr, now):
        with self._lock:
            sent = self._pings.pop(packet['seq'], None)
        if sent is None or sent[0] != packet['t_mono'] or sent[1] != packet['id']:
            self._reject('other', addr, f'pong as car {packet["id"]} answers no ping of ours')
            return
        self._event({'type': 'pong', 'from': packet['id'], 'seq': packet['seq'],
                     'rtt_ms': round((now - sent[0]) * 1000.0, 2)})

    def _peer_odom(self, packet):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'car{packet["id"]}/odom'
        msg.child_frame_id = f'car{packet["id"]}/base_footprint'
        msg.pose.pose.position.x = packet['x']
        msg.pose.pose.position.y = packet['y']
        qx, qy, qz, qw = quaternion_from_yaw(packet['yaw'])
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = packet['speed']
        if packet['yaw_rate'] is not None:
            msg.twist.twist.angular.z = packet['yaw_rate']
        return msg

    def _publish_peers(self):
        if not self._rx_thread.is_alive():
            raise RuntimeError('v2v receive thread died; see the traceback above')
        now = time.monotonic()
        with self._lock:
            gone = self._peers.prune(now, self.peer_timeout)
            for pid in self._peers.peers:
                if pid not in self._peer_pubs:
                    self._peer_pubs[pid] = self.create_publisher(
                        Odometry, f'/v2v/car{pid}/odom', 10)
            table = self._peers.snapshot(now)
            heard = self._book.live(now, self.addr_timeout)
            expired = [seq for seq, (t, _) in self._pings.items()
                       if now - t > PING_TIMEOUT_SEC]
            for seq in expired:
                del self._pings[seq]
            msgs = {'in': self._msgs_in, 'delivered': self._sender.delivered,
                    'pending': len(self._sender.pending),
                    'undelivered': self._sender.undelivered,
                    'retx': self._sender.retransmits}
        for pid in gone:
            self.get_logger().info(
                f'[INFO] car {pid} lost (no state for {self.peer_timeout:g} s)')
        status = {
            'id': self.car_id,
            'seq': self._bsm_seq,
            'ifaces': [[name, addr] for name, addr, _ in self._targets],
            'heard': {str(pid): addr for pid, addr in sorted(heard.items())},
            'rx': self._rx,
            'bad': self._rx_bad,
            'rejected': dict(self._rejected),
            'signing': key_fingerprint(self._key) if self._key else None,
            'tx_err': self._tx_err,
            'msgs': msgs,
            'peers': table,
        }
        self._peers_pub.publish(String(data=json.dumps(status)))

    def stop(self):
        """Stop the receive thread and close the socket."""
        self._stop.set()
        self._rx_thread.join(timeout=1.0)
        self._sock.close()


def main(args=None):
    rclpy.init(args=args)
    node = V2VNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        # try_shutdown: a SIGTERM from systemd already shut the context down;
        # a second shutdown() raises and turns clean stops into tracebacks.
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
