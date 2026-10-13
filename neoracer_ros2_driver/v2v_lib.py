"""
Pure V2V packet, peer-table, and reliable-message logic (no ROS).

Unit-tested in test/test_v2v.py. Imported by v2v_node on the car and by
scripts/v2v_peer.py on a laptop, so it carries no Linux-only import at
module level.

Wire format: one JSON object per UDP datagram, port 4747. ``epoch`` is the
sender's, a random 32-bit number drawn each time its node starts.

    {"ver": 1, "type": "hello", "id": 1, "epoch": 3735928559, "seq": 40}
    {"ver": 1, "type": "bsm", "id": 1, "epoch": 3735928559, "seq": 812,
     "t": 1789267601.38, "x": -0.006, "y": -0.002, "yaw": -0.72, "speed": 0.0,
     "yaw_rate": 0.0, "ax": 0.0, "cmd_speed": 0.0, "cmd_steer": 0.0, "batt": 0.72}
    {"ver": 1, "type": "msg", "id": 1, "epoch": 3735928559, "to": 2, "seq": 5,
     "base": 4, "t": 1789267601.5, "text": "slow down"}
    {"ver": 1, "type": "ack", "id": 2, "epoch": 2882400018, "to": 1,
     "for_epoch": 3735928559, "seq": 5}
    {"ver": 1, "type": "ping", "id": 1, "epoch": 3735928559, "seq": 7, "t_mono": 12345.678}
    {"ver": 1, "type": "pong", "id": 2, "epoch": 2882400018, "seq": 7, "t_mono": 12345.678}

Signing (on when the car holds a fleet key) appends one field,
``,"sig":"<32 hex>"``, to the object as sent: HMAC-SHA256 over the bytes
before it, truncated to 128 bits. A car with a key rejects anything unsigned
or wrongly signed; a car without one ignores the field.

Delivery. ``hello`` is the only broadcast: once a second, so cars find each
other. Everything else is unicast to the address a peer was last heard from,
because 802.11 acknowledges and retries a unicast frame and sends a broadcast
frame once, unacknowledged. Measured on Halo 2.4 GHz: about 5% of broadcasts
lost, no unicast lost.

``bsm`` follows the core of the SAE J2735 Basic Safety Message: position and
heading in the sender's own odom frame, speed, yaw rate, longitudinal
acceleration, the commanded speed and steering, and the battery fraction. A
field the sender has no reading for is null. A newer bsm supersedes an older
one, so state is not retransmitted.

``msg`` is delivered exactly once and in order. Each sender numbers a separate
stream per recipient from 0, under an ``epoch`` drawn at random when the
sender starts. The recipient delivers the stream in sequence and acks a
message when it delivers it (and again for any later copy, since the first ack
may be lost); a message held behind a gap is not acked, so the sender keeps
resending it. The sender retransmits until acked and reports the message
undelivered when it gives up. ``base`` is the sender's lowest unacked seq to
that recipient: everything below it was delivered (or reported undelivered),
so a recipient whose node restarted mid-stream starts there instead of waiting
for seq 0. A new epoch is a restarted sender and starts a new stream.

``ping`` carries the sender's monotonic clock and ``pong`` echoes it, so the
round trip is measured on one clock; the cars' wall clocks are not
synchronized. Pings are not retried: they measure the link.

A signature proves who sent a packet, not when, so a car with a key also
rejects replays: a packet from an epoch it has seen replaced, and a hello or state
packet whose seq does not move forward (ReplayGuard).
"""

import hashlib
import hmac
import json
import math
import re
import socket
import struct

PROTOCOL_VERSION = 1
# Signature: HMAC-SHA256 truncated to 128 bits, as hex, in a trailing field.
SIG_HEX = 32
_SIG_TAIL = re.compile(rb',"sig":"([0-9a-f]{32})"\}\Z')
_KEY_HEX = re.compile(r'[0-9a-fA-F]{64}')
# A receive buffer for the largest UDP payload: a 1000-character message of
# 4-byte UTF-8 is about 4.2 kB on the wire.
MAX_DATAGRAM = 65535
MAX_ID = 101
MAX_SEQ = 2**53
MAX_EPOCH = 2**32 - 1
MAX_TEXT = 1000
BSM_FIELDS = ('x', 'y', 'yaw', 'speed', 'yaw_rate', 'ax',
              'cmd_speed', 'cmd_steer', 'batt')
PACKET_TYPES = ('hello', 'bsm', 'msg', 'ack', 'ping', 'pong')

# Linux ioctl numbers and interface flags (linux/sockios.h, linux/if.h).
_SIOCGIFFLAGS = 0x8913
_SIOCGIFADDR = 0x8915
_SIOCGIFBRDADDR = 0x8919
_IFF_UP = 0x1
_IFF_BROADCAST = 0x2
_IFF_LOOPBACK = 0x8
_IFF_RUNNING = 0x40


def encode(packet: dict) -> bytes:
    """Serialize one packet for the wire, non-ASCII text as UTF-8, not escaped."""
    return json.dumps(packet, separators=(',', ':'), ensure_ascii=False).encode('utf-8')


def _number_or_none(name, value):
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f'{name} is not a finite number')
    if not math.isfinite(value):
        raise ValueError(f'{name} is not a finite number')
    return float(value)


def _number(name, value):
    number = _number_or_none(name, value)
    if number is None:
        raise ValueError(f'{name} is missing')
    return number


def _integer(name, value, low, high):
    if isinstance(value, bool) or not isinstance(value, int) or not low <= value <= high:
        raise ValueError(f'{name} must be an integer in {low}..{high}')
    return value


def parse_packet(data: bytes) -> dict:
    """
    Validate one datagram from the network and return it as a dict.

    Raises ValueError naming the first problem. Everything on the wire is
    untrusted: a testbed sends malformed and forged packets on purpose.
    Fields a type does not define are dropped.
    """
    try:
        obj = json.loads(data.decode('utf-8'))
    except ValueError as e:
        raise ValueError(f'not JSON: {e}') from None
    if not isinstance(obj, dict):
        raise ValueError('not a JSON object')
    if obj.get('ver') != PROTOCOL_VERSION:
        raise ValueError(f'ver {obj.get("ver")!r}, expected {PROTOCOL_VERSION}')
    ptype = obj.get('type')
    if ptype not in PACKET_TYPES:
        raise ValueError(f'unknown type {ptype!r}')
    packet = {
        'ver': PROTOCOL_VERSION,
        'type': ptype,
        'id': _integer('id', obj.get('id'), 1, MAX_ID),
        'epoch': _integer('epoch', obj.get('epoch'), 0, MAX_EPOCH),
        'seq': _integer('seq', obj.get('seq'), 0, MAX_SEQ),
    }
    if ptype == 'bsm':
        packet['t'] = _number('t', obj.get('t'))
        for field in BSM_FIELDS:
            packet[field] = _number_or_none(field, obj.get(field))
    elif ptype in ('msg', 'ack'):
        packet['to'] = _integer('to', obj.get('to'), 1, MAX_ID)
        if ptype == 'ack':
            packet['for_epoch'] = _integer('for_epoch', obj.get('for_epoch'), 0, MAX_EPOCH)
        else:
            packet['base'] = _integer('base', obj.get('base'), 0, packet['seq'])
            packet['t'] = _number('t', obj.get('t'))
            text = obj.get('text')
            if not isinstance(text, str):
                raise ValueError('text is not a string')
            if len(text) > MAX_TEXT:
                raise ValueError(f'text longer than {MAX_TEXT} characters')
            packet['text'] = text
    elif ptype in ('ping', 'pong'):
        packet['t_mono'] = _number('t_mono', obj.get('t_mono'))
    return packet


def load_key(path: str):
    """
    Return the fleet signing key from a file, or None when the file is absent.

    The file holds 64 hex characters (a 256-bit key), as `racecar v2v key new`
    writes it. Anything else in the file is an error, never a silent "off".
    """
    try:
        with open(path, encoding='ascii') as f:
            text = f.read().strip()
    except FileNotFoundError:
        return None
    if not _KEY_HEX.fullmatch(text):
        raise ValueError(f'{path} must hold exactly 64 hex characters')
    return bytes.fromhex(text)


def key_fingerprint(key: bytes) -> str:
    """Return 8 hex characters that match on every car holding the same key."""
    return hashlib.sha256(b'neoracer-v2v-fingerprint' + key).hexdigest()[:8]


def sign(payload: bytes, key: bytes) -> bytes:
    """
    Append a signature field to an encoded packet.

    The tag is HMAC-SHA256 over the exact bytes of the packet as sent without
    the field, truncated to 128 bits, so any language can check it without
    re-encoding the JSON.
    """
    tag = hmac.new(key, payload, hashlib.sha256).hexdigest()[:SIG_HEX]
    return payload[:-1] + b',"sig":"' + tag.encode('ascii') + b'"}'


def verify(data: bytes, key: bytes) -> bytes:
    """Return the signed packet bytes; raise ValueError('unsigned' | 'forged')."""
    match = _SIG_TAIL.search(data)
    if match is None:
        raise ValueError('unsigned')
    payload = data[:match.start()] + b'}'
    tag = hmac.new(key, payload, hashlib.sha256).hexdigest()[:SIG_HEX]
    if not hmac.compare_digest(tag.encode('ascii'), match.group(1)):
        raise ValueError('forged')
    return payload


class ReplayGuard:
    """
    Reject old packets replayed into the network.

    A signature proves who sent a packet, not when, so a recorded packet would
    verify forever. Every packet carries its sender's epoch, drawn at random
    when the sender starts: a packet from an epoch this car has seen replaced
    is old, and so is a hello or a state packet whose sequence does not move
    forward within the current epoch. Messages need no check here: the
    message receiver already drops what it has delivered.
    """

    def __init__(self):
        self.current = {}
        self.retired = {}
        self.last_seq = {}

    def check(self, packet: dict):
        """Raise ValueError for a replayed packet; record a fresh one."""
        pid, epoch = packet['id'], packet['epoch']
        current = self.current.get(pid)
        if epoch != current:
            if epoch in self.retired.get(pid, ()):
                raise ValueError(f'car {pid} epoch {epoch} was replaced: replay')
            if current is not None:
                self.retired.setdefault(pid, set()).add(current)
            self.current[pid] = epoch
            for key in [k for k in self.last_seq if k[0] == pid]:
                del self.last_seq[key]
        if packet['type'] in ('hello', 'bsm'):
            key = (pid, packet['type'])
            if packet['seq'] <= self.last_seq.get(key, -1):
                raise ValueError(
                    f'car {pid} {packet["type"]} seq {packet["seq"]} is not newer: replay')
            self.last_seq[key] = packet['seq']


def yaw_from_quaternion(x, y, z, w) -> float:
    """Return the heading (rad) of a planar orientation quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw) -> tuple:
    """Return (x, y, z, w) for a rotation of yaw radians about Z."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def broadcast_targets(sock) -> list:
    """
    Return (interface, address, broadcast) for every interface worth sending on.

    An interface counts when it is up with carrier, supports broadcast, is
    not loopback, and holds an IPv4 address. Called before every hello so a
    Wi-Fi link that comes up later joins without a restart. Linux only.
    """
    import fcntl

    targets = []
    wanted = _IFF_UP | _IFF_RUNNING | _IFF_BROADCAST
    for _, name in socket.if_nameindex():
        ifreq = struct.pack('256s', name.encode('utf-8')[:15])
        try:
            flags = struct.unpack(
                'H', fcntl.ioctl(sock.fileno(), _SIOCGIFFLAGS, ifreq)[16:18])[0]
            if flags & wanted != wanted or flags & _IFF_LOOPBACK:
                continue
            addr = socket.inet_ntoa(fcntl.ioctl(sock.fileno(), _SIOCGIFADDR, ifreq)[20:24])
            bcast = socket.inet_ntoa(fcntl.ioctl(sock.fileno(), _SIOCGIFBRDADDR, ifreq)[20:24])
        except OSError:
            # Up with carrier but no IPv4 yet (Wi-Fi between association and
            # its lease), or gone since the listing (a USB link unplugged).
            # Either joins, or stays out, on a later call.
            continue
        targets.append((name, addr, bcast))
    return targets


class AddressBook:
    """Where each peer was last heard from, and when (local monotonic clock)."""

    def __init__(self):
        self.entries = {}

    def heard(self, pid: int, addr: str, now: float) -> bool:
        """Record a valid packet from pid at addr; True when pid is new."""
        new = pid not in self.entries
        self.entries[pid] = (addr, now)
        return new

    def live(self, now: float, timeout: float) -> dict:
        """Return {pid: addr} for every peer heard within timeout seconds."""
        return {pid: addr for pid, (addr, last) in self.entries.items()
                if now - last <= timeout}

    def addr(self, pid: int):
        """Return the last address pid was heard from, or None."""
        entry = self.entries.get(pid)
        return None if entry is None else entry[0]


class PeerTable:
    """Latest state per peer car, keyed by id, with ages on the local clock."""

    def __init__(self):
        self.peers = {}

    def update_bsm(self, packet: dict, addr: str, now: float) -> tuple:
        """
        Record one bsm from a peer.

        Returns (first_sight, lost): whether this is the first packet from
        that id, and how many sequence numbers were skipped since the last
        one. A sequence that goes backwards is a restarted sender, not loss.
        """
        pid = packet['id']
        peer = self.peers.get(pid)
        if peer is None:
            self.peers[pid] = {
                'addr': addr, 'seq': packet['seq'], 'rx': 1, 'lost': 0,
                'first': now, 'last': now, 'bsm': packet,
            }
            return True, 0
        gap = packet['seq'] - peer['seq']
        lost = gap - 1 if gap > 1 else 0
        peer['addr'] = addr
        peer['seq'] = packet['seq']
        peer['last'] = now
        peer['bsm'] = packet
        peer['rx'] += 1
        peer['lost'] += lost
        return False, lost

    def prune(self, now: float, timeout: float) -> list:
        """Drop peers silent for longer than timeout seconds; return their ids."""
        gone = [pid for pid, peer in self.peers.items() if now - peer['last'] > timeout]
        for pid in gone:
            del self.peers[pid]
        return gone

    def snapshot(self, now: float) -> dict:
        """Return a JSON-ready view of the table, ages measured from now."""
        table = {}
        for pid, peer in sorted(self.peers.items()):
            row = {
                'age': round(now - peer['last'], 3),
                'addr': peer['addr'],
                'rx': peer['rx'],
                'lost': peer['lost'],
                'seq': peer['seq'],
                't': peer['bsm']['t'],
            }
            for field in BSM_FIELDS:
                row[field] = peer['bsm'][field]
            table[str(pid)] = row
        return table


class MessageSender:
    """
    Outgoing message streams, one per recipient, retransmitted until acked.

    Pure bookkeeping: the caller sends the packets this returns and feeds
    back the acks it receives.
    """

    def __init__(self, car_id: int, epoch: int, retry_sec: float, give_up_sec: float):
        self.car_id = car_id
        self.epoch = epoch
        self.retry_sec = retry_sec
        self.give_up_sec = give_up_sec
        self.next_seq = {}
        self.pending = {}
        self.delivered = 0
        self.undelivered = 0
        self.retransmits = 0

    def base(self, to: int) -> int:
        """Return the lowest unacked seq to a recipient (its next seq when none is)."""
        return min((seq for (r, seq) in self.pending if r == to),
                   default=self.next_seq.get(to, 0))

    def send(self, text: str, recipients, now: float, wall: float) -> list:
        """Queue text for every recipient; return [(recipient, packet)] to send now."""
        out = []
        for to in recipients:
            seq = self.next_seq.get(to, 0)
            self.next_seq[to] = seq + 1
            packet = {'ver': PROTOCOL_VERSION, 'type': 'msg', 'id': self.car_id, 'to': to,
                      'epoch': self.epoch, 'seq': seq, 'base': 0, 't': wall, 'text': text}
            self.pending[(to, seq)] = {'packet': packet, 'first': now, 'last': now}
            packet['base'] = self.base(to)
            out.append((to, packet))
        return out

    def due(self, now: float) -> tuple:
        """
        Return (resend, expired), each a list of (recipient, packet).

        expired: unacked give_up_sec after the first copy; dropped from pending.
        resend: unacked for retry_sec since the last copy went out, its base
        brought up to date (expiries above move it).
        """
        expired = []
        for key, entry in list(self.pending.items()):
            if now - entry['first'] > self.give_up_sec:
                del self.pending[key]
                self.undelivered += 1
                expired.append((key[0], entry['packet']))
        resend = []
        for key, entry in self.pending.items():
            if now - entry['last'] >= self.retry_sec:
                entry['last'] = now
                entry['packet']['base'] = self.base(key[0])
                self.retransmits += 1
                resend.append((key[0], entry['packet']))
        return resend, expired

    def ack(self, packet: dict, now: float) -> tuple:
        """
        Apply one ack.

        Returns ('delivered', msg packet, seconds since first sent),
        ('duplicate', None, None) for a second ack of a message already
        settled (each retransmitted copy is acked), or ('unknown', None, None)
        for an ack that answers nothing this sender sent.
        """
        if packet['for_epoch'] != self.epoch or packet['to'] != self.car_id:
            return 'unknown', None, None
        entry = self.pending.pop((packet['id'], packet['seq']), None)
        if entry is not None:
            self.delivered += 1
            return 'delivered', entry['packet'], now - entry['first']
        if packet['seq'] < self.next_seq.get(packet['id'], 0):
            return 'duplicate', None, None
        return 'unknown', None, None


class MessageReceiver:
    """Exactly-once, in-order delivery of each sender's message stream."""

    # A legitimate sender is never this far ahead of what it has had acked.
    MAX_AHEAD = 1000

    def __init__(self):
        self.streams = {}

    def accept(self, packet: dict) -> tuple:
        """
        Take one msg addressed to this car.

        Returns (ready, acks): the packets now deliverable, in sequence order,
        and the packets to ack. A delivered message is acked; so is a later
        copy of one already delivered, because the first ack may have been
        lost. A message held behind a gap is neither delivered nor acked, so
        its sender keeps resending it. Raises ValueError for a sequence
        implausibly far ahead.
        """
        sid = packet['id']
        stream = self.streams.get(sid)
        if stream is None or stream['epoch'] != packet['epoch']:
            stream = {'epoch': packet['epoch'], 'next': 0, 'held': {}}
            self.streams[sid] = stream
        if packet['base'] > stream['next']:
            # Below base the sender has settled everything: acked by an earlier
            # run of this node, or given up on and reported undelivered.
            for seq in [s for s in stream['held'] if s < packet['base']]:
                del stream['held'][seq]
            stream['next'] = packet['base']
        seq = packet['seq']
        if seq - stream['next'] > self.MAX_AHEAD:
            raise ValueError(
                f'seq {seq} is {seq - stream["next"]} ahead of car {sid} stream')
        if seq < stream['next']:
            return [], [packet]
        stream['held'][seq] = packet
        ready = []
        while stream['next'] in stream['held']:
            ready.append(stream['held'].pop(stream['next']))
            stream['next'] += 1
        return ready, ready


def ack_for(msg: dict, car_id: int, epoch: int) -> dict:
    """Return the ack this car (running under epoch) sends for a msg to it."""
    return {'ver': PROTOCOL_VERSION, 'type': 'ack', 'id': car_id, 'epoch': epoch,
            'to': msg['id'], 'for_epoch': msg['epoch'], 'seq': msg['seq']}
