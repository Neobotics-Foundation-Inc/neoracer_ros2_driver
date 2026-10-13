"""Tests for the pure V2V logic: packets, peer table, and reliable messages."""

import json
import math
import random
import socket

from neoracer_ros2_driver import v2v_lib as v2v
import pytest


def bsm(**overrides):
    packet = {'ver': 1, 'type': 'bsm', 'id': 2, 'epoch': 5, 'seq': 10, 't': 1789267601.38,
              'x': 1.5, 'y': -0.25, 'yaw': 0.3, 'speed': 1.2, 'yaw_rate': 0.01,
              'ax': 0.0, 'cmd_speed': 0.4, 'cmd_steer': -0.1, 'batt': 0.72}
    packet.update(overrides)
    return packet


def msg(seq, sender=1, to=2, epoch=7, base=0, text=None):
    return {'ver': 1, 'type': 'msg', 'id': sender, 'to': to, 'epoch': epoch, 'seq': seq,
            'base': base, 't': 5.0, 'text': text if text is not None else f'm{seq}'}


def seqs(packets):
    return [p['seq'] for p in packets]


def test_bsm_round_trip():
    assert v2v.parse_packet(v2v.encode(bsm())) == bsm()


def test_null_reading_is_kept_as_none():
    packet = v2v.parse_packet(v2v.encode(bsm(batt=None, x=None)))
    assert packet['batt'] is None
    assert packet['x'] is None
    assert packet['y'] == -0.25


def test_every_type_round_trips():
    hello = {'ver': 1, 'type': 'hello', 'id': 3, 'epoch': 11, 'seq': 40}
    assert v2v.parse_packet(v2v.encode(hello)) == hello
    assert v2v.parse_packet(v2v.encode(msg(0))) == msg(0)
    ack = v2v.ack_for(msg(4), 2, 9)
    assert ack == {'ver': 1, 'type': 'ack', 'id': 2, 'epoch': 9, 'to': 1, 'for_epoch': 7,
                   'seq': 4}
    assert v2v.parse_packet(v2v.encode(ack)) == ack
    ping = {'ver': 1, 'type': 'ping', 'id': 1, 'epoch': 7, 'seq': 7, 't_mono': 12345.678}
    assert v2v.parse_packet(v2v.encode(ping)) == ping
    pong = dict(ping, type='pong', id=2, epoch=9)
    assert v2v.parse_packet(v2v.encode(pong)) == pong


@pytest.mark.parametrize('data, reason', [
    (b'\xff\xfe', 'not JSON'),
    (b'[1, 2]', 'not a JSON object'),
    (json.dumps(bsm(ver=2)).encode(), 'ver 2'),
    (json.dumps(bsm(type='cam')).encode(), "unknown type 'cam'"),
    (json.dumps(bsm(id=0)).encode(), 'id must be an integer'),
    (json.dumps(bsm(id=102)).encode(), 'id must be an integer'),
    (json.dumps(bsm(id=True)).encode(), 'id must be an integer'),
    (json.dumps(bsm(seq=-1)).encode(), 'seq must be an integer'),
    (json.dumps(bsm(x='1.0')).encode(), 'x is not a finite number'),
    (json.dumps(bsm(t=None)).encode(), 't is missing'),
    (b'{"ver":1,"type":"bsm","id":1,"epoch":1,"seq":1,"t":1.0,"x":NaN}',
     'x is not a finite number'),
    (json.dumps(dict(msg(0), text=5)).encode(), 'text is not a string'),
    (json.dumps(dict(msg(0), text='x' * (v2v.MAX_TEXT + 1))).encode(), 'text longer than'),
    (json.dumps(dict(msg(0), to=None)).encode(), 'to must be an integer'),
    (json.dumps(dict(msg(0), epoch=2**32)).encode(), 'epoch must be an integer'),
    (json.dumps(dict(msg(3), base=4)).encode(), 'base must be an integer in 0..3'),
    (json.dumps({'ver': 1, 'type': 'ack', 'id': 2, 'to': 1, 'seq': 0}).encode(),
     'epoch must be an integer'),
    (json.dumps({'ver': 1, 'type': 'ack', 'id': 2, 'epoch': 9, 'to': 1, 'seq': 0}).encode(),
     'for_epoch must be an integer'),
    (json.dumps(bsm(epoch=-1)).encode(), 'epoch must be an integer'),
    (json.dumps({'ver': 1, 'type': 'ping', 'id': 1, 'epoch': 7, 'seq': 0}).encode(),
     't_mono is missing'),
])
def test_parse_rejects(data, reason):
    with pytest.raises(ValueError) as err:
        v2v.parse_packet(data)
    assert reason in str(err.value)


def test_extra_fields_are_dropped():
    assert 'evil' not in v2v.parse_packet(json.dumps(bsm(evil='payload')).encode())
    assert 'text' not in v2v.parse_packet(
        json.dumps({'ver': 1, 'type': 'hello', 'id': 1, 'epoch': 1, 'seq': 0,
                    'text': 'x'}).encode())


def test_yaw_quaternion_round_trip():
    for yaw in (-3.0, -1.0, 0.0, 0.3, 2.9):
        assert v2v.yaw_from_quaternion(*v2v.quaternion_from_yaw(yaw)) == pytest.approx(yaw)


def test_broadcast_targets_skip_loopback():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        targets = v2v.broadcast_targets(sock)
    finally:
        sock.close()
    assert all(name != 'lo' for name, _, _ in targets)
    for _, addr, bcast in targets:
        socket.inet_aton(addr)
        socket.inet_aton(bcast)


def test_address_book_tracks_last_address_and_age():
    book = v2v.AddressBook()
    assert book.heard(2, '10.0.0.94', 100.0) is True
    assert book.heard(2, '10.0.0.95', 101.0) is False
    assert book.heard(3, '10.0.0.96', 99.0) is True
    assert book.addr(2) == '10.0.0.95'
    assert book.addr(9) is None
    assert book.live(101.9, 3.0) == {2: '10.0.0.95', 3: '10.0.0.96'}
    assert book.live(102.5, 3.0) == {2: '10.0.0.95'}


def test_peer_table_counts_loss_and_restarts():
    table = v2v.PeerTable()
    assert table.update_bsm(bsm(seq=10), '10.0.0.5', 100.0) == (True, 0)
    assert table.update_bsm(bsm(seq=11), '10.0.0.5', 100.1) == (False, 0)
    assert table.update_bsm(bsm(seq=14), '10.0.0.5', 100.4) == (False, 2)
    assert table.update_bsm(bsm(seq=0), '10.0.0.5', 100.5) == (False, 0)
    peer = table.peers[2]
    assert peer['rx'] == 4
    assert peer['lost'] == 2
    assert peer['seq'] == 0


def test_peer_table_snapshot_and_prune():
    table = v2v.PeerTable()
    table.update_bsm(bsm(id=2, seq=1), '10.0.0.5', 100.0)
    table.update_bsm(bsm(id=3, seq=1, batt=None), '10.0.0.6', 100.5)
    snap = table.snapshot(101.0)
    assert list(snap) == ['2', '3']
    assert snap['2']['age'] == pytest.approx(1.0)
    assert snap['2']['addr'] == '10.0.0.5'
    assert snap['3']['batt'] is None
    assert math.isclose(snap['3']['yaw'], 0.3)
    assert table.prune(101.2, 1.0) == [2]
    assert list(table.peers) == [3]
    assert table.prune(101.2, 1.0) == []


def test_receiver_delivers_in_order_once_and_acks_on_delivery():
    rx = v2v.MessageReceiver()
    ready, acks = rx.accept(msg(0))
    assert seqs(ready) == [0] and seqs(acks) == [0]
    assert rx.accept(msg(2)) == ([], [])
    assert rx.accept(msg(3)) == ([], [])
    assert rx.accept(msg(2)) == ([], [])
    ready, acks = rx.accept(msg(1))
    assert seqs(ready) == [1, 2, 3] and seqs(acks) == [1, 2, 3]
    assert rx.accept(msg(1)) == ([], [msg(1)])
    assert rx.accept(msg(0)) == ([], [msg(0)])
    assert seqs(rx.accept(msg(4))[0]) == [4]


def test_receiver_keeps_senders_apart_and_restarts_on_new_epoch():
    rx = v2v.MessageReceiver()
    assert seqs(rx.accept(msg(0, sender=1))[0]) == [0]
    assert seqs(rx.accept(msg(0, sender=3))[0]) == [0]
    assert seqs(rx.accept(msg(1, sender=1))[0]) == [1]
    assert rx.accept(msg(1, sender=1))[0] == []
    ready, _ = rx.accept(msg(0, sender=1, epoch=8, text='after restart'))
    assert [m['text'] for m in ready] == ['after restart']


def test_restarted_receiver_starts_at_base():
    rx = v2v.MessageReceiver()
    ready, acks = rx.accept(msg(5, base=5))
    assert seqs(ready) == [5] and seqs(acks) == [5]
    ready, _ = rx.accept(msg(7, base=6))
    assert ready == []
    assert seqs(rx.accept(msg(6, base=6))[0]) == [6, 7]


def test_base_past_a_held_message_drops_it():
    rx = v2v.MessageReceiver()
    assert rx.accept(msg(3, base=2)) == ([], [])
    ready, _ = rx.accept(msg(4, base=4))
    assert seqs(ready) == [4]
    assert rx.accept(msg(3, base=3)) == ([], [msg(3, base=3)])


def test_receiver_refuses_a_stream_far_ahead():
    rx = v2v.MessageReceiver()
    with pytest.raises(ValueError):
        rx.accept(msg(v2v.MessageReceiver.MAX_AHEAD + 1))
    assert rx.accept(msg(v2v.MessageReceiver.MAX_AHEAD)) == ([], [])


def test_sender_numbers_each_recipient_separately():
    tx = v2v.MessageSender(1, 7, retry_sec=0.1, give_up_sec=10.0)
    first = tx.send('a', [2, 3], 0.0, 5.0)
    second = tx.send('b', [2], 0.0, 5.0)
    third = tx.send('c', [2, 3, 4], 0.0, 5.0)
    assert [(to, p['seq']) for to, p in first] == [(2, 0), (3, 0)]
    assert [(to, p['seq']) for to, p in second] == [(2, 1)]
    assert [(to, p['seq']) for to, p in third] == [(2, 2), (3, 1), (4, 0)]
    assert all(p['to'] == to and p['epoch'] == 7 and p['id'] == 1 for to, p in third)
    assert [(to, p['base']) for to, p in third] == [(2, 0), (3, 0), (4, 0)]


def test_sender_base_follows_acks_and_expiry():
    tx = v2v.MessageSender(1, 7, retry_sec=0.1, give_up_sec=1.0)
    first, second, third = (tx.send(t, [2], 0.0, 5.0)[0][1] for t in 'abc')
    assert tx.base(2) == 0
    tx.ack(v2v.ack_for(first, 2, 99), 0.05)
    assert tx.base(2) == 1
    resend, _ = tx.due(0.1)
    assert [(p['seq'], p['base']) for _, p in resend] == [(1, 1), (2, 1)]
    tx.ack(v2v.ack_for(third, 2, 99), 0.15)
    resend, expired = tx.due(1.05)
    assert [p['seq'] for _, p in expired] == [1]
    assert resend == []
    assert tx.base(2) == 3
    [(_, fourth)] = tx.send('d', [2], 1.1, 5.0)
    assert (fourth['seq'], fourth['base']) == (3, 3)


def test_sender_retransmits_until_acked():
    tx = v2v.MessageSender(1, 7, retry_sec=0.1, give_up_sec=10.0)
    [(to, packet)] = tx.send('hold', [2], 0.0, 5.0)
    assert tx.due(0.05) == ([], [])
    assert tx.due(0.1) == ([(2, packet)], [])
    assert tx.due(0.15) == ([], [])
    assert tx.due(0.2) == ([(2, packet)], [])
    assert tx.retransmits == 2
    result, acked, latency = tx.ack(v2v.ack_for(packet, 2, 99), 0.25)
    assert result == 'delivered'
    assert acked['text'] == 'hold'
    assert latency == pytest.approx(0.25)
    assert tx.due(10.0) == ([], [])
    assert tx.delivered == 1


def test_sender_classifies_acks():
    tx = v2v.MessageSender(1, 7, retry_sec=0.1, give_up_sec=10.0)
    [(_, packet)] = tx.send('x', [2], 0.0, 5.0)
    ack = v2v.ack_for(packet, 2, 99)
    assert tx.ack(ack, 0.1)[0] == 'delivered'
    assert tx.ack(ack, 0.2) == ('duplicate', None, None)
    assert tx.ack(dict(ack, seq=5), 0.2) == ('unknown', None, None)
    assert tx.ack(dict(ack, for_epoch=8), 0.2) == ('unknown', None, None)
    assert tx.ack(dict(ack, id=3), 0.2) == ('unknown', None, None)
    assert tx.ack(dict(ack, to=4), 0.2) == ('unknown', None, None)


def test_sender_gives_up_and_reports():
    tx = v2v.MessageSender(1, 7, retry_sec=0.1, give_up_sec=1.0)
    [(_, packet)] = tx.send('lost', [2], 0.0, 5.0)
    resend, expired = tx.due(1.01)
    assert resend == []
    assert expired == [(2, packet)]
    assert tx.pending == {}
    assert tx.undelivered == 1
    assert tx.ack(v2v.ack_for(packet, 2, 99), 1.1) == ('duplicate', None, None)


def run_link(tx, receivers, drop, now=0.0, until=10.0):
    """Deliver tx's traffic to car 2 through a link; return the texts delivered."""
    rng = random.Random(4747)
    delivered = []
    outbox = []
    while tx.pending and now < until:
        rx = receivers(now)
        for _, packet in outbox:
            if rng.random() >= drop:
                ready, acks = rx.accept(v2v.parse_packet(v2v.encode(packet)))
                delivered += [m['text'] for m in ready]
                for done in acks:
                    if rng.random() >= drop:
                        tx.ack(v2v.ack_for(done, 2, 99), now)
        now += 0.1
        outbox, _ = tx.due(now)
    return delivered


def test_lossy_link_still_delivers_everything_in_order():
    """Half the frames dropped each way: all 20 arrive once, in order."""
    tx = v2v.MessageSender(1, 7, retry_sec=0.1, give_up_sec=10.0)
    rx = v2v.MessageReceiver()
    for i in range(20):
        tx.send(f'm{i}', [2], 0.0, 0.0)
    delivered = run_link(tx, lambda now: rx, drop=0.5)
    assert delivered == [f'm{i}' for i in range(20)]
    assert tx.delivered == 20
    assert tx.undelivered == 0


def test_receiver_restart_mid_stream_loses_nothing():
    """The receiving node restarts with messages in flight; the rest still arrive in order."""
    tx = v2v.MessageSender(1, 7, retry_sec=0.1, give_up_sec=10.0)
    first, second = v2v.MessageReceiver(), v2v.MessageReceiver()
    for i in range(10):
        tx.send(f'm{i}', [2], 0.0, 0.0)
    delivered = run_link(tx, lambda now: first, drop=0.0, until=0.05)
    assert delivered == []
    for _, packet in sorted(tx.pending.items())[:4]:
        ready, acks = first.accept(packet['packet'])
        delivered += [m['text'] for m in ready]
        for done in acks:
            tx.ack(v2v.ack_for(done, 2, 99), 0.1)
    for i in range(10, 15):
        tx.send(f'm{i}', [2], 0.1, 0.0)
    delivered += run_link(tx, lambda now: second, drop=0.3, now=0.1)
    assert delivered == [f'm{i}' for i in range(15)]
    assert tx.undelivered == 0


def test_unicode_text_fits_the_receive_buffer():
    text = '🚗' * v2v.MAX_TEXT
    packet = dict(msg(0), text=text)
    wire = v2v.encode(packet)
    assert len(wire) < 4 * v2v.MAX_TEXT + 200
    assert v2v.parse_packet(wire)['text'] == text


KEY = bytes(range(32))


def test_sign_verify_round_trip_and_tamper():
    wire = v2v.sign(v2v.encode(bsm()), KEY)
    assert v2v.parse_packet(v2v.verify(wire, KEY)) == bsm()
    tampered = wire.replace(b'"speed":1.2', b'"speed":9.9')
    assert tampered != wire
    with pytest.raises(ValueError, match='forged'):
        v2v.verify(tampered, KEY)
    with pytest.raises(ValueError, match='forged'):
        v2v.verify(wire, bytes(32))
    with pytest.raises(ValueError, match='unsigned'):
        v2v.verify(v2v.encode(bsm()), KEY)


def test_signed_packet_still_parses_on_a_car_without_a_key():
    assert v2v.parse_packet(v2v.sign(v2v.encode(bsm()), KEY)) == bsm()


def test_signature_covers_utf8_text():
    packet = dict(msg(0), text='slow down \U0001F697')
    wire = v2v.sign(v2v.encode(packet), KEY)
    assert v2v.parse_packet(v2v.verify(wire, KEY))['text'] == packet['text']


def test_load_key_and_fingerprint(tmp_path):
    assert v2v.load_key(str(tmp_path / 'absent')) is None
    path = tmp_path / 'v2v.key'
    path.write_text('ab' * 32 + '\n')
    assert v2v.load_key(str(path)) == bytes([0xAB]) * 32
    path.write_text('not a key')
    with pytest.raises(ValueError):
        v2v.load_key(str(path))
    assert v2v.key_fingerprint(KEY) == v2v.key_fingerprint(bytes(range(32)))
    assert v2v.key_fingerprint(KEY) != v2v.key_fingerprint(bytes(32))
    assert len(v2v.key_fingerprint(KEY)) == 8


def test_replay_guard():
    guard = v2v.ReplayGuard()
    guard.check(bsm(seq=10))
    guard.check(bsm(seq=11))
    with pytest.raises(ValueError, match='not newer'):
        guard.check(bsm(seq=11))
    with pytest.raises(ValueError, match='not newer'):
        guard.check(bsm(seq=4))
    guard.check({'ver': 1, 'type': 'hello', 'id': 2, 'epoch': 5, 'seq': 0})
    guard.check(bsm(seq=0, epoch=6))
    with pytest.raises(ValueError, match='replaced'):
        guard.check(bsm(seq=99, epoch=5))
    guard.check(msg(3))
    guard.check(msg(3))
