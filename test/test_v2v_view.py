"""Tests for the bird's-eye view's logic (scripts/v2v_view.py), without a network or a browser."""

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                'scripts'))
import v2v_view as view  # noqa: E402,I100


def bsm(seq, t, odom=None, on_map=None, car=7, epoch=1):
    """Return a parsed state packet: odom and on_map are (x, y, yaw) or None."""
    x, y, yaw = odom if odom is not None else (None, None, None)
    map_x, map_y, map_yaw = on_map if on_map is not None else (None, None, None)
    return {'ver': 1, 'type': 'bsm', 'id': car, 'epoch': epoch, 'seq': seq, 't': t,
            'x': x, 'y': y, 'yaw': yaw, 'speed': 1.0, 'yaw_rate': 0.0, 'ax': 0.0,
            'cmd_speed': 0.0, 'cmd_steer': 0.0, 'batt': 0.5,
            'map_x': map_x, 'map_y': map_y, 'map_yaw': map_yaw}


def telemetry(x, y, z, yaw):
    """Return what AutoDRIVE's simulator sends: V1 Position is its GPS, above the pose."""
    return {'V1 Position': '%.4f %.4f %.4f' % (x, y, z + view.AUTODRIVE_GPS_HEIGHT),
            'V1 Orientation Quaternion': '0 0 %.6f %.6f' % (math.sin(yaw / 2), math.cos(yaw / 2))}


def sim_pose(command):
    """Return (x, y, z, yaw) from a CoSim command."""
    yaw = 2 * math.atan2(float(command['V1 RotZ']), float(command['V1 RotW']))
    return (float(command['V1 PosX']), float(command['V1 PosY']), float(command['V1 PosZ']),
            yaw)


# -- smoothing between packets ------------------------------------------------

def test_drawn_moves_from_the_pose_before_to_the_latest_by_the_cars_clock():
    fleet = view.Fleet({})
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 0.0)), 'a', 10.0)
    car = fleet.cars[7]
    assert view.drawn(car, 10.05) == (0.0, 0.0, 0.0)  # one packet: the pose itself
    fleet.update(bsm(2, 100.1, (0.1, 0.0, 0.0)), 'a', 10.1)
    assert view.drawn(car, 10.1)[0] == pytest.approx(0.0)
    assert view.drawn(car, 10.15)[0] == pytest.approx(0.05)
    assert view.drawn(car, 10.4)[0] == pytest.approx(0.1)  # a late packet: hold the latest


def test_drawn_turns_the_short_way_across_pi():
    fleet = view.Fleet({})
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 3.1)), 'a', 10.0)
    fleet.update(bsm(2, 100.1, (0.0, 0.0, -3.1)), 'a', 10.1)
    yaw = view.drawn(fleet.cars[7], 10.15)[2]
    assert math.cos(yaw) == pytest.approx(-1.0)


def test_packets_together_are_spaced_by_the_cars_clock():
    fleet = view.Fleet({})
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 0.0)), 'a', 10.0)
    fleet.update(bsm(2, 100.1, (0.1, 0.0, 0.0)), 'a', 10.2)
    fleet.update(bsm(3, 100.2, (0.2, 0.0, 0.0)), 'a', 10.2)  # same receive tick
    car = fleet.cars[7]
    assert view.drawn(car, 10.2)[0] == pytest.approx(0.1)
    assert view.drawn(car, 10.25)[0] == pytest.approx(0.15)


def test_drawn_shows_the_latest_when_the_cars_clock_steps_back():
    fleet = view.Fleet({})
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 0.0)), 'a', 10.0)
    fleet.update(bsm(2, 99.0, (0.1, 0.0, 0.0)), 'a', 10.1)  # a time sync stepped it back
    assert view.drawn(fleet.cars[7], 10.1) == (0.1, 0.0, 0.0)


# -- which packets count, and in which frame ----------------------------------

def test_a_late_or_repeated_packet_is_dropped_and_a_restart_is_not():
    fleet = view.Fleet({})
    fleet.update(bsm(3, 100.3, (0.3, 0.0, 0.0)), 'a', 10.0)
    fleet.update(bsm(2, 100.2, (0.2, 0.0, 0.0)), 'a', 10.1)
    fleet.update(bsm(3, 100.3, (9.0, 0.0, 0.0)), 'a', 10.1)
    assert fleet.cars[7]['sent'] == (0.3, 0.0, 0.0)
    fleet.update(bsm(0, 5.0, (1.0, 0.0, 0.0), epoch=2), 'a', 10.2)  # the car restarted
    assert fleet.cars[7]['sent'] == (1.0, 0.0, 0.0)


def test_odometry_is_placed_and_a_map_pose_is_not():
    fleet = view.Fleet({7: (1.0, 2.0, math.pi / 2)})
    fleet.update(bsm(1, 100.0, (1.0, 0.0, 0.0)), 'a', 10.0)
    assert fleet.cars[7]['frame'] == 'odom'
    assert fleet.cars[7]['pose'] == pytest.approx((1.0, 3.0, math.pi / 2))
    fleet.update(bsm(2, 100.1, (1.1, 0.0, 0.0), on_map=(5.0, 5.0, 0.5)), 'a', 10.1)
    assert fleet.cars[7]['frame'] == 'map'
    assert fleet.cars[7]['pose'] == (5.0, 5.0, 0.5)


def test_localizing_starts_a_new_path_and_does_not_smooth_across_frames():
    fleet = view.Fleet({})
    for i in range(5):
        fleet.update(bsm(i, 100.0 + i / 10, (i * 0.1, 0.0, 0.0)), 'a', 10.0 + i / 10)
    car = fleet.cars[7]
    assert len(car['trail']) == 5
    fleet.update(bsm(5, 100.5, (0.5, 0.0, 0.0), on_map=(3.0, 4.0, 1.0)), 'a', 10.5)
    assert car['trail'] == [(3.0, 4.0)]
    assert car['before'] is None
    assert view.drawn(car, 10.5) == (3.0, 4.0, 1.0)
    fleet.update(bsm(6, 100.6, (0.6, 0.0, 0.0)), 'a', 10.6)  # lost the map
    assert car['frame'] == 'odom' and car['trail'] == [(0.6, 0.0)]


def test_a_packet_without_a_pose_keeps_the_last_pose():
    fleet = view.Fleet({})
    fleet.update(bsm(1, 100.0, (1.0, 0.0, 0.0)), 'a', 10.0)
    fleet.update(bsm(2, 100.1), 'a', 10.1)
    car = fleet.cars[7]
    assert car['pose'] == (1.0, 0.0, 0.0) and car['seen'] == 10.1
    snap = fleet.snapshot(10.2)['7']
    assert snap['frame'] == 'odom' and snap['age'] == pytest.approx(0.1)


# -- AutoDRIVE co-simulation poses -------------------------------------------

START = (18.66, -3.25, 0.039, math.radians(-30))


def test_the_simulator_waits_idle_until_the_car_is_heard():
    projector = view.Projector(view.Fleet({}), 7, None)
    command = projector.reply(telemetry(*START))
    assert command['V1 CoSim'] == '0'
    assert projector.sim_origin == pytest.approx(START)  # the pose, below the GPS


def test_the_cars_moves_are_moved_onto_the_simulators_start():
    fleet = view.Fleet({})
    projector = view.Projector(fleet, 7, None)
    # Received long ago by the monotonic clock, so the reply holds the latest pose.
    fleet.update(bsm(1, 100.0, (0.5, -0.3, math.radians(20))), 'a', 0.0)
    assert sim_pose(projector.reply(telemetry(*START))) == pytest.approx(START, abs=1e-4)
    ahead = (0.5 + math.cos(math.radians(20)), -0.3 + math.sin(math.radians(20)),
             math.radians(20 + 90))
    fleet.update(bsm(2, 100.1, ahead), 'a', 0.0)
    x, y, z, yaw = sim_pose(projector.reply(telemetry(*START)))
    # one metre along the simulated car's own heading, then the same quarter turn left
    assert (x, y, z) == pytest.approx((START[0] + math.cos(START[3]),
                                       START[1] + math.sin(START[3]), START[2]), abs=1e-4)
    assert math.cos(yaw - (START[3] + math.pi / 2)) == pytest.approx(1.0)


def test_at_puts_the_simulated_car_elsewhere():
    fleet = view.Fleet({})
    projector = view.Projector(fleet, 7, (1.0, 2.0, math.pi))
    fleet.update(bsm(1, 100.0, (5.0, 5.0, 0.0)), 'a', 0.0)
    assert sim_pose(projector.reply(telemetry(*START))) == pytest.approx(
        (1.0, 2.0, START[2], math.pi), abs=1e-4)


def test_localizing_re_anchors_without_a_jump():
    fleet = view.Fleet({})
    projector = view.Projector(fleet, 7, None)
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 0.0)), 'a', 0.0)
    projector.reply(telemetry(*START))
    fleet.update(bsm(2, 100.1, (0.3, 0.0, 0.0)), 'a', 0.0)
    before = sim_pose(projector.reply(telemetry(*START)))
    # Localized: the map pose is nowhere near the odometry, the sim car stays put.
    fleet.update(bsm(3, 100.2, (0.3, 0.0, 0.0), on_map=(10.0, -4.0, 2.0)), 'a', 0.0)
    assert sim_pose(projector.reply(telemetry(*START))) == pytest.approx(before, abs=1e-4)
    # and moves as the car moves on the map: half a metre along its heading there
    fleet.update(bsm(4, 100.3, (0.8, 0.0, 0.0),
                     on_map=(10.0 + 0.5 * math.cos(2.0), -4.0 + 0.5 * math.sin(2.0), 2.0)),
                 'a', 0.0)
    x, y, _, _ = sim_pose(projector.reply(telemetry(*START)))
    assert (x, y) == pytest.approx((before[0] + 0.5 * math.cos(before[3]),
                                    before[1] + 0.5 * math.sin(before[3])), abs=1e-4)


# -- saved maps ---------------------------------------------------------------

def write_map(folder, pixels, width, height, yaml_extra=None, header=None):
    """Write a map the way nav2's map_saver does; return the yaml path."""
    header = header or b'P5\n# CREATOR: map_saver.cpp 0.050 m/pix\n%d %d\n255\n' % (width, height)
    (folder / 'room.pgm').write_bytes(header + bytes(pixels))
    keys = {'image': 'room.pgm', 'mode': 'trinary', 'resolution': '0.05',
            'origin': '[-2, -1.5, 0]', 'negate': '0', 'occupied_thresh': '0.65',
            'free_thresh': '0.25'}
    keys.update(yaml_extra or {})
    path = folder / 'room.yaml'
    path.write_text(''.join(f'{k}: {v}\n' for k, v in keys.items() if v is not None))
    return str(path)


def test_read_map_keeps_map_savers_three_values_apart(tmp_path):
    meta, cells = view.read_map(write_map(tmp_path, [0, 254, 205, 254], 2, 2))
    assert meta == {'width': 2, 'height': 2, 'resolution': 0.05, 'origin': [-2.0, -1.5, 0.0],
                    'name': 'room.yaml'}
    # occupied, free, and unknown: 205 stays unknown though Nav2 would load it free
    assert list(cells) == [2, 1, 0, 1]


def test_read_map_follows_negate_and_the_file_thresholds(tmp_path):
    path = write_map(tmp_path, [0, 254, 205, 128], 4, 1, {'negate': '1'})
    assert list(view.read_map(path)[1]) == [1, 2, 2, 0]


def test_a_pgm_header_may_carry_comments_anywhere(tmp_path):
    path = write_map(tmp_path, [0, 254, 205], 3, 1, header=b'P5\n# a\n3 # b\n1\n# c\n255\n')
    assert list(view.read_map(path)[1]) == [2, 1, 0]


@pytest.mark.parametrize('extra, pixels, header, reason', [
    ({'mode': 'raw'}, [0], None, 'mode raw'),
    ({'free_thresh': None}, [0], None, 'no free_thresh'),
    ({}, [0], b'\x89PNG\r\n\x1a\n', 'not a binary PGM'),
    ({}, [0], b'P5\n2 2\n255\n', 'ends before its 2x2 pixels'),
])
def test_read_map_refuses_what_it_cannot_draw(tmp_path, extra, pixels, header, reason):
    with pytest.raises(ValueError) as err:
        view.read_map(write_map(tmp_path, pixels, 1, 1, extra, header))
    assert reason in str(err.value)


def test_parse_place():
    car, (x, y, yaw) = view.parse_place('2:1.5,0,90')
    assert (car, x, y) == (2, 1.5, 0.0)
    assert yaw == pytest.approx(math.pi / 2)
