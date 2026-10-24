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


def sim_pose(command, vehicle=1):
    """Return (x, y, z, yaw) of one vehicle from a CoSim command."""
    v = 'V%d ' % vehicle
    yaw = 2 * math.atan2(float(command[v + 'RotZ']), float(command[v + 'RotW']))
    return (float(command[v + 'PosX']), float(command[v + 'PosY']), float(command[v + 'PosZ']),
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


def test_a_packet_that_comes_early_does_not_jump_the_drawn_car():
    fleet = view.Fleet({})
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 0.0)), 'a', 10.0)
    fleet.update(bsm(2, 100.1, (0.1, 0.0, 0.0)), 'a', 10.1)
    car = fleet.cars[7]
    shown = view.drawn(car, 10.13)  # 30 % of the way from the first pose to the second
    fleet.update(bsm(3, 100.2, (0.2, 0.0, 0.0)), 'a', 10.13)  # 70 ms early
    assert view.drawn(car, 10.13) == pytest.approx(shown)
    assert view.drawn(car, 10.18)[0] == pytest.approx((0.03 + 0.2) / 2)
    assert view.drawn(car, 10.5)[0] == pytest.approx(0.2)


def test_a_late_packet_goes_on_from_where_the_car_was_held():
    fleet = view.Fleet({})
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 0.0)), 'a', 10.0)
    fleet.update(bsm(2, 100.1, (0.1, 0.0, 0.0)), 'a', 10.1)
    car = fleet.cars[7]
    assert view.drawn(car, 10.35)[0] == pytest.approx(0.1)  # held at the latest
    fleet.update(bsm(3, 100.2, (0.2, 0.0, 0.0)), 'a', 10.35)  # 150 ms late
    assert view.drawn(car, 10.35)[0] == pytest.approx(0.1)
    assert view.drawn(car, 10.40)[0] == pytest.approx(0.15)


def test_packets_together_do_not_jump_the_drawn_car():
    fleet = view.Fleet({})
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 0.0)), 'a', 10.0)
    fleet.update(bsm(2, 100.1, (0.1, 0.0, 0.0)), 'a', 10.2)
    fleet.update(bsm(3, 100.2, (0.2, 0.0, 0.0)), 'a', 10.2)  # same receive tick
    car = fleet.cars[7]
    assert view.drawn(car, 10.2)[0] == pytest.approx(0.0)
    assert view.drawn(car, 10.25)[0] == pytest.approx(0.1)
    assert view.drawn(car, 10.3)[0] == pytest.approx(0.2)


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


def test_the_page_gets_a_copy_of_the_path():
    fleet = view.Fleet({})
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 0.0)), 'a', 10.0)
    trail = fleet.snapshot(10.0)['7']['trail']
    assert trail == fleet.cars[7]['trail'] and trail is not fleet.cars[7]['trail']


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


def test_a_restart_starts_a_new_path_and_the_simulated_car_goes_on_from_where_it_stands():
    fleet = view.Fleet({})
    projector = view.Projector(fleet, 7, None)
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 0.0)), 'a', 0.0)
    projector.reply(telemetry(*START))
    fleet.update(bsm(2, 100.1, (4.0, 0.0, 0.0)), 'a', 0.0)
    stood = sim_pose(projector.reply(telemetry(*START)))
    # The car's V2V restarted: a new epoch, its odometry from 0 again.
    fleet.update(bsm(0, 200.0, (0.0, 0.0, 0.0), epoch=2), 'a', 0.0)
    car = fleet.cars[7]
    assert car['before'] is None and car['trail'] == [(0.0, 0.0)]
    assert sim_pose(projector.reply(telemetry(*START))) == pytest.approx(stood, abs=1e-4)
    fleet.update(bsm(1, 200.1, (0.5, 0.0, 0.0), epoch=2), 'a', 0.0)
    x, y, _, _ = sim_pose(projector.reply(telemetry(*START)))
    assert (x, y) == pytest.approx((stood[0] + 0.5 * math.cos(stood[3]),
                                    stood[1] + 0.5 * math.sin(stood[3])), abs=1e-4)


@pytest.mark.parametrize('data', [
    {'foo': 1},
    'x',
    {'V1 Position': '', 'V1 Orientation Quaternion': '0 0 0 1'},
    {'V1 Position': '18,66 -3,25 0,01', 'V1 Orientation Quaternion': '0 0 0 1'},
])
def test_telemetry_without_the_neoracers_pose_is_refused_and_pins_no_start(data):
    projector = view.Projector(view.Fleet({}), 7, None)
    with pytest.raises(ValueError):
        projector.reply(data)
    assert projector.sim_origin is None


# -- two cars, two vehicles ---------------------------------------------------

SECOND = (18.66, -2.65, 0.039, math.radians(-30))  # vehicle 2 beside vehicle 1


def telemetry2(first, second):
    """Return the two-car scene's telemetry: both vehicles' poses."""
    out = telemetry(*first)
    other = telemetry(*second)
    out['V2 Position'] = other['V1 Position']
    out['V2 Orientation Quaternion'] = other['V1 Orientation Quaternion']
    return out


def two_projectors(fleet):
    shared = view.MapFrame()
    return [view.Projector(fleet, 7, None, 1, shared), view.Projector(fleet, 8, None, 2, shared)]


def test_every_answer_covers_both_vehicles_even_before_telemetry():
    reply = view.answer({}, [view.Projector(view.Fleet({}), 7, None)], [])
    for vehicle in (1, 2):
        for key in ('Throttle', 'Steering', 'Reset', 'CoSim'):
            assert reply['V%d %s' % (vehicle, key)] in ('0', 'False')


def test_two_cars_drive_two_vehicles_each_from_where_it_stands():
    fleet = view.Fleet({})
    projectors = two_projectors(fleet)
    fleet.update(bsm(1, 100.0, (5.0, 5.0, 0.0), car=7), 'a', 0.0)
    fleet.update(bsm(1, 100.0, (-3.0, 1.0, 1.0), car=8), 'a', 0.0)
    reply = view.answer(telemetry2(START, SECOND), projectors, [])
    assert sim_pose(reply, 1) == pytest.approx(START, abs=1e-4)
    assert sim_pose(reply, 2) == pytest.approx(SECOND, abs=1e-4)
    # car 8 drives a metre along its heading: vehicle 2 a metre along its own, vehicle 1 stays
    fleet.update(bsm(2, 100.1, (-3.0 + math.cos(1.0), 1.0 + math.sin(1.0), 1.0), car=8), 'a', 0.0)
    reply = view.answer(telemetry2(START, SECOND), projectors, [])
    assert sim_pose(reply, 1) == pytest.approx(START, abs=1e-4)
    assert sim_pose(reply, 2)[:2] == pytest.approx(
        (SECOND[0] + math.cos(SECOND[3]), SECOND[1] + math.sin(SECOND[3])), abs=1e-4)


def test_cars_on_one_map_keep_their_real_places_in_the_simulator():
    fleet = view.Fleet({})
    projectors = two_projectors(fleet)
    # both localized on one map, car 8 1.5 m ahead of car 7
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 0.0), on_map=(2.0, 1.0, 0.0), car=7), 'a', 0.0)
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 0.0), on_map=(3.5, 1.0, 0.0), car=8), 'a', 0.0)
    reply = view.answer(telemetry2(START, SECOND), projectors, [])
    # the first car on the map pins it where vehicle 1 stands; vehicle 2 goes
    # where the map puts car 8, 1.5 m ahead of vehicle 1, not to its own start
    assert sim_pose(reply, 1) == pytest.approx(START, abs=1e-4)
    x, y, z, yaw = sim_pose(reply, 2)
    assert (x, y, z) == pytest.approx((START[0] + 1.5 * math.cos(START[3]),
                                       START[1] + 1.5 * math.sin(START[3]), START[2]), abs=1e-4)
    assert math.cos(yaw - START[3]) == pytest.approx(1.0)


def test_a_one_car_scene_drives_vehicle_1_and_refuses_vehicle_2_once():
    fleet = view.Fleet({})
    projectors = two_projectors(fleet)
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 0.0), car=7), 'a', 0.0)
    fleet.update(bsm(1, 100.0, (0.0, 0.0, 0.0), car=8), 'a', 0.0)
    refused = []
    for _ in range(3):
        reply = view.answer(telemetry(*START), projectors, refused)
    assert reply['V1 CoSim'] == '1' and reply['V2 CoSim'] == '0'
    assert len(refused) == 1 and 'vehicle 2' in refused[0]


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


# -- a lane map (--lanes) -----------------------------------------------------

LANES_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                         'scripts', 'lanes')
sys.path.insert(0, LANES_DIR)
import lane_sim  # noqa: E402,I100
from neoracer_ros2_driver.lane_lock import LaneMap  # noqa: E402,I100


@pytest.fixture(scope='module')
def lane_map():
    return LaneMap.load(os.path.join(LANES_DIR, 'purdue_testbed.json'))


def odometry_packets(lane_map, start, lanes_on, seconds, epoch=1, turn=0.7, seed=1):
    """Return a simulated car's state packets: odometry in its own frame, turned by `turn`."""
    driven = lane_sim.route(lane_map, start, [], lanes_on)
    out = []
    c, s = math.cos(turn), math.sin(turn)
    for i, (t, x, y, yaw, lane, ls) in enumerate(lane_sim.drive(lane_map, driven, seed=seed)):
        if t > seconds:
            break
        out.append((bsm(i, 100.0 + t, (x * c - y * s, x * s + y * c, yaw + turn), epoch=epoch),
                    10.0 + t, lane, ls))
    return out


def test_a_car_on_a_lane_map_is_found_and_drawn_on_its_lane(lane_map):
    fleet = view.Fleet({}, lane_map)
    for packet, now, lane, s in odometry_packets(lane_map, 'loop2_fwd', 6, 30.0):
        fleet.update(packet, 'a', now)
    car = fleet.cars[7]
    x, y, _ = lane.pose(s)
    assert car['frame'] == 'lane'
    assert math.hypot(car['pose'][0] - x, car['pose'][1] - y) < 0.5 * lane_map.data['lane_width']
    assert fleet.snapshot(now)['7']['lane']['id'] == car['lane']['lane']


def test_a_restart_keeps_the_car_on_its_lane(lane_map):
    fleet = view.Fleet({}, lane_map, starts={7: ('loop2_fwd', 0.0)})
    packets = odometry_packets(lane_map, 'loop2_fwd', 6, 30.0)
    for packet, now, _, _ in packets[:150]:
        fleet.update(packet, 'a', now)
    # the car's V2V restarts: a new epoch, its odometry from 0 again where it stands
    x0, y0 = packets[150][0]['x'], packets[150][0]['y']
    for packet, now, lane, s in packets[150:]:
        p = dict(packet, epoch=2, x=packet['x'] - x0, y=packet['y'] - y0)
        fleet.update(p, 'a', now)
    car = fleet.cars[7]
    x, y, _ = lane.pose(s)
    assert car['frame'] == 'lane'
    assert math.hypot(car['pose'][0] - x, car['pose'][1] - y) < 0.5 * lane_map.data['lane_width']


def test_autodrive_places_a_locked_car_where_its_lane_puts_it(lane_map):
    fleet = view.Fleet({}, lane_map, starts={7: ('loop2_fwd', 0.0)})
    projector = view.Projector(fleet, 7, None)
    for packet, now, _, _ in odometry_packets(lane_map, 'loop2_fwd', 6, 10.0):
        fleet.update(packet, 'a', 0.0)
    command = projector.reply(telemetry(*START))
    car = fleet.cars[7]
    x, y, z, yaw = sim_pose(command)
    assert (x, y, z) == pytest.approx((car['sent'][0], car['sent'][1], START[2]), abs=1e-3)
    assert math.cos(yaw - car['sent'][2]) == pytest.approx(1.0)


def test_autodrive_keeps_a_car_still_being_found_where_it_stands(lane_map):
    fleet = view.Fleet({}, lane_map)
    projector = view.Projector(fleet, 7, None)
    for packet, now, _, _ in odometry_packets(lane_map, 'loop2_fwd', 6, 0.5):
        fleet.update(packet, 'a', 0.0)
    assert fleet.cars[7]['frame'] == 'searching'
    assert projector.reply(telemetry(*START))['V1 CoSim'] == '0'
