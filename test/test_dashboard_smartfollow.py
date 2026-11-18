"""
Controller and arbiter tests for the smart follow dashboard (smartfollow.py).

Runs without a ROS graph: Node.__init__ and the create_* factories are
stubbed, so the real SmartFollowNode.__init__ still sets every field and the
callbacks are driven with hand-built messages. Scans follow the LakiBeam's
pattern: 1440 points over 360 degrees, scan zero at the nose.
"""

import math
import time

from conftest import load_script
import pytest
from sensor_msgs.msg import LaserScan
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)

# Loaded by the module fixture below, after the session fixtures have moved
# the data root and the car profile to temporary folders.
sf = None


@pytest.fixture(scope='module', autouse=True)
def _smartfollow():
    global sf
    sf = load_script('smartfollow')
    return sf


class Recorder:
    """Stands in for the /drive publisher."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture(autouse=True)
def params(monkeypatch, tmp_path):
    """Fresh parameters per test; saves go to a temporary car profile file."""
    monkeypatch.setattr(sf, 'YAML_PATH', tmp_path / 'profile' / 'smartfollow.yaml')
    monkeypatch.setattr(sf, 'LOG_DIR', tmp_path / 'logs')
    sf._params.clear()
    sf._marks.clear()
    sf._rows.clear()
    sf._hist.clear()
    sf.load_params()
    return sf._params


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(sf.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(
        sf.SmartFollowNode, 'create_publisher', lambda self, *a, **k: Recorder(), raising=False
    )
    monkeypatch.setattr(
        sf.SmartFollowNode, 'create_subscription', lambda self, *a, **k: None, raising=False
    )
    monkeypatch.setattr(
        sf.SmartFollowNode, 'create_timer', lambda self, *a, **k: None, raising=False
    )
    n = sf.SmartFollowNode()
    n._enc_stamp = time.monotonic()  # pretend odometry is fresh
    return n


def make_scan(range_at):
    """Build a LakiBeam scan of 1440 samples; range_at(student_deg) gives meters."""
    msg = LaserScan()
    msg.angle_min = -math.pi
    msg.angle_increment = 2 * math.pi / 1440
    msg.range_min = 0.05
    msg.range_max = 25.0
    ranges = []
    for i in range(1440):
        raw = msg.angle_min + i * msg.angle_increment
        ranges.append(float(range_at(-math.degrees(raw))))
    msg.ranges = ranges
    return msg


def make_dets(boxes):
    """Build a Detection2DArray from (cx, cy, w, h, score, label) boxes."""
    dets = []
    for cx, cy, w, h, score, label in boxes:
        d = Detection2D()
        d.bbox = BoundingBox2D()
        d.bbox.center = Pose2D()
        d.bbox.center.position = Point2D(x=float(cx), y=float(cy))
        d.bbox.size_x = float(w)
        d.bbox.size_y = float(h)
        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis = ObjectHypothesis()
        hyp.hypothesis.score = float(score)
        hyp.hypothesis.class_id = label
        d.results = [hyp]
        dets.append(d)
    msg = Detection2DArray()
    msg.detections = dets
    return msg


# ---- parameters ----


def test_speed_is_capped():
    assert sf.set_max_speed(5.0) == sf.MAX_SPEED
    assert sf.set_max_speed(-1.0) == 0.0


def test_shipped_yaml_cannot_drive(params):
    assert params['wall']['speed'] == 0.0
    assert params['car']['speed'] == 0.0


def test_save_keeps_comments_and_block_scoping(params):
    shipped = sf.DEFAULTS_PATH.read_text()
    params['wall']['speed_kp'] = 0.11
    params['car']['speed_kp'] = 0.99
    params['target_hold'] = 1.5
    sf.save_params()
    assert sf.DEFAULTS_PATH.read_text() == shipped  # defaults untouched
    text = sf.YAML_PATH.read_text()
    assert '# damper. Raise this if the car wobbles.' in text  # comments survive
    sf._params.clear()
    sf.load_params()
    # The colliding key lands back in the block it came from.
    assert sf._params['wall']['speed_kp'] == 0.11
    assert sf._params['car']['speed_kp'] == 0.99
    assert sf._params['target_hold'] == 1.5
    assert sf._params['camera_topic'] == '/camera/color'  # non-numeric untouched


def test_update_group_namespaces_colliding_keys(params):
    sf._update_group('wall', sf.WALL_KEYS, {'speed': 0.3})
    assert params['wall']['speed'] == 0.3
    assert params['car']['speed'] == 0.0
    changed = sf._update_group('car', sf.CAR_KEYS, {'speed': 0.4})
    assert changed == ['car.speed 0.4']
    assert params['wall']['speed'] == 0.3


def test_update_group_clamps_speed(params):
    sf._update_group('car', sf.CAR_KEYS, {'speed': 9.0})
    assert params['car']['speed'] == sf.MAX_SPEED


# ---- wall following ----


def test_gap_follower_steers_toward_the_open_side(node):
    node._scan_cb(make_scan(lambda deg: 3.5 if deg > 5 else 0.6))
    assert node._wall_error > 0  # student convention: positive is right
    assert node._wall_steer_cmd > 0
    node._scan_cb(make_scan(lambda deg: 3.5 if deg < -5 else 0.6))
    assert node._wall_error < 0


def test_gap_follower_centers_in_a_symmetric_corridor(node):
    node._scan_cb(make_scan(lambda deg: 3.5 if abs(deg) < 45 else 0.6))
    assert abs(node._wall_error) < 0.1


def test_gap_follower_publishes_the_overlay(node, params):
    node._scan_cb(make_scan(lambda deg: 3.5))
    ov = sf._state['overlay']
    assert ov['width'] == params['wall']['width']
    assert ov['la'] == params['wall']['lookahead']
    assert 'target_deg' in ov


def test_zero_speed_gains_hold_the_slider_command(node, params):
    params['wall'].update(speed=0.4, speed_kp=0.0, speed_kd=0.0)
    scan = make_scan(lambda deg: 3.5)
    for _ in range(5):
        node._enc_stamp = time.monotonic()
        node._scan_cb(scan)
    assert node._wall_speed_cmd == 0.4


def test_stale_odometry_falls_back_to_the_slider_command(node, params):
    params['wall'].update(speed=0.4, speed_kp=0.08, speed_kd=0.03)
    node._enc_stamp = time.monotonic() - 5.0
    node._scan_cb(make_scan(lambda deg: 3.5))
    assert node._wall_speed_cmd == 0.4


def test_speed_gains_trim_toward_the_road_target(node, params):
    params['wall'].update(speed=1.0, speed_kp=0.08, speed_kd=0.03)
    node._enc = 2.0  # measured speed above the road target: trim pulls down
    scan = make_scan(lambda deg: 1.0)
    for _ in range(20):
        node._enc_stamp = time.monotonic()
        node._scan_cb(scan)
    assert node._wall_speed_cmd < 1.0


# ---- car following ----


def test_largest_box_above_threshold_becomes_the_target(node):
    node._res = (640, 480)
    node._det_cb(
        make_dets(
            [
                (100, 240, 40, 40, 0.9, 'car'),  # high score, small
                (500, 240, 200, 160, 0.8, 'car'),  # slightly lower score, largest
                (320, 240, 300, 300, 0.1, 'car'),  # biggest but below threshold
            ]
        )
    )
    assert sf._state['target']['w'] == 200
    assert node._car_error > 0  # target is right of center


def test_detections_below_threshold_leave_no_target(node):
    node._res = (640, 480)
    node._det_cb(make_dets([(320, 240, 100, 100, 0.1, 'car')]))
    assert sf._state['target'] is None
    assert sf._state['car_error'] is None


def test_pursuit_holds_its_command_when_the_target_vanishes(node):
    node._res = (640, 480)
    node._det_cb(make_dets([(500, 240, 200, 160, 0.9, 'car')]))
    steer, speed = node._car_steer_cmd, node._car_speed_cmd
    node._det_cb(make_dets([]))
    assert (node._car_steer_cmd, node._car_speed_cmd) == (steer, speed)


def test_throttle_stops_at_the_safety_area(node, params):
    params['car']['speed'] = 1.0
    node._res = (640, 480)
    # Just past the stop size, so the target speed clamps to zero.
    big = int(math.sqrt(params['car']['safety_area'] * 640 * 480)) + 2
    for _ in range(30):
        node._det_cb(make_dets([(320, 240, big, big, 0.9, 'car')]))
    assert node._car_speed_cmd == pytest.approx(0.0, abs=1e-3)


# ---- arbiter ----


def test_wall_follower_drives_when_nothing_was_ever_detected(node, params):
    params['wall']['speed'] = params['car']['speed'] = 1.0
    node._wall_steer_cmd, node._wall_speed_cmd = 0.4, 0.3
    node._car_steer_cmd, node._car_speed_cmd = -0.9, 0.8
    node._actuate()
    assert sf._state['state'] == 'wall'
    assert sf._state['since'] is None
    msg = node._pub.msgs[-1]
    assert msg.drive.speed == 0.3
    assert msg.drive.steering_angle == pytest.approx(-0.4)  # negated on the wire


def test_a_fresh_target_hands_over_to_car_following(node, params):
    params['wall']['speed'] = params['car']['speed'] = 1.0
    node._wall_steer_cmd, node._wall_speed_cmd = 0.4, 0.3
    node._car_steer_cmd, node._car_speed_cmd = -0.9, 0.8
    node._last_target_stamp = time.monotonic()
    node._actuate()
    assert sf._state['state'] == 'car'
    msg = node._pub.msgs[-1]
    assert msg.drive.speed == 0.8
    assert msg.drive.steering_angle == pytest.approx(0.9)


def test_the_wall_follower_takes_back_over_after_target_hold(node, params):
    params['target_hold'] = 0.2
    params['wall']['speed'] = params['car']['speed'] = 1.0
    node._wall_steer_cmd, node._wall_speed_cmd = 0.4, 0.3
    node._car_steer_cmd, node._car_speed_cmd = -0.9, 0.8
    node._last_target_stamp = time.monotonic() - 0.1
    node._actuate()
    assert sf._state['state'] == 'car'
    node._last_target_stamp = time.monotonic() - 0.3
    node._actuate()
    assert sf._state['state'] == 'wall'
    assert node._pub.msgs[-1].drive.speed == 0.3


def test_arbiter_output_is_capped(node, params):
    params['wall']['speed'] = 1.0
    node._wall_speed_cmd = 99.0
    node._actuate()
    assert node._pub.msgs[-1].drive.speed == sf.MAX_SPEED


def test_each_row_records_the_driving_controller(node):
    node._actuate()
    node._last_target_stamp = time.monotonic()
    node._actuate()
    assert [r.strip().split(',')[-1] for r in sf._rows] == ['0', '1']
    assert [h[2] for h in sf._hist] == [0, 1]


# ---- car profile and model ----


def test_a_car_profile_block_merges_over_the_defaults(params):
    sf.YAML_PATH.parent.mkdir(parents=True)
    sf.YAML_PATH.write_text('wall:\n  kp: 0.9\ntarget_hold: 1.0\n')
    sf.load_params()
    assert sf._params['wall']['kp'] == 0.9
    assert sf._params['wall']['kd'] == 4.0  # the rest of the block from the defaults
    assert sf._params['car']['angle_kp'] == 1.0
    assert sf._params['target_hold'] == 1.0


def test_logs_go_to_the_data_root(data_root_dir, monkeypatch):
    monkeypatch.undo()  # the module's own value, not the per-test folder
    assert sf.LOG_DIR == data_root_dir / 'dashboards' / 'smartfollow'


def test_the_model_name_takes_the_profile_override(tmp_path, car_profile_dir):
    (tmp_path / 'inference.yaml').write_text(
        'inference_node:\n  ros__parameters:\n    model_path: models/yolo.pt\n'
    )
    (car_profile_dir / 'params').mkdir(parents=True)
    (car_profile_dir / 'params' / 'inference.yaml').write_text(
        'inference_node:\n  ros__parameters:\n    model_path: cars.engine\n'
    )
    assert sf.read_model_name(tmp_path) == 'cars.engine'


def test_an_empty_block_in_the_profile_keeps_the_shipped_one(params):
    shipped_car = dict(sf._params['car'])
    sf.YAML_PATH.parent.mkdir(parents=True)
    sf.YAML_PATH.write_text('car:\nwall:\n  kp: 0.7\n')
    sf.load_params()
    assert sf._params['car'] == shipped_car
    assert sf._params['wall']['kp'] == 0.7


def test_a_slider_at_zero_stops_a_held_command(node, params):
    # The car follower holds its last command while it waits for a target.
    params['car']['speed'] = 0.0
    node._car_steer_cmd, node._car_speed_cmd = -0.5, 0.6
    node._last_target_stamp = time.monotonic()
    node._actuate()
    assert node._pub.msgs[-1].drive.speed == 0.0
