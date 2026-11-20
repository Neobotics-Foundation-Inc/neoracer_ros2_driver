"""
Command, timeout, and actuation tests for the web teleop dashboard (teleop.py).

Runs without a ROS graph: Node.__init__ and the create_* factories are
stubbed, so the real TeleopNode.__init__ still sets every field and the
timer callback is driven by hand with an explicit clock. Scans follow the
NeoRacer's LakiBeam (1440 points over 360 degrees, scan zero at the nose),
and frames are JPEG, as the camera node sends them.
"""

import json
import math
from pathlib import Path

from conftest import load_script
import cv2
import numpy as np
import pytest
from sensor_msgs.msg import BatteryState, Image, LaserScan
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)

BASE = Path(__file__).resolve().parents[1] / 'scripts' / 'dashboards' / 'webteleop'
# Loaded by the module fixture below, after the session fixtures have moved
# the data root and the car profile to temporary folders.
tp = None

SHIPPED = (BASE / 'teleop.yaml').read_text()


class Recorder:
    """Stands in for the /drive publisher."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture(scope='module', autouse=True)
def _teleop():
    global tp
    tp = load_script('teleop')
    return tp


@pytest.fixture(autouse=True)
def params(monkeypatch, tmp_path):
    """Fresh parameters per test; saves go to a temporary car profile file."""
    monkeypatch.setattr(tp, 'YAML_PATH', tmp_path / 'profile' / 'teleop.yaml')
    monkeypatch.setattr(tp, 'LIDAR_MOUNT_YAW_DEG', 0.0)
    tp._display.update(text='', pending=False)
    monkeypatch.setattr(tp, 'LOG_DIR', tmp_path / 'logs')
    tp._params.clear()
    tp._marks.clear()
    tp._rows.clear()
    tp._hist.clear()
    tp._cmd.update(fwd=0, turn=0, stamp=0.0)
    tp.load_params()
    tp._params.update(speed=0.5, angle=0.8)
    yield tp._params


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(tp.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(
        tp.TeleopNode, 'create_publisher', lambda self, *a, **k: Recorder(), raising=False
    )
    monkeypatch.setattr(
        tp.TeleopNode, 'create_subscription', lambda self, *a, **k: None, raising=False
    )
    monkeypatch.setattr(tp.TeleopNode, 'create_timer', lambda self, *a, **k: None, raising=False)
    return tp.TeleopNode()


def drive(node, fwd, turn, at=100.0, tick=None):
    """Post a command at time `at`, then run the timer at `tick` (default: same instant)."""
    tp.set_cmd({'fwd': fwd, 'turn': turn}, now=at)
    node._actuate(now=at if tick is None else tick)
    return node._pub.msgs[-1]


# ---- parameters ----


def test_speed_is_capped_both_ways():
    assert tp.clamp_speed(5.0) == tp.MAX_SPEED
    assert tp.clamp_speed(-5.0) == -tp.MAX_SPEED


def test_shipped_yaml_cannot_drive():
    import yaml

    assert yaml.safe_load(SHIPPED)['speed'] == 0.0


def test_save_keeps_comments(params):
    params.update(speed=0.35, angle=0.6)
    tp.save_params()
    text = tp.YAML_PATH.read_text()
    assert (BASE / 'teleop.yaml').read_text() == SHIPPED  # defaults untouched
    assert 'the buttons do' in text  # comments survive
    tp._params.clear()
    tp.load_params()
    assert tp._params['speed'] == 0.35
    assert tp._params['angle'] == 0.6
    assert tp._params['camera_topic'] == '/camera/color'


def test_update_clamps(params):
    tp.update_params({'speed': 4.0, 'angle': -1.0})
    assert params['speed'] == tp.MAX_SPEED
    assert params['angle'] == 0.0
    assert tp.update_params({'speed': 1.0}) == []
    assert tp.update_params({'speed': 0.2}) == ['speed 0.2']


# ---- command mapping ----


def test_forward_publishes_the_speed_slider(node, params):
    msg = drive(node, 1, 0)
    assert msg.drive.speed == pytest.approx(0.5)
    assert msg.drive.steering_angle == 0.0
    assert tp._state['fwd'] == 1 and tp._state['held']


def test_back_is_negative_speed(node):
    assert drive(node, -1, 0).drive.speed == pytest.approx(-0.5)


def test_left_turns_left_on_the_wire(node):
    # Student convention: left is negative; the wire is negated, so left is
    # positive on /drive (REP-103, physically verified on the sibling dashboards).
    assert drive(node, 0, -1).drive.steering_angle == pytest.approx(0.8)
    assert drive(node, 0, 1).drive.steering_angle == pytest.approx(-0.8)


def test_forward_and_left_at_once(node):
    msg = drive(node, 1, -1)
    assert msg.drive.speed == pytest.approx(0.5)
    assert msg.drive.steering_angle == pytest.approx(0.8)
    assert (tp._state['fwd'], tp._state['turn']) == (1, -1)


def test_axes_are_squashed_to_unit(node):
    tp.set_cmd({'fwd': 7, 'turn': -3})
    assert (tp._cmd['fwd'], tp._cmd['turn']) == (1, -1)
    tp.set_cmd({})
    assert (tp._cmd['fwd'], tp._cmd['turn']) == (0, 0)


def test_release_stops(node):
    drive(node, 1, 1)
    msg = drive(node, 0, 0)
    assert msg.drive.speed == 0.0 and msg.drive.steering_angle == 0.0
    assert not tp._state['held']


def test_speed_zero_means_no_motion_but_steering(node, params):
    params['speed'] = 0.0
    msg = drive(node, 1, 1)
    assert msg.drive.speed == 0.0
    assert msg.drive.steering_angle == pytest.approx(-0.8)


# ---- timeout ----


def test_stale_command_drives_zero(node, params):
    params['cmd_timeout'] = 0.5
    msg = drive(node, 1, -1, at=100.0, tick=100.4)
    assert msg.drive.speed == pytest.approx(0.5)
    assert not tp._state['timed_out']
    node._actuate(now=100.6)
    msg = node._pub.msgs[-1]
    assert msg.drive.speed == 0.0 and msg.drive.steering_angle == 0.0
    assert tp._state['timed_out']
    assert not tp._state['held']


def test_a_fresh_command_clears_the_timeout(node, params):
    params['cmd_timeout'] = 0.5
    drive(node, 1, 0, at=100.0, tick=101.0)
    assert tp._state['timed_out']
    drive(node, 1, 0, at=101.0)
    assert not tp._state['timed_out']
    assert node._pub.msgs[-1].drive.speed == pytest.approx(0.5)


def test_a_released_command_never_reads_as_timed_out(node):
    drive(node, 0, 0, at=100.0, tick=200.0)
    assert not tp._state['timed_out']


# ---- sensors ----


def make_scan(points=1440, span_deg=360.0, fill=2.0):
    """Build a scan shaped like this car's LakiBeam: 1440 points over 360 degrees."""
    msg = LaserScan()
    msg.angle_min = -math.radians(span_deg) / 2
    msg.angle_max = math.radians(span_deg) / 2
    msg.angle_increment = math.radians(span_deg) / points
    msg.range_min, msg.range_max = 0.05, 12.0
    msg.ranges = [fill] * points
    return msg


def place(msg, raw_deg, value):
    """Put `value` at the ray nearest raw angle `raw_deg`; return its index."""
    i = round((math.radians(raw_deg) - msg.angle_min) / msg.angle_increment) % len(msg.ranges)
    ranges = list(msg.ranges)
    ranges[i] = value
    msg.ranges = ranges
    return i


def test_scan_is_thinned(node):
    node._scan_cb(make_scan(points=360, span_deg=360.0))
    assert len(tp._state['scan']) == 120


def test_the_fleet_mount_puts_the_nose_at_raw_0():
    # config/frames.yaml ships laser_yaw_deg 0: the LakiBeam faces forward.
    assert tp.lidar_mount_yaw_deg(BASE.parents[2] / 'config') == 0.0


def test_the_mount_yaw_comes_from_frames_yaml(tmp_path):
    # A lidar turned 2 deg right sees the nose at raw +2.
    (tmp_path / 'frames.yaml').write_text('laser_x_mm: 99.62\nlaser_yaw_deg: -2.0\n')
    assert tp.lidar_mount_yaw_deg(tmp_path) == pytest.approx(2.0)


def test_object_in_front_reads_as_zero_degrees(node):
    msg = make_scan()
    i = place(msg, 0.0, 1.0)
    assert node._student_deg(msg, i) == pytest.approx(0.0, abs=0.5)


def test_object_off_the_right_reads_as_positive(node):
    # Scan angles turn counterclockwise, so the car's right is raw -90.
    msg = make_scan()
    i = place(msg, -90.0, 1.0)
    assert node._student_deg(msg, i) == pytest.approx(90.0, abs=0.5)


def test_left_and_right_are_not_mirrored(node):
    """A handedness flip would put the whole view in a mirror."""
    msg = make_scan()
    right = place(msg, -90.0, 2.0)
    left = place(msg, 90.0, 3.0)
    assert node._student_deg(msg, right) == pytest.approx(90.0, abs=0.5)
    assert node._student_deg(msg, left) == pytest.approx(-90.0, abs=0.5)


def test_a_measured_yaw_turns_the_view(node, monkeypatch):
    # laser_yaw_deg 3: the lidar is turned 3 deg left, so the nose is raw -3.
    monkeypatch.setattr(tp, 'LIDAR_MOUNT_YAW_DEG', -3.0)
    msg = make_scan()
    for raw in (-150, -90, -30, 0, 30, 90, 150):
        i = place(msg, raw, 1.0)
        assert node._student_deg(msg, i) == pytest.approx(-3.0 - raw, abs=0.5)


def test_the_blank_rear_is_not_drawn(node):
    # The LakiBeam's rear 90 degrees always read inf.
    msg = make_scan()
    msg.ranges = [
        float('inf') if abs(math.degrees(msg.angle_min + i * msg.angle_increment)) > 135.0 else 2.0
        for i in range(len(msg.ranges))
    ]
    node._scan_cb(msg)
    assert tp._state['scan']
    assert all(abs(deg) <= 135.5 for deg, _ in tp._state['scan'])


def test_270_degree_scan_still_maps(node):
    """Nothing here assumes a sweep width; the angle comes from the message."""
    msg = make_scan(points=1080, span_deg=270.0)
    i = place(msg, -90.0, 2.0)
    assert node._student_deg(msg, i) == pytest.approx(90.0, abs=0.5)


def test_upstreams_aft_facing_lidar_maps_with_180(node, monkeypatch):
    monkeypatch.setattr(tp, 'LIDAR_MOUNT_YAW_DEG', 180.0)
    msg = make_scan(points=1080)
    i = place(msg, 180.0, 1.5)
    assert node._student_deg(msg, i) == pytest.approx(0.0, abs=0.5)


def test_camera_preview_is_encoded(node):
    img = np.full((480, 640, 3), 90, np.uint8)
    msg = Image()
    msg.encoding = 'jpeg'
    msg.height, msg.width = 480, 640
    msg.data = cv2.imencode('.jpg', img)[1].tobytes()
    node._last_preview = 0.0
    node._image_cb(msg)
    assert tp._preview[:2] == b'\xff\xd8'
    assert tp._state['res'] == [640, 480]


def test_there_is_no_depth_stream(monkeypatch, params):
    """The NeoRacer's camera has no depth, so nothing subscribes to one."""
    topics = []
    monkeypatch.setattr(tp.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(
        tp.TeleopNode, 'create_publisher', lambda self, *a, **k: Recorder(), raising=False
    )
    monkeypatch.setattr(
        tp.TeleopNode,
        'create_subscription',
        lambda self, kind, topic, *a, **k: topics.append(topic),
        raising=False,
    )
    monkeypatch.setattr(tp.TeleopNode, 'create_timer', lambda self, *a, **k: None, raising=False)
    tp.TeleopNode()
    assert not any('depth' in t for t in topics)
    assert params['camera_topic'] in topics
    assert 'depth_topic' not in params


def make_detection(cx, cy, w, h, label='person', score=0.9):
    d = Detection2D()
    d.bbox = BoundingBox2D()
    d.bbox.center = Pose2D()
    d.bbox.center.position = Point2D(x=float(cx), y=float(cy))
    d.bbox.size_x, d.bbox.size_y = float(w), float(h)
    hyp = ObjectHypothesisWithPose()
    hyp.hypothesis = ObjectHypothesis()
    hyp.hypothesis.class_id = label
    hyp.hypothesis.score = float(score)
    d.results = [hyp]
    return d


def det_array(*dets):
    msg = Detection2DArray()
    msg.detections = list(dets)
    return msg


def test_detections_leave_as_fractions_of_the_frame(node):
    """Pixels in, fractions out: the browser scales them to its own preview."""
    tp._state['res'] = [640, 480]
    # Centred 320x240 box: a quarter in from each edge, half the frame wide.
    node._det_cb(det_array(make_detection(320, 240, 320, 240)))
    x, y, w, h, label, score = tp._state['dets'][0]
    assert (x, y, w, h) == (0.25, 0.25, 0.5, 0.5)
    assert label == 'person'
    assert score == 0.9


def test_a_corner_detection_stays_inside_the_frame(node):
    tp._state['res'] = [640, 480]
    node._det_cb(det_array(make_detection(50, 40, 100, 80)))
    x, y, w, h = tp._state['dets'][0][:4]
    assert (x, y) == (0.0, 0.0)
    # Boxes ship rounded to 4 places: 0.0001 of a 368 px view is 0.04 px.
    assert w == pytest.approx(100 / 640, abs=5e-5)
    assert h == pytest.approx(80 / 480, abs=5e-5)


def test_detections_before_the_first_frame_are_dropped(node):
    """Without a frame size there is nothing to normalize against."""
    tp._state['res'] = [0, 0]
    tp._state['dets'] = []
    node._det_cb(det_array(make_detection(320, 240, 320, 240)))
    assert tp._state['dets'] == []


def test_a_detection_with_no_hypothesis_still_draws(node):
    tp._state['res'] = [640, 480]
    d = Detection2D()
    d.bbox = BoundingBox2D()
    d.bbox.center = Pose2D()
    d.bbox.center.position = Point2D(x=320.0, y=240.0)
    d.bbox.size_x, d.bbox.size_y = 64.0, 48.0
    d.results = []
    node._det_cb(det_array(d))
    assert tp._state['dets'][0][4:] == ['', 0.0]


def test_stale_detections_are_cleared(node):
    """A frozen box on a live frame would be a lie about what is out there."""
    tp._state['res'] = [640, 480]
    node._det_available = True
    node._det_cb(det_array(make_detection(320, 240, 320, 240)))
    node._expire_detections(node._last_det + 0.1)
    assert tp._state['det_ok'] is True
    assert tp._state['dets']
    node._expire_detections(node._last_det + tp.DET_TIMEOUT_S + 0.1)
    assert tp._state['det_ok'] is False
    assert tp._state['dets'] == []


def test_a_car_with_no_detector_never_subscribes(monkeypatch, params):
    topics = []
    monkeypatch.setattr(tp.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(
        tp.TeleopNode, 'create_publisher', lambda self, *a, **k: Recorder(), raising=False
    )
    monkeypatch.setattr(
        tp.TeleopNode,
        'create_subscription',
        lambda self, kind, topic, *a, **k: topics.append(topic),
        raising=False,
    )
    monkeypatch.setattr(tp.TeleopNode, 'create_timer', lambda self, *a, **k: None, raising=False)
    params['detections_topic'] = ''
    node = tp.TeleopNode()
    assert node._det_available is False
    assert '/edgetpu/inference' not in topics


def test_rows_and_history_record_the_command(node):
    node._enc = 1.25
    drive(node, 1, -1)
    assert tp._rows[-1].strip().split(',')[1:] == ['1', '-1', '0.500', '-0.800', '1.250']
    assert tp._hist[-1][1:] == [1.25, 0.5]


def test_thin_keeps_the_newest_row():
    rows = [[i, 0.0, 0.0] for i in range(1000)]
    sent = tp._thin(rows)
    assert len(sent) < len(rows)
    assert sent[-1] is rows[-1]


# ---- sensors and dot matrix ----


def test_heading_is_zero_facing_north_and_counterclockwise_positive():
    assert tp.heading_deg([20.0, 0.0, -40.0]) == pytest.approx(0.0)
    # Field along -y in the body: north is to the car's right, so the car
    # faces 90 deg counterclockwise of north.
    assert tp.heading_deg([0.0, -20.0, -40.0]) == pytest.approx(90.0)


def test_dot_matrix_text_is_sent_once(node):
    # The NeoRacer's 8x8 panel holds and scrolls the last text; sending it
    # again would restart the scroll.
    tp.set_text('hi')
    node._actuate(now=100.0)
    node._actuate(now=101.5)
    assert [m.data for m in node._pub_text.msgs] == ['hi']


def test_dot_matrix_text_is_capped(node):
    tp.set_text('x' * 400)
    node._actuate(now=100.0)
    assert len(node._pub_text.msgs[-1].data) == tp.MAX_TEXT


def test_there_is_no_pixel_or_led_strip_publisher(monkeypatch):
    topics = []
    monkeypatch.setattr(tp.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(
        tp.TeleopNode,
        'create_publisher',
        lambda self, kind, topic, *a, **k: topics.append(topic) or Recorder(),
        raising=False,
    )
    monkeypatch.setattr(tp.TeleopNode, 'create_subscription', lambda *a, **k: None, raising=False)
    monkeypatch.setattr(tp.TeleopNode, 'create_timer', lambda *a, **k: None, raising=False)
    tp.TeleopNode()
    assert sorted(topics) == ['/dotmatrix/text', '/drive']


def battery(voltage, current):
    msg = BatteryState()
    msg.voltage, msg.current = float(voltage), float(current)
    return msg


def test_a_missing_current_reads_as_none_and_the_state_stays_json(node):
    # The NeoRacer has no current sensor: /battery/state carries NaN current.
    node._battery_cb(battery(12.18, float('nan')))
    node._actuate(now=100.0)
    assert tp._state['volts'] == 12.18
    assert tp._state['amps'] is None
    json.dumps(tp._state, allow_nan=False)


def test_a_measured_current_reads_as_draw(node):
    node._battery_cb(battery(11.5, -2.25))
    node._actuate(now=100.0)
    assert (tp._state['volts'], tp._state['amps']) == (11.5, 2.25)


def test_the_profile_loads_over_the_defaults(params):
    tp.YAML_PATH.parent.mkdir(parents=True)
    tp.YAML_PATH.write_text('speed: 0.3\n')
    tp.load_params()
    assert tp._params['speed'] == 0.3
    assert tp._params['angle'] == 1.0  # from the defaults


def test_logs_go_to_the_data_root(data_root_dir, monkeypatch):
    monkeypatch.undo()  # the module's own value, not the per-test folder
    assert tp.LOG_DIR == data_root_dir / 'dashboards' / 'teleop'
