"""
Target choice, control and tuning tests for the pure pursuit dashboard (pursuit.py).

Runs without a ROS graph: Node.__init__ and the create_* factories are
stubbed, so the real PursuitNode.__init__ still sets every field, and the
detection callback and the timer are driven by hand.
"""

from conftest import load_script
import pytest
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesis,
    ObjectHypothesisWithPose,
    Point2D,
    Pose2D,
)
import yaml


class Recorder:
    """Stands in for the /drive publisher."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture(scope='module')
def pp():
    return load_script('pursuit')


@pytest.fixture
def params(pp, monkeypatch, tmp_path):
    """Fresh parameters per test; saves go to a temporary car profile file."""
    monkeypatch.setattr(pp, 'YAML_PATH', tmp_path / 'profile' / 'pursuit.yaml')
    pp._params.clear()
    pp.load_params()
    pp._params.update(speed=0.5, speed_kp=1.0, safety_area=0.2, angle_kp=1.0, angle_kd=0.0)
    return pp._params


@pytest.fixture
def node(pp, params, monkeypatch):
    monkeypatch.setattr(pp.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(
        pp.PursuitNode, 'create_publisher', lambda *a, **k: Recorder(), raising=False
    )
    monkeypatch.setattr(pp.PursuitNode, 'create_subscription', lambda *a, **k: None, raising=False)
    monkeypatch.setattr(pp.PursuitNode, 'create_timer', lambda *a, **k: None, raising=False)
    monkeypatch.setattr(pp, 'read_model_name', lambda config_dir=None: 'best.engine')
    return pp.PursuitNode()


def detection(cx, cy, w, h, score=0.9, label='car'):
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


def detections(*dets):
    msg = Detection2DArray()
    msg.detections = list(dets)
    return msg


def test_shipped_yaml_cannot_drive(pp):
    assert yaml.safe_load(pp.DEFAULTS_PATH.read_text())['speed'] == 0.0


def test_save_writes_the_profile_not_the_shipped_yaml(pp, params):
    shipped = pp.DEFAULTS_PATH.read_text()
    params.update(speed=0.3, angle_kp=1.5)
    pp.save_params()
    assert pp.DEFAULTS_PATH.read_text() == shipped
    text = pp.YAML_PATH.read_text()
    assert 'speed: 0.3' in text and 'angle_kp: 1.5' in text
    assert 'Raise this if the car wobbles' in text  # comments survive
    pp._params.clear()
    pp.load_params()
    assert pp._params['speed'] == 0.3 and pp._params['angle_kp'] == 1.5
    assert pp._params['detections_topic'] == '/edgetpu/inference'


def test_the_largest_confident_box_is_the_target(pp, node):
    node._det_cb(
        detections(
            detection(100, 240, 50, 50, score=0.9),
            detection(500, 240, 200, 200, score=0.3),  # largest, below the threshold
            detection(400, 240, 120, 120, score=0.6),
        )
    )
    assert pp._state['target']['cx'] == 400.0
    assert len(pp._state['detections']) == 2


def test_max_detections_keeps_the_best_scores(pp, node, params):
    params['max_detections'] = 1
    node._det_cb(
        detections(
            detection(100, 240, 50, 50, score=0.9), detection(400, 240, 120, 120, score=0.6)
        )
    )
    assert pp._state['target']['cx'] == 100.0


def test_a_target_on_the_right_steers_right_on_the_wire(node):
    node._det_cb(detections(detection(480, 240, 40, 40)))
    assert node._steer_cmd == pytest.approx(0.5)
    node._actuate()
    # Student convention: right is positive; the wire is REP-103, left positive.
    assert node._pub.msgs[-1].drive.steering_angle == pytest.approx(-0.5)


def test_a_target_on_the_left_steers_left_on_the_wire(node):
    node._det_cb(detections(detection(160, 240, 40, 40)))
    node._actuate()
    assert node._pub.msgs[-1].drive.steering_angle == pytest.approx(0.5)


def test_the_car_stops_at_the_safety_area(node, params):
    # A box covering a fifth of the frame is the stop size.
    side = (0.2 * 640 * 480) ** 0.5
    node._det_cb(detections(detection(320, 240, side, side)))
    assert node._speed_cmd == pytest.approx(0.0, abs=1e-6)


def test_a_small_target_drives_at_the_speed_cap(node, params):
    node._det_cb(detections(detection(320, 240, 4, 4)))
    assert node._speed_cmd == pytest.approx(0.5, abs=1e-3)
    assert node._speed_cmd <= 1.0


def test_with_no_target_the_last_command_keeps_going(node):
    node._det_cb(detections(detection(480, 240, 40, 40)))
    speed, steer = node._speed_cmd, node._steer_cmd
    node._det_cb(detections())
    node._actuate()
    assert node._pub.msgs[-1].drive.speed == pytest.approx(speed)
    assert node._pub.msgs[-1].drive.steering_angle == pytest.approx(-steer)


def test_the_model_name_takes_the_profile_override(pp, tmp_path, car_profile_dir):
    (tmp_path / 'inference.yaml').write_text(
        'inference_node:\n  ros__parameters:\n    model_path: models/yolo.pt\n'
    )
    assert pp.read_model_name(tmp_path) == 'yolo.pt'
    (car_profile_dir / 'params').mkdir(parents=True)
    (car_profile_dir / 'params' / 'inference.yaml').write_text(
        'inference_node:\n  ros__parameters:\n    model_path: /home/racecar/cones.engine\n'
    )
    assert pp.read_model_name(tmp_path) == 'cones.engine'


def test_an_unreadable_inference_config_reads_unknown(pp, tmp_path):
    assert pp.read_model_name(tmp_path / 'missing') == 'unknown'


def test_a_slider_at_zero_stops_a_car_that_has_lost_its_target(pp, node, params):
    node._speed_cmd, node._steer_cmd = 0.4, 0.2  # the held command
    params['speed'] = 0.0
    node._actuate()
    assert node._pub.msgs[-1].drive.speed == 0.0
    assert node._pub.msgs[-1].drive.steering_angle == pytest.approx(-0.2)
