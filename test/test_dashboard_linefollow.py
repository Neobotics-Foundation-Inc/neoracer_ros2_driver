"""
Pipeline and controller tests for the line follow dashboard (linefollow.py).

Runs without a ROS graph or a camera: Node.__init__ and the create_*
factories are stubbed, so the real LineFollowNode.__init__ still sets every
field, and frames are synthetic BGR images carrying a painted stripe, sent
as JPEG the way the NeoRacer's camera node sends them.
"""

from pathlib import Path

from conftest import load_script
import cv2
import numpy as np
import pytest
from sensor_msgs.msg import Image

BASE = Path(__file__).resolve().parents[1] / 'scripts' / 'dashboards' / 'linefollow'
# Loaded by the module fixture below, after the session fixtures have moved
# the data root and the car profile to temporary folders.
lf = None

SHIPPED = (BASE / 'linefollow.yaml').read_text()

W, H = 640, 480
BLUE = (255, 80, 0)  # BGR; hue about 105 in OpenCV's 0..179
BLUE_HSV = {'h_low': 95, 'h_high': 125, 's_low': 100, 's_high': 255, 'v_low': 100, 'v_high': 255}
RED = (0, 0, 255)  # hue 0, on the wrap


class Recorder:
    """Stands in for the /drive publisher."""

    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


@pytest.fixture(scope='module', autouse=True)
def _linefollow():
    global lf
    lf = load_script('linefollow')
    return lf


@pytest.fixture(autouse=True)
def params(monkeypatch, tmp_path):
    """Fresh parameters per test; saves go to a temporary car profile file."""
    monkeypatch.setattr(lf, 'YAML_PATH', tmp_path / 'profile' / 'linefollow.yaml')
    monkeypatch.setattr(lf, 'LOG_DIR', tmp_path / 'logs')
    lf._params.clear()
    lf._marks.clear()
    lf._rows.clear()
    lf._hist.clear()
    lf.load_params()
    lf._params.update(BLUE_HSV)
    yield lf._params


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setattr(lf.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(
        lf.LineFollowNode, 'create_publisher', lambda self, *a, **k: Recorder(), raising=False
    )
    monkeypatch.setattr(
        lf.LineFollowNode, 'create_subscription', lambda self, *a, **k: None, raising=False
    )
    monkeypatch.setattr(
        lf.LineFollowNode, 'create_timer', lambda self, *a, **k: None, raising=False
    )
    return lf.LineFollowNode()


def frame(stripes=()):
    """Grey floor with vertical stripes. stripes: (x_frac, width_px, bgr, y0_frac, y1_frac)."""
    img = np.full((H, W, 3), 110, np.uint8)
    for x_frac, width, color, y0, y1 in stripes:
        x = int(x_frac * W)
        cv2.rectangle(img, (x - width // 2, int(y0 * H)), (x + width // 2, int(y1 * H)), color, -1)
    return img


def jpeg_msg(img):
    msg = Image()
    msg.encoding = 'jpeg'
    msg.height, msg.width = img.shape[:2]
    msg.data = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes()
    return msg


def feed(node, img):
    """Push a frame through the subscription callback, ignoring the rate gate."""
    node._last_proc = 0.0
    node._image_cb(jpeg_msg(img))


# ---- parameters ----


def test_speed_is_capped():
    assert lf.set_max_speed(5.0) == lf.MAX_SPEED
    assert lf.set_max_speed(-1.0) == 0.0


def test_shipped_yaml_cannot_drive():
    assert yaml_value('speed') == 0.0


def yaml_value(key):
    import yaml

    return yaml.safe_load(SHIPPED)[key]


def test_save_keeps_comments_and_types(params):
    params.update(h_low=12, speed=0.35, crop_top=0.5, min_area=40)
    lf.save_params()
    text = lf.YAML_PATH.read_text()
    assert (BASE / 'linefollow.yaml').read_text() == SHIPPED  # defaults untouched
    assert 'Raise this if the car weaves.' in text  # comments survive
    assert 'h_low: 12\n' in text  # ints stay ints
    lf._params.clear()
    lf.load_params()
    assert lf._params['h_low'] == 12
    assert lf._params['speed'] == 0.35
    assert lf._params['crop_top'] == 0.5
    assert lf._params['min_area'] == 40
    assert lf._params['camera_topic'] == '/camera/color'  # non-numeric untouched


def test_update_clamps_hsv_and_speed(params):
    lf.update_params({'h_high': 500, 'v_low': -3, 'speed': 4.0})
    assert params['h_high'] == 179
    assert params['v_low'] == 0
    assert params['speed'] == lf.MAX_SPEED


def test_update_keeps_the_band_ordered(params):
    lf.update_params({'crop_top': 0.9, 'crop_bottom': 0.4})
    assert params['crop_top'] < params['crop_bottom']
    assert params['crop_bottom'] - params['crop_top'] >= lf.MIN_BAND - 1e-9
    assert params['crop_bottom'] <= 1.0


def test_update_marks_only_real_changes(params):
    assert lf.update_params({'speed': 0.0}) == []
    assert lf.update_params({'speed': 0.2}) == ['speed 0.2']
    assert lf._marks[-1][1] == 'speed 0.2'


# ---- threshold and contour ----


def test_threshold_wraps_hue_for_red(params):
    hsv = cv2.cvtColor(np.full((4, 4, 3), RED, np.uint8), cv2.COLOR_BGR2HSV)
    params.update(h_low=170, h_high=10, s_low=100, s_high=255, v_low=100, v_high=255)
    assert lf.threshold(hsv, params).all()
    params.update(h_low=20, h_high=160)
    assert not lf.threshold(hsv, params).any()


def test_find_line_takes_the_largest_contour_in_the_band():
    mask = np.zeros((240, 320), np.uint8)
    cv2.rectangle(mask, (40, 150), (60, 230), 255, -1)  # small, in band
    cv2.rectangle(mask, (200, 150), (260, 230), 255, -1)  # large, in band
    cv2.rectangle(mask, (150, 0), (170, 100), 255, -1)  # above the band
    contour, (cx, cy), area = lf.find_line(mask, 144, 240, 30)
    assert 225 <= cx <= 235
    assert 185 <= cy <= 195  # in full-mask rows, not band rows
    assert contour[:, 0, 1].min() >= 144


def test_find_line_ignores_specks():
    mask = np.zeros((240, 320), np.uint8)
    cv2.rectangle(mask, (100, 200), (103, 203), 255, -1)
    assert lf.find_line(mask, 144, 240, 30) is None


# ---- controller ----


def test_line_right_of_center_steers_right(node, params):
    params['angle_kp'] = 1.0
    feed(node, frame([(0.75, 40, BLUE, 0.6, 1.0)]))
    assert node._found
    assert node._error == pytest.approx(0.5, abs=0.05)  # student convention: + is right
    assert node._steer_cmd == pytest.approx(0.5, abs=0.05)


def test_line_left_of_center_steers_left(node):
    feed(node, frame([(0.25, 40, BLUE, 0.6, 1.0)]))
    assert node._error < -0.4
    assert node._steer_cmd < 0


def test_stripe_above_the_band_is_not_the_line(node, params):
    params.update(crop_top=0.6, crop_bottom=1.0)
    feed(node, frame([(0.5, 40, BLUE, 0.0, 0.5)]))
    assert not node._found
    params.update(crop_top=0.1, crop_bottom=0.5)
    feed(node, frame([(0.5, 40, BLUE, 0.0, 0.5)]))
    assert node._found


def test_lost_line_holds_the_last_command(node, params):
    params['speed'] = 0.3
    feed(node, frame([(0.75, 40, BLUE, 0.6, 1.0)]))
    steer, speed = node._steer_cmd, node._speed_cmd
    assert speed == 0.3
    feed(node, frame())
    assert not node._found
    assert (node._steer_cmd, node._speed_cmd) == (steer, speed)
    assert lf._state['lost_for'] is not None


def test_derivative_term_damps_a_jump(node, params):
    params.update(angle_kp=1.0, angle_kd=0.0)
    feed(node, frame([(0.5, 40, BLUE, 0.6, 1.0)]))
    feed(node, frame([(0.75, 40, BLUE, 0.6, 1.0)]))
    p_only = node._steer_cmd
    params['angle_kd'] = 0.5
    feed(node, frame([(0.5, 40, BLUE, 0.6, 1.0)]))
    feed(node, frame([(0.75, 40, BLUE, 0.6, 1.0)]))
    assert node._steer_cmd > p_only  # the jump adds to the P command


def test_speed_kp_slows_off_center(node, params):
    params.update(speed=1.0, speed_kp=0.0)
    feed(node, frame([(0.75, 40, BLUE, 0.6, 1.0)]))
    assert node._speed_cmd == 1.0
    params['speed_kp'] = 1.0
    feed(node, frame([(0.75, 40, BLUE, 0.6, 1.0)]))
    assert node._speed_cmd == pytest.approx(0.5, abs=0.05)
    feed(node, frame([(0.5, 40, BLUE, 0.6, 1.0)]))
    assert node._speed_cmd == pytest.approx(1.0, abs=0.05)


def test_steer_is_clamped(node, params):
    params['angle_kp'] = 4.0
    feed(node, frame([(0.9, 40, BLUE, 0.6, 1.0)]))
    assert node._steer_cmd == 1.0


def test_previews_and_state_are_produced(node):
    feed(node, frame([(0.5, 40, BLUE, 0.6, 1.0)]))
    assert lf._preview_color[:2] == b'\xff\xd8'
    assert lf._preview_mask[:2] == b'\xff\xd8'
    assert lf._state['res'] == [W, H]
    assert lf._state['proc_res'] == [320, 240]
    assert lf._state['cx'] == pytest.approx(160, abs=3)
    assert lf._hist[-1][2] == 0
    row = lf._rows[-1].strip().split(',')
    assert row[5] == '0' and row[6] == str(lf._state['cx'])


def test_lost_rows_are_flagged(node):
    feed(node, frame())
    assert lf._hist[-1][2] == 1
    assert lf._rows[-1].strip().split(',')[5:7] == ['1', '']


# ---- actuation ----


def test_actuate_negates_steering_and_caps_speed(node):
    lf._params['speed'] = 1.0
    node._steer_cmd, node._speed_cmd = 0.4, 99.0
    node._actuate()
    msg = node._pub.msgs[-1]
    assert msg.drive.steering_angle == pytest.approx(-0.4)  # wire positive = left
    assert msg.drive.speed == lf.MAX_SPEED


def test_thin_keeps_the_newest_row():
    rows = [[i, 0.0, 0] for i in range(1000)]
    sent = lf._thin(rows)
    assert len(sent) < len(rows)
    assert sent[-1] is rows[-1]
    assert lf._thin(rows[:50]) == rows[:50]  # short histories go through whole


# ---- NeoRacer ----


def test_the_profile_loads_over_the_defaults(params):
    lf.YAML_PATH.parent.mkdir(parents=True)
    lf.YAML_PATH.write_text('h_low: 40\nspeed: 0.25\n')
    lf.load_params()
    assert lf._params['h_low'] == 40
    assert lf._params['speed'] == 0.25
    assert lf._params['h_high'] == 179  # from the defaults


def test_logs_go_to_the_data_root(data_root_dir, monkeypatch):
    monkeypatch.undo()  # the module's own value, not the per-test folder
    assert lf.LOG_DIR == data_root_dir / 'dashboards' / 'linefollow'


def test_the_camera_is_the_jpeg_color_stream():
    import yaml

    shipped = yaml.safe_load(SHIPPED)
    assert shipped['camera_topic'] == '/camera/color'
    assert 'depth_topic' not in shipped


def test_stop_stops_a_car_that_has_lost_the_line(node):
    lf._params['speed'] = 0.4
    feed(node, frame([(0.7, 30, BLUE, 0.0, 1.0)]))
    feed(node, frame())  # the line is lost; the last command is held
    lf.update_params({'speed': 0})  # the page's Stop
    node._actuate()
    assert node._pub.msgs[-1].drive.speed == 0.0
