"""Sensor panels for the System Health page: scripts/overview.py and its 8080 routes."""

from http.server import ThreadingHTTPServer
import json
import math
from pathlib import Path
import re
import sys
import threading
from types import ModuleType, SimpleNamespace
import urllib.error
import urllib.request

from conftest import load_script
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
JPEG = b'\xff\xd8\xff\xe0' + b'frame' + b'\xff\xd9'


def _ros_stubs():
    """Return stand-ins for the ROS modules overview.py and dashboard.py import, off the car."""
    try:
        import rclpy.qos  # noqa: F401
        import sensor_msgs.msg  # noqa: F401
        import std_msgs.msg  # noqa: F401

        return {}
    except ImportError:
        pass

    def module(name, **attrs):
        mod = ModuleType(name)
        mod.__dict__.update(attrs)
        return mod

    msg_type = lambda name: type(name, (), {})  # noqa: E731
    enum = SimpleNamespace(KEEP_LAST=0, BEST_EFFORT=0, VOLATILE=0)
    return {
        'rclpy': module('rclpy'),
        'rclpy.node': module('rclpy.node', Node=object),
        'rclpy.qos': module(
            'rclpy.qos',
            qos_profile_sensor_data=object(),
            QoSProfile=lambda **kw: kw,
            QoSDurabilityPolicy=enum,
            QoSHistoryPolicy=enum,
            QoSReliabilityPolicy=enum,
        ),
        'sensor_msgs': module('sensor_msgs'),
        'sensor_msgs.msg': module(
            'sensor_msgs.msg',
            **{n: msg_type(n) for n in ('Image', 'LaserScan', 'Imu', 'BatteryState')},
        ),
        'std_msgs': module('std_msgs'),
        'std_msgs.msg': module('std_msgs.msg', Float32=msg_type('Float32')),
    }


@pytest.fixture(scope='module')
def ros():
    with pytest.MonkeyPatch.context() as mp:
        for name, mod in _ros_stubs().items():
            mp.setitem(sys.modules, name, mod)
        yield


@pytest.fixture(scope='module')
def ov(ros):
    return load_script('overview')


@pytest.fixture(autouse=True)
def no_sampling_thread(ov, monkeypatch):
    # Tests sample with sample_once(); a thread per Overview would outlive the stubs.
    monkeypatch.setattr(ov.Overview, '_sample', lambda self: None)


@pytest.fixture(scope='module')
def dash(ov):
    path = list(sys.path)
    try:
        yield load_script('dashboard')
    finally:
        sys.path[:] = path


class FakeNode:
    def __init__(self):
        self.subs = []

    def create_subscription(self, msg_type, topic, cb, qos, raw=False):
        assert raw, 'overview feeds must not deserialize every message'
        handle = SimpleNamespace(topic=topic, cb=cb)
        self.subs.append(handle)
        return handle

    def destroy_subscription(self, handle):
        self.subs.remove(handle)


def _cb(node, topic):
    return next(s for s in node.subs if s.topic == topic).cb


# -- ported from upstream --


def test_scan_points_in_base_link(ov):
    # Lidar 0.2 m ahead, facing aft: its 0 rad beam points backward.
    pts = ov.scan_points([1.0, float('inf'), 0.0], 0.0, 0.1, (0.2, 0.0, math.pi))
    assert pts == [[-0.8, 0.0]]


def test_scan_points_drop_far_returns(ov):
    assert ov.scan_points([ov.SCAN_MAX_M + 1.0], 0.0, 0.1, (0.0, 0.0, 0.0)) == []


def test_image_array_swaps_rgb_to_bgr(ov):
    msg = SimpleNamespace(
        height=1, width=2, step=6, encoding='rgb8', data=bytes([1, 2, 3, 4, 5, 6])
    )
    img = ov.image_array(msg)
    assert img.shape == (1, 2, 3) and img[0, 0].tolist() == [3, 2, 1]


def test_subscriptions_open_on_request_and_close_when_idle(ov, monkeypatch):
    node = FakeNode()
    feed = ov.Overview(node, deserialize=lambda data, cls: data, config_dir=None)
    assert node.subs == []
    feed.touch()
    cameras = {ov.FEEDS[n][0] for n in ov.CAMERAS}
    assert sorted(s.topic for s in node.subs) == sorted(
        t for t, _ in ov.FEEDS.values() if t not in cameras
    )
    feed.last_request -= ov.IDLE_SEC + 1
    with feed._lock:
        feed._close()
    assert node.subs == [] and not feed.active


def fake_message(data, cls):
    zero = SimpleNamespace(x=0.0, y=0.0, z=0.0)
    return SimpleNamespace(data=0.0, angular_velocity=zero, linear_acceleration=zero)


def test_state_reports_feed_ages(ov):
    node = FakeNode()
    feed = ov.Overview(node, deserialize=fake_message, config_dir=None)
    feed.touch()
    _cb(node, '/encoder/speed')(b'x')
    state = feed.state()
    assert 'encoder' in state['ages'] and state['scan'] == []
    assert state['graph_sec'] == ov.GRAPH_SEC


def test_cameras_open_only_when_frames_are_requested(ov):
    node = FakeNode()
    feed = ov.Overview(node, deserialize=lambda data, cls: data, config_dir=None)
    feed.touch()
    topics = {s.topic for s in node.subs}
    assert '/camera/color' not in topics and '/scan' in topics
    feed.touch_cameras()
    assert '/camera/color' in {s.topic for s in node.subs}
    feed.last_frame_request -= ov.CAMERA_IDLE_SEC + 1
    with feed._lock:
        feed._drop([n for n in ov.CAMERAS if n in feed._handles])
    topics = {s.topic for s in node.subs}
    assert '/camera/color' not in topics and '/scan' in topics


# -- the NeoRacer's feeds --


def test_feeds_are_the_neoracer_topics_with_no_depth_camera(ov):
    assert {t for t, _ in ov.FEEDS.values()} == {
        '/camera/color',
        '/scan',
        '/imu/fused',
        '/encoder/speed',
    }
    assert ov.CAMERAS == ('color',)


def _frame(encoding, data, height=480, width=640, step=0):
    return SimpleNamespace(encoding=encoding, data=data, height=height, width=width, step=step)


def test_a_jpeg_frame_is_served_as_the_camera_sent_it(ov):
    node = FakeNode()
    feed = ov.Overview(node, deserialize=lambda data, cls: _frame('jpeg', data), config_dir=None)
    assert feed.jpeg('color') is None
    feed.touch_cameras()
    _cb(node, '/camera/color')(JPEG)
    assert feed.jpeg('color') == JPEG


def test_a_stopped_camera_is_not_served_its_last_frame(ov):
    node = FakeNode()
    feed = ov.Overview(node, deserialize=lambda data, cls: _frame('jpeg', data), config_dir=None)
    feed.touch_cameras()
    _cb(node, '/camera/color')(JPEG)
    data, arrived = feed._latest['color']
    feed._latest['color'] = (data, arrived - ov.FRAME_STALE_SEC - 0.1)
    feed._jpeg.clear()
    assert feed.jpeg('color') is None


def test_a_jpeg_frame_without_the_jpeg_marker_is_not_served(ov):
    node = FakeNode()
    feed = ov.Overview(node, deserialize=lambda data, cls: _frame('jpeg', data), config_dir=None)
    feed.touch_cameras()
    _cb(node, '/camera/color')(b'not a jpeg')
    assert feed.jpeg('color') is None


def test_a_raw_frame_is_scaled_and_encoded(ov):
    cv2 = pytest.importorskip('cv2')
    raw = bytes(np.zeros((48, 64, 3), dtype=np.uint8))
    node = FakeNode()
    feed = ov.Overview(
        node,
        deserialize=lambda data, cls: _frame('bgr8', data, height=48, width=64, step=192),
        config_dir=None,
    )
    feed.touch_cameras()
    _cb(node, '/camera/color')(raw)
    img = cv2.imdecode(np.frombuffer(feed.jpeg('color'), np.uint8), cv2.IMREAD_COLOR)
    assert img.shape[1] == ov.PREVIEW_WIDTH


def test_the_lidar_mount_and_the_car_outline_come_from_frames_yaml(ov):
    x, y, yaw = ov.laser_mount_from_config(ROOT / 'config')
    assert (x, y, yaw) == pytest.approx((0.09962, 0.0, 0.0))
    outline = ov.footprint_from_config(ROOT / 'config')
    assert outline == pytest.approx(
        {'front': 0.403, 'rear': -0.080, 'left': 0.118, 'right': -0.124}, abs=1e-3
    )


def test_no_frames_yaml_draws_the_scan_unrotated_and_no_outline(ov, tmp_path):
    assert ov.laser_mount_from_config(tmp_path) == (0.0, 0.0, 0.0)
    assert ov.footprint_from_config(tmp_path) is None


def test_imu_rows_carry_gyro_in_deg_per_s_and_accel_in_m_per_s2(ov):
    imu = SimpleNamespace(
        angular_velocity=SimpleNamespace(x=0.0, y=0.0, z=math.pi / 2),
        linear_acceleration=SimpleNamespace(x=0.1, y=-0.2, z=9.81),
    )
    node = FakeNode()
    feed = ov.Overview(node, deserialize=lambda data, cls: imu, config_dir=None)
    feed.touch()
    _cb(node, '/imu/fused')(b'x')
    feed.sample_once()
    assert feed.state()['imu'][-1][1:] == [0.0, 0.0, 90.0, 0.1, -0.2, 9.81]


def test_a_non_finite_sample_is_left_out_of_the_graphs(ov):
    imu = SimpleNamespace(
        angular_velocity=SimpleNamespace(x=0.0, y=math.nan, z=0.0),
        linear_acceleration=SimpleNamespace(x=0.0, y=0.0, z=9.81),
        data=math.inf,
    )
    node = FakeNode()
    feed = ov.Overview(node, deserialize=lambda data, cls: imu, config_dir=None)
    feed.touch()
    _cb(node, '/imu/fused')(b'x')
    _cb(node, '/encoder/speed')(b'x')
    feed.sample_once()
    state = feed.state()
    assert state['imu'] == [] and state['speed'] == []
    json.dumps(state, allow_nan=False)


def test_sampling_closes_the_camera_first_and_everything_when_idle(ov):
    node = FakeNode()
    feed = ov.Overview(node, deserialize=fake_message, config_dir=None)
    feed.touch_cameras()
    feed.last_frame_request -= ov.CAMERA_IDLE_SEC + 1
    feed.sample_once()
    topics = {s.topic for s in node.subs}
    assert '/camera/color' not in topics and '/imu/fused' in topics
    feed.last_request -= ov.IDLE_SEC + 1
    feed.sample_once()
    assert node.subs == [] and not feed.active


def test_history_keeps_the_mean_of_each_bucket_within_the_window(ov):
    h = ov.History(window_sec=60.0)
    for t, v in ((0.0, 11.0), (5.0, 12.0), (12.0, 12.4), (30.0, 12.2)):
        h.add(t, v)
    assert h.buckets(30.0, bucket_sec=10) == [[5.0, 11.5], [15.0, 12.4], [35.0, 12.2]]
    h.add(65.0, 12.0)
    assert h.buckets(65.0, bucket_sec=10) == [
        [5.0, 12.0],
        [15.0, 12.4],
        [35.0, 12.2],
        [65.0, 12.0],
    ]
    assert h.latest(70.0) == ((12.0,), 5.0)


def test_history_drops_a_sample_json_cannot_carry(ov):
    h = ov.History()
    h.add(1.0, math.nan)
    assert h.latest(2.0) is None


def test_system_history_records_temperature_and_memory_in_use(ov):
    h = ov.History()
    ov.sample_system(h, 100.0, 51.26, {'total': 7620, 'available': 1905, 'used': 5715})
    assert h.latest(100.0) == ((51.3, 75.0), 0.0)
    ov.sample_system(h, 103.0, None, {'total': 7620, 'available': 1905, 'used': 5715})
    ov.sample_system(h, 103.0, 50.0, None)
    assert h.latest(103.0)[1] == 3.0


# -- the 8080 routes, served by dashboard.py --


@pytest.fixture
def server(dash, ov, monkeypatch):
    monkeypatch.setattr(dash, '_battery_history', ov.History())
    monkeypatch.setattr(dash, '_system_history', ov.History())
    srv = ThreadingHTTPServer(('127.0.0.1', 0), dash.DashboardHandler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f'http://127.0.0.1:{srv.server_address[1]}'
    srv.shutdown()
    srv.server_close()


def _get(url):
    try:
        with urllib.request.urlopen(url, timeout=5) as r:
            return r.status, r.headers['Content-Type'], r.read()
    except urllib.error.HTTPError as err:
        return err.code, err.headers['Content-Type'], b''


def test_overview_is_unavailable_until_the_feeds_start(dash, server, monkeypatch):
    monkeypatch.setattr(dash, '_overview', None)
    assert _get(server + '/api/overview')[0] == 503
    assert _get(server + '/api/overview/color.jpg')[0] == 404


def test_overview_route_opens_the_feeds_and_returns_their_state(dash, ov, server, monkeypatch):
    node = FakeNode()
    feed = ov.Overview(node, deserialize=fake_message, config_dir=ROOT / 'config')
    monkeypatch.setattr(dash, '_overview', feed)
    status, kind, body = _get(server + '/api/overview')
    assert status == 200 and kind == 'application/json'
    state = json.loads(body)
    assert state['scan'] == [] and state['footprint']['front'] == pytest.approx(0.403, abs=1e-3)
    assert '/scan' in {s.topic for s in node.subs}


def test_frame_route_serves_the_camera_jpeg(dash, ov, server, monkeypatch):
    node = FakeNode()
    feed = ov.Overview(node, deserialize=lambda data, cls: _frame('jpeg', data), config_dir=None)
    monkeypatch.setattr(dash, '_overview', feed)
    assert _get(server + '/api/overview/color.jpg')[0] == 404
    _cb(node, '/camera/color')(JPEG)
    assert _get(server + '/api/overview/color.jpg?t=1') == (200, 'image/jpeg', JPEG)


def test_history_route_returns_battery_temperature_and_memory(dash, server):
    dash._battery_history.add(1000.0, 12.1)
    dash._system_history.add(1000.0, 51.0, 40.0)
    status, kind, body = _get(server + '/api/history')
    data = json.loads(body)
    assert status == 200 and kind == 'application/json'
    assert data['window_sec'] == 2 * 3600
    assert set(data) == {'now', 'window_sec', 'battery', 'system'}


def test_the_other_pages_still_serve(server):
    assert _get(server + '/')[:2] == (200, 'text/html; charset=utf-8')
    assert _get(server + '/health')[:2] == (200, 'text/html; charset=utf-8')
    assert _get(server + '/api/overview/depth.jpg')[0] == 404
    assert _get(server + '/nowhere')[0] == 404


def test_the_health_page_names_only_elements_it_has():
    page = (ROOT / 'scripts' / 'health.html').read_text()
    ids = set(re.findall(r'\bid="([^"]+)"', page))
    used = set(re.findall(r"\$\('([^']+)'\)", page))
    for prefix in ('camera', 'lidar', 'imu', 'encoder'):
        used |= {f'{prefix}-view', f'{prefix}-empty'}
    for prefix in ('batt', 'junction', 'ram'):
        used |= {f'{prefix}-chart', f'{prefix}-empty'}
    assert used <= ids, sorted(used - ids)
    assert '/api/overview' in page and '/api/history' in page
