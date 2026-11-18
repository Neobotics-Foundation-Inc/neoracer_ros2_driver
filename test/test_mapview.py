"""Web map viewer: scripts/mapview.py, its data helpers, HTTP endpoints, and idle close."""

import gzip
import http.client
import json
import math
from pathlib import Path
import re
import sys
import threading
import time
from types import SimpleNamespace

from conftest import load_script
import cv2
from neoracer_ros2_driver import autonomy as au
import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope='module')
def mv():
    load_script('sysinfo')
    load_script('map_control')
    load_script('webui')
    return load_script('mapview')


@pytest.fixture
def page(mv):
    return (Path(mv.__file__).parent / 'mapview.html').read_text()


def write_map(folder: Path, name: str, pixels: np.ndarray, origin=(-1.0, -2.0, 0.0)) -> Path:
    folder.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(folder / f'{name}.pgm'), pixels)
    meta = {
        'image': f'{name}.pgm',
        'resolution': 0.05,
        'origin': list(origin),
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196,
    }
    (folder / f'{name}.yaml').write_text(yaml.safe_dump(meta))
    return folder / name


def decode_png(png: bytes) -> np.ndarray:
    """Return the PNG's pixels, channels in OpenCV's order (blue first)."""
    return cv2.imdecode(np.frombuffer(png, np.uint8), cv2.IMREAD_COLOR)


def room_pixels() -> np.ndarray:
    # 4 rows x 5 columns: wall along the top row, unknown at the bottom left.
    px = np.full((4, 5), 254, dtype=np.uint8)
    px[0, :] = 0
    px[3, 0] = 205
    return px


@pytest.fixture
def root(mv, tmp_path, monkeypatch):
    maps = tmp_path / 'maps'
    maps.mkdir()
    monkeypatch.setenv('RACECAR_MAP_ROOT', str(maps))
    # The map list asks systemd whether autonomy runs; off the car there is no
    # systemctl, so the unit reads as stopped.
    monkeypatch.setattr(mv.mc, 'unit_state', lambda unit=mv.mc.UNIT: 'inactive')
    return maps


def saved_map(root: Path, name: str) -> Path:
    stem = write_map(root / name, name, room_pixels())
    for suffix in ('.posegraph', '.data'):
        (root / name / f'{name}{suffix}').write_text('x')
    return stem


class TestMapsFromDisk:
    def test_occupancy_from_pixels(self, mv):
        grid = mv.occupancy_from_pixels(room_pixels().astype(float), {})
        assert grid[0].tolist() == [100] * 5
        assert grid[3, 0] == -1 and grid[3, 1] == 0

    def test_read_saved_flips_to_map_rows(self, mv, tmp_path):
        info, grid = mv.read_saved(write_map(tmp_path, 'room', room_pixels()))
        assert info == {'width': 5, 'height': 4, 'resolution': 0.05, 'origin': [-1.0, -2.0]}
        assert grid[-1].tolist() == [100] * 5  # the top image row is the last map row
        assert grid[0, 0] == -1
        assert mv.read_saved(tmp_path / 'missing') is None

    def test_saved_stem_prefers_saved_then_autosave(self, mv, root):
        assert mv.saved_stem('none') is None
        saved_map(root, 'a')
        assert mv.saved_stem('a') == root / 'a' / 'a'
        auto = root / 'b' / au.AUTOSAVE_DIR
        write_map(auto, 'b', room_pixels())
        for suffix in ('.posegraph', '.data'):
            (auto / f'b{suffix}').write_text('x')
        assert mv.saved_stem('b') == auto / 'b'

    def test_thumbnail_png(self, mv, tmp_path):
        png = mv.thumbnail_png(write_map(tmp_path, 'room', room_pixels()), max_px=3)
        img = decode_png(png)
        assert png[:4] == b'\x89PNG' and max(img.shape[:2]) <= 3

    def test_thumbnail_colors_match_page(self, mv, tmp_path):
        png = mv.thumbnail_png(write_map(tmp_path, 'room', room_pixels()), max_px=1000)
        colors = {tuple(int(v) for v in px[::-1]) for px in decode_png(png).reshape(-1, 3)}
        assert colors <= set(mv.THUMB_RGB.values())
        page = (Path(mv.__file__).parent / 'mapview.html').read_text()
        for name, (r, g, b) in mv.THUMB_RGB.items():
            assert f'--map-{name}: #{r:02x}{g:02x}{b:02x};' in page


class TestLiveData:
    def test_laser_mount(self, mv):
        assert mv.laser_mount(None) is None
        frames = {'laser_x_mm': 238.3, 'laser_y_mm': 0.0, 'laser_yaw_deg': 180.0}
        x, y, yaw = mv.laser_mount(frames)
        assert (x, y) == pytest.approx((0.2383, 0.0)) and yaw == pytest.approx(math.pi)

    def test_scan_to_map_places_returns(self, mv):
        # Car at (1, 0) facing +y; laser 0.2 m ahead, facing aft. Beam 0 (the laser's +x)
        # points down -y from the laser at (1, 0.2): 1 m out lands at (1, -0.8).
        pts = mv.scan_to_map(
            [1.0, 0.1], 0.0, math.pi / 2, (1.0, 0.0, math.pi / 2), (0.2, 0.0, math.pi)
        )
        assert pts == [[pytest.approx(1.0), pytest.approx(-0.8)]]

    def test_trail_steps_ages_and_restarts(self, mv):
        t = mv.Trail(step_m=0.1, limit=3)
        for x in (0.0, 0.05, 0.2, 0.4, 0.6):
            t.add(x, 0.0)
        assert [p[0] for p in t.points] == [0.2, 0.4, 0.6] and t.first == 1
        assert t.since(3, t.id) == {'id': t.id, 'from': 3, 'points': [[0.6, 0.0]]}
        assert t.since(0, t.id)['from'] == 1  # aged out: from the oldest kept
        assert t.since(3, t.id + 5)['points'] == t.points  # another id: the whole trail
        t.restart()
        assert t.since(0, -1) == {'id': t.id, 'from': 0, 'points': []}


def test_cpu_meter_reads_busy_share(mv, monkeypatch):
    samples = iter([(100, 400), (160, 500)])  # (busy, total) jiffies
    monkeypatch.setattr(mv.sysinfo, 'read_cpu_times', lambda: next(samples))
    assert mv.CpuMeter().read() == pytest.approx(60.0)


class TestJetsonReadings:
    """The host line from the Jetson readers in scripts/sysinfo.py."""

    @staticmethod
    def counters(hwmon: Path, **counts: int) -> None:
        # The SoC's over-current event counters, as the Orin's hwmon shows them.
        hwmon.mkdir(exist_ok=True)
        (hwmon / 'name').write_text('soctherm_oc\n')
        for name, count in counts.items():
            (hwmon / f'{name}_event_cnt').write_text(f'{count}\n')

    @pytest.fixture
    def sys_dir(self, mv, tmp_path, monkeypatch):
        real = mv.sysinfo.read_throttled
        monkeypatch.setattr(mv.sysinfo, 'read_throttled', lambda: real(tmp_path))
        monkeypatch.setattr(mv.sysinfo, 'read_soc_temp', lambda: 62.5)
        return tmp_path

    def test_reports_over_current_during_the_run_only(self, mv, sys_dir):
        hwmon = sys_dir / 'hwmon3'
        self.counters(hwmon, oc1=2, oc2=0)
        host = mv.HostReadings()
        first = host.read()
        # oc1 fired before the run: the flags show it, the run does not.
        assert first['throttled'] == '0x1' and first['throttled_this_run'] == []
        assert first['temp_c'] == 62.5 and first['hot_c'] == mv.HOT_C
        self.counters(hwmon, oc1=3)
        during = host.read()['throttled_this_run']
        assert len(during) == 1 and 'oc1' in during[0]
        host.restart()
        assert host.read()['throttled_this_run'] == []
        self.counters(hwmon, oc2=1)
        during = host.read()['throttled_this_run']
        assert len(during) == 1 and 'oc2' in during[0]

    def test_no_counters_reads_none(self, mv, sys_dir):
        reading = mv.HostReadings().read()
        assert reading['throttled'] is None and reading['throttled_this_run'] == []

    def test_hot_is_the_health_page_limit(self, mv):
        assert mv.HOT_C == load_script('diagnose').SOC_FAIL_C
        health = (ROOT / 'scripts' / 'health.html').read_text()
        found = re.search(r'TEMP_HOT_C = (\d+)', health)
        assert found is not None and float(found.group(1)) == mv.HOT_C

    def test_the_page_warns_from_the_jetson_readings(self, page):
        assert 'sys.temp_c >= sys.hot_c' in page and 'sys.throttled_this_run' in page
        assert 'Pi ' not in page and '>= 78' not in page

    def test_a_new_session_restarts_the_trail_and_the_run(self, mv):
        restarts = []
        viewer = SimpleNamespace(
            _lock=threading.Lock(),
            status={},
            _session='',
            trail=mv.Trail(),
            _host=SimpleNamespace(restart=lambda: restarts.append(1)),
        )

        def status(session):
            values = [SimpleNamespace(key='session', value=session)]
            return SimpleNamespace(values=values, level=0, message='')

        mv.Viewer._on_status(viewer, status('a'))
        trail_id = viewer.trail.id
        mv.Viewer._on_status(viewer, status('a'))
        assert restarts == [1] and viewer.trail.id == trail_id
        mv.Viewer._on_status(viewer, status('b'))
        assert restarts == [1, 1] and viewer.trail.id == trail_id + 1


class TestActions:
    @pytest.mark.parametrize(
        'req, argv',
        [
            ({'action': 'save'}, ['save']),
            ({'action': 'stop'}, ['stop']),
            ({'action': 'start', 'name': 'hall'}, ['start', 'hall']),
            (
                {'action': 'start', 'name': 'hall2', 'source': 'hall', 'pose': [1, -2, 0.5]},
                ['start', 'hall2', '--pose=1.000,-2.000,0.5000', '--from', 'hall'],
            ),
            ({'action': 'load', 'name': 'hall'}, ['load', 'hall', '--localizer', 'amcl']),
            (
                {'action': 'load', 'name': 'hall', 'search': True},
                ['load', 'hall', '--global', '--localizer', 'amcl'],
            ),
            ({'action': 'recover', 'name': 'hall'}, ['recover', 'hall']),
            ({'action': 'start', 'name': 'hall', 'replace': True}, ['start', 'hall', '--force']),
            (
                {
                    'action': 'start',
                    'name': 'hall2',
                    'source': 'hall',
                    'pose': [0, 0, 0],
                    'replace': True,
                },
                ['start', 'hall2', '--pose=0.000,0.000,0.0000', '--from', 'hall', '--force'],
            ),
        ],
    )
    def test_action_argv(self, mv, req, argv):
        assert mv.action_argv(req) == argv

    @pytest.mark.parametrize(
        'req',
        [
            {'action': 'rm'},
            {'action': 'start', 'name': '../x'},
            {'action': 'start', 'name': 'a', 'pose': [0, 0, 0]},
            {'action': 'load', 'name': 'a', 'pose': [0, 0, 0], 'search': True},
            {'action': 'load', 'name': 'a', 'localizer': 'other'},
            {'action': 'save', 'pose': [0, 0, 0]},
            {'action': 'load', 'name': 'a', 'replace': True},
            {'action': 'start', 'name': 'a', 'source': 'a', 'pose': [0, 0, 0], 'replace': True},
        ],
    )
    def test_action_argv_refuses(self, mv, req):
        with pytest.raises(ValueError):
            mv.action_argv(req)

    def test_job_runs_one_at_a_time(self, mv):
        code = 'import sys, time; print("hello", sys.argv[1:]); time.sleep(0.3); sys.exit(3)'
        job = mv.Job([sys.executable, '-c', code])
        assert job.start(['x'])
        assert not job.start(['y'])
        deadline = time.monotonic() + 5
        while job.state()['running'] and time.monotonic() < deadline:
            time.sleep(0.05)
        state = job.state()
        assert state['code'] == 3 and state['action'] == 'x'
        assert state['lines'] == ["hello ['x']"]


class FakeViewer:
    def __init__(self):
        self.touched = 0
        self.poses = []
        self.cleared = 0

    def clear_trail(self):
        self.cleared += 1

    def touch(self):
        self.touched += 1

    def snapshot(self, trail_from, trail_id):
        return {'status': {'mode': 'localization'}, 'trail': {'from': trail_from, 'id': trail_id}}

    def map_payload(self):
        info = {'width': 2, 'height': 1, 'resolution': 0.05, 'origin': [0, 0], 'version': 4}
        return info, gzip.compress(bytes([0, 100]))

    def set_pose(self, x, y, yaw):
        self.poses.append((x, y, yaw))

    def find_me(self):
        return True, 'searching the whole map; drive slowly'


class BusyJob:
    def start(self, argv):
        self.argv = argv
        return argv[0] != 'save'

    def state(self):
        return {'running': False}


@pytest.fixture
def server(mv, root):
    mv.Handler.viewer = FakeViewer()
    mv.Handler.job = BusyJob()
    srv = mv.ThreadingHTTPServer(('127.0.0.1', 0), mv.Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield srv
    srv.shutdown()
    srv.server_close()


def request(srv, method, path, body=None):
    conn = http.client.HTTPConnection('127.0.0.1', srv.server_address[1], timeout=5)
    conn.request(method, path, body=None if body is None else json.dumps(body))
    res = conn.getresponse()
    data = res.read()
    conn.close()
    return res, data


class TestHttp:
    def test_page(self, server):
        res, data = request(server, 'GET', '/')
        assert res.status == 200 and b'<canvas' in data

    def test_state_touches_viewer(self, mv, server):
        res, data = request(server, 'GET', '/api/state?trail_from=7&trail_id=2')
        state = json.loads(data)
        assert state['trail'] == {'from': 7, 'id': 2} and state['job'] == {'running': False}
        assert mv.Handler.viewer.touched == 1

    def test_map_is_gzip_with_info(self, server):
        res, data = request(server, 'GET', '/api/map')
        assert res.getheader('Content-Encoding') == 'gzip'
        assert json.loads(res.getheader('X-Map-Info'))['version'] == 4
        assert gzip.decompress(data) == bytes([0, 100])

    def test_maps_thumb_and_saved(self, server, root):
        saved_map(root, 'hall')
        res, data = request(server, 'GET', '/api/maps')
        assert [m['name'] for m in json.loads(data)['maps']] == ['hall']
        res, data = request(server, 'GET', '/api/thumb?name=hall')
        assert res.status == 200 and data[:4] == b'\x89PNG'
        res, data = request(server, 'GET', '/api/saved?name=hall')
        info = json.loads(res.getheader('X-Map-Info'))
        assert info['name'] == 'hall' and len(gzip.decompress(data)) == 20
        assert request(server, 'GET', '/api/thumb?name=../etc')[0].status == 404
        assert request(server, 'GET', '/api/saved?name=none')[0].status == 404

    def test_action_validates_and_reports_busy(self, server):
        res, data = request(server, 'POST', '/api/action', {'action': 'load', 'name': 'hall'})
        assert res.status == 200 and json.loads(data)['argv'][:2] == ['load', 'hall']
        res, _ = request(server, 'POST', '/api/action', {'action': 'save'})
        assert res.status == 409
        res, data = request(server, 'POST', '/api/action', {'action': 'rm'})
        assert res.status == 400 and 'unknown action' in json.loads(data)['message']

    def test_pose_and_find(self, mv, server):
        res, _ = request(server, 'POST', '/api/pose', {'x': 1, 'y': 2, 'yaw': 0.5})
        assert res.status == 200 and mv.Handler.viewer.poses == [(1.0, 2.0, 0.5)]
        res, _ = request(server, 'POST', '/api/pose', {'x': 1})
        assert res.status == 400
        res, data = request(server, 'POST', '/api/find', {})
        assert json.loads(data)['ok'] is True

    def test_driving_apps(self, mv, server, monkeypatch):
        monkeypatch.setattr(mv.mc, 'running_driving_apps', lambda: ['Web Teleop'])
        res, data = request(server, 'GET', '/api/driving')
        assert json.loads(data) == {'apps': ['Web Teleop']}

    def test_clear_trail(self, mv, server):
        res, _ = request(server, 'POST', '/api/trail/clear', {})
        assert res.status == 200 and mv.Handler.viewer.cleared == 1

    def test_unknown_path(self, server):
        assert request(server, 'GET', '/nope')[0].status == 404
        assert request(server, 'POST', '/nope', {})[0].status == 404


class TestNeoRacerPage:
    """The page in the NeoRacer look, and the NeoRacer's outline on the map."""

    def test_has_no_upstream_brand(self, page):
        lower = page.lower()
        for banned in ('rcn-', 'barlow', 'racecar neo', 'brand/mark.png', 'app-icon'):
            assert banned not in lower, banned

    def test_shows_the_logo_for_each_theme(self, page):
        assert 'class="for-dark" src="/ui/brand/neoracer-logo-reversed.svg"' in page
        assert 'class="for-light" src="/ui/brand/neoracer-logo.svg"' in page
        assert 'data-theme-pick="dark"' in page and 'data-theme-pick="light"' in page

    def test_uses_only_icons_the_shared_set_has(self, mv, page):
        icons = (mv.webui.WEB_DIR / 'icons.svg').read_text()
        ids = set(re.findall(r'<symbol id="([\w-]+)"', icons))
        # Icons in the markup, the status pills' icons, and each button's glyph.
        used = set(re.findall(r'icons\.svg#([\w-]+)"', page))
        used |= set(re.findall(r"(?:ok|warn|err|off): '([\w-]+)'", page))
        used |= set(re.findall(r", '([\w-]+)'\)\);", page))
        assert {'plus', 'check', 'save', 'target', 'layers'} <= used
        assert used <= ids, used - ids

    def test_draws_the_neoracer_outline(self, page):
        frames = yaml.safe_load((ROOT / 'config' / 'frames.yaml').read_text())
        number = r'([\d.]+)'
        found = re.search(
            rf'const FOOTPRINT = \{{ front: {number}, rear: {number}, half: {number} \}};', page
        )
        assert found is not None
        front, rear, half = (float(v) for v in found.groups())
        assert front == pytest.approx(frames['footprint_front_x_mm'] / 1000, abs=0.001)
        assert rear == pytest.approx(-frames['footprint_rear_x_mm'] / 1000, abs=0.001)
        width = frames['footprint_left_y_mm'] - frames['footprint_right_y_mm']
        assert half == pytest.approx(width / 2000, abs=0.001)

    def test_serves_the_shared_files(self, mv, server):
        res, _ = request(server, 'GET', f'/ui/neoracer.css?v={mv.webui.ASSET_VERSION}')
        assert res.status == 200 and res.getheader('Content-Type').startswith('text/css')
        res, _ = request(server, 'GET', '/ui/brand/neoracer-logo.svg')
        assert res.status == 200 and res.getheader('Content-Type') == 'image/svg+xml'


def test_viewer_subscribes_only_while_polled(mv, monkeypatch):
    rclpy = pytest.importorskip('rclpy')
    if getattr(rclpy, '__file__', None) is None:
        pytest.skip('needs the real rclpy; a stand-in ROS module has no file')
    monkeypatch.setenv('ROS_DOMAIN_ID', '91')  # away from a running car
    monkeypatch.setattr(mv, 'IDLE_SEC', 0.5)
    monkeypatch.setattr(mv.sysinfo, 'read_throttled', lambda: (0, []))
    rclpy.init()
    try:
        viewer = mv.Viewer()
        assert not viewer.active and viewer.node.get_subscriptions_info_by_topic('/map') == []
        viewer.touch()
        assert viewer.active
        topics = {name for name, _ in viewer.node.get_topic_names_and_types()}
        assert {'/autonomy/status', '/autonomy/pose', '/map', '/scan'} <= topics
        deadline = time.monotonic() + 5
        while viewer.active and time.monotonic() < deadline:
            time.sleep(0.1)
        assert not viewer.active
        viewer.shutdown()
    finally:
        rclpy.try_shutdown()
