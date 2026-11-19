"""Unit tests for scripts/dashboard.py, the 8080 apps page and node monitor."""

from collections import deque
import http.client
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import re
import threading
import time
from types import SimpleNamespace

from conftest import load_script
import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'


@pytest.fixture(scope='module')
def dashboard():
    return load_script('dashboard')


class TestMonitoredAndRateTopics:
    EXPECTED_NODES = {
        'controller',
        'throttle',
        'mux',
        'gamepad',
        'odom',
        'joy',
        'lidar',
        'camera',
        'inference',
    }

    def test_monitored_covers_all_subsystems(self, dashboard):
        # Includes nodes the watchdog does not supervise (odom, joy, inference).
        assert set(dashboard.MONITORED) == self.EXPECTED_NODES

    @pytest.mark.parametrize('name', sorted(EXPECTED_NODES))
    def test_monitored_entry_has_label_and_topic(self, dashboard, name):
        cfg = dashboard.MONITORED[name]
        assert 'label' in cfg and cfg['label']
        assert 'topic' in cfg and cfg['topic'].startswith('/')

    def test_rate_topics_subset_of_known_publishers(self, dashboard):
        known = {cfg['topic'] for cfg in dashboard.MONITORED.values()}
        for t in dashboard.RATE_TOPICS:
            assert t in known, f'{t} in the rates table but no known node publishes it'

    def test_no_hardware_the_neoracer_lacks(self, dashboard):
        # No depth camera, RealSense IMU, LED strip or dot matrix frame topic.
        topics = set(dashboard.RATE_TOPICS) | {c['topic'] for c in dashboard.MONITORED.values()}
        for gone in ('/camera/depth', '/imu/realsense', '/led/pixels', '/dotmatrix/frame'):
            assert gone not in topics


class TestGetStatus:
    def test_returns_dict_with_required_keys(self, dashboard):
        snapshot = dashboard.get_status()
        for key in (
            'timestamp',
            'nodes',
            'node_list',
            'topic_list',
            'rates',
            'watchdog_log',
            'log_dir',
            'autonomy',
        ):
            assert key in snapshot

    def test_status_is_json_serializable(self, dashboard):
        # /api/status serves this through json.dumps.
        json.dumps(dashboard.get_status())


class _Clock:
    def __init__(self, t: float = 1000.0) -> None:
        self.t = t

    def monotonic(self) -> float:
        return self.t


@pytest.fixture
def sampler(dashboard, monkeypatch):
    """Build a _RateSampler with its bookkeeping but no rclpy node behind it."""
    clock = _Clock()
    monkeypatch.setattr(dashboard, 'time', clock)
    s = dashboard._RateSampler.__new__(dashboard._RateSampler)
    s._window = 3.0
    s._stamps = {'/scan': deque(), '/motor': deque()}
    s._lock = threading.Lock()
    s._battery = None
    s._autonomy = None
    return s, clock


class TestRateSampler:
    def test_rate_is_arrivals_over_the_window(self, sampler):
        s, clock = sampler
        for i in range(30):
            clock.t = 1000.0 + i * 0.1
            s._record('/scan')
        clock.t = 1002.95
        assert s.measure_hz('/scan') == pytest.approx(10.0)

    def test_arrivals_older_than_the_window_are_dropped(self, sampler):
        s, clock = sampler
        for _ in range(30):
            s._record('/scan')
        clock.t += 3.5
        assert s.measure_hz('/scan') is None

    def test_a_single_arrival_is_not_a_rate(self, sampler):
        s, _ = sampler
        s._record('/motor')
        assert s.measure_hz('/motor') is None

    def test_unknown_topic_is_none(self, sampler):
        s, _ = sampler
        assert s.measure_hz('/nope') is None

    def test_battery_reading_has_its_age(self, sampler):
        s, clock = sampler
        assert s.battery() is None
        s._battery = (12.1, clock.t)
        clock.t += 4.0
        assert s.battery() == (12.1, 4.0)

    def test_autonomy_status_values_and_age(self, sampler):
        s, clock = sampler
        assert s.autonomy() == (None, None)
        kv = [
            SimpleNamespace(key='mode', value='mapping'),
            SimpleNamespace(key='map', value='hall'),
        ]
        s._record_autonomy(SimpleNamespace(values=kv))
        clock.t += 2.0
        assert s.autonomy() == ({'mode': 'mapping', 'map': 'hall'}, 2.0)


class TestMeasureHz:
    def test_no_sampler_reads_as_no_data(self, dashboard, monkeypatch):
        monkeypatch.setattr(dashboard, '_sampler', None)
        assert dashboard._measure_hz('/scan') is None

    def test_delegates_to_the_sampler(self, dashboard, monkeypatch):
        monkeypatch.setattr(dashboard, '_sampler', SimpleNamespace(measure_hz=lambda t: 7.2))
        assert dashboard._measure_hz('/scan') == 7.2


class TestSystemHealth:
    def test_collect_system_health_keys(self, dashboard):
        # The Jetson's rows: no RTC cell or PMIC under-voltage alarm to read.
        health = dashboard._collect_system_health()
        assert set(health) == {'data', 'temperature', 'throttling'}
        for entry in health.values():
            assert {'label', 'status', 'detail', 'word'} <= set(entry)
            assert entry['status'] in ('healthy', 'stale', 'dead')

    @pytest.mark.parametrize(
        'celsius, status, word',
        [
            (None, 'stale', 'No reading'),
            (45.0, 'healthy', ''),
            (69.9, 'healthy', ''),
            (70.0, 'stale', 'Warm'),
            (84.9, 'stale', 'Warm'),
            (85.0, 'dead', 'HOT'),
        ],
    )
    def test_temperature_bands_match_racecar_status(self, dashboard, celsius, status, word):
        card = dashboard.temperature_health(celsius)
        assert (card['status'], card['word']) == (status, word)

    def test_throttling_since_boot_is_a_check(self, dashboard):
        names = ['over-current throttling oc1 has occurred (3 events)']
        card = dashboard.throttle_health(0x1, names)
        assert (card['status'], card['word']) == ('stale', 'Has occurred') and 'oc1' in card[
            'detail'
        ]
        assert dashboard.throttle_health(0, [])['status'] == 'healthy'
        assert dashboard.throttle_health(None, [])['word'] == 'No reading'

    def test_data_root(self, dashboard, monkeypatch, tmp_path):
        from neoracer_ros2_driver import data_root

        monkeypatch.setattr(data_root, 'current', lambda: (tmp_path, ''))
        monkeypatch.setattr(data_root, 'session_problem', lambda: '')
        assert dashboard.data_root_health()['status'] == 'healthy'
        low = SimpleNamespace(free=2e9)
        monkeypatch.setattr(dashboard.shutil, 'disk_usage', lambda path: low)
        assert dashboard.data_root_health()['word'] == 'Low space'
        monkeypatch.setattr(data_root, 'session_problem', lambda: 'cannot be written')
        assert dashboard.data_root_health()['word'] == 'Not writable'
        monkeypatch.setattr(data_root, 'current', lambda: (tmp_path / 'gone', ''))
        card = dashboard.data_root_health()
        assert (card['status'], card['word']) == ('dead', 'Missing')


class TestBattery:
    @pytest.mark.parametrize(
        'volts, age, state, status, word',
        [
            (12.4, 1.0, 'ok', 'healthy', ''),
            (11.0, 1.0, 'low', 'stale', 'Low'),
            (10.7, 1.0, 'charge', 'dead', 'CHARGE NOW'),
            (12.4, 20.0, 'stale', 'stale', 'No reading'),
        ],
    )
    def test_bands_of_a_3s_pack(self, dashboard, monkeypatch, volts, age, state, status, word):
        monkeypatch.setattr(dashboard, '_sampler', SimpleNamespace(battery=lambda: (volts, age)))
        assert dashboard._battery_host()['state'] == state
        card = dashboard._battery_card()
        assert (card['status'], card['word']) == (status, word)

    def test_no_reading_yet(self, dashboard, monkeypatch):
        monkeypatch.setattr(dashboard, '_sampler', SimpleNamespace(battery=lambda: None))
        assert dashboard._battery_host() is None and dashboard._battery_card() is None


class TestDashboardHTML:
    def test_html_template_present(self, dashboard):
        assert '<!DOCTYPE html>' in dashboard.DASHBOARD_HTML

    def test_html_references_api_endpoint(self, dashboard):
        html = dashboard.DASHBOARD_HTML
        assert "'/api/status'" in html and "'/api/status?refresh=1'" in html

    def test_title_says_neoracer(self, dashboard):
        html = dashboard.DASHBOARD_HTML
        assert '<title>NeoRacer</title>' in html
        assert 'RACECAR Neo' not in html and 'UAV Neo' not in html

    def test_system_health_section_present(self, dashboard):
        # The target div and the JS field name must agree.
        html = dashboard.DASHBOARD_HTML
        assert 'id="system-health"' in html
        assert 'data.system_health' in html


class TestConfig:
    def test_port_matches_the_service(self, dashboard):
        assert dashboard.PORT == 8080
        unit = (SCRIPTS / 'neoracer-dashboard.service').read_text()
        assert 'Description=Neoracer Web Dashboard (port 8080)' in unit
        assert 'neoracer_ros2_driver/scripts/dashboard.py' in unit


class TestWatchdogView:
    def test_no_watchdog_state_keeps_the_topic_status(self, dashboard):
        assert dashboard.watchdog_view('dead', None) == ('dead', '')

    def test_healthy_keeps_the_topic_status(self, dashboard):
        assert dashboard.watchdog_view('healthy', {'state': 'healthy'}) == ('healthy', '')

    def test_gave_up_is_dead_with_advice(self, dashboard):
        status, detail = dashboard.watchdog_view('dead', {'state': 'gave_up'})
        assert status == 'dead' and 'gave up' in detail

    def test_no_device_is_amber(self, dashboard):
        assert dashboard.watchdog_view('dead', {'state': 'no_device'})[0] == 'stale'

    def test_restarting_shows_the_count(self, dashboard):
        status, detail = dashboard.watchdog_view('dead', {'state': 'restarting', 'restarts': 3})
        assert status == 'stale' and '3' in detail

    def test_disabled_is_off(self, dashboard):
        # Off, as on System Health; the NeoRacer's unsupervised reads not detected.
        assert dashboard.watchdog_view('dead', {'state': 'disabled'})[0] == 'off'


class TestNodeStatus:
    CAMERA = {'supervised': True}
    INFERENCE = {'supervised': False}

    def test_a_publisher_is_healthy(self, dashboard):
        assert dashboard.node_status(True, self.CAMERA, True, None) == ('healthy', '')

    def test_a_subsystem_launched_off_reads_off(self, dashboard):
        assert dashboard.node_status(False, self.INFERENCE, False, None)[0] == 'off'
        assert dashboard.node_status(False, self.CAMERA, False, None)[0] == 'off'

    def test_missing_reads_dead_or_unsupervised(self, dashboard):
        assert dashboard.node_status(False, self.CAMERA, True, None)[0] == 'dead'
        assert dashboard.node_status(False, self.INFERENCE, True, None)[0] == 'unsupervised'

    def test_the_watchdog_state_wins(self, dashboard):
        watched = {'state': 'disabled'}
        assert dashboard.node_status(False, self.CAMERA, True, watched) == ('off', 'turned off')


class TestAutonomyView:
    def test_stopped_or_never_started_is_off(self, dashboard):
        for values in (None, {'mode': 'off', 'map': 'room'}):
            view = dashboard.autonomy_view(values, 1.0)
            assert view['status'] == 'off' and view['headline'] == 'Stopped'
            assert view['port'] == 8088

    def test_localized_is_healthy_with_fit_and_spread(self, dashboard):
        values = {
            'mode': 'localization', 'map': 'room', 'localizer': 'amcl', 'localized': 'true',
            'scan_fit': '0.81', 'pose_sd_m': '0.12', 'pose_sd_deg': '5.3',
        }  # fmt: skip
        view = dashboard.autonomy_view(values, 0.5)
        assert view['status'] == 'healthy'
        assert view['headline'] == 'Localization on room (amcl)'
        assert view['lines'] == ['Localized', 'Scan fit 81%', 'Spread 0.12 m, 5 deg']

    def test_not_localized_is_yellow(self, dashboard):
        view = dashboard.autonomy_view({'mode': 'localization', 'localized': 'false'}, 0.5)
        assert view['status'] == 'stale' and view['lines'][0] == 'Not localized'

    def test_mapping_shows_the_save(self, dashboard):
        values = {'mode': 'mapping', 'map': 'hall', 'localized': 'true'}
        view = dashboard.autonomy_view(values, 0.5)
        assert view['headline'] == 'Mapping on hall' and view['lines'][-1] == 'Saved not yet'

    def test_old_status_is_red(self, dashboard):
        view = dashboard.autonomy_view({'mode': 'mapping', 'map': 'hall'}, 12.0)
        assert view['status'] == 'dead' and '12 s' in view['lines'][0]

    def test_html_has_the_autonomy_section(self, dashboard):
        html = (Path(dashboard.__file__).parent / 'dashboard.html').read_text()
        assert 'id="autonomy"' in html and 'data.autonomy' in html


def _short(unit):
    return unit.removesuffix('.service')


class FakeRun:
    """subprocess.run stand-in: answers is-active from `states`, records the rest."""

    def __init__(self, states, fail=None, stderr=''):
        self.states = {_short(u): s for u, s in states.items()}
        self.fail, self.stderr, self.calls = fail, stderr, []

    def __call__(self, argv, **_):
        self.calls.append(argv)
        if argv[:2] == ['systemctl', 'is-active']:
            out = '\n'.join(self.states.get(_short(u), 'inactive') for u in argv[2:])
            return SimpleNamespace(stdout=out, returncode=0, stderr='')
        assert argv[:3] == ['sudo', '-n', 'systemctl'], argv
        verb, unit = argv[3], argv[4]
        if self.fail == verb:
            return SimpleNamespace(stdout='', returncode=1, stderr=self.stderr)
        self.states[_short(unit)] = 'inactive' if verb == 'stop' else 'active'
        return SimpleNamespace(stdout='', returncode=0, stderr='')

    def sent(self):
        """Return (verb, unit) for every unit command, in order."""
        return [(c[3], c[4]) for c in self.calls if c[:3] == ['sudo', '-n', 'systemctl']]


NO_SUDOERS = 'sudo: a password is required'
LABS = ['wallfollow', 'camlabel', 'pursuit', 'eps', 'smartfollow', 'linefollow', 'webteleop']


class TestHome:
    def test_apps_are_the_core_and_the_seven_lab_dashboards(self, dashboard):
        apps = {a['id']: a for a in dashboard.APPS}
        assert [a['id'] for a in dashboard.APPS if a['group'] == 'core'] == ['jupyter', 'mapview']
        assert [a['id'] for a in dashboard.APPS if a['group'] == 'apps'] == LABS
        ports = {a['id']: a['port'] for a in dashboard.APPS}
        assert ports == {
            'jupyter': 8888, 'mapview': 8088, 'wallfollow': 8081, 'camlabel': 8082,
            'pursuit': 8083, 'eps': 8084, 'smartfollow': 8085, 'linefollow': 8086,
            'webteleop': 8087,
        }  # fmt: skip
        for app_id, app in apps.items():
            assert app['unit'] == f'neoracer-{app_id}'
        exclusive = [a['id'] for a in dashboard.APPS if a['exclusive']]
        assert exclusive == [
            'wallfollow',
            'pursuit',
            'eps',
            'smartfollow',
            'linefollow',
            'webteleop',
        ]
        startable = [a['id'] for a in dashboard.APPS if a['startable']]
        assert startable == LABS

    def test_lab_names_match_racecar_service(self):
        # The tile units are the ones `racecar service` and setup install.
        tool = (SCRIPTS / 'racecar-tool.sh').read_text()
        listed = re.search(r'_RC_DASH_UNITS=\(([^)]*)\)', tool).group(1).split()
        assert sorted(listed) == sorted(f'neoracer-{lab}' for lab in LABS)

    def test_no_override_or_placeholder_tile(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert 'override' not in html.lower()
        assert all(a['unit'] for a in dashboard.APPS)

    def test_every_app_icon_exists(self, dashboard):
        icons = (SCRIPTS / 'web' / 'icons.svg').read_text()
        for item in dashboard.APPS + dashboard.FEATURES:
            assert f'id="{item["icon"]}"' in icons, item['icon']

    def test_open_exclusive_stops_the_other_first(self, dashboard):
        run = FakeRun({'neoracer-webteleop': 'active'})
        ok, _ = dashboard.open_app('linefollow', run)
        assert ok
        assert run.sent() == [
            ('stop', 'neoracer-webteleop.service'),
            ('start', 'neoracer-linefollow.service'),
        ]

    def test_open_running_exclusive_does_nothing(self, dashboard):
        run = FakeRun({'neoracer-wallfollow': 'active'})
        assert dashboard.open_app('wallfollow', run)[0]
        assert not run.sent()

    def test_open_camlabel_leaves_the_driving_app_running(self, dashboard):
        run = FakeRun({'neoracer-webteleop': 'active'})
        ok, message = dashboard.open_app('camlabel', run)
        assert ok and message == 'Camera Label started.'
        assert run.states['neoracer-webteleop'] == 'active'
        assert run.states['neoracer-camlabel'] == 'active'

    def test_open_stopped_core_app_names_the_command(self, dashboard):
        run = FakeRun({'neoracer-mapview': 'inactive'})
        ok, message = dashboard.open_app('mapview', run)
        assert not ok and 'racecar service start mapview' in message
        assert not run.sent()

    def test_clear_apps_stops_lab_dashboards_only(self, dashboard):
        run = FakeRun({'neoracer-mapview': 'active', 'neoracer-jupyter': 'active',
                       'neoracer-webteleop': 'active', 'neoracer-camlabel': 'active',
                       'neoracer-dashboard': 'active', 'neoracer-teleop': 'active'})  # fmt: skip
        assert dashboard.clear_apps(run) == (True, 'Lab dashboards stopped.')
        assert run.states['neoracer-webteleop'] == 'inactive'
        assert run.states['neoracer-camlabel'] == 'inactive'
        for unit in (
            'neoracer-mapview',
            'neoracer-jupyter',
            'neoracer-dashboard',
            'neoracer-teleop',
        ):
            assert run.states[unit] == 'active'

    def test_clear_apps_names_the_unit_it_may_not_stop(self, dashboard):
        run = FakeRun({'neoracer-linefollow': 'active'}, fail='stop', stderr=NO_SUDOERS)
        ok, message = dashboard.clear_apps(run)
        assert not ok and 'neoracer-linefollow.service' in message
        assert 'racecar service install' in message

    def test_clear_apps_with_none_running(self, dashboard):
        assert dashboard.clear_apps(FakeRun({})) == (True, 'No lab dashboards running.')

    def test_clear_log_hides_earlier_lines_and_keeps_the_file(
        self, dashboard, tmp_path, monkeypatch
    ):
        log = tmp_path / 'watchdog.log'
        log.write_text('old 1\nold 2\n')
        monkeypatch.setattr(dashboard, 'WATCHDOG_LOG', log)
        monkeypatch.setattr(dashboard, 'WATCHDOG_CLEARED', tmp_path / 'watchdog.cleared')
        assert dashboard._read_watchdog_tail() == ['old 1', 'old 2']
        assert dashboard.clear_watchdog_log() == (True, 'Log cleared.')
        assert dashboard._read_watchdog_tail() == []
        with log.open('a') as f:
            f.write('new 1\n')
        assert dashboard._read_watchdog_tail() == ['new 1']
        assert log.read_text().startswith('old 1')
        log.write_text('fresh\n')  # a new session's log is shorter than the mark
        assert dashboard._read_watchdog_tail() == ['fresh']

    def test_open_refuses_unknown(self, dashboard):
        assert dashboard.open_app('../x', FakeRun({})) == (False, 'Unknown app: ../x')

    def test_app_ready_needs_an_answer(self, dashboard):
        server = ThreadingHTTPServer(('127.0.0.1', 0), BaseHTTPRequestHandler)
        port = server.server_address[1]
        threading.Thread(target=server.serve_forever, daemon=True).start()
        try:
            assert dashboard.app_ready(port)
        finally:
            server.shutdown()
            server.server_close()
        assert not dashboard.app_ready(port, timeout=0.2)

    def test_launch_page_opens_then_waits(self, dashboard):
        html = dashboard._read_page(dashboard._LAUNCH_PATH)
        assert "fetch('/api/open'" in html and '/api/ready?app=' in html
        assert 'location.replace' in html and 'id="cancel"' in html
        assert 'window.close()' in html
        assert 'Try again' not in html

    def test_unit_states_in_order(self, dashboard):
        run = FakeRun({'neoracer-mapview': 'active', 'neoracer-jupyter': 'failed'})
        states = dashboard.unit_states(['neoracer-mapview', 'neoracer-jupyter', 'neoracer-x'], run)
        assert states == {
            'neoracer-mapview': 'active', 'neoracer-jupyter': 'failed', 'neoracer-x': 'inactive'
        }  # fmt: skip
        apps = dashboard.app_list(states)
        by_id = {a['id']: a['state'] for a in apps}
        assert by_id['mapview'] == 'active' and by_id['jupyter'] == 'failed'
        assert by_id['webteleop'] == 'unknown'

    def test_activate_stops_the_running_lab_first(self, dashboard):
        run = FakeRun({'neoracer-webteleop': 'active', 'neoracer-mapview': 'active'})
        ok, message = dashboard.activate('wallfollow', run)
        assert ok and message == 'Wall Follow active.'
        assert run.sent() == [
            ('stop', 'neoracer-webteleop.service'),
            ('start', 'neoracer-wallfollow.service'),
        ]
        assert run.states['neoracer-mapview'] == 'active'  # core services untouched

    def test_activate_none_stops_every_lab(self, dashboard):
        run = FakeRun({'neoracer-linefollow': 'active', 'neoracer-pursuit': 'active'})
        assert dashboard.activate('none', run) == (True, 'Lab dashboards stopped.')
        assert run.states['neoracer-linefollow'] == 'inactive'
        assert run.states['neoracer-pursuit'] == 'inactive'

    def test_activate_names_the_missing_permission(self, dashboard):
        run = FakeRun({}, fail='start', stderr=NO_SUDOERS)
        ok, message = dashboard.activate('linefollow', run)
        assert not ok and message == (
            'No permission to start neoracer-linefollow.service. Run: racecar service install'
        )

    def test_a_systemctl_error_is_passed_on(self, dashboard):
        stderr = 'Failed to start neoracer-eps.service: Unit neoracer-eps.service not found.'
        ok, message = dashboard.activate('eps', FakeRun({}, fail='start', stderr=stderr))
        assert not ok and message == stderr

    def test_activate_refuses_unknown_and_core_units(self, dashboard):
        run = FakeRun({})
        for target in ('mapview', 'jupyter', 'camlabel', '../x', ''):
            ok, _ = dashboard.activate(target, run)
            assert not ok
        assert not run.sent()

    def test_every_command_is_one_the_sudoers_file_allows(self, dashboard, monkeypatch, tmp_path):
        # Plan D9: each neoracer-* unit by its full name; start, stop and
        # restart only; the lab dashboards and teleop's restart from this page.
        monkeypatch.setattr(dashboard.features, 'state_file', lambda: tmp_path / 'f.conf')
        run = FakeRun({})
        for lab in LABS:
            dashboard.open_app(lab, run)
        dashboard.clear_apps(run)
        dashboard.apply_features(dashboard.features.DEFAULTS, run)
        labs = {f'neoracer-{lab}.service' for lab in LABS}
        for argv in run.calls:
            if argv[0] == 'systemctl':
                assert argv[1] == 'is-active', argv
                continue
            assert re.fullmatch(r'sudo -n systemctl (start|stop|restart) neoracer-\w+\.service',
                                ' '.join(argv)), argv  # fmt: skip
            verb, unit = argv[3], argv[4]
            assert (verb in ('start', 'stop') and unit in labs) or (
                verb, unit) == ('restart', 'neoracer-teleop.service'), argv  # fmt: skip

    def test_page_uses_the_shared_assets(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert '/ui/neoracer.css' in html and '/ui/brand/' in html and '/launch?app=' in html
        assert 'class="for-dark" src="/ui/brand/neoracer-logo-reversed.svg"' in html
        assert 'class="for-light" src="/ui/brand/neoracer-logo.svg"' in html
        assert 'href="/health"' in html

    @pytest.mark.parametrize('page', ['dashboard.html', 'dashboard_launch.html'])
    def test_pages_carry_no_upstream_brand_or_licensed_face(self, page):
        text = (SCRIPTS / page).read_text().lower()
        for banned in ('--rcn-', 'rcn-', 'barlow', 'kernel', 'racecar neo', 'base64'):
            assert banned not in text, (page, banned)


class TestHttp:
    @pytest.fixture
    def server(self, dashboard):
        srv = ThreadingHTTPServer(('127.0.0.1', 0), dashboard.DashboardHandler)
        threading.Thread(target=srv.serve_forever, daemon=True).start()

        def request(method, path, body=None):
            conn = http.client.HTTPConnection('127.0.0.1', srv.server_address[1], timeout=5)
            conn.request(method, path, body=None if body is None else json.dumps(body))
            resp = conn.getresponse()
            return resp.status, resp.getheader('Content-Type'), resp.read()

        yield request
        srv.shutdown()
        srv.server_close()

    def test_pages(self, server):
        assert server('GET', '/launch?app=wallfollow')[:2] == (200, 'text/html; charset=utf-8')
        assert server('GET', '/')[:2] == (200, 'text/html; charset=utf-8')

    def test_ready_answers_for_known_apps_only(self, server, dashboard, monkeypatch):
        monkeypatch.setattr(dashboard, 'app_ready', lambda port: port == 8081)
        status, _, body = server('GET', '/api/ready?app=wallfollow')
        assert status == 200 and json.loads(body) == {'ready': True, 'port': 8081}
        assert server('GET', '/api/ready?app=nope')[0] == 404

    def test_open_an_unknown_app_is_refused(self, server, dashboard, monkeypatch):
        monkeypatch.setattr(dashboard, '_refresh_apps', lambda: None)
        status, _, body = server('POST', '/api/open', {'app': 'nope'})
        assert status == 400 and json.loads(body) == {'ok': False, 'message': 'Unknown app: nope'}

    def test_unknown_post_routes_are_404(self, server):
        assert server('POST', '/api/override', {'action': 'start'})[0] == 404


TELEOP, AUTONOMY = 'neoracer-teleop.service', 'neoracer-autonomy.service'


class TestFeatures:
    def test_tiles(self, dashboard):
        assert [f['id'] for f in dashboard.FEATURES] == ['camera', 'detections', 'autonomy']
        camera = [a['id'] for a in dashboard.APPS if a.get('needs') == 'camera']
        assert camera == ['camlabel', 'linefollow']
        # pursuit and smartfollow read /edgetpu/inference.
        inference = [a['id'] for a in dashboard.APPS if a.get('needs') == 'inference']
        assert inference == ['pursuit', 'smartfollow']

    def test_every_need_is_a_feature_the_page_turns_on(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert "camera: { feature: 'camera'" in html
        assert "inference: { feature: 'detections'" in html
        assert dashboard.features.NEEDS == {'detections': 'camera'}

    def test_states(self, dashboard):
        wanted = {'camera': True, 'detections': False}
        items = dashboard.feature_list(
            {TELEOP: 'active', AUTONOMY: 'inactive'}, wanted, wanted, False, {}
        )
        assert [i['state'] for i in items] == ['active', 'inactive', 'inactive']

    def test_pending_restart_shows_starting(self, dashboard):
        items = dashboard.feature_list(
            {TELEOP: 'active'},
            {'camera': True, 'detections': True},
            {'camera': True, 'detections': False},
            False,
            {},
        )
        assert items[1]['state'] == 'activating'
        items = dashboard.feature_list(
            {TELEOP: 'active'}, dashboard.features.DEFAULTS, dashboard.features.DEFAULTS, True, {}
        )
        assert items[0]['state'] == items[1]['state'] == 'activating'

    def test_teleop_down_reads_off(self, dashboard):
        items = dashboard.feature_list(
            {TELEOP: 'inactive'},
            dashboard.features.DEFAULTS,
            dashboard.features.DEFAULTS,
            False,
            {},
        )
        assert items[0]['state'] == 'inactive'

    def test_last_run(self, dashboard, tmp_path):
        path = tmp_path / 'autonomy.conf'
        assert dashboard.last_run(path) == {}
        path.write_text('AUTONOMY_MODE=localization\nAUTONOMY_MAP=hall\n')
        assert dashboard.last_run(path) == {'mode': 'localization', 'map': 'hall'}
        path.write_text('AUTONOMY_MODE=off\nAUTONOMY_MAP=hall\n')
        assert dashboard.last_run(path) == {}


class _ToolRun(FakeRun):
    """FakeRun that also answers `racecar map` calls."""

    def __init__(self, dashboard, states, codes=None):
        super().__init__(states)
        self.map_tool = list(dashboard.MAP_TOOL)
        self.codes = codes or {}
        self.tool = []

    def __call__(self, argv, **kw):
        if argv[:2] == self.map_tool:
            action = argv[2]
            self.tool.append(action)
            if action == 'stop':
                self.states[_short(AUTONOMY)] = 'inactive'
            if action == 'resume':
                self.states[_short(AUTONOMY)] = 'active'
            code = self.codes.get(action, 0)
            return SimpleNamespace(stdout=f'{action} done\n', stderr='', returncode=code)
        return super().__call__(argv, **kw)


RESTART_TELEOP = ['sudo', '-n', 'systemctl', 'restart', TELEOP]


class TestApplyFeatures:
    @pytest.fixture
    def tool(self, dashboard, monkeypatch, tmp_path):
        monkeypatch.setattr(dashboard.features, 'state_file', lambda: tmp_path / 'f.conf')
        monkeypatch.setattr(dashboard.features, 'boot_id', lambda: 'b')
        return lambda *a, **k: _ToolRun(dashboard, *a, **k)

    def test_restarts_teleop_and_writes_the_choice(self, dashboard, tool):
        run = tool({TELEOP: 'active'})
        message = dashboard.apply_features({'camera': False, 'detections': False}, run)
        assert RESTART_TELEOP in run.calls
        assert run.tool == [] and message == 'Camera off. Detections off.'
        assert dashboard.features.read() == {'camera': False, 'detections': False}

    def test_autonomy_is_stopped_first_and_resumed(self, dashboard, tool):
        run = tool({TELEOP: 'active', AUTONOMY: 'active'})
        message = dashboard.apply_features(
            {'camera': True, 'detections': True}, run, sleep=lambda s: None
        )
        restart = run.calls.index(RESTART_TELEOP)
        assert run.tool == ['stop', 'resume'] and restart > 0
        assert message.endswith('resume done')

    def test_a_driving_app_opened_meanwhile_keeps_autonomy_stopped(self, dashboard, tool):
        # The apps page turns detections on while Pure Pursuit opens; a resume
        # would stop Pure Pursuit again (racecar map resume stops driving apps).
        run = tool({TELEOP: 'active', AUTONOMY: 'active'})

        def sleep(_sec):
            run.states['neoracer-pursuit'] = 'active'

        message = dashboard.apply_features({'camera': True, 'detections': True}, run, sleep=sleep)
        assert run.tool == ['stop']
        assert message.endswith('Autonomy stays stopped, because Pure Pursuit is open.')

    def test_a_failed_stop_leaves_teleop_alone(self, dashboard, tool):
        run = tool({TELEOP: 'active', AUTONOMY: 'active'}, codes={'stop': 1})
        message = dashboard.apply_features(dashboard.features.DEFAULTS, run)
        assert message.startswith('Could not stop autonomy')
        assert not [c for c in run.calls if 'restart' in c]

    def test_a_missing_sudoers_file_is_named(self, dashboard, tool):
        run = tool({TELEOP: 'active'})
        run.fail, run.stderr = 'restart', NO_SUDOERS
        message = dashboard.apply_features(dashboard.features.DEFAULTS, run)
        assert message == f'No permission to restart {TELEOP}. Run: racecar service install'

    def test_the_map_tool_is_neoracer_map(self, dashboard):
        # scripts/neoracer_map.py comes with F25 (Names and specs).
        assert Path(dashboard.MAP_TOOL[-1]).name == 'neoracer_map.py'

    def test_autonomy_actions(self, dashboard, tool):
        run = tool({AUTONOMY: 'active'})
        assert dashboard.autonomy_action('save-stop', run) == 'stop done'
        assert run.tool == ['save', 'stop']
        run = tool({AUTONOMY: 'active'}, codes={'save': 1})
        assert dashboard.autonomy_action('save-stop', run).startswith('Not stopped.')
        assert run.tool == ['save']
        assert dashboard.autonomy_action('resume', tool({})) == 'resume done'
        with pytest.raises(ValueError):
            dashboard.autonomy_action('rm', run)


class TestFeatureEndpoint:
    def test_rejects_unknown_and_busy(self, dashboard, monkeypatch):
        job = dashboard.FeatureJob()
        monkeypatch.setattr(dashboard, '_features_job', job)
        assert dashboard.change_feature('lidar', False) == (False, "unknown feature 'lidar'")
        assert dashboard.change_feature('autonomy', True, 'go')[0] is False
        assert job.start('busy', lambda: 'x' if __import__('time').sleep(0.3) is None else '')
        assert dashboard.change_feature('autonomy', True, 'resume') == (
            False,
            'Another change is running.',
        )

    def test_only_a_teleop_restart_marks_camera_and_detections(
        self, dashboard, monkeypatch, tmp_path
    ):
        # Starting or stopping autonomy leaves teleop running, so the camera
        # and detections tiles keep their state; a camera change restarts teleop.
        gate = threading.Event()
        monkeypatch.setattr(dashboard, '_features_job', dashboard.FeatureJob())
        monkeypatch.setattr(dashboard, 'autonomy_action', lambda action: gate.wait(5) and 'done')
        monkeypatch.setattr(dashboard, 'apply_features', lambda values: gate.wait(5) and 'done')
        monkeypatch.setattr(dashboard, 'unit_states', lambda units: dict.fromkeys(units, 'active'))
        monkeypatch.setattr(dashboard, 'last_run', lambda: {})
        monkeypatch.setattr(dashboard.features, 'state_file', lambda: tmp_path / 'f.conf')
        defaults = dict(dashboard.features.DEFAULTS)
        monkeypatch.setattr(dashboard.features, 'running', lambda: defaults)

        def tiles():
            return [i['state'] for i in dashboard.features_view()['items'][:2]]

        def finish():
            gate.set()
            deadline = time.monotonic() + 5
            while dashboard._features_job.state()['busy'] and time.monotonic() < deadline:
                time.sleep(0.01)
            gate.clear()

        assert dashboard.change_feature('autonomy', True, 'resume')[0]
        assert tiles() == ['active', 'inactive']
        finish()
        assert dashboard.change_feature('camera', False)[0]
        assert tiles() == ['activating', 'activating']
        finish()

    def test_a_topic_launched_off_reads_off_in_the_rates(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert "if (r.off) return '<span class=\"nr-badge off\">Off</span>';" in html
        source = Path(dashboard.__file__).read_text()
        assert "rates[topic] = {'hz': None, 'stale': False, 'off': True}" in source

    def test_page_has_features(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert 'id="features"' in html and 'id="autosheet"' in html
        assert 'needs the camera. Turn it on?' in html
        assert 'needs detections. Turn them on?' in html

    def test_camera_and_detections_read_restarting_while_teleop_restarts(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert "const RESTARTING = ['starting', 'Restarting', 'Restarting'];" in html
        assert "state === 'activating' && item.restarts ? RESTARTING : APP_STATE[state]" in html
        assert "restarts: f.id !== 'autonomy'" in html

    @pytest.mark.parametrize(('page', 'seconds'), [('dashboard.html', 3), ('health.html', 2)])
    def test_the_live_square_explains_itself_at_the_page_poll_rate(self, page, seconds):
        html = (Path(__file__).resolve().parent.parent / 'scripts' / page).read_text()
        assert f'const POLL_MS = {seconds}000;' in html and 'const MISSES = 2;' in html
        title = f'id="live" title="The page asks the car for its status every {seconds} seconds'
        assert title in html
        assert 'It reads No answer after two requests in a row fail.' in html

    def test_the_title_hides_the_default_host_name(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert "const DEFAULT_HOST = 'neoracer';" in html
        assert 'name.toLowerCase() !== DEFAULT_HOST' in html
        assert "$('host').textContent = shownHost(data.host);" in html

    def test_the_badge_names_the_parts_behind_it(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert 'Every box under System status, except Autonomy, is running or turned off.' in html
        assert "${named('Fault', faults)}${named('Check', checks)}Open System status" in html
        assert "n.status !== 'off' && !faults.includes(n)" in html


class TestDrivingAppsAndAutonomy:
    @pytest.fixture
    def tool(self, dashboard):
        return lambda *a, **k: _ToolRun(dashboard, *a, **k)

    def test_opening_a_driving_app_saves_and_stops_mapping(self, dashboard, tool, monkeypatch):
        monkeypatch.setattr(dashboard, 'last_run', lambda: {'mode': 'mapping', 'map': 'hall'})
        run = tool({AUTONOMY: 'active'})
        ok, _ = dashboard.open_app('webteleop', run)
        assert ok and run.tool == ['save', 'stop']
        assert run.states['neoracer-webteleop'] == 'active'

    def test_opening_a_driving_app_stops_localization(self, dashboard, tool, monkeypatch):
        monkeypatch.setattr(dashboard, 'last_run', lambda: {'mode': 'localization', 'map': 'hall'})
        run = tool({AUTONOMY: 'active'})
        assert dashboard.open_app('linefollow', run)[0] and run.tool == ['stop']

    def test_a_failed_save_keeps_mapping_and_the_app_closed(self, dashboard, tool, monkeypatch):
        monkeypatch.setattr(dashboard, 'last_run', lambda: {'mode': 'mapping', 'map': 'hall'})
        run = tool({AUTONOMY: 'active'}, codes={'save': 1})
        ok, message = dashboard.open_app('wallfollow', run)
        assert not ok and message.startswith('Autonomy is still running.')
        assert run.states.get('neoracer-wallfollow', 'inactive') == 'inactive'

    def test_camera_label_leaves_autonomy_running(self, dashboard, tool):
        run = tool({AUTONOMY: 'active'})
        assert dashboard.open_app('camlabel', run)[0] and run.tool == []

    def test_page_asks_before_stopping_autonomy(self, dashboard):
        html = dashboard._load_dashboard_html()
        assert 'Save it, stop it, and open' in html and 'will close.' in html


class TestOnDemand:
    class FakeGraphNode:
        def __init__(self):
            self.destroyed = []

        def destroy_subscription(self, sub):
            self.destroyed.append(sub)

    def test_rate_subscriptions_detach_and_counts_reset(self, sampler):
        s, _clock = sampler
        node = self.FakeGraphNode()
        s._subs = {'/scan': 'sub-scan', '/motor': 'sub-motor'}
        s.destroy_subscription = node.destroy_subscription
        s.attached_at = 1.0
        s._stamps['/scan'].extend([1.0, 1.1])
        s.detach_subscriptions()
        assert sorted(node.destroyed) == ['sub-motor', 'sub-scan']
        assert s._subs == {} and s.attached_at is None and not s._stamps['/scan']

    def test_warming_until_the_window_fills(self, sampler):
        s, clock = sampler
        s.attached_at = clock.t
        assert s.warming()
        clock.t += s._window + 0.1
        assert not s.warming()

    def test_view_tracking(self, dashboard, monkeypatch):
        monkeypatch.setattr(dashboard, '_last_view', 0.0)
        dashboard._wake.clear()
        assert dashboard.note_view() is True and dashboard._wake.is_set()
        assert dashboard.viewing()
        assert dashboard.note_view() is False
