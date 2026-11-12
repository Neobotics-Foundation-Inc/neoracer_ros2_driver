"""Unit tests for scripts/watchdog.py: NODES schema and restart logic."""

import json
import logging
from pathlib import Path

from conftest import load_script
import pytest

LAUNCH_DIR = Path(__file__).resolve().parent.parent / 'launch'
SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'


@pytest.fixture(scope='module')
def watchdog():
    return load_script('watchdog')


class TestNodesDict:
    EXPECTED_NAMES = {
        'controller',
        'throttle',
        'mux',
        'gamepad',
        'lidar',
        'camera',
    }
    REQUIRED_KEYS = {
        'topic',
        'launch',
        'device_check',
        'device_label',
        'kill_pattern',
        'process_pattern',
    }

    def test_all_expected_nodes_present(self, watchdog):
        assert set(watchdog.NODES) == self.EXPECTED_NAMES

    def test_led_matrix_and_inference_are_not_supervised(self, watchdog):
        # A USB-serial re-init and a crash-looping TensorRT engine are both
        # riskier than what a restart would recover.
        assert 'led_matrix' not in watchdog.NODES
        assert 'inference' not in watchdog.NODES

    @pytest.mark.parametrize('name', sorted(EXPECTED_NAMES))
    def test_node_has_required_keys(self, watchdog, name):
        missing = self.REQUIRED_KEYS - set(watchdog.NODES[name])
        assert not missing, f'{name} missing keys: {missing}'

    @pytest.mark.parametrize('name', sorted(EXPECTED_NAMES))
    def test_topic_starts_with_slash(self, watchdog, name):
        topic = watchdog.NODES[name]['topic']
        assert topic.startswith('/'), f'{name} topic {topic!r} must start with /'

    @pytest.mark.parametrize('name', sorted(EXPECTED_NAMES))
    def test_launch_file_exists(self, watchdog, name):
        launch_file = LAUNCH_DIR / watchdog.NODES[name]['launch']
        assert launch_file.is_file(), f'{name}: launch file {launch_file} missing'

    @pytest.mark.parametrize('name', sorted(EXPECTED_NAMES))
    def test_device_check_returns_bool(self, watchdog, name):
        assert isinstance(watchdog.NODES[name]['device_check'](), bool)

    def test_enable_args_match_teleop_launch(self, watchdog):
        teleop = (LAUNCH_DIR / 'teleop.launch.py').read_text()
        for name, cfg in watchdog.NODES.items():
            if 'enable_arg' in cfg:
                subsystem = cfg['enable_arg'].removesuffix('_enable')
                assert f"'{subsystem}'" in teleop, f'{name}: {subsystem} not a teleop subsystem'

    def test_control_nodes_cannot_be_disabled(self, watchdog):
        for name in ('controller', 'throttle', 'mux', 'gamepad'):
            assert 'enable_arg' not in watchdog.NODES[name]

    def test_camera_topic(self, watchdog):
        assert watchdog.NODES['camera']['topic'] == '/camera/color'

    def test_controller_watches_the_esp32_imu_stream(self, watchdog):
        cfg = watchdog.NODES['controller']
        assert cfg['topic'] == '/imu/fused'
        assert cfg['freshness_sec'] <= watchdog.RESTART_COOLDOWN

    def test_lidar_has_freshness_threshold(self, watchdog):
        # A LakiBeam that drops its UDP frames keeps its process and
        # advertisement; only freshness catches it. The window must span
        # several scans and stay under the cooldown.
        fresh = watchdog.NODES['lidar'].get('freshness_sec')
        assert fresh is not None, 'lidar must define freshness_sec'
        assert 1.0 <= fresh <= watchdog.RESTART_COOLDOWN

    def test_lidar_device_is_its_usb_link(self, watchdog):
        assert watchdog.NODES['lidar']['device_check'] is watchdog.lidar_link_up


class TestLidarLink:
    def _iface(self, root, carrier):
        d = root / 'usb0'
        d.mkdir()
        if carrier is not None:
            (d / 'carrier').write_text(carrier + '\n')
        return root

    def test_link_with_carrier(self, watchdog, tmp_path):
        assert watchdog.lidar_link_up(self._iface(tmp_path, '1')) is True

    def test_link_without_carrier(self, watchdog, tmp_path):
        assert watchdog.lidar_link_up(self._iface(tmp_path, '0')) is False

    def test_down_interface_whose_carrier_does_not_read(self, watchdog, tmp_path):
        assert watchdog.lidar_link_up(self._iface(tmp_path, None)) is False

    def test_unplugged_cable_removes_the_interface(self, watchdog, tmp_path):
        assert watchdog.lidar_link_up(tmp_path) is False


class TestConfig:
    def test_poll_interval_reasonable(self, watchdog):
        assert 1 <= watchdog.POLL_INTERVAL <= 30

    def test_restart_cooldown_at_least_15s(self, watchdog):
        # A node that crashes on start must not respawn in a tight loop.
        assert watchdog.RESTART_COOLDOWN >= 15

    def test_package_name(self, watchdog):
        assert watchdog.PACKAGE == 'neoracer_ros2_driver'


class TestStaleAge:
    def test_no_window_is_never_stale(self, watchdog):
        assert watchdog.stale_age(None, True, True, 100.0, 60.0) is None

    def test_old_message_is_stale(self, watchdog):
        assert watchdog.stale_age(5.0, True, True, 100.0, 7.5) == 7.5

    def test_recent_message_is_fresh(self, watchdog):
        assert watchdog.stale_age(5.0, True, True, 100.0, 0.2) is None

    def test_exactly_at_the_window_is_fresh(self, watchdog):
        assert watchdog.stale_age(5.0, True, True, 100.0, 5.0) is None

    def test_no_message_yet_is_not_stale(self, watchdog):
        assert watchdog.stale_age(5.0, True, True, 100.0, None) is None

    def test_grace_after_a_restart(self, watchdog):
        # The restarted node gets one full window before it is judged.
        assert watchdog.stale_age(5.0, True, True, 4.9, 30.0) is None
        assert watchdog.stale_age(5.0, True, True, 5.0, 30.0) == 30.0

    @pytest.mark.parametrize('topic_alive,proc_alive', [(False, True), (True, False)])
    def test_not_judged_while_down(self, watchdog, topic_alive, proc_alive):
        assert watchdog.stale_age(5.0, topic_alive, proc_alive, 100.0, 30.0) is None


class TestFailureReason:
    @pytest.mark.parametrize(
        'topic_alive,proc_alive,stale,expected',
        [
            (True, True, None, None),
            (False, False, None, 'topic+process down'),
            (False, True, None, 'topic not advertised'),
            (True, False, None, 'process not running'),
            (True, True, 7.25, 'topic stale (7.2s)'),
            (False, True, 7.25, 'topic not advertised'),
        ],
    )
    def test_reasons(self, watchdog, topic_alive, proc_alive, stale, expected):
        assert watchdog.failure_reason(topic_alive, proc_alive, stale) == expected


def _healthy_run(watchdog, st, t0, seconds):
    """Report the node healthy every poll for `seconds`; return the end time."""
    t = t0
    while t < t0 + seconds:
        watchdog.step(st, t, 0.0, None, lambda: True, 'dev')
        t += watchdog.POLL_INTERVAL
    return t


def _fail(watchdog, st, t, started=-1e9, device=True):
    return watchdog.step(st, t, started, 'topic not advertised', lambda: device, 'dev')


def _restart_times(watchdog, st, t0, seconds):
    """Fail every poll for `seconds`; return the times a restart was ordered."""
    times = []
    t = t0
    while t <= t0 + seconds:
        restart, _ = _fail(watchdog, st, t)
        if restart:
            times.append(t - t0)
        t += 1.0
    return times


class TestBackoff:
    def test_schedule(self, watchdog):
        assert [watchdog.backoff(n) for n in range(7)] == [0, 30, 60, 120, 240, 300, 300]


class TestStep:
    def test_healthy_node_skips_the_device_check(self, watchdog):
        def device_check():
            raise AssertionError('device_check must not run for a healthy node')

        st = watchdog.NodeState()
        restart, _ = watchdog.step(st, 100.0, 0.0, None, device_check, 'dev')
        assert (restart, st.state) == (False, 'healthy')

    def test_startup_grace_holds_off_a_boot_race(self, watchdog):
        # The first poll runs before discovery has listed teleop's topics.
        st = watchdog.NodeState()
        restart, events = _fail(watchdog, st, 1.0, started=0.0)
        assert (restart, st.state, events) == (False, 'starting', [])

    def test_grace_ends_after_the_window(self, watchdog):
        st = watchdog.NodeState()
        restart, _ = _fail(watchdog, st, watchdog.STARTUP_GRACE_SEC, started=0.0)
        assert restart is True

    def test_grace_ends_once_the_topic_was_seen(self, watchdog):
        st = watchdog.NodeState()
        watchdog.step(st, 1.0, 0.0, None, lambda: True, 'dev')
        restart, _ = _fail(watchdog, st, 6.0, started=0.0)
        assert restart is True

    def test_crash_loop_backs_off_then_gives_up(self, watchdog):
        st = watchdog.NodeState()
        times = _restart_times(watchdog, st, 0.0, 2000.0)
        assert times == [0.0, 30.0, 90.0, 210.0, 450.0]
        assert st.state == 'gave_up'
        assert st.restarts == watchdog.MAX_RESTARTS

    def test_gave_up_logs_once_then_reminds(self, watchdog):
        st = watchdog.NodeState()
        errors = []
        for t in range(0, 3000):
            _, events = _fail(watchdog, st, float(t))
            errors += [(t, m) for level, m in events if level == logging.ERROR]
        assert errors[0][1].startswith('topic not advertised; giving up')
        gaps = [b[0] - a[0] for a, b in zip(errors, errors[1:], strict=False)]
        assert gaps and all(g == watchdog.REMINDER_SEC for g in gaps)

    def test_missing_device_never_restarts_and_logs_once(self, watchdog):
        st = watchdog.NodeState()
        logged = []
        for t in range(0, 300, 5):
            restart, events = _fail(watchdog, st, float(t), device=False)
            assert restart is False
            logged += events
        assert st.state == 'no_device'
        assert len(logged) == 1

    def test_replug_rearms_a_node_that_gave_up(self, watchdog):
        st = watchdog.NodeState()
        _restart_times(watchdog, st, 0.0, 2000.0)
        assert st.state == 'gave_up'
        _fail(watchdog, st, 2100.0, device=False)
        assert st.state == 'no_device'
        restart, events = _fail(watchdog, st, 2105.0)
        assert restart is True
        assert st.restarts == 1
        assert any('restart budget reset' in m for _, m in events)

    def test_recovering_on_its_own_rearms(self, watchdog):
        st = watchdog.NodeState()
        _restart_times(watchdog, st, 0.0, 2000.0)
        watchdog.step(st, 2100.0, 0.0, None, lambda: True, 'dev')
        assert st.state == 'healthy'

    def test_sixty_seconds_of_health_clears_the_backoff(self, watchdog):
        st = watchdog.NodeState()
        _fail(watchdog, st, 0.0)
        _fail(watchdog, st, 30.0)
        assert st.restarts == 2
        t = _healthy_run(watchdog, st, 40.0, watchdog.HEALTHY_RESET_SEC + 5)
        assert st.restarts == 0
        restart, _ = _fail(watchdog, st, t)
        assert restart is True

    def test_short_health_keeps_the_backoff(self, watchdog):
        st = watchdog.NodeState()
        _fail(watchdog, st, 0.0)
        _healthy_run(watchdog, st, 10.0, 20.0)
        restart, _ = _fail(watchdog, st, 31.0)
        assert (restart, st.restarts) == (True, 2)

    def test_disabled_node_is_left_alone(self, watchdog):
        st = watchdog.NodeState()
        for t in (0.0, 100.0, 1000.0):
            restart, _ = watchdog.step(
                st, t, -1e9, 'topic+process down', lambda: True, 'dev', 'lidar_enable'
            )
            assert restart is False
        assert st.state == 'disabled'


class TestDisabledArgs:
    def test_false_values(self, watchdog, tmp_path):
        f = tmp_path / 'launch_args'
        f.write_text(
            'lidar_enable:=false\ncamera_enable:=0\n'
            'inference_enable:=true\nsensorip:=192.168.8.2\n'
        )
        assert watchdog.disabled_args(f) == {'lidar_enable', 'camera_enable'}

    def test_missing_or_empty_file(self, watchdog, tmp_path):
        assert watchdog.disabled_args(tmp_path / 'absent') == set()
        (tmp_path / 'empty').write_text('\n')
        assert watchdog.disabled_args(tmp_path / 'empty') == set()

    def test_launcher_records_the_arguments(self):
        script = (SCRIPTS / 'launch_teleop.sh').read_text()
        assert 'launch_args' in script


class TestProcessTable:
    @pytest.mark.skipif(not Path('/proc').is_dir(), reason='no /proc off Linux')
    def test_reads_this_process(self, watchdog):
        lines = watchdog.read_cmdlines()
        assert any('pytest' in line or 'python' in line for line in lines)

    def test_no_proc_reads_as_empty(self, watchdog, tmp_path):
        assert watchdog.read_cmdlines(tmp_path / 'missing') == []

    def test_cmdline_nul_separators_become_spaces(self, watchdog, tmp_path):
        (tmp_path / '42').mkdir()
        (tmp_path / '42' / 'cmdline').write_bytes(
            b'/usr/bin/python3\0/x/lib/neoracer_ros2_driver/controller\0--ros-args\0'
        )
        (tmp_path / 'self').mkdir()
        assert watchdog.read_cmdlines(tmp_path) == [
            '/usr/bin/python3 /x/lib/neoracer_ros2_driver/controller --ros-args'
        ]

    def test_pattern_match(self, watchdog):
        assert watchdog.process_running(
            '/lib/neoracer_ros2_driver/controller',
            ['python3 /x/lib/neoracer_ros2_driver/controller'],
        )
        assert not watchdog.process_running(
            '/lib/neoracer_ros2_driver/controller',
            ['ros2 launch neoracer_ros2_driver controller.launch.py'],
        )


class TestStateFile:
    def test_snapshot_round_trips_through_sysinfo(self, watchdog, tmp_path):
        sysinfo = load_script('sysinfo')
        states = {'lidar': watchdog.NodeState(state='gave_up', restarts=5, reason='topic stale')}
        watchdog.write_state(
            tmp_path / sysinfo.WATCHDOG_STATE_FILE, watchdog.state_snapshot(states, 0.0)
        )
        got = sysinfo.read_watchdog_state(tmp_path)
        assert got['nodes']['lidar'] == {
            'state': 'gave_up',
            'restarts': 5,
            'reason': 'topic stale',
            'since': got['nodes']['lidar']['since'],
        }
        assert got['stale'] is False

    def test_old_file_reads_as_stale(self, tmp_path):
        sysinfo = load_script('sysinfo')
        (tmp_path / sysinfo.WATCHDOG_STATE_FILE).write_text(
            json.dumps({'updated': '2020-01-01T00:00:00', 'poll_sec': 5, 'nodes': {}})
        )
        assert sysinfo.read_watchdog_state(tmp_path)['stale'] is True

    def test_absent_or_corrupt_file(self, tmp_path):
        sysinfo = load_script('sysinfo')
        assert sysinfo.read_watchdog_state(tmp_path) is None
        (tmp_path / sysinfo.WATCHDOG_STATE_FILE).write_text('{not json')
        assert sysinfo.read_watchdog_state(tmp_path) is None


class TestOvercurrent:
    def test_clean_counters_say_nothing(self, watchdog):
        assert watchdog.overcurrent_message(0, []) is None

    def test_missing_counters_say_nothing(self, watchdog):
        assert watchdog.overcurrent_message(None, []) is None

    def test_a_fired_counter_is_named(self, watchdog):
        message = watchdog.overcurrent_message(
            0b010, ['over-current throttling oc2 has occurred (3 events)']
        )
        assert 'oc2' in message and 'battery' in message


class TestHelpers:
    def test_clean_fastrtps_orphans_returns_a_count(self, watchdog):
        assert watchdog._clean_fastrtps_orphans() >= 0


class _StubNode:
    def __init__(self) -> None:
        self.destroyed = []

    def destroy_subscription(self, sub) -> None:
        self.destroyed.append(sub)


@pytest.fixture
def monitor(watchdog):
    node = _StubNode()
    return watchdog._FreshnessMonitor(node, ['/scan']), node


class TestFreshnessMonitor:
    def test_age_none_before_any_message(self, monitor):
        fm, _ = monitor
        assert fm.age('/scan') is None

    def test_age_recent_after_mark(self, monitor):
        fm, _ = monitor
        fm._mark('/scan')
        assert 0 <= fm.age('/scan') < 0.5

    def test_reset_clears_last_seen_and_drops_the_subscription(self, monitor):
        fm, node = monitor
        fm._subs['/scan'] = 'sub'
        fm._mark('/scan')
        fm.reset('/scan')
        assert fm.age('/scan') is None
        assert node.destroyed == ['sub']
