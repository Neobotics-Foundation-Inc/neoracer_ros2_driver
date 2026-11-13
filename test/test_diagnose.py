"""Unit tests for scripts/diagnose.py (the `racecar status` diagnostic)."""

import json
from pathlib import Path
import re
import subprocess
from types import SimpleNamespace

from conftest import load_script
import pytest

ROOT = Path(__file__).resolve().parent.parent
SCRIPT = ROOT / 'scripts' / 'diagnose.py'


@pytest.fixture(scope='module')
def diag():
    return load_script('diagnose')


def _run(*args, timeout=60):
    return subprocess.run(
        ['python3', str(SCRIPT), *args],
        capture_output=True,
        text=True,
        timeout=timeout,
    )


class TestTopicSpecs:
    def test_floor_is_a_fraction_of_nominal(self, diag):
        spec = diag.TopicSpec('/t', 'T', 100.0, 0.8)
        assert spec.floor == pytest.approx(80.0)

    def test_sensors_default_to_eighty_percent(self, diag):
        for spec in diag.SENSOR_TOPICS:
            if spec.topic in diag.BOARD_TOPICS or spec.topic == '/battery/voltage':
                continue
            assert spec.floor_frac == 0.8, f'{spec.topic} should use the 80% floor'

    def test_board_topics_carry_the_wider_floor(self, diag):
        # The topics off the ESP32's frames slow when the controller's serial
        # reader is loaded, so their floor leaves room for it. A halved frame
        # rate must still warn, or the check stops meaning anything.
        board = [
            s
            for s in diag.SENSOR_TOPICS
            if s.topic in diag.BOARD_TOPICS and s.topic != '/battery/voltage'
        ]
        assert len(board) == 6
        for spec in board:
            assert spec.floor_frac == diag.BOARD_FLOOR_FRAC
            assert spec.floor > spec.nominal / 2, 'floor must still flag a halved frame rate'

    def test_rates_match_the_esp32_frames(self, diag):
        # docs/esp32_protocol.md: `s` at about 200 Hz, `m` 20, `r` 10, `b` 0.5.
        by_topic = {s.topic: s.nominal for s in diag.SENSOR_TOPICS}
        for topic in ('/imu/fused', '/odom', '/encoder/speed'):
            assert by_topic[topic] == 200.0, topic
        assert by_topic['/mag'] == 20.0
        assert by_topic['/rc/channels'] == by_topic['/joy'] == 10.0
        assert by_topic['/battery/voltage'] == 0.5

    def test_battery_floor_passes_two_frames_and_fails_one(self, diag):
        # One `b` frame every 2 s puts two or three in the default window.
        spec = next(s for s in diag.SENSOR_TOPICS if s.topic == '/battery/voltage')
        assert spec.floor <= 2 / diag.DEFAULT_WINDOW
        assert spec.floor > 1 / diag.DEFAULT_WINDOW

    def test_lidar_nominal_is_the_configured_rate(self, diag):
        launch = (ROOT / 'launch' / 'lidar.launch.py').read_text()
        scanfreq = re.search(r"'scanfreq', default_value=TextSubstitution\(text='\"(\d+)", launch)
        hz = float(scanfreq[1])
        scan = next(s for s in diag.SENSOR_TOPICS if s.topic == '/scan')
        assert scan.nominal == hz == 30.0

    def test_camera_nominal_is_the_configured_rate(self, diag):
        camera = (ROOT / 'config' / 'camera.yaml').read_text()
        fps = float(re.search(r'framerate:\s*([\d.]+)', camera)[1])
        color = next(s for s in diag.SENSOR_TOPICS if s.topic == '/camera/color')
        assert color.nominal == fps == 60.0

    def test_inference_nominal_is_the_configured_cap(self, diag):
        inference = (ROOT / 'config' / 'inference.yaml').read_text()
        cap = float(re.search(r'max_rate_hz:\s*([\d.]+)', inference)[1])
        spec = next(s for s in diag.SENSOR_TOPICS if s.topic == '/edgetpu/inference')
        assert spec.nominal == cap

    def test_no_stream_is_diagnostic_sourced(self, diag):
        # The USB camera node reports no rates on /diagnostics, so every
        # stream is counted.
        assert diag.DIAGNOSTIC_SOURCED == set()

    def test_every_nominal_sits_well_above_the_stall_line(self, diag):
        # A nominal near STALL_HZ would turn an ordinary dip into a FAIL. The
        # 0.5 Hz battery frame is under it by nature and has its own floor.
        for spec in diag.SENSOR_TOPICS + diag.ACTUATOR_TOPICS:
            if spec.topic == '/battery/voltage':
                continue
            assert spec.floor > 2 * diag.STALL_HZ, spec.topic

    def test_every_notebook_topic_is_covered(self, diag):
        covered = {s.topic for s in diag.SENSOR_TOPICS + diag.ACTUATOR_TOPICS}
        for topic in (
            '/camera/color',
            '/scan',
            '/imu/fused',
            '/mag',
            '/odom',
            '/encoder/speed',
            '/battery/voltage',
            '/rc/channels',
            '/edgetpu/inference',
            '/joy',
            '/motor',
            '/mux_out',
        ):
            assert topic in covered, f'{topic} lost from the check set'

    def test_no_upstream_only_topic_is_checked(self, diag):
        covered = {s.topic for s in diag.SENSOR_TOPICS + diag.ACTUATOR_TOPICS}
        for topic in ('/camera/depth', '/imu/realsense', '/imu/lsm9ds1', '/battery/current'):
            assert topic not in covered, topic

    def test_every_sampled_topic_has_a_reader(self, diag):
        sampled = {s.topic for s in diag.SENSOR_TOPICS + diag.ACTUATOR_TOPICS}
        sampled |= {s.topic for s in diag.AUTONOMY_TOPICS}
        assert set(diag.SAMPLE_READERS) == sampled


class TestRateChecks:
    def _ros(self, diag, counts, elapsed=2.0):
        return diag.RosResult(available=True, counts=counts, elapsed=elapsed, present=set(counts))

    def test_rate_above_floor_passes(self, diag):
        spec = diag.TopicSpec('/t', 'T', 100.0, 0.8)
        ros = self._ros(diag, {'/t': 200})  # 100 Hz over 2 s
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.OK

    def test_rate_below_floor_warns(self, diag):
        # Degraded but delivering: the car is still usable.
        spec = diag.TopicSpec('/t', 'T', 100.0, 0.8)
        ros = self._ros(diag, {'/t': 20})  # 10 Hz
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert check.status == diag.WARN
        assert check.detail == '10.0/100 Hz'

    def test_slow_inference_warns(self, diag):
        spec = next(s for s in diag.SENSOR_TOPICS if s.topic == '/edgetpu/inference')
        ros = self._ros(diag, {spec.topic: 43}, elapsed=5.0)  # 8.6 Hz
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.WARN

    @pytest.mark.parametrize('count', [0, 1, 9])
    def test_stalled_rate_fails(self, diag, count):
        # 0, 0.2 and 1.8 Hz over 5 s: stopped, not degraded.
        spec = diag.TopicSpec('/t', 'T', 200.0, 0.65)
        ros = self._ros(diag, {'/t': count}, elapsed=5.0)
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.FAIL

    def test_stall_line_is_inclusive_of_two_hz(self, diag):
        spec = diag.TopicSpec('/t', 'T', 30.0)
        assert diag.rate_status(2.0, spec) == diag.WARN
        assert diag.rate_status(1.99, spec) == diag.FAIL

    def test_rate_exactly_at_floor_passes(self, diag):
        spec = diag.TopicSpec('/t', 'T', 100.0, 0.8)
        ros = self._ros(diag, {'/t': 160})  # 80 Hz
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.OK

    def test_absent_topic_fails(self, diag):
        spec = diag.TopicSpec('/missing', 'M', 10.0)
        ros = diag.RosResult(available=True, counts={}, elapsed=2.0, present=set())
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert check.status == diag.FAIL
        assert 'not published' in check.detail

    def test_no_ros_graph_skips_rather_than_fails(self, diag):
        spec = diag.TopicSpec('/t', 'T', 10.0)
        ros = diag.RosResult(available=False, reason='no ROS graph visible')
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert check.status == diag.SKIP
        assert 'no ROS graph' in check.detail

    def test_detail_carries_no_annotation(self, diag):
        for spec in diag.SENSOR_TOPICS:
            ros = self._ros(diag, {spec.topic: 1000}, elapsed=5.0)
            ros.reported = {spec.topic: 200.0}
            detail = diag.rate_checks('sensors', [spec], ros)[0].detail
            assert ';' not in detail and detail.endswith(' Hz'), spec.topic

    def test_sample_is_carried_in_the_data_column(self, diag):
        spec = diag.TopicSpec('/encoder/speed', 'Encoder', 100.0)
        ros = self._ros(diag, {'/encoder/speed': 200})
        ros.values = {'/encoder/speed': SimpleNamespace(data=1.5)}
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert (check.status, check.data) == (diag.OK, '+1.50 m/s')

    def test_bad_sample_fails_a_healthy_rate(self, diag):
        spec = diag.TopicSpec('/battery/voltage', 'Battery voltage', 100.0)
        ros = self._ros(diag, {'/battery/voltage': 200})
        ros.values = {'/battery/voltage': SimpleNamespace(data=2.0)}
        assert diag.rate_checks('sensors', [spec], ros)[0].status == diag.FAIL

    def test_missing_sample_does_not_change_the_status(self, diag):
        spec = diag.TopicSpec('/encoder/speed', 'Encoder', 100.0)
        check = diag.rate_checks('sensors', [spec], self._ros(diag, {'/encoder/speed': 200}))[0]
        assert (check.status, check.data) == (diag.OK, 'no sample captured')

    def test_stopped_stack_fails_every_row(self, diag):
        # rclpy works and the graph is visible, but none of the car's topics.
        ros = diag.RosResult(available=True, present=set())
        checks = diag.rate_checks('sensors', diag.SENSOR_TOPICS, ros)
        assert all(c.status == diag.FAIL for c in checks)

    @pytest.mark.parametrize('count, status', [(3, 'OK'), (2, 'OK'), (1, 'FAIL'), (0, 'FAIL')])
    def test_battery_rate(self, diag, count, status):
        spec = next(s for s in diag.SENSOR_TOPICS if s.topic == '/battery/voltage')
        ros = self._ros(diag, {spec.topic: count}, elapsed=5.0)
        assert diag.rate_checks('sensors', [spec], ros)[0].status == status

    def test_diagnostic_sourced_topic_uses_the_reported_rate(self, diag, monkeypatch):
        # A topic whose publisher reports its own rate is never counted here,
        # so a zero count must not read as a dead stream.
        monkeypatch.setattr(diag, 'DIAGNOSTIC_SOURCED', {'/t'})
        spec = diag.TopicSpec('/t', 'T', 60.0)
        ros = diag.RosResult(
            available=True, counts={}, elapsed=5.0, present={'/t'}, reported={'/t': 59.0}
        )
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert check.status == diag.OK
        assert check.detail == '59.0/60 Hz'

    @pytest.mark.parametrize('hz, status', [(12.0, 'WARN'), (0.5, 'FAIL')])
    def test_diagnostic_sourced_topic_below_floor(self, diag, monkeypatch, hz, status):
        monkeypatch.setattr(diag, 'DIAGNOSTIC_SOURCED', {'/t'})
        spec = diag.TopicSpec('/t', 'T', 30.0)
        ros = diag.RosResult(
            available=True, counts={}, elapsed=5.0, present={'/t'}, reported={'/t': hz}
        )
        assert diag.rate_checks('sensors', [spec], ros)[0].status == status

    def test_missing_diagnostic_rate_warns_rather_than_reading_zero(self, diag, monkeypatch):
        # A DiagnosticArray that never arrived says nothing about the stream.
        monkeypatch.setattr(diag, 'DIAGNOSTIC_SOURCED', {'/t'})
        spec = diag.TopicSpec('/t', 'T', 60.0)
        ros = diag.RosResult(available=True, counts={}, elapsed=5.0, present={'/t'}, reported={})
        check = diag.rate_checks('sensors', [spec], ros)[0]
        assert check.status == diag.WARN
        assert '/diagnostics' in check.detail


class TestSampleWindow:
    def test_window_is_long_enough_for_the_board_stream(self, diag):
        # Shorter windows measure the ESP32 stream too noisily to judge and
        # hold too few battery frames.
        assert diag.DEFAULT_WINDOW >= 5.0


def _vec(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


_LEVEL = SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


def _imu(accel, q=_LEVEL, gyro_z=0.0):
    return SimpleNamespace(
        orientation=q, angular_velocity=_vec(0.0, 0.0, gyro_z), linear_acceleration=accel
    )


class TestSampleReaders:
    def test_gravity_magnitude_at_rest_passes(self, diag):
        status, data = diag.read_fused(_imu(_vec(0.0, 0.0, 9.81)))
        assert status == diag.OK
        assert data == 'yaw +0.0 deg, gyro z +0.00 rad/s, |a| 9.81'

    def test_implausible_gravity_fails(self, diag):
        status, data = diag.read_fused(_imu(_vec(0.0, 0.0, 0.2)))
        assert status == diag.FAIL
        assert 'expect 8 to 12' in data

    @pytest.mark.parametrize(
        'volts, status',
        [
            (12.4, 'OK'),
            (11.1, 'OK'),
            (11.0, 'WARN'),
            (10.79, 'FAIL'),
            (2.0, 'FAIL'),
            (14.0, 'FAIL'),
        ],
    )
    def test_pack_voltage(self, diag, volts, status):
        assert diag.read_pack_voltage(SimpleNamespace(data=volts))[0] == status

    def test_pack_voltage_details(self, diag):
        assert diag.read_pack_voltage(SimpleNamespace(data=11.4)) == (diag.OK, '11.40 V')
        assert diag.read_pack_voltage(SimpleNamespace(data=10.5))[1].endswith('charge the pack')
        assert diag.read_pack_voltage(SimpleNamespace(data=0.0))[1].endswith('expect 5 to 13')

    def _scan(self, ranges):
        return SimpleNamespace(ranges=ranges, range_min=0.05, range_max=25.0)

    def test_lidar_sample_count(self, diag):
        ranges = [1.0] * 720 + [float('inf')] * 720
        status, data = diag.read_scan(self._scan(ranges))
        assert status == diag.OK
        assert data == '1440 ranges, 720 returns, median 1.00 m'

    def test_unexpected_lidar_sample_count_warns(self, diag):
        # A different scan resolution: worth surfacing, not a hardware fault.
        status, data = diag.read_scan(self._scan([2.0] * 1080))
        assert status == diag.WARN
        assert 'expect 1440' in data

    def test_rc_channel_count(self, diag):
        status, data = diag.read_rc(SimpleNamespace(data=[0.0, 1.0, -1.0, 0.5] + [0.0] * 6))
        assert status == diag.OK
        assert data == '10 channels: +0.00 +1.00 -1.00 +0.50 ...'
        assert diag.read_rc(SimpleNamespace(data=[0.0] * 8))[0] == diag.FAIL

    def test_magnetometer_reads_in_microtesla(self, diag):
        msg = SimpleNamespace(magnetic_field=_vec(2e-5, 0.0, -4.5e-5))
        assert diag.read_mag(msg) == (diag.OK, 'field +20.00 +0.00 -45.00 uT')

    def test_fused_yaw(self, diag):
        # Quarter turn about z.
        q = SimpleNamespace(x=0.0, y=0.0, z=0.7071068, w=0.7071068)
        msg = _imu(_vec(-0.04, -0.06, 9.94), q, gyro_z=0.1)
        assert diag.read_fused(msg) == (diag.OK, 'yaw +90.0 deg, gyro z +0.10 rad/s, |a| 9.94')

    def test_odometry(self, diag):
        q = SimpleNamespace(x=0.0, y=0.0, z=0.7071068, w=0.7071068)
        pose = SimpleNamespace(position=_vec(1.0, -2.0, 0.0), orientation=q)
        msg = SimpleNamespace(
            pose=SimpleNamespace(pose=pose),
            twist=SimpleNamespace(twist=SimpleNamespace(linear=_vec(0.5, 0.0, 0.0))),
        )
        expected = 'x +1.00 m, y -2.00 m, heading +90 deg, v +0.50 m/s'
        assert diag.read_odom(msg) == (diag.OK, expected)

    def test_detections_report_the_top_score(self, diag):
        def det(*hyps):
            return SimpleNamespace(
                results=[
                    SimpleNamespace(hypothesis=SimpleNamespace(class_id=c, score=p))
                    for c, p in hyps
                ]
            )

        msg = SimpleNamespace(detections=[det(('cup', 0.41)), det(('person', 0.87))])
        assert diag.read_detections(msg) == (diag.OK, '2 detections, top person 0.87')
        assert diag.read_detections(SimpleNamespace(detections=[])) == (diag.OK, '0 detections')

    def test_joy(self, diag):
        msg = SimpleNamespace(axes=[0.0] * 8, buttons=[0, 1] + [0] * 9)
        assert diag.read_joy(msg) == (diag.OK, '8 axes, 11 buttons, 1 pressed')

    def test_drive_command(self, diag):
        msg = SimpleNamespace(drive=SimpleNamespace(speed=0.5, steering_angle=-0.1))
        assert diag.read_drive(msg) == (diag.OK, 'speed +0.50 m/s, steer -0.10 rad')

    def test_unexpected_payload_warns(self, diag):
        # A sim publishing another type under the same name must not crash the run.
        status, data = diag.read_sample('/imu/fused', SimpleNamespace(data=1.0))
        assert status == diag.WARN
        assert 'unreadable sample' in data


class TestRender:
    def test_groups_are_labeled_and_ordered(self, diag):
        checks = [
            diag.Check('network', 'eth0', diag.FAIL, 'two addresses'),
            diag.Check('devices', 'osrbot_base', diag.OK, '/dev/ttyACM0'),
        ]
        out = diag.render(checks, 1.0, 1)
        assert out.index('DEVICES') < out.index('NETWORK'), 'section order is fixed'

    def test_tally_and_exit_code_reported(self, diag):
        checks = [
            diag.Check('devices', 'a', diag.OK),
            diag.Check('devices', 'b', diag.WARN),
            diag.Check('devices', 'c', diag.FAIL),
            diag.Check('devices', 'd', diag.SKIP),
        ]
        out = diag.render(checks, 3.4, 1)
        assert '1 ok' in out and '1 warn' in out and '1 fail' in out and '1 skipped' in out
        assert 'RESULT FAIL' in out
        assert 'exit 1' in out

    def test_strict_note_only_in_strict_mode_on_failure(self, diag):
        warn = [diag.Check('devices', 'a', diag.WARN)]
        assert 'Strict' not in diag.render(warn, 1.0, 0)
        assert 'Strict' in diag.render(warn, 1.0, 1, strict=True)
        ok = [diag.Check('devices', 'a', diag.OK)]
        assert 'Strict' not in diag.render(ok, 1.0, 0, strict=True)

    def test_sample_column_is_tab_separated_and_aligned(self, diag):
        checks = [
            diag.Check('sensors', 'LakiBeam lidar', diag.OK, '30.0/30 Hz', '1440 ranges'),
            diag.Check('sensors', 'ESP32 IMU', diag.OK, '200.0/200 Hz', 'yaw'),
            diag.Check('system', 'throttling', diag.OK, 'none'),
        ]
        rows = [ln for ln in diag.render(checks, 1.0, 0).splitlines() if 'Hz' in ln]
        assert all(ln.count('\t') == 1 for ln in rows)
        assert len({ln.index('\t') for ln in rows}) == 1
        assert [ln.split('\t')[1] for ln in rows] == ['1440 ranges', 'yaw']
        # The widest rate still has a space before the tab.
        assert all(ln.split('\t')[0].endswith(' ') for ln in rows)

    def test_rows_without_a_sample_have_no_tab(self, diag):
        out = diag.render([diag.Check('system', 'disk', diag.OK, '32G free')], 1.0, 0)
        assert '\t' not in out

    def test_plain_output_has_no_escape_codes(self, diag):
        checks = [diag.Check('devices', s, s) for s in (diag.OK, diag.WARN, diag.FAIL)]
        assert '\033[' not in diag.render(checks, 1.0, 1)

    def test_color_marks_each_status(self, diag):
        checks = [diag.Check('devices', s, s) for s in (diag.OK, diag.WARN, diag.FAIL)]
        out = diag.render(checks, 1.0, 1, color=True)
        assert '\033[32m[ OK ]\033[0m' in out
        assert '\033[33m[WARN]\033[0m' in out
        assert '\033[31m[FAIL]\033[0m' in out
        assert 'RESULT \033[1;31mFAIL\033[0m' in out

    @pytest.mark.parametrize(
        'statuses, result',
        [
            (['OK'], 'OK'),
            (['OK', 'SKIP'], 'WARN'),
            (['OK', 'WARN'], 'WARN'),
            (['WARN', 'FAIL'], 'FAIL'),
        ],
    )
    def test_overall_result(self, diag, statuses, result):
        assert diag.overall([diag.Check('devices', s, s) for s in statuses]) == result

    def test_no_color_when_not_a_terminal(self, diag, monkeypatch):
        monkeypatch.setattr(diag.sys.stdout, 'isatty', lambda: False, raising=False)
        assert diag.use_color() is False

    def test_no_color_env_opts_out(self, diag, monkeypatch):
        monkeypatch.setattr(diag.sys.stdout, 'isatty', lambda: True, raising=False)
        monkeypatch.setenv('NO_COLOR', '1')
        assert diag.use_color() is False


class TestCommandLine:
    def test_unknown_section_errors(self):
        result = _run('--section', 'nonsense')
        assert result.returncode == 2
        assert 'unknown section' in result.stderr

    def test_quick_skips_the_ros_phase(self):
        # Host-only, so it must not wait on discovery.
        result = _run('--quick', timeout=30)
        assert result.returncode in (0, 1)
        assert 'SENSORS' not in result.stdout
        assert 'DEVICES' in result.stdout

    def test_quick_does_not_report_skipped_sensors(self):
        # Deselecting is not the same as failing to run: a section that was
        # never requested adds no rows, so it cannot drag the exit code down.
        result = _run('--quick', '--json', '--section', 'devices,sensors', timeout=30)
        groups = {c['group'] for c in json.loads(result.stdout)['checks']}
        assert groups == {'devices'}

    def test_section_narrows_output(self):
        result = _run('--quick', '--section', 'system', timeout=30)
        assert 'SYSTEM' in result.stdout
        assert 'DEVICES' not in result.stdout

    def test_json_is_valid_and_carries_the_exit_code(self):
        result = _run('--quick', '--json', timeout=30)
        payload = json.loads(result.stdout)
        assert 'checks' in payload
        assert payload['exit_code'] == result.returncode
        assert payload['result'] in ('OK', 'WARN', 'FAIL')
        assert payload['strict'] is False
        for check in payload['checks']:
            assert set(check) == {'group', 'name', 'status', 'detail', 'data'}
            assert check['status'] in ('OK', 'WARN', 'FAIL', 'SKIP')

    def test_json_groups_are_known_sections(self, diag):
        result = _run('--quick', '--json', timeout=30)
        payload = json.loads(result.stdout)
        groups = {c['group'] for c in payload['checks']}
        assert groups <= set(diag.SECTIONS)

    def test_help_documents_the_exit_rule(self):
        result = _run('--help')
        assert result.returncode == 0
        assert 'Exits 1 when a check fails' in result.stdout
        assert '--strict' in result.stdout


class TestExitCode:
    def test_all_ok_passes(self, diag):
        assert diag.exit_code_for([diag.Check('devices', 'a', diag.OK)]) == 0

    def test_nothing_checked_passes(self, diag):
        assert diag.exit_code_for([]) == 0

    @pytest.mark.parametrize('status', ['WARN', 'SKIP'])
    def test_warn_and_skip_leave_the_car_usable(self, diag, status):
        checks = [diag.Check('devices', 'a', diag.OK), diag.Check('sensors', 'b', status)]
        assert diag.exit_code_for(checks) == 0

    def test_fail_fails(self, diag):
        checks = [diag.Check('devices', 'a', diag.OK), diag.Check('sensors', 'b', diag.FAIL)]
        assert diag.exit_code_for(checks) == 1

    @pytest.mark.parametrize('status', ['WARN', 'FAIL', 'SKIP'])
    def test_strict_fails_anything_but_ok(self, diag, status):
        # A skipped sensor check on a car with teleop stopped is not a healthy car.
        checks = [diag.Check('devices', 'a', diag.OK), diag.Check('sensors', 'b', status)]
        assert diag.exit_code_for(checks, strict=True) == 1

    def test_strict_passes_all_ok(self, diag):
        assert diag.exit_code_for([diag.Check('devices', 'a', diag.OK)], strict=True) == 0


def _addr(iface, *addrs):
    return ''.join(
        f'2: {iface}    inet {a} brd 192.168.1.255 scope global {iface}\n' for a in addrs
    )


NETWORK_ENV = ('RACECAR_ETH_IFACE', 'RACECAR_WIFI_IFACE', 'RACECAR_AP_SSID')


@pytest.fixture
def network(diag, monkeypatch, tmp_path):
    """Run check_network against canned command output; returns {check name: Check}."""
    monkeypatch.setenv('HOME', str(tmp_path))
    for key in NETWORK_ENV:
        monkeypatch.delenv(key, raising=False)

    def run(eth='', nmcli='', target='', iface='nr_eth0'):
        table = {
            f'ip -4 -o addr show {iface} ': eth,
            'nmcli': nmcli,
            'systemctl get-default': target,
        }

        def fake(cmd, timeout=5.0):
            key = ' '.join(cmd)
            return next((out for prefix, out in table.items() if key.startswith(prefix)), '')

        monkeypatch.setattr(diag, '_run', fake)
        return {c.name: c for c in diag.check_network()}

    return run


def _networking_env(tmp_path, text):
    cfg = tmp_path / '.config' / 'racecar'
    cfg.mkdir(parents=True)
    (cfg / 'networking.env').write_text(text)


class TestCheckNetwork:
    def test_static_and_dhcp_addresses_are_ok(self, network, diag):
        # setup_networking.sh gives the router link both, by design.
        eth = _addr('nr_eth0', '192.168.10.100/24', '192.168.10.142/24')
        check = network(eth=eth)['nr_eth0 router']
        assert (check.status, check.detail) == (diag.OK, '192.168.10.100/24, 192.168.10.142/24')

    def test_no_router_address_warns(self, network, diag):
        assert network()['nr_eth0 router'].status == diag.WARN

    def test_interface_comes_from_networking_env(self, network, diag, tmp_path):
        _networking_env(tmp_path, 'RACECAR_ETH_IFACE="eth1"\n')
        rows = network(eth=_addr('eth1', '192.168.10.100/24'), iface='eth1')
        assert rows['eth1 router'].status == diag.OK

    def test_environment_wins_over_networking_env(self, network, diag, tmp_path, monkeypatch):
        _networking_env(tmp_path, 'RACECAR_ETH_IFACE="eth1"\n')
        monkeypatch.setenv('RACECAR_ETH_IFACE', 'eth2')
        assert 'eth2 router' in network(iface='eth2')

    def test_hotspot_states(self, network, diag):
        assert network()['hotspot'].status == diag.SKIP
        up = network(nmcli='neoracer-ap:wlP1p1s0\nracecar-eth0:nr_eth0\n')['hotspot']
        assert (up.status, up.detail) == (diag.OK, 'neoracer-1 up on wlP1p1s0')
        assert network(nmcli='HomeNet:wlP1p1s0\n')['hotspot'].status == diag.WARN

    def test_hotspot_reports_the_configured_ssid(self, network, diag, tmp_path):
        _networking_env(tmp_path, 'RACECAR_AP_SSID="neoracer-7"\nRACECAR_WIFI_IFACE=wlan0\n')
        detail = network(nmcli='neoracer-ap:wlan0\n')['hotspot'].detail
        assert detail == 'neoracer-7 up on wlan0'

    def test_no_upstream_rows(self, network):
        rows = network()
        assert not {'eth0 address', 'eth0 v6 default', 'wlan0 client', 'wlan1 AP'} & set(rows)

    def test_desktop_target(self, network, diag):
        assert network(target='graphical.target\n')['desktop'].detail == 'enabled'
        headless = network(target='multi-user.target\n')['desktop'].detail
        assert headless == 'headless (multi-user.target)'
        assert network()['desktop'].status == diag.SKIP


class TestActuatorChecks:
    @pytest.mark.parametrize('running, status', [(True, 'OK'), (False, 'WARN')])
    def test_led_matrix_row_follows_the_running_node(self, diag, monkeypatch, running, status):
        def fake(cmd, timeout=5.0):
            return '123\n' if running and cmd[-1] == diag.LED_MATRIX_PROCESS else ''

        monkeypatch.setattr(diag, '_run', fake)
        checks = {c.name: c for c in diag.actuator_checks(diag.RosResult())}
        assert checks['LED matrix'].status == status
        assert not {'Dot matrix', 'LED strip'} & set(checks)

    def test_led_matrix_pattern_is_the_watchdogs(self, diag):
        assert diag.LED_MATRIX_PROCESS.endswith('/lib/neoracer_ros2_driver/led_matrix')


class TestCpu:
    def _feed(self, diag, monkeypatch, samples, own=(0.0, 0.0)):
        stat = iter(samples)
        own_times = iter(own)
        monkeypatch.setattr(diag.sysinfo, 'read_cpu_times', lambda: next(stat))
        monkeypatch.setattr(diag, '_own_cpu_seconds', lambda: next(own_times))

    def test_busy_share_is_bounded_by_the_cores(self, diag, monkeypatch):
        # 4 cores over 1 s at 100 jiffies/s: 400 total, 300 busy.
        self._feed(diag, monkeypatch, [(1000, 4000), (1300, 4400)])
        assert diag.measure_cpu_busy(0) == pytest.approx(75.0)

    def test_own_time_is_excluded(self, diag, monkeypatch):
        # 0.5 s of this process's CPU is 50 of the 300 busy jiffies.
        tck = diag.os.sysconf('SC_CLK_TCK')
        self._feed(diag, monkeypatch, [(0, 0), (300, 400)], own=(0.0, 50 / tck))
        assert diag.measure_cpu_busy(0) == pytest.approx(62.5)

    def test_unreadable_stat(self, diag, monkeypatch):
        self._feed(diag, monkeypatch, [None, None])
        assert diag.measure_cpu_busy(0) is None

    @pytest.mark.parametrize('busy, status', [(77.0, 'OK'), (95.0, 'WARN'), (100.0, 'WARN')])
    def test_cpu_row(self, diag, monkeypatch, busy, status):
        monkeypatch.setattr(diag, 'measure_cpu_busy', lambda interval: busy)
        monkeypatch.setattr(diag.sysinfo, 'read_arm_clock', lambda: (1000, 2400))
        row = next(c for c in diag.check_system(0) if c.name == 'cpu')
        assert row.status == status
        assert row.detail == f'{busy:.0f}% busy, arm 1000 of 2400 MHz'


def _system(diag, monkeypatch, **readers):
    """Run check_system with canned sysinfo readers; returns {check name: Check}."""
    monkeypatch.setattr(diag, 'measure_cpu_busy', lambda interval: 10.0)
    for name, value in readers.items():
        monkeypatch.setattr(diag.sysinfo, name, lambda *a, value=value, **k: value)
    return {c.name: c for c in diag.check_system(0)}


class TestJetsonSystem:
    @pytest.mark.parametrize(
        'celsius, status',
        [(45.0, 'OK'), (69.9, 'OK'), (70.0, 'WARN'), (84.9, 'WARN'), (85.0, 'FAIL')],
    )
    def test_soc_temp_bands_match_the_dashboard(self, diag, monkeypatch, celsius, status):
        row = _system(diag, monkeypatch, read_soc_temp=celsius)['soc temp']
        assert (row.status, row.detail) == (status, f'{celsius:.1f} C')

    def test_over_current_events_warn(self, diag, monkeypatch):
        names = ['over-current throttling oc1 has occurred (3 events)']
        row = _system(diag, monkeypatch, read_throttled=(0x1, names))['throttling']
        assert row.status == diag.WARN and 'oc1' in row.detail
        assert _system(diag, monkeypatch, read_throttled=(0, []))['throttling'].status == 'OK'
        row = _system(diag, monkeypatch, read_throttled=(None, []))['throttling']
        assert (row.status, row.detail) == (diag.SKIP, 'no over-current counters')

    def test_no_raspberry_pi_rows(self, diag, monkeypatch):
        # The Jetson has no RTC cell voltage or PMIC under-voltage alarm.
        rows = _system(diag, monkeypatch)
        assert 'rtc cell' not in rows and 'under-voltage' not in rows
        for name in ('cpu', 'gpu', 'soc temp', 'throttling', 'fan', 'power mode', 'board power'):
            assert name in rows, name

    def test_power_mode(self, diag):
        assert diag.power_mode_check('25W', 1).detail == '25W (mode 1)'
        assert diag.power_mode_check('MAXN', 0).status == diag.OK
        assert diag.power_mode_check(None, None).status == diag.SKIP

    @pytest.mark.parametrize(
        'fan, status, detail',
        [
            ({'pwm': 41, 'percent': 16, 'rpm': 1641}, 'OK', '16% drive, 1641 rpm'),
            ({'pwm': 0, 'percent': 0, 'rpm': 0}, 'OK', '0% drive, 0 rpm'),
            ({'pwm': 41, 'percent': 16, 'rpm': None}, 'OK', '16% drive'),
            (
                {'pwm': 120, 'percent': 47, 'rpm': 0},
                'WARN',
                '47% drive, 0 rpm: driven but not turning',
            ),
            (None, 'SKIP', 'no fan hwmon'),
        ],
    )
    def test_fan(self, diag, fan, status, detail):
        row = diag.fan_check(fan)
        assert (row.status, row.detail) == (status, detail)

    def test_board_power(self, diag):
        rails = {
            'VDD_IN': {'mv': 5072, 'ma': 1320, 'mw': 6695},
            'VDD_CPU_GPU_CV': {'mv': 5064, 'ma': 392, 'mw': 1985},
            'VDD_SOC': {'mv': 5064, 'ma': 328, 'mw': 1663},
        }
        row = diag.board_power_check(rails)
        assert (row.status, row.detail) == (diag.OK, '6.7 W (5.07 V, 1.32 A)')
        assert row.data == 'VDD_CPU_GPU_CV 1985 mW, VDD_SOC 1663 mW'
        assert diag.board_power_check({}).status == diag.SKIP

    def test_gpu(self, diag):
        row = diag.gpu_check({'load_pct': 40.0, 'cur_mhz': 306.0, 'max_mhz': 624.75})
        assert (row.status, row.detail) == (diag.OK, '40% busy, 306 of 625 MHz')
        assert diag.gpu_check({'load_pct': None, 'cur_mhz': 306.0, 'max_mhz': None}).detail == (
            '306 MHz'
        )
        assert diag.gpu_check(None).status == diag.SKIP


class TestDataRoot:
    def test_writable_root_is_ok(self, diag, data_root_dir, monkeypatch):
        monkeypatch.setattr(
            diag.sysinfo, 'read_disk', lambda path: {'free': 100, 'percent_used': 20}
        )
        row = diag.check_data_root()
        assert (row.status, row.detail) == (diag.OK, f'{data_root_dir}, 100G free (20% used)')

    def test_unwritable_root_fails(self, diag, data_root_dir, monkeypatch):
        from neoracer_ros2_driver import data_root

        monkeypatch.setattr(data_root, 'session_problem', lambda: 'cannot be written')
        row = diag.check_data_root()
        assert row.status == diag.FAIL and row.detail.endswith('not writable')

    def test_missing_root_fails(self, diag, tmp_path, monkeypatch):
        from neoracer_ros2_driver import data_root

        monkeypatch.setattr(data_root, 'current', lambda: (tmp_path / 'gone', ''))
        row = diag.check_data_root()
        assert (row.status, row.detail) == (diag.FAIL, f'{tmp_path / "gone"} missing')

    def test_low_space_warns(self, diag, data_root_dir, monkeypatch):
        monkeypatch.setattr(
            diag.sysinfo, 'read_disk', lambda path: {'free': 4, 'percent_used': 60}
        )
        assert diag.check_data_root().status == diag.WARN


class TestCarProfile:
    def test_profile_row_names_the_car(self, diag, car_profile_dir):
        row = diag.check_profile()
        assert row.status == diag.OK
        assert row.detail.endswith(str(car_profile_dir))

    def test_unmigrated_files_warn(self, diag, monkeypatch):
        from neoracer_ros2_driver import car_profile

        monkeypatch.setattr(car_profile, 'unmigrated', lambda c, d: [Path('a.local.yaml')])
        row = diag.check_profile()
        assert row.status == diag.WARN and 'racecar profile migrate' in row.detail


@pytest.fixture
def devices(diag, monkeypatch, tmp_path):
    """Point the device links and the lidar interface at a temporary tree."""
    dev = tmp_path / 'dev'
    dev.mkdir()
    net = tmp_path / 'net'
    net.mkdir()
    links = tuple((str(dev / Path(link).name), required) for link, required in diag.DEVICE_LINKS)
    monkeypatch.setattr(diag, 'DEVICE_LINKS', links)
    monkeypatch.setattr(diag, 'NET_DIR', net)
    return SimpleNamespace(dev=dev, net=net)


def _usb0(net, carrier='1'):
    iface = net / 'usb0'
    iface.mkdir()
    if carrier is not None:
        (iface / 'carrier').write_text(f'{carrier}\n')


class TestDevices:
    def test_device_links_are_the_neoracers(self, diag):
        assert [p for p, _ in diag.DEVICE_LINKS] == [
            '/dev/osrbot_base',
            '/dev/osrbot_usb_cam',
            '/dev/osrbot_led_matrix',
        ]

    def test_links_match_the_udev_rules(self, diag):
        rules = (ROOT / 'scripts' / 'udev' / '99-racecar.rules').read_text()
        for link, _ in diag.DEVICE_LINKS:
            assert f'SYMLINK+="{Path(link).name}"' in rules, link

    def test_missing_board_and_camera_fail_and_matrix_warns(self, diag, devices, monkeypatch):
        monkeypatch.setattr(diag, '_run', lambda cmd, timeout=5.0: '')
        target = devices.dev / 'ttyACM0'
        target.write_text('')
        (devices.dev / 'osrbot_base').symlink_to(target)
        rows = {c.name: c for c in diag.check_devices()}
        assert (rows['osrbot_base'].status, rows['osrbot_base'].detail) == (
            diag.OK,
            str(target.resolve()),
        )
        assert rows['osrbot_usb_cam'].status == diag.FAIL
        assert 'racecar udev' in rows['osrbot_usb_cam'].detail
        assert rows['osrbot_led_matrix'].status == diag.WARN

    def test_no_upstream_device_rows(self, diag, devices, monkeypatch):
        monkeypatch.setattr(diag, '_run', lambda cmd, timeout=5.0: '')
        names = {c.name for c in diag.check_devices()}
        for gone in ('neo-pit-pcb', 'lidar', 'coral', 'RealSense D435i', 'gpiochip', 'i2c-1'):
            assert gone not in names, gone

    def test_lidar_link_unplugged_fails(self, diag, devices):
        row = diag.lidar_link_check(devices.net)
        assert (row.name, row.status) == ('usb0 lidar link', diag.FAIL)
        assert 'unplugged' in row.detail

    @pytest.mark.parametrize('carrier', ['0', None])
    def test_lidar_link_without_carrier_fails(self, diag, devices, carrier):
        # Reading carrier on a down interface raises, which counts as none.
        _usb0(devices.net, carrier)
        assert diag.lidar_link_check(devices.net).detail == 'no carrier'

    def test_lidar_link_without_the_host_address_warns(self, diag, devices, monkeypatch):
        _usb0(devices.net)
        monkeypatch.setattr(diag, '_run', lambda cmd, timeout=5.0: '')
        row = diag.lidar_link_check(devices.net)
        assert (row.status, row.detail) == (diag.WARN, 'no IPv4 address, expect 192.168.8.1')

    def test_lidar_link_up(self, diag, devices, monkeypatch):
        _usb0(devices.net)
        monkeypatch.setattr(diag, '_run', lambda cmd, timeout=5.0: _addr('usb0', '192.168.8.1/24'))
        row = diag.lidar_link_check(devices.net)
        assert (row.status, row.detail) == (diag.OK, 'carrier, 192.168.8.1/24')


def _wd_state(stale=False, **nodes):
    return {
        'stale': stale,
        'age_sec': 99.0 if stale else 1.0,
        'nodes': {
            n: {'state': s, 'restarts': 5, 'reason': 'topic stale'} for n, s in nodes.items()
        },
    }


class TestWatchdogCheck:
    def test_no_state_file(self, diag):
        assert diag.watchdog_check(None).status == diag.SKIP

    def test_stale_state_warns(self, diag):
        assert diag.watchdog_check(_wd_state(stale=True, controller='healthy')).status == diag.WARN

    def test_all_healthy(self, diag):
        row = diag.watchdog_check(_wd_state(controller='healthy', lidar='healthy'))
        assert (row.status, row.detail) == (diag.OK, '2 healthy')

    def test_gave_up_fails(self, diag):
        row = diag.watchdog_check(_wd_state(controller='healthy', lidar='gave_up'))
        assert row.status == diag.FAIL
        assert 'lidar' in row.detail

    @pytest.mark.parametrize('state', ['no_device', 'restarting'])
    def test_waiting_warns(self, diag, state):
        assert diag.watchdog_check(_wd_state(camera=state)).status == diag.WARN

    def test_disabled_and_starting_are_listed_but_ok(self, diag):
        state = _wd_state(controller='healthy', lidar='disabled', mux='starting')
        row = diag.watchdog_check(state)
        assert row.status == diag.OK
        assert 'disabled: lidar' in row.detail and 'starting: mux' in row.detail


def _status(**values):
    return SimpleNamespace(values=[SimpleNamespace(key=k, value=v) for k, v in values.items()])


class TestAutonomy:
    def _ros(self, diag, status, pose_count=100, elapsed=5.0):
        present = {'/autonomy/pose', '/autonomy/status'}
        return diag.RosResult(
            available=True,
            counts={'/autonomy/pose': pose_count, '/autonomy/status': 5},
            elapsed=elapsed,
            present=present,
            values={'/autonomy/status': status} if status is not None else {},
        )

    def test_stopped_unit_is_one_ok_row(self, diag):
        rows = diag.autonomy_checks('inactive', diag.RosResult(), sampled=False)
        assert [(c.name, c.status) for c in rows] == [('unit', 'OK')]
        assert rows[0].detail == 'stopped'

    def test_failed_unit_warns(self, diag):
        assert diag.autonomy_checks('failed', diag.RosResult(), sampled=False)[0].status == 'WARN'

    def test_quick_reports_the_unit_only(self, diag):
        rows = diag.autonomy_checks('active', diag.RosResult(), sampled=False)
        assert [c.name for c in rows] == ['unit']

    def test_no_autonomy_node_is_normal(self, diag):
        # The usual state: the unit is not running and nothing publishes the status.
        ros = diag.RosResult(available=True, present={'/scan'})
        rows = diag.autonomy_checks('inactive', ros, sampled=True)
        assert [(c.name, c.status, c.detail) for c in rows] == [('unit', 'OK', 'stopped')]

    def test_node_outside_the_unit_is_read(self, diag):
        status = _status(mode='mapping', map='hall', localized='true')
        rows = {c.name: c for c in diag.autonomy_checks('inactive', self._ros(diag, status), True)}
        assert rows['unit'].detail == 'stopped, node running'
        assert rows['mode'].detail == 'mapping on hall'
        assert rows['pose rate'].status == 'OK'

    def test_localization_rows(self, diag):
        status = _status(
            mode='localization', map='room', localizer='amcl', localized='true',
            scan_fit='0.812', pose_sd_m='0.120', pose_sd_deg='5.300', session='s1',
        )  # fmt: skip
        rows = {c.name: c for c in diag.autonomy_checks('active', self._ros(diag, status), True)}
        assert rows['mode'].detail == 'localization on room with amcl'
        assert rows['localized'].status == 'OK'
        assert rows['scan fit'].status == 'OK' and rows['scan fit'].detail.startswith('81%')
        assert rows['pose spread'].detail == '0.12 m, 5.3 deg'
        assert rows['pose rate'].status == 'OK'

    def test_not_localized_and_poor_fit_warn(self, diag):
        status = _status(mode='localization', map='room', localized='false', scan_fit='0.300')
        rows = {c.name: c for c in diag.autonomy_checks('active', self._ros(diag, status), True)}
        assert rows['localized'].status == 'WARN'
        assert '/autonomy/global_localization' in rows['localized'].detail
        assert rows['scan fit'].status == 'WARN'

    def test_mapping_reports_saves(self, diag):
        status = _status(mode='mapping', map='hall', localized='true', last_autosave='12:00:01')
        rows = {c.name: c for c in diag.autonomy_checks('active', self._ros(diag, status), True)}
        assert 'scan fit' not in rows
        assert rows['saves'].detail == 'saved not saved yet; autosave 12:00:01'

    def test_silent_node_fails(self, diag):
        rows = {c.name: c for c in diag.autonomy_checks('active', self._ros(diag, None), True)}
        assert rows['status'].status == 'FAIL'
        assert rows['status'].detail == 'no message on /autonomy/status'

    def test_pose_samples_read(self, diag):
        q = SimpleNamespace(x=0.0, y=0.0, z=0.7071068, w=0.7071068)
        pose = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=1.0, y=-2.0), orientation=q)))  # fmt: skip
        assert diag.read_autonomy_pose(pose) == ('OK', 'x +1.00 m, y -2.00 m, heading +90 deg')


class TestTurnedOff:
    def test_turned_off_subsystems_are_skipped(self, diag):
        checks = [
            diag.Check('sensors', 'YOLO inference', diag.FAIL, 'not published'),
            diag.Check('sensors', 'USB camera', diag.FAIL, 'not published'),
            diag.Check('sensors', 'LakiBeam lidar', diag.OK, '30.0 Hz'),
        ]
        out = diag.note_turned_off(checks, {'camera': True, 'detections': False})
        assert [c.status for c in out] == [diag.SKIP, diag.FAIL, diag.OK]
        out = diag.note_turned_off(checks, {'camera': False, 'detections': False})
        assert [c.status for c in out] == [diag.SKIP, diag.SKIP, diag.OK]
        assert out[0].detail.startswith('turned off')

    def test_rows_are_the_features_ids_and_sensor_labels(self, diag):
        assert set(diag.FEATURE_ROWS) == set(diag.features.SUBSYSTEMS)
        labels = {s.label for s in diag.SENSOR_TOPICS}
        for names in diag.FEATURE_ROWS.values():
            assert set(names) <= labels

    def test_boot_defaults_read_inference_as_off(self, diag, tmp_path):
        # Inference is off at boot (scripts/features.py), so its absence is not a fault.
        (tmp_path / 'launch_args').write_text('camera_enable:=true\ninference_enable:=false\n')
        running = diag.features.running(tmp_path)
        row = diag.Check('sensors', 'YOLO inference', diag.FAIL, 'not published')
        assert diag.note_turned_off([row], running)[0].status == diag.SKIP
