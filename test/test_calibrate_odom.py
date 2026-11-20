"""
Unit tests for calibrate_odom.py: turn resolution, scale limits, criteria, YAML merge.

Upstream's tests, plus the NeoRacer parts: controller_node, the /imu/fused
gyro, the source the speed scale is measured against, and the handset text.
"""

import math
from pathlib import Path
from types import SimpleNamespace

from conftest import load_script
import numpy as np
import pytest
import yaml

# calibrate_odom.py imports rclpy and the message packages, so these run on
# the car (off car the module is skipped; a stand-in ROS plugin runs them there).
pytest.importorskip('rclpy')

ROOT = Path(__file__).resolve().parent.parent


@pytest.fixture(scope='module')
def odom():
    return load_script('calibrate_odom')


@pytest.fixture(scope='module')
def common():
    return load_script('calibrate_common')


QUARTER_TURN_PER_S = math.radians(90)


def _spin(turns, scale=1.0, bias=0.0, rate=QUARTER_TURN_PER_S, dt=0.006):
    """Gyro samples for `turns` at `rate` (true), read with gain `scale` plus `bias`."""
    n = round(turns * 2 * math.pi / rate / dt)
    return [(i * dt, rate * scale + bias) for i in range(n + 1)]


class TestIntegrate:
    def test_constant_rate(self, odom):
        assert odom.integrate_rate([(0.0, 1.0), (2.0, 1.0)]) == pytest.approx(2.0)

    def test_bias_removed(self, odom):
        samples = _spin(1.0, bias=0.01)
        assert odom.integrate_rate(samples, bias=0.01) == pytest.approx(2 * math.pi, rel=1e-3)

    def test_too_few_samples(self, odom):
        assert odom.integrate_rate([(0.0, 1.0)]) == 0.0


class TestResolveTurns:
    def test_lidar_heading_plus_the_gyro_turn_count(self, odom):
        lidar = math.radians(3.0)  # modulo one turn
        true = odom.resolve_turns(lidar, math.radians(3 * 360 + 8.0))
        assert true == pytest.approx(math.radians(3 * 360 + 3.0))

    def test_turning_right(self, odom):
        true = odom.resolve_turns(math.radians(-2.0), math.radians(-725.0))
        assert true == pytest.approx(math.radians(-722.0))

    def test_gyro_a_half_turn_off_is_refused(self, odom):
        assert odom.resolve_turns(0.0, math.radians(720 + 174)) is None

    def test_scale_from_the_resolved_heading(self, odom):
        gyro = odom.integrate_rate(_spin(3.0, scale=1.04))
        true = odom.resolve_turns(math.radians(0.4), gyro)
        assert true / gyro == pytest.approx(1 / 1.04, rel=1e-3)


class TestOdomMotion:
    def test_heading_unwraps_through_full_turns(self, odom):
        yaws = [math.atan2(math.sin(a), math.cos(a)) for a in np.linspace(0, 4 * math.pi, 400)]
        poses = [[i * 0.02, 0.0, 0.0, y, 0.5] for i, y in enumerate(yaws)]
        assert odom.odom_motion(poses)[2] == pytest.approx(4 * math.pi)

    def test_translation_in_the_start_frame(self, odom):
        poses = [[0.0, 1.0, 1.0, math.pi / 2, 0.0], [1.0, 1.0, 3.0, math.pi / 2, 0.0]]
        x, y, th = odom.odom_motion(poses)
        assert (x, y, th) == (pytest.approx(2.0), pytest.approx(0.0, abs=1e-12), 0.0)


class TestScaleLimits:
    def test_degrees_for_radians_is_refused(self, odom):
        # A deg/s-vs-rad/s mix-up reads 57x; the fitted scale is far outside the band.
        assert not odom.scale_ok(1 / 57.3)
        assert not odom.scale_ok(57.3)

    def test_ordinary_corrections_pass(self, odom):
        assert odom.scale_ok(0.96) and odom.scale_ok(1.08)


class TestEnoughMotion:
    def test_a_spin_needs_most_of_the_circles_asked_for(self, odom):
        turn = 2 * math.pi
        assert odom.turned_enough(-2.97 * turn, odom.SPIN_TURNS)
        assert odom.turned_enough(2.0 * turn, odom.CHECK_TURNS)
        assert not odom.turned_enough(0.0, odom.CHECK_TURNS)
        assert not odom.turned_enough(1.4 * turn, odom.CHECK_TURNS)

    def test_a_check_drive_needs_the_same_length_as_a_calibration(self, odom):
        assert odom.MIN_DRIVE_M == 2.0


class TestPrompts:
    def test_the_stopped_prompt_waits_for_the_car(self, odom):
        # The prompt says to wait. An Enter typed after it appears still
        # answers it; a run with too little motion is then offered again.
        assert odom.STOPPED.startswith('Press Enter only when the car has stopped')


class TestCriteria:
    @pytest.mark.parametrize(
        'measured,status', [(721.0, 'PASS'), (735.0, 'WARN'), (760.0, 'FAIL'), (-722.0, 'PASS')]
    )
    def test_spin(self, odom, measured, status):
        lidar = 720.0 if measured > 0 else -720.0
        assert odom.spin_criterion(measured, lidar).status == status

    def test_spin_a_quarter_too_high_fails(self, odom):
        assert odom.spin_criterion(-893.9, -720.0).status == 'FAIL'

    @pytest.mark.parametrize('measured,status', [(3.03, 'PASS'), (3.12, 'WARN'), (3.3, 'FAIL')])
    def test_drive(self, odom, measured, status):
        assert odom.drive_criterion(measured, 3.0).status == status

    def test_poor_scan_match_fails(self, odom):
        sm = load_script('scan_match')
        good = sm.MatchResult((0.0, 0.0, 0.0), 0.01, 0.9, 10)
        moved = sm.MatchResult((0.0, 0.0, 0.0), 0.08, 0.3, 10)
        assert odom.match_criterion('m', good).status == 'PASS'
        assert odom.match_criterion('m', moved).status == 'FAIL'


class TestMergeWrite:
    def test_merge_keeps_the_other_step(self, common):
        common.write_local_yaml(
            'odom', 'controller_node', {'odometry.yaw_rate_scale': 0.98}, 't', merge=True
        )
        (path,) = common.write_local_yaml(
            'odom', 'controller_node', {'odometry.speed_scale': 1.05}, 't', merge=True
        )
        assert path.name == 'odom.yaml' and path.parent.name == 'params'
        params = yaml.safe_load(path.read_text())['controller_node']['ros__parameters']
        assert params == {'odometry.yaw_rate_scale': 0.98, 'odometry.speed_scale': 1.05}

    def test_without_merge_the_file_is_replaced(self, common):
        common.write_local_yaml('x', 'n', {'a': 1}, 't')
        (path,) = common.write_local_yaml('x', 'n', {'b': 2}, 't')
        assert yaml.safe_load(path.read_text())['n']['ros__parameters'] == {'b': 2}

    def test_keys_match_what_the_controller_declares(self):
        src = (ROOT / 'neoracer_ros2_driver' / 'controller.py').read_text()
        for key in ('odometry.yaw_rate_scale', 'odometry.speed_scale'):
            assert f"declare_float_param(self, '{key}'" in src


class TestInstructions:
    def test_circle_count_is_in_the_text(self):
        odom = load_script('calibrate_odom')
        assert 'circle (fully right or fully left) 2 times.' in odom.spin_lines(2)[0]

    def test_one_sentence_per_line(self):
        odom = load_script('calibrate_odom')
        lines = odom.spin_lines(3) + odom.DRIVE_LINES + odom.HOW_TO_DRIVE
        for line in lines + [odom.SPIN_PLACE, odom.DRIVE_PLACE]:
            assert line.count('. ') <= 1
            assert '; ' not in line


class TestScalesInUse:
    def test_matching_scales_are_info(self):
        odom = load_script('calibrate_odom')
        c = odom.scales_criterion((1.00462, 0.90338), (1.00462, 0.90338))
        assert c.status == 'INFO'

    def test_teleop_not_restarted_fails(self):
        odom = load_script('calibrate_odom')
        c = odom.scales_criterion((1.00462, 0.90338), (1.0, 1.0))
        assert c.status == 'FAIL' and 'restart teleop' in c.advice

    def test_unreachable_node_is_reported_not_judged(self):
        odom = load_script('calibrate_odom')
        c = odom.scales_criterion((1.0, 1.0), None)
        assert c.status == 'INFO' and 'not reachable' in c.limit


class TestBestMatch:
    def test_recovers_from_a_gyro_guess_35_degrees_off(self, odom):
        from test_scan_match import _points, _scan

        sm = load_script('scan_match')
        ref = _points(sm, _scan((0.0, 0.0, 0.0), noise=0.005, seed=1))
        cur = _points(sm, _scan((0.3, -0.2, math.radians(3.0)), noise=0.005, seed=2))
        result = odom.best_match(ref, cur, [(0.0, 0.0, math.radians(38.0))])
        assert math.degrees(result.pose[2]) == pytest.approx(3.0, abs=0.1)
        assert result.pose[:2] == (pytest.approx(0.3, abs=0.005), pytest.approx(-0.2, abs=0.005))


class TestConsistentMatch:
    def _result(self, x, y, inliers):
        sm = load_script('scan_match')
        return sm.MatchResult((x, y, 0.0), 0.01, inliers, 5)

    def test_far_from_the_odometry_is_inconsistent(self):
        odom = load_script('calibrate_odom')
        # The cold reverse run: odometry 2.89 m, the wrong match 0.29 m.
        assert not odom.consistent(self._result(-0.28, 0.06, 0.6), (-2.85, -0.4, 0.5))
        assert odom.consistent(self._result(-2.80, -0.42, 0.4), (-2.85, -0.4, 0.5))

    def test_short_moves_get_a_minimum_tolerance(self):
        odom = load_script('calibrate_odom')
        assert odom.consistent(self._result(0.3, 0.1, 0.9), (0.0, 0.0, 6.28))

    def test_best_match_prefers_the_consistent_alignment(self, monkeypatch):
        odom = load_script('calibrate_odom')
        wrong, right = self._result(0.0, 0.0, 0.62), self._result(-2.8, -0.4, 0.55)
        calls = iter([right] + [wrong] * 200)
        monkeypatch.setattr(odom, 'match_scans', lambda a, b, g: next(calls))
        got = odom.best_match(None, None, [(-2.85, -0.4, 0.5), (0.0, 0.0, 0.5)])
        assert got is right


class TestNeoRacer:
    def test_the_controller_and_the_gyro_its_integrator_reads(self, odom):
        assert odom.NODE == 'controller_node'
        assert odom.GYRO_TOPIC == '/imu/fused'
        assert odom.CAL_NAME == 'odom'

    def test_a_person_drives_in_manual(self, odom):
        assert 'mode switch UP (manual)' in odom.HOW_TO_DRIVE[0]
        assert odom.HOW_TO_DRIVE[1] == 'Keep the speed switch UP (slow).'
        cal = load_script('calibrate')
        assert 'mode switch UP (manual)' in cal.ODOM_SETUP

    def test_the_drive_length_fits_the_setup_text(self, odom):
        cal = load_script('calibrate')
        assert odom.MIN_DRIVE_M == 2.0
        assert '3 to 5 meters' in odom.DRIVE_PLACE and '3 to 5 meters' in cal.ODOM_SETUP

    def test_the_speed_scale_moves_by_lidar_over_the_integrated_estimate(self, odom):
        # The integrated estimate carries the current scale on either source,
        # so the new scale is the current one times this ratio.
        assert odom.speed_ratio(3.0, 3.042) == pytest.approx(3.0 / 3.042)
        assert odom.speed_ratio(3.0, 0.0) == math.inf

    def test_the_note_says_where_the_scales_act(self, odom):
        assert 'applies the scales' in odom.source_note('integrated')
        firmware = odom.source_note('firmware')
        assert 'firmware pose' in firmware and 'odometry.source: integrated' in firmware

    def test_a_dynamically_typed_parameter_reads_as_a_number(self, odom):
        types = odom.ParameterType
        as_int = SimpleNamespace(type=types.PARAMETER_INTEGER, integer_value=1)
        as_float = SimpleNamespace(type=types.PARAMETER_DOUBLE, double_value=0.98)
        as_text = SimpleNamespace(type=types.PARAMETER_STRING, string_value='1')
        assert odom.parameter_number(as_int) == 1.0
        assert odom.parameter_number(as_float) == 0.98
        assert odom.parameter_number(as_text) is None

    def test_odometry_yaml_is_for_the_controller(self):
        shipped = yaml.safe_load((ROOT / 'config' / 'odom.yaml').read_text())
        params = shipped['controller_node']['ros__parameters']
        keys = {'odometry.yaw_rate_scale', 'odometry.speed_scale', 'odometry.source'}
        assert keys <= set(params)


def _value(types, kind, **fields):
    return SimpleNamespace(type=getattr(types, f'PARAMETER_{kind}'), **fields)


class _Future:
    def __init__(self, values):
        self._values = values

    def done(self):
        return True

    def exception(self):
        return None

    def result(self):
        return SimpleNamespace(values=self._values)


def _node_answering(odom, values, monkeypatch):
    """Return an OdomCalibrator whose controller_node answers GetParameters with `values`."""
    node = odom.OdomCalibrator.__new__(odom.OdomCalibrator)
    asked = []

    class Client:
        def wait_for_service(self, timeout_sec):
            return values is not None

        def call_async(self, request):
            asked.append(list(request.names))
            return _Future(values)

    monkeypatch.setattr(node, 'create_client', lambda *_a: Client(), raising=False)
    monkeypatch.setattr(odom.GetParameters, 'Request', SimpleNamespace, raising=False)
    return node, asked


class TestSources:
    """The default firmware source: what is reported, what is advised, what is read live."""

    def test_check_advice_follows_the_source(self, odom):
        for criterion in (odom.spin_criterion, odom.drive_criterion):
            assert 'racecar calibrate odom' in criterion(1.05, 1.0).advice
            assert 'racecar calibrate odom' in criterion(1.05, 1.0, 'integrated').advice
            advice = criterion(1.05, 1.0, 'firmware').advice
            assert advice == odom.FIRMWARE_ADVICE
            assert 'odometry.source: integrated' in advice

    def test_unused_scales_do_not_fail_the_firmware_check(self, odom):
        firmware = odom.scales_criterion((0.98, 0.99), (1.0, 1.0), 'firmware')
        assert firmware.status == 'INFO' and 'firmware' in firmware.limit
        assert odom.scales_criterion((0.98, 0.99), (1.0, 1.0)).status == 'FAIL'

    @pytest.mark.parametrize(
        'source,live_source,live_scales,with_scales,stale',
        [
            ('firmware', 'firmware', (1.0, 1.0), True, False),
            ('integrated', 'firmware', (1.0, 1.0), True, True),
            ('firmware', 'integrated', (0.98, 0.99), False, True),
            ('integrated', 'integrated', (1.0, 1.0), True, True),
            ('integrated', 'integrated', (1.0, 1.0), False, False),
            ('integrated', 'integrated', (0.98, 0.99), True, False),
            ('firmware', 'firmware', (0.5, 0.5), True, False),
        ],
    )
    def test_the_files_must_match_the_running_controller(
        self, odom, source, live_source, live_scales, with_scales, stale
    ):
        live = {
            'odometry.source': live_source,
            'odometry.yaw_rate_scale': live_scales[0],
            'odometry.speed_scale': live_scales[1],
        }
        text = odom.stale_files(live, source, (0.98, 0.99), with_scales)
        assert bool(text) is stale
        if live_source != source:
            assert f'runs odometry.source {live_source}' in text

    def test_the_live_read_takes_the_source_and_integer_scales(self, odom, monkeypatch):
        types = odom.ParameterType
        values = [
            _value(types, 'STRING', string_value='integrated'),
            _value(types, 'INTEGER', integer_value=1),
            _value(types, 'DOUBLE', double_value=0.98),
        ]
        node, asked = _node_answering(odom, values, monkeypatch)
        assert node.live_params() == {
            'odometry.source': 'integrated',
            'odometry.yaw_rate_scale': 1.0,
            'odometry.speed_scale': 0.98,
        }
        assert node.live_scales() == (1.0, 0.98)
        assert asked[0] == list(odom.LIVE_KEYS)

    def test_no_answer_or_a_text_scale_reads_as_no_answer(self, odom, monkeypatch):
        node, _ = _node_answering(odom, None, monkeypatch)
        assert node.live_params() is None and node.live_scales() is None
        types = odom.ParameterType
        values = [
            _value(types, 'STRING', string_value='firmware'),
            _value(types, 'STRING', string_value='1.0'),
            _value(types, 'DOUBLE', double_value=1.0),
        ]
        node, _ = _node_answering(odom, values, monkeypatch)
        assert node.live_params() is None

    def test_both_poses_are_reported_against_the_lidar(self, odom):
        line = odom.pose_line('firmware', 'distance', 3.042, 3.0, odom.DRIVE_PASS)
        assert line.startswith('Firmware distance: +1.40% against the lidar')
        assert 'within 2%' in line
        line = odom.pose_line('integrated', 'heading', 1068.0, 1080.0, odom.SPIN_PASS)
        assert line.startswith('Integrated heading: -1.11% against the lidar')
        assert 'within 1%' in line
        assert 'odometry.source: integrated' in odom.UNUSED_ON_FIRMWARE

    @pytest.mark.parametrize('kind', ['spin', 'drive'])
    def test_the_source_not_in_use_is_listed_without_a_verdict(self, odom, kind):
        criterion = odom.spin_criterion if kind == 'spin' else odom.drive_criterion
        judged = criterion(1.05, 1.0, 'firmware', True)
        listed = criterion(1.05, 1.0, 'integrated', False)
        assert judged.name == f'{kind} firmware' and judged.status == 'FAIL'
        assert listed.name == f'{kind} integrated' and listed.status == 'INFO'
        # The same limits, so the two rows read side by side; no advice without a verdict.
        assert listed.limit == criterion(1.05, 1.0, 'integrated', True).limit
        assert listed.advice == ''

    def test_both_accounts_come_firmware_first(self, odom):
        sm = load_script('scan_match')
        match = sm.MatchResult((0.0, 0.0, 0.0), 0.01, 0.9, 10)
        run = odom.Run((1, 0, 0), match, (1, 0, 0), 0.0, (1.01, 0, 0), (1.02, 0, 0))
        assert run.poses() == (('firmware', (1.01, 0, 0)), ('integrated', (1.02, 0, 0)))


def _frames(duration, speed, rate, dt=0.005, t0=100.0):
    """/odom [t, x, y, yaw, v] and gyro (t, rate) samples at the `s` frame rate."""
    n = round(duration / dt)
    odom = [[t0 + i * dt, 0.0, 0.0, 0.0, speed] for i in range(n + 1)]
    gyro = [(t0 + i * dt, rate) for i in range(n + 1)]
    return odom, gyro


class TestIntegratedReplay:
    """The integrated estimate on the firmware source: the driver's integrator, replayed."""

    def test_a_straight_drive_carries_the_speed_scale(self, odom):
        rows, gyro = _frames(2.0, 1.0, 0.0)
        cfg = odom.Integration(speed_scale=0.98)
        x, y, heading = odom.integrated_motion(rows, gyro, 0.0, cfg)
        assert x == pytest.approx(2.0 * 0.98, rel=1e-3)
        assert y == pytest.approx(0.0) and heading == pytest.approx(0.0)

    def test_three_turns_carry_the_yaw_scale_and_unwrap(self, odom):
        rate = math.radians(90)
        rows, gyro = _frames(12.0, 0.5, rate)
        cfg = odom.Integration(yaw_scale=1.02)
        heading = odom.integrated_motion(rows, gyro, 0.0, cfg)[2]
        assert heading == pytest.approx(3 * 2 * math.pi * 1.02, rel=1e-3)

    def test_the_rest_bias_is_removed(self, odom):
        bias = math.radians(0.5)
        rows, gyro = _frames(3.0, 1.0, bias)
        heading = odom.integrated_motion(rows, gyro, bias, odom.Integration())[2]
        assert heading == pytest.approx(0.0, abs=1e-9)

    def test_it_matches_the_controller_stepping_the_integrator(self, odom):
        # The controller steps once per `s` frame with vx x speed_scale and the
        # gyro x sign x yaw_rate_scale; the replay does the same from /odom.
        from neoracer_ros2_driver.odometry import OdomIntegrator

        rows, gyro = _frames(4.0, 0.8, math.radians(40))
        cfg = odom.Integration(speed_scale=0.99, yaw_scale=1.01, turn_gain=0.5)
        controller = OdomIntegrator(
            turn_gain=0.5, wheelbase=cfg.wheelbase_m, max_steer=math.radians(30.0)
        )
        for (t, *_rest, v), (_t, rate) in zip(rows, gyro, strict=True):
            controller.step(t, v * 0.99, rate * 1.01)
        x, y, heading = odom.integrated_motion(rows, gyro, 0.0, cfg)
        assert x == pytest.approx(controller.state.x, abs=1e-9)
        assert y == pytest.approx(controller.state.y, abs=1e-9)
        assert math.atan2(math.sin(heading), math.cos(heading)) == pytest.approx(
            controller.state.theta, abs=1e-9
        )

    def test_no_samples_give_no_estimate(self, odom):
        assert odom.integrated_motion([], [], 0.0, odom.Integration()) is None
