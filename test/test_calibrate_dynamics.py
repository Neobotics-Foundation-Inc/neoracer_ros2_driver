"""Unit tests for calibrate_dynamics.py: circle fit, Ackermann angles, timing, scan chaining."""

import math

from conftest import load_script
import numpy as np
import pytest

# calibrate_dynamics.py imports calibrate_common, which imports rclpy, so these
# run on the car (off car the module is skipped, as test_calibrate.py is).
pytest.importorskip('rclpy')


@pytest.fixture(scope='module')
def md():
    return load_script('calibrate_dynamics')


class TestGeometry:
    def test_circle_fit(self, md):
        a = np.linspace(0, 5, 60)
        xy = np.column_stack((1.0 + 0.7 * np.cos(a), -2.0 + 0.7 * np.sin(a)))
        cx, cy, r, resid = md.fit_circle(xy)
        assert (cx, cy, r) == (pytest.approx(1.0), pytest.approx(-2.0), pytest.approx(0.7))
        assert resid < 1e-9

    def test_ackermann_inner_turns_more(self, md):
        s = md.ackermann(0.6)
        assert s.inner_deg > s.bicycle_deg > s.outer_deg
        assert s.bicycle_deg == pytest.approx(math.degrees(math.atan(md.WHEELBASE_M / 0.6)))

    def test_outer_front_track_diameter(self, md):
        s = md.ackermann(0.6, wheelbase=0.23, track=0.178)
        assert s.outer_front_track_diameter_m == pytest.approx(2 * math.hypot(0.689, 0.23))

    def test_neoracer_wheelbase_and_track(self, md):
        # config/odom.yaml odometry.wheelbase_m and the neoracer-mujoco spec (CAD).
        assert md.WHEELBASE_M == pytest.approx(0.28764)
        assert md.TRACK_M == pytest.approx(0.224)


class TestTiming:
    def test_transition_of_a_ramp(self, md):
        t = np.arange(0.0, 2.0, 0.005)
        kappa = np.clip((t - 1.0) / 0.1, 0.0, 1.0) * 3.0 - 1.5  # -1.5 to +1.5 over 0.1 s
        rise, total = md.transition_times(t, kappa, 1.0, -1.5, 1.5)
        assert rise == pytest.approx(0.08, abs=0.006)
        assert total == pytest.approx(0.095, abs=0.006)

    def test_straight_metrics(self, md):
        t = np.arange(0.0, 8.0, 0.01)
        v = np.where(t < 2.0, 0.0, np.minimum((t - 2.0) / 1.0, 1.0) * 0.75)
        v = np.where(t >= 5.0, np.maximum(0.75 - (t - 5.0) * 1.5, 0.0), v)
        s = np.concatenate(([0.0], np.cumsum(v[:-1] * 0.01)))
        r = md.straight_metrics(t, v, s, 2.0, 5.0)
        assert r.steady_mps == pytest.approx(0.75, rel=1e-3)
        assert r.rise_90_s == pytest.approx(0.9, abs=0.02)
        assert r.stop_s == pytest.approx(0.5, abs=0.02)
        assert r.stop_m == pytest.approx(0.1875, abs=0.005)


class TestChain:
    def test_chain_recovers_a_circle(self, md):
        from test_scan_match import _points, _scan

        sm = load_script('scan_match')
        laser = (0.24, 0.0, math.pi)
        radius, n = 0.7, 40
        base = [
            (1.0 + radius * math.sin(a), 0.5 - radius * math.cos(a) + radius, a)
            for a in np.linspace(0.0, 2.2 * math.pi, n)
        ]
        scans, odom_t, odom = [], [], []
        for k, b in enumerate(base):
            lp = sm.compose(b, laser)
            pts = _points(sm, _scan(lp, noise=0.003, seed=k))
            scans.append((float(k), pts, np.full(len(pts), float(k))))
            odom_t.append(float(k))
            odom.append(sm.compose(sm.invert(base[0]), b))
        odom = np.array(odom)
        odom[:, 2] = np.unwrap(odom[:, 2])
        times, poses, rms = md.chain_scans(scans, np.array(odom_t), odom, laser)
        _, _, r, _ = md.fit_circle(poses[:, :2])
        assert r == pytest.approx(radius, abs=0.005)
        assert rms < 0.01


class TestKeys:
    def test_speed_tags(self, md):
        assert [md.speed_tag(v) for v in (0.62, 0.75, 1.0)] == ['025', '05', '10']

    def test_a_speed_that_is_no_run_has_no_tag(self, md):
        with pytest.raises(ValueError, match='not one of the straight runs'):
            md.speed_tag(0.3)

    def test_the_runs_carry_upstreams_names(self, md):
        assert list(md.RUNS) == [
            'straight025',
            'straight05',
            'straight10',
            'circle_right',
            'circle_left',
        ]
        for tag, mps in md.STRAIGHT_MPS.items():
            assert md.RUNS[f'straight{tag}'][1][0] == mps

    def test_calibrate_lists_every_key_the_tool_writes(self, md):
        cal = load_script('calibrate')
        (step,) = [s for s in cal.STEPS if s.key == 'dynamics']
        written = {
            f'{n}_{md.speed_tag(r[1][0])}'
            for r in [md.RUNS[k] for k in ('straight025', 'straight05', 'straight10')]
            for n in ('steady_speed_mps', 'rise_90_s', 'stop_distance_mm', 'stop_time_s')
        }
        assert written <= set(step.keys)

    def test_motor_speed_is_read_back_in_mps(self, md, monkeypatch):
        sd = load_script('drive_speed')
        monkeypatch.setattr(md, 'drive_limits', lambda: sd.DriveLimits(6.0, 1.0, 1.0))
        assert md.speed_tag(md.motor_mps(0.62 / 6.0)) == '025'
        assert md.motor_mps(1.0 / 6.0) == pytest.approx(1.0)


class TestArming:
    def test_a_switch_left_down_does_not_arm(self, md):
        arming = md.Arming()
        assert [arming.update(True) for _ in range(5)] == [False] * 5

    def test_up_then_down_arms(self, md):
        arming = md.Arming()
        assert [arming.update(d) for d in (True, False, False, True)] == [
            False,
            False,
            False,
            True,
        ]
        assert arming.update(False)  # stays armed; the drive loop watches the switch

    def test_reset_needs_a_new_flip(self, md):
        arming = md.Arming()
        arming.update(False)
        arming.update(True)
        arming.reset()
        assert not arming.update(True)


class Log:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(m)

    error = info


class TestDriveGate:
    def _driver(self, md, monkeypatch, tmp_path, buttons=(False, True), before=None):
        """
        Return (driver, held, sent) with ROS and the recorder replaced.

        The 2 s rest reads `before` as button 5 (None: no /joy message). Each
        0.1 s hold of the wait for the flip reads the next of `buttons`.
        `held` records (speed, steering, need_rb) for every hold and `sent`
        every single command.
        """

        class Proc:
            def send_signal(self, sig):
                pass

            def wait(self, timeout=None):
                return 0

        monkeypatch.setattr(md.subprocess, 'Popen', lambda *a, **k: Proc())
        monkeypatch.setattr(md, 'load_config', lambda: {})
        monkeypatch.setattr(md, 'resolve_log_root', lambda d: tmp_path)
        sd = load_script('drive_speed')
        monkeypatch.setattr(md, 'drive_limits', lambda: sd.DriveLimits(6.0, 1.0, 1.0))
        clock = [0.0]
        monkeypatch.setattr(md.time, 'monotonic', lambda: clock[0])

        d = md.Driver.__new__(md.Driver)
        d.log = Log()
        d.rb = False
        d.arming = md.Arming()
        d.timed_out = False
        held = []
        sent = []
        script = list(buttons)

        def hold(seconds, speed, steer, need_rb):
            clock[0] += seconds
            if seconds == 2.0 and before is not None:
                d._button(before)
            elif seconds < 1.0 and script:
                d._button(script.pop(0))
            held.append((speed, steer, need_rb))
            return d.rb or not need_rb

        d._hold = hold
        d._send = lambda speed, steer: sent.append((speed, steer))
        return d, held, sent

    def test_flipping_up_after_the_drive_keeps_the_run(self, md, monkeypatch, tmp_path):
        d, held, _ = self._driver(md, monkeypatch, tmp_path)
        real_hold = d._hold

        def hold(seconds, speed, steer, need_rb):
            ok = real_hold(seconds, speed, steer, need_rb)
            if speed == 0.0 and any(s != 0.0 for s, _, _ in held[:-1]):
                d._button(False)
            return ok

        d._hold = hold
        assert d.drive('straight05') is not None
        assert 'You can flip the mode switch UP now.' in d.log.lines
        assert all(need for speed, _, need in held if speed != 0.0)

    def test_flipping_up_while_driving_stops_the_run(self, md, monkeypatch, tmp_path):
        d, held, sent = self._driver(md, monkeypatch, tmp_path)
        real_hold = d._hold

        def hold(seconds, speed, steer, need_rb):
            if speed != 0.0:
                d._button(False)
            return real_hold(seconds, speed, steer, need_rb)

        d._hold = hold
        assert d.drive('circle_right') is None
        assert not d.timed_out
        assert sent[-1] == (0.0, 0.0)
        assert any(m.startswith('Stopped: the mode switch was flipped UP') for m in d.log.lines)

    def test_flipping_up_in_the_rest_before_the_first_move_stops_the_run(
        self, md, monkeypatch, tmp_path
    ):
        d, held, _ = self._driver(md, monkeypatch, tmp_path)
        real_hold = d._hold

        def hold(seconds, speed, steer, need_rb):
            if need_rb and seconds == 2.0:  # the plan's rest after the countdown
                d._button(False)
            return real_hold(seconds, speed, steer, need_rb)

        d._hold = hold
        assert d.drive('straight05') is None
        assert all(speed == 0.0 for speed, _, _ in held)

    def test_the_operator_is_told_when_the_car_starts(self, md, monkeypatch, tmp_path):
        # The 3 s countdown, then the plan's 2 s at rest before the first move.
        d, _, _ = self._driver(md, monkeypatch, tmp_path)
        d.drive('straight05')
        assert (
            'Flip the mode switch DOWN (autonomy). The car starts driving 5 seconds after '
            'you flip it.'
        ) in d.log.lines

    def test_runs_go_out_as_drive_values_for_their_mps(self, md, monkeypatch, tmp_path):
        d, held, _ = self._driver(md, monkeypatch, tmp_path)
        assert d.drive('straight10') is not None
        moving = {round(speed, 4) for speed, _, _ in held if speed != 0.0}
        assert moving == {round(1.0 / 6.0, 4)}

    def test_a_switch_left_down_must_go_up_first(self, md, monkeypatch, tmp_path):
        d, held, _ = self._driver(
            md, monkeypatch, tmp_path, buttons=(True, False, True), before=True
        )
        assert d.drive('straight025') is not None
        assert 'The mode switch is DOWN. Flip it UP first.' in d.log.lines
        first_move = next(i for i, (speed, _, _) in enumerate(held) if speed != 0.0)
        # The rest, three waits (DOWN, UP, DOWN), the countdown, the first still segment.
        assert first_move == 6

    def test_no_flip_times_out_without_driving(self, md, monkeypatch, tmp_path):
        # Before the RC calibration update button 5 never sets: the switch reads UP.
        d, held, sent = self._driver(md, monkeypatch, tmp_path, buttons=[False] * 2000)
        assert d.drive('straight05') is None
        assert d.timed_out
        assert all(speed == 0.0 for speed, _, _ in held)
        assert sent == [(0.0, 0.0)]
        assert any('did not go UP and then DOWN within 60 seconds' in m for m in d.log.lines)

    def test_a_switch_held_down_throughout_times_out(self, md, monkeypatch, tmp_path):
        d, held, _ = self._driver(md, monkeypatch, tmp_path, buttons=[True] * 2000, before=True)
        assert d.drive('straight10') is None
        assert d.timed_out
        assert all(speed == 0.0 for speed, _, _ in held)

    def test_no_joy_at_all_times_out(self, md, monkeypatch, tmp_path):
        d, held, _ = self._driver(md, monkeypatch, tmp_path, buttons=())
        assert d.drive('circle_left') is None
        assert d.timed_out
        assert all(speed == 0.0 for speed, _, _ in held)

    def test_no_rb_wording_reaches_the_operator(self, md, monkeypatch, tmp_path):
        d, _, _ = self._driver(md, monkeypatch, tmp_path)
        d.drive('straight05')
        assert not any('RB' in m for m in d.log.lines)


class FakeDriver:
    timed_out = False
    live = {'throttle_node': {}, 'controller_node': {}}
    bag = None

    def __init__(self):
        self.log = Log()
        FakeDriver.made = self

    def drive(self, run, segments=None):
        self.timed_out = self.bag is None
        return self.bag

    def live_limits(self):
        return self.live

    def close(self):
        pass


def _offer_once(n, show, run):
    # run_sequence keeps a run's result only when it is not None.
    got = run(0)
    return {} if got is None else {0: got}


class TestSession:
    @pytest.fixture
    def plans(self, md):
        return {run: [] for run in md.RUNS}

    def test_a_timeout_ends_the_session_with_nothing_written(self, md, monkeypatch, plans):
        monkeypatch.setattr(md, 'Driver', FakeDriver)
        monkeypatch.setattr(md, 'file_limits', lambda: {})
        monkeypatch.setattr(md, 'run_sequence', _offer_once)
        written = []
        monkeypatch.setattr(md, 'write_values', lambda *a: written.append(a) or 0)
        assert md.run_session(plans) == 1
        assert written == []

    def test_live_limits_that_differ_stop_before_any_run(self, md, monkeypatch, plans):
        class Driver(FakeDriver):
            live = {'controller_node': {'max_speed_mps': 3.0}}

        monkeypatch.setattr(md, 'Driver', Driver)
        monkeypatch.setattr(md, 'file_limits', lambda: {'controller_node': {'max_speed_mps': 6.0}})
        monkeypatch.setattr(md, 'run_sequence', lambda *a: pytest.fail('a run was offered'))
        assert md.run_session(plans) == 1
        assert (
            'controller_node runs max_speed_mps 3; the files give 6.' in FakeDriver.made.log.lines
        )
        assert md.LIMITS_ADVICE in FakeDriver.made.log.lines

    def test_a_run_that_cannot_be_analyzed_is_offered_again(
        self, md, monkeypatch, plans, tmp_path
    ):
        class Driver(FakeDriver):
            bag = tmp_path / 'run'

        def analyze(bag):
            raise ValueError('0.50 m/s is not one of the straight runs (0.62, 0.75, 1 m/s)')

        monkeypatch.setattr(md, 'Driver', Driver)
        monkeypatch.setattr(md, 'file_limits', lambda: {})
        monkeypatch.setattr(md, 'analyze', analyze)
        monkeypatch.setattr(md, 'run_sequence', _offer_once)
        assert md.run_session(plans) == 0
        assert any('could not be analyzed: 0.50 m/s' in m for m in FakeDriver.made.log.lines)

    def test_limits_that_cannot_give_a_speed_stop_before_any_node(self, md, monkeypatch, capsys):
        sd = load_script('drive_speed')
        monkeypatch.setattr(md, 'drive_limits', lambda: sd.DriveLimits(6.0, 0.1, 1.0))
        monkeypatch.setattr(md, 'Driver', lambda: pytest.fail('the driver started'))
        assert md.main([]) == 1
        assert 'The dynamics step cannot run: +0.62 m/s needs /drive' in capsys.readouterr().err


class TestLimits:
    def test_matching_limits_give_no_problem(self, md):
        files = {
            'throttle_node': {'max_steering': 0.625},
            'controller_node': {'max_speed_mps': 6.0},
        }
        live = {'throttle_node': {'max_steering': 0.625}, 'controller_node': {'max_speed_mps': 6}}
        assert md.limit_problems(live, files) == []

    def test_a_node_that_does_not_answer_is_named(self, md):
        problems = md.limit_problems({}, {'throttle_node': {'max_steering': 0.625}})
        assert problems == ['throttle_node did not answer with its parameters. Is teleop running?']

    def test_a_limit_that_is_no_number_is_named(self, md):
        problems = md.limit_problems(
            {'throttle_node': {'max_steering': None}}, {'throttle_node': {'max_steering': 0.625}}
        )
        assert problems == ['throttle_node runs max_steering no number; the files give 0.625.']

    def test_the_shipped_files_give_full_lock_at_18_75_deg(self, md):
        files = md.file_limits()
        assert files['throttle_node']['max_speed_forward'] == pytest.approx(1.0)
        assert files['controller_node']['max_speed_mps'] == pytest.approx(6.0)
        full_lock = (
            files['throttle_node']['max_steering']
            * files['controller_node']['max_steering_angle_deg']
        )
        assert full_lock == pytest.approx(18.75)


class TestJoyFreshness:
    def _driver(self, md, monkeypatch, joy_every=None):
        clock = [0.0]
        monkeypatch.setattr(md.time, 'monotonic', lambda: clock[0])
        d = md.Driver.__new__(md.Driver)
        d.arming = md.Arming()
        d.node = None
        d._send = lambda speed, steer: None
        last = [0.0]

        class Rclpy:
            @staticmethod
            def spin_once(node, timeout_sec):
                clock[0] += timeout_sec
                if joy_every is not None and clock[0] - last[0] >= joy_every - 1e-9:
                    last[0] = clock[0]
                    d._button(True)

        d.rclpy = Rclpy
        d._button(True)
        return d, clock

    def test_a_run_stops_when_joy_stops_arriving(self, md, monkeypatch):
        # A /joy that stops with the switch DOWN does not keep the car driving.
        d, clock = self._driver(md, monkeypatch)
        assert not d._hold(3.0, 0.1, 0.0, need_rb=True)
        assert clock[0] == pytest.approx(md.JOY_TIMEOUT_SEC, abs=0.03)

    def test_joy_at_10_hz_keeps_the_run_going(self, md, monkeypatch):
        d, _ = self._driver(md, monkeypatch, joy_every=0.1)
        assert d._hold(3.0, 0.1, 0.0, need_rb=True)


class TestCircleRuns:
    @pytest.mark.parametrize('run,lock', [('circle_right', -1.0), ('circle_left', 1.0)])
    def test_straight_lead_in_then_full_lock(self, md, run, lock):
        speeds_steers = [(sp, st) for sp, st, _ in md.RUNS[run]]
        assert speeds_steers == [(0.0, 0.0), (0.62, 0.0), (0.62, lock), (0.0, 0.0)]

    def test_each_circle_has_its_own_placement(self, md):
        assert 'turns right' in md.CIRCLE_PLACE['circle_right']
        assert 'turns left' in md.CIRCLE_PLACE['circle_left']

    def test_lock_to_lock_needs_both_sides(self, md):
        assert md.lock_to_lock({'servo_center_to_lock_s_right': 0.06}) == {}
        both = {'servo_center_to_lock_s_right': 0.06, 'servo_center_to_lock_s_left': 0.07}
        assert md.lock_to_lock(both) == {'servo_lock_to_lock_s': 0.13}

    def test_calibrate_lists_the_circle_keys(self, md):
        cal = load_script('calibrate')
        (step,) = [s for s in cal.STEPS if s.key == 'dynamics']
        for side in ('right', 'left'):
            assert f'servo_center_to_lock_s_{side}' in step.keys
        assert 'servo_lock_to_lock_s' in step.keys


class TestTurnSide:
    def test_the_gyro_names_the_side(self, md):
        assert md.turn_side(1.0, 1.1) == ('left', '')
        assert md.turn_side(-1.0, -1.1) == ('right', '')

    def test_a_reversed_steering_sign_is_reported(self, md):
        side, warning = md.turn_side(1.0, -1.1)
        assert side == 'right'
        assert 'turned right on a full left command' in warning

    def test_no_gyro_falls_back_to_the_command(self, md):
        assert md.turn_side(-1.0, math.nan) == ('right', '')

    def test_a_circle_run_that_did_not_turn_stores_nothing(self, md):
        # Wheels in the air, or a car that slid: the gyro reads no turn.
        t = np.arange(0.0, 20.0, 0.02)
        bag = md.Bag(
            odom_t=t,
            odom=np.zeros((len(t), 3)),
            speed=np.full_like(t, 0.67),
            path_m=t * 0.67,
            gyro_t=t,
            yaw_rate=np.zeros_like(t),
            motor_t=np.array([0.0, 2.0, 2.8, 15.8]),
            motor=np.array([[0, 0], [0.103, 0], [0.103, -0.625], [0, 0]], float),
            scans=[],
        )
        with pytest.raises(ValueError, match='the gyro shows no turn'):
            md.analyze_circle(bag)


class TestStraightLidarCheck:
    def test_scans_that_do_not_match_give_no_lidar_check(self, md):
        rng = np.random.default_rng(0)
        t = np.arange(0.0, 9.0, 0.02)
        v = np.where((t >= 2.0) & (t < 5.0), 0.62, 0.0)
        start = rng.uniform(-3.0, 3.0, (200, 2))
        end = rng.uniform(20.0, 26.0, (200, 2))
        scans = [(k * 0.1, start, np.zeros(200)) for k in range(18)]
        scans += [(7.0 + k * 0.1, end, np.zeros(200)) for k in range(18)]
        bag = md.Bag(
            odom_t=t,
            odom=np.column_stack((np.cumsum(v) * 0.02, np.zeros_like(t), np.zeros_like(t))),
            speed=v,
            path_m=np.cumsum(v) * 0.02,
            gyro_t=t,
            yaw_rate=np.zeros_like(t),
            motor_t=np.array([0.0, 2.0, 5.0]),
            motor=np.array([[0, 0], [0.62 / 6.0, 0], [0, 0]], float),
            scans=scans,
        )
        values, lines = md.analyze_straight(bag)
        assert 'steady_speed_mps_025' in values
        assert any('so there is no lidar check' in line for line in lines)


class TestPauseBetweenMoves:
    def test_the_switch_stays_down_through_the_pause_and_up_after_the_last_move(
        self, md, monkeypatch, tmp_path
    ):
        gate = TestDriveGate()
        d, held, _ = gate._driver(md, monkeypatch, tmp_path)
        plan = [(0, 0, 2), (0.25, 0, 2), (0, 0, 2), (-0.25, 0, 2), (0, 0, 3)]
        assert d.drive('yaw', plan, prefix='lidar') is not None
        needs = [need for _, _, need in held][-5:]
        assert needs == [True, True, True, True, False]
        assert d.log.lines.count('You can flip the mode switch UP now.') == 1
