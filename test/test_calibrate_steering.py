"""Unit tests for calibrate_steering.py: arc geometry, trim fit, criteria, a simulated run."""

import math

from conftest import load_script
import numpy as np
import pytest


@pytest.fixture(scope='module')
def st():
    return load_script('calibrate_steering')


def _arc(st, command, kappa):
    sm = load_script('scan_match')
    return st.Arc(command, kappa, 0.0, 1.0, sm.MatchResult((0, 0, 0), 0.01, 0.9, 5))


class TestArcGeometry:
    @pytest.mark.parametrize('kappa,heading_deg', [(1.2, 100.0), (-2.0, -250.0), (0.05, 6.0)])
    def test_curvature_from_chord_and_heading(self, st, kappa, heading_deg):
        th = math.radians(heading_deg)
        chord = abs(2 * math.sin(th / 2) / kappa)
        assert st.arc_curvature(th, chord) == pytest.approx(kappa)

    def test_straight_line_has_no_curvature(self, st):
        assert st.arc_curvature(0.0, 2.0) == 0.0


class TestTrimFit:
    def test_center_drift_and_slope_give_the_trim(self, st):
        # kappa = 0.09 + 3.6 * command near center: straight at -0.025.
        arcs = [_arc(st, c, 0.09 + 3.6 * c) for c in (0.0, 0.25, -0.25)]
        fit = st.fit_trim(arcs + [_arc(st, 0.5, 1.85), _arc(st, -0.5, -2.05)])
        assert fit.trim_delta == pytest.approx(-0.025)
        assert fit.drift_deg_per_m == pytest.approx(math.degrees(0.09))
        assert fit.gain_left == pytest.approx(3.6) and fit.gain_right == pytest.approx(3.6)

    def test_saturated_commands_do_not_move_the_trim(self, st):
        # The reference car: asymmetric at +/-0.5, nearly straight at 0.
        arcs = [_arc(st, 0.0, -0.016), _arc(st, 0.25, 0.9), _arc(st, -0.25, -0.9)]
        arcs += [_arc(st, 0.5, 1.845), _arc(st, -0.5, -2.05)]
        assert st.fit_trim(arcs).trim_delta == pytest.approx(0.016 / 3.6, rel=0.01)

    def test_one_side_is_enough(self, st):
        fit = st.fit_trim([_arc(st, 0.0, 0.036), _arc(st, -0.25, -0.864)])
        assert fit.trim_delta == pytest.approx(-0.01)

    def test_needs_the_straight_run(self, st):
        assert st.fit_trim([_arc(st, 0.25, 0.9), _arc(st, -0.25, -0.9)]) is None


class TestRuns:
    @pytest.mark.parametrize('command', [0.0, 0.25, -1.0])
    def test_steering_never_changes_within_a_run(self, st, command):
        assert {steer for _, steer, _ in st.segments(command)} == {command}
        assert st.segments(command)[0][0] == 0.0 and st.segments(command)[-1][0] == 0.0

    def test_curved_runs_turn_under_about_100_degrees(self, st):
        # Measured curvature on the reference car at 0.72 m/s.
        for command, kappa in ((0.25, 0.9), (0.5, 2.05), (1.0, 2.4)):
            drive = st.segments(command)[1][2]
            assert math.degrees(kappa * 0.72 * drive) < 110

    def test_names(self, st):
        assert [st.run_name(c) for c in (0.0, 0.25, -0.5, 1.0)] == ['c0', 'l025', 'r05', 'l10']


class TestCriteria:
    @pytest.mark.parametrize(
        'drift,status', [(0.4, 'PASS'), (-2.7, 'WARN'), (-4.0, 'WARN'), (5.2, 'FAIL')]
    )
    def test_drift(self, st, drift, status):
        assert st.drift_criterion(drift).status == status

    def test_warning_names_hardware_play(self, st):
        assert 'servo and linkage play' in st.drift_criterion(-2.7).advice
        assert 'check the servo horn' in st.drift_criterion(6.0).advice

    def test_trim_not_loaded_fails(self, st):
        assert st.trim_criterion(-0.045, 0.0).status == 'FAIL'
        assert st.trim_criterion(-0.045, -0.045).status == 'INFO'


class TestMeasureArc:
    def test_simulated_run(self, st):
        from test_scan_match import _points, _scan

        sm = load_script('scan_match')
        md = load_script('calibrate_dynamics')
        laser = (0.2383, 0.0, math.pi)
        kappa, turn = 1.2, math.radians(100.0)
        world0 = (-0.5, -0.3, 0.2)
        rel = (math.sin(turn) / kappa, (1 - math.cos(turn)) / kappa, turn)
        world1 = sm.compose(world0, rel)

        def scans(world, t0):
            pts = _points(sm, _scan(sm.compose(world, laser), noise=0.004, seed=int(t0)))
            return [(t0, pts, np.zeros(len(pts)))]

        t = np.linspace(0.0, 10.0, 501)
        frac = np.clip((t - 3.0) / 3.0, 0.0, 1.0)
        # Odometry reads the turn 3% high, as a gyro scale error would.
        odom = np.column_stack(
            (
                np.sin(frac * turn) / kappa,
                (1 - np.cos(frac * turn)) / kappa,
                frac * turn * 1.03,
            )
        )
        speed = np.where((t > 3.0) & (t < 6.0), 0.7, 0.0)
        bag = md.Bag(
            odom_t=t,
            odom=odom,
            speed=speed,
            path_m=np.cumsum(speed) * 0.02,
            gyro_t=t,
            yaw_rate=np.zeros_like(t),
            motor_t=np.array([0.0, 3.0, 6.0]),
            motor=np.array([[0.0, 0.5], [0.25, 0.5], [0.0, 0.5]]),
            scans=scans(world0, 1.5) + scans(world1, 8.3),
        )
        arc = st.measure_arc(bag, laser, 0.5)
        assert arc.curvature == pytest.approx(kappa, rel=0.005)
        assert arc.heading_deg == pytest.approx(100.0, abs=0.1)
        assert arc.command == 0.5


class TestArcProblem:
    def _arc(self, st, inliers, rms, lidar_deg, odom_deg, chord=1.5, odom_chord=1.52):
        sm = load_script('scan_match')
        match = sm.MatchResult((0, 0, 0), rms, inliers, 5)
        return st.Arc(0.25, 0.9, lidar_deg, chord, match, odom_deg, odom_chord)

    def test_a_long_run_with_43_percent_matched_is_trusted(self, st):
        assert st.arc_problem(self._arc(st, 0.43, 0.0274, 82.0, 80.5)) == ''

    def test_disagreement_with_the_odometry_is_refused(self, st):
        assert 'odometry' in st.arc_problem(self._arc(st, 0.6, 0.015, 82.0, 120.0))

    def test_too_few_points_is_refused(self, st):
        assert 'too poor' in st.arc_problem(self._arc(st, 0.2, 0.015, 82.0, 82.0))

    def test_wrong_chord_is_refused(self, st):
        # The 16:25 run: heading agreed, chord a third of the drive.
        arc = self._arc(st, 0.32, 0.0257, -128.2, -124.7, chord=0.477, odom_chord=1.40)
        assert 'chord' in st.arc_problem(arc)


class TestTurnGain:
    def _arc(self, st, command, kappa, enc, lidar):
        sm = load_script('scan_match')
        m = sm.MatchResult((0, 0, 0), 0.01, 0.9, 5)
        return st.Arc(command, kappa, 0.0, 1.0, m, 0.0, 1.0, enc, lidar)

    def _x(self, kappa, wheelbase=0.23):
        return 1 / math.cos(math.atan(wheelbase * abs(kappa))) - 1

    def test_recovers_half_with_a_short_straight_baseline(self, st):
        base = 0.983  # straight runs read 1.7% short this session
        arcs = [self._arc(st, 0.0, 0.0, base * 2.3, 2.3)]
        for c, k in ((0.5, 1.85), (-0.5, -2.06), (1.0, 2.37), (-1.0, -2.14)):
            arcs.append(self._arc(st, c, k, base * 4.0 * (1 + 0.5 * self._x(k)), 4.0))
        fit = st.fit_turn_gain(arcs, current_gain=0.0, wheelbase=0.23)
        assert fit.gain == pytest.approx(0.5, abs=1e-6)
        assert fit.straight_ratio == pytest.approx(base) and fit.runs == 4

    def test_a_correction_already_applied_is_put_back(self, st):
        # /odom/integrated already divides by the 0.4 factor; the fit still finds 0.5.
        arcs = [self._arc(st, 0.0, 0.0, 2.3, 2.3)]
        for c, k in ((0.5, 1.85), (1.0, 2.37)):
            x = self._x(k)
            arcs.append(self._arc(st, c, k, 4.0 * (1 + 0.5 * x) / (1 + 0.4 * x), 4.0))
        assert st.fit_turn_gain(arcs, 0.4, 0.23).gain == pytest.approx(0.5, abs=1e-6)

    def test_needs_the_straight_run_and_two_curved(self, st):
        arcs = [self._arc(st, 0.5, 1.85, 4.2, 4.0), self._arc(st, 1.0, 2.37, 4.3, 4.0)]
        assert st.fit_turn_gain(arcs, 0.0, 0.23) is None
        arcs.append(self._arc(st, 0.25, 0.9, 1.6, 1.6))  # carries too little steering
        arcs = arcs[:1] + [self._arc(st, 0.0, 0.0, 2.3, 2.3)]
        assert st.fit_turn_gain(arcs, 0.0, 0.23) is None


class TestNeoRacer:
    def test_one_drive_unit_is_18_75_degrees_on_this_car(self, st):
        limits = {
            'throttle_node': {'max_steering': 0.625},
            'controller_node': {'max_steering_angle_deg': 30.0},
        }
        assert st.degrees_per_command(limits) == pytest.approx(18.75)

    def test_the_new_board_trim_adds_the_correction_and_folds_the_driver_trim(self, st):
        # Measured with both trims in use; the driver's goes to 0 afterwards.
        assert st.new_board_trim(-1.0, 0.0, -0.75) == pytest.approx(-1.75)
        assert st.new_board_trim(-1.0, 0.5, -0.75) == pytest.approx(-1.25)

    def test_runs_are_stated_in_mps_and_go_out_as_drive_values(self, st, monkeypatch):
        sd = load_script('drive_speed')
        monkeypatch.setattr(st, 'drive_limits', lambda: sd.DriveLimits(6.0, 1.0, 1.0))
        assert st.segments(0.25)[1] == (st.DRIVE_MPS, 0.25, 2.0)
        plans = st.session_plans()
        assert set(plans) == set(st.COMMANDS)
        speed, steer, _ = plans[-0.5][1]
        assert speed == pytest.approx(0.62 / 6.0) and steer == -0.5

    def test_curved_runs_turn_under_about_100_degrees_on_this_car(self, st):
        # Full lock 18.75 deg on the 287.64 mm wheelbase, at the 0.66 m/s the
        # ESP32 holds for a 0.62 m/s command.
        for command in (0.25, 0.5, 1.0):
            kappa = math.tan(math.radians(18.75 * command)) / 0.28764
            drive = st.segments(command)[1][2]
            assert math.degrees(kappa * 0.66 * drive) < 100

    def test_the_board_trim_is_compared_with_the_profile_copy(self, st):
        assert st.trim_criterion(-1.0, -1.0).status == 'INFO'
        assert st.trim_criterion(-1.0, -1.004).status == 'INFO'
        mismatch = st.trim_criterion(-1.5, -1.0)
        assert mismatch.status == 'FAIL' and 'full flash' in mismatch.advice
        assert 'board.steering_trim_deg -1.50' in mismatch.advice
        assert st.trim_criterion(-1.0, math.nan).status == 'FAIL'
        assert st.trim_criterion(None, -1.0).status == 'INFO'
        assert st.trim_criterion(-1.0, None).status == 'INFO'

    def test_a_driver_trim_warns(self, st):
        assert st.driver_trim_criterion(0.0) is None
        assert st.driver_trim_criterion(0.5).status == 'WARN'

    def test_the_profile_copy_is_read_from_the_steering_file(self, st, car_profile_dir):
        params = car_profile_dir / 'params'
        params.mkdir(parents=True, exist_ok=True)
        assert st.profile_copy() is None
        (params / 'steering.yaml').write_text('trim_deg: -1.0\nprevious_trim_deg: -0.5\n')
        assert st.profile_copy() == -1.0

    def test_measure_arc_with_the_neoracer_lidar_mount(self, st):
        from test_scan_match import _points, _scan

        sm = load_script('scan_match')
        md = load_script('calibrate_dynamics')
        laser = (0.09962, 0.0, 0.0)
        kappa, turn = 0.58, math.radians(22.0)
        world0 = (0.4, -0.2, 0.1)
        rel = (math.sin(turn) / kappa, (1 - math.cos(turn)) / kappa, turn)
        world1 = sm.compose(world0, rel)

        def scans(world, t0):
            pts = _points(sm, _scan(sm.compose(world, laser), noise=0.004, seed=int(t0)))
            return [(t0, pts, np.zeros(len(pts)))]

        t = np.linspace(0.0, 10.0, 501)
        frac = np.clip((t - 3.0) / 1.0, 0.0, 1.0)
        odom = np.column_stack(
            (np.sin(frac * turn) / kappa, (1 - np.cos(frac * turn)) / kappa, frac * turn)
        )
        speed = np.where((t > 3.0) & (t < 4.0), 0.66, 0.0)
        bag = md.Bag(
            odom_t=t,
            odom=odom,
            speed=speed,
            path_m=np.cumsum(speed) * 0.02,
            gyro_t=t,
            yaw_rate=np.zeros_like(t),
            motor_t=np.array([0.0, 3.0, 4.0]),
            motor=np.array([[0.0, -0.3125], [0.103, -0.3125], [0.0, -0.3125]]),
            scans=scans(world0, 1.5) + scans(world1, 8.3),
        )
        arc = st.measure_arc(bag, laser, -0.5)
        assert arc.curvature == pytest.approx(kappa, rel=0.01)
        assert arc.heading_deg == pytest.approx(22.0, abs=0.2)
