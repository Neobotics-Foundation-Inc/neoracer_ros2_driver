"""Unit tests for calibrate_lidar.py: wall fit and yaw correction on synthetic scans."""

import math
from pathlib import Path

from conftest import load_script as _load
import numpy as np
import pytest
import yaml

# calibrate_lidar.py imports rclpy, so these run on the car (off car the module
# is skipped, as test_calibrate.py is).
pytest.importorskip('rclpy')

# The LakiBeam: 1440 beams over the full turn, zero straight ahead.
N_BEAMS = 1440
ANGLE_MIN = -math.pi
ANGLE_INC = 2 * math.pi / N_BEAMS
# config/frames.yaml: laser_x_mm 99.62, laser_y_mm 0, laser_yaw_deg 0.
LASER_X, LASER_Y = 0.09962, 0.0
CALIBRATION = Path(__file__).resolve().parent.parent / 'scripts' / 'calibration'
NEEDS_DRIVE_TOOLS = pytest.mark.skipif(
    not all((CALIBRATION / f'{n}.py').is_file() for n in ('calibrate_odom', 'calibrate_dynamics')),
    reason='the driving method needs calibrate_odom.py and calibrate_dynamics.py',
)


@pytest.fixture(scope='module')
def cal():
    return _load('calibrate_lidar')


@pytest.fixture(scope='module')
def common():
    return _load('calibrate_common')


def _wall_scan(
    true_yaw_deg: float, wall_x: float, half_width: float = 1.5, noise: float = 0.0, seed: int = 0
):
    """Ranges a lidar at (LASER_X, LASER_Y) sees of a wall at base x = wall_x."""
    rng = np.random.default_rng(seed)
    ranges = np.full(N_BEAMS, np.inf)
    for i in range(N_BEAMS):
        heading = math.radians(true_yaw_deg) + ANGLE_MIN + i * ANGLE_INC
        dx = math.cos(heading)
        if dx <= 1e-6:
            continue
        r = (wall_x - LASER_X) / dx
        if abs(LASER_Y + r * math.sin(heading)) <= half_width:
            ranges[i] = r + (rng.normal(0.0, noise) if noise else 0.0)
    return ranges


def _fit(cal, ranges, assumed_yaw_deg=0.0):
    pts = cal.scan_points(ranges, ANGLE_MIN, ANGLE_INC, 0.15, 12.0)
    base = cal.to_base(pts, LASER_X, LASER_Y, math.radians(assumed_yaw_deg))
    return cal.fit_wall(cal.wall_sector(base, LASER_X, LASER_Y))


class TestWallFit:
    def test_square_wall_fits_at_zero(self, cal):
        fit = _fit(cal, _wall_scan(0.0, 1.2))
        assert fit.angle_deg == pytest.approx(0.0, abs=1e-6)
        assert fit.distance_mm == pytest.approx(1200.0)
        assert fit.rms_mm == pytest.approx(0.0, abs=1e-6)

    @pytest.mark.parametrize('true_yaw', [0.7, -1.2, 3.0])
    def test_recovers_the_true_yaw(self, cal, true_yaw):
        fit = _fit(cal, _wall_scan(true_yaw, 1.0))
        assert cal.corrected_yaw_deg(0.0, fit) == pytest.approx(true_yaw, abs=1e-3)

    def test_noise_averages_out_over_a_collection(self, cal):
        # 36 scans, fewer than 5 s gives at the LakiBeam's rate; 5 mm range noise.
        scans = [list(_wall_scan(0.4, 1.0, noise=0.005, seed=s)) for s in range(36)]
        fit = _fit(cal, cal.median_ranges(scans))
        assert cal.corrected_yaw_deg(0.0, fit) == pytest.approx(0.4, abs=0.05)
        assert fit.rms_mm < 5.0

    def test_sector_keeps_only_points_ahead(self, cal):
        pts = np.array([[1.5, 0.0], [0.0, 0.0], [1.5, 2.0]])
        kept = cal.wall_sector(pts, LASER_X, LASER_Y)
        assert kept.tolist() == [[1.5, 0.0]]

    def test_too_few_points(self, cal):
        assert cal.fit_wall(np.zeros((cal.MIN_WALL_POINTS - 1, 2))) is None

    def test_returns_beyond_range_limits_are_dropped(self, cal):
        ranges = np.array([0.1, 1.0, 13.0, np.inf, np.nan])
        assert len(cal.scan_points(ranges, 0.0, 0.01, 0.15, 12.0)) == 1


class TestMedianRanges:
    def test_median_per_beam_ignoring_inf(self, cal):
        got = cal.median_ranges([[1.0, np.inf], [3.0, 2.0], [2.0, 4.0]])
        assert got.tolist() == [2.0, 3.0]

    def test_odd_length_scans_dropped(self, cal):
        got = cal.median_ranges([[1.0, 1.0], [5.0], [3.0, 3.0]])
        assert got.tolist() == [2.0, 2.0]

    def test_no_scans(self, cal):
        assert cal.median_ranges([]) is None


class TestUpdateLocalValues:
    def test_keeps_other_keys(self, common, car_profile_dir):
        (car_profile_dir / 'params').mkdir(parents=True)
        (car_profile_dir / 'params' / 'frames.yaml').write_text(
            'camera_pitch_deg: 1.5\nlaser_yaw_deg: 0.0\n'
        )
        (path,) = common.update_local_values('frames', {'laser_yaw_deg': 2.4}, 'test')
        text = path.read_text()
        assert text.startswith('#')
        assert yaml.safe_load(text) == {'camera_pitch_deg': 1.5, 'laser_yaw_deg': 2.4}

    def test_creates_the_file(self, common, car_profile_dir):
        (path,) = common.update_local_values('frames', {'laser_yaw_deg': 2.2}, 't')
        assert path == car_profile_dir / 'params' / 'frames.yaml'
        assert yaml.safe_load(path.read_text()) == {'laser_yaw_deg': 2.2}


class TestCheckCriteria:
    def test_square_wall_passes(self, cal):
        fit = _fit(cal, _wall_scan(0.0, 1.0))
        statuses = {c.name: c.status for c in cal.check_criteria(fit, 0.0)}
        assert statuses == {
            'wall angle': 'PASS',
            'wall straightness': 'PASS',
            'wall distance': 'INFO',
        }

    def test_within_hand_squaring_warns_with_the_fitted_yaw(self, cal):
        fit = _fit(cal, _wall_scan(0.7, 1.0))
        angle = cal.check_criteria(fit, 0.0)[0]
        assert angle.status == 'WARN'
        assert '0.70' in angle.measured

    def test_beyond_hand_squaring_fails(self, cal):
        fit = _fit(cal, _wall_scan(3.0, 1.0))
        assert cal.check_criteria(fit, 0.0)[0].status == 'FAIL'


class TestYawFromTravel:
    LASER_XY = (LASER_X, LASER_Y)

    def _laser_motion(self, base, yaw_deg):
        sm = _load('scan_match')
        return sm.laser_motion(base, (*self.LASER_XY, math.radians(yaw_deg)))

    @pytest.mark.parametrize(
        'base',
        [
            (1.3, -0.02, -math.radians(1.8)),  # forward, drifting right
            (-1.3, -0.03, math.radians(2.5)),  # reverse
            (2.3, 0.0, 0.0),  # dead straight
        ],
    )
    def test_recovers_the_mount_yaw(self, base, cal):
        # base_link's chord at half the turn: a no-sideslip arc.
        th = base[2]
        d = math.hypot(base[0], base[1]) * (1 if base[0] > 0 else -1)
        base = (d * math.cos(th / 2), d * math.sin(th / 2), th)
        m = self._laser_motion(base, 2.5)
        direction = math.atan2(base[1], base[0])
        got = cal.yaw_from_leg(m, th, self.LASER_XY, math.radians(0.0), direction)
        assert math.degrees(got) == pytest.approx(2.5, abs=1e-6)

    def test_growing_curvature_needs_the_odometry_direction(self, cal):
        # Curvature that grows along the leg: the chord sits well off half the
        # turn, so the half-turn rule would be biased; the true direction is not.
        sm = _load('scan_match')
        pose = (0.0, 0.0, 0.0)
        for k in range(50):
            pose = sm.compose(pose, (0.03, 0.0, math.radians(-0.012 * k)))
        m = self._laser_motion(pose, 2.5)
        got = cal.yaw_from_leg(
            m, pose[2], self.LASER_XY, math.radians(0.0), math.atan2(pose[1], pose[0])
        )
        assert math.degrees(got) == pytest.approx(2.5, abs=1e-6)
        assert abs(math.atan2(pose[1], pose[0]) - pose[2] / 2) > math.radians(1.5)

    def test_summary(self, cal):
        mean, se = cal.yaw_summary([2.1, 2.6, 2.3, 2.5])
        assert mean == pytest.approx(2.375)
        assert se == pytest.approx(np.std([2.1, 2.6, 2.3, 2.5], ddof=1) / 2)
        assert cal.yaw_summary([2.0])[1] == math.inf

    @pytest.mark.parametrize('measured,status', [(0.3, 'PASS'), (0.8, 'WARN'), (1.5, 'FAIL')])
    def test_offset_criterion(self, cal, measured, status):
        assert cal.offset_criterion(measured, 0.0).status == status

    def test_offset_wraps_at_180(self, cal):
        assert cal.offset_criterion(-179.9, 179.9).status == 'PASS'


class TestDrivePlan:
    """NeoRacer: the drive is stated in m/s and stops on the mode switch."""

    def test_the_legs_are_half_a_meter_per_second_and_about_1_3_m(self, cal):
        forward = [s for s in cal.SEGMENTS_MPS if s[0] > 0]
        reverse = [s for s in cal.SEGMENTS_MPS if s[0] < 0]
        assert forward == [(0.5, 0.0, cal.LEG_SECONDS)]
        assert reverse == [(-0.5, 0.0, cal.LEG_SECONDS)]
        assert 1.2 <= cal.DRIVE_SPEED_MPS * cal.LEG_SECONDS <= 1.4

    def test_the_shipped_config_drives_at_half_a_meter_per_second(self, cal):
        ds = _load('drive_speed')
        config = Path(__file__).resolve().parent.parent / 'config'
        limits = ds.drive_limits(config)
        segments = ds.drive_segments(cal.SEGMENTS_MPS, limits)
        for (cmd, _, _), (mps, _, _) in zip(segments, cal.SEGMENTS_MPS, strict=True):
            assert cmd * limits.mps_per_command(mps >= 0) == pytest.approx(mps)

    def test_the_operator_is_told_how_to_stop(self, cal):
        assert 'mode switch DOWN' in cal.STOP
        assert 'Flip the mode switch UP to stop.' in cal.STOP
        assert 'RB' not in cal.STOP + cal.MOTION + cal.PLACE

    def test_missing_drive_tools_are_named(self, cal, tmp_path):
        assert cal.missing_drive_tools(tmp_path) == [
            'calibrate_odom.py',
            'calibrate_dynamics.py',
        ]
        for name in ('calibrate_odom.py', 'calibrate_dynamics.py'):
            (tmp_path / name).write_text('')
        assert cal.missing_drive_tools(tmp_path) == []


@NEEDS_DRIVE_TOOLS
class TestMeasureLegs:
    def test_forward_and_reverse_from_a_simulated_run(self, cal):
        from test_scan_match import _points, _scan

        sm = _load('scan_match')
        md = _load('calibrate_dynamics')
        true_yaw = math.radians(2.5)
        laser_true = (LASER_X, LASER_Y, true_yaw)
        laser_cfg = (LASER_X, LASER_Y, 0.0)
        world0 = (-1.2, -0.6, 0.15)
        th1 = math.radians(-2.0)
        leg1 = (1.3 * math.cos(th1 / 2), 1.3 * math.sin(th1 / 2), th1)
        world1 = sm.compose(world0, leg1)
        th2 = math.radians(1.5)
        leg2 = (-1.25 * math.cos(th2 / 2), -1.25 * math.sin(th2 / 2), th2)
        world2 = sm.compose(world1, leg2)

        def scans(world, t0, seed):
            pts = _points(sm, _scan(sm.compose(world, laser_true), noise=0.004, seed=seed))
            return [(t0, pts, np.zeros(len(pts)))]

        t = np.arange(0.0, 14.0, 0.02)
        odom = []
        for tt in t:
            if tt < 3.0:
                p = (0.0, 0.0, 0.0)
            elif tt < 7.0:
                p = leg1
            else:
                p = sm.compose(leg1, leg2)
            odom.append(p)
        odom = np.array(odom)
        bag = md.Bag(
            odom_t=t,
            odom=odom,
            speed=np.zeros_like(t),
            path_m=np.zeros_like(t),
            gyro_t=t,
            yaw_rate=np.zeros_like(t),
            motor_t=np.array([0.0, 3.0, 5.0, 7.0, 9.0]),
            motor=np.array([[0, 0], [0.083, 0], [0, 0], [-0.083, 0], [0, 0]], float),
            scans=scans(world0, 2.5, 1) + scans(world1, 6.5, 2) + scans(world2, 13.5, 3),
        )
        legs = cal.measure_legs(bag, laser_cfg)
        assert [leg.direction for leg in legs] == ['forward', 'reverse']
        for leg in legs:
            assert leg.problem == ''
            assert leg.yaw_deg == pytest.approx(2.5, abs=0.1)
        assert [leg.counted for leg in legs] == [True, False]

    def test_no_reverse_motion_is_reported(self, cal):
        md = _load('calibrate_dynamics')
        t = np.arange(0.0, 14.0, 0.02)
        odom = np.zeros((len(t), 3))
        bag = md.Bag(
            odom_t=t,
            odom=odom,
            speed=np.zeros_like(t),
            path_m=np.zeros_like(t),
            gyro_t=t,
            yaw_rate=np.zeros_like(t),
            motor_t=np.array([0.0, 3.0, 5.0, 7.0, 9.0]),
            motor=np.array([[0, 0], [0.083, 0], [0, 0], [-0.083, 0], [0, 0]], float),
            scans=[],
        )
        legs = cal.measure_legs(bag, (LASER_X, LASER_Y, 0.0))
        assert legs[1].problem == 'The car did not reverse.'
