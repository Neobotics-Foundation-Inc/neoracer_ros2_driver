"""
Unit tests for odometry.py and its use in controller.py: integration, bias, config.

The integrator tests need no ROS. The controller tests import controller.py,
which imports rclpy and the message packages, so they run on the car.
"""

import math
from pathlib import Path
import re
from types import SimpleNamespace

from neoracer_ros2_driver import controller_lib
from neoracer_ros2_driver.odometry import OdomIntegrator, OdomState, yaw_quaternion
import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent
CONTROLLER = ROOT / 'neoracer_ros2_driver' / 'controller.py'
DT = 0.005  # ESP32 `s` frame period, s (about 200 Hz)
# The spec's wheelbase (neoracer-mujoco neoracer_spec, from the CAD) and its
# inner-wheel steering lock.
SPEC_WHEELBASE = 0.28764
SPEC_INNER_LOCK_DEG = 30.0


def _drive(integ, seconds, v, omega_raw, t0=0.0):
    """Feed constant samples for `seconds`; return the end time."""
    t = t0
    for _ in range(round(seconds / DT)):
        t += DT
        integ.step(t, v, omega_raw)
    return t


class TestIntegration:
    def test_first_sample_only_sets_the_clock(self):
        integ = OdomIntegrator()
        assert integ.step(1.0, 0.5, 0.0) is False
        assert (integ.state.x, integ.state.t) == (0.0, 1.0)

    def test_straight_line(self):
        integ = OdomIntegrator()
        integ.step(0.0, 0.0, 0.0)
        t = _drive(integ, 2.0, 0.5, 0.0)
        assert integ.state.x == pytest.approx(0.5 * t, abs=1e-9)
        assert integ.state.y == pytest.approx(0.0, abs=1e-12)

    def test_circle_closes(self):
        # 1 m radius at 0.5 m/s: one lap in 4 pi s.
        integ = OdomIntegrator()
        integ.step(0.0, 0.0, 0.0)
        _drive(integ, 4 * math.pi, 0.5, 0.5)
        s = integ.state
        assert math.hypot(s.x, s.y) < 0.01
        assert abs(s.theta) < math.radians(0.5)

    def test_quarter_turn_lands_on_the_arc(self):
        integ = OdomIntegrator()
        integ.step(0.0, 0.0, 0.0)
        t = _drive(integ, math.pi, 0.5, 0.5)  # about 90 deg of a 1 m circle, turning left
        s, heading = integ.state, 0.5 * t
        assert s.theta == pytest.approx(heading, abs=1e-9)
        assert s.x == pytest.approx(math.sin(heading), abs=1e-5)
        assert s.y == pytest.approx(1.0 - math.cos(heading), abs=1e-5)

    def test_reverse_moves_backwards(self):
        integ = OdomIntegrator()
        integ.step(0.0, 0.0, 0.0)
        t = _drive(integ, 1.0, -0.5, 0.0)
        assert integ.state.x == pytest.approx(-0.5 * t, abs=1e-9)

    def test_time_not_moving_forward_is_ignored(self):
        integ = OdomIntegrator()
        integ.step(1.0, 0.5, 0.0)
        assert integ.step(1.0, 0.5, 0.0) is False
        assert integ.step(0.9, 0.5, 0.0) is False

    def test_gap_holds_the_pose(self):
        integ = OdomIntegrator(stale_sec=0.25)
        integ.step(0.0, 0.0, 0.0)
        t = _drive(integ, 1.0, 0.5, 0.0)
        x = integ.state.x
        assert integ.step(t + 1.0, 0.5, 0.0) is False  # 1 s gap: no 0.5 m jump
        assert integ.state.x == x and integ.gaps == 1
        end = _drive(integ, 1.0, 0.5, 0.0, t0=t + 1.0)
        assert integ.state.x == pytest.approx(x + 0.5 * (end - t - 1.0), abs=1e-9)


class TestBias:
    BIAS = math.radians(0.2)  # residual after calibration, rad/s

    def test_stopped_car_learns_the_bias_and_holds_heading(self):
        integ = OdomIntegrator(zero_hold_sec=1.0, bias_tau_sec=30.0)
        integ.step(0.0, 0.0, self.BIAS)
        t = _drive(integ, 1.0, 0.0, self.BIAS)
        heading_at_hold = integ.state.theta
        _drive(integ, 150.0, 0.0, self.BIAS, t0=t)  # five time constants
        assert integ.holding
        assert integ.bias == pytest.approx(self.BIAS, rel=0.01)
        assert integ.state.theta == heading_at_hold

    def test_learned_bias_stops_drift_when_driving(self):
        integ = OdomIntegrator()
        integ.step(0.0, 0.0, self.BIAS)
        t = _drive(integ, 200.0, 0.0, self.BIAS)
        start = integ.state.theta
        _drive(integ, 60.0, 0.5, self.BIAS, t0=t)  # straight for a minute
        assert abs(integ.state.theta - start) < math.radians(0.1)
        assert not integ.holding

    def test_without_the_update_a_minute_drifts(self):
        integ = OdomIntegrator()
        integ.step(0.0, 0.5, self.BIAS)
        _drive(integ, 60.0, 0.5, self.BIAS)
        assert math.degrees(integ.state.theta) == pytest.approx(12.0, abs=0.1)

    def test_no_update_before_the_hold_time(self):
        integ = OdomIntegrator(zero_hold_sec=1.0)
        integ.step(0.0, 0.0, self.BIAS)
        _drive(integ, 0.9, 0.0, self.BIAS)
        assert integ.bias == 0.0 and not integ.holding

    def test_car_carried_by_hand_is_not_learned(self):
        integ = OdomIntegrator(zero_max_rate=math.radians(2.0))
        integ.step(0.0, 0.0, 0.0)
        _drive(integ, 10.0, 0.0, math.radians(30.0))  # wheels still, car turning
        assert integ.bias == 0.0
        assert math.degrees(integ.state.theta) == pytest.approx(-60.0, abs=0.5)  # wrapped 300

    def test_moving_car_is_not_learned(self):
        integ = OdomIntegrator()
        integ.step(0.0, 0.5, self.BIAS)
        _drive(integ, 10.0, 0.5, self.BIAS)
        assert integ.bias == 0.0


class TestQuaternion:
    @pytest.mark.parametrize('theta', [0.0, 0.7, -2.5, math.pi])
    def test_unit_and_about_z(self, theta):
        x, y, z, w = yaw_quaternion(theta)
        assert (x, y) == (0.0, 0.0)
        assert z * z + w * w == pytest.approx(1.0)
        assert 2 * math.atan2(z, w) == pytest.approx(theta)


@pytest.fixture(scope='module')
def controller():
    """Import controller.py, or skip off the car where rclpy is missing."""
    return pytest.importorskip('neoracer_ros2_driver.controller')


def _odom_controller(controller, source='integrated'):
    """Return the controller's odometry methods over plain attributes."""
    node = controller.ControllerNode

    class _OdomController:
        pub_imu = node.pub_imu
        pub_odom_integrated = node.pub_odom_integrated
        _integrate_odometry = node._integrate_odometry
        _firmware_odometry = node._firmware_odometry
        _odometry_due = node._odometry_due
        _publish_odometry = node._publish_odometry

        def __init__(self):
            self._odometry = OdomIntegrator() if source == 'integrated' else None
            self._odom_period = 0.02
            self._odom_next_pub = None
            self.odom_frame, self.base_frame = 'odom', 'base_link'
            self.sent = []
            self.transforms = []
            self._odom_tf = None
            self._pub_odom_integrated = SimpleNamespace(publish=self.sent.append)
            self._yaw_axis = 2
            self._yaw_gain = 1.0
            self._speed_scale = 1.0
            self._odom_gaps_logged = 0
            self._odom_bias_logged = 0.0
            self.logged = []
            self.imu_frame = 'imu_link'
            self._orientation_valid = True
            self._ag_order = list(controller_lib.IDENTITY_AXIS_ORDER)
            self._ag_sign = list(controller_lib.IDENTITY_AXIS_SIGN)
            self.imu_pub = SimpleNamespace(publish=lambda msg: None)
            self.now_ns = 1_000_000_000

        def get_logger(self):
            return SimpleNamespace(info=self.logged.append, warn=self.logged.append)

        def get_clock(self):
            from rclpy.time import Time

            return SimpleNamespace(now=lambda: Time(nanoseconds=self.now_ns))

        def tf_on(self):
            self._odom_tf = SimpleNamespace(sendTransform=self.transforms.append)

    return _OdomController()


def _s_frame(px=0.0, py=0.0, yaw=0.0, vx=0.0, gx=0.0, gy=0.0, gz=0.0):
    """Parse an ESP32 `s` line with these fields and the car level and still otherwise."""
    line = f's {px} {py} 0.0 {vx} 0.0 0.0 {yaw} 0.0 0.0 0.0 1.0 0.0 0.0 9.81 {gx} {gy} {gz}'
    data, tag = controller_lib.parse_serial_data(line)
    assert tag == 's'
    return data


def _feed(node, data):
    """Run one `s` frame through the controller as _dispatch does, 5 ms after the last."""
    node.now_ns += 5_000_000
    imu = node.pub_imu(data)
    node.pub_odom_integrated(data, imu)


class TestControllerOdometry:
    def test_yaw_is_body_z(self, controller):
        node = _odom_controller(controller)
        node._integrate_odometry(1_000_000_000, 0.5, [0.0, 0.0, 0.0])
        node._integrate_odometry(1_005_000_000, 0.5, [0.3, -0.2, 0.9])
        assert node._odometry.state.omega == pytest.approx(0.9)  # left turn

    def test_speed_scale_applies(self, controller):
        node = _odom_controller(controller)
        node._speed_scale = 1.1
        node._integrate_odometry(1_000_000_000, 0.5, [0.0, 0.0, 0.0])
        node._integrate_odometry(1_005_000_000, 0.5, [0.0, 0.0, 0.0])
        assert node._odometry.state.v == pytest.approx(0.55)

    def test_gap_is_logged_once(self, controller):
        node = _odom_controller(controller)
        node._integrate_odometry(1_000_000_000, 0.5, [0.0, 0.0, 0.0])
        node._integrate_odometry(2_000_000_000, 0.5, [0.0, 0.0, 0.0])
        assert sum('gap' in m for m in node.logged) == 1

    def test_publishes_at_the_configured_rate_from_the_reader_thread(self, controller):
        node = _odom_controller(controller)
        node.tf_on()
        for i in range(200):  # 1 s of 5 ms `s` frames
            node._integrate_odometry(1_000_000_000 + i * 5_000_000, 0.5, [0.0, 0.0, 0.0])
        assert 49 <= len(node.transforms) <= 51  # 50 Hz average from 5 ms frames
        assert len(node.sent) == len(node.transforms)

    def test_message_and_transform_agree(self, controller):
        state = OdomState(x=1.5, y=-0.25, theta=0.6, v=0.5, omega=0.1, t=12.5)
        tf, odom = controller.build_integrated_odom(state, 'odom', 'base_link')
        assert (tf.header.frame_id, tf.child_frame_id) == ('odom', 'base_link')
        assert (odom.header.frame_id, odom.child_frame_id) == ('odom', 'base_link')
        assert tf.header.stamp == odom.header.stamp
        assert (tf.header.stamp.sec, tf.header.stamp.nanosec) == (12, 500_000_000)
        assert tf.transform.translation.x == odom.pose.pose.position.x == 1.5
        assert tf.transform.rotation.z == odom.pose.pose.orientation.z
        assert 2 * math.atan2(tf.transform.rotation.z, tf.transform.rotation.w) == pytest.approx(
            0.6
        )
        assert (odom.twist.twist.linear.x, odom.twist.twist.angular.z) == (0.5, 0.1)

    def test_the_s_frame_feeds_vx_and_the_published_gz(self, controller):
        node = _odom_controller(controller)
        _feed(node, _s_frame(px=9.0, vx=0.5, gx=0.3, gy=-0.2, gz=0.9))
        _feed(node, _s_frame(px=9.0, vx=0.5, gx=0.3, gy=-0.2, gz=0.9))
        state = node._odometry.state
        assert (state.v, state.omega) == (pytest.approx(0.5), pytest.approx(0.9))
        assert state.x == pytest.approx(0.5 * 0.005, abs=1e-6)  # integrated, not the px of 9
        assert state.t == pytest.approx(1.010)  # the /imu/fused stamp

    def test_the_integrator_reads_the_remapped_gyro(self, controller):
        # A board mounted upside down: the profile flips y and z (F06).
        node = _odom_controller(controller)
        node._ag_sign = [1.0, -1.0, -1.0]
        _feed(node, _s_frame(vx=0.5, gz=0.9))
        _feed(node, _s_frame(vx=0.5, gz=0.9))
        assert node._odometry.state.omega == pytest.approx(-0.9)

    @pytest.mark.parametrize('source', ['firmware', 'integrated'])
    def test_both_sources_publish_odom_integrated(self, controller, source):
        node = _odom_controller(controller, source)
        _feed(node, _s_frame(px=1.25, py=-0.5, yaw=0.6, vx=0.4, gz=0.2))
        _feed(node, _s_frame(px=1.25, py=-0.5, yaw=0.6, vx=0.4, gz=0.2))
        assert len(node.sent) == 1  # the first frame; the next is due in 20 ms
        odom = node.sent[0]
        assert (odom.header.frame_id, odom.child_frame_id) == ('odom', 'base_link')
        if source == 'firmware':
            assert (odom.pose.pose.position.x, odom.pose.pose.position.y) == (1.25, -0.5)
            q = odom.pose.pose.orientation
            assert 2 * math.atan2(q.z, q.w) == pytest.approx(0.6)
            assert (odom.twist.twist.linear.x, odom.twist.twist.angular.z) == (0.4, 0.2)
        else:
            assert odom.pose.pose.position.x == 0.0  # the integrator's first sample

    def test_no_tf_unless_asked(self, controller):
        node = _odom_controller(controller)
        for _ in range(10):
            _feed(node, _s_frame(vx=0.5))
        assert node.sent and node.transforms == []
        node.tf_on()
        for _ in range(10):
            _feed(node, _s_frame(vx=0.5))
        assert node.transforms
        assert node.transforms[-1].header.stamp == node.sent[-1].header.stamp


def _odom_params():
    return yaml.safe_load((ROOT / 'config' / 'odom.yaml').read_text())['controller_node'][
        'ros__parameters'
    ]


def _declared_default(src, key):
    m = re.search(
        rf"declare_(?:parameter|float_param)\((?:self, )?'{re.escape(key)}', ([^)]+)\)", src
    )
    return m and m.group(1)


class TestConfig:
    def test_yaml_keys_are_declared_by_the_controller(self):
        src = CONTROLLER.read_text()
        missing = [k for k in _odom_params() if not _declared_default(src, k)]
        assert missing == []

    def test_yaw_axis_is_body_z(self):
        p = _odom_params()
        assert (p['odometry.yaw_rate_axis'], p['odometry.yaw_rate_sign']) == (2, 1.0)

    def test_controller_launch_loads_odom_yaml(self):
        assert "'odom.yaml'" in (ROOT / 'launch' / 'controller.launch.py').read_text()

    def test_the_code_defaults_are_the_yaml_values(self):
        src = CONTROLLER.read_text()
        for key, value in _odom_params().items():
            assert yaml.safe_load(_declared_default(src, key).replace("'", '"')) == value, key

    def test_neoracer_spec_values(self):
        p = _odom_params()
        assert p['odometry.wheelbase_m'] == SPEC_WHEELBASE
        assert p['odometry.max_steer_deg'] == SPEC_INNER_LOCK_DEG

    def test_firmware_is_the_default_source(self):
        # Plan D4: the default changes only on the on-car acceptance numbers.
        assert _odom_params()['odometry.source'] == 'firmware'

    def test_the_tf_is_off_so_the_ekf_stays_the_only_parent(self):
        assert _odom_params()['odometry.publish_tf'] is False
        assert _declared_default(CONTROLLER.read_text(), 'odometry.publish_tf') == 'False'
        controller_yaml = yaml.safe_load((ROOT / 'config' / 'controller.yaml').read_text())
        assert controller_yaml['controller_node']['ros__parameters']['publish_tf'] is False
        # The autonomy launch's EKF is the odom -> base_link publisher.
        autonomy = (ROOT / 'launch' / 'autonomy.launch.py').read_text()
        assert "'base_link_frame': 'base_link'" in autonomy
        assert "'publish_tf': True" in autonomy

    def test_the_s_frame_feeds_odometry_from_the_published_imu(self):
        src = CONTROLLER.read_text()
        start, end = src.index("if tag == 's':"), src.index("elif tag == 'i':")
        branch = src[start:end]
        assert 'imu = self.pub_imu(data)' in branch
        assert 'self.pub_odom_integrated(data, imu)' in branch
        assert "self._integrate_odometry(stamp_ns, float(data['v_x']), gyro)" in src


class TestTurnCorrection:
    @staticmethod
    def _encoder(v_true, omega, k, wheelbase=0.23):
        delta = math.atan(wheelbase * abs(omega / v_true))
        return v_true * (1 + k * (1 / math.cos(delta) - 1))

    def test_off_by_default(self):
        from neoracer_ros2_driver.odometry import turn_corrected_speed

        assert turn_corrected_speed(0.7, 1.6, 0.0, 0.23, math.radians(30)) == 0.7

    @pytest.mark.parametrize('v_true,omega', [(0.72, 1.70), (0.72, -1.55), (-0.8, 0.4)])
    def test_recovers_the_true_speed(self, v_true, omega):
        from neoracer_ros2_driver.odometry import turn_corrected_speed

        v_enc = self._encoder(v_true, omega, 0.5)
        got = turn_corrected_speed(v_enc, omega, 0.5, 0.23, math.radians(30))
        assert got == pytest.approx(v_true, rel=2e-3)

    def test_full_lock_over_read_is_about_six_percent(self):
        # Reference car, left full lock: curvature 2.37/m at 0.72 m/s.
        v_enc = self._encoder(0.72, 0.72 * 2.37, 0.5)
        assert v_enc / 0.72 - 1 == pytest.approx(0.07, abs=0.005)

    def test_straight_and_slow_are_untouched(self):
        from neoracer_ros2_driver.odometry import turn_corrected_speed

        assert turn_corrected_speed(0.8, 0.0, 0.5, 0.23, math.radians(30)) == 0.8
        assert turn_corrected_speed(0.05, 1.0, 0.5, 0.23, math.radians(30)) == 0.05

    def test_steering_angle_is_capped(self):
        from neoracer_ros2_driver.odometry import turn_corrected_speed

        # A spin in place by hand would read as 90 deg of steer; capped at 30.
        got = turn_corrected_speed(0.2, 5.0, 0.5, 0.23, math.radians(30))
        assert got == pytest.approx(0.2 / (1 + 0.5 * (1 / math.cos(math.radians(30)) - 1)))

    def test_integrator_applies_it(self):
        from neoracer_ros2_driver.odometry import OdomIntegrator

        odo = OdomIntegrator(turn_gain=0.5)
        v_enc = self._encoder(0.72, 1.7, 0.5)
        for i in range(101):
            odo.step(i * 0.01, v_enc, 1.7)
        assert odo.state.v == pytest.approx(0.72, rel=2e-3)

    def test_the_neoracer_wheelbase(self):
        from neoracer_ros2_driver.odometry import turn_corrected_speed

        # The spec's wheelbase in the same correction: curvature 1.5/m, about
        # 23 deg of steer, inside the 30 deg lock (full lock is about 2.0/m).
        v_enc = self._encoder(0.72, 0.72 * 1.5, 0.5, wheelbase=SPEC_WHEELBASE)
        got = turn_corrected_speed(
            v_enc, 0.72 * 1.5, 0.5, SPEC_WHEELBASE, math.radians(SPEC_INNER_LOCK_DEG)
        )
        assert got == pytest.approx(0.72, rel=2e-3)
