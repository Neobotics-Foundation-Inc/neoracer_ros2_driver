"""Unit tests for scripts/analyze_odom.py: run windows, criteria, bag reading."""

import math

from conftest import load_script
import numpy as np
import pytest

DT = 0.02  # /odom/integrated at 50 Hz


@pytest.fixture(scope='module')
def odom():
    return load_script('analyze_odom')


@pytest.fixture
def bags():
    """rosbag2_py and the storage this car records (Humble may lack the MCAP plugin)."""
    rosbag2_py = pytest.importorskip('rosbag2_py')
    if getattr(rosbag2_py, '__file__', None) is None:
        pytest.skip('rosbag2_py is a stand-in here; the round trip needs the real one')
    storage, _ = load_script('neoracer_log').choose_storage(None, None)
    return rosbag2_py, storage


def _writer(rosbag2_py, storage, bag, topics):
    """Open a bag writer with Humble's TopicMetadata (no id field)."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id=storage),
        rosbag2_py.ConverterOptions('cdr', 'cdr'),
    )
    for name, typ in topics:
        writer.create_topic(
            rosbag2_py.TopicMetadata(name=name, type=typ, serialization_format='cdr')
        )
    return writer


def _arc(odom, radius, laps, v=0.5, still=1.0, over_read=0.0, steer=None):
    """Build a run that sits still, drives `laps` around `radius` at speed v, then sits still."""
    t, x, y, yaw, vs, ws = [], [], [], [], [], []
    omega = v / radius if radius else 0.0
    moving = (laps * 2 * math.pi / omega) if omega else laps
    n_still, n_move = round(still / DT), round(moving / DT)
    theta = px = py = 0.0
    for i in range(n_still + n_move + n_still):
        is_moving = n_still <= i < n_still + n_move
        if is_moving:
            mid = theta + 0.5 * omega * DT
            px += v * math.cos(mid) * DT
            py += v * math.sin(mid) * DT
            theta += omega * DT
        t.append(i * DT)
        x.append(px)
        y.append(py)
        yaw.append(math.atan2(math.sin(theta), math.cos(theta)))  # wrapped, as in the quaternion
        vs.append(v * (1 + over_read) if is_moving else 0.0)
        ws.append(omega if is_moving else 0.0)
    steer_t = t if steer is not None else None
    steer_v = [steer] * len(t) if steer is not None else None
    return odom.run_from_arrays(t, x, y, yaw, vs, ws, steer_t, steer_v)


def _status(criteria, name_start):
    return next(c.status for c in criteria if c.name.startswith(name_start))


class TestWindow:
    def test_trims_the_still_ends(self, odom):
        run = _arc(odom, 1.0, 0.25, still=2.0).window()
        assert run.v.min() > 0
        assert run.t[0] == pytest.approx(2.0, abs=DT)

    def test_never_moving_is_an_error(self, odom):
        run = odom.run_from_arrays([0, 1], [0, 0], [0, 0], [0, 0], [0, 0], [0, 0])
        with pytest.raises(ValueError, match='never turned'):
            run.window()

    def test_yaw_is_unwrapped(self, odom):
        run = _arc(odom, 1.0, 1.5)
        assert math.degrees(run.theta[-1] - run.theta[0]) == pytest.approx(540.0, abs=1.0)


class TestCircle:
    def test_one_lap_passes(self, odom):
        crit = odom.circle_criteria(_arc(odom, 0.8, 1.0))
        assert _status(crit, 'heading closure') == 'PASS'

    def test_seven_degrees_over_fails(self, odom):
        crit = odom.circle_criteria(_arc(odom, 0.8, 1.0 + 7.0 / 360.0))
        assert _status(crit, 'heading closure') == 'FAIL'

    def test_right_turns_count_too(self, odom):
        run = _arc(odom, 0.8, 1.0)
        run.theta = -run.theta
        assert _status(odom.circle_criteria(run), 'heading closure') == 'PASS'


class TestStraight:
    def _line(self, odom, length, v=0.5):
        t = np.arange(0, abs(length) / abs(v) + 2.0, DT)
        x = np.clip((t - 1.0) * v, min(0, length), max(0, length))
        moving = (t > 1.0) & (np.abs(x) < abs(length))
        vs = np.where(moving, v, 0.0)
        z = np.zeros_like(t)
        return odom.run_from_arrays(
            t.tolist(), x.tolist(), z.tolist(), z.tolist(), vs.tolist(), z.tolist()
        )

    def test_within_five_percent_passes(self, odom):
        crit = odom.straight_criteria(self._line(odom, 3.09), 3.0)
        assert _status(crit, 'distance, forward') == 'PASS'

    def test_ten_percent_long_fails(self, odom):
        crit = odom.straight_criteria(self._line(odom, 3.3), 3.0)
        assert _status(crit, 'distance, forward') == 'FAIL'

    def test_reverse_is_labeled(self, odom):
        crit = odom.straight_criteria(self._line(odom, -3.0, v=-0.5), 3.0)
        assert _status(crit, 'distance, reverse') == 'PASS'


class TestTurn:
    def test_exact_encoder_passes(self, odom):
        crit = odom.turn_criteria(_arc(odom, 0.75, 2.0, steer=1.0), 0.75)
        assert _status(crit, 'turning bias, left') == 'PASS'
        steer = next(c for c in crit if c.name == 'steering command')
        assert steer.measured == '+1.000'

    def test_six_percent_over_read_warns(self, odom):
        crit = odom.turn_criteria(_arc(odom, 0.75, 2.0, over_read=0.06), 0.75)
        bias = next(c for c in crit if c.name.startswith('turning bias'))
        assert bias.status == 'WARN'
        assert bias.measured.startswith('+6.0%')

    def test_curvature_matches_the_radius(self, odom):
        crit = odom.turn_criteria(_arc(odom, 0.75, 1.0), 0.75)
        curve = next(c for c in crit if c.name == 'curvature')
        assert 'radius 0.750 m' in curve.measured


class TestCpu:
    def test_the_controller_runs_the_integrator(self, odom):
        assert odom.CONTROLLER_PATH == '/lib/neoracer_ros2_driver/controller'

    def test_criteria(self, odom):
        assert odom.cpu_criteria(50.0, 40.0)[0].status == 'PASS'
        assert odom.cpu_criteria(55.0, 40.0)[0].status == 'FAIL'
        assert odom.cpu_criteria(50.0, None)[0].status == 'INFO'

    def test_own_process_cpu_reads(self, odom):
        import os
        from pathlib import Path

        if not Path('/proc/self/stat').exists():
            pytest.skip('reads /proc, which only Linux has')
        assert odom.cpu_seconds(os.getpid()) > 0


class TestReadBag:
    def test_round_trip_through_a_bag(self, odom, bags, tmp_path):
        rosbag2_py, storage = bags
        from ackermann_msgs.msg import AckermannDriveStamped
        from nav_msgs.msg import Odometry
        from rclpy.serialization import serialize_message

        bag = tmp_path / 'run'
        writer = _writer(
            rosbag2_py,
            storage,
            bag,
            (
                ('/odom/integrated', 'nav_msgs/msg/Odometry'),
                ('/motor', 'ackermann_msgs/msg/AckermannDriveStamped'),
            ),
        )
        for i in range(50):
            o = Odometry()
            o.header.stamp.sec, o.header.stamp.nanosec = 100 + i // 50, (i % 50) * 20_000_000
            o.pose.pose.position.x = 0.01 * i
            o.pose.pose.orientation.z, o.pose.pose.orientation.w = math.sin(0.1), math.cos(0.1)
            o.twist.twist.linear.x = 0.5
            writer.write('/odom/integrated', serialize_message(o), 1_000_000 * i)
            m = AckermannDriveStamped()
            m.header.stamp = o.header.stamp
            m.drive.steering_angle = 0.25
            writer.write('/motor', serialize_message(m), 1_000_000 * i)
        del writer

        run = odom.read_bag(bag)
        assert len(run.t) == 50
        assert run.x[-1] == pytest.approx(0.49)
        assert run.theta[0] == pytest.approx(0.2)
        assert run.steer.tolist() == [0.25] * 50

    def test_bag_without_odometry_is_explained(self, odom, bags, tmp_path):
        rosbag2_py, storage = bags
        bag = tmp_path / 'empty'
        writer = _writer(rosbag2_py, storage, bag, (('/scan', 'sensor_msgs/msg/LaserScan'),))
        del writer
        with pytest.raises(ValueError, match='/odom/integrated'):
            odom.read_bag(bag)

    def test_the_firmware_pose_and_the_gyro_are_read_too(self, odom, bags, tmp_path):
        rosbag2_py, storage = bags
        from nav_msgs.msg import Odometry
        from rclpy.serialization import serialize_message
        from sensor_msgs.msg import Imu

        bag = tmp_path / 'both'
        writer = _writer(
            rosbag2_py,
            storage,
            bag,
            (
                ('/odom/integrated', 'nav_msgs/msg/Odometry'),
                ('/odom', 'nav_msgs/msg/Odometry'),
                ('/imu/fused', 'sensor_msgs/msg/Imu'),
            ),
        )
        for i in range(10):
            for topic in ('/odom/integrated', '/odom'):
                o = Odometry()
                o.header.stamp.sec = 100 + i
                o.pose.pose.position.x = 0.1 * i
                o.pose.pose.orientation.w = 1.0
                o.twist.twist.linear.x = 0.1
                writer.write(topic, serialize_message(o), 1_000_000 * i)
            g = Imu()
            g.header.stamp.sec = 100 + i
            g.angular_velocity.z = 0.25
            writer.write('/imu/fused', serialize_message(g), 1_000_000 * i)
        del writer

        data = odom.read_topics(bag)
        assert len(data.firmware) == 10 and len(data.gyro) == 10
        assert data.gyro[0][3] == pytest.approx(0.25)
        assert odom.source_in_use(data.run, odom.firmware_run(data.firmware)) == 'firmware'


def _rows(length, v=0.5, still=1.0, dx=1.0):
    """/odom rows [t, x, y, yaw, v, omega] for a straight run: still, `length` m, still."""
    rows, t, x = [], 0.0, 0.0
    moving_s = length / v
    while t < still + moving_s + still:
        moving = still <= t < still + moving_s
        if moving:
            x += v * DT * dx
        rows.append([t, x, 0.0, 0.0, v if moving else 0.0, 0.0])
        t += DT
    return rows


class TestOtherSource:
    def test_a_run_that_matches_odom_is_the_firmware_source(self, odom):
        rows = _rows(2.0)
        firmware = odom.firmware_run(rows)
        assert odom.source_in_use(firmware, firmware) == 'firmware'

    def test_a_run_that_drifts_from_odom_is_the_integrated_source(self, odom):
        firmware = odom.firmware_run(_rows(2.0))
        integrated = odom.firmware_run(_rows(2.0, dx=0.97))
        assert odom.source_in_use(integrated, firmware) == 'integrated'

    def test_the_bias_comes_from_the_rest_before_the_wheels_turn(self, odom):
        rows = _rows(1.0)
        rates = [(row[0], 0.01 if row[0] < 1.0 else 0.5) for row in rows]
        assert odom.rest_bias(rows, rates) == pytest.approx(0.01)

    def test_without_odom_the_source_is_unknown(self, odom):
        data = odom.BagData(odom.firmware_run(_rows(1.0)), [], [])
        assert odom.other_source(data)[:2] == ('unknown', '')

    def test_on_the_integrated_source_the_other_is_the_firmware_pose(self, odom):
        firmware_rows = _rows(2.0)
        integrated = odom.firmware_run(_rows(2.0, dx=0.97))
        data = odom.BagData(integrated, firmware_rows, [])
        in_use, other, run = odom.other_source(data)
        assert (in_use, other) == ('integrated', 'firmware')
        assert run.x[-1] == pytest.approx(firmware_rows[-1][1])

    def test_on_the_firmware_source_the_other_is_the_replayed_estimate(self, odom):
        pytest.importorskip('rclpy')
        rows = _rows(2.0)
        gyro = [(row[0], 0.0, 0.0, 0.0) for row in rows]
        data = odom.BagData(odom.firmware_run(rows), rows, gyro)
        params = {'odometry.speed_scale': 0.5, 'odometry.yaw_rate_scale': 1.0}
        in_use, other, run = odom.other_source(data, params)
        assert (in_use, other) == ('firmware', 'integrated')
        # The replay applies the profile's speed scale to the firmware's speed.
        assert run.window().path_length() == pytest.approx(1.0, rel=0.02)

    def test_the_other_source_is_shown_not_graded(self, odom):
        main = odom.straight_criteria(odom.firmware_run(_rows(3.3)), 3.0)[0]
        assert main.status == 'FAIL'
        info = odom.as_info(main, 'firmware distance')
        assert (info.name, info.status, info.measured) == (
            'firmware distance',
            'INFO',
            main.measured,
        )


class TestReport:
    def test_straight_grades_the_source_in_use_and_shows_the_other(
        self, odom, monkeypatch, tmp_path, capsys
    ):
        # The integrated source in use, 2 percent long; the firmware pose 6 percent long.
        firmware_rows = _rows(3.18)
        integrated = odom.firmware_run(_rows(3.06))
        data = odom.BagData(integrated, firmware_rows, [])
        monkeypatch.setattr(odom, 'resolve_bag', lambda arg: tmp_path)
        monkeypatch.setattr(odom, 'read_topics', lambda bag: data)
        assert odom.main(['straight', '--tape-m', '3']) == 0
        out = capsys.readouterr().out
        assert '=== straight: ' in out
        assert 'INFO  odometry source    integrated' in out
        assert 'PASS  distance, forward' in out
        assert 'INFO  firmware distance' in out and '(+6.0%)' in out
        assert 'Result: PASS' in out
