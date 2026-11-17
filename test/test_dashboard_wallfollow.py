"""
Scan-geometry and tuning tests for the wall follow dashboard (wallfollow.py).

Runs without a ROS graph or a lidar: LaserScan messages are built by hand at
the NeoRacer's LakiBeam pattern (1440 points over 360 degrees, scan zero at
the nose, the rear 90 degrees blank) and at a 270 degree pattern, so the
angle mapping is exercised on both.
"""

import math

from conftest import load_script
import pytest
from sensor_msgs.msg import LaserScan
import yaml

LAKIBEAM_POINTS = 1440


@pytest.fixture(scope='module')
def wf():
    return load_script('wallfollow')


@pytest.fixture
def node_cls(wf):
    return wf.WallFollowNode


def make_scan(points=LAKIBEAM_POINTS, span_deg=360.0, fill=5.0):
    """Build a LaserScan with `points` rays evenly covering `span_deg`, centered on raw 0."""
    msg = LaserScan()
    msg.angle_min = math.radians(-span_deg / 2)
    msg.angle_max = math.radians(span_deg / 2)
    msg.angle_increment = (msg.angle_max - msg.angle_min) / points
    msg.range_min = 0.05
    msg.range_max = 25.0
    msg.ranges = [fill] * points
    return msg


def place(msg, raw_deg, value):
    """Put a return of `value` at raw angle `raw_deg`."""
    i = round((math.radians(raw_deg) - msg.angle_min) / msg.angle_increment)
    msg.ranges[i % len(msg.ranges)] = value
    return i % len(msg.ranges)


def place_wedge(msg, raw_deg, half_deg, value):
    """
    Fill every ray within `half_deg` of `raw_deg` with `value`.

    A windowed mean there sees only `value`. Returns the center index.
    """
    n = len(msg.ranges)
    centre = round((math.radians(raw_deg) - msg.angle_min) / msg.angle_increment) % n
    half = round(math.radians(half_deg) / msg.angle_increment)
    for k in range(-half, half + 1):
        msg.ranges[(centre + k) % n] = value
    return centre


def frames_dir(tmp_path, yaw_deg):
    """Write a config folder whose frames.yaml has the lidar at `yaw_deg`."""
    (tmp_path / 'frames.yaml').write_text(f'laser_x_mm: 99.62\nlaser_yaw_deg: {yaw_deg}\n')
    return tmp_path


class TestScanPattern:
    def test_lakibeam_scan_is_1440_points_over_360_degrees(self):
        msg = make_scan()
        assert len(msg.ranges) == 1440
        assert math.degrees(msg.angle_increment) == pytest.approx(0.25)

    def test_the_fleet_mount_puts_the_nose_at_raw_0(self, wf):
        # config/frames.yaml ships laser_yaw_deg 0: the LakiBeam faces forward.
        config = wf.BASE.parents[2] / 'config'
        assert wf.lidar_mount_yaw_deg(config) == 0.0

    def test_the_mount_yaw_comes_from_frames_yaml(self, wf, tmp_path):
        # A lidar turned 3 deg left sees the nose at raw -3.
        assert wf.lidar_mount_yaw_deg(frames_dir(tmp_path, 3.0)) == pytest.approx(-3.0)

    def test_the_car_profile_yaw_wins(self, wf, tmp_path, car_profile_dir):
        # The lidar calibration writes laser_yaw_deg to the profile's params/frames.yaml.
        (car_profile_dir / 'params').mkdir(parents=True)
        (car_profile_dir / 'params' / 'frames.yaml').write_text('laser_yaw_deg: -2.5\n')
        assert wf.lidar_mount_yaw_deg(frames_dir(tmp_path, 0.0)) == pytest.approx(2.5)

    def test_an_unreadable_config_reads_as_no_yaw(self, wf, tmp_path):
        assert wf.lidar_mount_yaw_deg(tmp_path / 'missing') == 0.0


class TestAngleMapping:
    @pytest.fixture(autouse=True)
    def fleet_mount(self, wf, monkeypatch):
        monkeypatch.setattr(wf, 'LIDAR_MOUNT_YAW_DEG', 0.0)

    @pytest.mark.parametrize('student', [0, 15, 45, 90, 135, 180, -15, -45, -90, -135])
    def test_index_and_angle_round_trip(self, node_cls, student):
        msg = make_scan()
        back = node_cls._student_deg(msg, node_cls._index_at(msg, student))
        err = (back - student + 180) % 360 - 180
        assert abs(err) <= math.degrees(msg.angle_increment)

    def test_object_in_front_reads_as_zero_degrees(self, node_cls):
        # The LakiBeam's raw 0 is the nose.
        msg = make_scan()
        i = place_wedge(msg, 0.0, 5.0, 1.0)
        assert node_cls._student_deg(msg, i) == pytest.approx(0.0, abs=0.5)
        assert node_cls._mean_at(node_cls, msg, 0, 4.0) == pytest.approx(1.0, abs=0.01)

    def test_object_off_the_right_reads_as_positive(self, node_cls):
        # Scan angles turn counterclockwise, so the car's right is raw -90.
        msg = make_scan()
        i = place_wedge(msg, -90.0, 5.0, 2.0)
        assert node_cls._student_deg(msg, i) == pytest.approx(90.0, abs=0.5)
        assert node_cls._mean_at(node_cls, msg, 90, 4.0) == pytest.approx(2.0, abs=0.01)

    def test_left_and_right_are_not_mirrored(self, node_cls):
        # A handedness flip would swap these two and steer the car into the
        # wall it is trying to hold away from.
        msg = make_scan()
        place_wedge(msg, -90.0, 5.0, 2.0)  # car's right
        place_wedge(msg, 90.0, 5.0, 3.0)  # car's left
        assert node_cls._mean_at(node_cls, msg, 90, 4.0) == pytest.approx(2.0, abs=0.01)
        assert node_cls._mean_at(node_cls, msg, -90, 4.0) == pytest.approx(3.0, abs=0.01)

    def test_a_measured_yaw_turns_the_view(self, wf, node_cls, monkeypatch):
        # laser_yaw_deg 3: the lidar is turned 3 deg left, so the nose is raw -3.
        monkeypatch.setattr(wf, 'LIDAR_MOUNT_YAW_DEG', -3.0)
        msg = make_scan()
        i = place_wedge(msg, -3.0, 2.0, 1.0)
        assert node_cls._student_deg(msg, i) == pytest.approx(0.0, abs=0.5)
        assert node_cls._mean_at(node_cls, msg, 0, 2.0) == pytest.approx(1.0, abs=0.01)


class TestOtherScanPatterns:
    def test_270_degree_scan_still_maps(self, wf, node_cls, monkeypatch):
        # Nothing here assumes a sweep width; indices come from the message.
        monkeypatch.setattr(wf, 'LIDAR_MOUNT_YAW_DEG', 0.0)
        msg = make_scan(points=1080, span_deg=270.0)
        i = place_wedge(msg, -90.0, 5.0, 2.0)
        assert node_cls._student_deg(msg, i) == pytest.approx(90.0, abs=0.5)
        assert node_cls._mean_at(node_cls, msg, 90, 4.0) == pytest.approx(2.0, abs=0.01)

    def test_upstreams_aft_facing_lidar_maps_with_180(self, wf, node_cls, monkeypatch):
        monkeypatch.setattr(wf, 'LIDAR_MOUNT_YAW_DEG', 180.0)
        msg = make_scan(points=1080)
        i = place(msg, 180.0, 1.5)
        assert node_cls._student_deg(msg, i) == pytest.approx(0.0, abs=0.5)


class TestWindowAveraging:
    @pytest.fixture(autouse=True)
    def fleet_mount(self, wf, monkeypatch):
        monkeypatch.setattr(wf, 'LIDAR_MOUNT_YAW_DEG', 0.0)

    def test_window_averages_over_its_cone(self, node_cls):
        msg = make_scan(fill=4.0)
        for raw in (-0.5, -0.25, 0.0, 0.25, 0.5):
            place(msg, raw, 1.0)
        # A 10 degree window at the nose spans about 40 rays, only 5 of which
        # carry the near return, so the mean sits between the two values.
        got = node_cls._mean_at(node_cls, msg, 0, 10.0)
        assert 1.0 < got < 4.0

    def test_no_return_reads_as_open_space_not_zero(self, node_cls):
        msg = make_scan(fill=float('inf'))
        assert node_cls._mean_at(node_cls, msg, 0, 5.0) == msg.range_max

    def test_the_blank_rear_reads_as_open_space(self, node_cls):
        # The LakiBeam's rear 90 degrees always read inf.
        msg = make_scan(fill=2.0)
        for i in range(len(msg.ranges)):
            if abs(math.degrees(msg.angle_min + i * msg.angle_increment)) > 135.0:
                msg.ranges[i] = float('inf')
        assert node_cls._mean_at(node_cls, msg, 180, 10.0) == msg.range_max
        assert node_cls._mean_at(node_cls, msg, 90, 10.0) == pytest.approx(2.0)


class TestTuning:
    @pytest.fixture
    def params(self, wf, monkeypatch, tmp_path):
        monkeypatch.setattr(wf, 'YAML_PATH', tmp_path / 'profile' / 'wallfollow.yaml')
        wf._params.clear()
        wf._marks.clear()
        wf.load_params()
        return wf._params

    def test_shipped_yaml_cannot_drive(self, wf):
        assert yaml.safe_load(wf.DEFAULTS_PATH.read_text())['speed'] == 0.0

    def test_save_writes_the_profile_not_the_shipped_yaml(self, wf, params):
        shipped = wf.DEFAULTS_PATH.read_text()
        params.update(speed=0.3, kp=0.75)
        wf.save_params()
        assert wf.DEFAULTS_PATH.read_text() == shipped
        text = wf.YAML_PATH.read_text()
        assert 'speed: 0.3' in text and 'kp: 0.75' in text
        assert 'Raise this if the car wobbles' in text  # comments survive
        wf._params.clear()
        wf.load_params()
        assert wf._params['speed'] == 0.3 and wf._params['kp'] == 0.75

    def test_the_profile_loads_over_the_defaults(self, wf, params):
        wf.YAML_PATH.parent.mkdir(parents=True)
        wf.YAML_PATH.write_text('mode: 0\nlook_angle: 60.0\n')
        wf.load_params()
        assert wf._params['mode'] == 0
        assert wf._params['look_angle'] == 60.0
        assert wf._params['width'] == 80.0  # from the defaults

    def test_the_speed_is_capped(self, wf):
        assert wf.set_max_speed(4.0) == wf.MAX_SPEED
        assert wf.set_max_speed(-1.0) == 0.0
