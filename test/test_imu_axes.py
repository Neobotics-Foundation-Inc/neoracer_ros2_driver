"""REP-103 IMU axes: the shipped remaps applied to the dev car's readings."""

import math
from pathlib import Path

from neoracer_ros2_driver import controller_lib as cl
import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent
PARAMS = yaml.safe_load((ROOT / 'config' / 'controller.yaml').read_text())['controller_node'][
    'ros__parameters'
]
AG = (PARAMS['imu.accel_gyro_axis_order'], PARAMS['imu.accel_gyro_axis_sign'])
MAG = (PARAMS['imu.mag_axis_order'], PARAMS['imu.mag_axis_sign'])

# /imu/fused on the dev car, 2026-11-12 (Ansh). Accel in m/s^2, gyro in rad/s.
CAR_LEVEL = (-0.0311, -0.0589, 10.003)
CAR_NOSE_DOWN = (-4.5001, -0.0068, 8.9727)
CAR_LEFT_SIDE_DOWN = (-0.8436, -4.9201, 8.3644)
# Near the peak of a turn to the left, counter-clockwise seen from above; the
# x and y rates are the car wobbling in a hand.
CAR_TURNING_LEFT_GYRO = (-0.2431, -0.3087, 0.4559)
GRAVITY = 9.81


def _det(order, sign) -> float:
    m = [[0.0] * 3 for _ in range(3)]
    for i, (j, s) in enumerate(zip(order, sign, strict=True)):
        m[i][j] = float(s)
    return (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def test_gravity_is_up_at_rest():
    a = cl.remap_axes(CAR_LEVEL, *AG)
    assert a[2] == pytest.approx(10.003) and abs(a[0]) < 1 and abs(a[1]) < 1


def test_nose_down_reads_negative_x():
    a = cl.remap_axes(CAR_NOSE_DOWN, *AG)
    assert a[0] < -4
    # About 27 degrees of pitch, as held on the bench.
    assert 20 < math.degrees(math.atan2(-a[0], a[2])) < 35


def test_left_side_down_reads_negative_y():
    a = cl.remap_axes(CAR_LEFT_SIDE_DOWN, *AG)
    assert a[1] < -4


def test_a_left_turn_reads_positive_yaw_rate():
    assert cl.remap_axes(CAR_TURNING_LEFT_GYRO, *AG)[2] > 0.3


def test_gravity_magnitude_is_within_three_percent():
    # About 2 percent high when level; no accelerometer scale is applied.
    for reading in (CAR_LEVEL, CAR_NOSE_DOWN, CAR_LEFT_SIDE_DOWN):
        norm = math.sqrt(sum(v * v for v in cl.remap_axes(reading, *AG)))
        assert abs(norm / GRAVITY - 1) < 0.03, reading


def test_shipped_remaps_are_the_identity():
    # The firmware's axes are REP-103 already, so the firmware's orientation stays valid.
    assert cl.is_identity_remap(*AG)
    assert cl.is_identity_remap(*MAG)
    assert _det(*AG) == pytest.approx(1.0) and _det(*MAG) == pytest.approx(1.0)


def test_remap_reorders_flips_and_scales():
    assert cl.remap_axes((1.0, 2.0, 3.0), (2, 0, 1), (1.0, -1.0, 1.0), 2.0) == (6.0, -2.0, 4.0)


def test_a_board_mounted_upside_down_is_fixed_by_the_profile():
    # A board turned 180 degrees about x reads (x, -y, -z); the remap
    # [0, 1, 2] with signs [1, -1, -1] is a rotation that undoes it.
    order, sign = (0, 1, 2), (1.0, -1.0, -1.0)
    upside_down = (CAR_LEVEL[0], -CAR_LEVEL[1], -CAR_LEVEL[2])
    assert cl.remap_axes(upside_down, order, sign) == pytest.approx(CAR_LEVEL)
    assert _det(order, sign) == pytest.approx(1.0)
    assert not cl.is_identity_remap(order, sign)


def test_integer_signs_from_a_hand_written_profile_work():
    assert cl.axis_remap_problem([0, 1, 2], [1, 1, -1]) == ''
    assert cl.remap_axes(CAR_LEVEL, [0, 1, 2], [1, 1, 1]) == pytest.approx(CAR_LEVEL)


@pytest.mark.parametrize(
    'order,sign,word',
    [
        ([0, 0, 2], [1.0, 1.0, 1.0], 'permutation'),
        ([0, 1], [1.0, 1.0, 1.0], 'permutation'),
        ([0, 1, 2], [1.0, 2.0, 1.0], 'each 1.0 or -1.0'),
        ([0, 1, 2], [1.0, 1.0], 'each 1.0 or -1.0'),
        (['x', 1, 2], [1.0, 1.0, 1.0], 'lists of numbers'),
    ],
)
def test_a_bad_remap_is_reported(order, sign, word):
    assert word in cl.axis_remap_problem(order, sign)


def test_the_controller_applies_the_remap_to_imu_and_mag():
    # controller.py imports rclpy, so check its source off the car.
    text = (ROOT / 'neoracer_ros2_driver' / 'controller.py').read_text()
    assert "self._declare_axis_remap('imu.accel_gyro_axis')" in text
    assert "self._declare_axis_remap('imu.mag_axis')" in text
    assert text.count('controller_lib.remap_axes(') == 3
    assert 'orientation_covariance[0] = -1.0' in text
