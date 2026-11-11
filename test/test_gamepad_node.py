"""Unit tests for gamepad_node.joy_to_drive (Joy axes to a normalized drive command)."""

import pytest
from neoracer_ros2_driver.gamepad_node import joy_to_drive

# FlySky through the ESP32, laid out like a gamepad: axis 1 throttles, axis 3 steers.
AXES = {'throttle_axis': 1, 'steering_axis': 3, 'throttle_sign': 1, 'steering_sign': 1}


def _axes(throttle=0.0, steering=0.0):
    a = [0.0] * 8
    a[1], a[3] = throttle, steering
    return a


class TestJoyToDrive:
    def test_centered_sticks_are_zero(self):
        assert joy_to_drive(_axes(), **AXES) == (0.0, 0.0)

    def test_axes_map_to_speed_and_steering(self):
        assert joy_to_drive(_axes(0.5, -0.25), **AXES) == (0.5, -0.25)

    def test_signs_flip_each_axis(self):
        cfg = {**AXES, 'throttle_sign': -1, 'steering_sign': -1}
        assert joy_to_drive(_axes(0.5, -0.25), **cfg) == (-0.5, 0.25)

    def test_output_is_clamped(self):
        cfg = {**AXES, 'throttle_sign': 2, 'steering_sign': 2}
        assert joy_to_drive(_axes(0.8, -0.9), **cfg) == (1.0, -1.0)

    def test_short_frame_is_ignored(self):
        assert joy_to_drive([0.0, 0.5, 0.0], **AXES) is None

    def test_frame_just_long_enough(self):
        assert joy_to_drive([0.0, 0.5, 0.0, 0.1], **AXES) == pytest.approx((0.5, 0.1))

    def test_custom_axis_indices(self):
        assert joy_to_drive([0.3, 0.0, -0.6], 2, 0, 1, 1) == pytest.approx((-0.6, 0.3))
