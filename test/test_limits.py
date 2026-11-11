"""Unit tests for neoracer_ros2_driver.limits."""

from neoracer_ros2_driver.limits import clamp


class TestClamp:
    def test_within(self):
        assert clamp(0.3) == 0.3

    def test_saturates(self):
        assert clamp(2.0) == 1.0
        assert clamp(-2.0) == -1.0

    def test_custom_bounds(self):
        assert clamp(5.0, 0.0, 2.5) == 2.5
        assert clamp(-1.0, 0.0, 2.5) == 0.0
