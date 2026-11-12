"""Unit tests for throttle math helpers and the shipped throttle caps."""

from pathlib import Path
from unittest import mock

from ackermann_msgs.msg import AckermannDriveStamped
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from neoracer_ros2_driver.throttle_node import (
    cap_speed,
    scale_speed,
    scale_steering,
    status_mode,
    ThrottleNode,
)
import pytest
import yaml

CONFIG = Path(__file__).resolve().parent.parent / 'config'
# The mapping speed the cap is meant to give, in m/s (plan D12).
MAPPING_SPEED_MPS = 0.5


def _params(name: str, node: str) -> dict:
    return yaml.safe_load((CONFIG / name).read_text())[node]['ros__parameters']


class TestScaleSpeed:
    def test_zero_passes_through(self):
        assert scale_speed(0.0, 0.5, 0.6) == 0.0

    def test_full_forward(self):
        assert scale_speed(1.0, 0.5, 0.6) == pytest.approx(0.5)

    def test_full_reverse(self):
        assert scale_speed(-1.0, 0.5, 0.6) == pytest.approx(-0.6)

    def test_half_forward(self):
        assert scale_speed(0.5, 0.5, 0.6) == pytest.approx(0.25)

    def test_clamp_above_one(self):
        assert scale_speed(2.5, 0.5, 0.6) == pytest.approx(0.5)

    def test_clamp_below_neg_one(self):
        assert scale_speed(-2.5, 0.5, 0.6) == pytest.approx(-0.6)

    def test_asymmetric_caps(self):
        # Reverse cap should apply only when speed < 0
        assert scale_speed(1.0, 0.17, 0.24) == pytest.approx(0.17)
        assert scale_speed(-1.0, 0.17, 0.24) == pytest.approx(-0.24)


class TestScaleSteering:
    def test_zero_passes_through(self):
        assert scale_steering(0.0, 0.625) == 0.0

    def test_full_left(self):
        assert scale_steering(1.0, 0.625) == pytest.approx(0.625)

    def test_full_right(self):
        assert scale_steering(-1.0, 0.625) == pytest.approx(-0.625)

    def test_clamp_above_one(self):
        assert scale_steering(5.0, 0.625) == pytest.approx(0.625)

    def test_clamp_below_neg_one(self):
        assert scale_steering(-5.0, 0.625) == pytest.approx(-0.625)


class TestMappingSpeedCap:
    @pytest.mark.parametrize(
        'speed,expected',
        [(1.0, 0.083), (-0.4, -0.083), (0.05, 0.05), (0.005, 0.005), (0.0, 0.0)],
    )
    def test_clips_and_keeps_smaller_commands(self, speed, expected):
        assert cap_speed(speed, 0.083) == pytest.approx(expected)

    def test_mode_from_status(self):
        msg = DiagnosticStatus(
            values=[KeyValue(key='map', value='room'), KeyValue(key='mode', value='mapping')]
        )
        assert status_mode(msg) == 'mapping'
        assert status_mode(DiagnosticStatus()) == ''


class TestMappingSteeringCap:
    @pytest.mark.parametrize(
        'angle,expected', [(1.0, 0.5), (-1.0, -0.5), (0.3, 0.3), (-0.5, -0.5), (0.0, 0.0)]
    )
    def test_clips_large_commands_and_keeps_small_ones(self, angle, expected):
        assert scale_steering(angle, 1.0, cap=0.5) == pytest.approx(expected)

    def test_no_cap_by_default(self):
        assert scale_steering(1.0, 1.0) == 1.0


class TestCapThroughDirectionCaps:
    """The mapping cap applies on the /motor side, after the per-direction caps."""

    @staticmethod
    def _motor(speed: float, max_fwd: float, max_back: float, cap: float = 0.083) -> float:
        return cap_speed(scale_speed(speed, max_fwd, max_back), cap)

    @pytest.mark.parametrize('max_fwd', [1.0, 0.17])
    def test_full_forward_lands_on_the_cap(self, max_fwd):
        assert self._motor(1.0, max_fwd, 0.24) == pytest.approx(0.083)

    @pytest.mark.parametrize('max_back', [1.0, 0.24])
    def test_full_reverse_lands_on_the_cap(self, max_back):
        assert self._motor(-1.0, 1.0, max_back) == pytest.approx(-0.083)

    def test_a_command_under_the_cap_passes_unchanged(self):
        # 0.4 x 0.17 = 0.068 and -0.3 x 0.24 = -0.072, both inside 0.083.
        assert self._motor(0.4, 0.17, 0.24) == pytest.approx(0.068)
        assert self._motor(-0.3, 0.17, 0.24) == pytest.approx(-0.072)

    def test_the_cap_does_not_raise_a_lower_direction_cap(self):
        assert self._motor(1.0, 0.05, 0.24, cap=0.083) == pytest.approx(0.05)


class TestShippedConfig:
    def test_mapping_cap_gives_half_a_metre_per_second(self):
        cap = _params('throttle.yaml', 'throttle_node')['mapping_speed_cap']
        max_speed_mps = _params('controller.yaml', 'controller_node')['max_speed_mps']
        assert cap * max_speed_mps == pytest.approx(MAPPING_SPEED_MPS, abs=0.01)

    def test_mapping_steering_cap_ships_off(self):
        assert _params('throttle.yaml', 'throttle_node')['mapping_steering_cap'] == 1.0

    def test_no_steering_trim_in_the_driver(self):
        # Plan D11: the trim lives in the ESP32 firmware (`trim set`).
        assert 'steering_trim' not in _params('throttle.yaml', 'throttle_node')
        assert _params('controller.yaml', 'controller_node')['steering_trim_deg'] == 0.0


class TestThrottleNodeCallbacks:
    """The mapping state and the caps through the node's own callbacks."""

    @staticmethod
    def _node(mapping_speed_cap: float = 0.083, mapping_steering_cap: float = 1.0):
        node = ThrottleNode.__new__(ThrottleNode)
        node._max_fwd, node._max_back, node._max_steer = 1.0, 1.0, 0.625
        node._mapping_cap, node._mapping_steer = mapping_speed_cap, mapping_steering_cap
        node._mapping = False
        node._pub = mock.Mock()
        node.logger = mock.Mock()
        node.get_logger = lambda: node.logger
        return node

    @staticmethod
    def _status(node, **values) -> None:
        pairs = [KeyValue(key=k, value=v) for k, v in values.items()]
        node._status_cb(DiagnosticStatus(values=pairs))

    @staticmethod
    def _drive(node, speed: float, steering: float) -> tuple:
        msg = AckermannDriveStamped()
        msg.drive.speed, msg.drive.steering_angle = speed, steering
        node._drive_cb(msg)
        out = node._pub.publish.call_args[0][0].drive
        return out.speed, out.steering_angle

    def test_no_status_means_no_caps(self):
        node = self._node()
        assert self._drive(node, 1.0, 1.0) == pytest.approx((1.0, 0.625))
        assert self._drive(node, -1.0, -1.0) == pytest.approx((-1.0, -0.625))

    def test_mapping_caps_both_directions_and_lifts_on_off(self):
        node = self._node(mapping_steering_cap=0.3)
        self._status(node, mode='mapping')
        assert self._drive(node, 1.0, 1.0) == pytest.approx((0.083, 0.3))
        assert self._drive(node, -1.0, -1.0) == pytest.approx((-0.083, -0.3))
        self._status(node, mode='off')
        assert self._drive(node, 1.0, 1.0) == pytest.approx((1.0, 0.625))

    def test_a_status_without_a_mode_lifts_the_caps(self):
        node = self._node()
        self._status(node, mode='mapping')
        self._status(node, map='room')
        assert self._drive(node, 1.0, 0.0) == pytest.approx((1.0, 0.0))

    def test_the_caps_line_is_logged_once_per_change(self):
        node = self._node()
        for mode in ('mapping', 'mapping', 'localization', 'off'):
            self._status(node, mode=mode)
        lines = [c.args[0] for c in node.logger.info.call_args_list]
        assert lines == [
            'Mapping caps (speed 0.083, steering 1) on',
            'Mapping caps (speed 0.083, steering 1) off',
        ]
