"""Tests for the ROS side of controller.py. They import rclpy, so they run on the car."""

import math

from neoracer_ros2_driver.controller import build_battery_state


def test_build_battery_state_from_voltage_alone():
    msg = build_battery_state(11.7, 10.8, 12.6)
    assert msg.voltage == 11.7
    assert math.isnan(msg.current)
    assert abs(msg.percentage - 0.5) < 1e-9
    assert msg.present
    assert msg.power_supply_status == msg.POWER_SUPPLY_STATUS_DISCHARGING
    assert msg.power_supply_technology == msg.POWER_SUPPLY_TECHNOLOGY_LIPO
