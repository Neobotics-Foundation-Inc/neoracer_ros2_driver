#!/usr/bin/env python3

"""
Full Neoracer teleop stack.

Always-on control pipeline (FlySky RC and autonomy both flow through it):

    controller (ESP32 bridge)  -- publishes /joy, /imu/fused, /odom; subscribes /motor
        |  /joy
        v
    gamepad_node               -- /joy -> /gamepad_drive
        |                          (student autonomy publishes /drive)
        v
    mux_node                   -- /gamepad_drive (LB) or /drive (RB) -> /mux_out
        |
        v
    throttle_node              -- applies speed/steer caps, /mux_out -> /motor
        |  /motor
        v
    controller -> ESP32 serial "v <m/s> <deg>"

Optional sensors/display, each gated by ``<name>_enable:=true|false``:
lidar (Lakibeam -> /scan), camera (-> /camera/color), led_matrix (/dotmatrix/text),
inference (/camera/color -> /edgetpu/inference).

``inference`` is the one subsystem defaulting to false: it holds a YOLO model on
the GPU for the life of the stack, which is wasted on a car whose student code
never reads ``/edgetpu/inference``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EqualsSubstitution, LaunchConfiguration

# Optional subsystems, each toggled by a <name>_enable launch arg, mapped to the
# default that arg takes.
_SUBSYSTEMS = {
    'lidar': 'true',
    'camera': 'true',
    'led_matrix': 'true',
    'inference': 'false',
}


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('neoracer_ros2_driver'), 'launch')

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, f'{name}.launch.py')))

    def gated_include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, f'{name}.launch.py')),
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration(f'{name}_enable'), 'true')))

    enable_args = [
        DeclareLaunchArgument(f'{name}_enable', default_value=default)
        for name, default in _SUBSYSTEMS.items()
    ]

    control = [include(n) for n in ('controller', 'gamepad', 'mux', 'throttle')]
    sensors = [gated_include(n) for n in _SUBSYSTEMS]

    return LaunchDescription([*enable_args, *control, *sensors])
