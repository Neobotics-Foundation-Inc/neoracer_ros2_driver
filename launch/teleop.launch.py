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
    mux_node                   -- fresh /drive -> /mux_out (autonomy_passthrough)
        |
        v
    throttle_node              -- applies speed/steer caps, /mux_out -> /motor
        |  /motor
        v
    controller -> ESP32 serial "v <m/s> <deg>"

Optional subsystems, each gated by ``<name>_enable:=true|false``:
frames (robot_state_publisher: the static TF tree base_link -> laser, imu_link,
camera_link and base_footprint, from config/frames.yaml), lidar (Lakibeam ->
/scan), camera (-> /camera/color), led_matrix (/dotmatrix/text), inference
(/camera/color -> /edgetpu/inference).

The control pipeline's config files are launch arguments
(``controller_config`` and ``odom_config``, ``gamepad_config``, ``mux_config``,
``throttle_config``), passed to each node's launch file, which loads the car
profile's ``params/<name>.yaml`` on top.

Every subsystem defaults to true in this file. ``scripts/launch_teleop.sh``,
which the teleop service and ``racecar teleop`` run, adds the per-boot choice
from ``scripts/features.py``: ``camera_enable:=true`` and
``inference_enable:=false`` unless changed, and a launch argument given on the
command line wins. ``inference`` is the expensive one: it holds a YOLO model on
the GPU for the life of the stack, and it exits at startup on a car where
``racecar setup ml`` has not run.
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
    'frames': 'true',
    'lidar': 'true',
    'camera': 'true',
    'led_matrix': 'true',
    'inference': 'true',
}


# The always-on control pipeline: each node's launch file and the config
# arguments it takes, as (argument, YAML) pairs.
_CONTROL = {
    'controller': (('controller_config', 'controller.yaml'), ('odom_config', 'odom.yaml')),
    'gamepad': (('gamepad_config', 'gamepad.yaml'),),
    'mux': (('mux_config', 'mux.yaml'),),
    'throttle': (('throttle_config', 'throttle.yaml'),),
}


def generate_launch_description() -> LaunchDescription:
    pkg_dir = get_package_share_directory('neoracer_ros2_driver')
    launch_dir = os.path.join(pkg_dir, 'launch')
    config_dir = os.path.join(pkg_dir, 'config')

    config_args = [
        DeclareLaunchArgument(
            arg,
            default_value=os.path.join(config_dir, yaml),
            description=f'Path to {yaml}',
        )
        for files in _CONTROL.values()
        for arg, yaml in files
    ]

    def include(name: str) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, f'{name}.launch.py')),
            launch_arguments={arg: LaunchConfiguration(arg) for arg, _ in _CONTROL[name]}.items(),
        )

    def gated_include(name: str) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, f'{name}.launch.py')),
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration(f'{name}_enable'), 'true')
            ),
        )

    enable_args = [
        DeclareLaunchArgument(
            f'{name}_enable',
            default_value=default,
            description=f'Bring up {name} subsystem ("true"/"false")',
        )
        for name, default in _SUBSYSTEMS.items()
    ]

    control = [include(n) for n in _CONTROL]
    sensors = [gated_include(n) for n in _SUBSYSTEMS]

    return LaunchDescription([*config_args, *enable_args, *control, *sensors])
