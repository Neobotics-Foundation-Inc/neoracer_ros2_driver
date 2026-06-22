#!/usr/bin/env python3

"""
SLAM bringup: slam_toolbox (online async) over the localization + TF layer.

Layers on top of the running teleop driver (which provides /scan, /odom, /imu).
Includes localization.launch.py for the EKF odom->base_footprint TF plus the
static base_footprint->laser / ->imu_link transforms, then starts slam_toolbox
in mapping mode -> /map + map->odom.

    ros2 launch neoracer_ros2_driver slam.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('neoracer_ros2_driver')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'localization.launch.py'))),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'slam_toolbox.yaml')]),
    ])
