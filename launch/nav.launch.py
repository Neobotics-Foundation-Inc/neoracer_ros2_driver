#!/usr/bin/env python3

"""
Navigation bringup: the SLAM stack plus the Nav2 planner for rc.nav.

Includes slam.launch.py (slam_toolbox + EKF + static TFs -> /map + map->odom)
and starts the Nav2 planner_server (global costmap fed from /map) under a
lifecycle manager, exposing /compute_path_to_pose for rc.nav.

    ros2 launch neoracer_ros2_driver nav.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('neoracer_ros2_driver')
    nav2_params = os.path.join(pkg, 'config', 'nav2_planner.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'slam.launch.py'))),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planner',
            output='screen',
            parameters=[nav2_params]),
    ])
