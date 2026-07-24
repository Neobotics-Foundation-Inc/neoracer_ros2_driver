#!/usr/bin/env python3

"""
Autonomy layer on top of teleop: TF + SLAM + Nav2 from the osracer stack,
bridged into the neoracer mux.

Teleop owns every device (serial, lidar, camera, LED); this launch adds the
intelligence and no hardware drivers. The osracer chassis, lidar, camera,
and LED launches are deliberately NOT included here - they would collide
with the teleop nodes on the same devices.

  osracer_description  robot_state_publisher + odom-topic TF, fed from the
                       controller's /odom (odom -> base_footprint -> laser)
  osracer_slam         slam_toolbox online async on /scan
  osracer_navigation   Nav2, publishing /cmd_vel
  twist_bridge         /cmd_vel -> normalized /drive into the mux, so SWB
                       manual override and the throttle caps still apply

Requires the osracer workspace sourced underneath this one
(scripts/launch_autonomy.sh does that; see also `racecar service`).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('neoracer_ros2_driver')

    enable_slam = DeclareLaunchArgument(
        'slam', default_value='true',
        description='Start slam_toolbox (osracer_slam)')
    enable_nav = DeclareLaunchArgument(
        'nav', default_value='true',
        description='Start Nav2 (osracer_navigation)')

    # TF: robot_state_publisher + odom-topic broadcaster from the vendor
    # description package, fed by the neoracer controller's /odom.
    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('osracer_description'),
            'launch', 'osracer_description.launch.py')),
        launch_arguments={
            'start_jsp': 'false',
            'jsp_gui': 'false',
            'use_rviz': 'false',
            'publish_frequency': '100.0',
            'odom_topic': '/odom',
        }.items(),
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('osracer_slam'),
            'launch', 'slam_toolbox.launch.py')),
        condition=IfCondition(LaunchConfiguration('slam')),
    )

    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('osracer_navigation'),
            'launch', 'nav2.launch.py')),
        condition=IfCondition(LaunchConfiguration('nav')),
    )

    twist_bridge = Node(
        package='neoracer_ros2_driver',
        executable='twist_bridge',
        name='twist_bridge_node',
        parameters=[os.path.join(pkg_share, 'config', 'twist_bridge.yaml')],
        output='screen',
    )

    return LaunchDescription([
        enable_slam,
        enable_nav,
        description,
        twist_bridge,
        slam,
        nav,
    ])
