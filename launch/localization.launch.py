#!/usr/bin/env python3

"""
Localization + TF layer for the SLAM / Nav stack.

The teleop driver publishes /scan, /odom, and /imu but no TF. This brings up
everything slam_toolbox and Nav2 need below the map frame:

    * robot_localization EKF -> odom -> base_footprint at 100 Hz, fusing the
      20 Hz wheel /odom (pose + forward speed) with the 168 Hz IMU gyro
      (yaw-rate). High rate + smooth heading; SLAM corrects the slow drift.
    * base_footprint -> laser    static (LiDAR mount; yaw 0 = forward-aligned).
    * base_footprint -> imu_link  static.

Override the LiDAR mount once measured: laser_x:= / laser_z:=.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ekf_params = os.path.join(
        get_package_share_directory('neoracer_ros2_driver'), 'config', 'ekf.yaml')
    laser_x = LaunchConfiguration('laser_x')
    laser_z = LaunchConfiguration('laser_z')

    return LaunchDescription([
        DeclareLaunchArgument('laser_x', default_value='0.12'),
        DeclareLaunchArgument('laser_z', default_value='0.15'),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params]),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            arguments=['--x', laser_x, '--y', '0', '--z', laser_z,
                       '--yaw', '0', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_footprint', '--child-frame-id', 'laser'],
            output='screen'),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_imu',
            arguments=['--x', '0', '--y', '0', '--z', '0.05',
                       '--yaw', '0', '--pitch', '0', '--roll', '0',
                       '--frame-id', 'base_footprint', '--child-frame-id', 'imu_link'],
            output='screen'),
    ])
