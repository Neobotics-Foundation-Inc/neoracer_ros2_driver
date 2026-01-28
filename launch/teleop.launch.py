#!/usr/bin/python3

import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    return launch.LaunchDescription([

        # Launch ESP32 Controller Node
        # Contains: IMU, Odom, Motor Control
        Node(
            package='neoracer_ros2_driver',
            executable='controller',
            name='controller_node'
        ),

        # Launch Joystick Node
        Node(
	    package='joy',
        executable='joy_node',
        name='joy_node',
	    ),

        # Launch Camera Node
        Node(
	        package='neoracer_ros2_driver',
            executable='camera',
            name='camera_node'
	    ),
        
        # Launch LIDAR Node (see lidar.launch.py for details, in dep package)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("neoracer_ros2_driver"),
                'launch', 'lidar.launch.py'))
    ),
    ])

    