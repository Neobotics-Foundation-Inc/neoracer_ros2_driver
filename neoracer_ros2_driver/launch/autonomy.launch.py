#!/usr/bin/env python3

"""
Launch the autonomy base layer on top of teleop.

TF and the twist bridge are always cheap to run. SLAM and Nav2 are on-demand
activities, launched by `racecar mapping` and `racecar navigation` when needed
rather than living in the service.

Teleop owns every device (serial, lidar, camera, LED); this launch adds the
intelligence and no hardware drivers. The osracer chassis, lidar, camera,
and LED launches are deliberately NOT included here - they would collide
with the teleop nodes on the same devices.

  osracer_description  robot_state_publisher + odom-topic TF, fed from the
                       controller's /odom (odom -> base_footprint -> laser)
  twist_bridge         /cmd_vel -> normalized /drive into the mux, so SWB
                       manual override and the throttle caps still apply
  EKF                  imu_complementary_filter + robot_localization fusing
                       /odom + /imu; owns the odom -> base_footprint TF and
                       publishes the real /odometry/filtered
  osracer_slam         opt-in (slam:=true); `racecar mapping` is the front door
  osracer_navigation   opt-in (nav:=true); `racecar navigation` is the front door

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

    # Both default off: mapping and navigation are on-demand activities
    # (`racecar mapping` / `racecar navigation`), not part of the service.
    # They are also mutually exclusive: slam_toolbox and Nav2's map-file
    # localization both publish map->odom.
    enable_slam = DeclareLaunchArgument(
        'slam', default_value='false',
        description='Start slam_toolbox (osracer_slam)')
    enable_nav = DeclareLaunchArgument(
        'nav', default_value='false',
        description='Start Nav2 on a saved map (osracer_navigation)')

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

    # use_namespace/use_rviz must be Python-cased: Nav2's bringup evaluates
    # them in a PythonExpression, and the vendor's lowercase defaults throw
    # NameError. RViz stays off; this runs headless under systemd.
    nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('osracer_navigation'),
            'launch', 'nav2.launch.py')),
        condition=IfCondition(LaunchConfiguration('nav')),
        launch_arguments={
            'use_namespace': 'False',
            'use_rviz': 'False',
        }.items(),
    )

    twist_bridge = Node(
        package='neoracer_ros2_driver',
        executable='twist_bridge',
        name='twist_bridge_node',
        parameters=[os.path.join(pkg_share, 'config', 'twist_bridge.yaml')],
        output='screen',
    )

    # EKF pair from the vendor stack, fed by our controller's topics. The
    # complementary filter smooths the raw IMU; robot_localization fuses it
    # with wheel odometry. It owns odom -> base_footprint (the controller's
    # publish_tf ships off) and publishes /odometry/filtered for Nav2 and
    # cartographer, replacing the bridge's old relay.
    imu_filter = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_gain_node',
        output='screen',
        remappings=[
            ('imu/data_raw', 'imu'),
            ('imu/data', 'imu_filter'),
        ],
        parameters=[{
            'do_bias_estimation': True,
            'do_adaptive_gain': True,
            'use_mag': False,
            'gain_acc': 0.01,
            'gain_mag': 0.01,
        }],
    )
    ekf = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(
                get_package_share_directory('osracer_bringup'),
                'param', 'chassis_ekf_params.yaml'),
            {
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'base_footprint',
                'world_frame': 'odom',
                'publish_tf': True,
            },
        ],
    )

    return LaunchDescription([
        enable_slam,
        enable_nav,
        description,
        twist_bridge,
        imu_filter,
        ekf,
        slam,
        nav,
    ])
