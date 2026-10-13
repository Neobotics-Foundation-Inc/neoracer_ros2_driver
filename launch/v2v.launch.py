"""V2V node launch: car id from NEORACER_CAR_ID, the rest from config/v2v.yaml."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('neoracer_ros2_driver')
    default_cfg = os.path.join(pkg_dir, 'config', 'v2v.yaml')

    cfg_arg = DeclareLaunchArgument(
        'v2v_config', default_value=default_cfg,
        description='Path to v2v_node config YAML')

    # The identity lives in the environment (~/.neoracer/env, written by
    # `racecar v2v id N`) rather than in the YAML, which is shared by every
    # car through git. It overrides the YAML's car_id: 0.
    node = Node(
        package='neoracer_ros2_driver',
        executable='v2v_node',
        name='v2v_node',
        output='screen',
        parameters=[
            LaunchConfiguration('v2v_config'),
            {'car_id': ParameterValue(
                EnvironmentVariable('NEORACER_CAR_ID', default_value='0'),
                value_type=int)},
        ],
    )

    return LaunchDescription([cfg_arg, node])
