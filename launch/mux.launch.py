"""Standalone mux_node launch (watchdog restart target)."""

from launch import LaunchDescription
from neoracer_ros2_driver.launch_common import single_node_launch


def generate_launch_description() -> LaunchDescription:
    return single_node_launch(
        arg_name='mux_config',
        default_yaml='mux.yaml',
        package='neoracer_ros2_driver',
        executable='mux_node',
    )
