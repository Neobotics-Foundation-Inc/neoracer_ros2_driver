"""Standalone throttle_node launch (watchdog restart target)."""

from neoracer_ros2_driver.launch_common import single_node_launch


def generate_launch_description():
    return single_node_launch(
        arg_name='throttle_config',
        default_yaml='throttle.yaml',
        package='neoracer_ros2_driver',
        executable='throttle_node',
    )
