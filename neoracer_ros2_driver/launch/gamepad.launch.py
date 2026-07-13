"""Standalone gamepad_node launch (watchdog restart target)."""

from neoracer_ros2_driver.launch_common import single_node_launch


def generate_launch_description():
    return single_node_launch(
        arg_name='gamepad_config',
        default_yaml='gamepad.yaml',
        package='neoracer_ros2_driver',
        executable='gamepad_node',
    )
