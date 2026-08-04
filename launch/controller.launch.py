"""Standalone controller (ESP32 bridge) launch - also a watchdog restart target."""

from neoracer_ros2_driver.launch_common import single_node_launch


def generate_launch_description():
    return single_node_launch(
        arg_name='controller_config',
        default_yaml='controller.yaml',
        package='neoracer_ros2_driver',
        executable='controller',
        node_name='controller_node',
    )
