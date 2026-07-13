"""Standalone camera launch - also a watchdog restart target."""

from neoracer_ros2_driver.launch_common import single_node_launch


def generate_launch_description():
    return single_node_launch(
        arg_name='camera_config',
        default_yaml='camera.yaml',
        package='neoracer_ros2_driver',
        executable='camera',
        node_name='camera_node',
    )
