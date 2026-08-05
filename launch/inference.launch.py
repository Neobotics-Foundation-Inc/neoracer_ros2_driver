"""Standalone YOLO inference launch - camera frames to /edgetpu/inference."""

from neoracer_ros2_driver.launch_common import single_node_launch


def generate_launch_description():
    return single_node_launch(
        arg_name='inference_config',
        default_yaml='inference.yaml',
        package='neoracer_ros2_driver',
        executable='inference_node',
        node_name='inference_node',
    )
