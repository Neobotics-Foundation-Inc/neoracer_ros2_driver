"""Standalone LED matrix (USB-UART display) launch - also a watchdog restart target."""

from neoracer_ros2_driver.launch_common import single_node_launch


def generate_launch_description():
    return single_node_launch(
        arg_name='led_matrix_config',
        default_yaml='led_matrix.yaml',
        package='neoracer_ros2_driver',
        executable='led_matrix',
        node_name='led_matrix_node',
    )
