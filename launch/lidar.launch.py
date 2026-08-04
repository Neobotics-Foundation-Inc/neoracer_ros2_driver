"""Lakibeam 2D LIDAR launch - publishes /scan (frame_id: laser)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('frame_id', default_value='laser'),
        DeclareLaunchArgument('output_topic', default_value='scan'),
        DeclareLaunchArgument('inverted', default_value='false'),
        DeclareLaunchArgument('hostip', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value=TextSubstitution(text='"2368"')),
        DeclareLaunchArgument('angle_offset', default_value='0'),
        DeclareLaunchArgument('filter', default_value=TextSubstitution(text='"0"')),
        DeclareLaunchArgument('scanfreq', default_value=TextSubstitution(text='"30"')),
        DeclareLaunchArgument(
            'laser_enable', default_value=TextSubstitution(text='"true"')),
        DeclareLaunchArgument(
            'scan_range_start', default_value=TextSubstitution(text='"45"')),
        DeclareLaunchArgument(
            'scan_range_stop', default_value=TextSubstitution(text='"315"')),
        # Lakibeam in USB-C mode answers at 192.168.8.2 (host holds 192.168.8.1
        # on usb0). Override to 192.168.198.2 if wired over Ethernet.
        DeclareLaunchArgument('sensorip', default_value='192.168.8.2'),
    ]

    lidar_node = Node(
        package='lakibeam1',
        name='richbeam_lidar_node0',
        executable='lakibeam1_scan_node',
        parameters=[{
            'frame_id': LaunchConfiguration('frame_id'),
            'output_topic': LaunchConfiguration('output_topic'),
            'inverted': LaunchConfiguration('inverted'),
            'hostip': LaunchConfiguration('hostip'),
            'port': LaunchConfiguration('port'),
            'angle_offset': LaunchConfiguration('angle_offset'),
            'sensorip': LaunchConfiguration('sensorip'),
            'scanfreq': LaunchConfiguration('scanfreq'),
            'filter': LaunchConfiguration('filter'),
            'laser_enable': LaunchConfiguration('laser_enable'),
            'scan_range_start': LaunchConfiguration('scan_range_start'),
            'scan_range_stop': LaunchConfiguration('scan_range_stop'),
        }],
    )

    return LaunchDescription([*args, lidar_node])
