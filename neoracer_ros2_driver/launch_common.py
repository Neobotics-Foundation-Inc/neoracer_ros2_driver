"""Shared helpers for the per-node launch files (watchdog restart targets)."""

from collections.abc import Sequence
import os
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from neoracer_ros2_driver.car_profile import param_overrides


def local_overrides(yamls: Sequence[str]) -> list[str]:
    """Return the car profile's override (params/<name>.yaml) for each YAML that has one."""
    return param_overrides(yamls)


def single_node_launch(
    arg_name: str,
    default_yaml: str,
    package: str,
    executable: str,
    node_name: str | None = None,
    remappings: list[tuple[str, str]] | None = None,
    description: str | None = None,
    extra_yamls: Sequence[tuple[str, str]] = (),
) -> LaunchDescription:
    """
    Build a 1-node LaunchDescription configured from YAML param files.

    arg_name: launch arg the YAML path is exposed as (e.g. 'throttle_config').
    default_yaml: filename inside this package's share/config (e.g. 'throttle.yaml').
    extra_yamls: (arg_name, filename) pairs loaded after default_yaml, in order.

    Per-car overrides (params/<name>.yaml in the car profile,
    neoracer_ros2_driver.car_profile) load last, one per YAML that has
    one, so their keys win. They are read at launch, so no rebuild is needed.
    """
    pkg_dir = get_package_share_directory('neoracer_ros2_driver')
    config_dir = os.path.join(pkg_dir, 'config')

    files = [(arg_name, default_yaml), *extra_yamls]
    args = [
        DeclareLaunchArgument(
            name,
            default_value=os.path.join(config_dir, yaml),
            description=(
                (description or f'Path to {executable} config YAML')
                if name == arg_name
                else f'Path to {yaml}'
            ),
        )
        for name, yaml in files
    ]
    parameters: list[Any] = [LaunchConfiguration(name) for name, _ in files]
    parameters += local_overrides([yaml for _, yaml in files])

    node_kwargs: dict[str, Any] = {
        'package': package,
        'executable': executable,
        'name': node_name or executable,
        'output': 'screen',
        'parameters': parameters,
    }
    if remappings:
        node_kwargs['remappings'] = remappings
    node = Node(**node_kwargs)

    return LaunchDescription([*args, node])
