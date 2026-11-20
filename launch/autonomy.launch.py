"""
SLAM on top of a running teleop stack: mapping, or localization on a saved map.

    racecar launch autonomy map:=room                         # mapping, saved in <data root>/maps
    racecar launch autonomy mode:=localization map:=room      # AMCL on <data root>/maps/room
    racecar launch autonomy mode:=localization map:=room localizer:=slam_toolbox pose:=1.0,0.5,1.57
    racecar launch autonomy mode:=localization map:=room pose:=global    # AMCL searches the map

Teleop provides /scan and the odom -> base_link -> laser transforms. Mapping
runs slam_toolbox; localization runs AMCL with map_server (default) or the
slam_toolbox localizer. autonomy_node runs in every mode and publishes
/autonomy/*. `map` is a name under the map root (neoracer_ros2_driver/
maps.py) or a path stem; mapping saves under it. `pose` (x,y,yaw in the map
frame) is the start pose; without one, both localizers start at the map
origin, which is the odometry origin of the mapping run (where the odom
frame started); `global` (AMCL only) searches the whole map. `resume` continues
a mapping run from an autosaved pose graph at `pose`; `extend` starts a mapping
run from a saved pose graph, scan-matched from the rough `pose`, and saves
under `map`. `foxglove:=true` adds foxglove_bridge. Each node's config loads
its car profile override last. A node that exits ends the launch.

On Humble, slam_toolbox (2.6) is a plain node, so it starts directly; the Nav2
lifecycle manager configures and activates map_server and AMCL only. The
launch adds no namespace, so slam_toolbox serves /slam_toolbox/serialize_map
and the other /slam_toolbox/* services under those names.
"""

from __future__ import annotations

import os
from typing import Any

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from neoracer_ros2_driver.launch_common import local_overrides
from neoracer_ros2_driver.maps import map_stem, parse_pose

MODES = ('mapping', 'localization')
LOCALIZERS = ('amcl', 'slam_toolbox')


def _files(config_dir: str, yaml: str) -> list[str]:
    return [os.path.join(config_dir, yaml), *local_overrides([yaml])]


def _lifecycle_manager(name: str, files: list[str]) -> dict[str, Any]:
    """Return a lifecycle manager that configures and activates its node_names."""
    return {
        'package': 'nav2_lifecycle_manager',
        'executable': 'lifecycle_manager',
        'name': name,
        'parameters': files,
    }


def autonomy_spec(
    mode: str,
    localizer: str,
    map_name: str,
    pose_text: str,
    session: str,
    config_dir: str,
    pose_source: str = '',
    resumed: bool = False,
    source_stem: str = '',
    extended_from: str = '',
) -> dict[str, Any]:
    if pose_source:
        source = pose_source
    elif resumed:
        source = 'autosave'
    elif mode == 'mapping':
        source = 'pose' if source_stem else ''
    else:
        source = 'global' if pose_text == 'global' else 'pose' if pose_text else 'origin'
    return {
        'package': 'neoracer_ros2_driver',
        'executable': 'autonomy_node',
        'name': 'autonomy_node',
        'parameters': [
            *_files(config_dir, 'autonomy.yaml'),
            {
                'mode': mode,
                'map': map_name,
                'localizer': localizer,
                'session': session,
                'pose': pose_text,
                'pose_source': source,
                'resumed_from_autosave': resumed,
                'source': source_stem,
                'extended_from': extended_from,
            },
        ],
    }


def foxglove_spec(config_dir: str) -> dict[str, Any]:
    return {
        'package': 'foxglove_bridge',
        'executable': 'foxglove_bridge',
        'name': 'foxglove_bridge',
        'parameters': _files(config_dir, 'foxglove.yaml'),
    }


def node_specs(
    mode: str,
    localizer: str,
    map_name: str,
    pose_text: str,
    config_dir: str,
    session: str = '',
    resume: str = '',
    foxglove: bool = False,
    pose_source: str = '',
    extend: str = '',
) -> list[dict[str, Any]]:
    """Return (package, executable, name, parameters) per node; ValueError on a bad argument."""
    specs = [
        *slam_specs(mode, localizer, map_name, pose_text, config_dir, resume, extend),
        autonomy_spec(
            mode,
            localizer,
            map_name,
            pose_text,
            session,
            config_dir,
            pose_source,
            bool(resume),
            resume or extend,
            # The stem's last part is the source map's name, saved or autosaved.
            os.path.basename(extend),
        ),
    ]
    return [*specs, foxglove_spec(config_dir)] if foxglove else specs


def slam_specs(
    mode: str,
    localizer: str,
    map_name: str,
    pose_text: str,
    config_dir: str,
    resume: str = '',
    extend: str = '',
) -> list[dict[str, Any]]:
    if mode not in MODES:
        raise ValueError(f'mode must be one of {", ".join(MODES)}: {mode!r}')
    if localizer not in LOCALIZERS:
        raise ValueError(f'localizer must be one of {", ".join(LOCALIZERS)}: {localizer!r}')
    search = pose_text.strip() == 'global'
    if search and (mode != 'localization' or localizer != 'amcl'):
        raise ValueError('pose:=global needs mode:=localization with localizer:=amcl')
    pose = None if search else parse_pose(pose_text)
    if (resume or extend) and mode != 'mapping':
        raise ValueError('resume:= and extend:= are for mode:=mapping')
    if resume and extend:
        raise ValueError('resume:= and extend:= exclude each other')
    if mode == 'mapping':
        files = _files(config_dir, 'slam_mapping.yaml')
        # A pose graph given at startup is loaded at map_start_pose, and the
        # first scan is matched against the saved scans near it; mapping then
        # continues. (LOCALIZE_AT_POSE, the other match type, stops adding to
        # the map.)
        loaded = resume or extend
        resumed = (
            [{'map_file_name': loaded, 'map_start_pose': list(pose or (0.0, 0.0, 0.0))}]
            if loaded
            else []
        )
        # Humble's slam_toolbox is not a lifecycle node, so no lifecycle manager starts it.
        return [
            {
                'package': 'slam_toolbox',
                'executable': 'async_slam_toolbox_node',
                'name': 'slam_toolbox',
                'parameters': [*files, *resumed],
            },
        ]
    if not map_name:
        raise ValueError('mode:=localization needs map:=NAME')
    stem = str(map_stem(map_name))
    if localizer == 'slam_toolbox':
        start = list(pose) if pose else [0.0, 0.0, 0.0]
        files = _files(config_dir, 'slam_localization.yaml')
        return [
            {
                'package': 'slam_toolbox',
                'executable': 'localization_slam_toolbox_node',
                'name': 'slam_toolbox',
                'parameters': [*files, {'map_file_name': stem, 'map_start_pose': start}],
            },
        ]
    amcl_files = _files(config_dir, 'amcl.yaml')
    x, y, yaw = pose or (0.0, 0.0, 0.0)
    # A global search starts without a pose; autonomy_node asks AMCL to spread
    # its particles over the map once AMCL is up.
    amcl_params = [
        *amcl_files,
        {
            'set_initial_pose': not search,
            'initial_pose.x': x,
            'initial_pose.y': y,
            'initial_pose.z': 0.0,
            'initial_pose.yaw': yaw,
        },
    ]
    return [
        {
            'package': 'nav2_map_server',
            'executable': 'map_server',
            'name': 'map_server',
            'parameters': [*amcl_files, {'yaml_filename': stem + '.yaml'}],
        },
        {
            'package': 'nav2_amcl',
            'executable': 'amcl',
            'name': 'amcl',
            'parameters': amcl_params,
        },
        _lifecycle_manager('lifecycle_manager_localization', amcl_files),
    ]


def _launch_setup(context: LaunchContext) -> list[Node]:
    config_dir = os.path.join(get_package_share_directory('neoracer_ros2_driver'), 'config')
    specs = node_specs(
        LaunchConfiguration('mode').perform(context),
        LaunchConfiguration('localizer').perform(context),
        LaunchConfiguration('map').perform(context),
        LaunchConfiguration('pose').perform(context),
        config_dir,
        LaunchConfiguration('session').perform(context),
        LaunchConfiguration('resume').perform(context),
        LaunchConfiguration('foxglove').perform(context).lower() in ('1', 'true'),
        LaunchConfiguration('pose_source').perform(context),
        LaunchConfiguration('extend').perform(context),
    )
    # 'both': node output also goes to the launch's log directory.
    return [
        Node(**spec, output='both', on_exit=Shutdown(), **_stop_timeouts(spec)) for spec in specs
    ]


def _stop_timeouts(spec: dict[str, Any]) -> dict[str, Any]:
    """Return short stop timeouts for foxglove_bridge; it can ignore SIGINT with a client on."""
    if spec['name'] != 'foxglove_bridge':
        return {}
    return {'sigterm_timeout': '2', 'sigkill_timeout': '2'}


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'mode', default_value='mapping', description='mapping or localization'
            ),
            DeclareLaunchArgument(
                'localizer', default_value='amcl', description='localization: amcl or slam_toolbox'
            ),
            DeclareLaunchArgument(
                'map',
                default_value='',
                description='map name or path stem: saved to (mapping) or loaded (localization)',
            ),
            DeclareLaunchArgument(
                'pose',
                default_value='',
                description='localization: start pose x,y,yaw (map frame), or global (AMCL)',
            ),
            DeclareLaunchArgument(
                'session', default_value='', description='run id recorded with saves'
            ),
            DeclareLaunchArgument(
                'resume',
                default_value='',
                description='mapping: autosaved pose graph stem to continue from, at pose',
            ),
            DeclareLaunchArgument(
                'extend',
                default_value='',
                description='mapping: saved pose graph stem to extend, near pose',
            ),
            DeclareLaunchArgument(
                'foxglove',
                default_value='false',
                description='also start foxglove_bridge (port 8765)',
            ),
            DeclareLaunchArgument(
                'pose_source', default_value='', description='reported in /autonomy/status'
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
