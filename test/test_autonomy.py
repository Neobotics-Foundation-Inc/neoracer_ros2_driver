"""
Unit tests for the autonomy launch, maps.py, the SLAM configs and autonomy_node's setup.

The launch tests load the launch file, which imports launch and launch_ros, so
they run where ROS is installed (on the car) and are skipped elsewhere.
"""

import ast
import importlib.util
import json
from pathlib import Path
import re

from neoracer_ros2_driver import maps
import pytest
import test_topic_contract as contract
import yaml

ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / 'config'
NODE = ROOT / 'neoracer_ros2_driver' / 'autonomy_node.py'
SLAM_CONFIGS = ('slam_mapping.yaml', 'slam_localization.yaml')
# Keys upstream's Jazzy configs set that Humble's slam_toolbox (2.6.10) does not
# declare; upstream's values match what 2.6.10 does without them.
JAZZY_ONLY_KEYS = ('use_lifecycle_manager', 'restamp_tf', 'check_min_dist_and_heading_precisely')
# The LakiBeam1's rated range at 10% reflectivity (Richbeam's specification), m.
LAKIBEAM_MAX_RANGE = 15.0
# The dev car, 2026-11-12: the LakiBeam's nearest return with the nose square
# against a wall, m (test_frames.py).
CAR_WALL_RANGE = 0.306


@pytest.fixture(scope='module')
def autonomy():
    pytest.importorskip('launch_ros')
    spec = importlib.util.spec_from_file_location(
        'autonomy_launch', ROOT / 'launch' / 'autonomy.launch.py'
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _dicts(spec):
    out = {}
    for p in spec['parameters']:
        if isinstance(p, dict):
            out.update(p)
    return out


def _slam_params(name):
    return yaml.safe_load((CONFIG / name).read_text())['slam_toolbox']['ros__parameters']


class TestMaps:
    def test_name_resolves_under_the_root(self, tmp_path):
        assert maps.map_stem('room', tmp_path) == tmp_path / 'room' / 'room'

    def test_root_from_the_environment(self, tmp_path, monkeypatch):
        monkeypatch.setenv(maps.ROOT_ENV, str(tmp_path))
        assert maps.map_stem('room') == tmp_path / 'room' / 'room'

    def test_default_root_is_maps_under_the_data_root(self, monkeypatch, data_root_dir):
        monkeypatch.delenv(maps.ROOT_ENV, raising=False)
        assert maps.map_root() == data_root_dir / 'maps'

    def test_path_drops_a_map_suffix(self):
        assert maps.map_stem('/tmp/a/room.yaml') == Path('/tmp/a/room')
        assert maps.map_stem('/tmp/a/room') == Path('/tmp/a/room')

    @pytest.mark.parametrize('name', ['room', 'hall_2', 'a-b'])
    def test_valid_names(self, name):
        assert maps.valid_name(name)

    @pytest.mark.parametrize('name', ['', 'a b', '../x', 'a/b'])
    def test_invalid_names(self, name):
        assert not maps.valid_name(name)

    def test_pose(self):
        assert maps.parse_pose('1, -2.5, 1.57') == (1.0, -2.5, 1.57)
        assert maps.parse_pose('') is None

    @pytest.mark.parametrize('text', ['1,2', '1,2,3,4', 'a,b,c', 'nan,0,0'])
    def test_bad_pose(self, text):
        with pytest.raises(ValueError):
            maps.parse_pose(text)


class TestLaunch:
    def test_mapping_runs_slam_toolbox_async(self, autonomy):
        (node,) = autonomy.slam_specs('mapping', 'amcl', '', '', str(CONFIG))
        assert node['executable'] == 'async_slam_toolbox_node'
        assert node['parameters'] == [str(CONFIG / 'slam_mapping.yaml')]

    def test_amcl_localization(self, autonomy, tmp_path, monkeypatch):
        monkeypatch.setenv(maps.ROOT_ENV, str(tmp_path))
        nodes = autonomy.slam_specs('localization', 'amcl', 'room', '', str(CONFIG))
        assert [n['name'] for n in nodes] == [
            'map_server',
            'amcl',
            'lifecycle_manager_localization',
        ]
        assert _dicts(nodes[0])['yaml_filename'] == str(tmp_path / 'room' / 'room.yaml')
        p = _dicts(nodes[1])
        assert p['set_initial_pose'] is True
        assert (p['initial_pose.x'], p['initial_pose.y'], p['initial_pose.yaw']) == (0, 0, 0)

    def test_amcl_start_pose(self, autonomy):
        nodes = autonomy.slam_specs('localization', 'amcl', 'room', '1,2,0.5', str(CONFIG))
        p = _dicts(nodes[1])
        assert p['set_initial_pose'] is True
        assert (p['initial_pose.x'], p['initial_pose.y'], p['initial_pose.yaw']) == (1.0, 2.0, 0.5)

    def test_slam_toolbox_localization(self, autonomy, tmp_path, monkeypatch):
        monkeypatch.setenv(maps.ROOT_ENV, str(tmp_path))
        (node,) = autonomy.slam_specs('localization', 'slam_toolbox', 'room', '', str(CONFIG))
        assert node['executable'] == 'localization_slam_toolbox_node'
        p = _dicts(node)
        assert p['map_file_name'] == str(tmp_path / 'room' / 'room')
        assert p['map_start_pose'] == [0.0, 0.0, 0.0]

    @pytest.mark.parametrize(
        'args',
        [
            ('drive', 'amcl', '', ''),
            ('localization', 'gmapping', 'room', ''),
            ('localization', 'amcl', '', ''),
            ('localization', 'amcl', 'room', '1,2'),
        ],
    )
    def test_bad_arguments(self, autonomy, args):
        with pytest.raises(ValueError):
            autonomy.slam_specs(*args, str(CONFIG))

    def test_local_override_loads_last(self, autonomy, tmp_path, car_profile_dir):
        (tmp_path / 'slam_mapping.yaml').write_text('slam_toolbox: {ros__parameters: {}}\n')
        (car_profile_dir / 'params').mkdir(parents=True)
        override = car_profile_dir / 'params' / 'slam_mapping.yaml'
        override.write_text('slam_toolbox: {ros__parameters: {}}\n')
        (node,) = autonomy.slam_specs('mapping', 'amcl', '', '', str(tmp_path))
        assert node['parameters'] == [str(tmp_path / 'slam_mapping.yaml'), str(override)]

    def test_launch_description_declares_the_arguments(self, autonomy):
        from launch.actions import DeclareLaunchArgument

        ld = autonomy.generate_launch_description()
        names = [e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)]
        assert names == [
            'mode',
            'localizer',
            'map',
            'pose',
            'session',
            'resume',
            'extend',
            'foxglove',
            'pose_source',
        ]


class TestHumbleLaunch:
    """Humble's slam_toolbox is a plain node; the lifecycle manager runs map_server and AMCL."""

    @pytest.mark.parametrize(
        'mode,localizer,nodes',
        [
            ('mapping', 'amcl', [('slam_toolbox', 'async_slam_toolbox_node')]),
            (
                'localization',
                'amcl',
                [
                    ('nav2_map_server', 'map_server'),
                    ('nav2_amcl', 'amcl'),
                    ('nav2_lifecycle_manager', 'lifecycle_manager'),
                ],
            ),
            ('localization', 'slam_toolbox', [('slam_toolbox', 'localization_slam_toolbox_node')]),
        ],
    )
    def test_each_mode_starts_these_nodes(
        self, autonomy, tmp_path, monkeypatch, mode, localizer, nodes
    ):
        monkeypatch.setenv(maps.ROOT_ENV, str(tmp_path))
        specs = autonomy.node_specs(mode, localizer, 'room', '', str(CONFIG))
        expected = [*nodes, ('neoracer_ros2_driver', 'autonomy_node')]
        assert [(s['package'], s['executable']) for s in specs] == expected
        # No namespace: slam_toolbox serves /slam_toolbox/*, which autonomy_node calls.
        assert all(set(s) == {'package', 'executable', 'name', 'parameters'} for s in specs)

    def test_the_lifecycle_manager_runs_map_server_and_amcl_only(
        self, autonomy, tmp_path, monkeypatch
    ):
        monkeypatch.setenv(maps.ROOT_ENV, str(tmp_path))
        managers = [
            s
            for mode, localizer in (
                ('mapping', 'amcl'),
                ('localization', 'slam_toolbox'),
                ('localization', 'amcl'),
            )
            for s in autonomy.node_specs(mode, localizer, 'room', '', str(CONFIG))
            if s['package'] == 'nav2_lifecycle_manager'
        ]
        assert [m['name'] for m in managers] == ['lifecycle_manager_localization']
        assert managers[0]['parameters'] == [str(CONFIG / 'amcl.yaml')]

    def test_the_autonomy_override_loads_after_the_shipped_file(self, autonomy, car_profile_dir):
        (car_profile_dir / 'params').mkdir(parents=True)
        override = car_profile_dir / 'params' / 'autonomy.yaml'
        override.write_text('autonomy_node: {ros__parameters: {}}\n')
        spec = autonomy.node_specs('mapping', 'amcl', 'room', '', str(CONFIG))[-1]
        # The shipped config, the car profile's params/autonomy.yaml, then the launch's values.
        assert spec['parameters'][:2] == [str(CONFIG / 'autonomy.yaml'), str(override)]
        assert isinstance(spec['parameters'][2], dict) and len(spec['parameters']) == 3

    def test_every_node_ends_the_launch_with_no_namespace(self, autonomy, monkeypatch):
        from launch import LaunchContext
        from launch.actions import Shutdown

        monkeypatch.setattr(autonomy, 'get_package_share_directory', lambda _: str(ROOT))
        context = LaunchContext()
        args = {'mode': 'mapping', 'localizer': 'amcl', 'map': 'room', 'foxglove': 'true'}
        for name in ('pose', 'session', 'resume', 'pose_source', 'extend'):
            args[name] = ''
        context.launch_configurations.update(args)
        nodes = autonomy._launch_setup(context)
        assert [n.node_executable for n in nodes] == [
            'async_slam_toolbox_node',
            'autonomy_node',
            'foxglove_bridge',
        ]
        for node in nodes:
            assert node._Node__node_namespace is None
            assert isinstance(node._ExecuteLocal__on_exit, Shutdown)


class TestAutonomyNodeSpec:
    def test_every_mode_adds_autonomy_node(self, autonomy):
        for args in (('mapping', 'amcl', 'room', ''), ('localization', 'amcl', 'room', '')):
            assert autonomy.node_specs(*args, str(CONFIG))[-1]['name'] == 'autonomy_node'

    def test_autonomy_node_parameters(self, autonomy):
        spec = autonomy.node_specs(
            'localization', 'amcl', 'room', '1,2,0', str(CONFIG), session='s1'
        )[-1]
        assert spec['parameters'][0] == str(CONFIG / 'autonomy.yaml')
        p = _dicts(spec)
        assert (p['mode'], p['map'], p['session'], p['pose_source']) == (
            'localization',
            'room',
            's1',
            'pose',
        )

    @pytest.mark.parametrize(
        'pose,source', [('', 'origin'), ('global', 'global'), ('1,2,0', 'pose')]
    )
    def test_pose_source(self, autonomy, pose, source):
        spec = autonomy.node_specs('localization', 'amcl', 'room', pose, str(CONFIG))[-1]
        assert _dicts(spec)['pose_source'] == source

    def test_global_starts_amcl_without_a_pose(self, autonomy):
        nodes = autonomy.slam_specs('localization', 'amcl', 'room', 'global', str(CONFIG))
        assert _dicts(nodes[1])['set_initial_pose'] is False

    @pytest.mark.parametrize(
        'mode,localizer', [('mapping', 'amcl'), ('localization', 'slam_toolbox')]
    )
    def test_global_needs_amcl_localization(self, autonomy, mode, localizer):
        with pytest.raises(ValueError):
            autonomy.slam_specs(mode, localizer, 'room', 'global', str(CONFIG))


class TestResumeAndFoxglove:
    def test_resume_loads_the_autosave_at_the_pose(self, autonomy):
        specs = autonomy.node_specs(
            'mapping',
            'amcl',
            'room',
            '1,2,0.5',
            str(CONFIG),
            session='s',
            resume='/m/room/autosave/room',
        )
        slam = _dicts(specs[0])
        assert slam == {
            'map_file_name': '/m/room/autosave/room',
            'map_start_pose': [1.0, 2.0, 0.5],
        }
        node = _dicts(specs[-1])
        assert node['resumed_from_autosave'] is True and node['pose_source'] == 'autosave'

    def test_fresh_mapping_loads_no_graph(self, autonomy):
        specs = autonomy.node_specs('mapping', 'amcl', 'room', '', str(CONFIG))
        assert _dicts(specs[0]) == {} and _dicts(specs[-1])['resumed_from_autosave'] is False

    def test_resume_is_for_mapping(self, autonomy):
        with pytest.raises(ValueError):
            autonomy.slam_specs('localization', 'amcl', 'room', '', str(CONFIG), resume='/x')

    def test_foxglove_on_demand(self, autonomy):
        assert 'foxglove_bridge' not in [
            s['name'] for s in autonomy.node_specs('mapping', 'amcl', 'r', '', str(CONFIG))
        ]
        specs = autonomy.node_specs('mapping', 'amcl', 'r', '', str(CONFIG), foxglove=True)
        assert specs[-1]['parameters'][0] == str(CONFIG / 'foxglove.yaml')

    def test_pose_source_given(self, autonomy):
        spec = autonomy.node_specs(
            'localization', 'amcl', 'r', '1,2,0', str(CONFIG), pose_source='last_pose'
        )[-1]
        assert _dicts(spec)['pose_source'] == 'last_pose'


class TestConfigs:
    @pytest.mark.parametrize('name', SLAM_CONFIGS)
    def test_slam_toolbox_frames_and_ranges(self, name):
        p = _slam_params(name)
        assert (p['base_frame'], p['odom_frame'], p['map_frame']) == ('base_link', 'odom', 'map')
        assert p['scan_topic'] == '/scan'
        assert p['max_laser_range'] == LAKIBEAM_MAX_RANGE
        assert p['mode'] == name.split('_')[1].removesuffix('.yaml')

    @pytest.mark.parametrize('name', SLAM_CONFIGS)
    def test_slam_toolbox_is_a_plain_node_on_humble(self, name):
        # Upstream's test_slam_toolbox_runs_under_the_lifecycle_manager cannot
        # hold on Humble: slam_toolbox 2.6.10 has no lifecycle, so the file
        # configures slam_toolbox alone and sets no Jazzy-only key.
        data = yaml.safe_load((CONFIG / name).read_text())
        assert list(data) == ['slam_toolbox']
        assert not set(JAZZY_ONLY_KEYS) & set(data['slam_toolbox']['ros__parameters'])

    @pytest.mark.parametrize('name', SLAM_CONFIGS)
    def test_scan_matching_weights_the_odometry(self, name):
        p = _slam_params(name)
        assert p['angle_variance_penalty'] <= 0.05 and p['minimum_angle_penalty'] <= 0.5
        assert p['distance_variance_penalty'] <= 0.1

    def test_amcl(self):
        data = yaml.safe_load((CONFIG / 'amcl.yaml').read_text())
        p = data['amcl']['ros__parameters']
        assert p['base_frame_id'] == 'base_link' and p['laser_max_range'] == LAKIBEAM_MAX_RANGE
        assert (p['odom_frame_id'], p['global_frame_id'], p['scan_topic']) == (
            'odom',
            'map',
            '/scan',
        )
        assert p['robot_model_type'] == 'nav2_amcl::DifferentialMotionModel'
        assert all(p[f'alpha{i}'] <= 0.05 for i in range(1, 5))
        assert p['sigma_hit'] <= 0.1 and p['z_hit'] >= 0.9 and p['z_rand'] <= 0.1
        manager = data['lifecycle_manager_localization']['ros__parameters']
        assert manager == {'autostart': True, 'node_names': ['map_server', 'amcl']}


class TestLakiBeam:
    def test_one_minimum_range_that_hides_the_car_and_keeps_a_wall_at_the_nose(self):
        amcl = yaml.safe_load((CONFIG / 'amcl.yaml').read_text())['amcl']['ros__parameters']
        node = yaml.safe_load((CONFIG / 'autonomy.yaml').read_text())
        ranges = {
            *(_slam_params(name)['min_laser_range'] for name in SLAM_CONFIGS),
            amcl['laser_min_range'],
            node['autonomy_node']['ros__parameters']['fit_min_range_m'],
        }
        assert len(ranges) == 1
        (min_range,) = ranges
        # The rear of the outline lies in the LakiBeam's blind span; its nearest
        # edge in the live window is a side.
        f = yaml.safe_load((CONFIG / 'frames.yaml').read_text())
        side = min(
            f['footprint_left_y_mm'] - f['laser_y_mm'], f['laser_y_mm'] - f['footprint_right_y_mm']
        )
        assert side / 1e3 <= min_range < CAR_WALL_RANGE

    @pytest.mark.parametrize('name', SLAM_CONFIGS)
    def test_the_time_interval_sets_the_match_rate(self, name):
        # 2.6.10 throttles after the transform wait, so throttle_scans saves no
        # work; minimum_time_interval holds matching to 2 scans a second.
        p = _slam_params(name)
        assert p['throttle_scans'] == 1 and p['minimum_time_interval'] == 0.5


class TestExtend:
    def test_extend_loads_the_saved_graph_near_the_pose(self, autonomy):
        specs = autonomy.node_specs(
            'mapping', 'amcl', 'hall', '1,2,0.5', str(CONFIG), session='s', extend='/m/room/room'
        )
        assert _dicts(specs[0]) == {
            'map_file_name': '/m/room/room',
            'map_start_pose': [1.0, 2.0, 0.5],
        }
        node = _dicts(specs[-1])
        assert node['resumed_from_autosave'] is False and node['pose_source'] == 'pose'
        assert node['source'] == '/m/room/room' and node['extended_from'] == 'room'

    def test_extend_an_autosave_names_the_map(self, autonomy):
        spec = autonomy.node_specs(
            'mapping', 'amcl', 'hall', '0,0,0', str(CONFIG), extend='/m/room/autosave/room'
        )[-1]
        assert _dicts(spec)['extended_from'] == 'room'

    def test_resume_carries_from_the_autosave(self, autonomy):
        spec = autonomy.node_specs(
            'mapping', 'amcl', 'room', '0,0,0', str(CONFIG), resume='/m/room/autosave/room'
        )[-1]
        assert _dicts(spec)['source'] == '/m/room/autosave/room'
        assert _dicts(spec)['extended_from'] == ''

    @pytest.mark.parametrize(
        'mode,resume', [('localization', ''), ('mapping', '/m/room/autosave/room')]
    )
    def test_bad_extend(self, autonomy, mode, resume):
        with pytest.raises(ValueError):
            autonomy.slam_specs(mode, 'amcl', 'room', '', str(CONFIG), resume, '/m/room/room')


def test_foxglove_stops_quickly(autonomy):
    specs = autonomy.node_specs('mapping', 'amcl', 'r', '', str(CONFIG), foxglove=True)
    assert autonomy._stop_timeouts(specs[-1]) == {'sigterm_timeout': '2', 'sigkill_timeout': '2'}
    assert autonomy._stop_timeouts(specs[0]) == {}


def test_foxglove_layout_uses_only_bridged_topics():
    layout = json.loads((CONFIG / 'foxglove_layout.json').read_text())
    bridge = yaml.safe_load((CONFIG / 'foxglove.yaml').read_text())
    allowed = bridge['foxglove_bridge']['ros__parameters']['topic_whitelist']
    panels = layout['configById']
    topics = set(panels['3D!map']['topics'])
    topics.add(panels['Table!status']['topicPath'].split('.')[0])
    topics |= {p['value'].split('.')[0] for p in panels['Plot!pose']['paths']}
    for topic in topics:
        assert any(re.match(pattern, topic) for pattern in allowed), f'{topic} not bridged'
    client = bridge['foxglove_bridge']['ros__parameters']['client_topic_whitelist']
    estimate = panels['3D!map']['publish']['poseEstimateTopic']
    assert any(re.match(pattern, estimate) for pattern in client)
    # Every configured panel appears in the mosaic layout.
    text = json.dumps(layout['layout'])
    assert all(f'"{panel}"' in text for panel in panels)


# foxglove_bridge 3.4.3's capability names (common.hpp in foxglove-sdk).
BRIDGE_CAPABILITIES = {
    'clientPublish',
    'parameters',
    'parametersSubscribe',
    'services',
    'connectionGraph',
    'assets',
    'time',
}
# Topics the Foxglove layout reads that the topic contract has no row for, with
# the driver sources that start their publishers on the NeoRacer.
LAYOUT_TOPICS_OUTSIDE_THE_CONTRACT = {
    # AMCL publishes its particles in localization with localizer:=amcl.
    '/particle_cloud': (contract.AMCL_NODE,),
    # robot_state_publisher in the teleop frames publishes the URDF.
    '/robot_description': (
        ('launch/frames.launch.py', "executable='robot_state_publisher'"),
        (contract.TELEOP, "'frames': 'true'"),
    ),
}


def _layout_topics(node, key=''):
    """Return the topics a layout's panels read; its publish settings name client topics."""
    if key == 'publish':
        return set()
    if isinstance(node, dict):
        found = {k.split('.')[0] for k in node if k.startswith('/')}
        return found.union(*(_layout_topics(v, k) for k, v in node.items()))
    if isinstance(node, list):
        return set().union(*(_layout_topics(v, key) for v in node))
    if isinstance(node, str) and node.startswith('/'):
        return {node.split('.')[0]}
    return set()


class TestFoxgloveOnTheNeoRacer:
    def test_the_layout_reads_only_topics_the_neoracer_publishes(self):
        topics = _layout_topics(json.loads((CONFIG / 'foxglove_layout.json').read_text()))
        assert {'/map', '/scan', '/autonomy/pose', '/autonomy/status'} <= topics
        # The topic contract's live rows are what the NeoRacer publishes. Its
        # own tests check every row against the driver sources.
        published = {r.name for r in contract.LIVE if r.kind == 'topic' and r.publishers}
        for topic in topics - published:
            assert topic in LAYOUT_TOPICS_OUTSIDE_THE_CONTRACT, f'nothing publishes {topic}'
            for path, text in LAYOUT_TOPICS_OUTSIDE_THE_CONTRACT[topic]:
                source = contract._squash(contract._code_text(ROOT / path))
                assert contract._squash(text) in source, f'{topic}: {path} lacks {text!r}'

    def test_no_client_publishes_while_the_bridge_ignores_its_whitelist(self):
        # foxglove_bridge 3.x reads client_topic_whitelist but does not apply it
        # (foxglove-sdk PR #1508), so clientPublish would let a client publish
        # /drive and /motor. Humble cannot load an empty list, so one name stays.
        bridge = yaml.safe_load((CONFIG / 'foxglove.yaml').read_text())
        capabilities = bridge['foxglove_bridge']['ros__parameters']['capabilities']
        assert capabilities and not set(capabilities) & BRIDGE_CAPABILITIES

    def test_the_setup_installs_the_layout_beside_the_bridge_config(self):
        setup = (ROOT / 'setup.py').read_text()
        assert "glob('config/*.yaml')" in setup and "glob('config/*.json')" in setup

    def test_the_launch_starts_the_bridge_with_config_foxglove_yaml(
        self, autonomy, car_profile_dir
    ):
        shipped = CONFIG / 'foxglove.yaml'
        spec = autonomy.foxglove_spec(str(CONFIG))
        assert spec['parameters'] == [str(shipped)]
        # ROS applies a parameter file's values to the node named at its top.
        assert list(yaml.safe_load(shipped.read_text())) == [spec['name']] == ['foxglove_bridge']
        (car_profile_dir / 'params').mkdir(parents=True)
        override = car_profile_dir / 'params' / 'foxglove.yaml'
        override.write_text('foxglove_bridge: {ros__parameters: {}}\n')
        assert autonomy.foxglove_spec(str(CONFIG))['parameters'] == [str(shipped), str(override)]


def _declared_defaults():
    """Return {name: default} for every p('name', default) in AutonomyNode.__init__."""
    out = {}
    for node in ast.walk(ast.parse(NODE.read_text())):
        if (
            isinstance(node, ast.Call)
            and isinstance(node.func, ast.Name)
            and node.func.id == 'p'
            and len(node.args) == 2
        ):
            out[ast.literal_eval(node.args[0])] = ast.literal_eval(node.args[1])
    return out


class TestNodeSource:
    def test_the_yaml_sets_only_declared_parameters_of_their_type(self):
        # The NeoRacer is tuned in the YAML, so the values may differ from the
        # code's defaults, which stay upstream's. The type may not: Humble
        # stops the node at startup on, for example, fit_rate_hz: 2 for 2.0.
        declared = _declared_defaults()
        shipped = yaml.safe_load((CONFIG / 'autonomy.yaml').read_text())
        params = shipped['autonomy_node']['ros__parameters']
        assert params, 'config/autonomy.yaml sets nothing'
        for name, value in params.items():
            assert name in declared, f'{name} is not declared by autonomy_node'
            # Same type both ways: an int where a float is declared stops the node
            # at startup on Humble, and bool must not pass as int.
            default = declared[name]
            assert isinstance(value, type(default)) and isinstance(default, type(value)), name

    def test_it_publishes_no_transform(self):
        # odom -> base_link comes from the controller and map -> odom from the
        # SLAM or localizer; a broadcaster here would give a frame two parents.
        names = {
            n.id if isinstance(n, ast.Name) else n.attr
            for n in ast.walk(ast.parse(NODE.read_text()))
            if isinstance(n, (ast.Name, ast.Attribute))
        }
        assert not names & {'TransformBroadcaster', 'StaticTransformBroadcaster'}
        assert "'/tf'" not in NODE.read_text()

    def test_package_name_is_this_package(self):
        package = re.search(r'<name>(.+)</name>', (ROOT / 'package.xml').read_text()).group(1)
        assert f"PACKAGE = '{package}'" in NODE.read_text()

    def test_the_entry_point_is_installed(self):
        setup = (ROOT / 'setup.py').read_text()
        assert "'autonomy_node = neoracer_ros2_driver.autonomy_node:main'" in setup

    def test_the_status_carries_the_mode_the_throttle_node_reads(self):
        # throttle_node.status_mode() looks for the key 'mode' (F14).
        text = NODE.read_text()
        assert 'mode=mode or self._mode' in text
        assert "self._publish_status(mode='off')" in text
