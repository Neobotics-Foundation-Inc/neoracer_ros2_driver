"""
Unit tests for maps.py and autonomy_node.py's setup, read from the source.

The autonomy launch and the SLAM configs arrive with the autonomy launch
feature, with their own tests here.
"""

import ast
from pathlib import Path
import re

from neoracer_ros2_driver import maps
import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / 'config'
NODE = ROOT / 'neoracer_ros2_driver' / 'autonomy_node.py'


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
    def test_the_yaml_sets_only_declared_parameters_at_their_defaults(self):
        declared = _declared_defaults()
        shipped = yaml.safe_load((CONFIG / 'autonomy.yaml').read_text())
        params = shipped['autonomy_node']['ros__parameters']
        assert params, 'config/autonomy.yaml sets nothing'
        for name, value in params.items():
            assert name in declared, f'{name} is not declared by autonomy_node'
            assert value == declared[name], name

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
