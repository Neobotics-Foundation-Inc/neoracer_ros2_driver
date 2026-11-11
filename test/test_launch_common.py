"""Unit tests for launch_common: parameter-file order and per-car overrides."""

from pathlib import Path

from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
import pytest

from neoracer_ros2_driver import launch_common
from neoracer_ros2_driver.launch_common import local_overrides, single_node_launch

# Every per-node launch file ships one config YAML and exposes it as one argument.
SINGLE_NODE_LAUNCHES = (
    'camera',
    'controller',
    'gamepad',
    'inference',
    'led_matrix',
    'mux',
    'throttle',
)


@pytest.fixture
def share(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> Path:
    (tmp_path / 'config').mkdir()
    monkeypatch.setattr(launch_common, 'get_package_share_directory', lambda _: str(tmp_path))
    return tmp_path / 'config'


def _params(ld):
    node = next(e for e in ld.entities if isinstance(e, Node))
    return node._Node__parameters


def _args(ld):
    return [e.name for e in ld.entities if isinstance(e, DeclareLaunchArgument)]


class TestLocalOverrides:
    def test_only_existing_files_in_input_order(self, car_profile_dir):
        params = car_profile_dir / 'params'
        params.mkdir(parents=True)
        (params / 'b.yaml').write_text('')
        (params / 'a.yaml').write_text('')
        got = local_overrides(['a.yaml', 'missing.yaml', 'b.yaml'])
        assert got == [str(params / 'a.yaml'), str(params / 'b.yaml')]

    def test_none_present(self):
        assert local_overrides(['a.yaml']) == []

    def test_the_driver_folder_is_never_read(self, share):
        (share / 'a.local.yaml').write_text('')
        assert local_overrides(['a.yaml']) == []


class TestSingleNodeLaunch:
    def test_declares_one_arg_per_yaml(self, share):
        ld = single_node_launch(
            'controller_config',
            'controller.yaml',
            'neoracer_ros2_driver',
            'controller',
            extra_yamls=(('odom_config', 'odom.yaml'),),
        )
        assert _args(ld) == ['controller_config', 'odom_config']

    def test_locals_load_after_all_shipped_files(self, share, car_profile_dir):
        overrides = car_profile_dir / 'params'
        overrides.mkdir(parents=True)
        (overrides / 'controller.yaml').write_text('')
        (overrides / 'odom.yaml').write_text('')
        ld = single_node_launch(
            'controller_config',
            'controller.yaml',
            'neoracer_ros2_driver',
            'controller',
            extra_yamls=(('odom_config', 'odom.yaml'),),
        )
        params = _params(ld)
        assert len(params) == 4
        locals_ = [p.param_file[0].text for p in params[2:]]
        assert locals_ == [str(overrides / 'controller.yaml'), str(overrides / 'odom.yaml')]

    def test_no_locals_means_shipped_files_only(self, share):
        ld = single_node_launch('mux_config', 'mux.yaml', 'neoracer_ros2_driver', 'mux_node')
        assert len(_params(ld)) == 1


class TestShippedLaunchFiles:
    """Each per-node launch file exposes exactly its own config argument."""

    @staticmethod
    def _load(name):
        import importlib.util

        path = Path(__file__).parent.parent / 'launch' / f'{name}.launch.py'
        spec = importlib.util.spec_from_file_location(name, path)
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)
        return mod.generate_launch_description()

    @pytest.mark.parametrize('name', SINGLE_NODE_LAUNCHES)
    def test_one_config_arg_each(self, share, name):
        assert _args(self._load(name)) == [f'{name}_config']
