"""Tests for neoracer_ros2_driver.car_profile and scripts/neoracer_profile.py."""

from datetime import datetime
from pathlib import Path

from conftest import load_script
from neoracer_ros2_driver import car_profile
import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent
NOW = datetime(2026, 10, 24, 12, 0, 0)


class TestResolver:
    def test_env_wins(self, car_profile_dir):
        assert car_profile.profile_dir() == car_profile_dir

    def test_xdg_then_home(self, monkeypatch, tmp_path):
        monkeypatch.delenv(car_profile.PROFILE_ENV)
        monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path / 'xdg'))
        assert car_profile.profile_dir() == tmp_path / 'xdg' / 'racecar'
        monkeypatch.delenv('XDG_CONFIG_HOME')
        monkeypatch.setenv('HOME', str(tmp_path / 'home'))
        assert car_profile.profile_dir() == tmp_path / 'home' / '.config' / 'racecar'

    def test_paths(self, car_profile_dir):
        assert car_profile.param_path('odom.yaml') == car_profile_dir / 'params' / 'odom.yaml'
        assert car_profile.param_path('odom') == car_profile_dir / 'params' / 'odom.yaml'
        assert car_profile.dashboard_settings('teleop') == (
            car_profile_dir / 'dashboards' / 'teleop.yaml'
        )

    def test_overrides_only_for_existing_files(self, car_profile_dir):
        assert car_profile.param_override('odom') is None
        car_profile.write_values('odom', {'a': 1})
        assert car_profile.param_overrides(['mux.yaml', 'odom.yaml']) == [
            str(car_profile_dir / 'params' / 'odom.yaml')
        ]


class TestWrites:
    def test_write_archives_the_replaced_file(self, car_profile_dir):
        path = car_profile.param_path('frames')
        car_profile.write_text(path, 'laser_yaw_deg: 180.0\n', NOW)
        car_profile.write_text(path, 'laser_yaw_deg: 179.0\n', NOW)
        old = car_profile_dir / 'history' / '20261024-120000' / 'params' / 'frames.yaml'
        assert old.read_text() == 'laser_yaw_deg: 180.0\n'
        assert path.read_text() == 'laser_yaw_deg: 179.0\n'
        assert not list(path.parent.glob('.*.tmp'))

    def test_files_outside_the_profile_are_not_archived(self, tmp_path, car_profile_dir):
        outside = tmp_path / 'elsewhere.yaml'
        outside.write_text('a: 1\n')
        car_profile.write_text(outside, 'a: 2\n', NOW)
        assert outside.read_text() == 'a: 2\n'
        assert not (car_profile_dir / 'history').exists()

    def test_write_params_merge(self):
        car_profile.write_params('odom', 'controller_node', {'a': 1})
        car_profile.write_params('odom', 'controller_node', {'b': 2}, merge=True)
        data = yaml.safe_load(car_profile.param_path('odom').read_text())
        assert data == {'controller_node': {'ros__parameters': {'a': 1, 'b': 2}}}

    def test_node_params_layers_the_profile_on_the_defaults(self, tmp_path):
        (tmp_path / 'odom.yaml').write_text(
            yaml.safe_dump({'controller_node': {'ros__parameters': {'a': 0, 'b': 0}}})
        )
        car_profile.write_params('odom', 'controller_node', {'a': 5})
        assert car_profile.node_params('odom', 'controller_node', tmp_path) == {'a': 5, 'b': 0}

    def test_car_info_defaults_to_the_host_name(self, car_profile_dir):
        assert car_profile.car_info()['name']
        car_profile.ensure_profile()
        info = yaml.safe_load((car_profile_dir / 'car.yaml').read_text())
        assert info['profile_version'] == car_profile.PROFILE_VERSION


@pytest.fixture
def tree(tmp_path):
    config = tmp_path / 'driver' / 'config'
    dashboards = tmp_path / 'driver' / 'scripts' / 'dashboards'
    (dashboards / 'webteleop').mkdir(parents=True)
    config.mkdir(parents=True)
    (config / 'odom.yaml').write_text('shipped\n')
    (config / 'odom.local.yaml').write_text('odom on this car\n')
    (config / 'frames.local.yaml').write_text('laser_yaw_deg: 179.6\n')
    (dashboards / 'webteleop' / 'teleop.local.yaml').write_text('speed: 0.4\n')
    return config, dashboards


class TestMigrate:
    def test_moves_every_per_car_file(self, tree, car_profile_dir):
        config, dashboards = tree
        assert len(car_profile.unmigrated(config, dashboards)) == 3
        lines = car_profile.migrate(config, dashboards, NOW)
        assert len(lines) == 3
        assert (car_profile_dir / 'params' / 'odom.yaml').read_text() == 'odom on this car\n'
        assert (car_profile_dir / 'params' / 'frames.yaml').exists()
        assert (car_profile_dir / 'dashboards' / 'teleop.yaml').read_text() == 'speed: 0.4\n'
        assert sorted(p.name for p in config.iterdir()) == ['odom.yaml']
        assert (car_profile_dir / 'car.yaml').exists()
        assert car_profile.unmigrated(config, dashboards) == []

    def test_rerun_changes_nothing(self, tree, car_profile_dir):
        config, dashboards = tree
        car_profile.migrate(config, dashboards, NOW)
        before = {p: p.read_bytes() for p in car_profile_dir.rglob('*') if p.is_file()}
        assert car_profile.migrate(config, dashboards, NOW) == []
        after = {p: p.read_bytes() for p in car_profile_dir.rglob('*') if p.is_file()}
        assert before == after

    def test_driver_copy_wins_and_the_old_one_is_kept(self, tree, car_profile_dir):
        config, dashboards = tree
        car_profile.write_text(car_profile.param_path('odom'), 'older\n')
        car_profile.migrate(config, dashboards, NOW)
        assert car_profile.param_path('odom').read_text() == 'odom on this car\n'
        old = car_profile_dir / 'history' / '20261024-120000' / 'params' / 'odom.yaml'
        assert old.read_text() == 'older\n'

    def test_hand_kept_backup_is_reported_and_moved_aside(self, tree, car_profile_dir):
        config, dashboards = tree
        backup = car_profile_dir / 'calibration'
        backup.mkdir(parents=True)
        (backup / 'odom.local.yaml').write_text('stale copy\n')
        (backup / 'README.md').write_text('old instructions\n')
        lines = car_profile.migrate(config, dashboards, NOW)
        assert any('differs' in line and 'odom.local.yaml' in line for line in lines)
        assert not backup.exists()
        kept = car_profile_dir / 'history' / '20261024-120000' / 'calibration'
        assert (kept / 'odom.local.yaml').read_text() == 'stale copy\n'

    def test_same_content_already_in_the_profile(self, tree):
        config, dashboards = tree
        car_profile.write_text(car_profile.param_path('frames'), 'laser_yaw_deg: 179.6\n')
        lines = car_profile.migrate(config, dashboards, NOW)
        assert any('already in the profile' in line for line in lines)
        assert not (config / 'frames.local.yaml').exists()


class TestExportImport:
    def test_round_trip(self, tmp_path, car_profile_dir, monkeypatch):
        car_profile.ensure_profile()
        car_profile.write_values('frames', {'laser_yaw_deg': 179.6})
        car_profile.write_values('frames', {'laser_yaw_deg': 179.5})
        archive = car_profile.export(tmp_path / 'car.tar.gz')

        other = tmp_path / 'other'
        monkeypatch.setenv(car_profile.PROFILE_ENV, str(other))
        lines = car_profile.import_profile(archive, NOW)
        assert 'Imported params/frames.yaml.' in lines
        assert yaml.safe_load((other / 'params' / 'frames.yaml').read_text()) == {
            'laser_yaw_deg': 179.5
        }
        assert not (other / 'history').exists()
        assert car_profile.import_profile(archive, NOW) == []

    def test_import_keeps_the_replaced_file(self, tmp_path, car_profile_dir):
        car_profile.write_values('frames', {'laser_yaw_deg': 179.6})
        archive = car_profile.export(tmp_path / 'car.tar.gz')
        car_profile.write_values('frames', {'laser_yaw_deg': 170.0})
        car_profile.import_profile(archive, NOW)
        kept = car_profile_dir / 'history' / '20261024-120000' / 'params' / 'frames.yaml'
        assert 'laser_yaw_deg: 170.0' in kept.read_text()


class TestCommand:
    def test_path_and_show(self, car_profile_dir, capsys):
        cli = load_script('neoracer_profile')
        assert cli.main(['path']) == 0
        assert capsys.readouterr().out.strip() == str(car_profile_dir)
        assert cli.main([]) == 0
        assert f'Car profile: {car_profile_dir}' in capsys.readouterr().out

    def test_check_names_ignored_files(self, tree, monkeypatch, capsys):
        cli = load_script('neoracer_profile')
        config, dashboards = tree
        monkeypatch.setattr(cli, 'PACKAGE_ROOT', config.parent)
        monkeypatch.setattr(cli, 'HERE', dashboards.parent)
        assert cli.main(['check']) == 0
        out = capsys.readouterr().out
        assert out.count('is not used. Run: racecar profile migrate') == 3

    def test_tool_and_setup_run_it(self):
        tool = (ROOT / 'scripts' / 'racecar-tool.sh').read_text()
        assert 'scripts/neoracer_profile.py' in tool
        assert 'neoracer_profile.py" migrate' in (ROOT / 'scripts' / 'setup_all.sh').read_text()
        assert 'neoracer_profile.py" check' in (ROOT / 'scripts' / 'launch_teleop.sh').read_text()


def test_the_driver_folder_has_no_per_car_files_in_git():
    ignored = (ROOT / '.gitignore').read_text()
    assert 'config/*.local.yaml' in ignored
