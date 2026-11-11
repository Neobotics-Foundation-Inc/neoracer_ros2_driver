"""Tests for neoracer_ros2_driver.data_root: where the car's data goes."""

from pathlib import Path

from neoracer_ros2_driver import data_root
import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'


@pytest.fixture
def fresh(monkeypatch):
    data_root.reset()
    yield
    data_root.reset()


class TestResolve:
    def test_default_root_under_home(self):
        assert data_root.resolve() == (Path.home() / 'racecar_data', '')

    def test_default_is_outside_the_folders_the_installer_removes(self):
        root = data_root.fallback()
        assert root.parent == Path.home()
        assert root.name not in ('data', 'logs', 'jupyter_ws', 'ros2_ws')

    def test_writable_checks_by_writing(self, tmp_path):
        assert data_root.writable(tmp_path)
        assert not data_root.writable(tmp_path / 'missing')


class TestCurrent:
    def test_env_first(self, fresh, monkeypatch, tmp_path):
        monkeypatch.setenv(data_root.DATA_ENV, str(tmp_path))
        assert data_root.current() == (tmp_path, '')

    def test_run_file_next(self, fresh, monkeypatch, tmp_path):
        monkeypatch.delenv(data_root.DATA_ENV)
        run_file = tmp_path / 'data_root'
        run_file.write_text('/home/racecar/racecar_data\n')
        monkeypatch.setattr(data_root, 'RUN_FILE', run_file)
        assert data_root.current()[0] == Path('/home/racecar/racecar_data')

    def test_default_last(self, fresh, monkeypatch, tmp_path):
        monkeypatch.delenv(data_root.DATA_ENV)
        monkeypatch.setattr(data_root, 'RUN_FILE', tmp_path / 'missing')
        assert data_root.current() == (Path.home() / 'racecar_data', '')

    def test_worked_out_once(self, fresh, monkeypatch, tmp_path):
        monkeypatch.setenv(data_root.DATA_ENV, str(tmp_path / 'a'))
        first = data_root.current()
        monkeypatch.setenv(data_root.DATA_ENV, str(tmp_path / 'b'))
        assert data_root.current() == first

    def test_data_dir_creates_the_folder(self, data_root_dir):
        assert data_root.data_dir('bags') == data_root_dir / 'bags'
        assert (data_root_dir / 'bags').is_dir()


def test_boot_service_writes_both_files(monkeypatch, tmp_path, capsys):
    monkeypatch.setattr(data_root, 'RUN_FILE', tmp_path / 'data_root')
    monkeypatch.setattr(data_root, 'RUN_ENV_FILE', tmp_path / 'environment')
    root = tmp_path / 'home_data'
    monkeypatch.setattr(data_root, 'resolve', lambda: (root, ''))
    monkeypatch.setattr('sys.argv', ['data_root.py', 'write'])
    assert data_root.main() == 0
    assert (tmp_path / 'data_root').read_text() == f'{root}\n'
    assert (tmp_path / 'environment').read_text() == f'RACECAR_DATA_DIR={root}\n'
    assert root.is_dir()
    assert f'Data root: {root}' in capsys.readouterr().out


def test_units_and_shells_get_the_variable():
    for unit in ('teleop', 'dashboard', 'jupyter', 'watchdog', 'autonomy'):
        text = (SCRIPTS / f'neoracer-{unit}.service').read_text()
        assert 'EnvironmentFile=-/run/racecar/environment' in text, unit
        assert 'After=neoracer-data.service' in text, unit
    service = (SCRIPTS / 'neoracer-data.service').read_text()
    assert 'data_root.py write' in service and 'RuntimeDirectoryPreserve=yes' in service
    assert 'RACECAR_DATA_DIR' in (SCRIPTS / 'profile.d-racecar-data.sh').read_text()
    assert 'neoracer-data.service' in (SCRIPTS / 'setup_services.sh').read_text()
    assert 'export RACECAR_DATA_DIR' in (SCRIPTS / 'racecar-tool.sh').read_text()
    assert 'racecar_data' in (SCRIPTS / 'setup_jupyter.sh').read_text()


def test_session_problem(data_root_dir, monkeypatch):
    assert data_root.session_problem() == ''
    monkeypatch.setattr(data_root, 'writable', lambda p: False)
    assert 'cannot be written' in data_root.session_problem()
