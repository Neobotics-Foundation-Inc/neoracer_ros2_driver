"""Shared test helpers: load scripts/*.py by path (scripts/ is not a package)."""

import importlib.util
from pathlib import Path
import sys
from types import ModuleType

import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'
# Script folders searched by load_script, in order.
SCRIPT_DIRS = (SCRIPTS, SCRIPTS / 'calibration', SCRIPTS / 'validation')


def load_script(name: str) -> ModuleType:
    """
    Import scripts/<name>.py and register it in sys.modules under `name`.

    Registration is required: dataclasses resolve annotations through
    sys.modules, and dashboard.py and watchdog.py import sysinfo by that name.
    A module already loaded from the same file is returned as is, so every
    test file sees one module object to monkeypatch.
    """
    path = next((d / f'{name}.py' for d in SCRIPT_DIRS if (d / f'{name}.py').is_file()), None)
    assert path is not None, f'{name}.py not found in {SCRIPT_DIRS}'
    existing = sys.modules.get(name)
    if existing is not None and Path(getattr(existing, '__file__', '') or '').resolve() == path:
        return existing
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='session', autouse=True)
def _car_profile_session(tmp_path_factory):
    """Point the car profile away from the car's own before module fixtures run."""
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv('RACECAR_CONFIG_DIR', str(tmp_path_factory.mktemp('car_profile_session')))
        yield


@pytest.fixture(autouse=True)
def car_profile_dir(_car_profile_session, tmp_path, monkeypatch):
    """Point the car profile at a temporary folder, so no test touches the car's own."""
    path = tmp_path / 'car_profile'
    monkeypatch.setenv('RACECAR_CONFIG_DIR', str(path))
    return path


@pytest.fixture(scope='session', autouse=True)
def _data_root_session(tmp_path_factory):
    """
    Point the data root at a temporary folder for the whole session.

    Session scope runs before module fixtures that import scripts, which read
    the data root into module constants (LOG_DIR, DATASET). The data_root
    module arrives with the data root port; until then this fixture only sets
    the environment variable.
    """
    path = tmp_path_factory.mktemp('data_root')
    with pytest.MonkeyPatch.context() as mp:
        mp.setenv('RACECAR_DATA_DIR', str(path))
        yield path


@pytest.fixture
def data_root_dir(_data_root_session):
    return _data_root_session
