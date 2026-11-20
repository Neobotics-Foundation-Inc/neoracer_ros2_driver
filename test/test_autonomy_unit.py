"""The autonomy unit: neoracer-autonomy.service, its launcher, mode file, and stop script."""

from configparser import ConfigParser
from datetime import datetime
import os
from pathlib import Path
import subprocess

from conftest import load_script
from neoracer_ros2_driver import autonomy as au
import pytest

SCRIPTS = Path(__file__).resolve().parent.parent / 'scripts'
MODE_FILE = 'autonomy.conf'


@pytest.fixture(scope='module')
def args():
    return load_script('autonomy_args')


@pytest.fixture(scope='module')
def unit():
    parser = ConfigParser(strict=False, interpolation=None)
    parser.optionxform = str
    parser.read(SCRIPTS / 'neoracer-autonomy.service')
    return parser


class TestUnit:
    def test_rides_on_teleop(self, unit):
        u = unit['Unit']
        assert u['After'] == u['BindsTo'] == u['PartOf'] == 'neoracer-teleop.service'

    def test_restarts_after_a_crash_but_not_a_stop(self, unit):
        # ros2 launch exits 0 when slam_toolbox dies; on-failure would never restart it.
        assert unit['Service']['Restart'] == 'always'
        assert int(unit['Unit']['StartLimitBurst']) == 3

    def test_not_enabled_at_boot(self, unit):
        assert 'Install' not in unit

    def test_mode_file_and_stop_sequence(self, unit):
        s = unit['Service']
        assert s['EnvironmentFile'].startswith('-') and s['EnvironmentFile'].endswith(MODE_FILE)
        assert s['ExecStart'].endswith('scripts/launch_autonomy.sh')
        assert 'autonomy_stop.py $MAINPID' in s['ExecStop']
        assert s['User'] == 'racecar' and s['KillSignal'] == 'SIGINT'
        assert int(s['TimeoutStopSec']) >= 30

    def test_launcher_uses_the_mode_file_arguments(self):
        text = (SCRIPTS / 'launch_autonomy.sh').read_text()
        assert 'autonomy_args.py' in text
        # tee must outlive the stop signal, or launch exits 120 on a broken pipe.
        assert "exec &> >(trap '' INT TERM; exec tee -a" in text
        assert 'exec ros2 launch neoracer_ros2_driver autonomy.launch.py "${ARGS[@]}"' in text


class TestNeoRacerUnit:
    """What the NeoRacer unit adds to upstream's: names, the data root, Humble."""

    def test_maps_land_under_the_users_data_root(self):
        # ConfigParser keeps the last of two EnvironmentFile lines, so read the text.
        text = (SCRIPTS / 'neoracer-autonomy.service').read_text()
        assert 'EnvironmentFile=-/run/racecar/environment\n' in text
        assert 'Environment=HOME=/home/racecar\n' in text
        writer = (SCRIPTS / 'neoracer-data.service').read_text()
        assert 'neoracer-autonomy.service' in writer.split('[Service]')[0]

    def test_the_stop_outlasts_the_autonomy_node(self, unit):
        # autonomy_node takes up to about 3 s to send mode off (1 s for its
        # executor, up to 2 s for the acknowledgements), and ros2 launch gives
        # each node 5 s after SIGINT before SIGTERM.
        assert int(unit['Service']['TimeoutStopSec']) > 3 + 5

    def test_humble_and_no_discovery_range(self, unit):
        text = (SCRIPTS / 'neoracer-autonomy.service').read_text()
        launcher = (SCRIPTS / 'launch_autonomy.sh').read_text()
        for source in (text, launcher):
            assert 'source /opt/ros/humble/setup.bash' in source
            assert 'jazzy' not in source and 'ROS_AUTOMATIC_DISCOVERY_RANGE' not in source
        assert 'osracer' not in launcher

    @pytest.mark.parametrize(
        'name', ['launch_autonomy.sh', 'autonomy_args.py', 'autonomy_stop.py']
    )
    def test_scripts_are_executable_with_a_hashbang(self, name):
        # The autonomy rows of upstream's test_setup_scripts.py.
        path = SCRIPTS / name
        first = path.read_text().splitlines()[0]
        assert first == ('#!/bin/bash' if name.endswith('.sh') else '#!/usr/bin/env python3')
        assert os.access(path, os.X_OK), f'{name} missing +x bit'

    def test_launcher_passes_bash_syntax(self):
        result = subprocess.run(
            ['bash', '-n', str(SCRIPTS / 'launch_autonomy.sh')],
            capture_output=True,
            text=True,
            timeout=10,
        )
        assert result.returncode == 0, result.stderr

    def test_setup_installs_what_the_launch_starts(self):
        text = (SCRIPTS / 'setup_ros2.sh').read_text()
        for package in ('slam-toolbox', 'nav2-amcl', 'nav2-map-server', 'nav2-lifecycle-manager'):
            assert f'ros-humble-{package}' in text, package


class TestModeFile:
    def test_mapping(self, args, tmp_path, monkeypatch):
        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path))
        got = args.launch_args({'AUTONOMY_MAP': 'room', 'AUTONOMY_SESSION': 's1'})
        assert got == ['mode:=mapping', 'map:=room', 'session:=s1']

    def test_localization(self, args):
        env = {
            'AUTONOMY_MODE': 'localization',
            'AUTONOMY_MAP': 'room',
            'AUTONOMY_POSE': '-1.5,2,0.3',
            'AUTONOMY_POSE_SOURCE': 'last_pose',
            'AUTONOMY_LOCALIZER': 'slam_toolbox',
            'AUTONOMY_FOXGLOVE': '1',
        }
        assert args.launch_args(env) == [
            'mode:=localization',
            'map:=room',
            'session:=',
            'localizer:=slam_toolbox',
            'pose:=-1.5,2,0.3',
            'pose_source:=last_pose',
            'foxglove:=true',
        ]

    def test_resume_from_the_same_session(self, args, tmp_path, monkeypatch):
        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path))
        auto = tmp_path / 'room' / au.AUTOSAVE_DIR
        auto.mkdir(parents=True)
        (auto / 'room.posegraph').write_text('g')
        au.write_yaml_atomic(
            auto / au.AUTOSAVE_FILE,
            au.pose_record((1.0, -2.0, 0.5), datetime(2026, 10, 9), session='s1'),
        )
        got = args.launch_args({'AUTONOMY_MAP': 'room', 'AUTONOMY_SESSION': 's1'})
        assert got[-2:] == [f'resume:={auto / "room"}', 'pose:=1.0,-2.0,0.5']
        fresh = args.launch_args({'AUTONOMY_MAP': 'room', 'AUTONOMY_SESSION': 's2'})
        assert not any(a.startswith('resume:=') for a in fresh)

    def test_extend_a_saved_map(self, args, tmp_path, monkeypatch):
        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path))
        _files(tmp_path / 'room', 'room')
        env = {
            'AUTONOMY_MAP': 'hall',
            'AUTONOMY_SESSION': 's1',
            'AUTONOMY_FROM': 'room',
            'AUTONOMY_POSE': '-1.5,2,0.3',
            'AUTONOMY_POSE_SOURCE': 'last_pose',
        }
        assert args.launch_args(env) == [
            'mode:=mapping',
            'map:=hall',
            'session:=s1',
            f'extend:={tmp_path / "room" / "room"}',
            'pose:=-1.5,2,0.3',
            'pose_source:=last_pose',
        ]

    def test_extend_an_unsaved_map_from_its_autosave(self, args, tmp_path, monkeypatch):
        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path))
        _files(tmp_path / 'room' / au.AUTOSAVE_DIR, 'room')
        got = args.launch_args({'AUTONOMY_MAP': 'hall', 'AUTONOMY_FROM': 'room'})
        assert f'extend:={tmp_path / "room" / au.AUTOSAVE_DIR / "room"}' in got
        assert 'pose:=0,0,0' in got

    def test_a_crashed_extension_resumes_its_own_autosave(self, args, tmp_path, monkeypatch):
        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path))
        _files(tmp_path / 'room', 'room')
        auto = tmp_path / 'hall' / au.AUTOSAVE_DIR
        _files(auto, 'hall')
        au.write_yaml_atomic(
            auto / au.AUTOSAVE_FILE,
            au.pose_record((1.0, 0.0, 0.0), datetime(2026, 10, 9), session='s1'),
        )
        env = {'AUTONOMY_MAP': 'hall', 'AUTONOMY_SESSION': 's1', 'AUTONOMY_FROM': 'room'}
        got = args.launch_args(env)
        assert f'resume:={auto / "hall"}' in got
        assert not any(a.startswith('extend:=') for a in got)

    @pytest.mark.parametrize(
        'env',
        [
            {'AUTONOMY_MAP': 'hall', 'AUTONOMY_FROM': 'missing'},
            {'AUTONOMY_MAP': 'hall', 'AUTONOMY_FROM': '../x'},
            {},
            {'AUTONOMY_MAP': '../x'},
            {'AUTONOMY_MAP': 'room', 'AUTONOMY_MODE': 'drive'},
            {'AUTONOMY_MAP': 'room', 'AUTONOMY_LOCALIZER': 'gmapping'},
        ],
    )
    def test_bad_values(self, args, env):
        with pytest.raises(ValueError):
            args.launch_args(env)


def _files(folder, name):
    folder.mkdir(parents=True, exist_ok=True)
    for suffix in au.MAP_SUFFIXES:
        (folder / f'{name}{suffix}').write_text('m')


def test_stop_waits_only_while_the_launch_runs():
    stop = load_script('autonomy_stop')
    assert stop.running(str(os.getpid()))
    assert not stop.running('0') and not stop.running('') and not stop.running('x')
    assert stop.SERVICE_WAIT_SEC >= 8 and stop.GONE_WAIT_SEC <= 1
