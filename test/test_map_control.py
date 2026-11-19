"""`racecar map`: scripts/map_control.py and scripts/neoracer_map.py, without ROS or systemd."""

from contextlib import nullcontext
from datetime import datetime
from pathlib import Path
import re
import subprocess
import time
from types import SimpleNamespace

from conftest import load_script
from neoracer_ros2_driver import autonomy as au, controller_lib as cl
import pytest
import yaml

NOW = datetime(2026, 10, 9, 14, 0, 0)


@pytest.fixture(scope='module')
def mc():
    return load_script('map_control')


@pytest.fixture(scope='module')
def cli():
    return load_script('neoracer_map')


@pytest.fixture
def root(tmp_path, monkeypatch, mc):
    maps = tmp_path / 'maps'
    maps.mkdir()
    monkeypatch.setenv('RACECAR_MAP_ROOT', str(maps))
    monkeypatch.setenv(mc.MODE_FILE_ENV, str(tmp_path / 'autonomy.conf'))
    monkeypatch.setattr(mc, 'current_frames', lambda: None)
    return maps


# /rc/channels as the dev car read it (N01, 2026-11-17): at rest with the mode
# switch and the speed switch UP, then the speed switch DOWN. A frame with no
# signal, and the frames while the handset reconnects, read 0.0 everywhere.
SWITCH_UP = [0.0, 0.0, 0.0, 0.0, -1.0, -1.0, -1.0, -1.0, 0.0, 0.0]
SWITCH_DOWN = [0.0, 0.0, 0.0, 0.0, -1.0, -1.0, -1.0, 1.0, 0.0, 0.0]
ALL_ZERO = [0.0] * 10


class Handset:
    """Stands in for /rc/channels and /rc/link: the speed switch UP and linked by default."""

    def __init__(self, real) -> None:
        self.real = real
        self.channels: list[float] | None = list(SWITCH_UP)
        self.link: bool | None = True
        self.reads = 0

    def __call__(self, timeout: float = 0.0) -> tuple[list[float] | None, bool | None]:
        self.reads += 1
        return self.channels, self.link


@pytest.fixture(autouse=True)
def handset(mc, monkeypatch):
    """Keep every test off the car's real handset; read_rc returns this reading."""
    fake = Handset(mc.read_rc)
    monkeypatch.setattr(mc, 'read_rc', fake)
    return fake


def files(folder: Path, name: str, text: str = 'm') -> None:
    folder.mkdir(parents=True, exist_ok=True)
    for suffix in au.MAP_SUFFIXES:
        (folder / f'{name}{suffix}').write_text(text)


def saved_map(root: Path, name: str, **meta) -> Path:
    folder = root / name
    files(folder, name)
    au.write_yaml_atomic(folder / au.META_FILE, {'saved': '2026-10-09 12:00:00', **meta})
    return folder


def autosave(root: Path, name: str, time: str = '2026-10-09 13:00:00', **meta) -> Path:
    folder = root / name / au.AUTOSAVE_DIR
    files(folder, name, 'auto')
    au.write_yaml_atomic(folder / au.AUTOSAVE_FILE, {'time': time, 'session': 's0'})
    au.write_yaml_atomic(folder / au.META_FILE, {'saved': time, **meta})
    return folder


def last_pose(root: Path, name: str, time: str = '2026-10-09 13:58:00') -> None:
    au.write_yaml_atomic(
        root / name / au.LAST_POSE_FILE, {'x': -1.5, 'y': 2.0, 'yaw': 0.3, 'time': time}
    )


class TestModeFile:
    def test_round_trip_in_key_order(self, mc, root):
        mc.write_mode_file({'AUTONOMY_MODE': 'mapping', 'AUTONOMY_MAP': 'room'})
        text = mc.mode_file().read_text()
        assert text.splitlines()[0] == 'AUTONOMY_MODE=mapping'
        assert [line.split('=')[0] for line in text.splitlines()] == list(mc.KEYS)
        assert mc.read_mode_file()['AUTONOMY_MAP'] == 'room'

    def test_foxglove_carries_over(self, mc, root):
        mc.set_foxglove(True, out=lambda _: None)
        mc.write_mode_file({'AUTONOMY_MODE': 'localization', 'AUTONOMY_MAP': 'room'})
        values = mc.read_mode_file()
        assert values['AUTONOMY_FOXGLOVE'] == '1' and values['AUTONOMY_MAP'] == 'room'
        mc.set_foxglove(False, out=lambda _: None)
        assert mc.read_mode_file()['AUTONOMY_MAP'] == 'room'
        assert mc.read_mode_file()['AUTONOMY_FOXGLOVE'] == '0'

    def test_every_key_is_read_by_the_unit(self, mc):
        text = (Path(mc.__file__).parent / 'autonomy_args.py').read_text()
        for key in mc.KEYS:
            assert key in text

    def test_missing_file(self, mc, root):
        assert mc.read_mode_file() == {}


class TestStartPlan:
    def test_new_map(self, mc, root):
        plan = mc.plan_start(root, 'room', 's1', now=NOW)
        assert plan.values == {
            'AUTONOMY_MODE': 'mapping',
            'AUTONOMY_MAP': 'room',
            'AUTONOMY_SESSION': 's1',
            'AUTONOMY_FROM': '',
        }
        assert not plan.replace and not plan.notes

    def test_existing_name_refused(self, mc, root):
        saved_map(root, 'room')
        with pytest.raises(mc.MapError) as err:
            mc.plan_start(root, 'room', 's1', now=NOW)
        assert err.value.code == 1 and '--force' in str(err.value)

    def test_force_replaces(self, mc, root):
        saved_map(root, 'room')
        assert mc.plan_start(root, 'room', 's1', force=True, now=NOW).replace

    @pytest.mark.parametrize(
        'kwargs',
        [
            {'name': 'a b'},
            {'name': 'replaced'},
            {'name': 'room', 'pose': '1,2,0'},
            {'name': 'room', 'source': 'room', 'force': True},
            {'name': 'room', 'source': '../x'},
        ],
    )
    def test_usage(self, mc, root, kwargs):
        name = kwargs.pop('name')
        with pytest.raises(mc.MapError) as err:
            mc.plan_start(root, name, 's1', now=NOW, **kwargs)
        assert err.value.code == 2

    def test_extend_from_the_last_pose(self, mc, root):
        saved_map(root, 'room', start_pose={'x': 9, 'y': 9, 'yaw': 0})
        last_pose(root, 'room')
        plan = mc.plan_start(root, 'hall', 's1', source='room', now=NOW)
        assert plan.values['AUTONOMY_FROM'] == 'room'
        assert plan.values['AUTONOMY_POSE'] == '-1.5000,2.0000,0.30000'
        assert plan.values['AUTONOMY_POSE_SOURCE'] == 'last_pose'
        assert not any('moved since' in n for n in plan.notes)

    def test_a_stale_last_pose_says_so(self, mc, root):
        saved_map(root, 'room')
        last_pose(root, 'room', time='2026-10-09 09:00:00')
        plan = mc.plan_start(root, 'hall', 's1', source='room', now=NOW)
        assert any('moved since' in n for n in plan.notes)

    def test_extend_refuses_a_pose_from_a_lost_run(self, mc, root):
        saved_map(root, 'room')
        last_pose(root, 'room')
        au.mark_pose_lost(root / 'room' / au.LAST_POSE_FILE, NOW)
        with pytest.raises(mc.MapError) as err:
            mc.plan_start(root, 'hall', 's1', source='room', now=NOW)
        assert 'not localized' in str(err.value) and '--pose' in str(err.value)
        plan = mc.plan_start(root, 'hall', 's1', source='room', pose='1,2,0', now=NOW)
        assert plan.values['AUTONOMY_POSE_SOURCE'] == 'pose'

    def test_extend_from_the_start_pose(self, mc, root):
        saved_map(root, 'room', start_pose={'x': 1, 'y': -2, 'yaw': 0.5})
        plan = mc.plan_start(root, 'hall', 's1', source='room', now=NOW)
        assert plan.values['AUTONOMY_POSE'] == '1.0000,-2.0000,0.50000'
        assert plan.values['AUTONOMY_POSE_SOURCE'] == 'start_pose'

    def test_extend_from_the_origin(self, mc, root):
        saved_map(root, 'room')
        plan = mc.plan_start(root, 'hall', 's1', source='room', now=NOW)
        assert plan.values['AUTONOMY_POSE'] == '0.0000,0.0000,0.00000'
        assert plan.values['AUTONOMY_POSE_SOURCE'] == 'origin'

    def test_given_pose_wins(self, mc, root):
        saved_map(root, 'room')
        last_pose(root, 'room')
        plan = mc.plan_start(root, 'hall', 's1', source='room', pose='-3,-1,-1.57', now=NOW)
        assert plan.values['AUTONOMY_POSE'] == '-3.0000,-1.0000,-1.57000'
        assert plan.values['AUTONOMY_POSE_SOURCE'] == 'pose'

    def test_bad_pose(self, mc, root):
        saved_map(root, 'room')
        with pytest.raises(mc.MapError) as err:
            mc.plan_start(root, 'hall', 's1', source='room', pose='1,2', now=NOW)
        assert err.value.code == 2

    def test_extend_in_place(self, mc, root):
        saved_map(root, 'room')
        plan = mc.plan_start(root, 'room', 's1', source='room', now=NOW)
        assert not plan.replace and plan.values['AUTONOMY_FROM'] == 'room'

    def test_extend_in_place_refused_over_unsaved_work(self, mc, root):
        saved_map(root, 'room', last_change='2026-10-09 12:00:00.000')
        autosave(root, 'room', last_change='2026-10-09 12:30:00.000')
        with pytest.raises(mc.MapError) as err:
            mc.plan_start(root, 'room', 's1', source='room', now=NOW)
        assert 'recover' in str(err.value)

    def test_extend_an_unsaved_map(self, mc, root):
        autosave(root, 'room', start_pose={'x': 1, 'y': 0, 'yaw': 0})
        plan = mc.plan_start(root, 'hall', 's1', source='room', now=NOW)
        assert any('never saved' in n for n in plan.notes)
        assert plan.values['AUTONOMY_POSE_SOURCE'] == 'start_pose'

    def test_extend_a_map_with_newer_autosave_warns(self, mc, root):
        saved_map(root, 'room', last_change='2026-10-09 12:00:00.000')
        autosave(root, 'room', last_change='2026-10-09 12:30:00.000')
        plan = mc.plan_start(root, 'hall', 's1', source='room', now=NOW)
        assert any('recover room' in n for n in plan.notes)

    def test_extend_a_missing_map(self, mc, root):
        with pytest.raises(mc.MapError) as err:
            mc.plan_start(root, 'hall', 's1', source='room', now=NOW)
        assert err.value.code == 1


class TestLoadPlan:
    def test_last_pose_first(self, mc, root):
        saved_map(root, 'room', start_pose={'x': 9, 'y': 9, 'yaw': 0})
        last_pose(root, 'room')
        plan = mc.plan_load(root, 'room', 's1', now=NOW)
        assert plan.values == {
            'AUTONOMY_MODE': 'localization',
            'AUTONOMY_MAP': 'room',
            'AUTONOMY_SESSION': 's1',
            'AUTONOMY_LOCALIZER': 'amcl',
            'AUTONOMY_POSE': '-1.5000,2.0000,0.30000',
            'AUTONOMY_POSE_SOURCE': 'last_pose',
        }

    def test_a_pose_from_a_lost_run_warns(self, mc, root):
        saved_map(root, 'room')
        last_pose(root, 'room')
        au.mark_pose_lost(root / 'room' / au.LAST_POSE_FILE, NOW)
        plan = mc.plan_load(root, 'room', 's1', now=NOW)
        assert plan.values['AUTONOMY_POSE_SOURCE'] == 'last_pose'
        assert any('--global' in n and 'not localized' in n for n in plan.notes)

    def test_then_the_start_pose(self, mc, root):
        saved_map(root, 'room', start_pose={'x': 1, 'y': 2, 'yaw': 3})
        plan = mc.plan_load(root, 'room', 's1', now=NOW)
        assert plan.values['AUTONOMY_POSE_SOURCE'] == 'start_pose'
        assert plan.values['AUTONOMY_POSE'] == '1.0000,2.0000,3.00000'

    def test_then_the_origin(self, mc, root):
        saved_map(root, 'room')
        plan = mc.plan_load(root, 'room', 's1', now=NOW)
        assert plan.values['AUTONOMY_POSE'] == ''
        assert plan.values['AUTONOMY_POSE_SOURCE'] == 'origin'

    def test_global(self, mc, root):
        saved_map(root, 'room')
        last_pose(root, 'room')
        plan = mc.plan_load(root, 'room', 's1', search=True, now=NOW)
        assert plan.values['AUTONOMY_POSE'] == plan.values['AUTONOMY_POSE_SOURCE'] == 'global'

    def test_slam_toolbox_localizer(self, mc, root):
        saved_map(root, 'room')
        plan = mc.plan_load(root, 'room', 's1', pose='1,2,0', localizer='slam_toolbox', now=NOW)
        assert plan.values['AUTONOMY_LOCALIZER'] == 'slam_toolbox'

    @pytest.mark.parametrize(
        'kwargs',
        [
            {'pose': '1,2,0', 'search': True},
            {'search': True, 'localizer': 'slam_toolbox'},
            {'localizer': 'gmapping'},
        ],
    )
    def test_usage(self, mc, root, kwargs):
        saved_map(root, 'room')
        with pytest.raises(mc.MapError) as err:
            mc.plan_load(root, 'room', 's1', now=NOW, **kwargs)
        assert err.value.code == 2

    def test_unsaved_map_needs_recover(self, mc, root):
        autosave(root, 'room')
        with pytest.raises(mc.MapError) as err:
            mc.plan_load(root, 'room', 's1', now=NOW)
        assert 'recover room' in str(err.value)

    def test_missing_map(self, mc, root):
        with pytest.raises(mc.MapError):
            mc.plan_load(root, 'room', 's1', now=NOW)

    def test_frames_changed(self, mc, root, monkeypatch):
        saved_map(root, 'room', frames={'laser_yaw_deg': 179.619, 'laser_x_mm': 238.3})
        monkeypatch.setattr(
            mc, 'current_frames', lambda: {'laser_yaw_deg': 180.0, 'laser_x_mm': 238.3}
        )
        plan = mc.plan_load(root, 'room', 's1', now=NOW)
        changed = [n for n in plan.notes if 'Frames changed' in n]
        assert len(changed) == 1 and 'laser_yaw_deg' in changed[0]


class FakeSystemd:
    """Stands in for systemctl: records calls, tracks the two units' states."""

    def __init__(self, teleop: str = 'active', autonomy: str = 'inactive') -> None:
        self.states = {'neoracer-teleop.service': teleop, 'neoracer-autonomy.service': autonomy}
        self.calls: list[tuple[str, ...]] = []
        self.on_stop = lambda: None

    def __call__(self, *args: str) -> subprocess.CompletedProcess:
        self.calls.append(args)
        action, unit = args
        if action == 'is-active':
            state = self.states.get(unit, 'inactive')
            return subprocess.CompletedProcess(args, 0, state + '\n', '')
        if action == 'stop':
            self.on_stop()
            self.states[unit] = 'inactive'
        elif action == 'start':
            self.states[unit] = 'active'
        return subprocess.CompletedProcess(args, 0, '', '')

    def actions(self) -> list[str]:
        return [a for a, _ in self.calls if a != 'is-active']


@pytest.fixture
def systemd(mc, monkeypatch):
    fake = FakeSystemd()
    monkeypatch.setattr(mc, 'systemctl', fake)
    monkeypatch.setattr(mc, '_wait_ready', lambda values, out, timeout: 0)
    return fake


class TestChangeMode:
    def test_start(self, mc, root, systemd):
        lines: list[str] = []
        assert mc.start('room', out=lines.append) == 0
        assert systemd.actions() == ['start']
        values = mc.read_mode_file()
        assert values['AUTONOMY_MODE'] == 'mapping' and values['AUTONOMY_MAP'] == 'room'
        assert values['AUTONOMY_SESSION'] and 'racecar map save' in lines[-1]

    def test_mapping_stops_the_driving_apps(self, mc, root, systemd):
        systemd.states['neoracer-webteleop.service'] = 'active'
        systemd.states['neoracer-camlabel.service'] = 'active'
        lines: list[str] = []
        assert mc.start('room', out=lines.append) == 0
        assert ('stop', 'neoracer-webteleop.service') in systemd.calls
        assert systemd.states['neoracer-camlabel.service'] == 'active'
        assert 'Stopped Web Teleop.' in lines

    def test_localizing_stops_the_driving_apps(self, mc, root, systemd):
        saved_map(root, 'room')
        systemd.states['neoracer-linefollow.service'] = 'active'
        assert mc.running_driving_apps() == ['Line Follow']
        assert mc.load('room', out=lambda line: None) == 0
        assert systemd.states['neoracer-linefollow.service'] == 'inactive'
        assert mc.running_driving_apps() == []

    def test_resume_mapping_restarts_the_same_session(self, mc, root, systemd):
        assert mc.start('room', out=lambda line: None) == 0
        session = mc.read_mode_file()['AUTONOMY_SESSION']
        mc.stop(out=lambda line: None)
        lines: list[str] = []
        assert mc.resume(out=lines.append) == 0
        assert mc.read_mode_file()['AUTONOMY_SESSION'] == session
        assert systemd.states['neoracer-autonomy.service'] == 'active'
        assert lines[-1] == 'Mapping room again from its last autosave.'

    def test_resume_localization_loads_the_map_again(self, mc, root, systemd):
        saved_map(root, 'room')
        assert mc.load('room', localizer='slam_toolbox', out=lambda line: None) == 0
        mc.stop(out=lambda line: None)
        assert mc.resume(out=lambda line: None) == 0
        values = mc.read_mode_file()
        assert values['AUTONOMY_MAP'] == 'room' and values['AUTONOMY_LOCALIZER'] == 'slam_toolbox'
        assert values['AUTONOMY_MODE'] == 'localization'

    def test_resume_needs_an_earlier_run(self, mc, root, systemd):
        with pytest.raises(mc.MapError) as err:
            mc.resume()
        assert 'no earlier run' in str(err.value)

    def test_resume_while_running_does_nothing(self, mc, root, systemd):
        systemd.states['neoracer-autonomy.service'] = 'active'
        lines: list[str] = []
        assert mc.resume(out=lines.append) == 0
        assert lines == ['Autonomy is already running.'] and systemd.actions() == []

    def test_refuses_when_the_data_root_cannot_be_written(self, mc, root, systemd, monkeypatch):
        monkeypatch.delenv('RACECAR_MAP_ROOT', raising=False)
        monkeypatch.setattr(mc.data_root, 'session_problem', lambda: 'drive dropped out')
        with pytest.raises(mc.MapError, match='drive dropped out'):
            mc.start('room', out=lambda line: None)
        assert systemd.actions() == []

    def test_teleop_must_run(self, mc, root, systemd):
        systemd.states['neoracer-teleop.service'] = 'inactive'
        with pytest.raises(mc.MapError) as err:
            mc.start('room')
        assert 'teleop' in str(err.value) and systemd.actions() == []
        assert not mc.mode_file().exists()

    def test_usage_errors_stop_nothing(self, mc, root, systemd):
        systemd.states['neoracer-autonomy.service'] = 'active'
        with pytest.raises(mc.MapError):
            mc.load('room')
        assert systemd.actions() == []

    def test_load_stops_the_run_first_and_reads_the_pose_it_left(self, mc, root, systemd):
        saved_map(root, 'room')
        systemd.states['neoracer-autonomy.service'] = 'active'
        # The stop is when autonomy_node writes last_pose.yaml.
        systemd.on_stop = lambda: last_pose(root, 'room')
        assert mc.load('room', out=lambda _: None) == 0
        assert systemd.actions() == ['stop', 'start']
        assert mc.read_mode_file()['AUTONOMY_POSE_SOURCE'] == 'last_pose'

    def test_force_moves_the_old_map_aside(self, mc, root, systemd):
        saved_map(root, 'room')
        lines: list[str] = []
        mc.start('room', force=True, out=lines.append)
        moved = list((root / au.REPLACED_DIR).iterdir())
        assert len(moved) == 1 and au.has_map_files(moved[0], 'room')
        assert not (root / 'room').exists() and any('Moved' in line for line in lines)

    def test_a_failed_systemctl(self, mc, root, monkeypatch):
        def denied(*args):
            if args[0] == 'is-active':
                return subprocess.CompletedProcess(args, 0, 'active\n', '')
            return subprocess.CompletedProcess(args, 1, '', 'Access denied')

        monkeypatch.setattr(mc, 'systemctl', denied)
        with pytest.raises(mc.MapError) as err:
            mc.start('room', out=lambda _: None)
        assert 'Access denied' in str(err.value)
        assert 'sudoers' not in str(err.value)

    def test_no_sudoers_file_says_so(self, mc, root, monkeypatch):
        def no_rule(*args):
            if args[0] == 'is-active':
                state = 'inactive' if args[1] in dict(mc.DRIVING_APPS) else 'active'
                return subprocess.CompletedProcess(args, 0, state + '\n', '')
            return subprocess.CompletedProcess(args, 1, '', 'sudo: a password is required\n')

        monkeypatch.setattr(mc, 'systemctl', no_rule)
        with pytest.raises(mc.MapError) as err:
            mc.start('room', out=lambda _: None)
        message = str(err.value)
        assert 'sudo systemctl stop neoracer-autonomy.service failed' in message
        assert '/etc/sudoers.d/neoracer is not installed' in message

    def test_a_driving_app_that_cannot_be_stopped_names_the_sudoers_file(
        self, mc, root, systemd, monkeypatch
    ):
        systemd.states['neoracer-pursuit.service'] = 'active'

        def no_rule_for_apps(*args):
            if args[0] == 'stop' and args[1] == 'neoracer-pursuit.service':
                return subprocess.CompletedProcess(args, 1, '', 'sudo: a password is required\n')
            return systemd(*args)

        monkeypatch.setattr(mc, 'systemctl', no_rule_for_apps)
        lines: list[str] = []
        assert mc.start('room', out=lines.append) == 0
        stopped = [line for line in lines if line.startswith('Could not stop Pure Pursuit.')]
        assert len(stopped) == 1 and '/etc/sudoers.d/neoracer' in stopped[0]

    def test_stop_reports_unsaved_work(self, mc, root, systemd):
        saved_map(root, 'room', last_change='2026-10-09 12:00:00.000')
        autosave(root, 'room', last_change='2026-10-09 12:30:00.000')
        mc.write_mode_file({'AUTONOMY_MODE': 'mapping', 'AUTONOMY_MAP': 'room'})
        systemd.states['neoracer-autonomy.service'] = 'active'
        lines: list[str] = []
        assert mc.stop(out=lines.append) == 0
        assert systemd.actions() == ['stop'] and 'recover room' in lines[-1]

    def test_stop_saved(self, mc, root, systemd):
        saved_map(root, 'room', last_change='2026-10-09 12:00:00.000')
        autosave(root, 'room', last_change='2026-10-09 12:00:00.000')
        mc.write_mode_file({'AUTONOMY_MODE': 'mapping', 'AUTONOMY_MAP': 'room'})
        systemd.states['neoracer-autonomy.service'] = 'active'
        lines: list[str] = []
        mc.stop(out=lines.append)
        assert lines[-1] == 'Stopped. room is saved.'

    def test_stop_when_off(self, mc, root, systemd):
        lines: list[str] = []
        assert mc.stop(out=lines.append) == 0 and systemd.actions() == []

    def test_save_needs_a_mapping_run(self, mc, root, systemd):
        with pytest.raises(mc.MapError):
            mc.save()
        systemd.states['neoracer-autonomy.service'] = 'active'
        mc.write_mode_file({'AUTONOMY_MODE': 'localization', 'AUTONOMY_MAP': 'room'})
        with pytest.raises(mc.MapError) as err:
            mc.save()
        assert 'not mapping' in str(err.value)

    def test_status_when_off(self, mc, root, systemd):
        mc.write_mode_file({'AUTONOMY_MODE': 'mapping', 'AUTONOMY_MAP': 'room'})
        lines: list[str] = []
        assert mc.status(out=lines.append) == 0
        assert lines == ['autonomy  off (last run: mapping room)']


class TestRecover:
    def test_keeps_the_autosave_and_moves_the_old_map(self, mc, root, systemd):
        saved_map(root, 'room', last_change='2026-10-09 12:00:00.000')
        autosave(root, 'room', last_change='2026-10-09 12:30:00.000')
        assert mc.recover('room', out=lambda _: None) == 0
        assert (root / 'room' / 'room.posegraph').read_text() == 'auto'
        previous = list((root / 'room' / au.PREVIOUS_DIR).iterdir())
        assert len(previous) == 1 and (previous[0] / 'room.posegraph').read_text() == 'm'
        assert not au.map_state(root, 'room').unsaved

    def test_an_unsaved_map(self, mc, root, systemd):
        autosave(root, 'room')
        mc.recover('room', out=lambda _: None)
        state = au.map_state(root, 'room')
        assert state.saved and not state.unsaved
        assert not (root / 'room' / au.PREVIOUS_DIR).exists()

    def test_nothing_newer(self, mc, root, systemd):
        saved_map(root, 'room', last_change='2026-10-09 12:00:00.000')
        autosave(root, 'room', last_change='2026-10-09 12:00:00.000')
        lines: list[str] = []
        mc.recover('room', out=lines.append)
        assert lines[0] == 'room is saved. Nothing to recover.'
        assert (root / 'room' / 'room.posegraph').read_text() == 'm'

    def test_refused_while_mapping_it(self, mc, root, systemd):
        autosave(root, 'room')
        mc.write_mode_file({'AUTONOMY_MODE': 'mapping', 'AUTONOMY_MAP': 'room'})
        systemd.states['neoracer-autonomy.service'] = 'active'
        with pytest.raises(mc.MapError) as err:
            mc.recover('room')
        assert 'racecar map stop' in str(err.value)

    def test_no_autosave(self, mc, root, systemd):
        saved_map(root, 'room')
        with pytest.raises(mc.MapError):
            mc.recover('room')


def pgm(folder: Path, name: str, width: int, height: int, resolution: float) -> None:
    (folder / f'{name}.pgm').write_bytes(
        f'P5\n# CREATOR: map_saver.cpp {resolution:.3f} m/pix\n{width} {height}\n255\n'.encode()
        + bytes(width * height)
    )
    (folder / f'{name}.yaml').write_text(f'image: {name}.pgm\nresolution: {resolution}\n')


class TestListAndInfo:
    def test_grid_size(self, mc, tmp_path):
        files(tmp_path, 'room')
        pgm(tmp_path, 'room', 217, 166, 0.05)
        assert mc.grid_size(tmp_path, 'room') == (217, 166, 0.05)

    def test_list(self, mc, root, systemd):
        pgm(saved_map(root, 'room', last_change='2026-10-09 12:00:00.000'), 'room', 200, 100, 0.05)
        autosave(root, 'room', last_change='2026-10-09 12:30:00.000')
        autosave(root, 'hall')
        saved_map(root, 'lab')
        (root / au.REPLACED_DIR / 'lab-20261009-120000').mkdir(parents=True)
        (root / 'empty').mkdir()
        mc.write_mode_file({'AUTONOMY_MODE': 'localization', 'AUTONOMY_MAP': 'lab'})
        systemd.states['neoracer-autonomy.service'] = 'active'
        lines: list[str] = []
        mc.list_maps(out=lines.append)
        body = {line.split()[0]: line for line in lines[1:-1]}
        assert list(body) == ['hall', 'lab', 'room']
        assert 'unsaved' in body['hall'] and 'loaded now' in body['lab']
        assert 'saved, newer autosave' in body['room'] and '10.0 x 5.0 m' in body['room']

    def test_list_empty(self, mc, root, systemd):
        lines: list[str] = []
        mc.list_maps(out=lines.append)
        assert lines == [f'No maps under {root}.']

    def test_info(self, mc, root, monkeypatch):
        folder = saved_map(
            root,
            'room',
            session='s1',
            start_pose={'x': 0.5, 'y': 0.0, 'yaw': 0.0},
            extended_from='hall',
            frames={'laser_yaw_deg': 179.0},
        )
        pgm(folder, 'room', 100, 60, 0.05)
        last_pose(root, 'room')
        monkeypatch.setattr(mc, 'current_frames', lambda: {'laser_yaw_deg': 180.0})
        lines: list[str] = []
        assert mc.info('room', out=lines.append) == 0
        text = '\n'.join(lines)
        assert '5.0 x 3.0 m (100 x 60 cells of 5 cm)' in text
        assert any(line.startswith('extended from') and line.endswith('hall') for line in lines)
        assert 'start pose' in text and 'last pose' in text
        assert lines[-1].startswith('warning: frames changed') and 'laser_yaw_deg' in lines[-1]

    def test_info_missing(self, mc, root):
        with pytest.raises(mc.MapError):
            mc.info('room')


class TestCli:
    @pytest.mark.parametrize(
        'argv,joined',
        [
            (['load', 'room', '--pose', '-1.5,2,0.3'], ['load', 'room', '--pose=-1.5,2,0.3']),
            (['load', 'room', '--pose=-1,2,0'], ['load', 'room', '--pose=-1,2,0']),
            (['list'], ['list']),
        ],
    )
    def test_negative_pose(self, cli, argv, joined):
        assert cli.join_pose(argv) == joined
        assert cli.parser().parse_args(joined).action == argv[0]

    def test_load_arguments(self, cli):
        args = cli.parser().parse_args(cli.join_pose(['load', 'room', '--pose', '-1,-2,-3']))
        assert (args.name, args.pose, args.search, args.localizer) == (
            'room',
            '-1,-2,-3',
            False,
            'amcl',
        )

    def test_start_arguments(self, cli):
        args = cli.parser().parse_args(
            cli.join_pose(['start', 'hall', '--from', 'room', '--pose', '-1,0,3.14'])
        )
        assert (args.name, args.source, args.pose, args.force) == (
            'hall',
            'room',
            '-1,0,3.14',
            False,
        )

    def test_pose_and_global_exclude(self, cli):
        with pytest.raises(SystemExit) as err:
            cli.parser().parse_args(['load', 'room', '--pose=1,2,0', '--global'])
        assert err.value.code == 2

    def test_errors_exit_with_their_code(self, cli, root, capsys):
        assert cli.main(['info', 'missing']) == 1
        assert cli.main(['start', 'a b']) == 2
        assert 'racecar map:' in capsys.readouterr().err


def test_tool_dispatch_and_completion():
    tool = (Path(__file__).resolve().parent.parent / 'scripts' / 'racecar-tool.sh').read_text()
    assert '"${RACECAR_MAP_SCRIPT:-$pkg_dir/scripts/neoracer_map.py}"' in tool
    assert 'start save stop resume load recover status list info foxglove' in tool
    assert '    map <action>' in tool
    assert 'speed switch UP' in tool
    # Completion lists the maps under the NeoRacer data root, without replaced/.
    assert 'maps_dir="${RACECAR_MAP_ROOT:-${RACECAR_DATA_DIR:-$HOME/neoracer_data}/maps}"' in tool
    assert "sed 's#/$##' | grep -vx replaced)" in tool


def test_the_vendor_slam_commands_are_gone():
    # Plan D1: racecar map replaces racecar mapping and racecar navigation.
    tool = (Path(__file__).resolve().parent.parent / 'scripts' / 'racecar-tool.sh').read_text()
    assert not re.search(r'^\s+(mapping|navigation)\)', tool, re.MULTILINE)
    assert '_rc_autonomy_env' not in tool
    assert 'osracer_slam' not in tool and 'osracer_navigation' not in tool


def test_setup_links_the_maps_into_jupyter():
    text = (Path(__file__).resolve().parent.parent / 'scripts' / 'setup_jupyter.sh').read_text()
    assert 'MAP_ROOT="$DATA_ROOT/maps"' in text
    assert 'ln -sfn "$target" "$JUPYTER_WS/$link"' in text


class TestNeoracerUnits:
    """The unit names and the sudo commands map_control sends (NeoRacer)."""

    def test_units_by_full_name(self, mc):
        units = [mc.UNIT, mc.TELEOP_UNIT, *(unit for unit, _ in mc.DRIVING_APPS)]
        assert all(u.startswith('neoracer-') and u.endswith('.service') for u in units)

    def test_driving_apps_are_the_drive_publishers(self, mc):
        tool = (Path(__file__).resolve().parent.parent / 'scripts' / 'racecar-tool.sh').read_text()
        listed = tool.split('_RC_DASH_UNITS=(', 1)[1].split(')', 1)[0].split()
        driving = [unit.removesuffix('.service') for unit, _ in mc.DRIVING_APPS]
        assert driving == [u for u in listed if u != 'neoracer-camlabel']
        assert [name for _, name in mc.DRIVING_APPS] == [
            'Wall Follow',
            'Pure Pursuit',
            'Policy Search',
            'Smart Follow',
            'Line Follow',
            'Web Teleop',
        ]

    @pytest.mark.parametrize(
        'args,argv',
        [
            (
                ('is-active', 'neoracer-teleop.service'),
                ['systemctl', 'is-active', 'neoracer-teleop.service'],
            ),
            (
                ('start', 'neoracer-autonomy.service'),
                ['sudo', '-n', 'systemctl', 'start', 'neoracer-autonomy.service'],
            ),
            (
                ('stop', 'neoracer-wallfollow.service'),
                ['sudo', '-n', 'systemctl', 'stop', 'neoracer-wallfollow.service'],
            ),
        ],
    )
    def test_start_and_stop_go_through_sudo(self, mc, monkeypatch, args, argv):
        sent: list[list[str]] = []

        def run(cmd, **kwargs):
            sent.append(cmd)
            return subprocess.CompletedProcess(cmd, 0, '', '')

        monkeypatch.delenv('RACECAR_SYSTEMCTL', raising=False)
        monkeypatch.delenv('RACECAR_SUDO', raising=False)
        monkeypatch.setattr(mc.subprocess, 'run', run)
        mc.systemctl(*args)
        assert sent == [argv]


class TestSpeedSwitch:
    """Mapping starts only with the speed switch UP (NeoRacer, plan D12)."""

    def test_up_and_linked(self, mc):
        assert mc.speed_switch_problem(SWITCH_UP, True) == ''

    @pytest.mark.parametrize(
        'channels,link,says',
        [
            (SWITCH_DOWN, True, 'reads 1.00, not UP'),
            (ALL_ZERO, True, 'reads 0.00, not UP'),
            (ALL_ZERO, False, 'not linked'),
            (SWITCH_UP, False, 'not linked'),
            (None, None, 'no /rc/channels'),
            (SWITCH_UP[:7], True, 'index 7) is missing'),
        ],
    )
    def test_refused(self, mc, channels, link, says):
        assert says in mc.speed_switch_problem(channels, link)

    def test_without_rc_link_the_channel_counts(self, mc):
        assert mc.speed_switch_problem(SWITCH_UP, None) == ''
        assert 'not UP' in mc.speed_switch_problem(ALL_ZERO, None)

    @pytest.mark.parametrize(
        'value,ok', [(-1.0, True), (-0.5, True), (-0.49, False), (0.6, False)]
    )
    def test_the_up_band(self, mc, value, ok):
        channels = list(SWITCH_UP)
        channels[mc.SPEED_SWITCH_INDEX] = value
        assert (mc.speed_switch_problem(channels, True) == '') is ok

    def test_the_band_fits_the_shipped_calibration(self, mc):
        root = Path(__file__).resolve().parent.parent
        params = yaml.safe_load((root / 'config' / 'controller.yaml').read_text())[
            'controller_node'
        ]['ros__parameters']

        def reads(count: int) -> float:
            return cl.normalize_channel(
                count,
                params['rc_min'],
                params['rc_center'],
                params['rc_max'],
                params['rc_deadband'],
                params['rc_failsafe_below'],
            )

        # UP 240, DOWN 1807, the reconnect frame 1023, no signal -1 (N01, F08).
        assert reads(240) <= mc.SPEED_SWITCH_UP_AT_MOST
        assert all(reads(c) > mc.SPEED_SWITCH_UP_AT_MOST for c in (1807, 1023, -1))

    def test_start_with_the_switch_down_changes_nothing(self, mc, root, systemd, handset):
        handset.channels = list(SWITCH_DOWN)
        systemd.states['neoracer-autonomy.service'] = 'active'
        systemd.states['neoracer-wallfollow.service'] = 'active'
        with pytest.raises(mc.MapError) as err:
            mc.start('room', out=lambda _: None)
        assert err.value.code == 1 and 'not UP' in str(err.value)
        assert systemd.actions() == [] and not mc.mode_file().exists()

    def test_start_with_the_switch_up(self, mc, root, systemd, handset):
        assert mc.start('room', out=lambda _: None) == 0
        assert handset.reads == 1 and systemd.actions() == ['start']

    def test_extending_a_map_reads_the_switch(self, mc, root, systemd, handset):
        saved_map(root, 'room')
        handset.channels = list(SWITCH_DOWN)
        with pytest.raises(mc.MapError):
            mc.start('hall', source='room', out=lambda _: None)
        assert systemd.actions() == []

    def test_resume_mapping_reads_the_switch(self, mc, root, systemd, handset):
        mc.write_mode_file({'AUTONOMY_MODE': 'mapping', 'AUTONOMY_MAP': 'room'})
        handset.channels = list(SWITCH_DOWN)
        systemd.states['neoracer-wallfollow.service'] = 'active'
        with pytest.raises(mc.MapError, match='not UP'):
            mc.resume(out=lambda _: None)
        assert systemd.actions() == []
        handset.channels = list(SWITCH_UP)
        assert mc.resume(out=lambda _: None) == 0
        assert systemd.actions() == ['stop', 'start']

    def test_resume_localization_does_not_read_the_switch(self, mc, root, systemd, handset):
        saved_map(root, 'room')
        mc.write_mode_file({'AUTONOMY_MODE': 'localization', 'AUTONOMY_MAP': 'room'})
        handset.channels = list(SWITCH_DOWN)
        assert mc.resume(out=lambda _: None) == 0
        assert handset.reads == 0

    def test_load_does_not_read_the_switch(self, mc, root, systemd, handset):
        saved_map(root, 'room')
        handset.channels = list(SWITCH_DOWN)
        assert mc.load('room', out=lambda _: None) == 0
        assert handset.reads == 0

    def test_usage_and_teleop_come_first(self, mc, root, systemd, handset):
        with pytest.raises(mc.MapError) as err:
            mc.start('a b')
        assert err.value.code == 2
        systemd.states['neoracer-teleop.service'] = 'inactive'
        with pytest.raises(mc.MapError, match='teleop'):
            mc.start('room')
        assert handset.reads == 0

    def test_cli_prints_the_reason(self, cli, mc, root, systemd, handset, capsys):
        handset.link = False
        assert cli.main(['start', 'room']) == 1
        assert 'racecar map: the handset is not linked' in capsys.readouterr().err


class FakeRosNode:
    """Records subscriptions; spin_once delivers the frames given to it."""

    def __init__(self) -> None:
        self.callbacks: dict[str, object] = {}
        self.qos: dict[str, object] = {}
        self.destroyed: list[str] = []

    def create_subscription(self, msg_type, topic, callback, qos):
        self.callbacks[topic] = callback
        self.qos[topic] = qos
        return topic

    def destroy_subscription(self, sub):
        self.destroyed.append(sub)


class TestReadRc:
    def test_returns_the_channels_and_the_link(self, mc, handset, monkeypatch):
        import rclpy

        node = FakeRosNode()
        spins: list[float] = []

        def spin_once(n, timeout_sec=None):
            spins.append(timeout_sec)
            n.callbacks['/rc/channels'](SimpleNamespace(data=SWITCH_UP))
            n.callbacks['/rc/link'](SimpleNamespace(data=True))

        monkeypatch.setattr(mc, 'ros_node', lambda: nullcontext(node))
        monkeypatch.setattr(rclpy, 'spin_once', spin_once)
        assert handset.real(timeout=5.0) == (SWITCH_UP, True)
        assert len(spins) == 1 and sorted(node.destroyed) == ['/rc/channels', '/rc/link']
        # The controller publishes both best effort, so a reliable reader gets nothing.
        from rclpy.qos import qos_profile_sensor_data

        assert all(q is qos_profile_sensor_data for q in node.qos.values())

    def test_waits_for_a_frame_with_a_switch_position(self, mc, handset, monkeypatch):
        import rclpy

        node = FakeRosNode()
        # After a teleop restart: no-signal frames, then the reconnect frames
        # (1023 everywhere, linked), then live frames.
        frames = [(ALL_ZERO, False), (ALL_ZERO, True), (SWITCH_UP, True), (SWITCH_DOWN, True)]
        spins: list[float] = []

        def spin_once(n, timeout_sec=None):
            channels, link = frames[min(len(spins), len(frames) - 1)]
            spins.append(timeout_sec)
            n.callbacks['/rc/channels'](SimpleNamespace(data=channels))
            n.callbacks['/rc/link'](SimpleNamespace(data=link))

        monkeypatch.setattr(mc, 'ros_node', lambda: nullcontext(node))
        monkeypatch.setattr(rclpy, 'spin_once', spin_once)
        assert handset.real(timeout=5.0) == (SWITCH_UP, True)
        assert len(spins) == 3

    def test_gives_the_latest_reading_at_the_timeout(self, mc, handset, monkeypatch):
        import rclpy

        node = FakeRosNode()

        def spin_once(n, timeout_sec=None):
            time.sleep(0.01)
            n.callbacks['/rc/channels'](SimpleNamespace(data=ALL_ZERO))
            n.callbacks['/rc/link'](SimpleNamespace(data=False))

        monkeypatch.setattr(mc, 'ros_node', lambda: nullcontext(node))
        monkeypatch.setattr(rclpy, 'spin_once', spin_once)
        assert handset.real(timeout=0.2) == (ALL_ZERO, False)

    def test_nothing_arrives(self, mc, handset, monkeypatch):
        import rclpy

        node = FakeRosNode()
        monkeypatch.setattr(mc, 'ros_node', lambda: nullcontext(node))
        monkeypatch.setattr(rclpy, 'spin_once', lambda n, timeout_sec=None: None)
        assert handset.real(timeout=0.2) == (None, None)
