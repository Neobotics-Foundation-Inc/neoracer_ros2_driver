"""Unit tests for scripts/validation/slam_session.py (no ROS graph, no processes)."""

from datetime import datetime
import math
from pathlib import Path
import re

from conftest import load_script
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope='module')
def ss():
    return load_script('slam_session')


class TestCommands:
    def test_teleop_turns_inference_off(self, ss):
        argv = ss.teleop_argv(camera=True)
        assert argv[-1] == 'inference_enable:=false' and argv[1].endswith('launch_teleop.sh')

    def test_inference_on_leaves_its_default(self, ss):
        argv = ss.teleop_argv(camera=True, inference=True)
        assert not any(a.startswith('inference_enable') for a in argv)

    def test_teleop_without_camera(self, ss):
        assert ss.teleop_argv(camera=False)[-1] == 'camera_enable:=false'

    def test_autonomy_mapping(self, ss):
        argv = ss.autonomy_argv('mapping', 'room', 'amcl', '', 's1')
        assert argv[-3:] == ['mode:=mapping', 'map:=room', 'session:=s1']

    def test_autonomy_localization(self, ss):
        argv = ss.autonomy_argv('localization', 'room', 'slam_toolbox', '1,2,0')
        assert 'map:=room' in argv and argv[-2:] == ['localizer:=slam_toolbox', 'pose:=1,2,0']

    def test_bag_records_the_slam_topics(self, ss, tmp_path):
        argv = ss.bag_argv(tmp_path / 'bag')
        assert argv[:3] == ['ros2', 'bag', 'record'] and '/tf_static' in argv and '/map' in argv
        assert not any(t.startswith('/camera') for t in argv)

    def test_foxglove_whitelist_has_no_camera(self, ss):
        params = yaml.safe_load(ss.FOXGLOVE_PARAMS.read_text())['foxglove_bridge']
        topics = params['ros__parameters']['topic_whitelist']
        assert '^/map$' in topics and not any('camera' in t for t in topics)

    def test_session_name(self, ss):
        now = datetime(2026, 10, 8, 14, 5, 9)
        assert ss.session_dir_name('map', 'room', now) == '20261008-140509-map-room'


class TestDisk:
    GB = 1_000_000_000

    def test_ok(self, ss):
        assert ss.disk_status(58 * self.GB, 23 * self.GB, 33 * self.GB)[0] == 'ok'

    def test_warns_when_mostly_used(self, ss):
        level, text = ss.disk_status(58 * self.GB, 47 * self.GB, 11 * self.GB)
        assert level == 'warn' and '81% used' in text

    def test_warns_below_ten_gb(self, ss):
        assert ss.disk_status(200 * self.GB, 100 * self.GB, 9 * self.GB)[0] == 'warn'

    def test_refuses_below_three_gb(self, ss):
        assert ss.disk_status(58 * self.GB, 56 * self.GB, 2 * self.GB)[0] == 'refuse'

    def test_largest_entries(self, ss, tmp_path):
        (tmp_path / 'big').mkdir()
        (tmp_path / 'big' / 'a').write_bytes(b'x' * 300)
        (tmp_path / 'small.log').write_bytes(b'x' * 10)
        assert [p.name for _, p in ss.largest_entries(tmp_path)] == ['big', 'small.log']


class TestAnalysis:
    def test_max_gap_only_counts_the_save_window(self, ss):
        times = [0.0, 0.1, 0.2, 2.0, 2.1, 5.0, 5.1]
        assert ss.max_gap(times, 0.15, 2.05) == pytest.approx(1.8)
        assert ss.max_gap([1.0], 0, 2) == 0.0

    def test_drift(self, ss):
        map_odom = [(0.0, 0, 0, 0), (1.0, 0.01, 0, 0), (2.0, 0.11, 0, math.radians(2))]
        odom = [(0.0, 0, 0), (1.0, 3, 0), (2.0, 3, 4)]
        d = ss.drift_summary(map_odom, odom)
        assert d.path_m == pytest.approx(7.0)
        assert d.largest_jump_m == pytest.approx(0.1) and d.largest_jump_t == pytest.approx(2.0)
        assert d.largest_jump_deg == pytest.approx(2.0)
        assert d.final[0] == pytest.approx(0.11)

    def test_no_drift_without_transforms(self, ss):
        assert ss.drift_summary([], [(0, 0, 0)]) is None

    def test_loop_lines(self, ss):
        assert ss.loop_lines('a\n[slam] Loop closure found\nb') == ['[slam] Loop closure found']

    def test_load_csv_round_trip(self, ss, tmp_path):
        ev = load_script('slam_eval')
        rec = object.__new__(ev.LoadRecorder)
        rec.samples = [ev.Sample(1.0, 40.0, 60.5, 0, 2400, {'slam_toolbox': 5.0})]
        rec.write_csv(tmp_path / 'load.csv')
        (s,) = ss.read_load_csv(tmp_path / 'load.csv')
        assert (s.t, s.cpu_busy, s.temp_c, s.throttled, s.arm_mhz) == (1.0, 40.0, 60.5, 0, 2400)
        assert s.per_process == {'slam_toolbox': 5.0}

    def test_report_for_a_mapping_session(self, ss, tmp_path, monkeypatch):
        # Pillow is on the car (python3-pil); the off-car venv may lack it.
        Image = pytest.importorskip('PIL.Image')

        maps = tmp_path / 'maps' / 'room'
        maps.mkdir(parents=True)
        Image.new('L', (4, 2), 254).save(maps / 'room.pgm')
        (maps / 'room.yaml').write_text('image: room.pgm\nresolution: 0.05\n')
        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path / 'maps'))
        session = tmp_path / 'session'
        (session / 'logs').mkdir(parents=True)
        (session / 'logs' / 'autonomy.log').write_text('x\nLoop closure ok\n')
        (session / 'session.yaml').write_text(
            yaml.safe_dump({'mode': 'mapping', 'map': 'room', 'camera': True, 'git': {}})
        )
        (session / 'stalls.csv').write_text('n,call_s\n1,0.80\n')
        report = ss.analyze(session).read_text()
        assert '## Load' in report and 'No load record.' in report
        assert 'free 8 (0.0 m^2)' in report and (session / 'map.png').exists()
        assert '1 log lines mention a loop.' in report
        assert '1,0.80' in report


class TestReplay:
    def test_overrides_are_yaml_values(self, ss):
        got = ss.parse_overrides(['angle_variance_penalty=0.03', 'do_loop_closing=false'])
        assert got == {'angle_variance_penalty': 0.03, 'do_loop_closing': False}

    @pytest.mark.parametrize('item', ['novalue', '=1'])
    def test_bad_override(self, ss, item):
        with pytest.raises(ValueError):
            ss.parse_overrides([item])

    def test_play_drops_nothing_but_remaps_tf(self, ss, tmp_path):
        play = ss.play_argv(tmp_path / 'bag')
        assert play[3] == str(tmp_path / 'bag') and play[-2:] == ['--remap', '/tf:=/tf_bag']
        assert '/map' not in play  # the replayed nodes publish their own

    def test_mapping_nodes(self, ss, tmp_path):
        names = [n for n, _ in ss.replay_specs('mapping', tmp_path / 'o.yaml')]
        assert names == ['slam_toolbox']  # Humble: a plain node, no lifecycle manager
        argv = dict(ss.replay_specs('mapping', tmp_path / 'o.yaml'))['slam_toolbox']
        assert 'async_slam_toolbox_node' in argv and argv[-1] == str(tmp_path / 'o.yaml')

    def test_amcl_nodes(self, ss, tmp_path):
        specs = dict(ss.replay_specs('amcl', tmp_path / 'o.yaml'))
        assert list(specs) == ['map_server', 'amcl', 'lifecycle_manager']
        assert '__node:=lifecycle_manager_localization' in specs['lifecycle_manager']

    def test_amcl_overrides_start_at_the_pose(self, ss, tmp_path):
        o = ss.replay_overrides('amcl', {'alpha1': 0.05}, tmp_path / 'room', (1.0, 2.0, 0.5))
        p = o['amcl']['ros__parameters']
        assert p['initial_pose'] == {'x': 1.0, 'y': 2.0, 'z': 0.0, 'yaw': 0.5}
        assert p['alpha1'] == 0.05 and p['use_sim_time'] is True
        assert o['map_server']['ros__parameters']['yaml_filename'] == f'{tmp_path / "room"}.yaml'

    def test_slam_localization_overrides(self, ss, tmp_path):
        o = ss.replay_overrides('slam_toolbox', {}, tmp_path / 'room', (0.0, 0.0, 0.0))
        p = o['slam_toolbox']['ros__parameters']
        assert p['map_file_name'] == str(tmp_path / 'room') and p['map_start_pose'] == [0, 0, 0]

    def test_drift_spread(self, ss):
        d = ss.drift_summary(
            [(0.0, 0, 0, 0), (1.0, 0.3, 0.4, math.radians(8)), (2.0, 0, 0, 0)], [(0, 0, 0)]
        )
        assert d.max_offset_m == pytest.approx(0.5) and d.max_yaw_deg == pytest.approx(8)
        assert d.yaw_sd_deg > 3


def test_status_lines(ss, tmp_path):
    f = tmp_path / 'status.csv'
    f.write_text(
        't_s,mode,localized,pose_sd_m,last_autosave\n'
        '0.0,localization,false,,\n'
        '2.0,localization,true,0.1,\n'
        '6.0,localization,false,0.5,\n'
        '8.0,localization,true,0.1,2026-10-09 10:00:00\n'
        '10.0,off,false,,\n'
    )
    (line,) = ss.status_lines(f)
    assert line.startswith('Localized 50% of 8 s; first at 2 s; dropped 1 time(s).')
    assert 'Autosaves seen: 1' in line


def test_first_moved(ss):
    odom = [(0.0, 1.0, 1.0), (1.0, 1.01, 1.0), (2.0, 1.06, 1.0), (3.0, 2.0, 1.0)]
    assert ss.first_moved(odom) == 2.0
    assert ss.first_moved(odom[:2]) is None and ss.first_moved([]) is None


def test_autonomy_replay_section(ss, tmp_path):
    name, argv, section = ss.autonomy_replay('amcl', 'room', 'amcl', '1,2,0', tmp_path / 'o.yaml')
    assert name == 'autonomy_node' and argv[-1] == str(tmp_path / 'o.yaml')
    assert section == {
        'mode': 'localization',
        'map': 'room',
        'localizer': 'amcl',
        'pose': '1,2,0',
        'use_sim_time': True,
    }


class TestNeoRacer:
    """The session drives F24's autonomy launch and the NeoRacer teleop stack."""

    def test_teleop_arguments_are_teleop_subsystems(self, ss):
        launch = (ROOT / 'launch' / 'teleop.launch.py').read_text()
        block = re.search(r'_SUBSYSTEMS = \{(.*?)\}', launch, re.S).group(1)
        subsystems = set(re.findall(r"'(\w+)': '(?:true|false)'", block))
        for arg in ss.teleop_argv(camera=False)[2:]:
            name, _, value = arg.partition('_enable:=')
            assert name in subsystems and value == 'false', arg

    def test_teleop_unit_is_the_neoracer_unit(self, ss):
        assert ss.TELEOP_UNIT == 'neoracer-teleop'
        assert (ROOT / 'scripts' / f'{ss.TELEOP_UNIT}.service').is_file()

    def test_autonomy_arguments_are_declared_by_the_launch(self, ss):
        launch = (ROOT / 'launch' / 'autonomy.launch.py').read_text()
        declared = set(re.findall(r"DeclareLaunchArgument\(\s*'(\w+)'", launch))
        package = re.search(r'<name>(.+)</name>', (ROOT / 'package.xml').read_text()).group(1)
        for argv in (
            ss.autonomy_argv('mapping', 'room', 'amcl', '', 's1'),
            ss.autonomy_argv('localization', 'room', 'slam_toolbox', '1,2,0', 's1'),
        ):
            assert argv[2:4] == [package, 'autonomy.launch.py']
            keys = {a.split(':=')[0] for a in argv[4:]}
            assert keys <= declared, keys - declared

    @pytest.mark.parametrize('kind', ['mapping', 'amcl', 'slam_toolbox'])
    def test_replay_nodes_are_the_launch_nodes(self, ss, tmp_path, kind):
        """Each replay node runs the launch's executable under its name, with its YAML section."""
        launch = (ROOT / 'launch' / 'autonomy.launch.py').read_text()
        executables = set(re.findall(r"'executable': '(\w+)'", launch))
        names = set(re.findall(r"'name': '(\w+)'", launch))
        names |= set(re.findall(r"_lifecycle_manager\('(\w+)'", launch))
        for _, argv in ss.replay_specs(kind, tmp_path / 'o.yaml'):
            assert argv[3] in executables
            node = next(a for a in argv if a.startswith('__node:=')).removeprefix('__node:=')
            assert node in names
            files = [Path(a) for a in argv if a.endswith('.yaml')]
            assert files[-1] == tmp_path / 'o.yaml'
            assert any(node in (yaml.safe_load(f.read_text()) or {}) for f in files[:-1]), node

    def test_the_lifecycle_manager_runs_the_other_amcl_nodes(self, ss, tmp_path):
        amcl = yaml.safe_load((ROOT / 'config' / 'amcl.yaml').read_text())
        managed = amcl['lifecycle_manager_localization']['ros__parameters']['node_names']
        names = [n for n, _ in ss.replay_specs('amcl', tmp_path / 'o.yaml')]
        assert sorted(managed) == sorted(n for n in names if n != 'lifecycle_manager')

    def test_load_csv_keeps_the_jetson_readings(self, ss, tmp_path):
        ev = load_script('slam_eval')
        rec = object.__new__(ev.LoadRecorder)
        jetson = {'gpu_pct': 23.0, 'power_mw': 6695, 'mem_used_mib': 2893, 'mem_avail_mib': 4727}
        sample = ev.Sample(1.0, 40.0, 51.5, 1, 1344, {'amcl': 5.0}, **jetson, swap_used_mib=3)
        rec.samples = [sample]
        rec.write_csv(tmp_path / 'load.csv')
        assert ss.read_load_csv(tmp_path / 'load.csv') == rec.samples

    def test_report_gives_the_power_mode_and_the_discard(self, ss, tmp_path, monkeypatch):
        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path / 'maps'))
        session = tmp_path / 'session'
        session.mkdir()
        (session / 'session.yaml').write_text(
            yaml.safe_dump({'mode': 'localization', 'map': 'room', 'power_mode': '25W', 'git': {}})
        )
        ev = load_script('slam_eval')
        rec = object.__new__(ev.LoadRecorder)
        rec.samples = [
            ev.Sample(0.0, 40.0, 51.5, 0, 1344, {}, swap_used_mib=0),
            ev.Sample(1.0, 40.0, 51.5, 0, 1344, {}, swap_used_mib=3),
        ]
        rec.write_csv(session / 'load.csv')
        report = ss.analyze(session).read_text()
        assert 'Jetson power mode 25W.' in report
        assert 'Swap in use: 0 MiB at the start, up to 3 MiB. Discard this run.' in report

    def test_a_session_needs_the_mcap_plugin(self, ss, tmp_path, monkeypatch, capsys):
        import argparse

        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path / 'maps'))
        monkeypatch.setattr(ss, 'disk_line', lambda path=None: ('ok', '50 GB free'))
        monkeypatch.setattr(ss.neoracer_log, 'storage_installed', lambda storage: False)
        args = argparse.Namespace(cmd='map', name='room', camera='on', inference='off')
        assert ss.Session(args).preflight() is False
        assert 'ros-humble-rosbag2-storage-mcap' in capsys.readouterr().out
        assert ss.bag_argv(tmp_path / 'bag')[5:7] == ['--storage', 'mcap']

    def test_localize_starts_at_the_maps_start_pose(self, ss, tmp_path, monkeypatch):
        import argparse

        monkeypatch.setenv('RACECAR_MAP_ROOT', str(tmp_path / 'maps'))
        monkeypatch.setattr(ss, 'disk_line', lambda path=None: ('ok', '50 GB free'))
        monkeypatch.setattr(ss.neoracer_log, 'storage_installed', lambda storage: True)
        monkeypatch.setattr(ss.Session, 'teleop_service_active', staticmethod(lambda: True))
        room = tmp_path / 'maps' / 'room'
        room.mkdir(parents=True)
        (room / 'room.yaml').write_text('image: room.pgm\n')
        (room / 'meta.yaml').write_text('start_pose: {x: -0.12, y: 0.03, yaw: 1.5}\n')
        args = argparse.Namespace(
            cmd='localize', map='room', localizer='amcl', pose='', camera='on', inference='off'
        )
        session = ss.Session(args)
        assert session.preflight() is True
        assert session.pose == '-0.12,0.03,1.5'
        assert 'pose:=-0.12,0.03,1.5' in ss.autonomy_argv(
            'localization', 'room', 'amcl', session.pose
        )
        given = argparse.Namespace(**{**vars(args), 'pose': '1,2,0'})
        session = ss.Session(given)
        assert session.preflight() is True and session.pose == '1,2,0'
        (room / 'meta.yaml').unlink()
        assert ss.start_mark_pose(room / 'room') == ''

    def test_slam_replays_add_no_lifecycle_manager_section(self, ss, tmp_path):
        for kind in ('mapping', 'slam_toolbox'):
            assert list(ss.replay_overrides(kind, {}, tmp_path / 'room', (0, 0, 0))) == [
                'slam_toolbox'
            ]
