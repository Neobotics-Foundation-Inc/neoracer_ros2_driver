"""Unit tests for scripts/neoracer_log.py (the `racecar log` implementation)."""

import json
import os
from pathlib import Path
import sys
import time

from conftest import load_script
import pytest

rl = load_script('neoracer_log')

ROOT = Path(__file__).resolve().parent.parent

# `ros2 bag record --help` on the dev car (Humble), 2026-11-12: every flag it
# accepts. Topics are positional; there is no --topics, --all-topics or
# --exclude-regex.
HUMBLE_RECORD_FLAGS = {
    '-h',
    '--help',
    '-a',
    '--all',
    '-e',
    '--regex',
    '-x',
    '--exclude',
    '--include-unpublished-topics',
    '--include-hidden-topics',
    '-o',
    '--output',
    '-s',
    '--storage',
    '-f',
    '--serialization-format',
    '--no-discovery',
    '-p',
    '--polling-interval',
    '-b',
    '--max-bag-size',
    '-d',
    '--max-bag-duration',
    '--max-cache-size',
    '--compression-mode',
    '--compression-format',
    '--compression-queue-size',
    '--compression-threads',
    '--snapshot-mode',
    '--ignore-leaf-topics',
    '--qos-profile-overrides-path',
    '--storage-preset-profile',
    '--storage-config-file',
    '--start-paused',
    '--use-sim-time',
    '--log-level',
}

# metadata.yaml as Humble's rosbag2 writes it, cut down from the dev car's
# 0.6.1 baseline bag (2026-11-11): the real duration, total and five topics.
CAR_METADATA = """rosbag2_bagfile_information:
  version: 5
  storage_identifier: sqlite3
  duration:
    nanoseconds: 64119755515
  starting_time:
    nanoseconds_since_epoch: 1794422327593933248
  message_count: 54613
  topics_with_message_count:
    - topic_metadata:
        name: /odom
        type: nav_msgs/msg/Odometry
        serialization_format: cdr
        offered_qos_profiles: ""
      message_count: 12220
    - topic_metadata:
        name: /camera/color
        type: sensor_msgs/msg/Image
        serialization_format: cdr
        offered_qos_profiles: ""
      message_count: 3660
    - topic_metadata:
        name: /scan
        type: sensor_msgs/msg/LaserScan
        serialization_format: cdr
        offered_qos_profiles: ""
      message_count: 1904
    - topic_metadata:
        name: /battery
        type: sensor_msgs/msg/BatteryState
        serialization_format: cdr
        offered_qos_profiles: ""
      message_count: 31
    - topic_metadata:
        name: /parameter_events
        type: rcl_interfaces/msg/ParameterEvent
        serialization_format: cdr
        offered_qos_profiles: ""
      message_count: 0
  compression_format: ""
  compression_mode: ""
"""


class TestNaming:
    def test_timestamp_then_test_name(self):
        now = time.mktime((2026, 9, 6, 14, 30, 0, 0, 0, -1))
        assert rl.build_bag_name('wallfollow', now) == '20260906_143000_wallfollow'

    def test_spaces_and_slashes_are_flattened(self):
        assert rl.sanitize_name('lap 3 / turn 2') == 'lap-3-turn-2'

    def test_empty_name_falls_back(self):
        assert rl.sanitize_name('') == 'run'
        assert rl.sanitize_name('///') == 'run'

    def test_safe_characters_survive(self):
        assert rl.sanitize_name('run_2.1-a') == 'run_2.1-a'


class TestRecordArgv:
    """The flags must match `ros2 bag record` on Humble, which the car runs."""

    def test_all_topics_uses_humbles_flag(self):
        argv = rl.build_record_argv('/data/bag')
        assert argv[-1] == '--all'
        assert '--all-topics' not in argv

    def test_exclude_uses_humbles_flag(self):
        argv = rl.build_record_argv('/data/bag', exclude=['/camera/.*'])
        assert argv[argv.index('--exclude') + 1] == '/camera/.*'
        assert '--exclude-regex' not in argv

    def test_several_excludes_join_into_one_regex(self):
        # Humble keeps only the last --exclude, so the patterns become one.
        argv = rl.build_record_argv('/data/bag', exclude=['/camera/.*', '/rosout'])
        assert argv.count('--exclude') == 1
        assert argv[argv.index('--exclude') + 1] == '(/camera/.*)|(/rosout)'

    def test_explicit_topics_replace_all(self):
        argv = rl.build_record_argv('/data/bag', topics=['/scan', '/odom'])
        assert '--all' not in argv
        assert argv[-2:] == ['/scan', '/odom']

    def test_output_and_storage(self):
        argv = rl.build_record_argv('/data/bag')
        assert argv[:3] == ['ros2', 'bag', 'record']
        assert '--output' in argv and '/data/bag' in argv
        assert argv[argv.index('--storage') + 1] == 'sqlite3'

    @pytest.mark.parametrize(
        'topics,exclude',
        [(None, None), (['/scan', '/odom'], None), (None, ['/camera/.*', '/rosout'])],
    )
    def test_every_flag_is_one_humble_has(self, topics, exclude):
        argv = rl.build_record_argv('/data/bag', topics=topics, exclude=exclude)
        flags = [a for a in argv[3:] if a.startswith('-')]
        assert flags and set(flags) <= HUMBLE_RECORD_FLAGS


class TestResolveLogRoot:
    def test_explicit_wins(self, tmp_path):
        assert rl.resolve_log_root(str(tmp_path)) == tmp_path

    def test_env_var_is_next(self, tmp_path):
        got = rl.resolve_log_root(None, environ={'RACECAR_LOG_DIR': str(tmp_path)})
        assert got == tmp_path

    def test_bags_under_the_data_root(self, data_root_dir):
        got = rl.resolve_log_root(None, environ={})
        assert got == data_root_dir / 'bags' and got.is_dir()


class TestBandwidth:
    def test_all_topics_estimate_is_the_cars_camera(self):
        assert rl.estimate_mbps([]) == pytest.approx(4.0)

    def test_named_topics_without_the_camera_are_cheap(self):
        assert rl.estimate_mbps(['/scan', '/odom']) == 0.0

    def test_the_whole_car_fits_an_sd_card(self, monkeypatch):
        monkeypatch.setattr(rl, 'root_is_sd', lambda root: True)
        assert rl.bandwidth_warning([], '/home/racecar', free_bytes=40_000_000_000) is None

    def test_warns_when_the_rate_outruns_the_disk(self, monkeypatch):
        monkeypatch.setattr(rl, 'root_is_sd', lambda root: True)
        warning = rl.bandwidth_warning(
            [], '/home/racecar', sustained=2.0, free_bytes=40_000_000_000
        )
        assert warning is not None
        assert 'dropped' in warning

    def test_silent_on_nvme(self, monkeypatch):
        monkeypatch.setattr(rl, 'root_is_sd', lambda root: False)
        assert rl.bandwidth_warning([], '/data', free_bytes=400_000_000_000) is None

    def test_silent_for_a_cheap_selection(self, monkeypatch):
        monkeypatch.setattr(rl, 'root_is_sd', lambda root: True)
        assert rl.bandwidth_warning(['/scan'], '/home/racecar') is None


class TestFormatting:
    @pytest.mark.parametrize(
        'n,expected',
        [
            (512, '512 B'),
            (2048, '2.0 KB'),
            (5 * 1024**2, '5.0 MB'),
            (3 * 1024**3, '3.0 GB'),
        ],
    )
    def test_sizes(self, n, expected):
        assert rl.format_size(n) == expected

    @pytest.mark.parametrize(
        'sec,expected',
        [
            (0, '0:00'),
            (65, '1:05'),
            (3661, '1:01:01'),
            (-5, '0:00'),
        ],
    )
    def test_durations(self, sec, expected):
        assert rl.format_duration(sec) == expected


class TestConfig:
    def test_roundtrip(self, tmp_path):
        path = tmp_path / 'log.conf'
        rl.save_config({'TOPICS': '/scan,/odom', 'DIR': '/data'}, path)
        assert rl.load_config(path) == {'TOPICS': '/scan,/odom', 'DIR': '/data'}

    def test_empty_values_are_dropped(self, tmp_path):
        path = tmp_path / 'log.conf'
        rl.save_config({'TOPICS': '', 'DIR': '/data'}, path)
        assert rl.load_config(path) == {'DIR': '/data'}

    def test_missing_file_is_empty(self, tmp_path):
        assert rl.load_config(tmp_path / 'absent.conf') == {}

    def test_comments_ignored(self, tmp_path):
        path = tmp_path / 'log.conf'
        path.write_text('# a comment\nDIR=/data\n\n')
        assert rl.load_config(path) == {'DIR': '/data'}


class TestState:
    def test_roundtrip(self, tmp_path):
        path = tmp_path / 'log.state'
        rl.write_state({'pid': 1, 'bag': '/data/b'}, path)
        assert rl.read_state(path)['bag'] == '/data/b'

    def test_missing_state_is_none(self, tmp_path):
        assert rl.read_state(tmp_path / 'absent') is None

    def test_corrupt_state_is_none(self, tmp_path):
        path = tmp_path / 'log.state'
        path.write_text('not json')
        assert rl.read_state(path) is None

    def test_clear_is_idempotent(self, tmp_path):
        path = tmp_path / 'log.state'
        rl.write_state({'pid': 1}, path)
        rl.clear_state(path)
        rl.clear_state(path)
        assert rl.read_state(path) is None

    def test_pid_alive_on_self(self):
        assert rl.pid_alive(os.getpid()) is True

    def test_pid_alive_rejects_garbage(self):
        assert rl.pid_alive(None) is False
        assert rl.pid_alive('nope') is False


class TestSummarizeTopics:
    def test_sorted_by_count_descending(self):
        rows = rl.summarize_topics({'/a': 10, '/b': 300, '/c': 50}, 10.0)
        assert [r['topic'] for r in rows] == ['/b', '/c', '/a']

    def test_mean_rate(self):
        rows = rl.summarize_topics({'/scan': 100}, 10.0)
        assert rows[0]['hz'] == pytest.approx(10.0)

    def test_zero_duration_does_not_divide(self):
        rows = rl.summarize_topics({'/scan': 5}, 0.0)
        assert rows[0]['hz'] == 0.0

    def test_empty_bag(self):
        assert rl.summarize_topics({}, 10.0) == []


class TestDirSize:
    def test_sums_nested_files(self, tmp_path):
        (tmp_path / 'a').write_bytes(b'x' * 100)
        nested = tmp_path / 'sub'
        nested.mkdir()
        (nested / 'b').write_bytes(b'y' * 50)
        assert rl.dir_size(tmp_path) == 150

    def test_empty_dir(self, tmp_path):
        assert rl.dir_size(tmp_path) == 0


class TestCli:
    def test_parser_exposes_every_action(self):
        parser = rl.build_parser()
        actions = parser._subparsers._group_actions[0].choices.keys()
        assert set(actions) == {'start', 'stop', 'status', 'list', 'analyze', 'config'}

    def test_start_defaults_to_all_topics(self):
        args = rl.build_parser().parse_args(['start', 'lap1'])
        assert args.test_name == 'lap1'
        assert args.topics is None

    def test_storage_is_sqlite3_only(self):
        assert rl.build_parser().parse_args(['start']).storage is None
        with pytest.raises(SystemExit):
            rl.build_parser().parse_args(['start', '--storage', 'mcap'])

    def test_stop_has_a_finalize_timeout(self):
        args = rl.build_parser().parse_args(['stop'])
        assert args.timeout == 15.0

    def test_status_reports_not_recording(self, tmp_path, monkeypatch, capsys):
        monkeypatch.setattr(rl, 'STATE_FILE', tmp_path / 'absent')
        assert rl.main(['status']) == 0
        assert 'not recording' in capsys.readouterr().out

    def test_status_json_when_idle(self, tmp_path, monkeypatch, capsys):
        monkeypatch.setattr(rl, 'STATE_FILE', tmp_path / 'absent')
        rl.main(['status', '--json'])
        assert json.loads(capsys.readouterr().out) == {'recording': False}

    def test_stale_state_is_cleared(self, tmp_path, monkeypatch):
        # A recorder killed by a reboot leaves state behind; treating that as
        # "recording" would make stop and status lie.
        path = tmp_path / 'log.state'
        rl.write_state({'pid': 999999, 'bag': str(tmp_path / 'b'), 'started': 0}, path)
        monkeypatch.setattr(rl, 'STATE_FILE', path)
        assert rl.active_state() is None
        assert rl.read_state(path) is None


class TestAnalyze:
    def _bag(self, tmp_path, metadata=CAR_METADATA):
        bag = tmp_path / '20261111_101000_baseline'
        bag.mkdir()
        (bag / 'baseline_0.db3').write_bytes(b'x' * 2048)
        if metadata is not None:
            (bag / 'metadata.yaml').write_text(metadata)
        return bag

    def test_reads_humbles_metadata_without_ros(self, tmp_path):
        summary = rl.read_bag_summary(self._bag(tmp_path))
        assert summary['duration_sec'] == pytest.approx(64.119755515)
        assert summary['messages'] == 54613
        assert summary['counts']['/scan'] == 1904

    def test_prints_counts_and_rates(self, tmp_path, capsys):
        bag = self._bag(tmp_path)
        assert rl.main(['analyze', str(bag)]) == 0
        out = capsys.readouterr().out
        assert '/odom' in out and '190.6' in out
        assert 'recorded no messages: /parameter_events' in out

    def test_no_metadata_and_no_rosbag2_is_unreadable(self, tmp_path, monkeypatch, capsys):
        monkeypatch.setitem(sys.modules, 'rosbag2_py', None)
        bag = self._bag(tmp_path, metadata=None)
        assert rl.read_bag_summary(bag) is None
        assert rl.main(['analyze', str(bag)]) == 1
        assert 'ros2 bag reindex' in capsys.readouterr().err


def test_the_tool_runs_neoracer_log():
    tool = (ROOT / 'scripts' / 'racecar-tool.sh').read_text()
    assert '$pkg_dir/scripts/neoracer_log.py}' in tool
    assert 'racecar_log.py' not in tool
    start = tool.index('_racecar_complete()')
    complete = tool[start:]
    top = next(line for line in complete.splitlines() if 'compgen -W "build test' in line)
    assert ' log ' in top
    assert 'start stop status list analyze config help' in complete
