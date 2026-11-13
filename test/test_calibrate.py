"""
Unit tests for scripts/calibration/calibrate.py (`racecar calibrate`).

The NeoRacer runs four steps (lidar, odom, steering, dynamics). Their tools
arrive one by one, so tests that run a tool install stand-in tool files with
the `tools` fixture.
"""

import json
from pathlib import Path
import subprocess
import sys

from conftest import load_script
import pytest
import yaml

# calibrate_common.py imports rclpy, so these run on the car (off car the
# module is skipped; a stand-in ROS plugin runs them there).
pytest.importorskip('rclpy')


@pytest.fixture(scope='module')
def cal():
    return load_script('calibrate')


@pytest.fixture
def config(tmp_path):
    d = tmp_path / 'config'
    d.mkdir()
    return d


@pytest.fixture
def params(car_profile_dir):
    d = car_profile_dir / 'params'
    d.mkdir(parents=True)
    return d


@pytest.fixture
def tools(cal, tmp_path, monkeypatch):
    """Point calibrate.py at a folder with a stand-in file for every step's tool."""
    here = tmp_path / 'calibration'
    here.mkdir()
    for step in cal.STEPS:
        (here / step.tool).write_text('')
    monkeypatch.setattr(cal, 'HERE', here)
    return here


def _ros_yaml(path: Path, node: str, params: dict) -> None:
    path.write_text(yaml.safe_dump({node: {'ros__parameters': params}}))


class TestSteps:
    def test_installed_follows_the_tool_file(self, cal, tools):
        # The lidar tool imports calibrate_odom.py, so the lidar step needs it too.
        (tools / 'calibrate_odom.py').unlink()
        assert [s.key for s in cal.STEPS if not s.installed] == ['lidar', 'odom']

    def test_the_lidar_step_names_the_tools_it_still_needs(self, cal, tools):
        (tools / 'calibrate_odom.py').unlink()
        (tools / 'calibrate_dynamics.py').unlink()
        lidar = cal.STEPS[0]
        assert lidar.missing == ['calibrate_odom.py', 'calibrate_dynamics.py']
        assert lidar.missing_text == (
            'calibrate_odom.py and calibrate_dynamics.py are not in this driver yet'
        )
        (tools / 'calibrate_lidar.py').unlink()
        assert lidar.missing_text == 'calibrate_lidar.py is not in this driver yet'

    def test_four_steps_lidar_first(self, cal):
        assert cal.STEP_KEYS == ['lidar', 'odom', 'steering', 'dynamics']
        assert not {'imu', 'mag', 'realsense'} & set(cal.STEP_KEYS)


class TestReadValues:
    def test_ros_parameter_file(self, cal, config):
        f = config / 'odom.local.yaml'
        _ros_yaml(f, 'controller_node', {'odometry.speed_scale': 1.02})
        assert cal.read_values(f) == {'odometry.speed_scale': 1.02}

    def test_plain_file(self, cal, config):
        f = config / 'frames.local.yaml'
        f.write_text('# header\nlaser_yaw_deg: 179.2\n')
        assert cal.read_values(f) == {'laser_yaw_deg': 179.2}

    def test_missing_or_corrupt(self, cal, config):
        assert cal.read_values(config / 'absent.yaml') == {}
        (config / 'bad.yaml').write_text('a: [')
        assert cal.read_values(config / 'bad.yaml') == {}


class TestStatus:
    def test_rows(self, cal, config, params, tools):
        _ros_yaml(params / 'odom.yaml', 'controller_node', {'odometry.speed_scale': 1.02})
        (params / 'frames.yaml').write_text('laser_yaw_deg: 179.2\n')
        rows = {key: (state, detail) for key, state, detail in cal.status_rows(config)}
        assert rows['odom'][0] == 'calibrated'
        assert rows['lidar'][0] == 'calibrated'
        assert rows['steering'][0] == 'default'
        assert rows['dynamics'][0] == 'default'

    def test_a_step_without_its_tool_reads_not_installed(self, cal, config, params, tools):
        (tools / 'calibrate_steering.py').unlink()
        rows = {key: (state, detail) for key, state, detail in cal.status_rows(config)}
        assert rows['steering'] == (
            'not installed',
            'calibrate_steering.py is not in this driver yet',
        )
        assert cal.status_verdict(cal.STEPS[2], 'not installed') == 'WARN'
        # Stored values still read as calibrated without the tool.
        (params / 'steering.yaml').write_text('trim_deg: 0.8\n')
        rows = {key: state for key, state, _ in cal.status_rows(config)}
        assert rows['steering'] == 'calibrated'

    def test_driver_folder_files_do_not_count(self, cal, config, tools):
        (config / 'frames.local.yaml').write_text('laser_yaw_deg: 179.2\n')
        rows = {key: state for key, state, _ in cal.status_rows(config)}
        assert rows['lidar'] == 'default'

    def test_status_names_the_profile(self, cal, config, car_profile_dir):
        assert cal.status_lines(config)[0] == f'Car profile: {car_profile_dir}'

    def test_status_lists_stored_values_under_each_step(self, cal, config, params, tools):
        (params / 'frames.yaml').write_text('laser_yaw_deg: 179.16\n')
        (params / 'dynamics.yaml').write_text('servo_lock_to_lock_s: 0.12\n')
        lines = cal.status_lines(config)
        i = lines.index('lidar: Lidar yaw')
        assert lines[i + 1].startswith('  calibrated')
        assert lines[i + 2] == '  laser_yaw_deg  179.16'
        j = next(n for n, line in enumerate(lines) if line.startswith('dynamics:'))
        assert lines[j + 2] == '  servo_lock_to_lock_s  0.12'
        k = next(n for n, line in enumerate(lines) if line.startswith('odom:'))
        assert lines[k + 1].startswith('  default') and lines[k + 2] == ''

    def test_status_colors(self, cal, config, params, tools):
        (params / 'frames.yaml').write_text('laser_yaw_deg: 179.16\n')
        text = '\n'.join(cal.status_lines(config, color=True))
        green, yellow, red = '\033[1;32mcalibrated', '\033[1;33mdefault', '\033[1;31mdefault'
        assert green in text  # lidar
        assert red in text  # a checked step not measured on this car
        assert yellow in text  # dynamics not measured
        assert '\033' not in '\n'.join(cal.status_lines(config))

    def test_dynamics_has_no_check(self, cal):
        (step,) = [s for s in cal.STEPS if s.key == 'dynamics']
        assert step.check == '' and step.key == cal.STEP_KEYS[-1]


class TestRun:
    @pytest.fixture
    def harness(self, cal, params, tools, monkeypatch):
        """Stub the tools and prompts; returns (answers list, commands run)."""
        ran = []
        answers = []

        def fake_run(cmd, check=False, **_kw):
            ran.append(cmd)
            if str(cmd[-1]).endswith('calibrate_lidar.py'):
                (params / 'frames.yaml').write_text('laser_yaw_deg: 179.5\n')
            return type('R', (), {'returncode': 0, 'stdout': 'active\n'})()

        monkeypatch.setattr(cal.subprocess, 'run', fake_run)
        monkeypatch.setattr(cal, 'ask', lambda _p='': answers.pop(0))
        monkeypatch.setenv('RACECAR_SUDO', '')
        return answers, ran

    def test_run_reports(self, cal, harness, capsys):
        answers, ran = harness
        answers += ['', 'n']  # run lidar; no restart
        lidar = [s for s in cal.STEPS if s.key == 'lidar']
        assert cal.run_steps(lidar) == 0
        out = capsys.readouterr().out
        assert 'laser_yaw_deg: default -> 179.5' in out
        assert 'Apply later: systemctl restart neoracer-teleop' in out
        assert len(ran) == 1

    def test_skip_and_quit(self, cal, harness):
        answers, ran = harness
        answers += ['s', 'q']
        steps = list(cal.STEPS)
        assert cal.run_steps(steps) == 0
        assert ran == []

    def test_restart_on_yes(self, cal, harness):
        answers, ran = harness
        answers += ['', 'y']
        lidar = [s for s in cal.STEPS if s.key == 'lidar']
        cal.run_steps(lidar)
        assert ran[-1] == ['systemctl', 'restart', 'neoracer-teleop']

    def test_tools_run_in_session_mode(self, cal, tools, monkeypatch):
        envs = []

        def fake_run(cmd, check=False, env=None, **_kw):
            envs.append(env)
            return type('R', (), {'returncode': 0})()

        monkeypatch.setattr(cal.subprocess, 'run', fake_run)
        answers = ['', 'n']
        monkeypatch.setattr(cal, 'ask', lambda _p='': answers.pop(0))
        cal.run_steps([s for s in cal.STEPS if s.key == 'odom'])
        assert envs[0][cal.SESSION_ENV] == '1'


class TestBackupReminder:
    def test_standalone_tool_says_how_to_apply(self, monkeypatch):
        common = load_script('calibrate_common')
        monkeypatch.delenv(common.SESSION_ENV, raising=False)
        text = common.backup_reminder([Path('/x/params/odom.yaml')])
        assert 'Wrote /x/params/odom.yaml' in text
        assert 'Back it up' not in text and 'Apply it' in text

    def test_session_leaves_the_restart_to_calibrate(self, monkeypatch):
        common = load_script('calibrate_common')
        monkeypatch.setenv(common.SESSION_ENV, '1')
        text = common.backup_reminder([Path('/x/params/odom.yaml')])
        assert text == 'Wrote /x/params/odom.yaml'


class TestSetupText:
    def test_one_instruction_per_line_under_the_label(self, cal):
        assert cal.setup_text('Do one.\nDo two.') == 'Setup: Do one.\n       Do two.'


class TestMain:
    def test_unknown_step_is_a_usage_error(self, cal):
        assert cal.main(['gps']) == 2

    def test_dynamics_runs_only_when_named(self, cal):
        default = [s.key for s in cal.select_steps([])]
        assert 'dynamics' not in default and default == cal.STEP_KEYS[:-1]
        assert [s.key for s in cal.select_steps(['dynamics', 'lidar'])] == ['lidar', 'dynamics']

    def test_refuses_without_teleop(self, cal, monkeypatch):
        monkeypatch.setattr(cal, 'teleop_active', lambda: False)
        assert cal.main(['lidar']) == 1

    def test_status_needs_no_ros_graph(self, cal, capsys):
        assert cal.main(['status']) == 0
        out = capsys.readouterr().out
        for key in cal.STEP_KEYS:
            assert key in out


class TestChecks:
    def test_every_check_step_has_a_check_flag_in_its_tool(self, cal):
        # Only the tools already in this driver; the others arrive with F36 to F39.
        for step in cal.STEPS:
            if step.check and step.installed:
                result = subprocess.run(
                    [sys.executable, str(cal.HERE / step.tool), '--help'],
                    capture_output=True,
                    text=True,
                    timeout=30,
                )
                assert '--check' in result.stdout, step.tool

    def test_run_checks_prints_every_criterion(self, cal, tools, monkeypatch, capsys):
        reports = {
            'calibrate_lidar.py': (
                'PASS',
                [('yaw', '0.4 deg', 'within 1 deg', 'PASS')],
            ),
            'calibrate_odom.py': (
                'FAIL',
                [
                    ('speed scale', '6.1% off', 'within 5%', 'FAIL', 'recalibrate'),
                    ('yaw scale', '3.2% off', 'within 2%', 'WARN', 'redo the circles?'),
                ],
            ),
        }
        ran = []

        def fake_run(cmd, check=False, **_kw):
            ran.append(cmd)
            verdict, rows = reports[Path(cmd[1]).name]
            crit = [
                dict(zip(('name', 'measured', 'limit', 'status', 'advice'), r, strict=False))
                for r in rows
            ]
            Path(cmd[cmd.index('--report') + 1]).write_text(
                json.dumps({'overall': verdict, 'criteria': crit})
            )
            return type('R', (), {'returncode': 1 if verdict == 'FAIL' else 0})()

        monkeypatch.setattr(cal.subprocess, 'run', fake_run)
        answers = ['', '', 's']
        monkeypatch.setattr(cal, 'ask', lambda _p='': answers.pop(0))
        steps = [s for s in cal.STEPS if s.key in ('lidar', 'odom', 'steering')]
        assert cal.run_checks(steps) == 1
        out = capsys.readouterr().out
        start = out.index('=== Check summary ===')
        summary = out[start:]
        assert 'lidar      PASS' in summary and 'odom       FAIL' in summary
        assert 'steering   skipped' in summary
        assert 'FAIL  speed scale' in summary and '-> recalibrate' in summary
        assert 'WARN  yaw scale' in summary
        assert 'RESULT FAIL  1 pass  0 warn  1 fail  1 skipped' in summary
        assert all('--check' in cmd for cmd in ran)

    def test_tool_without_a_report_counts_as_a_fail(self, cal, tools, monkeypatch, capsys):
        monkeypatch.setattr(
            cal.subprocess, 'run', lambda *a, **k: type('R', (), {'returncode': 1})()
        )
        monkeypatch.setattr(cal, 'ask', lambda _p='': '')
        steps = [s for s in cal.STEPS if s.key == 'lidar']
        assert cal.run_checks(steps) == 1
        assert 'lidar      FAIL' in capsys.readouterr().out

    def test_check_routes_only_checkable_steps(self, cal, monkeypatch):
        seen = []
        monkeypatch.setattr(cal, 'teleop_active', lambda: True)
        monkeypatch.setattr(
            cal, 'run_checks', lambda steps: seen.extend(s.key for s in steps) or 0
        )
        assert cal.main(['check']) == 0
        assert seen == ['lidar', 'odom', 'steering']

    def test_check_of_an_unknown_step_is_a_usage_error(self, cal):
        assert cal.main(['check', 'gps']) == 2


class TestAsk:
    def _stdin(self, cal, monkeypatch, data: bytes):
        buf = bytearray(data)

        def read(fd, n):
            assert fd == 0
            out = bytes(buf[:n])
            del buf[:n]
            return out

        monkeypatch.setattr(cal.os, 'read', read)
        return buf

    def test_reads_exactly_one_line(self, cal, monkeypatch):
        rest = self._stdin(cal, monkeypatch, b'S\nfor the tool\n')
        assert cal.ask('? ') == 's'
        assert bytes(rest) == b'for the tool\n'

    def test_end_of_input_stops(self, cal, monkeypatch):
        self._stdin(cal, monkeypatch, b'')
        assert cal.ask('? ') == 'q'


class TestNotRun:
    def test_stopping_lists_the_rest(self, cal, monkeypatch, capsys):
        monkeypatch.setattr(cal, 'ask', lambda _p='': 'q')
        steps = [s for s in cal.STEPS if s.key in ('lidar', 'odom')]
        assert cal.run_checks(steps) == 0
        out = capsys.readouterr().out
        assert 'lidar      not run' in out and 'odom       not run' in out
        assert '2 not run' in out

    def test_read_line_raises_at_end_of_input(self, cal, monkeypatch):
        common = load_script('calibrate_common')
        monkeypatch.setattr(common.os, 'read', lambda fd, n: b'')
        with pytest.raises(EOFError):
            common.read_line()


class TestColor:
    def _results(self, cal):
        common = load_script('calibrate_common')
        step = next(s for s in cal.STEPS if s.key == 'odom')
        crit = [
            common.Criterion('speed scale', '1.2% off', 'within 5%', 'PASS'),
            common.Criterion('yaw scale', '3.2% off', 'within 2%', 'WARN', 'redo?'),
        ]
        return [(step, 'WARN', crit)]

    def test_plain_without_color(self, cal):
        text = '\n'.join(cal.summary_lines(self._results(cal), color=False))
        assert '\033[' not in text
        assert 'RESULT WARN  0 pass  1 warn  0 fail  0 skipped' in text

    def test_colors_match_racecar_status(self, cal):
        text = '\n'.join(cal.summary_lines(self._results(cal), color=True))
        assert '\033[32mPASS ' in text  # green criterion
        assert '\033[33mWARN ' in text  # yellow criterion
        assert '\033[1;33mWARN' in text  # bold verdict and result
        assert '\033[33m1 warn\033[0m' in text
        assert '0 fail' in text and '\033[31m0 fail' not in text  # zero counts stay plain

    def test_no_color_opt_out(self, cal, monkeypatch):
        common = load_script('calibrate_common')
        monkeypatch.setattr(common.sys.stdout, 'isatty', lambda: True)
        monkeypatch.setenv('TERM', 'xterm')
        monkeypatch.delenv('NO_COLOR', raising=False)
        assert common.use_color() is True
        monkeypatch.setenv('NO_COLOR', '1')
        assert common.use_color() is False

    def test_not_a_terminal(self, cal, monkeypatch):
        common = load_script('calibrate_common')
        monkeypatch.setattr(common.sys.stdout, 'isatty', lambda: False)
        assert common.use_color() is False


class TestTypeAhead:
    def test_terminal_input_is_flushed_before_the_prompt(self, monkeypatch):
        common = load_script('calibrate_common')
        flushed = []
        monkeypatch.setattr(common.os, 'isatty', lambda fd: True)
        monkeypatch.setattr(common.termios, 'tcflush', lambda fd, q: flushed.append(fd))
        monkeypatch.setattr(common.os, 'read', lambda fd, n: b'\n')
        common.read_line()
        assert flushed == [0]

    def test_piped_input_is_kept(self, monkeypatch):
        common = load_script('calibrate_common')
        monkeypatch.setattr(common.os, 'isatty', lambda fd: False)
        monkeypatch.setattr(common.termios, 'tcflush', lambda fd, q: pytest.fail('flushed'))
        monkeypatch.setattr(common.os, 'read', lambda fd, n: b'\n')
        assert common.read_line() == ''


class TestRunSequence:
    def _run(self, monkeypatch, answers, results, allow_skip=True):
        common = load_script('calibrate_common')
        queue = list(answers)
        monkeypatch.setattr(common, 'read_line', lambda _p='': queue.pop(0))
        ran = []
        outcomes = list(results)

        def run(i):
            ran.append(i)
            return outcomes.pop(0)

        got = common.run_sequence(3, lambda i: None, run, allow_skip=allow_skip)
        return got, ran

    def test_back_redoes_the_previous_step(self, monkeypatch):
        got, ran = self._run(monkeypatch, ['', '', 'b', '', '', ''], ['a', 'b1', 'b2', 'c'])
        assert ran == [0, 1, 1, 2]
        assert got == {0: 'a', 1: 'b2', 2: 'c'}

    def test_back_twice_discards_both(self, monkeypatch):
        got, ran = self._run(
            monkeypatch, ['', '', 'b', 'b', '', '', ''], ['a1', 'b1', 'a2', 'b2', 'c']
        )
        assert ran == [0, 1, 0, 1, 2]
        assert got == {0: 'a2', 1: 'b2', 2: 'c'}

    def test_failed_step_is_offered_again(self, monkeypatch):
        got, ran = self._run(monkeypatch, ['', '', '', ''], ['a', None, 'b', 'c'])
        assert ran == [0, 1, 1, 2]
        assert got == {0: 'a', 1: 'b', 2: 'c'}

    def test_skip_and_back_at_the_first_step(self, monkeypatch):
        got, ran = self._run(monkeypatch, ['b', 's', '', ''], ['b', 'c'])
        assert ran == [1, 2]
        assert got == {1: 'b', 2: 'c'}

    def test_skip_refused_when_every_step_is_needed(self, monkeypatch):
        got, ran = self._run(monkeypatch, ['s', '', '', ''], ['a', 'b', 'c'], allow_skip=False)
        assert ran == [0, 1, 2]

    def test_back_is_offered_from_the_second_step(self):
        common = load_script('calibrate_common')
        assert 'b to redo' not in common.step_prompt(0)
        assert 'b to redo the previous step' in common.step_prompt(1)


class TestSessionBack:
    def test_back_reruns_the_previous_step_with_its_first_values(
        self, cal, params, tools, monkeypatch
    ):
        answers = ['', 'b', '', '', 'n']  # odom, back, odom again, steering, no restart
        monkeypatch.setattr(cal, 'ask', lambda _p='': answers.pop(0))
        ran = []

        def fake_run(cmd, check=False, **_kw):
            ran.append(Path(cmd[-1]).name)
            values = {'odometry.speed_scale': 1.0 + len(ran) / 100}
            _ros_yaml(params / 'odom.yaml', 'controller_node', values)
            return type('R', (), {'returncode': 0})()

        monkeypatch.setattr(cal.subprocess, 'run', fake_run)
        monkeypatch.setenv('RACECAR_SUDO', '')
        steps = [s for s in cal.STEPS if s.key in ('odom', 'steering')]
        assert cal.run_steps(steps) == 0
        assert ran == ['calibrate_odom.py', 'calibrate_odom.py', 'calibrate_steering.py']

    def test_prompt_offers_back_after_the_first_step(self, cal, monkeypatch):
        prompts = []

        def ask(p=''):
            prompts.append(p)
            return 's'

        monkeypatch.setattr(cal, 'ask', ask)
        cal.run_checks([s for s in cal.STEPS if s.key in ('lidar', 'odom')])
        assert prompts == [cal.RUN_PROMPT, cal.RUN_PROMPT_BACK]


class TestNeoRacer:
    """What the NeoRacer changes: the deadman, the firmware trim, missing tools."""

    def test_every_self_driving_step_says_how_to_stop(self, cal):
        for key in ('lidar', 'steering', 'dynamics'):
            step = next(s for s in cal.STEPS if s.key == key)
            for text in (step.position, step.check):
                if text:
                    assert 'Flip the mode switch UP to stop.' in text, key
                    assert 'mode switch DOWN (autonomy)' in text, key
        source = (cal.HERE / 'calibrate.py').read_text()
        assert 'Hold RB' not in source

    def test_steering_result_is_the_firmware_trim_with_a_copy(self, cal, config, params, tools):
        step = next(s for s in cal.STEPS if s.key == 'steering')
        assert (step.local_name, step.keys) == ('steering', ('trim_deg',))
        assert 'ESP32 firmware' in step.label and 'trim set' in step.note
        (params / 'steering.yaml').write_text('trim_deg: 0.8\n')
        lines = cal.status_lines(config)
        i = lines.index('steering: Steering trim, set in the ESP32 firmware')
        assert lines[i + 2] == f'  {step.note}'
        assert lines[i + 3] == '  trim_deg  0.8'

    def test_teleop_is_the_neoracer_unit(self, cal):
        assert cal.TELEOP_UNIT == 'neoracer-teleop'

    def test_a_missing_tool_is_reported_not_failed(self, cal, tmp_path, monkeypatch, capsys):
        monkeypatch.setattr(cal, 'HERE', tmp_path)  # no tools here
        ran = []
        monkeypatch.setattr(cal.subprocess, 'run', lambda cmd, **_kw: ran.append(cmd))
        answers = ['', '']
        monkeypatch.setattr(cal, 'ask', lambda _p='': answers.pop(0))
        assert cal.run_checks([s for s in cal.STEPS if s.key in ('lidar', 'odom')]) == 0
        out = capsys.readouterr().out
        assert 'calibrate_lidar.py is not in this driver yet; nothing to check.' in out
        assert '2 not installed' in out and ran == []
        answers += ['', 'n']
        assert cal.run_steps([s for s in cal.STEPS if s.key == 'steering']) == 0
        assert 'steering   not installed' in capsys.readouterr().out
