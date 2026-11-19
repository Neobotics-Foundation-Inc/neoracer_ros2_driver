"""
Tests for scripts/racecar-tool.sh, the `racecar` shell function.

Ported from upstream's test_racecar_tool.py at v0.9.0, without the eth, wifi,
desktop, disk and RealSense tests: the NeoRacer has none of those commands.
Tests for the NeoRacer's own commands (update, ws, clear, launch, compile) are
added. Every test that would reach the car runs against stub sudo, systemctl,
git, colcon and ros2 commands and a temporary HOME, so a run on a car never
starts, stops or updates anything.
"""

import os
from pathlib import Path
import re
import shlex
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parent.parent
TOOL = ROOT / 'scripts' / 'racecar-tool.sh'
PKG = 'neoracer_ros2_driver'
TEXT = TOOL.read_text()

COMMANDS = (
    'build',
    'test',
    'lint',
    'profile',
    'data',
    'log',
    'calibrate',
    'source',
    'ws',
    'cd',
    'teleop',
    'map',
    'update',
    'launch',
    'compile',
    'clear',
    'udev',
    'watchdog',
    'service',
    'setup',
    'library',
    'cleanup',
    'selftest',
    'status',
)
CORE = ('teleop', 'watchdog', 'dashboard', 'jupyter', 'mapview')
DRIVE = ('wallfollow', 'pursuit', 'eps', 'smartfollow', 'linefollow', 'webteleop')
LABS = ('camlabel', *DRIVE)
EVERY_UNIT = ' '.join(f'neoracer-{u}' for u in (*CORE, 'autonomy', *LABS))

SYSTEMCTL = r"""#!/bin/bash
# Stub: the unit lists come from RC_INSTALLED, RC_ACTIVE, RC_ACTIVATING and
# RC_ENABLED; every other call is logged.
quiet=0
rest=()
for a in "$@"; do
    case "$a" in
        --quiet|-q) quiet=1 ;;
        --no-legend) ;;
        *) rest+=("$a") ;;
    esac
done
unit="${rest[1]%.service}"
has() { [[ " $1 " == *" $2 "* ]]; }
case "${rest[0]}" in
    list-unit-files)
        if has "$RC_INSTALLED" "$unit"; then echo "$unit.service disabled enabled"; fi ;;
    is-active)
        if has "$RC_ACTIVE" "$unit"; then
            [[ $quiet == 1 ]] || echo active; exit 0
        fi
        if has "${RC_ACTIVATING:-}" "$unit"; then
            [[ $quiet == 1 ]] || echo activating; exit 3
        fi
        [[ $quiet == 1 ]] || echo inactive; exit 3 ;;
    is-enabled)
        if has "$RC_ENABLED" "$unit"; then
            [[ $quiet == 1 ]] || echo enabled; exit 0
        fi
        [[ $quiet == 1 ]] || echo disabled; exit 1 ;;
    *) echo "systemctl $*" >> "$RC_LOG" ;;
esac
"""
LOGGER = '#!/bin/bash\necho "{name} $*" >> "$RC_LOG"\n'


class Car:
    """A temporary HOME whose ~/ros2_ws/src/neoracer_ros2_driver is this tree."""

    def __init__(self, tmp_path, package=None):
        self.home = tmp_path / 'home'
        src = self.home / 'ros2_ws' / 'src'
        src.mkdir(parents=True)
        (self.home / 'ros2_ws' / 'install').mkdir()
        (self.home / 'ros2_ws' / 'install' / 'setup.bash').write_text('')
        if package is None:
            (src / PKG).symlink_to(ROOT)
        else:
            (src / PKG).symlink_to(package)
        self.pkg_dir = src / PKG
        self.bin = tmp_path / 'bin'
        self.bin.mkdir()
        (self.bin / 'systemctl').write_text(SYSTEMCTL)
        for name in ('sudo', 'git', 'colcon', 'ros2', 'journalctl'):
            (self.bin / name).write_text(LOGGER.format(name=name))
        (self.bin / 'hostname').write_text('#!/bin/bash\necho neoracer\n')
        for stub in self.bin.iterdir():
            stub.chmod(0o755)
        (self.bin / 'python3').symlink_to(sys.executable)
        self.units = tmp_path / 'units'
        self.units.mkdir()
        self.log = tmp_path / 'calls.log'
        self.log.write_text('')

    def run(self, *args, **env_extra):
        env = dict(os.environ)
        env.update(
            {
                'HOME': str(self.home),
                'PATH': f'{self.bin}:{env["PATH"]}',
                'PYTHONUSERBASE': str(self.home / '.local'),
                'RC_LOG': str(self.log),
                'RC_INSTALLED': EVERY_UNIT,
                'RC_ACTIVE': '',
                'RC_ENABLED': '',
                'RACECAR_SYSTEMD_DIR': str(self.units),
            }
        )
        env.update(env_extra)
        script = f'set +u; source "{TOOL}"; racecar {shlex.join(args)}'
        return subprocess.run(
            ['bash', '-c', script], capture_output=True, text=True, timeout=30, env=env
        )

    def calls(self):
        return self.log.read_text().splitlines()


@pytest.fixture
def car(tmp_path):
    return Car(tmp_path)


def _unit_ports():
    """Return {name: port} from each unit file's Description, as setup installs them."""
    ports = {}
    files = [*(ROOT / 'scripts').glob('neoracer-*.service')]
    files += (ROOT / 'scripts' / 'dashboards').glob('*/neoracer-*.service.in')
    for path in files:
        found = re.search(r'^Description=.*\(port (\d+)\)', path.read_text(), re.M)
        if found:
            name = path.name.split('.')[0].removeprefix('neoracer-')
            ports[name] = int(found.group(1))
    return ports


def _bash(script):
    return subprocess.run(['bash', '-c', script], capture_output=True, text=True, timeout=15)


def test_bash_syntax_clean():
    result = _bash(f'bash -n "{TOOL}"')
    assert result.returncode == 0, f'bash -n failed:\n{result.stderr}'


def test_sourcing_defines_racecar_function():
    result = _bash(f'source "{TOOL}" && type -t racecar')
    assert result.returncode == 0
    assert result.stdout.strip() == 'function'


@pytest.mark.parametrize('args', [[], ['help'], ['--help'], ['-h']])
def test_help_renders(car, args):
    result = car.run(*args)
    assert result.returncode == 0
    assert 'Commands:' in result.stdout
    for sub in COMMANDS:
        assert re.search(rf'^    {sub}\b', result.stdout, re.M), f'help missing "{sub}"'


def test_every_command_is_dispatched_helped_and_completed():
    start, end = TEXT.index('racecar() {'), TEXT.index('_racecar_complete() {')
    body = TEXT[start:end]
    dispatched = set()
    for label in re.findall(r'^        ([a-z][a-z|_-]*)\)\s*$', body, re.M):
        dispatched |= set(label.split('|'))
    dispatched -= {'help'}
    assert dispatched == set(COMMANDS)
    top = re.search(r'COMP_CWORD -eq 1 \]\]; then\n\s+COMPREPLY=\( \$\(compgen -W "([^"]+)"', TEXT)
    assert top and set(top.group(1).split()) == {*COMMANDS, 'help'}


def test_unknown_command_errors(car):
    result = car.run('bogus_subcommand')
    assert result.returncode == 2
    assert 'unknown command' in result.stderr


def test_launch_without_name_errors(car):
    result = car.run('launch')
    assert result.returncode == 2
    assert 'usage:' in result.stderr


def test_launch_runs_the_named_launch_file(car):
    assert car.run('launch', 'lidar').returncode == 0
    assert car.calls() == [f'ros2 launch {PKG} lidar.launch.py']


def _gnu_find():
    probe = subprocess.run(
        ['find', '.', '-maxdepth', '0', '-xtype', 'l'], capture_output=True, text=True
    )
    return probe.returncode == 0


@pytest.mark.skipif(not _gnu_find(), reason='needs GNU find (-xtype), as on the car')
class TestBuild:
    """`racecar build` clears dangling symlinks left by deleted sources."""

    def _build(self, tmp_path):
        car = Car(tmp_path)
        ws = car.home / 'ros2_ws'
        build = ws / 'build' / PKG / 'scripts'
        install = ws / 'install' / PKG / 'lib' / PKG
        other = ws / 'build' / 'lakibeam1'
        for d in (build, install, other):
            d.mkdir(parents=True)
        live = tmp_path / 'live.py'
        live.write_text('')
        links = {
            'stale_build': build / 'v2v_node.py',
            'stale_install': install / 'v2v_node.py',
            'live': build / 'live.py',
            'other_pkg': other / 'gone.py',
        }
        for name, link in links.items():
            link.symlink_to(live if name == 'live' else tmp_path / 'deleted.py')
        seen = tmp_path / 'seen'
        (car.bin / 'colcon').write_text(
            f'#!/bin/bash\nfind "{ws}/build/{PKG}" "{ws}/install/{PKG}" -xtype l > "{seen}"\n'
        )
        return car.run('build'), links, seen

    def test_dangling_links_removed_before_colcon(self, tmp_path):
        result, links, seen = self._build(tmp_path)
        assert result.returncode == 0, result.stderr
        assert not links['stale_build'].is_symlink()
        assert not links['stale_install'].is_symlink()
        assert seen.read_text() == '', 'colcon ran with dangling links still present'

    def test_each_removal_is_printed(self, tmp_path):
        result, links, _ = self._build(tmp_path)
        assert f'removed dangling symlink: {links["stale_build"]}' in result.stdout
        assert f'removed dangling symlink: {links["stale_install"]}' in result.stdout

    def test_live_links_and_other_packages_are_kept(self, tmp_path):
        _, links, _ = self._build(tmp_path)
        assert links['live'].is_symlink()
        assert links['other_pkg'].is_symlink()


def test_cd_changes_pwd_to_package_root(car):
    # `cd` runs in the caller's shell, not a subshell.
    env = dict(os.environ, HOME=str(car.home))
    script = f'set +u; source "{TOOL}"; racecar cd && pwd'
    result = subprocess.run(
        ['bash', '-c', script], capture_output=True, text=True, timeout=5, env=env
    )
    assert result.returncode == 0
    assert result.stdout.strip() == str(car.pkg_dir)


def test_status_forwards_flags(car):
    result = car.run('status', '--section', 'nonsense')
    assert result.returncode == 2
    assert 'unknown section' in result.stderr


class TestService:
    def test_default_action_is_status(self, car):
        result = car.run('service')
        assert result.returncode == 0
        assert 'neoracer-teleop' in result.stdout

    def test_status_lists_core_autonomy_and_dashboards(self, car):
        out = car.run('service', 'status').stdout
        for unit in (*CORE, 'autonomy', *LABS):
            assert f'neoracer-{unit}' in out, unit
        assert out.index('neoracer-autonomy') < out.index('lab dashboards')

    def test_status_gives_the_address_of_a_page(self, car):
        (car.units / 'neoracer-mapview.service').write_text(
            '[Unit]\nDescription=Neoracer Web Map Viewer (port 8088)\n'
        )
        out = car.run('service', 'status').stdout
        line = next(ln for ln in out.splitlines() if 'neoracer-mapview' in ln)
        assert line.rstrip().endswith('http://neoracer.local:8088')

    def test_status_gives_every_page_its_address(self, car):
        for path in (ROOT / 'scripts').glob('neoracer-*.service'):
            (car.units / path.name).write_text(path.read_text())
        for path in (ROOT / 'scripts' / 'dashboards').glob('*/neoracer-*.service.in'):
            (car.units / path.name.removesuffix('.in')).write_text(path.read_text())
        lines = car.run('service', 'status').stdout.splitlines()
        ports = _unit_ports()
        assert {'dashboard', 'jupyter', 'mapview', *LABS} == set(ports)
        for name in (*CORE, 'autonomy', *LABS):
            line = next(ln for ln in lines if re.match(rf'\s+neoracer-{name}\s', ln))
            if name in ports:
                assert line.rstrip().endswith(f'http://neoracer.local:{ports[name]}'), line
            else:
                assert 'http://' not in line, line

    def test_help_ports_match_the_unit_files(self, car):
        service_help = car.run('service', 'help').stdout
        top_help = ' '.join(car.run('help').stdout.split())
        for name, port in _unit_ports().items():
            assert f'{name} ({port})' in service_help, name
            assert f'{name} ({port})' in top_help, name

    def test_status_names_a_unit_that_is_not_installed(self, car):
        out = car.run('service', 'status', RC_INSTALLED='neoracer-teleop').stdout
        assert re.search(r'neoracer-autonomy\s+not installed', out)

    def test_help_lists_actions_ports_and_the_one_at_a_time_rule(self, car):
        out = car.run('service', 'help').stdout
        for action in ('install', 'start', 'stop', 'restart', 'enable', 'logs', 'status'):
            assert action in out
        for name, port in (('dashboard', 8080), ('jupyter', 8888), ('mapview', 8088)):
            assert f'{name} ({port})' in out
        assert 'only one of them runs at a time' in out
        assert 'racecar map' in out

    def test_starting_a_drive_dashboard_stops_the_others(self, car):
        result = car.run(
            'service', 'start', 'linefollow', RC_ACTIVE='neoracer-wallfollow neoracer-camlabel'
        )
        assert result.returncode == 0, result.stderr
        assert 'only one /drive publisher' in result.stdout
        assert car.calls() == [
            'sudo systemctl stop neoracer-wallfollow.service',
            'sudo systemctl start neoracer-linefollow.service',
        ]

    def test_restarting_a_drive_dashboard_stops_the_others(self, car):
        result = car.run('service', 'restart', 'eps', RC_ACTIVE='neoracer-pursuit')
        assert result.returncode == 0, result.stderr
        assert car.calls() == [
            'sudo systemctl stop neoracer-pursuit.service',
            'sudo systemctl restart neoracer-eps.service',
        ]

    def test_starting_the_running_dashboard_does_not_stop_it(self, car):
        car.run('service', 'start', 'linefollow', RC_ACTIVE='neoracer-linefollow neoracer-eps')
        assert car.calls() == [
            'sudo systemctl stop neoracer-eps.service',
            'sudo systemctl start neoracer-linefollow.service',
        ]

    def test_a_dashboard_waiting_to_restart_is_stopped(self, car):
        # A crashed unit waits RestartSec in the activating state, then comes back.
        car.run('service', 'start', 'wallfollow', RC_ACTIVATING='neoracer-webteleop')
        assert car.calls() == [
            'sudo systemctl stop neoracer-webteleop.service',
            'sudo systemctl start neoracer-wallfollow.service',
        ]

    def test_a_dashboard_that_does_not_stop_leaves_the_new_one_stopped(self, car):
        (car.bin / 'sudo').write_text(
            '#!/bin/bash\necho "sudo $*" >> "$RC_LOG"\n[[ "$2" == stop ]] && exit 1\nexit 0\n'
        )
        result = car.run('service', 'start', 'wallfollow', RC_ACTIVE='neoracer-pursuit')
        assert result.returncode == 1
        assert 'neoracer-pursuit did not stop, so neoracer-wallfollow was not started' in (
            result.stderr
        )
        assert car.calls() == ['sudo systemctl stop neoracer-pursuit.service']

    @pytest.mark.parametrize(
        'args, call',
        [
            (('start', 'camlabel.service'), 'sudo systemctl start neoracer-camlabel.service'),
            (('enable', 'camlabel.service'), 'sudo systemctl enable neoracer-camlabel.service'),
            (('logs', 'camlabel.service'), 'sudo journalctl -u neoracer-camlabel -f'),
        ],
    )
    def test_a_unit_may_be_named_with_its_suffix(self, car, args, call):
        car.run('service', *args)
        assert car.calls() == [call]

    def test_camlabel_runs_alongside_a_drive_dashboard(self, car):
        car.run('service', 'start', 'camlabel', RC_ACTIVE='neoracer-wallfollow')
        assert car.calls() == ['sudo systemctl start neoracer-camlabel.service']

    def test_stopping_a_dashboard_stops_only_it(self, car):
        car.run('service', 'stop', 'pursuit', RC_ACTIVE='neoracer-pursuit neoracer-eps')
        assert car.calls() == ['sudo systemctl stop neoracer-pursuit.service']

    def test_a_unit_that_is_not_installed_is_named(self, car):
        result = car.run('service', 'start', 'mapview', RC_INSTALLED='neoracer-teleop')
        assert result.returncode == 1
        assert 'neoracer-mapview is not installed' in result.stderr
        assert car.calls() == []

    def test_bare_start_covers_the_core_stack_one_unit_per_call(self, car):
        car.run('service', 'start')
        assert car.calls() == [f'sudo systemctl start neoracer-{u}.service' for u in CORE]

    def test_bare_restart_restarts_only_enabled_units(self, car):
        enabled = 'neoracer-teleop neoracer-watchdog neoracer-camlabel'
        result = car.run('service', 'restart', RC_ENABLED=enabled)
        assert result.returncode == 0, result.stderr
        assert car.calls() == [
            'sudo systemctl restart neoracer-teleop.service',
            'sudo systemctl restart neoracer-watchdog.service',
            'sudo systemctl restart neoracer-camlabel.service',
        ]

    def test_bare_restart_leaves_a_second_drive_dashboard_stopped(self, car):
        result = car.run(
            'service',
            'restart',
            RC_ENABLED='neoracer-teleop neoracer-wallfollow',
            RC_ACTIVE='neoracer-pursuit',
        )
        assert result.returncode == 0, result.stderr
        assert car.calls() == ['sudo systemctl restart neoracer-teleop.service']
        assert 'leaving neoracer-wallfollow stopped' in result.stderr

    def test_bare_restart_restarts_one_of_two_enabled_drive_dashboards(self, car):
        enabled = 'neoracer-teleop neoracer-wallfollow neoracer-pursuit'
        car.run('service', 'restart', RC_ENABLED=enabled)
        assert car.calls() == [
            'sudo systemctl restart neoracer-teleop.service',
            'sudo systemctl restart neoracer-wallfollow.service',
        ]

    def test_bare_enable_covers_core_units_only(self, car):
        # Enabling every dashboard would put six /drive publishers on the mux
        # at boot, so the bare form does not reach them.
        car.run('service', 'enable')
        assert car.calls() == [f'sudo systemctl enable neoracer-{u}.service' for u in CORE]

    def test_logs_follow_the_named_unit(self, car):
        car.run('service', 'logs', 'mapview')
        assert car.calls() == ['sudo journalctl -u neoracer-mapview -f']

    def test_rejects_unknown_action(self, car):
        result = car.run('service', 'flambe')
        assert result.returncode == 2
        assert 'unknown action' in result.stderr

    def test_drive_units_match_the_apps_map_control_stops(self):
        drive = TEXT.split('_RC_DRIVE_UNITS=(', 1)[1].split(')', 1)[0].split()
        assert drive == [f'neoracer-{u}' for u in DRIVE]
        map_control = (ROOT / 'scripts' / 'map_control.py').read_text()
        apps = map_control.split('DRIVING_APPS = (', 1)[1].split('\n)', 1)[0]
        assert re.findall(r"'(neoracer-[a-z]+)\.service'", apps) == drive


class TestUpdate:
    """`racecar update`: reset to origin/main, run setup, restart what was enabled."""

    def _update(self, tmp_path, **env):
        package = tmp_path / 'package'
        (package / 'scripts').mkdir(parents=True)
        (package / 'scripts' / 'setup_all.sh').write_text('echo "setup_all.sh" >> "$RC_LOG"\n')
        car = Car(tmp_path, package=package)
        return car, car.run('update', **env)

    def test_resets_before_setup_then_restarts_enabled_units(self, tmp_path):
        car, result = self._update(tmp_path, RC_ENABLED='neoracer-teleop neoracer-mapview')
        assert result.returncode == 0, result.stderr
        assert car.calls() == [
            'git fetch origin',
            'git reset --hard origin/main',
            'setup_all.sh',
            'sudo systemctl restart neoracer-teleop.service',
            'sudo systemctl restart neoracer-mapview.service',
        ]

    def test_restarts_at_most_one_drive_dashboard(self, tmp_path):
        car, result = self._update(
            tmp_path, RC_ENABLED='neoracer-teleop neoracer-eps', RC_ACTIVE='neoracer-webteleop'
        )
        assert result.returncode == 0, result.stderr
        assert car.calls()[-1] == 'sudo systemctl restart neoracer-teleop.service'
        assert 'leaving neoracer-eps stopped' in result.stderr

    def test_a_failed_setup_restarts_nothing(self, tmp_path):
        package = tmp_path / 'package'
        (package / 'scripts').mkdir(parents=True)
        (package / 'scripts' / 'setup_all.sh').write_text('exit 1\n')
        car = Car(tmp_path, package=package)
        result = car.run('update', RC_ENABLED='neoracer-teleop')
        assert result.returncode == 1
        assert not any('restart' in c for c in car.calls())


class TestSetup:
    def test_no_phase_errors(self, car):
        result = car.run('setup')
        assert result.returncode == 2
        assert 'phases: all, dashboards, ml, networking' in result.stderr

    def test_unknown_phase_errors(self, car):
        result = car.run('setup', 'whatever')
        assert result.returncode == 2
        assert 'unknown phase' in result.stderr
        assert 'phases: all, dashboards, ml, networking' in result.stderr

    def test_dashboards_runs_setup_dashboards(self, car):
        result = car.run('setup', 'dashboards', '--help')
        assert result.returncode == 0, result.stderr
        assert 'Install the lab dashboards' in result.stdout
        assert car.calls() == []

    def test_networking_help(self, car):
        result = car.run('setup', 'networking', '--help')
        assert result.returncode == 0
        for flag in ('--ssid', '--psk', '--channel', '--ap-addr', '--eth-static', '--lidar-host'):
            assert flag in result.stdout
        for flag in ('--wifi-iface', '--eth-iface', '--show', '--reset'):
            assert flag in result.stdout

    def test_networking_unknown_flag_errors(self, car):
        result = car.run('setup', 'networking', '--bogus')
        assert result.returncode == 2
        assert 'unknown flag' in result.stderr

    def test_networking_show_with_no_persisted_file(self, car):
        result = car.run('setup', 'networking', '--show')
        assert result.returncode == 0
        assert 'No persisted networking config' in result.stdout

    def test_networking_reset_removes_persisted_file(self, car):
        cfg = car.home / '.config' / 'racecar' / 'networking.env'
        cfg.parent.mkdir(parents=True)
        cfg.write_text('RACECAR_AP_SSID="dummy"\n')
        result = car.run('setup', 'networking', '--reset')
        assert result.returncode == 0
        assert not cfg.exists()
        assert 'Cleared' in result.stdout

    def test_networking_flag_persists_when_combined_with_show(self, car):
        result = car.run('setup', 'networking', '--ssid=test-ssid', '--psk=test-pass', '--show')
        assert result.returncode == 0
        text = (car.home / '.config' / 'racecar' / 'networking.env').read_text()
        assert 'RACECAR_AP_SSID="test-ssid"' in text
        assert 'RACECAR_AP_PSK="test-pass"' in text
        assert 'Persisted networking config' in result.stdout

    def test_networking_reset_with_overrides_errors(self, car):
        result = car.run('setup', 'networking', '--ssid=lab-net', '--reset')
        assert result.returncode == 2
        assert 'cannot be combined' in result.stderr

    def test_top_level_help_counts_the_setup_phases(self, car):
        phases = (ROOT / 'scripts' / 'setup_all.sh').read_text().count('==> [')
        assert f'the {phases}-phase orchestrator' in car.run('help').stdout


class TestLibrary:
    """`racecar library` manages racecar_student.pth in user site-packages."""

    @pytest.fixture
    def jws(self, car):
        root = car.home / 'jupyter_ws'
        (root / 'neoracer-os' / 'library').mkdir(parents=True)
        (root / 'neoracer-os' / 'library' / 'racecar_core.py').write_text('')
        (root / 'empty').mkdir()
        return root

    def pth(self, car):
        out = subprocess.run(
            [sys.executable, '-c', 'import site; print(site.getusersitepackages())'],
            capture_output=True,
            text=True,
            env=dict(os.environ, HOME=str(car.home), PYTHONUSERBASE=str(car.home / '.local')),
        ).stdout.strip()
        return Path(out) / 'racecar_student.pth'

    def test_no_action_errors(self, car):
        result = car.run('library')
        assert result.returncode == 2
        assert 'usage:' in result.stderr

    def test_help_lists_actions(self, car):
        out = car.run('library', '--help').stdout
        for action in ('--select', '--list', '--reset', '--status'):
            assert action in out

    def test_unknown_flag_errors(self, car):
        result = car.run('library', '--bogus')
        assert result.returncode == 2
        assert 'unknown flag' in result.stderr

    def test_status_with_no_pth(self, car):
        result = car.run('library', '--status')
        assert result.returncode == 0
        assert 'No racecar library is currently selected' in result.stdout

    def test_list_with_no_jupyter_ws(self, car):
        assert 'No ~/jupyter_ws/ directory found' in car.run('library', '--list').stdout

    def test_select_writes_pth_file(self, car, jws):
        result = car.run('library', '--select', 'neoracer-os')
        assert result.returncode == 0, result.stderr
        assert self.pth(car).read_text().strip() == str(jws / 'neoracer-os' / 'library')

    def test_select_with_equals_form(self, car, jws):
        assert car.run('library', '--select=neoracer-os').returncode == 0
        assert self.pth(car).is_file()

    def test_select_rejects_missing_folder(self, car, jws):
        result = car.run('library', '--select', 'nope')
        assert result.returncode == 2
        assert 'is not a folder' in result.stderr

    def test_select_rejects_folder_without_racecar_core(self, car, jws):
        result = car.run('library', '--select', 'empty')
        assert result.returncode == 2
        assert 'library/racecar_core.py' in result.stderr

    def test_select_requires_target(self, car):
        result = car.run('library', '--select')
        assert result.returncode == 2
        assert 'requires a folder name' in result.stderr

    def test_reset_removes_pth(self, car, jws):
        car.run('library', '--select', 'neoracer-os')
        result = car.run('library', '--reset')
        assert result.returncode == 0
        assert 'removed' in result.stdout
        assert not self.pth(car).exists()

    def test_list_skips_folders_without_racecar_core(self, car, jws):
        out = car.run('library', '--list').stdout
        assert 'neoracer-os' in out
        assert 'empty' not in out

    def test_reset_when_nothing_to_remove(self, car):
        result = car.run('library', '--reset')
        assert result.returncode == 0
        assert 'no .pth file to remove' in result.stdout

    def test_status_after_select_reports_path(self, car, jws):
        car.run('library', '--select', 'neoracer-os')
        out = car.run('library', '--status').stdout
        assert f'Current library: {jws / "neoracer-os" / "library"}' in out

    def test_list_marks_current_with_asterisk(self, car, jws):
        car.run('library', '--select', 'neoracer-os')
        assert ' * neoracer-os' in car.run('library', '--list').stdout


class TestCleanup:
    def _cleanup(self, car, *args):
        (car.bin / 'ps').write_text(
            '#!/bin/bash\n'
            f'echo "99999 $USER /home/racecar/ros2_ws/install/{PKG}/lib/{PKG}/controller_node"\n'
        )
        (car.bin / 'ps').chmod(0o755)
        return car.run('cleanup', *args)

    def test_dry_run_is_the_default_and_kills_nothing(self, car):
        result = self._cleanup(car)
        assert result.returncode == 0, result.stderr
        assert 'pid=99999' in result.stdout
        assert '(dry-run; pass --force to kill)' in result.stdout
        assert car.calls() == []

    def test_explicit_dry_run_kills_nothing(self, car):
        assert self._cleanup(car, '--dry-run').returncode == 0
        assert car.calls() == []

    def test_reports_both_inventories(self, car):
        out = self._cleanup(car).stdout.lower()
        assert 'racecar processes' in out
        assert 'fastrtps' in out

    def test_help_flag_describes_behavior(self, car):
        result = car.run('cleanup', '--help')
        assert result.returncode == 0
        assert 'dry-run' in result.stdout and '--force' in result.stdout

    def test_rejects_unknown_flag(self, car):
        result = car.run('cleanup', '--burn-it-all')
        assert result.returncode == 2
        assert 'unknown flag' in result.stderr


class TestLint:
    """`racecar lint`: ruff, black --check and mypy from the package root."""

    STUB = """#!/bin/bash
printf '%s %s %s\\n' "$(basename "$0")" "$PWD" "$*" >> "$LINT_LOG"
case "$(basename "$0")" in
    ruff)  exit "${RUFF_RC:-0}" ;;
    black) exit "${BLACK_RC:-0}" ;;
    mypy)  exit "${MYPY_RC:-0}" ;;
esac
"""

    def _lint(self, car, tools=('ruff', 'black', 'mypy'), **env):
        lint_bin = car.home / 'lint-bin'
        lint_bin.mkdir()
        for tool in tools:
            (lint_bin / tool).write_text(self.STUB)
            (lint_bin / tool).chmod(0o755)
        log = car.home / 'lint.log'
        log.write_text('')
        result = car.run('lint', RACECAR_LINT_BIN=str(lint_bin), LINT_LOG=str(log), **env)
        return result, log.read_text().splitlines()

    def test_all_pass(self, car):
        result, calls = self._lint(car)
        assert result.returncode == 0, result.stderr
        assert [c.split()[0] for c in calls] == ['ruff', 'black', 'mypy']
        assert 'passed' in result.stdout

    def test_runs_from_the_package_root(self, car):
        _, calls = self._lint(car)
        for call in calls:
            assert call.split()[1] == str(car.pkg_dir), call

    def test_invocations(self, car):
        _, calls = self._lint(car)
        assert calls[0].endswith(' check .')
        assert calls[1].endswith(' --check .')

    @pytest.mark.parametrize('tool', ['ruff', 'black', 'mypy'])
    def test_any_failure_fails_and_is_named(self, car, tool):
        result, calls = self._lint(car, **{f'{tool.upper()}_RC': '1'})
        assert result.returncode == 1
        assert f'failed: {tool}' in result.stderr
        # A failure does not stop the remaining tools from reporting.
        assert len(calls) == 3

    def test_missing_tool_points_at_setup(self, car):
        result, calls = self._lint(car, tools=('ruff', 'black'))
        assert result.returncode == 3
        assert 'setup_dev_tools.sh' in result.stderr
        assert calls == []


class TestNeoRacerCommands:
    """Commands upstream does not have."""

    def test_ws_rejects_an_unknown_workspace(self, car):
        result = car.run('ws', 'nowhere')
        assert result.returncode == 2
        assert 'usage: racecar ws' in result.stderr

    def test_clear_needs_its_flag(self, car):
        result = car.run('clear')
        assert result.returncode == 2
        assert 'usage: racecar clear --led' in result.stderr

    def test_clear_led_blanks_the_display(self, car):
        assert car.run('clear', '--led').returncode == 0
        calls = car.calls()
        assert len(calls) == 1 and calls[0].startswith('ros2 topic pub --once /dotmatrix/text')

    def test_compile_help(self, car):
        result = car.run('compile', '--help')
        assert result.returncode == 0
        assert '--imgsz' in result.stdout and '--force' in result.stdout


class TestLogAndCalibrate:
    def test_log_help_lists_actions(self, car):
        result = car.run('log', '--help')
        assert result.returncode == 0, result.stderr
        for action in ('start', 'stop', 'status', 'list', 'analyze', 'config'):
            assert action in result.stdout

    def test_top_level_help_documents_log_calibrate_and_map(self, car):
        out = car.run('help').stdout
        for line in ('log <action>', 'calibrate [steps]', 'map <action>', 'profile [action]'):
            assert line in out


class TestCompletion:
    def _complete(self, *words):
        script = (
            f'set +u; source "{TOOL}"; '
            f'COMP_WORDS=({shlex.join(words)}); COMP_CWORD={len(words) - 1}; '
            '_racecar_complete; printf "%s\\n" "${COMPREPLY[@]}"'
        )
        return _bash(script).stdout.split()

    def test_completion_function_registered(self):
        result = _bash(f'source "{TOOL}" && complete -p racecar')
        assert result.returncode == 0
        assert '_racecar_complete' in result.stdout

    def test_top_level_offers_every_command(self):
        assert set(self._complete('racecar', '')) == {*COMMANDS, 'help'}

    def test_service_units(self):
        words = self._complete('racecar', 'service', 'start', '')
        for name in (*CORE, 'autonomy', *LABS):
            assert name in words, name

    def test_service_actions_drop_update(self):
        assert 'update' not in self._complete('racecar', 'service', '')

    def test_setup_offers_every_phase(self):
        words = set(self._complete('racecar', 'setup', ''))
        assert words == {'all', 'dashboards', 'ml', 'networking'}

    def test_setup_networking_offers_every_flag(self):
        words = set(self._complete('racecar', 'setup', 'networking', ''))
        parser = TEXT.split('local action="apply"')[1].split('__RC_NET_HELP__')[0]
        flags = set(re.findall(r'^\s+--([a-z-]+)\)', parser, re.M))
        assert {'ssid', 'lidar-host', 'eth-iface', 'show'} <= flags
        assert {f'--{f}=' for f in flags - {'show', 'reset', 'help'}} <= words
        assert {'--show', '--reset', '--help'} <= words

    def test_profile_actions(self):
        words = set(self._complete('racecar', 'profile', ''))
        assert words == {'path', 'migrate', 'export', 'import'}

    def test_enable_completes_unit_names(self):
        assert 'wallfollow' in self._complete('racecar', 'service', 'enable', '')

    def test_log_actions_and_start_flags(self):
        for action in ('start', 'stop', 'status', 'list', 'analyze', 'config'):
            assert action in self._complete('racecar', 'log', '')
        for flag in ('--topics', '--dir', '--name', '--storage', '--force'):
            assert flag in self._complete('racecar', 'log', 'start', '')

    def test_calibrate_steps(self):
        words = set(self._complete('racecar', 'calibrate', ''))
        assert {'status', 'check', 'lidar', 'odom', 'steering', 'dynamics'} <= words

    def test_map_actions(self):
        words = set(self._complete('racecar', 'map', ''))
        assert {'start', 'save', 'stop', 'resume', 'load', 'recover', 'status', 'list'} <= words
