"""
Static checks for the setup scripts, the systemd units and the sudoers file.

Ported from upstream's test_setup_scripts.py at v0.9.0. The Raspberry Pi,
Coral, RealSense, dot matrix, NVMe, Ethernet and polkit tests are left out,
because the NeoRacer has none of those. The lab dashboard install and the udev
rules have their own test files. The sudoers tests are the NeoRacer's: it uses
a sudoers drop-in where upstream uses polkit rules, which Ubuntu 22.04's polkit
0.105 cannot read.
"""

import os
from pathlib import Path
import re
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / 'scripts'

PHASE_SCRIPTS = [
    'setup_ros2.sh',
    'setup_dev_tools.sh',
    'setup_user_env.sh',
    'setup_udev.sh',
    'setup_workspace.sh',
    'setup_ml.sh',
    'setup_jupyter.sh',
    'setup_services.sh',
]
ORCHESTRATOR = 'setup_all.sh'
# setup_services.sh runs it, so setup_all.sh does not name it.
DASHBOARD_SETUP = 'setup_dashboards.sh'
# Shipped but never run by setup_all.sh: it reconfigures the car's network and
# can drop the SSH session that runs the setup.
STANDALONE_SCRIPTS = ['setup_networking.sh']
RUNTIME_SCRIPTS = ['launch_teleop.sh', 'launch_autonomy.sh']
ALL_SCRIPTS = [
    *PHASE_SCRIPTS,
    ORCHESTRATOR,
    DASHBOARD_SETUP,
    *STANDALONE_SCRIPTS,
    *RUNTIME_SCRIPTS,
]

PY_SCRIPTS = sorted(
    str(p.relative_to(SCRIPTS))
    for p in [
        *SCRIPTS.glob('*.py'),
        *SCRIPTS.glob('calibration/*.py'),
        *SCRIPTS.glob('validation/*.py'),
    ]
)
# Entry points run by path or by `ros2 run`; the rest are imported siblings.
PY_ENTRY_POINTS = [
    name for name in PY_SCRIPTS if "if __name__ == '__main__':" in (SCRIPTS / name).read_text()
]

# Enabled at boot.
BOOT_UNITS = ['data', 'teleop', 'watchdog', 'dashboard', 'jupyter', 'mapview']
# Installed, started on demand.
ON_DEMAND_UNITS = ['autonomy']
LAB_UNITS = ['wallfollow', 'camlabel', 'pursuit', 'eps', 'smartfollow', 'linefollow', 'webteleop']
# Started and stopped with no password. neoracer-data is a oneshot that runs
# only at boot.
SUDO_UNITS = ['teleop', 'watchdog', 'dashboard', 'jupyter', 'mapview', 'autonomy', *LAB_UNITS]
SUDOERS = SCRIPTS / 'sudoers.d-neoracer'


def read(name):
    return (SCRIPTS / name).read_text()


def bash_array(text, name):
    """Return the words of a bash array assignment such as `SERVICES=( ... )`."""
    return text.split(f'\n{name}=(', 1)[1].split(')', 1)[0].split()


@pytest.mark.parametrize('name', ALL_SCRIPTS)
def test_script_exists(name):
    assert (SCRIPTS / name).is_file(), f'{name} missing from scripts/'


@pytest.mark.parametrize('name', ALL_SCRIPTS)
def test_script_is_executable(name):
    assert os.access(SCRIPTS / name, os.X_OK), f'{name} missing +x bit'


@pytest.mark.parametrize('name', ALL_SCRIPTS)
def test_script_has_bash_hashbang(name):
    first = read(name).splitlines()[0]
    assert first.startswith('#!'), f'{name} missing shebang'
    assert 'bash' in first, f'{name} should use bash (got: {first!r})'


@pytest.mark.parametrize('name', [*ALL_SCRIPTS, 'racecar-tool.sh'])
def test_script_passes_bash_syntax(name):
    result = subprocess.run(
        ['bash', '-n', str(SCRIPTS / name)], capture_output=True, text=True, timeout=10
    )
    assert result.returncode == 0, f'{name} fails bash -n:\n{result.stderr}'


@pytest.mark.parametrize('name', PY_SCRIPTS)
def test_python_script_compiles(name):
    result = subprocess.run(
        [sys.executable, '-m', 'py_compile', str(SCRIPTS / name)],
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('name', PY_ENTRY_POINTS)
def test_python_entry_point_is_executable(name):
    path = SCRIPTS / name
    assert path.read_text().startswith('#!/usr/bin/env python3\n'), f'{name} missing shebang'
    assert os.access(path, os.X_OK), f'{name} missing +x bit'


def test_python_entry_points_found():
    assert {'dashboard.py', 'diagnose.py', 'watchdog.py', 'mapview.py'} <= set(PY_ENTRY_POINTS)


def test_orchestrator_calls_every_phase_script():
    text = read(ORCHESTRATOR)
    for phase in PHASE_SCRIPTS:
        assert f'"$SCRIPT_DIR/{phase}"' in text, f'{ORCHESTRATOR} does not run {phase}'
    assert f'"$SCRIPT_DIR/{DASHBOARD_SETUP}"' in read('setup_services.sh')


@pytest.mark.parametrize('name', [*PHASE_SCRIPTS, ORCHESTRATOR])
def test_scripts_fail_fast(name):
    # pipefail catches `wget | dpkg -i` style chains where the first command fails.
    assert 'set -eo pipefail' in read(name), f'{name} should `set -eo pipefail`'


@pytest.mark.parametrize('name', [*PHASE_SCRIPTS, ORCHESTRATOR, DASHBOARD_SETUP])
def test_no_grep_q_after_a_pipe(name):
    # Under pipefail, grep -q exiting at the first match can make the writer
    # fail on a closed pipe, and the check then fails although it matched.
    assert '| grep -q' not in read(name)


def test_no_stray_colcon_dirs_in_package():
    """build/, install/ and log/ live in the workspace root, not the package."""
    for d in ('build', 'install', 'log'):
        stray = ROOT / d
        assert not stray.exists(), (
            f'{stray} exists; colcon was run from the wrong folder. '
            f'Run `colcon build` from ~/ros2_ws, or `racecar build`.'
        )


@pytest.mark.parametrize('name', STANDALONE_SCRIPTS)
def test_standalone_scripts_stay_out_of_the_orchestrator(name):
    assert f'"$SCRIPT_DIR/{name}"' not in read(ORCHESTRATOR)


class TestOneShotSetup:
    """What a car flashed fresh with JetPack needs from setup_all.sh."""

    def all_setup_text(self):
        return '\n'.join(read(s) for s in [*PHASE_SCRIPTS, ORCHESTRATOR])

    @pytest.mark.parametrize(
        'package',
        [
            'ros-humble-ros-base',
            'ros-humble-robot-state-publisher',
            'ros-humble-xacro',
            'ros-humble-slam-toolbox',
            'ros-humble-nav2-amcl',
            'ros-humble-nav2-map-server',
            'ros-humble-nav2-lifecycle-manager',
            'ros-humble-foxglove-bridge',
            'ros-humble-rosbag2-storage-mcap',
            'ros-humble-topic-tools',
            'ros-humble-diagnostic-updater',
            'python3-pil',
            'python3-scipy',
        ],
    )
    def test_runtime_packages_are_installed(self, package):
        assert package in self.all_setup_text(), package

    def test_refuses_the_wrong_user_root_or_folder(self):
        text = read(ORCHESTRATOR)
        assert text.index('EXPECTED_DIR=') < text.index('sudo -v')
        assert '"$EUID" -eq 0' in text and '"$(id -un)" != "racecar"' in text
        assert 'EXPECTED_DIR="/home/racecar/ros2_ws/src/neoracer_ros2_driver/scripts"' in text

    def test_network_installs_are_pinned(self):
        assert 'LAKI_PIN="${LAKI_PIN:-v' in read('setup_workspace.sh')
        jupyter = read('setup_jupyter.sh')
        pin = re.search(r'LIB_PIN="\$\{LIB_PIN:-([^}]+)\}"', jupyter)
        assert pin and pin.group(1) != 'main', 'the student library must be pinned to a tag'
        assert '--branch "$LIB_PIN"' in jupyter
        for pin in ('ruff==0.16.8', 'black==26.5.1', 'mypy==2.3.1'):
            assert pin in read('setup_dev_tools.sh')

    def test_every_unit_the_tools_manage_is_installed(self):
        # The arrays, not the header comment, which names every unit too.
        services = read('setup_services.sh')
        assert bash_array(services, 'SERVICES') == [f'neoracer-{u}.service' for u in BOOT_UNITS]
        assert bash_array(services, 'INSTALL_ONLY') == [
            f'neoracer-{u}.service' for u in ON_DEMAND_UNITS
        ]

    def test_ros_apt_source_lookup_has_a_fallback(self):
        # Under pipefail an empty lookup would stop the script with no message.
        text = read('setup_ros2.sh')
        assert 'curl -fsS --retry 3' in text and 'ROS_APT_SOURCE_FALLBACK=' in text
        assert "awk -F'\"' '{print $4}' || true)" in text
        assert text.index('apt-get update') < text.index('apt-get install')

    def test_udev_trigger_is_scoped(self):
        # A bare trigger replays every device, the network included.
        text = read('setup_udev.sh')
        assert 'sudo udevadm trigger\n' not in text
        assert 'sudo udevadm trigger --subsystem-match="$subsystem"' in text
        rules = (SCRIPTS / 'udev' / '99-racecar.rules').read_text()
        matched = set(re.findall(r'\bSUBSYSTEM=="([a-z0-9_]+)"', rules))
        loop = re.search(r'for subsystem in ([a-z0-9_ ]+); do', text)
        assert loop and matched <= set(loop.group(1).split()), matched

    def test_a_missing_library_pin_says_so(self):
        # The tag is made when the library release merges; until then the
        # clone fails, and setup names the override.
        text = read('setup_jupyter.sh')
        clone = 'if ! git clone --depth 1 --branch "$LIB_PIN" "$LIB_REPO" "$TMP/lib"; then'
        assert clone in text
        assert 'set LIB_PIN' in text.split(clone, 1)[1].split('\nfi\n', 1)[0]

    def test_every_boot_unit_is_a_status_row(self):
        text = read('diagnose.py')
        rows = text.split('SERVICE_UNITS = (', 1)[1].split(')', 1)[0]
        for unit in BOOT_UNITS:
            assert f"'neoracer-{unit}'" in rows, unit

    def test_tool_completion_names_every_unit(self):
        text = read('racecar-tool.sh')
        line = re.search(r'start\|stop\|restart\|logs\|enable\|disable\)\n\s+COMPREPLY=.*', text)
        assert line
        names = set(re.search(r'compgen -W "([^"]+)"', line.group(0)).group(1).split())
        for unit in SUDO_UNITS:
            assert unit in names, unit


class TestRerunOnASetUpCar:
    """A second run, such as `racecar update`, on a car set up before."""

    def test_build_clears_dangling_links_first(self):
        # A file deleted since the last --symlink-install build leaves a link,
        # and the build fails when setuptools copies it.
        text = read('setup_workspace.sh')
        assert 'find "$tree" -xtype l -print0' in text
        assert text.index('-xtype l') < text.rindex('colcon build')

    def test_old_autonomy_unit_is_disabled_before_the_new_one_is_installed(self):
        # The pre-0.9.0 unit could be enabled at boot. Its boot link would
        # start the new unit at boot too, so setup stops the old unit and
        # removes the link, even one whose unit file is gone.
        text = read('setup_services.sh')
        assert 'SYSTEMD_DIR="${RACECAR_SYSTEMD_DIR:-/etc/systemd/system}"' in text
        assert 'link="$SYSTEMD_DIR/multi-user.target.wants/$svc"' in text
        assert 'if [[ -L "$link" ]]; then' in text
        remove = text.index('sudo rm -f "$link"')
        assert text.index('sudo systemctl stop "$svc"') < remove
        assert remove < text.index('sudo install -m 0644 "$src" "$dst"')

    def test_local_yaml_is_not_installed(self):
        # Per-car files from before the car profile; profile migrate moves them.
        setup_py = (ROOT / 'setup.py').read_text()
        assert "if not f.endswith('.local.yaml')" in setup_py

    def test_a_running_car_starts_what_this_run_enabled(self):
        # `racecar update` from 0.6.x restarts only the units its older tool
        # lists, so the map viewer would otherwise wait for a reboot.
        text = read('setup_services.sh')
        assert 'newly_enabled+=("$svc")' in text
        start = text.index('if systemctl is-active --quiet neoracer-teleop.service; then')
        assert 'for svc in "${newly_enabled[@]}"; do' in text[start:]
        assert text.index('newly_enabled+=("$svc")') < start


SERVICES_SUDO = r"""#!/bin/bash
# Stub: an install into RACECAR_SYSTEMD_DIR runs; every other call is logged.
if [[ "$1" == install && "${!#}" == "$RACECAR_SYSTEMD_DIR"/* ]]; then
    exec "$@"
fi
echo "sudo $*" >> "$RC_LOG"
"""
SERVICES_SYSTEMCTL = r"""#!/bin/bash
# Stub: is-enabled and is-active answer from RC_ENABLED and RC_ACTIVE; every
# other call is logged.
args=()
for a in "$@"; do [[ "$a" == --quiet ]] || args+=("$a"); done
case "${args[0]}" in
    is-enabled) [[ " $RC_ENABLED " == *" ${args[1]} "* ]] ;;
    is-active) [[ " $RC_ACTIVE " == *" ${args[1]} "* ]] ;;
    list-unit-files) ;;
    *) echo "systemctl ${args[*]}" >> "$RC_LOG" ;;
esac
"""


class TestEnableState:
    """
    setup_services.sh enables a unit at its first install and never after.

    The script runs against stub sudo and systemctl commands and a temporary
    unit folder, so a run on a car never installs, enables or starts anything.
    """

    def run(self, tmp_path, installed=(), enabled=(), active=()):
        units = tmp_path / 'units'
        units.mkdir(exist_ok=True)
        for svc in installed:
            (units / svc).write_text('an older unit file\n')
        stubs = tmp_path / 'bin'
        stubs.mkdir(exist_ok=True)
        for name, body in (('sudo', SERVICES_SUDO), ('systemctl', SERVICES_SYSTEMCTL)):
            (stubs / name).write_text(body)
            (stubs / name).chmod(0o755)
        log = tmp_path / 'calls.log'
        log.write_text('')
        env = {
            **os.environ,
            'PATH': f'{stubs}:{os.environ["PATH"]}',
            'RACECAR_SYSTEMD_DIR': str(units),
            'RACECAR_DASHBOARDS': '0',
            'RC_LOG': str(log),
            'RC_ENABLED': ' '.join(enabled),
            'RC_ACTIVE': ' '.join(active),
        }
        result = subprocess.run(
            ['bash', str(SCRIPTS / 'setup_services.sh')],
            capture_output=True,
            text=True,
            timeout=30,
            env=env,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        return result.stdout, log.read_text().splitlines()

    def boot_units(self):
        return [f'neoracer-{u}.service' for u in BOOT_UNITS]

    def test_a_first_install_enables_every_boot_unit(self, tmp_path):
        out, calls = self.run(tmp_path)
        enabled = [c.split()[-1] for c in calls if c.startswith('sudo systemctl enable')]
        assert enabled == self.boot_units()
        assert (tmp_path / 'units' / 'neoracer-autonomy.service').is_file()
        assert not any(c.startswith('sudo systemctl start') for c in calls)

    def test_a_unit_disabled_on_the_car_stays_disabled(self, tmp_path):
        units = [*self.boot_units(), 'neoracer-autonomy.service']
        kept_on = [u for u in self.boot_units() if u != 'neoracer-mapview.service']
        out, calls = self.run(tmp_path, installed=units, enabled=kept_on)
        assert not any(c.startswith('sudo systemctl enable') for c in calls)
        assert 'neoracer-mapview.service: installed before; enable state kept' in out
        assert out.count('already enabled') == len(kept_on)

    def test_an_upgrade_enables_and_starts_only_the_new_unit(self, tmp_path):
        # A 0.6.x car has no map viewer unit. Its jupyter unit was disabled
        # by hand, and stays so.
        old = [u for u in self.boot_units() if u != 'neoracer-mapview.service']
        on = [u for u in old if u != 'neoracer-jupyter.service']
        out, calls = self.run(
            tmp_path, installed=old, enabled=on, active=['neoracer-teleop.service']
        )
        changes = [
            c for c in calls if c.startswith(('sudo systemctl enable', 'sudo systemctl start'))
        ]
        assert changes == [
            'sudo systemctl enable neoracer-mapview.service',
            'sudo systemctl start neoracer-mapview.service',
        ]
        assert 'neoracer-jupyter.service: installed before; enable state kept' in out


class TestLaunchWrapper:
    """launch_teleop.sh is what neoracer-teleop.service and `racecar teleop` run."""

    def test_creates_log_dir_and_symlink(self):
        text = read('launch_teleop.sh')
        assert 'mkdir -p "$LOG_DIR"' in text
        assert 'ln -sfn "$LOG_DIR" "$HOME/logs/latest.tmp"' in text
        assert 'mv -Tf "$HOME/logs/latest.tmp" "$HOME/logs/latest"' in text

    def test_log_mirror_starts_before_any_output(self):
        text = read('launch_teleop.sh')
        assert text.index('exec &> >(tee -a "$LOG_DIR/teleop.log")') < text.index('echo ')

    def test_sweeps_fastrtps_shm_orphans(self):
        assert '/dev/shm/fastrtps_port' in read('launch_teleop.sh')

    def test_execs_ros2_launch(self):
        # `exec` lets systemd track the launch PID.
        assert 'exec ros2 launch neoracer_ros2_driver teleop.launch.py' in read('launch_teleop.sh')


class TestSystemdServices:
    @pytest.mark.parametrize('name', [*BOOT_UNITS, *ON_DEMAND_UNITS])
    def test_runs_as_racecar_user(self, name):
        text = read(f'neoracer-{name}.service')
        assert '[Unit]' in text and '[Service]' in text
        assert 'User=racecar' in text and 'Group=racecar' in text

    @pytest.mark.parametrize('name', BOOT_UNITS)
    def test_boot_units_are_wanted_by_multi_user(self, name):
        text = read(f'neoracer-{name}.service')
        assert '[Install]' in text and 'WantedBy=multi-user.target' in text

    def test_autonomy_is_never_started_at_boot(self):
        # Mapping at boot would build an unsaved map and take CPU from every lab.
        text = read('neoracer-autonomy.service')
        assert '[Install]' not in text
        assert 'BindsTo=neoracer-teleop.service' in text
        services = read('setup_services.sh')
        listed = services.split('SERVICES=(', 1)[1].split(')', 1)[0].split()
        assert 'neoracer-autonomy.service' not in listed
        assert 'INSTALL_ONLY=(\n    neoracer-autonomy.service\n)' in services

    def test_watchdog_bindsto_teleop(self):
        text = read('neoracer-watchdog.service')
        assert 'BindsTo=neoracer-teleop.service' in text
        assert 'After=neoracer-teleop.service' in text
        assert 'watchdog.py' in text

    def test_mapview_independent_of_teleop(self):
        # The page offers "Start mapping" while teleop and autonomy are down.
        text = read('neoracer-mapview.service')
        assert 'teleop' not in text.split('[Service]')[0]
        assert 'scripts/mapview.py' in text

    def test_teleop_wants_watchdog(self):
        # Starting teleop, by hand or at boot, pulls the watchdog along.
        assert 'Wants=neoracer-watchdog.service' in read('neoracer-teleop.service')

    def test_teleop_calls_launch_wrapper(self):
        assert 'launch_teleop.sh' in read('neoracer-teleop.service')


class TestSudoers:
    """/etc/sudoers.d/neoracer, installed from scripts/sudoers.d-neoracer."""

    RULE = re.compile(r'/usr/bin/systemctl (start|stop|restart) (neoracer-[a-z]+\.service)')

    def rules(self):
        text = SUDOERS.read_text()
        return {(m.group(1), m.group(2)) for m in self.RULE.finditer(text)}

    def test_every_rule_names_one_unit_and_one_verb(self):
        # In sudoers * also matches spaces, so a wildcard would allow other
        # units and options.
        body = [ln for ln in SUDOERS.read_text().splitlines() if not ln.startswith('#')]
        assert not any('*' in ln for ln in body)
        commands = [
            ln.strip().rstrip(', \\') for ln in body if ln.strip().startswith('/usr/bin/systemctl')
        ]
        assert commands and all(self.RULE.fullmatch(c) for c in commands), commands

    def test_grants_only_the_racecar_user_with_no_password(self):
        grants = [ln for ln in SUDOERS.read_text().splitlines() if 'NOPASSWD' in ln]
        assert grants == ['racecar ALL=(root) NOPASSWD: NEORACER_CORE, NEORACER_LABS']

    def test_every_managed_unit_has_start_stop_and_restart(self):
        rules = self.rules()
        for name in SUDO_UNITS:
            for verb in ('start', 'stop', 'restart'):
                assert (verb, f'neoracer-{name}.service') in rules, (verb, name)

    def test_every_unit_in_the_file_is_installed(self):
        named = {unit for _, unit in self.rules()}
        shipped = {p.name for p in SCRIPTS.glob('neoracer-*.service')}
        shipped |= {
            p.name[: -len('.in')] for p in (SCRIPTS / 'dashboards').glob('*/neoracer-*.service.in')
        }
        assert named <= shipped, named - shipped

    def test_setup_checks_the_file_before_installing_it(self):
        text = read('setup_services.sh')
        check = text.index('sudo visudo -cf "$SUDOERS_SRC"')
        install = text.index('sudo install -o root -g root -m 0440 "$SUDOERS_SRC" "$SUDOERS_TMP"')
        rename = text.index('sudo mv -f "$SUDOERS_TMP" "$SUDOERS_DST"')
        assert check < install < rename
        assert 'SUDOERS_DST="/etc/sudoers.d/neoracer"' in text
        # sudo skips a file in /etc/sudoers.d whose name has a dot, so it never
        # reads the copy before the rename.
        tmp = re.search(r'SUDOERS_TMP="/etc/sudoers\.d/([^"]+)"', text)
        assert tmp and '.' in tmp.group(1)

    def test_an_installed_file_with_the_wrong_mode_is_replaced(self):
        # A copy made by hand with the same text but another mode or owner is
        # installed again as root, 0440.
        text = read('setup_services.sh')
        mode = '"$(sudo stat -c \'%a %U %G\' "$SUDOERS_DST" 2>/dev/null)" == "440 root root"'
        assert mode in text

    def test_visudo_parses_the_file(self):
        visudo = shutil.which('visudo') or next(
            (p for p in ('/usr/sbin/visudo', '/sbin/visudo') if os.access(p, os.X_OK)), None
        )
        if visudo is None:
            pytest.skip('visudo is not installed')
        result = subprocess.run(
            [visudo, '-cf', str(SUDOERS)], capture_output=True, text=True, timeout=10
        )
        assert result.returncode == 0, result.stdout + result.stderr

    def test_every_unit_the_python_callers_manage_is_in_the_file(self):
        # The 8080 page and `racecar map` run `sudo -n systemctl` with these
        # names; one missing from the file would fail with no password prompt.
        named = {unit for _, unit in self.rules()}
        for script in ('dashboard.py', 'map_control.py'):
            units = set(re.findall(r"'(neoracer-[a-z]+)(?:\.service)?'", read(script)))
            assert units, script
            assert {f'{u}.service' for u in units} <= named, (script, units)

    def test_tool_calls_systemctl_one_full_unit_at_a_time(self):
        # A call naming several units, or a unit without .service, would not
        # match a rule and would ask for the password.
        text = read('racecar-tool.sh')
        calls = re.findall(r'sudo systemctl (\S+) (.+)', text)
        assert calls
        for verb, rest in calls:
            assert rest.rstrip().endswith('.service"') or rest.rstrip().endswith(
                '.service" || return'
            ), (verb, rest)
        assert not re.search(r'sudo systemctl \S+ "\$\{[a-z]+\[@\]\}"', text)
