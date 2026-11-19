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
# Started and stopped with no password. neoracer-data is a oneshot that only
# boot runs.
SUDO_UNITS = ['teleop', 'watchdog', 'dashboard', 'jupyter', 'mapview', 'autonomy', *LAB_UNITS]
SUDOERS = SCRIPTS / 'sudoers.d-neoracer'


def read(name):
    return (SCRIPTS / name).read_text()


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
        services = read('setup_services.sh')
        for unit in [*BOOT_UNITS, *ON_DEMAND_UNITS]:
            assert f'neoracer-{unit}.service' in services, unit


class TestRerunOnASetUpCar:
    """A second run, such as `racecar update`, on a car set up before."""

    def test_build_clears_dangling_links_first(self):
        # A file deleted since the last --symlink-install build leaves a link
        # that setuptools refuses to copy.
        text = read('setup_workspace.sh')
        assert 'find "$tree" -xtype l -print0' in text
        assert text.index('-xtype l') < text.rindex('colcon build')

    def test_old_autonomy_unit_is_disabled_before_the_new_one_is_installed(self):
        # The pre-0.9.0 unit could be enabled at boot. The new one has no
        # [Install] section, so its boot link must go while the old file is in place.
        text = read('setup_services.sh')
        disable = text.index('sudo systemctl disable "$svc"')
        install = text.index('sudo install -m 0644 "$src" "$dst"')
        assert disable < install
        assert 'multi-user.target.wants/$svc' in text


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
        check = text.index('sudo visudo -cqf "$SUDOERS_SRC"')
        install = text.index('sudo install -o root -g root -m 0440 "$SUDOERS_SRC" "$SUDOERS_DST"')
        assert check < install
        assert 'SUDOERS_DST="/etc/sudoers.d/neoracer"' in text

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
