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
import shlex
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
            'ros-humble-vision-msgs',
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
# Stub: an install or a removal in RACECAR_SYSTEMD_DIR runs; every other call
# is logged. Enabling the unit named in RC_ENABLE_FAILS fails.
if [[ ("$1" == install || "$1" == rm) && "${!#}" == "$RACECAR_SYSTEMD_DIR"/* ]]; then
    exec "$@"
fi
echo "sudo $*" >> "$RC_LOG"
[[ "$*" != "systemctl enable ${RC_ENABLE_FAILS:-}" ]]
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

    def run(self, tmp_path, installed=(), enabled=(), active=(), code=0, **env_extra):
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
            **env_extra,
        }
        result = subprocess.run(
            ['bash', str(SCRIPTS / 'setup_services.sh')],
            capture_output=True,
            text=True,
            timeout=30,
            env=env,
        )
        assert result.returncode == code, result.stdout + result.stderr
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

    def test_a_failed_first_enable_is_retried_on_the_next_run(self, tmp_path):
        # The first run stops at the map viewer's enable. Its new file is
        # removed again, so the next run counts it as a first install. The
        # units enabled before the failure keep their files.
        units = tmp_path / 'units'
        self.run(tmp_path, code=1, RC_ENABLE_FAILS='neoracer-mapview.service')
        assert not (units / 'neoracer-mapview.service').exists()
        before = [u for u in self.boot_units() if u != 'neoracer-mapview.service']
        assert all((units / u).is_file() for u in before)
        out, calls = self.run(tmp_path, enabled=before)
        enabled = [c.split()[-1] for c in calls if c.startswith('sudo systemctl enable')]
        assert enabled == ['neoracer-mapview.service']


JUPYTER_GIT = r"""#!/bin/bash
# Stub: `git clone` makes a one-file library or labs tree at its last
# argument, and every call is logged. A clone of a repository named in
# RC_CLONE_FAILS fails.
echo "git $*" >> "$RC_LOG"
[[ "$1" == clone ]] || exit 0
repo="${@: -2:1}"
dest="${!#}"
for name in ${RC_CLONE_FAILS:-}; do
    if [[ "$repo" == */"$name" ]]; then exit 128; fi
done
mkdir -p "$dest/.git"
case "$repo" in
    */racecar-neo-library)
        mkdir -p "$dest/library"
        echo '# library from the pin' > "$dest/library/racecar_core.py" ;;
    */neoracer-labs)
        mkdir -p "$dest/lab_a"
        echo '# shipped lab' > "$dest/lab_a/lab_a.py" ;;
esac
"""
JUPYTER_MKTEMP = """#!/bin/bash
# Stub: `mktemp -d` makes its folder under TMPDIR, as GNU mktemp does on the
# car. The Mac's mktemp ignores TMPDIR.
exec {mktemp} -d "$TMPDIR/tmp.XXXXXXXXXX"
"""
LABS_KEPT = "Labs left unchanged. Run 'racecar update --labs' to refresh them."


class TestStudentLibraryAndLabs:
    """
    setup_jupyter.sh replaces the library on every run and copies the labs only when asked.

    The part of the script from its "Student library + labs." banner to the
    end runs on its own, against a stub git, a temporary HOME and a temporary
    TMPDIR. A run on a car never fetches anything, installs a pip package, or
    touches the car's own labs, library or data root.
    """

    def run(self, tmp_path, **env_extra):
        text = read('setup_jupyter.sh')
        assert 'JUPYTER_WS="$USER_HOME/jupyter_ws"' in text
        part = text.split('\n# Student library + labs.\n', 1)[1]
        script = tmp_path / 'student_side.sh'
        script.write_text('#!/bin/bash\nset -eo pipefail\nJUPYTER_WS="$HOME/jupyter_ws"\n' + part)
        stubs = self.stubs(tmp_path)
        (tmp_path / 'tmp').mkdir(exist_ok=True)
        log = tmp_path / 'calls.log'
        log.write_text('')
        skip = ('RACECAR_LABS_REFRESH', 'LIB_REPO', 'LIB_PIN', 'LABS_REPO', 'LABS_PIN')
        env = {k: v for k, v in os.environ.items() if k not in skip}
        env.update(
            {
                'HOME': str(self.home(tmp_path)),
                'PATH': f'{stubs}:{os.environ["PATH"]}',
                'PYTHONUSERBASE': str(self.home(tmp_path) / '.local'),
                'RC_LOG': str(log),
                'TMPDIR': str(tmp_path / 'tmp'),
                **env_extra,
            }
        )
        result = subprocess.run(
            ['bash', str(script)], capture_output=True, text=True, timeout=30, env=env
        )
        return result, log.read_text().splitlines()

    def stubs(self, tmp_path):
        stubs = tmp_path / 'bin'
        if not stubs.exists():
            stubs.mkdir()
            (stubs / 'git').write_text(JUPYTER_GIT)
            (stubs / 'mktemp').write_text(
                JUPYTER_MKTEMP.format(mktemp=shlex.quote(shutil.which('mktemp')))
            )
            for name in ('git', 'mktemp'):
                (stubs / name).chmod(0o755)
            (stubs / 'python3').symlink_to(sys.executable)
        return stubs

    def home(self, tmp_path):
        home = tmp_path / 'home'
        home.mkdir(exist_ok=True)
        return home

    def neo(self, tmp_path):
        return self.home(tmp_path) / 'jupyter_ws' / 'neoracer-os'

    def pth(self, tmp_path):
        out = subprocess.run(
            [sys.executable, '-c', 'import site; print(site.getusersitepackages())'],
            capture_output=True,
            text=True,
            env=dict(
                os.environ,
                HOME=str(self.home(tmp_path)),
                PYTHONUSERBASE=str(self.home(tmp_path) / '.local'),
            ),
        ).stdout.strip()
        return Path(out) / 'racecar_student.pth'

    def select(self, tmp_path, library):
        pth = self.pth(tmp_path)
        pth.parent.mkdir(parents=True, exist_ok=True)
        pth.write_text(f'{library}\n')
        return pth

    def edited_labs(self, tmp_path):
        """Make the labs folder of a car set up before, with one shipped lab edited."""
        lab = self.neo(tmp_path) / 'labs' / 'lab_a' / 'lab_a.py'
        lab.parent.mkdir(parents=True)
        lab.write_text('# student edit\n')
        return lab

    def fetched_labs(self, calls):
        return any('neoracer-labs' in c for c in calls)

    def backups(self, tmp_path):
        return sorted(self.neo(tmp_path).glob('labs-backup-*'))

    def test_a_first_install_copies_the_labs(self, tmp_path):
        result, calls = self.run(tmp_path)
        assert result.returncode == 0, result.stdout + result.stderr
        labs = self.neo(tmp_path) / 'labs'
        assert (labs / 'lab_a' / 'lab_a.py').read_text() == '# shipped lab\n'
        assert not (labs / '.git').exists()
        assert self.fetched_labs(calls)
        assert not self.backups(tmp_path)
        assert self.pth(tmp_path).read_text() == f'{self.neo(tmp_path) / "library"}\n'
        assert not list((tmp_path / 'tmp').iterdir())

    @pytest.mark.parametrize('pin', [None, 'labs-test-branch'])
    def test_the_labs_are_cloned_at_their_pin(self, tmp_path, pin):
        # The labs are pinned like the library: the tag made for this driver
        # release by default, or the LABS_PIN a test car sets.
        env = {} if pin is None else {'LABS_PIN': pin}
        result, calls = self.run(tmp_path, **env)
        assert result.returncode == 0, result.stdout + result.stderr
        clones = [c for c in calls if c.startswith('git clone') and 'neoracer-labs' in c]
        assert len(clones) == 1
        assert f'--branch {pin or "v0.9.0-neobotics"} ' in clones[0]
        assert f'Fetching labs ({pin or "v0.9.0-neobotics"})...' in result.stdout

    @pytest.mark.parametrize('refresh', [None, '0'])
    def test_a_rerun_leaves_the_labs_alone_and_does_not_fetch_them(self, tmp_path, refresh):
        lab = self.edited_labs(tmp_path)
        env = {} if refresh is None else {'RACECAR_LABS_REFRESH': refresh}
        result, calls = self.run(tmp_path, **env)
        assert result.returncode == 0, result.stdout + result.stderr
        assert lab.read_text() == '# student edit\n'
        assert not self.fetched_labs(calls)
        assert LABS_KEPT in result.stdout.splitlines()
        assert not self.backups(tmp_path)

    def test_a_refresh_backs_up_the_labs_then_replaces_the_shipped_files(self, tmp_path):
        lab = self.edited_labs(tmp_path)
        own = self.neo(tmp_path) / 'labs' / 'my_lab.py'
        own.write_text('# student file\n')
        result, calls = self.run(tmp_path, RACECAR_LABS_REFRESH='1')
        assert result.returncode == 0, result.stdout + result.stderr
        backups = self.backups(tmp_path)
        assert len(backups) == 1
        assert re.fullmatch(r'labs-backup-\d{8}-\d{6}', backups[0].name)
        assert (backups[0] / 'lab_a' / 'lab_a.py').read_text() == '# student edit\n'
        assert (backups[0] / 'my_lab.py').read_text() == '# student file\n'
        assert f'Labs backed up to {backups[0]}' in result.stdout
        assert lab.read_text() == '# shipped lab\n'
        assert own.read_text() == '# student file\n'
        assert LABS_KEPT not in result.stdout

    def test_every_run_replaces_the_library(self, tmp_path):
        self.edited_labs(tmp_path)
        old = self.neo(tmp_path) / 'library'
        old.mkdir()
        (old / 'racecar_core.py').write_text('# old library\n')
        (old / 'stale.py').write_text('')
        result, _ = self.run(tmp_path)
        assert result.returncode == 0, result.stdout + result.stderr
        assert (old / 'racecar_core.py').read_text() == '# library from the pin\n'
        assert not (old / 'stale.py').exists()

    def test_another_selected_library_is_kept(self, tmp_path):
        self.edited_labs(tmp_path)
        mine = self.home(tmp_path) / 'jupyter_ws' / 'my-library' / 'library'
        mine.mkdir(parents=True)
        (mine / 'racecar_core.py').write_text('')
        pth = self.select(tmp_path, mine)
        result, _ = self.run(tmp_path)
        assert result.returncode == 0, result.stdout + result.stderr
        assert pth.read_text() == f'{mine}\n'
        assert f'Library selection kept: {mine}' in result.stdout.splitlines()

    @pytest.mark.parametrize('folder', ['missing', 'without racecar_core.py'])
    def test_a_selection_whose_library_is_gone_is_reset(self, tmp_path, folder):
        # racecar library --select and --list count a folder as a library only
        # when it holds racecar_core.py.
        self.edited_labs(tmp_path)
        gone = self.home(tmp_path) / 'jupyter_ws' / 'old-library' / 'library'
        if folder != 'missing':
            gone.mkdir(parents=True)
        pth = self.select(tmp_path, gone)
        result, _ = self.run(tmp_path)
        assert result.returncode == 0, result.stdout + result.stderr
        assert pth.read_text() == f'{self.neo(tmp_path) / "library"}\n'
        assert 'Library selection kept' not in result.stdout

    @pytest.mark.parametrize('repo', ['racecar-neo-library', 'neoracer-labs'])
    def test_a_failed_fetch_changes_nothing(self, tmp_path, repo):
        lab = self.edited_labs(tmp_path)
        old = self.neo(tmp_path) / 'library'
        old.mkdir()
        (old / 'racecar_core.py').write_text('# old library\n')
        pth = self.select(tmp_path, old)
        result, _ = self.run(tmp_path, RACECAR_LABS_REFRESH='1', RC_CLONE_FAILS=repo)
        assert result.returncode == 1
        assert 'the labs on the car' in result.stderr
        assert lab.read_text() == '# student edit\n'
        assert (old / 'racecar_core.py').read_text() == '# old library\n'
        assert pth.read_text() == f'{old}\n'
        assert not self.backups(tmp_path)
        assert not list((tmp_path / 'tmp').iterdir())

    def test_a_failed_backup_leaves_no_temp_folder(self, tmp_path):
        # A stub date gives the backup the name of a folder already there,
        # as two refreshes in one second would, so its mkdir fails.
        lab = self.edited_labs(tmp_path)
        date = self.stubs(tmp_path) / 'date'
        date.write_text('#!/bin/bash\necho 20260101-000000\n')
        date.chmod(0o755)
        taken = self.neo(tmp_path) / 'labs-backup-20260101-000000'
        taken.mkdir()
        result, _ = self.run(tmp_path, RACECAR_LABS_REFRESH='1')
        assert result.returncode == 1
        assert lab.read_text() == '# student edit\n'
        assert not list(taken.iterdir())
        assert not (self.neo(tmp_path) / 'library').exists()
        assert not list((tmp_path / 'tmp').iterdir())


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
