"""
Tests for the in-tree lab dashboards' install: setup_dashboards.sh and checkouts.py.

setup_dashboards.sh runs against a temporary unit folder and a copy of the
dashboards, with sudo and systemctl stubbed, so a test on a car never moves
that car's own old checkouts. checkouts.py runs against a temporary dashboards folder
holding checkouts built the way the old setup_services.sh cloned them, with
the car profile and the data root in temporary folders.
"""

from datetime import datetime
import os
from pathlib import Path
import re
import shlex
import shutil
import subprocess
import sys

from conftest import load_script
import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent
SCRIPTS = ROOT / 'scripts'
DASH = SCRIPTS / 'dashboards'
SETUP = SCRIPTS / 'setup_dashboards.sh'
# The ids, ports and units shared with the 8080 apps page (F32).
TABLE = {
    'wallfollow': 8081,
    'camlabel': 8082,
    'pursuit': 8083,
    'eps': 8084,
    'smartfollow': 8085,
    'linefollow': 8086,
    'webteleop': 8087,
}
# Each dashboard's script and yaml name.
STEMS = {name: name for name in TABLE} | {'webteleop': 'teleop'}
NOW = datetime(2026, 11, 17, 12, 0, 0)


@pytest.fixture(scope='module')
def co():
    return load_script('checkouts')


def tree_copy(tmp_path):
    """Copy the seven dashboard folders, so a test never moves a car's own checkouts."""
    out = tmp_path / 'dashboards'
    if not out.exists():
        for name in TABLE:
            shutil.copytree(DASH / name, out / name, ignore=shutil.ignore_patterns('__pycache__'))
    return out


def run_setup(tmp_path, units, systemctl='true', **env):
    """Run setup_dashboards.sh on a copy of the tree, with every car folder in tmp_path."""
    home = tmp_path / 'home'
    home.mkdir(exist_ok=True)
    full = {
        **os.environ,
        'HOME': str(home),
        'RACECAR_SYSTEMD_DIR': str(units),
        'RACECAR_SUDO': '',
        'RACECAR_SYSTEMCTL': systemctl,
        'RACECAR_CONFIG_DIR': str(tmp_path / 'profile'),
        'RACECAR_DATA_DIR': str(tmp_path / 'data'),
        'RACECAR_DASHBOARDS_DIR': str(tree_copy(tmp_path)),
        **env,
    }
    return subprocess.run(['bash', str(SETUP)], capture_output=True, text=True, env=full)


@pytest.fixture(scope='module')
def text():
    return SETUP.read_text()


class TestSetupScript:

    def test_never_enables_or_starts(self, text):
        # Six publish /drive; enabling them all would put six publishers on
        # the mux at boot.
        assert 'systemctl enable' not in text
        assert 'systemctl start' not in text

    def test_needs_no_network(self, text):
        assert 'git clone' not in text and 'git fetch' not in text

    def test_verifies_every_substitution_token(self, text):
        for token in ('@DIR@', 'Environment=HOME='):
            assert token in text

    def test_does_not_add_a_discovery_range(self, text):
        # Upstream's discovery scope (D6) is not ported.
        assert 'ROS_AUTOMATIC_DISCOVERY_RANGE' not in text

    def test_env_var_skips_the_phase(self, tmp_path):
        r = run_setup(tmp_path, tmp_path / 'units', RACECAR_DASHBOARDS='0')
        assert r.returncode == 0 and 'skipping' in r.stdout

    def test_ships_the_seven_dashboards(self, text):
        listed = text.split('DASHBOARDS=(', 1)[1].split(')', 1)[0].split()
        assert listed == list(TABLE)

    @pytest.mark.parametrize('name', list(TABLE))
    def test_each_template_and_script_carry_the_shared_port(self, name):
        body = (DASH / name / f'neoracer-{name}.service.in').read_text()
        assert '/opt/ros/humble' in body and 'jazzy' not in body
        assert f'(port {TABLE[name]})' in body
        assert body.count('@DIR@') == 2
        assert f'exec python3 @DIR@/{STEMS[name]}.py' in body
        script = (DASH / name / f'{STEMS[name]}.py').read_text()
        assert re.search(rf'^PORT = {TABLE[name]}$', script, re.M)

    def test_installs_rendered_units_and_reruns_quietly(self, tmp_path):
        units = tmp_path / 'units'
        units.mkdir()
        r = run_setup(tmp_path, units)
        assert r.returncode == 0, r.stdout + r.stderr
        tree = tmp_path / 'dashboards'
        for name in TABLE:
            body = (units / f'neoracer-{name}.service').read_text()
            assert f'WorkingDirectory={tree / name}' in body
            assert f'exec python3 {tree / name}/{STEMS[name]}.py' in body
            assert f'racecar service start {name}' in r.stdout
        again = run_setup(tmp_path, units)
        assert again.stdout.count('already up to date') == len(TABLE)

    def test_an_old_checkout_moves_once_the_units_install(self, tmp_path):
        units = tmp_path / 'units'
        units.mkdir()
        tree = tree_copy(tmp_path)
        shipped = (tree / 'wallfollow' / 'wallfollow.yaml').read_text()
        old_checkout(
            tree,
            'wallfollow_dashboard',
            'wallfollow',
            shipped,
            shipped.replace('kp: 0.5 ', 'kp: 0.8 '),
        )
        r = run_setup(tmp_path, units)
        assert r.returncode == 0, r.stdout + r.stderr
        profile = yaml.safe_load(
            (tmp_path / 'profile' / 'dashboards' / 'wallfollow.yaml').read_text()
        )
        assert profile['kp'] == 0.8
        assert (tree / '.migrated' / 'wallfollow_dashboard').is_dir()
        assert 'no old checkouts' in run_setup(tmp_path, units).stdout

    def test_a_failed_install_leaves_the_old_checkouts_in_place(self, tmp_path):
        # An old unit still runs its checkout, so the checkout moves only once
        # every unit has installed.
        units = tmp_path / 'units'
        units.mkdir()
        tree = tree_copy(tmp_path)
        shipped = (tree / 'wallfollow' / 'wallfollow.yaml').read_text()
        old_checkout(tree, 'wallfollow_dashboard', 'wallfollow', shipped)
        sudo = tmp_path / 'sudo'
        sudo.write_text('#!/bin/bash\n[[ "$1" == install ]] && exit 1\nexec "$@"\n')
        sudo.chmod(0o755)
        r = run_setup(tmp_path, units, RACECAR_SUDO=str(sudo))
        assert r.returncode == 1
        assert 'left in place' in r.stdout and 'Dashboards installed' not in r.stdout
        assert (tree / 'wallfollow_dashboard' / 'wallfollow.yaml').read_text() == shipped
        assert not (tree / '.migrated').exists()

    def test_a_changed_unit_is_restarted_only_if_it_runs(self, tmp_path):
        units = tmp_path / 'units'
        units.mkdir()
        log = tmp_path / 'systemctl.log'
        stub = tmp_path / 'systemctl'
        stub.write_text(f'#!/bin/bash\necho "$*" >> "{log}"\n')
        stub.chmod(0o755)
        run_setup(tmp_path, units, systemctl=str(stub))
        calls = log.read_text().splitlines()
        assert 'daemon-reload' in calls
        assert sorted(c for c in calls if c.startswith('try-restart')) == sorted(
            f'try-restart neoracer-{name}.service' for name in TABLE
        )
        log.unlink()
        run_setup(tmp_path, units, systemctl=str(stub))
        assert not log.exists()  # nothing changed, so nothing was called

    def test_an_old_dashboard_is_disabled_before_its_checkout_moves(self, tmp_path):
        # The old setup enabled each dashboard at boot, and the new unit keeps
        # that enable state, so setup disables it once, while its old
        # checkout still names it. The stub notes whether the checkout is
        # still there at each call.
        units = tmp_path / 'units'
        units.mkdir()
        tree = tree_copy(tmp_path)
        old = {'wallfollow': 'wallfollow_dashboard', 'webteleop': 'teleop_dashboard'}
        for name, repo in old.items():
            stem = STEMS[name]
            old_checkout(tree, repo, stem, (tree / name / f'{stem}.yaml').read_text())
        log = tmp_path / 'systemctl.log'
        stub = tmp_path / 'systemctl'
        stub.write_text(
            '#!/bin/bash\n'
            f'[[ -d "{tree}/wallfollow_dashboard" ]] && here=present || here=moved\n'
            f'echo "$* $here" >> "{log}"\n'
        )
        stub.chmod(0o755)
        r = run_setup(tmp_path, units, systemctl=str(stub))
        assert r.returncode == 0, r.stdout + r.stderr
        calls = log.read_text().splitlines()
        assert [c for c in calls if c.startswith('disable')] == [
            'disable neoracer-wallfollow.service present',
            'disable neoracer-webteleop.service present',
        ]
        assert not any(c.startswith('stop') for c in calls)
        assert r.stdout.count('disabled at boot (the old setup enabled it)') == 2
        assert (tree / '.migrated' / 'teleop_dashboard').is_dir()
        log.unlink()
        again = run_setup(tmp_path, units, systemctl=str(stub))
        assert again.returncode == 0 and not log.exists()

    def test_a_failed_disable_leaves_the_old_checkouts_in_place(self, tmp_path):
        # The next run finds the checkout and disables the unit again.
        units = tmp_path / 'units'
        units.mkdir()
        tree = tree_copy(tmp_path)
        shipped = (tree / 'pursuit' / 'pursuit.yaml').read_text()
        old_checkout(tree, 'pursuit_dashboard', 'pursuit', shipped)
        stub = tmp_path / 'systemctl'
        stub.write_text('#!/bin/bash\n[[ "$1" == disable ]] && exit 1\nexit 0\n')
        stub.chmod(0o755)
        r = run_setup(tmp_path, units, systemctl=str(stub))
        assert r.returncode == 1
        assert 'not every old dashboard is disabled' in r.stdout
        assert (tree / 'pursuit_dashboard' / 'pursuit.yaml').read_text() == shipped
        assert not (tree / '.migrated').exists()

    def test_a_failed_checkout_listing_leaves_the_old_checkouts_in_place(self, tmp_path):
        # With no list of old checkouts, setup disables no unit and moves no
        # checkout. A stub python3 fails only the `checkouts.py --list` call.
        units = tmp_path / 'units'
        units.mkdir()
        tree = tree_copy(tmp_path)
        shipped = (tree / 'pursuit' / 'pursuit.yaml').read_text()
        old_checkout(tree, 'pursuit_dashboard', 'pursuit', shipped)
        stubs = tmp_path / 'bin'
        stubs.mkdir()
        (stubs / 'python3').write_text(
            '#!/bin/bash\n'
            '[[ "$2" == --list ]] && { echo "checkouts.py: listing failed" >&2; exit 1; }\n'
            f'exec {shlex.quote(sys.executable)} "$@"\n'
        )
        (stubs / 'python3').chmod(0o755)
        log = tmp_path / 'systemctl.log'
        stub = tmp_path / 'systemctl'
        stub.write_text(f'#!/bin/bash\necho "$*" >> "{log}"\n')
        stub.chmod(0o755)
        r = run_setup(tmp_path, units, systemctl=str(stub), PATH=f'{stubs}:{os.environ["PATH"]}')
        assert r.returncode == 1
        assert 'checkouts.py: listing failed' in r.stderr
        assert 'not every old dashboard is disabled' in r.stdout
        assert 'Incomplete: old checkouts' in r.stdout
        assert not any(c.startswith('disable') for c in log.read_text().splitlines())
        assert (tree / 'pursuit_dashboard' / 'pursuit.yaml').read_text() == shipped
        assert not (tree / '.migrated').exists()

    def test_a_failed_install_is_not_reported_as_success(self, text):
        # install_unit runs under `|| true`, which suppresses errexit for its
        # whole body, so the install has to be tested explicitly.
        assert 'elif $SUDO install -m 0644' in text
        assert 'install failed' in text

    def test_setup_services_installs_the_tree_instead_of_cloning(self):
        text = (SCRIPTS / 'setup_services.sh').read_text()
        assert 'git clone' not in text and 'github.com' not in text
        assert 'bash "$SCRIPT_DIR/setup_dashboards.sh"' in text


class TestTreeAndLint:
    def test_gitignore_tracks_the_dashboards_but_not_old_checkouts(self):
        lines = (ROOT / '.gitignore').read_text().splitlines()
        assert 'scripts/dashboards/' not in lines
        assert 'scripts/dashboards/*_dashboard/' in lines
        assert 'scripts/dashboards/.migrated/' in lines
        assert 'scripts/dashboards/*/*.local.yaml' in lines

    def test_the_linters_skip_only_old_checkouts(self):
        flake8 = (ROOT / 'test' / 'test_flake8.py').read_text()
        pep257 = (ROOT / 'test' / 'test_pep257.py').read_text()
        assert ',dashboards' not in flake8 and '.migrated' in flake8
        assert "'.migrated'" in pep257 and "'teleop'" in pep257
        pyproject = (ROOT / 'pyproject.toml').read_text()
        assert "'scripts/dashboards'," not in pyproject
        assert "'scripts/dashboards/*_dashboard'" in pyproject

    def test_no_dashboard_page_names_a_licensed_font_or_upstream(self):
        # Only the seven tracked folders. A car that has not run the install yet
        # still holds its old checkouts, and their pages name the licensed fonts.
        pages = [p for name in TABLE for p in (DASH / name).glob('*.html')]
        assert len(pages) == len(TABLE)
        for page in pages:
            text = page.read_text()
            for word in ('Kernel', 'Barlow', 'Akzidenz', 'RACECAR Neo', '--rcn-', 'override.js'):
                assert word not in text, f'{page.name}: {word}'
            assert '/ui/neoracer.css' in text and '/ui/lost.js' in text


def git(cwd, *args):
    subprocess.run(
        ['git', '-c', 'user.name=t', '-c', 'user.email=t@t', *args],
        cwd=cwd,
        check=True,
        capture_output=True,
    )


def old_checkout(dash, repo, name, text, edited=None, history=True):
    """Make an old checkout the way setup_services.sh cloned it, edited after the clone."""
    checkout = dash / repo
    checkout.mkdir(parents=True)
    (checkout / f'{name}.yaml').write_text(text)
    (checkout / f'{name}.py').write_text('# old code\n')
    if history:
        git(checkout, 'init', '-q')
        git(checkout, 'add', '.')
        git(checkout, 'commit', '-q', '-m', 'old')
    if edited is not None:
        (checkout / f'{name}.yaml').write_text(edited)
    return checkout


@pytest.fixture
def dash(tmp_path):
    """Return a dashboards folder holding the shipped yamls, as the driver tree does."""
    out = tmp_path / 'dashboards'
    for name, stem in STEMS.items():
        (out / name).mkdir(parents=True)
        shutil.copy(DASH / name / f'{stem}.yaml', out / name / f'{stem}.yaml')
    return out


@pytest.fixture
def data(data_root_dir):
    yield data_root_dir
    for sub in ('dashboards', 'datasets'):
        shutil.rmtree(data_root_dir / sub, ignore_errors=True)


class TestValues:
    def test_tuned_values_are_the_changed_numbers(self, co):
        shipped = {'kp': 0.5, 'kd': 4.0, 'camera_topic': '/camera/color', 'wall': {'speed': 0.0}}
        current = {'kp': 0.8, 'kd': 4.0, 'camera_topic': '/camera', 'wall': {'speed': 0.3}}
        assert co.tuned_values(current, shipped) == {('kp',): 0.8, ('wall', 'speed'): 0.3}

    def test_a_spelled_out_mode_compares_as_its_number(self, co):
        assert co.tuned_values({'mode': 'static'}, {'mode': 1}) == {('mode',): 0}
        assert co.tuned_values({'mode': 'dynamic'}, {'mode': 1}) == {}

    def test_changed_text_is_named(self, co):
        shipped = {'camera_topic': '/camera/color', 'car': {'label': 'a'}, 'kp': 1}
        current = {'camera_topic': '/camera', 'car': {'label': 'a'}, 'kp': 2}
        assert co.changed_text(current, shipped) == ['camera_topic']

    def test_a_small_number_reads_back_as_a_number(self, co):
        out, _ = co.apply_values('sigma_min: 0.05\n', {('sigma_min',): 1e-05})
        assert yaml.safe_load(out)['sigma_min'] == 1e-05

    def test_keys_deeper_than_one_block_are_left_alone(self, co):
        text = 'a:\n  b:\n    kp: 1\n  kp: 2\n'
        out, missing = co.apply_values(text, {('a', 'kp'): 5})
        assert out == 'a:\n  b:\n    kp: 1\n  kp: 5\n' and missing == []

    def test_apply_values_keeps_comments_and_blocks(self, co):
        text = (
            '# header\nspeed: 0.0   # top\nwall:\n  speed: 0.0  # wall speed\n'
            'car:\n  speed: 0.0\nname: x\n'
        )
        out, missing = co.apply_values(
            text, {('car', 'speed'): 0.4, ('speed',): 0.2, ('gone',): 1}
        )
        assert out == (
            '# header\nspeed: 0.2   # top\nwall:\n  speed: 0.0  # wall speed\n'
            'car:\n  speed: 0.4\nname: x\n'
        )
        assert missing == ['gone']


class TestMigrate:
    def test_tuned_values_move_to_the_profile_over_the_new_defaults(
        self, co, dash, data, car_profile_dir
    ):
        shipped = (dash / 'wallfollow' / 'wallfollow.yaml').read_text()
        old_checkout(
            dash,
            'wallfollow_dashboard',
            'wallfollow',
            shipped,
            shipped.replace('kp: 0.5 ', 'kp: 0.8 '),
        )
        lines, ok = co.migrate(dash, NOW, home=dash.parent)
        assert ok, lines
        profile = car_profile_dir / 'dashboards' / 'wallfollow.yaml'
        saved = yaml.safe_load(profile.read_text())
        assert saved['kp'] == 0.8
        assert saved['kd'] == yaml.safe_load(shipped)['kd']
        assert 'Raise this if the car wobbles' in profile.read_text()
        assert not (dash / 'wallfollow_dashboard').exists()
        assert (dash / '.migrated' / 'wallfollow_dashboard' / 'wallfollow.py').is_file()
        assert co.migrate(dash, NOW, home=dash.parent) == ([], True)  # nothing left to move

    def test_nested_blocks_keep_their_own_keys(self, co, dash, data, car_profile_dir):
        shipped = (dash / 'smartfollow' / 'smartfollow.yaml').read_text()
        edited = shipped.replace('  speed_kp: 0.2 ', '  speed_kp: 0.6 ')  # the car block's
        assert edited != shipped
        old_checkout(dash, 'smartfollow_dashboard', 'smartfollow', shipped, edited)
        co.migrate(dash, NOW, home=dash.parent)
        saved = yaml.safe_load((car_profile_dir / 'dashboards' / 'smartfollow.yaml').read_text())
        assert saved['car']['speed_kp'] == 0.6
        assert saved['wall']['speed_kp'] == 0.0

    def test_a_checkout_without_history_compares_with_the_shipped_defaults(
        self, co, dash, data, car_profile_dir
    ):
        shipped = (dash / 'webteleop' / 'teleop.yaml').read_text()
        old_checkout(
            dash,
            'teleop_dashboard',
            'teleop',
            shipped.replace('angle: 1.0', 'angle: 0.7'),
            history=False,
        )
        lines, ok = co.migrate(dash, NOW, home=dash.parent)
        assert ok and any('no git history' in ln for ln in lines)
        saved = yaml.safe_load((car_profile_dir / 'dashboards' / 'teleop.yaml').read_text())
        assert saved['angle'] == 0.7

    def test_an_existing_profile_file_wins(self, co, dash, data, car_profile_dir):
        shipped = (dash / 'linefollow' / 'linefollow.yaml').read_text()
        old_checkout(
            dash,
            'linefollow_dashboard',
            'linefollow',
            shipped,
            shipped.replace('h_low: 0', 'h_low: 90'),
        )
        profile = car_profile_dir / 'dashboards' / 'linefollow.yaml'
        profile.parent.mkdir(parents=True)
        profile.write_text('h_low: 40\n')
        lines, _ = co.migrate(dash, NOW, home=dash.parent)
        assert profile.read_text() == 'h_low: 40\n'
        assert any('already exists' in ln for ln in lines)
        # The checkout's own values are still on the car, beside its code.
        moved = dash / '.migrated' / 'linefollow_dashboard' / 'linefollow.yaml'
        assert 'h_low: 90' in moved.read_text()

    def test_logs_and_the_eps_ledger_move_to_the_data_root(self, co, dash, data):
        shipped = (dash / 'eps' / 'eps.yaml').read_text()
        checkout = old_checkout(dash, 'eps_dashboard', 'eps', shipped)
        (checkout / 'ledger.jsonl').write_text('{"ep": 1}\n')
        (checkout / 'ledger_20261001_120000.jsonl').write_text('{"ep": 9}\n')
        (checkout / '.init_set').write_text('')
        wf = old_checkout(
            dash,
            'wallfollow_dashboard',
            'wallfollow',
            (dash / 'wallfollow' / 'wallfollow.yaml').read_text(),
        )
        (wf / 'logs').mkdir()
        (wf / 'logs' / 'Log1.csv').write_text('t,error\n')
        co.migrate(dash, NOW, home=dash.parent)
        eps = data / 'dashboards' / 'eps'
        assert (eps / 'ledger.jsonl').read_text() == '{"ep": 1}\n'
        assert (eps / 'ledger_20261001_120000.jsonl').is_file()
        assert (eps / '.init_set').is_file()
        assert (data / 'dashboards' / 'wallfollow' / 'Log1.csv').read_text() == 't,error\n'

    def test_a_log_already_in_the_data_root_is_not_overwritten(self, co, dash, data):
        wf = old_checkout(
            dash,
            'wallfollow_dashboard',
            'wallfollow',
            (dash / 'wallfollow' / 'wallfollow.yaml').read_text(),
        )
        (wf / 'logs').mkdir()
        (wf / 'logs' / 'Log1.csv').write_text('old\n')
        new = data / 'dashboards' / 'wallfollow'
        new.mkdir(parents=True)
        (new / 'Log1.csv').write_text('new\n')
        co.migrate(dash, NOW, home=dash.parent)
        assert (new / 'Log1.csv').read_text() == 'new\n'
        assert (dash / '.migrated' / 'wallfollow_dashboard' / 'logs' / 'Log1.csv').is_file()

    def test_the_camlabel_dataset_moves_to_the_data_root(self, co, dash, data, tmp_path):
        home = tmp_path / 'home'
        old = home / 'data' / 'camlabel'
        (old / 'images').mkdir(parents=True)
        (old / 'images' / 'img1.jpg').write_bytes(b'jpg')
        (old / 'classes.txt').write_text('cone\n')
        # camlabel makes its empty folders when it starts.
        (data / 'datasets' / 'camlabel' / 'unlabeled').mkdir(parents=True)
        lines, ok = co.migrate(dash, NOW, home=home)
        assert ok, lines
        new = data / 'datasets' / 'camlabel'
        assert (new / 'images' / 'img1.jpg').read_bytes() == b'jpg'
        assert (new / 'classes.txt').read_text() == 'cone\n'
        assert not old.exists()

    def test_two_datasets_are_never_merged(self, co, dash, data, tmp_path):
        home = tmp_path / 'home'
        (home / 'data' / 'camlabel').mkdir(parents=True)
        (home / 'data' / 'camlabel' / 'classes.txt').write_text('cone\n')
        (data / 'datasets' / 'camlabel').mkdir(parents=True)
        (data / 'datasets' / 'camlabel' / 'classes.txt').write_text('car\n')
        lines, _ = co.migrate(dash, NOW, home=home)
        assert any('merge them by hand' in ln for ln in lines)
        assert (home / 'data' / 'camlabel' / 'classes.txt').read_text() == 'cone\n'
        assert (data / 'datasets' / 'camlabel' / 'classes.txt').read_text() == 'car\n'

    def test_an_unreadable_yaml_is_reported_and_the_checkout_still_moves(self, co, dash, data):
        old_checkout(dash, 'pursuit_dashboard', 'pursuit', 'speed: [unclosed\n', history=False)
        lines, ok = co.migrate(dash, NOW, home=dash.parent)
        assert ok, lines
        assert any('cannot be read' in ln for ln in lines)
        moved = dash / '.migrated' / 'pursuit_dashboard' / 'pursuit.yaml'
        assert moved.read_text() == 'speed: [unclosed\n'

    def test_tuning_committed_on_the_car_still_counts(
        self, co, dash, data, car_profile_dir, tmp_path
    ):
        # The old setup fast-forwarded the checkouts, so a car that committed
        # its yaml kept a local commit on top of the upstream branch.
        shipped = (dash / 'eps' / 'eps.yaml').read_text()
        origin = tmp_path / 'origin'
        origin.mkdir()
        (origin / 'eps.yaml').write_text(shipped)
        git(origin, 'init', '-q')
        git(origin, 'add', '.')
        git(origin, 'commit', '-q', '-m', 'upstream')
        git(dash, 'clone', '-q', str(origin), 'eps_dashboard')
        checkout = dash / 'eps_dashboard'
        (checkout / 'eps.yaml').write_text(shipped.replace('  kp: 2.0', '  kp: 2.6'))
        git(checkout, 'commit', '-q', '-am', 'tuned on the car')
        lines, ok = co.migrate(dash, NOW, home=dash.parent)
        assert ok, lines
        saved = yaml.safe_load((car_profile_dir / 'dashboards' / 'eps.yaml').read_text())
        assert saved['nominal']['kp'] == 2.6

    def test_text_values_are_reported_not_copied(self, co, dash, data, car_profile_dir):
        shipped = (dash / 'wallfollow' / 'wallfollow.yaml').read_text()
        edited = shipped.replace('mode: 1 ', 'mode: static ') + 'camera_topic: /camera\n'
        old_checkout(dash, 'wallfollow_dashboard', 'wallfollow', shipped, edited)
        lines, _ = co.migrate(dash, NOW, home=dash.parent)
        saved = yaml.safe_load((car_profile_dir / 'dashboards' / 'wallfollow.yaml').read_text())
        assert saved['mode'] == 0
        assert any('text values not copied' in ln and 'camera_topic' in ln for ln in lines)

    def test_the_eps_ledger_stays_when_the_profile_kept_its_own_tuning(
        self, co, dash, data, car_profile_dir
    ):
        checkout = old_checkout(
            dash, 'eps_dashboard', 'eps', (dash / 'eps' / 'eps.yaml').read_text()
        )
        (checkout / 'ledger.jsonl').write_text('{"ep": 1}\n')
        profile = car_profile_dir / 'dashboards' / 'eps.yaml'
        profile.parent.mkdir(parents=True)
        profile.write_text('nominal:\n  kp: 9.0\n')
        lines, _ = co.migrate(dash, NOW, home=dash.parent)
        assert not (data / 'dashboards' / 'eps' / 'ledger.jsonl').exists()
        assert (dash / '.migrated' / 'eps_dashboard' / 'ledger.jsonl').is_file()
        assert any('stays with the old checkout' in ln for ln in lines)

    def test_old_checkouts_lists_each_dashboard_with_one(self, co, dash):
        assert co.old_checkouts(dash) == []
        (dash / 'teleop_dashboard').mkdir()
        (dash / 'camlabel_dashboard').mkdir()
        assert co.old_checkouts(dash) == ['camlabel', 'webteleop']

    def test_a_second_set_aside_does_not_overwrite_the_first(self, co, dash, data):
        (dash / '.migrated' / 'eps_dashboard').mkdir(parents=True)
        old_checkout(dash, 'eps_dashboard', 'eps', (dash / 'eps' / 'eps.yaml').read_text())
        co.migrate(dash, NOW, home=dash.parent)
        assert (dash / '.migrated' / 'eps_dashboard-20261117-120000' / 'eps.py').is_file()
