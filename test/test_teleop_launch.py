"""Tests for launch/teleop.launch.py and scripts/launch_teleop.sh, read as text."""

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TELEOP = ROOT / 'launch' / 'teleop.launch.py'
WRAPPER = ROOT / 'scripts' / 'launch_teleop.sh'


def _literal(path, name):
    """Return the value of a module-level `name = <literal>` in `path`."""
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and any(
            isinstance(t, ast.Name) and t.id == name for t in node.targets
        ):
            return ast.literal_eval(node.value)
    raise AssertionError(f'{name} not found in {path.name}')


def _node_launch_args(name):
    """Return the config arguments a per-node launch file declares, as (argument, YAML)."""
    tree = ast.parse((ROOT / 'launch' / f'{name}.launch.py').read_text())
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', '') == 'single_node_launch':
            kw = {k.arg: k.value for k in node.keywords}
            pairs = [(ast.literal_eval(kw['arg_name']), ast.literal_eval(kw['default_yaml']))]
            if 'extra_yamls' in kw:
                pairs += [tuple(p) for p in ast.literal_eval(kw['extra_yamls'])]
            return tuple(pairs)
    raise AssertionError(f'{name}.launch.py does not call single_node_launch')


class TestTeleopLaunch:
    def test_control_config_args_match_each_node_launch(self):
        control = _literal(TELEOP, '_CONTROL')
        assert list(control) == ['controller', 'gamepad', 'mux', 'throttle']
        for name, pairs in control.items():
            assert tuple(tuple(p) for p in pairs) == _node_launch_args(name), name

    def test_subsystems_keep_their_defaults(self):
        assert _literal(TELEOP, '_SUBSYSTEMS') == {
            'frames': 'true',
            'lidar': 'true',
            'camera': 'true',
            'led_matrix': 'true',
            'inference': 'true',
        }

    def test_every_launch_argument_is_described(self):
        text = TELEOP.read_text()
        assert "description=f'Bring up {name} subsystem" in text
        assert "description=f'Path to {yaml}'" in text


class TestWrapper:
    def test_the_log_mirror_starts_before_anything_is_printed(self):
        text = WRAPPER.read_text()
        assert text.count('exec &> >(tee -a "$LOG_DIR/teleop.log")') == 1
        tee = text.index('exec &> >(tee')
        assert tee < text.index('echo "=== NeoRacer Teleop')
        assert tee < text.index('features.py" args')

    def test_latest_is_repointed_with_a_rename(self):
        text = WRAPPER.read_text()
        assert 'ln -sfn "$LOG_DIR" "$HOME/logs/latest.tmp"' in text
        assert 'mv -Tf "$HOME/logs/latest.tmp" "$HOME/logs/latest"' in text
        assert 'ln -sfn "$LOG_DIR" "$HOME/logs/latest"\n' not in text

    def test_a_real_latest_folder_is_moved_aside_before_the_rename(self):
        # mv -T cannot rename a link over a folder; under set -e teleop would not start.
        text = WRAPPER.read_text()
        guard = text.index('if [ -d "$HOME/logs/latest" ] && [ ! -L "$HOME/logs/latest" ]; then')
        assert guard < text.index('mv -Tf "$HOME/logs/latest.tmp" "$HOME/logs/latest"')
        assert 'mv "$HOME/logs/latest" "$HOME/logs/latest.$TIMESTAMP"' in text

    def test_the_feature_arguments_still_go_first(self):
        assert 'set -- "${FEATURE_ARGS[@]}" "$@"' in WRAPPER.read_text()

    def test_shm_orphans_are_swept(self):
        assert 'for f in /dev/shm/fastrtps_port*; do' in WRAPPER.read_text()

    def test_ros2_launch_replaces_the_shell(self):
        # systemd then tracks the ros2 launch PID, not this shell.
        last = WRAPPER.read_text().rstrip().splitlines()[-1]
        assert last == 'exec ros2 launch neoracer_ros2_driver teleop.launch.py "$@"'
