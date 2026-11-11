"""
Where the car's data goes: the data root.

Everything the car produces (bags, maps, sessions, captures, datasets,
dashboard logs, health reports) goes under one folder, ~/racecar_data.
Settings never go here; they live in the car profile (car_profile.py).

The folder is deliberately not ~/data: neoracer-installer removes
~/jupyter_ws, ~/ros2_ws, ~/data and ~/logs on every run, and maps and
recordings have to survive a reinstall. The Jetson boots from its NVMe, so
there is no second drive to prefer over the home folder.

Programs find the root through, in order:

    RACECAR_DATA_DIR        set for services, login shells, and JupyterLab
    /run/racecar/data_root  written once per boot by neoracer-data.service
    resolve()               the default, worked out here

Layout under the root: bags/, images/, datasets/, maps/, slam_eval/,
dashboards/, health/.
"""

from __future__ import annotations

import os
from pathlib import Path
import tempfile

DATA_ENV = 'RACECAR_DATA_DIR'
RUN_FILE = Path('/run/racecar/data_root')
# The same, as KEY=value for the units' EnvironmentFile=.
RUN_ENV_FILE = Path('/run/racecar/environment')
# The default root's name under the home folder.
DEFAULT_NAME = 'racecar_data'
SUBDIRS = ('bags', 'images', 'datasets', 'maps', 'slam_eval', 'dashboards', 'health')

_resolved: tuple[Path, str] | None = None


def fallback() -> Path:
    """The default root: ~/racecar_data."""
    return Path.home() / DEFAULT_NAME


def writable(path: Path) -> bool:
    """Check by creating and removing a file, which also catches a read-only mount."""
    try:
        with tempfile.NamedTemporaryFile(dir=path, prefix='.racecar-write-test-'):
            return True
    except OSError:
        return False


def resolve() -> tuple[Path, str]:
    """
    Return (root, warning) by the data root rule; the warning is '' here.

    Does not consult RACECAR_DATA_DIR or the run file. The tuple is kept so
    callers written for a root that can move between drives work unchanged.
    """
    return fallback(), ''


def current() -> tuple[Path, str]:
    """Return (root, warning) for this process, worked out once."""
    global _resolved
    if _resolved is None:
        env = os.environ.get(DATA_ENV)
        if env:
            _resolved = (Path(env).expanduser(), '')
        else:
            try:
                text = RUN_FILE.read_text().strip()
            except OSError:
                text = ''
            _resolved = (Path(text), '') if text else resolve()
    return _resolved


def data_dir(sub: str | None = None) -> Path:
    """Return the data root, or the folder `sub` under it, created if absent."""
    root = current()[0]
    path = root / sub if sub else root
    path.mkdir(parents=True, exist_ok=True)
    return path


def session_problem() -> str:
    """
    Return why a new session cannot write to the data root, or ''.

    A root that stops being writable (full disk, wrong owner after a reinstall)
    makes a session refuse to start, rather than switch folders partway through.
    """
    root = current()[0]
    if root.is_dir() and writable(root):
        return ''
    return (
        f'{root} cannot be written. Check that it exists and belongs to this user, '
        'or reboot so neoracer-data.service makes it again.'
    )


def reset() -> None:
    """Forget the root worked out for this process (tests, long-running tools)."""
    global _resolved
    _resolved = None


def main() -> int:
    """`python3 -m neoracer_ros2_driver.data_root write`: the boot oneshot's work."""
    import sys

    if sys.argv[1:] != ['write']:
        print(current()[0])
        return 0
    root, warning = resolve()
    root.mkdir(parents=True, exist_ok=True)
    RUN_FILE.parent.mkdir(parents=True, exist_ok=True)
    RUN_FILE.write_text(f'{root}\n')
    RUN_ENV_FILE.write_text(f'{DATA_ENV}={root}\n')
    print(warning or f'Data root: {root}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
