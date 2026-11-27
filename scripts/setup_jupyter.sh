#!/bin/bash
# JupyterLab - install + workspace setup for racecar student notebooks.
# Idempotent: re-runs skip work already done.
set -eo pipefail

USER_NAME="${SUDO_USER:-$USER}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"
JUPYTER_WS="$USER_HOME/jupyter_ws"

# Per-user install. --break-system-packages only exists in pip >= 23.0.1 and
# is only needed on PEP 668 images (Pi-OS bookworm/noble, where this script
# came from); Jammy's pip 22.0.2 errors on it. Probe instead of assuming.
PIP_FLAGS=(--user)
# The help text is read whole. Under pipefail, a pipe into `grep -q` can
# fail pip on a closed pipe although the flag was found.
if [[ "$(pip3 install --help 2>/dev/null)" == *--break-system-packages* ]]; then
    PIP_FLAGS+=(--break-system-packages)
fi

# jupyterlab pulls jupyter_server, ipykernel, tornado, et al; ~100 MB on disk.
if ! command -v "$USER_HOME/.local/bin/jupyter" >/dev/null 2>&1; then
    pip3 install "${PIP_FLAGS[@]}" jupyterlab
fi

# jupyterlab's anyio dependency registers a pytest11 plugin that imports
# _pytest.scope, added in pytest 7. Jammy's apt pytest is 6.2.5, so once
# jupyterlab is installed every `colcon test` run dies during plugin loading,
# before collecting a single test. Put a 7.x in the user site to shadow it;
# 8.x is held back because ROS2 Humble's ament_cmake_pytest targets 6/7.
if ! python3 -c "
import sys
from importlib.metadata import version
sys.exit(0 if int(version('pytest').split('.')[0]) >= 7 else 1)
" >/dev/null 2>&1; then
    pip3 install "${PIP_FLAGS[@]}" "pytest>=7,<8"
fi

# Student-library runtime deps for the v2 racecar-neo library / labs:
#   - ipywidgets: live FPS / joystick / detection widgets in
#     labs/tests/test_async_core_real.ipynb. JupyterLab 4.x renders
#     ipywidgets >= 8 natively (no labextension install needed).
#   - pandas: backs telemetry_real.visualize() reading the recorded CSV.
#   - matplotlib-inline<0.2: matplotlib-inline 0.2.x calls
#     matplotlib.rcParams._get(...), which only exists in matplotlib >= 3.10.
#     Pi-OS bookworm/noble ship apt matplotlib 3.6.3, so on Py3.12 the
#     transitive 0.2.x release (pulled in by ipykernel) breaks plt.subplots()
#     inside Jupyter with AttributeError: 'RcParams' object has no attribute
#     '_get'. Pinning <0.2 here keeps the IPython inline backend usable until
#     someone upgrades apt matplotlib or rebases on a newer Python image.
#
# Not installed: nptyping. Earlier v0.0.8 drafts pinned nptyping<2 because
# the v1 library used the deprecated NDArray[(480, 640, 3), np.uint8] form
# (the v2 nptyping release replaced it with Shape["..."]). On Py3.12 both
# nptyping 1.x and 2.x are broken: 2.x raises InvalidArgumentsError at the
# class def, and 1.x triggers a runaway typing._type_repr recursion that
# adds ~30 s to a cold import. MITUavNeo/uav-neo-library hit the same wall
# and resolved it by dropping nptyping entirely - they ship a 2-line inline
# NDArray stub in every module that needs the syntax. The racecar-neo v2
# library v1.2.0 mirrors that pattern, so nptyping is not a dep here.
LIB_DEPS=(ipywidgets pandas 'matplotlib-inline<0.2')
MISSING_DEPS=()
for dep in "${LIB_DEPS[@]}"; do
    # Strip any version specifier (e.g. 'matplotlib-inline<0.2' -> 'matplotlib-inline')
    # for the import probe; pip handles the constraint on install.
    mod="${dep%%[<>=!~ ]*}"
    if ! sudo -u "$USER_NAME" python3 -c "
import importlib, sys
try:
    m = importlib.import_module('${mod//-/_}')
except Exception:
    sys.exit(1)
# matplotlib-inline 0.2.x calls matplotlib.rcParams._get(...), which only
# exists in matplotlib >= 3.10. Pi-OS bookworm/noble ship apt matplotlib
# 3.6.3, so plt.subplots() inside Jupyter raises AttributeError. Treat
# 0.2.x as 'missing' here so pip downgrades to 0.1.x.
if '${mod}' == 'matplotlib-inline':
    parts = m.__version__.split('.')
    major, minor = int(parts[0]), int(parts[1]) if len(parts) > 1 else 0
    sys.exit(0 if (major, minor) < (0, 2) else 1)
" >/dev/null 2>&1; then
        MISSING_DEPS+=("$dep")
    fi
done
if [ ${#MISSING_DEPS[@]} -gt 0 ]; then
    pip3 install "${PIP_FLAGS[@]}" "${MISSING_DEPS[@]}"
fi

# Notebook root. Empty unless we ship example notebooks later.
if [ ! -d "$JUPYTER_WS" ]; then
    mkdir -p "$JUPYTER_WS"
    cat > "$JUPYTER_WS/README.md" <<'EOF'
# Neoracer Jupyter Workspace

JupyterLab serves this directory at http://<robot>:8888 when
neoracer-jupyter.service is running.

Start a notebook and `import rclpy` - the systemd unit pre-sets
PYTHONPATH/AMENT_PREFIX_PATH/LD_LIBRARY_PATH so ROS2 messages and the
racecar driver are importable.
EOF
    echo "Created $JUPYTER_WS"
fi

# The data root (neoracer_ros2_driver/data_root.py) and its saved maps, linked
# into the JupyterLab file browser, which is rooted at ~/jupyter_ws. The links
# are made here because neoracer-installer removes ~/jupyter_ws.
if [ -r /run/racecar/data_root ]; then
    DATA_ROOT="$(cat /run/racecar/data_root)"
else
    DATA_ROOT="$USER_HOME/neoracer_data"
fi
MAP_ROOT="$DATA_ROOT/maps"
mkdir -p "$MAP_ROOT"
for link in data maps; do
    target="$DATA_ROOT"
    [ "$link" = maps ] && target="$MAP_ROOT"
    if [ -L "$JUPYTER_WS/$link" ] || [ ! -e "$JUPYTER_WS/$link" ]; then
        ln -sfn "$target" "$JUPYTER_WS/$link"
        echo "Linked $JUPYTER_WS/$link -> $target"
    fi
done

# ---------------------------------------------------------------------------
# Student library + labs.
# ---------------------------------------------------------------------------
# Two Neobotics repos supply the student side. The racecar-neo library fork is
# pinned to the tag made for this driver release, so a car set up later gets
# the library this driver was tested with. The fork carries the NeoRacer
# changes, such as the FlySky auto-start (go() enters the user program with no
# START button, because the FlySky handset has none), the JPEG camera decode
# and the 1,440-point lidar. Override LIB_REPO/LIB_PIN to test an unreleased
# library revision on a car. LIB_PIN names a tag or a branch; `git clone
# --branch` does not take a commit hash. LIB_REPO can be a local clone.
# The labs come from neoracer-labs. Assemble both into
# ~/jupyter_ws/neoracer-os/{library,labs} and point the student .pth at the
# library so notebooks can `import racecar_core`.
# Every run replaces the library, so it matches this driver. The labs folder
# is the student's workspace. The first run copies the labs in. A later run
# leaves the labs as they are and does not fetch them, unless
# RACECAR_LABS_REFRESH is 1. `racecar update --labs` sets it to 1, and
# `racecar update` sets it to 0. With it at 1, the run copies the labs folder
# to labs-backup-YYYYMMDD-HHMMSS beside it, then overlays the shipped files.
# Files a student added stay in place. To refresh the labs with no git reset,
# run `RACECAR_LABS_REFRESH=1 racecar setup all`. A fetch failure stops setup
# and leaves the library and the labs as they were.
LIB_REPO="${LIB_REPO:-https://github.com/Neobotics-Foundation-Inc/racecar-neo-library}"
LIB_PIN="${LIB_PIN:-v0.9.0-neobotics}"
LABS_REPO="https://github.com/Neobotics-Foundation-Inc/neoracer-labs"
NEO_OS="$JUPYTER_WS/neoracer-os"
LABS="$NEO_OS/labs"

FETCH_LABS=1
if [ -d "$LABS" ] && [ "${RACECAR_LABS_REFRESH:-0}" != 1 ]; then
    FETCH_LABS=0
fi

echo "Fetching student library ($LIB_PIN)..."
TMP="$(mktemp -d)"
if ! git clone --depth 1 --branch "$LIB_PIN" "$LIB_REPO" "$TMP/lib"; then
    rm -rf "$TMP"
    echo "Could not fetch the student library at $LIB_PIN from $LIB_REPO." >&2
    echo "Check the network. If that tag or branch does not exist, set LIB_PIN" >&2
    echo "(and LIB_REPO) to one that does." >&2
    echo "The library and the labs on the car are unchanged." >&2
    exit 1
fi
if [ "$FETCH_LABS" = 1 ]; then
    echo "Fetching labs..."
    if ! git clone --depth 1 "$LABS_REPO" "$TMP/labs"; then
        rm -rf "$TMP"
        echo "Could not fetch the labs from $LABS_REPO. Check the network." >&2
        echo "The library and the labs on the car are unchanged." >&2
        exit 1
    fi
fi
mkdir -p "$NEO_OS"
# A refresh copies the labs folder before anything changes. mkdir fails on a
# folder that exists, so a backup never lands inside an older one.
if [ "$FETCH_LABS" = 1 ] && [ -d "$LABS" ]; then
    BACKUP="$NEO_OS/labs-backup-$(date +%Y%m%d-%H%M%S)"
    mkdir "$BACKUP"
    cp -a "$LABS/." "$BACKUP/"
    echo "Labs backed up to $BACKUP"
fi
# The library is replaced wholesale. The labs are copied only on a first
# install or a refresh. The copy overlays the shipped files and deletes
# nothing a student saved.
rm -rf "$NEO_OS/library"
cp -r "$TMP/lib/library" "$NEO_OS/library"
if [ "$FETCH_LABS" = 1 ]; then
    rm -rf "$TMP/labs/.git"
    mkdir -p "$LABS"
    cp -r "$TMP/labs/." "$LABS/"
    echo "Labs ready: $LABS"
else
    echo "Labs left unchanged. Run 'racecar update --labs' to refresh them."
fi
rm -rf "$TMP"

# Select this library by writing the same racecar_student.pth that
# `racecar library --select neoracer-os` manages, so a fresh shell can
# `import racecar_core` with no extra step. A student who selected another
# library keeps that choice. The .pth is kept when its first line names a
# folder other than this library and that folder holds racecar_core.py.
# `racecar library --select` and `--list` apply the same test to a folder.
SITE="$(python3 -c 'import site; print(site.getusersitepackages())')"
PTH="$SITE/racecar_student.pth"
SELECTED=""
if [ -f "$PTH" ]; then
    SELECTED="$(head -n 1 "$PTH")"
fi
if [ -n "$SELECTED" ] && [ "$SELECTED" != "$NEO_OS/library" ] \
    && [ -f "$SELECTED/racecar_core.py" ]; then
    echo "Library selection kept: $SELECTED"
    echo "Student library ready: $NEO_OS/library"
    echo "Run 'racecar library --select neoracer-os' to use it."
else
    mkdir -p "$SITE"
    echo "$NEO_OS/library" > "$PTH"
    echo "Student library ready: $NEO_OS/library (selected via $PTH)"
fi
