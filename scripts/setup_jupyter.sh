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
if pip3 install --help 2>/dev/null | grep -q -- '--break-system-packages'; then
    PIP_FLAGS+=(--break-system-packages)
fi

# jupyterlab pulls jupyter_server, ipykernel, tornado, et al; ~100 MB on disk.
if ! command -v "$USER_HOME/.local/bin/jupyter" >/dev/null 2>&1; then
    pip3 install "${PIP_FLAGS[@]}" jupyterlab
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

# ---------------------------------------------------------------------------
# Student library + labs.
# ---------------------------------------------------------------------------
# Two Neobotics repos supply the student side. The racecar-neo library fork is
# pinned to the neobotics-slam-nav branch, which carries the FlySky auto-start
# fix (go() enters user program mode without a START button - the FlySky RC has
# none, unlike the Xbox pad the upstream MIT library assumes) plus rc.slam and
# rc.nav. The labs come from neoracer-labs. Assemble both into
# ~/jupyter_ws/neoracer-os/{library,labs} and point the student .pth at the
# library so notebooks can `import racecar_core`. Re-running refreshes both; a
# fetch failure aborts setup rather than leaving a half-installed library.
LIB_REPO="https://github.com/Neobotics-Foundation-Inc/racecar-neo-library"
LIB_BRANCH="main"
LABS_REPO="https://github.com/Neobotics-Foundation-Inc/neoracer-labs"
NEO_OS="$JUPYTER_WS/neoracer-os"

echo "Fetching student library ($LIB_BRANCH) + labs..."
TMP="$(mktemp -d)"
git clone --depth 1 --branch "$LIB_BRANCH" "$LIB_REPO" "$TMP/lib"
git clone --depth 1 "$LABS_REPO" "$TMP/labs"
mkdir -p "$NEO_OS"
rm -rf "$NEO_OS/library" "$NEO_OS/labs"
cp -r "$TMP/lib/library" "$NEO_OS/library"
cp -r "$TMP/labs" "$NEO_OS/labs"
rm -rf "$NEO_OS/labs/.git"
rm -rf "$TMP"

# Select this library by writing the same racecar_student.pth that
# `racecar library --select neoracer-os` manages, so a fresh shell can
# `import racecar_core` with no extra step.
SITE="$(python3 -c 'import site; print(site.getusersitepackages())')"
mkdir -p "$SITE"
echo "$NEO_OS/library" > "$SITE/racecar_student.pth"
echo "Student library ready: $NEO_OS/library (selected via $SITE/racecar_student.pth)"
