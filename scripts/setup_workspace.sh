#!/bin/bash
# Build the workspace: neoracer driver + the lakibeam1 lidar driver cloned as a
# sibling package, then reconcile the ~/osracer_ws vendor stack onto that same
# shared lakibeam1 source.
#
# The lidar driver (package lakibeam1) is third-party (RichbeamTechnology) and
# is NOT vendored into this repo. It is cloned from the Neobotics fork, which
# carries the scan-health fixes described in docs/lidar_scan_health.md (remote
# logger nullptr crash, bounded config-push readiness wait, all-inf watchdog).
# The clone is pinned to a tag so a car provisioned today builds the same
# driver as a car provisioned six months ago.
set -eo pipefail

WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
SRC_DIR="$WS_DIR/src"

mkdir -p "$SRC_DIR"

# ---------------------------------------------------------------------------
# lakibeam1 lidar driver (sibling package, pinned).
# ---------------------------------------------------------------------------
# Override LAKI_REPO/LAKI_PIN to test an unreleased driver revision.
LAKI_REPO="${LAKI_REPO:-https://github.com/Neobotics-Foundation-Inc/Lakibeam_ROS2_Driver.git}"
LAKI_PIN="${LAKI_PIN:-v1.0-neobotics}"
LAKI_DIR="$SRC_DIR/lakibeam1"

if [ ! -d "$LAKI_DIR/.git" ]; then
    # A non-git directory here is a stale unpacked copy from an older image.
    if [ -d "$LAKI_DIR" ]; then
        echo "  replacing non-git lidar copy at $LAKI_DIR"
        rm -rf "$LAKI_DIR" "$WS_DIR/build/lakibeam1" "$WS_DIR/install/lakibeam1"
    fi
    echo "  cloning lakibeam1 $LAKI_PIN from $LAKI_REPO"
    git clone -q "$LAKI_REPO" "$LAKI_DIR"
else
    # Factory images ship a clone of RichbeamTechnology upstream, which lacks
    # the scan-health fixes. Re-point it at the fork rather than leaving the
    # car on an unpatched driver.
    CUR_REMOTE=$(git -C "$LAKI_DIR" remote get-url origin 2>/dev/null || echo '')
    if [ "$CUR_REMOTE" != "$LAKI_REPO" ]; then
        echo "  re-pointing lakibeam1 origin: $CUR_REMOTE -> $LAKI_REPO"
        git -C "$LAKI_DIR" remote set-url origin "$LAKI_REPO"
        rm -rf "$WS_DIR/build/lakibeam1" "$WS_DIR/install/lakibeam1"
    fi
    git -C "$LAKI_DIR" fetch -q --tags origin
fi

# Detached checkout at the pin; local edits are discarded on purpose so every
# car converges on the pinned driver.
if ! git -C "$LAKI_DIR" checkout -q --force "$LAKI_PIN" 2>/dev/null; then
    echo "  ERROR: lakibeam1 pin '$LAKI_PIN' not found in $LAKI_REPO" >&2
    echo "         (has the fork been created and tagged?)" >&2
    exit 1
fi
echo "  lakibeam1 at $LAKI_PIN ($(git -C "$LAKI_DIR" rev-parse --short HEAD))"

# Cars provisioned before the driver was unvendored built lakibeam1 from
# src/neoracer_ros2_driver/lakibeam1. CMake records the source directory in
# CMakeCache.txt, so after the source moves to src/lakibeam1 the stale cache
# still points at the old path and the build dies with
#   CMake Error: The source directory "..." does not exist.
# Drop the build/install trees whenever the cached source dir is not the
# current one; colcon then reconfigures from scratch.
LAKI_CACHE="$WS_DIR/build/lakibeam1/CMakeCache.txt"
if [ -f "$LAKI_CACHE" ]; then
    CACHED_SRC=$(sed -n 's/^CMAKE_HOME_DIRECTORY:INTERNAL=//p' "$LAKI_CACHE" | head -1)
    if [ "$CACHED_SRC" != "$LAKI_DIR" ]; then
        echo "  clearing stale lakibeam1 build (was built from ${CACHED_SRC:-unknown})"
        rm -rf "$WS_DIR/build/lakibeam1" "$WS_DIR/install/lakibeam1"
    fi
fi

# Lakibeam build dependency: libcurl (the sensor's HTTP config calls).
if ! dpkg -s libcurl4-openssl-dev >/dev/null 2>&1; then
    echo "  installing libcurl4-openssl-dev (lakibeam1 build dep)"
    sudo apt-get install -y -qq libcurl4-openssl-dev
fi

# The factory image's .bashrc sources the vendor stack in ~/osracer_ws, whose
# install space ships its own lakibeam1. With that on AMENT_PREFIX_PATH colcon
# treats our lakibeam1 clone as an underlay override and aborts the build.
# Scrub osracer_ws entries inherited from the invoking shell here;
# setup_user_env.sh retires the .bashrc line for future shells.
for var in AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH PATH; do
    val="${!var:-}"
    [ -n "$val" ] || continue
    case "$val" in *osracer_ws*) ;; *) continue ;; esac
    val=$(printf '%s' "$val" | tr ':' '\n' | grep -v 'osracer_ws' | paste -sd: -) || true
    if [ -n "$val" ]; then
        export "$var=$val"
    else
        unset "$var"
    fi
    echo "  scrubbed vendor osracer_ws entries from \$$var"
done

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd "$WS_DIR"
# --symlink-install lets YAML / launch / Python edits land without a rebuild.
colcon build --symlink-install

# ---------------------------------------------------------------------------
# Single shared Lakibeam across both workspaces.
# ---------------------------------------------------------------------------
# The factory vendor stack in ~/osracer_ws ships its own copy of lakibeam1 that
# has diverged from the fork (it lacks the scan-health fixes in
# docs/lidar_scan_health.md). Two copies of the same package means the lidar you
# get depends on overlay order. Make osracer build the pinned fork instead of
# its own copy: mask the vendor copy, symlink the pinned clone into its src, and
# rebuild just that package. Both workspaces still carry a lakibeam1 install (so `racecar ws`
# works in either mode) but from one source of truth. Only runs if the vendor
# workspace is present; its buildability is not a precondition for this repo, so
# a failure here warns and continues rather than aborting neoracer setup.
VENDOR_WS="$HOME/osracer_ws"
CANON_LAKIBEAM="$LAKI_DIR"
VENDOR_LAKIBEAM="$VENDOR_WS/src/osracer/osracer_dependency/Lakibeam_ROS2_Driver"
if [ -d "$VENDOR_WS/src" ] && [ -d "$CANON_LAKIBEAM" ]; then
    echo
    echo "Reconciling ~/osracer_ws onto the shared lakibeam1 source..."
    [ -d "$VENDOR_LAKIBEAM" ] && touch "$VENDOR_LAKIBEAM/COLCON_IGNORE"
    ln -sfn "$CANON_LAKIBEAM" "$VENDOR_WS/src/lakibeam1"
    rm -rf "$VENDOR_WS/build/lakibeam1" "$VENDOR_WS/install/lakibeam1"
    # The vendor lidar launch defaults to the LakiBeam ethernet IP
    # (192.168.198.2), but the car connects the lidar over USB-C, where it is
    # 192.168.8.2 (matching neoracer's setup_networking). Without this the
    # shared driver's config push can't reach the sensor, so the lidar takes
    # ~2 min to start and runs on stale persisted config.
    OSR_LIDAR_LAUNCH="$VENDOR_WS/src/osracer/osracer_bringup/launch/lidar.launch.py"
    [ -f "$OSR_LIDAR_LAUNCH" ] && sed -i 's/192\.168\.198\.2/192.168.8.2/g' "$OSR_LIDAR_LAUNCH"
    # Runtime deps the vendor bringup needs that the factory image omits.
    sudo apt-get install -y -qq ros-humble-tf-transformations python3-transforms3d \
        ros-humble-robot-localization ros-humble-imu-complementary-filter || true
    # Build with base ROS only so our ros2_ws install is not an underlay that
    # trips colcon's package-override check.
    if env -i HOME="$HOME" USER="$USER" PATH="/usr/bin:/bin:/usr/sbin:/sbin" \
        bash -c 'source /opt/ros/humble/setup.bash \
                 && cd "$HOME/osracer_ws" \
                 && colcon build --symlink-install --packages-select lakibeam1'; then
        echo "  ~/osracer_ws now builds the shared lakibeam1"
    else
        echo "  WARN: osracer_ws lakibeam1 rebuild failed (vendor stack may be unbuilt); continuing"
    fi
fi

echo
echo "Workspace built. Packages:"
colcon list 2>/dev/null | awk '{print "  " $1}'
