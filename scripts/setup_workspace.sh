#!/bin/bash
# Build the workspace: neoracer driver + vendored lakibeam1 lidar driver, then
# reconcile the ~/osracer_ws vendor stack onto the same shared lakibeam1 source.
#
# The lidar driver (package lakibeam1) is vendored inside this repo and carries
# local fixes (see docs/lidar_scan_health.md); nothing else needs to be cloned.
# Earlier setups cloned upstream Lakibeam_ROS2_Driver into src/lakibeam1 as a
# sibling package; if that clone is still present it collides with the vendored
# copy (colcon aborts on the duplicate package name), so mask it.
set -eo pipefail

WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
SRC_DIR="$WS_DIR/src"

mkdir -p "$SRC_DIR"

# Legacy sibling clone from pre-vendoring setups (factory images ship one).
# lakibeam1 is vendored inside this repo now (with local fixes), so the
# sibling collides with it (duplicate package name) and colcon would abort.
# It's always a disposable git clone - remove it and its build artifacts.
if [ -d "$SRC_DIR/lakibeam1" ]; then
    echo "  removing superseded lidar clone $SRC_DIR/lakibeam1 (lakibeam1 is vendored in-repo)"
    rm -rf "$SRC_DIR/lakibeam1" "$WS_DIR/build/lakibeam1" "$WS_DIR/install/lakibeam1"
fi

# Lakibeam build dependency: libcurl (the sensor's HTTP config calls).
if ! dpkg -s libcurl4-openssl-dev >/dev/null 2>&1; then
    echo "  installing libcurl4-openssl-dev (lakibeam1 build dep)"
    sudo apt-get install -y -qq libcurl4-openssl-dev
fi

# The factory image's .bashrc sources the vendor stack in ~/osracer_ws, whose
# install space ships its own lakibeam1. With that on AMENT_PREFIX_PATH colcon
# treats our vendored lakibeam1 as an underlay override and aborts the build.
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
# has diverged from ours (it lacks the scan-health fixes in
# docs/lidar_scan_health.md). Two copies of the same package means the lidar you
# get depends on overlay order. Make osracer build OUR canonical source instead
# of its own: mask the vendor copy, symlink ours into its src, and rebuild just
# that package. Both workspaces still carry a lakibeam1 install (so `racecar ws`
# works in either mode) but from one source of truth. Only runs if the vendor
# workspace is present; its buildability is not a precondition for this repo, so
# a failure here warns and continues rather than aborting neoracer setup.
VENDOR_WS="$HOME/osracer_ws"
CANON_LAKIBEAM="$SRC_DIR/neoracer_ros2_driver/lakibeam1"
VENDOR_LAKIBEAM="$VENDOR_WS/src/osracer/osracer_dependency/Lakibeam_ROS2_Driver"
if [ -d "$VENDOR_WS/src" ] && [ -d "$CANON_LAKIBEAM" ]; then
    echo
    echo "Reconciling ~/osracer_ws onto the shared lakibeam1 source..."
    [ -d "$VENDOR_LAKIBEAM" ] && touch "$VENDOR_LAKIBEAM/COLCON_IGNORE"
    ln -sfn "$CANON_LAKIBEAM" "$VENDOR_WS/src/lakibeam1"
    rm -rf "$VENDOR_WS/build/lakibeam1" "$VENDOR_WS/install/lakibeam1"
    # Runtime deps the vendor bringup needs that the factory image omits.
    sudo apt-get install -y -qq ros-humble-tf-transformations python3-transforms3d || true
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
