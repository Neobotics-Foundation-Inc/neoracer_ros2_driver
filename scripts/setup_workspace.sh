#!/bin/bash
# Build the workspace: neoracer driver + vendored lakibeam1 lidar driver.
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
# We don't delete a directory we didn't create - tell the user and stop.
if [ -d "$SRC_DIR/lakibeam1" ]; then
    echo "ERROR: legacy lidar clone found at $SRC_DIR/lakibeam1." >&2
    echo "lakibeam1 is now vendored inside neoracer_ros2_driver and this copy" >&2
    echo "collides with it. Remove it and its build artifacts, then re-run setup:" >&2
    echo "  rm -rf $SRC_DIR/lakibeam1 $WS_DIR/build/lakibeam1 $WS_DIR/install/lakibeam1" >&2
    exit 1
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

echo
echo "Workspace built. Packages:"
colcon list 2>/dev/null | awk '{print "  " $1}'
