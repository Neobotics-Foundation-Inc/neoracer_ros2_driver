#!/bin/bash
# Clone the Lakibeam lidar driver as a sibling package, then colcon build.
#
# The lidar driver (package lakibeam1) is a separate upstream project; it lives
# alongside this package in the workspace src/ rather than inside the repo.
# Override the source with LIDAR_REPO=... if you maintain a fork.
set -eo pipefail

WS_DIR="${WS_DIR:-$HOME/ros2_ws}"
SRC_DIR="$WS_DIR/src"
LIDAR_REPO="${LIDAR_REPO:-https://github.com/RichbeamTechnology/Lakibeam_ROS2_Driver.git}"

mkdir -p "$SRC_DIR"

if [ ! -d "$SRC_DIR/lakibeam1" ]; then
    echo "  cloning Lakibeam lidar driver into src/lakibeam1"
    git clone --depth=1 "$LIDAR_REPO" "$SRC_DIR/lakibeam1"
else
    echo "  src/lakibeam1 already present"
fi

# Lakibeam build dependency: libcurl (the sensor's HTTP config calls).
if ! dpkg -s libcurl4-openssl-dev >/dev/null 2>&1; then
    echo "  installing libcurl4-openssl-dev (lakibeam1 build dep)"
    sudo apt-get install -y -qq libcurl4-openssl-dev
fi

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd "$WS_DIR"
# --symlink-install lets YAML / launch / Python edits land without a rebuild.
colcon build --symlink-install

echo
echo "Workspace built. Packages:"
colcon list 2>/dev/null | awk '{print "  " $1}'
