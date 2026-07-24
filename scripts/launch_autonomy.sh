#!/bin/bash
# Launch wrapper for autonomy.launch.py (autonomy base: TF + twist bridge).
#
# Same shape as launch_teleop.sh: timestamped log dir, output tee'd to a
# plain-text log and journald, exec so systemd tracks the ros2 launch PID.
# Differences: the osracer workspace is sourced UNDER the neoracer overlay
# (the autonomy launch includes osracer_description/slam/navigation), and
# the ~/logs/latest symlink is left alone - it belongs to teleop, and the
# dashboard reads it for the watchdog tail.
#
# Usage:
#   ./scripts/launch_autonomy.sh [extra launch args...]
#   systemctl start neoracer-autonomy   (calls this script)

set -eo pipefail

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$HOME/logs/autonomy_$TIMESTAMP"
mkdir -p "$LOG_DIR"

echo "=== Neoracer Autonomy - $(date) ==="
echo "Log directory: $LOG_DIR"

export ROS_LOG_DIR="$LOG_DIR"
export ROS_HOME="$LOG_DIR"

# ---------------------------------------------------------------------------
# Source ROS2, then the osracer underlay, then the neoracer overlay.
# Order matters: later sources win lookup conflicts, and the one shared
# lakibeam package must resolve from the neoracer side.
# ---------------------------------------------------------------------------
# shellcheck source=/opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash

if [ -f "$HOME/osracer_ws/install/setup.bash" ]; then
    # shellcheck source=/home/racecar/osracer_ws/install/setup.bash
    source "$HOME/osracer_ws/install/setup.bash"
else
    echo "osracer workspace not found at ~/osracer_ws - autonomy needs it" >&2
    exit 1
fi

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    # shellcheck source=/home/racecar/ros2_ws/install/setup.bash
    source "$HOME/ros2_ws/install/setup.bash"
fi

exec &> >(tee -a "$LOG_DIR/autonomy.log")

exec ros2 launch neoracer_ros2_driver autonomy.launch.py "$@"
