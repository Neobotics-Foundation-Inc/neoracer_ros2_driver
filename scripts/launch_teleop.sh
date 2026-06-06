#!/bin/bash
# Launch wrapper for teleop.launch.py with centralized logging.
#
# Creates a timestamped log directory under ~/logs/, updates the ~/logs/latest
# symlink atomically, sweeps FastRTPS shared-memory orphans, redirects stdout/
# stderr to both a plain-text log and systemd journald, and execs the full
# stack so systemd tracks the ros2 launch PID directly.
#
# Usage:
#   ./scripts/launch_teleop.sh [extra launch args...]
#   systemctl start neoracer-teleop   (calls this script)

set -eo pipefail

# ---------------------------------------------------------------------------
# Log directory setup
# ---------------------------------------------------------------------------
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$HOME/logs/$TIMESTAMP"
mkdir -p "$LOG_DIR"

# Atomic symlink update - create the link in tmp then rename onto target.
ln -sfn "$LOG_DIR" "$HOME/logs/latest"

echo "=== Neoracer Teleop - $(date) ==="
echo "Log directory: $LOG_DIR"

# ---------------------------------------------------------------------------
# FastRTPS SHM cleanup
# ---------------------------------------------------------------------------
# A 0-byte /dev/shm/fastrtps_port<N> segment left by a killed process causes
# any new rclpy participant that hashes to that port to spin forever in
# _rclpy.Node() - looks like a Jupyter cell hang. Sweep orphans before launch.
for f in /dev/shm/fastrtps_port*; do
    [ -e "$f" ] || continue
    case "$f" in *_el) continue ;; esac
    if [ ! -s "$f" ]; then
        base=$(basename "$f")
        rm -f "$f" "/dev/shm/${base}_el" "/dev/shm/sem.${base}_mutex"
        echo "Removed orphan SHM: $base"
    fi
done
for el in /dev/shm/fastrtps_port*_el; do
    [ -e "$el" ] || continue
    data="${el%_el}"
    if [ ! -e "$data" ]; then
        base=$(basename "$data")
        rm -f "$el" "/dev/shm/sem.${base}_mutex"
        echo "Removed orphan SHM lock: $(basename "$el")"
    fi
done

# ROS2's internal logs (rosout, launch.log) land in the same dir as our tee.
export ROS_LOG_DIR="$LOG_DIR"
export ROS_HOME="$LOG_DIR"

# ---------------------------------------------------------------------------
# Source ROS2 + workspace overlay
# ---------------------------------------------------------------------------
# shellcheck source=/opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    # shellcheck source=/home/racecar/ros2_ws/install/setup.bash
    source "$HOME/ros2_ws/install/setup.bash"
fi

# ---------------------------------------------------------------------------
# Mirror stdout/stderr to teleop.log AND console (journald when run via systemd).
# ---------------------------------------------------------------------------
exec &> >(tee -a "$LOG_DIR/teleop.log")

# ---------------------------------------------------------------------------
# Launch - `exec` so systemd tracks the ros2 launch PID, not this shell.
# ---------------------------------------------------------------------------
exec ros2 launch neoracer_ros2_driver teleop.launch.py "$@"
