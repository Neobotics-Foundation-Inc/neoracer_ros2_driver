#!/bin/bash
# Launch wrapper for teleop.launch.py with centralized logging.
#
# Creates a timestamped log directory under ~/logs/, repoints the ~/logs/latest
# symlink with a rename, mirrors stdout/stderr to teleop.log and the console
# (journald under systemd), sweeps FastRTPS shared-memory orphans, and execs
# the full stack so systemd tracks the ros2 launch PID directly.
#
# Usage:
#   ./scripts/launch_teleop.sh [extra launch args...]
#   systemctl start neoracer-teleop   (calls this script)

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ---------------------------------------------------------------------------
# Log directory setup
# ---------------------------------------------------------------------------
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$HOME/logs/$TIMESTAMP"
mkdir -p "$LOG_DIR"

# A real folder at ~/logs/latest cannot be renamed over, and set -e would then
# stop teleop on every start. The Jupyter unit makes one when it starts on a
# car before teleop has ever made the link. Move it aside.
if [ -d "$HOME/logs/latest" ] && [ ! -L "$HOME/logs/latest" ]; then
    mv "$HOME/logs/latest" "$HOME/logs/latest.$TIMESTAMP"
fi

# Build the link beside the target, then rename it over; readers never see
# ~/logs/latest missing.
ln -sfn "$LOG_DIR" "$HOME/logs/latest.tmp"
mv -Tf "$HOME/logs/latest.tmp" "$HOME/logs/latest"

exec &> >(tee -a "$LOG_DIR/teleop.log")

echo "=== NeoRacer Teleop: $(date) ==="
echo "Log directory: $LOG_DIR"

# ---------------------------------------------------------------------------
# FastRTPS SHM cleanup
# ---------------------------------------------------------------------------
# A 0-byte /dev/shm/fastrtps_port<N> segment left by a killed process makes
# any new rclpy participant that hashes to that port spin forever in
# _rclpy.Node(), which looks like a Jupyter cell hang. Same sweep as
# `racecar cleanup --force` (scripts/racecar-tool.sh).
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

# Features chosen for this boot (scripts/features.py): the camera, on by
# default, and inference, off by default. Arguments given to this script win.
# The feature arguments go first, so a ros2 launch option given to this script
# (such as -a) still comes after every launch argument.
mapfile -t FEATURE_ARGS < <(python3 "$SCRIPT_DIR/features.py" args "$@" || true)
set -- "${FEATURE_ARGS[@]}" "$@"

# Launch arguments, one per line, so the watchdog leaves subsystems disabled
# at launch (<name>_enable:=false) alone.
printf '%s\n' "$@" > "$LOG_DIR/launch_args"

# ROS2's internal logs (rosout, launch.log) land beside teleop.log.
export ROS_LOG_DIR="$LOG_DIR"
export ROS_HOME="$LOG_DIR"

# ---------------------------------------------------------------------------
# ROS2 and workspace overlay
# ---------------------------------------------------------------------------
# shellcheck source=/opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash

if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    # shellcheck source=/home/racecar/ros2_ws/install/setup.bash
    source "$HOME/ros2_ws/install/setup.bash"
fi

# Per-car files left in the driver folder are ignored; say so in the log.
python3 "$SCRIPT_DIR/neoracer_profile.py" check || true

# ---------------------------------------------------------------------------
# Launch; `exec` so systemd tracks the ros2 launch PID, not this shell.
# ---------------------------------------------------------------------------
exec ros2 launch neoracer_ros2_driver teleop.launch.py "$@"
