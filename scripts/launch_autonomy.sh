#!/bin/bash
# Launch wrapper for autonomy.launch.py, run by neoracer-autonomy.service.
#
# The mode comes from ~/.config/racecar/autonomy.conf, which systemd loads
# (EnvironmentFile); autonomy_args.py turns it into launch arguments and
# resumes a mapping run from its autosave after a crash or teleop restart.
# Output goes to autonomy_<time>/autonomy.log in the current teleop log
# directory (~/logs/latest) and to the journal; node logs go beside it.
#
# Usage:
#   systemctl start neoracer-autonomy   (calls this script)

set -eo pipefail

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_ROOT="$HOME/logs/latest"
[[ -d "$LOG_ROOT" ]] || LOG_ROOT="$HOME/logs"
LOG_DIR="$LOG_ROOT/autonomy_$TIMESTAMP"
mkdir -p "$LOG_DIR"
# tee ignores the stop signal systemd sends the whole group, so it keeps reading
# until ros2 launch closes its output; dying first would end the launch with a
# broken pipe (exit 120) and mark a clean stop as failed.
exec &> >(trap '' INT TERM; exec tee -a "$LOG_DIR/autonomy.log")

echo "=== Neoracer Autonomy: $(date) ==="
echo "Log directory: $LOG_DIR"

export ROS_LOG_DIR="$LOG_DIR"
export ROS_HOME="$LOG_DIR"

# shellcheck source=/opt/ros/humble/setup.bash
source /opt/ros/humble/setup.bash
if [ -f "$HOME/ros2_ws/install/setup.bash" ]; then
    # shellcheck source=/home/racecar/ros2_ws/install/setup.bash
    source "$HOME/ros2_ws/install/setup.bash"
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mapfile -t ARGS < <(python3 "$SCRIPT_DIR/autonomy_args.py")
if [[ ${#ARGS[@]} -eq 0 ]]; then
    echo "No launch arguments; see the message above." >&2
    exit 2
fi
echo "Arguments: ${ARGS[*]}"

# `exec` so systemd tracks the ros2 launch PID, not this shell.
exec ros2 launch neoracer_ros2_driver autonomy.launch.py "${ARGS[@]}"
