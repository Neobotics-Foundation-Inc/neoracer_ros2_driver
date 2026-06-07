#!/bin/bash
# Add the invoking user to hardware groups, source ROS2 in .bashrc, and install
# convenience aliases.
set -eo pipefail

USER_NAME="${SUDO_USER:-$USER}"
USER_HOME="$(getent passwd "$USER_NAME" | cut -d: -f6)"

# Groups: dialout (the ESP32 + LED-matrix USB-serial ports) and video (the USB
# camera). All Neoracer peripherals are USB, so no i2c/spi/gpio groups needed.
# Skip groups that don't exist on this OS image.
for grp in dialout video; do
    if ! getent group "$grp" >/dev/null 2>&1; then
        continue
    fi
    if id -nG "$USER_NAME" | grep -qw "$grp"; then
        echo "  $USER_NAME already in $grp"
    else
        sudo usermod -aG "$grp" "$USER_NAME"
        echo "  added $USER_NAME to $grp"
    fi
done

BASHRC="$USER_HOME/.bashrc"

# Block 1: ROS2 + workspace overlay sourcing.
SOURCE_MARKER="# Neoracer - ROS2 + workspace overlay"
if grep -qF "$SOURCE_MARKER" "$BASHRC" 2>/dev/null; then
    echo "  $BASHRC already sources ROS2"
else
    cat >> "$BASHRC" <<EOF

$SOURCE_MARKER
source /opt/ros/humble/setup.bash
[ -f "\$HOME/ros2_ws/install/setup.bash" ] && source "\$HOME/ros2_ws/install/setup.bash"
EOF
    echo "  added ROS2 sourcing to $BASHRC"
fi

# Block 2: source the `racecar` shell tool.
TOOL_MARKER="# Neoracer - shell tool"
if grep -qF "$TOOL_MARKER" "$BASHRC" 2>/dev/null; then
    echo "  $BASHRC already sources racecar-tool.sh"
else
    cat >> "$BASHRC" <<'EOF'

# Neoracer - shell tool
[ -f "$HOME/ros2_ws/src/neoracer_ros2_driver/scripts/racecar-tool.sh" ] && \
    source "$HOME/ros2_ws/src/neoracer_ros2_driver/scripts/racecar-tool.sh"
EOF
    echo "  added racecar-tool source line to $BASHRC"
fi

# Block 3: clean up the legacy aliases (anyone who ran an earlier setup_user_env
# still has them; the new `racecar` function replaces them).
LEGACY_ALIAS_MARKER="# Neoracer - aliases"
if grep -qF "$LEGACY_ALIAS_MARKER" "$BASHRC" 2>/dev/null; then
    # Delete the 6 lines starting at the marker (5 aliases + the marker line).
    sed -i "/^${LEGACY_ALIAS_MARKER}$/,+5d" "$BASHRC"
    echo "  removed legacy racecar-* aliases from $BASHRC"
fi
