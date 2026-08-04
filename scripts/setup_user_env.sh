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

# Block 1: default-workspace reset, append-only. The stock image's .bashrc
# sources the vendor stack in ~/osracer_ws (which ships its own lakibeam1);
# we deliberately do NOT edit or comment any existing line - we can't know
# what's in a factory .bashrc, and commenting lines can orphan an if/fi or
# kill unrelated exports on compound lines. Instead this block is appended
# at the end, runs after whatever came before, and resets the ROS env to
# base Humble + the neoracer overlay. `racecar ws osracer` flips a shell to
# the vendor stack; deleting this block restores factory behavior exactly.
SOURCE_MARKER="# Neoracer - default workspace"
if grep -qF "$SOURCE_MARKER" "$BASHRC" 2>/dev/null; then
    echo "  $BASHRC already has the default-workspace block"
else
    cat >> "$BASHRC" <<'EOF'

# Neoracer - default workspace
# Appended by setup_user_env.sh; keep after any workspace sourcing above.
# Resets the ROS environment to the neoracer overlay no matter what earlier
# lines (e.g. the factory ~/osracer_ws sourcing) set up. Switch a shell with
# `racecar ws osracer` / `racecar ws neoracer`. Deleting this block restores
# the previous default; no other line in this file was modified by setup.
for _rc_var in PATH PYTHONPATH LD_LIBRARY_PATH; do
    _rc_val="${!_rc_var}"
    [ -n "$_rc_val" ] || continue
    case "$_rc_val" in *osracer_ws*|*ros2_ws*) ;; *) continue ;; esac
    _rc_val=$(printf '%s' "$_rc_val" | tr ':' '\n' | grep -vE '/(osracer_ws|ros2_ws)(/|$)' | paste -sd: -)
    if [ -n "$_rc_val" ]; then export "$_rc_var=$_rc_val"; else unset "$_rc_var"; fi
done
unset _rc_var _rc_val AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
export RACECAR_WS=neoracer
EOF
    echo "  added default-workspace block to $BASHRC"
fi

# Migrate the pre-reset Block 1 (plain overlay sourcing, superseded by the
# reset block; harmless but redundant). Shape is exactly 3 lines: marker,
# humble source, conditional overlay source - all written by earlier setups.
OLD_SOURCE_MARKER="# Neoracer - ROS2 + workspace overlay"
if grep -qF "$OLD_SOURCE_MARKER" "$BASHRC" 2>/dev/null; then
    sed -i "/^${OLD_SOURCE_MARKER}$/,+2d" "$BASHRC"
    echo "  removed superseded ROS2-sourcing block from $BASHRC"
fi

# Informational only: vendor references outside .bashrc are left untouched;
# flag them so a login shell picking the vendor stack back up isn't a mystery.
for f in "$USER_HOME/.profile" "$USER_HOME/.bash_profile" "$USER_HOME/.bash_login" "$USER_HOME/.bash_aliases"; do
    if [ -f "$f" ] && grep -q "osracer_ws" "$f" 2>/dev/null; then
        echo "  NOTE: $f references osracer_ws (left untouched); login shells may still load the vendor env"
    fi
done

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

# Block 4: CUDA + TensorRT command-line tools. JetPack installs both but puts
# neither on PATH, so `nvcc` and `trtexec` are missing from a login shell even
# though the toolkit is right there. trtexec is what benchmarks a .engine
# outside Python; nvcc is needed to build custom TensorRT plugins.
CUDA_MARKER="# Neoracer - CUDA toolchain"
if grep -qF "$CUDA_MARKER" "$BASHRC" 2>/dev/null; then
    echo "  $BASHRC already has the CUDA toolchain block"
elif [ -d /usr/local/cuda/bin ]; then
    cat >> "$BASHRC" <<'EOF'

# Neoracer - CUDA toolchain
# Appended by setup_user_env.sh. JetPack ships nvcc and trtexec but leaves both
# off PATH; /usr/local/cuda is a symlink to the installed CUDA point release.
export CUDA_HOME=/usr/local/cuda
export PATH="$CUDA_HOME/bin:/usr/src/tensorrt/bin:$PATH"
EOF
    echo "  added CUDA toolchain block to $BASHRC"
fi
