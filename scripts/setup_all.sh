#!/bin/bash
# Neoracer - one-shot setup orchestrator.
#
# Usage: bash scripts/setup_all.sh
# Idempotent: re-runs skip completed phases.
# Target: Jetson Orin Nano, JetPack 6.x / Ubuntu 22.04 (Jammy), ROS2 Humble.
# A sudo password may be prompted once and cached for the rest of the run.

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "$(lsb_release -cs)" != "jammy" ]; then
    echo "WARNING: this script targets Ubuntu 22.04 (Jammy); detected '$(lsb_release -cs)'."
    echo "Continue anyway? [y/N]"
    read -r ans
    [ "$ans" = "y" ] || exit 1
fi

# Keep sudo alive across the whole run.
sudo -v
( while true; do sudo -n true; sleep 60; kill -0 "$$" || exit; done ) 2>/dev/null &
SUDO_KEEPALIVE_PID=$!
trap 'kill $SUDO_KEEPALIVE_PID 2>/dev/null' EXIT

echo "==> [1/8] ROS2 Humble + driver dependencies"
bash "$SCRIPT_DIR/setup_ros2.sh"

echo
echo "==> [2/8] Robotics dev tools"
bash "$SCRIPT_DIR/setup_dev_tools.sh"

echo
echo "==> [3/8] User environment (groups, .bashrc, racecar tool)"
bash "$SCRIPT_DIR/setup_user_env.sh"

echo
echo "==> [4/8] udev rules (stable /dev/osrbot_base, _led_matrix, _usb_cam)"
bash "$SCRIPT_DIR/setup_udev.sh"

echo
echo "==> [5/8] Workspace build (driver + pinned Lakibeam lidar clone)"
bash "$SCRIPT_DIR/setup_workspace.sh"

echo
echo "==> [6/8] GPU stack (PyTorch, Ultralytics, TensorRT check)"
bash "$SCRIPT_DIR/setup_ml.sh"

echo
echo "==> [7/8] JupyterLab + workspace"
bash "$SCRIPT_DIR/setup_jupyter.sh"

echo
echo "==> [8/8] systemd services (teleop, watchdog, dashboard, jupyter)"
bash "$SCRIPT_DIR/setup_services.sh"

echo
echo "Setup complete."
echo "Networking (Wi-Fi AP + Lakibeam subnet) is configured separately:"
echo "  racecar setup networking        # or: bash scripts/setup_networking.sh"
echo "Log out and back in (or 'newgrp dialout') so group changes apply, then:"
echo "  racecar teleop                  # or: sudo systemctl start neoracer-teleop"
