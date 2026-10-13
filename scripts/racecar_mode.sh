#!/bin/bash
# Switch this car between the two ROS stacks that drive the same hardware.
#
# usage: racecar_mode.sh osracer|neoracer|status   (what `racecar mode` runs)
#
# The neoracer driver (neoracer-teleop) and the vendor osracer stack in
# ~/osracer_ws both open the ESP32 on /dev/osrbot_base and the lidar on UDP
# 2368. Two owners of either fail: two readers of one serial port split its
# frames between them and both write drive commands to it, and the second
# lidar driver cannot bind its port and publishes nothing. So one stack owns
# the hardware at a time.
#
# The mode is the enable state of neoracer-teleop: enabled is neoracer,
# disabled is osracer. It survives a reboot because unit enablement does; an
# osracer-mode car boots with the ESP32 and the lidar free for osracer's own
# launch files. neoracer-watchdog follows teleop (it BindsTo teleop, so an
# enabled watchdog would start teleop at boot). neoracer-v2v, the web
# dashboard and Jupyter run in both modes; v2v sends whatever /odom the
# active stack publishes in this car's ROS domain.
set -eo pipefail

ESP32=/dev/osrbot_base
LIDAR_PORT=2368
OSRACER_WS="$HOME/osracer_ws"
VENDOR_STOP="$OSRACER_WS/src/osracer/osracer_demo/scripts/stop_all_demo.sh"
# Lab dashboards hold the camera or the GPU and publish /drive; none of that
# belongs to osracer. They are off at boot by fleet policy and started per
# session (`racecar service start <lab>`), so the switch to osracer stops and
# disables any that run or were enabled by hand, and the switch back leaves
# them off.
LABS=(camlabel wallfollow pursuit eps smartfollow linefollow webteleop)

usage() {
    echo "usage: racecar mode osracer|neoracer|status" >&2
    exit 2
}

# No pipe into `grep -q`: it exits on the first match, systemctl then dies on
# SIGPIPE, and pipefail reports an installed unit as missing.
installed() {
    systemctl cat "$1" > /dev/null 2>&1
}

# PIDs holding the ESP32, one per line; empty when it is free.
serial_holders() {
    local pids
    if pids=$(sudo fuser "$ESP32" 2>/dev/null); then
        echo "$pids" | tr -s ' ' '\n' | grep -v '^$'
    fi
}

# PIDs bound to the lidar port, one per line; empty when it is free. sed, not
# grep: a free port is no match, and grep's exit 1 would end this set -e script.
lidar_holders() {
    sudo ss -ulpnH "sport = :$LIDAR_PORT" | sed -n 's/.*pid=\([0-9]*\).*/\1/p'
}

describe() {
    local pid
    for pid in "$@"; do
        ps -o pid=,user=,args= -p "$pid" | cut -c1-150
    done
}

# Wait until nothing holds the ESP32 or the lidar port; fail naming the holder.
wait_free() {
    local deadline=$((SECONDS + 20)) serial lidar
    while true; do
        serial=$(serial_holders)
        lidar=$(lidar_holders)
        [[ -z "$serial" && -z "$lidar" ]] && return 0
        if ((SECONDS >= deadline)); then
            echo "Still held after 20 s:" >&2
            [[ -n "$serial" ]] && { echo "  $ESP32:" >&2; describe $serial >&2; }
            [[ -n "$lidar" ]] && { echo "  UDP $LIDAR_PORT:" >&2; describe $lidar >&2; }
            echo "Stop that process, then run racecar mode again." >&2
            exit 1
        fi
        sleep 0.5
    done
}

# New shells follow the mode: a block at the end of ~/.bashrc, after setup's
# default-workspace block and the car identity block, moves an interactive
# shell to the osracer workspace while teleop is disabled. Added once, by the
# first `racecar mode`; it reads the mode at every shell start, so switching
# back needs no edit.
SHELL_MARKER="# Neoracer - follow racecar mode"
follow_mode_in_shells() {
    grep -qF "$SHELL_MARKER" "$HOME/.bashrc" && return
    cat >> "$HOME/.bashrc" <<'EOF'

# Neoracer - follow racecar mode
# Added by `racecar mode`. While the car is in osracer mode (neoracer-teleop
# disabled), new shells open on the osracer workspace; after
# `racecar mode neoracer` they open on the neoracer workspace again. Delete
# this block to stop new shells following the mode.
if systemctl cat neoracer-teleop.service > /dev/null 2>&1 \
        && ! systemctl is-enabled --quiet neoracer-teleop.service \
        && [ -f "$HOME/osracer_ws/install/setup.bash" ]; then
    racecar ws osracer > /dev/null
fi
EOF
    echo "  ~/.bashrc: new shells now follow the mode"
}

current_mode() {
    if systemctl is-enabled --quiet neoracer-teleop.service; then
        echo neoracer
    else
        echo osracer
    fi
}

to_osracer() {
    if [[ ! -f "$OSRACER_WS/install/setup.bash" ]]; then
        echo "No osracer workspace at $OSRACER_WS/install; nothing to switch to." >&2
        exit 1
    fi
    local lab
    for lab in "${LABS[@]}"; do
        if systemctl is-enabled --quiet "neoracer-$lab.service" 2>/dev/null; then
            sudo systemctl disable --now "neoracer-$lab.service"
            echo "  neoracer-$lab stopped and disabled (lab dashboards are off at boot)"
        elif systemctl is-active --quiet "neoracer-$lab.service"; then
            sudo systemctl stop "neoracer-$lab.service"
            echo "  stopped neoracer-$lab (it holds the camera or the GPU)"
        fi
    done
    sudo systemctl disable --now neoracer-watchdog.service neoracer-teleop.service
    echo "  neoracer-teleop and neoracer-watchdog stopped and disabled"
    wait_free
    echo
    echo "osracer mode: $ESP32 and the lidar (UDP $LIDAR_PORT) are free, and stay free across reboots."
    echo "This shell and every new terminal use the osracer workspace until 'racecar mode neoracer'."
    echo "Run osracer's commands as its README shows, for example:"
    echo "  ros2 launch osracer_bringup bringup.launch.py"
    echo "osracer drives the ESP32 directly: the neoracer mux and the FlySky arming switch are not in its path."
    echo "Back to the neoracer driver: racecar mode neoracer"
}

to_neoracer() {
    if [[ -f "$VENDOR_STOP" ]]; then
        echo "Stopping osracer processes (osracer_demo stop_all_demo.sh)..."
        bash "$VENDOR_STOP"
    fi
    wait_free
    sudo systemctl enable --now neoracer-teleop.service neoracer-watchdog.service
    echo
    echo "neoracer mode: neoracer-teleop and neoracer-watchdog enabled and started."
    echo "Check it: racecar service status, or ros2 topic hz /odom (about 200 Hz)."
}

show_status() {
    if ! installed neoracer-teleop.service; then
        echo "mode: neoracer-teleop is not installed (racecar service install)"
    else
        echo "mode: $(current_mode)   neoracer-teleop is $(systemctl is-enabled neoracer-teleop.service), $(systemctl is-active neoracer-teleop.service)"
    fi
    echo "ROS domain of this shell: ${ROS_DOMAIN_ID:-unset (0)}"
    local serial lidar
    serial=$(serial_holders)
    lidar=$(lidar_holders)
    echo "$ESP32:"
    if [[ -n "$serial" ]]; then describe $serial | sed 's/^/  /'; else echo "  free"; fi
    echo "lidar UDP $LIDAR_PORT:"
    if [[ -n "$lidar" ]]; then describe $lidar | sed 's/^/  /'; else echo "  free"; fi
}

case "${1:-status}" in
    osracer|neoracer) follow_mode_in_shells ;;
esac

case "${1:-status}" in
    osracer)
        if [[ "$(current_mode)" == osracer ]] && ! systemctl is-active --quiet neoracer-teleop.service; then
            # osracer owning the hardware is the point of this mode, so a
            # running osracer stack is reported, not waited out.
            echo "Already in osracer mode."
            show_status
        else
            to_osracer
        fi
        ;;
    neoracer)
        if [[ "$(current_mode)" == neoracer ]] && systemctl is-active --quiet neoracer-teleop.service; then
            # stop_all_demo.sh kills by process name, lakibeam1 included, so it
            # must never run under a live neoracer stack.
            echo "Already in neoracer mode."
        else
            to_neoracer
        fi
        ;;
    status) show_status ;;
    *) usage ;;
esac
