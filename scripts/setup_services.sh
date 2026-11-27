#!/bin/bash
# Install the racecar systemd services.
#
# Two groups, installed the same way but left in different states.
#
# Core stack, dropped in /etc/systemd/system/ and enabled at boot:
#   neoracer-data.service      - oneshot: the data root, ~/neoracer_data
#   neoracer-teleop.service    - full stack via launch_teleop.sh
#   neoracer-watchdog.service  - BindsTo=teleop, restart-on-failure supervisor
#   neoracer-dashboard.service - web apps page (port 8080)
#   neoracer-jupyter.service   - JupyterLab (port 8888)
#   neoracer-mapview.service   - web map viewer (port 8088)
#
# Installed, not enabled:
#   neoracer-autonomy.service  - SLAM mapping or localization, BindsTo=teleop.
#                                `racecar map` and the map viewer start it.
#                                Mapping at boot would build an unsaved map
#                                and take CPU from every lab.
#
# /etc/sudoers.d/neoracer, from sudoers.d-neoracer, lets the racecar user
# start, stop and restart these units and the lab dashboards with no password.
# It is checked with `visudo -cf` before it is installed.
#
# Lab dashboards, shipped in scripts/dashboards/ and installed disabled by
# setup_dashboards.sh:
#   neoracer-wallfollow.service  - wall following (port 8081)
#   neoracer-camlabel.service    - image capture and labeling (port 8082)
#   neoracer-pursuit.service     - target following (port 8083)
#   neoracer-eps.service         - episodic policy search (port 8084)
#   neoracer-smartfollow.service - wall and car following (port 8085)
#   neoracer-linefollow.service  - line following (port 8086)
#   neoracer-webteleop.service   - manual driving from a browser (port 8087)
#
# A dashboard holds the GPU or the camera for its whole run and only one lab
# is used at a time, so none of them start on boot:
# `racecar service start camlabel` turns one on for a session. Their tuning
# lives in the car profile, so setup never touches it.
#
# Idempotent: re-runs update only what changed and never change a service's
# enable state after the first install.
#
# Test hook: RACECAR_SYSTEMD_DIR, a folder in place of /etc/systemd/system.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SYSTEMD_DIR="${RACECAR_SYSTEMD_DIR:-/etc/systemd/system}"

SERVICES=(
    neoracer-data.service
    neoracer-teleop.service
    neoracer-watchdog.service
    neoracer-dashboard.service
    neoracer-jupyter.service
    neoracer-mapview.service
)
INSTALL_ONLY=(
    neoracer-autonomy.service
)

echo "Core services:"

changed=0

# Before 0.9.0, neoracer-autonomy.service ran the osracer autonomy base and
# could be enabled at boot. A boot link left from that unit would also start
# the new unit at boot, although the new unit has no [Install] section. Setup
# stops the old unit and removes the link. `-L` also finds a link whose unit
# file is gone.
for svc in "${INSTALL_ONLY[@]}"; do
    link="$SYSTEMD_DIR/multi-user.target.wants/$svc"
    if [[ -L "$link" ]]; then
        sudo systemctl stop "$svc" 2>/dev/null || true
        sudo rm -f "$link"
        changed=1
        echo "  $svc: stopped and removed from boot (it now starts on demand)"
    fi
done

# A unit whose file is not on the car yet is a first install. When a run stops
# before such a core unit is enabled, its new file is removed again, so the
# next run still counts it as a first install and enables it.
declare -A first_install=()
remove_new_units() {
    local svc
    for svc in "${SERVICES[@]}"; do
        if [[ -n "${first_install[$svc]:-}" ]]; then
            sudo rm -f "${SYSTEMD_DIR}/${svc}"
        fi
    done
}
trap remove_new_units EXIT
for svc in "${SERVICES[@]}" "${INSTALL_ONLY[@]}"; do
    src="${SCRIPT_DIR}/${svc}"
    dst="${SYSTEMD_DIR}/${svc}"
    if [[ ! -f "$src" ]]; then
        echo "Missing $src - skipping" >&2
        continue
    fi
    if [[ ! -e "$dst" ]]; then
        first_install[$svc]=1
    fi
    if cmp -s "$src" "$dst" 2>/dev/null; then
        echo "  $svc: already up to date"
    else
        sudo install -m 0644 "$src" "$dst"
        echo "  $svc: installed/updated"
        changed=1
    fi
done

if [[ $changed -eq 1 ]]; then
    sudo systemctl daemon-reload
    echo "  systemctl daemon-reload"
fi

# Retire the legacy jupyterlab.service if present: neoracer-jupyter.service
# replaces it, and both bind port 8888, so leaving the old one enabled would
# double-bind on boot.
# The unit list is read whole, not piped into `grep -q`. Under pipefail, grep
# exits at the first match, and the writer can then fail on a closed pipe
# although the name was found.
if [[ -n "$(systemctl list-unit-files 'jupyterlab.service' --no-legend 2>/dev/null)" ]]; then
    echo "  retiring legacy jupyterlab.service (replaced by neoracer-jupyter)"
    sudo systemctl disable --now jupyterlab.service 2>/dev/null || true
fi

# Enable a unit at its first install, so it starts on boot. A unit already
# on the car keeps its enable state, so `racecar service disable mapview`
# keeps the map viewer off across setup runs. We deliberately don't `start`
# here; the user controls when the stack first comes up (avoids surprise
# launch during install). The one exception follows the loop.
newly_enabled=()
for svc in "${SERVICES[@]}"; do
    if systemctl is-enabled --quiet "$svc"; then
        echo "  $svc: already enabled"
    elif [[ -z "${first_install[$svc]:-}" ]]; then
        echo "  $svc: installed before; enable state kept"
    else
        sudo systemctl enable "$svc"
        echo "  $svc: enabled"
        newly_enabled+=("$svc")
    fi
    unset 'first_install[$svc]'
done
trap - EXIT

# A unit enabled for the first time normally comes up at the next boot. On a
# car whose stack already runs, setup starts it now. `racecar update` from
# 0.6.x restarts only the units that the older tool lists, so without this the
# map viewer would stay down until the car reboots.
if systemctl is-active --quiet neoracer-teleop.service; then
    for svc in "${newly_enabled[@]}"; do
        if sudo systemctl start "$svc"; then
            echo "  $svc: started, because teleop is running"
        else
            echo "  $svc: did not start; see journalctl -u $svc" >&2
        fi
    done
fi

# The sudoers file gives the racecar user password-free start, stop and
# restart. A file that fails `visudo -cf` is never installed, because a broken
# sudoers file can lock sudo out for every user. The copy goes to a name with
# a dot first, which sudo never reads, and is then renamed into place in one
# step, so sudo never reads a half-written file.
SUDOERS_SRC="${SCRIPT_DIR}/sudoers.d-neoracer"
SUDOERS_DST="/etc/sudoers.d/neoracer"
SUDOERS_TMP="/etc/sudoers.d/.neoracer.new"
if sudo cmp -s "$SUDOERS_SRC" "$SUDOERS_DST" 2>/dev/null \
    && [[ "$(sudo stat -c '%a %U %G' "$SUDOERS_DST" 2>/dev/null)" == "440 root root" ]]; then
    echo "  $SUDOERS_DST: already up to date"
elif sudo visudo -cf "$SUDOERS_SRC" >/dev/null; then
    sudo install -o root -g root -m 0440 "$SUDOERS_SRC" "$SUDOERS_TMP"
    sudo mv -f "$SUDOERS_TMP" "$SUDOERS_DST"
    echo "  $SUDOERS_DST: installed/updated"
else
    echo "  $SUDOERS_SRC fails visudo -c; not installed" >&2
    exit 1
fi

# The lab dashboards ship in this tree; setup_dashboards.sh installs their
# units (no network needed) and moves the checkouts an older setup cloned
# here out of the way, keeping their tuning.
echo
echo "Lab dashboards:"
if ! bash "$SCRIPT_DIR/setup_dashboards.sh"; then
    echo
    echo "One or more dashboards did not install. The core stack is unaffected;"
    echo "re-run 'racecar service install' once the problem is resolved."
fi

# RACECAR_DATA_DIR for login shells (SSH) from the path neoracer-data.service writes.
PROFILE_SRC="${SCRIPT_DIR}/profile.d-neoracer-data.sh"
PROFILE_DST="/etc/profile.d/neoracer-data.sh"
if cmp -s "$PROFILE_SRC" "$PROFILE_DST" 2>/dev/null; then
    echo "  $PROFILE_DST: already up to date"
else
    sudo install -m 0644 "$PROFILE_SRC" "$PROFILE_DST"
    echo "  $PROFILE_DST: installed/updated"
fi

echo
echo "Core services installed. Each one is enabled at its first install and keeps"
echo "its enable state after that. neoracer-autonomy is installed and starts on"
echo "demand: racecar map start NAME, or the map viewer on port 8088."
echo "To start the core services now:"
echo "  sudo systemctl start neoracer-teleop.service"
echo "Or reboot to bring everything up automatically."
echo
echo "Dashboards are installed but off. Start one for a session with:"
echo "  racecar service start camlabel     # or wallfollow, pursuit, eps, smartfollow, linefollow, webteleop"
