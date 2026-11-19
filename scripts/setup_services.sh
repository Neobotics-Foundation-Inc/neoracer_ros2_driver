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
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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

# Before 0.9.0, neoracer-autonomy.service ran the osracer autonomy base and
# could be enabled at boot. The new unit has no [Install] section, so
# `systemctl disable` cannot undo that link once it is installed. Disable the
# old unit first, while its file is still in place.
for svc in "${INSTALL_ONLY[@]}"; do
    if [[ -e "/etc/systemd/system/multi-user.target.wants/$svc" ]]; then
        sudo systemctl disable "$svc"
        echo "  $svc: disabled at boot (it now starts on demand)"
    fi
done

changed=0
for svc in "${SERVICES[@]}" "${INSTALL_ONLY[@]}"; do
    src="${SCRIPT_DIR}/${svc}"
    dst="/etc/systemd/system/${svc}"
    if [[ ! -f "$src" ]]; then
        echo "Missing $src - skipping" >&2
        continue
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
# No pipe into `grep -q`. Under pipefail, grep exits at the first match,
# and the writer can then fail on a closed pipe although the name was found.
if [[ -n "$(systemctl list-unit-files 'jupyterlab.service' --no-legend 2>/dev/null)" ]]; then
    echo "  retiring legacy jupyterlab.service (replaced by neoracer-jupyter)"
    sudo systemctl disable --now jupyterlab.service 2>/dev/null || true
fi

# Enable so they auto-start on boot. `enable` is idempotent - no-op if
# already enabled. We deliberately don't `start` here; the user controls
# when the stack first comes up (avoids surprise launch during install).
for svc in "${SERVICES[@]}"; do
    if systemctl is-enabled --quiet "$svc"; then
        echo "  $svc: already enabled"
    else
        sudo systemctl enable "$svc"
        echo "  $svc: enabled"
    fi
done

# Password-free start, stop and restart for the racecar user. A file that
# fails `visudo -cf` is never installed: a broken sudoers file can lock sudo
# out for every user.
SUDOERS_SRC="${SCRIPT_DIR}/sudoers.d-neoracer"
SUDOERS_DST="/etc/sudoers.d/neoracer"
if sudo cmp -s "$SUDOERS_SRC" "$SUDOERS_DST" 2>/dev/null; then
    echo "  $SUDOERS_DST: already up to date"
elif sudo visudo -cqf "$SUDOERS_SRC"; then
    sudo install -o root -g root -m 0440 "$SUDOERS_SRC" "$SUDOERS_DST"
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
echo "Core services installed and enabled. neoracer-autonomy is installed and"
echo "starts on demand: racecar map start NAME, or the map viewer on port 8088."
echo "To start the core services now:"
echo "  sudo systemctl start neoracer-teleop"
echo "Or reboot to bring everything up automatically."
echo
echo "Dashboards are installed but off. Start one for a session with:"
echo "  racecar service start camlabel     # or wallfollow, pursuit, eps, smartfollow, linefollow, webteleop"
