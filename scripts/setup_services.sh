#!/bin/bash
# Install the racecar systemd services.
#
# Two groups, installed the same way but left in different states.
#
# Core stack, dropped in /etc/systemd/system/ and enabled at boot:
#   neoracer-teleop.service    - full stack via launch_teleop.sh
#   neoracer-watchdog.service  - BindsTo=teleop, restart-on-failure supervisor
#   neoracer-dashboard.service - web status page (port 8080)
#   neoracer-jupyter.service   - JupyterLab (port 8888)
#
# neoracer-autonomy.service is held: the unit stays in this directory but is
# not installed, and an existing install is retired below. SLAM and Nav2 still
# run on demand through `racecar mapping` / `racecar navigation`.
#
# Lab dashboards, cloned into scripts/dashboards/ and installed disabled:
#   neoracer-camlabel.service   - image capture and labeling (port 8082)
#   neoracer-wallfollow.service - wall following (port 8081)
#   neoracer-pursuit.service    - target following (port 8083)
#   neoracer-eps.service        - episodic policy search (port 8084)
#
# Each dashboard is a repository of its own; its setup.sh writes the unit
# pointing at the checkout here. A dashboard holds the GPU or the camera for
# its whole run and only one lab is used at a time, so none of them start on
# boot: `racecar service start camlabel` turns one on for a session.
#
# Idempotent: re-runs update only what changed, keep a car's tuned dashboard
# yaml, and never change a service's enable state after the first install.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DASHBOARDS_DIR="$SCRIPT_DIR/dashboards"
GITHUB_ORG=https://github.com/Neobotics-Foundation-Inc

SERVICES=(
    neoracer-teleop.service
    neoracer-watchdog.service
    neoracer-dashboard.service
    neoracer-jupyter.service
)

# Units this setup deliberately does not install, and takes back off a car that
# has them from an earlier run. Their .service files stay in scripts/ so the
# work resumes by moving the name back into SERVICES.
HELD=(
    neoracer-autonomy.service
)

DASHBOARDS=(
    camlabel:camlabel_dashboard
    wallfollow:wallfollow_dashboard
    pursuit:pursuit_dashboard
    eps:eps_dashboard
)

echo "Core services:"

changed=0
for svc in "${SERVICES[@]}"; do
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
if systemctl list-unit-files 2>/dev/null | grep -q '^jupyterlab\.service'; then
    echo "  retiring legacy jupyterlab.service (replaced by neoracer-jupyter)"
    sudo systemctl disable --now jupyterlab.service 2>/dev/null || true
fi

# Take a held unit back off a car that installed it before the hold. Stopping
# and disabling would be enough to keep it off the boot path; the unit file
# goes too so `racecar service status` reports it as absent rather than as a
# service someone could start by hand and expect to work.
held_removed=0
for svc in "${HELD[@]}"; do
    if [[ -f "/etc/systemd/system/$svc" ]]; then
        echo "  $svc: held, removing (development paused)"
        sudo systemctl disable --now "$svc" 2>/dev/null || true
        sudo rm -f "/etc/systemd/system/$svc"
        sudo systemctl reset-failed "$svc" 2>/dev/null || true
        held_removed=1
    fi
done

if [[ $held_removed -eq 1 ]]; then
    sudo systemctl daemon-reload
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

# Bring one dashboard checkout to the tip of its default branch, then hand off
# to its own setup.sh, which renders the unit against this path and leaves a
# first install stopped and disabled.
#
# A fast-forward is used rather than a reset: a car's tuned wallfollow.yaml,
# pursuit.yaml, or eps.yaml lives in the checkout, and discarding it on every
# setup run would cost the tuning the lab was built around. A checkout that
# cannot fast-forward is reported and left alone.
sync_dashboard() {
    local name="$1" repo="$2" dir="$DASHBOARDS_DIR/$2"
    if [[ ! -d "$dir/.git" ]]; then
        if ! git clone -q "$GITHUB_ORG/$repo.git" "$dir" 2>/dev/null; then
            echo "  $name: clone failed; skipped (needs internet)" >&2
            return 1
        fi
        echo "  $name: cloned"
    elif ! git -C "$dir" fetch -q origin 2>/dev/null; then
        echo "  $name: fetch failed; using the checkout already here" >&2
    elif git -C "$dir" merge --ff-only -q '@{u}' 2>/dev/null; then
        echo "  $name: at origin tip"
    else
        echo "  $name: local commits or edits block a fast-forward; left as is" >&2
    fi
    bash "$dir/setup.sh" || { echo "  $name: setup.sh failed" >&2; return 1; }
}

echo
echo "Lab dashboards:"
mkdir -p "$DASHBOARDS_DIR"
dash_failed=0
for entry in "${DASHBOARDS[@]}"; do
    sync_dashboard "${entry%%:*}" "${entry#*:}" || dash_failed=1
done

if [[ $dash_failed -eq 1 ]]; then
    echo
    echo "One or more dashboards did not install. The core stack is unaffected;"
    echo "re-run 'racecar service install' once the car has internet."
fi

echo
echo "Core services installed and enabled. To start now:"
echo "  sudo systemctl start neoracer-teleop"
echo "Or reboot to bring everything up automatically."
echo
echo "Dashboards are installed but off. Start one for a session with:"
echo "  racecar service start camlabel     # or wallfollow, pursuit, eps"
