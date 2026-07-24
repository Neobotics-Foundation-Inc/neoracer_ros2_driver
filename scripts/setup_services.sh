#!/bin/bash
# Install + enable racecar systemd services.
#
# Drops the .service files in /etc/systemd/system/, runs daemon-reload,
# and enables each unit so it auto-starts on boot. Idempotent: re-runs
# only update files that changed.
#
# Services installed:
#   neoracer-teleop.service    - full stack via launch_teleop.sh
#   neoracer-watchdog.service  - BindsTo=teleop, restart-on-failure supervisor
#   neoracer-dashboard.service - web status page (port 8080, after Phase 4E)
#   neoracer-jupyter.service   - JupyterLab (port 8888)
#   neoracer-autonomy.service  - TF + SLAM + Nav2 + twist bridge (osracer layer)
#
# After install: `sudo systemctl start neoracer-teleop` or reboot.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SERVICES=(
    neoracer-teleop.service
    neoracer-watchdog.service
    neoracer-dashboard.service
    neoracer-jupyter.service
    neoracer-autonomy.service
)

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

echo
echo "Services installed and enabled. To start now:"
echo "  sudo systemctl start neoracer-teleop"
echo "Or reboot to bring everything up automatically."
