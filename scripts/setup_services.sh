#!/bin/bash
# Install + enable racecar systemd services.
#
# Drops the .service files in /etc/systemd/system/, runs daemon-reload,
# and enables each unit so it auto-starts on boot. Idempotent: re-runs
# only update files that changed.
#
# Services installed:
#   racecar-teleop.service    - full stack via launch_teleop.sh
#   racecar-watchdog.service  - BindsTo=teleop, restart-on-failure supervisor
#   racecar-dashboard.service - web status page (port 8080, after Phase 4E)
#   racecar-jupyter.service   - JupyterLab (port 8888)
#
# After install: `sudo systemctl start racecar-teleop` or reboot.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

SERVICES=(
    racecar-teleop.service
    racecar-watchdog.service
    racecar-dashboard.service
    racecar-jupyter.service
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
echo "  sudo systemctl start racecar-teleop"
echo "Or reboot to bring everything up automatically."
