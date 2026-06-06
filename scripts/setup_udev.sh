#!/bin/bash
# Install the Neoracer udev rules, then reload so the /dev/osrbot_* symlinks
# appear. Idempotent - re-installs every run (cheap).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

RULES_SRC="${SCRIPT_DIR}/udev/99-racecar.rules"
RULES_DST="/etc/udev/rules.d/99-racecar.rules"

if [[ ! -f "${RULES_SRC}" ]]; then
    echo "Missing ${RULES_SRC}" >&2
    exit 1
fi

sudo install -m 0644 "${RULES_SRC}" "${RULES_DST}"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Installed ${RULES_DST}."
echo "Symlinks /dev/osrbot_base, /dev/osrbot_led_matrix, /dev/osrbot_usb_cam"
echo "should now appear (re-plug a device if one is missing)."
