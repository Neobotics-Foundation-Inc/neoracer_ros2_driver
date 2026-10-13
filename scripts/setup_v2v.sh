#!/bin/bash
# Stamp this car with its V2V identity and install neoracer-v2v.service.
#
# usage: setup_v2v.sh <id>          (what `racecar v2v id <id>` runs)
#
# The id becomes NEORACER_CAR_ID, the number this car broadcasts under, and
# ROS_DOMAIN_ID, so two cars on one network keep separate ROS graphs. Without
# that, DDS discovery merges them and car 2's /drive steers car 1.
#
# Writes ~/.neoracer/env, sources it from ~/.bashrc, hands it to every
# neoracer-*.service (present and future) through one systemd drop-in,
# installs and enables neoracer-v2v.service, then restarts the enabled units
# so they come back in the new domain. The car must be stationary: the control
# chain restarts.
#
# Idempotent: re-run with a new id to renumber a car.
set -eo pipefail

usage() {
    echo "usage: racecar v2v id <1-101>" >&2
    echo "The id doubles as ROS_DOMAIN_ID; 1-101 is the range DDS maps to ports on every host." >&2
    exit 2
}
ID="${1:-}"
[[ "$ID" =~ ^[0-9]+$ ]] || usage
ID=$((10#$ID))
(( ID >= 1 && ID <= 101 )) || usage

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_DIR="$HOME/.neoracer"
ENV_FILE="$ENV_DIR/env"
DROPIN_DIR=/etc/systemd/system/neoracer-.service.d
UNIT=neoracer-v2v.service
WS="$HOME/ros2_ws"

# The node and its launch file enter the install on a build. A car that pulled
# this release without rebuilding gets its build here, so the stamp is the
# only command a car needs.
if [[ ! -x "$WS/install/neoracer_ros2_driver/lib/neoracer_ros2_driver/v2v_node" ]]; then
    echo "  v2v_node not built yet: building neoracer_ros2_driver"
    (
        # shellcheck disable=SC1091
        source /opt/ros/humble/setup.bash
        cd "$WS" && colcon build --packages-select neoracer_ros2_driver --symlink-install >/dev/null
    )
    echo "  built"
fi

mkdir -p "$ENV_DIR"
cat > "$ENV_FILE" <<EOF
# Written by \`racecar v2v id $ID\` on $(date -Is). Re-run it to renumber.
NEORACER_CAR_ID=$ID
ROS_DOMAIN_ID=$ID
EOF
echo "  $ENV_FILE: NEORACER_CAR_ID=$ID ROS_DOMAIN_ID=$ID"

if grep -qF '# Neoracer - car identity' "$HOME/.bashrc"; then
    echo "  ~/.bashrc: identity block already present"
else
    cat >> "$HOME/.bashrc" <<'EOF'

# Neoracer - car identity
# NEORACER_CAR_ID and ROS_DOMAIN_ID, written by `racecar v2v id N`. The
# services read the same file, so a shell and the stack share one ROS domain.
set -a
[ -f "$HOME/.neoracer/env" ] && . "$HOME/.neoracer/env"
set +a
EOF
    echo "  ~/.bashrc: identity block added"
fi

# The desktop session reads ~/.config/environment.d, not ~/.bashrc, so
# programs started from the car's desktop (osracer's demo shortcuts among
# them) join this car's ROS domain too. Takes effect at the next login.
mkdir -p "$HOME/.config/environment.d"
ln -sfn "$ENV_FILE" "$HOME/.config/environment.d/10-neoracer.conf"
echo "  ~/.config/environment.d/10-neoracer.conf -> $ENV_FILE (desktop session, next login)"

# One drop-in under the truncated unit name applies to every neoracer-*
# service, including lab dashboards installed later. The leading dash makes
# a missing env file a no-op: driving must not depend on an optional feature,
# and v2v_node fails loudly on its own when the id is gone.
sudo install -d -m 0755 "$DROPIN_DIR"
printf '[Service]\nEnvironmentFile=-%s\n' "$ENV_FILE" | sudo tee "$DROPIN_DIR/10-car-identity.conf" >/dev/null
echo "  $DROPIN_DIR/10-car-identity.conf: EnvironmentFile=-$ENV_FILE"

if cmp -s "$SCRIPT_DIR/$UNIT" "/etc/systemd/system/$UNIT" 2>/dev/null; then
    echo "  $UNIT: already up to date"
else
    sudo install -m 0644 "$SCRIPT_DIR/$UNIT" "/etc/systemd/system/$UNIT"
    echo "  $UNIT: installed/updated"
fi
sudo systemctl daemon-reload
sudo systemctl enable "$UNIT"
echo "  $UNIT: enabled"

# Restart what is enabled so every unit picks up the domain; a disabled lab
# dashboard stays down, as `racecar service restart` would leave it.
live=()
for f in /etc/systemd/system/neoracer-*.service; do
    u="$(basename "$f")"
    systemctl is-enabled --quiet "$u" && live+=("$u")
done
sudo systemctl restart "${live[@]}"
echo "  restarted: ${live[*]}"

echo
echo "Car $ID is stamped. In a new shell (or after: set -a; . ~/.neoracer/env; set +a):"
echo "  racecar v2v status"
