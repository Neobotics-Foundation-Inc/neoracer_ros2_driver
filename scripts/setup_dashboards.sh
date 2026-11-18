#!/bin/bash
# Install the lab dashboards as neoracer-* systemd units.
#
# Seven dashboards ship in scripts/dashboards/: wallfollow (8081), camlabel
# (8082), pursuit (8083), eps (8084), smartfollow (8085), linefollow (8086),
# webteleop (8087). Units are rendered from each dashboard's .service.in. See
# docs/dashboards.md.
#
# Usage:
#   setup_dashboards.sh                move old checkouts aside, then render
#                                      and install the units
#
# setup_services.sh runs this, so `racecar service install` and setup_all.sh
# both install the dashboards.
#
# A car set up before the dashboards moved in-tree has a checkout of each
# one's repository at scripts/dashboards/<repo>_dashboard/, with the car's
# tuning in it. dashboards/checkouts.py copies that tuning into the car
# profile, moves the logs and camlabel's dataset under the data root, and
# moves the checkout to scripts/dashboards/.migrated/. Nothing is deleted.
#
# A first install leaves a unit stopped and disabled. Six of the dashboards
# publish /drive and fight the mux if a second one runs, so enabling is per
# unit and deliberate: `racecar service enable wallfollow`. A unit already
# installed keeps its enable state; one that is running when its unit file
# changes is restarted on the new file.
#
# Set RACECAR_DASHBOARDS=0 to skip entirely.
#
# Test hooks: RACECAR_SYSTEMD_DIR, RACECAR_SYSTEMCTL, RACECAR_SUDO, and
# RACECAR_DASHBOARDS_DIR (a copy of scripts/dashboards/, so a test never moves
# a car's own checkouts).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
DASH_DIR="${RACECAR_DASHBOARDS_DIR:-$SCRIPT_DIR/dashboards}"
SYSTEMD_DIR="${RACECAR_SYSTEMD_DIR:-/etc/systemd/system}"
SUDO="${RACECAR_SUDO-sudo}"
SYSTEMCTL="$SUDO ${RACECAR_SYSTEMCTL:-systemctl}"

DASHBOARDS=(
    wallfollow
    camlabel
    pursuit
    eps
    smartfollow
    linefollow
    webteleop
)

case "${1:-}" in
    "") ;;
    -h|--help)
        awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "${BASH_SOURCE[0]}"
        exit 0
        ;;
    *)
        echo "usage: setup_dashboards.sh" >&2
        exit 2
        ;;
esac

if [[ "${RACECAR_DASHBOARDS:-1}" == "0" ]]; then
    echo "  RACECAR_DASHBOARDS=0: skipping lab dashboards"
    exit 0
fi

failed=()
changed=0
updated=()

# Render a template for this platform. @DIR@ and Environment=HOME= are required
# anchors; a template missing either is not installed.
render_unit() {
    local src="$1" dir="$2" token
    for token in '@DIR@' 'Environment=HOME='; do
        if ! grep -qF -- "$token" "$src"; then
            echo "  $(basename "$src"): expected '$token' not found; template changed" >&2
            return 1
        fi
    done
    sed -e "s|@DIR@|$dir|g" "$src"
}

install_unit() {
    # Two statements: bash expands every word of a `local` before any of its
    # assignments take effect, so a single one would build dir from the
    # caller's name rather than from $1.
    local name="$1"
    local dir="$DASH_DIR/$name"
    local src="$dir/neoracer-$name.service.in"
    local unit="neoracer-$name.service"
    local rendered
    if [[ ! -f "$src" ]]; then
        echo "  $name: no $(basename "$src"); skipping" >&2
        failed+=("$name (no unit template)")
        return 1
    fi

    rendered="$(mktemp)"
    if ! render_unit "$src" "$dir" >"$rendered"; then
        rm -f "$rendered"
        failed+=("$name (render)")
        return 1
    fi

    if cmp -s "$rendered" "$SYSTEMD_DIR/$unit"; then
        echo "  $unit: already up to date"
    elif $SUDO install -m 0644 "$rendered" "$SYSTEMD_DIR/$unit"; then
        echo "  $unit: installed"
        changed=1
        updated+=("$unit")
    else
        # Called under `|| true`, so errexit is off here; test the install
        # explicitly.
        echo "  $unit: install failed (sudo?)" >&2
        failed+=("$name (install)")
        rm -f "$rendered"
        return 1
    fi
    rm -f "$rendered"
}

echo "==> Old dashboard checkouts"
# The driver's Python package is imported from this tree, so the move works
# before the workspace is built. It runs as the car user: the car profile and
# the data root are theirs.
if ! PYTHONPATH="$PKG_DIR${PYTHONPATH:+:$PYTHONPATH}" python3 "$SCRIPT_DIR/dashboards/checkouts.py" "$DASH_DIR"; then
    failed+=("old checkouts")
fi

echo "==> Rendering and installing units"
for name in "${DASHBOARDS[@]}"; do
    install_unit "$name" || true
done

if [[ $changed -eq 1 ]]; then
    # Not fatal: aborting here would skip the summary of the car's state.
    if $SYSTEMCTL daemon-reload; then
        echo "  systemctl daemon-reload"
    else
        echo "  systemctl daemon-reload failed; run it by hand" >&2
        failed+=("daemon-reload")
    fi
    # A dashboard running from an old checkout or an old unit picks up the
    # new one; try-restart leaves a stopped unit stopped.
    for unit in "${updated[@]}"; do
        $SYSTEMCTL try-restart "$unit" 2>/dev/null || true
    done
fi

echo
echo "Dashboards installed. A new one is stopped and disabled. One driving dashboard at a time:"
for name in "${DASHBOARDS[@]}"; do
    src="$DASH_DIR/$name/neoracer-$name.service.in"
    [[ -f "$src" ]] || continue
    port="$(sed -n 's/.*(port \([0-9]*\)).*/\1/p' "$src" | head -1)"
    printf '  racecar service start %-12s http://%s:%s\n' \
        "$name" "$(hostname).local" "${port:-?}"
done

if [[ ${#failed[@]} -gt 0 ]]; then
    echo
    echo "Incomplete: ${failed[*]}"
    echo "Re-run 'racecar service install' once the problem is resolved."
    exit 1
fi
exit 0
