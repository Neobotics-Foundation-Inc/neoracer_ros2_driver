#!/bin/bash
# Manage the V2V fleet signing key (what `racecar v2v key` runs).
#
# usage: v2v_key.sh new | set <64 hex> | show | off
#
# With ~/.neoracer/v2v.key present, v2v_node signs every packet it sends and
# rejects anything unsigned or signed with another key. Every car in the fleet
# needs the same key: make it once with `new` on one car, then `set` it on the
# others. `show` prints a fingerprint, equal on cars that share a key, without
# printing the key. Each action restarts neoracer-v2v so it takes effect.
set -eo pipefail

KEY_FILE="$HOME/.neoracer/v2v.key"

restart_v2v() {
    if systemctl is-enabled --quiet neoracer-v2v.service; then
        sudo systemctl restart neoracer-v2v.service
        echo "  neoracer-v2v restarted"
    else
        echo "  neoracer-v2v is not enabled on this car; stamp it first: racecar v2v id N"
    fi
}

fingerprint() {
    python3 -c "
import sys
sys.path.insert(0, '$HOME/ros2_ws/src/neoracer_ros2_driver')
from neoracer_ros2_driver.v2v_lib import key_fingerprint, load_key
print(key_fingerprint(load_key('$KEY_FILE')))"
}

write_key() {
    mkdir -p "$(dirname "$KEY_FILE")"
    ( umask 077; printf '%s\n' "$1" > "$KEY_FILE" )
    echo "  $KEY_FILE written (mode 600), fingerprint $(fingerprint)"
}

case "${1:-show}" in
    new)
        key=$(python3 -c 'import secrets; print(secrets.token_hex(32))')
        write_key "$key"
        restart_v2v
        echo
        echo "Give every other car the same key:"
        echo "  racecar v2v key set $key"
        ;;
    set)
        if ! [[ "${2:-}" =~ ^[0-9a-fA-F]{64}$ ]]; then
            echo "usage: racecar v2v key set <64 hex characters>" >&2
            exit 2
        fi
        write_key "$2"
        restart_v2v
        ;;
    show)
        if [[ -f "$KEY_FILE" ]]; then
            echo "signing ON, key fingerprint $(fingerprint) ($KEY_FILE)"
        else
            echo "signing off (no $KEY_FILE)"
        fi
        ;;
    off)
        rm -f "$KEY_FILE"
        echo "  $KEY_FILE removed: signing off"
        restart_v2v
        ;;
    *)
        echo "usage: racecar v2v key new | set <64 hex> | show | off" >&2
        exit 2
        ;;
esac
