#!/bin/bash
# setup_networking.sh - configure the Wi-Fi AP, Ethernet dual-IP, and verify the
# Lakibeam lidar subnet on the Neoracer (Jetson Orin Nano).
#
# This script:
#   1. Installs a NetworkManager dispatcher that blocks FORWARD on the Wi-Fi
#      interface so AP clients can reach the car's services (dashboard, jupyter,
#      SSH) but cannot use the car as an internet gateway.
#   2. Creates the Neoracer AP NetworkManager connection (WPA2 / 2.4 GHz / ch 6).
#   3. Writes a netplan file so the Ethernet interface carries both a static
#      address and a DHCP-assigned address.
#   4. Removes any prior Wi-Fi client connection so the AP can claim the radio.
#   5. Verifies the Lakibeam lidar subnet (the USB-C link presents the host at
#      192.168.8.1 and the sensor at 192.168.8.2).
#
# WARNING: this reconfigures the Wi-Fi interface. If you're SSH'd in over Wi-Fi
# the connection drops when the AP comes up - run from a wired session/console.
#
# Interface names differ across boards; override via environment variables:
#   RACECAR_WIFI_IFACE    (default: wlP1p1s0)
#   RACECAR_ETH_IFACE     (default: enP8p1s0)
# Tunables (override via env or `racecar setup networking --flag=...`):
#   RACECAR_AP_SSID       (default: neoracer-1)
#   RACECAR_AP_PSK        (default: neobotics)
#   RACECAR_AP_CHANNEL    (default: 6)
#   RACECAR_AP_ADDR       (default: 10.42.0.1/24)
#   RACECAR_ETH_STATIC    (default: 192.168.1.101/24)
#   RACECAR_LIDAR_HOST    (default: 192.168.8.1/24, sensor at .2)
#
# All steps are idempotent - re-running is safe.

set -eo pipefail

# Persisted overrides - `racecar setup networking --ssid=...` writes here.
# Precedence: env vars in the current shell > persisted file > defaults.
USER_HOME="$(getent passwd "${SUDO_USER:-$USER}" | cut -d: -f6)"
PERSIST_FILE="${USER_HOME}/.config/racecar/networking.env"
if [ -f "$PERSIST_FILE" ]; then
    while IFS='=' read -r key val; do
        [ -z "$key" ] && continue
        [[ "$key" =~ ^[A-Z_][A-Z0-9_]*$ ]] || continue
        if [ -z "${!key:-}" ]; then
            val="${val%\"}"
            val="${val#\"}"
            export "$key=$val"
        fi
    done < "$PERSIST_FILE"
fi

echo "=== Neoracer Networking Setup ==="

WIFI_IFACE="${RACECAR_WIFI_IFACE:-wlP1p1s0}"
ETH_IFACE="${RACECAR_ETH_IFACE:-enP8p1s0}"

AP_SSID="${RACECAR_AP_SSID:-neoracer-1}"
AP_PSK="${RACECAR_AP_PSK:-neobotics}"
AP_CON_NAME="neoracer-ap"
AP_BAND="bg"
AP_CHANNEL="${RACECAR_AP_CHANNEL:-6}"
AP_ADDR="${RACECAR_AP_ADDR:-10.42.0.1/24}"

ETH_STATIC_ADDR="${RACECAR_ETH_STATIC:-192.168.1.101/24}"
LIDAR_HOST="${RACECAR_LIDAR_HOST:-192.168.8.1/24}"
LIDAR_SENSOR_IP="192.168.8.2"

DISPATCHER_PATH="/etc/NetworkManager/dispatcher.d/99-racecar-ap-isolate"
NETPLAN_ETH_PATH="/etc/netplan/99-racecar-eth0.yaml"

CHANGES_MADE=false

# --- 1. AP-isolation dispatcher ----------------------------------------------
echo "[1/5] Installing AP isolation dispatcher at $DISPATCHER_PATH..."
TMP_DISPATCHER=$(mktemp)
cat >"$TMP_DISPATCHER" <<SCRIPT
#!/bin/sh
# Neoracer hotspot isolation - NM's ipv4.method=shared enables IP forwarding
# and NAT, which would let AP clients route out through Ethernet. Block FORWARD
# in/out of the Wi-Fi interface so clients reach the car's own services
# (dashboard, jupyter, SSH) but cannot use the car as an internet gateway.

iface="\$1"
action="\$2"

[ "\$iface" = "$WIFI_IFACE" ] || exit 0
[ "\$CONNECTION_ID" = "$AP_CON_NAME" ] || exit 0

case "\$action" in
    up)
        iptables -D FORWARD -i $WIFI_IFACE -j REJECT 2>/dev/null
        iptables -D FORWARD -o $WIFI_IFACE -j REJECT 2>/dev/null
        iptables -I FORWARD -i $WIFI_IFACE -j REJECT
        iptables -I FORWARD -o $WIFI_IFACE -j REJECT
        ;;
    down|pre-down)
        iptables -D FORWARD -i $WIFI_IFACE -j REJECT 2>/dev/null
        iptables -D FORWARD -o $WIFI_IFACE -j REJECT 2>/dev/null
        ;;
esac
exit 0
SCRIPT
if sudo cmp -s "$TMP_DISPATCHER" "$DISPATCHER_PATH" 2>/dev/null; then
    echo "  $DISPATCHER_PATH already up to date."
else
    sudo install -m 755 -o root -g root "$TMP_DISPATCHER" "$DISPATCHER_PATH"
    echo "  Dispatcher installed."
    CHANGES_MADE=true
fi
rm -f "$TMP_DISPATCHER"

# The dispatcher is socket-activated by NetworkManager-dispatcher.service; it is
# often disabled on Desktop/JetPack images. is-enabled is the property we care
# about: will systemd start it on the next NM connection event?
if ! systemctl is-enabled --quiet NetworkManager-dispatcher.service; then
    echo "  Enabling NetworkManager-dispatcher.service..."
    sudo systemctl enable --now NetworkManager-dispatcher.service
    CHANGES_MADE=true
fi

# --- 2. Create or update the AP connection -----------------------------------
# Order matters: the AP comes up BEFORE we delete the prior Wi-Fi client below,
# so a failure under `set -e` leaves the existing connection intact for recovery.
echo "[2/5] Configuring AP connection '$AP_CON_NAME' (SSID: $AP_SSID)..."
if nmcli -t -f NAME con show | grep -qx "$AP_CON_NAME"; then
    nmcli_get() { sudo nmcli --show-secrets -g "$1" con show "$AP_CON_NAME" 2>/dev/null; }
    diff_ap=false
    for spec in \
        "802-11-wireless.ssid=$AP_SSID" \
        "802-11-wireless.mode=ap" \
        "802-11-wireless.band=$AP_BAND" \
        "802-11-wireless.channel=$AP_CHANNEL" \
        "802-11-wireless-security.key-mgmt=wpa-psk" \
        "802-11-wireless-security.psk=$AP_PSK" \
        "ipv4.method=shared" \
        "ipv4.addresses=$AP_ADDR" \
        "connection.autoconnect=yes"; do
        key="${spec%%=*}"; want="${spec#*=}"
        have=$(nmcli_get "$key" || true)
        if [ "$have" != "$want" ]; then
            diff_ap=true
            break
        fi
    done
    if [ "$diff_ap" = "true" ]; then
        echo "  Settings differ - applying."
        sudo nmcli connection modify "$AP_CON_NAME" \
            802-11-wireless.ssid "$AP_SSID" \
            802-11-wireless.mode ap \
            802-11-wireless.band "$AP_BAND" \
            802-11-wireless.channel "$AP_CHANNEL" \
            802-11-wireless-security.key-mgmt wpa-psk \
            802-11-wireless-security.psk "$AP_PSK" \
            ipv4.method shared \
            ipv4.addresses "$AP_ADDR" \
            connection.autoconnect yes
        CHANGES_MADE=true
    else
        echo "  Connection already matches desired settings."
    fi
else
    echo "  Creating new AP connection..."
    sudo nmcli connection add \
        type wifi \
        ifname "$WIFI_IFACE" \
        con-name "$AP_CON_NAME" \
        autoconnect yes \
        ssid "$AP_SSID" \
        802-11-wireless.mode ap \
        802-11-wireless.band "$AP_BAND" \
        802-11-wireless.channel "$AP_CHANNEL" \
        802-11-wireless-security.key-mgmt wpa-psk \
        802-11-wireless-security.psk "$AP_PSK" \
        ipv4.method shared \
        ipv4.addresses "$AP_ADDR"
    CHANGES_MADE=true
fi

ap_state=$(nmcli -t -f GENERAL.STATE con show "$AP_CON_NAME" 2>/dev/null | head -1)
if [ "$CHANGES_MADE" = "true" ] || [ "$ap_state" != "activated" ]; then
    sudo nmcli connection up "$AP_CON_NAME" >/dev/null 2>&1 || true
fi

# --- 3. Ethernet dual-IP via netplan -----------------------------------------
echo "[3/5] Configuring $ETH_IFACE dual-IP (static $ETH_STATIC_ADDR + DHCP)..."
TMP_NETPLAN=$(mktemp)
cat >"$TMP_NETPLAN" <<YAML
network:
  version: 2
  ethernets:
    $ETH_IFACE:
      renderer: NetworkManager
      addresses:
      - "$ETH_STATIC_ADDR"
      dhcp4: true
      dhcp6: true
      optional: true
      networkmanager:
        passthrough:
          ipv4.method: "auto"
          ipv4.address1: "$ETH_STATIC_ADDR"
          ipv4.dhcp-timeout: "15"
          ipv4.may-fail: "true"
YAML
if sudo cmp -s "$TMP_NETPLAN" "$NETPLAN_ETH_PATH" 2>/dev/null; then
    echo "  $NETPLAN_ETH_PATH already up to date."
else
    sudo install -m 600 -o root -g root "$TMP_NETPLAN" "$NETPLAN_ETH_PATH"
    echo "  Wrote $NETPLAN_ETH_PATH"
    CHANGES_MADE=true
fi
rm -f "$TMP_NETPLAN"

if [ "$CHANGES_MADE" = "true" ]; then
    echo
    echo "Applying netplan..."
    sudo netplan apply
fi

# --- 4. Delete prior Wi-Fi client connections --------------------------------
# Done after the AP + Ethernet are up so removing the old client can't strand us.
echo "[4/5] Removing any prior Wi-Fi client connection..."
mapfile -t prior_wifi < <(
    nmcli -t -f NAME,TYPE,DEVICE con show |
    awk -F: -v ap="$AP_CON_NAME" '
        $2 == "802-11-wireless" && $1 != ap { print $1 }
    '
)
if [ "${#prior_wifi[@]}" -eq 0 ]; then
    echo "  No prior Wi-Fi client connections found."
else
    for con in "${prior_wifi[@]}"; do
        echo "  Deleting connection '$con'..."
        sudo nmcli connection delete "$con"
        CHANGES_MADE=true
    done
fi

# --- 5. Lakibeam lidar subnet ------------------------------------------------
# The Lakibeam over USB-C presents a usb* gadget interface that should hold
# $LIDAR_HOST (sensor at $LIDAR_SENSOR_IP). It's normally auto-configured when
# the lidar is plugged in; verify rather than reconfigure so a working link
# isn't disturbed.
echo "[5/5] Verifying Lakibeam lidar subnet ($LIDAR_HOST, sensor $LIDAR_SENSOR_IP)..."
if ip -4 addr show 2>/dev/null | grep -q "${LIDAR_HOST%%/*}"; then
    echo "  Host address ${LIDAR_HOST%%/*} present."
elif ping -c1 -W1 "$LIDAR_SENSOR_IP" >/dev/null 2>&1; then
    echo "  Sensor reachable at $LIDAR_SENSOR_IP."
else
    echo "  NOTE: lidar subnet not detected. Plug in the Lakibeam (USB-C); it"
    echo "        should bring up a usb* interface at ${LIDAR_HOST%%/*}. If not,"
    echo "        assign it manually, e.g.:"
    echo "          sudo nmcli con add type ethernet ifname usb0 con-name lidar \\"
    echo "            ipv4.method manual ipv4.addresses $LIDAR_HOST"
fi

echo
echo "=== Done ==="
echo
echo "Verify with:"
echo "  ip -br addr show $ETH_IFACE        # static $ETH_STATIC_ADDR + DHCP"
echo "  iw dev $WIFI_IFACE info            # ssid $AP_SSID, type AP, ch $AP_CHANNEL"
echo "  sudo iptables-nft -L FORWARD -nv   # two REJECT rules with $WIFI_IFACE"
echo
echo "Join the AP from a client:"
echo "  SSID: $AP_SSID    Password: $AP_PSK"
echo "  Car reachable at ${AP_ADDR%%/*} (or http://neoracer.local)"
if [ "$CHANGES_MADE" = "false" ]; then
    echo
    echo "(No configuration changes were necessary - system already matched.)"
fi
