#!/usr/bin/env bash
# Keep AI-Drone-Zero on wlan0 while a second network interface supplies internet.
set -euo pipefail

AP_INTERFACE="wlan0"
HOTSPOT_PROFILE="Hotspot"
UPLINK_INTERFACE="wlan1"
SOURCE_PROFILE="eduroam"
UPLINK_PROFILE="eduroam-uplink"
APPLY=0

die() {
    echo "Error: $*" >&2
    exit 1
}

usage() {
    cat <<'EOF'
Usage: sudo scripts/setup-pi-dual-network.sh [options]

By default this is a read-only preflight check. Add --apply to clone an existing
NetworkManager client profile onto a second Wi-Fi adapter and activate it while
wlan0 continues to host AI-Drone-Zero.

Options:
  --uplink-interface IFACE  Second Wi-Fi adapter (default: wlan1)
  --source-profile NAME     Existing credential-bearing profile (default: eduroam)
  --uplink-profile NAME     Cloned profile name (default: eduroam-uplink)
  --ap-interface IFACE      Hotspot adapter (default: wlan0)
  --hotspot-profile NAME    Hotspot profile (default: Hotspot)
  --apply                   Make and activate the changes
  -h, --help                Show this help

The script never prints or accepts an eduroam password. NetworkManager copies
the already-installed, root-only profile locally on the Pi.
EOF
}

require_value() {
    [[ -n "${2:-}" ]] || die "$1 requires a value"
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --uplink-interface)
            require_value "$1" "${2:-}"
            UPLINK_INTERFACE="$2"
            shift 2
            ;;
        --source-profile)
            require_value "$1" "${2:-}"
            SOURCE_PROFILE="$2"
            shift 2
            ;;
        --uplink-profile)
            require_value "$1" "${2:-}"
            UPLINK_PROFILE="$2"
            shift 2
            ;;
        --ap-interface)
            require_value "$1" "${2:-}"
            AP_INTERFACE="$2"
            shift 2
            ;;
        --hotspot-profile)
            require_value "$1" "${2:-}"
            HOTSPOT_PROFILE="$2"
            shift 2
            ;;
        --apply)
            APPLY=1
            shift
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            die "unknown option: $1"
            ;;
    esac
done

[[ "$(uname -s)" == "Linux" ]] || die "this script only supports Linux"
command -v nmcli >/dev/null 2>&1 || die "NetworkManager/nmcli is required"
[[ "$AP_INTERFACE" != "$UPLINK_INTERFACE" ]] \
    || die "the access point and uplink must use different interfaces"
nmcli -g connection.id connection show "$HOTSPOT_PROFILE" >/dev/null 2>&1 \
    || die "hotspot profile '$HOTSPOT_PROFILE' does not exist"
nmcli -g connection.id connection show "$SOURCE_PROFILE" >/dev/null 2>&1 \
    || die "source profile '$SOURCE_PROFILE' does not exist"
nmcli -g GENERAL.TYPE device show "$AP_INTERFACE" 2>/dev/null | grep -qx wifi \
    || die "$AP_INTERFACE is not an available Wi-Fi interface"
nmcli -g GENERAL.TYPE device show "$UPLINK_INTERFACE" 2>/dev/null | grep -qx wifi \
    || die "$UPLINK_INTERFACE is not an available Wi-Fi interface; attach a USB Wi-Fi adapter"

echo "Access point: $HOTSPOT_PROFILE on $AP_INTERFACE"
echo "Internet uplink: $SOURCE_PROFILE -> $UPLINK_PROFILE on $UPLINK_INTERFACE"
echo "The source profile remains unchanged and no credentials will be displayed."

if [[ $APPLY -eq 0 ]]; then
    echo "Preflight passed. Re-run with sudo and --apply to activate this topology."
    exit 0
fi

[[ $EUID -eq 0 ]] || die "--apply must be run as root"

if nmcli -g connection.id connection show "$UPLINK_PROFILE" >/dev/null 2>&1; then
    echo "Using existing uplink profile '$UPLINK_PROFILE'."
else
    echo "Cloning '$SOURCE_PROFILE' without exposing its credentials."
    nmcli connection clone "$SOURCE_PROFILE" "$UPLINK_PROFILE"
fi

nmcli connection modify "$UPLINK_PROFILE" \
    connection.interface-name "$UPLINK_INTERFACE" \
    connection.autoconnect yes \
    connection.autoconnect-priority 100 \
    ipv4.method auto \
    ipv4.never-default no

nmcli connection modify "$HOTSPOT_PROFILE" \
    connection.interface-name "$AP_INTERFACE" \
    802-11-wireless.mode ap \
    ipv4.method shared \
    connection.autoconnect yes \
    connection.autoconnect-priority -10

echo "Activating internet uplink on $UPLINK_INTERFACE ..."
nmcli connection up "$UPLINK_PROFILE" ifname "$UPLINK_INTERFACE"
echo "Ensuring hotspot remains active on $AP_INTERFACE ..."
nmcli connection up "$HOTSPOT_PROFILE" ifname "$AP_INTERFACE"

echo
nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
echo
ip -4 route
echo "Dual-interface networking is active. Test with: ping -c3 1.1.1.1"
