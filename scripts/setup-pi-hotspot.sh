#!/usr/bin/env bash
# Save an inactive NetworkManager hotspot profile; never change the active network.
set -euo pipefail
umask 077

die() { echo "Error: $*" >&2; exit 1; }
require_value() { [[ -n "${2:-}" ]] || die "$1 requires a value"; }

SSID="AI-Drone-Zero"
PASSWORD=""
PASSWORD_FILE=""
IFACE="wlan0"
IP_ADDR="192.168.4.1/24"
CON_NAME="Hotspot"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --ssid|--password-file|--interface|--ip)
            require_value "$1" "${2:-}"
            case "$1" in
                --ssid) SSID="$2" ;;
                --password-file) PASSWORD_FILE="$2" ;;
                --interface) IFACE="$2" ;;
                --ip) IP_ADDR="$2" ;;
            esac
            shift 2
            ;;
        --password)
            die "use the secure prompt or --password-file"
            ;;
        -h|--help)
            echo "Usage: sudo $0 [--ssid SSID] [--password-file PATH] [--interface IFACE] [--ip IP/CIDR]"
            exit 0
            ;;
        *) die "Unknown option: $1" ;;
    esac
done

[[ "$(uname -s)" == "Linux" ]] || die "Raspberry Pi OS is required"
[[ -r /proc/device-tree/model ]] || die "Cannot confirm this is a Raspberry Pi"
tr '\0' '\n' < /proc/device-tree/model | grep -qi "raspberry pi" \
    || die "Raspberry Pi OS is required"
[[ $EUID -eq 0 ]] || die "Run with sudo"
command -v nmcli >/dev/null 2>&1 || die "NetworkManager is required"

# Never delete or rewrite an active connection, including an existing hotspot.
ACTIVE_PROFILES="$(nmcli -g NAME connection show --active)" \
    || die "Cannot inspect active profiles"
if grep -Fxq "$CON_NAME" <<< "$ACTIVE_PROFILES"; then
    die "Stop the hotspot explicitly before provisioning it"
fi

if [[ -n "$PASSWORD_FILE" ]]; then
    [[ -f "$PASSWORD_FILE" && -r "$PASSWORD_FILE" && ! -L "$PASSWORD_FILE" ]] \
        || die "Password file must be a readable regular file, not a symlink"
    PASSWORD_FILE_UID="$(stat -c '%u' -- "$PASSWORD_FILE")"
    [[ "$PASSWORD_FILE_UID" == "0" ]] || die "Password file must be owned by root"
    PASSWORD_FILE_MODE="$(stat -c '%a' -- "$PASSWORD_FILE")"
    (( (8#$PASSWORD_FILE_MODE & 077) == 0 )) \
        || die "Password file must not be accessible to group/others"
    IFS= read -r PASSWORD < "$PASSWORD_FILE" || [[ -n "$PASSWORD" ]] \
        || die "Could not read passphrase"
elif [[ -t 0 ]]; then
    read -r -s -p "Hotspot WPA2 passphrase: " PASSWORD
    echo
else
    die "Provide the passphrase with --password-file"
fi
[[ ${#PASSWORD} -ge 8 && ${#PASSWORD} -le 63 ]] \
    || die "WPA2 passphrase must contain between 8 and 63 characters"

if ! nmcli -g connection.id connection show "$CON_NAME" >/dev/null 2>&1; then
    nmcli connection add type wifi ifname "$IFACE" con-name "$CON_NAME" \
        autoconnect no ssid "$SSID" >/dev/null
fi
# nmcli briefly receives the secret in its process arguments; never print it.
nmcli connection modify "$CON_NAME" \
    connection.interface-name "$IFACE" \
    connection.autoconnect no \
    802-11-wireless.ssid "$SSID" \
    802-11-wireless.mode ap \
    802-11-wireless.band bg \
    ipv4.method shared \
    ipv4.addresses "$IP_ADDR" \
    802-11-wireless-security.key-mgmt wpa-psk \
    802-11-wireless-security.psk "$PASSWORD"
echo "Saved $SSID at ${IP_ADDR%/*}; hotspot remains off."
