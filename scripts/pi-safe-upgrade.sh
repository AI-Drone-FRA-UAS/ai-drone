#!/usr/bin/env bash
# pi-safe-upgrade.sh — Upgrade the companion Pi's packages recoverably.
#
# Run this script ON THE RASPBERRY PI as root or with sudo:
#   sudo scripts/pi-safe-upgrade.sh [--yes] [--check-only]
#
# scripts/setup-pi-power-resilience.sh masks the apt timers, because an
# unattended upgrade interrupted by a power cut is the failure that already
# cost this Pi a manual rescue on 2026-08-19. This script is how upgrades are
# meant to happen instead: deliberately, on known-stable power, with the
# recovery unit armed first so an interruption repairs itself on the next boot.
#
# It refuses to start when the Pi is not in a state to survive one.

set -euo pipefail
umask 022

STATE_DIR=/var/lib/ai-drone
RECOVERY_UNIT=ai-drone-first-boot-recover.service
RECOVERY_MARKER="$STATE_DIR/package-recovery-complete"

ASSUME_YES=0
CHECK_ONLY=0

die() {
    echo "Error: $*" >&2
    exit 1
}

note() { echo "  $*"; }

require_raspberry_pi_linux() {
    [[ "$(uname -s)" == "Linux" ]] || die "This script only runs on Raspberry Pi OS/Linux."
    [[ -r /proc/device-tree/model ]] || die "Cannot confirm this is a Raspberry Pi."
    tr '\0' '\n' < /proc/device-tree/model | grep -qi "raspberry pi" \
        || die "This script only runs on a Raspberry Pi."
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --yes) ASSUME_YES=1; shift ;;
        --check-only) CHECK_ONLY=1; shift ;;
        -h|--help) sed -n '2,14p' "$0"; exit 0 ;;
        *) die "Unknown option: $1" ;;
    esac
done

require_raspberry_pi_linux
[[ "$(id -u)" == "0" ]] || die "Run this script as root (sudo scripts/pi-safe-upgrade.sh)."

echo "Preflight for a recoverable package upgrade on $(hostname):"

# --- The vehicle must not be depending on this Pi right now ------------------
if [[ -e /dev/serial0 ]] && command -v fuser >/dev/null 2>&1; then
    if fuser /dev/serial0 >/dev/null 2>&1; then
        die "/dev/serial0 is in use; a recording or control session is running."
    fi
fi
note "No process is holding the flight-controller link"

# --- Power must be stable ---------------------------------------------------
# An upgrade takes minutes and rewrites the kernel and initramfs. Starting one
# on a supply that is already sagging is how the 2026-08-18 corruption happened.
if command -v vcgencmd >/dev/null 2>&1; then
    throttled="$(vcgencmd get_throttled 2>/dev/null | cut -d= -f2 || echo "")"
    if [[ -n "$throttled" && "$throttled" != "0x0" ]]; then
        die "Power is not stable (get_throttled=$throttled). Use a bench supply, not the airframe battery."
    fi
    note "No undervoltage or throttling reported"
else
    note "vcgencmd unavailable; could not verify the supply"
fi

# --- Enough space to unpack -------------------------------------------------
available_kb="$(df -Pk / | awk 'NR==2 {print $4}')"
if [[ "$available_kb" -lt 2097152 ]]; then
    die "Less than 2 GiB free on /; an interrupted unpack is likely."
fi
note "$(( available_kb / 1024 )) MiB free on /"

# --- The package database must already be consistent ------------------------
audit="$(dpkg --audit 2>/dev/null || true)"
[[ -z "$audit" ]] || die "dpkg already reports incomplete packages:
$audit
Run: sudo systemctl start $RECOVERY_UNIT"
note "dpkg reports a consistent package database"

if [[ "$CHECK_ONLY" == "1" ]]; then
    echo "Preflight passed. Re-run without --check-only to upgrade."
    exit 0
fi

if [[ "$ASSUME_YES" != "1" ]]; then
    echo
    echo "This rewrites system packages and may replace the kernel and initramfs."
    echo "Do not disconnect power until it reports completion."
    read -r -p "Type UPGRADE to continue: " reply
    [[ "$reply" == "UPGRADE" ]] || die "Not confirmed; nothing was changed."
fi

# --- Back up what a failed upgrade would be painful to rebuild --------------
backup_dir="$STATE_DIR/sd-recovery-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$backup_dir"
chmod 0700 "$backup_dir"
if compgen -G "/etc/NetworkManager/system-connections/*.nmconnection" >/dev/null; then
    # These carry Wi-Fi secrets; the 0700 directory keeps them unreadable to
    # other users and they must never leave the Pi.
    cp -a /etc/NetworkManager/system-connections/*.nmconnection "$backup_dir/"
    note "NetworkManager profiles backed up to $backup_dir"
fi

# --- Arm recovery BEFORE touching the package database ----------------------
rm -f "$RECOVERY_MARKER"
systemctl enable "$RECOVERY_UNIT" >/dev/null 2>&1 \
    || die "Could not arm $RECOVERY_UNIT; run scripts/setup-pi-power-resilience.sh first."
sync
note "Recovery armed: an interrupted upgrade will repair itself on next boot"

# --- Upgrade ----------------------------------------------------------------
echo
echo "Upgrading. Do not disconnect power ..."
export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a
apt-get update
apt-get -y -o Dpkg::Options::=--force-confdef -o Dpkg::Options::=--force-confold full-upgrade
apt-get -y autoremove --purge
sync

# --- Disarm only once the result is verifiably consistent -------------------
remaining="$(dpkg --audit 2>/dev/null || true)"
[[ -z "$remaining" ]] || die "Upgrade finished with incomplete packages:
$remaining
Recovery stays armed; reboot to let it finish."
apt-get check
touch "$RECOVERY_MARKER"
systemctl disable "$RECOVERY_UNIT" >/dev/null 2>&1 || true
sync

echo
echo "Upgrade complete and the package database is consistent."
echo "Reboot before the next flight if the kernel or firmware changed:"
echo "  uname -r  ->  $(uname -r)"
