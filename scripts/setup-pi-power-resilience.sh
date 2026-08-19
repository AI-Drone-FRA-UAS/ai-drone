#!/usr/bin/env bash
# setup-pi-power-resilience.sh — Harden the companion Pi against sudden power loss.
#
# Run this script ON THE RASPBERRY PI as root or with sudo:
#   sudo scripts/setup-pi-power-resilience.sh [--dry-run] [--revert]
#
# The drone's Pi is powered from the airframe, so it is disconnected without a
# clean shutdown as a matter of routine. On 2026-08-18 an unattended
# `apt-get full-upgrade` of 150+ packages (kernel, libc6, libcamera, picamera2)
# was interrupted that way and left the package database inconsistent; the
# machine needed a manual rescue on 2026-08-19. This script applies the
# measures that keep a power cut recoverable, and deliberately does NOT make
# the root filesystem read-only, because `drone-deploy` and the sensor
# recorders both need a writable card.
#
# What it changes, all reversible with --revert:
#   1. Masks the apt timers so an unattended upgrade can never be in flight.
#   2. Gives journald a bounded persistent journal, so a power cut leaves
#      evidence behind instead of discarding every log.
#   3. Arms the hardware watchdog so a wedged Pi reboots itself rather than
#      needing the power cycle that causes the corruption in the first place.
#   4. Sets the root filesystem to remount read-only on error, so corruption
#      stops instead of spreading.
#   5. Installs the interrupted-upgrade recovery unit that repairs dpkg on the
#      next boot without an operator.
#
# It changes nothing about arming, flight modes, or the flight controller, and
# it verifies that no unit auto-starts anything that talks to the vehicle.

set -euo pipefail
umask 022

JOURNAL_DROPIN=/etc/systemd/journald.conf.d/99-ai-drone-persistent.conf
WATCHDOG_DROPIN=/etc/systemd/system.conf.d/99-ai-drone-watchdog.conf
RECOVERY_SCRIPT=/usr/local/sbin/ai-drone-first-boot-recover
RECOVERY_UNIT=/etc/systemd/system/ai-drone-first-boot-recover.service
STATE_DIR=/var/lib/ai-drone

# Journal size on a 29 GB card: large enough to span several flights, small
# enough that the writes stay irrelevant next to the recorded datasets.
JOURNAL_MAX_USE=64M
JOURNAL_MAX_FILE=8M
# Raspberry Pi OS already arms the watchdog at 1 minute in
# 40-rpi-enable-watchdog.conf. Pin that verified value rather than tightening
# it: a Zero 2 W under camera and inference load must not risk a spurious
# reboot mid-recording, and pinning keeps it armed if the vendor default moves.
RUNTIME_WATCHDOG=1min
REBOOT_WATCHDOG=2min

DRY_RUN=0
REVERT=0

die() {
    echo "Error: $*" >&2
    exit 1
}

note() { echo "  $*"; }

run() {
    if [[ "$DRY_RUN" == "1" ]]; then
        echo "  DRY-RUN: $*"
        return 0
    fi
    "$@"
}

write_file() {
    local path="$1"
    local content="$2"
    if [[ "$DRY_RUN" == "1" ]]; then
        echo "  DRY-RUN: write $path"
        return 0
    fi
    mkdir -p "$(dirname "$path")"
    printf '%s' "$content" > "$path.tmp"
    mv -f "$path.tmp" "$path"
}

require_raspberry_pi_linux() {
    [[ "$(uname -s)" == "Linux" ]] || die "This script only runs on Raspberry Pi OS/Linux."
    [[ -r /proc/device-tree/model ]] || die "Cannot confirm this is a Raspberry Pi."
    tr '\0' '\n' < /proc/device-tree/model | grep -qi "raspberry pi" \
        || die "This script only runs on a Raspberry Pi."
    command -v systemctl >/dev/null 2>&1 || die "systemctl is required."
}

require_root() {
    [[ "$DRY_RUN" == "1" || "$(id -u)" == "0" ]] \
        || die "Run this script as root (sudo scripts/setup-pi-power-resilience.sh)."
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run) DRY_RUN=1; shift ;;
        --revert) REVERT=1; shift ;;
        -h|--help) sed -n '2,29p' "$0"; exit 0 ;;
        *) die "Unknown option: $1" ;;
    esac
done

require_raspberry_pi_linux
require_root

root_device() {
    findmnt -no SOURCE / 2>/dev/null || true
}

if [[ "$REVERT" == "1" ]]; then
    echo "Reverting AI drone power-resilience settings ..."
    run systemctl unmask apt-daily.timer apt-daily-upgrade.timer || true
    run systemctl enable --now apt-daily.timer apt-daily-upgrade.timer || true
    run rm -f "$JOURNAL_DROPIN" "$WATCHDOG_DROPIN" \
        /etc/systemd/journald.conf.d/10-ai-drone-persistent.conf \
        /etc/systemd/system.conf.d/10-ai-drone-watchdog.conf
    device="$(root_device)"
    if [[ -n "$device" ]] && command -v tune2fs >/dev/null 2>&1; then
        run tune2fs -e continue "$device"
    fi
    run systemctl daemon-reload
    run systemctl restart systemd-journald
    echo "Reverted. The recovery unit and its script were left installed; they are inert once complete."
    exit 0
fi

echo "Hardening $(hostname) against sudden power loss ..."

# --- 1. No unattended package upgrades -------------------------------------
# This is the measure that addresses the failure that actually occurred. An
# interrupted upgrade is far more damaging than an interrupted data write,
# because it can leave the kernel, initramfs, or libc half-installed.
echo "[1/5] Disabling unattended package upgrades"
run systemctl mask --now apt-daily.timer apt-daily-upgrade.timer
note "Upgrade deliberately with: sudo scripts/pi-safe-upgrade.sh"

# --- 2. Bounded persistent journal -----------------------------------------
# Raspberry Pi OS ships Storage=volatile, which keeps the card quiet but means
# a power cut destroys the only record of what the Pi was doing.
echo "[2/5] Enabling a bounded persistent journal"
run rm -f /etc/systemd/journald.conf.d/10-ai-drone-persistent.conf \
    /etc/systemd/system.conf.d/10-ai-drone-watchdog.conf
write_file "$JOURNAL_DROPIN" "# Installed by scripts/setup-pi-power-resilience.sh
# Overrides the Pi OS default (Storage=volatile), which loses every log on a
# power cut and leaves no post-mortem after an unexpected disconnection.
[Journal]
Storage=persistent
SystemMaxUse=$JOURNAL_MAX_USE
SystemMaxFileSize=$JOURNAL_MAX_FILE
# Cap how much of the log a power cut can discard without syncing constantly.
SyncIntervalSec=60s
"
run mkdir -p /var/log/journal
run systemd-tmpfiles --create --prefix /var/log/journal
run systemctl restart systemd-journald
# journald keeps writing to /run until it is told to migrate; without this
# the journal stays volatile until the next reboot.
run journalctl --flush
note "Journal capped at $JOURNAL_MAX_USE; boot history survives reboots"

# --- 3. Hardware watchdog ---------------------------------------------------
# A hung Pi otherwise has to be power-cycled by hand, which is the exact event
# this script exists to survive.
echo "[3/5] Pinning the hardware watchdog"
if [[ -e /dev/watchdog ]]; then
    write_file "$WATCHDOG_DROPIN" "# Installed by scripts/setup-pi-power-resilience.sh
# Reboot a wedged Pi automatically instead of requiring the manual power cycle
# that risks filesystem corruption.
[Manager]
RuntimeWatchdogSec=$RUNTIME_WATCHDOG
RebootWatchdogSec=$REBOOT_WATCHDOG
"
    run systemctl daemon-reexec
    note "Watchdog pinned at $RUNTIME_WATCHDOG"
else
    note "No /dev/watchdog present; skipped"
fi

# --- 4. Fail closed on filesystem errors ------------------------------------
echo "[4/5] Setting the root filesystem to remount read-only on error"
device="$(root_device)"
if [[ -n "$device" ]] && command -v tune2fs >/dev/null 2>&1; then
    run tune2fs -e remount-ro "$device"
    note "$device now stops on error instead of continuing"
else
    note "Could not resolve the root device; skipped"
fi

# --- 5. Interrupted-upgrade recovery ----------------------------------------
# Installed but left disabled. pi-safe-upgrade.sh arms it immediately before an
# upgrade, so an upgrade interrupted by a power cut repairs itself on the next
# boot rather than waiting for someone to notice.
echo "[5/5] Installing the interrupted-upgrade recovery unit"
write_file "$RECOVERY_SCRIPT" '#!/bin/sh
# Installed by scripts/setup-pi-power-resilience.sh
# Finish a package upgrade that a power loss interrupted. Armed by
# pi-safe-upgrade.sh and self-disabling once the package state is consistent.
set -u

state_dir=/var/lib/ai-drone
complete="$state_dir/package-recovery-complete"
log=/var/log/ai-drone-first-boot-recovery.log

mkdir -p "$state_dir"
exec >>"$log" 2>&1

echo "=== package recovery started $(date -Is) ==="
export DEBIAN_FRONTEND=noninteractive
export NEEDRESTART_MODE=a

if ! dpkg --force-confdef --force-confold --configure -a; then
    echo "dpkg configuration failed; recovery will retry on the next boot."
    exit 1
fi

if ! apt-get -y --no-download --no-remove -f install; then
    echo "cached dependency repair failed; recovery will retry on the next boot."
    exit 1
fi

if ! dpkg --force-confdef --force-confold --configure -a; then
    echo "final dpkg configuration failed; recovery will retry on the next boot."
    exit 1
fi

audit=$(dpkg --audit)
if [ -n "$audit" ]; then
    echo "$audit"
    echo "dpkg still reports incomplete packages; recovery will retry."
    exit 1
fi

if ! apt-get check; then
    echo "apt consistency check failed; recovery will retry on the next boot."
    exit 1
fi

if [ ! -s /boot/firmware/kernel8.img ] || [ ! -s /boot/firmware/initramfs8 ]; then
    echo "updated Pi boot files are missing; recovery will retry on the next boot."
    exit 1
fi

touch "$complete"
systemctl disable ai-drone-first-boot-recover.service
sync
echo "=== package recovery completed $(date -Is) ==="
'
run chmod 0755 "$RECOVERY_SCRIPT"
write_file "$RECOVERY_UNIT" "# Installed by scripts/setup-pi-power-resilience.sh
[Unit]
Description=Finish interrupted AI drone package upgrade
After=local-fs.target NetworkManager.service
Wants=NetworkManager.service
Before=apt-daily.service apt-daily-upgrade.service
ConditionPathExists=!$STATE_DIR/package-recovery-complete

[Service]
Type=oneshot
ExecStart=$RECOVERY_SCRIPT
TimeoutStartSec=infinity
Nice=10

[Install]
WantedBy=multi-user.target
"
run mkdir -p "$STATE_DIR"
run systemctl daemon-reload
note "Recovery unit installed (left disabled; armed by pi-safe-upgrade.sh)"

# --- Safety verification ----------------------------------------------------
# The Pi must never bring up anything that commands the vehicle on boot.
echo "Verifying no unit auto-starts vehicle control ..."
autostart="$(systemctl list-unit-files --state=enabled --no-legend 2>/dev/null \
    | awk '{print $1}' \
    | grep -Ei 'drone-(control|motor|servo|record|lidar|apriltag|picam|health)|mavproxy|mavlink' \
    || true)"
if [[ -n "$autostart" ]]; then
    die "Refusing to finish: these units would command the vehicle on boot:
$autostart"
fi
note "No vehicle-control unit is enabled at boot"

echo
echo "Done. Verify with:"
echo "  systemctl is-enabled apt-daily.timer apt-daily-upgrade.timer   # masked"
echo "  journalctl --list-boots                                        # >1 after a reboot"
echo "  systemctl show -p RuntimeWatchdogUSec                          # armed"
echo "  sudo tune2fs -l \$(findmnt -no SOURCE /) | grep 'Errors behavior'  # remount-ro"
