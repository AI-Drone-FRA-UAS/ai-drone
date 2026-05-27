#!/usr/bin/env bash
set -euo pipefail

IMG="${IMG:-/home/abaris/ai-drone/raspios-lite/2026-04-21-raspios-trixie-arm64-lite.img}"
TARGET="${TARGET:-/dev/sda}"
SSID="${SSID:-Xyz}"
PI_HOSTNAME="${PI_HOSTNAME:-seb-is-pm}"
PI_USERNAME="${PI_USERNAME:-seb}"
COUNTRY="${COUNTRY:-DE}"
BOOT_MOUNT="${BOOT_MOUNT:-/mnt/rpi-boot}"

cleanup() {
  local status=$?

  if mountpoint -q "$BOOT_MOUNT"; then
    sudo umount "$BOOT_MOUNT" || true
  fi

  if [[ -n "${LOOP_DEVICE:-}" ]]; then
    sudo losetup -d "$LOOP_DEVICE" || true
  fi

  exit "$status"
}
trap cleanup EXIT

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "Missing required command: $1" >&2
    exit 1
  fi
}

require_command lsblk
require_command losetup
require_command openssl
require_command dd

if [[ ! -f "$IMG" ]]; then
  echo "Image not found: $IMG" >&2
  exit 1
fi

if [[ ! -b "$TARGET" ]]; then
  echo "Target block device not found: $TARGET" >&2
  exit 1
fi

echo "Image:  $IMG"
echo "Target: $TARGET"
echo
echo "Current target device layout:"
lsblk -o NAME,PATH,SIZE,MODEL,TRAN,TYPE,FSTYPE,LABEL,MOUNTPOINTS "$TARGET"
echo
echo "This will erase everything on $TARGET."
read -r -p "Type FLASH to continue: " CONFIRM
if [[ "$CONFIRM" != "FLASH" ]]; then
  echo "Aborted."
  exit 1
fi

read -r -s -p "Phone hotspot password for SSID $SSID: " WIFI_PASS
echo
read -r -s -p "New Pi password for user $PI_USERNAME: " PI_PASS
echo

PI_HASH="$(openssl passwd -6 "$PI_PASS")"

echo "Mounting image boot partition..."
LOOP_DEVICE="$(sudo losetup --find --partscan --show "$IMG")"
sudo mkdir -p "$BOOT_MOUNT"
sudo mount "${LOOP_DEVICE}p1" "$BOOT_MOUNT"

echo "Writing Wi-Fi config..."
sudo tee "$BOOT_MOUNT/network-config" >/dev/null <<EOF
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      optional: false
      access-points:
        "$SSID":
          password: "$WIFI_PASS"
      regulatory-domain: $COUNTRY
EOF

echo "Writing first-boot user and SSH config..."
sudo tee "$BOOT_MOUNT/user-data" >/dev/null <<EOF
#cloud-config
hostname: $PI_HOSTNAME
ssh_pwauth: true

users:
  - name: $PI_USERNAME
    gecos: Raspberry Pi
    groups:
      - adm
      - dialout
      - cdrom
      - audio
      - users
      - sudo
      - video
      - games
      - plugdev
      - input
      - gpio
      - spi
      - i2c
      - netdev
      - render
      - lpadmin
    sudo: ["ALL=(ALL) NOPASSWD:ALL"]
    shell: /bin/bash
    lock_passwd: false
    passwd: '$PI_HASH'

runcmd:
  - systemctl enable ssh
  - systemctl start ssh
EOF

sync
sudo umount "$BOOT_MOUNT"
sudo losetup -d "$LOOP_DEVICE"
unset LOOP_DEVICE

echo "Unmounting target partitions..."
while read -r mountpoint; do
  if [[ -n "$mountpoint" ]]; then
    sudo umount "$mountpoint" || true
  fi
done < <(lsblk -rn -o MOUNTPOINT "$TARGET" | sed '/^$/d')

echo "Flashing image to $TARGET..."
sudo dd if="$IMG" of="$TARGET" bs=4M status=progress conv=fsync
sync

echo "Ejecting $TARGET..."
sudo eject "$TARGET" || true

unset WIFI_PASS PI_PASS PI_HASH

echo
echo "Done. Put the microSD card in the Pi, keep hotspot '$SSID' running, wait 2-5 minutes, then run:"
echo "  ssh $PI_USERNAME@$PI_HOSTNAME.local"
