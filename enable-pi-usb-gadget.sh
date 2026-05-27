#!/usr/bin/env bash
set -euo pipefail

BOOT_DEV="${BOOT_DEV:-/dev/sda1}"
ROOT_DEV="${ROOT_DEV:-/dev/sda2}"
BOOT_MOUNT="${BOOT_MOUNT:-/mnt/rpi-boot}"
ROOT_MOUNT="${ROOT_MOUNT:-/mnt/rpi-root}"
PI_USB_IP="${PI_USB_IP:-192.168.7.2}"

cleanup() {
  local status=$?

  if mountpoint -q "$ROOT_MOUNT"; then
    sudo umount "$ROOT_MOUNT" || true
  fi
  if mountpoint -q "$BOOT_MOUNT"; then
    sudo umount "$BOOT_MOUNT" || true
  fi

  exit "$status"
}
trap cleanup EXIT

require_block_device() {
  if [[ ! -b "$1" ]]; then
    echo "Missing block device: $1" >&2
    exit 1
  fi
}

append_config_once() {
  local file="$1"
  local line="$2"

  if ! grep -qxF "$line" "$file"; then
    printf '%s\n' "$line" | sudo tee -a "$file" >/dev/null
  fi
}

require_block_device "$BOOT_DEV"
require_block_device "$ROOT_DEV"

echo "Patching Raspberry Pi USB Ethernet gadget mode"
echo "Boot: $BOOT_DEV"
echo "Root: $ROOT_DEV"
echo "Pi USB IP: $PI_USB_IP"
echo

sudo mkdir -p "$BOOT_MOUNT" "$ROOT_MOUNT"

if findmnt -rn -S "$BOOT_DEV" >/dev/null 2>&1; then
  existing_boot_mount="$(findmnt -rn -S "$BOOT_DEV" -o TARGET | head -n1)"
  sudo umount "$existing_boot_mount"
fi

if findmnt -rn -S "$ROOT_DEV" >/dev/null 2>&1; then
  existing_root_mount="$(findmnt -rn -S "$ROOT_DEV" -o TARGET | head -n1)"
  sudo umount "$existing_root_mount"
fi

sudo mount "$BOOT_DEV" "$BOOT_MOUNT"
sudo mount "$ROOT_DEV" "$ROOT_MOUNT"

echo "Enabling dwc2 overlay in config.txt..."
append_config_once "$BOOT_MOUNT/config.txt" "dtoverlay=dwc2,dr_mode=peripheral"

echo "Adding USB Ethernet modules to cmdline.txt..."
cmdline="$(tr -d '\n' < "$BOOT_MOUNT/cmdline.txt")"
if [[ "$cmdline" != *"modules-load=dwc2,g_ether"* ]]; then
  cmdline="${cmdline/rootwait/rootwait modules-load=dwc2,g_ether}"
  printf '%s\n' "$cmdline" | sudo tee "$BOOT_MOUNT/cmdline.txt" >/dev/null
fi

echo "Creating static NetworkManager config for usb0..."
sudo mkdir -p "$ROOT_MOUNT/etc/NetworkManager/system-connections"
sudo tee "$ROOT_MOUNT/etc/NetworkManager/system-connections/usb0-gadget.nmconnection" >/dev/null <<EOF
[connection]
id=usb0-gadget
type=ethernet
interface-name=usb0
autoconnect=true

[ipv4]
method=manual
address1=$PI_USB_IP/24

[ipv6]
method=ignore
EOF
sudo chmod 600 "$ROOT_MOUNT/etc/NetworkManager/system-connections/usb0-gadget.nmconnection"
sudo chown root:root "$ROOT_MOUNT/etc/NetworkManager/system-connections/usb0-gadget.nmconnection"

echo "Creating fallback systemd service to force usb0 up..."
sudo tee "$ROOT_MOUNT/etc/systemd/system/usb0-static.service" >/dev/null <<EOF
[Unit]
Description=Configure Raspberry Pi USB gadget network
After=local-fs.target
Before=network-online.target ssh.service

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/usr/sbin/ip link set usb0 up
ExecStart=/usr/sbin/ip addr replace $PI_USB_IP/24 dev usb0

[Install]
WantedBy=multi-user.target
EOF

sudo mkdir -p "$ROOT_MOUNT/etc/systemd/system/multi-user.target.wants"
sudo ln -sf /etc/systemd/system/usb0-static.service \
  "$ROOT_MOUNT/etc/systemd/system/multi-user.target.wants/usb0-static.service"

echo "Ensuring SSH is enabled..."
sudo rm -f "$ROOT_MOUNT/etc/ssh/sshd_not_to_be_run"
sudo ln -sf /lib/systemd/system/ssh.service \
  "$ROOT_MOUNT/etc/systemd/system/multi-user.target.wants/ssh.service"

sync

echo
echo "Done. The Pi will expose USB Ethernet at $PI_USB_IP after boot."
