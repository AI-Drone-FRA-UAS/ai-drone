#!/usr/bin/env bash
set -euo pipefail

PI_IP="${PI_IP:-192.168.7.2}"
HOST_IP="${HOST_IP:-192.168.7.1}"
PI_USER="${PI_USER:-seb}"
TIMEOUT_SECONDS="${TIMEOUT_SECONDS:-180}"

is_usb_netdev() {
  local iface="$1"
  local dev_path

  [[ "$iface" == "lo" ]] && return 1
  [[ "$iface" == wlan* || "$iface" == wl* ]] && return 1
  [[ "$iface" == docker* || "$iface" == br-* || "$iface" == virbr* || "$iface" == veth* ]] && return 1

  dev_path="$(readlink -f "/sys/class/net/$iface/device" 2>/dev/null || true)"
  [[ "$dev_path" == *"/usb"* ]]
}

find_usb_iface() {
  local iface
  for iface in /sys/class/net/*; do
    iface="${iface##*/}"
    if is_usb_netdev "$iface"; then
      echo "$iface"
      return 0
    fi
  done
  return 1
}

echo "Waiting for Pi USB Ethernet interface..."
echo "Use a data-capable USB cable from the laptop to the Pi Zero 2 WH port labeled USB, not PWR IN."
echo

deadline=$((SECONDS + TIMEOUT_SECONDS))
iface=""

while (( SECONDS < deadline )); do
  iface="$(find_usb_iface || true)"
  if [[ -n "$iface" ]]; then
    break
  fi
  sleep 2
done

if [[ -z "$iface" ]]; then
  echo "Timed out waiting for a USB Ethernet interface."
  echo
  echo "Check:"
  echo "  - The cable is a data cable, not charge-only."
  echo "  - The cable is plugged into the Pi port labeled USB."
  echo "  - The microSD was patched with ./enable-pi-usb-gadget.sh."
  echo "  - The Pi has had 1-3 minutes to boot."
  exit 1
fi

echo "Found USB network interface: $iface"
echo "Configuring laptop side as $HOST_IP/24..."

sudo ip link set "$iface" up
if ! ip -4 addr show dev "$iface" | grep -q "$HOST_IP/24"; then
  sudo ip addr add "$HOST_IP/24" dev "$iface"
fi

echo "Waiting for Pi at $PI_IP..."
deadline=$((SECONDS + TIMEOUT_SECONDS))
while (( SECONDS < deadline )); do
  if ping -c 1 -W 2 "$PI_IP" >/dev/null 2>&1; then
    echo "Pi answers ping."
    echo "Connecting with SSH. Use the Pi password you set while flashing."
    exec ssh "$PI_USER@$PI_IP"
  fi
  sleep 3
done

echo "Timed out waiting for $PI_IP."
echo
echo "Useful checks:"
echo "  ip addr show dev $iface"
echo "  ping $PI_IP"
echo "  ssh $PI_USER@$PI_IP"
exit 1
