#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/pi-targets.sh"

PI_IP="${PI_IP:-$DEFAULT_PI_USB_IP}"
HOST_IP="${HOST_IP:-$DEFAULT_HOST_USB_IP}"
PI_USER="${PI_USER:-$DEFAULT_PI_USERNAME}"
PI_HOSTNAME="${PI_HOSTNAME:-$DEFAULT_PI_HOSTNAME}"
PI_USB_PORT_HINT="${PI_USB_PORT_HINT:-$DEFAULT_PI_USB_PORT_HINT}"
USB_IFACE="${USB_IFACE:-}"
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

find_usb_iface_darwin() {
  # Get all interfaces that are active/up
  local active_ifaces
  active_ifaces=$(ifconfig -u | grep -E '^[a-z0-9]+:' | cut -d: -f1)

  for iface in $active_ifaces; do
    # Skip loopback, Wi-Fi, bridge, awdl, llw, utun, etc.
    [[ "$iface" == "lo0" ]] && continue
    [[ "$iface" == "en0" ]] && continue
    [[ "$iface" =~ ^bridge ]] && continue
    [[ "$iface" =~ ^awdl ]] && continue
    [[ "$iface" =~ ^llw ]] && continue
    [[ "$iface" =~ ^utun ]] && continue
    [[ "$iface" =~ ^ap ]] && continue
    [[ "$iface" =~ ^anpi ]] && continue # Skip Apple USB Ethernet/Private interfaces

    # Check if the interface is physically active (not "status: inactive")
    if ifconfig "$iface" 2>/dev/null | grep -q "status: inactive"; then
      continue
    fi

    # Return the first active ethernet-like interface found
    echo "$iface"
    return 0
  done
  return 1
}

echo "Waiting for Pi USB Ethernet interface..."
echo "Target: $PI_HOSTNAME"
echo "Use a data-capable USB cable from the laptop to the $PI_USB_PORT_HINT."
echo

deadline=$((SECONDS + TIMEOUT_SECONDS))
iface=""
IS_DARWIN=false
if [[ "$(uname)" == "Darwin" ]]; then
  IS_DARWIN=true
fi

if [[ -n "$USB_IFACE" ]]; then
  if $IS_DARWIN; then
    if ! ifconfig "$USB_IFACE" >/dev/null 2>&1; then
      echo "USB_IFACE does not exist: $USB_IFACE" >&2
      exit 1
    fi
  else
    if [[ ! -d "/sys/class/net/$USB_IFACE" ]]; then
      echo "USB_IFACE does not exist: $USB_IFACE" >&2
      exit 1
    fi
  fi
  iface="$USB_IFACE"
else
  while (( SECONDS < deadline )); do
    if $IS_DARWIN; then
      iface="$(find_usb_iface_darwin || true)"
    else
      iface="$(find_usb_iface || true)"
    fi
    if [[ -n "$iface" ]]; then
      break
    fi
    sleep 2
  done
fi

if [[ -z "$iface" ]]; then
  echo "Timed out waiting for a USB Ethernet interface."
  echo
  echo "Check:"
  echo "  - The cable is a data cable, not charge-only."
  echo "  - The cable is plugged into the $PI_USB_PORT_HINT."
  echo "  - The microSD or USB boot drive was patched with ./enable-pi-usb-gadget.sh."
  echo "  - The Pi has had 1-3 minutes to boot."
  exit 1
fi

echo "Found USB network interface: $iface"
echo "Configuring laptop side as $HOST_IP/24..."

if $IS_DARWIN; then
  if ! ifconfig "$iface" | grep -qF "$HOST_IP"; then
    echo "Configuring interface $iface..."
    sudo ifconfig "$iface" up
    sudo ifconfig "$iface" mtu 1412 || true
    sudo ifconfig "$iface" "$HOST_IP" netmask 255.255.255.0
  fi
else
  if ! ip -4 addr show dev "$iface" | grep -qF "$HOST_IP/24"; then
    echo "Configuring interface $iface..."
    sudo ip link set "$iface" up
    sudo ip link set dev "$iface" mtu 1412
    sudo ip addr add "$HOST_IP/24" dev "$iface"
  fi
fi

echo "Waiting for Pi at $PI_IP..."
deadline=$((SECONDS + TIMEOUT_SECONDS))
while (( SECONDS < deadline )); do
  if ping -c 1 -W 2 "$PI_IP" >/dev/null 2>&1; then
    echo "Pi answers ping."
    echo "Connecting with SSH. Use the Pi password you set while flashing."
    if $IS_DARWIN; then
      exec ssh -t "$PI_USER@$PI_IP" "sudo ifconfig usb0 mtu 1412 2>/dev/null || true; exec \$SHELL --login"
    else
      exec ssh -t "$PI_USER@$PI_IP" "sudo ip link set dev usb0 mtu 1412 || true; exec \$SHELL --login"
    fi
  fi
  sleep 3
done

echo "Timed out waiting for $PI_IP."
echo
echo "Useful checks:"
if $IS_DARWIN; then
  echo "  ifconfig $iface"
else
  echo "  ip addr show dev $iface"
fi
echo "  ping $PI_IP"
echo "  ssh $PI_USER@$PI_IP"
exit 1
