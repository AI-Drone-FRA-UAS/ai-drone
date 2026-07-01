#!/usr/bin/env bash
# setup-pi-hotspot.sh — Configure the Raspberry Pi to act as a Wi-Fi Access Point (Hotspot) on startup.
#
# Run this script ON THE RASPBERRY PI as root or with sudo:
#   sudo ./setup-pi-hotspot.sh [--ssid MY_SSID] [--password MY_PASSWORD]
#
# This script automatically detects whether the Pi uses NetworkManager (standard on Pi OS Bookworm/12+)
# or the older dhcpcd/hostapd configuration (standard on Pi OS Bullseye/11 and older) and applies
# the correct hotspot configuration.

set -euo pipefail

# Default settings
SSID="AI-Drone-Zero"
PASSWORD="aidrone123"
IFACE="wlan0"
IP_ADDR="192.168.4.1/24"
CON_NAME="Hotspot"

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ssid)
      SSID="$2"
      shift 2
      ;;
    --password)
      PASSWORD="$2"
      shift 2
      ;;
    --interface)
      IFACE="$2"
      shift 2
      ;;
    --ip)
      IP_ADDR="$2"
      shift 2
      ;;
    -h|--help)
      echo "Usage: sudo $0 [--ssid SSID] [--password PASSWORD] [--interface IFACE] [--ip IP/CIDR]"
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      exit 1
      ;;
  esac
done

# Ensure script is run as root
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root (using sudo)." >&2
   exit 1
fi

# Check if Password is at least 8 characters
if [[ ${#PASSWORD} -lt 8 ]]; then
    echo "Error: Password must be at least 8 characters long for WPA2 security." >&2
    exit 1
fi

echo "=================================================="
echo " Configuring Wi-Fi Hotspot on Raspberry Pi"
echo " SSID:      $SSID"
echo " Password:  $PASSWORD"
echo " Interface: $IFACE"
echo " IP Address:$IP_ADDR"
echo "=================================================="

# Detect configuration system
if systemctl is-active --quiet NetworkManager || command -v nmcli >/dev/null 2>&1; then
    echo "▸ NetworkManager detected."
    
    # Remove existing hotspot connection if any
    echo "▸ Cleaning up any existing '$CON_NAME' connection..."
    nmcli connection delete "$CON_NAME" >/dev/null 2>&1 || true
    
    # Create the hotspot
    echo "▸ Creating hotspot connection profile..."
    nmcli connection add \
        type wifi \
        ifname "$IFACE" \
        con-name "$CON_NAME" \
        autoconnect yes \
        ssid "$SSID"
    
    # Configure the parameters
    echo "▸ Configuring hotspot parameters..."
    nmcli connection modify "$CON_NAME" \
        802-11-wireless.mode ap \
        802-11-wireless.band bg \
        ipv4.method shared \
        ipv4.addresses "$IP_ADDR" \
        802-11-wireless-security.key-mgmt wpa-psk \
        802-11-wireless-security.psk "$PASSWORD"
    
    # Set high autoconnect priority
    nmcli connection modify "$CON_NAME" connection.autoconnect-priority 100
    
    # Bring it up
    echo "▸ Activating hotspot..."
    nmcli connection up "$CON_NAME"
    
    echo "✓ Hotspot successfully configured and started via NetworkManager."
    echo "  The hotspot will automatically start at boot when $IFACE is available."

else
    echo "▸ dhcpcd/hostapd system detected (legacy Pi OS)."
    
    # Install dependencies
    echo "▸ Installing hostapd and dnsmasq..."
    apt-get update
    apt-get install -y hostapd dnsmasq
    
    # Stop services during configuration
    systemctl stop hostapd
    systemctl stop dnsmasq
    
    # 1. Configure dhcpcd static IP for wlan0
    echo "▸ Configuring /etc/dhcpcd.conf..."
    if ! grep -q "interface $IFACE" /etc/dhcpcd.conf; then
        cat <<EOF >> /etc/dhcpcd.conf

# AI-Drone Hotspot Interface Configuration
interface $IFACE
    static ip_address=${IP_ADDR%/*}
    nohook wpa_supplicant
EOF
    else
        echo "  (dhcpcd.conf already contains configuration for $IFACE, skipping)"
    fi
    
    # 2. Configure hostapd
    echo "▸ Configuring /etc/hostapd/hostapd.conf..."
    cat <<EOF > /etc/hostapd/hostapd.conf
interface=$IFACE
driver=nl80211
ssid=$SSID
hw_mode=g
channel=7
wmm_enabled=0
macaddr_acl=0
auth_algs=1
ignore_broadcast_ssid=0
wpa=2
wpa_key_mgmt=WPA-PSK
wpa_pairwise=TKIP
rsn_pairwise=CCMP
wpa_passphrase=$PASSWORD
EOF

    # Point hostapd daemon to configuration file
    if [[ -f /etc/default/hostapd ]]; then
        sed -i 's|#DAEMON_CONF=""|DAEMON_CONF="/etc/hostapd/hostapd.conf"|g' /etc/default/hostapd
    fi
    
    # 3. Configure dnsmasq
    echo "▸ Configuring /etc/dnsmasq.conf..."
    if [[ -f /etc/dnsmasq.conf ]]; then
        mv /etc/dnsmasq.conf /etc/dnsmasq.conf.orig
    fi
    
    IP_BASE="${IP_ADDR%.*}"
    cat <<EOF > /etc/dnsmasq.conf
interface=$IFACE
dhcp-range=${IP_BASE}.10,${IP_BASE}.200,255.255.255.0,24h
domain=local
address=/gw.local/${IP_BASE}.1
EOF

    # 4. Configure packet forwarding for internet sharing (optional, but good practice)
    echo "▸ Enabling IP forwarding..."
    sed -i 's|#net.ipv4.ip_forward=1|net.ipv4.ip_forward=1|g' /etc/sysctl.conf
    sysctl -p
    
    # Enable NAT (Network Address Translation)
    iptables -t nat -A POSTROUTING -o eth0 -j MASQUERADE
    # Save iptables rules
    if command -v iptables-save >/dev/null 2>&1; then
        apt-get install -y iptables-persistent
        iptables-save > /etc/iptables/rules.v4
    fi

    # Unmask, enable and start services
    echo "▸ Starting services..."
    systemctl unmask hostapd
    systemctl enable hostapd
    systemctl enable dnsmasq
    
    systemctl restart dhcpcd
    systemctl start hostapd
    systemctl start dnsmasq
    
    echo "✓ Hotspot successfully configured and started via hostapd/dnsmasq."
    echo "  The hotspot services are enabled and will start automatically at boot."
fi

echo "=================================================="
echo " Hotspot details for connecting:"
echo " SSID:     $SSID"
echo " Password: $PASSWORD"
echo " IP range: ${IP_ADDR%.*}.10 - ${IP_ADDR%.*}.200"
echo " Gateway:  ${IP_ADDR%/*}"
echo "=================================================="
