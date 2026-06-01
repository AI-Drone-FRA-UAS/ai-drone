#!/usr/bin/env bash

PI_PROFILE="${PI_PROFILE:-zero2}"

case "$PI_PROFILE" in
  zero2 | pi-zero2 | zero2wh | seb-is-pm)
    DEFAULT_PI_HOSTNAME="seb-is-pm"
    DEFAULT_PI_USERNAME="seb"
    DEFAULT_PI_USB_IP="192.168.7.2"
    DEFAULT_HOST_USB_IP="192.168.7.1"
    DEFAULT_PI_USB_PORT_HINT="Pi Zero 2 WH micro-USB port labeled USB, not PWR IN"
    ;;
  pi4 | raspberry-pi-4 | seb-is-pm2 | pm2)
    DEFAULT_PI_HOSTNAME="seb-is-pm2"
    DEFAULT_PI_USERNAME="seb"
    DEFAULT_PI_USB_IP="192.168.8.2"
    DEFAULT_HOST_USB_IP="192.168.8.1"
    DEFAULT_PI_USB_PORT_HINT="Pi 4 USB-C power/data port, not the USB-A ports"
    ;;
  *)
    echo "Unknown PI_PROFILE: $PI_PROFILE" >&2
    echo "Known profiles: zero2, pi4" >&2
    return 1 2>/dev/null || exit 1
    ;;
esac
