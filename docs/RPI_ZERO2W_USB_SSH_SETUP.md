# Raspberry Pi Zero 2 W USB SSH

The prepared Pi exposes a USB Ethernet gadget as a last-resort SSH path:

```text
Pi USB address:   192.168.7.2/24
Host USB address: 192.168.7.1/24
SSH:              seb@192.168.7.2
```

Use the Pi port labelled `USB`, not the power-only `PWR IN` port, and use a
data-capable cable. These instructions assume the Pi image already has USB
gadget networking configured; the obsolete flashing helpers formerly
documented here are not present in this repository.

## Connection helper

Identify the network adapter that appears when the Pi is connected and pass it
explicitly as `USB_IFACE`. The helper deliberately refuses to reconfigure an
adapter selected only by a broad auto-detection heuristic.

Preview first:

```bash
USB_IFACE=usb0 uv run drone-connect --dry-run
```

Then connect through the normal transport sequence or select USB directly:

```bash
USB_IFACE=usb0 uv run drone-connect
USB_IFACE=usb0 uv run drone-connect --transport usb
```

Replace `usb0` with the adapter verified on the host. `drone-connect` tries
Tailscale, the Pi hotspot, then USB.

## Linux

Find the newly attached interface with `ip link`, then use the helper above.
For a prepared static connection, the manual SSH command is:

```bash
ssh -F /dev/null seb@192.168.7.2
```

## Windows

In Network Connections, identify the new USB/RNDIS adapter and set:

```text
Address: 192.168.7.1
Mask:    255.255.255.0
Gateway: empty
DNS:     empty
```

Run an elevated PowerShell terminal when allowing the helper to change adapter
settings:

```powershell
$env:USB_IFACE = "Ethernet 4"
uv run drone-connect --dry-run
uv run drone-connect --transport usb
```

Manual connection:

```powershell
ping 192.168.7.2
ssh -F NUL seb@192.168.7.2
```

If no adapter appears, select Microsoft's USB RNDIS/Remote NDIS driver in
Device Manager.

## macOS

Identify the new USB Ethernet hardware port with
`networksetup -listallhardwareports`. Configure it manually with address
`192.168.7.1`, subnet mask `255.255.255.0`, and no router or DNS, or pass its
verified BSD name to the helper:

```bash
USB_IFACE=en7 uv run drone-connect --dry-run
USB_IFACE=en7 uv run drone-connect --transport usb
```

Manual connection:

```bash
ssh -F /dev/null seb@192.168.7.2
```

## Troubleshooting

- Wait up to three minutes for first boot and USB enumeration.
- Try another known data-capable cable.
- Connect one Pi at a time unless the adapter identity is unambiguous.
- Internet sharing can replace the Pi's static USB address with a DHCP address;
  in that mode, try `seb-is-pm.local` or inspect the sharing interface's leases.
- After reflashing, remove a stale SSH host key with:

  ```bash
  ssh-keygen -R 192.168.7.2
  ```

After login, verify the expected machine and address:

```bash
hostname
whoami
ip -4 addr
```

## Pi startup configuration

The maintained [usb0-static.service](../scripts/usb0-static.service) loads
`g_ether` after `systemd-modules-load.service` and `network.target`, then brings
up `usb0` with `192.168.7.2/24` before SSH. It does not wait for Wi-Fi association
or internet access. Deployment copies this service asset; installing it into
`/etc/systemd/system/` is a separate image-maintenance step.

For this startup arrangement, keep `dwc2` in the single-line
`/boot/firmware/cmdline.txt` module list and remove only `g_ether` from that list:
`modules-load=dwc2`. Preserve the other command-line settings and the existing
`dwc2` peripheral overlay in `/boot/firmware/config.txt`. Do not also load
`g_ether` through `/etc/modules` or another boot service.

Before changing the startup files, keep root-only backups of the command line,
the installed service, and any local gadget module options. Record when a file
did not previously exist so rollback can remove it. Preserve the Pi's current
device and host MAC addresses in a separate root-owned, mode-0600
`/etc/modprobe.d/ai-drone-usb-gadget.conf` file:

```text
options g_ether dev_addr=<saved-device-MAC> host_addr=<saved-host-MAC>
```

Replace both placeholders with that Pi's verified addresses; keep them in local
configuration. Check existing module options first and retain the distribution's
`/usr/lib/modprobe.d/g_ether.conf`, which can supply USB vendor/product identity.
Using a different local filename allows the address options to supplement it.

This boot-order change is a workaround for a recurring host transmit stall
after Pi reboot. An [upstream report](https://github.com/raspberrypi/linux/issues/3430#issuecomment-656128698)
describes similar symptoms and improvement from loading `g_ether` later; it
does not establish that every USB fault has the same cause. After maintenance,
verify USB ping/SSH over repeated Pi reboots and independently confirm the
normal network path. Keep the saved files available for rollback. This unit
does not reset the gadget periodically or start any drone task.
