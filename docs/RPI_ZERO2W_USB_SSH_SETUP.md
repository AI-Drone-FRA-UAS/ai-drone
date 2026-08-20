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
USB_IFACE=usb0 uv run autoconnect --dry-run
```

Then connect through the normal transport sequence or select USB directly:

```bash
USB_IFACE=usb0 uv run autoconnect
USB_IFACE=usb0 uv run manuconnect
```

Replace `usb0` with the adapter verified on the host. `autoconnect` tries
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
uv run autoconnect --dry-run
uv run manuconnect
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
USB_IFACE=en7 uv run autoconnect --dry-run
USB_IFACE=en7 uv run manuconnect
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
