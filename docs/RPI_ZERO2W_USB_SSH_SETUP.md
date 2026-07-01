# Raspberry Pi USB SSH: Flash Storage and Connect

## Target Setup

```text
Pi username: seb
Pi hostname: seb-is-pm
Pi USB IP:   192.168.7.2
Host USB IP: 192.168.7.1
SSH command: ssh seb@192.168.7.2
```

Use the Pi Zero 2 WH micro-USB port labeled `USB` for the PC connection. The `PWR IN` port is power-only.

## Linux: Flash the Storage

Insert the microSD card or USB boot stick into the PC. The scripts cannot flash or patch storage while it is inserted into the Pi.

Check that the storage is `/dev/sda`:

```bash
lsblk -o NAME,PATH,SIZE,MODEL,TRAN,TYPE,FSTYPE,LABEL,MOUNTPOINTS
```

Flash and configure the card:

```bash
cd /home/abaris/ai-drone
./prepare-and-flash-pi.sh
```

When prompted:

```text
Type FLASH to continue:
```

type:

```text
FLASH
```

Then enter:

- the phone hotspot password for `Xyz`
- the new SSH password for Pi user `seb`

Wait until the script prints `Done`.

## Linux: Enable USB SSH on the storage

Keep the storage in the PC and run:

```bash
cd /home/abaris/ai-drone
./enable-pi-usb-gadget.sh
sync
sudo eject /dev/sda
```

Now remove the storage from the PC.

## Linux: Boot and Connect

1. Put the prepared storage into the Raspberry Pi.
2. Connect the PC to the Pi USB gadget port with a data-capable cable
   (Zero 2 WH: micro-USB port labeled `USB`).
3. Wait 1-3 minutes.
4. Run:

```bash
cd /home/abaris/ai-drone
uv run autoconnect          # tries Tailscale, then AI-Drone-Zero, then USB
# or, to go straight to the cable:
uv run manuconnect          # then choose 3) USB cable
```

When SSH asks for a password, enter the Pi password you chose during flashing.

Manual SSH command:

```bash
ssh seb@192.168.7.2
```

Confirm login:

```bash
hostname
whoami
ip addr show usb0
```

Expected:

```text
seb-is-pm
seb
192.168.7.2/24
```

## Windows: Connect to the Prepared Pi

1. Put the prepared microSD into the Pi.
2. Connect Windows to the Pi port labeled `USB`.
3. Wait 1-3 minutes.
4. In `Network Connections`, find the USB/RNDIS Ethernet adapter.
5. Set IPv4 manually:

```text
IP address: 192.168.7.1
Subnet mask: 255.255.255.0
Gateway: leave empty
DNS: leave empty
```

Then run in PowerShell:

```powershell
ping 192.168.7.2
ssh seb@192.168.7.2
```

From a checkout of this repo, the cross-platform helper can auto-detect the
adapter, configure it, and open SSH from native Windows. Run PowerShell or
Windows Terminal as Administrator so the adapter IP can be changed:

```powershell
uv run autoconnect
```

`autoconnect` first tries Tailscale (`seb@seb-is-pm`), then the Pi's own AP
`AI-Drone-Zero` (`192.168.4.1`), and finally configures USB. To skip straight to
the cable, use `uv run manuconnect` and choose `3) USB cable`.

Use `uv run autoconnect --dry-run` to preview every transport's commands,
including the PowerShell and `netsh` USB commands, before changing adapter
settings. If USB auto-detection fails, name the adapter with the `USB_IFACE`
environment variable:

```powershell
$env:USB_IFACE = "Ethernet 4"; uv run manuconnect   # then choose 3
```

If the USB network adapter does not appear, install/select the Microsoft `USB RNDIS Adapter` or `Remote NDIS Compatible Device` driver in Device Manager.

## macOS: Connect to the Prepared Pi

1. Put the prepared microSD into the Pi.
2. Connect macOS to the Pi port labeled `USB`.
3. Wait 1-3 minutes.
4. Open `System Settings > Network`.
5. Select the new USB Ethernet adapter.
6. Set IPv4 manually:

```text
IP address: 192.168.7.1
Subnet mask: 255.255.255.0
Router: leave empty
DNS: leave empty
```

Then run in Terminal:

```bash
ping 192.168.7.2
ssh seb@192.168.7.2
```

Or use the repo helper:

```bash
uv run autoconnect
```

## Notes

- Use a data-capable USB cable. Charge-only cables will not work for SSH.
- Usually the Zero 2 WH can be powered from the same USB cable connected to the `USB` port.
- Connect one Raspberry Pi at a time unless you explicitly choose the correct `USB_IFACE`.
- If SSH warns about host authenticity on first connect, type `yes`.
- If SSH host keys conflict after reflashing, run:

```bash
ssh-keygen -R 192.168.7.2
```

The legacy `./connect-pi-usb-ssh.sh` script remains as a compatibility wrapper
around `uv run autoconnect`.
