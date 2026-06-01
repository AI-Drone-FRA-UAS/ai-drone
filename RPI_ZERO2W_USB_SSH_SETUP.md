# Raspberry Pi USB SSH: Flash Storage and Connect

## Target Setup

Default Zero 2 profile:

```text
Pi username: seb
Pi hostname: seb-is-pm
Pi USB IP:   192.168.7.2
Host USB IP: 192.168.7.1
SSH command: ssh seb@192.168.7.2
```

Use the Pi Zero 2 WH micro-USB port labeled `USB` for the PC connection. The `PWR IN` port is power-only.

Pi 4 profile:

```text
Profile:     PI_PROFILE=pi4
Pi username: seb
Pi hostname: seb-is-pm2
Pi USB IP:   192.168.8.2
Host USB IP: 192.168.8.1
SSH command: ssh seb@192.168.8.2
```

Use the Pi 4 USB-C power/data port for the PC connection. The USB-A ports are not the USB gadget port.

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

For the Pi 4:

```bash
cd /home/abaris/ai-drone
PI_PROFILE=pi4 ./prepare-and-flash-pi.sh
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

For the Pi 4:

```bash
cd /home/abaris/ai-drone
PI_PROFILE=pi4 ./enable-pi-usb-gadget.sh
sync
sudo eject /dev/sda
```

Now remove the storage from the PC.

## Linux: Boot and Connect

1. Put the prepared storage into the Raspberry Pi.
2. Connect the PC to the Pi USB gadget port with a data-capable cable:
   - Zero 2 WH: micro-USB port labeled `USB`
   - Pi 4: USB-C power/data port
3. Wait 1-3 minutes.
4. Run:

```bash
cd /home/abaris/ai-drone
./connect-pi-usb-ssh.sh
```

For the Pi 4:

```bash
cd /home/abaris/ai-drone
PI_PROFILE=pi4 ./connect-pi-usb-ssh.sh
```

When SSH asks for a password, enter the Pi password you chose during flashing.

Manual SSH command:

```bash
ssh seb@192.168.7.2
```

Pi 4 manual SSH command:

```bash
ssh seb@192.168.8.2
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

Pi 4 expected:

```text
seb-is-pm2
seb
192.168.8.2/24
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

## Notes

- Use a data-capable USB cable. Charge-only cables will not work for SSH.
- Usually the Zero 2 WH can be powered from the same USB cable connected to the `USB` port.
- The Pi 4 needs a stable USB-C power/data connection; if it reboots or disconnects, use a powered USB data path or separate stable power.
- Connect one Raspberry Pi at a time unless you explicitly choose the correct `USB_IFACE`.
- If SSH warns about host authenticity on first connect, type `yes`.
- If SSH host keys conflict after reflashing, run:

```bash
ssh-keygen -R 192.168.7.2
ssh-keygen -R 192.168.8.2
```
