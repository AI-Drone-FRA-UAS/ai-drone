# Raspberry Pi networking

The Pi uses one onboard Wi-Fi interface, `wlan0`. At boot,
`ai-drone-network.service` tries the preferred saved profile `Xyz`, then other
saved auto-connect client profiles, and finally starts the `AI-Drone-Zero`
fallback hotspot.

Only one `wlan0` mode is active at a time: joining a client network stops the
hotspot, and starting the hotspot disconnects the client network.

## Access and status

Prefer Tailscale whenever the Pi has an internet uplink:

```bash
ssh -F /dev/null seb@seb-is-pm
```

When the fallback hotspot is active, join `AI-Drone-Zero` and use:

```bash
ssh -F /dev/null seb@192.168.4.1
```

Inspect the active network and saved client profiles without changing them:

```bash
ai-drone-network status
ai-drone-network list
```

Network-changing commands are scheduled as transient systemd units so an SSH
session can disconnect without aborting the handover:

```bash
sudo ai-drone-network auto
sudo ai-drone-network connect "PROFILE"
sudo ai-drone-network hotspot
```

`auto` tries `Xyz`, other saved auto-connect profiles, then `Hotspot`. Profile
names and credentials stay in NetworkManager on the Pi and must not be stored
in this repository.

## Configure the fallback hotspot

Run on the Pi:

```bash
sudo scripts/setup-pi-hotspot.sh
```

The script prompts for the WPA2 passphrase without echoing it. For unattended
setup, use a root-owned file inaccessible to group and other users:

```bash
sudo scripts/setup-pi-hotspot.sh \
  --password-file /root/hotspot-passphrase
```

The resulting NetworkManager profile is named `Hotspot`, serves
`AI-Drone-Zero`, and gives the Pi `192.168.4.1/24`. It has no internet uplink
unless a second interface supplies one. Phones may need explicit confirmation
to remain connected to a network without internet.

## Add a client network

Create credentials directly on the Pi. For ordinary WPA networks, use
NetworkManager interactively and then verify that the profile is listed:

```bash
sudo nmcli device wifi connect "SSID" --ask
ai-drone-network list
```

For Frankfurt UAS eduroam, prefer the institution's current eduroam CAT
installer. The expected profile uses PEAP/MSCHAPv2, validates the institution's
CA and server domain, and stores its password only in the root-readable
NetworkManager profile. Do not disable certificate validation or copy the
credential-bearing profile into the repository.

Enabling a reachable client profile can immediately terminate hotspot SSH.
Confirm Tailscale is logged in before switching:

```bash
tailscale status --self=true --peers=false
sudo ai-drone-network connect eduroam
```

If the connection fails, inspect NetworkManager without printing secrets:

```bash
journalctl -u NetworkManager --since '-10 min'
nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
```

## Simultaneous hotspot and internet

Keeping `AI-Drone-Zero` active while also joining Wi-Fi requires a second
interface. The recommended topology is:

```text
laptop/phone <-- Wi-Fi --> wlan0 / AI-Drone-Zero / 192.168.4.1
                              Raspberry Pi
internet     <-- Wi-Fi --> wlan1 / USB Wi-Fi / client network
```

After attaching a Linux-compatible USB Wi-Fi adapter, run the read-only
preflight:

```bash
sudo scripts/setup-pi-dual-network.sh \
  --uplink-interface wlan1 \
  --source-profile eduroam
```

Apply only after the preflight identifies both interfaces and profiles:

```bash
sudo scripts/setup-pi-dual-network.sh \
  --uplink-interface wlan1 \
  --source-profile eduroam \
  --apply
```

The script clones the existing profile locally without printing its password,
binds the clone to `wlan1`, and keeps the hotspot on `wlan0`. A phone-hotspot
profile can be supplied instead. USB Ethernet or phone tethering are also
usable uplinks.

Do not rely on virtual AP/client concurrency on the single onboard radio for a
control network: both roles share airtime and one radio failure removes both
paths.

## Recovery

If the Pi becomes unreachable after a switch:

1. Wait for the boot selector to try the remaining saved profiles and hotspot.
2. Look for `AI-Drone-Zero` and connect to `192.168.4.1`.
3. If no Wi-Fi path returns, use the USB gadget procedure in
   [Raspberry Pi USB SSH](RPI_ZERO2W_USB_SSH_SETUP.md).
