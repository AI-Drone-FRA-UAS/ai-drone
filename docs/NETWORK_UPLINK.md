# Internet access while retaining `AI-Drone-Zero`

## Current state

The Pi Zero 2 W currently uses its single onboard Broadcom radio (`wlan0`) in
one mode at a time:

- phone hotspot profiles: client mode, autoconnect priority `100`
- `eduroam`: client mode, priority `5`
- `Hotspot` / `AI-Drone-Zero`: access-point mode, priority `-10`

This is a useful fallback arrangement, but the Pi has no internet while
`wlan0` is the access point. The installed eduroam profile and its root-only
password remain on the Pi; credentials are not stored in this repository.

## Recommended simultaneous topology

Keep the control network on the onboard radio and add an independent uplink:

```text
laptop/phone <-- Wi-Fi --> wlan0 / AI-Drone-Zero / 192.168.4.1
                              Raspberry Pi
internet     <-- Wi-Fi --> wlan1 / USB Wi-Fi / eduroam or phone hotspot
```

NetworkManager's `ipv4.method shared` on the hotspot supplies DHCP, forwarding,
and NAT. Raspberry Pi's official access-point documentation likewise recommends
Ethernet or a second wireless module as the internet-facing interface.

After attaching a Linux-compatible USB Wi-Fi adapter, first run the read-only
preflight:

```bash
sudo scripts/setup-pi-dual-network.sh \
  --uplink-interface wlan1 \
  --source-profile eduroam
```

If it succeeds, apply the configuration:

```bash
sudo scripts/setup-pi-dual-network.sh \
  --uplink-interface wlan1 \
  --source-profile eduroam \
  --apply
```

The script clones the existing profile to `eduroam-uplink`, binds the clone to
`wlan1`, keeps `Hotspot` bound to `wlan0`, and never reads or prints the stored
password. A phone-hotspot profile can be used instead:

```bash
sudo scripts/setup-pi-dual-network.sh \
  --uplink-interface wlan1 \
  --source-profile "<PHONE-HOTSPOT>" \
  --uplink-profile phone-uplink \
  --apply
```

Verify both paths:

```bash
nmcli -f DEVICE,TYPE,STATE,CONNECTION device status
ip -4 route
ping -c3 1.1.1.1
curl -I https://github.com
```

## Other workable uplinks

- Share the laptop's internet to the Pi over its USB gadget/Ethernet link while
  `wlan0` remains the AP. This is already described in [the hotspot guide](HOTSPOT.md).
- Use USB tethering from a phone. It should appear as an Ethernet-style
  interface and can provide the default route without touching `wlan0`.
- Use a powered USB hub if a Wi-Fi adapter plus camera exceed the Pi/drone's USB
  power budget during bench work.

Do not depend on a virtual AP+station pair on the one onboard radio for flight
control. Even where a firmware/driver combination exposes concurrency, both
roles share airtime/channel constraints and one radio failure removes control
and internet together.

References:

- [Raspberry Pi: host a wireless network](https://www.raspberrypi.com/documentation/configuration/wireless/wireless-access-point.md)
- [NetworkManager `nmcli`](https://networkmanager.dev/docs/api/latest/nmcli.html)
- [NetworkManager IPv4 shared mode](https://networkmanager.dev/docs/libnm/latest/NMSettingIP4Config.html)
