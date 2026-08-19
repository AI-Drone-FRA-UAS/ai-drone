# Live Raspberry Pi state — 2026-08-19

Captured through `seb@seb-is-pm` over Tailscale while the Pi was **disconnected
from the drone**. No arming, mode change, motor command, servo actuation, or
parameter write was performed. The only writes were the Pi system-hardening
changes described below.

## What changed since 2026-08-18

The 2026-08-18 capture recorded the Pi serving its own `AI-Drone-Zero` access
point at `192.168.4.1` with no default route and Tailscale logged out. That is
no longer the case: `wlan0` is a client on a saved phone hotspot at
`10.189.46.25/24` with a working default route, and Tailscale is online at
`100.84.84.2`. The documented priority fallback behaved exactly as designed.

## Power-loss incident and recovery

Reconstructed from the Pi on 2026-08-19:

- **2026-08-18 21:35** — `apt-daily-upgrade.timer` started an unattended
  `apt-get full-upgrade` of 150+ packages, including `linux-image`, `libc6`,
  `initramfs-tools`, `libcamera`, and `python3-picamera2`.
- Power was disconnected while it ran, leaving `dpkg` with unconfigured
  packages and half-written boot files.
- **2026-08-19 12:52** — `fsck` ran against the root filesystem.
- **12:58** — a recovery script and unit were installed by hand, and the
  NetworkManager profiles were backed up to
  `/var/lib/ai-drone/sd-recovery-20260819/`.
- **13:09** — the recovery completed and the unit disabled itself.

`EXT4-fs: orphan cleanup on readonly fs` also appears on verified clean boots of
this filesystem, because the `orphan_file` feature is enabled, so it is not
evidence of an unclean shutdown and is not cited as such here.

The Pi recovered fully. `dpkg --audit` is clean, `apt-get check` passes, the
running kernel matches the installed one, and the deployed venv still imports
Picamera2.

## Hardening applied on 2026-08-19

Applied with `sudo scripts/setup-pi-power-resilience.sh`, which is now in the
repository and reproducible. See [Pi power resilience](../../docs/PI_POWER_RESILIENCE.md).

- `apt-daily.timer` and `apt-daily-upgrade.timer` masked.
- Journal made persistent and capped at 64 MB.
- Hardware watchdog pinned at the vendor's 1 minute.
- Root filesystem set to `errors=remount-ro`.
- Interrupted-upgrade recovery unit installed, left disabled.

Verified afterwards: no systemd unit, timer, or crontab starts anything that
talks to the flight controller at boot.

## Files

- `pi-power-resilience-state.txt` — full post-hardening state capture,
  including the two items still marked as unverified pending a reboot.

## Reboot verification

A controlled reboot at 13:57 confirmed both previously pending items: the
journal now spans boots (`journalctl --list-boots` lists `-1` and `0`, 20 MB on
disk), and the live mount adopted `errors=remount-ro`. All five measures
survived the reboot, the Pi rejoined Tailscale within about five seconds, and
no vehicle-control unit started.

Not validated: none of this has been exercised against a **real power cut**.

No passwords, Wi-Fi or eduroam credentials, tokens, or keys are recorded here.
