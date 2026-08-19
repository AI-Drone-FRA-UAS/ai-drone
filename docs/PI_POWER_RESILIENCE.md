# Raspberry Pi power-loss resilience

Last verified: **2026-08-19, Europe/Berlin**

The companion Pi is powered from the airframe, so it loses power without a
clean shutdown as a matter of routine. This document records the failure that
already occurred, the measures now in place, and what is still unverified.

## The incident this responds to

On **2026-08-18 at 21:35** the Pi's `apt-daily-upgrade.timer` began an
unattended `apt-get full-upgrade` of more than 150 packages, including
`linux-image`, `libc6`, `initramfs-tools`, `libcamera`, and `python3-picamera2`.
Power was disconnected while it ran.

The consequences, reconstructed from the Pi on 2026-08-19:

- `dpkg` was left with unconfigured packages and the boot files half-written.
- `fsck` ran against the root filesystem at 12:52.
- A recovery script and unit were installed by hand at 12:58, and the
  NetworkManager profiles were backed up to `/var/lib/ai-drone/sd-recovery-20260819/`.
- The 13:09 boot completed the package recovery, and the unit disabled itself.

Note for future diagnosis: `EXT4-fs: orphan cleanup on readonly fs` appears on
this filesystem at **every** boot, including verified clean ones, because the
`orphan_file` feature is enabled. It is not a marker of an unclean shutdown.
Use `journalctl --list-boots` and the shutdown records instead.

The Pi recovered fully: `dpkg --audit` is clean, `apt-get check` passes, the
running kernel matches the installed one, and the deployed venv still imports
Picamera2. The rescue was manual, though, and nothing about it lived in this
repository until now.

An interrupted package upgrade is far more damaging than an interrupted data
write, because it can leave the kernel, initramfs, or libc half-installed. That
is why the first measure below matters most.

## Measures in place

Apply or re-apply them with `sudo scripts/setup-pi-power-resilience.sh`, which
is idempotent and supports `--dry-run` and `--revert`.

| # | Measure | Why |
|---|---|---|
| 1 | `apt-daily.timer` and `apt-daily-upgrade.timer` masked | Removes the exact mechanism that caused the 2026-08-18 corruption. Upgrades now happen only through `scripts/pi-safe-upgrade.sh`. |
| 2 | Bounded persistent journal (64 MB cap, 60 s sync) | Raspberry Pi OS ships `Storage=volatile`, so a power cut previously destroyed every log and left no post-mortem. |
| 3 | Hardware watchdog pinned at 1 min | A wedged Pi reboots itself instead of needing the manual power cycle that causes corruption. |
| 4 | Root filesystem `errors=remount-ro` | Corruption stops and becomes visible instead of spreading silently. |
| 5 | Interrupted-upgrade recovery unit installed | An upgrade interrupted by a power cut repairs itself on the next boot instead of waiting for an operator. |

Already correct before this work, and deliberately left alone:

- `fsck.repair=yes` on the kernel command line.
- `noatime` on the root filesystem.
- Swap is zram only (`/dev/zram0`), so swapping never writes to the SD card.
- `e2scrub_all.timer` and `fstrim.timer` enabled.

### Drop-in ordering

systemd merges drop-ins by **filename across all directories**, so a file in
`/etc/systemd/*.conf.d/` does not automatically win over one in
`/usr/lib/systemd/*.conf.d/`. Raspberry Pi OS ships
`40-rpi-volatile-storage.conf` and `40-rpi-enable-watchdog.conf`, so this
repository's drop-ins are named `99-ai-drone-*.conf` to sort after them. A
`10-` prefix silently loses to the vendor files.

### Why the watchdog is not tightened

Raspberry Pi OS already arms the watchdog at 1 minute. The setup script pins
that same value rather than shortening it: a Zero 2 W running the camera and
inference has little headroom, and a spurious watchdog reboot mid-recording
costs more than the extra seconds of hang detection would save. Pinning it
keeps the watchdog armed even if the vendor default changes.

### Why the root filesystem stays writable

An overlay read-only root is the only configuration that makes the rootfs
immune to a power cut, and it was considered and rejected. `drone-deploy`
writes the runtime into `~/ai-drone`, and the recorders write datasets into
`artifacts/`; both would land in a RAM overlay and vanish on reboot. Persistent
logs would also be lost, which defeats measure 2. On a 415 MiB machine already
running camera plus inference, the overlay's RAM cost is not affordable.

## Upgrading the Pi

```bash
sudo scripts/pi-safe-upgrade.sh --check-only   # preflight only
sudo scripts/pi-safe-upgrade.sh                # preflight, then upgrade
```

The script refuses to start when the flight-controller link is in use, when
`vcgencmd get_throttled` reports anything but `0x0`, when less than 2 GiB is
free, or when `dpkg` already reports incomplete packages. It then backs up the
NetworkManager profiles into a `0700` directory, **arms the recovery unit
before touching the package database**, upgrades, and disarms only once
`dpkg --audit` and `apt-get check` both pass.

Run it on a bench supply, never on the airframe battery.

## Recorded-data durability

Recording artifacts have an explicit durability policy in
`ai_drone/durability.py`, because the operating system's default write-back
timing loses whatever the card has not yet persisted.

- **Small, infrequent artifacts** — manifests, parameter snapshots, config
  exports — are replaced through `atomic_write_text`, which writes a temporary
  file, fsyncs it, renames it into place, and then fsyncs the parent directory.
  The directory sync is what makes the rename itself survive a power cut. A
  reader never observes partial contents.
- **Append-only streams** — `telemetry.jsonl` and `camera.jsonl` — flush every
  record and fsync at most once per `--sync-interval` (default 5 s). An fsync
  per record would stall capture on an SD card, so the policy trades a bounded
  loss window for recording throughput. Every stream is also synced once when
  it closes, including when a worker fails or the capture stops early.

`--sync-interval 0` disables periodic syncing for maximum throughput; the
closing sync still runs. Each dataset's `manifest.json` records the interval it
was captured with under `durability`.

A power cut therefore costs at most the last `--sync-interval` seconds of
JSONL, and the loss lands at a record boundary rather than mid-record.

## Verification

```bash
systemctl is-enabled apt-daily.timer apt-daily-upgrade.timer   # masked
journalctl --list-boots                                        # more than one boot
systemctl show -p RuntimeWatchdogUSec                          # 1min
sudo tune2fs -l "$(findmnt -no SOURCE /)" | grep 'Errors behavior'
```

## Reboot verification, 2026-08-19

A controlled reboot at 13:57 confirmed both items that were previously pending:

1. **Journal persistence across a reboot — confirmed.**
   `journalctl --list-boots` now lists the previous boot (`-1`) alongside the
   current one (`0`), and the previous boot's shutdown sequence is readable.
   The on-disk journal is 20 MB, under the 64 MB cap.
2. **`errors=remount-ro` on the live mount — confirmed.** The superblock reports
   `Errors behavior: Remount read-only`, and `/proc/mounts` shows `rw,noatime`
   with no `errors=` option. ext4 prints `errors=` only when the live behavior
   differs from the superblock default, so its absence is the signature of the
   mount having adopted remount-ro. Before the reboot the same command printed
   `errors=continue`, because the running mount still disagreed with the
   freshly written superblock.

The Pi came back on Tailscale within about five seconds, all five measures
survived the reboot, and no vehicle-control unit started.

## Still not validated

None of this has been exercised against a **real power cut**. The measures are
designed from the failure recorded above and verified by a clean reboot, not by
a repeat of the original event.
