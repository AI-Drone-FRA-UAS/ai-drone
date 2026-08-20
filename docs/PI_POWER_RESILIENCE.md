# Raspberry Pi power-loss resilience

The airframe-powered Pi can lose power without a clean shutdown. An unattended
package upgrade was interrupted on 2026-08-18 and left packages and boot files
partially written. The reconstruction and recovery evidence belong to the
[2026-08-19 state capture](../state/2026-08-19/README.md); this guide records
the maintained safeguards and operating procedure.

## Safeguards

Apply or reapply them on the Pi with:

```bash
sudo scripts/setup-pi-power-resilience.sh
```

The idempotent script supports `--dry-run` and `--revert` and configures:

| Measure | Purpose |
| --- | --- |
| Mask `apt-daily.timer` and `apt-daily-upgrade.timer` | Prevent unattended upgrades during an arbitrary power cut |
| Persistent journal capped at 64 MB | Preserve bounded post-mortem logs |
| One-minute hardware watchdog | Recover from a wedged Pi without tightening the vendor timeout |
| Root filesystem `errors=remount-ro` | Stop writes after detected filesystem corruption |
| Interrupted-upgrade recovery unit | Repair an upgrade interrupted after it was deliberately started |

The setup also verifies that no systemd unit, timer, or crontab starts a path
that talks to the flight controller at boot. Preserve that invariant.

The root filesystem remains writable because deployment, recordings, and
persistent logs need durable storage. A RAM overlay would lose those changes
and consume scarce memory on the Zero 2 W.

## Package upgrades

Never re-enable the unattended apt timers. Upgrade only on a bench supply:

```bash
sudo scripts/pi-safe-upgrade.sh --check-only
sudo scripts/pi-safe-upgrade.sh
```

The script refuses to proceed when the flight-controller link is in use, the
Pi reports throttling/undervoltage, free space is insufficient, or `dpkg` is
already incomplete. It backs up NetworkManager profiles, arms recovery before
touching the package database, and disarms recovery only after `dpkg --audit`
and `apt-get check` pass.

## Recording durability

`ai_drone/durability.py` defines the write policy:

- Small manifests, configuration exports, and snapshots use atomic replace
  with file and parent-directory syncing.
- Append-only JSONL streams flush every record and fsync at most once per
  `--sync-interval` (default five seconds), then sync again on close.

This bounds the normal power-cut loss window without forcing an expensive SD
card sync for every record. `--sync-interval 0` disables periodic syncing but
still syncs on a clean close. Each dataset manifest records the selected value.

## Verification

After setup or a controlled reboot:

```bash
systemctl is-enabled apt-daily.timer apt-daily-upgrade.timer
journalctl --list-boots
systemctl show -p RuntimeWatchdogUSec
sudo tune2fs -l "$(findmnt -no SOURCE /)" | grep 'Errors behavior'
```

Expected results are masked apt timers, journal entries spanning boots, a
one-minute watchdog, and `Remount read-only` filesystem error behavior.

These measures were verified across a controlled reboot on 2026-08-19. They
have not been validated by deliberately cutting power; do not perform such a
test without explicit authorization and a recovery plan.
