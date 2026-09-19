# Setup and networking

## Deployment status

The last Pi observation was **September 16, 2026**. The new application source
and `ai-drone-runtime.service` were not installed. That day's changes disabled
the legacy automatic-hotspot service and AP autoconnect; `eduroam` retained
autoconnect and priority 400. Client startup was still managed by NetworkManager.

An isolated staged runtime passed short disarmed shared-telemetry/camera captures
and an authenticated operator check through reverse SSH. Direct inbound laptop
routes did not respond. The test processes and laptop responder were stopped.
These saved results do not establish the current Pi state or validate installation,
hotspot switching, reboot recovery, servo motion or flight.

The following instructions describe this checkout after deployment. On reconnect,
check the actual state before applying changes; current work is local only.

## Connect

Use saved client Wi-Fi and ordinary SSH, preferably through Tailscale. Project
commands never change the laptop's Wi-Fi:

```bash
ssh seb@seb-is-pm
uv run drone connect
# Explicit local route when needed:
PI_HOST=seb@192.168.4.1 uv run drone connect
```

The last address requires a manually started Pi hotspot and a laptop already
joined to it. A shared Tailscale device may need this laptop SSH alias:

```sshconfig
Host seb-is-pm
    HostName seb-is-pm.tail59e6a4.ts.net
    User seb
```

Keep personal SSH key settings. Tailscale sharing and network policy must permit
TCP 22; a working direct LAN route needs neither Tailscale nor internet. The
[teammate-access record](../state/2026-09-07/team-access.md) retains previous checks.

## Environment and deployment

Use `uv` on every machine. The Pi uses Debian Python with apt-provided
Picamera2/libcamera, gpiozero and native AprilTag; its environment must retain
access to system packages. Laptop development needs none of those hardware
packages. Versions are defined by `pyproject.toml` and `uv.lock`.

```bash
# On the laptop:
uv sync --locked
uv run drone deploy --dry-run
uv run drone deploy
```

Deployment requires an existing Pi checkout and `.venv` with `pymavlink`.
For initial Pi setup, after installing the native packages:

```bash
uv venv --python /usr/bin/python3 --system-site-packages .venv
uv sync --locked --group raspi --python .venv/bin/python
```

Deploy only while grounded with no recording or control job. Every laptop uses
the same tar upload into private staging. The Pi obtains a maintenance lease,
stops an installed access service and backs up source plus the environment before
applying the update. File/dependency installation failure restores both before
restarting the previously active service. After restart, live status must show
fresh selected-FC telemetry and no reader/network error before the backup is
removed. A readiness failure retains the printed backup without interrupting a
newly active client; inspect the error before retrying. The transaction checks
fresh disarmed telemetry before changing files and starts no flight or recording.
It has local regression coverage and still needs its first live Pi validation.

`--offline` uses cached packages only. `PI_HOST`, `PI_USER`, `PI_DIR` and
`SSH_CONFIG` override the [local connection settings](OPERATIONS.md#local-settings).
Per-machine `drone.toml`, credentials and recordings remain outside deployment.
Installing the access service is a separate step.

## Shared access service

Once installed, `ai-drone-runtime.service` is the single physical FC reader. It starts at boot
and provides local telemetry subscriptions, operator-presence status and grounded
network selection. Checks, recording and control share `/run/ai-drone/vehicle.sock`;
recording and flight jobs start only by explicit command.

Create `/home/seb/ai-drone/drone.toml` from the example, provision saved client
networks, and review the installer on the Pi:

```bash
uv run python scripts/setup_runtime.py \
  --project /home/seb/ai-drone --config /home/seb/ai-drone/drone.toml
sudo /home/seb/.local/bin/uv run --no-sync python scripts/setup_runtime.py \
  --project /home/seb/ai-drone --config /home/seb/ai-drone/drone.toml --apply
uv run drone runtime status
systemctl status ai-drone-runtime.service
```

Without `--apply`, setup only previews its changes. Installation requires fresh
selected-FC disarmed telemetry and no active recording/control clients. It backs
up NetworkManager profiles, eligibility and service state under root-only
`/var/backups/ai-drone/`, retires the old automatic-hotspot selector, and retains
the current client connection. To restore a printed backup, repeat the same
project/config arguments with `--revert /var/backups/ai-drone/BACKUP`; add
`--apply` after reviewing the rollback. Rollback requires fresh disarmed idle
state and refuses to interrupt a new recording/control owner. The backup remains
available if safe rollback is blocked. This restores service files and Wi-Fi
startup flags; source/environment rollback belongs to deployment above.

The service runs as `seb`, with its socket/status directory private (0700).
Its root-owned eligibility file is readable (0644); backup files and the operator
key stay private. The installer checks service-user access to configuration and key.
Its network mutations use the Pi's existing passwordless sudo. For a hardware-free
runtime, use `drone runtime serve --no-network --device SIMULATOR_ENDPOINT`
with private socket/status paths in local configuration.

## Client Wi-Fi and manual hotspot

While grounded, provision a known client network and optionally a phone hotspot
as a client fallback. Keep credentials and enterprise CA/server-name validation
in NetworkManager:

```bash
sudo nmcli device wifi connect "SSID" --ask
uv run python scripts/network.py list
uv run python scripts/network.py status
```

Give `eduroam` the highest configured client priority. The installer records
originally eligible client UUIDs in `/etc/ai-drone/network-profiles.json`, then
sets **all Wi-Fi profile autoconnect flags off**. This is intentional: the
runtime alone requests a connection after a fresh disarmed FC observation,
including at boot. Rerun setup after provisioning additional client profiles.

A working connection stays in place. After actual link loss or expiration of
the configured [operator heartbeat](OPERATIONS.md#operator-heartbeat), grounded
failover tries eligible reachable clients by priority, rotating failed attempts
with a 15-second cooldown. It never selects an AP. Armed, stale or unknown FC
state blocks switching; an absent FC may require manual USB recovery. Internet
checks, Tailscale availability alone and SSH terminal closure are not failover
signals. Keep the aircraft grounded for a switch; independent RC arming cannot
be atomic with a radio transition.

Prepare the hotspot profile without activating it:

```bash
sudo scripts/setup-pi-hotspot.sh
# Alternatively: --password-file /root/hotspot-passphrase
uv run python scripts/network.py hotspot on
# From the laptop after manually joining the hotspot:
uv run python scripts/network.py --host seb@192.168.4.1 hotspot off
# After saved-client access returns, request a particular profile if needed:
uv run python scripts/network.py connect eduroam
```

It serves `AI-Drone-Zero` at `192.168.4.1/24` on the single `wlan0` radio.
Hotspot and client mode are mutually exclusive. An explicit hotspot remains
until stopped or rebooted; the next client selection again requires disarmed
telemetry. Failure to reach any saved network never starts the hotspot.

The helper runs locally on the Pi or sends the same request over SSH. The access
service completes queued requests after that SSH session disappears. Disarmed
recording may continue through a switch; active control holds an exclusion lease.
The helper never opens a second UART reader or bypasses a missing runtime.
Verify explicit hotspot on/off and reboot-to-client recovery on the actual Pi
before relying on a newly installed configuration. Establish an independent
recovery route first; the September 16 session did not have a connected USB route.

## Manual USB recovery

Use the Pi data port labelled `USB`, a data-capable cable, and an identified
laptop adapter. The prepared gadget uses Pi `192.168.7.2/24` and host
`192.168.7.1/24`, without a gateway or DNS:

```bash
USB_IFACE=usb0 uv run python scripts/usb_ssh.py --dry-run
USB_IFACE=usb0 uv run python scripts/usb_ssh.py
```

Find the adapter with Linux `ip link`, macOS `networksetup -listallhardwareports`,
or Windows adapter settings. On Windows use an elevated PowerShell when changing
its address and set `$env:USB_IFACE = "Ethernet 4"` to the verified name.
Manual addressing plus `ssh seb@192.168.7.2` also works. This recovery route is
never configured automatically by `drone connect`.

The prepared [usb0-static.service](../scripts/usb0-static.service) delays
`g_ether` until normal modules load. Preserve `modules-load=dwc2` in
`/boot/firmware/cmdline.txt`, the peripheral overlay, and saved gadget MACs in
root-only `/etc/modprobe.d/ai-drone-usb-gadget.conf`. Avoid a second boot-time
`g_ether` load. Back up boot files before changes and verify repeated reboot/SSH
recovery; see the [dated recovery evidence](../state/2026-08-19/README.md).

## Pi maintenance

Use stable bench power, disarmed idle state, and no recording/package job:

```bash
sudo scripts/setup-pi-power-resilience.sh --dry-run
sudo scripts/setup-pi-power-resilience.sh
sudo scripts/pi-safe-upgrade.sh --check-only
sudo scripts/pi-safe-upgrade.sh
```

Resilience setup supports `--revert`. It masks unattended apt timers, limits
persistent journals to 64 MB, enables a one-minute watchdog, and prepares
interrupted-upgrade recovery. Keep unattended upgrades disabled. Read back
`systemctl is-enabled apt-daily.timer apt-daily-upgrade.timer`,
`journalctl --list-boots` and `systemctl show -p RuntimeWatchdogUSec` after changes.
These measures do not make power cuts safe; use the
[shutdown procedure](OPERATIONS.md#finish-and-power-down).
