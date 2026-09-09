# Connection status and disconnect preparation

From the Linux laptop's project directory:

```bash
uv run --locked drone-power status
```

The command reports Pi SSH access, active walkthrough/package jobs and hardware
owners. It checks the project's direct FC USB port first and uses the Pi UART
only when that fallback is idle. FC observations use passive reception at
115200 baud: selected system/component 1/1, fresh disarmed heartbeats, and any
spontaneous battery or `POWER_STATUS` messages. It sends no MAVLink requests,
mode changes or actuator commands. Missing or busy data stays unavailable.
Each removal has an explicit `blocked` or `needs Pi shutdown` status and its
preparation command. Status exits nonzero if Pi safety checks or fresh disarmed
FC confirmation are unavailable; it never authorizes unplugging by itself.

The Pi's power-only USB port cannot be reliably detected in software. A working
USB network connection, FC battery voltage or `POWER_STATUS` flag does not prove
that another supply will keep the Pi powered after a cable is removed.

Choose the connection you intend to remove:

```bash
uv run --locked drone-power prepare battery
uv run --locked drone-power prepare fc-usb
uv run --locked drone-power prepare pi-usb
uv run --locked drone-power prepare all
```

These are alternatives; run only the command matching your intended removal.
Add `--dry-run` to preview the checks without contacting either device.
Preparation refuses active package work, incomplete package state, unknown
camera/serial/GPIO owners, and missing or armed FC confirmation. It stops only
the known `ai-drone-walk.service`, using its configured SIGINT cleanup, and
requires that invocation's report to exist before proceeding. Other hardware
tools must be closed by their operator.

Immediately before confirming idle state or requesting shutdown, the Pi must
independently receive fresh disarmed heartbeats on its idle UART. Final package
and hardware-owner checks share that heartbeat's two-second freshness deadline.
Missing UART telemetry or a delayed final check blocks preparation even if an
earlier laptop USB observation was disarmed.

By default, preparation requests a Pi shutdown for every removal choice because
the remaining power path is unknown. Keep power connected until the Pi has
physically finished shutting down. The helper explicitly reports only that
shutdown was requested; losing SSH is not confirmation of a clean halt.
Then remove the selected cable or battery. The helper cannot unplug anything.

If you have just verified that an independent supply will remain connected to
the Pi after the chosen removal, add this per-command confirmation:

```bash
uv run --locked drone-power prepare battery --pi-power-independent
```

This example finishes the recorder and checks the drone but leaves the Pi
running on the supply you confirmed. The same flag can be used for `fc-usb`
or `pi-usb`; it is rejected with `all`. The confirmation is never saved for a
future invocation. USB power can keep the Pi running while battery-powered FC
sensors become unavailable after battery removal.

Both status and preparation require permission to inspect all relevant process
descriptors. The helper uses existing noninteractive sudo access for this
read-only inspection, and for the explicitly requested Pi stop/shutdown. It
does not install sudo rules or stop an unrelated process. Permission failures
are reported as unknown or blocked.
If the laptop cannot inspect USB port owners, it reports the detected USB
connection and that limitation. A separate nonprivileged `fuser` check must
confirm no visible owner before it tries the Pi UART with the Pi's full owner
checks. A known USB owner or an inconclusive `fuser` check blocks preparation.
Telemetry from the fallback is explicitly labeled `Pi UART`; the limited host
check does not establish that descriptors hidden by permissions are absent.

The default SSH route uses the full Tailscale hostname, with existing target
settings from `PI_HOST`, `PI_DIR` and `SSH_CONFIG`; known host-key verification
remains enabled. It can fall back to the configured hotspot or a configured
USB connection without changing networking. The Pi must have the same deployed
helper. See [networking](pi-networking.md), [sensor recording](SENSOR_RECORDING.md)
and [power-loss resilience](PI_POWER_RESILIENCE.md).
