# Repository guidance

Read `docs/HANDOFF.md` before hardware, network, AprilTag, configuration,
motor, servo, flight, or GitHub work. Newer dated state captures and explicit
user statements override older documentation.

Commit `2c3fab2e637bac87ec131e83e82b12bb8131b1d5` is the functional-parity
baseline. Cleanup may consolidate wrappers and shared implementation, but must
not remove any user-visible capability present there or in later work.

## Safety boundary

Only disarmed sensing, recording, configuration export, and offline software
tests have been validated on the real vehicle. The package intentionally keeps
the complete flight-control capability, but consolidates it behind one
`drone-control` command. Its takeoff/hover, velocity, and person-follow paths
manage arming and landing as part of their bounded sequence. They are
implemented capabilities, not permission to use them on the current aircraft.
Exercise them in ArduPilot SITL first and retain their explicit preflight and
operator-confirmation gates.

Current physical/controller state:

- `ARMING_CHECK=0` on the live flight controller: optional pre-arm checks are
  bypassed. Never arm, fly, or run a motor test in this state.
- `FENCE_ENABLE=0`: there is no useful configured indoor containment.
- The camera is held only by a small cable and faces forward. Keep the vehicle
  stationary; it is not a flight-ready or calibrated nadir installation.
  Nearest-person output must remain non-metric unless an explicit project
  calibration is supplied; altitude geometry requires a rigid calibrated
  nadir mount and its exact acknowledgement.
- The payload servo is disconnected from the Pi. `drone-servo` controls Pi BCM
  GPIO 12 directly and is an actuator, not a MAVLink sensor test.
- The forward MT-15 rangefinder is disconnected and untested, and physical
  installation space is unconfirmed. Its proposed link is FC UART7, not a Pi
  UART. Do not apply `SERIAL7`/`RNGFND2` parameters based only on the proposal.
- The existing MTF-01P downward flow/range sensor is connected to FC UART5.
- Treat all live access as disarmed/read-only unless the user explicitly asks
  for an actuator test and confirms its documented physical prerequisites.

No systemd unit, timer, or crontab on the Pi may start anything that talks to
the flight controller at boot. An unexpected reboot must never bring up a
command path to the vehicle.

Sensor and camera tests must never send arm, disarm, flight-mode, throttle, RC
override, mission-start, motor, servo, or parameter-write commands. Always
filter MAVLink state by the selected vehicle source, reject armed heartbeats,
use monotonic absolute deadlines, and close serial/camera/output resources in
`finally` blocks.

## Environment and checks

Use uv. The development interpreter is pinned by `.python-version`; supported
project metadata is Python `>=3.11,<3.14`. The Pi uses Debian Python 3.13 plus
apt-installed Picamera2/libcamera bindings. Do not widen the version range
without validating the Pi-native camera and inference packages.

```bash
UV_CACHE_DIR=/tmp/uv-cache uv sync --frozen --group dev
UV_CACHE_DIR=/tmp/uv-cache uv run --frozen --group dev ruff format --check .
UV_CACHE_DIR=/tmp/uv-cache uv run --frozen --group dev ruff check .
UV_CACHE_DIR=/tmp/uv-cache uv run --frozen --group dev ty check .
UV_CACHE_DIR=/tmp/uv-cache uv run --frozen --group dev pytest -q
UV_CACHE_DIR=/tmp/uv-cache uv run --frozen --group dev lint-imports
UV_CACHE_DIR=/tmp/uv-cache uv run --frozen --group dev deptry .
git diff --check
```

`lint-imports` enforces the `.importlinter` contracts that encode the package
layering described under *Code organization*; `deptry` checks declared versus
imported dependencies. Both must stay green. Their configuration carries the
reason for every ignore — extend the reasons rather than widening the ignores.

The base dependency set is intentionally laptop-light. Pi vision dependencies
live in the `raspi` group. Picamera2, libcamera, gpiozero, and other Pi-native
packages are imported lazily inside Pi-only execution paths; do not move them
to module scope merely to simplify annotations. Keep `default-groups = []`.

## Maintained commands

- `drone-health`: read-only heartbeat plus `SYSID_THISMAV` round-trip on laptop
  USB and/or Pi `/dev/serial0`.
- `drone-lidar`: disarmed recording of existing MTF-01P range/flow telemetry.
- `drone-record`: disarmed camera, AprilTag, and MAVLink dataset capture.
- `drone-apriltag`: stationary vision diagnostics only.
- `drone-config-export` / `drone-config-sync`: read-only parameter capture;
  only `drone-config-sync --publish` intentionally writes to Git.
- `drone-console`: launches an interactive MAVProxy console. MAVProxy is
  hardware-capable, so its presence is not permission to send commands.
- `drone-motor-test`: guarded propeller-off bench utility. It must remain
  blocked unless `ARMING_CHECK` is exactly `1` (all checks) and retain exact
  physical confirmations plus a fresh source-filtered disarmed heartbeat
  immediately before the command.
- `drone-servo`: direct Pi GPIO actuator utility. It must retain an explicit
  mode, exact clearance confirmation, finite limits, and cleanup behavior.
- `drone-control`: the single flight-control CLI. It owns passive `status`,
  `hover`/`takeoff`, `velocity-test`, and `follow`; flight modes own their
  arming, stop, landing, and cleanup sequence. Keep these capabilities
  available, but never create duplicate per-mode entry points. All live
  actuator/flight paths require explicit confirmation and must fail closed when
  their health, controller, localization, or physical prerequisites are absent.
- `autoconnect` / `manuconnect`: choose Tailscale, the Pi access point, or USB.
  Transport selection belongs in `link/connect.py`; `link/usb_ssh.py`
  implements only the USB transport.
- `drone-deploy`: synchronize the explicit runtime allowlist, create a Pi venv
  with `--system-site-packages`, install the `raspi` group, and optionally run
  one selected mode. Preserve secret exclusions and remote-path validation.

Pi system hardening lives in `scripts/`, not in a command:
`setup-pi-power-resilience.sh` applies the power-loss measures (masked apt
timers, persistent journal, pinned watchdog, `errors=remount-ro`, recovery
unit) and is idempotent with `--dry-run`/`--revert`; `pi-safe-upgrade.sh` is
the only sanctioned way to upgrade Pi packages. Never re-enable
`apt-daily*.timer`: an unattended upgrade interrupted by a power cut already
corrupted this Pi once. Keep both in `deploy.RUNTIME_FILES`.

There are no maintained repo-root Python or shell compatibility wrappers. Use
package entry points and `scripts/` directly; do not add duplicate wrappers.

## Code organization

The package is grouped by concern. Subpackages carry the namespace, so module
names inside them do not repeat it (`link/wifi.py`, not `link/pi_wifi.py`).

- `ai_drone/cli/`: thin hardware-facing command adapters. Nothing outside
  `cli/` may import from `cli/`.
- `ai_drone/link/`: getting the development machine to the Pi.
  `connect.py` owns transport selection (Tailscale, access point, USB);
  `wifi.py` and `usb_ssh.py` implement one transport each; `targets.py` holds
  the shared host/address facts; `deploy.py` pushes the runtime allowlist.
  These are cross-platform (Linux, macOS, Windows): keep the pure command
  builders separate from the subprocess calls so both stay testable offline.
- `ai_drone/mavlink/`: everything that speaks to the flight controller but is
  not flight control. `devices.py` and `safety.py` hold the shared endpoint,
  source, and armed-state rules — do not duplicate MAVLink bit decoding in
  CLIs. `parameters.py`, `console.py`, and `health.py` build on them.
- `ai_drone/vision/`: `apriltags.py` and `stream.py`, camera-facing logic with
  hardware imports kept lazy.
- `ai_drone/flight/`: `controller.py` and `follower.py`, the MAVLink flight
  control and person-follow state. `cli/control.py` is their only adapter.
- `ai_drone/config/`: `snapshot.py` / `sync.py`, deterministic configuration
  capture and host synchronization.
- Shared leaves at package root, imported by everything and importing nothing
  internal: `durability.py` (the canonical atomic-write and bounded-fsync
  helpers — artifacts must be written through it; do not add a second
  implementation), `platform.py`, `cli_parsing.py`, and `recording.py`.
- `tests/`: offline tests. Hardware-specific behavior must be expressed behind
  injectable boundaries so it remains testable without a Pi or vehicle.

Dependencies run one way: `cli/` → concern packages → shared leaves. A shared
leaf must not import a concern package, and no concern package may import
`cli/`. There are no import cycles in `ai_drone`; keep it that way.

Prefer small typed functions, immutable dataclasses for validated settings,
finite numeric validation at CLI/file boundaries, bounded queues and shutdowns,
atomic artifact writes via `durability.atomic_write_text`, deterministic
serialization, and one canonical helper for each behavior. Avoid broad
exception suppression and loops whose timeout is reset by irrelevant input. Keep public behavior documented and delete dead
compatibility code instead of maintaining two implementations.

## Hardware and access facts

- Flywoo GOKU GN745 AIO, ArduPilot Copter 4.6.3.
- Pi Zero 2 W companion link: FC UART4 to Pi `/dev/serial0`, MAVLink2 at
  115200 baud.
- Downward MTF-01P: FC UART5, MAVLink1 at 115200 baud.
- Preferred Pi access when Tailscale is online:
  `ssh -F /dev/null seb@seb-is-pm` (MagicDNS).
- Pi hotspot fallback: `ssh -F /dev/null seb@192.168.4.1`.
- Direct connection and health tools default to the platform null SSH config;
  preserve `SSH_CONFIG` propagation so a broken host config cannot block them.

Do not put passwords, Wi-Fi/eduroam credentials, tokens, private keys, or real
secret defaults in source, documentation, command lines, logs, deployment
archives, or state captures. A sandboxed GitHub/SSH authentication failure may
be misleading; follow `AGENTS.md` and retry the read-only check with the
required elevated network permission before reporting it unavailable.
