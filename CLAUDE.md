# Repository guidance

## Environment and checks

Use uv. The development interpreter is pinned by `.python-version`; supported
project metadata is Python `>=3.11,<3.14`. The Pi uses Debian Python 3.13 plus
apt-installed Picamera2/libcamera bindings. Do not widen the version range
without validating the Pi-native camera packages.

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

`lint-imports` enforces the `.importlinter` contracts that encode the layering
below; `deptry` checks declared versus imported dependencies. Both must stay
green. Their configuration carries the reason for every ignore — extend the
reasons rather than widening the ignores.

The base dependency set is intentionally laptop-light. Pi vision dependencies
live in the `raspi` group. Picamera2, libcamera, gpiozero, and other Pi-native
packages are imported lazily inside Pi-only execution paths; do not move them
to module scope merely to simplify annotations. Keep `default-groups = []`.

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
- `ai_drone/flight/`: `controller.py` owns the MAVLink connection and the
  arm/mode/takeoff/velocity/land transitions; `guards.py` owns the
  `FlightController` contract and the battery, altitude-ceiling, and
  telemetry-staleness guards that every flight mode must apply.
  `cli/control.py` is their only adapter.
- `ai_drone/config/`: `snapshot.py` / `sync.py`, deterministic configuration
  capture and host synchronization.
- Shared leaves at package root, imported by everything and importing nothing
  internal: `durability.py` (the canonical atomic-write and bounded-fsync
  helpers), `validation.py` (the canonical finite/range number checks),
  `platform.py`, `cli_parsing.py`, and `recording.py`.
- `tests/`: offline tests. Hardware-specific behavior must be expressed behind
  injectable boundaries so it remains testable without a Pi or vehicle.
- `attic/`: retired code kept for reference. Not maintained, linted, type
  checked, tested, or deployed. Retire code here instead of deleting it.

Dependencies run one way: `cli/` → concern packages → shared leaves. A shared
leaf must not import a concern package, and no concern package may import
`cli/`. There are no import cycles in `ai_drone`; keep it that way.

There are no repo-root Python or shell compatibility wrappers. Use package
entry points and `scripts/` directly; do not add duplicate wrappers.

## Safety

- Treat live vehicle access as disarmed and read-only unless the user asks for
  an actuator test.
- Never arm, fly, or run a motor test while optional pre-arm checks are
  bypassed (`ARMING_CHECK` other than exactly `1`). `drone-motor-test` enforces
  this in code; do not weaken that gate.
- Sensor, camera, and recording paths must not send arm, disarm, flight-mode,
  throttle, RC override, mission-start, motor, servo, or parameter-write
  commands.
- Filter MAVLink state by the selected vehicle source, reject armed heartbeats,
  use monotonic absolute deadlines, and close serial, camera, and output
  resources in `finally` blocks.
- No systemd unit, timer, or crontab on the Pi may start anything that talks to
  the flight controller at boot. An unexpected reboot must never bring up a
  command path to the vehicle.
- Never re-enable `apt-daily*.timer` on the Pi: an unattended upgrade
  interrupted by a power cut already corrupted this Pi once. `pi-safe-upgrade.sh`
  is the only sanctioned way to upgrade its packages.
- Metric output from vision requires a rigid mount and a supplied calibration.
  Without both, keep the output non-metric rather than inventing a scale.

## Hardware and access facts

- Flywoo GOKU GN745 AIO, ArduPilot Copter 4.6.3.
- Pi Zero 2 W companion link: FC UART4 to Pi `/dev/serial0`, MAVLink2 at
  115200 baud.
- Downward MTF-01P flow/range sensor: FC UART5, MAVLink1 at 115200 baud.
- Preferred Pi access when Tailscale is online:
  `ssh -F /dev/null seb@seb-is-pm` (MagicDNS).
- Pi hotspot fallback: `ssh -F /dev/null seb@192.168.4.1`.
- Direct connection and health tools default to the platform null SSH config;
  preserve `SSH_CONFIG` propagation so a broken host config cannot block them.

Do not put passwords, Wi-Fi/eduroam credentials, tokens, private keys, or real
secret defaults in source, documentation, command lines, logs, deployment
archives, or state captures.
