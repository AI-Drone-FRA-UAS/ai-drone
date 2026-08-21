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
- Never arm, fly, or run a motor test with the pre-arm checks disabled or set
  to an arbitrary subset. Exactly two `ARMING_CHECK` values are permitted:
  `1` (every check) and `1043958` (every check except GPS lock and GPS
  configuration). The second exists because this airframe carries no GPS
  receiver and the project's purpose is indoor flight without one; every other
  check stays enabled and stays able to report. `ARMING_CHECK=0` is never
  acceptable: in that state the vehicle reports no `PreArm:` failure at all and
  cannot say what is wrong with it. `ai_drone.mavlink.arming_checks` is the
  single source of this policy and `drone-motor-test`, `DroneController`, and
  `drone-control preflight` all enforce it; do not widen it per call site.
- Sensor, camera, and recording paths must not send arm, disarm, flight-mode,
  throttle, RC override, mission-start, motor, servo, or parameter-write
  commands.
- Never release the link to an aircraft this software put in the air. A LAND
  request is not a landing: re-request it and wait for the vehicle's own
  heartbeat to report disarmed before closing the connection or exiting.
  `DroneController.ensure_landed` is that wait, and every teardown path must go
  through it. On 2026-08-20 an unverified LAND plus an immediate disconnect left
  an airborne aircraft with nobody commanding it; it was stopped by pulling the
  battery.
- A LAND request is an altitude-controlled mode, so it is only as safe as the
  vehicle's altitude estimate. On 2026-08-21 the EKF reported an altitude of
  -10000 m and a descent of 38 m/s while the aircraft sat on the floor. The
  aircraft was fine for thirteen seconds in STABILIZE, which ignores the EKF
  entirely; the moment this software requested LAND, the altitude controller
  read that estimate, went to full throttle in a single log sample, and flew
  the aircraft into a ceiling in one second. `emergency_stop`, `abort_to_land`,
  `ensure_landed`, and every guard in `flight/guards.py` route to LAND. Before
  requesting it, check that the vertical estimate is sane -- and when the
  aircraft is demonstrably still on the ground, disarm instead.
- Do not report a flight as started before the aircraft has left the ground.
  `climb_in_stabilize` and `takeoff_without_position` both set
  `_flight_started_by_controller` and `is_flying` immediately after arming, so
  a climb that never lifts is treated as an airborne emergency and lands, when
  the rangefinder has been reading 0.02 m throughout and the safe ending is a
  disarm.
- An EKF status flag is not an EKF estimate. `preflight` passed
  `vertical_position` on 2026-08-21 by reading the flag that says a vertical
  estimate exists, while the estimate itself was -10000 m. Check values, not
  only the bits that claim the values are available.
- A rehearsal against `ai_drone.sim.vehicle` proves the code matches the
  simulator, and the simulator encodes whatever was assumed when it was
  written. It cannot validate an assumption about how the vehicle interprets a
  message. Where a protocol assumption could produce dangerous motion, add a
  check that measures the aircraft instead of trusting the request.
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

## Hardware and access

Treat `AGENTS.md` as the concise access and hardware-safety reference. Stable
topology is in `docs/DRONE_CONFIGURATION.md`, networking in
`docs/pi-networking.md`, and live observations only in dated `state/` captures.
Preserve `SSH_CONFIG` propagation so a broken host SSH config cannot block the
connection and health tools.

Do not put passwords, Wi-Fi/eduroam credentials, tokens, private keys, or real
secret defaults in source, documentation, command lines, logs, deployment
archives, or state captures.
