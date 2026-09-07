# AI Drone

Software and guarded tools for a Raspberry Pi Zero 2 W companion, a FlywooF745
ArduPilot controller, downward MTF-01P range/optical flow, a forward MT-15 lidar,
an IMX500 camera, and a payload servo.

The implemented flight path is a bounded GPS-free takeoff, optical-flow Loiter
hold, and landing. Room navigation and calibrated tag approach remain planned
work. Keep live inspection disarmed; actuator and flight commands require
explicit authorization and their physical prerequisites.

The latest dated result is the [7 September 2026 repair and verification
report](state/2026-09-07/indoor-repair.md), with its [parameter
snapshot](params/flywoo-f745-live-2026-09-07.param). It records restored downward
range/flow and relative EKF aiding, an unresolved forward-lidar receive path,
and a compass pre-arm failure. Recheck hardware before use; a dated result is
not a continuing health guarantee.

## Start here

Install [uv](https://docs.astral.sh/uv/) and create the locked environment:

```bash
uv sync --frozen
uv run drone-connect --help
uv run drone-inspect --help
```

Python 3.11–3.13 is supported. The Pi uses Debian Python 3.13 with apt-installed
Picamera2/libcamera bindings; the laptop environment does not need Pi hardware
packages.

The helpers try Tailscale, hotspot, then Pi USB Ethernet:

```bash
uv run drone-connect
uv run drone-connect --transport hotspot
```

For deployment, select the reachable Pi address. This example uses Tailscale;
when joined to `AI-Drone-Zero`, use `PI_HOST=seb@192.168.4.1`:

```bash
PI_HOST=seb@seb-is-pm uv run drone-deploy
PI_HOST=seb@seb-is-pm uv run drone-deploy --run inspect -- --duration 15
```

Deployment alone starts no task. The inspection requests telemetry and records
available camera/sensor data without arming or moving anything. A battery may
be needed to power sensors. See [networking](docs/pi-networking.md) and the
separate procedures for [Pi USB Ethernet](docs/RPI_ZERO2W_USB_SSH_SETUP.md) and
[direct FC USB](docs/DEVELOPER_MACHINE_DRONE_CONNECTION.md).

## Commands

| Command | Purpose |
| --- | --- |
| `drone-connect` | Open Pi SSH through a selected or available transport |
| `drone-deploy` | Synchronize the runtime and optionally run an allowlisted task |
| `drone-inspect` | Record available disarmed camera and FC sensor streams |
| `drone-config-sync` | Capture a verified disarmed FC configuration |
| `drone-servo` | Guarded direct-BCM12 servo bench test |
| `drone-motor-test` | Guarded low-power, propeller-off motor check |
| `drone-control hover` | Guarded GPS-free takeoff, Loiter hold, and landing |
| `drone-tag-servo-record` | Explicit armed-flight tag recording and bounded servo pulses |

Use `uv run <command> --help` for authoritative options. Before any actuation,
follow the relevant [operating procedure](docs/index.md#operation). In
particular, normal arming checks must pass; simulation does not establish live
flight readiness, mechanical servo travel, or an emergency-control arrangement.

## Documentation and evidence

[The documentation map](docs/index.md) separates maintained procedures from
dated observations:

- **Hardware and configuration:** [inventory](docs/drone-project.md),
  [FC wiring and parameters](docs/DRONE_CONFIGURATION.md), [firmware](firmware/README.md).
- **Operation:** [sensor recording](docs/SENSOR_RECORDING.md),
  [GPS-free hover](docs/PI_MAVLINK_CONTROL.md), [planned AprilTag mission](docs/APRILTAG_MISSION.md).
- **History:** dated captures in `state/` and `params/`,
  [historical notes](notes/README.md), and [retired code](attic/README.md).

Raw recordings and local research/build artifacts remain under ignored
`artifacts/`; they are not automatically included in a Git checkout. Project
drone captures must never be replaced with another aircraft's configuration.

## Development checks

```bash
uv sync --frozen --group dev
uv run --frozen --group dev ruff format --check .
uv run --frozen --group dev ruff check .
uv run --frozen --group dev ty check .
uv run --frozen --group dev pytest -q
uv run --frozen --group dev lint-imports
uv run --frozen --group dev deptry .
git diff --check
```

The two opt-in simulator tests need the exact external ArduPilot checkout and
SITL binary; see the [pinned simulator acceptance
gate](docs/ARDUCOPTER_4_7_NOGPS_LOITER.md#exact-pinned-sitl-acceptance-gate).
Contributor architecture and hardware boundaries are documented in
[CLAUDE.md](CLAUDE.md) and [AGENTS.md](AGENTS.md).
