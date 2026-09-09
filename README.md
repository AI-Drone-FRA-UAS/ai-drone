# AI Drone

Software and guarded tools for a Raspberry Pi Zero 2 W companion, a FlywooF745
ArduPilot controller, downward MTF-01P range/optical flow, a forward MT-15 lidar,
an IMX500 camera, and a payload servo.

The implemented flight path is a bounded GPS-free takeoff, optical-flow Loiter
hold, and landing. Room navigation and calibrated tag approach remain planned
work. Keep live inspection disarmed; actuator and flight commands require
explicit authorization and their physical prerequisites.

The [9 September firmware and power update](state/2026-09-09/firmware-and-power.md)
records the installed custom ArduCopter 4.7.1 build, connection/shutdown helpers
and live checks. All four ESCs have the [WAV 34 startup melody](state/2026-09-09/startup-tone.md).
The [MT-15 integration report](state/2026-09-09/mt15-integration.md)
records forward lidar reaching the FC and Pi alongside downward range/flow. The earlier
[repair report](state/2026-09-07/indoor-repair.md) records relative EKF aiding
and the unresolved compass pre-arm issue. Recheck hardware before use; sensing
and simulation results do not establish flight readiness.

The [consolidation report](state/2026-09-07/consolidation.md) records the shared
`main` branch, Pi deployment and restored connectivity. The
[team-access follow-up](state/2026-09-07/team-access.md) verifies shared SSH
permissions and eduroam preference; use the
[SSH alias setup](docs/pi-networking.md#shared-teammate-ssh-access) for
`ssh seb@seb-is-pm`. See [CONTRIBUTING.md](CONTRIBUTING.md) before starting new
development work.

## Start here

Install [uv](https://docs.astral.sh/uv/) and create the locked environment:

```bash
uv sync --frozen
uv run --locked drone-connect --help
uv run --locked drone-inspect --help
```

Python 3.11–3.13 is supported. The Pi uses Debian Python 3.13 with apt-installed
Picamera2/libcamera bindings; the laptop environment does not need Pi hardware
packages.

Use `uv run --locked <command>` for project commands; uv manages `.venv`
without requiring activation. On the deployed Pi, include `--group raspi`
to retain its vision dependencies, for example
`uv run --locked --group raspi drone-walk --duration 30`.
The detached worker keeps the selected interpreter for the whole recording;
finish the job before syncing or updating its environment.

The helpers try Tailscale, hotspot, then Pi USB Ethernet:

```bash
uv run --locked drone-connect
uv run --locked drone-connect --transport hotspot
```

For deployment, select the reachable Pi address. This example uses Tailscale;
when joined to `AI-Drone-Zero`, use `PI_HOST=seb@192.168.4.1`:

```bash
PI_HOST=seb@seb-is-pm.tail59e6a4.ts.net uv run --locked drone-deploy
PI_HOST=seb@seb-is-pm.tail59e6a4.ts.net uv run --locked drone-deploy --run inspect -- --duration 30
```

Deployment alone starts no task. The inspection requests telemetry and records
available camera/sensor data without arming or moving anything. A battery may
be needed to power sensors. See [networking](docs/pi-networking.md) and the
separate procedures for [Pi USB Ethernet](docs/RPI_ZERO2W_USB_SSH_SETUP.md) and
[direct FC USB](docs/DEVELOPER_MACHINE_DRONE_CONNECTION.md).

For a timed, hand-carried recording with detached capture, CSV exports and an
offline browser report, follow the [room walkthrough procedure](docs/SENSOR_RECORDING.md#timed-room-walkthrough).

## Commands

| Command | Purpose |
| --- | --- |
| `drone-connect` | Open Pi SSH through a selected or available transport |
| `drone-deploy` | Synchronize the runtime and optionally run an allowlisted task |
| `drone-inspect` | Record available disarmed camera and FC sensor streams |
| `drone-walk` | Start a timed, detached disarmed Pi recording and report |
| `drone-power status` | Check connections and show how to prepare each power/cable removal |
| `drone-power prepare` | Finalize the recorder and prepare a selected disconnection |
| `drone-check` | Read-only FC firmware, configuration and sensor checks |
| `drone-report` | Export an existing dataset to CSVs and an offline browser report |
| `drone-config-sync` | Capture a verified disarmed FC configuration |
| `drone-servo` | Guarded direct-BCM12 servo bench test |
| `drone-motor-test` | Guarded low-power, propeller-off motor check |
| `drone-control hover` | Guarded GPS-free takeoff, Loiter hold, and landing |
| `drone-tag-servo-record` | Explicit armed-flight tag recording and bounded servo pulses |

Use `uv run --locked <command> --help` for authoritative options. Before any actuation,
follow the relevant [operating procedure](docs/index.md#operation). In
particular, normal arming checks must pass; simulation does not establish live
flight readiness, mechanical servo travel, or an emergency-control arrangement.

## Documentation and evidence

[The documentation map](docs/index.md) separates maintained procedures from
dated observations:

- **Hardware and configuration:** [inventory](docs/drone-project.md),
  [FC wiring and parameters](docs/DRONE_CONFIGURATION.md), [firmware](firmware/README.md).
- **Operation:** [sensor recording](docs/SENSOR_RECORDING.md),
  [connection and shutdown status](docs/POWER_CONTROL.md), [FC checks](docs/DRONE_CHECK.md),
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

The three opt-in simulator cases need the exact external ArduPilot checkout and
SITL binary; see the [pinned simulator acceptance
gate](docs/ARDUCOPTER_4_7_NOGPS_LOITER.md#exact-pinned-sitl-acceptance-gate).
Contributor architecture and hardware boundaries are documented in
[CLAUDE.md](CLAUDE.md) and [AGENTS.md](AGENTS.md).

The [project site](site/README.md) presents the same maintained guides alongside
the [historical project poster](docs/poster/README.md). Preview it with
`uv run --group docs python site/build.py --serve`.
