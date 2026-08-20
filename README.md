# AI Drone

Software and guarded bench tools for a Raspberry Pi Zero 2 W companion
computer, an ArduPilot flight controller, an IMX500 camera, range/optical-flow
sensing, motors, and a payload servo.

Keep the aircraft disarmed unless an actuator or flight test has been
explicitly authorized and all physical prerequisites have been met. Use the
newest capture under `state/` and parameter dump under `params/` for live
configuration; evergreen documentation deliberately does not claim the current
wiring, mounting, or connection state.

## Setup

Install [uv](https://docs.astral.sh/uv/) and create the locked environment:

```bash
uv sync
```

Install development tools when needed:

```bash
uv sync --group dev
```

Python 3.11–3.13 is supported. The Pi uses Debian Python 3.13 so it can share
apt-installed Picamera2 and libcamera bindings.

## Commands

| Command | Purpose |
| --- | --- |
| `autoconnect` / `manuconnect` | Open Pi SSH over Tailscale, hotspot, or USB |
| `drone-deploy` | Synchronize the runtime to the Pi and optionally start one task |
| `drone-health` | Read-only heartbeat and parameter checks over USB and Pi UART |
| `drone-console` | Interactive MAVProxy console for expert inspection |
| `drone-lidar` | Record rangefinder and optical-flow telemetry while disarmed |
| `drone-apriltag` | Detect AprilTags; metric pose requires camera calibration |
| `drone-record` | Record synchronized camera, AprilTag, and MAVLink data while disarmed |
| `drone-servo` | Guarded direct-BCM12 payload-servo bench test |
| `drone-motor-test` | Guarded, low-power, propeller-off ArduPilot motor check |
| `drone-control` | Status, takeoff/hover, and bounded velocity tests |
| `drone-config-export` / `drone-config-sync` | Capture the ArduPilot configuration |

Run `uv run <command> --help` for authoritative options. Pi-only commands are
normally started through `drone-deploy` or from a Pi shell.

## Connect and deploy

Prefer Tailscale:

```bash
ssh -F /dev/null seb@seb-is-pm
```

When the Pi is serving its fallback hotspot, join `AI-Drone-Zero` and use:

```bash
ssh -F /dev/null seb@192.168.4.1
```

The connection helpers try Tailscale, hotspot, then USB:

```bash
uv run autoconnect
uv run manuconnect
```

USB setup requires `USB_IFACE` to identify the adapter that appeared when the
Pi was connected. See [USB SSH setup](docs/RPI_ZERO2W_USB_SSH_SETUP.md).

Deploy without starting a task:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm uv run drone-deploy
```

See [Pi networking](docs/pi-networking.md) for boot selection and manual
switching commands.

## Safety boundary

The following commands are read-only while the vehicle remains disarmed:

```bash
uv run drone-health
uv run drone-lidar --duration 10
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-deploy --record --duration 15
```

The flight battery may be needed to power attached sensors; these commands do
not power, arm, or move the aircraft.

Before any servo or motor test, remove all propellers, secure the frame, verify
power and wiring, and follow the command's confirmation gates. The motor
utility also requires `ARMING_CHECK=1` and resolved pre-arm failures. Follow
the [bench motor procedure](docs/BENCH_MOTOR_TEST.md).

No live arm, takeoff, altitude-hold, or velocity flight has been validated by
the repository documentation. Flight work must progress through SITL,
propeller-off checks, calibrated sensors, and restrained tests. See
[MAVLink control](docs/PI_MAVLINK_CONTROL.md).

## Development checks

```bash
uv run --group dev ruff format --check .
uv run --group dev ruff check .
uv run --group dev ty check .
uv run --group dev pytest -q
uv run --group dev lint-imports
uv run --group dev deptry .
git diff --check
```

## Documentation

- [Hardware inventory](docs/drone-project.md)
- [Flight-controller configuration](docs/DRONE_CONFIGURATION.md)
- [MAVLink control and staged flight tests](docs/PI_MAVLINK_CONTROL.md)
- [Sensor recording and wiring](docs/SENSOR_RECORDING.md)
- [AprilTag mission architecture](docs/APRILTAG_MISSION.md)
- [Pi networking](docs/pi-networking.md)
- [Direct flight-controller USB connection](docs/DEVELOPER_MACHINE_DRONE_CONNECTION.md)
- [Pi USB SSH](docs/RPI_ZERO2W_USB_SSH_SETUP.md)
- [Pi power-loss resilience](docs/PI_POWER_RESILIENCE.md)
- [Guarded motor test](docs/BENCH_MOTOR_TEST.md)

Dated hardware observations belong under `state/`; historical bring-up notes
belong under `notes/`; retired implementation material belongs under `attic/`.
