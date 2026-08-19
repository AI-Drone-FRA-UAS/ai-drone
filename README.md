# AI Drone

Software and bench tools for a Raspberry Pi Zero 2 W, ArduPilot flight
controller, IMX500 camera, downward MTF-01P range/optical-flow sensor, planned
forward MT-15 rangefinder, motors, and payload servo.

Keep the aircraft disarmed unless an actuator or flight test has been
explicitly authorized and its physical prerequisites are satisfied. Check the
newest capture under `state/` and `params/` for the controller's actual
configuration rather than assuming it from documentation.

## Setup

Install [uv](https://docs.astral.sh/uv/) and the locked project environment:

```bash
uv sync
```

Development tools are optional:

```bash
uv sync --group dev
```

The supported runtime is Python 3.11–3.13. The Pi uses Debian Python 3.13 so it
can share the apt-installed Picamera2 and libcamera bindings.

## Command toolbox

Each behavior has one package entry point; the old root wrappers and duplicate
flight commands are intentionally not maintained.

| Command | Purpose |
| --- | --- |
| `autoconnect` / `manuconnect` | Open Pi SSH over Tailscale, the Pi hotspot, or USB |
| `drone-deploy` | Synchronize the runtime to the Pi and optionally start one Pi task |
| `drone-health` | Read-only heartbeat and parameter round-trip over USB and Pi UART |
| `drone-console` | Interactive MAVProxy console for expert inspection |
| `drone-lidar` | Record rangefinder and optical-flow MAVLink telemetry while disarmed |
| `drone-apriltag` | Detect AprilTags; metric pose requires camera calibration |
| `drone-record` | Record synchronized camera, AprilTag, and MAVLink data while disarmed |
| `drone-servo` | Guarded direct-BCM12 payload-servo bench test |
| `drone-motor-test` | Guarded, low-power, propeller-off ArduPilot motor check |
| `drone-control` | Canonical status, takeoff/hover, and velocity CLI; flight sequences arm and land |
| `drone-config-export` / `drone-config-sync` | Capture the complete ArduPilot configuration |

Use `uv run <command> --help` for the authoritative options. Pi-only commands
are normally started through `drone-deploy` or after opening a Pi shell.

## Connect and deploy

The preferred endpoint is the Tailscale MagicDNS name:

```bash
ssh -F /dev/null seb@seb-is-pm
```

If Tailscale is offline, join `AI-Drone-Zero` and use the hotspot address:

```bash
ssh -F /dev/null seb@192.168.4.1
```

The helpers try Tailscale, hotspot, then USB in that order:

```bash
uv run autoconnect
uv run manuconnect
```

USB setup requires `USB_IFACE` to name the adapter that you verified appeared
when the Pi was connected. Use `--dry-run` to inspect the commands first.
See [Raspberry Pi USB SSH](docs/RPI_ZERO2W_USB_SSH_SETUP.md) for Linux, macOS,
and Windows instructions.

Deploy without starting a task:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm uv run drone-deploy
```

Replace `seb-is-pm` with `192.168.4.1` only when using the Pi hotspot. The
deployment includes the package, lock/build metadata, and maintained network
scripts; it excludes credentials, Git data, virtual environments, and
recordings.

## Tests possible in the current hardware state

The following checks are read-only/disarmed. Support the loose camera and keep
the aircraft stationary.

Check the developer-USB and Pi-UART MAVLink paths:

```bash
uv run drone-health
uv run drone-health --usb-only
uv run drone-health --pi-only
```

Record the connected downward MTF-01P range and optical-flow telemetry:

```bash
uv run drone-lidar --duration 10
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm uv run drone-deploy --lidar --duration 10
```

The flight battery must power the MTF-01P; the command itself never powers,
arms, or moves the aircraft.

Run the forward-facing camera diagnostics:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-deploy --apriltag --backend auto --tag-size 0.160
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-deploy --record --duration 15
```

The camera can test image quality, frame rate, and AprilTag pixel detection.
It cannot validate floor-relative metric pose or altitude geometry until its
mounting orientation is recorded and its intrinsics are calibrated.

## Hardware tests that require preparation

### Servo

The servo is connected and the utility is ready. It drives Pi BCM12 (physical
pin 32), not a flight-controller output, so it actuates whenever it is run.
Before using it, remove the propellers, secure the frame, confirm signal and
common ground, use a suitable regulated external 5 V supply, and verify the
linkage and pulse limits. Then follow the guarded modes shown by:

```bash
uv run drone-servo --help
```

### Forward MT-15

The MT-15 is disconnected and untested. The proposed connection is
flight-controller UART7, so it does not consume another Pi UART/header signal;
physical mounting, FC connector availability, wiring, and regulated power
still need verification. Once installed and configured as `RNGFND2`, use
`drone-lidar` and `drone-record` to verify that its sensor ID, forward
orientation, limits, validity, and measurements remain distinct from the
downward MTF-01P. A single forward beam is not full obstacle avoidance.

### Motor check

The low-power motor utility is present, capped at 10% and one second per motor,
and defaults to 7% for 0.5 seconds. It deliberately refuses the live controller
while `ARMING_CHECK` is not exactly `1`. Remove every propeller, secure the
frame, resolve all pre-arm failures, and follow the exact confirmations in the
[bench motor procedure](docs/BENCH_MOTOR_TEST.md).

### Altitude hold and flight control

`drone-control` keeps the arming, takeoff/hover, body-frame velocity, and
landing capabilities in one CLI. Arming and landing are
managed inside its guarded flight sequences rather than duplicated as raw
standalone commands. These paths have not been validated on this aircraft. Run
the command on the Pi after deploying:

```bash
uv run drone-deploy --ssh
# On the Pi:
cd ~/ai-drone
uv run drone-control --help
```

Validate in stages: SITL first; then correct all pre-arm failures; verify the
downward range/flow data and calibration; perform the propeller-off motor test;
and only then conduct a restrained, low-altitude hover/altitude-hold test under
the flight procedure. Person follow additionally requires a secure camera
mount, measured geometry, bounded tracking behavior, and an unobstructed test
area. See [MAVLink control](docs/PI_MAVLINK_CONTROL.md) and the
[AprilTag mission plan](docs/APRILTAG_MISSION.md).

## What `ARMING_CHECK=1` means

ArduPilot treats the special value `1` as “run all available configurable
pre-arm checks.” `0` skips those configurable categories, although some
mandatory checks can still remain. It does not arm the vehicle or spin a motor;
it makes an arm request fail until reported `PreArm:` problems are resolved.
It also cannot detect physical problems such as the loose camera, disconnected
servo, or missing MT-15. See [flight-controller configuration](docs/DRONE_CONFIGURATION.md).

## Development checks

```bash
uv run --group dev ruff format --check .
uv run --group dev ruff check .
uv run --group dev ty check .
uv run --group dev pytest -q
```

## Documentation map

- [Repository state](docs/REPOSITORY_STATE.md): current layout, commands, and what is verified.
- [MAVLink control](docs/PI_MAVLINK_CONTROL.md): staged control and flight tests.
- [Sensor recording and wiring](docs/SENSOR_RECORDING.md): MTF-01P, MT-15, camera, and datasets.
- [AprilTag mission](docs/APRILTAG_MISSION.md): mounting, calibration, and approach architecture.
- [Pi power resilience](docs/PI_POWER_RESILIENCE.md): surviving sudden power loss and upgrading safely.
- [Motor test](docs/BENCH_MOTOR_TEST.md): propeller-off guarded procedure.
- [Configuration](docs/DRONE_CONFIGURATION.md): controller parameters and `ARMING_CHECK`.
- [Connection](docs/DEVELOPER_MACHINE_DRONE_CONNECTION.md): direct flight-controller USB workflow.
- [Network setup](docs/EDUROAM_SETUP.md), [hotspot](docs/HOTSPOT.md), and [dual-network design](docs/NETWORK_UPLINK.md).
