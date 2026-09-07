# Software architecture

The maintained implementation lives in `ai_drone/`. Historical flight experiments
are preserved through the [branch archive](../notes/archive/README.md); they are
not installed or deployed as operational flight tools.

## Runtime responsibilities

| Component | Responsibility |
| --- | --- |
| ArduPilot on FlywooF745 | Stabilization, EKF3 sensor fusion, flight modes and configured vehicle failsafes |
| `ai_drone.flight.controller` | Guarded GuidedNoGPS climb, relative-position acquisition, Loiter hold and landing |
| `ai_drone.cli.record` | Disarmed camera and MAVLink recording; optional explicitly enabled passive manual-flight recording |
| `ai_drone.vision.apriltags` | AprilTag IDs/corners and calibrated pose estimation on the Pi CPU |
| `ai_drone.cli.servo` | Explicitly guarded payload-servo bench operation on BCM12 |
| `ai_drone.cli.tag_servo_record` | Explicit armed-recording workflow with bounded tag-triggered servo pulses; it does not navigate or arm the FC |
| `ai_drone.link` | Pi connection discovery and runtime deployment |
| `ai_drone.config` | Source-filtered parameter snapshots and optional Git publication |

The IMX500 supplies images over CSI. The maintained AprilTag path does not run
neural inference on the camera accelerator. There is no maintained autonomous
search, path planner, person-following loop or verified obstacle-avoidance mission.
The intended forward MT-15 and downward MTF-01P have different roles; only the
downward distance is used as flight altitude. See the newest [configuration
record](DRONE_CONFIGURATION.md) for their actual reachability.

## Operator commands

| Command | Purpose |
| --- | --- |
| `drone-connect` | Pi SSH through the supported connection transports |
| `drone-deploy` | Maintenance deployment; optional allowlisted task through `--run` |
| `drone-inspect` | Camera/MAVLink capture, disarmed by default |
| `drone-servo` | Guarded servo bench test |
| `drone-tag-servo-record` | Explicit armed tag/servo recording |
| `drone-motor-test` | Guarded propeller-free motor bench test |
| `drone-control hover` | Guarded takeoff, timed Loiter and LAND sequence |
| `drone-config-sync` | Capture parameters through the Pi; `--no-sync` avoids deployment |

Use `uv run <command> --help` for current options. Old commands such as
`drone-console`, `drone-health`, `drone-deploy --picam` and standalone
`mission_drop.py` are not the maintained interface. An actuator or flight command
requires its documented physical checks and explicit confirmations; loading a
module or viewing help does not authorize a flight.

## Environment and installation

`uv sync` installs the locked runtime. The `dev` group adds tests, lint and type
checks; `raspi` adds OpenCV; `docs` adds the Markdown site renderer. The Pi uses a
virtual environment with system site packages for apt-installed Picamera2,
libcamera, gpiozero and native AprilTag. See [the project README](../README.md).

Connection defaults are implemented in `ai_drone.link.targets`: the Pi user is
`seb`, the Tailscale name is `seb-is-pm`, and the fallback hotspot address is
`192.168.4.1`. `PI_HOST`, `PI_USER`, `PI_DIR`, `PI_HOSTNAME`, `USB_IFACE` and
`SSH_CONFIG` provide explicit overrides. A USB interface must be identified
before the host adapter is configured. Use `SSH_CONFIG=/dev/null` for a direct
Pi link on hosts with a broken SSH configuration. [Pi networking](pi-networking.md)
describes the supported topology and the current installation procedure.

## Validation boundary

Offline tests mock hardware transports. The opt-in pinned ArduCopter SITL tests
exercise the production no-GPS hover sequence and GCS-link-loss recovery in a
simulated vehicle. Neither demonstrates that real optical flow, compass
calibration, sensor mounting, battery power or actuator motion is suitable for
flight. Bench observations and firmware/parameter provenance belong in dated
`state/` captures; follow [staged flight testing](PI_MAVLINK_CONTROL.md).
