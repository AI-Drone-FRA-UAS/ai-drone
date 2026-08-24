# Software Architecture

How the code in this repository is laid out, which part runs on which machine,
and what every entry point does. For the flight-control theory and the safety
rules behind `DroneController`, see [Pi MAVLink Control](PI_MAVLINK_CONTROL.md).

> **Status, 2026-08-24.** This describes the layout of this branch, which still
> carries the person-following stack. The reworked flight code — guards,
> throttle model, teardown, and the simulated vehicle they are rehearsed
> against — lives on `preflight-and-nogps-takeoff`, and the AprilTag drop
> mission on the `experimental-*` branches. What the project achieved and what
> it did not is summarised on the
> [project site](https://ai-drone-fra-uas.github.io/ai-drone/).

---

## 1. Where the code runs

Three machines are involved, and each piece of code belongs to exactly one of
them.

```text
  Developer machine (macOS / Linux / Windows)
  ├─ drone-connect   SSH to the Pi (Wi-Fi, hotspot, or USB gadget fallback)
  ├─ drone-deploy    sync this repo to the Pi and start a task there
  ├─ drone-console   MAVProxy against the flight controller over USB
  └─ drone-health    verify both MAVLink paths
                          │
                          │ SSH / rsync
                          ▼
  Raspberry Pi Zero 2 WH  (companion computer)
  ├─ nearest_person.py    NanoDet on the IMX500, annotated MJPEG on :8080
  ├─ control_drone.py     status / hover / velocity-test / follow
  ├─ follow_person.py     autonomous person following
  └─ test_lidar.py        MTF-01P range and flow telemetry to CSV
                          │
                          │ UART4 /dev/serial0, MAVLink 2
                          ▼
  Flywoo GOKU GN745 AIO   ArduPilot Copter 4.6.3 — motors, EKF3, failsafe
```

The developer machine never talks to the sensors directly. It either speaks to
the flight controller over USB, or it drives the Pi over SSH and lets the Pi
speak to the flight controller.

---

## 2. The `ai_drone` package

| Module | Responsibility |
|--------|----------------|
| `ai_drone.controller` | `DroneController` — MAVLink connection, telemetry thread, verified mode changes, arm/takeoff/land, body-frame velocity |
| `ai_drone.follower` | `AutonomousFollower` — turns person detections into velocity commands, plus the battery/altitude/target-loss guards |
| `ai_drone.nearest_person` | IMX500 NanoDet inference through `modlib`; `stream` and `headless` output modes |
| `ai_drone.stream` | Minimal threaded MJPEG HTTP server for the annotated camera frames |
| `ai_drone.console` | Launches MAVProxy with the project's device and baud defaults |
| `ai_drone.health` | Checks the USB link and the Pi UART link; both require a heartbeat and a `SYSID_THISMAV` read |
| `ai_drone.deploy` | Syncs the repo to the Pi (rsync, or a tar stream where rsync is missing) and runs the selected task |
| `ai_drone.pi_usb_ssh` | Configures the host side of the USB gadget network and opens SSH |
| `ai_drone.pi_targets` | Shared Pi connection defaults and environment-variable resolution used by both `deploy` and `pi_usb_ssh` |

`ai_drone/__init__.py` re-exports the flight API, so the common import is:

```python
from ai_drone import AutonomousFollower, DroneController
```

### `DroneController` in one paragraph

`DroneController` opens a `pymavlink` connection (auto-detecting `/dev/serial0`
on the Pi, `/dev/ttyACM*` over USB, or a `udp:` endpoint for SITL), requests
`LOCAL_POSITION_NED`, `RANGEFINDER`, `ATTITUDE`, and `SYS_STATUS` at a fixed
rate, and keeps `current_altitude`, `battery_voltage`, `flight_mode`, and
`is_armed` up to date from a background parse loop. `set_mode`, `arm`,
`takeoff`, and `land` all wait for confirmation rather than firing blind
commands. Used as a context manager, `__exit__` triggers `emergency_stop()`,
so an exception or `Ctrl+C` lands the vehicle instead of leaving it hovering.

---

## 3. Entry points

### Installed commands

Declared under `[project.scripts]` in `pyproject.toml`:

| Command | Module | What it does |
|---------|--------|--------------|
| `uv run drone-connect` | `ai_drone.pi_usb_ssh:main` | Tries `seb-is-pm.local`, `seb-is-pm`, `192.168.4.1`, `192.168.7.2`, then falls back to the USB gadget path |
| `uv run drone-console` | `ai_drone.console:main` | MAVProxy on the stable `/dev/serial/by-id/...` path at 115200 baud |
| `uv run drone-deploy` | `ai_drone.deploy:main` | Sync to the Pi; `--picam`, `--lidar`, `--servo`, `--ssh`, `--dry-run` |
| `uv run drone-health` | `ai_drone.health:main` | Both MAVLink paths; `--usb-only`, `--pi-only` |
| `uv run drone-pi-usb-ssh` | `ai_drone.pi_usb_ssh:main` | The same as `drone-connect`, kept under its original name |

`./deploy.sh` and `./connect-pi-usb-ssh.sh` are thin wrappers that `exec` into
`drone-deploy` and `drone-connect`; prefer the `uv run` form.

### Scripts

| Script | Runs on | What it does |
|--------|---------|--------------|
| `control_drone.py` | Pi or bench | Subcommands `status`, `hover`, `velocity-test`, `follow` |
| `follow_person.py` | Pi | Autonomous person following; `--sim-target` runs the loop with no camera and no takeoff |
| `test_lidar.py` | Pi or bench | Requests MTF-01P range and flow telemetry, writes a timestamped CSV under `artifacts/` |
| `test_picam.py` | Pi | Starts the IMX500 stream directly on the Pi |
| `test_servo.py` | Pi | Drives the 9 g servo on BCM GPIO 12 — see [Payload Drop Mechanism](PAYLOAD_DROP.md) |
| `fly_and_land.py` | Pi | The original linear takeoff/land script, superseded by `control_drone.py` |

Scripts that only read telemetry (`test_lidar.py`, `control_drone.py status`)
never arm the vehicle and never write parameters.

---

## 4. Environment variables

`ai_drone.pi_targets` resolves the Pi connection from the environment, so a
second Pi or a different user needs no code change. All of them are optional.

| Variable | Default | Used by |
|----------|---------|---------|
| `PI_HOST` | auto-probed: `192.168.7.2`, then `seb-is-pm` | `drone-deploy` — set it to skip probing, `user@host` is accepted |
| `PI_USER` | `seb` | `drone-deploy`, `drone-connect` |
| `PI_DIR` | `/home/$PI_USER/ai-drone` | `drone-deploy` — the checkout location on the Pi |
| `PI_IP` | `192.168.7.2` | `drone-connect` — the Pi address on the USB gadget network |
| `HOST_IP` | `192.168.7.1` | `drone-connect` — the address configured on the host adapter |
| `PI_HOSTNAME` | `seb-is-pm` | `drone-connect` |
| `USB_IFACE` | auto-detected | `drone-connect` — pass the adapter name when detection fails |
| `TIMEOUT_SECONDS` | `180` | `drone-connect` — how long to wait for the Pi to appear |
| `SSH_CONFIG` | `~/.ssh/config` if it exists | both — set to empty to ignore the SSH config |

Example:

```bash
PI_HOST=seb@100.84.84.1 uv run drone-deploy --picam
```

---

## 5. Dependency groups

`pyproject.toml` keeps `default-groups = []`, so `uv sync` installs only the
lightweight runtime. Everything else is opt-in.

| Group | Contents | Installed on |
|-------|----------|--------------|
| *(runtime)* | `pymavlink`, `mavproxy`, `pyserial`, `pillow`, `future` | developer machine and Pi |
| `raspi` | `modlib`, `numpy`, `opencv-python`, `rich` | Pi only — `drone-deploy` adds it automatically |
| `dev` | `pytest`, `ruff`, `ty` | developer machine |
| `docs` | `markdown-it-py` | developer machine and CI, for the project site |

The Pi virtual environment is created with system site packages enabled so it
can use the apt-installed `picamera2` and `libcamera`, which are not on PyPI.

---

## 6. Tests and checks

The test suite runs entirely on the developer machine with no hardware
attached: MAVLink connections, SSH calls, and the camera are faked.

| File | Covers |
|------|--------|
| `tests/test_controller.py` | Device discovery, telemetry parsing, mode/arm/takeoff sequencing, emergency stop |
| `tests/test_follower.py` | Target extraction, the proportional control law, deadzones, and the safety guards |
| `tests/test_drone_tools.py` | The `control_drone.py` subcommands and their argument handling |
| `tests/test_cross_platform_scripts.py` | Deploy and connect behaviour on Linux, macOS, and Windows |

```bash
uv run --group dev ruff format --check .
uv run --group dev ruff check .
uv run --group dev ty check .
uv run --group dev pytest
```

Run all four before pushing; they are the same checks used when reviewing a
pull request.

---

## 7. Adding a new Pi task

1. Write the script so that it works when run from the repository root on the
   Pi, and give it a `--dry-run` or read-only mode where that makes sense.
2. Add a flag to `_parser()` in `ai_drone/deploy.py` and the matching branch in
   `mode_command()`.
3. Cover the new command string in `tests/test_cross_platform_scripts.py`.
4. Document it in the [documentation index](README.md) and, if it touches the
   airframe, in the relevant hardware document.
