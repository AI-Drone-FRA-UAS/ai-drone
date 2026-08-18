# AI Drone Professor Baun

# Install dependencies

```bash
# Install uv (Linux/MacOS)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install uv (Windows)
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"


# Install the project runtime
uv sync

# Optional development tools
uv sync --group dev

# Smoke test
uv run python -c "from pymavlink import mavutil; print('pymavlink ready')"
```

## Connect the flight controller from a developer machine

The project includes MAVProxy plus the compatibility packages required by its
default console modules. With the flight controller connected over USB:

```bash
uv run drone-console
```

`drone-console` prefers the stable ArduPilot `/dev/serial/by-id/...` path,
falls back to `/dev/ttyACM*`, and uses 115200 baud. To override those defaults:

```bash
uv run drone-console --device /dev/ttyACM0 --baud 115200
```

Check both working MAVLink paths from the developer machine:

```bash
uv run drone-health
```

This checks:

1. developer machine -> flight controller over USB; and
2. `seb@seb-is-pm` -> `/dev/serial0` -> flight controller over UART4.

Both checks require a heartbeat and a read-only `SYSID_THISMAV` response. Enter
the Pi SSH password when prompted. To check only one path:

```bash
uv run drone-health --usb-only
uv run drone-health --pi-only
```

For the complete physical connection sequence, safety constraints, stable
device path, MAVProxy commands, shutdown procedure, and troubleshooting, see
[Developer Machine Drone Connection](docs/DEVELOPER_MACHINE_DRONE_CONNECTION.md).

## Quick Start — Raspberry Pi

**Prerequisites on the Pi** (already done):
```bash
sudo apt install -y python3-picamera2 imx500-all python3-apriltag
```
`uv` must also be installed on the Pi.

For:

- streaming the Raspberry Pi IMX500 AI-camera output; and
- checking the MicoAir MTF-01P range/optical-flow telemetry through ArduPilot.

## MTF-01P test

Connect the flight controller over USB and power the drone so the MTF-01P has
power. Keep the vehicle disarmed.

```bash
uv run drone-lidar
```

The script:

- auto-detects the ArduPilot `/dev/ttyACM*` device;
- aborts if the vehicle is armed;
- requests range and optical-flow telemetry for five seconds; and
- saves a timestamped CSV under `artifacts/`.

Useful options:

```bash
uv run drone-lidar --duration 10
uv run drone-lidar --device /dev/ttyACM0 --output artifacts/lidar.csv
```

It does not arm the drone, start motors, change flight mode, or write
parameters. It cannot power the sensor; the flight battery must do that.

## IMX500 AI stream

The camera test runs NanoDet on the IMX500 and serves annotated MJPEG from the
Pi. From the laptop, deploy and start it remotely:

```bash
uv run drone-deploy --picam
```

Open `http://192.168.7.2:8080/` (the Pi Zero 2 USB address).

When already logged into the Pi:

```bash
cd ~/ai-drone
uv run drone-picam
```

First deploy the project if the Pi environment has not been prepared:

```bash
uv run drone-deploy
```

The deploy step creates the Pi virtual environment with system site packages
enabled so it can use apt-installed `picamera2`/libcamera, then installs the
`raspi` dependency group (`modlib`, NumPy, OpenCV, and Rich).

## AprilTag detection

Safe, detection-only AprilTag processing uses native AprilTag 3 when available
and OpenCV as a fallback. It never arms, moves, or actuates the servo:

```bash
uv run drone-deploy --apriltag --backend auto --tag-size 0.160
```

The default 1280x960 mode prioritizes range. Use `--resolution 640x480` for
approximately 30 fps on the Pi Zero 2 W. Add `--output stream` to serve an
annotated stream on port 8081.

Metric distance requires a real camera calibration:

```bash
uv run drone-deploy --apriltag \
  --calibration config/imx500-1280x960.json \
  --tag-size 0.160
```

Without calibration, the command reports only IDs, corners, and detector
quality. See [AprilTag mission architecture](docs/APRILTAG_MISSION.md) before
attempting pose-guided movement or payload release.

## Disarmed all-sensor recording

Record H.264 camera video, frame metadata, AprilTag detections, a raw MAVLink
telemetry log, and JSONL telemetry for an exact requested interval:

```bash
uv run drone-deploy --record --duration 15
```

Recordings remain on the Pi under `~/ai-drone/artifacts/sensor-recordings/` and
are intentionally excluded from deployment deletion and Git. The recorder
refuses to start when armed and stops if a vehicle heartbeat becomes armed. It
never sends arm, disarm, mode, motor, RC override, mission, or servo commands.
See [sensor recording and wiring](docs/SENSOR_RECORDING.md).

## Pi lidar link

The Pi-to-flight-controller UART4 cable is connected and `/dev/serial0`
provides MAVLink. The same sensor test can run on the drone:

```bash
uv run drone-deploy --lidar
```

The UART wiring is Pi TXD (pin 8) to FC R4, Pi RXD (pin 10) to FC T4, and
Pi GND (pin 6) to FC GND.

## Servo test

The SG90/MS18-F signal wire is on BCM GPIO 12 (physical pin 32). The servo VCC
uses 5V and GND from the Pi for light bench tests; use a separate 5V supply for
loaded tests to avoid Pi brownouts.

```bash
uv run drone-deploy --servo
uv run drone-deploy --servo --mode manual
uv run drone-deploy --servo --mode center
```

## Development checks

```bash
uv run --group dev ruff format --check .
uv run --group dev ruff check .
uv run --group dev ty check .
uv run --group dev pytest
```

## Connect to the Pi

Fresh checkout flow:

```bash
git clone git@github.com:AI-Drone-FRA-UAS/ai-drone.git
cd ai-drone
uv run autoconnect
```

There are two connect commands, both trying the same three transports in the
same priority order:

1. **Tailscale** — `ssh seb@seb-is-pm`. The normal case: the Pi prefers a known
   Wi-Fi, so it has internet and is on the tailnet.
2. **The Pi's own Wi-Fi AP** `AI-Drone-Zero` (`192.168.4.1`). The Pi self-hosts
   this only when it can't reach a known network.
3. **USB cable** — the RNDIS gadget path (laptop side `192.168.7.1/24`, Pi at
   `192.168.7.2`). Always-there last resort.

```bash
uv run autoconnect     # try 1 -> 2 -> 3, stop at the first that connects
uv run manuconnect     # pick 1, 2, or 3 from a menu
```

`autoconnect --dry-run` prints the commands each transport would run without
connecting. On Windows, run the terminal as Administrator for the USB path
because changing the adapter IP requires elevation.

Joining `AI-Drone-Zero` uses a **pre-saved** OS Wi-Fi profile (the script never
handles the password). Save it once, e.g. on Linux:

```bash
nmcli dev wifi connect AI-Drone-Zero password <PSK>
```

The legacy `./connect-pi-usb-ssh.sh` wrapper still works and now calls
`autoconnect`.

`drone-deploy` uses `rsync` on Unix when available and otherwise streams a tar
archive over SSH, so native Windows does not need a local `rsync` install.

Hardware details and the parameter backup are in
`docs/DRONE_CONFIGURATION.md` and `params/`.

## Repository layout

- `docs/` contains maintained setup and operating guides, including the
  [hotspot guide](docs/HOTSPOT.md).
- `notes/` contains dated or ad hoc bring-up notes, including the
  [19 June 2026 session report](notes/19-06-session.md).
- `hardware/3d-prints/` contains STL/3MF print assets.
- `scripts/` contains shell helpers that are not Python package entry points.
  Root wrappers such as `./deploy.sh`, `./connect-pi-usb-ssh.sh`, and
  `./setup-pi-hotspot.sh` remain for compatibility.
- Large local reference assets belong under `docs/assets/`, for example the
  ignored `docs/assets/Drone-Handbook.pdf`.
