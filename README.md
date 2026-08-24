# AI Drone Professor Baun

A 3.5" CineWhoop FPV drone converted into a semi-autonomous indoor platform:
ArduPilot Copter on a Flywoo GN745, a Raspberry Pi Zero 2 WH as companion
computer, an IMX500 camera that detects AprilTags, and a servo-driven payload
drop the detection triggers. Project work at Frankfurt University of Applied
Sciences, supervised by Prof. Dr. Baun.

**Where the project stands.** Detecting a `tag36h11` marker and dropping the
payload on it works, verified on the ground and hand-held. Fully autonomous
flight does not: on 2026-08-21 the aircraft was destroyed during a takeoff
attempt. Nobody was hurt. The attempts, the three measured causes and what
changed because of them are written up on the project site under
[Flugversuche und Absturz](https://ai-drone-fra-uas.github.io/ai-drone/flugversuche.html).
The reworked flight code and the dated records of those flights live on the
`preflight-and-nogps-takeoff` branch; the AprilTag drop mission is on the
`experimental-*` branches. This branch still carries the earlier
person-following code.

**[Documentation index](docs/README.md)** — every document in this repository,
grouped by task. The rendered version is published as a
[project site](site/README.md); the A1 poster is in
[docs/poster/](docs/poster/README.md).

## Install dependencies

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
sudo apt install -y python3-picamera2 imx500-all
```
`uv` must also be installed on the Pi.

For:

- streaming the Raspberry Pi IMX500 AI-camera output; and
- checking the MicoAir MTF-01P range/optical-flow telemetry through ArduPilot.

## MTF-01P test

Connect the flight controller over USB and power the drone so the MTF-01P has
power. Keep the vehicle disarmed.

```bash
uv run test_lidar.py
```

The script:

- auto-detects the ArduPilot `/dev/ttyACM*` device;
- aborts if the vehicle is armed;
- requests range and optical-flow telemetry for five seconds; and
- saves a timestamped CSV under `artifacts/`.

Useful options:

```bash
uv run test_lidar.py --duration 10
uv run test_lidar.py --device /dev/ttyACM0 --output artifacts/lidar.csv
```

It does not arm the drone, start motors, change flight mode, or write
parameters. It cannot power the sensor; the flight battery must do that.

## IMX500 AI stream

This is the earlier person-detection path. The current approach detects
AprilTags instead — a geometric marker gives a unique ID and sub-pixel corners,
which a neural bounding box cannot; see
[AprilTag-Erkennung und Abwurf](https://ai-drone-fra-uas.github.io/ai-drone/apriltag.html).

The camera test runs NanoDet on the IMX500 and serves annotated MJPEG from the
Pi. From the laptop, deploy and start it remotely:

```bash
uv run drone-deploy --picam
```

Open `http://192.168.7.2:8080/` (the Pi Zero 2 USB address).

When already logged into the Pi:

```bash
cd ~/ai-drone
uv run test_picam.py
```

First deploy the project if the Pi environment has not been prepared:

```bash
uv run drone-deploy
```

The deploy step creates the Pi virtual environment with system site packages
enabled so it can use apt-installed `picamera2`/libcamera, then installs the
`raspi` dependency group (`modlib`, NumPy, OpenCV, and Rich).

## Pi lidar link

The Pi-to-flight-controller UART4 cable is connected and `/dev/serial0`
provides MAVLink. The same sensor test can run on the drone:

```bash
uv run drone-deploy --lidar
```

The UART wiring is Pi TXD (pin 8) to FC R4, Pi RXD (pin 10) to FC T4, and
Pi GND (pin 6) to FC GND.

## Servo test

To test the SG90 (9g) servo motor on the Raspberry Pi Zero:
1. Wire the servo:
   - **VCC (Red)** -> Pi 5V (e.g., physical Pin 2 or 4). *Warning: For heavy loads, use a separate 5V power supply to avoid brownouts.*
   - **GND (Brown/Black)** -> Pi GND (e.g., physical Pin 6).
   - **Signal (Yellow/Orange)** -> Pi BCM GPIO 12 (physical Pin 32).
2. Deploy and run the test script:
   ```bash
   ./deploy.sh --servo
   ```
   By default, this will run in **sweep** mode. You can customize the behavior or change the mode by forwarding parameters:
   ```bash
   ./deploy.sh --servo --mode manual
   ./deploy.sh --servo --mode center
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
uv run drone-connect
```

`drone-connect` first tries normal SSH targets for Wi-Fi or the Pi hotspot:
`seb-is-pm.local`, `seb-is-pm`, `192.168.4.1`, and `192.168.7.2`.
If none respond, it falls back to the USB/RNDIS cable path, configures the
laptop-side `192.168.7.1/24` address, waits for the Pi at `192.168.7.2`, and
opens SSH. On Windows, run the terminal as Administrator for USB fallback
because changing the adapter IP requires elevation.

To try only Wi-Fi/hotspot/network SSH and never configure USB:

```bash
uv run drone-connect --network-only
```

If adapter auto-detection fails, pass the interface explicitly:

```bash
uv run drone-connect --usb-iface "Ethernet 4"
```

The legacy `./connect-pi-usb-ssh.sh` wrapper remains for compatibility.

`drone-deploy` uses `rsync` on Unix when available and otherwise streams a tar
archive over SSH, so native Windows does not need a local `rsync` install.

On the Frankfurt UAS campus the Pi joins the `eduroam` Wi-Fi automatically and
is reachable over Tailscale (`ssh -tt seb@100.84.84.1`); off campus it falls
back to its own `AI-Drone-Zero` hotspot. See
[eduroam WLAN auf dem Raspberry Pi](docs/EDUROAM_SETUP.md) and
[README_HOTSPOT.md](README_HOTSPOT.md).

## Documentation

Start at the [documentation index](docs/README.md). The most-used entries:

| Topic | Document |
|-------|----------|
| What the project is building, and with what hardware | [Implementation Reference](docs/drone-project.md) |
| How the code is laid out and what each command does | [Software Architecture](docs/SOFTWARE_ARCHITECTURE.md) |
| Verified flight-controller state and parameters | [Drone Configuration](docs/DRONE_CONFIGURATION.md) |
| Autonomous control, body-frame velocity, safety rules | [Pi MAVLink Control](docs/PI_MAVLINK_CONTROL.md) |
| The payload drop servo | [Payload Drop Mechanism](docs/PAYLOAD_DROP.md) |
| Printed frame parts | [Frame Extension and 3D Prints](docs/FRAME_AND_3D_PRINTS.md) |

To preview the published site locally:

```bash
uv run --group docs python site/build.py --serve
```
