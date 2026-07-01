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

The camera test runs NanoDet on the IMX500 and serves annotated MJPEG from the
Pi. From the laptop, deploy and start it remotely:

```bash
./deploy.sh --picam
```

Open `http://192.168.7.2:8080/` (the Pi Zero 2 USB address).

When already logged into the Pi:

```bash
cd ~/ai-drone
uv run test_picam.py
```

First deploy the project if the Pi environment has not been prepared:

```bash
./deploy.sh
```

The deploy step creates the Pi virtual environment with system site packages
enabled so it can use apt-installed `picamera2`/libcamera, then installs the
`raspi` dependency group (`modlib`, NumPy, OpenCV, and Rich).

## Pi lidar link

The Pi-to-flight-controller UART4 cable is connected and `/dev/serial0`
provides MAVLink. The same sensor test can run on the drone:

```bash
./deploy.sh --lidar
```

The UART wiring is Pi TXD (pin 8) to FC R4, Pi RXD (pin 10) to FC T4, and
Pi GND (pin 6) to FC GND.

## Development checks

```bash
uv run --group dev ruff format --check .
uv run --group dev ruff check .
uv run --group dev ty check .
uv run --group dev pytest
```

Hardware details and the parameter backup are in
`docs/DRONE_CONFIGURATION.md` and `params/`.
