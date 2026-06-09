# AI Drone Professor Baun

# install dependencies:
```bash
# Install uv (if not already installed)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install all desktop dependencies
uv sync --group desktop --group dev

# Smoke test
uv run python main.py status
```
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
PI_PROFILE=pi4 ./deploy.sh --picam
```

Open `http://192.168.8.2:8080/` for the Pi 4 USB profile. For the Zero 2
profile, omit `PI_PROFILE=pi4` and open `http://192.168.7.2:8080/`.

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

After the Pi-to-flight-controller UART cable is connected and `/dev/serial0`
provides MAVLink, the same test can run on the drone:

```bash
./deploy.sh --lidar
```

The current project hardware notes still list the Pi end of that cable as
pending, so USB from the laptop to the flight controller is the working lidar
test path today.

## Development checks

```bash
uv run --group dev ruff format --check .
uv run --group dev ruff check .
uv run --group dev ty check .
uv run --group dev pytest
```

Hardware details and the parameter backup are in
`docs/DRONE_CONFIGURATION.md` and `params/`.
