# AI Drone — Fra-UAS / Professor Baun

Camera, computer vision, and flight integration for a 3.5" CineWhoop drone
with a Raspberry Pi Zero 2 WH companion computer and IMX500 AI Camera.

## Quick Start — Laptop

```bash
# Install uv (if not already installed)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Install all desktop dependencies
uv sync --group desktop --group dev

# Smoke test
uv run python main.py status

# Camera preview (opens an OpenCV window, press 'q' to quit)
uv run python main.py camera
```

## Quick Start — Raspberry Pi

> **Prerequisites on the Pi** (already done):
> ```bash
> sudo apt install -y python3-picamera2 imx500-all
> ```
> `uv` must also be installed on the Pi.

```bash
# Deploy the project and install raspi deps
./deploy.sh

# Deploy to the Pi 4 profile instead
PI_PROFILE=pi4 ./deploy.sh

# Deploy and immediately run the camera
./deploy.sh --run

# Deploy and open an interactive shell on the Pi
./deploy.sh --ssh
```

The deploy script rsyncs the project to `seb@192.168.7.2:/home/seb/ai-drone`
by default, or `seb@192.168.8.2:/home/seb/ai-drone` with `PI_PROFILE=pi4`.
It creates a `.venv` with system site packages enabled so the project can see
apt-installed `picamera2` and `libcamera`.

## CLI Commands

```
main.py status  [--name NAME] [--battery N]   # Drone status smoke test
main.py camera  [--frames N] [--output PATH]  # Camera preview / capture
```

| Command | Laptop | Pi |
|---------|--------|----|
| `status` | ✓ | ✓ |
| `camera` | OpenCV window (or synthetic if no webcam) | picamera2 + IMX500 (headless capture) |

## Project Structure

```
ai-drone/
├── main.py                  # Entry point
├── ai_drone/
│   ├── __init__.py
│   ├── camera.py            # Platform-aware camera (picamera2 / OpenCV)
│   └── cli.py               # Typer CLI (status, camera)
├── tests/
│   └── test_camera.py
├── deploy.sh                # Sync & run on the Pi
├── connect-pi-usb-ssh.sh    # USB-SSH connection helper
├── enable-pi-usb-gadget.sh  # Patch microSD for USB Ethernet
├── pi-targets.sh            # Pi profile defaults
├── prepare-and-flash-pi.sh  # Flash Raspberry Pi OS
├── wait-for-pi-ssh.sh       # Wait for Pi over Wi-Fi
├── pyproject.toml
└── drone-project.md         # Hardware & implementation reference
```

## Architecture

The `Camera` class in `ai_drone/camera.py` auto-detects the platform:

- **Raspberry Pi** → uses `picamera2` (system package) to talk to the
  IMX500 AI Camera via libcamera
- **Laptop** → uses `opencv-python` (`VideoCapture`) for webcam access
- **No webcam** → falls back to a synthetic test pattern

Detection works by reading `/proc/device-tree/model`.

## Other Software

- [MicoControl](https://micoair.com/configurator/) — web-based drone configuration
- [MissionPlanner](https://ardupilot.org/planner/) — ArduPilot ground control
- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) — FC firmware flashing

### Example Camera Projects

1. [Nearest Person (Sony)](https://github.com/SonySemiconductorSolutions/aitrios-rpi-sample-apps/tree/main/examples/nearest-person)
2. [Pi Camera GUI Tool (Sony)](https://github.com/SonySemiconductorSolutions/aitrios-rpi-sample-app-gui-tool)


### start stream

```
PI_HOST=seb@100.99.38.65 ./deploy.sh --stream
ssh -F /dev/null seb@100.99.38.65 'pkill -f "main.py stream"'
```

Open http://100.99.38.65:8080/ in browser
