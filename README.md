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
main.py status   [--name NAME] [--battery N]           # Drone status smoke test
main.py camera   [--frames N] [--output PATH]          # Camera preview / capture
main.py stream   [--port N]                            # MJPEG live video over HTTP
main.py nearest  [--output stream|headless|display]    # People detection + nearest-neighbour distance
                 [--altitude M] [--fov DEG] [--rotate 0|90|180|270]
                 [--regions FILE] [--confidence F] [--threshold M] [--port N]
```

| Command | Laptop | Pi |
|---------|--------|----|
| `status` | ✓ | ✓ |
| `camera` | OpenCV window (or synthetic if no webcam) | picamera2 + IMX500 (headless capture) |
| `stream` | webcam → browser | picamera2 + IMX500 → browser |
| `nearest` | — | IMX500 on-sensor NanoDet + ByteTrack → browser/log |

## Project Structure

```
ai-drone/
├── main.py                  # Entry point
├── ai_drone/
│   ├── __init__.py
│   ├── camera.py            # Platform-aware camera (picamera2 / OpenCV)
│   ├── stream.py            # MJPEG HTTP streaming server
│   ├── nearest_person.py    # People detection + nearest-neighbour distance (IMX500/modlib)
│   ├── data/
│   │   └── nearest_person_regions.json  # Default 3D-calibration regions
│   └── cli.py               # Typer CLI (status, camera, stream, nearest)
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

## Nearest-Person Detector

`main.py nearest` is our integration of Sony's
[`aitrios-rpi-sample-apps` *nearest-person*](https://github.com/SonySemiconductorSolutions/aitrios-rpi-sample-apps/tree/main/examples/nearest-person)
example, adapted for the drone. The NanoDet object-detection model runs **on
the IMX500 sensor itself** (not the Pi CPU) via `modlib`; people are tracked
with ByteTrack and the pixel gap to each person's nearest neighbour is
converted to metres.

**Output modes** (`--output`):

- `stream` *(default)* — annotated MJPEG at `http://<pi-ip>:8080/`; best for the drone.
- `headless` — no image, logs `people=N  nearest pair=… m` per frame.
- `display` — OpenCV window (needs a desktop session on the Pi).

**Distance calibration** — two options:

- *Fixed-camera* (default): per-region distance-per-pixel from a JSON file
  (`--regions`, defaults to `ai_drone/data/nearest_person_regions.json`).
  Regenerate it for a new scene with Sony's
  [`tools/3D-calibration`](aitrios-rpi-sample-apps/tools/3D-calibration/).
- *Altitude* (drone): pass `--altitude <m>` (and `--fov <deg>` for your lens)
  to compute metres-per-pixel live from flight geometry — assumes a
  straight-down camera over flat ground, so treat absolute values as
  approximate and calibrate `--fov` to your lens.

`--rotate {0,90,180,270}` rotates the displayed/streamed image when the camera
is mounted rotated (it does not affect the detection maths).

```bash
# From the laptop: deploy + start the detector stream on the Pi
PI_HOST=seb@100.99.38.65 ./deploy.sh --nearest

# Altitude-based distance at 5 m with a 66° lens, image flipped 180°
PI_HOST=seb@100.99.38.65 ./deploy.sh --nearest --altitude 5 --fov 66 --rotate 180
```

Then open `http://100.99.38.65:8080/` in a browser. First run uploads the model
to the sensor, which takes a few minutes; subsequent runs start in seconds.

## Other Software

- [MicoControl](https://micoair.com/configurator/) — web-based drone configuration
- [MissionPlanner](https://ardupilot.org/planner/) — ArduPilot ground control
- [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) — FC firmware flashing

### Example Camera Projects

1. [Nearest Person (Sony)](https://github.com/SonySemiconductorSolutions/aitrios-rpi-sample-apps/tree/main/examples/nearest-person)
2. [Pi Camera GUI Tool (Sony)](https://github.com/SonySemiconductorSolutions/aitrios-rpi-sample-app-gui-tool)

### connect to pi

Key-based login is set up via `~/.ssh/config` (key `~/.ssh/pi_drone`), so both
of these work without a password:

```
ssh drone-pi            # alias
ssh seb@100.99.38.65    # by IP
```

`deploy.sh` picks up that key automatically (it defaults `SSH_CONFIG` to
`~/.ssh/config`).

### start stream

Plain camera stream:

```
PI_HOST=seb@100.99.38.65 ./deploy.sh --stream
ssh drone-pi 'pkill -f "main.py stream"'
```

Nearest-person detector stream (see [Nearest-Person Detector](#nearest-person-detector)):

```
PI_HOST=seb@100.99.38.65 ./deploy.sh --nearest
ssh drone-pi 'pkill -f "main.py nearest"'
```

Open http://100.99.38.65:8080/ in browser

### enable eduroam and change wifi
```
ssh -t seb@100.99.38.65 'cd ~/eduroam-setup && ./install-eduroam-fuas.sh'

ssh -t seb@100.99.38.65 'sudo nmcli connection up eduroam ifname wlan0'

tailscale ping 100.99.38.65

ssh seb@100.99.38.65
```


PI_HOST=seb@100.99.38.65 ./deploy.sh --nearest
