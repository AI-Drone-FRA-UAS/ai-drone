#!/usr/bin/env bash
# deploy.sh — Sync the project to the Raspberry Pi and optionally run it.
#
# Usage:
#   ./deploy.sh            # sync + install deps on the Pi
#   ./deploy.sh --snap     # sync + take one photo + copy it to laptop
#   ./deploy.sh --stream   # sync + start live video stream (open in browser)
#   ./deploy.sh --run      # sync + run camera capture test
#   ./deploy.sh --ssh      # sync + open an interactive shell
#
# Environment variables (all optional):
#   PI_PROFILE — Pi target profile   (default: zero2; use pi4 for seb-is-pm2)
#   PI_HOST   — SSH target          (default: seb@192.168.7.2)
#   PI_DIR    — Remote project dir  (default: /home/seb/ai-drone)
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/pi-targets.sh"

PI_USER="${PI_USER:-$DEFAULT_PI_USERNAME}"
PI_HOST="${PI_HOST:-$PI_USER@$DEFAULT_PI_USB_IP}"
PI_DIR="${PI_DIR:-/home/$PI_USER/ai-drone}"
PI_USER="${PI_HOST%%@*}"
PI_IP="${PI_HOST#*@}"

UV="PATH=/home/${PI_USER}/.local/bin:\$PATH"

# ── 1. Sync ────────────────────────────────────────────────────────────────
echo "▸ Syncing to ${PI_HOST}:${PI_DIR}/ …"
rsync -az --delete \
  --exclude .git \
  --exclude .venv \
  --exclude .ruff_cache \
  --exclude .pytest_cache \
  --exclude __pycache__ \
  --exclude Drone-Handbook.pdf \
  --exclude raspios-lite \
  ./ "${PI_HOST}:${PI_DIR}/"
echo "  ✓ sync done"

# ── 2. Install dependencies ───────────────────────────────────────────────
echo "▸ Installing raspi dependencies on the Pi …"
# The venv must be created with system site packages so apt-installed
# picamera2/libcamera are visible inside the project environment.
ssh "${PI_HOST}" "cd ${PI_DIR} && ${UV} bash -lc 'if [[ ! -f .venv/pyvenv.cfg ]] || ! grep -qx \"include-system-site-packages = true\" .venv/pyvenv.cfg; then uv venv --clear --python /usr/bin/python3 --system-site-packages .venv; fi; uv sync --python .venv/bin/python --no-dev --group raspi'"
echo "  ✓ deps installed"

# ── 3. Optional: run or shell ─────────────────────────────────────────────
case "${1:-}" in
  --snap)
    echo "▸ Taking one photo on the Pi …"
    ssh -t "${PI_HOST}" "cd ${PI_DIR} && .venv/bin/python main.py camera --frames 1 --output /tmp/ai_drone_capture.jpg"
    echo "▸ Copying photo to laptop …"
    scp "${PI_HOST}:/tmp/ai_drone_capture.jpg" ./capture.jpg
    echo "  ✓ Saved to ./capture.jpg"
    ;;
  --stream)
    echo "▸ Starting live video stream on the Pi …"
    echo "  Open http://${PI_IP}:8080/ in your browser."
    echo "  Press Ctrl-C to stop."
    ssh -t "${PI_HOST}" "cd ${PI_DIR} && .venv/bin/python main.py stream"
    ;;
  --run)
    echo "▸ Running camera test on the Pi …"
    ssh -t "${PI_HOST}" "cd ${PI_DIR} && .venv/bin/python main.py camera"
    ;;
  --ssh)
    echo "▸ Opening shell on the Pi …"
    exec ssh -t "${PI_HOST}" "cd ${PI_DIR} && exec \$SHELL --login"
    ;;
  "")
    echo "Done. Use --snap, --stream, --run, or --ssh."
    ;;
  *)
    echo "Unknown flag: $1" >&2
    echo "Usage: $0 [--snap|--stream|--run|--ssh]" >&2
    exit 1
    ;;
esac
