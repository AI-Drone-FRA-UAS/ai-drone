#!/usr/bin/env bash
# deploy.sh — Sync the project to the Raspberry Pi and optionally run it.
#
# Usage:
#   ./deploy.sh            # sync + install dependencies on the Pi
#   ./deploy.sh --picam    # start the IMX500 AI stream on the Pi
#                          #   (extra args are forwarded)
#   ./deploy.sh --lidar    # sample MTF-01P data through the Pi's FC serial link
#   ./deploy.sh --ssh      # sync + open an interactive shell on the Pi
#
# Environment variables (all optional):
#   PI_HOST   — SSH target          (default: seb@192.168.7.2)
#   PI_DIR    — Remote project dir  (default: /home/seb/ai-drone)
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/pi-targets.sh"

PI_USER="${PI_USER:-$DEFAULT_PI_USERNAME}"

if [[ -z "${PI_HOST:-}" ]]; then
  if ping -c 1 -W 1 "$DEFAULT_PI_USB_IP" >/dev/null 2>&1; then
    PI_HOST="$PI_USER@$DEFAULT_PI_USB_IP"
  elif ping -c 1 -W 1 "$DEFAULT_PI_HOSTNAME" >/dev/null 2>&1; then
    PI_HOST="$PI_USER@$DEFAULT_PI_HOSTNAME"
  else
    PI_HOST="$PI_USER@$DEFAULT_PI_USB_IP"
  fi
fi

# Derive PI_USER from the (possibly overridden) PI_HOST before using it for
# PI_DIR, so a custom PI_HOST username and the remote home directory agree.
PI_USER="${PI_HOST%%@*}"
PI_IP="${PI_HOST#*@}"
PI_DIR="${PI_DIR:-/home/$PI_USER/ai-drone}"

UV="PATH=/home/${PI_USER}/.local/bin:\$PATH"
# Use the user's SSH config by default (so a Host entry with the deploy key is
# picked up automatically); fall back to /dev/null when there is no config.
DEFAULT_SSH_CONFIG="${HOME:-}/.ssh/config"
[[ -f "$DEFAULT_SSH_CONFIG" ]] || DEFAULT_SSH_CONFIG=/dev/null
SSH_CONFIG="${SSH_CONFIG:-$DEFAULT_SSH_CONFIG}"
SSH_CMD=(ssh -F "$SSH_CONFIG")
RSYNC_SSH="${SSH_CMD[*]}"

# ── 1. Sync ────────────────────────────────────────────────────────────────
echo "▸ Syncing to ${PI_HOST}:${PI_DIR}/ …"
rsync -az --delete \
  -e "$RSYNC_SSH" \
  --exclude .git \
  --exclude .venv \
  --exclude .ruff_cache \
  --exclude .pytest_cache \
  --exclude artifacts \
  --exclude __pycache__ \
  --exclude aitrios-rpi-sample-apps \
  --exclude Drone-Handbook.pdf \
  --exclude raspios-lite \
  ./ "${PI_HOST}:${PI_DIR}/"
echo "  ✓ sync done"

# ── 2. Install dependencies ───────────────────────────────────────────────
echo "▸ Installing raspi dependencies on the Pi …"
# The venv must be created with system site packages so apt-installed
# picamera2/libcamera are visible inside the project environment.
"${SSH_CMD[@]}" "${PI_HOST}" "cd ${PI_DIR} && ${UV} bash -lc 'if [[ ! -f .venv/pyvenv.cfg ]] || ! grep -qx \"include-system-site-packages = true\" .venv/pyvenv.cfg; then uv venv --clear --python /usr/bin/python3 --system-site-packages .venv; fi; uv sync --locked --python .venv/bin/python --no-dev --group raspi'"
echo "  ✓ deps installed"

# ── 3. Optional: run or shell ─────────────────────────────────────────────
EXTRA=""
if (( $# > 1 )); then
  printf -v EXTRA " %q" "${@:2}"
fi

case "${1:-}" in
  --picam)
    echo "▸ Starting the IMX500 AI stream on the Pi …"
    echo "  Open http://${PI_IP}:8080/ in your browser."
    echo "  Press Ctrl-C to stop."
    "${SSH_CMD[@]}" -t "${PI_HOST}" "cd ${PI_DIR} && .venv/bin/python test_picam.py${EXTRA}"
    ;;
  --lidar)
    echo "▸ Sampling the MTF-01P through the Pi's flight-controller link …"
    "${SSH_CMD[@]}" -t "${PI_HOST}" "cd ${PI_DIR} && .venv/bin/python test_lidar.py --device /dev/serial0${EXTRA}"
    ;;
  --ssh)
    echo "▸ Opening shell on the Pi …"
    exec "${SSH_CMD[@]}" -t "${PI_HOST}" "cd ${PI_DIR} && exec \$SHELL --login"
    ;;
  "")
    echo "Done. Use --picam, --lidar, or --ssh."
    ;;
  *)
    echo "Unknown flag: $1" >&2
    echo "Usage: $0 [--picam|--lidar|--ssh] [command options]" >&2
    exit 1
    ;;
esac
