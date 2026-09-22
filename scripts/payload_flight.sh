#!/usr/bin/env bash
# Start/stop colour video, telemetry and one-shot AprilTag payload release.
set -euo pipefail

project_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
unit="ai-drone-tag-mount.service"

case "${1:-help}" in
    start)
        tag_id="${2:-6}"
        if (( $# > 2 )) || [[ ! "$tag_id" =~ ^[0-9]{1,3}$ ]] || (( 10#$tag_id > 586 )); then
            echo "Usage: bash scripts/payload_flight.sh start [tag36h11 ID: 0–586]" >&2
            exit 2
        fi
        tag_id=$((10#$tag_id))
        if (( EUID == 0 )); then
            echo "Run this as the Pi user (seb), without sudo." >&2
            exit 2
        fi
        uv_bin="$(command -v uv)"
        if [[ ! -x "$project_dir/.venv/bin/python" ]]; then
            echo "The installed project Python environment is missing." >&2
            exit 1
        fi
        # /run is cleared on reboot; direct GPIO commands share this lock directory.
        sudo -n install -d -m 0700 -o "$(id -u)" -g "$(id -g)" /run/ai-drone-locks
        # Debian's native AprilTag binding resets SIGINT during detection.
        # SIGTERM reaches the recorder's cleanup handler; signal uv only once.
        sudo -n systemd-run --unit="$unit" --collect \
            --uid="$(id -u)" --gid="$(id -g)" \
            --working-directory="$project_dir" \
            --property=KillSignal=SIGTERM --property=KillMode=mixed \
            --property=TimeoutStopSec=60 \
            --setenv=PYTHONUNBUFFERED=1 \
            "$uv_bin" run --no-sync --python "$project_dir/.venv/bin/python" \
            python scripts/tag_mount_capture.py --device /dev/serial0 \
            --tag-id "$tag_id" --resolution 1280x960 --fps 30 --bitrate 8000000 \
            --decimate 4.0 --stream
        echo "Started. Watch for READY: bash scripts/payload_flight.sh logs"
        echo "Tag $tag_id can open the mount even while disarmed. Recording runs until stopped."
        ;;
    stop)
        sudo -n systemctl stop "$unit"
        echo "Stopped. Check the final recording summary: bash scripts/payload_flight.sh logs"
        ;;
    status)
        systemctl status "$unit" --no-pager
        ;;
    logs)
        journalctl -u "$unit" -n 40 -f
        ;;
    help|--help|-h)
        echo "Usage: bash scripts/payload_flight.sh {start [TAG_ID]|stop|status|logs}"
        echo "Default tag: 6 (tag36h11). Opens once; no automatic close."
        echo "Records colour video and logs under artifacts/sensor-recordings until stopped."
        echo "The systemd job survives SSH disconnects and does not start on boot."
        ;;
    *)
        echo "Unknown action: $1. Use start, stop, status or logs." >&2
        exit 2
        ;;
esac
