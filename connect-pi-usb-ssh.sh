#!/usr/bin/env bash
# Compatibility wrapper. Prefer: uv run autoconnect
# (autoconnect tries Tailscale, then the Pi's AI-Drone-Zero AP, then USB.)
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
exec uv run autoconnect "$@"
