#!/usr/bin/env bash
# Compatibility wrapper. Prefer: sudo scripts/setup-pi-hotspot.sh
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "$SCRIPT_DIR/scripts/setup-pi-hotspot.sh" "$@"
