#!/usr/bin/env bash
# Compatibility wrapper. Prefer: uv run drone-deploy
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
exec uv run drone-deploy "$@"
