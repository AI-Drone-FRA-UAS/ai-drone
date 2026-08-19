# Live flight-controller configuration snapshots

`drone-config-sync` reads the complete indexed MAVLink parameter set through the
Pi's `/dev/serial0` flight-controller link. It never sends `PARAM_SET`, changes a
flight mode, arms, or drives an actuator.

Capture locally for review:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-config-sync
```

The examples prefer Tailscale MagicDNS. When Tailscale is offline and the
laptop is joined to `AI-Drone-Zero`, use `PI_HOST=seb@192.168.4.1` instead.

The command synchronizes the current code to the Pi, performs a disarmed
parameter download with missing-index retries, transfers one checksummed JSON
bundle back over SSH, and writes:

```text
params/flywoo-f745-live-YYYY-MM-DD.param
state/YYYY-MM-DD/drone-config.json
```

To commit exactly those two generated files and push the current branch:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-config-sync --publish
```

`--publish` requires a clean worktree before capture. This prevents unrelated
local changes from entering an automated hardware-state commit. GitHub
credentials remain on the developer computer; they are not copied to the Pi.

The exporter verifies that every announced parameter index was received and
that names and indexes are unique. The developer-side program recomputes the
parameter-file SHA-256 before writing or publishing it.
