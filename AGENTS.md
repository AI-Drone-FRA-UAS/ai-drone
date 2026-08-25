# Repository agent notes

## Network access

- GitHub and Raspberry Pi network calls may fail or report misleading
  authentication errors inside the restricted execution sandbox.
- Before reporting GitHub access, SSH access, or authentication as unavailable,
  retry the relevant read-only command with `sandbox_permissions` set to
  `require_escalated`.
- GitHub CLI authentication for this repository is stored in the host keyring.
  A sandboxed `gh auth status` can report an invalid token even though the
  escalated command succeeds. Verified account: `Jannik99F`; Git operations use
  SSH; repository: `AI-Drone-FRA-UAS/ai-drone`.
- The Raspberry Pi hotspot address is `seb@192.168.4.1`. Use
  `ssh -F /dev/null seb@192.168.4.1` because a host-level SSH configuration file
  has previously had permissions that prevented the default client config from
  loading.

## Pi networking

- `ai-drone-network.service` tries `Xyz`, other saved auto-connect profiles,
  then the `AI-Drone-Zero` fallback hotspot. See `docs/pi-networking.md`.
- On the Pi, use `ai-drone-network status` or `ai-drone-network list`; switch
  with `sudo ai-drone-network auto`, `sudo ai-drone-network connect "PROFILE"`,
  or `sudo ai-drone-network hotspot`.
- Prefer Tailscale access with `ssh -F /dev/null seb@seb-is-pm` when online.

## Hardware safety

- Treat all live-hardware inspection as disarmed/read-only unless the user
  explicitly requests an armed or actuator test.
- Sensor recording and camera tests must not send arm, mode-change, motor,
  throttle, RC override, mission-start, or servo commands.

## Drone data provenance

- `syeed-drone-2026-08-24/` is a capture from a different drone, retained only
  as reference material. It is not this project's drone and must not be used to
  infer the project's hardware, firmware, configuration, health, or safety
  state.
- Never upload, merge, or copy parameters, missions, fences, calibration data,
  firmware defaults, or other configuration from `syeed-drone-2026-08-24/` to
  the project drone unless the user explicitly requests a reviewed individual
  value.
- The project's own recorded drone data is in `params/` and dated `state/`
  captures; use `docs/drone-project.md` and `docs/DRONE_CONFIGURATION.md` for
  the documented project hardware and topology.
