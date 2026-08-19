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

## Hardware safety

- Treat all live-hardware inspection as disarmed/read-only unless the user
  explicitly requests an armed or actuator test.
- Sensor recording and camera tests must not send arm, mode-change, motor,
  throttle, RC override, mission-start, or servo commands.
