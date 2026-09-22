# Pi update and focus preview — 2026-09-22

Reviewed session `01a0c3d4-eea9-7ac3-a1e7-25d0ea329183`, current
`461bb1e` source and the existing uncommitted changes. All application files
already matched the Pi before this session. Redeployed the current runtime and
locked dependencies, including the deployment correction below. The Pi retains
system CPython 3.13.5 and its environment with system camera bindings.

- Deployment initially stopped before mutation because lgpio's `.lgd-nfy0`
  notification FIFO could not be copied. Deployment now preserves these
  transient files. The existing transaction test covers a real FIFO.
- All 1,868 host tests passed (40 SITL cases deselected); the 86 deployment tests
  passed again after the fix. Lint, formatting, types, seven import contracts,
  dependency checks and the 13-page documentation build passed. Sandbox socket
  denials required rerunning host tests outside the sandbox.
- The new print helper generated valid A3 sheets and decoded IDs 0, 3, 17 and
  586. Corrected the range helper's implied-tag-size calculation to use only
  matched observations; a temporary synthetic recording verified missing-range
  handling. These remain host tools, outside the Pi runtime payload.
- Disarmed checks received expected ArduCopter 4.7.1 / `dbe79216`, IMU,
  optical flow and both rangefinders. Pre-arm diagnostics reported
  `PreArm: Check mag field (xy diff:279>100)`. No flight readiness is claimed.
- OS package metadata was refreshed and an upgrade was simulated: 84 updates,
  eight new packages, zero removals. **No OS packages were upgraded and no
  reboot occurred.** The user clarified that a stable focus preview is the
  immediate requirement. Python 3.14 and shared-service migration remain deferred.

At 14:41 CEST, started the existing passive `ai-drone-walk.service` for 1,800
seconds, using `/dev/serial0`, OpenCV detection every tenth frame, no saved
video, and a 1280×960 / 10 fps preview. A fetched JPEG was inspected. Logs
confirmed advancing frames, fresh disarmed heartbeats, `throttled=0x0` and
69.8°C. A laptop SSH tunnel exposes the verified preview at
`http://localhost:8081/`. It ends around 15:12 CEST; the restart/stop procedure
is in [Operations](../../docs/OPERATIONS.md#camera-focus-preview).

Pi dataset: `/home/seb/ai-drone/artifacts/walk-20260922T124130Z-21598bc3aa18`.
Detailed session logs are `/tmp/ai-drone-20260922-*` on the laptop.
The physical lens has not been adjusted; the user will turn its white focus
tool while viewing a target at the intended working distance. No actuator
commands, FC parameter changes or firmware flashing occurred.

## Temporary colour focus view, 14:53 CEST

At the user's request, stopped the first preview gracefully (its report was
finalized) and started `ai-drone-focus.service`. It runs the temporary
`/tmp/ai-drone-focus-colour.py` adapter for one hour using the installed passive
recorder and its existing disarmed checks. Installed application files and
camera/detection coordinates are unchanged; only the displayed image rotates
180 degrees. The same laptop tunnel and preview URL remain valid.

The adapter converts the complete YUV420 buffer to colour, draws decoded tag
outlines/IDs, and displays downward reading validity, detection frequency over
the last ten seconds, last-seen age, central-image Laplacian sharpness and camera
FocusFoM. Detection frequency is an observed hit rate, not a confidence estimate.
Sharpness is comparative only for the same target, distance and lighting.
Synthetic checks covered colour/rotation, stale/invalid range values and tag
overlay; a live colour JPEG was inspected and subsequent disarmed telemetry
remained fresh. Preview processing reduced actual camera capture to about 5 fps.

At 14:53:53, the downward sensor returned zero (invalid; configured 0.02–1.00 m),
no tags were decoded, central sharpness was 36.6 and FocusFoM was 1229. The image
looked across the hall with floor tags at a shallow angle; the user was advised
to face a printed tag toward the camera at the intended working distance.
Live measurements: `/run/user/1000/ai-drone-focus-status.json` on the Pi.
Stop this temporary view with `sudo systemctl stop ai-drone-focus.service`.
