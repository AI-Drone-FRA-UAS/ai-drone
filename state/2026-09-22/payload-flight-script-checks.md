# Payload script checks — 2026-09-22

Both scripts were tested on the Pi over Tailscale with the real camera, native
AprilTag detector and FC telemetry. Physical servo output was disabled using
`GPIOZERO_PIN_FACTORY=mock` and `GPIOZERO_MOCK_PIN_CLASS=mockpwmpin` for these
tests only. No ID 6 was observed and no physical release was attempted.

- Direct `scripts/tag_mount_capture.py --device /dev/serial0 --tag-id 6
  --duration 15`: completed; 120 analysed camera frames, 402 encoded frames,
  2012 telemetry messages; `stop_reason=duration_elapsed`, no errors.
  Pi dataset: `artifacts/tag6-direct-test-20260922`.
- Launcher `bash scripts/payload_flight.sh start 6`, followed by `stop`:
  final corrected run completed after 12.961 seconds; 97 analysed frames,
  351 encoded frames, 1736 telemetry messages;
  `stop_reason=operator_signal_SIGTERM`, no errors.
  Pi dataset: `artifacts/sensor-recordings/20260922-141637`.

The launcher initially used SIGINT. Two stop tests exited with status 130 and
did not write a final manifest. The Debian AprilTag binding temporarily resets
SIGINT during detection (see `scripts/native/apriltag-gil.patch`). The launcher
now uses `KillSignal=SIGTERM` and `KillMode=mixed`, reaching the recorder's
existing cooperative shutdown handler. This was verified on the Pi by checking
`completed=true` and the final stop reason. ShellCheck and shell syntax checks
passed. The user's start and stop commands are unchanged.

Launcher tests selected a temporary uv wrapper through PATH to supply mock GPIO
and a 40-second fallback duration. The wrapper was removed after each test; no
production mock-GPIO configuration was installed. The watcher was left stopped.

The earlier camera timeout did not recur during these tests. Its cause remains
unconfirmed; these checks do not establish physical ID 6 detection and release
or flight readiness.

Signal-forwarding reference: [uv documentation](https://docs.astral.sh/uv/concepts/projects/run/#signal-handling).
Context7 supplied current upstream documentation, not a version-pinned 0.12.5
reference; the installed Pi runtime and actual stop tests determined the result.
