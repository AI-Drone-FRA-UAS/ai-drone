# Armed AprilTag recording and payload-servo pulses

`drone-tag-servo-record` is the only recorder that permits BCM12 payload-servo
commands while the project flight controller may already be armed. The normal
`drone-inspect` command remains passive and refuses an initially armed vehicle.

This command does not arm, disarm, change flight mode, send motor/throttle or
RC overrides, start a mission, or use a flight-controller servo output. It
reads `ARMING_SKIPCHK`, requests bounded telemetry streams, records the selected
FC's incoming MAVLink traffic, and drives the documented payload servo directly
from Raspberry Pi BCM12.

## Required bench calibration

There are intentionally no defaults for the mechanism's active position, rest
position, or active duration. Before propeller-on use:

1. Remove every propeller, remove the payload, and secure the aircraft.
2. Verify a regulated servo supply and common Pi/servo ground under load. The
   servo must not brown out or reboot the Pi.
3. Use `drone-servo` to establish non-binding rest and active pulse widths.
4. Test the complete active → rest → detach sequence repeatedly with a printed
   tag and confirm that missing PWM is mechanically safe.
5. Run a camera-only printed-tag test under the actual hall lighting. Native
   AprilTag detection has not yet been live-validated with a tag in this
   project's recorded camera view.

Historical 900–2100 µs envelope values are only software bounds, not a
calibration of the installed linkage. Do not copy historical active/rest values
into a flight command without the bench test above.

## Counted run

After substituting the measured values, this stops after three **distinct tag
IDs have each completed** one pulse and returned to rest:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-deploy --run tag-servo-record -- \
  --all-tags --stop-after 3 \
  --active-us <MEASURED_ACTIVE_US> --rest-us <MEASURED_REST_US> \
  --pulse-duration <MEASURED_SECONDS> \
  --confirm-actuation SERVO_CLEAR \
  --confirm-armed-flight ARMED_FLIGHT_TAG_SERVO_CLEAR
```

Use repeatable `--tag-id ID` arguments instead of `--all-tags` to permit only a
reviewed allowlist. One ID can trigger at most once for the entire process, even
if it stays visible or disappears and reappears.

## Run until manually stopped

Omit `--stop-after` and `--duration`; press Ctrl-C to stop:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-deploy --run tag-servo-record -- \
  --tag-id 0 --tag-id 1 --tag-id 2 \
  --active-us <MEASURED_ACTIVE_US> --rest-us <MEASURED_REST_US> \
  --pulse-duration <MEASURED_SECONDS> \
  --confirm-actuation SERVO_CLEAR \
  --confirm-armed-flight ARMED_FLIGHT_TAG_SERVO_CLEAR
```

SIGTERM, SSH hangup, selected-vehicle disarm after arming, a stale FC heartbeat,
a stale detector worker, camera failure, or actuator error also stops the run.
SIGKILL and Pi power loss cannot run software cleanup, so the hardware must be
safe when PWM disappears.

## Detection and live status

Actuation requires the native AprilTag backend, Hamming distance zero, decision
margin at least 30, and the same ID in three consecutive fresh analyzed frames.
Frames older than 0.5 seconds are rejected. These thresholds can be tightened;
their CLI upper/lower bounds prevent disabling freshness and confirmation.

The terminal prints immediate JSON events for tag appearance/loss,
qualification, active/rest commands, PWM detach, and commanded completion. It
also prints a one-second status line such as:

```text
status ... apriltag=NO_TAG visible_ids=[] servo_progress=0/3 pending_ids=[]
status ... apriltag=DETECTED visible_ids=[7] servo_progress=1/3 pending_ids=[]
```

GPIO provides no physical position feedback. Logs therefore say “commanded”
and never claim that the linkage physically moved.

## Recorded dataset

Each run creates a unique directory containing:

- `camera.h264` and `camera.pts`;
- `camera.jsonl` with every analyzed frame and detection quality;
- `telemetry.tlog` and decoded `telemetry.jsonl`;
- `servo.jsonl`, synced after every actuator event;
- first/last preview images; and
- an atomic `manifest.json` containing stop reason, arm-state transitions,
  allowed/confirmed/completed IDs, pulse settings, message counts, and errors.

The command fails closed unless the heartbeat is the project ArduPilot
quadrotor at MAVLink target 1/1 and live `ARMING_SKIPCHK` is exactly zero.
