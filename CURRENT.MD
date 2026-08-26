# Current status — 2026-08-25

## Work completed today

- The working branch is `current25`.
- Today's committed work enabled the FlywooF745 ArduCopter 4.7 optical-flow
  Loiter build, corrected its runtime firmware identity, captured the verified
  post-flash project-drone state, added a Loiter battery guard, clarified that
  the Syeed capture is reference-only, and added opt-in passive manual-flight
  recording.
- The current active-recorder work is **uncommitted**. No flight controller,
  Raspberry Pi, servo, or other live hardware was changed or actuated while it
  was developed.

## New AprilTag/servo recorder

The new entry point is `drone-tag-servo-record`. It is intended to attach to an
already armed project drone, record H.264 video, timestamps, camera/AprilTag
events, raw MAVLink telemetry, decoded telemetry, previews, and servo-command
events, while printing a live `DETECTED`/`NO_TAG` status.

It uses the payload servo connected directly to Raspberry Pi **BCM12**. It does
not use an ArduPilot servo output and never sends arm, mode, throttle, RC,
mission, motor, or flight-controller-servo commands. A tag must be present in
three consecutive fresh native-detector frames with Hamming distance 0 and the
required decision margin. Each distinct tag ID is latched for the entire run
and can command at most one short active -> rest -> detach cycle.

Intended counted form (not yet cleared for live use):

```bash
drone-tag-servo-record \
  --all-tags --stop-after 3 \
  --active-us <CALIBRATED_ACTIVE_US> \
  --rest-us <CALIBRATED_REST_US> \
  --pulse-duration <CALIBRATED_SECONDS> \
  --confirm-actuation SERVO_CLEAR \
  --confirm-armed-flight ARMED_FLIGHT_TAG_SERVO_CLEAR
```

Omit `--stop-after` to run until Ctrl-C, SIGTERM, vehicle disarm, or a detected
fault. Use repeated `--tag-id ID` arguments instead of `--all-tags` to restrict
which IDs may trigger. The deploy allowlist name is `tag-servo-record`.

## Validation and stop point

Before the final concurrency edit, the repository had 419 passing non-SITL
tests (2 deselected), 29 focused active-recorder tests, and clean Ruff, format,
type, dependency, import-boundary, lock-file, and diff checks. The latest edit
made the pending-to-completed tag latch atomic, but the user stopped the work
before that edit and the remaining review findings could be retested.

The recorder is therefore **not cleared for propeller-on or airborne use yet**.
The interrupted review still has these known issues to resolve:

- persist a confirmed-tag intent before exposing it to the actuator worker;
- close the actuation gate immediately on disarm/stop and recheck it atomically
  at the GPIO command boundary;
- keep SIGTERM/SIGHUP cleanup handlers installed until rest/detach completes;
- guarantee Ctrl-C cleanup during startup and shutdown, not only after READY;
- make the duration stop reason deterministic; and
- rerun the full validation suite after those fixes.

Before any live run, reverify `ARMING_SKIPCHK=0`, resolve the magnetic-field
pre-arm problem in a clean location, calibrate the exact active/rest pulse
widths and duration with propellers removed and no payload, validate servo
power/brownout behavior, and prove detection using real printed AprilTags. The
software deliberately has no default servo positions because those mechanism
values have not been measured safely.
