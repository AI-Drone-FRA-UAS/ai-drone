# Project drone state on 2026-08-25

This is a dated state record for the project aircraft only. Lars's and
Syeed's aircraft were comparison sources; none of their parameters,
calibrations, firmware defaults, missions, or hardware settings were uploaded.
Raw logs, firmware binaries, and synchronized camera/telemetry captures remain
outside the repository under `/home/abaris/drone-logs/`.

## Versioned configuration

The read-only post-flash download completed while disarmed in STABILIZE and
received all 1,172 advertised parameters:

- [full parameter snapshot](../../params/flywoo-f745-live-2026-08-25.param)
- [snapshot metadata](drone-config.json)
- parameter SHA-256:
  `77439ffde8ef3191ba26e263d21eeae15c5f86a603ac47f2efc5c4b0cb56e046`

The five reviewed no-GPS changes are `EK3_SRC1_POSXY=0`,
`EK3_SRC1_POSZ=1`, `FS_DR_ENABLE=1`, `FS_OPTIONS=8`, and
`RNGFND1_MAX=1`. `ARMING_SKIPCHK=0` remains unchanged, so no configurable
pre-arm category is skipped. The targeted delta and its rationale are in the
[ArduCopter 4.7 no-GPS Loiter review](../../docs/ARDUCOPTER_4_7_NOGPS_LOITER.md).

## Installed firmware and Pi

- Flight controller: FlywooF745, APJ board ID 1027.
- Firmware: the reviewed feature image built from official ArduCopter 4.7.0
  commit `1511f271`.
- Two ROMFS reads across an accepted reboot were byte-identical to the reviewed
  image. `EK3_FEATURE_OPTFLOW_FUSION=1` and
  `MODE_GUIDED_NOGPS_ENABLED=1`; `EK3_FEATURE_OPTFLOW_SRTM=0` remains
  disabled. `AVAILABLE_MODES` advertises mode 20 (`GUIDED_NOGPS`).
- Complete pre-flash and post-reboot comparisons preserved all 1,172
  parameters apart from the separately reviewed five-value write and expected
  boot/runtime observations.
- The Pi's deployed runtime source matches commit `cf6642b` and imports
  successfully. No live flight task was started.

## SITL acceptance

The exact pinned ArduPilot checkout passed both end-to-end tests again in
108.50 seconds:

- production `STABILIZE -> GUIDED_NOGPS -> LOITER -> LAND`, 0.520 m
  Loiter altitude, 0.028 m maximum horizontal drift, zero RC channels, no GPS,
  externally injected MAVLink 2 flow/range, and final disarm;
- forced companion-process loss in Loiter, followed by GCS failsafe LAND,
  0.148 m/s touchdown, and final disarm.

This validates the software path under standard quad-X SITL physics. It does
not validate the real airframe, magnetic environment, sensor calibration,
battery, mass/inertia, thrust, or propeller installation.

## Props-off live observations

The 40.010-second synchronized hand lift remained disarmed in STABILIZE. No
arm, mode, motor, throttle, RC-override, mission, or servo command was sent.

- Downward range moved from 0.02 m to at most 0.78 m. No forward rangefinder
  was present or expected.
- Optical flow arrived at 19.97 Hz without a quality-zero dropout. At or above
  0.5 m, quality was 90 through 116 with median 99.
- All 200 EKF reports were flags `367`: relative and predicted-relative
  horizontal position available, horizontal/vertical velocity available,
  constant-position and absolute-horizontal-position bits clear.
- Local position was finite, monotonic, and reset-free during translation.
  Rapid-lift vertical estimation lagged the raw range change by up to 0.405 m,
  and isolated flow transients reached 1.94 m/s, so this does not replace
  controlled range/flow scale and axis calibration.
- Every RC report had zero channels; throttle stayed zero and motor outputs 1
  through 4 stayed at 1000. Battery telemetry was 15.807 through 15.826 V.
- The downward IMX500 view was unobstructed. It recorded 1,119 encoded frames
  at about 28 fps and analyzed at about 11 fps. Images at 0.56 to 0.77 m were
  visibly soft and unevenly lit. No AprilTag was placed in view, so zero
  detections do not test detector correctness.

Private evidence:

- `/home/abaris/drone-logs/project-drone-post-1511f271-handlift-retry-20260825T0810Z/`
- `/home/abaris/drone-logs/project-drone-post-1511f271-stationary-health-post-handlift-2026-08-25_1011.json`

## Current blocker

The final production-heartbeat audit cleared the recorder's transient GCS
failsafe message, but the pre-arm health bit remained false for one reason:
`PreArm: Check mag field (xy diff:376-385>100)`. Lifted checks also failed at
274 and 185 mG. Historical project logs contain the same problem, so it was not
introduced by the firmware change.

The aircraft is not cleared for propeller-on testing. Keep
`ARMING_SKIPCHK=0` and `ARMING_MAGTHRESH=100`. Relocate the complete aircraft
to a magnetically clean area and repeat the resting pre-arm audit. If the error
persists, inspect the external compass mounting/orientation and nearby power
wiring, perform onboard compass calibration in a clean location, reboot, and
require a normal pre-arm pass before continuing.
