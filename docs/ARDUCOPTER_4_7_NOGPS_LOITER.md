# ArduCopter 4.7 no-GPS Loiter review

This review applies only to the project aircraft captured on 2026-08-24 and to
ArduCopter commit `1511f27194f1dcc3728270883047bdf022b3fd53`. The Syeed
capture is comparison material from another aircraft. None of its parameters
are an upload source.

The corrected setup passes a real ArduCopter 4.7 quad-X SITL flight: GPS is
disabled, all arming checks remain enabled, the vehicle lifts in AltHold,
obtains an optical-flow relative position, remains in Loiter for 20 seconds,
lands, and confirms disarm. This is simulation evidence, not authorization for
a live flight.

## Root cause

The project capture declares mutually inconsistent primary EKF sources:

| Setting | Project | Meaning |
| --- | ---: | --- |
| `GPS1_TYPE` | `0` | GPS disabled |
| `GPS2_TYPE` | `0` | second GPS disabled |
| `EK3_SRC1_POSXY` | `3` | horizontal position must come from GPS |
| `EK3_SRC1_VELXY` | `5` | horizontal velocity comes from optical flow |
| `EK3_SRC1_POSZ` | `2` | rangefinder is the primary vertical source |

Optical flow is a velocity source in EKF3; it is not a valid `POSXY` source.
With `POSXY=3` and no GPS, the estimator can fuse flow velocity but cannot
provide the relative or absolute horizontal-position state that Loiter
requires. ArduCopter therefore rejects the mode with `requires position`.
AltHold can still work because it requires a vertical estimate, not a
horizontal position.

The captured evidence agrees:

- DataFlash log 10 records three `Mode change to Loiter failed: requires
  position` messages.
- The USB telemetry capture reports EKF flags `167`: attitude, horizontal and
  vertical velocity, vertical position, and constant-position mode. Neither
  relative nor absolute horizontal position is present.
- The logs also contain `Need Position Estimate`, `GPS 1: Bad fix`, and
  `AHRS: waiting for home` messages. Log 10 was recorded while `GPS1_TYPE=1`;
  the later final disarmed capture has GPS disabled. In both states no usable
  horizontal position was available when Loiter was requested.
- Optical-flow messages and good downward-range samples are present. Their
  presence proves the data path, but it cannot repair the contradictory source
  selection.

In the pinned source, Loiter declares `requires_position()`, mode entry calls
`position_ok()`, and no-GPS operation succeeds only after EKF3 reports a
relative horizontal position and leaves constant-position mode. See
[`mode.cpp`](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/ArduCopter/mode.cpp),
[`system.cpp`](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/ArduCopter/system.cpp),
and
[`AP_NavEKF_Source.cpp`](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/libraries/AP_NavEKF/AP_NavEKF_Source.cpp).

## Comparison with the other aircraft

The reference aircraft is not evidence that its Loiter was flow-only:

| Setting | Project | Reference | Consequence |
| --- | ---: | ---: | --- |
| active source-1 `POSXY` | `3` | `3` | both primary sets request GPS |
| active source-1 `VELXY` | `5` | `3` | project mixes flow velocity with GPS position |
| `GPS1_TYPE` | `0` | `1` | only the reference has GPS enabled |
| flow source set | source 3 | source 2 | both contain `POSXY=0`, `VELXY=5` |
| EKF source selector | none | `RC7_OPTION=90` | only the reference can select its flow set by RC |

The reference could have flown Loiter on GPS, or it could have selected source
2. Its parameter capture alone cannot distinguish those cases. Copying its GPS,
RC mapping, origins, sensor offsets, failsafes, or calibration would be both
unnecessary and unsafe.

The horizontal Loiter and position-controller gains are already effectively
the same on both aircraft (`LOIT_*` and `PSC_NE_*`). Retuning them cannot make
a missing EKF position estimate appear. Small rangefinder clearance/offset
differences describe physical mounting and likewise are not the cause.

## Reviewed project-drone delta

The only causal horizontal change is:

```text
EK3_SRC1_POSXY,0
```

The reviewed delta also selects barometer height:

```text
EK3_SRC1_POSZ,1
```

This second change is independent of the horizontal Loiter failure. It follows
ArduPilot's normal optical-flow setup: the barometer controls altitude while
the downward rangefinder supplies height-above-ground for flow scale and
terrain. It is also supported by this project's 2026-08-21 tests, where
changing `POSZ` from rangefinder to barometer stopped a severely diverging
vertical estimate. A rangefinder-only height source is appropriate only over a
flat, unobstructed floor.

The two-line, project-specific file is
[`params/project-drone-4.7-nogps-loiter-delta.param`](../params/project-drone-4.7-nogps-loiter-delta.param).
It deliberately contains no board, serial, calibration, motor-output, battery,
RC, compass, origin, or other aircraft's values.

These captured project values are prerequisites and remain unchanged:

```text
AHRS_EKF_TYPE,3
AHRS_OPTIONS,16
ARMING_SKIPCHK,0
EK3_FLOW_USE,1
EK3_SRC1_VELXY,5
EK3_SRC1_VELZ,0
EK3_SRC1_YAW,1
EK3_SRC_OPTIONS,0
FLOW_TYPE,5
FRAME_CLASS,1
FRAME_TYPE,1
GPS1_TYPE,0
GPS2_TYPE,0
RNGFND1_TYPE,10
RNGFND1_ORIENT,25
```

`AHRS_OPTIONS=16` restores the project aircraft's own recorded origin when GPS
is not used. Pure Loiter uses relative position and does not require a global
origin, but Auto, RTL, global Guided commands, and meaningful home handling do.
The project already has nonzero recorded origin fields; never replace them with
the reference aircraft's coordinates.

No EKF source-switch RC option is needed for a permanently GPS-less setup:
source 1 itself becomes the reviewed flow source. This also avoids depending on
an RC channel that the companion-only project does not provide.

## What changed in ArduCopter 4.7

ArduPilot performs many conversions during the firmware's first boot, but that
does not update repository overlays, Python code, shell scripts, documentation,
or historical parameter files. External consumers must use the 4.7 names and
units.

Relevant 4.6-to-4.7 changes found in this repository are:

| 4.6 name | 4.7 name | Migration rule |
| --- | --- | --- |
| `ARMING_CHECK` | `ARMING_SKIPCHK` | semantics inverted; require exact `0` to skip nothing |
| `EK3_MAX_FLOW` | `EK3_FLOW_MAX` | rename |
| `RNGFND1_MIN_CM` | `RNGFND1_MIN` | centimetres to metres |
| `RNGFND1_MAX_CM` | `RNGFND1_MAX` | centimetres to metres |
| `RNGFND1_GNDCLEAR` | `RNGFND1_GNDCLR` | centimetres to metres |
| `PILOT_SPEED_UP/DN` | `PILOT_SPD_UP/DN` | rename and SI-unit conversion |
| `PILOT_ACCEL_Z` | `PILOT_ACC_Z` | rename and SI-unit conversion |
| `PILOT_TKOFF_ALT` | `PILOT_TKO_ALT_M` | rename and SI-unit conversion |
| `PSC_VELXY_*` | `PSC_NE_VEL_*` | rename; `IMAX` is also rescaled |
| `PSC_JERK_XY/Z` | `PSC_NE_JERK` / `PSC_D_JERK` | rename |
| `WPNAV_SPEED*` | `WP_SPD*` | rename and SI-unit conversion |
| `WPNAV_RADIUS` | `WP_RADIUS_M` | rename and SI-unit conversion |
| `WPNAV_ACCEL*` | `WP_ACC*` | rename and SI-unit conversion |

The old files under `params/` are dated 4.6 captures. They remain untouched as
historical evidence and must not be loaded wholesale into 4.7. The live 4.7
capture already contains the converted names and units.

Repository flight and motor-test code now requests `ARMING_SKIPCHK` and accepts
only `0`; no pre-arm category is skipped. The SITL test also asserts the exact
pinned firmware commit, so a later tag movement or checkout mismatch cannot
silently change the result. See ArduPilot's complete
[parameter-name change table](https://ardupilot.org/copter/docs/common-param-name-changes.html).

## SITL acceptance test

The opt-in test uses:

- exact ArduCopter commit `1511f27194f1dcc3728270883047bdf022b3fd53`;
- the standard `x` physics model and `FRAME_CLASS=1`, `FRAME_TYPE=1`;
- clean SITL storage and the small overlay in
  [`tests/sitl/copter.parm`](../tests/sitl/copter.parm);
- `GPS1_TYPE=0`, `GPS2_TYPE=0`, and `SIM_GPS1_ENABLE=0`;
- the project's physical backend types, `FLOW_TYPE=5` and
  `RNGFND1_TYPE=10`, with internal SITL flow disabled;
- an independent MAVLink 2 link that feeds 20 Hz `OPTICAL_FLOW` and downward
  `DISTANCE_SENSOR` messages calculated from `SIM_STATE` truth using
  ArduPilot's own flow equations;
- the EKF configured as `POSXY=0`, `VELXY=5`, `POSZ=1`;
- `ARMING_SKIPCHK=0` throughout.

The sequence intentionally starts in AltHold. Optical flow cannot be assumed to
have usable range/terrain geometry while the vehicle is on the floor, so trying
to arm or take off in Loiter can correctly fail. Once airborne, the test waits
for `EKF_POS_HORIZ_REL`, rejects absolute/GPS position and constant-position
mode, enters Loiter, and checks:

- mode remains Loiter and the vehicle remains armed for 20 seconds;
- optical-flow quality and downward range remain present;
- the EKF retains relative horizontal position without absolute position;
- horizontal drift is at most 0.5 m;
- altitude remains between 0.7 m and 1.3 m;
- LAND ends in a confirmed disarm.

Four consecutive explicit-MAVLink-2 runs completed successfully:

| Run | Maximum XY drift | Loiter altitude | Injected samples |
| --- | ---: | ---: | ---: |
| 1 | 0.026 m | 1.174–1.188 m | 987 |
| 2 | 0.027 m | 1.189–1.202 m | 993 |
| 3 | 0.027 m | 1.202–1.215 m | 989 |
| 4 | 0.027 m | 1.177–1.192 m | 991 |

Run it with:

```bash
cd /home/abaris/ai-drone
ARDUPILOT_ROOT=/home/abaris/ardupilot \
  UV_CACHE_DIR=/tmp/uv-cache \
  uv run --group dev pytest -m sitl -vv -s
```

The companion's read-only inspection path connects first, while the simulator
is disarmed, and confirms flight-controller, external flow, and external-range
telemetry. The flight portion uses a test-only RC override for the documented
AltHold-to-Loiter handoff. It does not weaken the live controller's sensor,
altitude, logging, or arming-check guards. Quad-X physics and unrelated
simulator hardware defaults remain untouched.

## Remaining live-aircraft work

A SITL pass validates firmware configuration and control logic, not the real
sensor installation or airframe physics. Before considering a live Loiter
test:

1. Back up and diff a fresh disarmed project-drone capture.
2. Review and apply only the two-line project delta; reboot and read it back.
3. Keep `ARMING_SKIPCHK=0` and resolve every pre-arm message.
4. Bench-check flow axes/signs against IMU gyros, then calibrate
   `FLOW_FXSCALER` and `FLOW_FYSCALER`; message presence alone is insufficient.
5. Verify a healthy downward range throughout the intended height envelope and
   a healthy compass/yaw source. Some captured logs contain compass variance or
   glitch messages.
6. Confirm `EKF_POS_HORIZ_REL` is present and constant-position mode is absent
   before requesting Loiter.
7. Account for `FS_GCS_ENABLE=5`: the deployed companion must supply the
   expected GCS heartbeat continuously or the approximately five-second GCS
   failsafe can trigger.
8. Use a restrained, pilot-controlled AltHold liftoff and Loiter handoff in a
   netted area. Do not reuse the prior branch's STABILIZE-to-LAND experiment.

Higher-fidelity SITL still requires measured mass, geometry, inertia, and
motor/propeller thrust. Until those are supplied, drift and altitude bounds are
software acceptance criteria rather than a prediction of the physical drone.

## Prior branch finding

`origin/preflight-and-nogps-takeoff` should not be merged wholesale. Its short
apparent hover was a LAND response after STABILIZE failed to lift; the logged
modes were STABILIZE and LAND, never Loiter. A bad vertical estimate caused the
LAND controller to command unexpectedly high output, and a later run hit the
ceiling. Its custom Python vehicle is a protocol double with an unconditionally
healthy, GPS-backed EKF, not ArduCopter SITL. It also uses removed 4.6
`ARMING_CHECK` semantics. The useful evidence retained here is the measured
improvement from barometer height and the need for an AltHold-to-position-mode
handoff after optical flow becomes valid.

Primary setup references:

- [Optical-flow sensor setup](https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html)
- [Loiter mode](https://ardupilot.org/copter/docs/loiter-mode.html)
- [Non-GPS navigation](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html)
- [Pre-arm safety checks](https://ardupilot.org/copter/docs/common-prearm-safety-checks.html)
