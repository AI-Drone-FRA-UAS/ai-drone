# ArduCopter 4.7 no-GPS Loiter review

This review applies only to the project aircraft captured on 2026-08-24 and to
official ArduCopter 4.7.0 commit
`1511f27194f1dcc3728270883047bdf022b3fd53`. The
`syeed-drone-2026-08-24/` capture is comparison material from a different
aircraft and is not an upload source.

The intended first live mission is deliberately small: take off to 0.5 m,
wait for a flow-backed relative position, hold Loiter briefly, and land. The
companion has a hard 0.8 m software ceiling. The aircraft has only a downward
rangefinder; there is no forward-facing lidar and this design does not claim
forward obstacle detection or avoidance.

SITL is an acceptance gate for the firmware configuration and deployable
control path. It does not by itself authorize a propeller-on flight or validate
the real airframe, motor installation, flow alignment, lighting, floor texture,
rangefinder geometry, or battery.

## Why Loiter failed on the project aircraft

The captured primary EKF source set is internally inconsistent:

| Setting | Captured value | Meaning |
| --- | ---: | --- |
| `GPS1_TYPE` | `0` | GPS disabled |
| `GPS2_TYPE` | `0` | second GPS disabled |
| `EK3_SRC1_POSXY` | `3` | horizontal position must come from GPS |
| `EK3_SRC1_VELXY` | `5` | horizontal velocity comes from optical flow |
| `EK3_SRC1_POSZ` | `2` | rangefinder is the primary vertical source |

Optical flow is an EKF velocity source, not a `POSXY` source. With
`POSXY=3` and both GPS instances disabled, EKF3 can receive flow velocity but
cannot produce the relative or absolute horizontal-position state required by
Loiter. AltHold can still work because it needs a vertical estimate rather
than horizontal position.

The project capture supports that diagnosis:

- DataFlash log 10 records three `Mode change to Loiter failed: requires
  position` messages.
- The USB telemetry capture reports EKF flags `167`: attitude, horizontal and
  vertical velocity, vertical position, and constant-position mode. It does
  not report relative or absolute horizontal position.
- The logs contain `Need Position Estimate`, `GPS 1: Bad fix`, and
  `AHRS: waiting for home` messages. Log 10 was recorded while `GPS1_TYPE=1`;
  the final disarmed capture has GPS disabled. Neither state supplied a usable
  horizontal position when Loiter was requested.
- Optical-flow and downward-range messages are present. That proves the data
  path exists, but cannot repair the contradictory source selection.

In the pinned source, Loiter declares `requires_position()`, its entry check
calls `position_ok()`, and no-GPS entry succeeds only after EKF3 reports a
relative horizontal position and leaves constant-position mode. See the pinned
[`mode.cpp`](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/ArduCopter/mode.cpp),
[`system.cpp`](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/ArduCopter/system.cpp),
and
[`AP_NavEKF_Source.cpp`](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/libraries/AP_NavEKF/AP_NavEKF_Source.cpp).

## What the other aircraft does and does not prove

The reference aircraft's parameters do not prove that it flew flow-only
Loiter:

| Setting | Project | Reference | Consequence |
| --- | ---: | ---: | --- |
| active source-1 `POSXY` | `3` | `3` | both primary sets request GPS |
| active source-1 `VELXY` | `5` | `3` | project mixes flow velocity with GPS position |
| `GPS1_TYPE` | `0` | `1` | only the reference has GPS enabled |
| stored flow source set | source 3 | source 2 | both include `POSXY=0`, `VELXY=5` |
| EKF source selector | none | `RC7_OPTION=90` | only the reference can select its flow set by RC |

The reference could have used GPS Loiter or selected source 2. Its capture does
not distinguish those cases. Its origins, GPS and RC setup, sensor offsets,
calibrations, failsafes, and hardware parameters must not be copied.

The two aircraft already have effectively the same horizontal Loiter and
position-controller gains (`LOIT_*` and `PSC_NE_*`). Retuning them cannot make
a missing EKF position state appear. Small rangefinder mounting differences
are likewise not the cause.

## Reviewed five-parameter live-controller delta

Only these five values are proposed for the captured project controller:

| Parameter | Captured | Reviewed | Reason |
| --- | ---: | ---: | --- |
| `EK3_SRC1_POSXY` | `3` | `0` | stop requiring nonexistent GPS position; let flow velocity build a relative position |
| `EK3_SRC1_POSZ` | `2` | `1` | use barometer height while the downward rangefinder supplies flow scale/terrain distance |
| `FS_DR_ENABLE` | `2` | `1` | LAND on dead-reckoning loss instead of attempting indoor RTL |
| `FS_OPTIONS` | `16` | `8` | remove the pilot-mode GCS-failsafe bypass and retain continue-if-already-landing |
| `RNGFND1_MAX` | `8` | `1.0` | bound the intended downward-range/flow operating envelope to 1 m |

The machine-readable overlay is
[`params/project-drone-4.7-nogps-loiter-delta.param`](../params/project-drone-4.7-nogps-loiter-delta.param).
It intentionally contains no board, serial-port, sensor-calibration,
motor-output, battery-monitor, RC, compass, origin, or reference-aircraft
settings.

`EK3_SRC1_POSZ=1` is not what fixes horizontal Loiter. It follows ArduPilot's
normal optical-flow setup: the barometer is the vertical EKF source and the
rangefinder supplies height above the surface for optical-flow scaling. It is
also consistent with the project's 2026-08-21 evidence, in which barometer
height stopped a badly diverging rangefinder-primary vertical estimate.

`RNGFND1_MAX=1.0` is not a hard flight ceiling by itself. ArduPilot uses the
rangefinder limit when deciding where optical-flow navigation is usable; the
companion independently commands LAND if either fresh downward range or its
range-aligned local altitude exceeds 0.8 m. The 0.2 m margin is intentional.
`AVOID_ENABLE=2` remains unchanged because ArduPilot's flow height limiting is
gated by nonzero avoidance configuration. It does not create forward obstacle
detection, and no forward distance stream is configured or simulated.

These captured or migrated values are required invariants, not additional
blind writes:

```text
AHRS_EKF_TYPE,3
AHRS_OPTIONS,16
ARMING_NEED_LOC,0
ARMING_SKIPCHK,0
AVOID_ENABLE,2
EK3_ENABLE,1
EK3_FLOW_USE,1
EK3_SRC1_VELXY,5
EK3_SRC1_VELZ,0
EK3_SRC1_YAW,1
EK3_SRC_OPTIONS,0
FLOW_TYPE,5
FRAME_CLASS,1
FRAME_TYPE,1
FS_CRASH_CHECK,1
FS_EKF_ACTION,1
FS_EKF_THRESH,0.8
FS_GCS_ENABLE,5
FS_GCS_TIMEOUT,5
FS_THR_ENABLE,0
FS_VIBE_ENABLE,1
GPS1_TYPE,0
GPS2_TYPE,0
GUID_OPTIONS,0
LAND_SPD_MS,0.15
MAV_GCS_SYSID,255
RNGFND1_ORIENT,25
RNGFND1_TYPE,10
RNGFND2_TYPE,0
WP_SPD_UP,0.25
```

`ARMING_SKIPCHK=0` is exact and means no pre-arm category is skipped.
`FS_THR_ENABLE=0` is also an exact invariant for the captured, receiver-free
topology. It avoids an inapplicable `RC not found` pre-arm failure; it is not a
waiver of any `ARMING_SKIPCHK` category. Installing a receiver requires a new
review rather than silently changing this value.
`LAND_SPD_MS=0.15` preserves the project's captured gentle final descent rate;
the standard SITL value is 0.5 m/s, so leaving the simulator default in place
would test a materially faster touchdown than the real setup.
`AHRS_OPTIONS=16` permits the project aircraft's own stored origin to be
restored when GPS is absent; never replace its origin with coordinates from the
reference aircraft. Relative optical-flow Loiter does not require a global
position, but Auto, RTL, global Guided destinations, and meaningful home
handling do. No EKF source-switch RC option is needed when source 1 is the
reviewed permanent GPS-less source.

## ArduCopter 4.7 autonomous takeoff and Loiter handoff

The deployable path is in
[`ai_drone/flight/controller.py`](../ai_drone/flight/controller.py) and exposed
by the guarded `hover` command in
[`ai_drone/cli/control.py`](../ai_drone/cli/control.py). It does not use the
old AltHold/RC workaround.

Before arming, the controller requires:

- exact official firmware version 4.7.0 and custom-version bytes `1511f271`;
- exact `ARMING_SKIPCHK=0` and every reviewed no-GPS invariant above;
- onboard DataFlash logging enabled;
- fresh disarmed heartbeat, downward range, nonzero-quality flow, attitude,
  and an `RC_CHANNELS` report confirming the captured zero-channel topology;
- a fresh battery voltage at or above the configured minimum (14.4 V by
  default); a low or missing reading refuses arming;
- a system-255 GCS heartbeat, sent every second for the configured GCS
  failsafe.

The flight sequence is:

1. Enter and confirm `GUIDED_NOGPS`, then arm through the normal ArduPilot
   checks.
2. Send continuous level `SET_ATTITUDE_TARGET` messages using the current yaw.
   The type mask ignores all three body-rate fields and uses the quaternion.
3. Keep `GUID_OPTIONS=0`. In ArduCopter 4.7 this makes the message's final
   field a normalized climb-rate request, not raw motor thrust: `0.5` holds
   altitude and `1.0` requests `WP_SPD_UP` upward.
4. For takeoff, send field `0.65`, which is 30% of the configured
   `WP_SPD_UP=0.25 m/s`, or 0.075 m/s. At 90% of the 0.5 m target, switch to
   the neutral `0.5` field and keep sending it.
5. Require fresh flow plus EKF horizontal velocity and
   `EKF_POS_HORIZ_REL`, with `EKF_CONST_POS_MODE` absent, continuously for one
   second. Only then request and confirm Loiter.
6. From the first takeoff setpoint onward, continuously require fresh
   `RC_CHANNELS.chancount=0` in every mode. A receiver appearing, that report
   becoming stale, or an RC mode switch taking the aircraft out of the expected
   mode requests LAND. In ArduCopter 4.7, a valid low-throttle RC input is a
   Loiter descent command and the configured mode channel also regains mode
   authority.
7. In Loiter, continuously monitor downward range, a separately reported local
   altitude aligned to the rangefinder's takeoff datum, heartbeat, flow, and
   relative EKF health. Fresh battery telemetry and the configured voltage
   minimum are enforced throughout every controller-owned flight phase.
   A stale or unsafe value requests LAND.
8. On completion, signal, guard failure, or exception, command LAND rather
   than force-disarm. LAND is resent once per second and telemetry remains
   monitored until the flight controller confirms disarm. Once the first
   takeoff setpoint has been sent, cleanup never force-disarms a possibly
   airborne vehicle.

The 0.5 m takeoff target and 0.8 m ceiling are enforced by the CLI as well as
the controller. The CLI refuses targets above 0.6 m and refuses a configured
ceiling above 0.8 m. During the Loiter monitor, battery voltage below the
default 14.4 V guard requests LAND. The battery-monitor calibration and pack
chemistry still need live verification; they are deliberately absent from the
five-parameter overlay.

`MAV_CMD_NAV_TAKEOFF` was not retained for this aircraft. In exact 4.7 SITL,
AltHold accepted the command but still required pilot throttle to spool and
lift. `GUIDED_NOGPS` also acknowledged that command without executing a climb
through its attitude-only run loop. Continuous `SET_ATTITUDE_TARGET` is the
ArduCopter 4.7 interface that actually exercises the production no-position
takeoff path.

## ArduCopter 4.6-to-4.7 compatibility

The firmware converts many parameters during its first boot, but that does not
update repository overlays, Python code, scripts, or documentation. External
consumers must use 4.7 names, units, and semantics.

Relevant changes found in this repository include:

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

The dated 4.6 files under `params/` remain historical evidence and must not be
loaded wholesale into 4.7. The production controller now checks
`ARMING_SKIPCHK`, never the removed `ARMING_CHECK`. See ArduPilot's
[parameter-name change table](https://ardupilot.org/copter/docs/common-param-name-changes.html).

## Exact pinned SITL acceptance gate

The opt-in tests in [`tests/test_sitl.py`](../tests/test_sitl.py) refuse an
ArduPilot checkout whose `HEAD` is not
`1511f27194f1dcc3728270883047bdf022b3fd53`. They launch the built `arducopter`
binary with clean storage, ArduPilot's standard quad-X `x` physics model, its
standard Copter defaults, and only the targeted overlay in
[`tests/sitl/copter.parm`](../tests/sitl/copter.parm).

The test setup keeps GPS disabled (`GPS1_TYPE=0`, `GPS2_TYPE=0`, and
`SIM_GPS1_ENABLE=0`). It uses the project backend types (`FLOW_TYPE=5` and
`RNGFND1_TYPE=10`), disables SITL's internal flow backend, and supplies 20 Hz
MAVLink 2 `OPTICAL_FLOW` and downward `DISTANCE_SENSOR` messages on an
independent companion link. The injected samples are derived from `SIM_STATE`
truth with ArduPilot's flow geometry. No forward lidar is injected.

An early test appeared to complete but was not a valid airborne hold: standard
SITL synthesized an RC receiver at minimum throttle even though the project
aircraft has no active RC input. That minimum pilot throttle drove Loiter
downward and could create a false-positive mode sequence. The SITL-only
`SIM_RC_FAIL=1` now represents the absent receiver and removes that synthetic
pilot command. It is not present in the live-controller delta, does not disable
`ARMING_SKIPCHK=0`, and must never be uploaded to hardware. The overlay also
sets `FS_THR_ENABLE=0`, matching the already captured live value; unlike the
`SIM_*` setting, production code verifies this real-controller invariant.

The production-path test calls the same CLI and controller intended for the
Pi; it does not reproduce arm, mode, takeoff, Loiter, or LAND with test-only
MAVLink helpers. Its acceptance criteria require all of the following:

- externally observed `GUIDED_NOGPS -> LOITER -> LAND` transitions;
- an actual armed interval, continued arming throughout Loiter, and final
  confirmed disarm;
- simulator-truth altitude reaching at least 90% of the 0.5 m target, never
  reaching 0.8 m, and remaining at or above 0.3 m during Loiter;
- fresh injected flow and range data, plus `EKF_POS_HORIZ_REL` throughout
  Loiter without `EKF_POS_HORIZ_ABS` or `EKF_CONST_POS_MODE`;
- maximum Loiter horizontal displacement no greater than 0.5 m;
- no `requires position`, stopped-aiding, EKF-failsafe, or simulator-ground-hit
  status;
- one completed flight-recording manifest from the production CLI.

A second failure-path test kills the production process after Loiter entry so
its cleanup cannot run. It then requires ArduCopter's system-255 heartbeat
failsafe (`FS_GCS_ENABLE=5`, `FS_GCS_TIMEOUT=5`, `FS_OPTIONS=8`) to select LAND
and finish disarmed.

Three complete final executions against the exact checkout passed both tests:

| Execution | Clean-flight XY drift | Clean Loiter minimum | Maximum altitude | Forced-loss result |
| --- | ---: | ---: | ---: | --- |
| final run | 0.028 m | 0.520 m | 0.530 m | GCS failsafe, LAND, 0.148 m/s touchdown, disarmed |
| independent repeat | 0.030 m | 0.520 m | 0.520 m | GCS failsafe, LAND, 0.148 m/s touchdown, disarmed |
| settled-tree repeat | 0.027 m | 0.520 m | 0.520 m | GCS failsafe, LAND, 0.148 m/s touchdown, disarmed |

Both observed only `RC_CHANNELS.chancount=0`, used the external MAVLink 2 flow
and range streams, and remained armed and above 0.3 m throughout Loiter. The
full pairs completed as `2 passed in 109.08s` and `2 passed in 108.56s`.
The final repeat after adding the pre-arm battery and mode-retention guards was
`2 passed in 108.91s`.

Run the full acceptance gate with:

```bash
cd /home/abaris/ai-drone
ARDUPILOT_ROOT=/home/abaris/ardupilot \
  UV_CACHE_DIR=/tmp/uv-cache \
  uv run --group dev pytest -m sitl -vv -s
```

Higher-fidelity physics still requires measured total mass, dimensions,
inertia, and motor/propeller thrust. Until those are supplied, SITL altitude
and drift bounds validate software behavior rather than predict the physical
aircraft.

## Deployment and disarmed verification gate

Do not install the changes or write flight-controller parameters until the
current project-aircraft state is backed up and the repository backup decision
is complete. Deployment remains disarmed and propellers-off:

1. Capture a new full parameter file, firmware identity, mission/fence/rally
   state, and hashes. Confirm it is the project aircraft, not the reference.
2. Dry-run the Pi deployment and review every changed path. Deploy only the
   targeted runtime source; preserve the Pi's environment, virtual
   environment, recordings, and artifacts. Do not start a flight command.
3. Confirm the deployed files and Python environment import cleanly, and that
   the guarded CLI help works without connecting to actuators.
4. With a fresh disarmed heartbeat, write the five reviewed FC values one at a
   time and read back each value immediately. Write no other captured or
   reference parameters.
5. Reboot the flight controller, reconnect, verify official 4.7.0 commit
   `1511f271`, and read back the complete reviewed invariant set. Reboot and
   read back a second time to prove persistence.
6. Run ArduPilot's pre-arm checks without arming. Resolve every status message;
   do not change `ARMING_SKIPCHK=0` to suppress one.
7. With props still removed, hold the aircraft above a well-lit, textured floor
   within the intended range and translate it gently. Require fresh, plausible
   downward range, nonzero-quality correctly signed flow, fresh attitude, EKF
   horizontal velocity and `EKF_POS_HORIZ_REL`, and no
   `EKF_CONST_POS_MODE`. Also confirm repeated `RC_CHANNELS` reports remain at
   `chancount=0`. Do not infer a range-scaling fault from the prior log: the
   aircraft was not actually held at 0.9 m.
8. Verify signal handling and the GCS heartbeat/failsafe state while disarmed.
   No bench check may send arm, throttle, RC override, mode-change, mission,
   motor, or servo commands.

## Additional gate before any propeller-on autonomous test

The following cannot be proven by SITL or a remote disarmed inspection:

1. Charge and independently measure the flight battery. The last observed
   approximately 14.09 V is below the production command's 14.4 V default
   guard. Confirm pack chemistry/cell count and calibrate the battery monitor
   before selecting battery failsafe actions; do not guess those values.
2. Verify frame integrity, propeller type and orientation, motor order and
   direction, center of gravity, sensor mounting, and vibration isolation with
   props removed first.
3. Calibrate flow axes and `FLOW_FXSCALER`/`FLOW_FYSCALER` over the actual
   floor, height, and lighting. Confirm the downward rangefinder remains valid
   throughout 0 to 0.8 m. Camera/AprilTag health is useful for the later grid
   mission but is not part of Loiter stabilization.
4. Provide an independently tested pilot takeover or emergency LAND method.
   The capture showed no active RC input and `FS_THR_ENABLE=0`; do not enable an
   RC failsafe until a receiver is installed, calibrated, and tested props-off.
   The current controller deliberately refuses Loiter when receiver channels
   are present, so adding RC also requires a reviewed control-policy and SITL
   update before flight.
5. Use a netted, clear test area with a safety observer, no people in the
   flight volume, a textured level floor, adequate lighting, and lateral room
   well beyond the expected drift. Keep an operator at the independent stop
   control throughout.
6. First run only the bounded 0.5 m takeoff, short Loiter hold, and LAND
   sequence. Abort on unexpected tilt, climb, flow loss, EKF loss, mode
   rejection, GCS heartbeat loss, or altitude disagreement. Confirm final
   disarm before anyone approaches.
7. Review the companion recording and new DataFlash log before increasing
   duration or attempting the 2 m by 2 m AprilTag mission. Do not add grid
   navigation or servo actuation to the first hover test.

## Prior branch finding

`origin/preflight-and-nogps-takeoff` must not be merged wholesale. Its short
apparent hover was a LAND response after STABILIZE failed to lift: the recorded
modes were STABILIZE and LAND, never Loiter. A bad vertical estimate caused the
LAND controller to command unexpectedly high output, and a later run reached
the ceiling. Its custom Python vehicle was a protocol double with an
unconditionally healthy, GPS-backed EKF rather than ArduCopter SITL, and it
used removed 4.6 `ARMING_CHECK` semantics.

The useful evidence from that branch is limited to the improvement from
barometer height and the need to become airborne before flow-backed relative
position can be handed to Loiter. It is not evidence of a successful no-GPS
Loiter hover.

Primary ArduPilot references:

- [Optical-flow sensor setup](https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html)
- [Loiter mode](https://ardupilot.org/copter/docs/loiter-mode.html)
- [Non-GPS navigation](https://ardupilot.org/copter/docs/common-non-gps-navigation-landing-page.html)
- [GCS failsafe](https://ardupilot.org/copter/docs/gcs-failsafe.html)
- [Dead-reckoning failsafe](https://ardupilot.org/copter/docs/deadreckoning-failsafe.html)
- [Pre-arm safety checks](https://ardupilot.org/copter/docs/common-prearm-safety-checks.html)
