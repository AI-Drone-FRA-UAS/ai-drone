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

Two independent blockers were present: an inconsistent EKF source selection
and, decisively, features compiled out of the installed firmware. The captured
primary EKF source set was:

| Setting | Captured value | Meaning |
| --- | ---: | --- |
| `GPS1_TYPE` | `0` | GPS disabled |
| `GPS2_TYPE` | `0` | second GPS disabled |
| `EK3_SRC1_POSXY` | `3` | horizontal position must come from GPS |
| `EK3_SRC1_VELXY` | `5` | horizontal velocity comes from optical flow |
| `EK3_SRC1_POSZ` | `2` | rangefinder is the primary vertical source |

Optical flow is an EKF velocity source, not a `POSXY` source. With
`POSXY=3` and both GPS instances disabled, the source set asks for a GPS
position that cannot exist. AltHold can still work because it needs a vertical
estimate rather than horizontal position.

The project's captured `@ROMFS/hwdef.dat` then exposed the stronger
compile-time blocker in the installed image:

| Build feature | Installed value | Consequence |
| --- | ---: | --- |
| `AP_OPTICALFLOW_ENABLED` | `1` | the optical-flow front end exists |
| `AP_OPTICALFLOW_MAV_ENABLED` | `1` | MAVLink flow can be received, logged, and reported |
| `HAL_NAVEKF3_AVAILABLE` | `1` | EKF3 itself exists |
| `EK3_FEATURE_OPTFLOW_FUSION` | `0` | EKF3 cannot consume optical flow or enter flow-backed relative aiding |
| `MODE_GUIDED_NOGPS_ENABLED` | `0` | Copter mode 20 is absent, so the autonomous takeoff handoff cannot start |

This explains why valid `OPTICAL_FLOW` telemetry did not create a position
estimate. With `EK3_FEATURE_OPTFLOW_FUSION=0`, the EKF3 optical-flow write,
selection, terrain-estimator, and fusion paths are compiled out. The flow
front end can still make the sensor appear healthy in telemetry and logs, but
EKF3 remains in constant-position aiding and Loiter continues to reject entry.
Likewise, setting a parameter cannot restore mode 20 when
`MODE_GUIDED_NOGPS_ENABLED=0`.

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
- Optical-flow and downward-range messages are present. That proves only the
  sensor/front-end data path exists; it does not prove the installed EKF can
  fuse flow.
- A props-off 0.52-to-0.66 m hand lift still reported EKF flags `167` for the
  entire sample: no relative horizontal position and constant-position mode.
  The flow stream remained fresh and nonzero with quality 74 through 116, so
  increasing height could not bypass the compiled-out fusion path.

In the pinned source, Loiter declares `requires_position()`, its entry check
calls `position_ok()`, and no-GPS entry succeeds only after EKF3 reports a
relative horizontal position and leaves constant-position mode. That state is
unreachable in the installed feature build regardless of parameter tuning.
See the pinned
[`mode.cpp`](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/ArduCopter/mode.cpp),
[`system.cpp`](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/ArduCopter/system.cpp),
and
[`AP_NavEKF_Source.cpp`](https://github.com/ArduPilot/ardupilot/blob/1511f27194f1dcc3728270883047bdf022b3fd53/libraries/AP_NavEKF/AP_NavEKF_Source.cpp).

## What the Syeed reference aircraft does and does not prove

The `syeed-drone-2026-08-24/` reference aircraft's parameters do not prove
that it flew flow-only Loiter:

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

## What Lars's archived flight does prove

Lars's separately archived aircraft is useful evidence for the software path,
but not a configuration donor. Its ArduCopter 4.6.3 resolved `hwdef.dat` ends
by enabling both `EK3_FEATURE_OPTFLOW_FUSION` and
`MODE_GUIDED_NOGPS_ENABLED`. Its active source set also uses `POSXY=0` and
`VELXY=5`, the same horizontal-source correction reviewed for the project
aircraft.

In Lars log 2, the aircraft remained armed in Loiter for about 21 seconds while
every GPS sample reported status 1 and zero satellites. During that interval,
the EKF reported relative horizontal position, terrain altitude, and predicted
relative position, while constant-position and using-GPS flags remained clear.
Nonzero optical-flow innovations were also recorded. This independently shows
that no-GPS Loiter is available when the flow-fusion code is linked and the EKF
source set does not require GPS; it is not a 4.6-only capability.

The archive does not establish high-quality position or altitude hold. Its
recovered BIN contains gaps, and its rangefinder reports only about 0.02 to
0.26 m while controller altitude reaches about 0.96 m. No Lars calibration,
GPS, compass, RC, rangefinder, origin, motor, battery, or other hardware value
is copied to the project aircraft.

## Reviewed five-parameter live-controller delta

Only these five values were reviewed and applied to the captured project
controller:

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

These five corrections are necessary but not sufficient. They remove the
configuration-level contradictions and unsafe fallback choices, but no
parameter can enable code omitted from the firmware. The flow-fusion and
`GUIDED_NOGPS` firmware features described below must also pass linked-artifact
and post-flash verification before the controller can attempt this flight
path.

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

## Exact FlywooF745 firmware build and verification gate

A plain FlywooF745 build is not suitable for this aircraft's required flight
path. The installed 4.7 image and the board's flash-constrained feature
selection omitted EKF3 optical-flow fusion and `GUIDED_NOGPS`. Firmware version
`4.7.0` and Git identity `1511f271` alone therefore do not prove that two
artifacts have the same capabilities.

[`firmware/FlywooF745-nogps-loiter-extra.hwdef`](../firmware/FlywooF745-nogps-loiter-extra.hwdef)
contains the complete 820-directive feature matrix recovered from this
project controller's ROMFS. It deliberately retains every captured feature
choice and changes exactly these two values:

```text
EK3_FEATURE_OPTFLOW_FUSION  0 -> 1
MODE_GUIDED_NOGPS_ENABLED   0 -> 1
```

It does not copy board pins or hardware defaults: those continue to come from
ArduPilot's `FlywooF745` board definition. `EK3_FEATURE_OPTFLOW_SRTM` remains
disabled, and the MAVLink optical-flow and rangefinder backends remain the
only relevant external sensor paths. A comparison of the captured resolved
ROMFS definition with the candidate's resolved `hw.dat` found only the two
changes above.

Build from the exact initialized ArduPilot checkout. The local firmware build
image uses the official GNU Arm Embedded
`gcc-arm-none-eabi-10-2020-q4-major` toolchain downloaded from ArduPilot's
`Tools/STM32-tools` firmware archive; the SITL-only image has no ARM compiler:

```bash
cd /home/abaris/ardupilot
test "$(git rev-parse HEAD)" = \
  1511f27194f1dcc3728270883047bdf022b3fd53
git submodule update --init --recursive

podman run --rm --userns=keep-id \
  -v /home/abaris/ardupilot:/ardupilot \
  -v /home/abaris/ai-drone:/config:ro \
  localhost/ardupilot-firmware:4.7.0 \
  bash -lc './waf configure --board FlywooF745 \
    --extra-hwdef=/config/firmware/FlywooF745-nogps-loiter-extra.hwdef && \
    ./waf copter'
```

Do not use `--consistent-builds` for the deployable artifact. At this pinned
revision it intentionally replaces the runtime MAVLink custom-version value
with `abcdef`, while leaving the APJ Git identity as `1511f271`. The companion
firmware gate then correctly refuses to fly. The deployable build must report
`1511f271` both in APJ metadata and in `AUTOPILOT_VERSION` after flashing.

The output is
`/home/abaris/ardupilot/build/FlywooF745/bin/arducopter.apj`. Verify the linked
ELF rather than trusting source defines or build output alone:

```bash
cd /home/abaris/ai-drone
UV_CACHE_DIR=/tmp/uv-cache \
  uv run --group dev python scripts/verify_ardupilot_firmware.py \
    --ardupilot-root /home/abaris/ardupilot --nm nm

sha256sum \
  /home/abaris/ardupilot/build/FlywooF745/bin/arducopter \
  /home/abaris/ardupilot/build/FlywooF745/bin/arducopter.bin \
  /home/abaris/ardupilot/build/FlywooF745/bin/arducopter.apj \
  /home/abaris/ardupilot/build/FlywooF745/hw.dat \
  firmware/FlywooF745-nogps-loiter-extra.hwdef
```

The verifier requires the reviewed hashes in
[`firmware/FlywooF745-nogps-loiter.manifest.json`](../firmware/FlywooF745-nogps-loiter.manifest.json),
board ID 1027, APJ magic `APJFWv1`, Git identity `1511f271`, an image no larger
than 950,272 bytes, and matching APJ metadata and flash limits. It strictly
decodes the APJ image and requires it to be byte-identical to `arducopter.bin`.
It also verifies EKF3, MAVLink optical-flow and rangefinder support, linked
EKF3 optical-flow fusion, and linked `GUIDED_NOGPS`, while requiring
`EK3_FEATURE_OPTFLOW_SRTM` to remain absent.
The reviewed runtime-identity-preserving artifact has:

| Item | Reviewed value |
| --- | --- |
| APJ board ID | `1027` |
| Git identity | `1511f271` |
| APJ image size | `865792` bytes |
| image limit / flash total | `950272` bytes |
| free space | `84480` bytes |
| ELF SHA-256 | `498186052d8fa6bd78f047b3c48eff2e47c33ccf50922e91d62fc37339c21d36` |
| BIN SHA-256 | `3a410c8142f0ce91ca8f509634f264411b027399bb5bebf0d95234b32929adee` |
| APJ SHA-256 | `d8ab397bd41093a0669e36b0faf06af1845ad60b7280846e92a54be535400a04` |
| resolved `hw.dat` SHA-256 | `d89b4db7acd2811284c420fb79f0750661f8dfca6865bedf1725a17dfac4babe` |
| overlay SHA-256 | `21d270a4f0f0da12c8c5cfa5d14c4b305e03d72741736c4a47aa10363529d7ae` |

Do not flash a differently sized or hashed artifact merely because it reports
the same version. Re-run the verifier and review any intentional source,
toolchain, or feature-matrix change first.

## Exact pinned SITL acceptance gate

The SITL board includes capabilities that the captured flash-constrained
FlywooF745 image did not. A passing SITL run therefore validates the flight
logic, sensor injection, and failsafes only after the exact-board linked ELF
passes the firmware gate above; it is not evidence that an arbitrary physical
artifact contains flow fusion or mode 20.

Do not apply the FlywooF745's complete board feature matrix directly to SITL:
it contains target-specific HAL choices such as disabled on-board networking.
[`firmware/sitl-nogps-loiter-extra.hwdef`](../firmware/sitl-nogps-loiter-extra.hwdef)
explicitly enables only the same two corrected application capabilities and
therefore preserves the standard SITL board and simulator hardware defaults.
Build that simulator artifact with:

```bash
podman run --rm --userns=keep-id \
  -v /home/abaris/ardupilot:/ardupilot \
  -v /home/abaris/ai-drone:/config:ro \
  localhost/ardupilot-sitl:4.7.0 \
  bash -lc './waf configure --board sitl \
    --extra-hwdef=/config/firmware/sitl-nogps-loiter-extra.hwdef && \
    ./waf copter'
```

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
`2 passed in 108.91s`. The final explicit two-feature SITL build passed the
same pair again in `108.90s`.

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

The targeted Pi deployment and five-parameter controller write were completed
disarmed and propellers-off following the 2026-08-24 capture. The replacement
firmware described above has been built but was not flashed as part of that
deployment. The raw captures and live audit bundles are kept outside the
repository under `/home/abaris/drone-logs/`; neither aircraft's raw capture is
part of this branch.

The verified live state is:

- Pi runtime files from commit `cf6642b` match the developer-host SHA-256
  values and import successfully. No flight task was started.
- A fresh pre-write bundle downloaded all 1,172 parameters and matched the
  private project-FC hardware identity recorded in the local backup, official
  ArduCopter `4.7.0`, and custom version `1511f271`.
- The guarded writer changed exactly the five reviewed values. A complete
  before/after comparison found only those changes and the normal
  `STAT_RUNTIME` increment.
- Two accepted controller-only reboots incremented `STAT_BOOTCNT` once each.
  The FC identity, all five values, and the complete reviewed invariant set
  survived both reboots.
- Disarmed pre-arm checks passed, the mode remained STABILIZE, and the bench
  observer sent no arm, mode, motor, throttle, RC-override, mission, or servo
  command. Downward range reported 2 cm with a 100 cm maximum, optical-flow
  quality was 48 through 70, and every RC report had zero channels.
- A subsequent props-off lift held the aircraft at 0.52 through 0.66 m and
  captured 500 fresh flow samples at about 10 Hz. Quality was 74 through 116,
  flow was nonzero during translation, range was plausible, and every RC
  report still showed zero channels. Nevertheless, every EKF report remained
  at flags `167`: constant-position mode with no relative horizontal position.
  This is the expected result from the installed image's compiled-out flow
  fusion and is not grounds for more parameter tuning.
- The fresh-battery hand lift reported 16.246 through 16.254 V, above the
  application's 14.4 V minimum. Pack chemistry, independent voltage, and
  battery-monitor calibration still require operator verification before any
  propeller-on test.

The final snapshot differs from the pre-reboot snapshot in the expected boot
and runtime counters and boot-time barometer/gyro calibration values.
`SCHED_OPTIONS` also returned from 1 to its persisted value 0: reading
`@SYS/tasks.txt` during the raw capture calls ArduPilot's `task_info()`, which
temporarily enables its task-information bit with `_options.set()` but does not
save it. The reviewed writer did not touch this parameter; the complete
pre-reboot comparison proves it changed only during reboot.

The gate used for that deployment, with the remaining steps retained, is:

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
7. The pre-flash props-off lift described above established healthy range and
   flow-front-end data but failed the relative-position gate exactly as the
   installed feature matrix predicts. Do not repeat it as a tuning exercise.
   Repeat it only after the reviewed firmware passes the post-flash identity
   and feature checks below.
8. Verify signal handling and the GCS heartbeat/failsafe state while disarmed.
   No bench check may send arm, throttle, RC override, mode-change, mission,
   motor, or servo commands.

The firmware-specific post-flash gate is read-only and propellers-off:

1. Read ROMFS `hwdef.dat` twice using independent connections with an accepted
   reboot between them. Require matching reads and the reviewed complete
   feature matrix, including `EK3_FEATURE_OPTFLOW_FUSION=1` and
   `MODE_GUIDED_NOGPS_ENABLED=1`.
2. Request `AVAILABLE_MODES` and require custom mode 20 (`GUIDED_NOGPS`) to be
   advertised. Do not select it, and confirm the current mode did not change.
3. Compare the complete parameter set with the pre-flash backup and again
   after reboot. Persistent configuration must be unchanged, including the
   five reviewed values and all board, calibration, output, serial, and battery
   settings. Investigate every unexplained difference.
4. Run pre-arm checks without arming. Then repeat the above-0.5 m hand lift
   over a lit, textured floor. Require explicit optical-flow-fusion and
   started-relative-aiding evidence, EKF `AID_RELATIVE`,
   `EKF_POS_HORIZ_REL` set, `EKF_CONST_POS_MODE` clear, and fresh plausible
   local horizontal position, alongside healthy flow/range and zero RC
   channels.
5. Set the aircraft down and confirm the controller remained disarmed and no
   actuator or mode command was sent. Preserve the complete post-flash audit.

Failure of any item blocks propeller installation and all autonomous tests.

## Additional gate before any propeller-on autonomous test

The following cannot be proven by SITL or a remote disarmed inspection:

1. Independently measure the flight battery. The replacement pack reported
   16.246 through 16.254 V during the latest hand lift and passed the production
   command's 14.4 V software guard, but telemetry alone does not establish pack
   chemistry, cell balance, or monitor calibration. Confirm those before
   selecting battery failsafe actions; do not guess the values.
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
- [Limited-flash firmware](https://ardupilot.org/copter/docs/common-limited-firmware.html)
- [Custom Firmware Builder](https://ardupilot.org/copter/docs/common-custom-firmware.html)
- [Loading custom firmware](https://ardupilot.org/planner/docs/common-loading-firmware-onto-pixhawk.html)
- [GCS failsafe](https://ardupilot.org/copter/docs/gcs-failsafe.html)
- [Dead-reckoning failsafe](https://ardupilot.org/copter/docs/deadreckoning-failsafe.html)
- [Pre-arm safety checks](https://ardupilot.org/copter/docs/common-prearm-safety-checks.html)
