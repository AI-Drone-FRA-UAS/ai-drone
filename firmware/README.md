# FlywooF745 no-GPS Loiter firmware

This directory contains the reviewed feature overlay required to run the
project aircraft's optical-flow Loiter path on ArduCopter 4.7.0. It applies
only to the project's FlywooF745 controller. It is not configuration for the
Syeed reference aircraft.

## Why a custom feature build is required

The firmware installed during the 2026-08-24 project-drone capture identifies
as official ArduCopter 4.7.0 commit
`1511f27194f1dcc3728270883047bdf022b3fd53`, but version and commit do not
describe its compile-time feature matrix. Its recovered ROMFS `hwdef.dat`
enables the optical-flow and rangefinder MAVLink front ends while explicitly
disabling the two capabilities required by this flight path:

```text
AP_OPTICALFLOW_ENABLED=1
AP_OPTICALFLOW_MAV_ENABLED=1
AP_RANGEFINDER_MAVLINK_ENABLED=1
HAL_NAVEKF3_AVAILABLE=1
EK3_FEATURE_OPTFLOW_FUSION=0
MODE_GUIDED_NOGPS_ENABLED=0
```

Consequently, valid flow can appear in telemetry and DataFlash while never
reaching EKF3. The estimator cannot leave constant-position mode or establish
flow-backed relative horizontal position, so Loiter rejects entry. Copter mode
20 (`GUIDED_NOGPS`) is also absent from the installed firmware. Neither issue
can be fixed with parameters.

The five-value live parameter overlay in
[`../params/project-drone-4.7-nogps-loiter-delta.param`](../params/project-drone-4.7-nogps-loiter-delta.param)
is still necessary: it removes the nonexistent GPS position source, restores
barometer-primary height, and selects reviewed range and failsafe behavior.
It is insufficient until the firmware capabilities in this directory are
also installed and verified.

A plain/stock FlywooF745 build must not be substituted. The board's constrained
feature selection does not provide the required path, and a matching 4.7.0
version string or commit identity is not proof that it does.

## Reviewed overlay

[`FlywooF745-nogps-loiter-extra.hwdef`](FlywooF745-nogps-loiter-extra.hwdef)
preserves all 820 effective feature directives recovered from the project's
installed ROMFS and flips exactly:

```text
EK3_FEATURE_OPTFLOW_FUSION  0 -> 1
MODE_GUIDED_NOGPS_ENABLED   0 -> 1
```

Board pins and board defaults are not copied into the overlay; they continue
to come from the pinned ArduPilot `FlywooF745` hwdef. The resolved candidate
`hw.dat` differs from the captured resolved definition only at those two
feature values. `EK3_FEATURE_OPTFLOW_SRTM` remains disabled, as do unused
non-MAVLink optical-flow backends.

For provenance, the captured installed ROMFS `hwdef.dat` had SHA-256
`67404e7f31d096a010d810db2600617e9b4b8c287f017472136e299f5781e225`.
The raw capture remains private under `/home/abaris/drone-logs/` and must not be
added to this repository.

[`FlywooF745-nogps-loiter.manifest.json`](FlywooF745-nogps-loiter.manifest.json)
records the exact reviewed build identity, intended two-value delta, flash
limits, installed-baseline provenance hash, and all build/input hashes.

## Reviewed exact-board build

Use an initialized ArduPilot checkout at exactly
`1511f27194f1dcc3728270883047bdf022b3fd53`. The local
`localhost/ardupilot-firmware:4.7.0` build image contains the official GNU Arm
Embedded `gcc-arm-none-eabi-10-2020-q4-major` compiler; the SITL-only image does
not contain the ARM toolchain. The compiler archive used to derive that local
image was
`https://firmware.ardupilot.org/Tools/STM32-tools/gcc-arm-none-eabi-10-2020-q4-major-x86_64-linux.tar.bz2`;
it was extracted under `/opt` with its `bin` directory placed on `PATH`.
The local image tag is mutable and its creation recipe is not committed here,
so this procedure is not by itself a bit-reproducible supply-chain record.
Acceptance depends on the pinned source, linked-feature inspection, APJ checks,
runtime identity, and exact reviewed hashes below. Treat any different rebuild
as a new artifact requiring review.

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

Do not add `--consistent-builds`. At this pinned revision that option
deliberately replaces the runtime `AUTOPILOT_VERSION.flight_custom_version`
with `abcdef`, even though APJ metadata retains `1511f271`. The project Pi's
firmware safety gate correctly rejects that runtime identity. A usable build
must report `1511f271` both in APJ metadata and over MAVLink after flashing.

The flashable result is:

```text
/home/abaris/ardupilot/build/FlywooF745/bin/arducopter.apj
```

## Artifact verification

Do not accept the artifact based only on successful compilation. Verify APJ
identity and size plus the features actually linked into the ELF:

```bash
cd /home/abaris/ai-drone
UV_CACHE_DIR=/tmp/uv-cache \
  uv run --group dev python scripts/verify_ardupilot_firmware.py \
    --ardupilot-root /home/abaris/ardupilot --nm nm
```

The verifier requires:

- APJ magic `APJFWv1`, FlywooF745 board ID `1027`, and Git identity
  `1511f271`;
- the embedded runtime version banner `ArduCopter V4.7.0 (1511f271)`;
- APJ `image_maxsize` and `flash_total` equal to the 950,272-byte application
  limit, a strictly decoded image of the declared size, and a payload
  byte-identical to `arducopter.bin`;
- exact SHA-256 matches for the ELF, BIN, APJ, resolved `hw.dat`, and reviewed
  overlay from the manifest;
- EKF3, optical-flow and rangefinder support, both MAVLink sensor backends,
  `EK3_FEATURE_OPTFLOW_FUSION`, and `MODE_GUIDED_NOGPS_ENABLED` in the linked
  ELF; and
- no linked `EK3_FEATURE_OPTFLOW_SRTM`.

The final reviewed runtime-identity-preserving artifact is:

| Item | Value |
| --- | --- |
| APJ board ID | `1027` |
| APJ magic | `APJFWv1` |
| Git identity | `1511f271` |
| image size | `865792` bytes |
| application limit | `950272` bytes |
| free space | `84480` bytes |
| ELF SHA-256 | `498186052d8fa6bd78f047b3c48eff2e47c33ccf50922e91d62fc37339c21d36` |
| BIN SHA-256 | `3a410c8142f0ce91ca8f509634f264411b027399bb5bebf0d95234b32929adee` |
| APJ SHA-256 | `d8ab397bd41093a0669e36b0faf06af1845ad60b7280846e92a54be535400a04` |
| resolved `hw.dat` SHA-256 | `d89b4db7acd2811284c420fb79f0750661f8dfca6865bedf1725a17dfac4babe` |
| overlay SHA-256 | `21d270a4f0f0da12c8c5cfa5d14c4b305e03d72741736c4a47aa10363529d7ae` |

Recheck the hashes directly:

```bash
cd /home/abaris/ai-drone
sha256sum \
  /home/abaris/ardupilot/build/FlywooF745/bin/arducopter \
  /home/abaris/ardupilot/build/FlywooF745/bin/arducopter.bin \
  /home/abaris/ardupilot/build/FlywooF745/bin/arducopter.apj \
  /home/abaris/ardupilot/build/FlywooF745/hw.dat \
  firmware/FlywooF745-nogps-loiter-extra.hwdef
```

Any intentional change to the source commit, compiler, overlay, or build
procedure requires a new review; do not flash an unexplained hash mismatch.

## Safe post-flash acceptance gate

Building and passing SITL do not authorize a propeller-on test. Before flashing,
retain the current firmware, full parameter set, mission/fence/rally state, and
identity in the private project-drone backup. Flashing itself requires explicit
operator approval and the controller must remain disarmed with propellers
removed.

After flashing, complete every gate below without sending arm, throttle,
RC-override, mode-selection, mission-start, motor, or servo commands:

1. Confirm ArduCopter 4.7.0, Git identity `1511f271`, and FlywooF745 board
   identity. Download ROMFS `hwdef.dat` twice, with an accepted reboot and
   reconnect between independent reads. Require the reads to agree and the
   effective feature matrix to match the reviewed build, including
   `EK3_FEATURE_OPTFLOW_FUSION=1` and
   `MODE_GUIDED_NOGPS_ENABLED=1`.
2. Request the MAVLink `AVAILABLE_MODES` report read-only. Require advertised
   custom mode 20 (`GUIDED_NOGPS`) without selecting it; keep the current mode
   unchanged throughout the check.
3. Download and compare the complete parameter set before and after flashing
   and after reboot. All persistent configuration values, including the five
   reviewed corrections and every hardware/calibration value, must remain
   unchanged. Investigate every difference rather than restoring data from the
   reference aircraft; only documented volatile counters may advance.
4. Run pre-arm checks without arming. `ARMING_SKIPCHK` must remain `0`, and no
   failed check may be suppressed to make the gate pass.
5. With propellers still removed, hold the aircraft above 0.5 m over a
   well-lit, textured floor and translate it gently. Require fresh plausible
   downward range, nonzero-quality correctly signed flow, and explicit EKF3
   optical-flow-fusion/started-relative-aiding evidence. The estimator must
   reach `AID_RELATIVE`; `EKF_STATUS_REPORT` must set `POS_HORIZ_REL`, clear
   `CONST_POS_MODE`, and provide fresh plausible local horizontal position.
   Repeated RC reports must continue to show zero channels.
6. Set the aircraft down and confirm it remained disarmed and no actuator
   command was sent. Save the post-flash audit and hand-lift telemetry beside
   the private project-drone capture for review.

No propeller-on autonomous takeoff, Loiter, or LAND test may proceed unless
all six gates pass. A healthy flow front end alone is specifically not an
acceptable substitute for relative-aiding and `POS_HORIZ_REL` evidence.
