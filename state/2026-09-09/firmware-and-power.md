# ArduCopter 4.7.1 and disconnect helpers

9 September 2026, project FlywooF745 and Raspberry Pi Zero 2 W. The user
confirmed removed propellers and both battery and USB power. The aircraft
remained disarmed. The separate [startup-tone record](startup-tone.md)
records the completed ESC melody update.

## Installed firmware and source

The official uploader erased, programmed and verified the reviewed custom
ArduCopter **4.7.1 / dbe79216** image on board **1027 / FlywooF745**, then
rebooted it. This build preserves all 410 extracted feature states from the
working custom 4.7.0 image, including EKF3 optical-flow fusion, MAVLink range
and flow, and GUIDED_NOGPS. Proximity support remains absent; a forward range
reading does not establish obstacle avoidance or autonomous room navigation.

| Artifact | SHA-256 |
| --- | --- |
| Installed APJ | `f1f96750a8ee1d5c5a0db00ec0dc7951591b1fa370601c8dc8aea02fd1688603` |
| Expected live ROMFS `hwdef.dat` | `d89b4db7acd2811284c420fb79f0750661f8dfca6865bedf1725a17dfac4babe` |
| Preserved custom 4.7.0 rollback APJ | `d8ab397bd41093a0669e36b0faf06af1845ad60b7280846e92a54be535400a04` |

The firmware source in the parent `ardupilot` checkout is clean on
`project/copter-4.7.1`, at official stable commit
`dbe792162d06cab66c3475fd5556bf7a120f119e`, with its pinned submodules.
The previous checkout reference is retained as
`backup/project-copter-4.7.0-20260909`. The previous build was preserved under
`../ardupilot-backups/20260909/build-4.7.0`; a separate rollback APJ copy is in
`artifacts/firmware-migration-20260909/rollback-4.7.0/FlywooF745/bin/`.
Rollback also requires the matching companion firmware gate and verification
of the aircraft's own configuration.

The parent SITL binary was rebuilt from this exact 4.7.1 commit with the
project's simulation overlay. Its SHA-256 is
`cd2a0ce7319ddbdbaa23f452889f858b351340a7508e20d003222a3cd518c961`.
All three current scenarios passed in an isolated network namespace:
downward-flow hover, forward-range coexistence, and GCS-loss LAND/disarm.
The latter is simulation only. No live mode selection, arming, flight or servo
test was performed during firmware acceptance.

## Configuration and live checks

The complete before/after captures each contain 1,187 parameters. The final
[parameter file](../../params/flywoo-f745-live-2026-09-09-4.7.1.param) and
[capture metadata](drone-config-4.7.1.json) preserve the reboot verification.
The expected
renames preserve the value 5: `PSC_JERK_NE` becomes `PSC_NE_JERK`, and
`PSC_JERK_D` becomes `PSC_D_JERK`. Boot/runtime counters advance. Startup gyro
offsets and `BARO1_GND_PRESS` are separately reviewed in the saved comparisons;
the comparison tool flags these rather than silently approving them. No
parameter file was uploaded. `ARMING_SKIPCHK=0`, motor mapping, UART3 pin swap,
no-GPS EKF configuration and both rangefinder configurations were preserved.
Temporary ESC passthrough was restored to `SERVO_BLH_AUTO=0` before flashing.

Two post-flash readbacks, separated by a further FC reboot, confirmed exact
runtime identity and identical expected 35,238-byte ROMFS. Fence and rally
stores remain empty. Mission download contains only the index-zero home item;
its altitude changed from 144.01 m to 143.97 m and then 144.04 m, while its
other fields stayed unchanged. Read-only enumeration received all 19 advertised
modes, including selectable GUIDED_NOGPS/custom mode 20, before and after the
second reboot. No mode was selected.

The exact source generates mission item zero from the current AHRS home;
it is not a stored navigation waypoint. Startup also recalculates ground
pressure and gyro offsets (`INS_GYR_CAL=1` is unchanged). These observed
differences are accepted explicitly in `reviewed-acceptance.json`; the strict
comparison reports remain unchanged and retain their flags. No historical
home altitude or calibration values were restored.

The new `drone-check --prearm` received forward/downward range, optical flow,
IMU, compass, pressure, attitude, EKF and battery data. At 15:33 CEST the
battery was 15.93 V, forward range approximately 2.11 m and downward range
0.02 m on the bench. All required sensor presence/enablement/health bits were
true. The FC still reported **`PreArm: Check mag field`**, with z-field
deviation approximately 295–296 mG against its 200 mG check. The command
therefore returned failure, correctly. The check was not bypassed or weakened.
The current placement, nearby magnetic material and calibration need physical
investigation before flight. Stationary bench readings do not validate
airborne optical-flow performance or the payload servo.

After the verification reboot, the normal Pi walkthrough completed in
**30.006 seconds**, with 4,797 FC messages, 599 samples from each rangefinder,
599 optical-flow samples, 877 analyzed camera frames and 891 encoded video
frames. All these collectors reported `ok`, and the report finalized normally.
The camera's bench view was nearly uniform at this very low height; this
establishes acquisition and encoding, not focus or AprilTag performance.
No tags were detected and no metric vision calibration was claimed.
The copied [local browser report](../../artifacts/firmware-migration-20260909/walk-final/review/index.html)
includes CSVs and the camera files; the complete dataset remains on the Pi at
`~/ai-drone/artifacts/firmware471-walk-20260909-final/`.

The final Pi-UART `drone-check --prearm` again confirmed the expected firmware
and live sensors, reporting **15.815 V** and the same magnetic-field warning.
`drone-power status` and live `prepare fc-usb --pi-power-independent` passed.
The latter left the Pi running on the user-confirmed independent supply. Full
shutdown was previewed with `prepare all --dry-run`; its command and failure
paths are covered by offline tests. No Pi shutdown or cable removal was
performed as part of these final checks.

## Commands

From the laptop's project directory:

```bash
uv run --locked drone-power status
uv run --locked drone-check --prearm
```

Choose the removal you intend to prepare:

```bash
uv run --locked drone-power prepare battery
uv run --locked drone-power prepare fc-usb
uv run --locked drone-power prepare pi-usb
uv run --locked drone-power prepare all
```

By default these finish the known recorder, require fresh disarmed telemetry
and idle package/hardware state, and request a clean Pi shutdown. Keep power
connected until shutdown physically completes. SSH disappearing proves
neither clean shutdown nor independent power. If another verified Pi supply
will remain connected, the per-invocation `--pi-power-independent` option
allows `battery`, `fc-usb` or `pi-usb` preparation without shutting down the Pi.
It cannot be used for `all`. The Pi's power-only USB feed is not observable
reliably enough for software to infer this confirmation.

See [power command details](../../docs/POWER_CONTROL.md),
[FC checks](../../docs/DRONE_CHECK.md) and the
[30-second walkthrough](../../docs/SENSOR_RECORDING.md).

## Reproducible local evidence

The final maintained code passed **798 offline tests** with two optional tests
skipped. The three SITL cases passed separately. Formatting, Ruff, types,
import contracts, dependency declarations, whitespace checks and the
documentation build also passed. Live serial-owner inspection required a
portable `fuser` invocation; the tested fallback preserves USB detection while
using independently checked Pi-UART telemetry. Known owners or inconclusive
nonprivileged ownership checks block preparation.

Raw captures and firmware files are local ignored artifacts, not bundled into
Git. Main evidence is under `artifacts/firmware-migration-20260909/`:

- `fc-upload.log`, `flash-immediate-guards.json`, and full parameter captures.
- `result.json`, `migration-provenance.json`, and `sitl-tests.xml` for the
  rebuilt simulation and its exact source/artifact identity.
- `drone-check-after-flash.json` for the observed pre-arm blocker.
- `after-flash-comparison.json`, `after-reboot-comparison.json` and
  `reviewed-acceptance.json` for strict differences and their explicit review.
- `drone-check-pi-final.json`, `walk-final/`, `power-status-verified.log` and
  `power-prepare-fc-usb.log` for the Pi-side sensor/recording/power checks.
- Pi runtime backups, source hashes and deployment records. The existing Pi
  Python 3.13 environment and system Picamera2/libcamera bindings are retained;
  uv remains 0.12.11. The runtime contains no automatic FC boot service.

The first firmware/mission/ROMFS readback is under
`artifacts/software-update-20260909/firmware471-after-flash/`.
The matching second readback is under `firmware471-after-reboot/` alongside it.
Firmware build instructions and the manifest are in
[the firmware guide](../../firmware/README.md).
