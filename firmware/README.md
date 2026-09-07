# FlywooF745 no-GPS firmware artifacts

These files describe the reviewed ArduCopter 4.7.0 build for the project's
FlywooF745 controller, pinned to commit
`1511f27194f1dcc3728270883047bdf022b3fd53`. They are not defaults for another
aircraft.

The August 24 installed image omitted EKF3 optical-flow fusion and
`GUIDED_NOGPS`. The reviewed replacement was installed on August 25; its
effective ROMFS definition matched again in the [September 7
verification](../state/2026-09-07/indoor-repair.md).
Version and Git identity alone do not establish an image's compiled features.

## Files and canonical instructions

| File | Purpose |
| --- | --- |
| [FlywooF745 overlay](FlywooF745-nogps-loiter-extra.hwdef) | Preserve the captured feature matrix, enabling flow fusion and GUIDED_NOGPS |
| [SITL overlay](sitl-nogps-loiter-extra.hwdef) | Enable those two capabilities while preserving standard simulator hardware |
| [Reviewed manifest](FlywooF745-nogps-loiter.manifest.json) | Exact source identity, image limits and ELF/BIN/APJ/definition hashes |

The Flywoo overlay retains 820 effective directives and changes only
`EK3_FEATURE_OPTFLOW_FUSION` and `MODE_GUIDED_NOGPS_ENABLED` from 0 to 1.
Board pins/defaults still come from ArduPilot's board definition;
`EK3_FEATURE_OPTFLOW_SRTM` stays disabled. Never apply the whole Flywoo feature
matrix to SITL.

Use the single maintained [exact-board build and verification
procedure](../docs/ARDUCOPTER_4_7_NOGPS_LOITER.md#exact-flywoof745-firmware-build-and-verification-gate),
then the [pinned SITL acceptance
gate](../docs/ARDUCOPTER_4_7_NOGPS_LOITER.md#exact-pinned-sitl-acceptance-gate).
The [historical five-parameter correction](../docs/ARDUCOPTER_4_7_NOGPS_LOITER.md#reviewed-five-parameter-live-controller-delta)
explains the accompanying EKF/failsafe changes; use a fresh project capture
before deciding any live delta.

The verifier checks the linked ELF, exact manifest hashes, APJ board ID 1027,
`APJFWv1`, runtime identity 1511f271, decoded BIN equivalence and image limits.
The reviewed image occupies 865,792 of 950,272 bytes. It requires EKF3, MAVLink
range/flow, flow fusion and GUIDED_NOGPS, and rejects linked optical-flow SRTM.

Do not use `--consistent-builds`: at this revision it changes the runtime
custom-version identity and fails the companion's firmware gate. A changed
source, compiler, overlay or resulting hash requires a new artifact review;
a matching version string is insufficient. Preserve the manifest's historical
baseline hash rather than replacing it with a later installed-image hash.

## Post-flash acceptance

Building and passing SITL do not authorize live flight. Before an authorized
flash, retain the project's current firmware, complete parameters,
mission/fence/rally state and identity. The controller must remain disarmed
with propellers removed.

After flashing, complete all of these without arm, throttle, RC override,
mode selection, mission-start, motor or servo commands:

1. Verify ArduCopter 4.7.0, Git identity 1511f271 and FlywooF745 identity.
   Download ROMFS `hwdef.dat` twice with an authorized reboot/reconnect
   between reads. Require identical effective features, including flow fusion
   and GUIDED_NOGPS enabled.
2. Request `AVAILABLE_MODES` and require advertised custom mode 20
   (`GUIDED_NOGPS`) without selecting it.
3. Compare complete parameter captures before/after flashing and after reboot.
   Preserve persistent settings and investigate every difference except
   documented advancing counters. Do not restore another aircraft's data.
4. Run normal pre-arm checks without arming. Require `ARMING_SKIPCHK=0`
   and resolve every failure.
5. With propellers removed, perform a controlled hand lift above 0.5 m over a
   well-lit textured floor and gentle translation. Require plausible fresh
   downward range, correctly signed flow, explicit EKF optical-flow fusion and
   started-relative-aiding evidence, `POS_HORIZ_REL` set, `CONST_POS_MODE`
   clear, and plausible local position. Repeated RC reports must remain zero
   channels. A healthy flow front end alone is insufficient.
6. Set down, confirm the aircraft remained disarmed and no actuator command was
   sent, and save the audit/hand-lift recording beside the project capture.

These gates validate firmware and sensor integration. Physical calibration,
mechanical readiness and the independently validated emergency arrangement in
the [flight procedure](../docs/PI_MAVLINK_CONTROL.md) remain required before
any armed test. The current build does not provide forward proximity support;
a forward sensor reading by itself would not establish obstacle avoidance.
