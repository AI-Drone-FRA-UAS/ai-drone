# Disarmed Pi checks — 2026-09-21

All access used `ssh seb@seb-is-pm`. The aircraft remained grounded and disarmed.
No live arm, takeoff, LAND, mode, motor, throttle, RC override, mission start,
servo or payload command was sent. No FC parameter/firmware write, application
service switch or broad OS upgrade was performed.

## Entry points and fresh authorization evidence

Before hardware use, the deployed `ai_drone.cli.power`, connection and checker
paths were inspected. Each check began with `power._remote_action('fc')` and
`power._require_disarmed`. With no shared runtime installed, this path opens the
UART read-only, observes selected FC heartbeats and does not transmit MAVLink.
The actual recorder/checker then performs its own heartbeat checks. The checker
was invoked without `--prearm`; its transmissions only requested telemetry,
version and parameter reads. Passive recorders requested message intervals.

Selected FC proof was refreshed for every check. Examples:

- Initial probe: two disarmed heartbeats, latest age 0.518 s.
- Refactored recorder: four disarmed heartbeats, age 0.267 s, battery 14.006 V.
- Full parameter capture: four disarmed heartbeats, age 0.915 s, battery 13.972 V;
  exporter also obtained a final fresh disarmed heartbeat before saving.

## Deployed baseline observations

The deployed source has the older individual console scripts and no unified
runtime service installed. It has no usable local Git metadata/tool for a live
`rev-parse`; local integration commit identity must not be assigned to it.
The existing `.venv` is system-site CPython 3.13.5. The network service was
inactive, and the walk service was not installed. `/dev/serial0` resolves to
`ttyAMA0`.

The ten-second disarmed checker **failed** its unchanged battery minimum:
14.17 V versus 14.4 V. Firmware identity matched ArduCopter 4.7.1 / `dbe79216`.
Reported health bits were healthy at that time; this is not airborne yaw proof.
Downward range was 0.02 m (ID 0, orientation 25), forward range about 2.23 m
(ID 1, orientation 0), flow quality 59, RC channel count zero and EKF flags 367.
RAW_IMU magnetic values were [-135, 5, 518] raw units. No physical survey motion,
calibrated heading reference or texture/lighting/height variation was available.

## Camera, recording and AprilTag processing

Both executions explicitly selected `operation='inspect'`, native AprilTag,
two threads, `/dev/serial0`, ten seconds, no flight permission and no tag-servo
operation. The actual source paths were inspected before execution. The passive
branch does not construct a servo session or GPIO object. It aborts on arming.

| Result | Deployed baseline | Refactored source `0988628` |
| --- | --- | --- |
| Capture duration | 10.004 s | 10.007 s |
| Analyzed camera frames | 113 | 118 |
| Telemetry messages | 1,585 | 1,586 |
| Tag detections | 0 | 0 |
| Servo pulses | 0 | 0 |
| Stop / exit | duration_elapsed / 0 | duration_elapsed / 0 |
| Whole process elapsed | 18.531 s | 18.613 s |
| User/system CPU | 19.332 / 4.184 s | 19.256 / 4.387 s |
| Peak RSS | 103,852 KiB | 105,916 KiB |
| End temperature | 68.756 °C | 70.370 °C |

This verifies acquisition, encoding, native processing execution and teardown.
Zero detections does **not** verify recognition, pose accuracy, calibrated
latency or navigation suitability. These short measurements are not thermal
soak, worst-case disk, full flight-control load or long-duration qualification.

Refactored source was imported from a separate immutable source directory using
the existing working 3.13 interpreter. Services and installed source were not
replaced. Both captures were copied locally, preserving remote originals. The
new offline magnetic/estimator report generated successfully from the second
capture. Its RAW and SCALED magnetic units remain explicitly distinct.

## Files and environments created on the Pi

- `/home/seb/ai-drone/artifacts/refactor-passive-20260921-native/`:
  deployed passive capture, video, tlog, JSONL, images and manifest.
- `/home/seb/ai-drone/artifacts/refactor-passive-20260921-0988628/`:
  equivalent refactored passive capture.
- `/home/seb/ai-drone-candidates/20260921-refactor/source-5c610c0/`:
  staged source only; no hardware check used this superseded candidate.
- `/home/seb/ai-drone-candidates/20260921-refactor/source-0988628/`:
  source used for the second capture and parameter exporter; no environment or
  service was switched to it.
- `/home/seb/ai-drone-candidates/20260921-refactor/fc-parameters.json`:
  read-only complete 1,187-parameter snapshot; deterministic parameter SHA-256
  `0df839a43f7d770048ec9b0773013ee6be125d950f303fd3c71b9589843f5738`.
- Separate standard Python 3.14 candidate interpreter/environment/cache described
  in [the candidate report](python-314-candidate.md).

Local evidence is retained under `artifacts/refactor-20260921/pi/`; dated logs
are included in the implementation evidence bundle. The parameter snapshot is
not a full FC image/mission/fence/rally recovery capture. Installed linked-feature
proof and a complete restoration capture remain separate qualification items.

## Later readback and access interruption

A guarded read-only ROMFS attempt opened `@ROMFS/hwdef.dat` and reported
35,238 bytes. The installed pymavlink 2.4.49 client stopped its burst after
10,640 bytes because its idle predicate measures time since the last **send**,
even while reply data arrives. The partial transfer is not feature proof.
Only FTP reset-session/open-read/read/burst-read/terminate-session opcodes were
allowed; every other FTP operation was blocked in the sender. Neither a file
write to the FC nor an actuator command was available in that path.

The last completed diagnostic probe reported four disarmed heartbeats, age
0.740 s, battery 13.866 V, and again established final disarmed state. It created
`/home/seb/ai-drone-candidates/20260921-refactor/romfs-download.partial`; the first
attempt also used pymavlink's `/tmp/temp_mavftp_file`. Neither is a complete
ROMFS artifact.

A further bounded readback attempt produced no result when SSH connectivity
was lost. An independent SSH status request timed out. The identified task SSH
client was terminated locally to prevent an unobserved resumed check. No fresh
FC state could then be established, so further hardware checks were deferred.
Creation/completion of its proposed `romfs-download-complete.partial` and
`romfs-hwdef.dat` outputs is unverified; neither may be treated as a valid readback.

`source-bb49adc/` was successfully staged under the same refactor candidate
directory before connectivity failed. It contains the final passive transport
byte meter, but **no hardware capture using it was run**. Exact direct RX/TX
measurement is implemented and mock-tested; earlier captures cannot establish
actual TX totals retroactively. The metrics capture, final power snapshot and
complete ROMFS readback remain outstanding until fresh disarmed access returns.

A subsequent bounded SSH-only `date -u; uptime` request also timed out connecting to
port 22 (`pi-final-connectivity-20260921.log`). It did not open the FC. No later
successful access was claimed at that checkpoint. Access subsequently returned,
as recorded below.

## Closing reconnection and final-source passive check

The Pi reconnected at 18:39 UTC. The deployed read-only power/ownership path was
inspected again. A new probe established two disarmed heartbeats, age 0.622 s,
and a complete ownership snapshot with no hardware users. The final source
`f5ff571` was staged separately at
`/home/seb/ai-drone-candidates/20260921-refactor/source-f5ff571/`.

The final recorder's entry point, operation dispatch, telemetry request path and
servo construction branch were inspected. It ran under working Python 3.13.5,
with `operation='inspect'`, native tags, two threads, ten seconds, no flight
permission and no servo construction. An additional sender allowlist admitted
only COMMAND_LONG/MAV_CMD_SET_MESSAGE_INTERVAL (511); exactly 25 such requests
were sent. Fresh read-only FC probes before/after reported four disarmed
heartbeats, ages 0.841/0.577 s. Final battery was 15.863 V, 90%; the earlier
low-battery checker failure remains historical evidence, not a current reading.

New capture directory:
`/home/seb/ai-drone/artifacts/refactor-passive-20260921-f5ff571/`.
The capture completed in 10.056 s with 73 analyzed frames, 240 encoded frames,
1,254 telemetry messages, zero tags and zero servo pulses. The exact direct UART
meter counted **77,991 RX / 1,075 TX bytes over 19.373 s**, including startup,
with zero receive/write/parser errors. The capture window's maximum received
heartbeat gap was 1.369 s and delivery lag 0.0001921 s; its sequence-gap estimate
was zero, which is not proof of zero physical packet loss. No control heartbeat
or setpoint was transmitted. This closes the short passive physical byte-meter
measurement, not active-control timing or sustained load qualification.

The wrapper elapsed time was 24.120 s, cumulative child user/system CPU
22.147/4.178 s, peak child RSS 113,272 KiB and final temperature 69.832 °C.
Cumulative child CPU includes the initial ownership probe; it is not isolated
recorder CPU. The final ownership snapshot was complete with no hardware users.
The capture was copied locally and an offline report generated. It does not
establish tag-recognition accuracy: all three real-camera captures found zero
tags. Installed source, working environment and services were unchanged.

## Complete closing ROMFS file and client cleanup failure

A new fresh probe established four disarmed heartbeats, age 0.508 s, battery
15.799 V. A bounded readback allowed only FTP opcodes 1/2/4/5/15 and rejected all
other outgoing MAVLink messages; UART ownership was locked and disarm monitored.
The client saved the complete 35,238-byte file, then failed in its internal
TerminateSession wait: the default 5 s timeout conflicts with the adjusted 25 s
idle threshold. The command therefore exited unsuccessfully; its traceback is
retained rather than reported as a passing client run. Final read-only proof
still established four disarmed heartbeats, age 0.399 s, battery 15.798 V, and
the complete ownership snapshot was clear.

The saved `/home/seb/ai-drone-candidates/20260921-refactor/romfs-hwdef-final.dat`
was independently retrieved from the Pi filesystem. Its exact bytes and SHA-256
`d89b4db7acd2811284c420fb79f0750661f8dfca6865bedf1725a17dfac4babe`
match the reviewed FlywooF745 candidate `hw.dat` and manifest. The transfer also
created `romfs-download-final.partial`; that name is retained even though the
client reached completion. This establishes one complete installed-ROMFS match,
alongside the separately verified saved ELF/BIN/APJ. It is not full installed
flash-byte proof or the required reboot-separated repeat; no reboot or flash was
performed. Local copies and failed/successful observations are preserved in the
evidence bundle.

## Final-source disarmed checker

The final `f5ff571` checker passed with zero errors/warnings in 20.907 s,
including its ten-second observation window. Its unchanged battery minimum
was met: 15.732 V versus 14.4 V. Firmware 4.7.1 / `dbe79216` and required
parameters matched. Downward range was 0.02 m (ID 0, orientation 25), forward
2.06 m (ID 1, orientation 0), flow quality 63, RC channel count zero and EKF
flags 367. Magnetic RAW_IMU values were [-124, 149, 623] in raw wire units;
these stationary observations are not calibrated heading or a hall survey.

The inspected checker ran without `--prearm`. An additional sender guard
allowed only parameter reads, message-interval requests and version-message
requests: 48 PARAM_REQUEST_READ, eight command 511 and two command 512 writes.
Fresh read-only proof preceded the check; final proof reported four disarmed
heartbeats, age 0.737 s, battery 15.730 V. The final ownership snapshot was clear.
The earlier low-battery failure is retained; this new successful observation
does not grant flight clearance. The JSON result is preserved locally at
`artifacts/refactor-20260921/pi/final-f5ff571/bench-check.json`.

## Native-build access loss after hardware checks

The subsequent separate Python 3.14 libcamera build touched no hardware and
issued no FC messages. It exhausted the Pi's swap during its first C++ object;
SSH then became unresponsive. Targeted attempts to stop only the identified
candidate compiler could not be confirmed. Its final process state and resource
recovery remain unverified; obtain a new read-only disarmed proof before any
further hardware checks. The last completed FC proof is the successful checker
probe above. See the [candidate report](python-314-candidate.md) for exact inputs,
created directories, failed attempts and bounded recovery/build preparation.
