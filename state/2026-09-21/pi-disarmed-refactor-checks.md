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
FC state can now be established, so all further hardware checks are deferred.
Creation/completion of its proposed `romfs-download-complete.partial` and
`romfs-hwdef.dat` outputs is unverified; neither may be treated as a valid readback.

`source-bb49adc/` was successfully staged under the same refactor candidate
directory before connectivity failed. It contains the final passive transport
byte meter, but **no hardware capture using it was run**. Exact direct RX/TX
measurement is implemented and mock-tested; earlier captures cannot establish
actual TX totals retroactively. The metrics capture, final power snapshot and
complete ROMFS readback remain outstanding until fresh disarmed access returns.

A final bounded SSH-only `date -u; uptime` request also timed out connecting to
port 22 (`pi-final-connectivity-20260921.log`). It did not open the FC. No later
successful access or fresh disarmed proof is claimed.
