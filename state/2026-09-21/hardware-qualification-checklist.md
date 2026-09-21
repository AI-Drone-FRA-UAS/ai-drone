# Remaining hardware qualification

**Physical flight and actuator tests are explicitly deferred.** No live arming,
takeoff, LAND, mode, motor, throttle, RC override, mission-start, servo or payload
command was authorized in this implementation task. An isolated simulator result
never grants that authorization.

## Before any deployment or flight qualification

- [ ] Preserve a fresh complete restoration capture for this aircraft: Pi source,
  selected interpreter/native environment, system/service/network configuration,
  FC image/identity, full parameters, mission, fence and rally state. The current
  1,187-parameter export is complete as parameters, not as a full aircraft backup.
- [ ] Review the prepared source/environment deployment and paired rollback
  transaction. Live service installation, restart, failover and rollback were not
  executed. Confirm the new shared runtime's normal read-only status and hardware
  exclusion on the actual Pi after a separately authorized deployment.
- [ ] Resolve the unchanged battery minimum: the live checker measured 14.17 V
  against 14.4 V, and later passive probes were lower. No flight-readiness claim
  follows from the other healthy bits.
- [ ] Verify the installed linked firmware features against the reviewed 4.7.1
  artifact and ROMFS. Version/hash strings alone are insufficient. The current
  host SITL binary is a separate target. Reboot-separated readback remains a
  distinct gate; no FC reboot/flash/parameter writes were performed in this task.
- [ ] Keep the working Pi Python 3.13 environment selected until the separate
  3.14 native ABI and acquisition gates pass. See the candidate report for the
  five missing compatible native extension modules and development inputs.

## Disarmed sensor and hall survey

- [ ] Survey a cleaner area and marked hall positions/headings, retaining raw
  magnetic vectors, attitude, reported estimator diagnostics/current and events.
  Use an independent heading reference; neither EKF health nor stable estimated
  yaw establishes correct yaw. Do not infer accelerometer/gyro/magnetic units
  for RAW_IMU beyond the declared wire semantics.
- [ ] Verify mount orientation, wiring, interference sources and magnetometer
  calibration. Correlation with powered motors requires separately authorized
  actuator/flight work; it was not simulated by energizing the real motors.
- [ ] With an authorized physical handling procedure, characterize downward
  range offset, discontinuities, min/max validity and tilted-floor error;
  compare barometric/aligned-local height to independent measurements. The
  software's 0.05 m reserve is conservative policy, not measured overshoot.
- [ ] Verify optical-flow direction/scale, range association, relative aiding and
  texture/lighting limits under hand motion. The grounded fixed-position capture
  does not demonstrate airborne optical-flow scale or hall coverage.
- [ ] Repeat camera/tag processing with visible known tags, calibrated intrinsics
  at the actual crop/resolution and measured camera/body extrinsics. Both current
  captures found zero tags. Pose accuracy, ambiguity rejection, calibrated
  capture latency and camera-navigation suitability remain unverified.
- [ ] Measure sustained CPU/RSS/temperature, storage stalls/reserve behavior,
  per-direction serial throughput, delivery backlog and control timing on the
  deployed runtime. Short passive recordings do not establish flight-load or
  thermal-soak margins. GPIO backend compatibility must avoid actuation.
- [ ] Qualify the checked serial-write policy under UART congestion and reconnect:
  50 ms maximum write timeout, preserved tighter settings, partial-write errors,
  continued supervision and cleanup. Host exact-encoder/port tests establish
  error accounting; they do not measure this Pi's physical write timing.

## Separately authorized supervised flight progression

- [ ] Verify the independent emergency arrangement, mechanical readiness,
  propeller clearance and zero-channel RC topology. Adding a receiver changes
  the reviewed contract and needs its own analysis.
- [ ] Start with bounded takeoff–LAND using normal checks and measured floor
  reference. Confirm liftoff, climb progress, actual peak height, overshoot,
  descent/touchdown and fresh disarm; a sent command or mode ACK is insufficient.
- [ ] Qualify autonomous vertical hold with adequate whole-sequence horizontal
  clearance. The 30-second simulator vertical hold maintained altitude but
  exceeded the frozen 0.5 m clearance criterion. Ten seconds is only a provisional
  software bound; even its full armed sequence moved about 0.72 m in one test.
  No physical confined-hall vertical-hold clearance has been demonstrated.
- [ ] Qualify Loiter progressively at the named duration, floor/lighting and yaw
  disturbance envelope. The compass baseline does not resolve hall magnetic
  interference: one changing-field simulator trial completed with 117.968°
  yaw-estimate drift despite only 0.921° of true heading motion. The inertial-yaw
  profile remains simulator-only; do not copy its
  parameter delta onto the aircraft without separate review and qualification.
- [ ] Measure hold altitude error ≤0.10 m after 2 s settling, Loiter XY movement
  ≤0.5 m and heading motion/offset-corrected yaw-estimate drift ≤10° against
  independent references. The ceiling remains strictly below 0.8 m. Evaluate
  the full armed sequence and retain unsuccessful runs too.
- [ ] Exercise the applicable supervised fault progression, especially loss of
  range/flow/aiding, operator loss, actual FC-side failsafe behavior and LAND
  without position aiding. LAND after navigation failure does not promise
  precision horizontal position hold.
- [ ] Qualify payload/servo behavior separately, including mechanical travel,
  detach, interruption and power effects. Mock command accounting proves no
  physical position or completed movement; no such actuator test was authorized.

Do not automatically switch navigation sources or fall back to the disturbed
compass. A future camera profile also needs a reviewed board build/feature fit,
message/transformation/reset/uncertainty contract and measured pose validity;
current camera-relative detections are not a world navigation solution.
