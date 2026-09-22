# Remaining hardware qualification

Updated 2026-09-22. **The current evidence does not clear autonomous flight in
the hall.** Takeoff, altitude hold and LAND execute in simulation, but five
30-second vertical holds still fail horizontal clearance. Even the passing
10-second hold cases move about 0.73 m over the full armed sequence.

Physical flight and actuator tests are explicitly deferred. No live arming,
takeoff, LAND, mode, motor, throttle, RC override, mission-start, servo or payload
command is authorized in this task. Completing passive checks is a prerequisite
for later supervised qualification, not permission to begin it.

## Completed preparation

- [x] Recover SSH, verify the old compiler is absent, and obtain fresh read-only
  selected-FC disarmed proof and clear ownership before resumed hardware checks.
  Recheck these immediately before every subsequent hardware session.
- [x] Preserve this aircraft's Pi source/environment and system/service/network
  configuration, FC identity, 1,187 parameters, one mission item, empty fence and
  rally lists. Private copies are held in the separate restoration directory
  and the host's private archive. A full FC flash image was not read back.
- [x] Retrieve all 35,238 bytes of installed ROMFS again with successful session
  completion; SHA-256 matches the reviewed board definition. The earlier client
  cleanup failure remains retained. This does not identify every linked byte.
- [x] Preserve working Python 3.13 and services while preparing a separate uv
  Python 3.14 candidate. Matching-source full native smoke, synthetic ID17/project
  adapter, actual ARM installation and exact-sync preservation of all 30 wheels
  passed. A first real passive capture succeeded, but its shutdown supervisor
  failed. Corrected shutdown then passed; that recording instead failed with a
  real subscriber overflow/stale telemetry. Whole-check qualification stays open.
- [x] Implement explicit offline native deployment and paired rollback retention
  in `3a64313`. Its prepared 30-wheel payload and host replay pass; missing
  positive native/camera/disarmed evidence still prevents production selection.
- [x] Validate existing host checks and isolated pinned SITL. The latest full
  matrix at `9b4336d` is **35 passed, five failed, zero skipped**; all 40 required
  cases ran. At later recording cleanup `f520dd0`, both Python host suites passed
  all 1,868 cases with zero skips; its two affected isolated SITL cases passed.

## Before scheduling a supervised takeoff–LAND trial

- [ ] Review the prepared source/environment deployment and paired rollback.
  Actual service installation, restart, failover and rollback remain unexecuted.
  After separately authorized deployment, confirm normal read-only runtime status
  and exclusive hardware ownership on the Pi. Do not combine a flight trial
  with an unqualified interpreter or service migration.
- [ ] Retain Python 3.13 selection until native 3.14 tag API, real acquisition,
  recording, GPIO backend without actuation, resources and reproducible native
  payload/rollback gates pass. Imports alone are insufficient.
- [ ] Verify installed linked firmware against the reviewed 4.7.1 artifact and
  complete ROMFS. A reboot-separated readback and full flash-byte identity are
  still distinct outstanding evidence. No FC reboot, flash or parameter writes
  occurred in this task.
- [ ] Repeat unchanged fresh battery/health checks. The earlier checker failed at
  14.17 V against 14.4 V; a later source checker passed at 15.732 V. Historical
  voltage or a successful read-only heartbeat is not current flight clearance.
- [ ] Establish the intended FC power path and stable power/UART continuity.
  The owner reports external Pi power and no separate FC supply, expecting the
  Pi to power the FC. Resumed read-only probes alternated between fresh selected
  FC heartbeats and none. Pi access or occasional heartbeats do not verify a
  suitable power path, absence of unintended backfeeding or adequate current
  capacity. The [documented UART wiring](../../docs/DRONE_CONFIGURATION.md#hardware-topology)
  lists TX, RX and ground, without a power conductor. Resolve and document the
  actual physical arrangement before flight.
- [ ] Survey the actual takeoff area and intended motion volume as below. Resolve
  compass mounting/calibration/interference and usable flow/range observations
  without weakening normal checks.
- [ ] Confirm mechanical readiness, independent emergency arrangement and the
  reviewed zero-channel RC topology. Adding a receiver changes that contract.
- [ ] Have the owner schedule and explicitly authorize the supervised flight.
  Start with bounded takeoff–LAND; altitude hold and Loiter follow only after
  earlier stages have measured acceptable motion and cleanup.

## Disarmed sensor and hall survey

- [ ] Compare a cleaner area with marked hall positions/headings, saving raw
  magnetic vectors, attitude, estimator diagnostics, current and events. Use an
  independent heading reference; healthy EKF flags cannot validate their own yaw.
  Verify compass orientation, mounting, wiring and calibration. Motor-current
  interference needs later separately authorized actuator/flight evidence.
- [ ] With an authorized physical handling procedure, measure downward-range
  offset, tilt/floor error, validity limits and discontinuities against an
  independent height reference. Compare barometric and aligned local height;
  establish a measured disagreement/noise/duration policy. The 0.05 m ceiling
  reserve is software policy, not measured aircraft overshoot.
- [ ] Verify optical-flow direction/scale, range association and relative aiding
  over the actual floor, lighting, heights and hand motion. A fixed grounded
  recording cannot establish airborne scale or hall coverage.
- [ ] Record visible known tags using calibrated intrinsics at the actual crop/
  resolution and measured body/camera extrinsics. Synthetic recognition and
  camera imports do not establish physical pose accuracy, latency, ambiguity
  rejection or navigation suitability.
- [ ] Measure sustained runtime CPU/RSS/temperature, disk stalls/reserve behavior,
  serial throughput/backlog and control timing. The earlier passive Pi capture
  measured 77,991 RX / 1,075 TX bytes over 19.373 s; it sent only telemetry
  requests and cannot establish flight-load or thermal-soak margins.
- [ ] Qualify physical UART congestion/reconnect behavior for the 50 ms maximum
  write timeout, partial-write errors, continued supervision and cleanup.
  Host transport checks are separate from this physical timing measurement.

## Separately authorized supervised flight progression

- [ ] Takeoff–LAND: confirm liftoff, climb progress, true floor-referenced peak,
  overshoot, descent/touchdown and fresh disarm. A mode ACK or sent command is
  insufficient. Retain the normal 0.5 m gain, 0.6 m CLI cap and strict 0.8 m ceiling.
- [ ] Verify the post-takeoff lower-height guard with measured range/local noise
  and ordinary dynamics. Target minus 0.10 m is an engineering abort bound;
  simulated thrust-loss detection proves no physical recovery or impact safety.
- [ ] Qualify short altitude hold only after resolving whole-sequence clearance.
  At `9b4336d`, 30-second holds moved 1.378–1.458 m during hold and
  1.600–1.697 m over the armed sequence, failing the unchanged 0.5 m criterion.
  Ten-second holds moved 0.472–0.493 m during hold but 0.710–0.731 m overall.
  The current 10-second cap is provisional; no physical confined-hall envelope
  is qualified. Vertical hold deliberately provides no horizontal position control.
- [ ] Qualify Loiter progressively at the named duration, floor/lighting and yaw
  disturbance envelope. All eight current simulated Loiter profiles pass, but
  they are not measurements of the hall. The magnetic-reset regression now
  aborts the mission and lands/disarms; it does not make a disturbed compass
  reliable or detect all healthy-looking yaw drift. The inertial-yaw profile
  remains isolated-SITL-only; no aircraft parameter change is authorized here.
- [ ] Measure hold altitude error ≤0.10 m after 2 s settling, Loiter XY movement
  ≤0.5 m, heading motion and offset-corrected yaw-estimate drift ≤10° against
  independent references. Evaluate the entire armed sequence and unsuccessful
  runs. Any different operating envelope needs an explicit measured review;
  do not relax checks merely to obtain a passing result.
- [ ] Qualify loss of range/flow/aiding, operator loss, actual FC-side failsafe
  behavior and LAND without horizontal aiding under the supervised procedure.
  LAND after navigation failure does not promise precision position hold.
- [ ] Qualify payload/servo mechanics, travel, detach, interruption and power
  effects separately. Command accounting proves no physical completed movement.

Do not automatically switch navigation sources or fall back to the disturbed
compass. Camera navigation remains conditional: it requires fixed reference
geometry, calibration, a reviewed FC feature build, measured pose validity and
an explicit message/transform/reset/uncertainty contract. Current camera-relative
AprilTag detections are not a world navigation solution.
