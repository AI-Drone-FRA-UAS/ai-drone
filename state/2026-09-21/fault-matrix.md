# Refactor fault-matrix audit — 2026-09-21

This maps every row of `refactor.md` §8.1 to implementation tests and the evidence
still required. Local mocks, isolated simulator evidence and physical measurements
are separate. Final immutable `d86a955` host suites passed 1,868 tests each on
Python 3.14.7 and 3.13.15, with zero skips. The coordinator's final isolated
40-case simulator matrix completed **34 passed, six failed, zero skipped**;
the [implementation report](refactor-implementation-report.md) retains exact
revision, artifact paths and outcomes. Named tests below are evidence only to
the extent of those actual executions, not physical qualification.
All required cases ran, but the final SITL acceptance gate is **not green**;
the six qualification failures remain open.

The physical aircraft must remain grounded and disarmed. This audit opened no Pi
connection and ran no simulator. Every new command endpoint in
`tests/test_refactor_fault_matrix.py` is a `MagicMock`; opening a real transport
explicitly fails the fixture, and the mocked device has no network address.

## Executed local checks

- The independent integration batch passed **667 tests, no skips**, in 12.06 s.
  It covered controller, control ownership, state/phase/limits, altitude hold,
  profiles, shared MAVLink, vehicle server, runtime, flight recording, passive
  recording, tag recording, storage, signal scopes, operator and the first five
  matrix cases. This was a working integration-tree run during parallel work,
  not an immutable firmware or final integration qualification gate.
- Three subsequently added public-mission reentry regressions reproduced a defect:
  `takeoff`, `enter_loiter` and `hold_loiter` could resume a pending LAND mission.
  The flight owner fixed the command and public boundaries in `f3183f6`.
  The complete matrix test file then passed **8 tests, no skips**; its type check
  also passed. These regressions supplement existing writer/phase tests.
- Earlier owned checks passed 209 recording/tag tests, 55 review tests and
  52 checker tests. Scoped Ruff/type checks passed. A prior host-only vision run
  skipped the OpenCV synthetic-pose test because OpenCV was absent. After adding
  the complete dependency groups, the final recording/tag/AprilTag batch passed
  **250 tests, no skips**, in 4.08 s under uv-managed Python 3.14.7. This includes
  the synthetic-pose test and direct transport/delivery measurements added in
  `6df5903`; scoped Ruff/type checks also passed. These are host tests, not native
  Pi-library compatibility or physical sensor measurements.
- The pause checkpoint records five pinned SITL cases passing at `9de5fa1` in an
  immutable checkout. That is evidence for that revision only. No simulator
  result is inferred from the passing host batch above.
- The coordinator's expanded run in `/tmp/refactor-sensor-faults.log` completed
  with **7 passed and 3 failed**. This audit independently read that log and the
  retained summaries; it did not run or control the simulator. Six injected
  Loiter faults passed: flow loss, zero flow quality, range loss, wrong range
  orientation, a 1.2 m range discontinuity and combined aiding loss. Every case
  confirmed actual disarm; the largest fault-to-LAND response was **1.452 s**.
  Their maximum observed heights were 0.520–0.530 m and whole armed-sequence XY
  displacements were 0.157–0.311 m. These are the tested synthetic faults only.
  The 0.1 m ground-offset case also passed. Exact final revision/firmware/run
  records and later reruns belong to the coordinator's implementation report.
- All three failures remain visible: changing the native range message ID did
  not produce the expected failure because this pinned FC associates that
  input by orientation and republishes its configured instance ID; `1388333`
  corrects the harness contract without dropping wrong FC-output-ID mock checks.
  The pre-arm magnetic experiment failed during parameter readback before the
  production CLI ran, so it established no pre-arm refusal result. At 0.3 m
  ground offset the controller did refuse before arming, but the test failed
  because it required the literal word `ceiling` while the diagnostic said
  `maximum altitude`. Passing refusal behavior does not turn that failed test
  into a passing gate; retain its corrected rerun separately.

## Full scenario mapping

"Remaining" distinguishes the completed final simulator evidence from still-open
physical qualification. Historical failed attempts above are retained; later
passes do not erase them. Physical flight remains deferred throughout.

| §8.1 scenario | Local evidence and implemented behavior | Remaining qualification |
| --- | --- | --- |
| Normal takeoff / standalone altitude hold / Loiter / LAND | `test_controller.py::test_hover_cli_runs_guided_nogps_takeoff_loiter_hold_and_land`; `test_altitude_hold.py` covers dedicated CLI, 20 Hz neutral climb, duration and all required-input/mode failures. Original production SITL uses the actual CLI. | Final `d86a955` profiles: all eight Loiter cases and three 10-second vertical cases passed; five 30-second vertical cases failed unchanged XY clearance. All disarmed. The new inertial profile has three independent starts per operation/condition at headings 0/120/240 degrees; compass is the retained baseline. Full measurements are in the linked 16-row table. Physical envelope remains unqualified. |
| Ground offset and ceiling | Controller target-plus-pad-plus-overshoot rejection, late pre-arm pad change, independent aligned-local and range ceiling tests; `test_flight_limits.py::test_inclusive_floor_ceiling_and_conservative_local_range_are_independent`. Forward range is excluded from the flight altitude datum. | Both actual 0.1 m/0.3 m pad-offset cases passed at `d86a955`, including refusal before unsafe climb. Varied physical offsets and near-boundary measurements remain required; changed starting heading is not evidence of changed height. |
| No liftoff, stalled climb, overshoot, unexpected descent | New matrix test parametrizes no liftoff, stalled climb and descent. The real takeoff loop ends at its injected 1 s deadline, sends LAND, retains a flight cleanup obligation and never direct-disarms. Existing ceiling tests cover overshoot response. | Final `d86a955` dynamics passed all seven cases, including actual zero-thrust no-liftoff, tether-stalled climb and thrust-loss descent. The original false success is retained; `ab11d47` now rejects fresh post-target height below target minus 0.10 m and unexpected disarm during holds. Initial target acquisition still uses bounded progress/deadline policies. Physical noise/dynamics and recovery remain unqualified; the motor-cut case still impacts. |
| Magnetic anomaly before arm | Baseline exact safety/firmware/parameter checks remain; experimental profile is restricted to explicitly isolated local SITL and exact candidate parameters. `test_sitl_faults.py::test_compass_anomaly_before_arm_refuses` supplies the simulator refusal case. | The final `d86a955` refusal passed with its concrete magnetic reason after `e568ff0`; all enabled compasses were disturbed. Physical magnetic survey/heading reference and normal pre-arm conditions remain unqualified. |
| Magnetic anomaly after takeoff / during Loiter | `test_flight_limits.py` enforces failed relative-navigation health in Loiter; flight telemetry and FC failsafe policies remain. Inertial profile tests inject a changing global field and gyro Z bias after liftoff. | The final compass changing-field case completed LAND/disarm but FAILED truth qualification with 117.937804 degrees of estimated-yaw drift versus 1.761604 degrees of true motion (`d86a955`). All enabled-compass injections/readbacks are retained. All three disturbed inertial Loiter repetitions passed their tested bounds; hardware is not qualified. Healthy-looking EKF flags alone are insufficient. |
| Heading offsets, gyro bias, warm-up, resets, drift | Inertial profile has an explicit whole-sequence duration budget including landing reserve. Review reports keep raw/scaled magnetics distinct, show actual estimator fields and timestamped FC reset messages, and do not infer heading truth. | Three independently initialized normal and magnetic inertial-profile starts per operation were executed at headings 0/120/240 degrees with retained truth/bias records. Physical stationary/temperature/startup yaw reference and usable maximum duration remain unqualified. FC status-text reset messages do not prove reset timing or magnitude. |
| Flow/range loss, quality, wrong ID/orientation, replay/delay | State tests reject invalid/repeated/out-of-order evidence without refreshing receipt age; only downward FC-output instance 0 supplies flight altitude. Check/report/capture tests retain identity and data ages. New matrix tests prove recovered inputs cannot resume altitude hold or other pending-LAND missions. Coordinator simulator loss, zero-quality, wrong orientation and all-aiding-loss cases passed through actual disarm. | Final `d86a955` actual losses/discontinuity all reached LAND/disarm, maximum response 1.465602 s. Native input-ID changes map to the configured FC-output instance and are a passing contract observation, not a wrong-output-ID fault; that fault remains locally mocked. Physical dropout/recovery and flight-load delivered rates remain unqualified. No failed mission automatically resumes. |
| Barometer/local divergence and range discontinuity | Separate datums/alignment are retained; both range and aligned-local ceilings guard flight. Invalid new range evidence revokes earlier validity. The final `d86a955` 1.2 m discontinuity injection passed with a 0.100608 s LAND response and actual disarm. | A measured under-ceiling disagreement policy is still required: independent references, noise floor, duration, allowable range/local disagreement and failure threshold. No threshold or vertical-source switching is invented from source inspection. |
| Missing relative aiding / rejected Loiter entry | New matrix health-flicker case sends only neutral climb while waiting, resets the continuous-health interval and requests LAND at the bound. Existing entry tests require confirmed mode and zero receiver channels. | Both actual missing-aiding and blocked-Loiter-entry cases passed through LAND/disarm again in the final `d86a955` dynamics group. Physical startup/aiding and touchdown remain unqualified. |
| LAND without horizontal aiding | Controller retry/timeout tests never force-disarm an attempted flight; phase retains LAND obligation. The final `d86a955` combined-aiding-loss case passed with a 1.027181 s LAND response, 0.319099 m whole armed-sequence XY displacement and actual disarm. | The separately analyzed `0a640b2` tlog/DataFlash and pinned source support position-unavailable LAND selection (see below); do not relabel that internal-branch analysis as a new `d86a955` measurement. Precision physical landing cannot be promised without navigation. |
| Dropped LAND/disarm, failed RX/write, subscriber overflow | `test_controller.py` retry/fresh-disarm tests; `test_shared_mavlink.py` overflow, failed-reader and serialized-send tests. New phase tests preserve attempted/failed command records. Cached or queued disarm observations cannot confirm a new cleanup request. | Final host regressions and pinned integration gates completed; physical transport-loss, congestion and reconnect remain unqualified. Actual byte/gap counters were measured in final host full-load and separate passive Pi captures, with distinct scopes. |
| FC-link loss / hard companion termination | Existing `test_sitl.py::test_gcs_heartbeat_loss_in_loiter_lands_and_disarms` **SIGKILLs the control process group**, bypasses Python cleanup, and requires actual LAND, disarm and FC GCS-failsafe text. It is a hard-companion case, not only a mocked heartbeat callback. | The final `d86a955` actual GCS-loss case passed LAND/disarm. Whole Pi/serial electrical loss on hardware remains deferred. Operator-only loss and companion loss are distinct cases. |
| Human takeover before/during cleanup | `test_control_ownership.py` covers queued takeover before next LAND, stale/foreign evidence, callback failure, all command boundaries and latch persistence. `test_vehicle_server.py` preserves ownership/exclusion and prevents an armed replacement claim. | The final pinned pilot-handoff case passed with actual manual-mode evidence and post-operator-loss supervision. Positive handoff acknowledgement is required. Physical handoff is deferred. |
| Operator loss versus SIGHUP | Ownership/operator/signal tests distinguish authenticated loss, stale observations and nested handlers. Shared recording SITL checks hangup survival followed by operator-loss LAND; confirmed human ownership is retained. | The final pinned subprocess/hangup and operator-loss gate passed. No physical operator-loss experiment is authorized. |
| Unexpected mode / RC topology | Controller tests reject a newly active receiver even with low throttle, detect changed modes and preserve explicit human handoff. Altitude-hold tests prohibit resuming Guided after mode failure. | Both actual unrequested-mode and low-throttle receiver-arrival simulator cases passed through LAND/disarm again at `d86a955`; altitude-hold mode-failure paths remain locally mocked. Physical receiver/mode qualification is deferred. |
| Full video/tag recording, slow disk, low storage | Flight recorder tests prove event/tlog/storage failures cannot block control or final cleanup, bound worker close, and preserve incomplete manifests. Capture tests cover reserve enforcement, encoder/video stop, ongoing analysis/logs and camera timeout. | Final `d86a955` host full-load case passed with real OpenCV/H264, storage pressure, control gaps and exact TCP bytes. The separate passive Pi capture on staged `f5ff571` measured 77,991 RX / 1,075 TX serial bytes over 19.372824 s, with zero transport errors, but sent no control heartbeat/setpoint. Sustained native Pi flight-load timing/thermal/disk remains unqualified. Maintain 11,520 bytes/s nominal capacity **per direction** at 115200 baud/8N1. |
| Camera navigation profile faults, if selected | Existing tag/pose tests reject stale/malformed geometry, ambiguity and invalid calibration; tag actuation decisions are unrelated to navigation. No camera navigation source is selected or implemented as an estimator aid. | Conditional camera-navigation development/qualification is inactive unless inertial feasibility fails and a reviewed camera interface is selected. Existing AprilTag detection or a camera import is not odometry qualification. |
| Interrupted initialization/shutdown/runtime restart | Independent cleanup/manifest tests, immutable snapshot tests, worker/GPIO ownership retries, signal restoration, runtime status and server timeout-then-restart tests. `0988628` keeps failed-close startup resources reachable for a second teardown attempt. | Pi service/deployment restart and rollback evidence remain separate from host mocks; physical actuation is deferred. Full storage failure may prevent even a minimal manifest and must be reported honestly. |

## Review findings and closure

1. **Partial camera startup dropped teardown owners.** Regression added before
   changing ownership: a first camera close and servo-session close can both
   fail after a worker exists. `0988628` preserves the failed resources and
   worker for final retry/join; all later cleanup is attempted.
2. **Public operations could erase pending LAND.** The three new matrix cases
   failed before `f3183f6`, then passed after it. Recovered measurements cannot
   overwrite a failed mission's LAND state; command-boundary tests also reject
   arm/climb/non-LAND mode writes during pending cleanup. A new mission after
   confirmed cleanup still requires its ordinary explicit pre-arm sequence.
3. **Checker used unsupported RAW_IMU unit labels.** `0c6beb6` reports raw
   accelerometer/gyro/magnetometer units, consistent with historical review
   exports. This is not a sensor calibration measurement.
4. **Under-ceiling altitude disagreement is not qualified.** Current code retains
   both datums and enforces independent ceilings. Choosing an additional
   disagreement threshold requires measured noise/dynamics and independent
   height evidence; that missing policy is recorded rather than silently made up.
5. **Message counts did not establish serial bandwidth.** `1b89818` measures
   directly owned transport read/write return lengths, retaining unknown totals
   when the transport cannot establish an exact count. `6df5903` attaches that
   meter before recording startup requests and finishes it after connection
   teardown. The manifest separates actual per-direction bytes from encoded or
   decoded packet bytes, records receipt gaps/ages and delivery lag, and labels
   sequence gaps as estimates rather than proven packet loss. Mock tests prove
   interval boundaries and counter semantics; only the coordinator's actual Pi
   and simulator measurements can establish achieved rates under their loads.

All remaining physical arming, takeoff, altitude hold, Loiter, LAND, motor, servo,
payload and flight-qualification steps are explicitly deferred. No assertion in
this document authorizes them.

## Later integrated evidence

The corrected F01–F19 register is fully mapped in
[its reconciliation](findings-reconciliation.md). The expanded fault coverage
subsequently found and repaired three additional concrete problems: physical
MAVLink adapters could conceal partial/failed writes (`ed73b7e`); idle TCP reads
made exact RX totals unknown (`5194303`); and a real simulated motor cut could
return successful Loiter after impact (`ab11d47`). Original failed runs remain
archived, and their unchanged reruns are recorded in the implementation report.

The generated-camera workload uses actual OpenCV tag detection, CPU H264 encode
and ffprobe validation alongside the real runtime and controller, with simulated
write delays and storage reserve pressure. It is host evidence, not a physical
camera/native Debian AprilTag or Pi UART timing claim. GPIO/payload construction
is explicitly forbidden. Zero observed queue errors or sequence-gap estimates
do not prove zero physical packet loss.

The earlier `0a640b2` all-aiding-loss tlog and both DataFlash EKF cores provide
source-correlated LAND-without-position evidence: relative/absolute position
was lost before LAND, with no dead-reckoning status throughout landing. This
supports the pinned `nogps_run` branch inference; it is not direct logging of
the internal mode flag or a precision-landing promise. Exact timing and lateral
displacement appear in the implementation report.

The earlier `f5ff571` checkpoint remains separate from both the final `d86a955`
simulator rerun below and those earlier investigations:

- Dual host suites: 1,852 passed each, zero skips; required formatting, lint,
  typing, import contracts, dependency, installed-package and site gates passed.
- That checkpoint's 40-case simulator matrix: baseline 5 passed; vertical profiles 3 passed /
  5 failed; Loiter profiles 8 passed; fault cases 10 passed / 1 failed; dynamics
  7 passed; full recording workload 1 passed. No skips. The
  [earlier 16-row profile table](../../artifacts/refactor-20260921/sitl/refactor-f5ff571-profile-table.json)
  retains all outcomes and metadata. All 30-second vertical failures are XY
  clearance (1.377616–1.459474 m); the compass fault fails yaw-estimate drift
  (117.937366° versus 1.313253° true heading movement). Every profile disarmed.
- That checkpoint's host workload retained real tag recognition, valid H264, continued
  tag/telemetry processing after simulated reserve pressure and actual control
  in parallel. It measured 245,531 RX / 14,468 TX TCP bytes over 33.976 s,
  maximum heartbeat/setpoint gaps 1.0391 / 0.05502 s and delivery lag 0.01129 s,
  with no transport or subscriber/log overflow errors. These are host results.
- After a temporary SSH outage, the third passive Pi capture on separately
  staged `f5ff571` source passed: 73 analyzed frames, 240 encoded H264 frames,
  1,254 messages, zero tags and zero commanded servo pulses over 10.056188 s.
  Fresh disarmed proofs bracketed the run; the outbound allowlist contained
  only 25 telemetry-interval requests. Actual serial totals were 77,991 RX /
  1,075 TX bytes over 19.372824 s including startup, with zero transport/parser
  errors and 0.000192083 s maximum delivery lag. Received FC-heartbeat gap was
  1.368586 s; no outgoing control cadence was exercised. Resource-wrapper CPU
  includes the initial ownership probe, and the short run is not thermal-soak
  or combined native control-load qualification. Known-tag recognition remains
  unverified on the Pi because all three captures detected no tags.
- One complete 35,238-byte installed ROMFS hardware-definition file now matches
  the reviewed hash. The client's failed terminate-session exit is retained;
  independent retrieval/hash validation establishes the file comparison, not
  full flash identity or the still-deferred reboot-separated repeat.

The final immutable `d86a955` rerun includes the deployment-preservation and
loopback-interface safety guards. Its evidence is:

- Dual host suites: **1,868 passed each**, zero skips, with all 40 simulator
  cases deselected from each host run and executed separately below.
- Required simulator matrix: **34 passed, six failed, zero skipped**. Baseline
  5 passed (272.761 s); dynamics 7 passed (371.921 s); faults 10 passed / 1 failed
  (581.247 s); vertical profiles 3 passed / 5 failed (554.452 s); Loiter profiles
  8 passed (562.863 s); full recording workload 1 passed (61.659 s). These are
  group durations from the final JUnit files, not a sum of concurrent wall time.
  The final acceptance gate is **not green**.
- The [final 16-row profile table](../../artifacts/refactor-20260921/sitl/refactor-d86a955-profile-table.json)
  retains the exact measurements and observation paths. All eight Loiter cases
  passed, with maximum hold height error 0.010051 m, peak height 0.529968 m,
  sampled hold XY displacement zero at the truth-message resolution, and
  maximum whole armed-sequence XY displacement 0.176332 m. Maximum estimated-yaw
  drift was 5.652448 degrees during hold and 8.046356 degrees over the armed
  sequence; actual heading movement was at most 5.665594 and 6.982038 degrees,
  respectively. These results establish the tested short synthetic envelope,
  not stationary yaw or a measured physical duration limit.
- All three 10-second vertical cases passed, with hold XY displacement
  0.478869–0.492722 m and whole armed-sequence displacement 0.724010–0.731186 m.
  All five 30-second vertical cases failed solely on unchanged XY clearance:
  1.394228–1.458281 m during hold and 1.599904–1.688685 m over the armed sequence.
  Every profile ultimately disarmed. The compass changing-field case also
  landed/disarmed but failed its yaw qualification: 117.937804 degrees of
  estimated drift versus 1.761604 degrees of actual heading motion.
- The six injected sensor-loss/discontinuity cases all reached LAND and actual
  disarm. Maximum fault-to-LAND response was 1.465602 s; the range discontinuity
  response was 0.100608 s. Combined aiding loss responded in 1.027181 s with
  0.319099 m whole armed-sequence XY displacement. The native range-input-ID
  contract case is excluded from those fault-response maxima because it does
  not remove the FC's configured range-output instance.
- The final synthetic-camera workload recognized the known AprilTag in all
  392 analyzed frames and validated 208 encoded H264 frames. Tags and telemetry
  continued after the simulated low-storage video stop. Actual TCP totals were
  252,742 RX / 14,366 TX bytes over 34.981273 s, with maximum outgoing heartbeat
  gap 1.976630 s, setpoint gap 0.054708 s and delivery lag 0.010610 s. Transport,
  parser, receive, subscriber and log error counters were zero; queue peak was
  33. Recorder CPU was 12.344052 s over 33.827732 s wall time, maximum RSS
  104,800 KiB; the encoder separately used 2.170722 s CPU and 91,524 KiB maximum
  RSS. This is a host TCP/control workload with a generated camera, not a Pi
  serial or native camera-load measurement. No GPIO/payload object was created.

The separately prepared Python 3.14 Pi candidate remains blocked. Matching
development packages were extracted inside its isolated directory, and the
ARM64 libcamera build configured against the exact installed library, but its
first C++ object exhausted the Pi's 414 MiB swap. SSH then became unavailable;
the attempted targeted compiler stop and current Pi state are unconfirmed.
Resource-bounded libcamera/pyKMS build kits and host-tested small bindings are
prepared, but do not establish ARM64 native compatibility. See the
[native-candidate report](python-314-candidate.md). The candidate has not been
selected for services, and deployment refuses to replace it until a reviewed
native-artifact preservation/installation contract exists. No new Pi measurement
or fresh disarmed proof is claimed after the resource/network failure.
