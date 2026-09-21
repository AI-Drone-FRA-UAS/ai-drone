# Refactor fault-matrix audit — 2026-09-21

This maps every row of `refactor.md` §8.1 to implementation tests and the evidence
still required. Local mocks, isolated simulator evidence and physical measurements
are separate. A named simulator test is not a passing result; the coordinator's
final run report supplies exact revision, artifact paths and outcomes.

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

## Full scenario mapping

“Remaining” describes the physical or simulator evidence not established by this
independent local audit. Subsequent coordinator simulator results may close the
specified simulator portion; physical flight remains deferred throughout.

| §8.1 scenario | Local evidence and implemented behavior | Remaining qualification |
| --- | --- | --- |
| Normal takeoff / standalone altitude hold / Loiter / LAND | `test_controller.py::test_hover_cli_runs_guided_nogps_takeoff_loiter_hold_and_land`; `test_altitude_hold.py` covers dedicated CLI, 20 Hz neutral climb, duration and all required-input/mode failures. Original production SITL uses the actual CLI. | `test_sitl_profiles.py` supplies compass normal and inertial normal/disturbed truth gates. Record each execution, whole armed-sequence height/XY/yaw and actual disarm, starting heading, duration and independent restart. Compass profile needs at least three independent normal and magnetic starts; two parametrized normal headings alone do not meet that repetition gate. Physical envelope has no qualification. |
| Ground offset and ceiling | Controller target-plus-pad-plus-overshoot rejection, late pre-arm pad change, independent aligned-local and range ceiling tests; `test_flight_limits.py::test_inclusive_floor_ceiling_and_conservative_local_range_are_independent`. Forward range is excluded from the flight altitude datum. | Varied physical pad offsets and near-boundary truth measurements; simulator starts must document any tested ground offsets rather than assume a changed heading also varies height. |
| No liftoff, stalled climb, overshoot, unexpected descent | New matrix test parametrizes no liftoff, stalled climb and descent. The real takeoff loop ends at its injected 1 s deadline, sends LAND, retains a flight cleanup obligation and never direct-disarms. Existing ceiling tests cover overshoot response. | Actual dynamics and full configured takeoff timeout in simulator; actual touchdown and drift after each failed trajectory. No additional under-ceiling stall/descent threshold is claimed beyond bounded target waiting. |
| Magnetic anomaly before arm | Baseline exact safety/firmware/parameter checks remain; experimental profile is restricted to explicitly isolated local SITL and exact candidate parameters. `test_sitl_faults.py::test_compass_anomaly_before_arm_refuses` supplies the simulator refusal case. | Execute and retain normal-check refusal reason with every enabled simulated compass disturbed. Physical magnetic survey/heading reference and normal pre-arm conditions remain unqualified. |
| Magnetic anomaly after takeoff / during Loiter | `test_flight_limits.py` enforces failed relative-navigation health in Loiter; flight telemetry and FC failsafe policies remain. Inertial profile tests inject a changing global field and gyro Z bias after liftoff. | Execute simulator tests with truth drift and all enabled compass readbacks. Compass-profile post-takeoff magnetic degradation needs its own bounded LAND/disarm evidence. Healthy-looking EKF flags alone are insufficient. |
| Heading offsets, gyro bias, warm-up, resets, drift | Inertial profile has an explicit whole-sequence duration budget including landing reserve. Review reports keep raw/scaled magnetics distinct, show actual estimator fields and timestamped FC reset messages, and do not infer heading truth. | Simulator repeated starts at documented headings/biases and truth-error traces; physical stationary/temperature/startup yaw reference and usable maximum duration. FC status-text reset messages do not prove reset timing or magnitude. |
| Flow/range loss, quality, wrong ID/orientation, replay/delay | State tests reject invalid/repeated/out-of-order evidence without refreshing receipt age; only downward instance 0 supplies flight altitude. Check/report/capture tests retain identity and data ages. New matrix tests prove recovered inputs cannot resume altitude hold or other pending-LAND missions. `test_sitl_faults.py::test_loiter_sensor_fault_lands` supplies sensor fault injections. | Execute simulator loss, zero-quality, wrong ID/orientation and all-aiding-loss cases through actual disarm. Characterize physical dropout/recovery, sensor quality and delivered rates. No failed mission automatically resumes after recovery. |
| Barometer/local divergence and range discontinuity | Separate datums/alignment are retained; both range and aligned-local ceilings guard flight. Invalid new range evidence revokes earlier validity. Simulator fault file includes a 1.2 m discontinuity. | Execute discontinuity case. A measured under-ceiling disagreement policy is still required: independent references, noise floor, duration, allowable range/local disagreement and failure threshold. No threshold or vertical-source switching is invented from source inspection. |
| Missing relative aiding / rejected Loiter entry | New matrix health-flicker case sends only neutral climb while waiting, resets the continuous-health interval and requests LAND at the bound. Existing entry tests require confirmed mode and zero receiver channels. | Simulator startup with unavailable aiding and explicit rejected Loiter entry must preserve the bounded wait and actual touchdown. A sent mode request is not success. |
| LAND without horizontal aiding | Controller retry/timeout tests never force-disarm an attempted flight; phase retains LAND obligation. Simulator all-aiding-loss case exercises position-unavailable behavior when actually executed. | Record pinned FC horizontal-control selection, actual lateral motion and touchdown. Physical precision landing cannot be promised without navigation. |
| Dropped LAND/disarm, failed RX/write, subscriber overflow | `test_controller.py` retry/fresh-disarm tests; `test_shared_mavlink.py` overflow, failed-reader and serialized-send tests. New phase tests preserve attempted/failed command records. Cached or queued disarm observations cannot confirm a new cleanup request. | Final pinned regression gate and actual transport-loss qualification. Byte/gap counters must be measurements, not message counts or wire-capacity estimates. |
| FC-link loss / hard companion termination | Existing `test_sitl.py::test_gcs_heartbeat_loss_in_loiter_lands_and_disarms` **SIGKILLs the control process group**, bypasses Python cleanup, and requires actual LAND, disarm and FC GCS-failsafe text. It is a hard-companion case, not only a mocked heartbeat callback. | Final exact-revision simulator execution; whole Pi/serial electrical loss on hardware is deferred. Loss of only the operator and loss of the companion are distinct cases. |
| Human takeover before/during cleanup | `test_control_ownership.py` covers queued takeover before next LAND, stale/foreign evidence, callback failure, all command boundaries and latch persistence. `test_vehicle_server.py` preserves ownership/exclusion and prevents an armed replacement claim. | Rerun pinned pilot-handoff case, including actual manual-mode evidence and post-loss supervision. Physical handoff is deferred. |
| Operator loss versus SIGHUP | Ownership/operator/signal tests distinguish authenticated loss, stale observations and nested handlers. Shared recording SITL checks hangup survival followed by operator-loss LAND; confirmed human ownership is retained. | Final pinned subprocess/hangup gate. No physical operator-loss experiment is authorized. |
| Unexpected mode / RC topology | Controller tests reject a newly active receiver even with low throttle, detect changed modes and preserve explicit human handoff. Altitude-hold tests prohibit resuming Guided after mode failure. | Simulator changing receiver/mode topology during each profile; physical receiver/mode qualification deferred. |
| Full video/tag recording, slow disk, low storage | Flight recorder tests prove event/tlog/storage failures cannot block control or final cleanup, bound worker close, and preserve incomplete manifests. Capture tests cover reserve enforcement, encoder/video stop, ongoing analysis/logs and camera timeout. | Actual per-direction serial bytes, delivered rates, backlog/sequence gaps, control heartbeat/setpoint gaps, CPU/RSS/thermal/disk under a simultaneous full workload. Pi passive measurements cannot establish armed timing or altitude performance. Maintain 11,520 bytes/s nominal capacity **per direction** at 115200 baud/8N1. |
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
