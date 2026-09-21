# Flight reliability and functional-core refactor plan

Revised 2026-09-21 against project commit `ab00860` and official ArduCopter
4.7.1 source `dbe792162d06cab66c3475fd5556bf7a120f119e`.

## 1. Objective and constraints

The first priority is repeatable autonomous **takeoff, altitude hold, Loiter,
and landing in the test hall**. Magnetic interference is a flight-development
requirement, not a cleanup item. Code reduction and functional Python should
make these behaviors easier to verify and maintain.

- Use installed sensors only: FC IMU/barometer, MTF-01P downward flow/range,
  HGLRC M100 compass, forward MT-15, and IMX500 camera on the Pi Zero 2 W.
  GPS remains unavailable for the indoor navigation design. No additional
  compass, tracking camera, UWB, motion-capture system, or companion computer
  is assumed. Existing AprilTags may support camera experiments; map geometry
  and visibility must be established rather than presumed.
- Preserve the selected recording, payload, networking, transfer, recovery,
  and reporting capabilities in [the questionnaire](docs/CLEANUP_QUESTIONNAIRE.md).
  Full functional-core refactoring remains the intended direction. The approved
  removals remain `scripts/disarmed_tag_mount.py` and USB adapter auto-detection.
- Retain the initial 0.5 m takeoff gain, CLI target cap of 0.6 m, and 0.8 m
  floor-referenced software ceiling. The configured 1.0 m downward-range
  operating limit is separate. Flights at 1–3 m need a later envelope review;
  they are not unlocked by this refactor.
- This document specifies future implementation and validation. Revising it
  does not flash firmware, write parameters, deploy services, or run hardware.
  Initial hardware validation remains disarmed; flight trials remain a separate
  supervised activity under the existing project procedure.

Work lives on `codex/refactor` in `ai-drone-codex`; Antigravity has
`agy/refactor` in `ai-drone-agy`. Integrate reviewed commits, never edit across
the agents' working directories. ArduPilot source is an external, pinned
dependency; any experimental firmware changes need a separate checkout.

## 2. Evidence and baseline

Keep software state, historical aircraft observations, simulator results, and
present flight readiness distinct. This revision checks the prior plan against
the relevant code, tests, dated evidence, pinned firmware source, and primary
ArduPilot documentation. It does not claim another full-file audit or new
live flight-controller or flight-performance validation. The separately scoped
[Python readiness check](state/2026-09-21/python-314-readiness.md) records
read-only live Pi interpreter and library observations.

| Evidence | What it establishes | Limit |
| --- | --- | --- |
| [Controller](ai_drone/flight/controller.py), [CLI](ai_drone/cli/control.py), [SITL tests](tests/test_sitl.py) | Existing production path is `GUIDED_NOGPS -> LOITER -> LAND`; continuous attitude/climb targets initiate takeoff | No dedicated autonomous altitude-hold operation is currently exposed |
| [September 9 installation](state/2026-09-09/firmware-and-power.md) | Recorded installation of 4.7.1; flow/range and camera acquisition worked; normal pre-arm check still reported magnetic-field error | Dated evidence, not the current installed state or airborne accuracy |
| [September 7 investigation](state/2026-09-07/indoor-repair.md) | GPS was disabled; source 1 used optical-flow velocity, barometer height, compass yaw; external compass is on I2C | Does not isolate hall interference from mounting, calibration, or aircraft-generated fields |
| [Reviewed firmware](firmware/README.md) and [overlay](firmware/FlywooF745-nogps-loiter-extra.hwdef) | Flow fusion and `GUIDED_NOGPS` enabled; `EK3_FEATURE_EXTERNAL_NAV=0`, `HAL_VISUALODOM_ENABLED=0`, `MODE_FLOWHOLD_ENABLED=0` | A firmware identity string alone cannot establish these features |
| Prior focused review | 244 existing controller/ownership/power/server/servo tests passed | No new magnetic-fault or altitude-hold acceptance cases were run |

The original audit's September 19 measurements remain historical planning data:
19,271 non-test source lines in 65 files; 19,533 test lines in 46 files;
1,423 passing tests, two skips and five deselected SITL cases; ten `ty`
diagnostics, nine around deployment status typing. Re-measure before implementation.

Its counts of `Any`, dictionaries, `isinstance`, exceptions, prints, class methods,
and long functions identify inspection targets, not quality quotas. Preserve
diagnostics and necessary failure handling. Do not require dataclasses for
every JSON object, logging for every CLI print, or `match` for every branch.

The 83 audit candidates are not 83 established bugs. Withdraw the extrapolation
to 55–65 defects and the claim that a refactor eliminates roughly 40 of them.
Track each finding by trigger, observed consequence, evidence, intended behavior,
regression test, and resolution. Line savings are measured outcomes: roughly
800 lines were previously proposed as mechanical deletion/deduplication candidates;
the larger 2,000-line and 12–15% estimates are unverified, overlapping forecasts.

## 3. Flight behavior contract

### 3.1 Separate the four capabilities

| Capability | Required behavior | Acceptance evidence |
| --- | --- | --- |
| Autonomous takeoff | Normal pre-arm checks, confirmed `GUIDED_NOGPS`, bounded continuous climb commands, then a confirmed target/settling condition | Actual airborne height, climb rate and overshoot; mode acknowledgement alone is insufficient |
| Autonomous altitude hold | Continue neutral climb-rate targets in `GUIDED_NOGPS` for a bounded duration, supervise vertical state, attitude, operator and command delivery, then LAND | Height error, settling, oscillation, heading drift and XY drift measured independently |
| Loiter | Enter only after the selected navigation profile has stable usable horizontal aiding and acceptable heading behavior; retain mode and hold the local position | FC mode plus independent displacement/heading measurements, estimator health, and sustained altitude |
| Landing | Maintain LAND attempts despite receive failure, respect human ownership, and require a fresh post-request disarmed heartbeat for successful completion | Touchdown/disarm evidence; timeout is an unconfirmed landing, never permission to force-disarm |

Altitude hold does not hold XY position. ArduPilot's named `ALT_HOLD` mode has
pilot roll/pitch/yaw inputs; it is not automatically the correct autonomous
interface for this receiver-free aircraft. Keep the existing `GUIDED_NOGPS`
attitude/climb interface and add an explicit bounded hold operation, with a
separate test. Do not restore the former RC-override/AltHold takeoff workaround.
See [ArduPilot AltHold](https://ardupilot.org/copter/docs/altholdmode.html) and
the [project's pinned takeoff analysis](docs/ARDUCOPTER_4_7_NOGPS_LOITER.md#arducopter-47-autonomous-takeoff-and-loiter-handoff).

### 3.2 Invariants that every implementation must preserve

- Keep FC attitude, vertical stabilization and Loiter control in ArduPilot.
  The Pi supervises, supplies bounded requests, and records decisions; it does
  not introduce a Python motor-control loop or conceal EKF failures.
- Preserve the reviewed `GUID_OPTIONS=0` interpretation, 20 Hz takeoff/hold
  setpoints, one-second GCS heartbeat and LAND retry schedules, current firmware
  identity/features, battery checks and receiver-free topology until a separately
  tested change explicitly replaces an invariant.
- Give every altitude a named datum. `takeoff_gain_m` is relative to a fresh
  pad reading; `ceiling_m` remains floor-referenced. Require
  `ground_reference + takeoff_gain_m <= ceiling_m` before the first climb
  request, preferably before arming when fresh evidence is available. Keep a
  measured overshoot margin; equality is not proof of sufficient margin.
  Do not raise the physical ceiling by subtracting the pad reading from its guard.
- Keep raw range, barometer/local altitude and aligned local altitude distinct.
  Preserve the existing conservative range guard; document sensor mounting,
  tilt and floor geometry before introducing vertical-distance corrections.
  Forward MT-15 readings must never become altitude evidence.
- Parsing validates representation once. Freshness, ownership, cancellation,
  operation deadlines and sensor quality are checked at use/command boundaries.
  Sample timestamps are original receive/capture times, not queue-dequeue times.
- Observing an armed aircraft does not acquire control. An ambiguous arm write
  retains cleanup responsibility. An attempted first climb write requires LAND
  cleanup even if the write raises or no movement is observed.
- Human handoff is explicit and remains latched across disarm, stale RC,
  reconnect and operator loss. No autonomous flight command may overwrite it.
  Continue the required supervision heartbeat during human ownership.
- Preserve independent operator presence: SSH closure alone is not operator
  loss. Confirmed operator loss during autonomous flight requests LAND; it does
  not override an established human pilot. No network switching during flight.
- A camera/recorder/reporting failure must not prevent flight cleanup or starve
  control. Navigation-camera failure would be different if that camera becomes
  a required heading source: treat it as a navigation failure.

## 4. Magnetic interference with installed sensors

### 4.1 Establish the cause and the usable hall envelope

The user reports hall magnetic interference; the captured pre-arm faults support
a magnetic problem but do not establish its sole cause. Before changing yaw
sources, collect repeatable disarmed observations using the installed sensors:

1. Record sensor identities, exact firmware/features, complete parameter snapshot,
   compass calibration/orientation, selected EKF source set, battery conditions,
   sensor health, raw magnetic vectors, attitude/yaw, EKF diagnostics and pre-arm
   reasons. Save the project's own configuration for restoration.
2. Compare a magnetically cleaner location with the test area at marked positions,
   several headings and heights spanning the intended flight envelope. Use a
   marked physical reference for heading, not a phone compass as ground truth.
   Repeat after startup/warm-up and with the normal payload/camera powered.
3. Inspect the existing compass mounting, orientation, connectors and separation
   from power wiring. A disarmed test cannot establish motor-current interference;
   correlate current and magnetic data later in authorized flight evidence.
4. Validate downward range, flow direction/scale and quality over the actual floor,
   lighting and heights. Good magnetics alone do not establish good Loiter.

Calibration may correct sensor/airframe errors; it cannot be assumed to remove
a field disturbance that changes across the hall. Moving the installed compass
away from onboard interference may help without adding sensors. See
[magnetic interference](https://ardupilot.org/copter/docs/common-magnetic-interference.html)
and [compass calibration](https://ardupilot.org/copter/docs/common-compass-calibration-in-mission-planner.html).

### 4.2 Evaluate three explicit navigation profiles

Profiles are reviewed bundles of firmware capabilities, estimator sources,
observability assumptions, freshness limits and measured operating envelopes.
They are not arbitrary parameter presets or an automatic airborne fallback menu.

| Profile | Installed sensing | Decision |
| --- | --- | --- |
| Flow + compass yaw | MTF-01P, FC IMU/barometer, existing compass | Baseline. Accept only where normal pre-arm checks and measured heading/Loiter performance pass; a clean takeoff point alone does not qualify the whole test volume |
| Flow + unaided/inertial yaw | MTF-01P and FC IMU/barometer, without magnetic yaw fusion | First compass-independent feasibility experiment in pinned SITL. Qualify drift and duration before considering hardware; not a current supported flight profile |
| Flow + camera-derived yaw, or camera external navigation | Existing IMX500, calibrated camera-to-body transform and fixed AprilTag reference geometry | Conditional software/firmware development if inertial-yaw drift is unacceptable; no new sensors, but substantial calibration, performance and integration work |

**Why test inertial yaw rather than dismiss it:** in the pinned source,
`NavEKF3_core::readyToUseOptFlow()` explicitly does not require yaw alignment,
and `updateFilterStatus()` has a relative-aiding case without compass yaw.
The no-yaw-sensor branch uses static/predicted yaw handling. This supports a
feasibility experiment, not a claim of reliable compassless Loiter. Constant
heading offset, gyro bias/drift, yaw resets and horizontal-control behavior
must be evaluated separately. Source:
[EKF3 control](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_NavEKF3/AP_NavEKF3_Control.cpp),
[yaw fusion](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_NavEKF3/AP_NavEKF3_MagFusion.cpp),
[source selection](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/AP_NavEKF/AP_NavEKF_Source.cpp).

The experiment must establish source configuration and normal arming behavior
with `ARMING_SKIPCHK=0`; changing `EK3_SRC1_YAW` alone does not prove compass
checks, estimator selection and the companion's current `YAW=1` invariant agree.
Keep any candidate settings in a separately labelled simulator overlay. A future
nonmagnetic profile deliberately changes the estimator architecture; it must not
be implemented by skipping a failing check on the compass-dependent profile.

Without an independent heading reference, yaw drift is not reliably observable
from the attitude estimate or healthy EKF flags alone. Qualify that drift against
simulator truth and physical references, and enforce a conservative elapsed-time
limit measured from the profile's validated initialization/reference boundary.
Include warm-up, time spent armed before liftoff, climb and landing in that budget;
reserve time for landing before accepting a requested hold duration.

GSF is not the planned indoor replacement: its documented heading estimator
depends on usable GPS velocity, and the pinned source requires GPS for that
source selection. Dual-GPS heading would also violate the installed-sensors
constraint. [ArduPilot compassless operation](https://ardupilot.org/copter/docs/common-compassless.html).
FlowHold is a different mode, is disabled in the reviewed image, and is not an
automatic substitute for the required Loiter behavior.

### 4.3 Conditions for using the existing camera

Current AprilTag poses are camera-relative observations, not a navigation
solution. Before accepting camera-derived yaw, establish calibrated intrinsics
at the actual crop/resolution, camera-to-body extrinsics, known fixed tag
orientation/geometry, coordinate conventions and pose-ambiguity rejection.
No absolute north or metric world position may be invented from a tag ID.

Prototype against recorded/bench data first. Measure pose accuracy, capture-time
latency, delivered rate, long gaps, tag occlusion, blur, illumination, CPU/memory
and thermal behavior on the actual Pi Zero 2 W while recording. IMX500 presence
does not prove that the current AprilTag pipeline runs on its neural accelerator.
Existing acquisition tests detected no tags and claimed no metric calibration.

The reviewed FC image disables both external-navigation fusion and visual
odometry. A candidate needs a new reviewed overlay/manifest and linked-artifact
verification; the current image is 865,632 bytes against a 950,272-byte limit.
Build before promising that the additional features fit. Keep SITL and board
capability checks aligned.

Specify the FC message contract, transforms, timestamp/delay handling, reset
counters, uncertainty and validity. Do not fabricate unused position/velocity
measurements merely to carry yaw. Confirm the supported message path in the
pinned source, including shared-runtime write authorization and bandwidth.
ArduPilot documents ExternalNav yaw and companion pose inputs:
[EKF sources](https://ardupilot.org/copter/docs/common-ekf-sources.html),
[non-GPS position input](https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html).
Camera loss must have a tested response; never silently fall back to the
known-disturbed compass or replay stale poses with new timestamps.

### 4.4 Profile selection gate

Select a profile only after its normal and failure cases pass the matrix below.
Record maximum tested duration, drift, floor/lighting constraints, startup
conditions and disturbance envelope. No autonomous source switching is added
in the first implementation. If installed sensing cannot meet the envelope,
record the specific failed criterion and narrow the permitted operation; do
not describe rejected arming or successful mode entry as reliable hall Loiter.

## 5. Corrected finding register

These candidates have different evidence levels. Source-path confirmation is
not a reproduced failure; add the failing test before the fix. Do not combine
all rows into one "small safety PR".

| ID | Finding and evidence | Refined action |
| --- | --- | --- |
| F01 | `DroneController.land()` receives before sending; a failed shared endpoint raises before the retry | Attempt a bounded telemetry drain and resolve available human-takeover evidence; isolate receive/GCS-heartbeat failures so they cannot prevent a best-effort LAND write. Do not unconditionally send before ownership resolution. Preserve `test_takeover_during_landing_prevents_the_next_land_command` |
| F02 | A new shared endpoint's `mode_mapping()` can return `None` | Resolve the pinned Copter mapping in the flight layer, or expose established vehicle identity to the new subscription. Do not import `FlightSafetyError` into the lower-level MAVLink transport. Test LAND without a dequeued heartbeat and with failed RX |
| F03 | Ground-cleanup `disarm()` currently sends once | Retry at one-second intervals within its existing deadline, only where direct disarm is permitted. Keep pending-arm cleanup and require a newly received selected-FC disarmed heartbeat after the first cleanup request |
| F04 | Takeoff gain and ceiling use different datums | Validate pad reading + target + chosen overshoot allowance against the unchanged ceiling. Preserve independent range and aligned-local guards; reject before climbing |
| F05 | `_shared_final_action` can fall back to an unvalidated `_probe_fc` result | Apply the disarmed/age guard to the final observation regardless of its origin. `_runtime_fc` already checks disarm: the prior assertion that the runtime path skips it was incorrect. Test runtime disappearance, unavailable/busy/armed fallback and expired observation |
| F06 | Exceptions while constructing the recording summary can prevent manifest creation | Finalize after resource shutdown, inside a dedicated failure boundary. Preserve cleanup errors; write a minimal atomic manifest when storage is usable. Existing manifest-write `OSError` handling is real; do not promise a manifest when the filesystem cannot accept one |
| F07 | An interrupted mount-opening hold reports zero completed pulses despite an issued command | Preserve separate facts: command attempted/issued, hold completed, PWM detached, interruption and error. No physical feedback exists. Do not relabel every issued command as a completed hold; preserve mount-open behavior and normal pulse rest behavior |
| F08 | `VehicleServer.close()` retains its socket lock if a worker does not stop | Specify shutdown ownership before changing `finally`: quiesce endpoints, bound joins, preserve errors and prevent an old handler from producing effects alongside a replacement. Test timeout then restart. Unconditional lock release is not a complete fix |
| F09 | Servo lock open lacks `O_NOFOLLOW` | Reject symlinks and invalid ownership/type. Provision a stable per-boot directory independently of the restartable access service; test standalone CLI use, contention, restart and ownership lifetime. Moving into its `RuntimeDirectory=/run/ai-drone` alone can create an inode/lifetime race |
| F10 | Settings are CWD-relative with an optional default file | This is documented behavior, not a demonstrated wrong-host defect. Preserve precedence and explicit-file failure. Thread one parsed `Settings` through commands; expose the selected source where useful without warning on every library read |
| F11 | Network-profile JSON is accessed without a typed schema | Parse schema version, profile collection, types and required fields before use; report useful configuration errors and prevent unsafe network actions. Do not interpret malformed data as an empty valid policy |
| F12 | Generic recording directory names use local time | Use UTC names for new captures; retain collision handling and readability of existing paths. Do not rename historical artifacts |
| F13 | Two SSH-prefix builders have different parameter types | Consolidate representation and callers; their output logic is equivalent for matching configurations. This is duplication, not an established runtime incompatibility |
| F14 | Status validators/freshness windows differ | Share parsing and arithmetic; keep health-after-restart, disarmed-maintenance authorization, control freshness and capture-report freshness as explicit policies. A parsed status is not permanently authorized to act |
| F15 | Range validation differs for unspecified min/max bounds | Define wire-value semantics, source/ID/orientation identity, finite/positive range and quality handling. Share parsing with explicit policy where reporting and control need different outcomes; test missing bounds and invalid new samples |
| F16 | Handoff uses `result and 0` | Return an explicit exit code and validate the response. Current success is a nonempty dict; `int(False)` is valid. The prior generic "falsy means TypeError" claim was incorrect; classify this as clarity/boundary hardening |
| F17 | Recorder requests many streams on the shared link | Measure per-direction bytes, delivered rates, backlog, packet loss and control jitter under camera/recording load before reducing streams. At 115,200 baud with 8N1, each serial direction has nominal capacity 11,520 bytes/s; do not add full-duplex RX/TX into one capacity estimate |
| F18 | Rejected runtime request disconnects the peer and releases control ownership | Decide protocol-error versus recoverable-request-error behavior explicitly. Test subsequent control claims while armed and GCS-heartbeat continuity; do not change fail-closed semantics as deduplication |
| F19 | Capture workers mutate collections read by reporting/finalization | Provide locked updates and coherent immutable snapshots; keep serialization/I/O outside the lock. Test concurrent first-seen keys and shutdown snapshots deterministically; do not rely on the GIL |

**Parallel implementation checkpoint:** a read-only diff review of
`agy/refactor` at `08020d1` found that the original Stage 0 has already been
partly implemented there. It includes LAND receive-error suppression, repeated
ground disarm, the pad-plus-target ceiling check, a final power guard, recording
finalization changes, network schema checks and UTC naming. Reconcile that
commit against this register before creating duplicate fixes; no tests on that
branch were run as part of this plan revision.

The integration review must specifically resolve:

- F02: the shared transport now assumes a quadrotor mode map when it has no
  heartbeat; establish vehicle identity or keep that policy in the flight layer.
- F08: socket ownership is released even after worker-join timeout; prove that
  remaining workers cannot overlap a replacement server's effects.
- F09: servo locking tries `/run/ai-drone/bcm12-servo.lock` and falls back to
  `/tmp/ai-drone-bcm12-servo.lock` on an open/directory error. Processes choosing
  different files would not exclude one another. Require one stable lock
  namespace, consistent provisioning and restart/permission-failure tests.
- F10: an absent optional default configuration now emits a warning. Resolve
  that user-facing policy deliberately rather than treating it as a required fix.

Retain the previously rejected audit items as non-findings unless reproduced:
directory iteration during report replacement, integer-resolution aspect-ratio
comparison, and the motor-test heartbeat loop. Use supported Python versions
and actual blocking behavior rather than an abstract pattern match.

## 6. Functional design

### 6.1 Flight state and policy

Keep `DroneController` as the resource-owning shell. Extract focused pure modules;
do not promise a fixed attribute count, 320-line budget or 780-line controller.

- `flight/state.py`: immutable observations and `Sample(value, received_at)`;
  separate boot-clock anchor and altitude-alignment state; source-filtered
  `observe(previous, observation, now)`. Decode MAVLink into typed observations
  at the boundary. Preserve boot-time wrap handling, stale/future/out-of-order
  rejection, invalid-data revocation and reset behavior. A fresh attitude packet
  proves timeliness, not correct yaw. Add heading/estimator evidence as the
  selected profile needs it; unknown health stays explicit.
- `flight/phase.py`: distinguish ownership, cleanup responsibility and observed
  armed state. Start with `Unclaimed | ArmPending | Armed |
  Flight(stage, ground_reference) | Landing | Landed | Human`, with stages
  `TakingOff`, `HoldingAltitude`, `AwaitingLoiter`, `Loitering`. Define transitions
  and effect-attempt ordering before deleting flags. Heartbeats cannot turn
  passive observation into ownership. `Human` is absorbing for autonomous
  authority; `ArmPending` survives old disarmed data. Keep the landing latch
  after touchdown until an explicit new mission begins.
- `flight/limits.py`: explicit `FlightLimits`, `NavigationProfile` and pure
  readiness/violation/progress decisions. Separate vertical capability from
  Loiter capability without silently relaxing today's pre-arm gate. Degraded
  navigation cannot veto cleanup's LAND attempt. Define which reasons mean
  retry, refuse-start, request-LAND or yield-to-human.
- `flight/params.py`: one reviewed firmware identity/invariant definition per
  supported profile, usable by the checker and artifact verifier without
  importing the controller or initializing hardware. Experimental profiles
  must not weaken the baseline's exact checks.

Frozen records and tagged unions prevent some inconsistent combinations and
make tests simpler; they do not prove transitions or physical behavior. Keep
exhaustive matching checked by `ty`/`assert_never` where useful. Do not invent
a generic `Result` framework for ordinary Python exceptions.

### 6.2 Effect shell and failure behavior

Use a small typed command set (`SetMode`, `Arm`, `Disarm`, `Climb` plus explicitly
needed requests). Route autonomous flight writes through one ownership gate,
including command-ACK helpers. Keep passive telemetry requests and supervision
heartbeat separate and document their authorization.

Record intent before ambiguous arm/climb writes and outcomes separately from
observed confirmation. Best-effort emergency LAND should not mask the original
error, but a failed send remains visible; "never raises" cannot mean "succeeded".

Share a retry-loop mechanism for LAND and permitted ground disarm, with different
completion/ownership policies. Deadlines use monotonic time. Confirmation comes
from an accepted selected-FC heartbeat received after the first cleanup-request
boundary, not cached state, queue position, or after every resend. Continue
receive-error-tolerant retries until confirmed disarm or bounded timeout. A
timeout remains unconfirmed; preserve the independent FC failsafe.

Generalize waiting only after preserving source filtering, new-sample barriers,
cancellation points, heartbeat cadence and continuous-health windows. Combining
five pre-arm waits into one three-second window changes behavior: retain timing
initially and introduce a measured combined-readiness deadline separately.
Never increase a timeout merely until a simulator test passes.

Keep connection lifecycle, stream requests, firmware/parameter reads, heartbeat
pumping, command acknowledgements and human supervision in the shell. Moving
the Loiter stability timer into a pure transition is optional; reset and
continuous-health semantics are mandatory. Keep public methods as delegating
wrappers until repository callers migrate. Inventory removed methods explicitly;
do not simultaneously claim the whole public API is preserved.

### 6.3 Other state and resource owners

- **Tag servo:** extract qualification/streak/trigger selection as pure functions.
  Recheck stop, readiness, source heartbeat and detection age under the final
  actuator lock. Keep GPIO sequencing, interruption, detach-before-unlock and
  no-reclose mount behavior explicit. Simplify structural Protocols only where
  contracts are equivalent; `capture/state.py` does not contain three drop-in
  replacements for the existing Protocols.
- **Runtime network:** pure decisions plus explicit asynchronous job state.
  An `Idle/Activating/Deactivating/Blocked` union must also account for pending
  requests, maintenance and outstanding subprocess completion. Do not replace
  `network_busy` with `state is not Idle` without that mapping. Keep control
  acquisition versus network mutation atomic at the existing owner.
- **Recording:** compose resource contexts with `ExitStack`, preserving actual
  shutdown dependencies and error collection. Stop producers/actuation, join
  workers, flush/sync artifacts, and finalize the manifest in the order proven
  by lifecycle tests. Naive reverse acquisition order is not automatically
  equivalent. Preserve partial startup, optional-source degradation, signal
  restoration and bounded shutdown.
- **Capture state:** mutable behind a lock with coherent snapshot APIs, not a
  new immutable object for every counter increment.
- **Vision:** validate calibration on construction, normalize/freeze nested
  values, and retain boundary checks before removing repetition. Process-wide
  OpenCV thread settings need a deliberate startup owner including direct helper
  entry points. Preserve direct-construction and malformed-file behavior.

## 7. Implementation order and coverage of the original plan

Each item below is a focused commit/PR with its own tests and rollback boundary.
Tests move with the behavior they protect, not to a final test-rewrite stage.
Behavior-preserving extraction and changed flight policy must be reviewable
separately. Flight reliability work takes precedence over cosmetic cleanup.
Stage 2's survey and feasibility work can proceed alongside Stage 3 extraction.
Unavailable hardware does not block verified software work, but it leaves the
corresponding hardware qualification open. Production profile selection follows
the Stage 4 acceptance gate and section 8 hardware evidence.

### Stage 0 — reproducible baseline and flight contracts

- Capture the exact project commit, dependency lock, firmware source/artifact
  identity, baseline check results and known diagnostics. Preserve historical
  logs, snapshots and startup-tone functionality.
- Specify the mode/ownership transitions, altitude datums, sensor/profile
  contracts and acceptance matrix in sections 3, 4 and 8. Add characterization
  tests for current ambiguous writes, human takeover, cleanup and sensor identity.
- Resolve existing `ty` diagnostics with narrow boundary typing; a shared typed
  runtime-status parser may land here. Until that commit, compare with the
  captured diagnostic baseline and permit no new diagnostics; afterwards require
  a clean type check. Do not hide failures with broad ignores.
- Establish exclusive use or isolated ports for pinned SITL. Its current fixture
  skips when TCP 5760 is occupied; a skipped flight case does not pass a gate.

Exit: repeatable software baseline, explicit known failures, current acceptance
contracts, and no ambiguous claim that green unit tests establish hall readiness.

### Python 3.14 migration — separate compatibility gates

The intended upgrade to **standard, GIL-enabled Python 3.14** is an explicit
work item. It was absent from the original plan. The
[September 21 readiness check](state/2026-09-21/python-314-readiness.md)
records the live Pi inventory and isolated laptop experiment. Keep this as a
separate migration from flight-policy changes; it does not replace the flight
acceptance gates or block independent correctness repairs.

- **Laptops and CI:** test 3.14 in an isolated uv environment, expand
  `requires-python` only with passing dependency/application checks, regenerate
  `uv.lock`, and add 3.14 to CI. Update the development default and relevant
  commands deliberately. Keep syntax/lint targets compatible with the oldest
  retained version; preserve a tested 3.13 route for the Pi during migration.
- **Pi feasibility:** uv lists a compatible ARM64 3.14 interpreter, but the
  deployed environment uses system Python 3.13.5 and apt-provided native
  bindings. Installed libcamera, pyKMS, lgpio, prctl and AprilTag extensions
  target CPython 3.13. `--system-site-packages` cannot bridge that ABI change.
  Inventory all transitive native requirements and obtain/rebuild matching 3.14
  ARM64 artifacts, including bindings for the installed libcamera version.
- **Candidate and deployment:** prepare a separate uv-managed environment;
  retain `/usr/bin/python3` and the working application environment. Update
  deployment interpreter selection and validation so recovery cannot silently
  choose the wrong minor version. Preserve source/environment rollback together.
  Avoid broad OS/package upgrades as a shortcut to this migration.
- **Pi acceptance:** require imports plus actual disarmed camera acquisition,
  video/manifest finalization, AprilTag processing, GPIO-backend compatibility
  without actuation, and read-only runtime telemetry. Measure memory, CPU,
  camera throughput and control scheduling under representative load. Then run
  the existing deployment/rollback gates before selecting 3.14 for service use.
  Free-threaded Python is not part of the initial upgrade.

Exit: record laptop/CI 3.14 support and Pi 3.14 qualification separately. A
successful x86_64 test run or interpreter installation is not proof that the
Pi's camera/GPIO stack works under 3.14. Until that gate passes, retain the Pi's
working 3.13 runtime and an explicit outstanding migration item.

### Stage 1 — small correctness repairs before restructuring

1. Flight-cleanup PRs: F01–F04, each with a reproducing regression and the pinned
   production-path SITL gate after the relevant controller change. Preserve
   takeover-before-write and new-heartbeat confirmation.
2. Maintenance/resource PRs: F05, F08, F09 and F11, including shutdown/restart
   ownership and malformed/unavailable observations. Lock directory provisioning
   and service-lifetime tests belong with the lock change.
3. Recording/diagnostic PRs: F06, F07 and F19. Add manifest-finalization and
   interrupted-command tests without changing actuator motion semantics.
4. Small independent changes: F12 and F16. Treat F10 and F13–F15 as policy/API
   cleanup with explicitly preserved contracts, not a bundle of confirmed bugs.

Exit: fixes are individually explainable and failure paths retain their evidence.
No profile changes or weakened safety thresholds are hidden in this stage.

### Stage 2 — magnetic and flight-profile feasibility

- Implement read-only diagnostic/report improvements needed for the hall survey;
  distinguish raw sensor health, estimator aiding, heading confidence, and
  pre-arm acceptance. Integrate collected magnetic vectors, yaw/reset events,
  innovations where available, current and height into one review timeline.
- Extend pinned SITL with magnetic disturbances and gyro-bias/drift experiments.
  Run the existing production path, not a replacement flight sequence in tests.
  Add an explicitly experimental profile adapter if the hard-coded compass
  invariant prevents the simulator experiment; keep normal invocation unchanged.
- Evaluate flow/inertial yaw before committing to camera navigation. If it fails
  the required duration/drift envelope, prototype the installed-camera route and
  check firmware fit and measured Pi throughput before scheduling integration.
- Conduct the section 4 disarmed sensor survey when aircraft access is available.
  Store observations with profile/firmware/parameter hashes. Report capability
  limits if neither nonmagnetic candidate qualifies.

Exit: nominate a candidate profile and provisional envelope for Stage 4, with
simulator/bench evidence and explicit remaining qualification, or retain a named
feasibility blocker. A successful clean-field baseline is not completion of the
magnetic-interference requirement.

### Stage 3 — extract the flight core in small steps

1. Extract typed observations and pure state updates. Keep public properties as
   adapters and port message-driven tests to `test_flight_state.py`; retain
   end-to-end shell tests. Preserve timing/invalid-data behavior explicitly.
2. Extract the phase/ownership transition model and outbound flight-command gate.
   Cover the transition table and ambiguous effects before removing old flags.
   Introduce shared retry mechanics only with LAND/disarm-specific policies.
3. Extract limits/readiness/progress policy. Fold all required checks from
   `flight/guards.py`, `_enforce_flight_limits`, `hold_loiter` and `control._monitor`
   into the new policies; resolve differing behavior with explicit tests.
   Then remove `guards.py` and `_monitor` once, and migrate their coverage.
   `hold_loiter` is retained as the public behavior, not counted as dead-code savings.
4. Generalize repeated polling and remove obsolete public wrappers only after a
   repository caller search. Keep the continuous navigation-health gate and
   fresh-sample barriers. Any combined pre-arm timeout change gets a separate
   behavior commit and measured startup/failure cases.

Run the pinned SITL gate after each sub-stage. Defer altitude-hold feature changes
and new profile behavior to Stage 4 so extraction regressions are distinguishable.

### Stage 4 — explicit altitude hold and qualified hall Loiter

- Add `hold_altitude(duration)` and a clear CLI operation, proposed as
  `drone control altitude-hold`: take off, hold vertically for a bounded period,
  then LAND. Reuse the reviewed `GUIDED_NOGPS` interface and record its actual
  altitude target and hold entry. Keep existing `drone control hover` behavior
  compatible; describe it explicitly as takeoff–Loiter–landing.
- Introduce the selected navigation profile through exact firmware/parameter
  validation and operation-specific readiness. Decoupling vertical readiness
  from Loiter readiness is a deliberate change requiring its own cases; do not
  globally remove flow, compass or freshness prerequisites during cleanup.
- Retain a bounded neutral-climb wait for relative aiding before Loiter. Failure
  to acquire it requests LAND. Unexpected loss of the selected profile's required
  inputs in Loiter requests LAND; no indefinite altitude-hold fallback or
  automatic switch to an unqualified compass/source set.
- If the camera profile is selected, integrate its validated message path and
  firmware artifact as separate changes. Keep pose estimation failure isolated
  from reporting, and make navigation failure visible to the controller.

Exit: takeoff, altitude hold, Loiter and landing have separate production-path
acceptance results, including the selected magnetic-disturbance envelope.

### Stage 5 — approved deletions, shared boundaries and command cleanup

These can proceed alongside the flight track where files/contracts do not overlap.

| Original proposal | Refined scope and acceptance |
| --- | --- |
| Delete `scripts/disarmed_tag_mount.py` and its tests | Remove it and the deploy-list entry; preserve `scripts/tag_mount_capture.py`. Transfer any unique safety/lifecycle coverage before deleting tests |
| Remove USB auto-detection | Delete platform guessing helpers and their specific tests; keep explicit `--usb-iface`, `iface_exists`, cross-platform explicit-interface behavior and USB recovery |
| Protocol/unreachable-code cleanup | Compare structural contracts before consolidating. Use definitely initialized locals to remove redundant post-construction checks; retain partial-init resource cleanup |
| Remove fixed-value `--pin`, `mount.current_value`, unused site `Page` fields | Search all source/docs/callers; remove or migrate deliberately. Pin selection remains fixed at BCM12; render the site after `Page` changes |
| New `system.py` and subprocess helpers | Share systemctl parsing, subprocess result/error representation and systemd command construction where equivalent. Preserve per-command deadlines, accepted return codes, missing-unit states and privileges. Avoid a catch-all helper module |
| `require_uv`, namespace-to-flags and repeated command printing | Extract only when callers share semantics; round-trip boolean/list/path flags and retain dry-run behavior. Printed shell previews must not become execution strings |
| One SSH prefix and remote `uv` builder | Consolidate in the link layer with tested quoting, argument boundaries, SSH config, remote paths and environment rules. Preserve transfer/deployment safety checks |
| `remote.py` / `shared.py` duplication | Share genuine message/endpoint contracts; retain wire parsing, peer lifetime, error propagation, queue overflow and receive timestamp semantics specific to each transport |
| JSON scalar/bounds validators | Distinguish strict JSON integers/numbers from CLI coercion, booleans, nonfinite/overflow values and optional fields. Apply at demonstrated boundaries in config, transfer, vision, settings and review |
| Freshness helper | Share finite/nonnegative age arithmetic with explicit `now` and policy-specific limits. Audit inclusive boundaries and future timestamps; do not replace all 2.0/2.5 s policies with one constant by assumption |
| Runtime status record | One parsed record in a small dependency-appropriate module, used by power/deploy/setup. Separate health, freshness and authorization predicates; preserve selected FC identity and receipt/publication time semantics |
| Transfer `Endpoint`, `Inventory`, `Receipt` | Parse to typed domain records and reject malformed data at the edge; remove broad `KeyError/TypeError` catches only after errors have a stable explicit mapping. Preserve verified-copy-before-delete |
| Power `PiSnapshot` / `FcProbe` | Model observed, absent, busy and unavailable/unknown cases, carrying useful reasons. None of the unavailable cases authorizes power action; do not lose the existing serial-owner checks |
| Settings injection | Pass one `Settings` from the dispatcher; preserve direct module/script entry points, documented flag/env/file precedence and per-machine ignored configuration |
| `handled_signals()` | Share main-thread installation/restoration mechanics with an explicit mapping. Keep flight SIGHUP ignore, recorder cancellation, nested restoration and noninterruptible LAND semantics distinct |
| CLI harness and dispatcher | Register power/servo/mount/motor-test/config-export where the command layout stays clear. Prove common behavior with two or three adapters before wider adoption; do not force all 16 commands through one resource/exception policy |
| Remove four-line shims | Only after scripts, installed services, deployment manifests and operator instructions migrate together. Tiny adapters may be worth retaining for a stable interface |

Add `ai_drone.review` to `.importlinter` with the intended dependency direction;
check the new shared modules without allowing transport to depend on flight policy.

Exit: each deletion has a caller/coverage check; helpers reduce duplicated knowledge
without changing policy. Include `ai_drone.review` and new leaf modules in the
appropriate import contracts, preserving the existing dependency direction.

### Stage 6 — recording, vision, reports and runtime structure

- Apply the resource/state splits in section 6.3 with partial-startup, failed
  cleanup and concurrency tests. Recording teardown remains independent from
  flight-control cleanup and preserves the log storage reserve.
- Introduce `Manifest`, component/file records and operation variants where they
  remove ambiguity. Preserve JSON keys, schema versions, optional fields, artifact
  paths and compatibility with historical captures; use explicit serializers
  when `asdict()` deep copying or generic `json_safe` would hide invalid values.
  Never serialize resources/locks through the manifest model.
- Use per-operation parser specifications only for actual common options; keep
  inspect, tag-servo and tag-mount safety/confirmation differences explicit.
- Build the review `Signal` table for simple scalar mappings and derive CSV
  columns from it. Keep special handlers for range validity, sentinel values,
  multiple sensor instances, integrated flow and malformed historical rows.
  Assert output column order/units and compare representative saved fixtures.
- Make check analysers return structured summaries and diagnostics, rendered at
  the CLI boundary. Index observations by the full relevant identity: source,
  message type, sensor ID/orientation for range, and parameter name for parameters.
  `(type, orientation)` alone does not distinguish every sensor or parameter.
- Consolidate calibration/detector validation without changing OpenCV/native
  backend behavior, crop geometry or error diagnostics. Changes used by a future
  navigation camera must preserve its stricter timing/pose contract.
- Make firmware scalar-field validation declarative where useful, while keeping
  cross-field image-size, decompression, board identity, feature and hash checks
  explicit. Preserve exactly reviewed feature deltas and historical baseline hashes.
- Replace deployment booleans with states that retain rollback/source-install/
  service-restart obligations. Retain the fail-closed maintenance transaction and
  read-only post-restart health check, including when new clients appear.
- Extract network decisions and `LinkState` only with pending-job/control-owner
  race tests and preserved grounded-only behavior. Model external effects'
  completion or failure, not just their requested state.

Exit: old recordings still load, interrupted operations produce honest summaries,
and optional diagnostics cannot destabilize flight control.

### Stage 7 — integration, tooling, documentation and deployment evidence

- Keep existing Ruff, type, dependency and import checks mandatory. Add useful
  `BLE`/`ARG` rules incrementally with narrow justified exceptions. Evaluate
  `TRY`, `FBT` and `PLR0913` against actual defects/readability; do not introduce
  thousands of stylistic changes or obscure explicit keyword-only options to
  satisfy a numeric rule. A `[tool.ty]` section is not itself stronger typing.
- Update architecture boundaries, command list, flight/profile procedures,
  firmware verification and rollback instructions alongside relevant changes.
  Keep the README concise and preserve dated evidence, the website/poster and
  startup-tone materials. Relocating the HTML template is not a line deletion.
- If `review/html.py` moves its template to `review/report.html`, include it in
  built distributions and deployment, load via `importlib.resources`, and test
  installed-package report generation, escaping and generated asset references.
- Integrate each agent's commits into an integration branch and run the combined
  gates. Worktrees do not isolate physical FC/GPIO, fixed simulator ports, or
  remote services: coordinate those resources explicitly.
- Prepare a reviewed deployment and restoration bundle. Disarmed Pi checks must
  exercise normal entry points, runtime status, short capture with servo disabled,
  and a power snapshot; do not substitute a shutdown command for a snapshot.
  Verify installed source/firmware/parameters separately from local branch state.

Exit: combined software gates pass, disarmed deployment evidence is recorded,
and flight claims are limited to the profiles/envelopes actually demonstrated.

## 8. Acceptance matrix and verification

### 8.1 Quantitative flight acceptance

Retain the current five pinned production-path SITL cases and their assertions:
downward-only hover, forward/downward coexistence, hard GCS-process loss,
recording through hangup/operator loss, and explicit human handoff. The existing
normal-hover gate checks at least 90% of target height, height below 0.8 m,
Loiter height at least 0.3 m, maximum XY displacement 0.5 m, required relative
aiding, and final disarm. Do not reduce these checks to make new tests pass.

Add these initial **engineering acceptance targets**, not claims of measured
hardware performance: 10-second and then 30-second holds; after a two-second
settling allowance, altitude error within 0.10 m of the declared floor-referenced
target; Loiter XY displacement at most 0.5 m; uncommanded heading motion and
yaw-estimate drift at most 10 degrees during the hold. Measure yaw-estimate
drift after accounting for a constant initial reference-frame offset. Vertical
hold gets an XY drift measurement/clearance criterion but no position-hold claim.
Validate each profile over the entire armed sequence, including climb and LAND,
not only the hold interval. Limit requested mission duration to its qualified
envelope; the existing 3,600-second parser limit is not qualification evidence.

Freeze test thresholds before comparing candidates. Tighten them to the usable
hall clearance and measured dynamics before live qualification; changes require
an explained operating-envelope decision, not silent test relaxation. Use
simulator truth for SITL and independent physical references for hardware
measurement. An EKF estimate cannot validate itself.

| Scenario | Required result/evidence |
| --- | --- |
| Normal takeoff / standalone altitude hold / Loiter / LAND | Exercise actual CLI/controller, measure settling, height, XY and yaw, preserve mode, and confirm disarm. Test multiple starting headings, ground offsets and durations |
| Ground offset and ceiling | Refuse an incompatible target before climb; test near-boundary targets and sensor offsets. Both range and aligned local altitude preserve the ceiling; the forward sensor never triggers an altitude limit |
| No liftoff, stalled climb, overshoot or unexpected descent | Bound takeoff and progress waits; preserve the ceiling response and phase-appropriate cleanup. An attempted climb cannot be reclassified as ground-only cleanup because one sensor reports no movement |
| Magnetic anomaly before arm | Compass profile refuses unhealthy yaw/pre-arm state with a concrete reason. A candidate nonmagnetic profile passes only through its valid estimator configuration and normal remaining checks |
| Magnetic anomaly introduced after takeoff or during Loiter | Qualified nonmagnetic behavior remains within the declared envelope; a profile that loses required health requests LAND. Healthy-looking flags with truth drift outside the bound fail qualification |
| Heading offsets, gyro bias, warm-up, resets and drift | Distinguish constant frame rotation from growing error; no unnoticed yaw jump or accumulating drift outside the declared duration limit |
| Flow/range loss, low quality, wrong orientation/ID, repeated or delayed data | Reject invalid evidence and use original receipt time. Refuse start or initiate the phase-appropriate LAND response; test range/flow recovery without automatically resuming a failed mission |
| Barometer/local-altitude divergence and range discontinuities | Log both datums, detect disagreement according to measured policy, preserve ceiling and cleanup. Do not switch vertical sources automatically |
| No relative aiding after takeoff or rejected Loiter entry | Maintain the bounded neutral-climb waiting behavior, then LAND on timeout/rejection; never report Loiter success from a sent request |
| LAND under absent horizontal aiding | Exercise the pinned FC's position-capable and position-unavailable LAND behavior; record lateral motion and touchdown. LAND requests cannot promise precision position hold after navigation failure |
| Dropped LAND/disarm, failed receive, overflowed subscriber, failed outbound write | Retry allowed commands, keep attempts/errors visible, preserve heartbeat where possible, and never falsely confirm disarm from cached/queued state |
| Loss of FC link or hard companion termination | Exercise FC-side failsafes as well as companion cleanup; require actual LAND/disarm in the simulator and retained failure evidence |
| Human takeover before/during cleanup | Resolve fresh queued handoff before the next autonomous command; no LAND/disarm/climb write after confirmed human ownership |
| Operator loss versus terminal hangup | Autonomous operator loss requests LAND; SIGHUP alone does not. Recording continues as specified; established human ownership remains intact |
| Unexpected mode/RC topology change | Preserve explicit handoff handling; otherwise refuse/land according to phase. Do not accidentally interpret a newly connected low-throttle receiver as neutral input |
| Full recording/video/tag workload, slow disk and low storage | Measure actual serial RX/TX, dropped/backlogged telemetry, heartbeat/setpoint gaps and CPU/thermal behavior. Diagnostic degradation cannot block control or finalization |
| Camera profile, if selected: occlusion, ambiguous pose, latency, reset, tag/map change | Reject stale/invalid pose, carry reset/uncertainty correctly and execute the selected failure response; no implicit compass fallback |
| Interrupted initialization/shutdown and runtime restart | Preserve exclusive hardware ownership, attempt all independent cleanup, retain first and secondary errors, and write honest partial manifests when possible |

For magnetic SITL tests, verify available parameters from the pinned source and
runtime: `SIM_MAG1_OFS_*`, `SIM_MAG_ALY_*`/`SIM_MAG_ALY_HGT`, `SIM_MAG_RND`
and `SIM_MAG1_FAIL` provide starting mechanisms. Exercise all enabled simulated
compasses so a healthy extra instance cannot hide the intended fault. A static
offset is not a spatial hall-field model; include changing anomalies and replay
representative measured scenarios when available. Record injection timing and
amplitude with the result. Source:
[pinned SITL magnetic model parameters](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/libraries/SITL/SITL.cpp).
The pinned LAND implementation chooses horizontal-position control using
`position_ok()`:
[mode_land.cpp](https://github.com/ArduPilot/ardupilot/blob/dbe792162d06cab66c3475fd5556bf7a120f119e/ArduCopter/mode_land.cpp).

For each new profile, run at least three independent simulator starts for normal
and magnetic cases, with varied startup headings and documented seeds/biases.
Repetition reduces accidental success but is not a reliability probability.
Attach truth traces, tlogs/DataFlash, command/transition events, exact binaries,
parameter delta, dependency lock and generated summary to the result.

### 8.2 Software checks

Run applicable focused tests during each change and the repository gates before
integration. Preserve shell-level, failure-isolation and lifecycle tests while
moving decision cases to tables of pure inputs/outputs. Do not replace an
observable effect assertion with a test of the new implementation's own output.

```bash
uv run --locked --group dev --group docs ruff format --check .
uv run --locked --group dev --group docs ruff check .
uv run --locked --group dev --group docs ruff check . --select C901 --ignore-noqa
uv run --locked --group dev --group docs ty check .
uv run --locked --group dev --group docs lint-imports
uv run --locked --group dev --group docs deptry .
uv run --locked --group dev --group docs pytest -q -m 'not sitl'
uv run --locked --group dev --group docs python site/build.py
```

After each flight-behavior/core sub-stage and the final combined integration:

```bash
ARDUPILOT_ROOT=/home/abaris/drone/ardupilot UV_CACHE_DIR=/tmp/uv-cache \
  uv run --locked --group dev pytest -m sitl -vv -s
```

The directory must contain the verified pinned binary/feature build; a source
checkout at the right commit alone is insufficient. Use a separate, recorded
build directory/checkout for changed firmware. Required SITL cases must actually
run: unavailable binaries, busy ports, omitted environment variables or skipped
cases make this gate incomplete. Preserve and investigate failures before reruns.

For packaging/template/dispatcher changes, also verify an installed build in an
isolated uv-managed environment. For plan-only edits, check Markdown structure,
links and diff integrity; rerunning flight tests does not validate prose.

### 8.3 Hardware progression and completion

1. Preserve a fresh restorable snapshot; perform normal disarmed checks and the
   hall sensor survey. All live access uses `ssh seb@seb-is-pm`.
2. Validate the chosen profile's firmware/configuration and sensor behavior in
   disarmed bench/hand-motion tests. If nonmagnetic yaw is proposed, document why
   omitted compass fusion is correct for that profile rather than bypassing a
   failed check. Keep all unrelated checks and FC failsafes intact.
3. When the owner schedules supervised flight, progress from takeoff–LAND, to
   short altitude hold, to short Loiter, then the longer qualified duration.
   Use the normal guarded entry point and existing independent emergency
   arrangement; do not add an RC receiver without revisiting its topology contract.
4. Compare measured motion with section 8.1 and retain logs for unsuccessful
   attempts too. A successful check, hand lift, simulator run, or single flight
   does not establish performance across the whole hall.

Completion means all four flight capabilities are demonstrated within a named
installed-sensor profile and measured hall envelope, cleanup/failsafes survive
the relevant fault cases, retained recording/maintenance functions pass their
contracts, and the implementation/documentation gates pass. If a flight profile
does not qualify, report that limitation separately from completed software
cleanup. Reduced line count alone is never the completion criterion.
