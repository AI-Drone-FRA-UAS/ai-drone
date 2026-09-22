# Refactor implementation progress

Updated 2026-09-22 after resuming the preserved `12ccd2d` checkpoint; current
runtime source is `461bb1e`; its final validation is in progress.
Integration branch: `refactor/concise-runtime`, in `ai-drone`. Expanded plan
revisions `2eefca0` and `84855d0` are integrated; `ai-drone-codex` and
Antigravity's `ai-drone-agy` remain preserved inputs.

## Completed implementation

- [x] Reconcile Antigravity `08020d1` / `dd6a6c1`; preserve their commits and
  external worktree drafts.
- [x] Correct LAND/disarm retries, takeover-before-write, ambiguous writes,
  positive handoff acknowledgement and fresh post-request disarm proof.
- [x] Extract immutable observations, phase/command policy, limits and polling;
  retain the production-path simulator gates after the flight sub-stages.
- [x] Implement explicit autonomous altitude hold and retain separate Loiter
  readiness, neutral-climb waiting, ownership and navigation-loss policies.
- [x] Evaluate compass/inertial yaw, magnetic faults, gyro drift and durations
  in isolated pinned SITL; retain unsuccessful qualification results.
- [x] Repair false success after unexpected descent and, in `ff40754`, abort an
  owned flight on the pinned FC's fresh magnetic yaw-reset warning. The latter
  recognizes a reported reset; it does not measure arbitrary heading error.
- [x] Complete runtime status, power, transfer, deployment and network boundaries;
  provision one per-boot servo lock namespace and preserve hardware exclusion.
- [x] Extract tag decisions, immutable capture snapshots, manifests and operation
  records; preserve calibration and interrupted-command diagnostics.
- [x] Compose independent recording teardown; `12ccd2d` subsequently removes
  duplicate cleanup while preserving lifecycle assertions.
- [x] Close drained telemetry before slow camera teardown (`f520dd0`), preventing
  discarded messages from accumulating after the reader has stopped; 245
  existing affected cases pass on each Python.
- [x] Preload OpenCV before FC subscription (`461bb1e`), preserving later fresh
  disarm checks and camera acquisition. A successful diagnostic repeat correlated
  its slow import with queue growth; the failed run's exact stack was not captured.
- [x] Share strict scalar/freshness/remote/system boundaries, inject one command
  configuration, register commands and retain required compatibility adapters.
- [x] Complete checker identity/diagnostics, signal table, hall timeline,
  packaged report resource and declarative firmware checks.
- [x] Validate standard Python 3.14 development/CI configuration while preserving
  Python 3.13 support and the Pi's selected environment.
- [x] Measure actual transport bytes and host recording/control workload;
  reject partial physical writes and repair idle TCP accounting.
- [x] Freeze transport measurement before intentional descriptor closure
  (`9b4336d`), avoiding a false shutdown RX error; 110 affected existing cases
  pass on each of Python 3.13 and 3.14.
- [x] Verify actual loopback-only simulator interfaces before enabling experiments.
- [x] Prepare immutable runtime review and restoration bundles. `3a64313` adds
  explicit offline native payload deployment, positive qualification checks,
  source/wheel/interpreter/library identities and paired rollback retention.
  Generic 3.14 deployment and unqualified payloads remain refused.

## Validation checkpoint

At `f520dd0`, **1,868 existing host tests passed on each of CPython 3.13.15
and 3.14.7**, in 21.47 s and 21.83 s respectively, with zero skips and 40 SITL
cases deselected for separate execution. Formatting, lint, typing, import
contracts, dependency, site and packaging gates passed. The earlier `ff40754`
host suites passed too; their initial sandbox socket failures were retained and
the affected execution rerun outside the sandbox.

The completed pinned `9b4336d` simulator matrix recorded **35 passed / five failed /
zero skipped**, all 40 distinct cases executed. Baseline 5/5, Loiter profiles
8/8, faults 11/11, dynamics 7/7 and recording workload 1/1 passed. Vertical
profiles passed 3/8. The compass changing-field case now records mission failure,
LAND and disarm instead of
false success after a reported yaw reset.
The later recording-only cleanup at `f520dd0` passed its affected isolated
recording and full-workload cases (56.33 s and 61.42 s); no controller code
changed after the full matrix. These are two additional cases, not another
40-case execution at the later revision.

All five remaining failures are 30-second vertical-hold XY clearance:
**1.378–1.458 m during hold**, against the unchanged 0.5 m criterion;
**1.600–1.697 m over the full armed sequence**. The three 10-second cases moved
0.472–0.493 m during hold and **0.710–0.731 m over the full sequence**. All
profile runs disarmed. Altitude hold does not control XY; its 10-second physical
endpoint cap is provisional and does not qualify confined-hall clearance.
The acceptance gate remains **not green**. No threshold was relaxed.

At `12ccd2d`, the recording cleanup passed **274 affected cases on each Python**,
three installed-package cases, static checks and two affected isolated SITL
cases before the final `9b4336d` gates above. The earlier `ff40754` full matrix
and failed runs remain preserved. No tests were added for either resumed source
change.

The native deployment change passed 86 existing deployment tests on each Python
and isolated host install, launcher, environment-isolation and rollback replay.
The 30-wheel ARM64 payload passed offline extraction and group-preservation checks;
this is packaging evidence, not ARM execution or live service promotion.

At `461bb1e`, tracked production Python totals 22,282 lines versus baseline
19,271: **+3,011 (15.62%)**. Including the relocated 517-line HTML template gives
22,799 lines, **+3,528 (18.31%)**. Tests total 25,182 versus 19,533: +5,649.
`12ccd2d` removed 45 production lines and 19 test lines; native deployment added
465 production lines and one fixture line, with no new tests. The larger
refactor grew because it also implemented operations, safety handling and
runtime/deployment support. No further line-reduction target is claimed.
The separate 133-line native build patch records 30 C lines added and 34 removed
from the external AprilTag wrapper; it is not part of the Python count.

## Completed permitted Pi work

The working installation remains on system-site Python 3.13.5 and its existing
console-script layout. Staged source and foreground candidate runs do not mean
the new shared runtime service has been installed or selected.

- [x] Recover SSH and inspect candidate resources: no old compiler remained.
  An initial missing-heartbeat probe was retained; a repeat established fresh
  selected-FC disarmed state and clear ownership before hardware checks.
- [x] Preserve Pi source/environment and system/service/network configuration
  archives, FC identity, all 1,187 parameters, one mission item and empty fence
  and rally lists. Private restoration data is retained separately from public
  evidence. This is not a byte-for-byte FC flash backup.
- [x] Repeat complete 35,238-byte installed ROMFS retrieval with successful FTP
  completion and the reviewed hash; retain the earlier failed client cleanup.
- [x] Perform passive Python 3.13 recordings and disarmed checker runs. Latest
  source capture: 10.039 s, 43 camera frames, 881 messages, zero tags/servo
  commands, followed by fresh disarmed proof.
- [x] Install nine matching local native wheels plus dependencies in the separate
  uv-managed Python 3.14 candidate; all native imports succeeded. Preserve the
  system Python, working 3.13 `.venv` and service selection.

The temporary synthetic tag helper failures are preserved. Matching-source
`3a64313` native smoke now passes in 15.335 s, including imports, native ID17,
the project adapter and blank-image behavior. Actual ARM installation and a
subsequent exact sync preserve all 30 wheels; the CLI works outside the source
directory. A first 10.013 s passive native capture produced 80 analyzed frames,
263 valid H264 frames and 1,596 telemetry messages with no tags or servo commands.
Fresh disarmed proofs bracketed it. Its supervisor nevertheless exited 1 after
the uv wrapper returned 143 during shutdown; that failed whole-check result is
retained. A second run corrected shutdown (runtime and wrapper both exited 0),
but recording failed with a real 512-message subscriber overflow, 513 discarded
messages and stale telemetry. Native runtime qualification remains open while
the corrected native binding is prepared: the original detector held the GIL
and changed process SIGINT handling. Its reviewed C patch releases the GIL for
native detection, guards overlapping calls and preserves Python signal handling
(`44d0818`, patch/build instructions only).
Host 3.13/3.14 API/concurrency checks pass; the 3.14 ARM candidate rerun remains
unqualified. Its latest gil1 replay had no overflow when a slow SD status-file
publication failed the unchanged freshness bound: 1.771 s publication delay plus
0.298 s heartbeat age exceeded 2 s. The later teardown overflow is corrected by
`f520dd0`. The temporary helper now uses private tmpfs status/socket paths,
matching production storage; recording artifacts remain on SD. The working Pi
3.13 Debian binding is unchanged. Queue and freshness limits are unchanged, and
incomplete payload qualification still blocks promotion.

## Remaining gates

- [ ] Resolve the observed passive subscriber overflow, complete candidate
  acquisition/runtime/resource checks and attach positive matching-source
  evidence to the prepared native deployment payload.
- [ ] Resolve or retain the five failed vertical-clearance qualifications. A
  wider operating envelope requires separate measured review; changing an
  assertion or calling vertical hold position hold cannot close this gate.
- [ ] Complete physical hall/heading/range/flow survey and measured altitude
  disagreement policy. Fixed-position recordings cannot supply these references.
- [ ] Verify the reported Pi-powered FC arrangement and stable power/UART
  continuity. Fresh-heartbeat and no-heartbeat probes both occurred on resume;
  Pi reachability does not establish a qualified FC supply or flight power path.
- [ ] Qualify actual service deployment/restart/rollback and sustained physical
  transport/timing; the live service selection remains unchanged.
- [ ] Perform separately authorized supervised flight and actuator qualification.

All simulator endpoints run inside namespaces with only loopback and no route
to the aircraft. Host, simulator and Pi results remain separate. The physical
drone stays grounded/disarmed: no live arm, takeoff, mode, LAND, motor, throttle,
RC override, mission-start, servo/payload, FC parameter/firmware write or service
switch is authorized here. Software completion does not establish flight readiness.

See the [implementation report](refactor-implementation-report.md),
[Pi report](pi-disarmed-refactor-checks.md),
[native candidate report](python-314-candidate.md),
[fault matrix](fault-matrix.md) and
[hardware qualification checklist](hardware-qualification-checklist.md).
