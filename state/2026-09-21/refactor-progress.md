# Refactor implementation progress

Integration branch: `refactor/concise-runtime`, in `ai-drone`. The existing
`ai-drone-codex` and `ai-drone-agy` worktrees are read-only inputs. Plan revisions
`2eefca0` and `84855d0` are integrated without replacing preserved drafts.

## Implementation checklist

- [x] Reconcile Antigravity `08020d1` / `dd6a6c1`; preserve their commits.
- [x] Correct LAND/disarm retry, takeover-before-write and fresh disarm proof.
- [x] Run five pinned production-path cases against flight fixes `9de5fa1`.
- [x] Extract immutable flight observations (`babc8ae`); five-case gate passed.
- [x] Complete phase/command, limits and polling extraction, with separate gates.
- [x] Add autonomous altitude hold and its production-path acceptance cases.
- [x] Test inertial-yaw feasibility, magnetic faults, heading/gyro drift and duration;
  preserve the unqualified compass-disturbance and 30-second vertical cases.
- [x] Reconcile runtime shutdown exclusion and typed saved network policy.
- [x] Share typed runtime-status parsing with distinct authorization policies.
- [x] Provision a single per-boot servo lock namespace independent of service restart.
- [x] Complete typed power and transfer boundaries, network job and deployment states.
- [x] Restore tag diagnostic labels and honest interrupted command accounting.
- [x] Extract tag decisions, synchronize capture snapshots, freeze calibration.
- [x] Compose recording teardown with explicit dependency order and independent cleanup.
- [x] Complete immutable capture snapshots, manifest and operation/parser records.
- [x] Share strict JSON, range/freshness, remote command and system helpers.
- [x] Inject one command configuration without environment mutation; register commands.
- [x] Complete checker identity/diagnostics, report signal table and hall timeline.
- [x] Verify packaged HTML resource, declarative firmware checks and distributions.
- [x] Complete Python 3.14 host/CI checks and explicit 3.13 deployment preservation.
- [x] Reject incomplete physical MAVLink writes; separate queued submissions.
- [x] Validate handoff acknowledgements and preserve concrete arming refusals.
- [x] Run full synthetic camera/tag/H264/storage-pressure/control workload;
  fix and retest actual TCP idle-byte accounting.
- [x] Repair reproduced false success after unexpected descent during hold,
  then rerun the full immutable integration matrix.
- [x] Finish local gates, isolated pinned SITL, review and restoration bundle;
  preserve all six final qualification failures.
- [x] Repeat passive Pi capture at `f5ff571` with exact serial-byte metrics,
  fresh disarmed proofs and a clear final ownership snapshot.
- [x] Obtain one complete installed-ROMFS file and independently verify its hash;
  retain the client's failed session-termination exit.
- [x] Prepare the separate Python 3.14 native candidate and exact matching build
  inputs; record resource/access blockers and preserve working Python 3.13.
- [x] Reject spoofed simulator-isolation markers using actual interfaces; refuse
  3.14 deployment before it can discard native bindings.
- [ ] Finish ARM64 native qualification and candidate cleanup after Pi recovery.
- [ ] Resolve the six simulator profile qualification failures before selection.

## Evidence boundaries

Host checks, simulator results and Pi observations are separate. The initial
five-case run overlapped live edits and is exploratory; the isolated `9de5fa1`
run passed all five in 274.35 seconds. Subsequent simulations use a Linux user/
network namespace with only loopback and no external routes. This prevents their
endpoints from reaching the physical FC, including through an SSH forward.

The Pi remains grounded/disarmed. No live arming, mode, takeoff, LAND, motor,
throttle, RC override, mission-start, servo or payload command is permitted.
No FC firmware/parameter writes or service switching is part of this task.

## Current Pi observations and blockers

- Read-only UART probe received selected-FC disarmed heartbeats (no transmitted
  MAVLink commands). The later sensor check repeated that guard before telemetry.
- Deployed application still uses individual console scripts, system-site Python
  3.13.5, and no installed `ai-drone-runtime.service`; local source is not deployed.
- Sensor check matched 4.7.1 / `dbe79216`, fresh range/flow and zero RC channels.
  The earlier checker failed the unchanged battery minimum: 14.17 V versus
  14.4 V. A later final-source checker passed at 15.732 V with no errors or
  warnings; the earlier failure remains retained.
- Standard uv-managed 3.14.7 installed separately at
  `/home/seb/ai-drone-candidates/20260921-py314/interpreters`, without executable
  links or system/application changes. Native compatibility remains unqualified.
- Three passive camera/video/native-tag/telemetry captures completed under 3.13;
  all found zero tags and commanded no servo pulses. The final `f5ff571` capture
  recorded 73 analyzed frames, 240 encoded H264 frames and 1,254 messages over
  10.056188 s. Exact serial totals were 77,991 RX / 1,075 TX bytes over 19.372824 s
  including startup; the only 25 outbound packets were telemetry-interval requests.
- Fresh selected-FC disarmed proofs bracketed the final capture: four heartbeats
  each, latest ages 0.840889 s before / 0.576810 s after. Final hardware/package
  ownership inspection was complete and clear. No service/environment switched.
- Full 35,238-byte installed `@ROMFS/hwdef.dat` matched reviewed SHA-256
  `d89b4db7acd2811284c420fb79f0750661f8dfca6865bedf1725a17dfac4babe` after separate
  retrieval/hash validation, although the client's terminate-session timed out
  and asserted. The failed exit remains recorded. Fresh disarmed proofs and
  clear ownership also bracketed this check. A reboot-separated repeat and full
  flash-byte proof remain distinct outstanding items.
- Physical hall survey, independent heading/height references, native 3.14
  compatibility and all physical flight/actuator qualification remain outstanding.

Completed software gates are not a flight-readiness report.

## Integrated checkpoint

Runtime source is frozen at `d86a955`; subsequent commits only consolidate
reports. Final host gates passed **1,868 tests on each of CPython 3.14.7 and
3.13.15**, zero skips, with 40 SITL deselected for separate execution. Formatting,
lint, typing, seven import contracts, dependency analysis, site, distributions,
installed CLI/report and real generated-camera codec tests passed on both.

The final `d86a955` matrix completed **34 passed / six failed / zero skipped**:
production baseline 5/5, Loiter profiles 8/8, dynamics 7/7 and workload 1/1;
vertical profiles 3/8 and sensor/datum/magnetic faults 10/11. Five 30-second
vertical holds exceeded 0.5 m horizontal clearance (1.394–1.458 m), and the
changing-field compass case had 117.938° estimated-yaw drift versus 1.762° true
motion. All profile trials disarmed. The required acceptance gate remains **not
green**; no threshold was relaxed. Original failed runs, the failed-hold repair,
`f5ff571` matrix and intermediate gate results remain archived.

The final-source disarmed Pi checker and three 3.13 passive recordings completed;
all real captures found zero tags. One complete ROMFS file exactly matched the
reviewed board definition after independent retrieval, despite a client cleanup
assertion. Native 3.14 libcamera compilation then exhausted swap and SSH became
unavailable. Targeted compiler cleanup could not be confirmed. Re-establish
access, inspect the candidate process/resources and obtain fresh read-only
disarmed proof before further hardware checks. No physical flight, actuator
command, FC parameter/firmware write or live service switch was performed.

The separate candidate, exact source/header/build kits, host native API checks,
reviewed runtime archive and paired source/environment rollback preparation are
retained. ARM64 native acquisition and actual service migration remain blocked.
Antigravity's external worktree drafts were left untouched and independently
archived alongside committed history.

See the [implementation report](refactor-implementation-report.md),
[Pi report](pi-disarmed-refactor-checks.md),
[native candidate report](python-314-candidate.md),
[fault matrix](fault-matrix.md) and
[remaining hardware checklist](hardware-qualification-checklist.md).
