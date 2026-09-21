# Refactor implementation progress

Integration branch: `refactor/concise-runtime`, in `ai-drone`. The existing
`ai-drone-codex` and `ai-drone-agy` worktrees are read-only inputs. Plan revisions
`2eefca0` and `84855d0` are integrated without replacing preserved drafts.

## Completed and in progress

- [x] Reconcile Antigravity `08020d1` / `dd6a6c1`; preserve their commits.
- [x] Correct LAND/disarm retry, takeover-before-write and fresh disarm proof.
- [x] Run five pinned production-path cases against flight fixes `9de5fa1`.
- [x] Extract immutable flight observations (`babc8ae`); its SITL gate is running.
- [ ] Complete phase/command, limits and polling extraction, with separate gates.
- [ ] Add autonomous altitude hold and its production-path acceptance cases.
- [ ] Test inertial-yaw feasibility, magnetic faults, heading/gyro drift and duration.
- [x] Reconcile runtime shutdown exclusion and typed saved network policy.
- [x] Share typed runtime-status parsing with distinct authorization policies.
- [x] Provision a single per-boot servo lock namespace independent of service restart.
- [ ] Complete typed power and transfer boundaries, network job and deployment states.
- [x] Restore tag diagnostic labels and honest interrupted command accounting.
- [x] Extract tag decisions, synchronize capture snapshots, freeze calibration.
- [x] Compose recording teardown with explicit dependency order and independent cleanup.
- [ ] Complete immutable capture snapshots, manifest and operation/parser records.
- [x] Share strict JSON, range/freshness, remote command and system helpers.
- [x] Inject one command configuration without environment mutation; register commands.
- [ ] Complete checker identity/diagnostics, report signal table and hall timeline.
- [ ] Verify packaged HTML resource, declarative firmware checks and distributions.
- [ ] Complete Python 3.14 host/CI checks and explicit 3.13 deployment preservation.
- [ ] Finish all local gates, isolated pinned SITL, review and restoration bundle.

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
  It failed the unchanged minimum battery check: 14.17 V versus 14.4 V.
- Standard uv-managed 3.14.7 installed separately at
  `/home/seb/ai-drone-candidates/20260921-py314/interpreters`, without executable
  links or system/application changes. Native compatibility remains unqualified.
- Physical hall survey, independent heading/height references, installed firmware
  feature proof and all physical flight/actuator qualification remain outstanding.

This is a working checklist, not a completion or flight-readiness report.
