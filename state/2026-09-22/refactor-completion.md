# Refactor completion and qualification — 2026-09-22

The authorized software work is implemented on `refactor/concise-runtime`.
Aircraft qualification remains incomplete: five simulator vertical-clearance
cases fail, the physical power arrangement needs verification, and no live
flight, actuator test or service migration has been performed. This report
supersedes the validation checkpoint in the
[September 21 implementation report](../2026-09-21/refactor-implementation-report.md),
which retains the earlier implementation history.

## Implementation and reconciliation

The expanded `codex/refactor` plan, including `84855d0`, and its Python-readiness
document remain unchanged. Antigravity commits `08020d1` and `dd6a6c1` are
ancestors of the integration branch. Their overlap was reconciled before new
work; the separate Antigravity worktree and its uncommitted drafts were preserved.
The [finding register](../2026-09-21/findings-reconciliation.md) and
[progress checklist](../2026-09-21/refactor-progress.md) map the earlier stages.

The final Python runtime source is `f520dd07cf0f7877c0d6400b3b5480e274cf7eb6`.
Focused changes since the previous report:

| Commit | Result |
| --- | --- |
| `ff40754` | Abort an owned flight on a fresh selected-FC magnetic yaw-reset diagnostic, retaining LAND cleanup and human handoff. This does not detect arbitrary heading error. |
| `12ccd2d` | Remove duplicate recording cleanup: 45 production lines and 19 test lines removed, preserving the existing lifecycle assertions. |
| `3a64313` | Add explicit offline native-payload deployment, exact interpreter/library/wheel identities, positive native/passive qualification, local environment isolation and paired source/environment rollback retention. Generic Python 3.14 promotion remains refused. |
| `9b4336d` | End the measured transport interval before intentional descriptor-close cancellation of a receive. Earlier errors remain visible; independent close and lock release are preserved. |
| `44d0818` | Patch the separate native AprilTag binding to release the GIL during detection, reject overlapping calls on one detector, preserve Python signal handling and clean native results on error. |

| `f520dd0` | Join telemetry and close its subscription before slow camera/artifact teardown, preventing shutdown-only queue overflow. Actuation closes first; independent cleanup remains intact. |

No new permanent tests were added in these resumed commits. Existing tests were
used; native deployment changes one existing fixture. Independent agents handled
native artifacts, deployment implementation and readiness review; only the
coordinator accessed the Pi and managed simulator resources.

## Host checks actually run

All Python execution and environments used uv. Frozen final source `f520dd0` passed:

| Host interpreter | Existing tests | Skipped | SITL deselected | Test time |
| --- | --- | --- | --- | --- |
| CPython 3.14.7, standard GIL | 1,868 passed | 0 | 40 | 21.83 s |
| CPython 3.13.15 | 1,868 passed | 0 | 40 | 21.47 s |

Both passed formatting, Ruff, C901 with `--ignore-noqa`, scoped BLE/ARG, ty,
seven import contracts, dependency checks and the 13-page site build. Wheel/sdist,
isolated installed CLI/report/template and deployment-bundle checks passed.
CI configuration supports 3.14 and retains earlier versions; remote CI was not run.
The lock SHA-256 is
`7a9a9fe5dc982454c54594dd239220fca08c1e41b6c88fe2570dfe875d41b120`.

Native deployment additionally passed 86 existing deployment cases on each
Python, isolated offline install/launcher/environment-isolation exercises,
actual paired disk rollback for simulated import failure/interruption, with mocked
Pi identity/ownership, and successful-backup retention.
Missing/modified wheels and wrong installed library identity were rejected.
All 30 target-compatible wheels installed in a host foreign-target replay and survived a
generic exact sync; that replay did not execute ARM code.

## Pinned simulator evidence

The final full matrix at `9b4336d` completed **35 passed, five failed, zero skipped**:

| Group | Passed / executed | Elapsed |
| --- | --- | --- |
| Original production baseline | 5 / 5 | 273.67 s |
| Altitude profiles | 3 / 8 | 552.89 s |
| Loiter profiles | 8 / 8 | 561.30 s |
| Faults | 11 / 11 | 552.97 s |
| Dynamics | 7 / 7 | 369.56 s |
| Recording workload | 1 / 1 | 62.69 s |

After `f520dd0`, both affected simulator cases passed again: shared recording
(56.33 s; four unrelated baseline cases deselected) and camera/storage workload
(61.42 s). Flight policy was unchanged. No case was skipped.

Earlier `ff40754` full-matrix and `12ccd2d` recording-delta results remain in the
evidence history. The subsequent native-binding patch is qualified separately;
it changes no application Python or simulator source.

Every run used pinned ArduPilot
`dbe792162d06cab66c3475fd5556bf7a120f119e`, SITL ELF SHA-256
`cd2a0ce7319ddbdbaa23f452889f858b351340a7508e20d003222a3cd518c961`.
The runner and tests verified independent network namespaces containing only
loopback, with no route to the real FC. No flight case was skipped or converted
to an expected failure; thresholds were retained.

The five failures are 30-second vertical-hold horizontal clearance:
**1.378–1.458 m during hold versus 0.5 m**, and 1.600–1.697 m over the armed
sequence. The passing 10-second holds moved 0.472–0.493 m during hold and
0.710–0.731 m overall. Altitude hold controls height, not horizontal position;
its provisional 10-second cap does not establish confined-hall clearance.
All profile runs disarmed. The eight Loiter cases passed, with at most 0.177 m
whole-sequence displacement in this simulation.

The changing magnetic-field regression now reports mission failure and lands/
disarms on the FC's reported yaw reset, instead of reporting success despite a
large estimator error. Inertial-yaw Loiter remains an isolated simulator
experiment. No aircraft estimator parameters or firmware were changed, and
healthy-looking attitude/EKF flags are not independent heading truth.

## Pi checks and candidate files

The working `/home/seb/ai-drone/.venv` remains CPython 3.13.5. System Python,
packages, live source and service selection were preserved. All access used
`ssh seb@seb-is-pm`. Each hardware check required the existing read-only fresh
disarmed path; failed guards prevented the dependent check from starting.

The separate root is `/home/seb/ai-drone-candidates/20260921-py314`:

- `interpreters/cpython-3.14.7-linux-aarch64-gnu`: standard managed interpreter;
  executable and libpython match the retained official archive.
- `source-fbbe690/.venv`: earlier candidate, retained with its paired
  `before-native-source-environment.tar` backup.
- `payload-3a64313/.venv`: new candidate installed offline from all 30 wheels.
  Target ABI, installed native-library hashes and Debian package identities
  passed. A subsequent generic locked sync retained all 30 packages, and the
  source-pinned CLI help worked from outside the project directory.
- `payload-9b4336d-gil1/.venv`: subsequent isolated candidate with the transport
  fix and versioned AprilTag binding. Its full offline install, default exact
  sync, CLI help and native/tag smoke passed too. Earlier candidates were retained.
- `native-review-inputs-r5`: reviewed native/tag helpers and an inert,
  hash-checked ID17 fixture; previous helper versions remain preserved.
- `offline-install-final-3a64313.json` and `native-smoke-final-3a64313.json`:
  actual Pi install and native import/API evidence.
- `pi-passive314-shared-final.py`: inspected foreground-runtime/camera supervisor,
  with no service operations and an outgoing telemetry-request allowlist.
  Later revisions record the runtime's own exit, signal only its verified child
  PID, and preserve normal Python bytecode caching in the candidate.

Full native imports, GPIO backend compatibility without GPIO claims, native
ID17 detection, project-adapter corners and empty-image handling passed in
15.335 s. The full smoke began with five disarmed heartbeats (latest age 0.003 s)
and ended with four (0.090 s), followed by clear ownership. Picamera2 import
performs read-only video capability queries; it did not acquire a camera in
this smoke check.

The first shared-runtime camera capture at `3a64313` recorded 10.013 s, 80
analysis frames, 263 independently decoded H.264 frames and 1,596 telemetry
messages. Transport measured 118,609 RX / 1,100 TX bytes over 35.863 s, with
25 telemetry requests and zero heartbeat/setpoint writes, errors or overflows.
However, its supervisor returned failure because the signalled uv process
returned 143; the runtime's own return had not been separately recorded. This
attempt is not counted as a passing whole check. A host diagnostic did not
reproduce 143, so its exact cause is not asserted.

A second `3a64313` capture recorded only 883 messages and failed on a 512-entry
subscriber overflow; measured delivery lag reached 4.453 s. Its corrected
supervisor confirmed runtime and uv exit zero. Inspection established that the
native AprilTag binding held the GIL through detection and also replaced the
process's SIGINT handler. The versioned patch fixes both without increasing
queues or changing freshness limits. Host checks on both Python versions covered
real ID17 recognition, malformed-input recovery, overlapping-call rejection,
Python thread progress and interruption cleanup. The original binding stalled
another Python thread for 1.454 s; the patched observations had gaps below 5.5 ms.
These host timings are distinct from Pi measurements.

The first `9b4336d`/`gil1` attempt stopped on the freshness guard while importing
the recorder, before camera acquisition, subscription or any telemetry request.
Runtime RX remained error-free and its exit was zero. The supervisor had forced
bytecode caching off; subsequent checks restore normal candidate-only caching
and record the exact guard-failure status. The original failure remains retained.
The next attempt reached camera acquisition but stopped when status publication
on the SD card aged 1.771 s, plus a 0.298 s heartbeat age, exceeding the unchanged
2 s effective-age guard. At that instant errors and overflows were zero. During
subsequent camera teardown its still-open telemetry subscription overflowed
(512 entries, 513 discarded); `f520dd0` closes that drained subscription earlier.
The final helper uses a private verified `/run/user/1000` tmpfs directory for
status/socket, matching production's volatile-status arrangement, and retains
video/log artifacts on SD. Earlier failed captures remain evidence.
The first `f520dd0` capture then exposed a startup overflow (512 entries/513
discarded) before the recording epoch. A diagnostic repeat with temporary
half-second stack sampling completed: 10.005 s, 191 analyzed frames, 297
independently decoded H.264 frames and 1,599 telemetry messages. Runtime and uv
exited zero, with 138,763 RX / 1,100 TX bytes over 26.504 s and no errors or
overflows. Across 62 status samples, peak temperature was 77.364°C, minimum
available memory 98,464 KiB and maximum effective heartbeat age 1.166 s.
The trace tied a startup queue buildup to OpenCV's native module initialization;
the successful repeat does not erase the prior overflow. The AprilTag constructor
was not blamed: 30 host calls took at most 0.813 ms. Final startup correction and
validation are recorded below when complete.
The preserved Pi Python 3.13 Debian binding was not patched or replaced.

Earlier failed attempts remain evidence: a missing-heartbeat guard aborted
before hardware access; temporary helpers assumed an unavailable AprilTag
generator symbol and later a list result where the native binding returns a
tuple. Corrected inert-input helpers passed; production detection code did not
require a change. An SSH transfer timeout was retried after connectivity returned.

Previous permitted Python 3.13 passive capture at `12ccd2d` completed 10.039 s,
43 camera frames and 881 telemetry messages, with zero tags/servo commands and
fresh final disarmed proof. Separate restoration evidence contains 1,187 FC
parameters, one mission item, empty fence/rally lists and complete 35,238-byte
ROMFS retrieval. Private Pi source/environment and system/network configuration
archives remain separate from public evidence; they contain credentials and
are not a full FC flash backup.

## Deployment and rollback prepared for review

The normal review bundle is `ai-drone-runtime-f520dd0-review.tar.gz`, 284,245 bytes,
109 manifest paths, SHA-256
`d31e361121e122eb238b08043b200804a96abbd100aa7d7aa940eb328f6fa774`.
It excludes private configuration and environments. The native preview has 30
local wheels and refuses deployment while positive qualification is absent.

For a separately approved migration, review the exact payload and its hashed
evidence, verify fresh disarmed state and idle ownership, preserve the paired
source/environment backup, then use the explicit native-payload route. The
installer checks the retained interpreter/libpython and target native libraries
before maintenance, installs offline without builds, and requires native imports
and post-restart health. It retains the successful native update's backup.

On failed installation/import, restore source and `.venv` together. If restoration
fails, keep the service stopped and preserve the rollback directory. After a
successful install with failed health, retain the backup for recovery. Actual
live deployment/restart/rollback was not exercised; the isolated transaction
checks do not qualify those service transitions on this Pi. See
[runtime deployment contract](../../docs/PYTHON_RUNTIME.md).

## Size and remaining qualification

Tracked production Python grew from 19,271 to 22,275 lines: **+3,004**.
Including the relocated 517-line report template, comparable production grew
by **3,521 lines**. Tests grew from 19,533 to 25,182 over the complete refactor;
no new permanent cases were added in the resumed changes above. The new native
deployment route alone adds 465 production lines. This work expanded behavior,
failure handling, diagnostics and deployment support; it did not deliver an
overall code reduction. No further reduction estimate is claimed without an
identified, reviewed deletion.
The separate native patch is 133 lines of diff (30 C lines added, 34 removed in
the external binding), plus its 42-line build/readiness note; it is not included
in the production Python count.

Remaining requirements are listed in the
[hardware qualification checklist](../2026-09-21/hardware-qualification-checklist.md):
stable intended FC power and UART, measured hall magnetic/flow/range behavior,
independent height/heading references and disagreement policy, sustained physical
load/timing, reviewed live deployment and rollback, then separately authorized
supervised takeoff–LAND followed by short hold and Loiter qualification.

The owner reports Pi external power without a separate FC supply, expecting the
Pi to power the FC. Documented UART wiring lists TX/RX/ground only. Occasional
fresh heartbeats do not establish the actual power path or suitable flight
power. This must be physically verified; no electrical diagnosis is inferred
from SSH reachability.

**Physical flight and all actuator testing remain explicitly deferred.** No live
arming, takeoff, landing, mode, motor, throttle, RC override, mission-start,
servo/payload command, FC parameter write or flash was performed.
