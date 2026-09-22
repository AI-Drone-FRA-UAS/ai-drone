# Paused checkpoint

Historical checkpoint: the user subsequently resumed work. See the
[September 22 completion report](../2026-09-22/refactor-completion.md) for the
resumed implementation and validation. The original pause record follows.

Paused at the user's explicit request on 2026-09-21. Do not resume until asked.
Subagents stopped. No live service switch, flight or actuator command occurred.

## Current implementation

- Branch `refactor/concise-runtime`, latest runtime `12ccd2d`.
- `ff40754`: abort an owned flight on a fresh selected-FC magnetic yaw-reset
  diagnostic, preserving human handoff. No tests added.
- `12ccd2d`: remove duplicate recording cleanup; net 45 production lines and
  19 test lines removed, retaining the existing assertions. No tests added.
- Expanded plan and readiness document still match `codex/refactor` / `84855d0`.
  Antigravity's 33 external dirty entries remain untouched.

## Completed validation

- `ff40754`: 1,868 existing host tests passed on each of Python 3.13/3.14.
  Initial sandbox socket denials were retained, then rerun outside the sandbox.
- Pinned isolated SITL: **35 passed, five failed, zero skipped**. The magnetic
  reset case now fails the mission explicitly and lands/disarms. Five 30-second
  altitude-only horizontal-clearance cases remain unqualified; no limits changed.
- `12ccd2d`: 274 affected cases passed on each Python, three packaging cases
  passed; static checks passed. Two affected isolated SITL cases passed.
- Final review bundle: `/tmp/ai-drone-runtime-12ccd2d-review.tar.gz`.
  Detailed host evidence: `/tmp/refactor-12ccd2d-host-verification.md`.
- Current production Python count: 21,800 versus 19,271 baseline (+2,529).
  The relocated 517-line HTML template is additional to that count.

## Pi state and new evidence

- Access recovered; no old compiler remained. Initial UART probe got no
  heartbeats; repeated probe established fresh disarmed state and clear owners.
- Read-only restoration capture completed: 1,187 parameters, mission count one,
  fence/rally zero, complete 35,238-byte ROMFS with matching hash and clean FTP
  completion. Private source/environment and system/config archives retained at
  `/home/seb/ai-drone-candidates/20260921-restoration-resumed` and locally at
  `/tmp/ai-drone-private-restoration-20260921/pi-restoration.tar.gz`.
  These contain private configuration; do not publish their contents.
- Latest-source Python 3.13 passive recording passed: 10.039 seconds, 43 camera
  frames, 881 messages, zero tags/servo commands. Final fresh disarmed proof passed.
- Separate 3.14 candidate native stack installed using nine local wheels plus
  binary dependencies. System Python, working `.venv` and services unchanged.
  Candidate root: `/home/seb/ai-drone-candidates/20260921-py314`.
  Its pre-install paired backup is `before-native-source-environment.tar`.
- All native imports succeeded. The combined smoke command **failed** because its
  synthetic tag generator assumes exported `image_u8_destroy`, absent from this
  Pi's libapriltag. This is a smoke-helper issue to investigate; synthetic native
  tag/API validation and actual 3.14 camera acquisition remain incomplete.
  Log: `/tmp/pi-candidate-native-smoke-20260921.log`.
- Last completed read-only FC proof: four disarmed heartbeats, latest age
  0.126839 s, battery 15.241 V; final ownership snapshot clear.

## Resume next

1. Refresh fresh read-only disarmed state before hardware checks.
2. Repair the temporary synthetic tag input generator without assuming that
   unavailable symbol; preserve the failed result and rerun the relevant check.
3. Review the not-yet-run `/tmp/pi-passive314-shared-check.py` completely before
   any passive 3.14 foreground-runtime/camera execution. It has not been staged
   or run. No physical commands beyond passive request allowlists are permitted.
4. Capture candidate native manifest and offline replay/rollback evidence.
   `/tmp/rpi-cross314/PRODUCTION_NATIVE_PROMOTION_CONTRACT.md` is a review-only
   contract; native production payload support is **not implemented** and the
   production 3.14 deployment refusal remains intact.
5. Consolidate updated reports/checklists and preserved artifacts; existing final
   reports still describe the previous checkpoint. No physical qualification is
   authorized. Keep the five vertical-profile failures explicit.
