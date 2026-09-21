# Refactor implementation and validation — 2026-09-21

Integration branch: `refactor/concise-runtime`, workspace
`/home/abaris/drone/ai-drone`. The governing expanded plan from `codex/refactor`,
including `84855d0`, is integrated and remains text-identical to that branch's
`refactor.md` and Python-readiness document. Existing uncommitted work was
preserved and completed in focused commits. Antigravity's `08020d1` and
`dd6a6c1` were retained before reconciling their safety/ownership behavior.

The software implementation is distinct from aircraft qualification. Physical
flight, actuator tests, FC flashing/parameter writes and live service deployment
were not performed. Permitted passive telemetry requests were sent.
All armed/mode/control tests used mocks or network-isolated local SITL. Read the
[hardware checklist](hardware-qualification-checklist.md) before interpreting
any passing software test as flight readiness.

## Implemented work

| Area | Implemented result and representative commits |
| --- | --- |
| Antigravity reconciliation | Corrected fresh disarm barriers, bounded retry through RX/write failures, takeover-before-write, stable servo exclusion and runtime shutdown ownership: `9de5fa1`, `8cd15e0`, `799f1f8`, `7907ff5` |
| Functional flight core | Immutable observations, explicit phases/authority, one autonomous command boundary, pure limits and exact parameter/firmware contracts: `babc8ae`, `5b68e36`, `871d7f5`, `f24f0f9` |
| Flight evidence and cleanup | Invalid/replayed observation revocation, pre-arm floor datum plus 0.05 m reserve, persistent LAND latch and no mission reentry after failure, bounded all-command/gap audit retained through cleanup: `4017686`, `2ced490`, `f3183f6`, `7cf86ba`, `94f02e2` |
| Autonomous operations | Dedicated takeoff–vertical-hold–LAND command, existing hover remains takeoff–Loiter–LAND; ordinary vertical hold capped provisionally at 10 s after measured clearance failure, Loiter capped at 30 s: `0824a3e`, `a0fce46`, `2709958` |
| Inertial-yaw experiment | Exact separate simulator profile, literal loopback and isolation opt-in, normal remaining checks, no automatic source switching, 120 s experimental initialization-to-completion budget with 30 s LAND reserve: `d0ceb8d` |
| Sensor/reporting boundaries | Shared strict JSON/freshness/range validation, typed runtime/power/transfer records, full checker source/type/ID/orientation/parameter identity and pure diagnostics: `5af3dee`, `813fce3`, `b192389`, `28bc86b`, `d9986d1`, `0c6beb6` |
| Recording and vision | Pure tag decisions, construction-time frozen calibration, locked immutable snapshots, dependency-ordered independent teardown, explicit manifest/operation records, honest partial outcomes: `9afdded`, `7c42a85`, `3c4934e`, `44602cf`, `487738b`, `c7b72d5`, `0988628` |
| Review | Scalar signal table preserves historical CSV order/sentinels/units; packaged HTML resource; raw/scaled magnetic and reported estimator/event timeline without invented heading truth: `4e6d9c5`, `cf70d66`, `6239e67` |
| Runtime and deployment | Explicit outstanding network jobs/maintenance, typed power observations, paired source/interpreter rollback obligations, reviewed firmware inputs in deployment bundle: `e4dd2b3`, `5fae80b`, `5eaca15` |
| Shared mechanics | Quoted SSH/uv construction, signal restoration, command settings injection/dispatcher, service state and dry-run execution reuse; approved obsolete interfaces removed while recovery adapters remain: `06b93b1`, `8833802`, `405c49a`, `ece97b5`, `dabaa51` |
| Python/tooling/packages | Standard 3.14 development and CI, 3.13 retained, native import/deployment gates, mandatory GPIO/OpenCV host coverage, seven import contracts and installed wheel/sdist/report validation: `fbbe690`, `059f938`, `00effdc`, `45746fe`, `3bdfaec` |
| Transport measurements | Actual directional transport returns separated from encoded/decoded counts; gaps, delivery lag/ages, explicitly uncertain sequence gaps and overflow/discard/backlog counters, independent metrics cleanup: `1b89818`, `0a235dd`, `6df5903` |
| Physical command delivery | Reject failed/short/ambiguous serial, TCP, TCP-listener and UDP writes before encoder success accounting; distinguish Unix queued submission from physical write in command/gap audits: `ed73b7e`, `0a640b2`. Neither means an FC acknowledgement. Serial writes now have a 50 ms maximum timeout, preserving tighter settings; actual Pi timing remains unqualified |
| Arming diagnostics | Preserve fresh selected-FC warning/error Arm/PreArm messages after the arm write, with bounded audit history and concrete refusal reasons: `e568ff0` |
| Failed holds and handoff | Require positive handoff acknowledgement (`13312d2`); reject post-target altitude loss or unexpected disarm, retain original receipt boundary/target and session-first failure, preserve LAND cleanup (`ab11d47`) |
| Final boundary review | Require actual loopback-only interfaces as well as explicit simulator opt-in before enabling experimental flight (`3664e43`); refuse deployment of selected 3.14 environments before destructive rebuild/sync until native-artifact promotion is defined (`0cc15d9`, `d86a955`) |
| Simulator acceptance | Independent truth traces, all-compass changing anomalies, all-gyro bias, startup headings, 10/30 s operations, loss/identity/datum cases and isolated namespaces: `56f6f92`, `5c610c0`, `94c6a4b`, `6b4ad21`, `0c66bd1` |

Parallel agents owned flight, recording/review/checker and runtime/Python/transfer
files. Only the coordinator accessed the Pi and managed simulator instances.
A complete focused commit inventory is included in the evidence bundle.
The [F01–F19 reconciliation](findings-reconciliation.md) maps every corrected
finding to its implementation decision, commit and named regression, including
the deliberately retained F10 configuration and F18 fail-closed lease policies.

Comparable tracked Python-file counts (application package only, excluding
scripts/docs) are 64 files / 16,976 lines at `ab00860` and 76 / 19,760 at
`d86a955`; tests grew from 46 / 19,533 to 69 / 25,200. This refactor adds capabilities and explicit fault
contracts; it does not meet or claim an overall line-reduction forecast.

## Host validation actually executed

The final immutable `d86a955` source archive used separate uv environments and the same
locked dependency set (`dev`, `docs`, `raspi`).

| Runtime | Tests | Skips | SITL deselected |
| --- | --- | --- | --- |
| CPython 3.14.7, standard GIL | 1,868 passed in 24.83 s | 0 | 40 |
| CPython 3.13.15 | 1,868 passed in 24.50 s | 0 | 40 |

On both versions, all required formatting, Ruff, C901 with `--ignore-noqa`, ty,
import-linter, deptry and site gates passed. The additional scoped BLE/ARG gate
passed. Seven import contracts were kept; site generation produced 13 pages.
Both full suites included
building wheel/sdist, noneditable installation outside the checkout, installed
CLI/report rendering, template escaping/assets and deployment-bundle checks.
The old optional OpenCV/GPIO skips were closed by installing the required groups.

These suites include handoff acknowledgement validation, failed-hold supervision,
the TCP idle-counter repair, actual-interface experiment isolation, preservation
of selected 3.14 native environments, and the real OpenCV/H264/ffprobe generated-camera
host tests. CI explicitly installs ffmpeg for these required cases; absence is a
failure, not a skip. The earlier `bfc6c99` dual 1,732-test gate, `0a640b2` dual
1,801-test gate and focused
intermediate runs, including the dual 1,852-test `f5ff571` gate, remain in the evidence history. No remote CI workflow was run;
the CI version matrix and commands were implemented and validated locally.

Required command form (all Python work used uv):

```sh
UV_CACHE_DIR=/tmp/uv-cache uv run --locked --group dev --group docs --group raspi ruff format --check .
UV_CACHE_DIR=/tmp/uv-cache uv run --locked --group dev --group docs --group raspi ruff check .
UV_CACHE_DIR=/tmp/uv-cache uv run --locked --group dev --group docs --group raspi ruff check . --select C901 --ignore-noqa
UV_CACHE_DIR=/tmp/uv-cache uv run --locked --group dev --group docs --group raspi ty check .
UV_CACHE_DIR=/tmp/uv-cache uv run --locked --group dev --group docs --group raspi lint-imports
UV_CACHE_DIR=/tmp/uv-cache uv run --locked --group dev --group docs --group raspi deptry .
UV_CACHE_DIR=/tmp/uv-cache uv run --locked --group dev --group docs --group raspi pytest -q -m 'not sitl'
UV_CACHE_DIR=/tmp/uv-cache uv run --locked --group dev --group docs --group raspi python site/build.py
```

## Simulator provenance and results

Every scored simulator run used the pinned source
`dbe792162d06cab66c3475fd5556bf7a120f119e`, executable
`/home/abaris/drone/ardupilot/build/sitl/bin/arducopter` and standard CPython
3.14.6 for the later integration runs. The official linked-feature extractor
confirmed EKF3, optical-flow fusion, MAVLink flow/range and GUIDED_NOGPS.
This is the simulator target, not the installed Flywoo board artifact.

The project's saved FlywooF745 4.7.1 ELF/BIN/APJ and expanded board definition
were also checked by the full linked-artifact verifier, using the official
pinned source's feature extractor. That gate passed at `594b314`: board 1027,
image 865,632 bytes against 950,272 bytes (84,640 bytes free), exact reviewed
hashes/features and ELF/BIN/APJ consistency. The verifier's uv child now uses
`--no-project` to avoid mutating the external ArduPilot environment. Saved board
artifact verification is distinct from fresh installed-ROMFS proof. Nothing was
flashed, rebuilt or written to the FC.

| Input | SHA-256 |
| --- | --- |
| SITL ELF | `cd2a0ce7319ddbdbaa23f452889f858b351340a7508e20d003222a3cd518c961` |
| Final uv.lock | `7a9a9fe5dc982454c54594dd239220fca08c1e41b6c88fe2570dfe875d41b120` |
| tests/sitl/copter.parm | `bab5742a57c1fc4cb63a91c76e64e9ccece2004444e73fcf591a4a3adaae7130` |
| SITL feature overlay | `b96803ace0f2476bdbaa2c01757f4e0ca657c9200ef5a7345eab74ef658b4802` |

`scripts/run_sitl.py` creates a Linux user/network namespace and brings up only
loopback. The tests independently require the isolation marker and interface
set `{lo}`. No external interface or route exists, including to SSH forwards.
Busy ports fail instead of skipping. Parallel final groups ran in distinct
namespaces with separate temporary directories; they shared only a read-only
verified executable and immutable source/environment.

The original five-case production-path gates passed at `9de5fa1` (274.35 s),
`babc8ae` (274.32 s), `56f6f92` (272.84 s), and `871d7f5` (274.45 s).
The initial baseline run that overlapped edits is retained as exploratory only.

Initial truth gate `5c610c0`: three passed, one failed. Ten-second vertical hold
and 10/30-second Loiter passed. Thirty-second vertical hold kept height error
about 0.01 m but displaced 1.442 m horizontally against the frozen 0.5 m clearance
criterion. Its full armed sequence displaced 1.659 m. These failures were not
converted to skips/xfails or hidden by larger limits. They caused the ordinary
vertical-hold cap to be narrowed to 10 s, with longer trials isolated-SITL-only.
Even the passing 10 s trial displaced about 0.72 m over its full armed sequence;
it does not establish confined-hall clearance.

Initial inertial-yaw feasibility at `94c6a4b`: two 10 s Loiter cases passed,
normal and changing magnetic field plus gyro bias. Configured experimental time
budgets are hard limits, not claims that every permitted second is qualified.
Failure of vertical-hold XY clearance alone does not establish failed inertial
yaw; the conditional camera-navigation route is not triggered by that result.

Fault run `0c66bd1`: seven passed, three failed. All six actual sensor-loss/
discontinuity cases landed and disarmed. The failures exposed an upstream-vs-FC
range identity mistake in the harness, stale vector-parameter broadcast readback,
and a refusal-message string assertion. Each original failure is retained, with
its exact explanation in [the fault matrix](fault-matrix.md).

The first final baseline attempt at `bb49adc` passed the two normal cases but
rejected three test requests of 60/120 s at the new 30 s CLI cap. `8729364` changes
only those requested durations; every original failsafe/handoff assertion remains.

The corrected baseline at `8729364` passed all five cases (273.12 s). The
`bb49adc` profile run completed 11 passed / 5 failed (1,120.24 s): all five
30-second vertical-hold trials failed unchanged XY clearance bounds, while the
eight Loiter and three 10-second vertical trials passed. The `bb49adc` fault
run passed nine / failed one (507.03 s): the FC refused the magnetic pre-arm
fault correctly but the CLI lost its concrete reason. `e568ff0` repairs that
diagnostic; the failed run remains part of the record.

Pre-final common runtime `0a640b2` results (zero skips in every group):

| Group | Passed | Failed | Time |
| --- | --- | --- | --- |
| Original production baseline | 5 | 0 | 272.57 s |
| Independent-truth profiles | 11 | 5 | 1,115.80 s |
| Sensor, datum and magnetic faults | 10 | 1 | 584.17 s |

The six qualification failures remain failures. Five are 30-second vertical
hold XY clearance (1.394–1.463 m versus 0.5 m). The sixth is the compass
baseline under the injected changing field: its CLI completed and landed, but
offset-corrected **yaw-estimate drift reached 117.968°** against truth while
actual heading motion was only 0.921°. Healthy flags and small XY displacement
did not establish correct yaw. That compass profile is not qualified for this
disturbance. There is no invented onboard heading-truth check or automatic
airborne profile switch. The separate inertial-yaw Loiter trials passed their
tested engineering bounds and remain simulator-only.

Independent tlog/truth review confirms the compass failure is an FC reset, not
angle wrapping or clock mismatch: an estimated-yaw step of 117.579° coincided
with less than 0.060° true motion; nearest truth/attitude timestamps differed by
at most 0.000144 s. The FC reported both IMUs' ground magnetic anomaly and yaw
realignment, while all 499 EKF status messages retained flags 367. The report
timeline preserves those reset messages; it does not derive heading truth from
them or treat the healthy flags as proof of correct heading.

Across all eight `0a640b2` Loiter profile runs, maximum height error was 0.010052 m,
maximum height 0.529969 m, whole-armed XY displacement 0.176332 m, hold yaw
estimate drift 5.661994° and whole-armed drift 7.940523°. Sampled hold XY was
zero at the recorded latitude/longitude quantization, not a claim of zero
physical motion. The new inertial profile has three independently initialized
normal and magnetic starts for each operation (headings 0/120/240°), using the
documented deterministic simulator seed/default and 0.003 rad/s gyro-Z bias
on all three simulated IMUs. Its 30-second holds were tested within the full
elapsed initialization/landing budget, without qualifying the entire 120 s cap.
The three passing 10-second vertical holds still moved 0.724–0.732 m over the
whole armed sequence; 30-second vertical trials reached 1.688 m. A short hold
criterion cannot establish confined-hall clearance for takeoff and landing.

All six `0a640b2` sensor-loss/discontinuity cases confirmed LAND and disarm; the
largest observed loss-to-LAND delay was 1.506 s. Changing the native range ID is
a contract observation, not a loss injection: this FC republishes configured
instance ID 0 and completes the requested mission. Both pad-offset cases and
the concrete magnetic pre-arm refusal now passed. No normal arming checks or
acceptance thresholds were weakened.

For the `0a640b2` combined-aiding-loss case, EKF flags changed 367→103 about 46 ms
before the first LAND heartbeat. Both DataFlash EKF cores lost relative/absolute
position status about 70 ms before LAND, and all 77 subsequent samples per core
lacked those bits and dead-reckoning status. The pinned `position_ok()` and
`ModeLand::init()` branches therefore support selection of position-unavailable
LAND (`nogps_run`); this is source-correlated inference, not a directly logged
internal control-position flag. It moved 0.149 m after aiding loss, reached LAND
after 1.108 s and disarmed after 8.404 s; maximum height was 0.519959 m. This
establishes that synthetic fault's response, not precision landing without aiding.

Additional dynamics and full synthetic-camera workload cases supplied the
following failure discoveries and repairs before the final combined run.

The added seven-case dynamics run at `57946ac` passed six and failed one
(409.82 s). No liftoff and tether-stalled climb retained LAND obligations at the
existing 15 s deadline, with no direct disarm; missing aiding, rejected Loiter,
unexpected mode and newly arriving low-throttle RC also landed and disarmed.
The all-motor thrust-loss case exposed a real false success: impact followed in
0.350 s, while the FC remained armed in Loiter for another 27.7 s and the CLI
eventually returned success. Its original trajectory and logs remain preserved;
the phase-specific failed-hold repair and rerun are tracked below.

`ab11d47` adds a phase-owned floor target and original target-reaching receipt.
HoldingAltitude, AwaitingLoiter and Loitering compare fresh post-target downward
range and aligned local altitude independently against target minus 0.10 m.
Pre-target samples cannot falsely signal a lost hold. The existing conservative
upper ceiling remains unchanged; Takeoff, Landing and Human do not apply this
lower bound. A fresh unexpected disarm also fails the hold. The original reason
survives LAND failures in the controller-session audit. This is an explicit
engineering abort policy, not evidence of recoverability after motor loss;
actual noise/dynamics qualification remains in the hardware checklist.

The unchanged targeted descent case at checkpoint `f5ff571` passed in 51.12 s.
It returned failure with the measured range (0.33 m), declared lower bound
(0.42 m) and target (0.52 m) retained in the manifest/audit. The first verified
LAND write occurred 0.265 s after thrust loss, versus 28.063 s before the fix;
disarm followed at 8.702 s versus 38.959 s. Impact still occurred at 0.351 s.
Eleven LAND writes and no Disarm commands were recorded. Detecting the failed
hold before impact does not make an all-motor cut recoverable or qualified.

The first full recording/control workload at `57946ac` failed its RX byte-count
assertion after all preceding flight, capture, real tag detection, valid H264 and
low-storage continuation checks passed. Pymavlink returns the literal empty
string for an idle nonblocking TCP read; `5194303` correctly counts that as zero
bytes without altering the original return or accepting nonempty text.
The unchanged workload test then passed in 62.53 s. Its measured interval had
253,213 RX / 14,387 TX bytes over 35.023 s, zero transport/subscriber/log overflow
errors, maximum heartbeat gap 1.061 s, maximum setpoint gap 0.056753 s and
maximum telemetry-delivery lag 0.010761 s. The recorder used real OpenCV tag36h11
decoding and CPU libx264/ffprobe on generated frames; 40 ms periodic write delays
and simulated storage reserve stopped video while tags/telemetry continued.
Recorder peak RSS was 106,216 KiB; encoder peak RSS was separately 93,736 KiB.
These are host TCP and synthetic-camera measurements, not Pi UART, native camera
or Debian AprilTag-backend qualification. GPIO/payload construction was forbidden
by the test adapter and no actuation occurred.

The `f5ff571` workload rerun also passed (60.74 s): 382 analyzed frames/
tag detections, 199 valid H264 frames, 245,531 RX / 14,468 TX bytes over 33.976 s,
maximum heartbeat/setpoint gaps 1.0391 / 0.05502 s, and delivery lag 0.01129 s.
No transport/subscriber/log errors were observed; a zero sequence-gap estimate
is not proof of zero physical packet loss. Recorder CPU time was 12.023 s and
peak RSS 104,408 KiB; the separate codec used 2.418 s CPU and 91,736 KiB peak RSS.
Maximum truth height was 0.529969 m, minimum Loiter height 0.519958 m, XY drift
0.029456 m, with confirmed final disarm. The same host-only scope applies.

### Preceding `f5ff571` common-runtime matrix

All 40 cases were rerun against immutable application revision `f5ff571`,
including failed-hold supervision and handoff-response validation. The result
was **34 passed, 6 failed, zero skipped**. Parallel groups used separate isolated
namespaces; the times below are individual group durations.
All required cases were executed, but the final SITL acceptance gate is **not
green**: profile qualification remains incomplete because those six failures
remain failures.

| `f5ff571` group | Passed | Failed | Time |
| --- | --- | --- | --- |
| Original production baseline | 5 | 0 | 273.46 s |
| Vertical-hold truth profiles | 3 | 5 | 554.16 s |
| Loiter truth profiles | 8 | 0 | 563.76 s |
| Sensor, datum and magnetic faults | 10 | 1 | 583.90 s |
| Dynamics, entry and unrequested-control faults | 7 | 0 | 373.69 s |
| Full generated-camera recording/control workload | 1 | 0 | 60.74 s |

The [`f5ff571` 16-row profile table](../../artifacts/refactor-20260921/sitl/refactor-f5ff571-profile-table.json)
retains each operation, duration, starting heading, disturbance condition,
summary, failed criterion and source-evidence path. All 16 profile cases
confirmed final disarm. All eight Loiter profiles passed: maximum height error
0.010052 m, maximum height 0.529969 m and whole-armed XY displacement 0.176332 m.
Sampled hold XY remained zero at the recorded latitude/longitude quantization.
The three changing-field/gyro-bias inertial Loiter cases passed at starting
headings 0/120/240°; maximum offset-corrected estimated-yaw drift was 5.650094°
during the hold and 7.945322° over the armed sequence. Corresponding maximum
true heading movements were 5.658320° and 6.982025°. These remain simulator
engineering results for the tested durations, not absolute heading or physical
hall qualification.

The three 10-second vertical cases passed their hold criteria but displaced
0.724010–0.731186 m over the full armed sequence. All five 30-second vertical
cases still failed the unchanged 0.5 m hold-clearance criterion: measured hold
XY displacement was 1.377616–1.459474 m, and whole-armed displacement reached
1.696968 m. The sixth final failure was the changing-field compass case:
117.937366° yaw-estimate drift versus 1.313253° actual heading movement, despite
successful mission completion and disarm. Neither failure category was skipped,
reclassified as passing or resolved by weakening an acceptance threshold.

All six actual loss/discontinuity cases reached LAND and disarmed, with a maximum
fault-to-LAND response of 1.504453 s. The native-ID contract observation completed
normally after the requested hold and is excluded from that fault-response
maximum. Both floor-offset cases and magnetic pre-arm refusal passed. All seven
dynamics cases passed, including the unchanged all-motor-loss regression after
the failed-hold repair. Its passing failure-detection/cleanup contract does not
make the motor-loss impact recoverable. The original failed attempts and the
separate targeted descent measurements above remain retained evidence.

The subsequent boundary review reproduced acceptance of experimental settings
on ordinary host interfaces when only the opt-in marker was spoofed. The earlier
scored tests were actually network-isolated, so their evidence remains valid.
`3664e43` now independently checks real interfaces in the controller/CLI shell;
the pure policy core remains free of network inspection. The same review found
that isolated selected 3.14 environments could be rebuilt before the native
import gate failed. `0cc15d9` now refuses that path before uv mutation, preserving
the candidate; normal 3.13 deployment is retained. Both have regression coverage.

The `d86a955` workload rerun passed in 61.66 s: 392 analyzed/tag-detection frames,
208 valid H264 frames, 252,742 RX / 14,366 TX bytes over 34.981273 s. Maximum
heartbeat gap was **1.976630 s**, setpoint gap 0.054708 s and delivery lag
0.010610 s; no transport/receive/subscriber/log errors occurred. Recorder wall
time was 33.828 s, CPU 12.344 s and RSS 104,800 KiB; codec CPU was 2.171 s and
RSS 91,524 KiB. Truth height peaked at 0.519959 m, minimum Loiter height was
0.509949 m and XY drift 0.031318 m, with final disarm. These final measurements
remain host TCP/generated-camera evidence, separately from the Pi capture.

### Final `d86a955` integration matrix

After both boundary fixes, every required case was rerun at immutable
`d86a9557f5fa2f813ed34c17700d298b5bdd5384`. Result: **34 passed, six failed,
zero skipped**. The acceptance gate remains **not green**. No threshold was
relaxed and no failure was converted into a skip.

| Final group | Passed | Failed | Seconds |
| --- | --- | --- | --- |
| Production baseline | 5 | 0 | 272.76 |
| Vertical-hold profiles | 3 | 5 | 554.45 |
| Loiter profiles | 8 | 0 | 562.86 |
| Sensor/datum/magnetic faults | 10 | 1 | 581.25 |
| Dynamics/control faults | 7 | 0 | 371.92 |
| Recording/control workload | 1 | 0 | 61.66 |

The [final 16-row profile table](../../artifacts/refactor-20260921/sitl/refactor-d86a955-profile-table.json)
retains every start, profile and unsuccessful trial. All eight Loiter trials
passed; maximum height error was 0.010052 m, peak height 0.529969 m and
whole-armed XY displacement 0.176332 m. Maximum hold/whole-armed estimated-yaw
drift was 5.652449°/8.046356°, versus actual heading movement
5.665595°/6.982038°. Sampled hold XY was zero at truth-position quantization.
Three 10-second vertical holds passed their hold criteria but moved
0.724010–0.731186 m over the whole armed sequence. Five 30-second vertical
holds failed solely on hold XY displacement: **1.394228–1.458282 m versus the
unchanged 0.5 m bound**, with whole-armed displacement up to 1.688686 m.
All 16 profile trials disarmed; these results do not establish hall clearance.

The final changing-field compass case completed and disarmed but retained
117.937805° yaw-estimate drift versus 1.761604° actual heading motion. Maximum
height was 0.529969 m and whole-armed XY displacement 0.156716 m. Healthy
reported estimator flags and successful mission completion still do not
qualify compass-based hall Loiter.

The final thrust-loss regression retained the concrete 0.33 m range violation
of the 0.42 m lower bound for its 0.52 m target. LAND was first written after
0.264056 s; near-ground truth occurred after 0.351319 s and disarm after
8.700781 s. This passes failure detection and cleanup, not impact recovery.

In the new combined-aiding-loss trace, both EKF cores lost position bits 8/16
42.483 ms before DataFlash MODE9. All 76 subsequent samples per core lacked
those bits and dead-reckoning bit 18. Selected-FC telemetry reported flags 103
16.948 ms before the first LAND heartbeat. Together with the pinned source,
this supports position-unavailable LAND selection as an inference, not a
directly logged internal control-position flag. Earlier `f5ff571` DataFlash
sampling straddled MODE9; its timing must not be substituted for this trace.
Across the six actual sensor-loss/discontinuity cases, the maximum LAND response
was 1.465602 s. The new discontinuity response was 0.100608 s; combined aiding
loss reached LAND after 1.027182 s, with 0.319100 m whole-armed XY displacement.
Every one of those cases confirmed disarm. Native input-ID remapping remains a
contract observation, excluded from the loss-response maximum.

## Pi evidence and native migration

See [the disarmed Pi report](pi-disarmed-refactor-checks.md) for inspected entry
points, fresh FC proofs, resource numbers and every candidate/artifact path.
Three 10 s native camera/video/tag-processing/telemetry captures completed with
zero commanded servo pulses. The earlier refactored `0988628` source ran under the
working Pi 3.13 interpreter: 118 analyzed frames, 1,586 telemetry messages,
zero detected tags, peak RSS 105,916 KiB and end temperature 70.370 °C.
An offline report generated from the copied capture.

The earlier normal checker failed the unchanged battery minimum (14.17 V vs 14.4 V).
After reconnection, the final-source disarmed checker passed with zero
errors/warnings in 20.907 s, battery 15.732 V and the same 14.4 V minimum.
Firmware/parameters matched; only parameter reads and message/version requests
were allowed, without `--prearm`. Fresh read-only probes preceded/followed it;
the final snapshot was clear. This is bench health, not flight clearance.
The 1,187-parameter read-only export completed and confirmed final disarm.
Firmware identity matched 4.7.1 / `dbe79216`. The initial ROMFS attempt was
incomplete; partial readback is not installed-feature proof.

A separate standard uv-managed Pi 3.14.7 candidate built pymavlink and imported
NumPy, Pillow, OpenCV and gpiozero. The initial candidate could not import the installed CPython 3.13
camera/GPIO/tag native bindings under 3.14. Exact matching libcamera development
inputs and compatible pyKMS/lgpio/prctl/AprilTag extensions remain required.
See [candidate details](python-314-candidate.md). Working source, `.venv`, system
Python and services were preserved; no live service selection occurred.

SSH temporarily became unreachable; the failed attempts and last pre-outage
disarmed observations are retained. After connectivity returned, the coordinator
reviewed and ran the final `f5ff571` passive recorder from the separate
`/home/seb/ai-drone-candidates/20260921-refactor/source-f5ff571/` source archive
using the working Python 3.13.5 environment. This was candidate source staging,
not deployment of the prepared runtime review bundle or live-service selection.

The [final Pi manifest](../../artifacts/refactor-20260921/pi/final-f5ff571/refactor-passive-20260921-f5ff571/manifest.json)
records a successful 10.056188 s capture: 73 analyzed frames, 240 encoded H264
frames, 1,254 telemetry messages, zero tags, `allow_flight=false` and GPIO/servo
actuation disabled. Four fresh selected-FC disarmed heartbeats preceded it
(latest age 0.840889 s); four followed it (age 0.576810 s, battery 15.863 V).
The sender allowlist recorded exactly 25 telemetry-interval requests and no
actuator/control commands. Final ownership inspection was complete and clear.

The directly owned serial meter measured 77,991 RX / 1,075 TX bytes over
19.372824 s, including startup requests, with zero read/write/parser errors.
These are actual directional transport returns, separately about 4,025.79 and
55.49 bytes/s; the 115,200-baud 8N1 nominal capacity is 11,520 bytes/s per
direction. Capture-window maximum delivery lag was 0.000192083 s and received
FC-heartbeat gap 1.368586 s. No control heartbeat or setpoint was transmitted,
so this passive run does not measure their outgoing cadence under flight load.
A zero observed sequence-gap estimate is not proof of zero physical loss.

The resource wrapper elapsed 24.119604 s; cumulative child CPU was 22.147240 s
user plus 4.177578 s system, including the initial ownership probe. Peak child
RSS was 113,272 KiB and end temperature 69.832 °C. These measurement scopes
differ from capture duration and do not establish thermal-soak, long-duration
control-load or native tag-recognition accuracy; all three Pi captures found
zero tags. No physical flight or actuator qualification was authorized or performed.

The later read-only ROMFS retrieval produced all 35,238 bytes of
`@ROMFS/hwdef.dat`, saved remotely as
`/home/seb/ai-drone-candidates/20260921-refactor/romfs-hwdef-final.dat` and retained
in [the local evidence](../../artifacts/refactor-20260921/pi/romfs-hwdef-final.dat).
Its SHA-256 `d89b4db7acd2811284c420fb79f0750661f8dfca6865bedf1725a17dfac4babe`
exactly matches the reviewed `candidate-hwdef.dat` and manifest. The transfer
client nevertheless exited with an assertion during session termination: its
default 5 s terminate timeout conflicted with the adjusted 25 s idle setting.
That failed exit is retained; the independently retrieved complete file and
matching hash establish this one installed-ROMFS comparison. Four fresh
disarmed heartbeats preceded the check (age 0.507950 s, battery 15.799 V) and
followed it (age 0.399279 s, 15.798 V), with a clear ownership snapshot. This is
not a full flash-byte comparison; the reboot-separated repeat remains deferred
because no FC reboot was authorized.

Matching native development packages were downloaded and extracted into the
separate candidate sysroot, without OS installation. A pinned libcamera binding
build configured correctly for ARM64/3.14, but exhausted the Pi's 414 MiB swap
during its first C++ object. SSH then became unresponsive; targeted compiler
cleanup could not be confirmed. No completed native wheel/import is claimed.
Prepared memory-bounded libcamera/pyKMS inputs and host source-compatibility
checks are retained separately; pyKMS's capped host compilation failed for
memory. Access recovery, candidate process cleanup, ARM64 native imports and
3.14 passive acquisition remain blocked. Working 3.13 source/environment and
service selection were not changed.

## Prepared deployment and rollback

The reviewed runtime archive contains verified source/lock, the packaged report,
startup/recovery helpers and reviewed firmware metadata. It excludes live
configuration and the working native environment. The source/lock restoration
archive is separate from the wheel/sdist; a standard sdist alone is not the
complete deployment recovery bundle.

Prepared runtime: [ai-drone-runtime-d86a955-review.tar.gz](../../artifacts/refactor-20260921/bundles/ai-drone-runtime-d86a955-review.tar.gz),
279,628 bytes, 108 verified manifest paths; SHA-256
`bb4ad2f943f507ed8411c3ef09d2aad5e7cb43770016b0a90e9b3ef2a7c4ef0f`.
The [bundle manifest](../../artifacts/refactor-20260921/bundles/ai-drone-runtime-d86a955-review.json)
records its contents. This is prepared for review and was not uploaded/applied.

No archive was applied to the live installation. Candidate source directories
used for passive tests are separate. Review the bundle digest and file manifest,
complete the aircraft backup and restore normal disarmed health before any future
guarded deployment. Keep `/usr/bin/python3.13` selected while the 3.14 native gate
is open. The transaction preserves/backups source and interpreter environment as
a pair, validates native imports before restart, and retains rollback evidence
on failed health checks. Failed restoration leaves the service stopped. Restore
both source and environment together and repeat read-only health validation;
never restore source alone over a changed native environment.
Selected Python 3.14 deployment is now refused before environment mutation;
its native artifact manifest/install/preservation contract is still incomplete.

Durable local evidence lives in
[`artifacts/refactor-20260921`](../../artifacts/refactor-20260921):

- [Exact host commands/results](../../artifacts/refactor-20260921/host/ai-drone-final-d86a955-local-validation.json),
  version-separated logs and final distributions.
- `sitl/` archives retain independent truth, attitude, parameter deltas,
  command/transition events, tlogs, DataFlash and summaries for passing and failed
  attempts. `logs/` retains pytest output and JUnit results; final runtime
  provenance is recorded in `sitl/d86a955-provenance.json`.
- `pi/` preserves all three remote captures, the parameter snapshot and the generated
  [offline Pi report](../../artifacts/refactor-20260921/pi/refactor-passive-20260921-0988628/review/index.html).
  `firmware/` preserves the verified saved board artifacts and reviewed inputs.
- `bundles/` contains the final source archive, runtime review archive and Git
  history for the integration, governing-plan and Antigravity branches.
  Antigravity's separate worktree still contains 33 draft status entries,
  including three untracked source files; these were left untouched and also
  preserved as a binary Git patch, status record and untracked-file archive.
  The governing-plan worktree is clean. A Git history bundle alone does not
  capture uncommitted drafts.
- `index.json` and `SHA256SUMS` identify retained files and final simulator results;
  `logs/refactor-commit-list.txt` lists every focused implementation commit.

These local artifacts are ignored by Git and remain in this workspace. The
committed reports retain the essential outcomes and precise remaining blockers.

All remaining physical qualification is listed in the
[hardware checklist](hardware-qualification-checklist.md); software fault coverage
and unproved matrix rows are detailed in [the fault matrix](fault-matrix.md).
