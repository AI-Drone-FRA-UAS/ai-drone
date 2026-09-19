# ai-drone code audit and functional-core cleanup plan

## Context

The user asked for a detailed audit of `/home/abaris/drone/ai-drone` (this directory
only): how many lines can reliably be removed, how to raise code quality substantially,
how many likely bugs exist, and how much an OCaml/Haskell-inspired functional style
would help. `docs/CLEANUP_QUESTIONNAIRE.md` already commits the project to concise
functional Python with decisions separated from effects. Branch `refactor/concise-runtime`
(commit 37ecd84) is the starting point. The user chose: **full functional core**, delete
`scripts/disarmed_tag_mount.py`, remove the USB adapter auto-detection.

Method: three full-file audits (every source file read end to end), plus ruff/ty/pytest
runs and an AST scan. I re-verified the highest-severity claims by reading the code;
several audit "confident" bugs were false and are excluded below.

## Baseline (2026-09-19)

| Metric | Value |
| --- | --- |
| Non-test source (ai_drone, scripts, site) | 19,271 lines / 65 files |
| Tests | 19,533 lines / 46 files / 1,423 pass, 2 skip, 5 SITL deselected (16 s) |
| Docstring + comment lines | 645 (3.3%). The code is not over-documented; the Markdown is (45 files, 5,459 lines) |
| Definitions with zero references | 2 (17 lines); referenced only by tests: 2 (22 lines) |
| Functions over 60 lines | 27 (4 over 100) |
| `raise` / `try:` / blind `except Exception` | 665 / 220 / 68 |
| `isinstance(` / `dict[str, Any]` / `Any` | 165 / 91 / 263 |
| `print(` vs logging calls | 172 vs 10 |
| `@dataclass` total / frozen | 43 / 30. `match`: 0. NamedTuple: 0 |
| Largest classes | `DroneController` 60 methods, `TagServoSession` 29, `MavlinkEndpoint` 21, `VehicleServer` 19 |
| Entry points | 1 console script, 11 subcommands, 5 CLI modules outside the dispatcher, 6 script mains, 5 four-line shims; 25 `main()`, 148 `add_argument` |
| ruff (project config) | clean. Broad ruleset: 1,627 hits (raise-message style 1,178, magic values 110, blind except 43) |
| ty | 10 diagnostics, 9 from one untyped status dict in `cli/deploy.py:_runtime_ready` |

## Answers to the four questions

### 1. Lines that can reliably be removed

| Bucket | Lines | Main sources |
| --- | --- | --- |
| Dead code | ~230 | `scripts/disarmed_tag_mount.py` whole file (313 counted under duplicates), `link/usb_ssh.py:114-206` detection stack (~80), `flight/controller.py:1171 hold_loiter` (20, tests only), three duplicate Protocols in `cli/tag_servo_record.py:44-83` (45), `site/build.py` `Page.lead/body/in_nav` branches (12), `mount.py current_value` (7) |
| Duplicated logic | ~650 | systemctl state parse ×5, systemd-run builder ×2, runtime-status validator ×3 (they disagree), `ssh_base_command` ×2, JSON scalar validators ×5, freshness predicate ×7 in controller + 5 windows elsewhere, DISTANCE_SENSOR validity ×2, `remote.py` re-implementing `shared.py`, `_print_command/_run` ×3, whole `disarmed_tag_mount.py` (313) |
| Redundant defensiveness | ~330 | `vision/apriltags.py` triple validation (40), `network.py` per-tick re-validation, `isinstance` on typed params, asserts as guards, try/except that re-raise, 8 `_raise_if_startup_stopped` calls |
| Verbosity | ~870 | `cli/record.py` manifest literal (144-line function → dataclasses, −90) and 130-line parser (−50), `scripts/verify_ardupilot_firmware.py` 185 lines of `if type(x) is not int` (−170), `review/data.py` `CSV_FIELDS` restating the handlers (−90), `operator.py` ssh options (−22), table-driven validators |

**Reliably removable without changing structure: ~2,000 lines (about 10%)**, from
stages 0–3 below. Pure deletions and copy-paste removal alone are ~800. The functional
core (stage 4) is a restructuring, not a deletion: the design pass estimates `flight/`
at −220 (14%), `cli/control.py` −22, `cli/tag_servo_record.py` −40, `runtime.py` about
−10, and a context-manager restructure of `cli/record.py` around −100. Realistic end
state: **~16,500–17,000 source lines (−12 to −15%)**. Test volume stays roughly flat in
`flight/` (mock-heavy tests become table-driven pure tests) and shrinks modestly
elsewhere. Relocatable, not removable: the 503-line HTML/JS template in `review/html.py`.
The value of stage 4 is structural (testable decisions, unrepresentable bug classes),
not line count.

### 2. How to raise quality substantially

- Verified bugs first, in one small PR (stage 0), before any restructuring.
- One shared helper per concept that is currently copy-pasted (stage 2): subprocess/systemd, remote `uv run` over SSH, JSON scalar parsing, freshness, signal handling, runtime status record, CLI harness.
- Parse, don't validate (stage 3): every JSON/dict boundary becomes a frozen dataclass parsed once. This removes the 91 `dict[str, Any]`, most of the 165 `isinstance`, and all 10 `ty` diagnostics.
- Decisions return values; effects live in one shell per process (stage 4). Analysers return `(summary, diagnostics)`; printing happens once in `main`.
- Enforce it: add `ai_drone.review` to `.importlinter`; widen ruff to `BLE`, `TRY`, `FBT`, `ARG`, `PLR0913`; add `[tool.ty]` once the boundaries are typed.

### 3. Bugs

The audits produced **83 candidate findings** (37 marked confident, 46 plausible). I
verified 25 of the highest-severity ones by reading the code: 18 hold, 4 are false or
benign, 3 are design questions. Applying that ratio, expect **55–65 real defects or
fragilities**, about 10 of them on safety paths. The following are verified by me:

**Safety paths**
- `flight/controller.py:1198-1204` `land()` polls telemetry before sending LAND; once the shared endpoint has failed (`mavlink/shared.py:286-290` raises on any receive), the retry loop is unreachable. The CLI wrapper `cli/control.py:176-183` gives a second-chance LAND, so the safety property depends on the caller.
- `flight/controller.py:1218-1219` `emergency_stop` does `"LAND" not in mapping`, but `MavlinkEndpoint.mode_mapping()` (`shared.py:341-345`) returns `None` until the endpoint pops a heartbeat, and `cli/control.py:156` swaps in a fresh endpoint after connect. `TypeError` inside the emergency path; small window, trivial fix.
- `flight/controller.py:1002-1033` `disarm()` sends one unacknowledged command and waits 10 s; `land()` retries every second. A dropped frame leaves motors armed on the ground.
- `flight/controller.py:1072-1113` vs `:562-568` takeoff target is a gain above the pad reading, the ceiling guard is absolute. With `--takeoff-alt` near `--max-alt` any pad offset trips a spurious LAND.
- `cli/power.py:445-446` `_shared_final_action` indexes `fc["heartbeat_age_s"]` without the `_require_disarmed(fc)` guard that `:412` applies on the parallel path; the probe fallback returns dicts without that key → `KeyError` in the shutdown worker, and the disarmed check is skipped on the runtime path.
- `cli/tag_servo_record.py:870-920, 970-974` with `open_mount=True` the servo is commanded and never returned to rest, yet an interrupted hold reports `servo_pulse_interrupted` and the manifest says `completed_commanded_pulses: 0`. The audit record disagrees with the physical mount state.
- `mavlink/server.py:428-431` `close()` raises on join timeout before `_release_socket()`, leaking the lock fd and the `flock`.
- `flight/guards.py` is a weaker duplicate of `_enforce_flight_limits` (its battery check passes on `None`), but it also carries the altitude/heartbeat staleness checks the controller's guard lacks. Fold, don't just delete.

**Correctness / robustness**
- `cli/record.py:1494` `_finish_recording` runs outside the `try/finally`; any exception there loses `manifest.json`, which also disables `walk.py`'s automatic report.
- `link/deploy.py:89` and `link/targets.py:56` define two incompatible `ssh_base_command`.
- `mount.py:27` lock file at a fixed path in `/tmp` opened without `O_NOFOLLOW`; belongs in `/run/ai-drone`.
- `settings.py:123-130` config resolves relative to CWD and silently falls back to built-in defaults (wrong host) when absent.
- `runtime.py:72-77` `network-profiles.json` read with no schema check; a malformed file crashes the access service with a traceback.
- `recording.py:72` recording directories use local time; every other timestamp is UTC.
- Three divergent "runtime status is fresh and disarmed" validators (`cli/power.py:170`, `cli/deploy.py:100`, `scripts/setup_runtime.py:151`); two DISTANCE_SENSOR validity rules (`controller.py:110`, `capture/reporting.py:35`); five heartbeat-freshness windows (2.0 s and 2.5 s).
- `cli/control.py:354-356` handoff handler returns `result and 0`; `int()` of a falsy result is a `TypeError`.

**Needs measurement, not code reading**
- `flight/recording.py:64` requests 25 telemetry streams (four at 20 Hz) on the 115,200-baud link the controller depends on; estimated 65–75% utilisation. Measure in SITL/bench before changing.
- `mavlink/server.py:370-404` any rejected request closes the peer and releases control ownership. Possibly intentional fail-closed policy; decide explicitly.
- `capture/state.py` counters are mutated by worker threads and iterated by the main thread with no lock. Theoretically `RuntimeError`, practically rare under the GIL.

**Rejected audit claims (do not fix)**: `cli/report.py:89` iterdir-while-replacing is safe (`os.listdir` materialises first); `cli/record.py:249` float aspect-ratio equality is exact for integer resolutions; `cli/motor_test.py:130` "tight loop" blocks on each heartbeat.

### 4. What a functional style buys here

Of the ~60 real findings, roughly **40 fall into classes the restructuring makes
unrepresentable or single-sourced**:

| Class | Examples | Functional replacement |
| --- | --- | --- |
| `None` as error channel (~8) | `mode_mapping() -> None` into `emergency_stop`; `_probe_fc` five dict shapes; `Observations.fresh() or {}` | `Result[T, Reason]` / tagged unions matched at the edge |
| Untyped wire dicts (~10) | power snapshot chain, transfer receipts, runtime status, network-profiles, the 9 `ty` errors | frozen records parsed once at the boundary |
| Stringly-typed state (~6) | `flight_mode == "LOITER"` fail-open vs `!=` fail-closed; `disarmed_tag_mount` phase strings; power action strings | `Enum`/tagged union + `match` |
| Boolean-flag state machines (~6) | `cli/deploy.py` six flags; controller five ownership flags; motor-test `disarmed_observed` conflating two facts | explicit `Phase` union carrying its data |
| Diverged duplicate policy (~8) | 3 status validators, 2 range rules, 5 freshness windows, 2 firmware identities | one pure function, one constant |
| Effects mixed with decisions (~5) | `land()` ordering, manifest outside error boundary, compute-and-print analysers | `decide -> commands`, shell executes |

The remaining ~20 (resource leaks, thread safety, missing timeouts, single-shot disarm,
`/tmp` lock path, local-time names) are ordinary engineering fixes that no style removes.
Python limits: no exhaustiveness checking without `assert_never`, tagged unions cost
lines, and `ty` must be given typed boundaries before it can help. Expect the functional
core to be roughly line-neutral in `flight/` on its own; the savings come from deleting
the seven freshness predicates, six poll loops, twelve-method test stubs, `guards.py`,
`_monitor`, and the duplicated validators around it.

## Staged implementation (each stage: pytest green, ruff/ty/lint-imports clean, PR)

### Stage 0 — verified safety and correctness fixes (small, first)
Files: `ai_drone/flight/controller.py`, `ai_drone/mavlink/shared.py`, `ai_drone/mavlink/server.py`, `ai_drone/cli/power.py`, `ai_drone/cli/record.py`, `ai_drone/cli/tag_servo_record.py`, `ai_drone/cli/control.py`, `ai_drone/mount.py`, `ai_drone/settings.py`, `ai_drone/runtime.py`, `ai_drone/recording.py`.
- `land()`: send LAND first, poll inside `suppress(SharedMavlinkError)`; `mode_mapping()` total (return ArduCopter map or raise a `FlightSafetyError` with a message, never `None`); `disarm()` re-sends every 1 s like `land()`; make the takeoff ceiling compare the same quantity as the target (relative to `_ground_reference`) or validate `target + ground_reference <= max_altitude` before climbing.
- `_shared_final_action`: call `_require_disarmed(fc)` before using `heartbeat_age_s`.
- `VehicleServer.close()`: `_release_socket()` in `finally`.
- `run()` in `cli/record.py`: move `_finish_recording` inside the error boundary, write a minimal manifest on failure.
- `_command_pulse`: for `open_mount`, count the pulse completed once the active command was issued; keep `servo_pulse_interrupted` only when nothing was commanded.
- Handoff handler returns `0` explicitly; `mount.py` lock to `/run/ai-drone/` with `O_NOFOLLOW`; `load_settings` warns (stderr) when falling back to defaults; `runtime.py` validates `schema == 1` and required keys; `recording.py:72` use UTC.
- Run the SITL gate (`pytest -m sitl`) after the controller changes.

### Stage 1 — approved deletions
- Delete `scripts/disarmed_tag_mount.py`, `tests/test_disarmed_tag_mount.py`; drop it from `ai_drone/link/deploy.py` sync list (line ~44). `scripts/tag_mount_capture.py` remains the entry point.
- Delete `link/usb_ssh.py:114-206` detection functions and their tests in `tests/test_cross_platform_scripts.py`; keep `--usb-iface` path and `iface_exists`.
- Fold `guards.py` staleness checks into `_enforce_flight_limits`, then delete `flight/guards.py`, `tests/test_flight_guards.py`, and `cli/control.py:_monitor` in favour of `DroneController.hold_loiter` (currently test-only).
- Remove `cli/tag_servo_record.py:44-83` Protocol duplicates (import `capture/state.py` types), the unreachable `:511-513`, `--pin` with a single legal value in `cli/servo.py`; `site/build.py` dead `Page` fields; `mount.py current_value`.
- Add `ai_drone.review` to `.importlinter` contracts.

### Stage 2 — shared helpers (dedup)
- New `ai_drone/system.py`: `run()`, `unit_state()` (replaces 5 systemctl parsers incl. `mavlink/ownership.py:44-66`), `systemd_run_command()` (walk + control), `require_uv()`, `namespace_to_flags()`.
- `link/targets.py`: one `remote_uv_command(target, module, args)` used by `transfer.py:416`, `config/sync.py:54`, `cli/power.py:480`, `scripts/network.py:39`, `link/deploy.py:487`; delete `link/deploy.py:89 ssh_base_command`.
- Extend `ai_drone/validation.py` with `json_int`, `json_number`, `bounded`; adopt in `vision/apriltags.py:26-40`, `config/snapshot.py:70`, `review/data.py:132-151`, `config/sync.py:99`, `settings.py:105`, `transfer.py:186`, and the five CLI bound-check loops.
- `mavlink/safety.py`: `is_fresh(observed, now, max_age)` and one `HEARTBEAT_MAX_AGE_S`; adopt at the 7 controller predicates and `capture/state.py:73`, `capture/reporting.py:11`, `flight/ownership.py:15`, `runtime.py:31`, `shared.py:530`, `cli/tag_servo_record.py:765`.
- `RuntimeStatus` frozen record + `parse()` in `ai_drone/runtime.py` (or `mavlink/`), used by `cli/power.py`, `cli/deploy.py:_runtime_ready`, `scripts/setup_runtime.py:_fresh_disarmed`.
- One `distance_sensor_valid()` for `controller.py:110` and `capture/reporting.py:35`; firmware identity constants moved to `flight/params.py` and imported by `cli/check.py` and `scripts/verify_ardupilot_firmware.py`.
- `handled_signals()` context manager (shape of `cli/control.py:117-147`) replacing seven hand-rolled installs.
- CLI harness in `ai_drone/cli/harness.py`: parser skeleton, connection context manager (`resolve_mavlink_endpoint` → `open_ardupilot_connection` → close), exception→exit mapping, single print/log edge. Adopt in the 16 CLI modules; register `power`, `servo`, `mount`, `motor-test`, `config-export` in `cli/main.py:COMMANDS` so shims can go.

### Stage 3 — typed boundaries (parse, don't validate)
- `transfer.py`: `Endpoint`, `Inventory`, `Receipt` frozen records with `parse()`; drop `KeyError/TypeError` catches in `main`.
- `cli/power.py`: `PiSnapshot`, `FcProbe` tagged union (`Absent | Busy | Unavailable(reason) | Observed(...)`), `RuntimeStatus` from stage 2; delete the `.get()` re-validation chain.
- `settings`: thread one `Settings` from `cli/main.py` into commands (pattern in `cli/runtime.py:37-45`), remove the `os.environ` round trip.
- `cli/record.py`: `Manifest`, `Components`, `Files` dataclasses + `json_safe(asdict())`; `_parser` driven by a per-operation spec record; `operation` as `Inspect | TagServo | TagMount`.
- `review/data.py`: `Signal(mav_field, csv_column, scale, panel, unit)` table; derive `CSV_FIELDS`; one interpreter for `_imu/_motion/_environment/_flow`.
- `cli/check.py`: analysers return `(summary, Diagnostics)`; `Observations.latest` keyed by `(type, orientation)`; parameters not routed through the latest-per-type deduplicator; render once in `main`.
- `vision/apriltags.py`: fold `validate()` into `__post_init__`, shared `_non_negative_int`, hoist `cv2.setNumThreads` to the CLI.
- `scripts/verify_ardupilot_firmware.py`: declarative field spec.
- `cli/deploy.py:_apply_staged_update`: `DeployState` union replacing six booleans.
- Turn on `[tool.ty]` and ruff `BLE`, `TRY`, `FBT`, `ARG` once these land.

### Stage 4 — functional core (controller first, then tag servo, runtime, recording)

Design verified against the code (`controller.py:148-219` has 41 attributes, 17 of them
value/timestamp pairs; `_require_autonomous_control` is called at 15 sites; the SITL gate
in `tests/test_sitl.py:354-404` reads only external behaviour, so internals may change).
`ai_drone/flight/ownership.py` is the existing pure model to copy.

**New modules** (about 320 lines total):
- `flight/state.py`: `Sample(value, at)` generic; `Heartbeat(armed, mode)`; `BootClock`; frozen `VehicleState` with 11 optional `Sample` fields (heartbeat, altitude, local_altitude, yaw, battery, ekf_flags, flow_quality, rc_channels, firmware, boot, altitude_offset); `fresh(sample, now, max_age)`; `observe(state, message, *, received, now, heartbeat_max_age, fallback_mode) -> VehicleState` replacing `_process_*` (434-528), `_align_local_altitude` (530-543), `_timestamp_is_fresh` (403-425); one-line freshness predicates taking an explicit `now`.
- `flight/phase.py`: `Phase = Unclaimed | ArmPending | Armed | Flying(ground_reference) | Landing | Landed | Human(mode)` replacing the five booleans and `_ground_reference`; `after_heartbeat(phase, armed)`, `after_land_command(phase)`. `Landed` is required because `_landing_commanded` deliberately stays set after disarm (486-491, 605).
- `flight/limits.py`: `FlightLimits(max_altitude, min_battery_voltage, heartbeat_max_age)`; `Violation(reason, fatal)`; `Command = SetMode | Arm | Disarm | Climb`; `COPTER_MODES = mavutil.mode_mapping_byname(MAV_TYPE_QUADROTOR)` (needs no heartbeat); `flight_violations` (replaces `_enforce_flight_limits`, same order and message strings), `hold_violations` (replaces `hold_loiter` body, `control._monitor`, `guards.check_safety_guardrails`), `preflight_violations` (replaces the five `wait_for_*` predicates and `verify_battery_before_arming`; "active RC receiver" and "battery below minimum" are fatal, "no fresh …" are retry), `climb_command`, `takeoff_progress`, `takeoff_within_ceiling(target, ground_reference, max_altitude)` for defect (4).

**Shell** (`DroneController` stays a class, ~12 attributes, public API preserved):
`_execute(command)` is the single outbound choke point that refuses to write in `Human`
(replaces 15 `_require_autonomous_control` calls; GCS heartbeat and REQUEST_MESSAGE stay
outside it). `_command_until(command, done, timeout, period=1.0)` drives both `land()`
and `disarm()`: pump telemetry best-effort (receive errors logged, not raised), re-send
every second, finish on a *new* disarmed heartbeat. `emergency_stop()` becomes
`phase = after_land_command(phase); _execute_best_effort(SetMode("LAND"))` and never
raises. `__exit__` is a `match` over `Phase` with `case _ as x: assert_never(x)`.
`_wait_until(check, timeout)` replaces `_poll` and the five poll loops. `set_mode` uses
`COPTER_MODES`. Kept in the shell: `connect`, `close`, `request_telemetry_streams`,
`_pump_gcs_heartbeat`, `verify_*`, `_fresh_disarmed`, `_send_command_long_and_wait_ack`,
`supervise_human`, the `healthy_since` timer in `enter_loiter`. Removed public methods:
`wait_for_*` ×5, `verify_battery_before_arming`, `wait_for_relative_position`, the seven
`*_is_fresh`, `navigation_is_healthy`, `no_rc_input_is_confirmed`.

**Sub-stages, each followed by the SITL gate:**
1. `phase.py` + `Command`/`_execute` + `_command_until`; rewrite `land`/`disarm`/`emergency_stop`/`__exit__`/ownership resolution on `match phase`; delete the flag juggling (−190, +190; controller ≈ −60). Tests: 17 tests in `tests/test_controller.py` switch from flag pokes to `controller.phase = Flying(0.05)`; `tests/test_control_ownership.py:43-60` fixture; new `tests/test_flight_phase.py` transition table; regression tests for LAND resend under `SharedMavlinkError`, mode 9 sent with no heartbeat, disarm resend. Deliberate change to state in the commit: `emergency_stop` while merely `Armed` makes cleanup land rather than force-disarm.
2. `state.py` + `observe`; `__init__` 72→40, `_process_*` 110→12 (controller ≈ −200). Tests: message-driven tests port to `tests/test_flight_state.py` as pure `observe` tests. Preserve exactly: `now - at <= max_age` comparisons, the heartbeat age filter (472-476), the `connection.flightmode` fallback (482).
3. `limits.py`; delete `flight/guards.py`, `control._monitor`, delegate predicates; `cmd_hover` calls `hold_loiter` (flight ≈ −40, control −22). Tests: `tests/test_flight_guards.py` → `tests/test_flight_limits.py` table-driven; new test that takeoff refuses `ground + target > max`.
4. `preflight_violations`/`climb_command`/`takeoff_progress` + `_wait_until`; delete the poll loops (controller ≈ −120). The twelve-method `_stub_arm_preconditions` shrinks to the four `verify_*` reads plus `_fresh_disarmed`/`set_mode`; readiness is a `VehicleState`. One deliberate timing change: pre-arm freshness becomes one combined 3 s window instead of five sequential ones; if SITL shows "no fresh …" at arm, widen to `min(timeout, 6.0)`.

Cumulative: `controller.py` 1,225 → ~780; `flight/` 1,623 → ~1,400; `control.py` 383 → ~360.
Carry over exactly: 20 Hz climb setpoint cadence, `<=` freshness, 1 s LAND resend.

**Where the same shape does and does not pay off:**
- `cli/tag_servo_record.py` `TagServoSession.observe()` (658-709): yes, partially. Extract frozen `StreakState` and pure `qualify(detections, config, now, captured)`, `advance_streaks(prev, qualifying, captured, ready)`, `select_trigger(streaks, scheduled, completed, config)`; `observe` shrinks to ~20 lines of effects (−40, 29 → ~22 methods). Do not touch the actuator side (`_actuator_loop`, `_command_pulse`, `_release_pulse`, 813-1000): hardware sequencing under a lock.
- `runtime.py` `VehicleAccess` network handling: yes. `Idle | Activating | Deactivating | Blocked` with a pure `step(state, observation, now) -> (state, effects)`; `LinkState` NamedTuple replaces the 3-tuple indexed at 8 sites; `network_busy` becomes `state is not Idle`.
- `cli/record.py` `_Recording` (868-890): **no** observe/decide. Its 19 fields are resources and sync primitives with few real decisions. Apply the owner's own rule instead: one context manager per resource (flight capture, camera, encoder, stream server, servo session, storage monitor, signals) composed with `ExitStack`, replacing `_close_recording`'s ordered teardown and `_camera_startup_failed`'s four-field reset (≈ −100). `CaptureState` stays a lock-guarded cross-thread sink; add the lock around its counters.

### Stage 5 — tests and docs
- Replace monkeypatch-heavy controller tests (`tests/test_controller.py:46-62` twelve-method stub) with pure tests of `observe`/`limits`/`decide`; keep behaviour, failure-isolation and hardware-safeguard tests; keep SITL cases.
- Update `docs/SOFTWARE_ARCHITECTURE.md` boundaries table and the command list; remove `_PAGE` template to `review/report.html` via `importlib.resources`.

## Verification (per stage and at the end)
```bash
uv run --locked --group dev --group docs ruff format --check .
uv run --locked --group dev --group docs ruff check .
uv run --locked --group dev --group docs ty check .
uv run --locked --group dev --group docs lint-imports
uv run --locked --group dev --group docs deptry .
uv run --locked --group dev --group docs pytest -q -m 'not sitl'
ARDUPILOT_ROOT=/home/abaris/drone/ardupilot UV_CACHE_DIR=/tmp/uv-cache uv run --locked --group dev pytest -m sitl -vv -s   # after stages 0, 1, 4
uv run --locked --group dev --group docs python site/build.py
```
Disarmed Pi checks after stages 2–4: `uv run drone check`, `uv run drone runtime status`,
a short `scripts/tag_mount_capture.py` run with servo disabled, then a `drone power`
snapshot. No armed tests; flight validation stays in SITL until the owner schedules it.
Track the line count per stage (`rg --files -g '*.py' ai_drone scripts site | xargs wc -l`)
against the 19,271 baseline.
