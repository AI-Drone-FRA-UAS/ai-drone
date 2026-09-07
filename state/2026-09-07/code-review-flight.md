# Flight and MAVLink code review — 2026-09-07

Scope: maintained `ai_drone/flight/`, `ai_drone/mavlink/`, `ai_drone/cli/control.py`, `ai_drone/cli/motor_test.py`, their focused tests, and flight documentation. Read repository `AGENTS.md`. This review made no hardware connection, issued no vehicle command, and changed no production code.

## Architecture and actual capability

- `ai_drone/flight/controller.py` is the central connection owner and flight state machine. It negotiates telemetry, selects a vehicle, sends a system-255 GCS heartbeat at 1 Hz, processes sensor data, and implements arm → GuidedNoGPS climb → relative-position acquisition → Loiter → LAND.
- The real attitude stabilization and EKF sensor fusion are ArduCopter responsibilities. The Python controller sends a level quaternion and a climb-rate fraction at about 20 Hz during GuidedNoGPS. It does not implement a flight stabilizer, optical-flow fusion, path planner, or AprilTag approach.
- `ai_drone/cli/control.py` exposes guarded `hover` / `takeoff` aliases. Defaults are 0.5 m climb relative to the ground reading, 0.8 m absolute range-aligned ceiling, 5 s Loiter, and minimum 14.4 V. It requires the exact `FLIGHT_TEST_READY` confirmation string and records telemetry/events/code hash.
- Pre-arm gates check the exact 4.7.0/custom-version bytes `1511f271`, exact `ARMING_SKIPCHK=0`, 34 reviewed no-GPS parameters, enabled onboard logging, downward range, flow quality > 0, attitude, zero RC channels, and battery voltage. Firmware commit alone does not identify a complete binary/build-feature set; the separate firmware verification workflow covers that issue.
- Distance/attitude/local-position/RC data have boot-time plausibility checks. EKF, optical flow and battery use host receipt time. Relative position requires horizontal velocity + relative-position flags and excludes constant-position mode; optical flow must have positive quality.
- In flight, low/stale battery, altitude ceiling, changing/stale receiver topology, and unhealthy Loiter navigation request LAND. The CLI additionally checks heartbeat/range freshness and leaving LOITER. LAND is retransmitted each second for a bounded period. Flight cleanup avoids force-disarming an airborne aircraft.
- `ai_drone/mavlink/safety.py` centralizes source and armed-state checks, including a fresh-disarmed check that drains old heartbeats. `parameters.py` source-filters parameter replies and rejects nonfinite values; `devices.py` resolves serial and network endpoints.
- `ai_drone/cli/motor_test.py` is a separate explicit propeller-free bench utility using `MAV_CMD_DO_MOTOR_TEST`, never normal flight takeoff. It caps each motor at 10% and 1 s, uses physical confirmation strings and a countdown, checks output assignments and a new disarmed heartbeat, and requests stop plus disarm observation in cleanup. This utility is unrelated to the payload servo on Pi BCM12.

## Proven findings, ordered by significance

### 1. An intermediate disarmed heartbeat can erase pending-arm cleanup ownership

Locations: `ai_drone/flight/controller.py:308–312`, `:801–814`, `:182–195`.

`arm()` sets `_arm_command_sent=True` before writing the arm command, correctly anticipating ambiguous writes and acknowledgements. However, `_process_message()` clears it on *every* disarmed heartbeat. A disarmed heartbeat received after the arm write but before the armed confirmation can describe the pre-command state (or a still-pending transition). If the link then fails, `__exit__()` sees no flight/arm ownership and closes without issuing any disarm or LAND cleanup. The arm command could have reached the controller and taken effect after that heartbeat.

Mock reproduction exercised the actual `arm()` and `__exit__()` paths with pre-arm gates stubbed: received one disarmed heartbeat, then an `OSError`. Result: one arm write, zero cleanup disarm writes, zero LAND writes, connection closed. This proves the software behavior. It does not establish that the current aircraft experienced this timing.

Suggested correction: retain pending-command ownership until the command has been definitively rejected/cancelled or a post-cleanup state transition is confirmed. Model pending arming separately from confirmed armed/flight ownership. Add a regression test for disarmed → lost link while arming.

### 2. `disarm()` can report success without receiving any heartbeat

Location: `ai_drone/flight/controller.py:875–889`.

After sending disarm, the loop returns whenever the cached `self.is_armed` is false, even when `recv_match()` returned `None`. This occurs naturally during cleanup after a partial arm write: the software has never observed an armed heartbeat, so the cache remains false even though the vehicle state is uncertain. No fresh disarmed confirmation is required. The shared motor-test cleanup uses the stricter fresh-disarmed helper, but controller cleanup does not.

Mock reproduction: a controller with cached `is_armed=False`, `recv_match()` always returning `None`, and `disarm(timeout=0.5)` returned normally after one receive call. No heartbeat was received.

Suggested correction: require a matching disarmed heartbeat newly observed after the cleanup request (and retain the bounded timeout); do not treat a default/cached state as confirmation. This is distinct from issuing force-disarm: the existing normal disarm command can remain unchanged.

### 3. The battery guard accepts MAVLink's unknown-voltage sentinel as a full battery

Locations: `ai_drone/flight/controller.py:361–365`, `:645–666`, `:462–480`.

`SYS_STATUS.voltage_battery=65535` means voltage not provided. The controller divides it by 1000, stores 65.535 V, and refreshes `last_battery_time`. It then passes both the default 14.4 V pre-arm threshold and the in-flight minimum. A stream of unknown readings can therefore keep the battery guard satisfied.

Protocol evidence was read locally in the pinned ArduPilot checkout: `/home/abaris/drone/ardupilot/modules/mavlink/message_definitions/v1.0/common.xml:5014` explicitly defines `UINT16_MAX` as voltage not sent. Mock reproduction processed this value and ran the actual pre-arm battery verifier; result was `65.535`, fresh `True`, verifier passed.

Qualification: the checked pinned ArduCopter implementation currently sends `battery.gcs_voltage()*1000` (or zero when battery support is omitted), so the sentinel's occurrence on this aircraft is not established. The protocol handling is nevertheless demonstrably wrong. The Python battery guard also ignores SYS_STATUS battery-health flags; a freshly delivered packet does not by itself prove a functioning physical monitor.

Suggested correction: reject the sentinel before unit conversion and explicitly invalidate battery state on unknown/unhealthy reports. Preserve timeout behavior for genuinely missing updates. Add protocol-boundary tests.

### 4. Public `hold_loiter()` accepts a mid-hold departure from LOITER

Locations: `ai_drone/flight/controller.py:1022–1034`; contrast `ai_drone/cli/control.py:141–155`.

The public method checks `flight_mode == 'LOITER'` only before entering its loop. A later transition to ALTHOLD passes when heartbeat/range remain fresh; the controller's navigation check is itself conditional on current mode being LOITER. A caller using this advertised method can therefore receive success without holding Loiter throughout the requested duration.

Mock reproduction started in LOITER, changed to ALTHOLD on the first update, and kept heartbeat/range fresh. The method returned success with zero LAND writes. The deployed `drone-control hover` path uses `_monitor()`, which already checks mode every iteration, so this issue presently affects direct API consumers, not that CLI path.

Suggested correction: centralize the hold loop and use one mode-transition guard from both the API and CLI. Add a mode-loss regression test.

## Additional integration and observability gaps

- **Initial vehicle selection is not validated.** `controller.py:211–220` overwrites even supplied target IDs with the first heartbeat's source. Installed pymavlink's `wait_heartbeat()` simply receives the first HEARTBEAT; it does not require an ArduPilot vehicle heartbeat. On a routed/multi-vehicle endpoint, a GCS or a different vehicle can be selected. Later strict parameter/firmware gates often fail closed, but an intended target is not bound to an identity. Direct dedicated USB is less exposed. Motor-test bootstrap has the same selection pattern. Fix by filtering for expected autopilot/type/source before assigning the target, and preserve an explicitly supplied identity.
- **DataFlash log identification lacks source filtering.** `ai_drone/flight/dataflash.py:18–36` accepts any LOG_ENTRY, unlike other shared MAVLink helpers. On a shared endpoint it can associate another vehicle's log number/size/time with this flight. It identifies the latest log only after successful completion and does not download that log. No focused test currently covers this helper.
- **Optical-flow timestamp checks differ from range checks.** `controller.py:347–351` ignores `time_usec`; duplicates or delayed positive-quality flow messages count as fresh on arrival. Do not naively add spec microsecond conversion: the pinned ArduCopter `GCS_Common.cpp:2954` currently places `AP_HAL::millis()` into OPTICAL_FLOW's time field. Source-specific normalization or an explicit transport-age strategy is required. The pinned firmware does suppress OPTICAL_FLOW messages if its front end is unhealthy, limiting ordinary sensor-unplug exposure; backlog/replay freshness remains an integration concern, not an established sensor failure.
- **No physical horizontal boundary.** The maintained hover has altitude/time limits and optical-flow/EKF health checks, but does not enforce a radius from the takeoff point. SITL checks horizontal drift after a simulated flight; that is a test assertion rather than a runtime geofence. The downward sensor does not provide forward obstacle detection. This is an acknowledged capability limit, not evidence of a current code defect.
- **Cached mode confirmation.** `set_mode()` accepts `self.flight_mode == requested` after an update even without proving a new matching heartbeat. Usually transitions request a different mode and thus must observe a change, but repeated requests are not confirmation of a new controller response.
- **Documentation drift.** `docs/PI_MAVLINK_CONTROL.md` still describes a tested RC override/kill path as a required live-flight step, while the implemented autonomous gate deliberately rejects any nonzero RC channel count. The operational emergency-LAND method needs a clearly documented, compatible independent implementation. `ai_drone/flight/__init__.py` also mentions “person-follow state”, which is not present in the maintained controller.

## Verification and test coverage

Executed locally, with no live endpoints:

```text
.venv/bin/python -m pytest tests/test_controller.py tests/test_motor_test.py tests/test_flight_guards.py tests/test_flight_recording.py tests/test_flight_provenance.py tests/test_shared_helpers.py -q
83 passed in 0.40s
```

Four independent mock reproductions confirmed the battery sentinel, pending-arm cleanup, cached disarm-success, and hold-mode-loss behaviors above. They invoked no real connection and made no repository modifications.

Existing focused tests cover parameter and firmware gates, downward-vs-forward sensor filtering, stale/future range packets, logging configuration, LAND retries/timeouts, no force-disarm after takeoff, GCS heartbeat rate, EKF/flow flags, receiver topology, low battery, altitude alignment/ceiling, bounded climb-target construction, mode-entry gates, termination callbacks, motor-test bounds, and partial motor-command cleanup. These are useful safety foundations, but most controller state transitions are mocked; passing them does not exercise actual asynchronous arming timing or physical sensors/actuators.

`tests/test_sitl.py` contains two opt-in acceptance scenarios: the production inspection+hover flow with external simulated flow/range packets, and process SIGKILL to exercise the FC's GCS-loss LAND/disarm. They require `ARDUPILOT_ROOT`; ordinary pytest runs skip them without it. The local checkout at `/home/abaris/drone/ardupilot` is the exact expected commit and has `build/sitl/bin/arducopter`. SITL was subsequently run in a localhost-only network namespace: both existing integrations passed in 108.57 s; see `/tmp/drone-sitl-result.md`. The current SITL suite does not cover the pending-arm/unknown-voltage/mode-loss cases above, poor texture/lighting, biased sensors, battery droop, payload release dynamics, or mechanical servo operation.


## Follow-up SITL validation

Both existing pinned ArduCopter integration tests passed in 108.57 seconds in an isolated network namespace with only loopback. Normal production hover and recovery after SIGKILL/GCS-heartbeat loss each achieved GuidedNoGPS → Loiter → LAND/disarm, maximum altitude 0.520 m, minimum Loiter altitude 0.520 m, and maximum Loiter XY drift 0.029 m. These are simulated measurements, not physical drone readings. The tests disable the companion battery threshold (`--min-battery 0`) and do not operate the servo. Full metrics, isolation evidence, caveats, and artifact locations are recorded in `/tmp/drone-sitl-result.md`; raw output is `/tmp/drone-sitl-20260907-b9t_nun4/pytest-output.txt`. The four isolated mock findings above remain valid and are not exercised by these two passing integration scenarios.
