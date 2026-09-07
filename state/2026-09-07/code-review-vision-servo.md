# Detailed camera, recording, AprilTag, and payload servo code review

Scope: maintained `ai_drone/vision/{apriltags,stream}.py`, `ai_drone/recording.py`, `ai_drone/cli/{record,servo,tag_servo_record}.py`, and directly related tests. Read repository AGENTS.md. This review made no hardware connections, sent no commands to actuators, and changed no production code. Mocked fault injection and arithmetic checks ran locally. Different-drone reference captures were not used.

## Architecture and intended behavior

- `ai_drone/recording.py:19-43` defines bounded sensor-message request rates for the documented 115200-baud Pi/FC link. It requests attitude, rangefinder/distance, optical flow, IMUs, pressure, GPS, EKF, battery, RC, servo-output telemetry, and other status messages. `request_message_intervals()` at148-172 sends only `MAV_CMD_SET_MESSAGE_INTERVAL`, not any motor/servo/arm command. `SERVO_OUTPUT_RAW` is flight-controller output telemetry and cannot confirm movement of the payload servo connected directly to the Pi.
- `create_recording_paths()` at62-101 atomically reserves unique directories, avoiding concurrent capture collisions. The dataset contains H264 video, encoder PTS, per-frame detection/metadata JSONL, telemetry JSONL and raw tlog, first/last JPEG, manifest, and a separate servo event log for explicitly active mode. `json_safe()` at104-120 and `write_json_line()` at135-145 reject/nonfinite-normalize values into strict JSON.
- `ai_drone/cli/record.py` provides a common capture engine for ordinary passive inspection and the explicitly separate `drone-tag-servo-record` operation. CLI validation is at160-212. It validates finite positive times/rates, matching video/analysis aspect ratios, tag ID bounds, and exact optional manual-flight acknowledgment.
- Default inspection requires an initially disarmed vehicle, monitors only the selected vehicle heartbeat for arming, and stops if it becomes armed (`record.py:1106-1120`, `TelemetryWorker:378-409`). A separate explicitly confirmed passive manual-flight recording path becomes READY before allowing pilot arm transitions (`1356-1368`). Active tag-servo mode can attach to an armed project FC but never arms or commands flight controls.
- Telemetry starts monitoring during camera initialization. A `CaptureWindow` coordinates one epoch at the first encoded keyframe (`record.py:282-313`, `1182-1209`). Raw telemetry logging is opened before publishing the epoch (`784-805`); pre-epoch telemetry is discarded from JSONL (`412-420`). The code retains monotonic elapsed time and UTC wall-clock metadata.
- On the Pi, `Picamera2` supplies a main 1280x960 YUV420 stream for H264 and a separate 640x480 YUV420 low-resolution stream for detection; raw sensor mode is explicitly 2028x1520, six buffers, `queue=False` (`record.py:1227-1236`). Camera warmup and a fresh monitored heartbeat precede encoding. Hardware camera libraries are imported only on the Pi (`1164-1178`).
- This maintained camera pipeline uses ordinary camera frames and CPU AprilTag processing. It does not initialize `IMX500`, load an inference network, or read inference tensors. Successful camera recording therefore proves image acquisition, not AI accelerator inference.
- `DetectionWorker` processes the low-resolution luminance plane independently from camera capture and telemetry (`record.py:443-558`). Native AprilTag3 is preferred, with an OpenCV ArUco fallback in passive auto mode. It detects tag36h11 IDs and logs corners, center, Hamming distance, margin, and optionally metric pose. Passive mode has a queue of8 and drops new analysis frames when full; active mode has a latest-frame queue of1 (`1050-1052`, `1003-1030`). H264 encoding continues independently of detector throughput.
- Calibration is explicit JSON, with strict finite/pinhole validation and only same-aspect-ratio scaling (`apriltags.py:105-186`). Pose uses square IPPE, selects finite positive-depth candidate with the least reprojection error, and uses OpenCV camera axes right/down/forward (`399-487`). `record.py:501-516` marks pose validity by a configurable reprojection threshold. No maintained transform maps that camera-relative pose into drone/body coordinates or issues a precision-landing flight command from it.
- MJPEG streaming serves `/`, `/stream`, and `/snap`; a semaphore limits persistent clients to3 and socket timeout defaults5s (`stream.py:14-15`, `76-105`). Shared latest JPEG state is condition-protected. It binds all interfaces by default (`111-119`) and has no authentication. Actual recorder frames are grayscale and unannotated (`record.py:1456-1461`), despite legacy module wording about annotated frames/nearest-person pipeline.
- Cleanup attempts every resource independently and retains the first error (`record.py:622-632`, `724-781`). Telemetry join is bounded2s; detection shutdown is bounded10s and makes space for a sentinel rather than blocking forever on a full queue (`585-619`, `655-678`). Completed files are fsynced before manifest publication; JSONL supports periodic sync.

## Servo implementation and safe invocation constraints

- Standalone `servo.py` is strictly Pi-only and fixes BCM12/physical pin32 (`15`, `103-108`, `173-181`). No FC motor channel is involved.
- Exact `--confirm-actuation SERVO_CLEAR` and explicit `--mode` are required before platform/GPIO access (`123-132`, `168-171`). Pulse defaults are900..2100us, with absolute software limits750..2250us and geometry min<1500<max (`16-17`, `110-120`, `149-165`). These are software limits, not proof of a safe mechanical travel envelope.
- `ServoProcessLock` uses flock on `/tmp/ai-drone-bcm12-servo.lock` to prevent cooperating processes from concurrently controlling BCM12 (`36-61`). It does not discover third-party GPIO users.
- `gpiozero.Servo(initial_value=None)` initializes detached (`190-198`). Center and sweep modes run indefinitely (`215-239`); manual mode accepts normalized values, pulse widths, or nominal +/-60 degree mappings and releases GPIO on exit/EOF/Ctrl-C (`241-269`). For a short authorized test, use a bounded manual sequence with measured safe pulse widths and deliberate dwell; an indefinite sweep is unsuitable as a quick check.
- Standalone servo has no MAVLink connection/disarmed interlock. Disarmed status and physical mechanism clearance must be established by the operator/test harness.
- There is no payload position/current feedback. `_component_report()` truthfully labels passive servo status `not_detectable` (`record.py:953-955`), and active event records explicitly say `feedback_available=False` (`tag_servo_record.py:315-325`). Successful software commands alone cannot prove the servo moved, the horn returned, or the latch behaved correctly.
- The active tag-servo entry point requires two exact acknowledgments, a tag allowlist or all-tags selection, explicitly calibrated active/rest positions and pulse duration (`tag_servo_record.py:125-235`). Active hold and settle are bounded2s each; active/rest must differ; confirmation requires2..30 consecutive analyzed detections (`238-305`).
- Active startup additionally requires MAVLink target1/1, ArduPilot quadrotor, and exact `ARMING_SKIPCHK=0` (`record.py:1075-1105`), live camera/native detector, and exclusive servo GPIO (`1340-1351`). Those checks do not establish all flight-readiness conditions.
- Default trigger quality requires zero Hamming corrections, native margin>=30, three consecutive fresh frames, detection age<=0.5s, and FC heartbeat age<=2.5s (`tag_servo_record.py:152-177`, `482-534`, `555-639`). Triggers are serialized and per-tag completed IDs are latched for the run. Freshness is checked again before commanding (`766-800`). A pulse always attempts rest, a settle wait, and detach (`683-751`). SIGTERM/SIGHUP stop accepting and request graceful shutdown (`429-449`).
- This active mode is a tag-presence-triggered payload actuator. It does not require a centered tag, target distance/altitude, valid calibrated pose, ground clearance, or specified drone speed. Optional `pose_valid` is logged but not part of `_quality_results()`. Do not mistake the mechanism for a closed-loop precision drop or landing controller.

## Concrete defects and operational limitations

### 1. Wrong physical pulse widths with asymmetric servo limits (confirmed bug)

`servo.py:75-79` converts pulse-width commands to a normalized value using fixed1500us as the midpoint; `_pulse_us()` at93-96 prints the inverse of the same assumption. `gpiozero.Servo.value` instead interpolates linearly from the configured min to max. Its midpoint is `(min+max)/2`. The code permits asymmetric limits, so commands/logs can disagree with PWM.

Pure reproduction: min900,max2200, request1500us => helper value0 => gpiozero actual1550us. A default900..2100 range is symmetric and unaffected. Tag-servo active/rest commands reuse the same helper at `tag_servo_record.py:385-394`, so calibration-specific active/rest positions can also be wrong.

Official implementation used to verify the external contract: https://gpiozero.readthedocs.io/en/stable/_modules/gpiozero/output_devices.html#Servo (value setter uses a single linear interval). Fix should use `2*(requested-min)/(max-min)-1`, with matching reporting and center semantics, or enforce symmetric limits. Add an asymmetric-bounds regression using gpiozero's mock PWM pin, not a fake that merely remembers normalized values.

### 2. Corrupt video timestamps produce a falsely successful manifest (confirmed bug)

In `record.py:1554-1556`, the manifest's `completed` and `error` fields are evaluated before `record.py:1601` calls `_safe_video_timestamp_summary()`. That helper records parsing errors in `state.worker_error` (`876-888`). Consequently a late parsing error yields exit1 but manifest `completed=true,error=null`.

Reproduced with all endpoint/camera/GPIO access mocked and local `camera.pts` containing `not-a-timestamp`. Result: exit_code1, completedtrue, errornull. Artifact: `/tmp/drone-manifest-review-x_hmjgl7/capture/manifest.json`. Fix by computing the timestamp summary before constructing the final status fields. Existing helper-level failure tests do not cover final manifest consistency.

### 3. Optical-flow `ok` means packets arrived, including quality0 (confirmed reporting limitation)

`record.py:578-582` counts every optical-flow packet without a quality/finite-data check. `_component_report()` at943-946 maps any count to `ok`. A one-packet mocked `OPTICAL_FLOW quality=0` produced `{"status":"ok","samples":1,"quality":0,"rate_hz":1.0}`. Hardware evaluation must inspect quality, rates, finite measurements, and behavior under movement; the current label cannot be used as a sensor-health pass.

The reporter also does not cross-check SYS_STATUS sensor health, EKF use/rejection, data age, expected rates, or actual changing input. FC/camera `ok` status is established at startup (`1119`,`1271`) and does not get downgraded on later stream loss. Passive recording lacks active mode's heartbeat-staleness watchdog.

### 4. Sensor summaries can combine packets from other MAVLink sources and double-count range messages (static finding)

Only heartbeat safety processing filters the selected system/component (`record.py:378-384`). All other incoming messages increment summary counts and feed `_observe_sensor_message()` (`423-425`), even on a shared/network endpoint. Raw JSONL preserves source IDs, so downstream inspection can filter correctly, but summary status is not target-isolated.

`RANGEFINDER` is assumed downward and counted under orientation25 (`573-577`) without source/instance/orientation proof. The same range device can appear both in requested `DISTANCE_SENSOR` and `RANGEFINDER`, making reported sample rates the sum of telemetry messages rather than physical sensor updates. Multiple distance sensors at the same orientation are merged; sensor IDs are not separated. For the user's actual sensor check, parse sensor_id/source/orientation and validate one stream instead of treating aggregate counts as independent observations.

### 5. Inspection may return success with all hardware absent (intentional availability semantics)

`record.py:1121-1131` and1308-1327 retain availability details rather than failing passive inspection; `1646` returns0 if no capture/worker error. `tests/test_recording.py:899-915` explicitly requires successful manifest creation even when all hardware is absent. This is useful for an inspector, but means an exit-code-only smoke check is invalid. Read the manifest component statuses and sample counts.

### 6. A per-tag pose-solve error stops the entire capture (static failure behavior)

Calibrated `estimate_pose()` is inside the per-frame detection loop (`record.py:501-508`), while the exception handler wraps the entire worker (`556-558`). One pose solve with no valid candidate stops recording camera and telemetry, rather than logging a rejected pose and continuing. A geometric outlier should normally invalidate one observation; genuine worker/setup failure can still stop the capture. Existing detector-failure test verifies stopping, not robustness to one degenerate tag pose.

### 7. Camera reads are not bounded by capture deadline/watchdog in this code (static limitation)

`record.py:1425` calls `camera.capture_request()` synchronously with no timeout/cancellation. The loop checks deadline, stop, and active watchdog only before that call (`1396-1407`). A stalled camera API can therefore delay duration completion and cleanup. Bounded detection/telemetry thread joins do not address a blocked main camera request. This observation is about missing bounds at the call site; no physical camera stall was induced.

### 8. Stream availability is not new-frame freshness (static limitation)

`stream.py:89-102` sends the current JPEG again after a2s condition timeout even when no new image arrived. A browser can continue displaying apparently live stale imagery. Stream daemon handlers also have no explicit server-stop event and rely on client disconnect/write failure, though client writes are bounded by socket timeout. No sensor-age overlay is provided.

## Test evidence

Executed locally without hardware:

`.venv/bin/python -m pytest -q tests/test_apriltags.py tests/test_stream.py tests/test_recording.py tests/test_tag_servo_record.py tests/test_drone_tools.py`

Result: **111 passed, 1 skipped in0.76s**. The synthetic calibrated pose test is guarded by `pytest.importorskip("cv2")`; the default laptop environment lacks OpenCV. Hardware-specific camera acquisition, real native AprilTag bindings, actual GPIO PWM timing, servo movement/current, and mechanical safe positions are not validated by these tests.

Meaningful existing coverage:

- Calibration malformed/nonfinite inputs, geometry, corner-order normalization, and synthetic pose when OpenCV available (`test_apriltags.py`).
- Threaded stream handling, snapshot while a stream is connected, client limit, validation (`test_stream.py`).
- Unique capture directories, strict JSON, no actuator commands in telemetry setup, armed-vehicle rejection, selected-heartbeat isolation, pre-epoch recording policy, raw log before epoch, startup failures, fresh-heartbeat gates, bounded detector shutdown, resource cleanup and fsync, all availability combinations (`test_recording.py`).
- Exact actuator acknowledgments before GPIO access, disconnected startup state, bounded numeric arguments, exclusive reusable process lock (`test_drone_tools.py`, `test_tag_servo_record.py`).
- Three-frame qualified pulse, reject low-quality/disallowed/stale detections, reset pre-READY confirmation, per-run completed-ID latch, completed-count stop limit, latest-frame replacement, stale heartbeat stops commands, stop-during-pulse rest/detach, initialization cleanup, correct armed FC acceptance and ARMING_SKIPCHK rejection, disarm stop, and outbound MAVLink read/interval requests only (`test_tag_servo_record.py`).

Priority follow-up validation: asymmetric physical PWM mapping, timestamp-failure manifest end-to-end, quality0/stale sensor status, source-isolated sensor summaries, one-degenerate-tag continuation, actual Pi camera stall cleanup, and independently observed servo motion.

## Priority addendum: CURRENT.MD interrupted concurrency/cleanup review

Read `CURRENT.MD` against the current code on2026-09-07. **Every listed interrupted-review issue remains present.** This section takes priority over the lower-impact data/stream limitations above. Passing existing unit tests does not clear the active tag-servo recorder for airborne use.

Ran an additional **local-only, deterministic mocked fault-injection script**, `/tmp/drone-interrupted-review-repros.py`, using existing fake GPIO, fake lock, and fake MAVLink heartbeat classes. No live endpoint or GPIO factory was used. Full results and event logs: `/tmp/drone-interrupted-review-6zuh0eka/results.json` and sibling directories. These reproductions simulate scheduling and I/O delay rather than claiming those delays occurred on the drone.

### P1: A selected-FC disarm is observed before the actuation gate closes

At `record.py:386-388`, the telemetry worker immediately updates state to disarmed and records a fresh heartbeat timestamp. `record.py:407-409` computes the disarm-stop decision, but the shared stop flag is not set until **after telemetry JSONL writing and sync**, at `record.py:434-437`; logging occurs at426-433. It never directly closes the session's `_accepting` gate on that transition.

During that delay, the actuator sees a fresh heartbeat and a still-open stop gate. The active session checks heartbeat age, not the arm/disarm transition itself, at `tag_servo_record.py:635-639` and788-800. Thus a full servo pulse can start after disarm has already been received.

**Reproduced:** blocked only the disarm heartbeat JSONL write, after state observation. At pulse completion state was `last_vehicle_state='disarmed'`, `saw_disarmed_after_arm=true`, `stop_is_set=false`; fake servo actions were active0.5, rest-0.5, detach. The disarm stop finally happened after releasing the logger.

Required design change: publish disarm/stop to a shared actuation gate immediately, before potentially blocking telemetry logging, and synchronize that gate with the final GPIO command boundary. This must still allow the intended pre-arm behavior when active mode deliberately starts disarmed; checking a generic `last_state != armed` would silently change that behavior.

### P1: Even an already-closed stop gate is not rechecked at the GPIO boundary

`tag_servo_record.py:773-800` checks `_accepting`, `capture_stop`, trigger age, and heartbeat age. It then calls `_command_pulse()`. That method first performs synchronous event write/fsync and stdout printing through `_record_and_print()` at694-700, followed by unconditional active assignment at703. The last checks are therefore separated from the physical output by potentially blocking I/O and no common synchronization lock.

**Reproduced:** during the `servo_pulse_starting` event write, set `capture_stop`, closed `_accepting`, and recorded `vehicle_disarmed`. After the write returned, the code still assigned the active value0.5; it then immediately attempted rest and detach because the hold wait saw the stop flag. A short software assignment can still release a payload mechanism; immediate restoration does not make the unexpected active command acceptable.

Required design change: perform non-actuating durable preparation first, then revalidate stop/disarm, age, and pending ownership under the same gate synchronization used by stop transitions, immediately around the GPIO assignment. Merely adding one more unsynchronized check before the blocking logger leaves the defect.

### P1: SIGTERM/SIGHUP cleanup handlers are restored before physical cleanup

`TagServoSession.close()` sets `_closed` and calls `_restore_signal_handlers()` at `tag_servo_record.py:855-860`. It only afterward cancels/joins the worker at860-868, commands rest at871-875, and detaches PWM at877-880. Restoring the original default signal disposition allows SIGTERM/SIGHUP during these operations to kill the process before the intended rest/detach sequence finishes. If a worker pulse is still active, the vulnerability starts before the join.

**Reproduced:** installed the actual session signal handlers in the local mock process, then inspected the SIGTERM handler inside fake rest and detach callbacks. Both reported `cleanup_handler_installed=false`.

Required change: keep cleanup handlers installed through the entire rest/detach/resource-release sequence and restore them last in a reliable finally path. Handle repeated termination requests without recursively abandoning cleanup.

### P1: Ctrl-C during startup or shutdown can bypass cleanup; failed close cannot retry

The only `KeyboardInterrupt` handler in `record.run()` is inside the runtime capture loop at `record.py:1474-1487`. There is no outer ownership `try/finally` covering FC acquisition at1065-1131, camera/encoder/session startup at1166-1327, or resource cleanup at1489-1509. The startup exception lists exclude KeyboardInterrupt. `TagServoSession.__init__()` similarly catches only `Exception` at `tag_servo_record.py:403`, so a Ctrl-C after GPIO initialization but before constructor completion skips GPIO/event/lock release.

Shutdown is particularly concrete: `close()` sets `_closed=True` at857 before actual work. Its rest wait at874 catches only `Exception` at875. KeyboardInterrupt there escapes before PWM detach, GPIO close, event close, and lock close (877-893). A second call immediately returns at855-856 and cannot finish cleanup.

**Three reproductions:**

1. Injected KeyboardInterrupt from a fake FC startup `wait_heartbeat()`: connection remained unclosed and no manifest was written.
2. Injected KeyboardInterrupt from active-session initial event writing: fake servo and fake process lock both remained open.
3. Injected KeyboardInterrupt during shutdown rest hold: fake actions contained only rest assignment; no detach/close; lock remained open; `_closed=true`; a second `close()` changed nothing.

Required design change: structured ownership/finally cleanup across acquisition and shutdown, and a distinction between cleanup-in-progress and cleanup-complete. Ensure interrupts do not bypass the final detach/release operations; preserve the original error after cleanup.

### P2: Confirmed-tag intent is not durable before the worker receives its trigger

At `tag_servo_record.py:602-608`, the observer marks an ID scheduled and queues its trigger. The durable `tag_confirmed_queued` event is written only at614-620. The worker can take the trigger immediately and command/completely finish its pulse before the observer's confirmation log exists. A confirmation-log failure can therefore occur after an action was already made possible.

**Reproduced:** blocked the confirmed-intent write before persistence. The servo completed active/rest/detach and the journal contained `servo_pulse_completed_commanded`, but no `tag_confirmed_queued` record. The journal was not entirely empty: `_command_pulse()` persisted its separate `servo_pulse_starting` event first. The specific defect is the ordering and durability of confirmed detection intent, not a claim that every servo event lacks an audit record.

Required change: write/sync confirmed intent successfully before publishing to the actuator queue, with clear cancellation/queue-full outcomes and no possibility of duplicate ownership. Avoid holding a lock across an unbounded logger if that lock is also needed to close the actuation gate.

### P2: Duration stop reason remains a scheduling race

The main loop sets `duration_elapsed` at `record.py:1398-1400`. The telemetry worker independently notices the same deadline at `record.py:363-365` or420-422 and sets stop without a reason. If telemetry wins, the main `while not stop.is_set()` exits before setting its reason. Later `state.stop_reason` can remain null and console output says unspecified (`1636`).

**Reproduced:** started a telemetry worker with an already-expired capture window. It set stop but `stop_reason` remained null. This establishes the competing code path; actual winner depends on scheduling.

Required change: every duration-stop path should call the same helper that records `duration_elapsed` before signaling stop. Retain legitimate earlier stop causes.

### Status of the previous atomic latch edit and validation

The specific pending-to-completed race mentioned as the last edit in CURRENT.MD appears addressed: `_complete_scheduled()` removes scheduled ownership and adds the completed lifetime latch in one `_lock` section at `tag_servo_record.py:758-764`. Existing lifetime-deduplication tests passed in the111-test focused run. This does not resolve the publication, cancellation, GPIO-boundary, or cleanup defects above.

The earlier focused suite remains **111 passed,1 skipped**, plus the additional deterministic mocked reproductions above. No repository fixes were made or tests added because this task requested analysis. Full validation after future fixes remains required; current tests lack these interleaving, signal-order, and startup/shutdown interruption regressions.
