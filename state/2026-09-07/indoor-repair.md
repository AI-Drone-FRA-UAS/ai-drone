**Indoor sensor and software repairs — 7 September 2026**

The downward MicoAir MTF-01P and GPS-free relative-position estimator are working again. FC USB and the Raspberry Pi's UART4 connection are verified. The forward MT-15 still supplies no bytes to UART3, including a temporary RX/TX-swap test. The compass responds but fails its normal magnetic-field pre-arm check. These two physical integration issues prevent calling the drone ready for indoor flight or autonomous room navigation.

**Applied FC repair**

The FC was disarmed before every write and both reboots. All requested values were read back before and after reboot. The full snapshots contain 1,172 parameters each. The verified post-reboot capture is also saved in the usual [dated parameter file](../../params/flywoo-f745-live-2026-09-07.param) and [metadata](drone-config.json), retaining its actual 16:21 CEST capture time:

- Before: [fc-before.json](../../artifacts/indoor-repair/20260907/fc-before.json), SHA-256 `4ba50bb3e2e28bb24eb351c071291fdc45b29aebc724fd4f7883b47608fda5c5`.
- After: [fc-after.json](../../artifacts/indoor-repair/20260907/fc-after.json), SHA-256 `b8c6a32c4788ec0c9fcbfc2f7ce12dcfb14af97d6fff40c6a12726be82303c8f`.

| Parameter | Before | After | Reason |
| --- | ---: | ---: | --- |
| SERIAL1_PROTOCOL | 1 | -1 | Free a MAVLink channel occupied by the unused UART1 |
| MAV2_OPTIONS | 2 | 0 | Keep UART2 radio forwarding public after channel reallocation |
| MAV3_OPTIONS | 0 | 2 | Make the assigned forward sensor UART3 private |
| MAV4_OPTIONS | 2 | 0 | Restore public companion UART4 routing |
| ARMING_SKIPCHK | 4 | 0 | Restore all normal pre-arm checks, including compass |

MAV5_OPTIONS remains 2, making downward sensor UART5 private. Five firmware MAVLink channels now map to USB/UART2/UART3/UART4/UART5. Previously, enabling UART3 while unused UART1 remained enabled excluded UART5 entirely. Its sensor emitted MSP packets without the normal FC heartbeat. Restoring the UART5 MAVLink channel restored FC range and flow immediately, without changing the MTF-01P's saved sensor configuration.

SERIAL3_OPTIONS was temporarily changed 0→8 to receive on physical TX3, followed by a disarmed reboot and passive capture. That test also received zero bytes. The option was restored 8→0 and verified through another reboot. No half-duplex or sensor configuration heartbeat was used. UART3 remains MAVLink1 at 115200; RNGFND2 remains disabled because its input has not been verified.

The [full before/after delta](../../artifacts/indoor-repair/20260907/parameter-delta.json) additionally contains FC-managed boot/runtime counters, gyro startup offsets, barometer ground pressure and automatically restored compass declination. Those were not explicit calibration writes. Exact write journals are `stage1-plan.result.json` and `stage2-plan.result.json` in [the artifact directory](../../artifacts/indoor-repair/20260907/).

**GPS-free estimator and compass**

GPS1_TYPE and GPS2_TYPE remain 0. Existing EKF source 1 remains horizontal position=None, horizontal velocity=optical flow, vertical position=barometer, vertical velocity=None, yaw=compass. Downward range scales optical flow; it does not require GPS. The stored project origin and calibrations were preserved.

The HGLRC M100 houses the GPS and compass together, but the GPS uses UART6 and the QMC5883L compass uses I2C bus 0/address 0x0D. Disabling GPS does not disable the compass.

After repair, the FC reported `EKF3 IMU0 fusing optical flow` and `EKF3 IMU0 started relative aiding`. A 25-second disarmed audit with normal GCS heartbeats produced 26 EKF reports with flags 367: relative aiding present, constant-position mode absent. Range and optical-flow health bits were set. This is a live estimator result, not a flight or externally measured position-accuracy test.

The same audit requested normal pre-arm diagnostics without attempting to arm. Its only reported failure was `Check mag field`, with horizontal field error 113–117 mG exceeding the unchanged 100 mG threshold. Compass samples and health bits were present. Correct this by checking the aircraft away from laptop/steel/current-carrying wiring, then inspecting mounting and performing a proper physical calibration if it persists. The threshold was not relaxed and no compass check remains skipped.

An independent Pi↔FC verification at 16:31 CEST returned fresh disarmed heartbeats, parameter replies and range/flow data. The final recording at **17:01 CEST** again received these sensor streams and 40 EKF reports, all with flags 367. Battery voltage was **15.573–15.578 V**; the FC reported approximately 0.94 A and 76% remaining. That percentage is an analog-monitor consumed-charge estimate; individual cell condition and voltage/current calibration were not measured. See the [final telemetry summary](../../artifacts/indoor-repair/20260907/verified-pi/telemetry-summary.json). The [direct USB audit at 16:50 CEST](../../artifacts/indoor-repair/20260907/final-usb-audit.json) again received parameter replies, fresh disarmed heartbeats and EKF flags 367. Battery was 15.666–15.667 V (FC estimate 80%). An initial GCS-loss status cleared after normal GCS heartbeats resumed; the compass field check remained, with 111–121 mG horizontal error.

**Installed firmware verified**

The actual [ROMFS definition read from the FC](../../artifacts/indoor-repair/20260907/live-hwdef.dat) is byte-identical to the reviewed no-GPS build's resolved definition: 35,238 bytes, SHA-256 `d89b4db7acd2811284c420fb79f0750661f8dfca6865bedf1725a17dfac4babe`. Final effective values enable EKF3, MAVLink range/flow, optical-flow fusion and GUIDED_NOGPS. The local ELF/BIN/APJ also pass the existing manifest/linked-feature verifier. Matching ROMFS is not a readback hash of all executable flash.

No firmware flash was needed to restore this flow-based navigation path. The installed feature definition disables proximity and rangefinder-to-proximity support. A future forward obstacle-response implementation will need that support and suitable configuration after a working MT-15 receive path is established. The reviewed current image occupies 865,792 of 950,272 bytes; available space alone does not validate a changed build.

**Software fixes and deployment**

- [Flight controller code](../../ai_drone/flight/controller.py): preserve cleanup ownership after an ambiguous arm command; require a fresh matching disarmed heartbeat; reject unknown/invalid/unhealthy battery reports immediately; request LAND if public Loiter hold loses its expected mode. Existing downward-only altitude filtering remains tested. Bootstrap now requires the intended system/component and an ArduPilot quadrotor heartbeat instead of accepting the first heartbeat; the motor utility shares this identity check. DataFlash lookup ignores other vehicles' log entries.
- [Recorder](../../ai_drone/cli/record.py) and [tag/servo recorder](../../ai_drone/cli/tag_servo_record.py): synchronize stop/disarm with the GPIO command boundary, persist trigger intent before exposing work, protect startup cleanup, preserve signal handling through rest/detach/release, allow cleanup retry after GPIO-close failure, and report duration/timestamp failures accurately.
- Calibrated recording now treats only geometric no-solution/no-positive-depth poses as per-detection rejections. Rejected detections remain logged but cannot qualify for servo activation; invalid calibration, missing dependencies and detector failures remain fatal. Sensor component health/counts now use only the selected FC source, while raw telemetry keeps all received messages.
- [Servo utility](../../ai_drone/cli/servo.py): correct asymmetric pulse-width conversion and improve interruption/resource cleanup.
- [Deployment](../../ai_drone/link/deploy.py) and [SSH target selection](../../ai_drone/link/targets.py): use the known-working null SSH configuration by default and include the network selector/service in runtime deployment.
- [Configuration publication](../../ai_drone/config/sync.py): scope the Git commit to the generated snapshot pair, preserving unrelated staged changes. [Dual-network setup](../../scripts/setup-pi-dual-network.sh): reject profile-name collisions and stop/disable the conflicting single-radio boot selector before applying the two-interface topology. This dual-interface configuration was not applied to the current single-radio Pi.
- [Power-resilience setup](../../scripts/setup-pi-power-resilience.sh): recognize the active tag/servo service in its audit, fail if systemd enumeration fails, and describe the scope of the known-service check accurately.

The original Pi runtime is backed up as [pi-source-before.tar.gz](../../artifacts/indoor-repair/20260907/pi-source-before.tar.gz). The final tested runtime was installed offline at 17:00–17:01 CEST with the existing dependency lock. All 35 Python source hashes matched the laptop, and inspect/control/tag-servo CLI help and compilation passed. No control or actuator service was started. The Pi's actual gpiozero library passed the asymmetric pulse test using MockFactory/MockPWMPin only; no physical GPIO output was used. The Pi clock was six days behind; it was corrected against the host at 16:41 CEST and verified within one second. Earlier recordings retain their original incorrect Pi calendar timestamps. See the [final deployment result](../../artifacts/indoor-repair/20260907/verified-pi/pi-deployment-result.json).

The first 8-second camera recording stopped making image progress and needed interruption. A traced retry completed successfully; its observed cleanup delay was in file fsync. The recorder now uses asynchronous capture with a configurable 2-second frame timeout and checks cancellation/deadlines at intervals of at most 100 ms while waiting for a frame. Stalls produce a failed manifest with `camera_stalled`; abandoned completed requests are released. Camera-driver cancellation, encoder shutdown and durable filesystem writes are not guaranteed to finish within that frame timeout.

The **final deployed recording completed successfully in 8.031 seconds**, with **131 analyzed frames, 239 encoded frames and 1,116 telemetry messages**, including 159 downward-range and 159 optical-flow samples (about 19.8 Hz on the requested telemetry stream). Downward distance was 2–3 cm and flow quality ranged from 57 to 78. There were no forward-range samples. The FC remained disarmed and no physical servo output was enabled. The close-range view was featureless and contained no AprilTag, so this verifies image acquisition and detector initialization, not tag detection/pose accuracy or IMX500 neural inference. See the [final recording manifest](../../artifacts/indoor-repair/20260907/verified-pi/manifest.json) and [recording archive](../../artifacts/indoor-repair/20260907/verified-pi/pi-inspect-artifacts.tar.gz).

**Validation**

Final offline suite: **536 passed, 2 skipped** for unavailable laptop cv2/gpiozero, with the 2 SITL tests run separately. Ruff lint/format and diff checks passed. Both unavailable laptop dependency checks were additionally exercised on the Pi: gpiozero with mock pins, and [synthetic calibrated pose recovery using its installed OpenCV 4.11](../../artifacts/indoor-repair/20260907/verified-pi/pi-synthetic-pose.json). The synthetic pose test is not a physical camera calibration. See the [final pytest log](../../artifacts/indoor-repair/20260907/verified-offline-pytest.log).

Both exact pinned SITL tests were rerun after the final controller/bootstrap repairs and **passed in 108.53 seconds**, in a separate network namespace with only loopback available. The [final simulator result](../../artifacts/indoor-repair/20260907/verified-sitl/result.json) and source hashes identify the tested version. Both the normal hover/landing path and forced companion-loss LAND/disarm path passed.

The earlier post-repair simulator run also passed and printed these measured results:

- GPS-free production takeoff→Loiter→LAND: maximum horizontal drift 0.029 m and Loiter altitude 0.520 m.
- Forced companion loss: GCS failsafe selected LAND and finished disarmed; maximum horizontal drift 0.027 m, maximum altitude 0.530 m and touchdown speed 0.148 m/s.

These simulations validate the bounded hover and link-loss recovery paths, not MT-15 obstacle response, calibration or actual flight. Logs, JUnit results and flight recordings are in [the repair artifacts](../../artifacts/indoor-repair/20260907/).

**Pi USB networking**

Hotspot SSH at `192.168.4.1` remains verified and was used for deployment. The separate Pi USB Ethernet gadget enumerates but cannot transmit reliably: the host reports repeated `cdc_ether` transmit watchdog timeouts, 491 TX errors and failed ARP despite the correct `192.168.7.1→192.168.7.2` route. USB autosuspend was already disabled. A targeted NetworkManager reconnect restored the interface to active state but did not repair data transfer. A subsequent driver-rebind attempt stopped before any mutation because noninteractive host sudo requires a password. No whole-device USB reset was attempted; FC USB and eduroam were preserved. See [reconnect evidence](../../artifacts/indoor-repair/20260907/usb-network-reconnect.json) and [driver-rebind evidence](../../artifacts/indoor-repair/20260907/usb-network-driver-rebind.json). Physical Pi USB cable reconnection or an authorized host driver rebind remains necessary to test that path again.

**Remaining physical work**

1. With battery and USB disconnected, trace **MT-15 TX→FC RX3**, **MT-15 RX→FC TX3**, common ground and regulated 5 V. The old analog VTX connector has no RX3; the separate DJI connector provides it. Native forward MAVLink at 115200 was verified using the USB adapter, but never received through the FC in these checks. Compare the [supplied wiring diagram](../../../drone-sensor-diagram.png) with the [GN745 AIO V3 pinout](../../../ardupilot/libraries/AP_HAL_ChibiOS/hwdef/FlywooF745/GOKUGN745AIO_v3.0_Pinout.jpg). The electrical wiring cannot be repaired from software; no confirmation of physical reconnection was received during this session.
2. Resolve the compass field error through environment/mounting/calibration. GPS remains disabled; the compass is still needed by the configured yaw source.
3. Validate optical-flow axes/scale and camera geometry with suitable height, floor texture, lighting and controlled movement. A 2–3 cm stationary bench view is not sufficient. Servo software was repaired, but mechanical travel/feedback was not tested and no motors or physical servo were commanded.

The current application implements a guarded GPS-free hover sequence. Full room navigation, calibrated tag approach and obstacle coverage beyond a single forward beam remain separate work; this repair does not establish those capabilities.
