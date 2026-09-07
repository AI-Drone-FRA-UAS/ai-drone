**Connection and battery recheck — 7 September 2026, 13:10–13:11 CEST**

This recheck supersedes the earlier unavailable-FC result. The Pi now has a working bidirectional MAVLink connection to the project FC through `/dev/serial0` at 115200 baud. The selected source is ArduPilot quadrotor system/component 1/1. Eleven heartbeats and 55 acknowledgments were received during the approximately ten-second sample. The FC remained **disarmed in STABILIZE**. It reports ArduCopter 4.7.0, custom version `1511f271`.

The direct laptop-to-FC USB device was still absent at the start of this recheck. The successful path was laptop → Pi hotspot SSH → Pi UART → FC. The host Wi-Fi connection was restored after each SSH check. No cause for the earlier loss of FC communication was established; this report only establishes that it worked during the new sample.

| Live battery observation | Result |
| --- | --- |
| Pack voltage | **13.450–13.457 V** across five readings |
| Current | **0.82–0.83 A** |
| Reported remaining | 98% |
| Integrated consumption | 33–35 mAh |
| Battery monitor health bit | Present, enabled, healthy |
| FC charge-state code | 1, OK |
| Battery temperature, individual-cell voltages, time remaining | Not available |

The live `BATT_MONITOR=4` confirms analog voltage/current monitoring and `BATT_CAPACITY=3300` sets the capacity used by the consumed-charge estimate. The 98% result is not an independent measurement of state of charge and must not be treated as proof of a nearly full pack. The first voltage-array entry is the total pack voltage, not an individual cell. Voltage/current calibration was read but not verified against an external meter (`BATT_VOLT_MULT≈10.9`, `BATT_AMP_PERVLT=28.5`).

The measured pack voltage is below the application's default **14.4 V** flight-test minimum in `ai_drone/cli/control.py:214`. The FC itself currently has `BATT_ARM_VOLT=0`, `BATT_LOW_VOLT=10.5`, `BATT_CRT_VOLT=0`, and both `BATT_FS_LOW_ACT=0` and `BATT_FS_CRT_ACT=0`. Its reported OK state therefore does not establish suitability for the project's flight test; low/critical battery failsafe actions are disabled.

**Live pre-arm configuration change:** `ARMING_SKIPCHK=4`. Bit 2 is Compass in the pinned ArduCopter 4.7 source (`libraries/AP_Arming/AP_Arming.cpp:195–201`), so **compass pre-arm checks are being skipped**. This differs from the saved 25 August value of 0. The current overall pre-arm health bit was true, but cannot demonstrate that the skipped compass check would pass. No parameter was changed during either inspection. Under this repository's operating rules, flight or motor testing must not proceed with a nonzero skip mask.

| Sensor/interface | New evidence |
| --- | --- |
| Downward rangefinder | Five source-filtered orientation-25 readings, 2–3 cm; FC laser-position health bit true |
| Optical flow | Five readings, quality 60–67; FC optical-flow health bit true |
| Gyroscope/accelerometer | Five IMU readings; respective health bits true; resting acceleration near 1 g |
| Compass | Changing magnetometer readings and health bit true; compass pre-arm check is bypassed, so this is reachability only |
| Barometer | Five pressure readings near 1004.57 hPa; pressure health bit true |
| EKF | Five reports with flags 367 |
| RC | Five reports, zero channels |
| GPS | No GPS_RAW_INT response observed |
| Camera | Passed the earlier image-acquisition test; not retested in this FC/battery sample |
| Servo | No actuation or presence/position feedback test; unchanged |

These on-demand sensor samples establish a working communication path and reported health. They do not measure native sensor update rates, validate range/flow calibration, or establish flight readiness. Downward range was at the bottom of its reported interval; no controlled lift/translation was performed.

The Pi again reported `get_throttled=0x0`, temperature 50.5°C. Its clock remains unsynchronized and approximately six days behind the host. Use [host metadata](../../artifacts/sensor-recordings/recheck-20260907T111038Z/host-metadata.json) for the observation date. Raw [FC telemetry](../../artifacts/sensor-recordings/recheck-20260907T111038Z/fc-telemetry.json), [live battery/arming parameters](../../artifacts/sensor-recordings/recheck-20260907T111038Z/live-battery-parameters.json), and [Pi health](../../artifacts/sensor-recordings/recheck-20260907T111038Z/pi-health.txt) are retained locally.

Only `MAV_CMD_REQUEST_MESSAGE` and `PARAM_REQUEST_READ` were sent, after observing a selected disarmed heartbeat. No GCS heartbeat, arm, disarm, mode change, motor, throttle, RC override, servo, mission-start, or parameter-write command was sent.
