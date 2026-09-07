**Direct USB connection verified — 7 September 2026, 13:49 CEST**

The project flight controller now enumerates directly on the laptop as `1209:5741`, ArduPilot FlywooF745, serial `200023000451333436353531`. Kernel enumeration occurred at 13:48:11 CEST on USB port 3-3, using `cdc_acm` and `/dev/ttyACM0`.

Stable path: `/dev/serial/by-id/usb-ArduPilot_FlywooF745_200023000451333436353531-if00`.

The port was unowned before opening. Direct MAVLink heartbeats and requested SYS_STATUS, BATTERY_STATUS, and AUTOPILOT_VERSION responses verified communication with target 1/1. The FC remained **disarmed in STABILIZE**, reporting ArduCopter 4.7.0 / `1511f271`. The port was closed after inspection. No Pi connection was needed for this check.

Battery telemetry now reports **12.626 V**, **0.92 A**, 592 mAh consumed, and an estimated 82% remaining. The voltage is below the application's 14.4 V flight-test minimum and lower than the earlier 13.45 V observation. As established in the earlier live parameter read, the percentage is a consumed-charge estimate from an analog monitor and does not establish actual charge. No battery or arming parameters were reread or changed in this USB check.

Only three `MAV_CMD_REQUEST_MESSAGE` commands were sent. No actuator, arm/disarm, mode-change, reboot, or parameter-write commands were sent. [Raw USB results](../../artifacts/sensor-recordings/usb-check-20260907T114922Z/result.json).
