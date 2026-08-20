# Disarmed all-sensor recording and wiring

The `drone-record` command creates a synchronized bench dataset without arming
the vehicle or commanding any actuator:

```bash
cd ~/ai-drone
.venv/bin/python -m ai_drone.cli.record --duration 15
```

From the developer machine:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-deploy --record --duration 15
```

If Tailscale is offline and the laptop is joined to `AI-Drone-Zero`, replace
`seb-is-pm` with the hotspot fallback `192.168.4.1`.

The duration begins after the MAVLink heartbeat, camera setup, and configurable
camera warm-up have completed. The output directory contains:

- `camera.h264`: hardware-encoded 1280x960 camera video
- `camera.pts`: encoder presentation timestamps
- `camera.jsonl`: frame timestamps, exposure metadata, and AprilTag detections
- `telemetry.tlog`: timestamped raw MAVLink packets
- `telemetry.jsonl`: every decoded MAVLink message with source IDs and timing
- `first-frame.jpg` and `last-frame.jpg`: quick view of camera aim/focus
- `manifest.json`: requested/actual duration, counts, topology, and safety result

Without a camera calibration, tags are decoded but no metric pose is reported.
Supply `--calibration FILE --tag-size METRES` only after rigid mounting, focus,
and calibration at the selected resolution.

## Verified connection topology

| Device/data | Physical connection | What the Pi records |
| --- | --- | --- |
| IMX500 AI Camera | Directly to Pi CSI connector | H.264 video, frame metadata, and tag detections |
| MicoAir MTF-01P | Flight controller UART5 (`SERIAL5`, MAVLink1, 115200) | Range and optical flow forwarded by ArduPilot over the companion link |
| Flight-controller IMU, barometer, compass, GPS, battery, EKF and RC state | Directly to the FlywooF745 | Requested MAVLink telemetry over UART4 |
| Raspberry Pi companion link | Pi GPIO14/15 `/dev/serial0` to FC R4/T4 (`SERIAL4`, MAVLink2, 115200) | All FC telemetry in `.tlog` and `.jsonl` |
| Servo | Separate guarded utility targets Pi BCM12 directly | Not driven by this program; FC output telemetry may still be recorded |

The recorder requests bounded message rates that fit the verified 115200-baud
UART4 link. This activates telemetry streaming, not motors. Sensors that require
the flight battery, including the MTF-01P, must already be powered.

## Planned forward MicoAir MT-15

The proposed design uses a full-duplex FlywooF745 UART rather than connecting
the MT-15 to the Pi. A flight-controller connection keeps the immediate
measurement available if the companion process crashes. Confirm the live UART
allocation before selecting a port.

Proposed wiring, to be verified against the exact FC board revision before
soldering:

```text
MT-15 5V  -> regulated FC 5V
MT-15 GND -> FC GND
MT-15 TX  -> selected FC RX
MT-15 RX  -> selected FC TX
```

MicoAir sensors use 3.3 V UART logic. Configure the MT-15 through MicoAssistant
for ArduPilot MAVLink at 115200, forward orientation, and a MAVLink system ID
different from 1. After selecting the port, the likely starting point is:

```text
SERIALx_PROTOCOL = 1
SERIALx_BAUD = 115
SERIALx_OPTIONS = 1024
RNGFND2_TYPE = 10
RNGFND2_ORIENT = 0
```

Do not apply these parameters until the sensor is physically connected and its
outgoing `DISTANCE_SENSOR.orientation` has been confirmed. ArduPilot's MAVLink
rangefinder backend accepts a measurement only when the packet orientation
matches `RNGFNDx_ORIENT`.

For use as an avoidance source, the firmware must expose and correctly
configure the applicable proximity parameters. Confirm firmware support before
relying on automatic braking. A single forward beam is a stop-ahead sensor, not
safe 360-degree hall navigation.

## Safety behavior

The recorder:

- refuses to start if the vehicle heartbeat is armed;
- stops if the vehicle heartbeat changes to armed;
- contains no arm, disarm, mode-change, motor, throttle, RC override, mission,
  or servo command path;
- records `SERVO_OUTPUT_RAW` only as telemetry;
- does not enable disarmed onboard DataFlash logging or alter FC parameters.
