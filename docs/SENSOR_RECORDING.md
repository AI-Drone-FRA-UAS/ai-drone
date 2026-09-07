# Disarmed sensor inspection and recording

`drone-inspect` reports each available component and saves a bench dataset
without arming the vehicle or commanding any actuator:

```bash
cd ~/ai-drone
uv run drone-inspect --duration 15
```

From the developer machine:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-deploy --run inspect -- --duration 15
```

If Tailscale is offline and the laptop is joined to `AI-Drone-Zero`, replace
`seb-is-pm` with the hotspot fallback `192.168.4.1`.

Camera and MAVLink are independent: either can be unavailable without failing
the inspection. The output directory contains the files produced by the live
collectors:

- `camera.h264`: hardware-encoded 1280x960 camera video
- `camera.pts`: encoder presentation timestamps
- `camera.jsonl`: frame timestamps, exposure metadata, and AprilTag detections
- `telemetry.tlog`: timestamped raw MAVLink packets
- `telemetry.jsonl`: every decoded MAVLink message with source IDs and timing
- `first-frame.jpg` and `last-frame.jpg`: quick view of camera aim/focus
- `manifest.json`: observed component states, counts, rates, and safety result

Without a camera calibration, tags are decoded but no metric pose is reported.
Supply `--calibration FILE --tag-size METRES` only after rigid mounting, focus,
and calibration at the selected resolution.

## Documented connections

The wiring and UART allocation rules live in
[flight-controller configuration](DRONE_CONFIGURATION.md). Confirm the current
physical build and component health with a fresh inspection; this table is not
a live connection result.

| Device/data | Physical connection | What the Pi records |
| --- | --- | --- |
| IMX500 AI Camera | Directly to Pi CSI connector | H.264 video, frame metadata, and tag detections |
| MicoAir MTF-01P | Flight controller UART5 (`SERIAL5`, MAVLink1, 115200) | FC-published range and optical flow over the companion link |
| Flight-controller IMU, barometer, compass, GPS, battery, EKF and RC state | Directly to the FlywooF745 | Requested MAVLink telemetry over UART4 |
| Raspberry Pi companion link | Pi GPIO14/15 `/dev/serial0` to FC R4/T4 (`SERIAL4`, MAVLink2, 115200) | All FC telemetry in `.tlog` and `.jsonl` |
| Servo | Separate guarded utility targets Pi BCM12 directly | Not driven by this program; FC output telemetry may still be recorded |

The inspector requests bounded message rates that fit the verified 115200-baud
UART4 link. This activates telemetry streaming, not motors. Sensors that require
the flight battery, including the MTF-01P, must already be powered.

## Forward MicoAir MT-15

The MT-15 is the forward sensor. Its native ArduPilot MAVLink output was
verified through a USB-UART adapter on September 7, but the subsequent FC
receive path remained unverified. See the [direct sensor configuration
record](../state/2026-09-07/mt15-direct-usb-configuration.md) and the later
[integration repair result](../state/2026-09-07/indoor-repair.md).

The inspector distinguishes forward `DISTANCE_SENSOR.orientation=0` from
downward orientation25. Component health and counts require messages from the
selected flight controller; raw telemetry retains other sources for diagnosis.
Without a confirmed FC-published forward stream, the component is `no_data` and
flight logic must not depend on it. Sensor ID alone cannot distinguish these
two sensors: the tested MT-15 firmware emits MAVLink ID0 despite saving ID1.

Use the [canonical wiring and configuration guide](DRONE_CONFIGURATION.md#sensors)
before changing parameters. It explains RX3 versus the old analog VTX cable,
the five-channel MAVLink limit, and the need to preserve the downward sensor
and Pi connections. UART allocation and `MAVn_OPTIONS` mapping must be reviewed
together; a generic serial-port recipe is insufficient.

For use as an avoidance source, the firmware must expose and correctly
configure the applicable proximity parameters. Confirm firmware support before
relying on automatic braking. A single forward beam is a stop-ahead sensor, not
safe 360-degree hall navigation.

## Safety behavior

The inspector:

- refuses to start if the vehicle heartbeat is armed;
- stops if the vehicle heartbeat changes to armed;
- contains no arm, disarm, mode-change, motor, throttle, RC override, mission,
  or servo command path;
- records `SERVO_OUTPUT_RAW` only as telemetry;
- does not enable disarmed onboard DataFlash logging or alter FC parameters.
