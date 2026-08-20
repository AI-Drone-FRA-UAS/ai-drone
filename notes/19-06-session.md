# Development session — 19 June 2026

Historical summary of the first complete developer-USB, Raspberry Pi UART, and
MTF-01P MAVLink bring-up. Current procedures live under `docs/`; live state is
recorded under `state/` and `params/`.

No arming, motor, movement, flight-mode, throttle, mission, or actuator command
was issued during this session.

## Result

- The developer machine received a FlywooF745 heartbeat over USB and completed
  a read-only `SYSID_THISMAV` request.
- The Pi communicated bidirectionally with the controller over physical UART4.
- MTF-01P range and optical-flow telemetry reached the Pi through ArduPilot.
- `drone-console` and `drone-health` were introduced and exercised.
- The command channel was proven with a safe `AUTOPILOT_VERSION` request and
  acknowledgement; flight control itself was deliberately not tested.

## Verified topology at the time

| Path | Connection |
| --- | --- |
| Developer machine → controller | FlywooF745 USB, MAVLink at 115200 |
| Pi → controller | Pi `/dev/serial0` ↔ FC UART4, MAVLink2 at 115200 |
| MTF-01P → controller | FC UART5, MAVLink at 115200 |

The working Pi UART wiring was:

```text
Pi pin 8  / GPIO14 / TXD -> FC R4
Pi pin 10 / GPIO15 / RXD <- FC T4
Pi pin 6  / GND           -> FC GND
```

TX and RX must cross. The Pi's red power wire was connected to a regulated 5 V
source, never raw battery voltage.

## Important findings

### Restricted hardware visibility

The controller initially appeared absent because the restricted command
environment did not expose host USB devices. Retrying with direct host-device
access showed USB ID `1209:5741`, `/dev/ttyACM0`, and the stable ArduPilot
`/dev/serial/by-id/...` link.

### MAVProxy dependencies

MAVProxy 1.8.74 needed `future` at runtime, and its default ADS-B module needed
Pillow. Both became declared project dependencies. Generated `mav.tlog`,
`mav.tlog.raw`, and `mav.parm` files were added to `.gitignore`.

### Pi UART wiring

The first `/dev/serial0` test received no bytes in either direction. Swapping
the green and blue signal wires on the Pi produced a controller heartbeat and
a successful `SYSID_THISMAV` response, proving bidirectional communication.

### Live parameters differed from the older backup

The 9 June backup had UART4 disabled/differently configured, while the live
controller on 19 June reported UART4 as MAVLink2 at 115200. This established
the rule that dated backups must not substitute for a fresh live export.

### Sensor transport

A ten-second disarmed sample produced 296 messages, including valid range and
optical-flow records. This proved transport and sensor power only; the
stationary sample did not validate optical-flow navigation, calibration, or
altitude hold.

### Command channel

The Pi sent `MAV_CMD_REQUEST_MESSAGE` for `AUTOPILOT_VERSION` and received an
accepted acknowledgement plus the requested message. This proved the outbound
MAVLink path without changing mode, parameters, armed state, or actuators.

## Safety blockers observed

The live controller then reported disabled arming checks, geofence, and GCS
failsafe settings. Those historical values must not be assumed current, but
they were sufficient to rule out flight testing during the session.

Before any later Pi-controlled flight, the session identified the need to:

1. restore all arming checks and resolve every failure;
2. define link-loss and pilot-override behavior;
3. configure a meaningful indoor boundary;
4. validate range/flow estimates and mounting;
5. exercise mode and setpoint behavior in simulation; and
6. progress through propeller-off and restrained tests.

## Troubleshooting lessons

| Symptom | Cause found | Resolution |
| --- | --- | --- |
| USB device absent | Restricted environment hid host devices | Retry hardware inspection with host access |
| `No module named future` | MAVProxy packaging omitted an import | Declare `future` |
| ADS-B module import failure | Pillow missing | Declare Pillow |
| `uv` cache read-only | Default cache unavailable | Use `UV_CACHE_DIR=/tmp/uv-cache` |
| Build expected `src/ai_drone` | Backend assumed a src layout | Configure `module-root = ""` |
| Pi UART received zero bytes | TX/RX signals reversed | Cross Pi TX→FC RX and Pi RX←FC TX |
| Wrong Pi hostname | Obsolete `seb-is-pm2` used | Use `seb-is-pm` |
| MAVProxy shutdown traceback | Logging thread shutdown behavior | Confirm process exit and port release |

## Follow-up documentation

- [Direct USB connection](../docs/DEVELOPER_MACHINE_DRONE_CONNECTION.md)
- [Flight-controller configuration](../docs/DRONE_CONFIGURATION.md)
- [Sensor recording](../docs/SENSOR_RECORDING.md)
- [Staged MAVLink control](../docs/PI_MAVLINK_CONTROL.md)
