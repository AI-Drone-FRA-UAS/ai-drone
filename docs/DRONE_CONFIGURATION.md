# Flight-controller configuration

This document records stable topology and configuration rules. It is not the
source of truth for live parameter values. Before hardware or flight work,
inspect the newest file under `params/` and its matching capture under
`state/`.

## Hardware topology

| Component | Connection | Protocol |
| --- | --- | --- |
| Flywoo GOKU GN745 AIO | Developer USB | MAVLink, usually `/dev/serial/by-id/...` |
| Raspberry Pi companion | FC UART4 ↔ Pi `/dev/serial0` | MAVLink2, 115200 baud |
| MicoAir MTF-01P | FC UART5 | MAVLink1, 115200 baud |
| IMX500 camera | Pi CSI | Picamera2/libcamera |
| Payload servo | Pi BCM12 | Direct GPIO; not a flight-controller servo output |

The Pi UART wiring is:

```text
Pi pin 8  / GPIO14 / TXD -> FC R4
Pi pin 10 / GPIO15 / RXD <- FC T4
Pi pin 6  / GND           -> FC GND
```

Confirm power capacity and the exact board revision before relying on any
wiring description. Dated state captures record what was observed, not a
guarantee that the physical build is unchanged.

## Arming checks

`ARMING_CHECK=1` has special meaning in ArduPilot: run every available
configurable pre-arm check. `0` skips those categories, while other non-zero
values select only a subset.

Never arm, fly, or run the motor utility unless the live value is exactly `1`
and every reported `PreArm:` or `Arm:` failure has been resolved. Restoring the
checks does not itself prove the airframe, camera, payload, surroundings, or
failsafes safe.

Reference: [ArduPilot pre-arm safety checks](https://ardupilot.org/copter/docs/common-prearm-safety-checks.html).

## Geofence

`FENCE_ENABLE` is independent of arming checks. Enabling it only activates the
configured fence types and breach action; it does not create a useful indoor
boundary or provide collision avoidance.

Before using a fence indoors, define and test an appropriate boundary,
localization source, and recovery action. Do not blindly enable historical
100 m altitude or 300 m radius settings.

## Sensors

The MTF-01P provides downward range and optical flow. Historical captures used
`FLOW_TYPE=5`, `RNGFND1_TYPE=10`, and downward orientation, but transport data
alone does not validate mounting, calibration, floor texture, lighting, EKF
quality, or altitude hold.

A planned forward MT-15 should remain distinct from the downward sensor. Verify
its physical UART, regulated power, outgoing MAVLink sensor ID/orientation, and
firmware support before writing `SERIALx`, `RNGFND2`, proximity, or avoidance
parameters. One forward beam is not full obstacle avoidance.

See [sensor recording and wiring](SENSOR_RECORDING.md).

## Capture the live configuration

`drone-config-sync` reads the complete indexed MAVLink parameter set through
the Pi. It does not send `PARAM_SET`, change mode, arm, or drive an actuator.

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-config-sync
```

The command writes:

```text
params/flywoo-f745-live-YYYY-MM-DD.param
state/YYYY-MM-DD/drone-config.json
```

It retries missing indexes, rejects duplicate names/indexes, and verifies the
parameter-file checksum. To commit exactly the generated pair from a clean
worktree and push the current branch:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-config-sync --publish
```

Git credentials remain on the developer machine and are not copied to the Pi.

For direct USB inspection and MAVProxy troubleshooting, see
[Developer machine connection](DEVELOPER_MACHINE_DRONE_CONNECTION.md).
