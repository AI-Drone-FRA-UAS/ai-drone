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
| Forward MicoAir MT-15 | FC UART3 assigned; physical receive path unverified | Sensor configured for MAVLink/APM, 115200 baud, forward orientation |
| IMX500 camera | Pi CSI | Picamera2/libcamera |
| Payload servo | Pi BCM12 | Direct GPIO; not a flight-controller servo output |

The Pi UART wiring is:

```text
Pi pin 8  / GPIO14 / TXD -> FC R4
Pi pin 10 / GPIO15 / RXD <- FC T4
Pi pin 6  / GND           -> FC GND
```

The reviewed FlywooF745 firmware has five MAVLink channels. For this topology,
disable unused UART1 so USB, UART2, UART3, UART4 and UART5 receive those five
channels. Enabling an additional earlier UART can silently exclude UART5 from
MAVLink processing even when `SERIAL5_PROTOCOL=1`. `MAVn_OPTIONS` follows the
allocated MAVLink instance order, so review its mapping whenever serial
protocol assignments change. Dated repair records contain the applied values.

Confirm power capacity and the exact board revision before relying on any
wiring description. Dated state captures record what was observed, not a
guarantee that the physical build is unchanged.

## Arming checks

ArduCopter 4.7 replaced `ARMING_CHECK` with the inverse
`ARMING_SKIPCHK` bitmask. `ARMING_SKIPCHK=0` means no configurable pre-arm
checks are skipped; every set bit skips its corresponding check category.

Never arm, fly, or run the motor utility unless the live value is exactly `0`
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

The forward MT-15 must remain distinct from the downward sensor. Verify
its physical UART, regulated power, outgoing MAVLink sensor ID/orientation, and
firmware support before writing `SERIALx`, `RNGFND2`, proximity, or avoidance
parameters. One forward beam is not full obstacle avoidance.

The old analog VTX connector has TX3 and video connections but no RX3. MT-15 TX
must reach RX3 on the separate DJI connector; MT-15 RX connects to TX3. Verify
common ground and regulated 5 V rather than relying on old wire colours. The
MT-15 firmware tested on 2026-09-07 saves sensor ID1 but emits MAVLink ID0;
the pinned ArduPilot rangefinder backend separates readings by orientation.

See [sensor recording and wiring](SENSOR_RECORDING.md).
The reviewed ArduCopter 4.7 EKF correction, comparison evidence, and SITL
acceptance result are in the
[no-GPS Loiter review](ARDUCOPTER_4_7_NOGPS_LOITER.md).

## Capture the live configuration

`drone-config-sync` reads the complete indexed MAVLink parameter set through
the Pi. It does not send `PARAM_SET`, change mode, arm, or drive an actuator.

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm.tail59e6a4.ts.net \
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
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm.tail59e6a4.ts.net \
  uv run drone-config-sync --publish
```

Git credentials remain on the developer machine and are not copied to the Pi.

For direct USB inspection and MAVProxy troubleshooting, see
[Developer machine connection](DEVELOPER_MACHINE_DRONE_CONNECTION.md).
