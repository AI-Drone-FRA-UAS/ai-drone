# Connected Drone Configuration

Verified on 2026-06-09.

## Flight controller

- Flywoo GOKU GN745 AIO / FlywooF745 running ArduPilot Copter 4.6.3
- USB serial: `/dev/ttyACM0`
- Stable USB path:
  `/dev/serial/by-id/usb-ArduPilot_FlywooF745_200023000451333436353531-if00`
- Parameter backup: `params/flywoo-f745_copter-4.6.3_2026-06-09.param`
- `ARMING_CHECK=0` and `FENCE_ENABLE=0` in that backup; both require review
  before flight.

### Meaning of the disabled safety parameters

`ARMING_CHECK` controls the pre-arm gate. The live value `0` disables optional
pre-arm checks, so ArduPilot can permit arming despite problems such as an
uncalibrated/inconsistent IMU, unhealthy compass or GPS, bad battery level,
invalid parameters, missing rangefinder data, RC failure, or unavailable
logging. The normal safe value on Copter 4.6 is `1`, meaning perform all checks.
It should be restored only after each reported failure is diagnosed; the
individual warnings should not be bypassed to make the vehicle arm.

`FENCE_ENABLE` is independent: it turns the configured geofence behavior on or
off after the vehicle is operating. With `0`, ArduPilot will not enforce the
configured maximum-altitude, home-centered radius, or uploaded polygon fence,
and therefore will not run `FENCE_ACTION` on a breach. It does not prevent
arming and it is not collision avoidance.

The live values are `FENCE_TYPE=7` (maximum altitude, home circle, and
polygon/inclusion-exclusion fences), `FENCE_ACTION=1` (RTL, falling back to
LAND), `FENCE_ALT_MAX=100 m`, `FENCE_RADIUS=300 m`, and `FENCE_TOTAL=0`. Merely
changing `FENCE_ENABLE` to `1` would not create a useful indoor containment
area: there is no uploaded polygon, the limits are far larger than the hall,
and the circle/RTL behavior needs a reliable home position. Define and test the
intended indoor boundary and fallback action before enabling it.

## MicoAir MTF-01P

- Connected to ArduPilot `SERIAL5` / board UART5.
- `SERIAL5_PROTOCOL=1`, `SERIAL5_BAUD=115`, `SERIAL5_OPTIONS=1024`
- `FLOW_TYPE=5`, `RNGFND1_TYPE=10`
- Range: 1-800 cm, downward orientation (`RNGFND1_ORIENT=25`)
- EKF3 uses optical flow for horizontal velocity and rangefinder height.

The sensor requires drone battery power. `drone-lidar` does not power or
arm anything; it only requests telemetry from ArduPilot and saves received
range/flow messages.

## Raspberry Pi companion link

- ArduPilot port: `SERIAL4` / physical UART4 (`T4` and `R4`)
- `SERIAL4_PROTOCOL=2`, `SERIAL4_BAUD=115`, `SERIAL4_OPTIONS=0`
- Pi device: `/dev/serial0` -> `/dev/ttyAMA0`
- Pi pin 8 / GPIO14 / TXD -> FC `R4`
- Pi pin 10 / GPIO15 / RXD -> FC `T4`
- Pi pin 6 / GND -> FC GND
- Pi pin 2 / 5V is powered from the FC regulated 5V supply

Run `uv run drone-health` on the developer machine to verify both the direct
USB link and this Pi UART link.

## Cameras

The drone has two separate camera paths:

- RunCam Phoenix 2 analog FPV camera -> SpeedyBee TX800 at 5806 MHz.
- Raspberry Pi IMX500 AI Camera -> `modlib`/NanoDet processing on the Pi.

The repository only handles the Pi IMX500 path. It does not capture the
analog FPV signal on the laptop. No camera tilt motor or ArduPilot gimbal is
currently configured.

For the complete Linux USB and MAVProxy procedure, see
[Developer Machine Drone Connection](DEVELOPER_MACHINE_DRONE_CONNECTION.md).
