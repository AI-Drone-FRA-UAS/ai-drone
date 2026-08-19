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

`ARMING_CHECK` controls which optional pre-arm categories gate an arm request.
It is a bitmask with one deliberately special value: `1` means **run every
available check**, not merely "enable bit 0". Other non-zero masks select
categories such as barometer, compass, GPS, INS, RC, board voltage, battery,
logging, system, mission, and configured rangefinders. The live value `0`
skips those optional categories, so ArduPilot can permit arming despite serious
calibration, configuration, sensor, power, RC, or logging problems. Some
mandatory checks still run even at `0`; that does not make `0` safe for flight.

Setting `ARMING_CHECK=1` does not arm the aircraft, spin motors, calibrate a
sensor, or prove the physical build is safe. It restores the software gate:
ArduPilot runs the checks while disarmed and again when arming is requested,
blocks the request on failure, and reports `PreArm:`/`Arm:` status text. Set it
to `1`, read and resolve every reported failure, and do not replace it with a
partial mask merely to make the vehicle arm.

The checks only cover sensors and features that ArduPilot knows are configured.
They cannot detect the loose forward-facing Pi camera, an unplugged Pi GPIO
servo, absent propellers/guards, unsafe surroundings, or an unconfigured and
disconnected forward MT-15. Ongoing EKF, battery, RC, and link failsafes are
also separate from this one pre-arm parameter.

References: [ArduPilot pre-arm safety checks](https://ardupilot.org/copter/docs/common-prearm-safety-checks.html)
and the [Copter 4.6 parameter list](https://ardupilot.org/copter/docs/parameters-Copter-stable-V4.6.0.html).

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
