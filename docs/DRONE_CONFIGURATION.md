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

## MicoAir MTF-01P

- Connected to ArduPilot `SERIAL5` / board UART5.
- `SERIAL5_PROTOCOL=1`, `SERIAL5_BAUD=115`, `SERIAL5_OPTIONS=1024`
- `FLOW_TYPE=5`, `RNGFND1_TYPE=10`
- Range: 1-800 cm, downward orientation (`RNGFND1_ORIENT=25`)
- EKF3 uses optical flow for horizontal velocity and rangefinder height.

The sensor requires drone battery power. `test_lidar.py` does not power or
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
