# Hardware inventory and project objective

This is a stable inventory and requirements summary, not a live-state record or
operating procedure. Check the newest capture under `state/` and parameter dump
under `params/` before hardware, configuration, motor, or flight work.

## Objective

Develop an indoor aircraft that can eventually:

1. hold altitude and horizontal position using ArduPilot, downward range, and
   optical flow;
2. detect floor-mounted AprilTags with the Pi camera;
3. approach a selected tag using calibrated geometry;
4. release a payload through a guarded servo mechanism; and
5. search a bounded space only after suitable localization and obstacle
   sensing have been integrated.

Disarmed sensing and software implementation do not establish flight readiness.
Follow the staged procedures in [MAVLink control](PI_MAVLINK_CONTROL.md) and
the [AprilTag mission architecture](APRILTAG_MISSION.md).

## Aircraft

| Item | Model/details |
| --- | --- |
| Frame | SpeedyBee BEE35 Pro 3.5-inch CineWhoop |
| Flight controller | Flywoo GOKU GN745 AIO / STM32F745 / 45 A AM32 ESC |
| Firmware baseline | Official ArduCopter 4.7.0, commit `1511f271` |
| Motors | Four Emax Eco II 2004, 3000 KV |
| Propellers | Gemfan D90-5, 3.5-inch ducted five-blade |
| GPS/compass | HGLRC M100 |
| Receiver | Radiomaster XR4 Gemini Xrossband ELRS |
| FPV camera | RunCam Phoenix 2 analog, removed according to the user on 2026-09-07 |
| Video transmitter | Historical SpeedyBee TX800; its FC cable was reused for the forward sensor |

Radio binding credentials are intentionally not stored in the repository.

## Companion computing and sensing

| Item | Intended role |
| --- | --- |
| Raspberry Pi Zero 2 WH | MAVLink companion computer |
| Raspberry Pi IMX500 AI Camera | AprilTag detection and recording |
| MicoAir MTF-01P | Downward range and optical flow |
| MicoAir MT-15 | Forward lidar, ArduPilot MAVLink at 115200 baud; verified on FC T3 with UART3 RX/TX swap, rangefinder instance 2 |
| MacroSilicon MS210x USB grabber | Optional analog FPV capture |
| CP2102 USB-UART adapter | Sensor configuration |

The mission camera should be rigidly mounted downward, focused at the intended
working range, strain-relieved, and calibrated at its operating resolution.
Camera mounting and calibration are live-state facts and must be confirmed
rather than inferred from this inventory.

The HGLRC M100 contains both GPS and an external compass. The GPS uses FC
UART6; the compass uses I2C. Indoor optical-flow operation leaves GPS disabled
while retaining the compass as the EKF heading source. See the latest dated
state record for sensor health and any remaining calibration problems.

## Payload mechanism

Two micro servos are available for a custom release mechanism. The maintained
utility targets Pi BCM12 directly. Before actuation, remove propellers, secure
the aircraft, verify regulated power and common ground, establish safe pulse
limits, clear the linkage, and use the CLI's exact confirmation gate.

The historical MS18-F reference range was 900–2100 µs with 1500 µs neutral,
but datasheet values are not a substitute for cautious mechanism-specific
calibration. Never drive a loaded servo from a supply that can brown out the
Pi.

## Ground equipment and tools

| Category | Equipment |
| --- | --- |
| Pilot/video | Radiomaster GX12 transmitter, Skyzone Cobra X goggles |
| Batteries | Three flight packs; 18650 cells for controller/goggles |
| Charging | SkyRC B6neo+, 27 W USB-C supply, 20,000 mAh power bank |
| Fabrication | Tinkercad, UltiMaker Cura, 3D printer access |
| Fasteners | M3×9 mm and M3×12 mm screws/standoffs |
| Safety/service | Smoke stopper, SpeedyBee Adapter V3, hand tools |
| Spares/storage | Replacement props, 32 GB microSD, card reader, carry case |

Printable airframe assets are indexed in [hardware/README.md](../hardware/README.md).

## Integration rules

- Keep the MTF-01P downward-facing and unobstructed.
- Record the camera-to-body transform and payload release offset.
- Confirm regulated power budgets for the Pi, camera, sensors, and servo.
- Keep companion, RC, GPS, VTX, and sensor UART assignments distinct.
- Do not claim autonomous hall navigation from downward flow/range plus a
  single forward range beam; wider obstacle sensing and localization are
  required.
