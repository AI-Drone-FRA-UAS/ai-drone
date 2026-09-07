# Documentation index

The maintained code, current operating instructions and historical project
presentation now live in one repository. Hardware status comes from the newest
`state/` capture and `params/` export, not from the August poster or old branches.
No full autonomous indoor navigation or AprilTag approach-and-drop flight has
been demonstrated by the maintained workflow.

## Start here

| Document | Contents |
| --- | --- |
| [Project README](../README.md) | Installation, commands and development checks |
| [Hardware inventory](drone-project.md) | Project equipment and sensor roles |
| [Software architecture](SOFTWARE_ARCHITECTURE.md) | Maintained modules and command boundaries |
| [Flight-controller configuration](DRONE_CONFIGURATION.md) | Configuration provenance and latest checks |

## Sensors, recording and actuators

| Document | Contents |
| --- | --- |
| [Sensor recording](SENSOR_RECORDING.md) | Disarmed camera and MAVLink capture |
| [AprilTag architecture](APRILTAG_MISSION.md) | Detection and geometry; navigation limits |
| [Armed tag/servo recording](ARMED_TAG_SERVO_RECORDING.md) | Explicit recording and bounded pulse workflow |
| [Payload mechanism](PAYLOAD_DROP.md) | BCM12 servo, power and physical verification |
| [Frame and prints](FRAME_AND_3D_PRINTS.md) | Files under `hardware/3d-prints/` |
| [Motor bench procedure](BENCH_MOTOR_TEST.md) | Guarded propeller-free motor testing |

## Flight and Raspberry Pi

| Document | Contents |
| --- | --- |
| [Staged flight testing](PI_MAVLINK_CONTROL.md) | Maintained GuidedNoGPS/Loiter/LAND workflow |
| [ArduCopter 4.7 review](ARDUCOPTER_4_7_NOGPS_LOITER.md) | Pinned firmware and no-GPS prerequisites |
| [Developer connection](DEVELOPER_MACHINE_DRONE_CONNECTION.md) | FC USB and Pi access |
| [Pi networking](pi-networking.md) | Tailscale, campus network and fallback hotspot |
| [Pi USB SSH](RPI_ZERO2W_USB_SSH_SETUP.md) | USB gadget access |
| [Power-loss recovery](PI_POWER_RESILIENCE.md) | Maintenance and interrupted-upgrade recovery |

## History and presentation

| Document | Contents |
| --- | --- |
| [19 June bring-up](../notes/19-06-session.md) | Historical USB/UART/sensor observations |
| [Branch archive](../notes/archive/README.md) | Source SHAs and disposition of earlier implementations |
| [August flight records](../notes/archive/preflight-and-nogps-takeoff/README.md) | Incident reports and original log evidence, not operating procedures |
| [Project poster](poster/README.md) | Preserved August presentation and print PDFs |
| [Project site](../site/README.md) | Build and preview the current docs plus clearly dated historical result pages |

The site renders maintained Markdown and a few presentation pages. Build it
with `uv run --group docs python site/build.py`; append `--serve` to preview it
locally. Historical statements remain dated and do not establish today's
hardware health or permission to operate an actuator.
