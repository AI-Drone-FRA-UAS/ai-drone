# Documentation map

Start with [the project overview](../README.md). Maintained guides describe
interfaces, procedures and limits; dated captures describe what was actually
observed. Check the current aircraft before acting on either.

## Operation

| Guide | Scope |
| --- | --- |
| [Sensor inspection and recording](SENSOR_RECORDING.md) | Disarmed camera/MAVLink recording, outputs and component health |
| [FC checks](DRONE_CHECK.md) | Read-only firmware, configuration and live sensor checks |
| [GPS-free hover](PI_MAVLINK_CONTROL.md) | Guarded takeoff, optical-flow Loiter, landing and required validation |
| [Bench motor test](BENCH_MOTOR_TEST.md) | Explicit propeller-off motor/ESC checks |
| [Armed tag/servo recording](ARMED_TAG_SERVO_RECORDING.md) | Separate actuator workflow and mechanism calibration |
| [AprilTag mission design](APRILTAG_MISSION.md) | Planned tag approach/release and navigation limitations |

Run `uv run <command> --help` for current options. A documented workflow does
not mean its live hardware prerequisites or flight validation are complete.

## Connections and Raspberry Pi

| Guide | Scope |
| --- | --- |
| [Pi networking](pi-networking.md) | Client networks, fallback hotspot, boot selection and dual interfaces |
| [Pi USB Ethernet/SSH](RPI_ZERO2W_USB_SSH_SETUP.md) | Developer-host gadget networking on Linux, Windows and macOS |
| [Direct FC USB](DEVELOPER_MACHINE_DRONE_CONNECTION.md) | Controller serial access and passive inspection without the Pi |
| [Pi power resilience](PI_POWER_RESILIENCE.md) | Controlled upgrades, recording durability and boot-service checks |
| [Connection and shutdown status](POWER_CONTROL.md) | Check connections and prepare battery or USB removal |

Deployment commands are introduced in [the root overview](../README.md#start-here).
Pi USB Ethernet and flight-controller USB are different devices and protocols.

## Hardware, configuration and firmware

| Guide | Scope |
| --- | --- |
| [Hardware inventory](drone-project.md) | Project components and intended roles |
| [FC configuration](DRONE_CONFIGURATION.md) | Canonical UART/wiring rules, MAVLink allocation, arming checks and capture |
| [Hardware assets](../hardware/README.md) | Original 3MF projects and STL print exports |
| [Frame and print guide](FRAME_AND_3D_PRINTS.md) | Frame construction and source files |
| [Payload mechanism](PAYLOAD_DROP.md) | BCM12 servo, power and physical verification |
| [Firmware artifacts](../firmware/README.md) | Reviewed overlays, manifest and post-flash gates |
| [ArduCopter 4.7 no-GPS review](ARDUCOPTER_4_7_NOGPS_LOITER.md) | EKF rationale, exact build and SITL acceptance procedure |

## Dated evidence and history

- [9 September firmware and power update](../state/2026-09-09/firmware-and-power.md):
  installed custom ArduCopter 4.7.1, matched simulation and connection/shutdown checks.
- [9 September startup melody](../state/2026-09-09/startup-tone.md):
  WAV 34 adapted and verified on all four AM32 ESCs.
- [9 September MT-15 integration](../state/2026-09-09/mt15-integration.md):
  verified forward range through FC and Pi after UART3 pin swapping, with
  [parameters captured at integration](../params/flywoo-f745-live-2026-09-09.param).
- [7 September 2026 repair result](../state/2026-09-07/indoor-repair.md):
  completed software/FC repairs, verification and remaining physical work.
- [7 September initial analysis](../state/2026-09-07/README.md): earlier findings
  and attempts, retained with their original chronology.
- [MT-15 direct-USB configuration](../state/2026-09-07/mt15-direct-usb-configuration.md):
  verified sensor output and its ID quirk, separate from FC reception.
- [7 September FC parameters](../params/flywoo-f745-live-2026-09-07.param) and
  [capture metadata](../state/2026-09-07/drone-config.json).
- [25 August firmware/flow verification](../state/2026-08-25/README.md),
  [19 August capture](../state/2026-08-19/README.md), and
  [18 August capture](../state/2026-08-18/README.md).
- [Branch archive](../notes/archive/README.md), with original commit provenance and incident evidence.
- [Historical poster](poster/README.md) and [project site](../site/README.md).
- [Historical notes](../notes/README.md), including the archived August 25
  handoff formerly named `CURRENT.MD`.
- [Retired implementation archive](../attic/README.md), excluded from runtime
  deployment and active checks.

Older failures remain evidence of what happened then; the later repair report
records the resulting state. Raw artifact links may require the local workspace.
Other-aircraft reference data is not a configuration source for this drone.

## Development

[README](../README.md#development-checks) lists repository checks.
[Software architecture](SOFTWARE_ARCHITECTURE.md) explains runtime responsibilities.
[CLAUDE.md](../CLAUDE.md) describes architecture, dependencies and contributor
conventions; [AGENTS.md](../AGENTS.md) records access and hardware-safety rules.
