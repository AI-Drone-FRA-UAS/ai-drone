# Documentation Index

Every document in this repository, grouped by what you are trying to do.
The rendered version of this documentation is published as a project site —
see [Project Site](#project-site) at the bottom.

## Where the project stands

| | |
|---|---|
| **Works** | AprilTag (`tag36h11`) detection on the Pi, and a payload drop the detection triggers — verified on the ground and hand-held |
| **Does not work** | Fully autonomous flight. On 2026-08-21 a takeoff attempt destroyed the aircraft; nobody was hurt |
| **Read first** | [Flugversuche und Absturz](https://ai-drone-fra-uas.github.io/ai-drone/flugversuche.html) — the attempts, the three measured causes, and what changed because of them |
| **Where the code is** | The reworked flight code and the dated flight records are on `preflight-and-nogps-takeoff`; the AprilTag drop mission is on the `experimental-*` branches. This branch still carries the earlier person-following stack, and the documents below describe it |

## Start here

| Document | Contents |
|----------|----------|
| [Project README](../README.md) | Install, the four CLI commands, and the shortest path to a working setup |
| [Implementation Reference](drone-project.md) | Project goals, the provided equipment, and the full drone build specification |
| [Software Architecture](SOFTWARE_ARCHITECTURE.md) | The `ai_drone` package, the CLI entry points, environment variables, and the test suite |

## Hardware and configuration

| Document | Contents |
|----------|----------|
| [Drone Configuration](DRONE_CONFIGURATION.md) | Verified flight-controller state: serial ports, MTF-01P and Pi UART parameters, camera paths |
| [Payload Drop Mechanism](PAYLOAD_DROP.md) | The 9 g servo: specification, safe pulse range, power, and the three ways it can be driven |
| [Frame Extension and 3D Prints](FRAME_AND_3D_PRINTS.md) | What is in `3DPrints/`, how to slice it, and what each part mounts |
| `params/` | ArduPilot parameter backups — see [Drone Configuration](DRONE_CONFIGURATION.md) |

## Flying and control

| Document | Contents |
|----------|----------|
| [Pi MAVLink Control](PI_MAVLINK_CONTROL.md) | `DroneController`, body-frame velocity control, autonomous person following, and the safety rules — with the abort rules corrected after the 2026-08-21 accident |
| [Developer Machine Drone Connection](DEVELOPER_MACHINE_DRONE_CONNECTION.md) | The complete USB connection sequence, MAVProxy commands, and shutdown procedure |

## Raspberry Pi and networking

| Document | Contents |
|----------|----------|
| [RPi Zero 2 W USB SSH Setup](RPI_ZERO2W_USB_SSH_SETUP.md) | Bringing up the USB gadget link and SSH on a fresh Pi |
| [eduroam WLAN auf dem Raspberry Pi](EDUROAM_SETUP.md) | Campus Wi-Fi (Frankfurt UAS) and Tailscale reachability — German |
| [Raspberry Pi Wi-Fi Hotspot](../README_HOTSPOT.md) | The `AI-Drone-Zero` fallback access point and internet sharing — German |

## Records

| Document | Contents |
|----------|----------|
| [Development Session, 19 June 2026](../19-06-session.md) | The MAVLink, UART, and MTF-01P bring-up: every command, failure, and fix |
| [Arduino Servo Bench Test](../servo_instruction.md) | The `arduino-cli` sketch used to characterise the servo away from the drone |
| [Project Poster](poster/README.md) | The poster: HTML source and print PDFs in A0, A1, A2 and A3 |

## Project site

`site/` builds these documents into a static website that is published to
GitHub Pages on every push to `main`. To preview it locally:

```bash
uv run --group docs python site/build.py --serve
```

See [site/README.md](../site/README.md) for the build, the navigation manifest,
and how to add a page.
