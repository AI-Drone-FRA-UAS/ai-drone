# AI Drone — Implementation Reference

> This document describes what must be implemented and what hardware and software is available.
> It is intended as a concise technical reference.

---

## Table of Contents

1. [Implementation Goals](#1-implementation-goals)
2. [Available Software](#2-available-software)
3. [Available Hardware — Provided Equipment](#3-available-hardware--provided-equipment)
4. [Available Hardware — Drone Build Specifications](#4-available-hardware--drone-build-specifications)

---

## 1. Implementation Goals

### 1.1 — Autopilot Integration

Flash and configure **ArduPilot** as the flight controller firmware to replace the default
Betaflight installation. Set up mission planning and ground control via **QGroundControl**.

### 1.2 — Position Hold and Altitude Hold (Indoor / GPS-Denied)

Integrate stable Position Hold and Altitude Hold using the **MicroAir MTF-01P** sensor, which
provides both:

- **LiDAR** — downward-facing distance measurement for altitude hold
- **Optical Flow** — horizontal velocity estimation for position hold without GPS

The MTF-01P cable is already soldered to the flight controller. The sensor end must be connected
and the sensor must be permanently mounted (requires frame work — see 1.4).

### 1.3 — Delivery / Payload Drop Mechanism

Design, build, and integrate a mechanism that can release a payload on command.

- Use one of the provided **Micro Servo 9g** units as the actuator.
- The servo is controlled via the flight controller (ArduPilot servo output).
- 3D printing is available for custom brackets or mounts.

### 1.4 — Frame Extension

Design and build a frame extension or replacement that integrates:

- The **MicroAir MTF-01P** sensor (must face downward, unobstructed)
- The **delivery / payload mechanism** (servo + release arm)
- Optionally: the **Raspberry Pi Zero 2 WH** and its camera module

Use **Tinkercad** for CAD design and **UltiMaker Cura** for slicing. 3D printing is available.
M3 standoffs and screws (M3×9 mm, M3×12 mm) are provided for mounting.

### 1.5 — Raspberry Pi Integration (Computer Vision / AI)

Connect and configure the **Raspberry Pi Zero 2 WH** as a companion computer.

- The RPi cable is already soldered to the flight controller. The RPi end must be connected.
- Mount the **Raspberry Pi AI Camera Module** on the drone (frame extension from 1.4).
- Run AI inference (e.g. object detection with **TensorFlow Lite** or **YOLO**) on the RPi to
  support autonomous delivery targeting.
- The **USB A/V Grabber (MacroSilicon MS210x)** can be used to capture the analog FPV camera
  feed on the RPi as an additional video source.

---

## 2. Available Software

| Software | Purpose |
|----------|---------|
| **ArduPilot** | Flight controller firmware (autopilot, mission execution) |
| **QGroundControl** | Ground control station, mission planning, parameter configuration |
| **Raspberry Pi OS** | Operating system for the Raspberry Pi Zero 2 WH |
| **TensorFlow Lite** | On-device AI inference (object detection, classification) |
| **YOLO** | Real-time object detection alternative to TensorFlow Lite |
| **Tinkercad** | Browser-based 3D CAD for frame / mount design |
| **UltiMaker Cura** | Slicer for preparing 3D print files |

---

## 3. Available Hardware — Provided Equipment

### Flight System

| Item | Details |
|------|---------|
| FPV Drone (3.5" CineWhoop) | Fully assembled, flight-ready, with propeller guards |
| Skyzone Cobra X FPV Goggles | For first-person video monitoring |
| Li-Ion Battery Packs | 3 × flight batteries |

### Remote Control

| Item | Details |
|------|---------|
| Radiomaster GX12 Remote Controller | ELRS Dual-Band Gemini-X, 868 MHz / 2.4 GHz |
| Firmware | EdgeTX v2.11.5 |
| Binding Phrase | `drone[1-3]ffm` (pre-configured to match the receiver) |

### Computing & Sensing

| Item | Details |
|------|---------|
| Raspberry Pi Zero 2 WH | Companion single-board computer |
| RPi Zero 2 Cases | 2 different enclosures |
| Raspberry Pi AI Camera Module | CSI camera for computer vision |
| MicroAir MTF-01P | LiDAR (range) + Optical Flow sensor |
| CP2102 USB-UART Adapter | For configuring the MicroAir MTF-01P |
| USB A/V Grabber | MacroSilicon MS210x — captures analog FPV feed |

### Cables & Adapters

| Item |
|------|
| Mini-HDMI to standard HDMI cable |
| Micro-USB (male) to USB Type-A (female) cable |
| USB Type-A (male) to USB-C cable |
| USB-C to USB-C cable |

### Actuators & Electronics

| Item | Details |
|------|---------|
| Micro Servo 9g | 2 × units — for payload drop mechanism |
| Jumper wires | Female-to-female and female-to-male |
| Smoke Stopper | Short-circuit protection plug |
| Speedybee Adapter V3 | Flight controller configuration tool |

### Charging & Power

| Item | Details |
|------|---------|
| SkyRC B6neo+ Charger | For Li-Ion battery packs |
| 18650 Cells | For remote controller and FPV goggles |
| Power Bank | 20,000 mAh, PD 20 W |
| USB-C Power Adapter | 27 W — charges goggles, remote, power bank; powers SkyRC B6neo+ |

### Tools & Fasteners

| Item | Details |
|------|---------|
| Hex / Allen Key Set | 1.5 mm, 2.0 mm, 4.0 mm, 5.5 mm, 8.0 mm |
| Precision Screwdriver Set | 48-piece bit set |
| M3 Standoffs & Screws | M3×9 mm, M3×12 mm |

### Spares & Storage

| Item | Details |
|------|---------|
| Replacement Propellers (3.5") | Gemfan D90-5 or HQProp DT90MMX5 |
| MicroSD Card | 32 GB, with SD/MicroSD adapter |
| USB Card Reader | For SD and MicroSD cards |
| Aluminum Carry Case | Approx. 45 × 30 × 15 cm |

---

## 4. Available Hardware — Drone Build Specifications

### Airframe

| Property | Value |
|----------|-------|
| **Frame** | SpeedyBee BEE35 Pro 3.5" CineWhoop Frame Kit |
| **Propellers** | Gemfan 90mm D90-5 — 3.5", ducted, 5-blade |

### Flight Controller (FC / AIO)

| Property | Value |
|----------|-------|
| **Model** | Flywoo GOKU GN745 AIO |
| **MCU** | STM32F745, 216 MHz, 1 MB Flash |
| **ESC** | 45 A, 2–6S, AM32 |
| **Target firmware** | ArduPilot (currently ships with Betaflight v2025.12.2) |

### Radio Link

| Property | Value |
|----------|-------|
| **Receiver** | Radiomaster XR4 Gemini Xrossband Dual-Band ELRS |
| **Protocol** | ExpressLRS (ELRS) |
| **Firmware** | ExpressLRS 4.0.0 |
| **Binding Phrase** | `drone1ffm` |

### FPV Video System

| Property | Value |
|----------|-------|
| **Camera** | RunCam Phoenix 2 — 1000 TVL, 155° FOV, analog |
| **Video Transmitter (VTX)** | SpeedyBee TX800 |
| **VTX Channel** | 5806 MHz (Raceband, Channel 5) |
| **VTX Antenna** | TrueRC Singularity 5.8 GHz RHCP, SMA |

### Propulsion

| Property | Value |
|----------|-------|
| **Motors** | Emax Eco II 2004, 3–6S, 3000 KV (4 × units) |

### Navigation

| Property | Value |
|----------|-------|
| **GPS / Compass** | HGLRC M100 with integrated compass |

### Pending Physical Integration

> These cables are **already soldered to the flight controller** but are not yet connected at
> the other end. Connecting, configuring, and permanently mounting these components is part of
> the implementation work.

| Component | Status |
|-----------|--------|
| MicroAir MTF-01P (LiDAR / Optical Flow) | Connected on UART5; permanent mounting/power reliability still to verify |
| Raspberry Pi Zero 2 WH | Connected on UART4; Pi `/dev/serial0` MAVLink verified |
