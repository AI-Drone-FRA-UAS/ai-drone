# Autonomous MAVLink Drone Control via Raspberry Pi

This documentation describes the architecture, control interfaces, and safety guidelines for the modular MAVLink control system developed for the autonomous university drone project.

The primary objective is to link the FlywooF745 flight controller (running ArduPilot Copter 4.6.3) with the IMX500 AI Camera on the Raspberry Pi to enable visually guided, autonomous flight maneuvers—such as person tracking, collision avoidance, and dynamic obstacle navigation—in both indoor and outdoor environments.

---

## 1. System Overview & Hardware Configuration

The system architecture cleanly decouples high-level computer vision sensing from low-level flight stabilization:

```text
+---------------------+        UART (/dev/serial0)        +------------------------+
|  Raspberry Pi 4 /   | <-------------------------------> |   FlywooF745 (GOKU)    |
|     Zero 2 W        |        115200 Baud MAVLink 2      |  ArduPilot Copter 4.6  |
+---------------------+                                   +------------------------+
  |               ^                                         |                  ^
  |               | I2C/CSI                                 | SERIAL5          | Motors
  v               |                                         v                  |
+---------------------+                                   +------------------------+
| IMX500 AI-Camera    |                                   |  MicoAir MTF-01P       |
| (NanoDet Person Det)|                                   | (Lidar & Optical Flow) |
+---------------------+                                   +------------------------+
```

### Components & Interfaces
1. **Flight Controller**: Flywoo GOKU GN745 AIO / FlywooF745 running ArduPilot Copter 4.6.3.
2. **Position & Altitude Hold (Indoor GPS-Denied)**:
   * **MicoAir MTF-01P**: Connected to ArduPilot via `SERIAL5` (`SERIAL5_PROTOCOL=1`, `SERIAL5_BAUD=115`).
   * **EKF3 Fusion**: ArduPilot fuses the downward-facing Optical Flow sensor for horizontal velocity estimation and the integrated LiDAR rangefinder for vertical altitude hold (`FLOW_TYPE=5`, `RNGFND1_TYPE=10`).
3. **Raspberry Pi MAVLink Link**:
   * On the physical drone build, MAVLink communication operates over the hardware UART on the Pi GPIO header: `/dev/serial0` (Pin 8/10 TX/RX) at **115200 baud** (`SERIAL4_PROTOCOL=2`, `SERIAL4_BAUD=115`).
   * For bench testing on a developer laptop, the flight controller can be connected via USB (automatically detected under `/dev/ttyACM0` or `/dev/serial/by-id/...`).

---

## 2. Architecture: Why `DroneController`?

The legacy script `fly_and_land.py` was linear and sequential, making it unsuitable for reactive, real-time control loops driven by continuous AI camera feeds.

The **`ai_drone.controller.DroneController`** class introduces a modern, object-oriented MAVLink control interface with critical operational advantages:
* **Automatic Device Discovery (`_find_device`)**: Automatically locates the correct connection path (`/dev/serial0` on the Pi, `/dev/ttyACM*` on USB, or network sockets like `udp:127.0.0.1:14550` for SITL and simulations).
* **High-Frequency MAVLink 2 Streaming**: Requests continuous background telemetry feeds (`LOCAL_POSITION_NED`, `RANGEFINDER`, `ATTITUDE`, `SYS_STATUS`) using `MAV_CMD_SET_MESSAGE_INTERVAL`.
* **Non-Blocking State Tracking**: Parses incoming messages in a background thread and updates attributes such as `current_altitude`, `battery_voltage`, `flight_mode`, and `is_armed` in real time without blocking main execution.
* **Verified Mode Transitions**: Does not send blind commands; it actively waits for confirmation in the FC's `HEARTBEAT` before arming motors or executing takeoff maneuvers.
* **Integrated Safety Traps**: Prevents uncontrolled flight or flyaways if unhandled exceptions or user interruptions occur.

---

## 3. Autonomous Flight & AI Integration (Body-Frame Velocity)

For autonomous flight based on visual sensors (such as person detection from `ai_drone/nearest_person.py`), navigating to static GPS waypoints is impractical. Bounding boxes and depth estimation from a camera are inherently relative to the drone's **current orientation (Body Frame)**.

To facilitate this, `DroneController` implements **`send_velocity_body(vx, vy, vz, yaw_rate_deg)`**:
* Sends `SET_POSITION_TARGET_LOCAL_NED` using the `MAV_FRAME_BODY_NED` coordinate frame.
* Configures the bitmask (`0x05C7`) to control velocity vectors and yaw rate exclusively.

### Body-Frame Coordinate Axes
* $+v_x$: Fly **Forward** | $-v_x$: Fly **Backward** (m/s)
* $+v_y$: Strafe **Right** | $-v_y$: Strafe **Left** (m/s)
* $+v_z$: **Descend** | $-v_z$: **Ascend** (m/s — Note: Z points downwards in NED conventions!)
* $\text{yaw\_rate\_deg}$: Yaw rotation velocity in degrees per second ($+$ rotates **Right**, $-$ rotates **Left**).

### Autonomous Person Tracking Module (`ai_drone/follower.py`)

The project includes a dedicated autonomous follower module (**`ai_drone.follower.AutonomousFollower`**) that links bounding box detections directly to flight maneuvers:
1. **Target Extraction & Distance Estimation (`get_person_target`)**: Filters detections for COCO class `0` (person), calculates pixel offsets from the image center (`offset_x_px`, `offset_y_px`), and estimates metric distance using a simplified pinhole camera model based on bounding box height.
2. **Proportional Control (`compute_velocity_command`)**: Converts target distance and horizontal pixel offsets into smooth body-frame velocity commands ($v_x$ for maintaining distance, $\text{yaw\_rate}$ for centering the target). Includes deadzones to eliminate oscillation and velocity clamping for safety.
3. **Safety Guardrails (`check_safety_guardrails`)**: Continuously monitors flight altitude and LiPo battery voltage. If the voltage drops below critical thresholds (default: `14.4 V`) or the altitude limit is breached, it immediately triggers an emergency landing.
4. **Target-Loss Protection**: If visual contact is lost during tracking, the drone automatically switches to hover (`0 m/s`) after 3 seconds.

---

## 4. CLI Tools (`drone-control` & `drone-follow`)

Two command-line tools are provided for bench diagnostics, safety verification, and autonomous flight missions without requiring manual code edits.

### 1. General Flight Control (`drone-control`)

#### Passive Sensor Monitoring (`status`)
Connects to the flight controller and streams real-time LiDAR altitude, LiPo voltage, and flight mode. **Motors remain disarmed.** Ideal for bench testing sensor calibration.
```bash
uv run drone-control status --duration 10
```

#### Autonomous Hover Test (`hover`)
Arms the motors, ascends to `--takeoff-alt`, maintains a stable hover for the specified duration, and lands autonomously.
```bash
uv run drone-control hover --takeoff-alt 0.4 --duration 5 --max-alt 0.8
```

#### Body-Frame Velocity Demonstrator (`velocity-test`)
Demonstrates flight maneuverability by issuing body-frame velocity vectors during flight.
```bash
# Ascends to 0.4m, rotates right at 10 deg/s for 4 seconds, and lands
uv run drone-control velocity-test --takeoff-alt 0.4 --duration 4 --vx 0.0 --yaw-rate 10.0
```

### 2. Autonomous Person Tracking (`drone-follow`)

A standalone executable script for running the complete AI tracking pipeline:
```bash
# Run simulated person tracking on your desktop (no takeoff, no camera required)
uv run drone-follow --sim-target --duration 15

# Run live autonomous person tracking with the IMX500 AI camera in a flight safety net
uv run drone-follow --device /dev/serial0 --takeoff-alt 0.5 --target-dist 2.0 --max-alt 0.8
```
*(Alternatively, use the subcommand via `uv run drone-control follow --sim-target`.)*

---

## 5. Safety Guidelines (Safety First!)

Because current ArduPilot parameters on the drone have `ARMING_CHECK=0` and `FENCE_ENABLE=0` configured for testing purposes, operational safety relies entirely on software safeguards and pilot vigilance. The following rules must be strictly observed before any physical flight:

1. **Flight Safety Net / Enclosure**: All initial flight tests with autonomous control loops must be conducted inside a protective flight safety net or an indoor arena free of obstacles and personnel.
2. **Hardware RC Kill-Switch**: A safety pilot must hold a physical RC transmitter linked to the drone at all times. If any unexpected flight behavior or oscillation occurs, the pilot must immediately flip the RC kill-switch or override the flight mode to `LOITER` / `LAND`.
3. **Automated Software Traps**:
   * **Context Manager Protection**: All flight operations should be wrapped inside `with DroneController(...) as drone:`. If an unhandled exception, syntax error, or `Ctrl+C` keyboard interrupt occurs, the `__exit__` block automatically triggers `emergency_stop()` (`LAND` or `DISARM`).
   * **Altitude Guard**: If vibrations or optical flow drift cause the drone to exceed `max_altitude`, the controller immediately overrides velocity commands and lands.
   * **Battery Guard**: Never take off with a low LiPo battery. The automated follower aborts missions immediately if battery voltage drops below `14.4 V` (for a 4S battery).
4. **Pre-Flight Check**: Always run `uv run drone-control status` before arming to verify that LiDAR altitude reads ~0.00 m on the ground and LiPo battery voltage is above `15.0 V`.
