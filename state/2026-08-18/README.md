# Live drone and Raspberry Pi state — 2026-08-18

Captured through `seb@192.168.4.1` while the vehicle was disarmed. No arming,
mode change, motor command, or servo actuation was performed.

The Pi clock incorrectly reported **2026-08-11**, seven days behind the
workstation date used for this directory. NTP was enabled but not synchronized
because the Pi hotspot had no upstream internet. The raw files retain the Pi's
reported timestamps so the discrepancy is visible.

### Remediation later on 2026-08-18

The Pi timezone was corrected from `Europe/London` to `Europe/Berlin`, its clock
was seeded from the developer workstation, and NTP was left enabled. It remains
unsynchronized only because the standalone hotspot has no upstream route. The
systemd clock epoch file now preserves the corrected date across an offline
reboot. The stale `NetworkManager-wait-online` failure was restarted
successfully and the failed-service count is now zero. See
`pi-remediation.txt`.

The deployed controller's stale 921600-baud default was also corrected to the
verified UART4 rate of 115200.

## Flight controller

- ArduPilot Copter 4.6.3, custom commit `92b0cd78`
- Mode `STABILIZE`, armed `false`
- Full parameter download: 1152/1152
- Battery: 16.219 V, about 85%, no reported battery fault
- GPS: no fix, zero satellites
- No `LOCAL_POSITION_NED` or home position was available
- Vehicle was resting at approximately -133 degrees roll and -53 degrees pitch
- Live `ARMING_CHECK=0` and `FENCE_ENABLE=0`; do not fly autonomously
- Pi companion link: `/dev/serial0`, MAVLink2 at 115200
- MTF-01P link: MAVLink at 115200, downward orientation

## MTF-01P bench sample

Ten seconds produced 296 messages:

- Range: 198 samples, median 2.20 m, minimum 2.12 m, maximum 2.25 m
- Optical flow: 98 samples, median quality 38/255, maximum 39/255

These values prove communication and sensor power only. The vehicle orientation,
surface texture, lighting, and motion were not suitable for flight-quality
validation.

## Raspberry Pi and camera

- Raspberry Pi Zero 2 W, 415 MiB RAM, Debian 13 ARM64
- No undervoltage/throttling flags; 24 GiB storage free
- IMX500 detected at 2028x1520/30 fps and 4056x3040/10 fps
- Picamera2 0.3.36, rpicam-apps 1.12.0, libcamera 0.7.1
- modlib 1.3.1, OpenCV 4.11.0, NumPy 2.3.5, pymavlink 2.4.49
- Native AprilTag 3.4.2 ARM64 installed from checksum-verified Debian packages
- GPIO12 was an unclaimed input; the servo was not actuated
- `NetworkManager-wait-online.service` was the only failed service

The live camera throughput results are in `camera-apriltag-benchmark.json`.
At 1280x960 the native detector processed approximately 13.2 frames/s and
OpenCV approximately 14.9 frames/s end-to-end. At 640x480 both pipelines kept
up near the 30 fps camera rate. No physical tag was in view.

An eight-second integrated recorder test later captured 230 encoded camera
frames, 1,332 MAVLink messages, 196 range readings, and 196 optical-flow
readings while the vehicle remained disarmed. The LiDAR was approximately 2 cm
from the surface and median flow quality was 147/255. No tag was decoded because
the preview showed that a laptop blocked most of the camera view. See
`all-sensor-test-summary.json`; the full video and telemetry dataset remains in
the ignored `artifacts/sensor-recordings/` directory.

## Files

- `drone-live-state.json`: firmware identity and latest range/flow messages
- `drone-telemetry-detail.json`: battery, attitude, EKF, GPS, RC, and HUD data
- `drone-sensor-samples.csv`: ten-second MTF-01P sample
- `../../params/flywoo-f745-live-2026-08-18.param`: complete live parameters
- `pi-system.txt`: hardware, OS, camera, models, storage, and services
- `pi-debian-packages.txt`: complete Debian package manifest
- `pi-python-packages.txt`: Python distributions visible in the project venv
- `pi-opencv-build.txt`: OpenCV modules, threading, and ARM/NEON build details
- `pi-remediation.txt`: corrected time, service, serial, and deployed baud state
- `all-sensor-test-summary.json`: disarmed integrated recorder result
