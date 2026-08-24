# AprilTag detection, approach, and payload mission

This document describes the safe target architecture for detecting floor-mounted
AprilTags, holding altitude, approaching a selected tag, and releasing a payload.
It does not authorize autonomous flight: every stage must pass the bench,
restrained, and manual-override tests below first.

## Camera mounting

For dropping a payload directly onto a floor tag, mount the IMX500 **straight
down (nadir, optical axis parallel to body Z)**. The mount must be rigid and
must record the camera position and yaw relative to the flight-controller body
frame. Do not leave the camera supported by its ribbon cable.

A nadir mount provides the best corner geometry near the release point, keeps
the target visible while the drone is centered over it, and makes the camera,
downward LiDAR, and optical-flow frames conceptually aligned. A diagonal mount
is a compromise that reduces final pose accuracy without providing obstacle
depth. A forward mount is appropriate only when tags are placed on walls.

If the mission must detect wall tags or avoid previously unknown obstacles,
add a separate forward-looking depth/range sensor or a second camera. The
downward MTF-01P is a one-dimensional altitude sensor and cannot see a wall,
person, shelf, cable, or other forward obstacle.

Before calibration:

1. Rigidly mount the camera and strain-relieve the ribbon cable.
2. Mechanically focus the AI Camera at the intended 1-3 m working distance.
3. Lock the lens so vibration cannot change focus.
4. Calibrate intrinsics at 1280x960 using a large, flat calibration target.
5. Measure camera-to-body translation and yaw; do not assume the optical and
   body frames are identical.

## Detection and distance software

The Pi has two supported detector backends:

- Native AprilTag 3 from Debian's `python3-apriltag` package. This is preferred
  and reports Hamming correction and decision margin.
- OpenCV `ArucoDetector` with `DICT_APRILTAG_36h11`, used as the fallback and
  for calibration, square-IPPE pose solving, reprojection checking, and image
  annotation.

The IMX500 neural accelerator is not used for AprilTag decoding. AprilTag is a
geometric fiducial detector, and accurate sub-pixel corners matter more than a
neural bounding box.

Run detection from the developer machine:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm \
  uv run drone-deploy --run inspect -- --backend auto --tag-size 0.160 --stream
```

Use `--tag-size 0.224` for the supplied A3 tags. Print at 100% and use the
measured black-border square size, converted to metres.

Without `--calibration`, the command intentionally reports IDs and corners but
not metric distance. With a valid calibration JSON it reports camera-frame
position, Euclidean distance, and reprojection error. A calibration applies
only to the same focus, sensor crop, resolution, and aspect ratio used in
flight; never use placeholder intrinsics.

## Altitude and horizontal stability

Keep altitude and low-level attitude control in ArduPilot EKF3. The MTF-01P
provides downward range and optical-flow velocity; the companion computer
should consume ArduPilot's fused attitude/local-motion estimate rather than
independently double-fusing the same raw measurements.

Use the newest parameter dump to confirm the EKF range and optical-flow sources.
Configuration and received telemetry do not prove flight-ready quality.

## Mission state machine

Implement autonomy as explicit, abortable stages:

1. **Preflight:** valid camera calibration, rigid mount, healthy LiDAR/flow,
   synchronized clocks, full arming checks, RC/manual override, and payload
   interlock.
2. **Search:** fly a bounded, pre-approved raster at constant LiDAR altitude.
   This is acceptable only in a known obstacle-free test area.
3. **Acquire:** require the allowed tag ID, zero Hamming corrections, adequate
   decision margin, low reprojection error, and several consecutive frames.
4. **Approach:** transform camera pose into body frame and command capped XY
   velocity while ArduPilot holds altitude. Stop on stale or rejected vision.
5. **Center:** hold horizontal error within a small threshold for a dwell time;
   independently confirm LiDAR altitude and low body velocity.
6. **Release:** permit one servo movement only when an operator has enabled the
   mission, the expected tag remains valid, altitude is in the release window,
   and centering has remained stable. Latch the release so it cannot repeat.
7. **Exit:** stop, climb or retreat using a pre-approved action, and report the
   completed tag ID.

ArduPilot's MAVLink `LANDING_TARGET` service is suitable for precision loiter
or landing over a stationary tag. Mapped-tag localization for a hall should
instead produce a continuous pose/odometry estimate and send `ODOMETRY` as
ExternalNav. Body-frame velocity commands are suitable for a deliberately
bounded approach controller.

## Hall navigation limitation

The current sensor set is insufficient for safe autonomous exploration of an
unknown hall. Optical flow supplies velocity, not globally drift-free position;
the downward LiDAR supplies altitude, not obstacle avoidance; and a monocular
nadir camera sees floor texture/tags but not forward hazards.

Two realistic operating envelopes are:

- **Known, empty hall:** a surveyed rectangular geofence, pre-planned raster,
  optical-flow/LiDAR hold, and floor tags as periodic landmarks. Start with a
  restrained low-speed test and an immediate manual override.
- **Unknown or occupied hall:** add forward/360-degree obstacle sensing and a
  localization system. A 2D LiDAR or depth camera plus SLAM is typical, but a
  Zero 2 W with 415 MiB RAM is not a good platform for a heavy SLAM stack. Use
  a stronger companion computer or offboard localization.

## Required test sequence

1. Printed-image and synthetic detector tests.
2. Camera-only live capture, with no flight commands.
3. Rigid-mount focus and calibration.
4. Disarmed tag pose comparison against tape-measured distances and angles.
5. Propellers-off servo interlock tests with no payload.
6. Restrained altitude/flow test over representative flooring.
7. Manual hover with detection logging only.
8. Capped-velocity centering with no servo action.
9. Dummy payload release over a soft, isolated test area.
10. Only then, bounded search behavior.

Do not proceed to an armed test unless `ARMING_CHECK` is exactly `1`, a valid
local position is available, and the Pi clock is synchronized.

## Primary references

- AprilTag 3 detector and pose guidance: <https://github.com/AprilRobotics/apriltag>
- Raspberry Pi AI Camera: <https://www.raspberrypi.com/documentation/accessories/ai-camera.html>
- ArduPilot optical flow: <https://ardupilot.org/copter/docs/common-optical-flow-sensor-setup.html>
- ArduPilot precision landing: <https://ardupilot.org/copter/docs/precision-landing-and-loiter.html>
- ArduPilot ExternalNav: <https://ardupilot.org/dev/docs/mavlink-nongps-position-estimation.html>
