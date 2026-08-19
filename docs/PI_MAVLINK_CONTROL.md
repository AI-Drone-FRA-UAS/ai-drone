# Consolidated MAVLink control

`drone-control` is the single CLI for flight-controller status,
takeoff/hover, body-frame velocity, and person following. Its flight sequences
manage arming, stop, landing, and cleanup. Dedicated follow and linear-flight
wrappers are folded into these modes, so the capability has one maintained
implementation and command surface.

The control code is implemented, but no live arm, takeoff, altitude hold,
velocity flight, or person-follow flight has been validated on this aircraft.
Read [the handoff](HANDOFF.md) before using it. The current vehicle has
`ARMING_CHECK=0`, a loose forward-facing camera, no useful indoor fence, a
disconnected servo, and no forward MT-15; those conditions block live flight.

## Run the command

Deploy and open a Pi shell from the developer machine:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm uv run drone-deploy --ssh
```

When Tailscale is offline and the laptop is joined to `AI-Drone-Zero`, use
`PI_HOST=seb@192.168.4.1` instead. On the Pi:

```bash
cd ~/ai-drone
uv run drone-control --help
```

The command groups the following behaviors:

| Mode | Behavior | Current state |
| --- | --- | --- |
| `status` | Passive altitude, battery, mode, and armed-state monitoring | Disarmed bench use is possible |
| `hover` (`takeoff` alias) | Guided takeoff, timed altitude hold, then land | Implemented; SITL and restrained-flight validation required |
| `velocity-test` | Bounded body-frame velocity/yaw command, then stop and land | Implemented; SITL and restrained-flight validation required |
| `follow` | Offline simulation or IMX500 person-follow flight | Simulation available; live flight blocked by mounting/calibration/flight gates |

Use each subcommand's `--help` output for its exact confirmation token and
limits. Do not bypass those gates in wrappers or documentation.

The two non-actuating starting points are:

```bash
uv run drone-control status --duration 10
uv run drone-control follow --simulate --duration 15
```

Simulation exercises the follow control math without a camera, arming, or
takeoff. It is not a full ArduPilot dynamics test; use Copter SITL for mode,
arming, telemetry-loss, takeoff, velocity, and landing behavior.

Every live flight mode requires the exact acknowledgement:

```text
--confirm-flight FLIGHT_TEST_READY
```

Live `follow` additionally requires a measured `--focal-length-px` and:

```text
--confirm-live-follow CAMERA_RIGID_AND_CALIBRATED
```

The current loose forward-facing camera cannot truthfully satisfy that gate.

## Control architecture

`ai_drone.controller.DroneController` owns the MAVLink connection and the
arm/mode/takeoff/velocity/land state transitions. `ai_drone.follower` owns
person-target extraction and bounded follow-control math. The CLI only parses
operator intent and sequences those reusable components.

The flight controller remains responsible for attitude stabilization and EKF3
fusion. The connected MTF-01P supplies downward range and optical-flow data;
the Pi sends higher-level body-frame velocity requests.

Body-frame velocity uses NED conventions:

- positive `vx`: forward; negative: backward;
- positive `vy`: right; negative: left;
- positive `vz`: down; negative: up; and
- positive yaw rate: turn right.

The controller filters telemetry to the selected vehicle, rejects stale
heartbeats and altitude samples, requires exactly `ARMING_CHECK=1` before an
arm path, verifies mode/armed transitions, caps command values, and uses bounded
timeouts. Cleanup lands only a flight started by that controller instance and
disarms an arm started by it; passive observation of an already armed vehicle
does not itself send a command.

## Staged validation

1. Run unit tests and every control mode in ArduPilot Copter SITL, including
   rejected arm, stale telemetry, target loss, low battery, over-altitude,
   interruption, and landing timeout cases.
2. Rigidly mount the airframe sensors and camera. Calibrate the IMU, compass,
   optical-flow scale/orientation, downward rangefinder, and camera geometry.
3. Restore `ARMING_CHECK=1` in a separately authorized configuration session
   and resolve every `PreArm:`/`Arm:` message. Configure an appropriate indoor
   boundary and recovery behavior; do not simply enable the current 100 m /
   300 m fence.
4. With every propeller removed and the frame secured, validate motor numbering
   and direction using only the [guarded motor utility](BENCH_MOTOR_TEST.md).
5. With a safety pilot, RC override/kill path, protective enclosure, fresh
   battery, and clear area, perform the smallest restrained hover/altitude-hold
   test. Verify range/flow and EKF behavior from the recording before expanding
   the envelope.
6. Validate bounded velocity one axis at a time. Test loss-of-link and stale
   localization before enabling any camera-driven command.
7. Enable person-follow flight only after the camera is rigid and its geometry
   is measured, the target-loss behavior passes SITL and restrained tests, and
   people are outside the vehicle's possible path.

The forward MT-15 is an additional stop-ahead sensor, not a prerequisite that
the current follow loop already consumes and not a substitute for a protected
flight area or wider obstacle sensing.
