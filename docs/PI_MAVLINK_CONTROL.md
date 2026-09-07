# MAVLink control and staged flight testing

`drone-control hover` owns a guarded GPS-free takeoff, flow-backed Loiter hold,
landing, and cleanup sequence. Passive status belongs to `drone-inspect`.

The current guarded GPS-free hover path has passed pinned ArduCopter SITL tests;
its armed execution has not been validated on this aircraft. Check the newest
`state/` capture and `params/` dump before use. Firmware and acceptance details
are in the [no-GPS Loiter review](ARDUCOPTER_4_7_NOGPS_LOITER.md).

## Modes

| Mode | Behavior | Hardware effect |
| --- | --- | --- |
| `hover` (`takeoff` alias) | `GUIDED_NOGPS` climb, flow-backed `LOITER`, then `LAND` | Arms and flies |

Open a Pi shell and inspect the authoritative command help:

```bash
uv run drone-connect
# On the Pi: cd ~/ai-drone && uv run drone-control --help
```

The safe starting point is:

```bash
uv run drone-inspect --duration 10
```

`drone-inspect` sends no arm, mode-change, or setpoint command. Hover requires
the exact acknowledgement shown by `--help`; do not bypass that gate.

## Control behavior

`ai_drone.flight.controller.DroneController` owns the MAVLink connection and
arm, mode, takeoff, land, and cleanup transitions. Generic
battery, altitude-ceiling, and telemetry-staleness guards live in
`ai_drone.flight.guards` and apply to every flight mode.

The flight controller remains responsible for attitude stabilization and EKF
fusion.

The controller filters telemetry to the selected vehicle, rejects stale data,
requires exactly `ARMING_SKIPCHK=0` before an arm path, verifies state
transitions, caps commands, and uses bounded timeouts. Cleanup lands only a
flight started by that controller instance.

This control path requires fresh `RC_CHANNELS.chancount=0` before arming and
throughout its takeoff and hold. An active RC receiver causes preflight refusal;
receiver channels appearing or their report becoming stale during flight
requests LAND. The captured aircraft has no active RC receiver, and the code
does not provide an RC override or kill path. Installing a receiver requires
reviewing and testing mode authority and throttle behavior before changing this
guard.

## Required validation sequence

1. Run the repository checks and every control mode in ArduPilot Copter SITL,
   including rejected arm, stale telemetry, interruption, and landing timeout.
2. Rigidly mount and calibrate the IMU, compass, optical flow, downward
   rangefinder, and any camera needed by later missions.
3. In an authorized configuration session, restore `ARMING_SKIPCHK=0`, resolve
   every pre-arm message, and define an appropriate indoor boundary/recovery
   behavior.
4. With all propellers removed and the frame secured, verify motor numbering
   and direction using the [guarded motor procedure](BENCH_MOTOR_TEST.md).
5. Establish and separately validate an emergency-stop or emergency-control
   arrangement for the actual aircraft, including loss of the companion and its
   link. None is established by the present disarmed checks. The configured GCS
   heartbeat-loss LAND response has passed SITL, but that does not demonstrate
   an independent live emergency control. Resolve this limitation before flight.
6. With a safety observer, protective enclosure, suitable battery, clear area,
   and the validated emergency arrangement, perform the smallest authorized
   hover test: the default target is 0.5 m with a 0.8 m software ceiling.
7. Review range, flow, EKF, and battery recordings before expanding the envelope.

AprilTag approach, autonomous search, and payload release are mission work,
not existing `drone-control` modes. Their additional prerequisites are in the
[AprilTag mission architecture](APRILTAG_MISSION.md).
