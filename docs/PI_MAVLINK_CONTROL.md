# MAVLink control and staged flight testing

`drone-control hover` owns a guarded takeoff, timed hold, landing, and cleanup
sequence. Passive status belongs to `drone-inspect`.

No live arm, takeoff, or altitude hold has been validated on
this aircraft. Check the newest `state/` capture and `params/` dump before use.

## Modes

| Mode | Behavior | Hardware effect |
| --- | --- | --- |
| `hover` (`takeoff` alias) | Guided takeoff, timed hold, then land | Arms and flies |

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
5. With a safety pilot, tested RC override/kill path, protective enclosure,
   fresh battery, and clear area, perform the smallest restrained hover test.
6. Review range, flow, EKF, and battery recordings before expanding the
   envelope only after reviewing the results.

AprilTag approach, autonomous search, and payload release are mission work,
not existing `drone-control` modes. Their additional prerequisites are in the
[AprilTag mission architecture](APRILTAG_MISSION.md).
