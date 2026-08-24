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

## SITL

Keep ArduPilot outside this repository and pin the checkout. ArduPilot's
[native setup instructions](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)
are tested on Ubuntu; do not run its Ubuntu prerequisite script directly on
this Arch Linux host. Use the checkout's container with rootless Podman:

```bash
# Run on the Arch host. The clone command creates /home/abaris/ardupilot.
cd /home/abaris
git clone --branch Copter-4.7.0 --recurse-submodules \
  https://github.com/ArduPilot/ardupilot.git /home/abaris/ardupilot

# Build the reusable Ubuntu toolchain image from the ArduPilot checkout.
cd /home/abaris/ardupilot
podman build . -t localhost/ardupilot-sitl:4.7.0 \
  --build-arg USER_UID="$(id -u)" --build-arg USER_GID="$(id -g)" \
  --build-arg DO_AP_STM_ENV=0 --build-arg SKIP_AP_EXT_ENV=1

# Compile ArduCopter SITL into /home/abaris/ardupilot/build/sitl.
podman run --rm --userns=keep-id \
  -v /home/abaris/ardupilot:/ardupilot \
  localhost/ardupilot-sitl:4.7.0 \
  bash -lc './waf configure --board sitl && ./waf copter'

# Run this repository's complete inspect-hover-land simulation.
cd /home/abaris/ai-drone
ARDUPILOT_ROOT=/home/abaris/ardupilot \
  uv run --group dev pytest -m sitl -q -s
```

Docker can run the same image recipe if Podman is unavailable. The container
build is an explicit, one-time prerequisite. The opt-in test
never compiles ArduPilot: it runs the prepared binary directly, inspects
simulated downward range and optical flow, flies a 0.4 m hover below a 0.6 m
ceiling, lands, and verifies disarm. Ordinary test runs skip it when
`ARDUPILOT_ROOT` is unset. Windows users should follow ArduPilot's
[WSL instructions](https://ardupilot.org/dev/docs/sitl-on-windows-wsl.html).

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
