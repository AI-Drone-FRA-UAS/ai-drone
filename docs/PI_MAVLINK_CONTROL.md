# MAVLink control and staged flight testing

`drone-control` is the single CLI for flight-controller status,
takeoff/hover, and bounded body-frame velocity tests. Flight sequences own
their arming, stopping, landing, and cleanup behavior.

No live arm, takeoff, altitude hold, or velocity flight has been validated on
this aircraft. Check the newest `state/` capture and `params/` dump before use.

## Modes

| Mode | Behavior | Hardware effect |
| --- | --- | --- |
| `status` | Read altitude, battery, mode, and armed state | Read-only |
| `preflight` | Report what would block a guarded takeoff | Read-only |
| `arm-test` | Arm in a chosen mode, hold at idle, disarm | Arms; no takeoff |
| `alt-hold-takeoff` | Climb, hold and land in ALT_HOLD; ArduPilot flies the climb | Arms and flies |
| `nogps-takeoff` | Climb, hold and land with no position estimate | Arms and flies |
| `stabilize-takeoff` | Climb in STABILIZE, hold in ALT_HOLD, then land | Arms and flies |
| `hover` (`takeoff` alias) | Guided takeoff, timed hold, then land | Arms and flies |
| `velocity-test` | Bounded body-frame velocity/yaw, then stop and land | Arms and flies |

Open a Pi shell and inspect the authoritative command help:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm uv run drone-deploy --ssh
# On the Pi:
cd ~/ai-drone
uv run drone-control --help
```

The safe starting point is:

```bash
uv run drone-control status --duration 10
```

`status` sends no arm, mode-change, or setpoint command.

## Pre-arm assessment

`status` answers "is the link alive". `preflight` answers the question that
actually blocks a flight test:

```bash
uv run drone-control preflight
```

It reads parameters, the EKF status report, the downward rangefinder, and the
battery, then prints one verdict per check and names whatever would stop a
GUIDED takeoff. It sends only stream and parameter requests: no arm, no mode
change, no setpoint, and no parameter write. It exits `2` when a blocker
remains, so it can gate a procedure.

Two failures are worth recognizing on sight:

- `arming_checks: ARMING_CHECK=0` means every pre-arm check is bypassed and the
  vehicle will not report a single `PreArm:` failure. `DroneController` refuses
  to arm in this state. Fix it with `drone-arming-checks restore`, adding
  `--without-gps` on this airframe because it has no GPS receiver; see
  [flight-controller configuration](DRONE_CONFIGURATION.md).
- `horizontal_position` fails when the EKF is in constant-position mode with no
  horizontal source. A GUIDED takeoff cannot work without one: `EK3_SRC1_POSXY`
  must name a working source (GPS outdoors, or optical flow indoors) before
  `hover` or `velocity-test` can do anything but be refused.

## The route up this aircraft can actually fly

`alt-hold-takeoff` is the one to reach for. The other two exist for reasons
that no longer hold:

- **GUIDED is refused by the vehicle itself.** Asked to run its pre-arm checks
  in GUIDED it answers `PreArm: Need Position Estimate`, because EKF3 starts
  optical-flow navigation only once it has detected a takeoff. Asked the same
  question in `ALT_HOLD` or `GUIDED_NOGPS`, it does not. That is the vehicle's
  verdict, obtained with `MAV_CMD_RUN_PREARM_CHECKS` and without arming.
- **STABILIZE puts raw motor thrust on the stick**, through a mapping two
  flights failed to pin down: a commanded 0.313 of stick produced 0.128 of
  throttle against a 0.263 hover, which neither a direct reading nor
  ArduPilot's mid-stick-is-hover curve predicts. Until that is measured on a
  bench with the propellers off, every STABILIZE throttle is a guess.

In ALT_HOLD the stick is a climb rate that ArduPilot bounds by
`PILOT_SPEED_UP` (0.25 m/s here), and ArduPilot owns the altitude loop, so
there is no thrust curve left to get wrong:

```bash
ssh -t -F /dev/null seb@192.168.4.1 'cd ~/ai-drone && .venv/bin/drone-control \
  alt-hold-takeoff --takeoff-alt 0.3 --max-alt 0.5 --duration 3 --climb 0.5 \
  --confirm-flight FLIGHT_TEST_READY'
```

Use `ssh -t`, and run it from a terminal you are sitting at: without one the
abort key cannot arm, and the command will say so.

This depends on the vehicle's vertical estimate, which is exactly what was
diverged on 2026-08-21 -- `EK3_SRC1_POSZ` named the rangefinder as the height
source while `EK3_RNG_USE_HGT` disabled using it for height, leaving the filter
with no vertical position source and a vertical velocity that ran away to
-38 m/s. It is now the barometer. Confirm before flying that `preflight`
reports a plausible vertical estimate rather than merely claiming one exists.

## Flying without a position estimate

This aircraft has no GPS receiver and navigates from the MTF-01P's optical
flow. EKF3 does not begin fusing flow until it has detected a takeoff — it
waits for the rangefinder to read about 5 cm more than it did at arming — so
the position estimate GUIDED requires does not exist while the aircraft is on
the floor. `hover` and `velocity-test` are therefore refused with
`Arm: Need Position Estimate`, and no parameter changes that: it is ArduPilot's
design, not a misconfiguration.

`nogps-takeoff` is the way up:

```bash
uv run drone-control nogps-takeoff --takeoff-alt 0.3 --max-alt 0.5 \
  --duration 3 --confirm-flight FLIGHT_TEST_READY
```

It arms in `GUIDED_NOGPS`, which needs no position, and climbs on the downward
rangefinder. With `GUID_OPTIONS` bit 3 clear — the ArduPilot default — the
`thrust` field of `SET_ATTITUDE_TARGET` is a *climb rate* rather than a
throttle: `0.5` holds altitude and `1.0` climbs at `PILOT_SPEED_UP`. ArduPilot
keeps both the attitude loop and the altitude loop; this command never asks for
raw motor power, and `DroneController` clamps every thrust it sends to
`[0.25, 0.80]` regardless of what it is asked for.

Once the aircraft is off the ground the EKF starts flow navigation, so a later
switch to `GUIDED` or `LOITER` becomes possible. That handover is not yet
implemented.

## Climbing on the throttle stick

`stabilize-takeoff` exists because the two position-free routes up have each
failed on this airframe in a different way: `ALT_HOLD` has never lifted it
cleanly off the floor, and the thrust question `nogps-takeoff` raised on
2026-08-20 is still open.

```bash
uv run drone-control stabilize-takeoff --takeoff-alt 0.3 --max-alt 0.5 \
  --duration 3 --confirm-flight FLIGHT_TEST_READY
```

It arms in `STABILIZE` with the throttle already overridden to `RC3_MIN`,
ramps onto a throttle built from the vehicle's own learned `MOT_THST_HOVER`,
climbs to the target on the downward rangefinder, hands altitude control to
`ALT_HOLD`, holds, and lands in `LAND`.

Three things about it are worth knowing before running it.

**STABILIZE has no altitude controller.** The throttle stick is motor thrust,
and the aircraft accelerates upward for as long as that thrust exceeds its
weight. Every limit during the climb is in `climb_in_stabilize`: a throttle
capped at `MOT_THST_HOVER` plus `MAX_THROTTLE_ABOVE_HOVER`, a ramp onto it, a
measured climb-rate limit, and the altitude ceiling. The autopilot contributes
none of them.

**The same stick means two different things.** Centred is "hold this altitude"
in `ALT_HOLD` and roughly half throttle -- close to twice hover -- in
`STABILIZE`. `DroneController` therefore checks the mode the vehicle reports in
its own heartbeat before every throttle it sends, and the handover changes mode
*first* and centres the stick second. The other order commands half throttle
for as long as the mode change takes to confirm.

**An override lapses.** `RC_OVERRIDE_TIME` is 3 s, and when an override expires
ArduPilot hands the channel back to the receiver -- of which this airframe has
none. The override is re-sent from every loop that can block, including the
waits for a mode confirmation and an arm confirmation, and it is released only
once the vehicle's own heartbeat reports it disarmed.

Rehearse it, and watch it stop, before it is flown:

```bash
uv run drone-rehearse stabilize-takeoff
uv run drone-rehearse stabilize-takeoff --fault throttle-runaway
```

## The abort key

Every flying `drone-control` command watches the terminal for a keypress.
Pressing any key **force-disarms the aircraft**: it does not request a
landing, because LAND is an altitude-controlled mode and answering a panic key
with one is what destroyed the aircraft on 2026-08-21. The motors stop and the
aircraft drops. From the half-metre this airframe is flown at that is a far
better outcome than a climb nobody can stop.

The command prints, before anything arms, whether the key is actually live:

```
abort key ARMED: press any key to cut the motors
abort key NOT ARMED (stdin is not a terminal; run the command with 'ssh -t'). ...
```

Read that line every time. Over SSH the key only works with a terminal
allocated, so run flights as:

```bash
ssh -t -F /dev/null seb@192.168.4.1 'cd ~/ai-drone && .venv/bin/drone-control ...'
```

**It is only as good as the link.** The command runs on the Pi and the
keystroke travels over Wi-Fi; if the link drops, the key does nothing and
neither does anything else on the laptop. It is a second pair of hands, not a
substitute for someone able to cut the battery.

## Rehearsing a flight without an aircraft

`drone-rehearse` runs the real `drone-control` command against
`ai_drone.sim.vehicle`, a MAVLink double that answers like ArduPilot Copter.
The flight code under test is the code that flies the aircraft -- same arming
gate, same guards, same LAND cleanup, same flight recording. Only the vehicle
is simulated, and the endpoint is always loopback, so a rehearsal cannot reach
a serial port.

```bash
uv run drone-rehearse hover
uv run drone-rehearse velocity-test
uv run drone-rehearse preflight
```

The reason this exists is `--fault`. A guard nobody has watched stop an
aircraft is a guard nobody should trust, and the only safe place to watch one
trip is here:

| `--fault` | What it proves |
| --- | --- |
| `refuse-arm` | An arm refusal aborts before anything spins |
| `no-takeoff` | A takeoff that never climbs times out and lands |
| `altitude-runaway` | The altitude ceiling stops an uncommanded climb |
| `battery-sag` | The battery guard lands on a collapsing pack |
| `stale-altitude` | A dead rangefinder stream lands the aircraft |
| `heartbeat-loss` | A dead telemetry link lands the aircraft |
| `ekf-divergence` | 2026-08-21 end to end: a climb that never lifts, a diverged vertical estimate, and a LAND that answers it with full throttle. The abort must disarm rather than request LAND |
| `land-climbs` | The same broken LAND under an aircraft that *did* take off. The abort must abandon LAND for STABILIZE on a below-hover throttle and fly it back down |
| `throttle-runaway` | A commanded throttle producing far more thrust than the learned hover value predicts is measured and stopped |
| `refuse-land` | An ignored LAND is reported, and the controller still does not force-disarm an airborne vehicle |

Every fault applies to `nogps-takeoff` as well as `hover`.

```bash
uv run drone-rehearse hover --fault battery-sag
```

A rehearsal with an injected fault succeeds when the fault was caught. The
command prints what the simulated vehicle actually did, so the recovery -- LAND
commanded, disarm refused while airborne, descent, disarm on the ground -- is
visible rather than assumed. Every live flight mode
requires the exact acknowledgement shown by its `--help`; do not bypass that
gate in wrappers or documentation.

## Control behavior

`ai_drone.flight.controller.DroneController` owns the MAVLink connection and
arm, mode, takeoff, velocity, stop, land, and cleanup transitions. Generic
battery, altitude-ceiling, and telemetry-staleness guards live in
`ai_drone.flight.guards` and apply to every flight mode.

The flight controller remains responsible for attitude stabilization and EKF
fusion. Body-frame velocity follows NED conventions:

- positive `vx` is forward and positive `vy` is right;
- positive `vz` is down; and
- positive yaw rate turns right.

The controller filters telemetry to the selected vehicle, rejects stale data,
requires a permitted `ARMING_CHECK` value before an arm path, verifies state
transitions, caps commands, and uses bounded timeouts. Cleanup lands only a
flight started by that controller instance.

## Required validation sequence

1. Run the repository checks, then rehearse every control mode and every
   `drone-rehearse --fault` case: rejected arm, stale telemetry, lost
   heartbeat, altitude runaway, battery collapse, and landing timeout.
2. Rigidly mount and calibrate the IMU, compass, optical flow, downward
   rangefinder, and any camera needed by later missions.
3. In an authorized configuration session, set a permitted `ARMING_CHECK`, give the
   EKF a working horizontal position source, resolve every pre-arm message, and
   define an appropriate indoor boundary/recovery behavior. `drone-control
   preflight` must report no blocker before this step is considered done.
4. With all propellers removed and the frame secured, verify motor numbering
   and direction using the [guarded motor procedure](BENCH_MOTOR_TEST.md).
5. With a safety pilot, tested RC override/kill path, protective enclosure,
   fresh battery, and clear area, perform the smallest restrained hover test.
6. Review range, flow, EKF, and battery recordings before expanding the
   envelope; then validate velocity one axis at a time.

AprilTag approach, autonomous search, and payload release are mission work,
not existing `drone-control` modes. Their additional prerequisites are in the
[AprilTag mission architecture](APRILTAG_MISSION.md).
