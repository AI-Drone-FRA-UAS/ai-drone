# 2026-08-21 — a STABILIZE takeoff, and the loss of the aircraft

Live observations from one session. Not a specification: see
[flight-controller configuration](../../docs/DRONE_CONFIGURATION.md) for the
rules and [MAVLink control](../../docs/PI_MAVLINK_CONTROL.md) for the commands.

## Outcome

The aircraft flew twice under `drone-control stabilize-takeoff`, a command
written during this session. The first run never left the ground. The second
run, with the identical command, climbed, hovered, and then climbed away into
the ceiling of a sports hall. The aircraft is destroyed. It was on a line, but
the line was not being held. Nobody was hurt.

**Nothing should fly again until the two questions in "What is not known"
below are answered from the dataflash log.**

## What is established

Facts observed directly through the link, in the order they happened. Times
are the Pi's own clock.

| Time | Observation |
| --- | --- |
| ~17:23 | Pack at 14.42 V, on the 14.4 V abort threshold. Swapped. |
| ~17:34 | Fresh pack at 16.43 V. Pi rebooted with the power cut. |
| ~17:40 | New code deployed; rehearsed clean against the simulator *on the Pi*. |
| ~17:50 | First location: `PreArm: Check mag field: 1050, max 875`. Arming blocked. |
| ~17:56 | Sports hall: field 739 mGauss, stable over 201 samples, no PreArm complaint. |
| ~17:57 | `preflight`: every check passed except `horizontal_position`. Pack 16.26 V. |
| run 1 | `--takeoff-alt 0.3 --max-alt 0.5 --duration 3 --climb 0.05`. Never lifted. |
| run 1 | Climb timed out at 12 s, aborted to LAND, disarmed. Verified afterwards: `mode=LAND armed=False altitude=0.02 m battery=16.19 V`. |
| run 2 | Identical command. Climbed, hovered, then climbed away. Aircraft lost. |
| run 2 | The command ran past 300 s against an expected worst case of about 30 s. |
| since | The Pi has not answered a ping. |

The compass failure was **environmental, not a calibration fault**. The same
airframe read 1050 mGauss in the first location and 739 in the sports hall
with nothing changed in between. A recalibration would have been twenty
minutes spent on the wrong problem. `state/2026-08-20/README.md` still lists a
recalibration as outstanding; that entry should be read with this in mind.

## What is eyewitness account, not telemetry

Run 2 produced **no captured output at all**. Its stdout was piped through
`tail`, which never flushed, and the Pi became unreachable before anything
could be read back. Everything known about the second flight comes from the
operator watching it:

- it climbed,
- it hovered, described as "perfect",
- it then climbed away and struck the ceiling.

The flight recording written by `FlightRecorder` is on the Pi at
`~/ai-drone/artifacts/flights/`, unrecovered. So is the dataflash log on the
flight controller.

## What is not known

Two questions, and neither should be guessed at.

**Why did an identical command do nothing once and fly once?** Run 1 held
hover throttle plus a margin for twelve seconds against the floor and did not
move. Run 2 lifted. Any explanation has to account for both. The model derived
from run 1 -- that ArduPilot maps mid stick to `MOT_THST_HOVER` in STABILIZE,
so the commanded PWM 1308 was about 16.5 % throttle against a 26.3 % hover --
explains run 1 and is contradicted by run 2. This is the **same unresolved
thrust question as 2026-08-20**, in a different mode, and it has now cost the
aircraft. The props-off bench test that would have settled it has still never
been run.

**Why did neither altitude guard stop the climb?** `max_altitude` was 0.5 m
and the measured-climb limit was 0.60 m/s. The aircraft reached a sports-hall
ceiling. Both guards were in the flown build and both had been watched
stopping a simulated aircraft the same afternoon.

## The defect this exposed

Every altitude limit in this codebase reads `DroneController.current_altitude`,
and that value comes from exactly one place: the downward rangefinder.

```
guards.check_safety_guardrails  -> drone.current_altitude  -> DISTANCE_SENSOR
controller.update_telemetry     -> self.current_altitude   -> DISTANCE_SENSOR
climb_in_stabilize climb rate   -> self.current_altitude   -> DISTANCE_SENSOR
```

ArduPilot's own altitude controller uses that same rangefinder. So a
rangefinder that under-reports makes the autopilot climb *and* makes every one
of our guards see a low, healthy-looking altitude. The staleness check does not
help: a confidently wrong reading is not a stale one. This is a common-mode
failure, and it is consistent with "hovered perfectly, then climbed away, and
nothing complained".

`DroneController` already receives a second, independent altitude.
`LOCAL_POSITION_NED` is decoded into `self.local_position_altitude` at
`ai_drone/flight/controller.py:270`, and **no guard has ever read it**. The
cross-check needed to catch this was already arriving on the wire and was
being thrown away.

This is deliberately not fixed in this commit. The aircraft that would prove
the fix no longer exists, and adding untested behaviour to flight code on the
strength of a plausible story is how the afternoon went wrong in the first
place. It is written down as the thing to do first, not done quietly.

## Also unaddressed, and flagged before the flight

These were raised before anything armed and were not acted on:

- `FS_THR_ENABLE=0`. The throttle failsafe is off. `RC_OVERRIDE_TIME` is 3 s,
  and this airframe has no receiver, so an override that stops being sent
  hands the throttle to nothing with no failsafe behind it. The flight code
  refreshes the override from every blocking loop specifically because of
  this, but a stalled process defeats that -- and run 2 did run far past its
  own worst case.
- `BATT_FS_LOW_ACT=0` and `BATT_FS_CRT_ACT=0`. Both battery failsafes are off.
- `FENCE_ENABLE=0`. There is no vehicle-side altitude limit. Every limit was
  in our software, on the companion computer, over Wi-Fi. A fence with
  `FENCE_TYPE=1` and a low `FENCE_ALT_MAX` is the only limit that survives our
  software being wrong, and it is the one thing that might have stopped this.

## An error in how this was run

Between run 1 and run 2 the operator was told, in writing, that an identical
command would do nothing -- "das Fluggerät wird sich wieder nicht bewegen" --
and the option chosen was labelled harmless. The camera was rolling on the
strength of that. The prediction came from a model built on a single
observation, stated with far more confidence than one data point supports.

A model that explains one run is a hypothesis. Flying it is how it gets
tested, and it should have been described as a test with an uncertain outcome,
not as a repeat of a known-safe non-event.

## Recovery

- The flight controller's dataflash log is the only record of run 2. Read it
  **over the developer USB link with a ground station**, not over MAVLink: the
  2026-08-20 attempt lost the first 64 KiB block holding the `FMT` records and
  the log could not be decoded at all.
- `~/ai-drone/artifacts/flights/` on the Pi holds the `FlightRecorder` output
  for both runs, including the sampled altitudes and the events, if the card
  survived.
- Do not reconnect the pack to inspect the airframe. A damaged ESC or a
  pinched lead can spin a motor the moment power returns.
