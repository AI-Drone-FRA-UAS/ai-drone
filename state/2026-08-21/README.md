# 2026-08-21 — a STABILIZE takeoff, and the loss of the aircraft

Live observations from one session. Not a specification: see
[flight-controller configuration](../../docs/DRONE_CONFIGURATION.md) for the
rules and [MAVLink control](../../docs/PI_MAVLINK_CONTROL.md) for the commands.

## Outcome

`drone-control stabilize-takeoff`, written during this session, was run twice
against the aircraft. Neither run ever climbed: the commanded throttle was
about half of hover and the aircraft stayed on the floor both times, which the
command correctly detected and timed out on.

Both times, the abort that followed the timeout requested LAND. The vehicle's
EKF vertical solution was diverged -- reporting an altitude of -10000 m and a
descent of 38 m/s while sitting on the ground -- and LAND is an
altitude-controlled mode. On the first run it lifted the aircraft gently to
about half a metre and held it there. On the second run it went to full
throttle in a single log sample and drove the aircraft into the ceiling of a
sports hall in one second.

The aircraft is destroyed. It was on a line that was not being held. Nobody was
hurt.

**The abort path is the defect.** Nothing should fly again until LAND has
stopped being this codebase's universal answer to "something is wrong"; see
"The defects this exposed".

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

## What was seen from the ground

Run 2 produced no captured output: its stdout was piped through `tail`, which
never flushed, and the Pi became unreachable before it could be read back. For
a while the only account of the second flight was the operator's -- that it
climbed, hovered "perfectly", then climbed away.

The log shows why that account and the software disagreed so completely. The
hover was real, but it happened *after* the command had given up and asked for
LAND, not during the climb the command was trying to fly. The operator was
watching the abort path, and so was the aircraft.

The `FlightRecorder` output for both runs is still on the Pi at
`~/ai-drone/artifacts/flights/`, unrecovered.

## What the dataflash log shows

The log was recovered intact over the developer USB link and is committed here
as `dataflash-log-2.bin`: 1 757 996 bytes, every byte range covered on the
first pass, beginning with a real `FMT` record and carrying 164 of them. The
2026-08-20 failure mode -- a missing first block and an undecodable remainder
-- did not recur.

It contains both runs, and it contradicts what was believed while writing the
first half of this file. **The rangefinder never failed, and neither guard was
ever presented with something to catch.**

### The aircraft never flew in STABILIZE

For all thirteen seconds of run 2 the rangefinder read a steady 0.02 m at
status 4, and `ThrOut` sat at 0.128 against a learned hover of 0.263. Run 1 is
identical. The commanded stick simply never produced enough thrust to lift the
aircraft, which is what the timeout was for and what it correctly reported.

### The LAND request is what flew it

`ThrOut` goes from 0.128 to 1.000 in the single log sample at the mode change.

| t | mode | EKF alt | EKF climb | baro | rangefinder | ThrOut |
| --- | --- | --- | --- | --- | --- | --- |
| 170.91 | STABILIZE | -10002.2 m | -38.2 m/s | 2.74 | 0.01 | 0.128 |
| 171.01 | **LAND** | -10001.7 m | -38.0 m/s | 3.14 | 0.01 | **1.000** |
| 171.21 | LAND | -10001.6 m | -35.9 m/s | 11.50 | 0.27 | 1.000 |
| 171.71 | LAND | -10001.0 m | -31.7 m/s | 18.17 | 2.00 | 1.000 |
| 171.91 | LAND | -10000.5 m | -29.6 m/s | 21.31 | 3.40 | 1.000 |
| 172.02 | LAND | \[ERR\] `Potential Thrust Loss (3)` | | | | |

The EKF's vertical solution was diverged for the entire session: an altitude of
about -10000 m and a reported descent of 38 m/s while the aircraft stood on the
floor. STABILIZE is a manual-throttle mode and ignores all of it, which is why
thirteen seconds passed uneventfully. LAND is an altitude-controlled mode. It
read a 38 m/s descent, commanded full throttle to arrest it, and put the
aircraft through the ceiling in one second.

The rangefinder tracked that climb correctly and immediately -- 0.01, 0.27,
0.80, 1.37, 2.00, 2.63, 3.40 m -- so the guards were not blind. By then the
code had left the climb loop and was inside `land()`, which waits for a disarm
and applies no altitude limit at all, and the whole event took under a second.

Run 1 was the same mechanism at a survivable magnitude. LAND at 13.0 s brought
`ThrOut` up to about 0.196 and the aircraft rose gently to roughly 0.5 m and
sat there until `LAND_COMPLETE` at 28.3 s. **That was the "perfect hover" the
operator saw. It was not produced by the STABILIZE climb; it was produced by
the LAND that followed it.**

## The defects this exposed

1. **LAND is used as the universal abort and is only as good as the altitude
   estimate.** `emergency_stop`, `abort_to_land`, `ensure_landed` and every
   guard in `flight/guards.py` route to LAND. On a vehicle whose vertical
   estimate is invalid, that hands the aircraft to a controller running on
   garbage. When the aircraft is still on the ground, the safe ending is a
   disarm.
2. **The flight is declared started before the aircraft leaves the ground.**
   `climb_in_stabilize` sets `_flight_started_by_controller` and `is_flying`
   immediately after arming, so a climb that never lifts is treated as an
   airborne emergency. The rangefinder read 0.02 m for twelve seconds; nothing
   consulted it before choosing how to abort.
3. **`preflight` checked an EKF flag rather than an EKF value.** It reported
   `vertical_position: EKF has a vertical position estimate` while that
   estimate was -10000 m. The bit that says a value exists is not the value.
4. **`land()` applies no altitude limit.** It sets the mode and waits for a
   disarm. The ceiling check in `update_telemetry` is suppressed once
   `_landing_commanded` is set, which is correct for a real descent and wrong
   for a LAND that climbs.

None of these are fixed here. The aircraft that would prove a fix no longer
exists, and the first version of this file confidently blamed the rangefinder
on the strength of a plausible story. The evidence came from the log, not from
reasoning about the code, and the fixes should be proven the same way.

## What was wrongly believed before the log was read

Recorded because the reasoning failed in an instructive way. Before recovering
the log, this file argued that the rangefinder had under-reported and that both
guards were blind to it through a common-mode sensor failure, and named the
unused `local_position_altitude` as the missing cross-check. That story fitted
every observation available at the time and was wrong in its central claim: the
rangefinder was accurate throughout. The unused second altitude source is still
a real gap, but it is not what happened here.

## The repair, later the same day

The aircraft was rebuilt and put back on the bench. Everything below was
measured through the link with the propellers fitted and the vehicle disarmed;
no motor was run.

**The vehicle-side cause.** `EK3_SRC1_POSZ` was `2`, naming the rangefinder as
the primary height source, while `EK3_RNG_USE_HGT` was `-1`, disabling use of
the rangefinder for height. Between them the filter had no vertical position
source at all, and its vertical velocity ran as an uncorrected integral of
accelerometer bias. That is the -10000 m.

The barometer had been working the whole time -- it sat at a stable 2.7 m in
the accident log while the EKF was ten kilometres underground -- and it is the
right height source for indoor flight on flow. Moving `EK3_SRC1_POSZ` to `1`:

| | reported altitude | reported vertical rate |
| --- | --- | --- |
| before, stationary on the floor | flag said "estimate available" | **-17.78 m/s** |
| after, stationary on the floor | +0.05 m | **+0.01 m/s** |
| after a full flight-controller reboot | -0.01 m | **+0.00 m/s** |

The reboot matters: setting the parameter back to `2` did **not** bring the
divergence back, so it is established at EKF initialisation rather than during
operation. The accident happened on a freshly booted aircraft, and the third
row is that same condition with the fix in place.

`MOT_SPIN_MAX` went from `0.95` to `0.40` as a second, mode-independent limit.
It caps motor output at 40% against a learned hover of 26%, so the full-throttle
excursion of the accident is no longer physically available. It is a damper on
the consequence, not a fix for a cause.

**What the guard proof does and does not show.** The new `vertical_position`
check was watched reading real values off the aircraft and passing on healthy
ones. It has *not* been watched failing on real hardware: the attempt to
reproduce the accident configuration did not reproduce the divergence. The
guard is proven against the simulator and against the recorded accident values,
not against a live divergence.

## Also unaddressed, and flagged before the flight

These were raised before anything armed and were not acted on:

- `FS_THR_ENABLE=0`. The throttle failsafe is off. `RC_OVERRIDE_TIME` is 3 s,
  and this airframe has no receiver, so an override that stops being sent
  hands the throttle to nothing with no failsafe behind it. Not implicated in
  this accident -- the log shows the override held throughout -- but still
  true.
- `BATT_FS_LOW_ACT=0` and `BATT_FS_CRT_ACT=0`. Both battery failsafes are off.
- `FENCE_ENABLE=0`. There is no vehicle-side altitude limit. Every limit was
  in our software, on the companion computer, over Wi-Fi. A fence with
  `FENCE_TYPE=1` and a low `FENCE_ALT_MAX` is the only limit that survives our
  software being wrong. It is also the only one of the three that would
  plausibly have stopped this: the breach action runs on the vehicle, in the
  same second, without waiting for a companion computer to notice.

## An error in how this was run

Between run 1 and run 2 the operator was told, in writing, that an identical
command would do nothing -- "das Fluggerät wird sich wieder nicht bewegen" --
and the option chosen was labelled harmless. The camera was rolling on the
strength of that, and the line was let go.

The log makes the shape of that error precise, and it is not the obvious one.
The prediction about the *climb* was right: the aircraft did not move, for the
reason given. What was wrong was the word harmless, which quietly assumed that
because the interesting part of the command would do nothing, the whole command
would do nothing. Run 1 had already ended in a fifteen-second LAND and a
`PreArm: Check mag field: 905` -- an abort taking fifteen seconds to bring down
an aircraft that had supposedly never left the floor is a plain contradiction,
it was in front of me, and I did not look at it.

Predicting one phase of a sequence is not predicting the sequence. The abort
path had never been flown on this aircraft and was described as safe on the
strength of simulator runs against a vehicle model that had no EKF in it at
all.

## Recovery

- The dataflash log has been recovered and is committed here as
  `dataflash-log-2.bin`. Log 1 on the controller, 6.24 MiB, is the 2026-08-20
  flight and is still worth pulling the same way now that the method works.
- `~/ai-drone/artifacts/flights/` on the Pi holds the `FlightRecorder` output
  for both runs, including the sampled altitudes and the events, if the card
  survived.
- Do not reconnect the pack to inspect the airframe. A damaged ESC or a
  pinched lead can spin a motor the moment power returns.

## Evening: the first real liftoff, and why it overshot

The aircraft flew. It left the ground under its own power for the first time
since the crash, climbed past the 0.30 m target, and the altitude ceiling
stopped it at 0.57 m. Recorded in `alt-hold-takeoff-overshoot.tlog`.

### What the operator saw

"Das Abheben hat geklappt, er hat allerdings zu schnell hochreguliert." The
request was to ramp the thrust gradually so the ceiling would not trigger.

### What the telemetry shows

| t (s) | armed | rangefinder | pressure (hPa) | reported climb | throttle | motor 1 |
|---|---|---|---|---|---|---|
| 0.26 | no | 0.02 m | 994.471 | **-1.89 m/s** | 0 % | 1000 |
| 2.67 | no | 0.02 m | 994.466 | **-2.00 m/s** | 0 % | 1000 |
| 3.52 | yes | 0.02 m | 994.464 | -2.22 m/s | 0 % | 1000 |
| 4.91 | yes | 0.02 m | 994.455 | **-2.40 m/s** | 0 % | 1044 |
| 5.77 | yes | 0.02 m | 994.405 | -2.16 m/s | 48 % | 1316 |
| 6.04 | yes | 0.12 m | 994.284 | -1.64 m/s | **86 %** | 1378 |
| 6.32 | yes | 0.29 m | 993.905 | -1.32 m/s | **92 %** | 1336 |
| 6.65 | yes | 0.53 m | 993.663 | -0.76 m/s | 75 % | 1374 |
| 12.87 | yes | 0.02 m | 994.431 | **+1.02 m/s** | 9 % | 1168 |
| 13.86 | yes | 0.02 m | 994.469 | **+1.27 m/s** | 0 % | 1000 |

The first two rows are the whole finding. The aircraft is **disarmed, motors
stopped at 1000, rangefinder pinned at 0.02 m** -- and it reports itself
descending at 1.89 m/s. That is not prop wash and not a flight condition. The
estimate was already wrong before anything was commanded.

The operator supplied the cause unprompted: *"Es kann sein das einige Werte
falsch sind weil wir die Drohne zwischendurch angehoben haben."* The aircraft
had been picked up. Its barometer-derived vertical velocity was still ringing.

ALT_HOLD's altitude controller acts on that number. Believing it was falling at
2.3 m/s, ArduPilot commanded **92 % throttle against a 27.7 % hover**
(`MOT_THST_HOVER = 0.277447`). The aircraft broke free with the whole surplus
and went 0.02 m to 0.57 m in 0.4 s. The ceiling guard caught it correctly.

The last two rows close the case: seven seconds after touchdown, sitting
motionless on the floor, the same estimate has swung to **+1.27 m/s**.

### The control confirmed it

Thirty seconds later, aircraft still and untouched, read-only:

```
  1.0s armed=False  climb=-0.0010 m/s  rng=0.02 m  press=994.3998 hPa
 29.0s armed=False  climb=-0.0113 m/s  rng=0.02 m  press=994.4142 hPa
```

Steady at -0.01 m/s. The sensors are fine. Handling is what breaks them, and
only for a while.

### What was wrongly believed before the telemetry was read

- That the overshoot was caused by too high a *requested* climb rate. It was
  not. `PILOT_SPEED_UP = 25 cm/s`, so `--climb 0.5` asked for **0.125 m/s**
  and the aircraft did roughly 0.8 m/s -- six times the rate ArduPilot was
  supposed to bound it to.
- That ramping and tapering the request would fix it. The overshoot ran from
  0.25 m to 0.57 m, and `hold_started` -- the throttle stick returning to
  **centre**, asking for no climb at all -- is timestamped inside that window.
  Shaping a request the aircraft was not following could not have helped.
- That prop wash on the barometer was to blame. Ruled out by the motors being
  stopped at 1000 while the estimate already read -1.89 m/s.

### What changed

- `wait_for_vertical_estimate_to_settle`: the vehicle must report itself within
  0.10 m/s of stationary, continuously for 3 s, before this software will arm
  it. The console prints the estimate once a second while it waits.
- `_shaped_climb`: the requested rate is ramped from zero over 3 s and tapered
  to zero over the last 0.20 m below the target. This was the change the
  operator asked for. It is a real improvement to the liftoff and it is *not*
  what fixes the overshoot -- kept, and labelled as such.
- The live console line gained an `ask=` column, so the requested rate and the
  measured rate are visible side by side. On this flight they differed by 6x.

## Evening, second attempt: it did not lift, and that closed the case

The settle gate worked. The vehicle reported +0.013 m/s and held it for three
seconds before the software would arm. Then the aircraft sat on the floor for
twenty seconds and never moved. Recorded in
`alt-hold-takeoff-no-liftoff.tlog`.

| t | rangefinder | asked-for climb | **vehicle's reported climb** |
|---|---|---|---|
| 6.0 s | 0.020 m | 0.00 | +0.01 m/s |
| 9.0 s | 0.020 m | 0.50 | **+0.80 m/s** |
| 12.1 s | 0.020 m | 0.50 | **+3.09 m/s** |
| 16.1 s | 0.020 m | 0.50 | **+4.27 m/s** |

The rangefinder read 0.020 m for every one of 359 samples. The vehicle believed
it was climbing at 4.27 m/s, so ALT_HOLD held the throttle down against a climb
that was not happening, and it never left the ground. This is the mirror image
of the overshoot an hour earlier, from the same cause.

### The cause, measured

| t | armed | motors | \|accelerometer\| | vibration Z | roll / pitch |
|---|---|---|---|---|---|
| 1.5–6.1 s | no | 1000 | **9.74–9.80 m/s²** | 0.01 | 0.5° / 2.3° |
| 9.1–25.7 s | yes | 1085–1161 | **10.55–10.74 m/s²** | 0.17 | 0.5° / 2.3° |

A stationary aircraft measures exactly one g whatever its orientation. With the
motors stopped this one measures gravity correctly. With them turning at idle
spin -- 1090 of a 1000–1400 usable band, nowhere near hover -- it measures
**0.85 m/s² too much**, with the attitude unchanged and the vibration metric at
0.17. EKF3 integrates that difference: 0.75 m/s² of drift per second, which is
exactly the slope of the reported climb between 8 s and 12 s.

The VIBE metric cannot see this. It measures variance, and a rectified
vibration appears as a DC offset instead. This is the same +0.566 m/s² armed
bias noted on 2026-08-20 and left unexplained; it is now the confirmed cause of
two failed flights and it is not a software fault.

### What was wrongly believed before this telemetry was read

- That the settle gate would be enough. It fixed what it was built for -- the
  ringing left by handling -- and the aircraft still could not fly, because the
  estimate is corrupted again the moment the motors turn.
- That prop wash on the barometer was involved. The barometer is steady to
  0.03 hPa across the whole run; the pressure trace is flat while the reported
  climb runs to +4.27 m/s.
- That vibration would show up in `VIBE`. It does not, and the two flights
  before this one were analysed on the assumption that it would.

### What changed

`ai_drone/mavlink/accel_bias.py` and `drone-motor-test` now measure this
directly: the motor test takes a baseline during its countdown and compares it
with the readings while the motors run, then names the number. It needs the
propellers off, which is the point -- with them off, the same measurement
separates an unbalanced or damaged propeller from a bent shaft, a hard-mounted
flight controller, or an electrical coupling.

**Until that measurement is made and the shift is gone, ALT_HOLD cannot fly on
this aircraft.** Both failure directions are the same fault.
