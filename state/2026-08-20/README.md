# 2026-08-20 — first arm, first flight, and a flyaway

Live observations from one session. Not a specification: see
[flight-controller configuration](../../docs/DRONE_CONFIGURATION.md) for the
rules and [MAVLink control](../../docs/PI_MAVLINK_CONTROL.md) for the commands.

## Outcome

The aircraft armed for the first time, and flew for the first time. The flight
climbed past its commanded altitude, stayed airborne past its commanded hold,
and was stopped by physically disconnecting the battery. It was restrained on a
line; without that it would have been lost. Nobody was hurt.

**Do not fly `nogps-takeoff` again until the thrust question below is settled.**

## Configuration changed on the vehicle

| Parameter | Before | After | Why |
| --- | --- | --- | --- |
| `ARMING_CHECK` | 0 | 1, then 1043958 | With `0` the vehicle reports no `PreArm:` failure at all. `1043958` is every check except the two GPS ones. |
| `RNGFND2_TYPE` | 10 | 0 | Configured for a forward MT-15 that does not work. It was the only pre-arm failure once the checks were back on. |
| `GPS1_TYPE` | 1 | 0 | No GPS receiver exists on this airframe; `SERIAL3_PROTOCOL=44` is not even a GPS port. |
| `ANGLE_MAX` | 3000 | 1500 | 15° instead of 30° for a restrained first flight. |

`GPS_PRIMARY` is still `0` and the vehicle complained `GPS 1: primary but TYPE 0`
while the GPS checks were enabled. Clearing the two GPS check bits resolved it.

`parameter-diff-since-2026-08-18.txt` holds the full comparison against the
previous capture. It contains changes that were **not** made in this session
and that nobody has accounted for:

- `FS_THR_ENABLE` 3 -> 0. **The throttle failsafe is disabled.** This is a
  safety setting and it was on two days ago.
- `AVOID_ENABLE` 3 -> 2, `AVOID_BEHAVE` 0 -> 1, `AVOID_MARGIN` 2 -> 1.

The unversioned `~/fly_and_land.py` writes parameters, which is the likeliest
source. Establish who changed these and whether they should stay before the
next flight.

## Sensors

- Downward LiDAR (MTF-01P, `RNGFND1_TYPE=10`, orientation 25): healthy, 0.02 m
  on the floor, tracked correctly to 1.15 m when the aircraft was lifted.
- Optical flow (same unit, `FLOW_TYPE=5`): healthy. Quality median 50 on the
  floor, 100–132 held at 0.6–1.1 m, 176 in a later sample.
- Compass: field strength ~720 mGauss at rest against an 875 limit, and 941
  while the aircraft was moved. One arm attempt failed with
  `Check mag field: 878, max 875, min 185`. `COMPASS_OFS_Y = -298` is large.
  **A recalibration is outstanding.**
- No GPS receiver. Fix type 1, 0 satellites, `eph=9999`.

## Why GUIDED cannot take off from the ground

EKF3 does not begin optical-flow navigation until it detects a takeoff, which
it does by watching for the rangefinder to read about 5 cm more than it did at
arming. Held at 0.6–1.1 m while disarmed, the EKF still never produced
`pos_horiz_rel` or `pred_pos_horiz_rel`; armed on the floor in `ALT_HOLD` for
10 s, it stayed in `const_pos_mode`. So the position estimate GUIDED requires
exists only after leaving the ground. This is ArduPilot's design, not a
misconfiguration, and no parameter changes it.

## The open question

`nogps-takeoff` commands a climb by writing the `thrust` field of
`SET_ATTITUDE_TARGET`, on the reading that with `GUID_OPTIONS=0` ArduPilot
interprets it as a *climb rate* (0.5 holds) rather than a throttle. If that
reading is wrong, the commanded 0.7 was ~70% power against a 26% hover thrust,
and the 0.5 "hold" was still nearly double hover — which matches the observed
climb exactly.

**This must be settled from evidence before the next flight**, and the bench is
now the only way: with the propellers removed, arm in `GUIDED_NOGPS`, send
`thrust=0.9`, and watch `SERVO_OUTPUT_RAW`. A throttle interpretation drives
the motors to ~90% immediately; a climb-rate interpretation runs the altitude
controller instead.

The onboard log cannot answer it. All 6.24 MiB were downloaded over MAVLink
except the first 64 KiB block, which reads as empty — and that block held the
~95 `FMT` records that define every other message. Exactly one `FMT` header
survives in the remainder, so nothing can decode the rest. The raw download is
at `~/ai-drone-live/artifacts/logs/dataflash-log-1.bin` on the Pi; it is not
committed because without the format definitions it is undecodable. Reading the
chip over the developer USB link with a ground station would recover the whole
log including that first block.

## Also here

`pi-home-fly_and_land.py` is a copy of `~/fly_and_land.py` from the Pi, which
was unversioned and would have been lost. It is a different approach to the
same problem: RC override of the throttle channel in `ALT_HOLD` (PWM 1680 to
climb, 1500 neutral), plus parameter writes including `PILOT_SPEED_UP=80`. The
live value is 25, so it either never completed or was reset. Not reviewed, not
adopted, kept so it is not lost.

## Notes

- Flight artifacts were written under `/tmp` on the Pi and were destroyed when
  power was cut. The runtime now lives at `~/ai-drone-live`.
- The Pi survived the abrupt power loss: journal recovery ran, no failed
  services, root filesystem clean. `apt-daily*.timer` remains disabled.
- The deployed tree at `~/ai-drone` is still a much older layout and was left
  untouched all session.
