# Payload Drop Mechanism

The delivery mechanism from goal 1.3 of the
[Implementation Reference](drone-project.md): a 9 g micro servo releases a
3D-printed bracket on command. This document covers the servo itself, the
three ways it can be driven, and what is verified today.

**What works today:** the release is triggered by the detection. When the
downward camera has held the same allowed `tag36h11` marker over several
consecutive frames, the Pi drives the servo over GPIO and the payload falls,
with the release latched so it cannot repeat. That is verified on the ground
and hand-held, not in autonomous flight — see
[AprilTag-Erkennung und Abwurf](https://ai-drone-fra-uas.github.io/ai-drone/apriltag.html).

---

## 1. Servo specification

Miuzei SG90 / MS18-F, two units provided.

| Property | Value |
|----------|-------|
| Operating voltage | 4.8 V to 6.0 V |
| Commanded pulse range | **900 µs to 2100 µs** |
| Neutral position | 1500 µs |
| Operating travel | 120° ± 10° over that pulse range |
| Mechanical limit | 200° ± 1° — a hard stop, **not** a commandable range |
| No-load current | 400 mA at 4.8 V, 500 mA at 6.0 V |
| Stall current | 1300 mA at 4.8 V, **1600 mA at 6.0 V** |
| Direction | 900 µs → 2100 µs turns counter-clockwise |

Two numbers drive every decision below:

- **Never command outside 900–2100 µs.** The 200° figure is where the gears
  hit their stop. Driving into it stalls the servo, and a stalled servo pulls
  its full 1.6 A until something gives.
- **1.6 A is more than a logic rail wants to supply.** Any loaded test needs a
  supply that can deliver it, with the grounds tied together.

---

## 2. The three drive paths

The servo has been driven three different ways during the project. They are
not alternatives to pick between at random — each one exists for a different
stage.

| Path | Purpose | Status |
|------|---------|--------|
| Arduino bench | Characterise the servo away from the drone | Verified — see [servo_instruction.md](../servo_instruction.md) |
| Raspberry Pi GPIO | Drive the mechanism from the companion computer | Verified with `test_servo.py`, and the path the AprilTag drop uses |
| ArduPilot servo output | Trigger the drop from the RC transmitter or a mission | **Not yet configured** — see section 5 |

### 2.1 Arduino bench test

An `arduino-cli` sketch on a Leonardo-compatible board, servo signal on `D10`.
This was used to confirm the pulse range and the direction of travel before
anything was mounted on the airframe. The full command sequence — installing
the core, compiling, uploading, and stopping the sketch — is in
[servo_instruction.md](../servo_instruction.md).

### 2.2 Raspberry Pi GPIO

`test_servo.py` drives the servo from the Pi's software PWM.

```text
Pi pin 2  — 5 V      → servo VCC   (red)
Pi pin 6  — GND      → servo GND   (brown/black)
Pi pin 32 — BCM 12   → servo signal (yellow/orange)
```

Deploy and run it from the developer machine:

```bash
uv run drone-deploy --servo
uv run drone-deploy --servo --mode manual
uv run drone-deploy --servo --mode center
```

| Mode | Behaviour |
|------|-----------|
| `sweep` *(default)* | Runs continuously between the minimum and maximum pulse |
| `manual` | Interactive prompt — command individual positions |
| `center` | Holds 1500 µs, which is the position to assemble the bracket in |

Useful flags: `--pin` (BCM number, default 12), `--min-us` / `--max-us`
(default 900/2100), `--sweep-delay`, `--sweep-step`. The script refuses to run
anywhere that is not a Raspberry Pi.

> **Power warning.** The Pi's 5 V pin is acceptable for a short unloaded sweep.
> Under load the servo's inrush will brown out a Pi Zero 2 and reboot it —
> mid-flight that means losing the companion computer. Use the drone's
> regulated 5 V rail or a separate supply for anything carrying weight, and
> tie the grounds together.

### 2.3 ArduPilot servo output — the intended flight configuration

Driving the servo from the flight controller rather than the Pi is what the
poster describes and what the project is aiming at, because it puts the drop
on the same failsafe path as the rest of the aircraft:

- The pilot can trigger it from an RC channel without the Pi being alive.
- A mission can trigger it with `MAV_CMD_DO_SET_SERVO`.
- The output follows the FC's own arming and failsafe state.

This is **not configured yet.** In the current parameter backup
`params/flywoo-f745_copter-4.6.3_2026-06-09.param`, outputs 1–4 are the motors
(`SERVO1_FUNCTION=34`, `SERVO2_FUNCTION=35`, `SERVO3_FUNCTION=36`,
`SERVO4_FUNCTION=33`) and every output from `SERVO5_FUNCTION` upward is `0`,
meaning unassigned. Section 5 lists what still has to be done.

---

## 3. Mechanical design

The bracket is designed in Tinkercad, sliced in Cura, and printed in PLA. The
servo horn holds a release arm that retains the payload; commanding the servo
from neutral to one end of its travel swings the arm clear and drops it.

Design constraints that come out of the numbers above:

- The arm must reach its open and closed positions **inside** 900–2100 µs, so
  that neither end rests against the mechanical stop.
- Assemble the horn with the servo held at 1500 µs (`--mode center`), so the
  usable travel is symmetric around neutral.
- The payload must not hang where it blocks the downward-facing MTF-01P; the
  sensor needs a clear view of the ground for altitude and position hold.

Print files and slicing notes are in
[Frame Extension and 3D Prints](FRAME_AND_3D_PRINTS.md).

---

## 4. Safety

- The drop mechanism is only ever exercised with the **propellers removed** or
  the drone secured, until the release itself is repeatable on the bench.
- Test the servo with the flight battery disconnected wherever the servo can
  be powered separately.
- Never leave the servo commanded against a mechanical stop. If it buzzes and
  does not move, cut its power immediately — that is the 1.6 A stall case.
- Dropping anything over a person is out of the question. All drop tests
  happen inside the flight safety net, over a marked area.

---

## 5. Remaining work

1. **Pick a spare output** on the GN745 AIO and confirm from the pad map that
   it is a genuine PWM output that is not part of the motor group.
2. **Assign the function.** For RC passthrough, set that output's
   `SERVOn_FUNCTION` to the `RCINx` value matching the transmitter channel
   (51 = RCIN1 through 66 = RCIN16). To trigger it from a mission instead,
   leave the function at `0` and command it with `MAV_CMD_DO_SET_SERVO`.
   Verify the behaviour on the bench before trusting either.
3. **Set the limits** with `SERVOn_MIN=900`, `SERVOn_MAX=2100`,
   `SERVOn_TRIM=1500`, so ArduPilot cannot command past the safe range.
4. **Back up the parameters first.** Save the current set to `params/` before
   writing anything, as with every other configuration change.
5. **Record the result** here and in [Drone Configuration](DRONE_CONFIGURATION.md),
   and fill in the drop section of the [poster](poster/README.md).
