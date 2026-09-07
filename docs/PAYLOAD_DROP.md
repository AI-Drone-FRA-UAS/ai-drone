# Payload servo and release mechanism

The current project controls the payload servo directly from the Raspberry Pi
on **BCM12 (physical pin 32)** with a common ground and an appropriately rated
regulated supply. It is separate from the FC motor outputs. Neither a successful
GPIO write nor an observed pulse proves that the mechanism moved: this servo
has no position-feedback channel in the project.

The August project presentation reported tag-triggered release on the ground
and while hand-held. That historical result is not a current mechanism test or
an autonomous-flight result. Use the latest `state/` capture for what has actually
been checked on the present airframe.

## Maintained tools

- `drone-servo` is the explicit bench tool. Read its help and follow the physical
  confirmation gates with propellers removed, the frame secured and the linkage
  clear. Establish safe endpoints for the attached mechanism before moving it.
- `drone-inspect` records camera and sensor data; ordinary inspection does not
  pulse the servo.
- `drone-tag-servo-record` is a separately authorized armed-flight recording
  tool with bounded pulses. It neither arms the FC nor implements a target
  approach or search mission. See [its operating documentation](ARMED_TAG_SERVO_RECORDING.md).

The installed command options are shown without actuator movement by:

```bash
uv run drone-servo --help
uv run drone-tag-servo-record --help
```

The previous `test_servo.py`, automatic sweeps through `drone-deploy --servo`,
Arduino sketches and unreviewed `mission_drop.py` are historical implementations.
Their original revisions remain in the [archive](../notes/archive/README.md).
Do not substitute their pulse limits or wiring assumptions for measurements of
the present mechanism.

## Mechanical and electrical checks

The supplied kit lists Miuzei SG90/MS18-F-class micro servos. Earlier bench
notes describe 900–2100 µs and up to 1.6 A stall current; these are historical
component figures, not a verified safe travel range or measured consumption for
the mounted release. Confirm the actual model and the mechanism's endpoints.

The servo supply must tolerate starting and stalled-load current without
browning out the Pi. Connect its ground to the signal source's ground. Keep the
payload, release arm and cables out of the downward MTF-01P and camera views.
Inspect the mounting and load retention before each authorized release test.

No ArduPilot payload-servo output or RC-triggered release is established by
this document. Do not repurpose a motor output. The current no-receiver flight
workflow cannot assume a radio release or kill switch exists.

See [hardware configuration](DRONE_CONFIGURATION.md), [frame files](FRAME_AND_3D_PRINTS.md)
and [AprilTag architecture](APRILTAG_MISSION.md) for related details.
