# Disarmed flight-controller check

Use `drone-check` after a firmware update or before collecting a walkthrough.
It opens the project FC at MAVLink system/component 1/1, requires a fresh
disarmed heartbeat, reads the reviewed configuration and records ten seconds
of telemetry. It never arms, changes mode, writes parameters, sends RC
overrides or operates a motor or servo. Closing this command only closes its
connection; it does not invoke the flight controller's LAND cleanup.

On the deployed Pi:

```bash
cd ~/ai-drone
uv run --locked --group raspi drone-check --prearm
uv run --locked --group raspi drone-check --json > check.json
```

On the laptop with the project FC connected by USB:

```bash
uv run --locked drone-check --prearm
```

Automatic discovery selects only the exact project USB identifier or
`/dev/serial0` on a verified Raspberry Pi. Another USB or network endpoint
requires explicit `--device` selection; the command never guesses another
ArduPilot USB aircraft. Every selected endpoint must still supply the correct
ArduPilot quadrotor heartbeat at 1/1. Use `--device /dev/serial0`
to select the Pi UART explicitly. Baud defaults to 115200. Do not run this at
the same time as a recorder, flight controller, configuration capture or
another serial client.

Before opening a serial port or setting its baud rate, the command checks for
existing owners with Linux `fuser` (from `psmisc`). On the Pi it also refuses an
active walkthrough service, including its startup and shutdown phases.
A busy or uninspectable device returns status 1 without opening it or stopping
the other process. These checks need no sudo and cover the normal same-user
recorder; they do not provide an atomic lock against a new, noncooperating
client or guarantee visibility into another user's hidden descriptors.
Serial-owner inspection currently requires Linux. Explicit network endpoints
use separate sockets and do not receive the local serial-owner check.

The default parameter phase has one total 15-second deadline and retries only
missing values once. `--duration` controls the subsequent sensor collection
(default 10 seconds); `--timeout` bounds each fresh-heartbeat wait (default
5 seconds). The complete command therefore takes longer than its collection
duration. `--prearm` additionally requests ArduPilot's diagnostic pre-arm checks
and includes observed status text. A command acknowledgement is not treated
as proof that all pre-arm checks passed.
If the optional diagnostic request is rejected or its acknowledgement is
missing, the report records that failure and returns status 1.

## Reading the result

Exit status 0 means the requested bench checks passed. Status 1 means a missing
or stale required stream, firmware/configuration mismatch, low or unknown
battery, reported sensor/pre-arm problem, nonzero RC channel count, armed
heartbeat or connection failure. Invalid arguments use status 2. The JSON
report includes received message counts, parameter values, errors, warnings
and the exact endpoint and firmware expectation.

The command uses the active software's exact firmware version/commit gate and
the same no-GPS configuration constants as the guarded hover controller. This
whole-airframe check additionally requires the integrated forward MT-15
configuration. It reports:

- Battery voltage, with the existing 14.4 V operational threshold by default;
  `--min-battery` can select another explicit bench threshold.
- Separate forward and downward distances and whether each sample is inside
  the sensor's reported bounds. A low bench distance can be outside the usable
  range without requiring a hand lift to finish this check.
- Flow quality, IMU and compass vectors, attitude and barometric pressure.
- FC-reported presence, enablement and health bits separately from telemetry
  receipt. A received magnetometer sample alone does not establish calibration.
- EKF flags, relative-position availability, constant-position mode and RC
  channel count. A low, stationary bench check does not establish optical-flow
  fusion or airborne position-hold quality.

**A passing bench check is not flight clearance.** Firmware identity does not
prove that the right features were compiled: the separate
[firmware artifact and post-flash acceptance procedure](../firmware/README.md)
verifies image hashes, ROMFS definitions and reboot persistence. Calibration,
mechanical readiness and the emergency-control arrangement remain separate.

Camera recording belongs to the [walkthrough procedure](SENSOR_RECORDING.md):
use `uv run --locked --group raspi drone-walk --duration 30` on the Pi after
this check exits. Passive telemetry provides no physical servo-presence or
servo-motion feedback; `drone-check` does not claim to test that actuator.
