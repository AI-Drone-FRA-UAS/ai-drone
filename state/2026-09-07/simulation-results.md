# Pinned ArduCopter SITL validation — 2026-09-07

**Result: both existing integration tests passed in 108.57 seconds.**

This was software simulation only. The physical flight controller, Raspberry Pi, sensors, motors, and payload servo were not contacted or actuated by this run.

## Isolation and provenance

- Reviewed the existing `tests/test_sitl.py` harness before execution. Its clients explicitly connect to `127.0.0.1:5760` and `127.0.0.1:5762`. It launches a temporary simulator process and generates external MAVLink range/flow packets from simulator ground truth.
- Because the simulator's default TCP listeners bind all addresses, the entire pytest process tree was run in a separate user/network namespace with only loopback enabled. Verified interface output: `lo UNKNOWN 127.0.0.1/8 ::1/128`. No physical network interface existed inside that namespace.
- Executed the existing binary `/home/abaris/drone/ardupilot/build/sitl/bin/arducopter`; checked local repository HEAD `1511f27194f1dcc3728270883047bdf022b3fd53`, matching the test's required firmware commit.
- No firmware source or production code was edited. Simulator EEPROM, DataFlash, terrain files, telemetry, inspection recordings, and pytest output are under `/tmp/drone-sitl-20260907-b9t_nun4`.
- Full terminal output: `/tmp/drone-sitl-20260907-b9t_nun4/pytest-output.txt`.
- Flight code SHA-256 recorded by the real production recorder: `26558d4e938122fdb90bba093a4a0459bf732dcd8ffd187cb8b5584fbaf8bb54`.

## Results

| Existing test | Result | Mode sequence | Maximum altitude | Minimum Loiter altitude | Maximum XY drift during Loiter |
| --- | --- | --- | --- | --- | --- |
| `test_production_hover_no_gps_loiter_in_pinned_sitl` | PASS | STABILIZE → GUIDED_NOGPS → LOITER → LAND → disarmed | 0.520 m | 0.520 m | 0.029 m |
| `test_gcs_heartbeat_loss_in_loiter_lands_and_disarms` | PASS | STABILIZE → GUIDED_NOGPS → LOITER → LAND → disarmed | 0.520 m | 0.520 m | 0.029 m |

Both scenarios observed RC channel counts `[0]` and relative-position EKF navigation with no GPS/absolute-position aid. The first scenario injected 962 sensor samples using MAVLink 2.0. It ran the actual `drone-control hover` implementation, including guarded takeoff, Loiter, landing, and flight recording, rather than reproducing those commands in a separate test controller. Its flight manifest reports `completed=true`, runtime 26.05913 s, and simulated DataFlash log 1 (1,638,400 bytes).

The first scenario also ran the actual read-only inspection path for 5 seconds: 941 simulated telemetry messages, downward range around 0.02 m while grounded, flow quality 60, zero camera frames, zero detected tags, and zero servo pulses. Camera unavailability is expected on this host and is asserted by the test.

The second scenario deliberately SIGKILLed the production companion process while in Loiter, bypassing Python cleanup. ArduCopter subsequently reported `GCS Failsafe`, entered LAND, and disarmed. Final simulated touchdown status reported vertical speed 0.147404 m/s. This exercises the configured firmware response to loss of system-255 GCS heartbeats, independently of application exception handling.

The output also includes `No ap_message for mavlink id (106)`: the broad recorder requests OPTICAL_FLOW_RAD, which this build does not schedule. The tests consume the supported OPTICAL_FLOW stream and still pass. This is a telemetry request mismatch, not a demonstrated loss of optical-flow sensing.

## Limits

- These results validate the deployable flight path and GCS-loss recovery under the simulator's supplied conditions. They do not prove the real airframe can fly or that any physical sensor or actuator works.
- The test supplies idealized external range/flow at 20 Hz with fixed flow quality 60, simulated IMU/compass/barometer, no receiver, and no GPS. It does not test actual sensor alignment, textured-floor/light dependence, vibration, magnetic interference, payload dynamics, damaged hardware, or camera inference.
- The existing SITL CLI arguments set `--min-battery 0`, so these two passing integrations do **not** validate the default 14.4 V battery gate or its invalid-value handling.
- There is no servo operation in either scenario. No propeller or physical actuator result can be inferred from simulated takeoff.
- The asynchronous arm-cleanup, cached disarm-confirmation, unknown-voltage, and public hold-mode-loss findings in `/tmp/drone-flight-review.md` remain independently reproducible; these two acceptance tests do not cover them.

## Reproduction

The temporary runner `/tmp/run-drone-sitl-isolated.py` enables loopback, sets `ARDUPILOT_ROOT`, allocates a unique temporary directory, and invokes the unchanged test file using the repository Python environment. It was run with:

```text
unshare --user --map-root-user --net /home/abaris/drone/ai-drone/.venv/bin/python /tmp/run-drone-sitl-isolated.py
```

Inside that namespace it invokes the equivalent of:

```text
ARDUPILOT_ROOT=/home/abaris/drone/ardupilot /home/abaris/drone/ai-drone/.venv/bin/python -m pytest -vv -s --basetemp /tmp/<unique-run>/pytest /home/abaris/drone/ai-drone/tests/test_sitl.py
```
