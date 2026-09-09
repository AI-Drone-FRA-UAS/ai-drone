# Forward MT-15 integration verified

On 9 September 2026, after the user swapped two MT-15 signal wires, the sensor
was found on **physical FC T3**. Setting `SERIAL3_OPTIONS=8` makes that pin the
UART3 receive input. Enabling the second MAVLink rangefinder then made the FC
and Pi report forward distances alongside downward range and optical flow.

The final 10-second disarmed Pi recording at **12:29 CEST** contains 200 forward
readings at **1.50–1.54 m**, 200 downward readings and 200 optical-flow samples.
The camera also recorded successfully. No firmware flash was needed.

## Connection diagnosis

The laptop reached the FlywooF745 through its known USB serial identity
`200023000451333436353531`. The Pi was reached over Tailscale while connected
to eduroam; the user had intentionally not connected its USB cable.

The starting FC configuration already assigned UART3 to MAVLink1 at 115200
baud and kept that sensor channel private. A four-second zero-payload
`SERIAL_CONTROL` receive window on normal UART3 produced **zero bytes**. The
FC's own UART diagnostics also reported zero UART3 RX bytes and no framing,
noise or overrun errors. Only downward range was published on USB.

After a disarmed reboot with `SERIAL3_OPTIONS=8`, the same receive procedure
collected **4,576 bytes containing 208 valid `DISTANCE_SENSOR` packets** in
four seconds, about 52 Hz. Every packet was from system 1/component 88 with
orientation 0; distances were 152–156 cm. This matches the native stream
verified in the [September 7 direct-sensor setup](../2026-09-07/mt15-direct-usb-configuration.md).
The sensor still emits native ID 0 despite its saved ID 1 setting.

The STM32F745 USART swap makes physical T3/PB10 receive and R3/PB11 transmit.
These results establish the MT-15-to-FC receive path. The other signal wire
was not physically traced; bidirectional sensor configuration access was not
tested. No manufacturer configuration heartbeat or sensor payload was sent,
so the sensor was left in its working native MAVLink output mode.

Every passthrough lock was released. No baud sweep, other-UART takeover, or
change to the working Pi/downward sensor channel allocation was necessary.

## Saved FC changes

| Parameter | Before | After | Purpose |
| --- | ---: | ---: | --- |
| `SERIAL3_OPTIONS` | 0 | 8 | Receive the sensor signal on physical T3 |
| `RNGFND2_TYPE` | 0 | 10 | Instantiate a separate MAVLink forward rangefinder |

The existing `RNGFND2_ORIENT=0`, `RNGFND2_MIN=0.1 m`, and
`RNGFND2_MAX=15 m` were read explicitly while hidden and preserved. Native
packets advertise a 2 cm minimum; that is not an accuracy calibration and did
not justify lowering the existing conservative 0.1 m floor. FC telemetry
encodes that stored float32 minimum as 9 cm through integer truncation;
the parameter readback remains approximately 0.1 m.

The first type 10 write read back correctly in RAM but returned to 0 after an
immediate reboot. It was retried with 12 seconds of repeated disarmed
readbacks before reboot. Type10 then persisted and both FC rangefinder
instances published data normally. The cause of the first lost save is not
established; successful initial readback alone was not treated as persistence
proof. The working serial swap also persisted across these reboots.

The complete pre-check capture contains 1,172 parameters; enabling the backend
exposes 15 additional fields, giving the final **1,187-parameter snapshot**.
The [parameter file](../../params/flywoo-f745-live-2026-09-09.param) and
[capture metadata](drone-config.json) preserve the actual USB capture time and
SHA-256 `b0d603be51c08229d606b17749f1f2a59d144cde9a4c53616ef0e5aeb4dc316f`.

The complete before/after comparison contains only the two explicit changes,
FC-managed boot/runtime counters, gyro startup offsets and barometer ground
pressure. Existing downward range, optical flow, EKF/GPS source settings,
MAVLink channel allocation, and `ARMING_SKIPCHK=0` were preserved. No different
aircraft's configuration was used.

## Firmware verification

The FC reports ArduCopter **4.7.0**, Git identity **1511f271**, on FlywooF745.
Its downloaded `@ROMFS/hwdef.dat` is 35,238 bytes with SHA-256
`d89b4db7acd2811284c420fb79f0750661f8dfca6865bedf1725a17dfac4babe`, identical
to the previously reviewed installed feature definition. It includes MAVLink
rangefinders, optical flow, EKF flow fusion and GUIDED_NOGPS. This is feature
definition verification, not a hash of all executable flash.

The pinned MAVLink rangefinder backend matches incoming packets by orientation
and the FC publishes its own instance IDs. Native ID 0 from both MicoAir sensors
therefore remains compatible: the FC emits downward ID 0/orientation 25 and
forward ID 1/orientation 0. Its legacy `RANGEFINDER` message still reports only
downward distance. The firmware's proximity support remains disabled; this
integration provides range data, not automatic braking or obstacle avoidance.

## Pi verification and software

The deployed runtime's 50 files matched the local source and compiled on the
Pi before the recording. The prior runtime was archived locally for recovery.
No control or actuator service was started.

The final `/dev/serial0` capture at 115200 baud ran for **10.000625 seconds**:

| Observation | Result |
| --- | --- |
| Forward MT-15 | 200 FC-origin readings, ID 1/orientation 0, 1.50–1.54 m |
| Downward MTF-01P | 200 FC-origin readings, ID 0/orientation 25, 0.02–0.03 m |
| Optical flow | 200 samples, quality 50–73 |
| FC telemetry | 1,599 messages, all source 1/1; ten fresh disarmed heartbeats |
| EKF | All 50 reports had flags 367, consistent with relative aiding |
| Camera | 298 analyzed frames, 302 encoded frames |
| Battery | 14.201–14.209 V; cell balance and calibration were not measured |

Both range streams continued through the end, with strictly advancing FC
timestamps and maximum observed arrival gaps of about 0.13 seconds. Each was
received at about 20 Hz over the companion link, distinct from the sensor's
native 52 Hz rate. Range signal quality 0 means unknown, not a calibrated
quality score. The manifest reports successful completion with no error and
no arm, disarm, flight-mode, motor/throttle, RC override, servo, or mission-start
command. Servo motion and AprilTag detection were not established.

Two focused companion corrections accompany the configuration:

- The hover configuration gate accepts either a disabled second rangefinder or
  the reviewed type 10/orientation 0/0.1–15 m forward configuration. Its downward
  altitude filtering, firmware checks and arming requirements remain enforced.
- Forward health now expires after two seconds without a valid sample. Invalid
  MAVLink quality 1 samples cannot refresh range counts, values or freshness;
  quality 0 remains accepted as unknown. Forward and downward histories remain
  separate, and only selected FC-source samples affect component health.

## Validation and limits

The offline suite passed **573 tests**. Formatting, Ruff, typing, all five
import contracts, dependency checks and whitespace checks passed.

All **three isolated simulator cases passed in 163.58 seconds**: normal hover
with downward sensing, normal hover with both rangefinders, and GCS-loss
recovery. The additional case injects a constant 1.5 m forward reading, requires
distinct FC-origin forward/downward streams throughout Loiter, and preserves
the production 0.8 m ceiling, EKF, drift and final-disarm checks. Simulation ran
in a network namespace with only loopback and no route to the drone; source
hashes remained unchanged across the run.

This verifies sensing and software compatibility. Physical range calibration,
moving-target response, sensor command-receive wiring, and obstacle response
were not tested. The earlier compass pre-arm issue was not recalibrated or
cleared by this work. Battery voltage was below the application's 14.4 V flight
guard, so this record does not authorize a flight test.

Evidence and exact diagnostic scripts are under ignored local
`artifacts/mt15-configuration/20260909/`: baseline/final snapshots, normal and
swapped raw UART captures, parameter-write journals, UART/ROMFS downloads,
`pi-runtime-before.tar.gz`, `pi-runtime-verification.json`,
`pi-inspect.tar.gz`, `pi-telemetry-summary.json`, and `sitl/` results. The Pi's
recording remains at `/home/seb/ai-drone/artifacts/mt15-20260909-final/`.
