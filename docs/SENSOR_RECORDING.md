# Disarmed sensor inspection and recording

`drone-inspect` reports each available component and saves a bench dataset
without arming the vehicle or commanding any actuator:

```bash
cd ~/ai-drone
uv run --locked --group raspi drone-inspect --duration 30
```

From the developer machine:

```bash
SSH_CONFIG=/dev/null PI_HOST=seb@seb-is-pm.tail59e6a4.ts.net \
  uv run --locked drone-deploy --run inspect -- --duration 30
```

If Tailscale is offline and the laptop is joined to `AI-Drone-Zero`, replace
`seb-is-pm.tail59e6a4.ts.net` with the hotspot fallback `192.168.4.1`.

Camera and MAVLink are independent: either can be unavailable without failing
the inspection. The output directory contains the files produced by the live
collectors:

- `camera.h264`: hardware-encoded 1280x960 camera video
- `camera.pts`: encoder presentation timestamps
- `camera.jsonl`: frame timestamps, exposure metadata, and AprilTag detections
- `telemetry.tlog`: timestamped raw MAVLink packets
- `telemetry.jsonl`: every decoded MAVLink message with source IDs and timing
- `first-frame.jpg` and `last-frame.jpg`: quick view of camera aim/focus
- `manifest.json`: observed component states, counts, rates, and safety result

Without a camera calibration, tags are decoded but no metric pose is reported.
Supply `--calibration FILE --tag-size METRES` only after rigid mounting, focus,
and calibration at the selected resolution.

## Timed room walkthrough

Physically remove the propellers, keep the drone disarmed, and carry it by
hand. Power the flight controller, sensors and Pi throughout the recording.
Close other camera and flight-controller tools before starting. This workflow
records observations; it does not command the motors or servo.

On the laptop, connect with the [configured SSH alias](pi-networking.md#shared-teammate-ssh-access).
Then use the installed Pi environment:

```bash
ssh seb@seb-is-pm
cd ~/ai-drone
uv run --locked --group raspi drone-walk --duration 30 --dry-run
uv run --locked --group raspi drone-walk --duration 30
```

The preview starts nothing. The second command starts a detached recording for
30 seconds. For a five-minute walkthrough, use this command after the short
recording and its report have finished:

```bash
uv run --locked --group raspi drone-walk --duration 300
```

The default is 300 seconds when `--duration` is omitted. Capture starts after
initialization and camera warmup; there is no preparation countdown. The
recorder retries the Pi GPIO serial connection once if its initial heartbeat
wait times out, logging receive counters and the retry outcome. Each wait uses
`drone-inspect --timeout` (10 seconds by default). If both attempts fail, the
dataset records the FC as unavailable and can still contain camera data;
check the component statuses before relying on a walkthrough.

The command prints a unique dataset directory under `~/ai-drone/artifacts/`.
An optional `--output-dir PATH` must name a directory that does not already
exist. Keep the final `Dataset:` path from the journal if the recorder reports
a different suffix.

`uv` manages the project's `.venv`; no activation or direct `.venv/bin`
commands are needed. The deployed Pi environment uses Debian Python 3.13 with
system packages enabled for Picamera2/libcamera. `--group raspi` includes the
Pi vision dependencies, and `--locked` refuses to change the reviewed lockfile.
uv checks the environment before launch. The detached worker then reuses that
exact interpreter for recording and reporting, so it performs no second
dependency sync during capture. Do not update or sync the project while a job
is active. See uv's [locking and syncing behavior](https://docs.astral.sh/uv/concepts/projects/sync/).

Run the launcher as `seb`, without putting `sudo` before `uv run`. It uses
the Pi's existing noninteractive sudo permission to start the temporary
`ai-drone-walk.service` as that user. The job survives SSH disconnects and Wi-Fi
roaming, and is not enabled at boot. An existing walkthrough unit is never
stopped or replaced by another launch.

Monitor progress after launch or after reconnecting:

```bash
systemctl status ai-drone-walk.service
journalctl -u ai-drone-walk.service -f
```

Successful service startup is not a sensor health result. Check the journal for
camera and FC data, and inspect the final manifest's component results. Camera
or FC unavailability can still produce a partial dataset. Ctrl+C while following
the journal only closes the log viewer; recording continues. To finish early:

```bash
sudo systemctl stop ai-drone-walk.service
```

The stop signal is forwarded as SIGINT so the recorder can close its files and
generate the report. Normal completion also generates the report automatically.
Wait for the final `Report: .../review/index.html` journal line and review any
errors before copying the dataset or shutting down. Completed transient units
can disappear from `systemctl`; their journal and dataset remain. A crash or
forced termination can leave raw files without a finished report, which can be
rebuilt offline as described below.

Keep the Pi powered through recording, report generation and file transfer.
Afterward, run `sudo poweroff`, allow shutdown to finish, and then remove power.
If the battery powers the Pi, leave it connected until shutdown completes.
File syncing reduces loss but does not make an abrupt power cut safe; see
[recording durability](PI_POWER_RESILIENCE.md#recording-durability).

## Review and export a recording

`drone-report` reads an existing dataset without contacting the drone. It
preserves the raw files and writes CSV exports, `summary.json`, and an offline
browser report to `DATASET/review/`. Rebuild in the laptop's project directory:

```bash
uv run --locked drone-report artifacts/WALK_DIRECTORY
uv run --locked drone-report artifacts/WALK_DIRECTORY --no-video
```

On the Pi, include its dependency group:

```bash
uv run --locked --group raspi drone-report artifacts/WALK_DIRECTORY
```

Replace `WALK_DIRECTORY` with the recorded directory name. Adding `--no-video`
skips browser video creation while retaining charts, exports and camera
previews. By default, available `ffmpeg` copies the H.264 video into
`review/camera.mp4` without re-encoding. Missing `ffmpeg` or invalid timestamps
leave the original video available and produce an explanatory report note.
You can copy the recording to a laptop with `ffmpeg` and rebuild there.

From the laptop, copy the **whole dataset directory**, including the raw files
that the report links to:

```bash
mkdir -p artifacts/walkthroughs
scp -r seb@seb-is-pm:/home/seb/ai-drone/artifacts/WALK_DIRECTORY artifacts/walkthroughs/
```

Open `artifacts/walkthroughs/WALK_DIRECTORY/review/index.html` in a browser.
The report works offline. Its charts share an elapsed-time cursor; the camera
video has independent playback controls. Use `--output-dir PATH` to select a
different report directory, or `--system ID --component ID` to select a
different FC source. The project FC defaults are system 1, component 1; other
sources remain in the raw logs and are excluded from the derived FC charts.

CSV units and unavailable-value handling follow the
[MAVLink message definitions](https://mavlink.io/en/messages/common.html).
`RAW_IMU` fields remain labelled as raw values; the report does not infer
calibration scales or compass accuracy from their presence.

| Export | Contents |
| --- | --- |
| `ranges.csv` | Forward and downward distances, sensor ID/orientation, configured limits, signal quality, `valid` and `invalid_reason` |
| `optical_flow.csv` | Separate MAVLink flow formats, quality and validity, compensated velocity, angular rates or integrated flow where provided |
| `imu.csv` | Acceleration, angular velocity, magnetic measurements and available raw fields/temperature |
| `motion.csv` | Attitude and EKF local NED position/velocity |
| `environment.csv` | Pressure, temperature and FC system battery readings |
| `battery.csv` | Available per-battery readings, consumption and status |
| `events.csv` | Armed state, mode number, EKF flags, sensor health bits and status text |
| `camera.csv` | Analyzed-frame timing, exposure metadata and detected tag IDs |

Only messages and fields actually received can be exported. Unsupported or
unavailable channels are not filled with zero. The raw telemetry retains all
decoded messages for later analysis; the charts use a bounded sample envelope
while the CSVs retain the selected recording's rows.

Range validity follows the FC-reported limits. The project's downward range is
currently configured for a maximum of **1 metre**: readings above it during
hand carrying are retained with `outside_configured_range` and excluded from
healthy distance values. This does not change the FC parameters or clamp the
readings. Range signal quality 1 is invalid; 0 means unknown. Optical-flow
quality 0 is invalid. `OPTICAL_FLOW` and `OPTICAL_FLOW_RAD` have different fields
and units, so the exports keep their message types separate. EKF local position
is an estimate, not a ground-truth room map or measured walking route.

`elapsed_s` uses the capture's monotonic start time. Telemetry timestamps mark
receipt at the Pi; analyzed camera frames mark request retrieval, with the
camera's sensor timestamp retained separately. These are not precisely aligned
exposure and sensor measurements. The MP4 uses an approximate constant frame
rate from the median interval in `camera.pts`; it does not reproduce every
variable frame interval. Keep `camera.h264`, `camera.pts` and `camera.jsonl` for
more detailed timing work, and do not treat the report's video and chart cursor
as synchronized measurements.

## Documented connections

The wiring and UART allocation rules live in
[flight-controller configuration](DRONE_CONFIGURATION.md). Confirm the current
physical build and component health with a fresh inspection; this table is not
a live connection result.

| Device/data | Physical connection | What the Pi records |
| --- | --- | --- |
| IMX500 AI Camera | Directly to Pi CSI connector | H.264 video, frame metadata, and tag detections |
| MicoAir MTF-01P | Flight controller UART5 (`SERIAL5`, MAVLink1, 115200) | FC-published range and optical flow over the companion link |
| MicoAir MT-15 | FC physical T3 with `SERIAL3_OPTIONS=8`, MAVLink1, 115200 | FC-published forward rangefinder instance 2 over the companion link |
| Flight-controller IMU, barometer, compass, GPS, battery, EKF and RC state | Directly to the FlywooF745 | Requested MAVLink telemetry over UART4 |
| Raspberry Pi companion link | Pi GPIO14/15 `/dev/serial0` to FC R4/T4 (`SERIAL4`, MAVLink2, 115200) | All FC telemetry in `.tlog` and `.jsonl` |
| Servo | Separate guarded utility targets Pi BCM12 directly | Not driven by this program; FC output telemetry may still be recorded |

The inspector requests bounded message rates that fit the verified 115200-baud
UART4 link. This activates telemetry streaming, not motors. Sensors that require
the flight battery, including the MTF-01P, must already be powered.

## Forward MicoAir MT-15

The MT-15 is the forward sensor. Its FC receive path was restored on September
9 by enabling UART3 RX/TX swapping after a physical data-wire change, then
enabling the second MAVLink rangefinder. See the
[integration verification](../state/2026-09-09/mt15-integration.md) and the
earlier [direct sensor configuration](../state/2026-09-07/mt15-direct-usb-configuration.md).

The inspector distinguishes forward `DISTANCE_SENSOR.orientation=0` from
downward orientation25. Component health and counts require messages from the
selected flight controller; raw telemetry retains other sources for diagnosis.
Without a confirmed FC-published forward stream, the component is `no_data` and
flight logic must not depend on it. Sensor ID alone cannot distinguish these
two sensors: the tested MT-15 firmware emits MAVLink ID0 despite saving ID1.
The FC republishes it as instance ID1. Both forward and downward health expire
after two seconds without a valid sample. MAVLink signal quality1 means an
invalid reading and cannot refresh health; quality0 is accepted as unknown.

Use the [canonical wiring and configuration guide](DRONE_CONFIGURATION.md#sensors)
before changing parameters. It explains RX3 versus the old analog VTX cable,
the five-channel MAVLink limit, and the need to preserve the downward sensor
and Pi connections. UART allocation and `MAVn_OPTIONS` mapping must be reviewed
together; a generic serial-port recipe is insufficient.

For use as an avoidance source, the firmware must expose and correctly
configure the applicable proximity parameters. Confirm firmware support before
relying on automatic braking. A single forward beam is a stop-ahead sensor, not
safe 360-degree hall navigation.

## Safety behavior

The inspector:

- refuses to start if the vehicle heartbeat is armed;
- stops if the vehicle heartbeat changes to armed;
- contains no arm, disarm, mode-change, motor, throttle, RC override, mission,
  or servo command path;
- records `SERVO_OUTPUT_RAW` only as telemetry;
- does not enable disarmed onboard DataFlash logging or alter FC parameters.
