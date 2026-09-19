# Operator guide

Use the [networking guide](pi-networking.md) to reach the Pi. Run Pi commands
from `~/ai-drone` with `uv run --locked --group raspi`; on a laptop omit
`--group raspi`. These commands require the refactored checkout; see the
[saved deployment status](pi-networking.md#deployment-status). Once installed,
the [access service](pi-networking.md#shared-access-service) owns the FC UART;
recording, checks and control use its shared telemetry.
Only one workflow may own the camera or payload GPIO.
Check the latest [configuration](DRONE_CONFIGURATION.md) and dated `state/`
capture before hardware work.

## Local settings

Copy [drone.example.toml](../drone.example.toml) to `drone.toml` on each machine.
It holds connection, recording, transfer, operator and runtime defaults; Git ignores it and
deployment neither uploads nor deletes it. Select another file with
`uv run drone --config PATH COMMAND`, or `AI_DRONE_CONFIG=PATH` for any helper.
File selection is `--config` → `AI_DRONE_CONFIG` → current-directory
`drone.toml` → built-in defaults. Explicit command options override file values.
An explicitly selected file must exist. Only the standard `drone.toml` name
receives automatic Git/deployment protection.

Existing `PI_HOST`, `PI_HOSTNAME`, `PI_USER`, `PI_DIR` and `SSH_CONFIG`
connection overrides remain supported above TOML defaults; `PI_USER` supplies
the user for a host without `user@`. Wi-Fi credentials stay in NetworkManager
and SSH authentication stays in normal SSH configuration.

## Operator heartbeat

Autonomous control requires a running authenticated heartbeat on the operator
laptop. It runs independently of SSH terminals:

```bash
uv run drone operator keygen
uv run drone operator start
uv run drone operator status
# Stop after autonomous work has finished:
uv run drone operator stop
```

The default key is `~/.config/ai-drone/operator.key`. Copy the same key to the
Pi through existing authenticated SSH, keep both files private (0600 on POSIX),
and never commit it. Reuse an existing shared key rather than replacing one side.
On the Pi, set `[operator]` in `drone.toml`:

```toml
[operator]
endpoints = ["http://100.101.102.103:8787", "http://192.168.1.20:8787"]
token_file = "/home/seb/.config/ai-drone/operator.key"
interval = 1
timeout = 5
request_timeout = 1
```

Replace the example addresses with the laptop's numeric Tailscale and/or LAN
IPs; allow incoming TCP 8787 on those routes. One working endpoint suffices,
so direct LAN access remains useful when Tailscale is unavailable. Up to four
HTTP(S) origins are supported; DNS names, credentials, paths and redirects
are rejected. The supplied listener serves HTTP and accepts IPv4 `--bind`.
Each response authenticates a new challenge; the key is never sent over HTTP.

If incoming laptop connections are blocked, the responder can supervise its own
SSH reverse tunnel through the existing Pi login:

```bash
uv run drone operator stop
uv run drone operator start --ssh-host seb@seb-is-pm --remote-port 18787
```

Add `http://127.0.0.1:18787` to the Pi's `operator.endpoints`, retaining direct
routes where available. On the laptop, optional `[operator]` values `ssh_host`
and `remote_port` supply these command defaults. The tunnel reconnects after
failure, survives terminal closure, and stops with the responder. It requires
existing noninteractive SSH authentication. Local `operator status` checks the
responder; confirm the complete route with the Pi's `runtime status`
(`operator_alive`). Tunnel diagnostics are in the private key folder's
`operator.log`. Restart the responder to change its forwarding options.
The staged September 16 check verified this reverse route; recheck it when the
Pi returns. The responder is not started by source deployment or service setup.

The Pi samples once per second and expires operator presence after five seconds
by default. Closing SSH has no effect. Actual Wi-Fi/operator loss requests
LAND during autonomous flight; confirmed human RC ownership keeps pilot control.
Configuration changes require restarting the access service while idle and
disarmed. Check `uv run drone runtime status` before control.

## Disarmed checks

```bash
uv run --locked --group raspi drone check --prearm
uv run --locked --group raspi drone check --json > check.json
```

The check verifies the selected ArduPilot quadrotor at MAVLink system/component
1/1, firmware, parameters, battery and sensor streams. It requests diagnostics
without writing parameters, arming or commanding actuators. Exit 1 means a
failed or unavailable requirement; acknowledgement alone is not a passed
pre-arm check. A passing bench result does not establish airborne flow fusion,
compass calibration or emergency control.

For direct FC USB inspection on Linux, select its verified
`/dev/serial/by-id/usb-ArduPilot_FlywooF745_...-if00` path (`1209:5741`,
115200 baud) with `--device PATH`. Close other serial clients first. USB may
power the FC without powering the battery-fed sensors. Use the serial-access
group appropriate to the host (`uucp` or `dialout`); never make the port
world-writable. Serial-owner inspection currently requires Linux.

Capture a restorable parameter snapshot from the laptop:

```bash
uv run drone config
```

It writes a verified `.param` file in `params/` and matching JSON in `state/`.
It does not deploy, commit, push or write FC parameters.

## Recording

```bash
uv run --locked --group raspi drone record --duration 30
uv run --locked --group raspi drone record --duration 30 --no-video
```

Ordinary recording is passive and disarmed: it refuses an armed heartbeat and
stops if the FC arms. Camera and FC are independent; missing sources produce
an incomplete result while available sources continue. `--no-video` disables
saved H.264 while retaining camera analysis, AprilTags and JSONL logs.
`--stream` enables optional browser preview. Read `drone record --help` for
calibration, detector and preview options.

Add `--allow-flight` to record during manual or autonomous flight, including
attachment after arming. It replaces the old manual-flight confirmation string.
Passive recording never sends arm, takeoff, mode or attitude commands. It can
run beside `drone control` through the shared FC service; only the recorder
owns the camera. Servo activation requires its own explicit workflow below.

| Output | Contents |
| --- | --- |
| `camera.h264`, `camera.pts` | Optional local video and encoder timestamps |
| `camera.jsonl` | Analyzed-frame timing, exposure metadata, tag detections and available poses |
| `telemetry.tlog`, `telemetry.jsonl` | Native and decoded MAVLink traffic, including source IDs |
| `storage.jsonl` | Space samples, warnings and storage-triggered stops |
| `first-frame.jpg`, `last-frame.jpg` | Camera aim/focus previews when available |
| `manifest.json` | Settings, component status, counts, errors and stop reason |

Datasets are created under `artifacts/` unless `--output-dir PATH` is selected.
Logs flush per record and sync periodically (`--sync-interval`, default 5 s)
and on close. Atomic manifests reduce partial writes. Keep power connected
until recording and file finalization finish.

RPM and four-motor ESC telemetry are requested at 2 Hz. Values are saved only
when the FC supplies them; motor throttle/output values are never converted
into invented RPM.

### Storage limits

Defaults warn below **1024 MiB**, reserve **256 MiB** for logs after video
stops, and stop all capture at **16 MiB**, checking every **2 seconds**.
Override them with `--storage-warning-mib`, `--storage-reserve-mib`,
`--storage-stop-mib` and `--storage-check-interval`; require
`0 < stop < reserve <= warning`.

Video stops and syncs before the log reserve, allowing for encoder bitrate and
delayed checks. Camera analysis, tags, telemetry and preview continue. The
printed remaining-video estimate uses configured bitrate, not a measured
guarantee; other disk writers can consume space between checks. At the final
log floor, capture closes its files and writes a partial manifest with
`completed=false`, `stop_reason=storage_full`. Storage-check/write failures are
reported as failures. Preserve these partial datasets for diagnosis.

### Continue after SSH disconnect

For a disarmed, propellers-off hand-carried recording:

```bash
uv run --locked --group raspi drone walk --duration 30 --dry-run
uv run --locked --group raspi drone walk --duration 300
systemctl status ai-drone-walk.service
journalctl -u ai-drone-walk.service -f
```

Use the same recording options, including `--no-video`, `--calibration`,
`--tag-size`, storage limits and preview. Effective arguments are fixed when
the job launches; later configuration-file edits do not change that session.
The launcher runs a detached, non-boot-enabled systemd job as `seb`; it requires
the Pi's existing noninteractive sudo permission. Recording continues when
SSH closes. A second launch refuses an existing job. Do not sync dependencies
or deploy while it is active. Duration starts after initialization/warmup.

To finish early, run `sudo systemctl stop ai-drone-walk.service`. SIGINT cleanup
finalizes the dataset and generates a report. Wait for the journal's final
`Report:` line, inspect errors and component status, then copy files. Closing
the journal viewer does not stop recording.

For manual or autonomous flight, use `drone walk --allow-flight --duration 300`.
Wait for **READY** when starting before takeoff; attachment while already armed
is also supported. `--tag-servo` selects the separate native-detector/servo gates
below. Shared telemetry allows control and recording together without opening
the UART twice.

### AprilTag geometry

Use native AprilTag 3 on the Pi; OpenCV is a diagnostic detector fallback and
provides pose estimation. Servo qualification requires native quality fields.
Tags lie flat on the floor, so rigidly mount the camera downward and focus it
at the intended 1–3 m distance. Measure the printed black square, not the page.
Start with `tag36h11` A3 prints around 0.224 m; check real detection quality
before increasing processing resolution.

Use `--calibration FILE --tag-size METRES` for metric poses. Calibration must
match focus, crop and resolution; without it only IDs/corners are reported.
Measure the camera-to-body transform before using poses for control.

## Reports and transfer

Fetch a finalized dataset from the laptop into a new directory:

```bash
uv run python scripts/transfer.py fetch /home/seb/ai-drone/artifacts/DATASET \
  --destination artifacts/DATASET
```

`fetch` reads from the configured Pi; `push DATASET --host USER@HOST
--destination /absolute/new/DATASET` sends a local dataset to another machine.
`transfer.destination` can supply the destination. SSH peers need a POSIX shell,
`uv` and this project at the configured `connection.project_dir` (`PI_DIR` can
override it). Existing destinations are never overwritten. Complete and
finalized partial recordings are supported.

Copies are checked by size and SHA-256; originals remain by default. To remove
the source, explicitly run `uv run python scripts/transfer.py remove-source
RECEIPT`, using the printed receipt path. Both copies must still match; source
deletion also checks open writers and currently requires Linux on the source.
Complete process inspection needs existing `sudo` permission on the source;
only its fixed read-only scanner is elevated. Copying and deletion run as the
normal user. An unreadable process, timeout or denied scanner blocks removal.
Copying a finalized partial source also requires this Linux process check.
Perform any desired removal before adding reports or changing either copy.
Then run `uv run drone report artifacts/DATASET` on the laptop.

Open `artifacts/DATASET/review/index.html`. Reports preserve raw files and add
CSV exports, summary JSON and offline charts. Available `ffmpeg` remuxes H.264
to MP4; `drone report DATASET --no-video` skips that step. Missing video does
not prevent telemetry export. Recording itself never depends on transfer.

Exports retain received values and source identity; unavailable channels stay
unavailable. Downward/forward ranges remain distinct. Report validity uses
FC-reported limits, not guessed operating ranges. Telemetry timestamps record
receipt; camera timestamps record retrieval with sensor timestamps separately
retained. Video playback and chart cursors are not measurement-synchronized.

## Servo and tag-triggered release

The servo uses **Pi BCM12 / physical pin 32**, common ground and a regulated
supply sized for starting/stall current. Historical 900–2100 µs values are
bounds, not calibration of the current linkage. Remove propellers and payload,
secure the frame, clear the linkage and establish non-binding endpoints first:

```bash
uv run --locked --group raspi python scripts/servo.py --help
uv run --locked --group raspi python scripts/mount.py --help
```

The dedicated mount capture opens once on three qualifying observations of
`--tag-id ID` (default 3), records `servo.jsonl`, and never automatically closes.
It supports disarmed capture and flight recording without taking flight control:

```bash
uv run --locked --group raspi python scripts/mount.py close
uv run --locked --group raspi python scripts/tag_mount_capture.py --tag-id 3 --duration 60
```

Manual mount commands and tag capture share the GPIO setup in `ai_drone.mount`:
BCM12, 900–2100 µs mapping, PWM off at startup, and the existing gpiozero pin
factory. Open is 1500 µs and close is 900 µs, held for 0.5 s by default. Manual
commands then close GPIO; tag-mount capture detaches PWM after opening and
ignores later sightings. The old installed commands `uv run drone_mount open`
and `uv run drone_mount close` become `uv run python scripts/mount.py open`
and `uv run python scripts/mount.py close` in the refactored checkout.

The guarded pulse workflow accepts repeated `--tag-id ID`, inclusive
`--tag-range START:END` (repeat for disjoint ranges), or `--all-tags`, plus
measured active/rest positions and confirmation gates:

```bash
uv run --locked --group raspi drone tag-servo-record --help
```

Select `--active-us`, `--rest-us`, `--pulse-duration`,
`--confirm-actuation SERVO_CLEAR` and
`--confirm-armed-flight ARMED_FLIGHT_TAG_SERVO_CLEAR` only after physical
calibration. Each selected ID can trigger once per run. Defaults require zero
Hamming corrections, margin ≥30 and three consecutive fresh detections; frames
older than 0.5 s are rejected. GPIO logs record commanded movement, not observed
position. Use `drone walk --tag-servo` with the same arguments for detached
capture; no servo activation occurs unless explicitly selected. The foreground
workflow stops on SSH hangup, camera/heartbeat failure, or disarm after arming;
the mount-capture workflow has its own
arm-transition policy. Neither performs autonomous approach or flight control.

## Motor bench test

Remove **every propeller**, secure the frame, clear all motor bells, and keep
a second person ready to disconnect the battery. Confirm fresh disarmed FC
state, exactly `ARMING_SKIPCHK=0`, resolved pre-arm faults and motor mapping.

```bash
uv run --locked --group raspi python scripts/motor_test.py \
  --motor 1 --duration 0.5 --throttle-percent 7 \
  --confirm-props-removed PROPS_REMOVED \
  --confirm-vehicle-secured VEHICLE_SECURED
```

`--all-motors` runs sequentially. The helper caps duration at 1 s and throttle
at 10%, counts down, then uses `MAV_CMD_DO_MOTOR_TEST`. ArduPilot temporarily
soft-arms those outputs; cleanup requests stop and waits for disarm. A motor
that fails to spin needs wiring/power/status diagnosis before raising throttle.

## Takeoff, Loiter and landing

`uv run drone control hover --help` describes the existing guarded
`GUIDED_NOGPS` climb → flow-backed `LOITER` → `LAND` sequence. Default target
altitude is 0.5 m with a 0.8 m software ceiling. The controller requires fresh
selected-vehicle data, exact firmware/configuration, `ARMING_SKIPCHK=0`, and
bounded state transitions. Cleanup lands only a flight that instance started.

On the Pi, control runs in the detached `ai-drone-control.service` by default;
`--foreground` keeps it in the invoking process. Inspect it with
`journalctl -u ai-drone-control.service -f`. SSH closure does not stop it.
`sudo systemctl stop ai-drone-control.service` requests guarded landing and
waits for cleanup; it is not a substitute for independently validated emergency
control.

The existing hover preflight still requires fresh `RC_CHANNELS.chancount=0`.
An active receiver blocks autonomous startup. Unexpected receiver changes or
stale RC telemetry request LAND until a human handoff has been confirmed.

For a deliberate airborne handoff, run `uv run drone control handoff` and have
the pilot select `STABILIZE`, `ALT_HOLD`, `LOITER` or `POSHOLD` on the radio. Transfer occurs
only with the explicit request, a fresh armed FC heartbeat and fresh nonzero
RC channels. Receiver presence alone is insufficient. Once confirmed, that run
cannot retake autonomous control or command LAND for operator-link loss. The
onboard GCS heartbeat remains active until the pilot disarms; stopping a service
or losing a recording must not drop that heartbeat.

Each control run also writes telemetry and events under `artifacts/flights/`
using a separate subscription. Failure to start its log blocks startup. Later
disk, queue or storage-floor failures finalize an incomplete recording and leave
flight control running; `recording_error` preserves the cause when writable.

Before any propeller-on test:

1. Pass the [pinned firmware and SITL gates](ARDUCOPTER_4_7_NOGPS_LOITER.md#exact-pinned-sitl-acceptance-gate), including rejected arm, stale telemetry and link-loss landing.
2. Rigidly mount/calibrate IMU, compass, range and flow; resolve every pre-arm
   fault and validate the intended indoor boundary and recovery behavior.
3. Verify motor numbering/direction with propellers removed.
4. Establish an independently validated emergency-control arrangement. SITL's
   GCS-loss LAND result is not live emergency-control validation.
5. Use an authorized enclosed low hover with an observer and suitable battery;
   review range, flow, EKF and battery logs before expanding the envelope.

Tag centering, autonomous search and forward-lidar avoidance are not current
control modes. An unresolved magnetic-disturbance/pre-arm fault must be fixed,
not bypassed.

## Finish and power down

From the laptop, inspect `uv run python scripts/power.py status`. Then choose
`prepare battery`, `prepare fc-usb`, `prepare pi-usb` or `prepare all`.
Use `--dry-run` for a preview. Preparation finalizes only the known walkthrough
job and refuses unknown owners, active package work or missing disarmed state.
By default it requests Pi shutdown because the remaining supply is unknown.

Keep power connected until the Pi has physically halted; lost SSH only proves
loss of access. `--pi-power-independent` may leave it running only after the
operator verifies a separate supply will remain; it is invalid with `all`.
See [Pi maintenance](pi-networking.md#pi-maintenance) for upgrade/resilience tools.
