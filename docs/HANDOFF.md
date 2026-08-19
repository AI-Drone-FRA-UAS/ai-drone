# AI drone project handoff

Last verified: **2026-08-19, Europe/Berlin**

This is the starting document for a later development session. Read it before
connecting to the live vehicle or changing flight-controller parameters.

## Current objective

Build a safe indoor drone that can:

1. hold altitude using the flight controller, downward rangefinder, and optical
   flow;
2. detect floor-mounted AprilTags using the Raspberry Pi AI Camera;
3. estimate tag-relative position and distance from calibrated geometry;
4. approach and center over a selected tag;
5. release a payload through a servo; and
6. eventually search a hall autonomously while avoiding obstacles.

Only disarmed sensing, recording, configuration export, and software tests have
been validated. No autonomous or armed flight has been validated.

## Repository and GitHub state

- Repository: `AI-Drone-FRA-UAS/ai-drone`
- Working branch: `experimental`
- Functional-parity baseline for the cleanup:
  `2c3fab2e637bac87ec131e83e82b12bb8131b1d5`. Capabilities present there or in
  later work must survive even when duplicate wrappers or entry points are
  consolidated.
- The later safe-hardware implementation immediately before the original
  handoff was `9c8774f`; use `git log -1` for the current tree.
- Pull request: <https://github.com/AI-Drone-FRA-UAS/ai-drone/pull/7>
- PR target: `main`
- PR state at handoff: open, draft, and mergeable
- `experimental` already contains all commits from `main`, including the
  eduroam documentation and Raspberry Pi dependency work. The cleanup makes
  `pyproject.toml` plus `uv.lock` the single dependency source and removes the
  redundant generated `requirements-raspi.txt` export.
- Do not merge PR #7 into `main` unless the user explicitly approves the
  release. The PR was deliberately left as a draft because it includes
  hardware-control tooling.

The current working tree also contains a repository cleanup performed on
2026-08-18. It consolidates automatic flight behind one guarded
`drone-control` entry point, removes duplicate flight and root wrappers,
centralizes shared parsing/platform/MAVLink helpers, and hardens deployment and
actuator guards. The takeoff/hover, velocity, and person-follow sequences retain
their arming, landing, and cleanup capabilities even though they have not yet
passed live flight validation. Check `git status` and review this complete
change set before committing or updating the draft PR.

GitHub CLI authentication is available outside the restricted sandbox as user
`Jannik99F`. If sandboxed `gh` or SSH reports a misleading authentication or
network failure, follow the elevated retry instructions in `AGENTS.md`.

## Live access

Prefer the Pi's Tailscale MagicDNS name whenever Tailscale is online:

```bash
ssh -F /dev/null seb@seb-is-pm
```

If the Pi has fallen back to its own `AI-Drone-Zero` access point, use:

```bash
ssh -F /dev/null seb@192.168.4.1
```

Useful environment for host-side commands:

```bash
export SSH_CONFIG=/dev/null
export PI_HOST=seb@seb-is-pm
```

Do not place passwords, eduroam credentials, hotspot credentials, radio binding
phrases, or GitHub tokens in commands, logs, state files, or commits.

Literal hotspot, SSH, and radio-binding credentials were removed from the
maintained files during the cleanup. Because old values remain recoverable from
Git history and may still be active, rotate the hotspot and SSH credentials and
rebind the radio system with a new private phrase before treating those paths
as secure. The hotspot setup script accepts a hidden prompt or a root-owned,
root-only password file and never accepts the passphrase as its own command-line
argument. The NetworkManager path still passes it to the child `nmcli` process
as an argument while applying the profile, so use the script only on a trusted
Pi.

## Non-negotiable hardware safety state

- Treat the connected drone as disarmed/read-only unless the user explicitly
  authorizes an actuator test and confirms the physical safety prerequisites.
- The live controller has `ARMING_CHECK=0`. This disables important optional
  pre-arm checks. Do not perform flight or motor testing in this state.
- The live controller has `FENCE_ENABLE=0`. The configured 100 m altitude and
  300 m radius are not useful indoor containment, and no polygon is uploaded.
- Do not blindly enable the fence indoors; configure a meaningful boundary,
  localization source, and breach action first.
- The camera is held only by a small cable and currently faces forward. It is
  not a rigid, strain-relieved nadir mount. Do not fly or trust floor-tag
  geometry in this state.
- The payload servo is not connected to the Pi/vehicle output. Do not run the
  servo utility until its power, signal path, mechanical limits, and fail-safe
  position have been bench-verified with the propellers removed.
- The forward MT-15 rangefinder is not connected or tested, and usable physical
  mounting/wiring space on the aircraft has not been confirmed.
- No propeller-removal confirmation was received during the work recorded here.
- No motor-test, arm, throttle, flight-mode, mission, parameter-write, or servo
  command was sent during the live sessions.

The guarded motor utility intentionally refuses to run while
`ARMING_CHECK=0`. Its presence in the repository does not mean the vehicle is
ready for a motor test.

## Current test envelope

Keep the vehicle stationary and disarmed for every currently allowed hardware
check. Power only what a specific read-only check needs.

| Activity | Current status | Boundary |
| --- | --- | --- |
| Offline unit, lint, type, and SITL work | Allowed | No real hardware side effects |
| MAVLink health and configuration export | Allowed | Read-only; verify the heartbeat remains disarmed |
| Downward MTF-01P range/flow recording | Allowed | Existing sensor only; transport data does not validate altitude hold |
| Camera stream/person detection | Limited | Stationary only; physically support the loose camera and ribbon |
| AprilTag pixel detection/throughput | Limited | A visible tag may be tested; no floor geometry or metric pose claims with the current forward, uncalibrated mount |
| Integrated disarmed recording | Limited | Support the camera; stop on any armed heartbeat |
| Servo movement | Blocked | Servo is disconnected; later requires props off, secured frame, safe power/common ground, clear linkage, limits, and explicit confirmation |
| Forward MT-15 telemetry | Blocked | Sensor is disconnected, untested, and has no confirmed installation space or wiring |
| Motor test | Blocked | `ARMING_CHECK=0`; props-off and all guarded prerequisites are also required |
| Arming, takeoff, hover, velocity control, person follow, navigation, tag approach, payload drop | Implemented, currently blocked on hardware | The guarded control paths remain in the toolbox, but no real-flight path is validated and camera/localization/obstacle prerequisites are absent |

Restoring `ARMING_CHECK=1` is a configuration change, not an actuator test. It
can be done in a separately authorized, propellers-off configuration session to
observe and resolve every `PreArm:` failure. It is necessary but not sufficient
for a motor test or flight.

## Verified hardware topology

### Raspberry Pi and camera

- Raspberry Pi Zero 2 W, Debian 13 ARM64, approximately 415 MiB usable RAM.
- IMX500 Raspberry Pi AI Camera connected directly to the Pi CSI connector.
- Camera modes observed: 2028x1520 at 30 fps and 4056x3040 at 10 fps.
- Pi GPIO UART `/dev/serial0` (`/dev/ttyAMA0`) is the companion link.
- Pi GPIO14/TX and GPIO15/RX connect to flight-controller UART4 at 115200.

### Flight controller and sensors

- FlywooF745 / Flywoo GOKU GN745 AIO.
- ArduPilot Copter 4.6.3, custom commit `92b0cd78`.
- Pi companion link: FC UART4, MAVLink2, 115200 baud.
- MicoAir MTF-01P downward optical flow and rangefinder: FC UART5, MAVLink1,
  115200 baud, `SERIAL5_OPTIONS=1024`.
- Flight-controller IMU, barometer, compass, GPS, battery, RC input, and EKF
  state are forwarded to the Pi through MAVLink.
- The planned payload servo is currently disconnected. The repository's servo
  utility drives Raspberry Pi BCM GPIO 12 directly; it does not use MAVLink or
  a flight-controller servo output. It now requires an explicit mode plus the
  exact `--confirm-actuation SERVO_CLEAR` acknowledgement before GPIO setup.

### Planned forward rangefinder

The MicoAir MT-15 is not connected, has never been functionally tested here,
and does not yet have confirmed physical space on the aircraft. The proposed
electrical connection is the unused full-duplex **flight-controller UART7**,
not a Pi UART or Pi GPIO header:

```text
MT-15 5V  -> regulated FC 5V
MT-15 GND -> FC GND
MT-15 TX  -> FC R7
MT-15 RX  -> FC T7
```

Do not apply the proposed `SERIAL7`/`RNGFND2` parameters until the exact board
revision, wiring, power, and outgoing `DISTANCE_SENSOR.orientation` have been
verified. A single forward MT-15 beam is stop-ahead sensing, not 360-degree
hall mapping or obstacle avoidance. See [sensor recording and wiring](SENSOR_RECORDING.md).

## Pi corrections already made

- Corrected the date from 2026-08-11 to 2026-08-18.
- Corrected timezone from `Europe/London` to `Europe/Berlin`.
- Left NTP enabled and persisted a corrected offline clock epoch.
- NTP is not synchronized while the Pi hotspot has no upstream route.
- Cleared the stale `NetworkManager-wait-online` failed-service state.
- Corrected the deployed Pi-to-FC default baud from 921600 to 115200.
- Verified there were no undervoltage/throttling flags at inspection time.

For reliable time after long offline storage, give the Pi periodic NTP access
or add a hardware RTC.

Verified on 2026-08-19 and treated as an invariant: **no systemd unit, timer,
or crontab on the Pi starts anything that talks to the flight controller at
boot.** `scripts/setup-pi-power-resilience.sh` re-checks this and refuses to
finish if a vehicle-control unit is ever enabled. An unexpected reboot must
never bring up a command path to the vehicle.

## Power-loss resilience

The Pi is powered from the airframe and is disconnected without a clean
shutdown routinely. On **2026-08-18** an unattended `apt-get full-upgrade` of
150+ packages was interrupted that way and required a manual rescue on
2026-08-19; the machine has since recovered completely.

Applied on 2026-08-19 with `sudo scripts/setup-pi-power-resilience.sh`:

- `apt-daily.timer` and `apt-daily-upgrade.timer` are **masked**. Upgrade only
  through `sudo scripts/pi-safe-upgrade.sh`, on a bench supply.
- The journal is **persistent and capped at 64 MB**, so a power cut now leaves
  a post-mortem instead of discarding every log.
- The hardware watchdog is pinned at 1 minute (the vendor value, deliberately
  not tightened).
- The root filesystem is set to `errors=remount-ro`, effective at next mount.
- The interrupted-upgrade recovery unit is installed but disabled; only
  `pi-safe-upgrade.sh` arms it.

The root filesystem stays writable by design; an overlay read-only root was
rejected because `drone-deploy` and the recorders both need a writable card.

A controlled reboot on 2026-08-19 at 13:57 confirmed that the journal history
accumulates across boots and that the live mount adopts `errors=remount-ro`.
All five measures survived the reboot and the Pi rejoined Tailscale in about
five seconds. None of this has been exercised against a real power cut.

Recorded data has its own policy in `ai_drone/durability.py`: small artifacts
are replaced atomically, and the JSONL streams flush every record while
fsyncing on a bounded interval (`drone-record --sync-interval`, default 5 s).

See [Pi power resilience](PI_POWER_RESILIENCE.md).

## Current network configuration

The onboard `wlan0` operates in one mode at a time:

- saved phone hotspots (names redacted from the repository): priority `100`;
- `eduroam`: priority `5`;
- fallback `Hotspot` / `AI-Drone-Zero`: priority `-10`.

As verified on 2026-08-19, `wlan0` is a **client** on a saved phone hotspot at
`10.189.46.25/24` with a working default route, and Tailscale is **online** at
`100.84.84.2`. This supersedes the 2026-08-18 capture, which recorded `wlan0`
serving `AI-Drone-Zero` at `192.168.4.1` with no route and Tailscale logged out;
the documented priority fallback behaved exactly as designed. `usb0` still has
no carrier and no `wlan1` exists. The eduroam profile and valid DFN CA
certificate remain installed on the Pi; the password is intentionally not in
the repository.

For simultaneous internet and `AI-Drone-Zero`, attach a second supported USB
Wi-Fi adapter or provide a USB Ethernet/tethering uplink. Keep `wlan0` as the
control AP. The new script defaults to a read-only preflight:

```bash
sudo scripts/setup-pi-dual-network.sh \
  --uplink-interface wlan1 \
  --source-profile eduroam
```

Only after a second adapter is present and the preflight succeeds:

```bash
sudo scripts/setup-pi-dual-network.sh \
  --uplink-interface wlan1 \
  --source-profile eduroam \
  --apply
```

See [network uplink](NETWORK_UPLINK.md), [eduroam setup](EDUROAM_SETUP.md), and
[hotspot setup](HOTSPOT.md).

## Installed Pi vision/software state

Key versions at the last audit:

- Picamera2 0.3.36
- rpicam-apps / IMX500 packages 1.12.0
- libcamera 0.7.1
- native AprilTag 3.4.2 ARM64
- OpenCV Python 4.11.0
- modlib 1.3.1
- NumPy 2.3.5 in the project environment
- pymavlink 2.4.49
- MAVProxy 1.8.74

Complete Debian, Python, OpenCV, system, controller, and sensor inventories are
under [`state/2026-08-18`](../state/2026-08-18/README.md).

## AprilTag print files

Five tag IDs (`0` through `4`) were generated in the `tag36h11` family for both
paper sizes:

- A4: nominal 160 mm tag, combined five-page PDF at
  `artifacts/apriltags/tag36h11-ids0-4-A4-160mm-5pages.pdf`.
- A3: nominal 224 mm tag, combined five-page PDF at
  `artifacts/apriltags/tag36h11-ids0-4-A3-224mm-5pages.pdf`.
- Individual PDF/SVG files and printing instructions are in
  `artifacts/apriltags/`.

The `artifacts/` directory is intentionally Git-ignored, so preserve or
regenerate these local files if moving to another checkout. Print at actual
size/100%, do not fit to page, and measure the printed black tag square before
using its size for pose estimation.

## Camera mounting decision

For detecting floor tags and dropping directly over them, mount the IMX500
straight down (nadir), rigidly and with ribbon-cable strain relief. Record the
camera translation and yaw relative to the flight-controller body frame.

Before metric pose or flight:

1. focus at the intended working altitude, initially 1–3 m;
2. lock the lens against vibration;
3. calibrate camera intrinsics at the actual detection resolution, initially
   1280x960;
4. measure the true printed tag size; and
5. measure the camera-to-body transform and payload release offset.

A forward or diagonal camera is not preferred for a floor-tag drop because the
target leaves view near final centering. Use a separate forward sensor/camera
for wall tags or navigation. See [AprilTag mission architecture](APRILTAG_MISSION.md).

## Implemented software

### AprilTag detector

- Core: `ai_drone/vision/apriltags.py`
- CLI: `ai_drone/cli/apriltag.py`
- Entry point: `drone-apriltag`
- Native AprilTag 3 is preferred; OpenCV AprilTag/Aruco is the fallback and is
  also used for calibrated square-IPPE pose solving.
- The IMX500 neural accelerator is not used for tag decoding.
- Metric pose requires a calibration file plus `--tag-size` in metres.

Safe invocation:

```bash
uv run drone-deploy --apriltag --backend auto --tag-size 0.160
```

At 1280x960, observed throughput was approximately 13–15 processed frames/s.
At 640x480, both detector paths ran close to the 30 fps camera rate. This was a
throughput benchmark only: no physical tag was actually visible in the frame.

### Disarmed all-sensor recorder

- Core: `ai_drone/recording.py`
- CLI: `ai_drone/cli/record.py`
- Entry point: `drone-record`

It records H.264 video, frame/AprilTag JSONL, raw MAVLink `.tlog`, decoded
telemetry JSONL, first/last JPEGs, and a manifest for an exact requested
interval. It refuses an initially armed heartbeat and aborts if an armed
heartbeat appears.

```bash
uv run drone-deploy --record --duration 15
```

Live validation:

- requested 3.000 seconds;
- encoded H.264 duration 2.998417 seconds;
- 87 encoded frames;
- 562 MAVLink messages;
- vehicle remained disarmed.

An eight-second dataset is retained locally at
`artifacts/sensor-recordings/live-apriltag-20260818/`. Its preview showed a
laptop occupying nearly the whole image, so zero tag detections did not test the
detector's physical range.

### Consolidated flight control

- Core: `ai_drone/flight/controller.py` and `ai_drone/flight/follower.py`
- CLI: `ai_drone/cli/control.py`
- Entry point: `drone-control`

This single command retains passive `status`, `hover` (`takeoff` alias),
`velocity-test`, and `follow`. Each live flight sequence owns its arming,
stopping, landing, and cleanup; simulated follow is fully offline. Dedicated
follow and linear-flight wrappers are folded into this one command. The control
capability is implemented but has not been validated in live flight; its
presence does not override the current hardware blocks in the test envelope
above.

### Complete configuration snapshot

- Read-only Pi exporter: `ai_drone/cli/config_export.py`
- Host sync/publish command: `ai_drone/config/sync.py`
- Entry points: `drone-config-export` and `drone-config-sync`

Capture all current parameters through the Pi:

```bash
uv run drone-config-sync
```

From a clean worktree, capture, commit exactly the two generated files, and
push the current branch:

```bash
uv run drone-config-sync --publish
```

The final live validation received 1152/1152 unique indexed parameters while
disarmed. Snapshot metadata is in
[`state/2026-08-18/drone-config.json`](../state/2026-08-18/drone-config.json),
and the parameter file is
[`params/flywoo-f745-live-2026-08-18.param`](../params/flywoo-f745-live-2026-08-18.param).

### Guarded bench motor test

- CLI: `ai_drone/cli/motor_test.py`
- Entry point: `drone-motor-test`
- Deployment mode: `drone-deploy --motor-test`

The utility uses only ArduPilot `MAV_CMD_DO_MOTOR_TEST`; it does not send the
normal arm command. It requires exact propeller-removal and vehicle-secured
confirmations, a countdown, disarmed starting state, contiguous configured
motor outputs, and exactly `ARMING_CHECK=1`. It caps each motor at 10% and one
second, sends a stop request in cleanup, and waits for a disarmed heartbeat.

The program was unit-tested but deliberately not run on the real motors. Follow
[the bench motor-test procedure](BENCH_MOTOR_TEST.md) only after removing all
propellers, securing the vehicle, restoring `ARMING_CHECK=1`, and resolving all
pre-arm failures.

## Sensor observations

Two different resting geometries were sampled:

- Earlier ten-second MTF sample: median range about 2.20 m and flow quality
  about 38/255.
- Integrated recorder with the downward sensor nearly touching a surface:
  range about 2 cm and flow quality about 147/255.

These prove transport, power, and message flow only. They do not validate
altitude-hold quality. At the first state capture, the vehicle reported roughly
-133 degrees roll and -53 degrees pitch, no GPS fix, no home position, and no
`LOCAL_POSITION_NED` estimate. Level mounting, sensor calibration, correct
orientation, suitable floor texture/light, and controlled motion tests remain
necessary.

## Feasibility and architectural limits

AprilTag approach and payload placement are feasible after calibration and
staged testing. "Exact" distance is not physically achievable; use a
confidence-bounded estimate from known tag size, calibrated camera intrinsics,
reprojection error, camera/body transform, attitude, and downward LiDAR.

The current downward LiDAR/flow plus one future forward MT-15 are insufficient
for robust unknown-hall navigation. A single forward beam cannot detect side
hazards or build a 2D map. Reliable hall autonomy will likely require:

- a 2D LiDAR, depth camera, or equivalent wider obstacle sensor;
- SLAM/localization and a planner;
- a stronger companion computer than the Pi Zero 2 W for comfortable real-time
  vision plus mapping; and
- explicit manual-override, loss-of-localization, battery, link, and obstacle
  failsafes.

## Highest-priority next work

1. Rigidly mount and focus the nadir camera; make the AprilTag fully visible.
2. Build and run a camera calibration at 1280x960.
3. Record stationary A4/A3 tags across planned altitude, angle, lighting, and
   motion-blur conditions; produce detection-probability and pose-error plots.
4. Restore `ARMING_CHECK=1` and diagnose every pre-arm failure. Do not add a
   bypass flag to the motor-test program.
5. Verify the frame is level and calibrate IMU, compass, optical-flow scale and
   orientation, and downward rangefinder accuracy.
6. Run the guarded motor test with props removed, one motor at a time, then
   confirm numbering and rotation.
7. Exercise every existing `drone-control` mode in ArduPilot SITL, then develop
   the tag approach/release state machine there with strict
   velocity/altitude/time limits and no payload actuation.
8. Add a comprehensive preflight-health command that refuses flight on stale or
   missing EKF, range, flow, battery, RC, camera-calibration, and tag-map data.
9. Select and integrate the forward/hall navigation sensor and companion
   computer before attempting autonomous hall search.

## Validation commands

Run before committing later changes:

```bash
UV_CACHE_DIR=/tmp/uv-cache uv sync --frozen --group dev
UV_CACHE_DIR=/tmp/uv-cache uv run --frozen --group dev ruff format --check .
UV_CACHE_DIR=/tmp/uv-cache uv run --frozen --group dev ruff check .
UV_CACHE_DIR=/tmp/uv-cache uv run --frozen --group dev ty check .
UV_CACHE_DIR=/tmp/uv-cache uv run --frozen --group dev pytest -q
bash -n scripts/setup-pi-hotspot.sh scripts/setup-pi-dual-network.sh \
  scripts/setup-pi-power-resilience.sh scripts/pi-safe-upgrade.sh
git diff --check
```

Last complete result after cleanup: **305 passed, 1 skipped** on the supported
Python 3.12 environment, including a deprecation-warning-as-error pass. The
skipped host test requires the Pi's OpenCV pose environment; synthetic pose
solving was separately checked on the Pi earlier.
