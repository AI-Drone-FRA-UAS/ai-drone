# Drone handover report — 22 September 2026

Prepared from this conversation, the current working tree, recorded Pi checks,
and the linked earlier-session reports. Times are Europe/Berlin (CEST) unless
marked UTC. This is a handover of the last observed state; writing this report
did not rerun hardware tests or change the Pi.

## 1. Objective and immediate status

The owner wants a teammate to **fly manually with the RC controller** while the
Pi records the flight and **opens the payload mount once when AprilTag
tag36h11 ID 6 is detected**. The Pi must not take flight control. Autonomous
flight, object detection/model training, Python migration and general OS
upgrades are outside the immediate objective.

**The hotspot is active and the payload software is prepared, but flight
readiness has not been established.** The latest checks found compass and
rangefinder pre-arm failures, a motor-output cap affecting manual flight, and
failsafe settings that need review for RC operation. A physical ID 6 release
has not been verified.

| Item | Last observed state |
| --- | --- |
| Pi access | USB SSH working; USB cable connected according to owner |
| Pi hotspot | `AI-Drone-Zero` active, Pi `192.168.4.1`; visible to PC at signal 100 |
| Hotspot SSH | Server listening; PC has not yet been joined to test SSH through the AP |
| eduroam / Tailscale | eduroam connection failed; tailscaled running but Internet path unavailable |
| Flight controller | Disarmed, `STABILIZE`, ArduCopter 4.7.1 / `dbe79216` |
| Controller | Owner confirmed on and linked; teammate uses a dedicated arm switch |
| Release watcher | Stopped; does not start automatically after reboot |
| Other camera/runtime jobs | Inspected capture/runtime units inactive; UART and camera handles clear |
| Payload mount | Owner reported it stayed closed through the battery change; no position sensor verifies its present position |
| Focus | Preview prepared and subsequently paused; completed physical focus adjustment not confirmed |
| Live Python | System CPython 3.13.5 through the existing project environment |
| Flight-controller changes in these checks | No parameter writes, firmware flashing, arming or mode changes |

## 2. Locations, hardware and working rules

| Item | Location / connection |
| --- | --- |
| PC repository | `/home/abaris/drone/ai-drone` |
| Branch | `refactor/concise-runtime` |
| Current HEAD | `461bb1effb4be547f017b02c32b31101e5060133` |
| Prior session reviewed | `01a0c3d4-eea9-7ac3-a1e7-25d0ea329183` |
| Pi account / project | `seb`, `/home/seb/ai-drone` |
| Pi uv | `/home/seb/.local/bin/uv`, observed version 0.12.5 |
| Pi project Python | `/home/seb/ai-drone/.venv/bin/python`, system-site-packages enabled |
| Tailscale identity | `seb-is-pm.tail59e6a4.ts.net`, `100.84.84.2` |
| USB network | Pi `192.168.7.2`, PC `192.168.7.1` |
| FC connection from Pi | `/dev/serial0`, 115200 baud, MAVLink system/component 1/1 |
| Camera | Raspberry Pi AI Camera, IMX500, CSI, Picamera2/libcamera |
| Payload servo | Pi BCM12, physical pin 32; not an FC servo output |
| FC board | Documented Flywoo GOKU GN745 AIO / FlywooF745 firmware target |
| Downward sensor | MicoAir MTF-01P, FC UART5, range and optical flow |
| Forward sensor | MicoAir MT-15, FC UART3; swapped RX/TX pin mapping, `SERIAL3_OPTIONS=8` |

Use **uv** for all Python execution, dependencies and environments. Preserve the
working Pi environment and its system camera bindings. Use the SSH identity
`seb@seb-is-pm`, with the address overrides below when necessary. On this PC,
shell network access may require sandbox escalation. Do not change the PC's
Wi-Fi connection automatically.

The shared `ai-drone-runtime.service` was not installed as the live access
service. Current commands use direct UART access. Do not run two UART consumers
or two camera jobs at the same time. The older autonomous profile explicitly
assumes a receiver-free aircraft; it must not be blindly applied to this manual
RC configuration.

The documented Pi–FC UART wiring contains TX, RX and ground. Earlier work left
the actual flight power arrangement unverified; occasional heartbeats are not
proof that the intended power path is suitable. See
[hardware topology](../../docs/DRONE_CONFIGURATION.md).

## 3. Connection commands

Normal Tailscale connection, when the Pi has working network access:

```bash
ssh seb@seb-is-pm
```

USB connection from this PC:

```bash
ssh -o Hostname=192.168.7.2 \
  -o HostKeyAlias=seb-is-pm.tail59e6a4.ts.net \
  -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
  seb@seb-is-pm
```

For direct wireless access, manually join **AI-Drone-Zero**, using its existing
password, then run:

```bash
ssh -o Hostname=192.168.4.1 \
  -o HostKeyAlias=seb-is-pm.tail59e6a4.ts.net \
  -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=2 \
  seb@seb-is-pm
cd ~/ai-drone
```

The host-key alias preserves the Pi's existing trusted identity. Do not work
around identity problems by disabling host-key verification. Credentials are
intentionally not included in this report.

The existing NetworkManager profile is named `Hotspot`. Its activation was
verified, but `connection.autoconnect` remains `no`: it will not automatically
come back after another reboot. While connected over USB, the Pi command to
activate it again is:

```bash
sudo nmcli --wait 25 connection up Hotspot ifname wlan0
```

To return to the saved campus profile, use USB so the access path survives:

```bash
sudo nmcli --wait 30 connection up eduroam ifname wlan0
```

That last command is a recovery option, not a verified permanent fix. The
single Wi-Fi interface currently serves either the hotspot or the campus
connection. Direct hotspot SSH does not require Internet or Tailscale.

## 4. Work completed during the current sequence

### Application review and deployment

Reviewed the previous session, current source and existing uncommitted work.
Redeployed the application and locked dependencies while retaining the working
Python 3.13 environment. Fixed deployment handling of lgpio `.lgd-nfy*`
notification FIFOs: the transaction previously tried to copy a FIFO and stopped
before mutation. Those transient paths are now excluded/preserved, and the
existing deployment test exercises a real FIFO.

Validation recorded in the [focus/update report](focus-preview-status.md):
1,868 host tests passed, with 40 SITL cases deselected; all 86 deployment tests
passed after the fix. Lint, formatting, types, seven import contracts, dependency
checks and the 13-page documentation build passed. Sandbox socket denials were
resolved by rerunning the host checks outside the sandbox.

The host tag-print helper generated A3 sheets and decoded IDs 0, 3, 17 and 586.
The range-analysis helper's implied-size calculation was corrected to use
matched observations, including a synthetic missing-range check. These remain
host tools rather than additions to the Pi's flight runtime.

OS package metadata was refreshed and an upgrade simulated: 84 upgrades,
eight new packages, zero removals. **The OS upgrade was not executed.** The
user prioritized a stable focus preview and manual flight. Python 3.14 and
shared-service migration were also deferred. Later battery changes and the
requested shutdown/restart did occur; the earlier focus note's “no reboot”
statement describes that earlier stage only.

### Camera focus preview

Started and visually checked a passive preview on port 8081. The initial
greyscale display used the luminance image for detection; it was not evidence
that the camera cannot capture colour.

At the owner's request, a temporary adapter produced a **colour preview rotated
180 degrees**, with tag outlines/IDs, downward-range validity, recent detection
hit rate, last-seen age, central-image sharpness and camera FocusFoM. Display
rotation did not change detector coordinates or the installed application's
camera configuration. Actual capture with that preview was about 5 fps.

The adapter was `/tmp/ai-drone-focus-colour.py`, run as
`ai-drone-focus.service`. Its temporary location must not be assumed to survive
the subsequent reboot. The preview was paused at the owner's request. The
white focus tool must physically turn the lens; software did not adjust it.
Intended working distance and final focus quality remain unconfirmed.

At one earlier sample the downward reading was zero and invalid; no tag was
decoded. The scene showed tags at a shallow angle. Detection hit rate is not a
confidence probability, and sharpness scores should only be compared for the
same target, lighting and distance. These historical samples are not current
sensor readings. See [focus details](focus-preview-status.md) and the persistent
preview commands in [Operations](../../docs/OPERATIONS.md#camera-focus-preview).

### Mount operation and battery changes

The owner wanted independent, non-interactive open and close commands after
observing an earlier command open and immediately close the mount. The selected
dedicated mount path commands one position and releases PWM on exit; it does
not intentionally issue the opposite position during cleanup.

The owner confirmed the mount remained closed after a battery change/restart.
That is an observed mechanical result, not a guarantee of retention under
payload or flight loads. Software has no mount-position feedback.

The requested Pi shutdown was performed after stopping capture jobs and syncing
storage. The owner subsequently restarted the Pi. The latest network and
arming diagnosis below was made after that restart.

## 5. Payload script and exact operating commands

The convenience launcher is
[`scripts/payload_flight.sh`](../../scripts/payload_flight.sh). It starts
[`scripts/tag_mount_capture.py`](../../scripts/tag_mount_capture.py), which uses
the existing recorder in `tag-mount` operation.

These are the commands on the Pi as user `seb`, from `~/ai-drone`. They document
the prepared procedure; the unresolved flight issues in section 8 still apply.

Independent mount commands:

```bash
uv run --no-sync drone mount open
uv run --no-sync drone mount close
```

These are two separate actions. Execute only the one intended. Equivalent
entry points are `uv run --no-sync python scripts/mount.py open` and `close`.
The current packaged executable is `drone`; the owner's older `uv run mount`
spelling is not a declared entry point in this checkout.

Mount settings are BCM12, 900–2100 microseconds, open `0.0` / 1500 microseconds,
close `-1.0` / 900 microseconds, and a default 0.5-second hold. PWM then stops.
The shared lock directory `/run/ai-drone-locks` is private, mode 0700, owned by
seb. It is recreated at boot by `/etc/tmpfiles.d/ai-drone-locks.conf`; the
launcher also ensures it exists. An active watcher can hold the servo lock,
so stop it before using the manual mount commands.

Start recording and the ID 6 release watcher when the operator deliberately
wants tag-triggered release enabled:

```bash
bash scripts/payload_flight.sh start 6
bash scripts/payload_flight.sh logs
```

Wait for `READY` and continuing frame/telemetry progress. **ID 6 can trigger on
the ground while disarmed**, so keep it out of view until release is intended.
`READY` is startup status, not proof of continued camera health or flight
readiness. Closing SSH or pressing Ctrl-C in the log viewer leaves the watcher
running.

Status and stop commands:

```bash
bash scripts/payload_flight.sh status
bash scripts/payload_flight.sh stop
```

The job is the transient systemd unit `ai-drone-tag-mount.service`. It runs until
stopped or a failure occurs, survives SSH disconnection, and does not start on
boot. It continues recording across arming/disarming and after opening the
mount. It does not automatically close the mount.

The helper defaults to **ID 6**. The underlying Python script defaults to
**ID 3**, so direct invocations must explicitly include `--tag-id 6`.

Release qualification uses tag36h11, three consecutive qualifying observations,
zero corrected bits (`hamming=0`), decision margin at least 30, detection age at
most 0.5 seconds and selected-FC heartbeat age at most 2.5 seconds. The selected
ID triggers once per process run. Restarting the process resets that history.
The mount opens with the usual 0.5-second pulse, then PWM detaches.

The script requests telemetry and reads parameters but sends no arming, mode,
motor, RC-override, mission or flight-control setpoint commands. It does not
supply GCS heartbeats. It requires **`ARMING_SKIPCHK=0`**; disabling checks can
therefore also make this watcher refuse startup. Neither that requirement nor
a successful release log proves mechanical movement or flight readiness.

For the next battery change, stop an active recording and allow it to finalize
before executing `sync` and `sudo shutdown -h now`. Once shutdown is complete,
change power and reconnect after boot. The watcher must be started again
explicitly; no mount position is automatically commanded on boot.

## 6. Camera failure and script tests

The owner's 16:01 start reached `READY`, then libcamera reported a camera
frontend timeout after only two analysed frames. The script stopped with
`camera_stalled`, a missing-frame timeout and zero servo pulses. The dataset
on the Pi is `artifacts/sensor-recordings/20260922-140140/`.

The log suggested checking the camera connection, but a loose cable or defective
sensor was **not proven**. A short standalone camera test and subsequent passive
captures succeeded. The fault remains intermittent/unexplained.

At the owner's request, both scripts were then tested briefly with the real
camera, native AprilTag detector and real FC telemetry, but **mock GPIO**:

| Test | Result |
| --- | --- |
| Direct capture, 15 s | 120 analysed frames, 402 encoded frames, 2,012 telemetry messages; completed, `duration_elapsed`, no errors |
| Corrected launcher start/stop, 12.961 s | 97 analysed frames, 351 encoded frames, 1,736 messages; completed, `operator_signal_SIGTERM`, no errors |

Pi datasets are `artifacts/tag6-direct-test-20260922/` and
`artifacts/sensor-recordings/20260922-141637/`, respectively.

Two earlier launcher stop tests used SIGINT, exited 130 and failed to finalize
their manifests. The deployed Debian AprilTag binding temporarily changes
SIGINT handling during detection. The launcher was corrected to use:

```text
KillSignal=SIGTERM
KillMode=mixed
TimeoutStopSec=60
```

The corrected stop was verified against the final manifest. Shell syntax and
ShellCheck passed. Temporary mock-GPIO uv wrappers were removed; production
was not left in mock mode. Both scripts were left stopped.

**No ID 6 was seen in these tests and no physical tag-triggered release was
attempted.** They verify a short recording/telemetry run and graceful shutdown,
not detection range, payload release, sustained operation or flight readiness.
See [script test details](payload-flight-script-checks.md).

## 7. Networking diagnosis and changes

After a battery change, Wi-Fi/Tailscale access failed and USB restored access.
The saved eduroam profile was set to unlimited autoconnect retries
(`connection.autoconnect-retries=0`) and disabled Wi-Fi power saving
(`802-11-wireless.powersave=2`). A Wi-Fi radio off/on cycle and reconnection
temporarily restored connectivity. Tailscale recovered when network access
returned; node credentials were not reset.

After the later reboot, the Pi joined eduroam at **16:24:18**, acquired
`10.53.135.29`, and disconnected at **16:24:29**. Reassociation attempts to two
APs produced `CTRL-EVENT-ASSOC-REJECT`, status code 16, `CONN_FAILED`, and finally
`supplicant-timeout`. Earlier attempts had also logged driver association
failures. The precise root cause remains unproven.

The most recent Pi check reported `throttled=0x0`, temperature 58°C, and Wi-Fi
power saving already disabled. Those observations do not establish the cause
of the wireless failure. Tailscaled remained active; with Wi-Fi disconnected
there was no Internet route. Reauthentication was not indicated by the evidence.

The existing hotspot was activated at approximately 16:38. NetworkManager
reported `Hotspot` connected at `192.168.4.1/24`, DHCP running, and SSH listening
on all interfaces. A PC scan saw `AI-Drone-Zero` on channel 11 at signal 100.
The PC's own connection was not switched. No hotspot password was changed.

## 8. Current FC findings and unresolved flight issues

Latest complete evidence:
[`usb-prearm-final.json`](../../artifacts/manual-rc-check-20260922/usb-prearm-final.json)
and
[`usb-probe-paced.json`](../../artifacts/manual-rc-check-20260922/usb-probe-paced.json).
The final check began at 16:41:49 CEST. It returned failure because the reported
sensor/pre-arm checks failed; this was not an SSH failure.

The final explicit FC errors were:

```text
PreArm: Check mag field: 1364, max 875, min 185
PreArm: Rangefinder 1: No Data
PreArm: Check mag field: 1362, max 875, min 185
```

Earlier samples also reported `Check mag field (z diff:207>200)` and, before the
latest reboot, `GCS failsafe on`. The GCS-failsafe message did not recur in the
final sample. Optical-flow and rangefinder health were false despite some range
messages being received. Latest recorded distances were downward 0.02 m and
forward 0.45 m; battery was 15.772 V. These are timestamped observations, not
present measurements or validated height estimates.

| Parameter | Readback | Operational implication |
| --- | --- | --- |
| `MOT_SPIN_MAX` | 0.40 | Motor actuator output capped, including manual flight; not equivalent to measured 40% thrust |
| `MOT_PWM_MIN` / `MOT_PWM_MAX` | 1000 / 2000 | Configured motor output range |
| `MOT_PWM_TYPE` | 6 | DShot600 |
| `MOT_BAT_CURR_MAX` | 0 | Current-based throttle limiting disabled |
| `ATC_ANGLE_MAX` | 15 | Requested lean angle limited to 15 degrees |
| `RC8_OPTION` | 154 | Arm/disarm with AirMode on channel 8 |
| `ARMING_RUDDER` | 0 | Stick arming disabled; owner uses a dedicated switch |
| `ARMING_SKIPCHK` | 0 | Configurable pre-arm checks remain enabled |
| `FS_THR_ENABLE` | 0 | Radio-loss failsafe disabled |
| `FS_GCS_ENABLE` / `FS_GCS_TIMEOUT` | 5 / 5 | GCS-heartbeat-loss action is Land after five seconds |
| `FS_OPTIONS` | 8 | Continue landing on failsafe; no manual-mode GCS exemption |
| `EK3_SRC1_YAW` | 1 | Compass used for yaw |

RC3 and RC8 were 988 in the samples, consistent with low throttle and a low arm
switch. SYS_STATUS reported the receiver healthy, but RC_CHANNELS reported
`chancount=0`. Stick response and receiver-loss behavior have not been tested.
Do not infer either verified manual control or a physically absent receiver
from these observations alone.

No output limit, failsafe or pre-arm setting was changed. The owner asked
whether checks could be removed; the response explained that manual flight
still uses ArduPilot stabilization and recommended fixing the reported faults.
No blanket bypass was applied. The 4.7 parameter is `ARMING_SKIPCHK` with inverse
semantics to the older `ARMING_CHECK`; old instructions to set `ARMING_CHECK=0`
must not be applied to this firmware.

For further inspection, with capture stopped and the vehicle disarmed:

```bash
cd ~/ai-drone
uv run --no-sync --python .venv/bin/python drone check --duration 8 --prearm --json
```

This checker includes navigation-sensor criteria; distinguish its broader
diagnostics from the FC's explicit `PreArm:` messages. A range reading alone
does not establish stable range/flow input. Do not start the autonomous
controller merely to supply a GCS heartbeat or clear a failsafe.

## 9. Recordings, offload and storage

The payload launcher records colour H.264 at 1280×960, requested 30 fps,
target 8 Mbit/s, plus frame timestamps, camera metadata, tag detections,
telemetry and servo events. Analysis defaults to 640×480. Actual analysis rate
is lower than encoded video rate and must be read from each dataset.

Recordings live under `/home/seb/ai-drone/artifacts/sensor-recordings/`.
Keep each complete directory, including its manifest, negative/no-tag frames
and timing/telemetry files. This is compressed colour video, not raw sensor
capture or IMX500 inference tensors. No object-detection training was started.

At 8 Mbit/s, video alone consumes about **3.6 GB/hour**. The last relevant free
space estimate was about 19.4 GiB, approximately five hours with overhead and
reserves. Recheck with `df -h ~/ai-drone`; this is a storage estimate, not a
battery, thermal or reliability guarantee. Camera/heartbeat failure or storage
policy can stop the recording earlier.

Files were copied from the Pi to this PC under:

```text
/home/abaris/drone/ai-drone/artifacts/pi-offload-20260922-usb/
```

Copied areas include `home-seb/ai-drone-candidates`, `home-seb/ai-drone-archive`,
`home-seb/.cache/uv`, and `home-seb/ai-drone/artifacts`. The recorded transfer
covered about 51,201 entries, 3.703 GB transferred and 5.273 GB logical size with
hard links. **This was a copy, not a completed move:** independent verification
and removal from the Pi were not completed. Originals were preserved, so do
not claim storage was reclaimed or that later recordings are included.

The offload root is mode 0700. Some retained system/network restoration
archives contain private configuration and credentials; do not publish them.
No credential contents are included in this handover.

## 10. Earlier refactor and Python candidate work

The preceding session implemented runtime/deployment/recording changes and
prepared an isolated Python 3.14 candidate. Full history and evidence are in
[refactor completion](refactor-completion.md), with the older checkpoint in
[September 21](../2026-09-21/refactor-resume-checkpoint.md).

Relevant changes now in the branch include:

| Commit | Change |
| --- | --- |
| `ff40754` | Abort an owned autonomous flight on a fresh FC magnetic yaw-reset diagnostic |
| `12ccd2d` | Remove duplicate recording cleanup |
| `3a64313` | Explicit offline native-payload deployment and paired source/environment rollback support |
| `9b4336d` | Separate intentional receive cancellation from measured transport errors |
| `44d0818` | Separate native AprilTag patch releasing the GIL and preserving Python signal handling |
| `f520dd0` | Close drained telemetry subscription before slow recorder teardown |
| `461bb1e` | Initialize OpenCV before subscribing to telemetry, addressing startup queue buildup |

Earlier host suites passed 1,868 cases on both CPython 3.13 and 3.14. The full
isolated SITL matrix at `9b4336d` was **35 passed, five failed, zero skipped**;
two recording cases affected by `f520dd0` subsequently passed again. The five
failures were 30-second altitude-hold horizontal-clearance cases. These are
separate from the current manual RC task and are not evidence of autonomous
flight readiness. The refactor expanded functionality and did not produce an
overall code-size reduction.

The separate Pi candidate root is
`/home/seb/ai-drone-candidates/20260921-py314`. Native ARM64 artifacts, imports,
synthetic ID17 detection, offline installation and some passive capture checks
were exercised, with failed attempts also retained. Telemetry overflows, startup
work, status storage latency and cleanup behavior were investigated. A passing
short candidate run did not close full migration/service/rollback qualification.

**The live Python 3.13 Debian AprilTag binding was not replaced by that candidate
patch.** This explains why the current launcher still needs the verified SIGTERM
shutdown behavior. Do not confuse candidate build results with deployed runtime
behavior. No Python 3.14 promotion or shared-service migration was completed.

The earlier completion report describes the `f520dd0` stage and leaves startup
work open; HEAD now contains `461bb1e` and the later host validation described
above. Similarly, its historical statements about no deployment, shutdown or
actuator work should not be used as a description of all subsequent owner-led
mount/battery operations. This handover's operational snapshot is newer.

## 11. Working tree and evidence to preserve

No new commit was made for the focus/payload/network work. There is existing
uncommitted work from several stages; preserve it rather than resetting the
branch. At report preparation, modified tracked files were:

```text
ai_drone/link/deploy.py
docs/OPERATIONS.md
docs/PYTHON_RUNTIME.md
state/2026-09-21/hardware-qualification-checklist.md
state/2026-09-21/python-314-candidate.md
state/2026-09-21/refactor-implementation-report.md
state/2026-09-21/refactor-progress.md
tests/test_deploy.py
```

Untracked work includes:

```text
scripts/generate_apriltag_prints.py
scripts/payload_flight.sh
scripts/tag_range_check.py
state/2026-09-21/refactor-resume-checkpoint.md
state/2026-09-22/
```

The local application deployment fix, launcher and Operations additions belong
to the current sequence. The Python/refactor reports and helper scripts also
include earlier work; review their diffs before assigning provenance or
committing. Artifacts are separate from tracked source. The Pi checkout is a
deployed runtime, so do not assume it contains every host-only file.

Main handover references:

| Evidence | Purpose |
| --- | --- |
| [Focus/update status](focus-preview-status.md) | Application update, host tests, preview and colour adapter |
| [Payload tests](payload-flight-script-checks.md) | Direct/launcher metrics and SIGTERM correction |
| [RC/hotspot check](manual-rc-and-hotspot-check.md) | Latest network diagnosis and FC parameter interpretation |
| [Final pre-arm JSON](../../artifacts/manual-rc-check-20260922/usb-prearm-final.json) | Latest complete FC diagnostic capture |
| [Paced parameter probe](../../artifacts/manual-rc-check-20260922/usb-probe-paced.json) | Motor, arming, RC and failsafe readbacks |
| [Operations](../../docs/OPERATIONS.md) | Maintained preview, recording and mount instructions |
| [Python runtime](../../docs/PYTHON_RUNTIME.md) | Interpreter/native dependency deployment contract |
| [Hardware qualification](../2026-09-21/hardware-qualification-checklist.md) | Earlier unresolved physical/autonomous qualification; historical scope |

`artifacts/manual-rc-check-20260922/prearm.json` is the empty output from the
earlier dropped-SSH attempt; it is not a successful check. `usb-probe.json` is
the initial burst parameter query with missing replies; prefer the paced probe.
Temporary helpers under `/tmp` are convenience files, not durable evidence.

## 12. Recommended takeover sequence

1. Reconnect over USB or manually join the hotspot. Refresh disarmed state and
   active service/device ownership; do not treat this dated report as live state.
2. Resolve the compass-field failure by comparing a location away from metal and
   electronics, then investigate mounting/orientation/calibration if it persists.
   Diagnose intermittent MTF-01P range/flow data and its UART/power path.
3. Verify RC stick/channel mapping and the channel 8 arm-switch mapping with
   motors prevented from starting. Establish receiver-loss behavior, then review
   GCS-loss versus radio-loss handling for the intended manual RC configuration.
4. Review why `MOT_SPIN_MAX=0.40` and `ATC_ANGLE_MAX=15` were selected before
   changing them. No conclusion about adequate payload thrust can be drawn from
   the current bench checks.
5. Finish physical lens focus at the intended tag distance and angle. Then run
   a controlled bench test of real ID 6 recognition and actual mount release,
   accounting for the trigger being active while disarmed.
6. Verify sustained camera capture and graceful finalization. Keep the failed
   camera run as evidence; a short later pass did not establish its root cause.
7. Once the pilot has resolved flight readiness, prepare/close the mount,
   deliberately enable the ID 6 watcher, confirm continuing logs, conduct the
   manually piloted flight, and stop/finalize recording after landing.
8. Verify copied datasets before deleting anything from the Pi. Revisit OS
   upgrades, Python 3.14, autonomous qualification and model training separately
   after the immediate manual-flight work.

Documentation used in interpreting the existing observations:
[pre-arm checks](https://ardupilot.org/copter/docs/common-prearm-safety-checks.html),
[Stabilize](https://ardupilot.org/copter/docs/stabilize-mode.html),
[motor thrust scaling](https://ardupilot.org/copter/docs/motor-thrust-scaling.html),
[GCS failsafe](https://ardupilot.org/copter/docs/gcs-failsafe.html), and
[radio failsafe](https://ardupilot.org/copter/docs/radio-failsafe.html).
