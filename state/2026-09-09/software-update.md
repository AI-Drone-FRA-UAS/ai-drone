# Software update and UART startup investigation

The Pi's system packages, uv and project Python dependencies were updated on
9 September 2026. The final 30-second recording, launched as the first UART
access after another Pi reboot, captured both rangefinders, optical flow and
camera data successfully. Two earlier boots had startup heartbeat failures.
The recorder now retries the Pi UART once and logs diagnostics, but the final
run succeeded without using that retry. **The intermittent failure's underlying
cause and the retry's effectiveness on real hardware remain unproven.**

The flight controller remains on the project's custom ArduCopter 4.7.0 build,
`1511f271`. A custom 4.7.1 candidate has been built and tested offline; it has
**not been flashed**. No actuator commands or live flight tests were part of
this update checkpoint. Normal inspection requests telemetry message rates;
the UART diagnostic reads do not send MAVLink commands.

## Applied Pi and Python updates

The recoverable [Pi upgrade procedure](../../docs/PI_POWER_RESILIENCE.md)
completed with a consistent package database. The recorded package delta is
27 upgraded packages and two additions, with no removals. Examples include:

| Package | Before | After |
| --- | --- | --- |
| uv | 0.12.5 | 0.12.11 |
| Tailscale | 1.102.2 | 1.102.3 |
| raspberrypi-sys-mods | 1:20260612 | 1:20260907 |
| ffmpeg | 8:7.1.5-0+deb13u1+rpt1 | 8:7.1.5-0+deb13u1+rpt2 |
| OpenSSL | 3.5.6-1~deb13u2+rpt1 | 3.5.7-1~deb13u2 |
| Mesa packages | 26.2.0-1~bpo13+0~rpt3 | 26.2.1-2~bpo13+0~rpt1 |

uv was updated separately using its updater; it is not an apt package in this
delta. Apt also updated OpenJDK, xkbcommon and Pi swap/loop utilities and added
`ack` and `libfile-next-perl`. Kernel package versions remained at
6.18.39+rpt-rpi-v8; the upgrade regenerated the existing initramfs. Recorded
raspi-firmware, systemd/udev, bluez, rfkill and raspi-config versions did not
change.

After reboot, Tailscale and eduroam worked, `get_throttled` reported `0x0`,
the unattended apt timers remained masked, and the recovery service was
disabled after successful completion. These observations do not establish the
cause of the intermittent UART failure.

The refreshed `uv.lock` retains Python 3.11–3.13 support. Important dependency
changes are:

| Dependency | Previous lock | Updated lock |
| --- | --- | --- |
| NumPy | 2.3.5 | 2.4.6 on Python 3.11; 2.5.3 on Python 3.12–3.13 |
| OpenCV Python | 4.11.0.86 | 5.0.0.93 |
| Pillow | 12.2.0 | 12.3.0 |
| lxml | 6.1.1 | 6.1.3 |
| pytest | 9.0.3 | 9.1.1 |
| Ruff | 0.15.12 | 0.16.6 |
| ty | 0.0.46 | 0.0.79 |
| import-linter | 2.13 | 2.15 |
| markdown-it-py | 4.0.0 | 4.2.0 |

pymavlink remains 2.4.49 and pySerial remains 3.5. The deployed Pi uses Debian
Python 3.13.5, NumPy 2.5.3, OpenCV 5.0.0 and Pillow 12.3.0. Native camera
imports passed. The environment retains access to apt-installed camera
bindings; operating examples now use `uv run --locked --group raspi` on the
Pi. See the maintained [recording procedure](../../docs/SENSOR_RECORDING.md).

The initial deployment record verified 56 file hashes after backing up the
prior runtime. Its source provenance was commit
`44d04d96a089d26f0678e508f9cce28c4b910a91` **plus working-tree updates**, recorded
explicitly as dirty. It is not a claim that the deployed files equal that
unmodified commit. The deployed lock SHA-256 was
`a948f86e8b7469cfcd98c3ce4bd0f9815db28a5b19d2a3dfd9a6815a41e81a3a`.
The final runtime, including the bounded UART retry, was deployed and verified
again before its cold-start capture. Final source revision and hashes are
recorded in `artifacts/deployment/current.json` on the Pi and the local
`artifacts/software-update-20260909/pi-runtime-final.json`.

## Recording results and unresolved startup failure

All four runs below used the ordinary detached walker and inspector on
`/dev/serial0`, 115200 baud, without manual-flight or servo options. Camera
recording and automatic report generation completed in each case.

| Dataset suffix | Capture duration | Analyzed / encoded frames | FC messages | Result |
| --- | --- | --- | --- | --- |
| `update-check-20260909` | 30.039759 s | 516 / 898 | 0 | Startup heartbeat timed out after first Pi reboot |
| `update-check-20260909-retry` | 30.001977 s | 864 / 884 | 4,797 | FC, both ranges and optical flow healthy at capture end |
| `update-check-20260909-cold` | 30.002521 s | 865 / 873 | 0 | Startup heartbeat timeout repeated after another Pi reboot |
| `update-check-20260909-final` | 30.001438 s | 845 / 897 | 4,787 | First UART access after reboot; all requested sensor collectors worked |

The successful retry recorded 30 selected-FC disarmed heartbeats, 599 forward
range samples, 599 downward range samples and 599 optical-flow samples. Both
ranges and flow ran at approximately 20 Hz. It stopped with `duration_elapsed`
and reported zero commanded servo pulses. Its dataset and browser report are
under `/home/seb/ai-drone/artifacts/update-check-20260909-retry/` on the Pi.

The final run recorded 598 forward, 598 downward and 598 optical-flow samples,
with disarmed heartbeats throughout and no actuator commands. Its complete
dataset and generated CSV/video/browser report are in
`/home/seb/ai-drone/artifacts/update-check-20260909-final/` and copied locally
under the software-update artifact directory. No startup retry was logged.
The pack reported approximately 12.74 V during this last recording; its
percentage estimate is not a verified state-of-charge measurement.

For the failed runs, the manifest identifies the FC as unavailable with
`no ArduPilot heartbeat received`. Once startup times out, the inspector closes
that connection and continues camera-only recording. Thus the subsequent zero
telemetry count is not evidence of zero electrical UART activity throughout
the camera recording. `completed=true` means the available collectors closed
normally; it does not mean every sensor was available.

The following evidence narrows the investigation without establishing a cause:

- Saved post-update boot settings match the working configuration:
  `/dev/serial0` points to `ttyAMA0`, GPIO14/15 are TXD0/RXD0,
  `enable_uart=1` and `dtoverlay=disable-bt` remain enabled, there is no serial
  console, and serial gettys are inactive.
- A later receive-only comparison opened the ordinary pymavlink connection
  **before** opening direct pySerial. That first four-second sample already
  received 20,793 bytes, four heartbeats, 160 distance messages and 80 flow
  messages. Direct pySerial and the subsequent pymavlink reopen also worked.
  Reported termios settings matched across the comparison.
- pymavlink deliberately opens at 1200 baud before selecting the requested
  baud, whereas direct pySerial opens at 115200 immediately. This is a
  concrete implementation difference, but the successful first pymavlink
  stage prevents attributing recovery to direct pySerial alone. Time since
  reboot, reopen behavior and UART reconfiguration remain unseparated factors.
- The running FC's pinned source schedules heartbeat at 1 Hz independently of
  the zero stream-rate parameters and requires no first received message.
  The Pi is MAVLink instance 4 (`MAV4_OPTIONS=0`); the private instance 3 is the
  MT-15 UART3 link. `MAV_TELEM_DELAY=0`, serial passthrough is disabled, and
  UART4 has no RTS/CTS pins. No FC-side handshake explanation was identified.

Another reboot's first UART open received a disarmed FC heartbeat in 0.810 s,
before any workaround. The diagnostic recorded 4,087 parser bytes, five
initial parse errors, zero framing errors, one UART overrun and zero
transmitted bytes. Reopening the local serial descriptor also received a
heartbeat, but the initial success means this does not demonstrate recovery
from the failed condition. The issue is intermittent, rather than occurring
on every first open.

Do not describe the package upgrade, 1200-baud transition or a particular
serial reopen as the proven cause or fix.
The firmware and persistent FC parameters were not changed to investigate
these recording failures.

The recorder now logs received-byte, packet and parser-error counts if the
initial heartbeat wait times out on the Pi's `/dev/serial0` or `/dev/ttyAMA0`.
It reopens that local serial descriptor once at the requested baud and waits
again for a fresh heartbeat. This is not an FC reboot and sends no MAVLink
traffic. USB/network endpoints keep their prior startup behavior. Failed
recovery closes the connection and marks the FC unavailable; a recovered
armed heartbeat still aborts ordinary disarmed recording.

Final offline validation passed 666 tests, with two optional-dependency skips
and three SITL cases deselected. The 138 focused recorder/tag-servo tests
include successful recovery, repeated timeout, failed reopen, armed recovery,
cleanup and unchanged USB/network behavior. Ruff, type checking, all five
import contracts, dependency checks and the 18-page documentation build passed.
Earlier dependency validation also exercised OpenCV 5 with NumPy 2.5 and a
generated AprilTag; the Pi's native camera, libcamera and vision imports passed.

## Prepared ArduCopter 4.7.1 candidate

An isolated source checkout at
`dbe792162d06cab66c3475fd5556bf7a120f119e` produced the custom FlywooF745
candidate using the existing additional hardware definition. The APJ has board
ID 1027, runtime identity `dbe79216`, and an 865,632-byte image within the
950,272-byte limit. Its SHA-256 is
`f1f96750a8ee1d5c5a0db00ec0dc7951591b1fa370601c8dc8aea02fd1688603`.

Artifact verification passed, including exact APJ/BIN agreement, embedded
ROMFS, and all 410 extracted feature states matching the working build. The
candidate preserves MAVLink range/flow, EKF flow fusion and GUIDED_NOGPS;
proximity remains absent. Its isolated tests passed 84 focused controller
tests and all three SITL cases in 161.787 seconds: downward-only hover,
forward-range hover, and GCS-loss landing. These are simulator results, not
live flight or sensor calibration results.
These candidate simulator checks preceded the final recording-only UART retry;
they are not a test of that retry on physical hardware.

Candidate gate changes and the candidate firmware manifest remain separate
from the active 4.7.0 runtime. Flashing, coordinated gate migration and
post-flash parameter/ROMFS/sensor/reboot acceptance have not occurred at this
checkpoint. The migration review also identifies the two renamed position
controller jerk parameters; no blanket parameter restore has been performed.

## Evidence

Local evidence is retained under ignored
`artifacts/software-update-20260909/`; it is not part of a normal Git checkout:

- `apt-upgrade.log`, `apt-package-delta.json`, `packages-before.tsv`,
  `packages-after.tsv`, `uv-update.log`, `after-reboot.log` and
  `pi-post-upgrade-checks.log` record the system update.
- `final-system-checks.log` confirms no pending apt upgrades, no failed units,
  `get_throttled=0x0`, masked unattended apt timers, active eduroam/Tailscale,
  uv 0.12.11, Tailscale 1.102.3 and an inactive walkthrough service after the
  final capture.
- `pi-python-after.json`, `pi-runtime-verification.json` and `pi-deploy.log`
  record the Python environment and deployment provenance.
- `capture-status.log`, `capture-retry-result.log`, `cold-start-result.log`,
  `pi-uart-diagnosis.log` and `uart-open-comparison.log` retain the recording
  and UART observations; `uart-reset-comparison.log` includes the successful
  later cold read and UART error counters. Raw camera data and report output are retained with
  the corresponding datasets.
- `final-cold-launch.log`, `final-cold-result.log` and
  `update-check-20260909-final/` record the final successful first-start capture.
- `firmware/candidate-verification.json`, `firmware/migration-review.md`,
  `firmware/deployment-review.md` and `candidate-runtime/result.json` record
  the unflashed firmware and simulator checks.

Network credentials and private Pi configuration backups are not included in
this report.
