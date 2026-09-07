# Repository, Pi deployment and networking consolidation

Maintenance on 7 September 2026 consolidated the maintained software onto
`main`, restored Pi connectivity using its existing eduroam credentials and
Tailscale identity, and installed the consolidated runtime. The subsequent
[team-access follow-up](team-access.md) completed the central Tailscale policy
change and verified eduroam preference without an access-point lock.

This is a software/network maintenance record. The separate
[indoor repair report](indoor-repair.md) remains the hardware reference.

## Git history and recovery

The previously uncommitted hardware and correctness repairs were first saved
as commit `fc8e2f0e4b555ce055f9859aac52f94c66cc2a51`. Cleanup was committed as
`ce72a89`, then merge `8415cc07ffb6272da06bad43c8818bd283a24567` joined the
maintained runtime with the eleven unique commits from the former `main`.
The project site, poster and dated incident evidence are preserved. PR #7
became merged when its history became part of `main`.

Every previous branch tip is recoverable through a published annotated tag
under `archive/consolidation-2026-09-07/`: `current25`, `experimental`,
`experimental-berserkan`, `experimental-sebispm`,
`preflight-and-nogps-takeoff`, `exp-main-sync`, `local-main`, and `main`.
Experimental implementations that were superseded were retained as history;
they were not installed as competing Pi runtimes.

After CI passed, the five obsolete origin branches were removed in one atomic
push with an explicit expected SHA for every head. Their tips still matched
the original inventory and archive tags. Only `main` remains locally and at
origin. The clean temporary integration worktree and obsolete local branches
were removed; the pre-existing local recovery stash was retained.

A full local recovery bundle also preserves refs and the pre-existing stash:
`artifacts/consolidation/20260907/repository-before.bundle`, SHA-256
`bdb5f1ba3f535f70e352d069d9885ef42ef96836cc18d9da5408ba8ffc2d33a7`.
The adjacent `working-tree-before.patch`, `untracked-before.tar.gz` and
`backup-manifest.json` preserve the original dirty checkout. These large
local artifacts are ignored by Git; archive tags are available on origin.

Use `main` for the shared baseline, short-lived feature/fix branches and
reviewed pull requests as described in [CONTRIBUTING.md](../../CONTRIBUTING.md).

## Code and documentation

- Recorder state, background workers and reporting now live in
  `ai_drone/capture/`. The recorder CLI shrank from 1,884 to 1,348 lines;
  extraction preserved all 32 moved function/class bodies. Compatibility
  imports remain, and the capture package cannot import the CLI.
- Maintained procedures have one documentation map, `docs/index.md`.
  `CURRENT.MD` was preserved as a dated historical note; obsolete sensor
  recipes and duplicate firmware instructions now refer to canonical guides.
  Old flight incident files have a checksum-verified archive with provenance.
- CI checks locked dependencies, formatting, lint, typing, five import
  contracts, dependency declarations and offline tests on Python 3.11–3.13.
  The site builds from the same maintained documentation.
- Connection helpers use the Pi's full Tailscale DNS name so shared teammates
  can resolve the destination. Explicit host overrides remain supported.
- All maintained FC connections initialize the MAVLink 2 ArduPilot decoder
  through one lazy factory. A synthetic 100-byte stream reproduces a startup
  crash when the default parser caches a V1 `RAW_IMU` before switching to V2.
  The fix avoids that switch and retains V1 frame support; tests also verify
  V2-only message decoding and caller source IDs. This separately reproduced
  exception does not explain every earlier UART startup failure.
- CI exposed two tests that read a completion counter before the worker had
  finished syncing its event log and set its stop event. Commit `875cfa5`
  changes those tests to await that event. Injecting a 50 ms completion-log
  delay reproduced both old failures; both corrected tests pass. Runtime
  behavior and actuator-order assertions are unchanged.

A language rewrite is not justified by this review. Python suits companion
orchestration, with native camera, AprilTag and OpenCV processing and
ArduPilot's C++ flight controller. Clear ownership, bounded I/O and testable
module boundaries address the observed problems. Room navigation and
calibrated tag approach remain future features, not capabilities gained from
this cleanup.

## Pi runtime cleanup and deployment

`/home/seb/ai-drone` is the single active deployment. The unused
`/home/seb/ai-drone-live` clone, two standalone flight prototypes, old output
files and 27 orphan bytecode files were moved into
`/home/seb/ai-drone-archive/2026-09-07/`. The archive contains 134 regular files,
7,257,473 bytes, with matching content hashes and metadata. Enabled system and
user units, cron and shell startup were checked for references before moving
them. No flight-control service was enabled or started.

The archive manifest is retained locally as
`artifacts/consolidation/20260907/pi-legacy-archive-manifest.json`, SHA-256
`a12c4dc8cf472b8fb9463cc1d75899af8703a68b12863a869a549ff8edb7a4a2`.
The Pi virtual environment and recordings remain in place. The previous
runtime was separately backed up before deployment.

Final runtime revision `f46f353797de43fb6fa3b61e424d6b0f21bb0149` was exported
from its committed Git blobs and deployed with a locked offline install. All
40 Python source files and the complete source-file set match the tested
revision. Compilation, CLI help, mock GPIO pulse mapping and a synthetic
calibrated pose using the Pi's OpenCV 4.11.0 all passed. Mock GPIO checks did
not drive the physical servo. Final deployment, source hashes and capture
evidence are under `artifacts/consolidation/20260907/pi-final/`.

At **18:58 CEST**, an 8.00-second disarmed capture on the Pi recorded:

| Observation | Result |
| --- | --- |
| IMX500 camera | 161 analyzed frames, 240 encoded frames |
| Pi-to-FC UART | 1,109 messages over `/dev/serial0` at 115200 baud |
| Downward MTF-01P | 159 range samples and 159 flow samples, about 19.87 Hz |
| Downward range / flow quality | 0.02 m at the bench position; quality 46–68 |
| Forward MT-15 | No FC range samples; the physical receive path remains unverified |
| Battery telemetry | 14.787–14.789 V; FC reports 24%, capacity calibration not established |
| AprilTags | No physical tag detected in the scene |
| Physical servo | No pulse sent; mechanical operation remains unverified |

The FC remained disarmed. Earlier same-session checks reported the previously
known compass magnetic-field failure and a GCS failsafe warning; the final
capture did not repeat the complete pre-arm gate. This recording establishes
data acquisition, not flight readiness or calibrated sensor performance.
No real arm, disarm, mode, motor/throttle, RC override, servo or mission-start
command was sent during this maintenance.

## Networking

The saved eduroam profile still contained usable credentials and institution
certificate validation. It had autoconnect disabled and was pinned to an
unavailable access-point BSSID. Enabling autoconnect, setting priority 100 and
clearing that stale BSSID restriction restored the connection without copying
the laptop's profile. The installed boot selector now prefers `eduroam`, then
other saved client profiles, then the existing `AI-Drone-Zero` hotspot.
The later [network preference verification](team-access.md#eduroam-preference)
raised eduroam's priority to 400 after finding another saved client at 300.

The USB Ethernet gadget had a stalled transmit path. A module reload initially
recovered it, but the second Pi reboot reproduced the host's `NETDEV WATCHDOG`
errors with correct addresses and carrier at both ends. The final boot change
removes only `g_ether` from the early kernel command-line module list, keeping
`dwc2`. The existing `usb0-static.service` now loads the gadget after
`network.target`, then applies the same USB address before SSH. It does not
depend on successful Wi-Fi association or internet access. The observed device
and host MACs are retained in local root-only module options; distribution USB
vendor/product options remain intact. See the
[maintained service and recovery procedure](../../docs/RPI_ZERO2W_USB_SSH_SETUP.md#pi-startup-configuration).

The three original configuration paths and rollback metadata are backed up in
`/root/ai-drone-maintenance-20260907/usb-gadget-64uuu0p0/`. Systemd validation
passed. After the one initial gadget reload, **two normal Pi reboots** restored
both USB and full-name Tailscale SSH in **38–41 seconds**, without another
reload. Both retained the configured MACs and USB address, rejoined eduroam,
passed USB ping, and delivered fresh disarmed FC heartbeats at 115200 baud.
The first boot reached `network.target` at 22.619 seconds and started the USB
service at 22.703 seconds. Evidence is under
`artifacts/consolidation/20260907/usb-boot-fix/`. The host remained on eduroam.

With DNS/internet restored, restarting `tailscaled` recovered the existing
node without logging it out or replacing its identity. It reported `Running`
with no health warnings at `100.84.84.2`, named `seb-is-pm`, tagged
`tag:pi-drone`. Both access paths were verified:

```bash
ssh -F /dev/null seb@192.168.7.2
ssh -F /dev/null seb@seb-is-pm.tail59e6a4.ts.net
```

The earlier UART inspections also saw intermittent decoding failures after
reboot. Later default/v2/default/v2 comparisons all decoded 560–561 FC messages
and four disarmed heartbeats per four-second capture without persistent UART
changes. That intermittent failure's cause remains unresolved; it must not be
conflated with the separately reproduced and fixed parser-switch exception.
The final software and live-capture validation use the new connection
factory. Eduroam, DNS, NTP synchronization, Tailscale identity/health, authorized
SSH fingerprints and effective access rules persisted across the boot checks.

Credential-bearing NetworkManager profiles and Tailscale node state were
backed up only on the Pi in root-only
`/root/ai-drone-maintenance-20260907/`; they were not copied into Git.

### Shared-access policy follow-up

The initial inspection found TCP 22 allowed only from the owner's devices.
After the user identified the administrator credential in GNOME Keyring, the
[19:15–19:18 CEST follow-up](team-access.md) added and verified SSH access for
all three accepted drone-share recipients. Existing policy and the Pi's three
authorized OpenSSH keys were retained. Tailscale SSH remains disabled; access
uses ordinary OpenSSH over Tailscale.

The exact short command works on the maintenance laptop. Teammates need the
[documented SSH alias](../../docs/pi-networking.md#shared-teammate-ssh-access)
on their own computers; their individual SSH logins were not tested.

## Validation evidence

The final startup-fix revision passed **541 tests** on each supported Python
version, with two optional skips and two separately selected simulator tests.
Formatting, Ruff, ty, all five import
contracts, dependency checks and whitespace checks passed. The site built
18 pages and its GitHub Pages deployment succeeded.

Both final isolated simulator acceptance tests passed in 108.58 seconds: normal
GPS-free Loiter/landing and GCS-loss recovery. Maximum simulated altitude was
0.53 m and maximum Loiter horizontal drift was 0.028 m. Simulation ran in a
network namespace containing only loopback, with no route to physical
hardware. Runtime hashes before and after the tests match.

[GitHub Checks passed on `f46f353`](https://github.com/AI-Drone-FRA-UAS/ai-drone/actions/runs/34144960761)
on Python 3.11, 3.12 and 3.13: each ran 541 passing tests, with two optional
skips and two simulator tests deselected. The
[Pages build and deployment](https://github.com/AI-Drone-FRA-UAS/ai-drone/actions/runs/34144960769)
passed on the same revision.
Detailed logs, source hashes, deployment/capture results, branch inventory,
archive manifests and network observations are under the ignored local
`artifacts/consolidation/20260907/` directory. The source extraction record is
also committed as [capture-refactor-validation.json](capture-refactor-validation.json).
