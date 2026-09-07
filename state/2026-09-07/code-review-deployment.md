# Deployment, configuration, networking, power and firmware review

Review date: 2026-09-07. Repository: `/home/abaris/drone/ai-drone`.
Scope: static/read-only inspection plus local tests. No network, serial, GPIO,
hardware or system configuration calls were made. No repository source changed.
The Syeed reference capture was excluded entirely.

## Architecture and strengths

- `pyproject.toml:15` declares eight operator commands. The runtime is Python
  3.11–3.13; Pi-specific OpenCV is an explicit `raspi` group (`pyproject.toml:32`).
  Picamera2, gpiozero and native AprilTag are installed outside PyPI via the Pi's
  system packages (`pyproject.toml:78`). Deployment enables system site packages
  and synchronizes the locked dependency set (`ai_drone/link/deploy.py:336`).
- `ai_drone/config/snapshot.py:144` requests the complete indexed MAVLink
  parameter list, filters messages to the selected system/component, rejects
  armed heartbeats, and re-requests missing indexes. Snapshot serialization
  validates field types, finite values, names, unique/contiguous indexes and
  advertised totals (`ai_drone/config/snapshot.py:77`). It creates deterministic
  parameter text and SHA-256 evidence. These requests do not set parameters.
- `ai_drone/cli/config_export.py:115` requires a fresh final disarmed heartbeat
  before certifying the snapshot. `ai_drone/config/sync.py:77` validates the
  remote JSON and SHA-256 before writing parameter/metadata files with atomic
  writes (`ai_drone/config/sync.py:170`).
- `ai_drone/link/connect.py:173` tries Tailscale SSH, saved hotspot Wi-Fi, then
  USB gadget Ethernet. USB configuration requires an explicitly verified
  interface (`ai_drone/link/usb_ssh.py:282`), validates addresses/subnet, and does
  not silently reconfigure an auto-detected adapter.
- Deployment has substantial deletion protections: dedicated remote paths,
  realpath/owner checks, a runtime allowlist, symlink rejection and preservation
  of `.venv`, artifacts and `.env` state. Tar cleanup validates a per-upload
  manifest/sentinel before deleting stale source (`ai_drone/link/deploy.py:475`).
  Both tar and rsync transports have real local behavior tests.
- `scripts/ai-drone-network:64` implements preferred/client/hotspot selection;
  network switches run in independent transient systemd services so loss of SSH
  does not by itself kill the switch (`scripts/ai-drone-network:109`).
- `scripts/setup-pi-power-resilience.sh:128` masks unattended upgrade timers,
  enables bounded persistent journaling, pins the watchdog, sets filesystem
  error behavior, and installs an offline interrupted-upgrade recovery script.
  `scripts/pi-safe-upgrade.sh:112` arms recovery before touching packages and
  only marks completion after consistency checks.
- Firmware is a pinned custom ArduCopter build, not an implementation of the
  autopilot in this repository. The overlay preserves the captured project
  matrix and enables optical-flow EKF3 fusion and GUIDED_NOGPS. The verifier
  checks exact hashes, board/commit identity, APJ/BIN equality, flash limits,
  runtime banner and linked feature evidence (`scripts/verify_ardupilot_firmware.py:463`).
  The checked-in overlay hash matches the manifest:
  `21d270a4f0f0da12c8c5cfa5d14c4b305e03d72741736c4a47aa10363529d7ae`.

## Concrete findings

### 1. The autostart safety check misses the new actuating recorder

Priority: medium; confirmed by evaluating the actual pattern locally.

`scripts/setup-pi-power-resilience.sh:272` inspects enabled unit **names** using
`drone-(control|motor|servo|inspect|picam)|mavlink`. An enabled unit called
`drone-tag-servo-record.service` is not matched. That command is an active GPIO
actuator entry point (`pyproject.toml:23`) and the setup script nonetheless
prints "No vehicle-control unit is enabled at boot". An arbitrarily named unit
with a dangerous `ExecStart` is also invisible. The existing test only asserts
that the old regex string is present (`tests/test_power_resilience_scripts.py:71`).

This does not show that any such service is currently installed or enabled;
it shows that the script cannot establish the safety property it claims.
Fix: include the active recorder and inspect enabled unit command contents,
including relevant user units, or narrow the claim to the explicitly audited
unit list. Add behavioral tests with representative enabled unit definitions.

### 2. Deployment and config sync select the host SSH config that connection bypasses

Priority: medium usability/reliability; confirmed on this host without SSH.

`resolve_connection_target()` uses `_direct_ssh_config()` and defaults to
`/dev/null` (`ai_drone/link/targets.py:198`), matching AGENTS.md's workaround.
`resolve_deploy_target()` instead uses `_default_ssh_config()`
(`ai_drone/link/targets.py:179`), which selects any existing `~/.ssh/config`
(`ai_drone/link/targets.py:54`). On this checkout the outputs are:

```text
connect default config= /dev/null
deploy default config= /home/abaris/.ssh/config
```

Consequently `drone-connect` can work while `drone-deploy` and
`drone-config-sync` fail with the same documented bad-config permissions error.
Explicit `SSH_CONFIG=/dev/null` is the present workaround. Unify the direct Pi
default and preserve an explicit opt-in custom config.

### 3. The advertised network selector cannot be reconstructed by normal deployment

Priority: medium deployment/recovery gap; confirmed using the real allowlist.

`scripts/ai-drone-network` and `scripts/ai-drone-network.service` both exist but
are absent from `RUNTIME_FILES` (`ai_drone/link/deploy.py:36`); only the four
setup/upgrade scripts are included. They are also not protected remote state,
so an existing copy below the project directory is treated as stale source by
rsync/tar cleanup (`ai_drone/link/deploy.py:329`, `ai_drone/link/deploy.py:559`).
No checked-in installer installs/enables the selector/service. Meanwhile the
operator documentation assumes `ai-drone-network` exists (`docs/pi-networking.md:3`).

An existing `/usr/local/sbin/ai-drone-network` installation is unaffected by
project cleanup. The gap concerns a fresh/recovered Pi and distributing later
updates to this helper. Add the two files to deployment and an explicit,
idempotent reviewed install procedure.

### 4. Dual-network setup is not preserved by the boot selector

Priority: medium; deterministic conflict between the two scripts.

`scripts/setup-pi-dual-network.sh:122` configures the hotspot on wlan0 and a
cloned uplink on wlan1. It does not alter `ai-drone-network.service`.
That enabled boot service invokes `--worker auto`
(`scripts/ai-drone-network.service:8`), whose first operations bring the
hotspot down and try `Xyz` on wlan0 (`scripts/ai-drone-network:67`). When Xyz is
reachable, the selector returns successfully without restoring the hotspot.
Thus the "keep hotspot on wlan0" topology works immediately after setup but
need not survive reboot. Add an explicit persistent dual-interface policy to
the selector or adjust/disable that selector during the dual-network setup.

The setup script also accepts `--uplink-profile` equal to `--source-profile`,
so the later profile modification can alter the source despite claiming it
remains unchanged (`scripts/setup-pi-dual-network.sh:86`, `:115`). Reject profile
name collisions during preflight.

### 5. Snapshot publication can include unrelated work staged during capture

Priority: medium repository-integrity edge case; source-level confirmation.

`--publish` checks the tree is clean at the beginning (`ai_drone/config/sync.py:239`).
It may then spend minutes deploying/installing/downloading. Although
`publish_snapshot()` stages and checks only its two output paths, it ultimately
runs unrestricted `git commit -m ...` (`ai_drone/config/sync.py:199`). Files that
another process/person stages during capture enter that commit too, violating
the explicit "exactly the generated files" contract. It then pushes the branch.
Use a path-scoped commit/isolated index or revalidate the index immediately
before committing, with a regression test for unrelated staged work.

## Additional limitations and test gaps

- Deployment overwrites the active project tree and then may clear/update its
  environment (`ai_drone/link/deploy.py:610`, `:621`, `:336`). There is no active
  recorder/controller exclusion or disarmed check before this operation. A
  interrupted upload/install can leave mixed versions, and an existing process
  may import files/dependencies while they change. Treat deployment as a
  maintenance-only operation; staged releases, installation validation and a
  controlled switch would make this recoverable. `drone-config-sync` also does
  this by default (`ai_drone/config/sync.py:242`); use `--no-sync` for live
  read-only inspection with already-installed code.
- The package upgrade preflight is fail-open when `fuser` is absent or
  `/dev/serial0` is absent, and it checks no other link endpoint
  (`scripts/pi-safe-upgrade.sh:54`). It still prints "No process is holding the
  flight-controller link". `vcgencmd` absence or an empty error result also
  allows progression (`scripts/pi-safe-upgrade.sh:64`). These conditions need
  explicit unknown/refusal handling if this script is meant to prove readiness.
- A lost initial `PARAM_REQUEST_LIST` is never retried: missing-index retries
  only begin after at least one parameter supplies `expected`
  (`ai_drone/config/snapshot.py:163`, `:210`). A link can be alive but the export
  unnecessarily waits for the full timeout. The downloader test covers only a
  happy two-message download (`tests/test_config_snapshot.py:147`); add dropped
  initial request, partial responses, changing counts, foreign/armed heartbeat
  and timeout behavior tests.
- Two separate atomic snapshot writes do not form a cross-file transaction
  (`ai_drone/config/sync.py:170`). A power/storage failure between them leaves a
  new parameter file beside older metadata. Per-file SHA-256 makes such a
  mismatch detectable, but consumers need to verify it.
- `scripts/ai-drone-network` and its boot service currently have no behavioral
  tests. Shell power tests mostly assert text snippets and syntax rather than
  exercise mock command failure paths; the missed recorder demonstrates the
  practical limitation (`tests/test_power_resilience_scripts.py:54`).
- No checked-in `.github` workflow or other CI runner was found. Development
  checks are operator commands documented at `README.md:104`; local pytest,
  Ruff, ty, dependency and import-layer configuration is present.
- The firmware build recipe explicitly depends on a mutable local container
  tag whose creation recipe is not committed (`firmware/README.md:79`). Exact
  artifact verification is good, but another machine cannot reproduce the
  build environment solely from this repository. The documented
  `/home/abaris/ardupilot` ELF/BIN/APJ/hw.dat/extractor paths are absent on this
  machine, so this review verified the overlay/hash and verifier tests, not an
  actual compiled build or the live controller firmware image.

## Validation performed

Executed locally, without network/hardware access:

```text
.venv/bin/python -m pytest -q \
  tests/test_config_snapshot.py tests/test_deploy.py \
  tests/test_cross_platform_scripts.py tests/test_power_resilience_scripts.py \
  tests/test_hotspot_script.py tests/test_verify_ardupilot_firmware.py

194 passed in 0.38s
```

Also evaluated runtime allowlist membership, SSH default resolution, the
autostart regex, and the checked-in overlay SHA-256. No production edits were
made; SITL and live hardware verification belong to the other review tasks.
