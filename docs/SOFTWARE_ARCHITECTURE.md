# Development and architecture

## Boundaries

| Code | Responsibility |
| --- | --- |
| ArduPilot | Attitude control, EKF fusion, modes and vehicle failsafes |
| `ai_drone.mavlink` | One physical reader, bounded subscriptions, private Unix-socket clients and command ownership |
| `ai_drone.runtime`, `ai_drone.network` | Boot access service, fresh status, pure grounded Wi-Fi selection and NetworkManager adapter |
| `ai_drone.operator` | Authenticated operator presence and optional supervised SSH reverse route |
| `ai_drone.flight` | Guarded climb/Loiter/LAND, explicit human ownership and isolated flight logs |
| `ai_drone.capture`, `ai_drone.vision` | Camera/tag/telemetry workers, health, calibration and pose estimation |
| `ai_drone.settings`, `ai_drone.storage` | Frozen TOML defaults and storage decisions |
| `ai_drone.mount` | Shared servo mapping, GPIO setup and exclusive actuator ownership |
| `ai_drone.link`, `ai_drone.cli.deploy` | SSH, portable staged deployment, maintenance and source/environment rollback |
| `ai_drone.config`, `ai_drone.transfer` | Verified FC snapshots and hashed dataset copies |
| `ai_drone.cli`, `scripts/` | Explicit user workflows and maintenance adapters |

Use small functions with explicit inputs for decisions. Keep clocks, files,
networking, serial and GPIO at the edges; classes own resources and lifetimes.
Keep hardware invariants and failure rationale in comments. Avoid compatibility
layers and speculative frameworks.

## Ownership and failures

The implemented Pi boot service owns the FC UART and sends no autonomous flight
commands. [Installation remains pending](pi-networking.md#deployment-status).
Consumers receive independent bounded queues with original receipt timestamps;
slow consumers and failed log sinks cannot block the physical reader or refresh
stale data. The local socket and status directory are private. A command lease
excludes concurrent controllers and network maintenance. A camera and servo each
retain one owner; separate checkouts never isolate hardware.

Detached recording and detached control share FC telemetry. Flight recording
has its own endpoint, event queue and writer lifetime. Startup logging failure
blocks a control run; later disk/queue failure marks the recording incomplete
and preserves control. Capture stops video before consuming its log reserve;
its final storage floor closes the dataset.

Operator presence uses fresh authenticated challenges to configured numeric
Tailscale/LAN addresses or the responder's supervised SSH reverse route. Any
healthy route suffices. Autonomous loss requests
LAND; an explicit handoff additionally requires fresh RC channels and a pilot
mode. Human ownership lasts until disarm and a new run; the onboard GCS heartbeat
continues meanwhile. SSH terminal lifetime is not control ownership.

Persistent NetworkManager autoconnect flags are disabled by the installer;
a root-owned manifest retains eligible saved-client UUIDs. The policy initiates
connections only with fresh disarmed FC status. It preserves working clients,
rotates failed alternatives and never automatically activates an AP. Manual
requests are queued in the access service, so SSH disconnect does not cancel them.
An uncertain activation or asynchronous disconnect retains the command exclusion
until NetworkManager confirms a terminal state. Unknown state cannot release it.

## Checks

```bash
uv sync --locked --group dev --group docs
uv run --locked --group dev --group docs ruff format --check .
uv run --locked --group dev --group docs ruff check .
uv run --locked --group dev --group docs ruff check . --select C901 --ignore-noqa
uv run --locked --group dev --group docs ty check .
uv run --locked --group dev --group docs lint-imports
uv run --locked --group dev --group docs deptry .
uv run --locked --group dev --group docs pytest -q -m 'not sitl'
uv run --locked --group dev --group docs python site/build.py
git diff --check
```

Tests exercise observable behavior, concurrency and failure isolation. Loopback
HTTP/Unix-socket tests require permission to create local sockets. Pi packages
remain lazy; CI defines Python and laptop platform coverage. Commit dependency
changes with `uv.lock`.

Ruff limits function complexity to 10, including tests. The separate `C901`
check ignores inline suppressions. Treat the score as a prompt to separate
responsibilities; it does not measure naming, side effects or overall clarity.

Flight changes also require the [exact pinned SITL gate](ARDUCOPTER_4_7_NOGPS_LOITER.md#exact-pinned-sitl-acceptance-gate):

```bash
ARDUPILOT_ROOT=/home/abaris/drone/ardupilot UV_CACHE_DIR=/tmp/uv-cache uv run --locked --group dev pytest -m sitl -vv -s
```

Retain fresh test evidence; simulation does not establish physical readiness.
The five cases cover downward/forward range coexistence, GCS-loss landing,
shared recording through SSH hangup and operator-loss landing, and explicit
pilot handoff that preserves pilot control after operator loss.
The [answered questionnaire](CLEANUP_QUESTIONNAIRE.md) records the agreed scope.

## Deploy and document

Review and test source before `uv run drone deploy`; verify deployed files and
start with disarmed checks. [Deployment](pi-networking.md#environment-and-deployment)
uses private staging, an idle maintenance lease and source/environment rollback.
Source deployment does not flash the FC or install
system services. See [access setup and rollback](pi-networking.md#shared-access-service).
`drone.toml` stays local. Preserve configuration in dated `state/`/`params/`
records and raw data in ignored `artifacts/`. Transfer receipts verify hashes;
source removal is a separate explicit operation that rechecks both copies.

`site/build.py` renders maintained Markdown plus four presentation pages in
`site/content/`; preview with `uv run --group docs python site/build.py --serve`.
Output is ignored `site/_build/`, served on `http://127.0.0.1:8000/`. The Pages
workflow publishes relevant `main` changes. Edit source Markdown and poster
assets rather than generated HTML; keep historical results clearly dated.

## Remaining stages

When the Pi is available, validate staged deployment and service installation,
then operator reachability, shared checks/recording and restart/rollback behavior.
Test explicit hotspot on/off and reboot-to-client recovery with a verified alternate
access route. September 16 staging captures do not substitute for these installed
service checks; no new hardware or simulator runs are part of the local-only work.

Validate camera focus/calibration and floor-tag poses at 1–3 m; start with
measured 22.4 cm A3 `tag36h11` prints. RPM/ESC channels remain unavailable until
the actual FC reports them. Resolve compass/pre-arm faults and retain the
hover path's current safety gates, including its zero-RC-channel startup rule.

The IMX500 supplies images; AprilTags run on the Pi CPU. Tag centering, payload
approach, autonomous room search and forward-lidar obstacle avoidance are not
implemented control modes. Downward flow/range and one forward beam do not form
a complete obstacle map. Their physical prerequisites and supervised airborne
validation remain separate work.
