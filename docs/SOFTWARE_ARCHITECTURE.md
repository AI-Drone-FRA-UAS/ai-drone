# Development and architecture

## Boundaries

| Code | Responsibility |
| --- | --- |
| ArduPilot | Attitude control, EKF fusion, modes and vehicle failsafes |
| `ai_drone.mavlink` | One physical reader, bounded subscriptions, private Unix-socket clients and command ownership |
| `ai_drone.runtime`, `ai_drone.network` | Boot access service, fresh status, pure grounded Wi-Fi selection and NetworkManager adapter |
| `ai_drone.operator` | Authenticated operator presence and optional supervised SSH reverse route |
| `ai_drone.flight.controller` | Effect owner for guarded climb, bounded altitude hold, Loiter and LAND, plus explicit human ownership and flight logs |
| `ai_drone.flight.state`, `.phase`, `.limits`, `.params` | Frozen timestamped observations, pure state/cleanup decisions, envelope guards and reviewed parameter contracts |
| `ai_drone.capture`, `ai_drone.vision` | Camera/tag/telemetry workers, health, calibration and pose estimation |
| `ai_drone.settings`, `ai_drone.storage` | Frozen TOML defaults and storage decisions |
| `ai_drone.system`, `.runtime_status`, `.power_state` | Shared process mechanics and typed runtime/FC/Pi observations; operation-specific policy remains with callers |
| `ai_drone.mount` | Shared servo mapping, GPIO setup and exclusive actuator ownership |
| `ai_drone.link`, `ai_drone.cli.deploy` | SSH, portable staged deployment, maintenance and source/environment rollback |
| `ai_drone.config`, `ai_drone.transfer` | Verified FC snapshots and hashed dataset copies |
| `ai_drone.cli`, `scripts/` | Explicit user workflows and maintenance adapters |

Use small functions with explicit inputs for decisions. Keep clocks, files,
networking, serial and GPIO at the edges; classes own resources and lifetimes.
Keep hardware invariants and failure rationale in comments. Avoid compatibility
layers and speculative frameworks. Import contracts enforce shared-leaf, command,
capture and flight-core directions; they do not establish mathematical purity of
arbitrary third-party functions. The flight decoder depends on pymavlink message
representation and pure safety predicates. The controller owns clocks, stream
requests, firmware/parameter reads, heartbeat pumping, command acknowledgements
and human supervision. Its public methods delegate to the extracted decisions.

An observed armed heartbeat never acquires control. `ArmPending` records an arm
write before its result is known, and `Flight` begins before the first attempted
climb write. `Landing` retains the prior cleanup obligation; a failed write does
not convert an attempted flight to ground-only disarm. `CommandAttempt` records
intent, write outcome and error separately from received FC confirmation.
`Human` remains latched across disarm, stale RC and reconnect; those observations
do not return command ownership to autonomy.

Altitude values retain their datum: takeoff gain is relative to the fresh pad
reading, the 0.8 m ceiling is floor referenced, and raw downward range, local
altitude and aligned local altitude remain separate. A forward range reading is
never height evidence. `altitude-hold` uses bounded neutral climb targets in
`GUIDED_NOGPS`; it does not promise XY position hold. The production compass/flow
profile and the simulator-only inertial-yaw candidate have separate parameter
contracts. Neither mode entry nor green estimator flags qualifies heading drift
or hall performance.

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
mode. Human ownership remains latched for the controller's lifetime; the onboard
GCS heartbeat continues meanwhile. A separately initialized run must pass normal
fresh startup checks. SSH terminal lifetime is not control ownership.

Persistent NetworkManager autoconnect flags are disabled by the installer;
a root-owned manifest retains eligible saved-client UUIDs. The policy initiates
connections only with fresh disarmed FC status. It preserves working clients,
rotates failed alternatives and never automatically activates an AP. Manual
requests are queued in the access service, so SSH disconnect does not cancel them.
An uncertain activation or asynchronous disconnect retains the command exclusion
until NetworkManager confirms a terminal state. Unknown state cannot release it.
`Idle`, `Activating` and `Deactivating` describe outstanding external work;
`NetworkWork` separately records queued intent and maintenance. Maintenance can
block admission without erasing a pending subprocess outcome. The shared server
retains the lock that makes network exclusion and control acquisition atomic.

Transfer `Endpoint`, `Inventory` and `Receipt` values freeze parsed metadata.
Wire JSON remains version 1. Source deletion still compares the exact supplied
receipt with the source ledger, freshly hashes both copies, checks writers and
rechecks after quarantine. Normalization cannot make a tampered receipt match.
Report scalar definitions supply CSV columns and chart metadata together; range,
flow, IMU and aggregate battery parsing retain their special validity contracts.

## Checks

```bash
uv sync --locked --group dev --group docs --group raspi
uv run --locked --group dev --group docs ruff format --check .
uv run --locked --group dev --group docs ruff check .
uv run --locked --group dev --group docs ruff check . --select C901 --ignore-noqa
uv run --locked --group dev --group docs ty check .
uv run --locked --group dev --group docs lint-imports
uv run --locked --group dev --group docs deptry .
uv run --locked --group dev --group docs --group raspi pytest -q -m 'not sitl'
uv run --locked --group dev --group docs python site/build.py
git diff --check
```

Tests exercise observable behavior, concurrency and failure isolation. Loopback
HTTP/Unix-socket tests require permission to create local sockets. Pi packages
remain lazy; CI defines Python and laptop platform coverage. Commit dependency
changes with `uv.lock`. The development default is standard Python 3.14; 3.13
remains supported for the Pi. See [interpreter selection and migration](PYTHON_RUNTIME.md).
The development gpiozero dependency runs PWM tests with a mock pin factory;
install the raspi group for OpenCV host coverage. Required skipped tests remain
unverified and must not be reported as passing.

Ruff limits function complexity to 10, including tests. The separate `C901`
check ignores inline suppressions. Treat the score as a prompt to separate
responsibilities; it does not measure naming, side effects or overall clarity.

CI additionally enforces `BLE` and `ARG` on the extracted flight core, runtime
and power observations, tag decisions and transfer boundary. Existing effect
owners retain their explicit cleanup and first/secondary-error handling; their
broad catches need case-by-case review before extending this rule scope. A
September 21 candidate survey found 47 `BLE`, 38 `ARG`, 749 `TRY`, 67 `FBT` and
23 `PLR0913` diagnostics across source and tests. Most `ARG` reports concern
fixture/callback signatures; cleanup catch narrowing must not skip independent
finalizers. `TRY` is not enabled wholesale because moving hundreds of useful
exception messages into one-off classes obscures diagnostics. `FBT` would also
flag protocol booleans and test inputs; `PLR0913` does not justify hiding explicit
keyword-only options. These are readability/review signals, not defect counts.

Run the incremental boundary gate locally with:

```bash
uv run --locked --group dev ruff check ai_drone/flight/state.py \
  ai_drone/flight/phase.py ai_drone/flight/limits.py ai_drone/flight/params.py \
  ai_drone/runtime_status.py ai_drone/power_state.py \
  ai_drone/capture/operations.py ai_drone/transfer.py --select BLE,ARG
```

Flight changes also require the [exact pinned SITL gate](ARDUCOPTER_4_7_NOGPS_LOITER.md#exact-pinned-sitl-acceptance-gate):

```bash
ARDUPILOT_ROOT=/home/abaris/drone/ardupilot UV_CACHE_DIR=/tmp/uv-cache uv run --locked --group dev pytest -m sitl -vv -s
```

Retain fresh test evidence; simulation does not establish physical readiness.
The five original cases cover downward/forward range coexistence, GCS-loss landing,
shared recording through SSH hangup and operator-loss landing, and explicit
pilot handoff that preserves pilot control after operator loss.
The [answered questionnaire](CLEANUP_QUESTIONNAIRE.md) records the agreed scope.
The expanded [plan](../refactor.md#8-acceptance-matrix-and-verification) additionally
requires independent simulator truth for altitude/XY/yaw and profile fault cases;
retain failed traces, injection settings and exact artifact identities.

## Deploy and document

Review and test source before `uv run drone deploy`; verify deployed files and
start with disarmed checks. [Deployment](pi-networking.md#environment-and-deployment)
uses private staging, an idle maintenance lease and source/environment rollback.
Source deployment does not flash the FC or install
system services. See [access setup and rollback](pi-networking.md#shared-access-service).
`drone.toml` stays local. Preserve configuration in dated `state/`/`params/`
records and raw data in ignored `artifacts/`. Transfer receipts verify hashes;
source removal is a separate explicit operation that rechecks both copies.

`review/report.html` is loaded with `importlib.resources` and must be included
in both wheel and sdist. Verify an installed wheel from a separate working
directory, exercise `drone --help`, and generate a report with escaping and
relative asset references. A source checkout passing tests does not establish
that its package includes every resource. Source deployment's own staged bundle
also requires the template and recovery scripts; a wheel is not a Pi restoration
archive. Preserve source, environment, configuration, service and parameter
snapshots as distinct restoration inputs.

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
service checks. Current task evidence belongs in its dated implementation report,
with host tests, simulator output and Pi measurements identified separately.

Validate camera focus/calibration and floor-tag poses over the allowed operating
envelope using measured tag geometry. The 0.5 m initial takeoff gain, 0.6 m CLI
target cap and 0.8 m floor ceiling remain; 1–3 m flights need a later review.
RPM/ESC channels remain unavailable until
the actual FC reports them. Resolve compass/pre-arm faults and retain the
hover path's current safety gates, including its zero-RC-channel startup rule.

The IMX500 supplies images; AprilTags run on the Pi CPU. Tag centering, payload
approach, autonomous room search and forward-lidar obstacle avoidance are not
implemented control modes. Downward flow/range and one forward beam do not form
a complete obstacle map. Their physical prerequisites and supervised airborne
validation remain separate work.
