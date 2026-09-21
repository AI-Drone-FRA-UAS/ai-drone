# AI Drone

Python tools for a Raspberry Pi Zero 2 W, FlywooF745 ArduPilot controller,
IMX500 camera, MTF-01P optical flow/range sensor, forward MT-15 lidar and payload
servo. Record sensors and AprilTags, inspect configuration, and test a guarded
takeoff, bounded altitude hold, Loiter and landing. Autonomous tag approach and obstacle avoidance
remain development work.

## Start

```bash
uv sync --locked
uv run drone --help
ssh seb@seb-is-pm
# After deploying this checkout and installing its access service, on the Pi:
cd ~/ai-drone
uv run drone runtime status
uv run --locked --group raspi drone record --duration 30
```

Copy [drone.example.toml](drone.example.toml) to `drone.toml` on each machine
for local connection, recording, operator and runtime defaults. This file is ignored
by Git and preserved by deployment. [Configuration and precedence](docs/OPERATIONS.md#local-settings)
also cover `AI_DRONE_CONFIG` and `drone --config PATH`.

Development uses standard Python 3.14; the Pi keeps its working 3.13 environment
until native bindings and disarmed workload checks qualify a candidate. See
[Python environments and rollback](docs/PYTHON_RUNTIME.md). Flight remains
separate qualification work within the existing height limits.

The new source and boot service were **not installed** at the last Pi check on
September 16, 2026. See [deployment status](docs/pi-networking.md#deployment-status)
before using the Pi commands in this guide.

Once installed, the [Pi access service](docs/pi-networking.md#shared-access-service) owns the FC
connection and supplies telemetry to recording, checks and control together.
It joins eligible saved Wi-Fi while disarmed; its hotspot requires
an explicit command. Recording and flight tasks start only when requested.
Use `drone walk --allow-flight` for detached passive flight recording and
[independent operator presence](docs/OPERATIONS.md#operator-heartbeat) for
autonomous control. Camera recording and tag-triggered servo use remain optional.

| Guide | Contents |
| --- | --- |
| [Setup and networking](docs/pi-networking.md) | Wi-Fi, manual hotspot, USB recovery, deployment and Pi maintenance |
| [Operation](docs/OPERATIONS.md) | Checks, detached recording, storage limits, verified transfer and actuator tests |
| [Development](docs/SOFTWARE_ARCHITECTURE.md) | Resource ownership, checks, simulator and hardware limits |
| [Hardware](docs/drone-project.md) | Inventory, [wiring/configuration](docs/DRONE_CONFIGURATION.md), [frame/prints](docs/FRAME_AND_3D_PRINTS.md) |
| [Firmware](firmware/README.md) | Custom build and [exact simulator acceptance](docs/ARDUCOPTER_4_7_NOGPS_LOITER.md#exact-pinned-sitl-acceptance-gate) |

The [project site](https://ai-drone-fra-uas.github.io/ai-drone/) is generated from
these guides and retains the [historical poster](docs/poster/README.md).
Observations and restoration evidence live in [state/](state/), [params/](params/)
and [notes/archive/](notes/archive/README.md), including the
[startup tone](state/2026-09-09/startup-tone-articulation.md). Raw recordings in
ignored `artifacts/` are local files, separate from Git. Simulation and old
captures do not establish present flight readiness.
