# Contributing

`main` is the shared development and deployment branch. Start each change from
an updated `main`, use a short-lived `feature/<topic>` or `fix/<topic>` branch,
and open a pull request back to `main`. Keep each PR focused, explain the changed
behavior and validation, and merge after review and successful checks. Delete
the feature branch after merging.

The previous development branches are preserved as
`archive/consolidation-2026-09-07/*` tags. They are historical snapshots; start
new work from `main`. Retired implementations live in `attic/`, outside the
maintained checks and Pi deployment.

Read [CLAUDE.md](CLAUDE.md) for architecture and [AGENTS.md](AGENTS.md) for
hardware access and data provenance. Keep Pi hardware imports lazy and put
regressions behind offline test boundaries. Record hardware observations in
dated `state/` notes and keep bulky raw captures under ignored `artifacts/`.

Use uv and the Python version in `.python-version`. CI checks every supported
minor version, Python 3.11, 3.12, and 3.13, on Ubuntu. From the repository root:

```bash
uv sync --locked --group dev --group docs
uv run --locked --group dev --group docs ruff format --check .
uv run --locked --group dev --group docs ruff check .
uv run --locked --group dev --group docs ty check .
uv run --locked --group dev --group docs lint-imports
uv run --locked --group dev --group docs deptry .
uv run --locked --group dev --group docs pytest -q -m "not sitl"
uv run --locked --group dev --group docs python site/build.py
git diff --check
```

Commit intentional dependency changes together with the updated `uv.lock`.
The [Checks workflow](.github/workflows/checks.yml) uses locked dependencies and
runs without a Pi or flight controller. SITL is a separate acceptance check
requiring the [pinned ArduPilot setup](docs/ARDUCOPTER_4_7_NOGPS_LOITER.md#exact-pinned-sitl-acceptance-gate).
Hardware tests follow the relevant procedure in the [documentation map](docs/index.md).

Deploy from a clean, updated `main` after the checks for that exact commit
have passed. Before deployment, confirm `git status --short` is empty and
`git rev-parse HEAD` matches `git rev-parse origin/main` after fetching origin.
Record that commit in the deployment notes. With the Pi reachable, this command
synchronizes the runtime and its dependencies without starting a drone task:

```bash
PI_HOST=seb@seb-is-pm SSH_CONFIG=/dev/null uv run --locked drone-deploy
```

Use the appropriate host from [Pi networking](docs/pi-networking.md). Keep
deployment separate from optional `--run` tests and from system-service setup.
Verify the deployed source against the selected commit before testing; firmware
and flight-controller configuration use their own reviewed procedures.
