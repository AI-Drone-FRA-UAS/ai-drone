# Python development and Pi runtime migration

The development default is standard, GIL-enabled CPython 3.14. CI also checks
3.11, 3.12 and 3.13; source syntax remains compatible with 3.11. Use uv and the
committed lock for every environment. For an isolated compatibility check:

```sh
UV_CACHE_DIR=/tmp/uv-cache UV_PROJECT_ENVIRONMENT=/tmp/ai-drone-py314 \
  uv sync --locked --python 3.14 --group dev --group docs --group raspi
UV_CACHE_DIR=/tmp/uv-cache UV_PROJECT_ENVIRONMENT=/tmp/ai-drone-py314 \
  uv run --locked --python 3.14 --group dev --group docs --group raspi \
  pytest -q -m 'not sitl'
```

Repeat with a separate path and `--python 3.13` to verify compatibility. The
five production SITL gates and expanded qualification cases remain separate;
host tests do not qualify the Pi native camera, GPIO or AprilTag stack.

## Deployment and restoration contract

The Pi continues using its working CPython 3.13 environment until its native
stack passes the hardware gates in `refactor.md`. The development version file
does not authorize migration. The installer pins service commands to
`PROJECT/.venv/bin/python`, and the deployment transaction preserves the
supported selected interpreter. When a new environment is necessary, the default is
explicitly `/usr/bin/python3.13`. Repair of an existing environment requires
known supported version metadata and preserves its recorded base interpreter;
unknown metadata fails closed instead of falling back to `/usr/bin/python3`.

Deployment currently **refuses an existing Python 3.14 environment** before
clearing or syncing its dependencies, whether or not it includes system packages.
An isolated candidate can contain separately built native wheels absent from the
runtime lock; a rebuild or exact sync would discard them. Promotion needs a
reviewed native-artifact manifest and installation contract that preserves or
reinstalls the qualified wheels, verifies their hashes and matching system-library
versions, and tests paired rollback. This deployment work remains incomplete even
if candidate imports and acquisition succeed. Keep the working 3.13 route selected.

Before a successful deployment can restart the runtime, the selected interpreter
must be standard GIL-enabled Python within the supported range and import the
application's native dependencies, including libcamera, pyKMS, lgpio, prctl and
AprilTag. Imports do not construct camera or actuator objects. This check catches
ABI incompatibility; it does not replace acquisition or timing qualification.

A separate uv candidate environment can be prepared beside the working project,
with an explicit standard 3.14 interpreter and the same lock. Do not reuse
CPython 3.13 extension files as 3.14 bindings. Match the installed ARM64 native
libraries and record every rebuilt package. Preserve system Python, the current
`.venv`, services, configuration and source. Candidate selection for service use
requires native imports, disarmed camera/recording/tag work, GPIO compatibility
without actuation, passive telemetry and resource/timing results, followed by the
native-artifact deployment and rollback gate above. Record local candidate wheels
with `uv add --group candidate /absolute/path/to/package.whl` in the separate
candidate project; its metadata and lock are evidence, not automatic promotion
inputs for the working installation.

Deployment backs up source and `.venv` together before mutation. An installation
or native-import failure restores both; failed restoration leaves the service
stopped and retains the rollback directory. A successful source install with a
failed health check also retains the backup. Recovery must restore both trees
from the same directory, retain the explicit environment interpreter, and repeat
the read-only post-restart health check. Do not restore just source over a changed
native environment. No migration or rollback described here grants permission
for live flight commands, parameter writes or service changes.
