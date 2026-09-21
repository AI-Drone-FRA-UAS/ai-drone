# Separate Pi Python 3.14 candidate — 2026-09-21

This extends [the earlier read-only readiness check](python-314-readiness.md).
It is a compatibility experiment, not a deployed runtime migration.
The working `/home/seb/ai-drone/.venv` still uses system CPython 3.13.5;
`/usr/bin/python3`, installed source and services were not switched or upgraded.

## Candidate created

Base directory: `/home/seb/ai-drone-candidates/20260921-py314`.

- `interpreters/cpython-3.14.7-linux-aarch64-gnu/bin/python3.14`: installed with
  `uv python install --install-dir …/interpreters --no-bin
  cpython-3.14.7-linux-aarch64-gnu`. No executable links were installed.
- `source-fbbe690`: revision-based staged source from `fbbe690` initially containing
  the expanded Python metadata, lock, README and application package.
- `source-fbbe690/.venv`: created by `uv sync --locked --python <candidate>
  --group raspi`. It does not include system site packages.
- `cache`: separate uv package cache.
- Candidate-only `uv add --group candidate gpiozero==2.0.1` then added that
  deployed version, colorzero and setuptools and updated only this candidate's
  `pyproject.toml`/`uv.lock`. It did not modify the integration repository's lock
  or the working Pi application environment.

Installation completed, including an ARM64 source build of pymavlink 2.4.49.
The initial sync took approximately 4 minutes 32 seconds. The interpreter reports
`Py_GIL_DISABLED=0`; this is standard CPython, not a free-threaded experiment.

## Executed import results

| Module | Candidate result |
| --- | --- |
| NumPy 2.5.3 | Passed |
| Pillow 12.3.0 | Passed |
| pymavlink 2.4.49 | Passed; ARM64 source build completed |
| OpenCV 5.0.0 | Passed |
| gpiozero 2.0.1 | Passed after separate candidate dependency addition |
| Picamera2 | Missing normally; with the read-only apt path, fails on `libcamera._libcamera` |
| libcamera | Fails on `libcamera._libcamera` with apt path |
| pyKMS | Fails on `pykms.pykms` with apt path |
| lgpio | Fails on `_lgpio` with apt path |
| prctl | Fails on `_prctl` with apt path |
| apriltag | No compatible module found, including with apt path |

These imports constructed no camera, serial, GPIO or actuator objects.
The apt path was appended only within a short-lived import-check process.
No CPython 3.13 binary was copied, renamed or forced into the candidate.

The five installed extension files have CPython 3.13 ARM64 suffixes.
The installed libcamera library is 0.7.2, but `/usr/include/libcamera` and its
pkg-config development metadata are absent. A matched 3.14 camera binding build
therefore needs exact matching development inputs and a reproducible build for
that library version. Additional compatible pyKMS, lgpio, prctl and the project's
native AprilTag API must also be supplied. No broad OS upgrade or native library
replacement was attempted. This is a concrete native ABI/development-input gate,
not evidence that these packages can never support 3.14.

The binding requirement follows Raspberry Pi's [binding build guidance](https://github.com/raspberrypi/pylibcamera/blob/main/README.md)
and CPython's [minor-version ABI contract](https://docs.python.org/3.14/c-api/stable.html).
Installing the generic PyPI package named `apriltag` without checking its API
would not establish compatibility with the existing native backend.

## Selection and rollback status

The 3.14 candidate is **not eligible for service selection**. Native camera/tag
acquisition, hardware GPIO backend compatibility without actuation, resource
measurements and runtime telemetry under this candidate remain incomplete.
Successful host 3.14 tests and these ARM64 imports do not close those gates.

Prepared deployment code preserves an existing interpreter, defaults new Pi
runtime creation explicitly to `/usr/bin/python3.13`, checks native imports before
restart, and backs up source and environment together. See
[the deployment and restoration contract](../../docs/PYTHON_RUNTIME.md).
No live deployment transaction or service restart was executed during this work.

The candidate is separate and can remain for review. Removing it later needs no
system-Python rollback: the working application was never switched. A future
actual migration must retain a paired source/environment backup and validate the
read-only health path after restoration; restoring source alone is insufficient.
