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

## Matching inputs and actual ARM64 build attempt

After SSH returned, exact matching development packages were downloaded with
`apt-get download` and extracted using `dpkg-deb --extract` under the candidate's
`native-build/inputs` and `native-build/sysroot`. No package was installed into
the OS and no maintainer scripts ran. Inputs were:

| Development input | Exact version |
| --- | --- |
| libcamera-dev | 0.7.2+rpt20260817-1 |
| libkms++-dev | 0~git20250807.1813ada-1 |
| libdrm-dev | 2.4.134-3~bpo13+1+rpt1 |
| libcap-dev | 1:2.75-10+deb13u1+b1 |
| liblgpio-dev | 0.2.2-1~rpt1+trixie |
| libapriltag-dev | 3.4.2-1+b2 |

The exact libcamera source tag resolves to
`6c1dd9d55573010f710c9e190a73e7e76f0d9432`. The published binding wrapper's
patch targets an older source layout and fails against this version. A separate
minimal Meson wrapper therefore enters the unchanged matching upstream Python
binding directory, using the extracted headers and installed library SONAMEs.
Its locked tools include pybind11 3.1.0, Meson 1.12.0 and Ninja 1.13.2. All ten
binding objects compiled on the x86_64 host under CPython 3.14; this does not
establish ARM64 linkage or camera compatibility.

The reviewed kit was staged as `native-build/libcamera-kit`, with build files,
an isolated uv tools environment and relocated candidate-only pkg-config files
under `native-build/libcamera-314`. The ARM64 build configured successfully
against the exact installed 0.7.2 library, selected managed CPython 3.14.7 and
started a single compiler job with O1, no debug symbols and no LTO. During its
first C++ object, the Pi's 414 MiB swap became fully used, available RAM fell to
48 MiB, and temperature reached 77.364 °C. No completed wheel or successful
native import was observed.

Targeted attempts to terminate only the identified candidate compiler could not
be confirmed because SSH became unresponsive and then timed out. This temporal
association does not prove the cause of the connectivity loss. Final compiler
state and cleanup remain unverified until access returns. No swap change,
system upgrade, service restart, native library replacement or candidate
promotion was attempted. The original kit/build logs and failed stop attempts
are retained; resource-bounded retry inputs are prepared separately for review.

## Prepared bounded build inputs

Local evidence includes the original libcamera kit and a separate bounded kit
(`rpi-native-libcamera314-bounded-kit.tar.gz`, SHA-256
`145795f5896c7c3eb46783bf89ecbc70b3611c6fdbed18396f884b46ff5e0c0b`).
The latter uses O0, one job, GCC garbage-collection tuning, a compiler-only
256 MiB virtual-memory/300 s CPU/wall cap and a 20-minute wheel-build deadline.
It was not run on the Pi. A cap failure must remain a build failure; do not
remove the cap or expand system swap automatically to force a result.

A separate pyKMS kit pins upstream
`1813adae89203e15fc60344864fc2ca0d3c75e07` and the matching development package
hashes. `rpi-pykms-candidate-kit-r1.tar.gz` has SHA-256
`2ce6908cfd7a5fa0ad10b7759089bb4088818b727c6aa7d5518af49e069c9dc7`.
Its three units passed uncapped host CPython 3.14 syntax checks, but compilation
of the first unit under the 256 MiB cap failed with out-of-memory. This is a
recorded resource blocker, not an ARM64 compatibility pass. The kit only builds
and inspects a wheel; it does not access DRM/camera devices or install packages.
It was not uploaded or run on the Pi.

The small-bindings kit `rpi-native-small314-kit.tar.gz` (SHA-256
`0e39adbcf55506cc41e84623077fb13a00523e6da9c45366da31b859ae1115d6`)
contains exact Debian/Raspberry Pi sources for AprilTag 3.4.2, python-prctl 1.8.1
and lgpio 0.2.2, matching development-package checks, locked uv tools and a
generated lgpio wrapper pinned to SWIG 4.3.1. SWIG 4.5.0 removed compatibility
aliases required by this older source; the pinned generator built successfully.
All three x86_64 extensions built and imported under standard uv CPython 3.14.6
and NumPy 2.5.3. A generated tag17 passed the exact Debian-native dictionary
contract and the project's real `NativeAprilTagDetector` adapter. `prctl.get_name`
and lgpio version reads passed; no GPIO handles were opened. lgpio import creates
a local notification FIFO/thread, which is documented for any future candidate
smoke check. Host linkage used separately built matching AprilTag/lgpio runtimes
and host libcap; it does not prove ARM64 compatibility. The kit was not uploaded
or run on the Pi; it builds wheels only, sequentially, without installing them.

All three kits and their source/hash/host-check evidence are retained under
`artifacts/refactor-20260921/bundles/`. They are prepared review inputs, not a
successful Pi native migration or an instruction to weaken resource limits.

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

Review also found that the previous generic environment preparation could clear
an isolated selected 3.14 environment before the native-import failure triggered
rollback. Commit `0cc15d9` now refuses such deployment before any environment
rebuild or uv synchronization, preserving metadata and native files. Normal
3.13 deployment remains supported. A future 3.14 promotion requires a reviewed
native-wheel manifest and installation/preservation contract; the current
deployment entry point deliberately does not select it.

The candidate is separate and can remain for review. Removing it later needs no
system-Python rollback: the working application was never switched. A future
actual migration must retain a paired source/environment backup and validate the
read-only health path after restoration; restoring source alone is insufficient.
