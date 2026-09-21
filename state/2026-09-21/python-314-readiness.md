# Python 3.14 readiness check — 2026-09-21

Scope: read-only inspection through `ssh seb@seb-is-pm`, plus an isolated
laptop compatibility experiment against project commit `2eefca0`. No interpreter,
package, service configuration, or drone parameter was changed on the Pi. Module
imports did not construct camera, GPIO, serial, or flight-control objects.

## Result

Python 3.14 can be installed alongside the Pi's existing Python using uv. The
complete aircraft application cannot simply reuse its current hardware bindings
under 3.14. Treat laptop compatibility and Pi runtime migration as separate gates.

Before this check, `refactor.md` had no explicit Python 3.14 migration step.
The questionnaire permits selecting newer Python versions according to tested
dependency support, but does not itself establish 3.14 compatibility.

## Observed configuration

| Item | Live observation |
| --- | --- |
| OS | Debian GNU/Linux 13 (trixie), `DEBIAN_VERSION_FULL=13.6` |
| Architecture | `aarch64`, 64-bit user space |
| System Python | CPython 3.13.5, `/usr/bin/python3 -> python3.13` |
| Application environment | `/home/seb/ai-drone/.venv`, CPython 3.13.5, `include-system-site-packages = true` |
| uv | 0.12.11, aarch64 Linux |
| Available interpreter | `uv python list 3.14 --all-versions` lists CPython 3.14.7 for Linux aarch64 GNU as downloadable; 3.14 was not installed |
| APT | Existing package metadata returned no Python 3.14 candidate; package indexes were not refreshed |
| Resources | About 415 MiB reported RAM, 250 MiB available at inspection; about 23 GiB free root storage |

The deployed project and reviewed source both require `>=3.11,<3.14`. In the
reviewed source, `.python-version` selects 3.12 and CI covers 3.11–3.13. The
deployed checkout has no `.python-version` and a different console-script layout;
local branch state must not be treated as the installed source revision.

Current application-environment imports succeeded for `numpy` (2.5.3), Pillow
(12.3.0), `pymavlink` (2.4.49), OpenCV (5.0.0), `picamera2`, `libcamera`, `pykms`,
`gpiozero`, `lgpio`, `prctl`, and `apriltag`. These are import checks, not camera,
actuator, telemetry, or flight validation.

## Native hardware dependency gate

APT reports `python3-libcamera=0.7.2+rpt20260817-1`,
`python3-picamera2=0.3.37-1`, and `python3-gpiozero=2.0.1-0+rpt1+trixie`.
The following installed modules are specific to CPython 3.13 on ARM64:

- `libcamera/_libcamera.cpython-313-aarch64-linux-gnu.so`
- `pykms/pykms.cpython-313-aarch64-linux-gnu.so`
- `_lgpio.cpython-313-aarch64-linux-gnu.so`
- `_prctl.cpython-313-aarch64-linux-gnu.so`
- `apriltag.cpython-313-aarch64-linux-gnu.so`

`--system-site-packages` does not make these binaries compatible with Python 3.14.
Audit transitive dependencies too; this list establishes a blocker to reusing
the current environment, not an exhaustive inventory of all required rebuilds.
CPython documents minor-version ABI boundaries and the separate stable-ABI
exception: [C API stability](https://docs.python.org/3.14/c-api/stable.html).

Raspberry Pi provides a route for building Python libcamera bindings against an
installed libcamera library, while recommending the simpler system-Python setup
where possible. A 3.14 candidate must match this Pi's installed native library
version and verify the other camera/GPIO/tag dependencies independently; a
successful `rpi-libcamera` installation alone is insufficient.
[Raspberry Pi binding guidance](https://github.com/raspberrypi/pylibcamera/blob/main/README.md).

The deployment installer currently creates a system-Python environment when
system-package access is missing. Interpreter selection and validation need an
explicit migration design, including recovery, rather than only changing the
repository's Python version file.

## Laptop compatibility experiment

A temporary archive of `2eefca0` was tested under laptop CPython 3.14.7
(Linux x86_64). Only the support upper bound was expanded to `<3.15` and the
temporary `.python-version` changed to 3.14. The production checkout's metadata,
lockfile and environment were not changed. The temporary lock was regenerated;
dependency versions remained unchanged, with additional interpreter wheel data.

```bash
UV_CACHE_DIR=/tmp/uv-cache uv sync --python /usr/bin/python3.14 \
  --group dev --group docs --group raspi
UV_CACHE_DIR=/tmp/uv-cache uv run --no-sync --python /usr/bin/python3.14 \
  pytest -q -m 'not sitl'
```

Dependency installation succeeded, including a source build of
`pymavlink==2.4.49`. Result: **1,424 passed, one skipped, five deselected in
15.72 seconds**. The skip was the gpiozero mock-PWM test because gpiozero is not
installed in the laptop environment. The five deselected cases were SITL;
no simulation or aircraft tests were run for this compatibility check.

An ARM64 `uv sync --dry-run --locked --offline --python-platform
aarch64-unknown-linux-gnu --group raspi --no-install-project` using the candidate
lock also succeeded. Requiring binary-only installation with `--no-build`
failed for `pymavlink==2.4.49`: a source build is needed on that target. This
dry run did not execute an ARM64 build, install packages on the Pi, or include
the separately supplied camera/GPIO/AprilTag dependencies.

These results support starting the compatibility migration. They do not complete
the CI, SITL, native ARM64 or Pi performance gates.

## Migration decision

1. Add and verify standard, GIL-enabled Python 3.14 support locally and in CI;
   retain a supported 3.13 route for the Pi while its native stack is qualified.
2. Regenerate the dependency lock for the expanded support range. Keep language
   syntax and lint targets compatible with every version still supported.
3. Prepare a separate uv-managed 3.14 candidate environment and reproducible
   ARM64 bindings. Preserve the working system Python and application environment.
   CPU/memory costs make large on-device builds a choice to measure, not a default.
4. Require native imports, actual disarmed camera/recording/tag tests, GPIO backend
   compatibility without actuation, runtime telemetry, resource/timing measurements,
   and deployment/rollback tests before selecting the new Pi runtime.

The uv 0.12.11 behavior was checked locally and on the Pi. Context7 returned the
current upstream documentation rather than an exact versioned 0.12.11 snapshot:
[uv Python versions](https://docs.astral.sh/uv/concepts/python-versions/).
Free-threaded Python is a separate experiment; explicitly select the GIL-enabled
build for the first migration.
