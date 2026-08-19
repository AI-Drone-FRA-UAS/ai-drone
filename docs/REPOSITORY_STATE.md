# Repository state — 2026-08-19

What the codebase looks like right now, and what is and is not verified. This
records the *repository*; live vehicle and Pi state live in the dated captures
under `state/` and the parameter dumps under `params/`.

> Not yet reviewed with the project owner. Recheck this document and `README.md`
> together before relying on either.

## Package layout

`ai_drone/` is grouped by concern; subpackages carry the namespace.

| Package | Contents | Purpose |
| --- | --- | --- |
| `cli/` | `apriltag`, `config_export`, `control`, `lidar`, `motor_test`, `record`, `servo` | Thin hardware-facing command adapters |
| `link/` | `connect`, `wifi`, `usb_ssh`, `targets`, `deploy` | Host↔Pi transport selection and deployment, cross-platform |
| `mavlink/` | `devices`, `safety`, `parameters`, `console`, `health` | Talking to the flight controller, excluding flight control |
| `vision/` | `apriltags`, `stream` | Camera-facing detection and MJPEG streaming |
| `flight/` | `controller`, `guards` | MAVLink flight control and the safety guards every mode applies |
| `config/` | `snapshot`, `sync` | Deterministic parameter capture and host synchronization |
| root leaves | `durability`, `validation`, `platform`, `cli_parsing`, `recording` | Shared helpers that import nothing internal |

Dependencies run one way — `cli/` → concern packages → shared leaves — with no
import cycles. This is enforced, not just documented: `.importlinter` carries
four contracts and `lint-imports` runs in the standard check block.

## Commands

Thirteen console entry points, all verified to resolve:

`drone-health`, `drone-lidar`, `drone-record`, `drone-apriltag`,
`drone-config-export`, `drone-config-sync`, `drone-console`,
`drone-motor-test`, `drone-servo`, `drone-control`, `drone-deploy`,
`autoconnect`, `manuconnect`.

`drone-control` offers `status`, `hover`/`takeoff`, and `velocity-test`.
`hover` already implements take off → hold → land with the guards applied on
every loop iteration.

## Checks

The block in `CLAUDE.md` is the full gate: `ruff format --check`, `ruff check`,
`ty check`, `pytest -q`, `lint-imports`, `deptry .`, `git diff --check`. All
green as of this document, with 313 tests passing and 1 skipped.

`deptry` configuration in `pyproject.toml` carries a written reason for every
ignore. `mavproxy` is a subprocess CLI; `pyserial` and `future` back pymavlink;
`picamera2`, `gpiozero`, and `apriltag` are apt-native on the Pi.

## Retired this session

Moved to `attic/`, which is excluded from linting, type checking, tests, and
deployment. See `attic/README.md` for the full rationale.

- Person following: `flight/follower.py`, the `drone-control follow`
  subcommand, and its tests. This removed the last importer of `modlib`, which
  was dropped from the `raspi` dependency group.
- `docs/HANDOFF.md`: duplicated the `state/` and `params/` captures and drifted
  out of date faster than either.

Deliberately kept back from the person-follow retirement: the battery,
altitude-ceiling, and telemetry-staleness guards, now in `flight/guards.py`.
They previously existed **only** on the person-follow path, so `hover` and
`velocity-test` had no battery or ceiling guard at all. They do now.

## Known-good and known-unverified

Verified offline: the full check block, all entry points resolving, the
`drone-deploy` payload listing every subpackage, and the lazy package root
(`import ai_drone` no longer loads pymavlink).

Not verified: anything requiring the aircraft. No live arm, takeoff, altitude
hold, or velocity flight has been validated. There is no SITL harness in the
repository yet, so no flight path has been exercised end to end.

Open questions for the next review:

- `docs/APRILTAG_MISSION.md:158` and `docs/DRONE_CONFIGURATION.md:12` cite
  `ARMING_CHECK=0`. That still matches
  `params/flywoo-f745-live-2026-08-18.param`, so it is accurate until the
  controller is re-exported — but the export is a day old.
- The camera's orientation, rigidity, and intrinsic calibration have not been
  recorded anywhere. Metric floor-tag geometry stays untrusted until they are.
- No SITL harness exists, so `drone-control hover` has never been exercised
  end to end.

## Corrected on 2026-08-19

- `numpy` moved from the `raspi` group to the base dependencies.
  `vision/apriltags.py` imports it at module scope and is tested off-Pi, so it
  was resolving only incidentally through `mavproxy`. `opencv-python` stays in
  `raspi` because `cv2` is imported lazily inside functions, which is the
  pattern Pi-native packages should follow.
- `README.md` no longer says the servo is disconnected.
- `docs/drone-project.md` no longer says the camera is cable-held and
  forward-facing, and no longer advertises the retired person-follow mode.
