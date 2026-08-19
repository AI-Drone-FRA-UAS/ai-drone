# Attic

Retired code kept for reference only. Nothing here is maintained, linted,
type-checked, tested, deployed to the Pi, or exposed as an entry point.
Paths mirror their original location in the repository.

## Contents

### Nearest-person detection (retired 2026-08-19)

The IMX500 NanoDet person-detection pipeline and its `drone-picam` command.
Removed because the capability is no longer needed; person-follow flight
control in `ai_drone/follower.py` is independent and remains maintained.

- `ai_drone/nearest_person.py` — detection, ByteTrack tracking, nearest-pair
  metric conversion, and MJPEG annotation.
- `ai_drone/cli/picam.py` — the `drone-picam` entry point (also removed from
  `pyproject.toml` and from `drone-deploy --picam`).
- `ai_drone/data/nearest_person_regions.json` — metre-per-pixel calibration
  regions.
- `tests/test_nearest_person.py`, `tests/test_picam_cli.py` — its offline tests.

These files import `ai_drone.vision.stream`, which is still present and used
by `drone-apriltag`.

### Person following (retired 2026-08-19)

The forward-camera person-follow flight mode and its `drone-control follow`
subcommand. Retired because the project's mission is now a nadir AprilTag
search and payload drop, which shares none of the person-specific geometry or
the proportional follow law.

- `ai_drone/flight/follower.py` — `get_person_target`, `PersonTarget`, and the
  `AutonomousFollower` controller, including its modlib live-tracking loop.
- `ai_drone/cli/control_follow.py` — the retired `follow` subcommand, its
  `_SimulationController`, and the subparser wiring, kept as commented
  reference. Not importable as written.
- `tests/test_follower.py` — its offline tests.

What was deliberately **not** retired: the battery, altitude-ceiling, and
telemetry-staleness guards that used to live on `AutonomousFollower`. They are
generic flight safety, so they moved to `ai_drone/flight/guards.py` and are now
applied by `drone-control hover` and `drone-control velocity-test`, which
previously had no battery or ceiling guard at all.

Retiring this also removed the `modlib` dependency from the `raspi` group; it
was the only importer. Restoring the capability means restoring that entry.

### Session handoff document (retired 2026-08-19)

`docs/HANDOFF.md`, the long-running verified-state and next-actions document.
Retired because it duplicated the dated captures under `state/` and the
parameter dumps under `params/`, and drifted out of date faster than either.
Those captures are now the record of live vehicle and Pi state; the repository's
own state is described in `docs/REPOSITORY_STATE.md`.

- `docs/HANDOFF.md` — the last version, including the 2026-08-19 camera and
  servo update.
