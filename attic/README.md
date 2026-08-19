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

These files import `ai_drone.stream`, which is still present and used by
`drone-apriltag`.
