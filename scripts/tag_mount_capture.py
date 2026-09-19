"""Full sensor recording with a one-time mount open on a selected AprilTag.

On the Pi, from ~/ai-drone:
    uv run --locked --group raspi python scripts/tag_mount_capture.py
    uv run --locked --group raspi python scripts/tag_mount_capture.py --duration 60
    uv run --locked --group raspi python scripts/tag_mount_capture.py --tag-id 7

Works disarmed or already flying, including under manual RC control. Records
video, all available FC telemetry, detections and servo events until Ctrl-C or
the duration expires. Three fresh, high-quality detections of the selected
tag36h11 ID (default 3) open the mount with the position and settle time from
mount.py. PWM then stops; the mount never automatically re-closes. Prepare it
with `uv run python scripts/mount.py close`. No flight-control commands are sent.
"""

from ai_drone.cli.record import run


def main(arguments: list[str] | None = None) -> int:
    return run(arguments, operation="tag-mount")


if __name__ == "__main__":
    raise SystemExit(main())
