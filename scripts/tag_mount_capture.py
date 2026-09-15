"""Full sensor recording with a one-time mount open on AprilTag 3.

On the Pi, from ~/ai-drone:
    uv run --locked --group raspi python scripts/tag_mount_capture.py
    uv run --locked --group raspi python scripts/tag_mount_capture.py --duration 60

Works disarmed or already flying, including under manual RC control. Records
video, all available FC telemetry, detections and servo events until Ctrl-C or
the duration expires. Three fresh, high-quality tag36h11 ID 3 detections open
the mount with the same position and settle time as drone_mount open. The mount
is not closed at startup or shutdown. To prepare it, run drone_mount close
separately. No arm, mode, motor, RC override or flight setpoint is sent.
"""

from ai_drone.cli.record import run


def main(arguments: list[str] | None = None) -> int:
    return run(arguments, operation="tag-mount")


if __name__ == "__main__":
    raise SystemExit(main())
