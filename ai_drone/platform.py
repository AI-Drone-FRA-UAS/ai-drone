"""Runtime platform checks shared by Raspberry Pi command-line tools."""

from __future__ import annotations

from pathlib import Path

RASPBERRY_PI_MODEL_PATH = Path("/proc/device-tree/model")


def is_raspberry_pi(model_path: Path | None = None) -> bool:
    """Return whether the Linux device-tree model identifies a Raspberry Pi."""

    path = RASPBERRY_PI_MODEL_PATH if model_path is None else model_path
    try:
        model = path.read_text()
    except OSError:
        return False
    return "raspberry pi" in model.casefold()
