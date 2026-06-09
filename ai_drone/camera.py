"""Platform-aware camera abstraction.

On a Raspberry Pi (detected via /proc/device-tree/model) the camera uses
``picamera2`` which talks to the IMX500 AI Camera via libcamera.
On any other machine it falls back to OpenCV ``VideoCapture``.

``picamera2`` is a system package (``sudo apt install python3-picamera2``)
and is NOT listed in pyproject.toml.  The Pi venv must be created with
``--system`` so it can see the system site-packages.
"""

from __future__ import annotations

import logging
import sys
from importlib import import_module
from pathlib import Path
from typing import TYPE_CHECKING, Protocol, cast

import numpy as np

if TYPE_CHECKING:
    from numpy.typing import NDArray

logger = logging.getLogger(__name__)


class _PiCamera(Protocol):
    """Subset of picamera2 used by this module."""

    def create_still_configuration(self, *args: object, **kwargs: object) -> object: ...
    def configure(self, config: object) -> None: ...
    def start(self) -> None: ...
    def capture_array(self, name: str) -> NDArray[np.uint8]: ...
    def stop(self) -> None: ...
    def close(self) -> None: ...


class _OpenCVCamera(Protocol):
    """Subset of cv2.VideoCapture used by this module."""

    def isOpened(self) -> bool: ...
    def release(self) -> None: ...
    def read(self) -> tuple[bool, NDArray[np.uint8]]: ...


_CameraHandle = _PiCamera | _OpenCVCamera | None


# ---------------------------------------------------------------------------
# Platform detection
# ---------------------------------------------------------------------------


def is_raspberry_pi() -> bool:
    """Return *True* when running on a Raspberry Pi."""
    try:
        model = Path("/proc/device-tree/model").read_text()
        return "raspberry pi" in model.lower()
    except (FileNotFoundError, PermissionError):
        return False


_ON_PI = is_raspberry_pi()


# ---------------------------------------------------------------------------
# Camera wrapper
# ---------------------------------------------------------------------------


class Camera:
    """Unified camera interface for Pi (picamera2) and laptop (OpenCV).

    Usage::

        cam = Camera()
        frame = cam.capture()   # numpy HxWx3 BGR array
        cam.close()
    """

    def __init__(self) -> None:
        self._backend: str
        self._cam: _CameraHandle

        if _ON_PI:
            self._init_picamera2()
        else:
            self._init_opencv()

        logger.info("Camera ready  [backend=%s]", self._backend)

    # -- Pi -----------------------------------------------------------------

    def _init_picamera2(self) -> None:
        try:
            picamera2 = import_module("picamera2")
        except ImportError:
            logger.error(
                "picamera2 not found.  Install it with:\n"
                "  sudo apt install -y python3-picamera2 imx500-all\n"
                "and make sure the venv was created with --system."
            )
            sys.exit(1)

        picamera_cls = cast("type[_PiCamera]", getattr(picamera2, "Picamera2"))
        cam = picamera_cls()
        # Use a small still config for low memory footprint on the Zero 2.
        config = cam.create_still_configuration(
            main={"size": (640, 480), "format": "RGB888"},
        )
        cam.configure(config)
        cam.start()
        self._cam = cam
        self._backend = "picamera2"

    # -- Laptop / generic ---------------------------------------------------

    def _init_opencv(self) -> None:
        try:
            import cv2  # type: ignore[import-untyped]
        except ImportError:
            logger.error(
                "opencv-python not found.  Install it with:\n  uv sync --group desktop"
            )
            sys.exit(1)

        cap = cast("_OpenCVCamera", cv2.VideoCapture(0))
        if not cap.isOpened():
            logger.warning("No webcam detected — using a synthetic test pattern.")
            cap.release()
            self._cam = None
            self._backend = "synthetic"
            return

        self._cam = cap
        self._backend = "opencv"

    # -- Public API ---------------------------------------------------------

    @property
    def backend(self) -> str:
        """Return the name of the active backend."""
        return self._backend

    def capture(self) -> NDArray[np.uint8]:
        """Capture a single frame and return it as an HxWx3 BGR numpy array."""
        if self._backend == "picamera2":
            cam = cast("_PiCamera", self._cam)
            # picamera2 returns RGB; convert to BGR for OpenCV compat.
            rgb = cam.capture_array("main")
            return rgb[:, :, ::-1].copy()

        if self._backend == "opencv":
            cap = cast("_OpenCVCamera", self._cam)
            ret, frame = cap.read()
            if not ret:
                raise RuntimeError("Failed to read frame from webcam.")
            return frame

        # Synthetic fallback — purple/green gradient test pattern.
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:, :, 0] = np.linspace(0, 200, 640, dtype=np.uint8)  # B
        frame[:, :, 2] = np.linspace(200, 0, 640, dtype=np.uint8)  # R
        return frame

    def close(self) -> None:
        """Release camera resources."""
        if self._backend == "picamera2":
            cam = cast("_PiCamera", self._cam)
            cam.stop()
            cam.close()
        elif self._backend == "opencv":
            cap = cast("_OpenCVCamera", self._cam)
            cap.release()

        logger.info("Camera closed  [backend=%s]", self._backend)

    # -- Context manager ----------------------------------------------------

    def __enter__(self) -> Camera:
        return self

    def __exit__(self, *_: object) -> None:
        self.close()
