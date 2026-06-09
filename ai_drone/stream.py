"""Lightweight MJPEG server for the Pi's annotated AI-camera frames."""

from __future__ import annotations

import logging
import threading
from http.server import BaseHTTPRequestHandler, HTTPServer

logger = logging.getLogger(__name__)

# Shared state: the latest JPEG frame and a condition to notify waiters.
_frame_lock = threading.Lock()
_frame_condition = threading.Condition(_frame_lock)
_current_frame: bytes = b""


def push_frame(jpeg: bytes) -> None:
    """Push a new JPEG frame to all connected clients.

    The nearest-person pipeline uses this to hand the latest encoded frame to
    the MJPEG server.
    """
    global _current_frame  # noqa: PLW0603
    with _frame_condition:
        _current_frame = jpeg
        _frame_condition.notify_all()


class _MJPEGHandler(BaseHTTPRequestHandler):
    """Serves an MJPEG stream on GET / and a single snapshot on GET /snap."""

    def do_GET(self) -> None:  # noqa: N802 — required by BaseHTTPRequestHandler
        if self.path == "/snap":
            self._send_snapshot()
        elif self.path in ("/", "/stream"):
            self._send_mjpeg_stream()
        else:
            self.send_error(404)

    def _send_snapshot(self) -> None:
        with _frame_lock:
            frame = _current_frame
        if not frame:
            self.send_error(503, "No frame captured yet")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(frame)))
        self.end_headers()
        self.wfile.write(frame)

    def _send_mjpeg_stream(self) -> None:
        boundary = b"--frame"
        self.send_response(200)
        self.send_header(
            "Content-Type",
            "multipart/x-mixed-replace; boundary=frame",
        )
        self.end_headers()
        try:
            while True:
                with _frame_condition:
                    _frame_condition.wait(timeout=2.0)
                    frame = _current_frame
                if not frame:
                    continue
                self.wfile.write(boundary + b"\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame)}\r\n".encode())
                self.wfile.write(b"\r\n")
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass  # Client disconnected.

    def log_message(self, format: str, *args: object) -> None:  # noqa: A002
        """Suppress default per-request logging — too noisy for video."""


def start_server(host: str = "0.0.0.0", port: int = 8080) -> HTTPServer:
    """Start the MJPEG HTTP server in a background thread and return it.

    Producers then feed frames with :func:`push_frame`. Call
    ``server.shutdown()`` then ``server.server_close()`` when done.
    """
    global _current_frame  # noqa: PLW0603
    with _frame_condition:
        _current_frame = b""  # drop any frame left over from a previous session
    server = HTTPServer((host, port), _MJPEGHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    return server
