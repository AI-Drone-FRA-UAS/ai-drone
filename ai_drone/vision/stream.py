"""Lightweight MJPEG server for the Pi's annotated AI-camera frames."""

from __future__ import annotations

import math
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import cast

# Shared state: the latest JPEG frame and a condition to notify waiters.
_frame_lock = threading.Lock()
_frame_condition = threading.Condition(_frame_lock)
_current_frame: bytes = b""
DEFAULT_MAX_STREAM_CLIENTS = 3
DEFAULT_CLIENT_TIMEOUT_SECONDS = 5.0


class _MJPEGServer(ThreadingHTTPServer):
    """Threaded HTTP server with bounded persistent stream clients."""

    daemon_threads = True

    def __init__(
        self,
        server_address: tuple[str, int],
        handler: type[BaseHTTPRequestHandler],
        *,
        max_stream_clients: int,
        client_timeout: float,
    ) -> None:
        self.stream_slots = threading.BoundedSemaphore(max_stream_clients)
        self.client_timeout = client_timeout
        super().__init__(server_address, handler)


def push_frame(jpeg: bytes) -> None:
    """Push a new JPEG frame to all connected clients.

    The nearest-person pipeline uses this to hand the latest encoded frame to
    the MJPEG server.
    """
    global _current_frame
    with _frame_condition:
        _current_frame = jpeg
        _frame_condition.notify_all()


class _MJPEGHandler(BaseHTTPRequestHandler):
    """Serves an MJPEG stream on GET / and a single snapshot on GET /snap."""

    def setup(self) -> None:
        super().setup()
        server = cast(_MJPEGServer, self.server)
        self.connection.settimeout(server.client_timeout)

    def do_GET(self) -> None:
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
        server = cast(_MJPEGServer, self.server)
        if not server.stream_slots.acquire(blocking=False):
            self.send_error(503, "Too many stream clients")
            return
        boundary = b"--frame"
        try:
            self.send_response(200)
            self.send_header(
                "Content-Type",
                "multipart/x-mixed-replace; boundary=frame",
            )
            self.end_headers()
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
        except OSError:
            pass  # Client disconnected.
        finally:
            server.stream_slots.release()

    def log_message(self, format: str, *args: object) -> None:
        """Suppress default per-request logging — too noisy for video."""


def start_server(
    host: str = "0.0.0.0",
    port: int = 8080,
    *,
    max_stream_clients: int = DEFAULT_MAX_STREAM_CLIENTS,
    client_timeout: float = DEFAULT_CLIENT_TIMEOUT_SECONDS,
) -> ThreadingHTTPServer:
    """Start the MJPEG HTTP server in a background thread and return it.

    Producers then feed frames with :func:`push_frame`. Call
    ``server.shutdown()`` then ``server.server_close()`` when done.
    """
    if (
        isinstance(max_stream_clients, bool)
        or not isinstance(max_stream_clients, int)
        or max_stream_clients <= 0
    ):
        raise ValueError("max_stream_clients must be a positive integer")
    if not math.isfinite(client_timeout) or client_timeout <= 0:
        raise ValueError("client_timeout must be finite and positive")

    global _current_frame
    with _frame_condition:
        _current_frame = b""  # drop any frame left over from a previous session
    server = _MJPEGServer(
        (host, port),
        _MJPEGHandler,
        max_stream_clients=max_stream_clients,
        client_timeout=client_timeout,
    )
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    return server
