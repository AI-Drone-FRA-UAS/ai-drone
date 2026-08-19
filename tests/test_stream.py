from __future__ import annotations

from io import BytesIO
from socketserver import BaseServer, ThreadingMixIn
from threading import BoundedSemaphore, Event, Thread
from types import SimpleNamespace
from typing import cast

import pytest

from ai_drone import stream


class _MemoryHandler(stream._MJPEGHandler):
    def __init__(
        self,
        output: BytesIO,
        headers_sent: Event | None = None,
        *,
        server: object | None = None,
    ) -> None:
        self.wfile = output
        self.statuses: list[int] = []
        self.headers_sent = headers_sent
        self.server = cast(
            BaseServer,
            server
            or SimpleNamespace(
                stream_slots=BoundedSemaphore(stream.DEFAULT_MAX_STREAM_CLIENTS)
            ),
        )

    def send_response(self, code: int, message: str | None = None) -> None:
        del message
        self.statuses.append(code)

    def send_header(self, keyword: str, value: str) -> None:
        del keyword, value

    def end_headers(self) -> None:
        if self.headers_sent is not None:
            self.headers_sent.set()

    def send_error(
        self,
        code: int,
        message: str | None = None,
        explain: str | None = None,
    ) -> None:
        del message, explain
        self.statuses.append(code)


class _DisconnectingOutput(BytesIO):
    def write(self, data, /) -> int:
        del data
        raise BrokenPipeError


def test_server_uses_threaded_request_dispatch(monkeypatch) -> None:
    assert issubclass(stream._MJPEGServer, ThreadingMixIn)
    created: list[tuple[tuple[str, int], type[stream._MJPEGHandler]]] = []

    class FakeServer:
        def __init__(
            self,
            address: tuple[str, int],
            handler: type[stream._MJPEGHandler],
            *,
            max_stream_clients: int,
            client_timeout: float,
        ) -> None:
            created.append((address, handler))
            assert max_stream_clients == stream.DEFAULT_MAX_STREAM_CLIENTS
            assert client_timeout == stream.DEFAULT_CLIENT_TIMEOUT_SECONDS

        def serve_forever(self) -> None:
            return

    monkeypatch.setattr(stream, "_MJPEGServer", FakeServer)

    server = stream.start_server(host="127.0.0.1", port=0)

    assert isinstance(server, FakeServer)
    assert created == [(("127.0.0.1", 0), stream._MJPEGHandler)]


def test_snapshot_can_run_while_stream_handler_is_connected() -> None:
    jpeg = b"offline-test-jpeg"
    headers_sent = Event()
    stream.push_frame(jpeg)
    stream_handler = _MemoryHandler(_DisconnectingOutput(), headers_sent)
    stream_thread = Thread(target=stream_handler._send_mjpeg_stream)
    stream_thread.start()
    assert headers_sent.wait(timeout=1.0)

    snapshot_output = BytesIO()
    snapshot_handler = _MemoryHandler(snapshot_output)
    snapshot_handler._send_snapshot()

    assert snapshot_handler.statuses == [200]
    assert snapshot_output.getvalue() == jpeg

    stream.push_frame(jpeg)
    stream_thread.join(timeout=1.0)
    assert not stream_thread.is_alive()


def test_stream_client_limit_refuses_an_additional_persistent_client() -> None:
    server = SimpleNamespace(stream_slots=BoundedSemaphore(1))
    assert server.stream_slots.acquire(blocking=False)
    handler = _MemoryHandler(BytesIO(), server=server)

    handler._send_mjpeg_stream()

    assert handler.statuses == [503]
    server.stream_slots.release()


def test_server_rejects_invalid_resource_limits() -> None:
    with pytest.raises(ValueError, match="max_stream_clients"):
        stream.start_server(host="127.0.0.1", port=0, max_stream_clients=0)
    with pytest.raises(ValueError, match="max_stream_clients"):
        stream.start_server(
            host="127.0.0.1",
            port=0,
            max_stream_clients=1.5,  # ty: ignore[invalid-argument-type]
        )
    with pytest.raises(ValueError, match="client_timeout"):
        stream.start_server(host="127.0.0.1", port=0, client_timeout=float("nan"))
    with pytest.raises(ValueError, match="client_timeout"):
        stream.start_server(host="127.0.0.1", port=0, client_timeout=0.0)
