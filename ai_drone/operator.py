"""Authenticated operator presence, independent of SSH terminal lifetimes."""

from __future__ import annotations

import hashlib
import hmac
import ipaddress
import json
import logging
import os
import re
import secrets
import socket
import ssl
import stat
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path
from urllib.parse import urlsplit

from ai_drone.settings import OperatorSettings, validate_operator_forward

_IDENTIFIER = re.compile(r"[0-9a-f]{32}\Z")
_LOG = logging.getLogger(__name__)


def _reap_forward(process: subprocess.Popen) -> None:
    if process.poll() is None:
        process.terminate()
    try:
        process.wait(timeout=1)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=1)


@contextmanager
def operator_forward(
    host: str | None,
    *,
    local_host: str = "127.0.0.1",
    local_port: int = 8787,
    remote_port: int = 18787,
    reconnect_interval: float = 2,
):
    """Own an optional reverse SSH route for the responder's entire lifetime."""
    validate_operator_forward(host, remote_port)
    if host is None:
        yield
        return
    target = str(ipaddress.IPv4Address(local_host))
    if type(local_port) is not int or not 1 <= local_port <= 65535:
        raise ValueError("operator local port must be between 1 and 65535")
    command = [
        "ssh",
        "-N",
        "-T",
        "-o",
        "BatchMode=yes",
        "-o",
        "ExitOnForwardFailure=yes",
        "-o",
        "ConnectTimeout=5",
        "-o",
        "ConnectionAttempts=1",
        "-o",
        "ServerAliveInterval=2",
        "-o",
        "ServerAliveCountMax=2",
        "-o",
        "ControlMaster=no",
        "-o",
        "ControlPath=none",
        "-o",
        "ControlPersist=no",
        "-o",
        "ForkAfterAuthentication=no",
        "-o",
        "ForwardAgent=no",
        "-o",
        "ForwardX11=no",
        "-R",
        f"127.0.0.1:{remote_port}:{target}:{local_port}",
        host,
    ]
    stop = threading.Event()

    def supervise() -> None:
        while not stop.is_set():
            process = None
            try:
                process = subprocess.Popen(
                    command, stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL
                )
                _LOG.warning(
                    "Operator SSH forward process started to %s; verify the Pi's operator_alive status",
                    host,
                )
                while process.poll() is None and not stop.wait(0.2):
                    pass
                if not stop.is_set():
                    _LOG.warning(
                        "Operator SSH forward exited (%s); reconnecting",
                        process.returncode,
                    )
            except OSError as error:
                _LOG.warning("Operator SSH forward unavailable: %s", error)
            finally:
                if process is not None:
                    _reap_forward(process)
            stop.wait(reconnect_interval)

    thread = threading.Thread(
        target=supervise, name="operator-ssh-forward", daemon=True
    )
    thread.start()
    try:
        yield
    finally:
        stop.set()
        thread.join(timeout=3)
        if thread.is_alive():
            raise RuntimeError("operator SSH forward did not stop within its timeout")


def read_token(path: Path) -> bytes:
    """Read a private shared key without placing its value in arguments or logs."""
    info = path.lstat()
    if not stat.S_ISREG(info.st_mode):
        raise ValueError("operator token must be a regular file")
    if os.name == "posix" and info.st_mode & 0o077:
        raise ValueError("operator token must not be readable by group or others")
    raw = path.read_text().strip()
    if re.fullmatch(r"[0-9a-f]{64}", raw) is None:
        raise ValueError("operator token must contain 32 random bytes as hexadecimal")
    return bytes.fromhex(raw)


def create_token(path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    with os.fdopen(
        os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600), "w"
    ) as handle:
        handle.write(secrets.token_hex(32) + "\n")
        handle.flush()
        os.fsync(handle.fileno())


def proof(key: bytes, action: str, nonce: str, instance: str) -> str:
    message = f"ai-drone-operator-v1:{action}:{nonce}:{instance}".encode()
    return hmac.new(key, message, hashlib.sha256).hexdigest()


def _parse_http_response(response: bytearray) -> tuple[int, bytes] | None:
    """Return a complete bounded response, or wait for more bytes."""
    if b"\r\n\r\n" not in response:
        return None
    headers, _, body = response.partition(b"\r\n\r\n")
    lines = headers.decode("ascii").split("\r\n")
    status = int(lines[0].split()[1])
    if status == 204:
        return status, b""
    values = dict(line.lower().split(":", 1) for line in lines[1:] if ":" in line)
    length = int(values.get("content-length", "-1"))
    if not 0 <= length <= 1024 or "transfer-encoding" in values:
        raise ValueError("operator response needs a bounded content length")
    if len(body) >= length:
        return status, bytes(body[:length])
    return None


def _request(
    endpoint: str, path: str, *, timeout: float, signature: str | None = None
) -> tuple[int, bytes]:
    """Small bounded HTTP exchange: no DNS, proxies, redirects or streaming body."""
    address = urlsplit(endpoint)
    host = str(ipaddress.ip_address(address.hostname or ""))
    if address.scheme not in {"http", "https"}:
        raise ValueError("operator endpoint must use HTTP(S)")
    deadline = time.monotonic() + timeout
    connection = socket.create_connection(
        (host, address.port or (443 if address.scheme == "https" else 80)),
        timeout=timeout,
    )
    try:
        if address.scheme == "https":
            connection.settimeout(max(0.001, deadline - time.monotonic()))
            connection = ssl.create_default_context().wrap_socket(
                connection, server_hostname=host
            )
        method = "GET" if signature is None else "POST"
        header = (
            ""
            if signature is None
            else f"X-Operator-Proof: {signature}\r\nContent-Length: 0\r\n"
        )
        connection.settimeout(max(0.001, deadline - time.monotonic()))
        connection.sendall(
            f"{method} {path} HTTP/1.1\r\nHost: {address.netloc}\r\nConnection: close\r\n{header}\r\n".encode(
                "ascii"
            )
        )
        response = bytearray()
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError("operator response deadline expired")
            connection.settimeout(remaining)
            chunk = connection.recv(2048)
            if not chunk:
                break
            response.extend(chunk)
            if len(response) > 4096:
                raise ValueError("operator response too large")
            parsed = _parse_http_response(response)
            if parsed is not None:
                return parsed
        raise ConnectionError("operator response ended early")
    finally:
        connection.close()


def request_presence(endpoint: str, key: bytes, *, timeout: float = 1) -> str | None:
    """A fresh challenge must be answered by any configured operator route."""
    nonce = secrets.token_hex(16)
    try:
        status, body = _request(endpoint, "/heartbeat/" + nonce, timeout=timeout)
        if status != 200:
            return None
        payload = json.loads(body)
        instance, signature = payload["instance"], payload["proof"]
        if not isinstance(instance, str) or not _IDENTIFIER.fullmatch(instance):
            return None
        if not isinstance(signature, str):
            return None
        if hmac.compare_digest(signature, proof(key, "alive", nonce, instance)):
            return instance
    except (OSError, ValueError, KeyError, TypeError, IndexError):
        pass
    return None


class OperatorMonitor:
    """Bounded background polling; flight loops only read the last success time."""

    def __init__(self, settings: OperatorSettings):
        self.settings = settings
        self.last_seen: float | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._key = (
            read_token(Path(settings.token_file).expanduser())
            if settings.token_file
            else None
        )

    @property
    def configured(self) -> bool:
        return bool(self.settings.endpoints and self._key)

    def alive(self, now: float | None = None) -> bool:
        return (
            self.last_seen is not None
            and 0
            <= (time.monotonic() if now is None else now) - self.last_seen
            <= self.settings.timeout
        )

    def start(self) -> None:
        if not self.configured or self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name="operator-heartbeat", daemon=True
        )
        self._thread.start()

    def _run(self) -> None:
        assert self._key is not None
        with ThreadPoolExecutor(max_workers=len(self.settings.endpoints)) as pool:
            while not self._stop.is_set():
                started = time.monotonic()
                futures = [
                    pool.submit(
                        request_presence,
                        endpoint,
                        self._key,
                        timeout=self.settings.request_timeout,
                    )
                    for endpoint in self.settings.endpoints
                ]
                for future in as_completed(futures):
                    if future.result() is not None:
                        self.last_seen = time.monotonic()
                self._stop.wait(
                    max(0, self.settings.interval - (time.monotonic() - started))
                )

    def close(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=self.settings.request_timeout + 2)
            if self._thread.is_alive():
                raise RuntimeError("operator heartbeat did not stop within its timeout")


def presence_server(bind: str, port: int, key: bytes) -> HTTPServer:
    instance = secrets.token_hex(16)

    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            self.request.settimeout(2)
            super().setup()

        def log_message(self, format, *args):
            pass

        def do_GET(self):
            nonce = self.path.removeprefix("/heartbeat/")
            if not self.path.startswith("/heartbeat/") or not _IDENTIFIER.fullmatch(
                nonce
            ):
                self.send_error(404)
                return
            body = json.dumps(
                {"instance": instance, "proof": proof(key, "alive", nonce, instance)}
            ).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_POST(self):
            nonce = self.path.removeprefix("/stop/")
            signature = self.headers.get("X-Operator-Proof", "")
            if (
                not self.path.startswith("/stop/")
                or not _IDENTIFIER.fullmatch(nonce)
                or not hmac.compare_digest(
                    signature, proof(key, "stop", nonce, instance)
                )
            ):
                self.send_error(403)
                return
            self.send_response(204)
            self.end_headers()
            threading.Thread(target=self.server.shutdown, daemon=True).start()

    return HTTPServer((bind, port), Handler)


def stop_presence(endpoint: str, key: bytes) -> bool:
    instance = request_presence(endpoint, key)
    if instance is None:
        return False
    nonce = secrets.token_hex(16)
    status, _body = _request(
        endpoint,
        "/stop/" + nonce,
        timeout=2,
        signature=proof(key, "stop", nonce, instance),
    )
    return status == 204
