from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.error import HTTPError
from urllib.request import Request, urlopen

import pytest

from ai_drone import operator
from ai_drone.cli import operator as command
from ai_drone.settings import OperatorSettings

KEY = bytes(range(32))


@pytest.fixture(autouse=True)
def isolated_operator_configuration(tmp_path, monkeypatch):
    config = tmp_path / "operator-config.toml"
    config.write_text("")
    monkeypatch.setenv("AI_DRONE_CONFIG", str(config))


@contextmanager
def running(server):
    thread = threading.Thread(
        target=server.serve_forever, kwargs={"poll_interval": 0.01}, daemon=True
    )
    thread.start()
    try:
        yield f"http://127.0.0.1:{server.server_port}"
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)
        assert not thread.is_alive()


def responder(callback):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, format, *args):
            pass

        def do_GET(self):
            callback(self)

    return HTTPServer(("127.0.0.1", 0), Handler)


def reply(handler, payload, *, status=200, headers=None):
    body = payload if isinstance(payload, bytes) else json.dumps(payload).encode()
    handler.send_response(status)
    for key, value in (headers or {}).items():
        handler.send_header(key, value)
    handler.send_header("Content-Length", str(len(body)))
    handler.end_headers()
    handler.wfile.write(body)


def test_loopback_presence_proves_a_fresh_nonce_and_rejects_the_wrong_key():
    with running(operator.presence_server("127.0.0.1", 0, KEY)) as endpoint:
        first = operator.request_presence(endpoint, KEY)
        assert first is not None and len(first) == 32
        assert operator.request_presence(endpoint, KEY) == first
        assert operator.request_presence(endpoint, bytes(reversed(KEY))) is None


def test_captured_response_cannot_answer_a_different_nonce():
    captured = []

    def respond(handler):
        if not captured:
            nonce = handler.path.removeprefix("/heartbeat/")
            instance = "a" * 32
            captured.append(
                {
                    "instance": instance,
                    "proof": operator.proof(KEY, "alive", nonce, instance),
                }
            )
        reply(handler, captured[0])

    with running(responder(respond)) as endpoint:
        assert operator.request_presence(endpoint, KEY) == "a" * 32
        assert operator.request_presence(endpoint, KEY) is None


@pytest.mark.parametrize(
    "payload",
    [
        b"not json",
        [],
        None,
        {},
        {"instance": "x" * 32, "proof": "invalid"},
        {"instance": "a" * 32, "proof": 123},
        {"instance": "a" * 32, "proof": "\u2603"},
        b" " * 1026,
    ],
)
def test_malformed_or_oversized_presence_is_unavailable(payload):
    with running(responder(lambda handler: reply(handler, payload))) as endpoint:
        assert operator.request_presence(endpoint, KEY) is None


def test_presence_does_not_follow_redirects():
    redirected = []
    with (
        running(
            responder(
                lambda handler: redirected.append(handler.path) or reply(handler, {})
            )
        ) as target,
        running(
            responder(
                lambda handler: reply(
                    handler, b"", status=302, headers={"Location": target}
                )
            )
        ) as endpoint,
    ):
        assert operator.request_presence(endpoint, KEY) is None
    assert not redirected


def test_stop_requires_the_key_and_current_server_instance():
    with running(operator.presence_server("127.0.0.1", 0, KEY)) as endpoint:
        nonce = "b" * 32
        request = Request(
            endpoint + "/stop/" + nonce,
            method="POST",
            headers={"X-Operator-Proof": operator.proof(KEY, "stop", nonce, "0" * 32)},
        )
        with pytest.raises(HTTPError) as error:
            urlopen(request, timeout=1)
        assert error.value.code == 403
        assert operator.request_presence(endpoint, KEY) is not None
        assert operator.stop_presence(endpoint, bytes(reversed(KEY))) is False
        assert operator.stop_presence(endpoint, KEY) is True


def test_presence_rejects_unrecognized_paths():
    with running(operator.presence_server("127.0.0.1", 0, KEY)) as endpoint:
        with pytest.raises(HTTPError) as error:
            urlopen(endpoint + "/heartbeat/not-a-valid-nonce", timeout=1)
        assert error.value.code == 404


def drip_response(handler, started):
    handler.send_response(200)
    handler.send_header("Content-Length", "1000")
    handler.end_headers()
    started.set()
    for _ in range(1000):
        try:
            handler.wfile.write(b" ")
            handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            return
        time.sleep(0.01)


def test_trickling_response_cannot_extend_the_total_request_deadline():
    started = threading.Event()
    with running(
        responder(lambda handler: drip_response(handler, started))
    ) as endpoint:
        before = time.monotonic()
        assert operator.request_presence(endpoint, KEY, timeout=0.1) is None
        elapsed = time.monotonic() - before
        assert started.is_set()
        assert elapsed < 0.7


def test_monitor_close_is_bounded_even_with_a_trickling_peer(tmp_path):
    path = tmp_path / "operator.key"
    operator.create_token(path)
    started = threading.Event()
    with running(
        responder(lambda handler: drip_response(handler, started))
    ) as endpoint:
        monitor = operator.OperatorMonitor(
            OperatorSettings(
                endpoints=(endpoint,), token_file=str(path), request_timeout=0.1
            )
        )
        monitor.start()
        assert started.wait(1)
        before = time.monotonic()
        monitor.close()
        assert time.monotonic() - before < 0.7
        assert monitor._thread is not None and not monitor._thread.is_alive()


def test_presence_ignores_ambient_proxy_configuration(monkeypatch):
    with running(operator.presence_server("127.0.0.1", 0, KEY)) as endpoint:
        monkeypatch.setenv("http_proxy", "http://127.0.0.1:1")
        monkeypatch.setenv("HTTP_PROXY", "http://127.0.0.1:1")
        monkeypatch.setenv("no_proxy", "")
        monkeypatch.setenv("NO_PROXY", "")
        assert operator.request_presence(endpoint, KEY) is not None


def test_token_is_private_and_creation_never_overwrites(tmp_path):
    path = tmp_path / "private" / "operator.key"
    operator.create_token(path)
    key = operator.read_token(path)
    assert len(key) == 32
    if os.name == "posix":
        assert path.stat().st_mode & 0o777 == 0o600
    with pytest.raises(FileExistsError):
        operator.create_token(path)
    assert operator.read_token(path) == key


@pytest.mark.parametrize("value", ["short", "g" * 64, "0" * 65])
def test_malformed_token_fails_without_exposing_its_contents(tmp_path, value):
    path = tmp_path / "key"
    path.write_text(value)
    path.chmod(0o600)
    with pytest.raises(ValueError) as error:
        operator.read_token(path)
    assert value not in str(error.value)


@pytest.mark.skipif(os.name != "posix", reason="POSIX mode and symlink checks")
def test_public_or_symlink_token_is_rejected(tmp_path):
    path = tmp_path / "key"
    operator.create_token(path)
    path.chmod(0o644)
    with pytest.raises(ValueError, match="group or others"):
        operator.read_token(path)
    path.chmod(0o600)
    alias = tmp_path / "alias"
    alias.symlink_to(path)
    with pytest.raises(ValueError, match="regular file"):
        operator.read_token(alias)


def test_monitor_defaults_are_one_hz_and_five_seconds():
    settings = OperatorSettings()
    assert (settings.interval, settings.timeout) == (1, 5)
    monitor = operator.OperatorMonitor(settings)
    monitor.start()
    assert not monitor.configured
    assert not monitor.alive()
    assert monitor._thread is None
    monitor.close()


def test_monitor_timeout_uses_monotonic_age_not_ssh_session_state():
    monitor = operator.OperatorMonitor(OperatorSettings(timeout=5))
    monitor.last_seen = 100
    assert monitor.alive(100)
    assert monitor.alive(105)
    assert not monitor.alive(105.001)
    assert not monitor.alive(99)


def test_loopback_monitor_accepts_an_alternate_route_and_stops_cleanly(tmp_path):
    path = tmp_path / "key"
    operator.create_token(path)
    with running(
        operator.presence_server("127.0.0.1", 0, operator.read_token(path))
    ) as endpoint:
        monitor = operator.OperatorMonitor(
            OperatorSettings(
                endpoints=("http://127.0.0.1:1", endpoint),
                token_file=str(path),
                interval=0.05,
                request_timeout=0.1,
            )
        )
        try:
            monitor.start()
            deadline = time.monotonic() + 2
            while not monitor.alive() and time.monotonic() < deadline:
                threading.Event().wait(0.01)
            assert monitor.configured and monitor.alive()
        finally:
            monitor.close()
        assert monitor._thread is not None and not monitor._thread.is_alive()


def test_slow_first_endpoint_does_not_delay_a_successful_alternate(
    monkeypatch, tmp_path
):
    path = tmp_path / "key"
    operator.create_token(path)
    release = threading.Event()
    answered = threading.Event()

    def request(endpoint, _key, **_kwargs):
        if endpoint.endswith("slow"):
            release.wait(2)
            return None
        answered.set()
        return "a" * 32

    monkeypatch.setattr(operator, "request_presence", request)
    monitor = operator.OperatorMonitor(
        OperatorSettings(endpoints=("http://slow", "http://fast"), token_file=str(path))
    )
    try:
        monitor.start()
        assert answered.wait(1)
        deadline = time.monotonic() + 0.25
        while not monitor.alive() and time.monotonic() < deadline:
            threading.Event().wait(0.005)
        assert monitor.alive()
    finally:
        release.set()
        monitor.close()


class Child:
    def __init__(self):
        self.pid = 99999999
        self.terminated = False
        self.waited = False

    def poll(self):
        return None

    def terminate(self):
        self.terminated = True

    def wait(self, timeout=None):
        assert timeout is not None and timeout <= 5
        self.waited = True
        return 0


def launcher(monkeypatch, tmp_path, replies):
    path = tmp_path / "operator.key"
    operator.create_token(path)
    commands = []
    child = Child()
    responses = iter(replies)
    monkeypatch.setattr(
        command, "request_presence", lambda *_args, **_kwargs: next(responses)
    )
    monkeypatch.setattr(command.shutil, "which", lambda _: "/usr/bin/uv")

    def popen(arguments, **kwargs):
        commands.append((arguments, kwargs))
        return child

    monkeypatch.setattr(command.subprocess, "Popen", popen)
    if os.name == "posix":
        monkeypatch.setattr(
            command.os, "killpg", lambda _pid, _signal: child.terminate()
        )
    else:

        def terminate_tree(arguments, **_kwargs):
            assert arguments == ["taskkill", "/PID", str(child.pid), "/T", "/F"]
            child.terminate()
            return subprocess.CompletedProcess(arguments, 0)

        monkeypatch.setattr(command.subprocess, "run", terminate_tree)
    return path, child, commands


def test_launcher_detaches_with_private_logs_and_no_key_in_arguments(
    monkeypatch, tmp_path
):
    path, child, calls = launcher(monkeypatch, tmp_path, [None, "a" * 32])
    assert command.main(["start", "--token-file", str(path)]) == 0
    ((arguments, options),) = calls
    assert arguments[:3] == ["/usr/bin/uv", "run", "--no-sync"]
    assert arguments[6:9] == ["-m", "ai_drone.cli.operator", "serve"]
    assert options["stdin"] == command.subprocess.DEVNULL
    if os.name != "nt":
        assert options["start_new_session"] is True
        assert (tmp_path / "operator.log").stat().st_mode & 0o777 == 0o600
    assert path.read_text().strip() not in " ".join(arguments)
    assert not child.terminated and not child.waited


def test_launcher_does_not_spawn_a_second_listener(monkeypatch, tmp_path):
    path, child, calls = launcher(monkeypatch, tmp_path, ["a" * 32])
    assert command.main(["start", "--token-file", str(path)]) == 0
    assert not calls
    assert not child.terminated


@pytest.mark.parametrize(
    "bind,expected", [("0.0.0.0", "127.0.0.1"), ("127.0.0.2", "127.0.0.2")]
)
def test_specific_bind_uses_the_same_address_for_status(
    monkeypatch, tmp_path, bind, expected
):
    path = tmp_path / "operator.key"
    operator.create_token(path)
    seen = []
    monkeypatch.setattr(
        command,
        "request_presence",
        lambda endpoint, _key: seen.append(endpoint) or "a" * 32,
    )
    assert command.main(["status", "--bind", bind, "--token-file", str(path)]) == 0
    assert seen == [f"http://{expected}:8787"]


def test_failed_start_terminates_and_reaps_its_child(monkeypatch, tmp_path):
    path, child, _calls = launcher(monkeypatch, tmp_path, [None])
    times = iter([0, 6])
    monkeypatch.setattr(command.time, "monotonic", lambda: next(times))
    assert command.main(["start", "--token-file", str(path)]) == 1
    assert child.terminated and child.waited


def test_detached_uv_listener_survives_the_launching_process(tmp_path):
    uv = shutil.which("uv")
    if uv is None:
        pytest.skip("uv is required to verify detached process lifetime")
    path = tmp_path / "operator.key"
    operator.create_token(path)
    with socket.socket() as available:
        available.bind(("127.0.0.1", 0))
        port = available.getsockname()[1]
    endpoint = f"http://127.0.0.1:{port}"
    key = operator.read_token(path)
    try:
        launched = subprocess.run(
            [
                uv,
                "run",
                "--offline",
                "--no-sync",
                "--python",
                sys.executable,
                "python",
                "-m",
                "ai_drone.cli.operator",
                "start",
                "--bind",
                "127.0.0.1",
                "--port",
                str(port),
                "--token-file",
                str(path),
            ],
            check=False,
            capture_output=True,
            text=True,
            timeout=10,
            env={**os.environ, "UV_OFFLINE": "true"},
        )
        assert launched.returncode == 0, launched.stderr
        assert "independently of SSH" in launched.stdout
        assert operator.request_presence(endpoint, key, timeout=0.5) is not None
    finally:
        operator.stop_presence(endpoint, key)


@pytest.mark.parametrize("port", ["0", "-1", "65536"])
def test_cli_rejects_bad_ports_before_reading_key(port, tmp_path):
    with pytest.raises(SystemExit) as error:
        command.main(
            ["start", "--port", port, "--token-file", str(tmp_path / "missing")]
        )
    assert error.value.code == 2


class ForwardChild:
    def __init__(self, returncode=None, *, ignores_terminate=False):
        self.returncode = returncode
        self.ignores_terminate = ignores_terminate
        self.terminated = False
        self.killed = False
        self.waited = False

    def poll(self):
        return self.returncode

    def terminate(self):
        self.terminated = True
        if not self.ignores_terminate:
            self.returncode = -15

    def kill(self):
        self.killed = True
        self.returncode = -9

    def wait(self, timeout=None):
        assert timeout is not None and timeout <= 1
        if self.returncode is None:
            raise subprocess.TimeoutExpired("ssh", timeout)
        self.waited = True
        return self.returncode


def test_no_forward_is_started_without_an_explicit_host(monkeypatch):
    monkeypatch.setattr(
        operator.subprocess,
        "Popen",
        lambda *_args, **_kwargs: pytest.fail("SSH started"),
    )
    with operator.operator_forward(None):
        pass


@pytest.mark.parametrize("first_failure", ["exit", "spawn"])
def test_forward_reconnects_and_reaps_every_owned_process(monkeypatch, first_failure):
    failed = ForwardChild(255)
    active = ForwardChild()
    restarted = threading.Event()
    calls = []

    def spawn(arguments, **kwargs):
        calls.append((arguments, kwargs))
        if len(calls) == 1:
            if first_failure == "spawn":
                raise OSError("temporary SSH launch failure")
            return failed
        restarted.set()
        return active

    monkeypatch.setattr(operator.subprocess, "Popen", spawn)
    with operator.operator_forward(
        "seb@seb-is-pm",
        local_host="127.0.0.2",
        local_port=8788,
        remote_port=18788,
        reconnect_interval=0.01,
    ):
        assert restarted.wait(1)
        assert not active.terminated
    assert len(calls) == 2
    assert active.terminated and active.waited
    assert not active.killed
    if first_failure == "exit":
        assert failed.waited and not failed.terminated
    arguments, options = calls[-1]
    assert arguments[-3:] == ["-R", "127.0.0.1:18788:127.0.0.2:8788", "seb@seb-is-pm"]
    assert arguments[:3] == ["ssh", "-N", "-T"]
    assert "ControlMaster=no" in arguments and "ControlPath=none" in arguments
    assert (
        "ForkAfterAuthentication=no" in arguments and "ControlPersist=no" in arguments
    )
    assert "ExitOnForwardFailure=yes" in arguments
    assert "BatchMode=yes" in arguments and "ConnectTimeout=5" in arguments
    assert "ServerAliveInterval=2" in arguments and "ServerAliveCountMax=2" in arguments
    assert options["stdin"] == subprocess.DEVNULL
    assert "shell" not in options


def test_forward_stop_interrupts_retry_delay(monkeypatch):
    failed = ForwardChild(255)
    spawned = threading.Event()
    monkeypatch.setattr(
        operator.subprocess,
        "Popen",
        lambda *_args, **_kwargs: spawned.set() or failed,
    )
    with operator.operator_forward("seb@seb-is-pm", reconnect_interval=30):
        assert spawned.wait(1)
        before = time.monotonic()
    assert time.monotonic() - before < 1
    assert failed.waited


def test_forward_kills_and_reaps_a_stuck_ssh_child(monkeypatch):
    child = ForwardChild(ignores_terminate=True)
    started = threading.Event()
    monkeypatch.setattr(
        operator.subprocess,
        "Popen",
        lambda *_args, **_kwargs: started.set() or child,
    )
    with operator.operator_forward("seb@seb-is-pm"):
        assert started.wait(1)
    assert child.terminated and child.killed and child.waited


def test_launcher_passes_forward_settings_to_its_detached_responder(
    monkeypatch, tmp_path
):
    path, _child, calls = launcher(monkeypatch, tmp_path, [None, "a" * 32])
    assert (
        command.main(
            [
                "start",
                "--token-file",
                str(path),
                "--ssh-host",
                "seb@seb-is-pm",
                "--remote-port",
                "18788",
            ]
        )
        == 0
    )
    arguments, _options = calls[0]
    assert arguments[-4:] == ["--ssh-host", "seb@seb-is-pm", "--remote-port", "18788"]


def test_launcher_uses_configured_forward_defaults(monkeypatch, tmp_path):
    path, _child, calls = launcher(monkeypatch, tmp_path, [None, "a" * 32])
    config = tmp_path / "forward.toml"
    config.write_text('[operator]\nssh_host="seb@seb-is-pm"\nremote_port=18789\n')
    monkeypatch.setenv("AI_DRONE_CONFIG", str(config))
    assert command.main(["start", "--token-file", str(path)]) == 0
    assert calls[0][0][-4:] == ["--ssh-host", "seb@seb-is-pm", "--remote-port", "18789"]


@pytest.mark.parametrize(
    "arguments",
    [
        ["--ssh-host=-oProxyCommand=bad"],
        ["--ssh-host", "seb@pi\nother"],
        ["--ssh-host", "seb@pi;bad"],
        ["--ssh-host", "user@-option"],
        ["--remote-port", "0"],
        ["--remote-port", "65536"],
    ],
)
def test_cli_rejects_invalid_forward_settings_before_opening_key(arguments, tmp_path):
    with pytest.raises(SystemExit) as error:
        command.main(["start", "--token-file", str(tmp_path / "missing"), *arguments])
    assert error.value.code == 2


def test_signed_stop_reaps_forward_owned_by_responder(monkeypatch, tmp_path):
    token = tmp_path / "operator.key"
    operator.create_token(token)
    key = operator.read_token(token)
    child = ForwardChild()
    started = threading.Event()
    monkeypatch.setattr(
        operator.subprocess,
        "Popen",
        lambda *_args, **_kwargs: started.set() or child,
    )
    server = operator.presence_server("127.0.0.1", 0, key)
    monkeypatch.setattr(command, "presence_server", lambda *_args: server)
    result = []
    thread = threading.Thread(
        target=lambda: result.append(
            command.main(
                [
                    "serve",
                    "--token-file",
                    str(token),
                    "--bind",
                    "127.0.0.1",
                    "--port",
                    str(server.server_port),
                    "--ssh-host",
                    "seb@seb-is-pm",
                ]
            )
        )
    )
    thread.start()
    try:
        assert started.wait(1)
        assert operator.stop_presence(f"http://127.0.0.1:{server.server_port}", key)
        thread.join(timeout=3)
        assert not thread.is_alive()
        assert result == [0]
        assert child.terminated and child.waited
    finally:
        if thread.is_alive():
            server.shutdown()
            thread.join(timeout=3)
