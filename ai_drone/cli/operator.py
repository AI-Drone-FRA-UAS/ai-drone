"""Start or stop operator presence independently of terminal sessions."""

from __future__ import annotations

import argparse
import ipaddress
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

from ai_drone.operator import (
    create_token,
    operator_forward,
    presence_server,
    read_token,
    request_presence,
    stop_presence,
)
from ai_drone.settings import load_settings, validate_operator_forward


def _serve(args: argparse.Namespace, key: bytes, local_host: str) -> None:
    with (
        presence_server(args.bind, args.port, key) as server,
        operator_forward(
            args.ssh_host,
            local_host=local_host,
            local_port=args.port,
            remote_port=args.remote_port,
        ),
    ):
        previous = None
        if threading.current_thread() is threading.main_thread():
            previous = signal.signal(
                signal.SIGTERM,
                lambda *_: threading.Thread(
                    target=server.shutdown, daemon=True
                ).start(),
            )
        try:
            server.serve_forever(poll_interval=0.2)
        finally:
            if previous is not None:
                signal.signal(signal.SIGTERM, previous)


def _arguments(arguments: list[str] | None) -> argparse.Namespace:
    settings = load_settings().operator
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "action", choices=("keygen", "start", "serve", "status", "stop")
    )
    parser.add_argument(
        "--token-file",
        type=Path,
        default=Path(
            settings.token_file or Path.home() / ".config/ai-drone/operator.key"
        ),
    )
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8787)
    parser.add_argument("--ssh-host", default=settings.ssh_host)
    parser.add_argument("--remote-port", type=int, default=settings.remote_port)
    args = parser.parse_args(arguments)
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    try:
        validate_operator_forward(args.ssh_host, args.remote_port)
    except ValueError as error:
        parser.error(str(error))
    return args


def _stop_failed_launch(process: subprocess.Popen) -> None:
    """Terminate the owned process tree when its responder never becomes ready."""
    if process.poll() is not None:
        return
    if os.name == "posix":
        os.killpg(process.pid, signal.SIGTERM)
    else:
        subprocess.run(
            ["taskkill", "/PID", str(process.pid), "/T", "/F"],
            check=False,
            capture_output=True,
        )
    try:
        process.wait(timeout=2)
    except subprocess.TimeoutExpired:
        if os.name == "posix":
            os.killpg(process.pid, signal.SIGKILL)
        else:
            process.kill()
        process.wait(timeout=2)


def _launch(
    args: argparse.Namespace, token_file: Path, endpoint: str, key: bytes
) -> int:
    """Start the detached responder and require an authenticated local response."""
    uv = shutil.which("uv")
    if uv is None:
        raise RuntimeError("uv is required to start the operator heartbeat")
    command = [
        uv,
        "run",
        "--no-sync",
        "--python",
        sys.executable,
        "python",
        "-m",
        "ai_drone.cli.operator",
        "serve",
        "--token-file",
        str(token_file),
        "--bind",
        args.bind,
        "--port",
        str(args.port),
    ]
    if args.ssh_host:
        command.extend(
            ["--ssh-host", args.ssh_host, "--remote-port", str(args.remote_port)]
        )
    log = token_file.parent / "operator.log"
    with os.fdopen(
        os.open(log, os.O_CREAT | os.O_APPEND | os.O_WRONLY, 0o600), "ab"
    ) as output:
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=output,
            stderr=output,
            start_new_session=os.name != "nt",
            creationflags=(
                subprocess.DETACHED_PROCESS | subprocess.CREATE_NEW_PROCESS_GROUP
            )
            if os.name == "nt"
            else 0,
        )
    deadline = time.monotonic() + 5
    while process.poll() is None and time.monotonic() < deadline:
        if request_presence(endpoint, key, timeout=0.2) is not None:
            print(
                f"Operator heartbeat started independently of SSH on port {args.port}."
            )
            if args.ssh_host:
                print(
                    f"SSH forwarding requested; see {log} and verify the Pi's operator_alive status."
                )
            return 0
        time.sleep(0.1)
    _stop_failed_launch(process)
    raise RuntimeError(f"operator heartbeat did not start; see {log}")


def main(arguments: list[str] | None = None) -> int:
    args = _arguments(arguments)
    token_file = args.token_file.expanduser().absolute()
    try:
        if args.action == "keygen":
            create_token(token_file)
            print(f"Operator key saved privately: {token_file}")
            return 0
        key = read_token(token_file)
        bind = ipaddress.IPv4Address(args.bind)
        local_host = "127.0.0.1" if bind.is_unspecified else str(bind)
        endpoint = f"http://{local_host}:{args.port}"
        if args.action == "serve":
            _serve(args, key, local_host)
            return 0
        if args.action == "stop":
            stopped = stop_presence(endpoint, key)
            print(
                "Operator heartbeat stopped."
                if stopped
                else "Operator heartbeat is not running with this key."
            )
            return 0 if stopped else 1
        if request_presence(endpoint, key) is not None:
            print(f"Operator heartbeat is running on port {args.port}.")
            if args.ssh_host:
                print(
                    "Local status does not verify SSH forwarding; see operator.log. Restart to change forwarding options."
                )
            return 0
        if args.action == "status":
            print("Operator heartbeat is unavailable.")
            return 1
        return _launch(args, token_file, endpoint, key)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"Operator heartbeat: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
