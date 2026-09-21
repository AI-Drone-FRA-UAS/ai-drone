"""Verified copies of finalized recording directories over SSH."""

from __future__ import annotations

import argparse
import base64
import hashlib
import io
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import tarfile
import tempfile
import uuid
from dataclasses import dataclass, replace
from pathlib import Path, PurePosixPath
from typing import IO, Any

from ai_drone.durability import fsync_directory
from ai_drone.link.targets import remote_uv_command, resolve_deploy_target
from ai_drone.settings import load_settings

HEADER = ".ai-drone-transfer-start.json"
FINISH = ".ai-drone-transfer-finish.json"
MAX_METADATA = 8 * 1024 * 1024
CHUNK = 1024 * 1024
_WRITERS_SCRIPT = r"""
import json, os, sys
from pathlib import Path
identities = {tuple(value) for value in json.load(sys.stdin)['identities']}
writers, errors = set(), set()
for process in Path('/proc').iterdir():
    if not process.name.isdecimal():
        continue
    pid = int(process.name)
    try:
        for descriptor in (process / 'fd').iterdir():
            try:
                value = descriptor.stat()
                if (value.st_dev, value.st_ino) not in identities:
                    continue
                fields = dict(line.split(':', 1) for line in
                    (process / 'fdinfo' / descriptor.name).read_text().splitlines())
                if int(fields['flags'].strip(), 8) & os.O_ACCMODE:
                    writers.add(pid)
            except (FileNotFoundError, ProcessLookupError):
                continue
    except (FileNotFoundError, ProcessLookupError):
        continue
    except (OSError, ValueError, KeyError):
        errors.add(pid)
print(json.dumps({'writers': sorted(writers), 'errors': sorted(errors)}))
"""


def _json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, ensure_ascii=True).encode()


def _load(data: bytes) -> dict[str, Any]:
    if len(data) > MAX_METADATA:
        raise ValueError("Transfer metadata is too large")
    value = json.loads(data)
    if not isinstance(value, dict):
        raise ValueError("Expected transfer metadata object")
    return value


def _relative(name: str) -> Path:
    if not isinstance(name, str):
        raise ValueError("Dataset path must be a string")
    path = PurePosixPath(name)
    if (
        not name
        or path.is_absolute()
        or path.as_posix() != name
        or any(
            part in {".", ".."} or ":" in part or "\\" in part for part in path.parts
        )
        or name in {HEADER, FINISH}
        or "\0" in name
    ):
        raise ValueError(f"Unsafe dataset path: {name!r}")
    if os.name == "nt" and any(
        part.rstrip(". ") != part
        or any(ord(c) < 32 for c in part)
        or Path(part).is_reserved()
        for part in path.parts
    ):
        raise ValueError(f"Filename is not supported on Windows: {name!r}")
    return Path(*path.parts)


def _path(path: Path) -> Path:
    absolute = path.expanduser().absolute()
    if ".." in absolute.parts or absolute.is_symlink():
        raise ValueError("Paths must not contain symlinks or '..'")
    return absolute.parent.resolve() / absolute.name


@dataclass(frozen=True)
class Endpoint:
    host: str | None
    project: str | None
    path: str

    @classmethod
    def parse(cls, value: object) -> Endpoint:
        if not isinstance(value, dict) or set(value) != {"host", "project", "path"}:
            raise ValueError("Invalid transfer endpoint")
        host, project, path = value["host"], value["project"], value["path"]
        if not isinstance(path, str) or not path or "\0" in path:
            raise ValueError("Invalid transfer endpoint path")
        if any(
            item is not None and not isinstance(item, str) for item in (host, project)
        ):
            raise ValueError("Invalid transfer endpoint")
        if host is not None and (
            not re.fullmatch(r"[A-Za-z0-9_.@:\[\]-]+", host)
            or host.startswith("-")
            or not project
        ):
            raise ValueError("Invalid SSH endpoint")
        return cls(host, project, path)

    def document(self) -> dict[str, str | None]:
        return {"host": self.host, "project": self.project, "path": self.path}


@dataclass(frozen=True)
class FileRecord:
    size: int
    sha256: str
    stamp: tuple[int, ...] | None

    @classmethod
    def parse(cls, value: object) -> FileRecord:
        if not isinstance(value, dict):
            raise ValueError("Invalid file checksum or size")
        size, digest, stamp = value.get("size"), value.get("sha256"), value.get("stamp")
        if (
            type(size) is not int
            or size < 0
            or not isinstance(digest, str)
            or re.fullmatch(r"[0-9a-f]{64}", digest) is None
        ):
            raise ValueError("Invalid file checksum or size")
        if stamp is not None and (
            not isinstance(stamp, list)
            or len(stamp) != 5
            or any(type(item) is not int for item in stamp)
            or stamp[2] != size
        ):
            raise ValueError("Invalid source file stamp")
        return cls(size, digest, None if stamp is None else tuple(stamp))

    def document(self, *, stamps: bool = True) -> dict[str, object]:
        return {
            "size": self.size,
            "sha256": self.sha256,
            **(
                {"stamp": list(self.stamp)} if stamps and self.stamp is not None else {}
            ),
        }


@dataclass(frozen=True)
class Inventory:
    files: tuple[tuple[str, FileRecord], ...]
    directories: tuple[str, ...]

    @classmethod
    def parse(cls, value: object) -> Inventory:
        if not isinstance(value, dict):
            raise ValueError("Invalid transfer inventory")
        files, directories = value.get("files"), value.get("directories")
        if not isinstance(files, dict) or not isinstance(directories, list):
            raise ValueError("Invalid transfer inventory")
        for name in [*files, *directories]:
            _relative(name)
        if len(directories) != len(set(directories)) or set(files) & set(directories):
            raise ValueError("Duplicate transfer paths")
        return cls(
            tuple((name, FileRecord.parse(item)) for name, item in files.items()),
            tuple(sorted(directories)),
        )

    def document(self, *, stamps: bool = True) -> dict[str, Any]:
        return {
            "files": {name: item.document(stamps=stamps) for name, item in self.files},
            "directories": list(self.directories),
        }


@dataclass(frozen=True)
class Receipt:
    id: str
    source: Endpoint
    destination: Endpoint
    inventory: Inventory

    @classmethod
    def parse(cls, value: object) -> Receipt:
        if (
            not isinstance(value, dict)
            or type(value.get("version")) is not int
            or value["version"] != 1
            or set(value) != {"version", "id", "source", "destination", "snapshot"}
        ):
            raise ValueError("Unsupported transfer receipt")
        token = value["id"]
        if not isinstance(token, str) or re.fullmatch(r"[0-9a-f]{32}", token) is None:
            raise ValueError("Invalid transfer receipt ID")
        inventory = Inventory.parse(value["snapshot"])
        if any(item.stamp is None for _, item in inventory.files):
            raise ValueError("Transfer receipt requires source file stamps")
        return cls(
            token,
            Endpoint.parse(value["source"]),
            Endpoint.parse(value["destination"]),
            inventory,
        )

    def document(self) -> dict[str, Any]:
        return {
            "version": 1,
            "id": self.id,
            "source": self.source.document(),
            "destination": self.destination.document(),
            "snapshot": self.inventory.document(),
        }


def _stamp(value: os.stat_result) -> list[int]:
    return [
        value.st_dev,
        value.st_ino,
        value.st_size,
        value.st_mtime_ns,
        value.st_ctime_ns,
    ]


def _tree(root: Path) -> dict[str, list[int]]:
    result = {}

    def inaccessible(error: OSError) -> None:
        raise error

    for directory, folders, files in os.walk(
        root, onerror=inaccessible, followlinks=False
    ):
        for filename in sorted([*folders, *files]):
            path = Path(directory) / filename
            value = path.lstat()
            name = path.relative_to(root).as_posix()
            _relative(name)
            if not (stat.S_ISREG(value.st_mode) or stat.S_ISDIR(value.st_mode)):
                raise ValueError(
                    f"Only regular files and directories may be transferred: {name}"
                )
            if stat.S_ISREG(value.st_mode) and value.st_nlink != 1:
                raise ValueError(f"Hard-linked recording file: {name}")
            result[name] = _stamp(value)
    return result


def _verified_file(path: Path, stamp: list[int]) -> dict[str, Any]:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        if _stamp(os.fstat(handle.fileno())) != stamp:
            raise RuntimeError("Recording changed during verification")
        while block := handle.read(CHUNK):
            digest.update(block)
        if _stamp(os.fstat(handle.fileno())) != stamp:
            raise RuntimeError("Recording changed during verification")
    return {"size": stamp[2], "sha256": digest.hexdigest(), "stamp": stamp}


def snapshot(path: Path, *, check_writers: bool = True) -> dict[str, Any]:
    root = _path(path)
    if not root.is_dir() or root.parent == root or root == Path.home():
        raise ValueError("Choose one recording directory")
    initial = _tree(root)
    manifest = root / "manifest.json"
    if "manifest.json" not in initial or manifest.stat().st_size > MAX_METADATA:
        raise ValueError("A completed recording manifest.json is required")
    completion = _load(manifest.read_bytes())
    partial = completion.get("completed") is False and bool(completion.get("ended_utc"))
    if completion.get("completed") is not True and not partial:
        raise ValueError("Recording is not finalized; source was retained")
    files, directories = {}, []
    for name, stamp in initial.items():
        item = root / _relative(name)
        if item.is_dir():
            directories.append(name)
            continue
        files[name] = _verified_file(item, stamp)
    if _tree(root) != initial:
        raise RuntimeError("Recording changed during verification")
    result = {"files": files, "directories": sorted(directories)}
    if partial and check_writers:
        _no_writers(root, result)
    return result


def _content(value: dict[str, Any]) -> dict[str, Any]:
    return Inventory.parse(value).document(stamps=False)


def _receipt(value: dict[str, Any]) -> dict[str, Any]:
    Receipt.parse(value)
    # Exact wire bytes participate in the deletion ledger check; normalization
    # must never make a modified receipt equal to a previously recorded receipt.
    return value


def _save_receipt(receipt: dict[str, Any], directory: Path, suffix: str) -> Path:
    ledger = _path(directory) / ".ai-drone-transfers"
    ledger.mkdir(mode=0o700, exist_ok=True)
    if ledger.is_symlink() or not ledger.is_dir():
        raise ValueError("Invalid transfer ledger directory")
    path = ledger / f"{receipt['id']}.{suffix}.json"
    with path.open("xb") as handle:
        os.chmod(path, 0o600)
        handle.write(_json(receipt))
        handle.flush()
        os.fsync(handle.fileno())
    fsync_directory(ledger)
    fsync_directory(ledger.parent)
    return path


def _metadata(archive: tarfile.TarFile, name: str, value: dict[str, Any]) -> None:
    data = _json(value)
    if len(data) > MAX_METADATA:
        raise ValueError("Transfer metadata is too large")
    entry = tarfile.TarInfo(name)
    entry.size = len(data)
    archive.addfile(entry, io.BytesIO(data))


def send(
    source: dict[str, Any], destination: dict[str, Any], stream: IO[bytes]
) -> dict[str, Any]:
    source_endpoint, destination_endpoint = (
        Endpoint.parse(source),
        Endpoint.parse(destination),
    )
    source, destination = source_endpoint.document(), destination_endpoint.document()
    root = _path(Path(source_endpoint.path))
    inventory = snapshot(root)
    receipt = _receipt(
        {
            "version": 1,
            "id": uuid.uuid4().hex,
            "source": source,
            "destination": destination,
            "snapshot": inventory,
        }
    )
    with tarfile.open(fileobj=stream, mode="w|") as archive:
        _metadata(archive, HEADER, receipt)
        for name in inventory["directories"]:
            entry = tarfile.TarInfo(name)
            entry.type = tarfile.DIRTYPE
            archive.addfile(entry)
        for name, item in inventory["files"].items():
            entry = tarfile.TarInfo(name)
            entry.size = item["size"]
            with (root / _relative(name)).open("rb") as handle:
                if _stamp(os.fstat(handle.fileno())) != item["stamp"]:
                    raise RuntimeError("Source changed before copy")
                archive.addfile(entry, handle)
        if snapshot(root) != inventory:
            raise RuntimeError("Source changed during copy; source retained")
        _save_receipt(receipt, root.parent, "source")
        _metadata(archive, FINISH, receipt)
    return receipt


def _read_member(archive: tarfile.TarFile, entry: tarfile.TarInfo) -> bytes:
    if not entry.isfile() or not 0 <= entry.size <= MAX_METADATA:
        raise ValueError("Invalid archive metadata")
    handle = archive.extractfile(entry)
    if handle is None:
        raise ValueError("Missing archive metadata")
    return handle.read()


def _receive_entry(
    archive: tarfile.TarFile,
    entry: tarfile.TarInfo,
    stage: Path,
    inventory: dict[str, Any],
) -> None:
    path = stage / _relative(entry.name)
    if entry.name in inventory["directories"]:
        if not entry.isdir():
            raise ValueError("Archive directory type mismatch")
        path.mkdir(parents=True, exist_ok=True)
        return
    item = inventory["files"][entry.name]
    if not entry.isfile() or entry.size != item["size"]:
        raise ValueError("Archive file type or size mismatch")
    path.parent.mkdir(parents=True, exist_ok=True)
    reader = archive.extractfile(entry)
    if reader is None:
        raise ValueError("Missing archive file")
    digest = hashlib.sha256()
    with path.open("xb") as output:
        while block := reader.read(CHUNK):
            output.write(block)
            digest.update(block)
        output.flush()
        os.fsync(output.fileno())
    if digest.hexdigest() != item["sha256"]:
        raise ValueError("Copied file checksum mismatch")


def receive(
    stream: IO[bytes], stage: Path, source: dict[str, Any], destination: dict[str, Any]
) -> dict[str, Any]:
    with tarfile.open(fileobj=stream, mode="r|") as archive:
        first = archive.next()
        if first is None or first.name != HEADER:
            raise ValueError("Missing transfer inventory")
        receipt = _receipt(_load(_read_member(archive, first)))
        if receipt["source"] != source or receipt["destination"] != destination:
            raise ValueError("Transfer endpoint mismatch")
        inventory = receipt["snapshot"]
        if (
            sum(item["size"] for item in inventory["files"].values())
            > shutil.disk_usage(stage).free
        ):
            raise OSError("Insufficient destination storage")
        expected = set(inventory["files"]) | set(inventory["directories"])
        seen = set()
        finished = False
        while entry := archive.next():
            if entry.name == FINISH:
                if finished or _load(_read_member(archive, entry)) != receipt:
                    raise ValueError("Invalid final transfer receipt")
                finished = True
                continue
            if finished or entry.name not in expected or entry.name in seen:
                raise ValueError("Unexpected or duplicate archive entry")
            seen.add(entry.name)
            _receive_entry(archive, entry, stage, inventory)
        if not finished or seen != expected:
            raise ValueError("Incomplete recording transfer")
    if _content(snapshot(stage, check_writers=False)) != _content(inventory):
        raise ValueError("Destination checksum verification failed")
    return receipt


def _publish(stage: Path, destination: Path, receipt: dict[str, Any]) -> Path:
    destination.mkdir(mode=0o700, exist_ok=False)
    try:
        for path in sorted(
            stage.iterdir(), key=lambda item: item.name == "manifest.json"
        ):
            target = destination / path.name
            if target.exists() or target.is_symlink():
                raise FileExistsError(target)
            path.rename(target)
        for name in sorted(
            receipt["snapshot"]["directories"],
            key=lambda value: len(PurePosixPath(value).parts),
            reverse=True,
        ):
            fsync_directory(destination / _relative(name))
        fsync_directory(destination)
        fsync_directory(destination.parent)
        return _save_receipt(receipt, destination.parent, "receipt")
    except BaseException as error:
        raise RuntimeError(
            f"Publication incomplete; source retained; partial destination: {destination}"
        ) from error


def _ssh(
    endpoint: dict[str, Any], operation: str, payload: dict[str, Any] | None = None
) -> list[str]:
    if not re.fullmatch(r"[A-Za-z0-9_.@:\[\]-]+", endpoint["host"]) or endpoint[
        "host"
    ].startswith("-"):
        raise ValueError("Invalid SSH target")
    target = resolve_deploy_target()
    encoded = (
        "" if payload is None else base64.urlsafe_b64encode(_json(payload)).decode()
    )
    remote_target = replace(
        target, ssh_target=endpoint["host"], project_dir=endpoint["project"]
    )
    return remote_uv_command(remote_target, "ai_drone.transfer", [operation, encoded])


def _observed(endpoint: dict[str, Any]) -> dict[str, Any]:
    parsed = Endpoint.parse(endpoint)
    if parsed.host is None:
        return snapshot(Path(parsed.path), check_writers=False)
    result = subprocess.run(
        _ssh(endpoint, "_snapshot", endpoint), capture_output=True, check=True
    )
    return Inventory.parse(_load(result.stdout)).document()


def fetch(source: dict[str, Any], destination: dict[str, Any]) -> Path:
    target = _path(Path(destination["path"]))
    if target.exists():
        raise FileExistsError(target)
    with tempfile.TemporaryDirectory(
        prefix=".drone-copy-", dir=target.parent
    ) as temporary:
        process = subprocess.Popen(
            _ssh(source, "_send", {"source": source, "destination": destination}),
            stdout=subprocess.PIPE,
        )
        try:
            assert process.stdout is not None
            receipt = receive(process.stdout, Path(temporary), source, destination)
            if process.wait() != 0:
                raise RuntimeError("SSH source transfer failed; source retained")
            return _publish(Path(temporary), target, receipt)
        finally:
            if process.poll() is None:
                process.kill()
            process.wait()
            if process.stdout is not None:
                process.stdout.close()


def push(source: dict[str, Any], destination: dict[str, Any]) -> Path:
    process = subprocess.Popen(
        _ssh(destination, "_receive", {"source": source, "destination": destination}),
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
    )
    try:
        assert process.stdin is not None
        receipt = send(source, destination, process.stdin)
        process.stdin.close()
        process.stdin = None
        output, _ = process.communicate()
        if process.returncode or _load(output) != receipt:
            raise RuntimeError(
                "SSH destination did not verify the copy; source retained"
            )
        return _save_receipt(receipt, Path(source["path"]).parent, "receipt")
    finally:
        if process.poll() is None:
            process.kill()
        process.wait()
        if process.stdin is not None:
            process.stdin.close()
        if process.stdout is not None:
            process.stdout.close()


def _no_writers(root: Path, inventory: dict[str, Any]) -> None:
    if not sys.platform.startswith("linux"):
        raise RuntimeError("Recording-writer inspection requires Linux")
    uv = shutil.which("uv")
    if uv is None:
        raise RuntimeError("uv is required to inspect recording writers")
    command = [
        uv,
        "run",
        "--no-project",
        "--no-config",
        "--offline",
        "--python",
        "/usr/bin/python3",
        "python",
        "-I",
        "-c",
        _WRITERS_SCRIPT,
    ]
    if os.geteuid() != 0:
        command = ["sudo", "-n", *command]
    identities = [item["stamp"][:2] for item in inventory["files"].values()]
    try:
        result = subprocess.run(
            command,
            input=_json({"identities": identities}),
            capture_output=True,
            check=False,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise RuntimeError(
            "Recording-writer inspection failed or timed out; source retained"
        ) from error
    try:
        if result.returncode:
            raise RuntimeError(
                "Existing sudo permission is required for complete process inspection"
            )
        report = _load(result.stdout)
        if set(report) != {"writers", "errors"} or any(
            not isinstance(pids, list)
            or any(type(pid) is not int or pid <= 0 for pid in pids)
            for pids in report.values()
        ):
            raise ValueError("Invalid writer inspection result")
        if report["errors"]:
            raise RuntimeError(f"Unreadable process IDs: {report['errors']}")
        if report["writers"]:
            raise RuntimeError(
                f"Recording has an open writer (PIDs {report['writers']})"
            )
    except (ValueError, RuntimeError) as error:
        raise RuntimeError(
            f"Cannot clear recording writers for {root}; source retained: {error}"
        ) from error


def _remove_inventory(root: Path, inventory: dict[str, Any]) -> None:
    for name, item in inventory["files"].items():
        path = root / _relative(name)
        if _stamp(path.lstat()) != item["stamp"]:
            raise RuntimeError(f"Source file changed: {name}")
        path.unlink()
    for name in sorted(
        inventory["directories"],
        key=lambda value: len(PurePosixPath(value).parts),
        reverse=True,
    ):
        (root / _relative(name)).rmdir()
    root.rmdir()


def remove_source(
    receipt: dict[str, Any], observed_destination: dict[str, Any]
) -> None:
    supplied_receipt = _json(receipt)
    parsed = Receipt.parse(receipt)
    receipt = parsed.document()
    root = _path(Path(parsed.source.path))
    ledger = root.parent / ".ai-drone-transfers" / f"{parsed.id}.source.json"
    if ledger.parent.is_symlink() or _path(ledger).read_bytes() != supplied_receipt:
        raise ValueError("Receipt does not match the source transfer ledger")
    expected = parsed.inventory.document()
    if Inventory.parse(observed_destination).document(
        stamps=False
    ) != parsed.inventory.document(stamps=False):
        raise ValueError("Destination no longer matches the verified transfer")
    if snapshot(root) != expected:
        raise RuntimeError("Source changed since transfer; source retained")
    _no_writers(root, expected)
    quarantine = root.with_name(f".{root.name}.removing-{parsed.id}")
    if quarantine.exists() or quarantine.is_symlink():
        raise FileExistsError(quarantine)
    root.rename(quarantine)
    try:
        if snapshot(quarantine) != expected:
            raise RuntimeError("Source changed before deletion")
        _no_writers(quarantine, expected)
    except BaseException:
        if not root.exists():
            quarantine.rename(root)
        raise
    try:
        _remove_inventory(quarantine, expected)
        fsync_directory(root.parent)
    except BaseException as error:
        raise RuntimeError(
            f"Deletion incomplete; verified destination retained, remaining source: {quarantine}"
        ) from error


def _endpoint(path: str, host: str | None = None) -> dict[str, Any]:
    if host is None:
        return {"host": None, "project": None, "path": str(_path(Path(path)))}
    target = resolve_deploy_target()
    remote = PurePosixPath(path)
    if ".." in remote.parts or "\0" in path or "\\" in path:
        raise ValueError(
            "Remote dataset paths must not contain traversal or backslashes"
        )
    remote = (
        remote if remote.is_absolute() else PurePosixPath(target.project_dir) / remote
    )
    return {"host": host, "project": target.project_dir, "path": str(remote)}


def _worker_payload(operation: str, payload: object) -> dict[str, Any]:
    """Validate the entire untrusted request before filesystem or SSH effects."""
    if operation == "_snapshot":
        return Endpoint.parse(payload).document()
    if not isinstance(payload, dict):
        raise ValueError("Invalid transfer worker request")
    if operation in {"_send", "_receive"} and set(payload) == {"source", "destination"}:
        return {
            key: Endpoint.parse(payload[key]).document()
            for key in ("source", "destination")
        }
    if operation == "_remove" and set(payload) == {"receipt", "destination"}:
        return {
            "receipt": _receipt(payload["receipt"]),
            "destination": Inventory.parse(payload["destination"]).document(),
        }
    raise ValueError("Invalid transfer worker request")


def _worker(operation: str, payload: dict[str, Any]) -> None:
    payload = _worker_payload(operation, payload)
    if operation == "_snapshot":
        print(_json(snapshot(Path(payload["path"]), check_writers=False)).decode())
    elif operation == "_send":
        send(payload["source"], payload["destination"], sys.stdout.buffer)
    elif operation == "_receive":
        target = _path(Path(payload["destination"]["path"]))
        if target.exists():
            raise FileExistsError(target)
        with tempfile.TemporaryDirectory(
            prefix=".drone-copy-", dir=target.parent
        ) as temporary:
            receipt = receive(
                sys.stdin.buffer,
                Path(temporary),
                payload["source"],
                payload["destination"],
            )
            _publish(Path(temporary), target, receipt)
        print(_json(receipt).decode())
    elif operation == "_remove":
        remove_source(payload["receipt"], payload["destination"])
    else:
        raise ValueError("Unknown transfer operation")


def main(arguments: list[str] | None = None) -> int:
    arguments = sys.argv[1:] if arguments is None else arguments
    try:
        if arguments and arguments[0].startswith("_"):
            if arguments == ["_remove"]:
                payload = _load(sys.stdin.buffer.read(MAX_METADATA + 1))
            elif len(arguments) == 2:
                payload = _load(base64.urlsafe_b64decode(arguments[1]))
            else:
                raise ValueError("Invalid transfer worker request")
            _worker(arguments[0], payload)
            return 0
        parser = argparse.ArgumentParser(description=__doc__)
        commands = parser.add_subparsers(dest="command", required=True)
        for name in ("fetch", "push"):
            command = commands.add_parser(name)
            command.add_argument("dataset")
            command.add_argument(
                "--destination",
                help="new recording directory; existing directories are never overwritten",
            )
            command.add_argument("--host", required=name == "push", default=None)
        commands.add_parser("remove-source").add_argument("receipt", type=Path)
        args = parser.parse_args(arguments)
        if args.command == "remove-source":
            receipt = _receipt(_load(args.receipt.read_bytes()))
            observed = _observed(receipt["destination"])
            source = receipt["source"]
            if source["host"] is None:
                remove_source(receipt, observed)
            else:
                subprocess.run(
                    _ssh(source, "_remove"),
                    input=_json({"receipt": receipt, "destination": observed}),
                    check=True,
                )
            print("Source removed; verified destination retained.")
            return 0
        host = args.host or resolve_deploy_target().ssh_target
        args.destination = args.destination or load_settings().transfer.destination
        if not args.destination:
            parser.error(
                "--destination or transfer.destination in drone.toml is required"
            )
        source = _endpoint(args.dataset, host if args.command == "fetch" else None)
        destination = _endpoint(
            args.destination, host if args.command == "push" else None
        )
        receipt_path = (fetch if args.command == "fetch" else push)(source, destination)
        print(f"Verified copy. Source retained. Receipt: {receipt_path}")
        return 0
    except (
        OSError,
        ValueError,
        RuntimeError,
        tarfile.TarError,
        subprocess.SubprocessError,
    ) as error:
        print(f"Transfer failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
