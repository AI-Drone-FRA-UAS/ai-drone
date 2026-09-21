from __future__ import annotations

import base64
import io
import json
import os
import shlex
import shutil
import subprocess
import tarfile
from pathlib import Path

import pytest

from ai_drone import transfer


@pytest.fixture
def dataset(tmp_path):
    root = tmp_path / "recording"
    root.mkdir()
    (root / "manifest.json").write_text(json.dumps({"completed": True}))
    (root / "video.h264").write_bytes(bytes(range(256)) * 100)
    (root / "reports").mkdir()
    (root / "reports" / "O'Brien $HOME; café.jsonl").write_text('{"tag": 7}\n')
    (root / "empty").mkdir()
    return root


def endpoints(dataset, destination):
    return transfer._endpoint(str(dataset)), transfer._endpoint(str(destination))


def copied(dataset, destination):
    source, target = endpoints(dataset, destination)
    stream = io.BytesIO()
    receipt = transfer.send(source, target, stream)
    stream.seek(0)
    destination.mkdir()
    assert transfer.receive(stream, destination, source, target) == receipt
    return receipt


def test_copy_round_trip_preserves_contents_and_originals(dataset, tmp_path):
    before = transfer.snapshot(dataset)
    receipt = copied(dataset, tmp_path / "copy")
    assert transfer.snapshot(dataset) == before
    assert transfer._content(transfer.snapshot(tmp_path / "copy")) == transfer._content(
        before
    )
    assert receipt["source"]["path"] == str(dataset)
    assert list((tmp_path / ".ai-drone-transfers").glob("*.source.json"))


def test_receipt_domain_freezes_nested_wire_data(dataset, tmp_path):
    receipt = copied(dataset, tmp_path / "copy")
    parsed = transfer.Receipt.parse(receipt)
    expected = parsed.document()
    receipt["snapshot"]["files"]["video.h264"]["stamp"][0] += 1
    receipt["source"]["path"] = "changed"
    assert parsed.document() == expected
    assert transfer.Receipt.parse(expected) == parsed


@pytest.mark.parametrize(
    "operation,payload",
    [
        ("_snapshot", {}),
        ("_snapshot", {"path": 3, "host": None, "project": None}),
        ("_send", {"source": [], "destination": {}}),
        ("_receive", {"source": {}}),
        ("_remove", {"receipt": {}, "destination": {}}),
        ("_unknown", {}),
    ],
)
def test_malformed_worker_request_is_a_stable_error_before_effects(
    monkeypatch, capsys, operation, payload
):
    monkeypatch.setattr(
        transfer, "snapshot", lambda *_args, **_kwargs: pytest.fail("read source")
    )
    monkeypatch.setattr(transfer, "send", lambda *_args: pytest.fail("sent data"))
    monkeypatch.setattr(
        transfer, "remove_source", lambda *_args: pytest.fail("deleted source")
    )
    encoded = base64.urlsafe_b64encode(json.dumps(payload).encode()).decode()
    assert transfer.main([operation, encoded]) == 1
    assert "Transfer failed:" in capsys.readouterr().err


@pytest.mark.parametrize(
    "stamp", [None, [], [True, 2, 25600, 4, 5], [1, 2, 7, 4, 5], "invalid"]
)
def test_malformed_receipt_stamp_cannot_authorize_source_deletion(
    dataset, tmp_path, stamp
):
    receipt = copied(dataset, tmp_path / "copy")
    receipt["snapshot"]["files"]["video.h264"]["stamp"] = stamp
    with pytest.raises(ValueError, match="stamp"):
        transfer.remove_source(receipt, transfer.snapshot(tmp_path / "copy"))
    assert dataset.exists()


def test_incomplete_recording_is_refused(dataset):
    (dataset / "manifest.json").write_text('{"completed": false}')
    with pytest.raises(ValueError, match="not finalized"):
        transfer.snapshot(dataset)


def test_finalized_partial_recording_checks_writers(dataset, monkeypatch):
    (dataset / "manifest.json").write_text(
        json.dumps(
            {
                "completed": False,
                "stop_reason": "storage_full",
                "ended_utc": "2026-09-16T12:00:00+00:00",
            }
        )
    )
    checked = []
    monkeypatch.setattr(transfer, "_no_writers", lambda path, _: checked.append(path))
    assert transfer.snapshot(dataset)["files"]["video.h264"]["size"] > 0
    assert checked == [dataset]


@pytest.mark.parametrize("link", ["file", "directory", "hardlink"])
def test_links_are_refused_without_reading_external_files(dataset, tmp_path, link):
    outside = tmp_path / "outside"
    outside.mkdir()
    (outside / "secret").write_text("private")
    if link == "hardlink":
        os.link(outside / "secret", dataset / "linked")
    else:
        try:
            (dataset / "linked").symlink_to(
                outside if link == "directory" else outside / "secret",
                target_is_directory=link == "directory",
            )
        except OSError as error:
            pytest.skip(f"symlink creation unavailable: {error}")
    with pytest.raises(ValueError, match=r"regular files|Hard-linked"):
        transfer.snapshot(dataset)
    assert (outside / "secret").read_text() == "private"


def test_symlink_root_is_refused(dataset, tmp_path):
    alias = tmp_path / "alias"
    try:
        alias.symlink_to(dataset, target_is_directory=True)
    except OSError as error:
        pytest.skip(f"symlink creation unavailable: {error}")
    with pytest.raises(ValueError, match="symlinks"):
        transfer.snapshot(alias)


def test_source_mutation_during_hashing_is_refused(dataset, monkeypatch):
    real = transfer._tree
    calls = 0

    def tree(path):
        nonlocal calls
        calls += 1
        if calls == 2:
            (dataset / "video.h264").write_bytes(b"changed")
        return real(path)

    monkeypatch.setattr(transfer, "_tree", tree)
    with pytest.raises(RuntimeError, match="changed"):
        transfer.snapshot(dataset)


def test_source_mutation_during_copy_produces_no_completion_receipt(
    dataset, tmp_path, monkeypatch
):
    source, destination = endpoints(dataset, tmp_path / "copy")
    real = transfer.snapshot
    calls = 0

    def inspect(path):
        nonlocal calls
        calls += 1
        if calls == 2:
            (dataset / "video.h264").write_bytes(b"changed")
        return real(path)

    monkeypatch.setattr(transfer, "snapshot", inspect)
    stream = io.BytesIO()
    with pytest.raises(RuntimeError, match="changed during copy"):
        transfer.send(source, destination, stream)
    assert not (tmp_path / ".ai-drone-transfers").exists()
    assert (dataset / "video.h264").read_bytes() == b"changed"


def archive_bytes(dataset, tmp_path, transform):
    source, destination = endpoints(dataset, tmp_path / "copy")
    stream = io.BytesIO()
    transfer.send(source, destination, stream)
    stream.seek(0)
    altered = io.BytesIO()
    with (
        tarfile.open(fileobj=stream, mode="r:") as original,
        tarfile.open(fileobj=altered, mode="w:") as output,
    ):
        for member in original:
            reader = original.extractfile(member) if member.isfile() else None
            data = reader.read() if reader is not None else b""
            replacement = transform(member, data)
            if replacement is not None:
                entry, payload = replacement
                entry.size = len(payload)
                output.addfile(entry, io.BytesIO(payload))
    altered.seek(0)
    return altered, source, destination


@pytest.mark.parametrize(
    "attack",
    ["traversal", "symlink", "hardlink", "wrong_hash", "missing_finish", "duplicate"],
)
def test_untrusted_archive_is_rejected(dataset, tmp_path, attack):
    def change(member, data):
        if member.name == transfer.FINISH and attack == "missing_finish":
            return None
        if member.name == "video.h264":
            if attack == "traversal":
                member.name = "../outside"
            elif attack in {"symlink", "hardlink"}:
                member.type = (
                    tarfile.SYMTYPE if attack == "symlink" else tarfile.LNKTYPE
                )
                member.linkname = "../outside"
                data = b""
            elif attack == "wrong_hash":
                data = b"X" * len(data)
            elif attack == "duplicate":
                member.name = "manifest.json"
        return member, data

    stream, source, destination = archive_bytes(dataset, tmp_path, change)
    stage = tmp_path / "stage"
    stage.mkdir()
    with pytest.raises((ValueError, tarfile.TarError)):
        transfer.receive(stream, stage, source, destination)
    assert not (tmp_path / "outside").exists()
    assert (dataset / "video.h264").exists()


def test_truncated_stream_is_refused(dataset, tmp_path):
    source, destination = endpoints(dataset, tmp_path / "copy")
    stream = io.BytesIO()
    transfer.send(source, destination, stream)
    stage = tmp_path / "stage"
    stage.mkdir()
    with pytest.raises((ValueError, tarfile.TarError)):
        transfer.receive(
            io.BytesIO(stream.getvalue()[:4096]), stage, source, destination
        )


def test_publish_refuses_existing_destination(dataset, tmp_path):
    stage = tmp_path / "stage"
    receipt = copied(dataset, stage)
    target = tmp_path / "copy"
    target.mkdir()
    (target / "keep").write_text("existing")
    with pytest.raises(FileExistsError):
        transfer._publish(stage, target, receipt)
    assert (target / "keep").read_text() == "existing"
    assert (stage / "manifest.json").exists()


def test_receipt_tampering_cannot_delete_source(dataset, tmp_path):
    receipt = copied(dataset, tmp_path / "copy")
    receipt["destination"]["path"] = str(tmp_path / "another")
    with pytest.raises(ValueError, match="source transfer ledger"):
        transfer.remove_source(receipt, transfer.snapshot(tmp_path / "copy"))
    assert dataset.exists()


def test_receipt_normalization_cannot_hide_tampering_from_ledger(dataset, tmp_path):
    receipt = copied(dataset, tmp_path / "copy")
    receipt["snapshot"]["directories"].reverse()
    with pytest.raises(ValueError, match="source transfer ledger"):
        transfer.remove_source(receipt, transfer.snapshot(tmp_path / "copy"))
    assert dataset.exists()


def test_changed_source_is_retained(dataset, tmp_path):
    receipt = copied(dataset, tmp_path / "copy")
    (dataset / "new.log").write_text("new data")
    with pytest.raises(RuntimeError, match="changed since transfer"):
        transfer.remove_source(receipt, transfer.snapshot(tmp_path / "copy"))
    assert (dataset / "new.log").read_text() == "new data"


def test_changed_destination_cannot_authorize_deletion(dataset, tmp_path):
    receipt = copied(dataset, tmp_path / "copy")
    (tmp_path / "copy" / "video.h264").write_bytes(b"corrupt")
    with pytest.raises(ValueError, match="Destination no longer matches"):
        transfer.remove_source(receipt, transfer.snapshot(tmp_path / "copy"))
    assert (dataset / "video.h264").stat().st_size == 25600


@pytest.mark.skipif(
    not transfer.sys.platform.startswith("linux"), reason="Linux deletion guard"
)
def test_open_writer_blocks_deletion(dataset, tmp_path, monkeypatch):
    receipt = copied(dataset, tmp_path / "copy")
    monkeypatch.setattr(transfer.shutil, "which", lambda _: "/usr/bin/uv")
    monkeypatch.setattr(
        transfer.subprocess,
        "run",
        lambda command, **kwargs: subprocess.CompletedProcess(
            command, 0, b'{"writers": [123], "errors": []}'
        ),
    )
    with (
        (dataset / "video.h264").open("ab"),
        pytest.raises(RuntimeError, match="open writer"),
    ):
        transfer.remove_source(receipt, transfer.snapshot(tmp_path / "copy"))
    assert dataset.exists()


def test_verified_explicit_delete_only_removes_exact_source(
    dataset, tmp_path, monkeypatch
):
    receipt = copied(dataset, tmp_path / "copy")
    monkeypatch.setattr(transfer, "_no_writers", lambda *_: None)
    (tmp_path / "unrelated").write_text("keep")
    transfer.remove_source(receipt, transfer.snapshot(tmp_path / "copy"))
    assert not dataset.exists()
    assert (tmp_path / "copy" / "manifest.json").exists()
    assert (tmp_path / "unrelated").read_text() == "keep"


def test_mutation_after_quarantine_restores_source(dataset, tmp_path, monkeypatch):
    receipt = copied(dataset, tmp_path / "copy")
    calls = 0

    def check(root, _inventory):
        nonlocal calls
        calls += 1
        if calls == 2:
            raise RuntimeError("writer appeared")

    monkeypatch.setattr(transfer, "_no_writers", check)
    with pytest.raises(RuntimeError, match="writer appeared"):
        transfer.remove_source(receipt, transfer.snapshot(tmp_path / "copy"))
    assert dataset.exists()
    assert not list(tmp_path.glob(".recording.removing-*"))


def test_unlink_failure_reports_quarantine_and_keeps_destination(
    dataset, tmp_path, monkeypatch
):
    receipt = copied(dataset, tmp_path / "copy")
    monkeypatch.setattr(transfer, "_no_writers", lambda *_: None)
    original = Path.unlink

    def unlink(path, *args, **kwargs):
        if ".removing-" in str(path) and path.name == "video.h264":
            raise OSError("simulated filesystem failure")
        return original(path, *args, **kwargs)

    monkeypatch.setattr(Path, "unlink", unlink)
    with pytest.raises(RuntimeError, match="remaining source:"):
        transfer.remove_source(receipt, transfer.snapshot(tmp_path / "copy"))
    assert list(tmp_path.glob(".recording.removing-*/video.h264"))
    assert (tmp_path / "copy" / "video.h264").stat().st_size == 25600


def test_ssh_command_never_interpolates_dataset_shell_characters(dataset, monkeypatch):
    endpoint = {
        "host": "seb@drone",
        "project": "/home/seb/project's copy",
        "path": str(dataset),
    }
    payload = {"path": "foo'; touch /tmp/not-executed; echo '"}
    command = transfer._ssh(endpoint, "_snapshot", payload)
    assert command[0] == "ssh"
    remote = shlex.split(command[-1])
    assert remote[:3] == ["cd", endpoint["project"], "&&"]
    assert payload["path"] not in command[-1]
    with pytest.raises(ValueError, match="SSH target"):
        transfer._ssh(
            {**endpoint, "host": "-oProxyCommand=unexpected"}, "_snapshot", payload
        )


class FakeSSH:
    def __init__(self, data, code=0):
        self.stdout = io.BytesIO(data)
        self.returncode = code

    def wait(self):
        return self.returncode

    def poll(self):
        return self.returncode

    def kill(self):
        self.returncode = -9


@pytest.mark.parametrize("code", [0, 1])
def test_fetch_publishes_only_after_ssh_success(dataset, tmp_path, monkeypatch, code):
    source, destination = endpoints(dataset, tmp_path / "copy")
    source["host"] = "seb@drone"
    source["project"] = str(tmp_path)
    stream = io.BytesIO()
    transfer.send(source, destination, stream)
    monkeypatch.setattr(
        transfer.subprocess, "Popen", lambda *_a, **_k: FakeSSH(stream.getvalue(), code)
    )
    if code:
        with pytest.raises(RuntimeError, match="SSH source transfer failed"):
            transfer.fetch(source, destination)
        assert not (tmp_path / "copy").exists()
    else:
        receipt_path = transfer.fetch(source, destination)
        assert receipt_path.is_file()
        assert (tmp_path / "copy" / "video.h264").exists()
    assert dataset.exists()
    assert not list(tmp_path.glob(".drone-copy-*"))


def test_existing_fetch_destination_does_not_start_ssh(dataset, tmp_path, monkeypatch):
    source, target = endpoints(dataset, dataset)
    monkeypatch.setattr(
        transfer.subprocess, "Popen", lambda *_a, **_k: pytest.fail("SSH started")
    )
    with pytest.raises(FileExistsError):
        transfer.fetch(source, target)


def test_remove_cli_rehashes_destination_before_contacting_source(
    dataset, tmp_path, monkeypatch
):
    receipt = copied(dataset, tmp_path / "copy")
    receipt_path = tmp_path / "receipt.json"
    receipt_path.write_text(json.dumps(receipt))
    (tmp_path / "copy" / "video.h264").write_bytes(b"corrupt")
    monkeypatch.setattr(
        transfer.subprocess, "run", lambda *_a, **_k: pytest.fail("SSH executed")
    )
    assert transfer.main(["remove-source", str(receipt_path)]) == 1
    assert dataset.exists()


def test_push_uses_a_real_stream_and_peer_verification(dataset, tmp_path, monkeypatch):
    peer = tmp_path / "peer"
    peer.mkdir()
    source = transfer._endpoint(str(dataset))
    destination = {
        "host": "seb@drone",
        "project": str(Path.cwd()),
        "path": str(peer / "received"),
    }
    real_popen = subprocess.Popen
    uv = shutil.which("uv")
    assert uv is not None

    def local_peer(command, **kwargs):
        arguments = shlex.split(command[-1])
        assert arguments[-2] == "_receive"
        return real_popen(
            [
                uv,
                "run",
                "--no-sync",
                "python",
                "-m",
                "ai_drone.transfer",
                "_receive",
                arguments[-1],
            ],
            **kwargs,
        )

    monkeypatch.setattr(transfer.subprocess, "Popen", local_peer)
    receipt_path = transfer.push(source, destination)
    assert receipt_path.is_file()
    assert transfer._content(transfer.snapshot(peer / "received")) == transfer._content(
        transfer.snapshot(dataset)
    )
    assert dataset.exists()


def test_failed_push_keeps_source_and_issues_no_success_receipt(
    dataset, tmp_path, monkeypatch
):
    class RefusingPeer:
        def __init__(self):
            self.stdin = io.BytesIO()
            self.stdout = io.BytesIO()
            self.returncode = 1

        def communicate(self):
            return b"", None

        def poll(self):
            return self.returncode

        def wait(self):
            return self.returncode

    monkeypatch.setattr(transfer.subprocess, "Popen", lambda *_a, **_k: RefusingPeer())
    source, destination = endpoints(dataset, tmp_path / "copy")
    destination.update(host="seb@drone", project=str(tmp_path))
    with pytest.raises(RuntimeError, match="did not verify"):
        transfer.push(source, destination)
    assert dataset.exists()
    assert not list((tmp_path / ".ai-drone-transfers").glob("*.receipt.json"))


def test_normal_parent_symlink_is_resolved_without_accepting_dataset_symlinks(
    dataset, tmp_path
):
    alias = tmp_path / "system-alias"
    try:
        alias.symlink_to(tmp_path, target_is_directory=True)
    except OSError as error:
        pytest.skip(f"directory symlinks unavailable: {error}")
    assert transfer._path(alias / "recording") == dataset


def test_partial_destination_verification_needs_no_linux_process_check(
    dataset, tmp_path, monkeypatch
):
    (dataset / "manifest.json").write_text(
        json.dumps({"completed": False, "ended_utc": "2026-09-16T12:00:00Z"})
    )
    monkeypatch.setattr(
        transfer, "_no_writers", lambda *_: pytest.fail("destination scanned processes")
    )
    assert transfer.snapshot(dataset, check_writers=False)["files"]
    assert transfer._observed(transfer._endpoint(str(dataset)))["files"]


def test_publication_failure_retains_source_and_reports_partial_destination(
    dataset, tmp_path, monkeypatch
):
    stage = tmp_path / "stage"
    receipt = copied(dataset, stage)
    destination = tmp_path / "copy"
    rename = Path.rename

    def fail(path, target):
        if path.name == "manifest.json":
            raise OSError("simulated publication failure")
        return rename(path, target)

    monkeypatch.setattr(Path, "rename", fail)
    with pytest.raises(RuntimeError, match="partial destination:"):
        transfer._publish(stage, destination, receipt)
    assert dataset.exists()
    assert (stage / "manifest.json").exists()
    assert not (destination / "manifest.json").exists()
    assert not list((tmp_path / ".ai-drone-transfers").glob("*.receipt.json"))


def test_raced_destination_is_not_overwritten(dataset, tmp_path, monkeypatch):
    stage = tmp_path / "stage"
    receipt = copied(dataset, stage)
    destination = tmp_path / "copy"
    mkdir = Path.mkdir

    def raced(path, *args, **kwargs):
        if path == destination:
            mkdir(path)
            (path / "other-writer").write_text("keep")
        return mkdir(path, *args, **kwargs)

    monkeypatch.setattr(Path, "mkdir", raced)
    with pytest.raises(FileExistsError):
        transfer._publish(stage, destination, receipt)
    assert (destination / "other-writer").read_text() == "keep"
    assert (stage / "video.h264").exists()


def test_publication_persists_every_directory_before_receipt(
    dataset, tmp_path, monkeypatch
):
    (dataset / "reports" / "nested").mkdir()
    (dataset / "reports" / "nested" / "index.html").write_text("report")
    stage = tmp_path / "stage"
    receipt = copied(dataset, stage)
    destination = tmp_path / "copy"
    persisted = []
    monkeypatch.setattr(transfer, "fsync_directory", persisted.append)
    save = transfer._save_receipt

    def save_after_persistence(value, directory, suffix):
        assert set(persisted) == {
            destination / "reports" / "nested",
            destination / "reports",
            destination / "empty",
            destination,
            destination.parent,
        }
        assert persisted.index(destination / "reports" / "nested") < persisted.index(
            destination / "reports"
        )
        assert persisted.index(destination / "reports") < persisted.index(destination)
        return save(value, directory, suffix)

    monkeypatch.setattr(transfer, "_save_receipt", save_after_persistence)
    receipt_path = transfer._publish(stage, destination, receipt)
    assert receipt_path.is_file()
    assert persisted[-2:] == [receipt_path.parent, receipt_path.parent.parent]


def test_new_ledger_is_persisted_in_its_parent(dataset, tmp_path, monkeypatch):
    receipt = copied(dataset, tmp_path / "copy")
    destination_parent = tmp_path / "new-parent"
    destination_parent.mkdir()
    persisted = []
    monkeypatch.setattr(transfer, "fsync_directory", persisted.append)
    path = transfer._save_receipt(receipt, destination_parent, "receipt")
    assert path.is_file()
    assert persisted == [path.parent, destination_parent]


@pytest.mark.parametrize("uid", [0, 1000])
def test_writer_inspection_only_elevates_fixed_read_only_scanner(
    dataset, monkeypatch, uid
):
    inventory = transfer.snapshot(dataset)
    monkeypatch.setattr(transfer.sys, "platform", "linux")
    monkeypatch.setattr(transfer.os, "geteuid", lambda: uid, raising=False)
    monkeypatch.setattr(transfer.shutil, "which", lambda _: "/usr/bin/uv")
    calls = []

    def run(command, **options):
        calls.append((command, options))
        return subprocess.CompletedProcess(command, 0, b'{"writers": [], "errors": []}')

    monkeypatch.setattr(transfer.subprocess, "run", run)
    transfer._no_writers(dataset, inventory)
    command, options = calls[0]
    assert command == (["sudo", "-n"] if uid else []) + [
        "/usr/bin/uv",
        "run",
        "--no-project",
        "--no-config",
        "--offline",
        "--python",
        "/usr/bin/python3",
        "python",
        "-I",
        "-c",
        transfer._WRITERS_SCRIPT,
    ]
    assert json.loads(options["input"]) == {
        "identities": [item["stamp"][:2] for item in inventory["files"].values()]
    }
    assert options["timeout"] == 10
    assert options["capture_output"] is True


@pytest.mark.parametrize(
    ("code", "output", "message"),
    [
        (1, b"", "sudo permission"),
        (0, b"not json", "source retained"),
        (0, b'{"writers": [], "errors": [456]}', "Unreadable process IDs"),
        (0, b'{"writers": [123], "errors": []}', "open writer"),
        (
            0,
            b'{"writers": [], "errors": [], "unknown": true}',
            "Invalid writer inspection",
        ),
    ],
)
def test_writer_inspection_fails_closed(dataset, monkeypatch, code, output, message):
    inventory = transfer.snapshot(dataset)
    monkeypatch.setattr(transfer.sys, "platform", "linux")
    monkeypatch.setattr(transfer.os, "geteuid", lambda: 1000, raising=False)
    monkeypatch.setattr(transfer.shutil, "which", lambda _: "/usr/bin/uv")
    monkeypatch.setattr(
        transfer.subprocess,
        "run",
        lambda command, **kwargs: subprocess.CompletedProcess(command, code, output),
    )
    with pytest.raises(RuntimeError, match=message):
        transfer._no_writers(dataset, inventory)
    assert dataset.exists()


def test_writer_inspection_timeout_retains_source(dataset, monkeypatch):
    inventory = transfer.snapshot(dataset)
    monkeypatch.setattr(transfer.sys, "platform", "linux")
    monkeypatch.setattr(transfer.os, "geteuid", lambda: 1000, raising=False)
    monkeypatch.setattr(transfer.shutil, "which", lambda _: "/usr/bin/uv")

    def timeout(command, **kwargs):
        raise subprocess.TimeoutExpired(command, 10)

    monkeypatch.setattr(transfer.subprocess, "run", timeout)
    with pytest.raises(RuntimeError, match="timed out; source retained"):
        transfer._no_writers(dataset, inventory)


@pytest.mark.skipif(
    not transfer.sys.platform.startswith("linux"), reason="Linux /proc scanner"
)
def test_scanner_reports_only_writer_and_unreadable_pids(
    dataset, tmp_path, monkeypatch, capsys
):
    process = tmp_path / "123"
    (process / "fd").mkdir(parents=True)
    (process / "fdinfo").mkdir()
    try:
        (process / "fd" / "4").symlink_to(dataset / "video.h264")
    except OSError as error:
        pytest.skip(f"symlink creation unavailable: {error}")
    (process / "fdinfo" / "4").write_text("flags:\t0100002\n")
    (process / "cmdline").write_text("sensitive argument must never be read")
    unreadable = tmp_path / "456"
    original = Path.iterdir

    def directories(path):
        if str(path) == "/proc":
            return iter([process, unreadable])
        if path == unreadable / "fd":
            raise PermissionError("protected process")
        return original(path)

    monkeypatch.setattr(Path, "iterdir", directories)
    value = (dataset / "video.h264").stat()
    monkeypatch.setattr(
        transfer.sys,
        "stdin",
        io.StringIO(json.dumps({"identities": [[value.st_dev, value.st_ino]]})),
    )
    exec(transfer._WRITERS_SCRIPT, {})
    assert json.loads(capsys.readouterr().out) == {"writers": [123], "errors": [456]}
