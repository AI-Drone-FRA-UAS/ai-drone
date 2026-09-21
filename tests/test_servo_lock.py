import os

import pytest

from ai_drone import mount


def test_single_default_namespace_has_no_fallback(tmp_path, monkeypatch):
    path = tmp_path / "unprovisioned" / "servo.lock"
    monkeypatch.setattr(mount, "_LOCK_PATH", path)
    with pytest.raises(FileNotFoundError):
        mount.ServoProcessLock()
    assert not path.parent.exists()


def test_lock_survives_access_directory_restart(tmp_path, monkeypatch):
    stable = tmp_path / "locks"
    stable.mkdir(mode=0o700)
    runtime = tmp_path / "access"
    runtime.mkdir()
    path = stable / "servo.lock"
    monkeypatch.setattr(mount, "_LOCK_PATH", path)
    first = mount.ServoProcessLock()
    inode = path.stat().st_ino
    try:
        runtime.rmdir()
        runtime.mkdir()
        with pytest.raises(RuntimeError, match="already owned"):
            mount.ServoProcessLock()
        assert path.stat().st_ino == inode
    finally:
        first.close()
    replacement = mount.ServoProcessLock()
    replacement.close()
    assert path.stat().st_ino == inode


@pytest.mark.parametrize("kind", ["symlink", "directory", "fifo", "public", "hardlink"])
def test_rejects_invalid_lock_inode(tmp_path, kind):
    path = tmp_path / "servo.lock"
    if kind == "symlink":
        path.symlink_to(tmp_path / "target")
    elif kind == "directory":
        path.mkdir()
    elif kind == "fifo":
        os.mkfifo(path, 0o600)
    else:
        path.touch(mode=0o600)
        if kind == "public":
            path.chmod(0o644)
        else:
            os.link(path, tmp_path / "alias")
    with pytest.raises(OSError):
        mount.ServoProcessLock(path)


def test_rejects_symlink_directory(tmp_path):
    real = tmp_path / "real"
    real.mkdir(mode=0o700)
    alias = tmp_path / "alias"
    alias.symlink_to(real, target_is_directory=True)
    with pytest.raises(OSError):
        mount.ServoProcessLock(alias / "servo.lock")
