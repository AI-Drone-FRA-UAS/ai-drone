from __future__ import annotations

from hashlib import sha256
from pathlib import Path


def flight_code_sha256(root: Path | None = None) -> str:
    root = root or Path(__file__).parents[2]
    digest = sha256()
    for path in sorted((root / "ai_drone").rglob("*.py")):
        digest.update(path.relative_to(root).as_posix().encode())
        digest.update(b"\0")
        digest.update(path.read_bytes())
    return digest.hexdigest()
