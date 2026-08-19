"""Archived unit tests for the retired drone-picam entry point."""

from __future__ import annotations

import pytest

from ai_drone.cli import picam


@pytest.mark.parametrize(
    "arguments",
    [
        ["--port", "0"],
        ["--port", "65536"],
        ["--confidence", "nan"],
        ["--threshold", "inf"],
        ["--altitude", "-1"],
        ["--altitude", "1"],
        ["--fov", "180"],
    ],
)
def test_picam_rejects_invalid_numbers_before_platform_check(
    monkeypatch, arguments
) -> None:
    monkeypatch.setattr(
        picam,
        "is_raspberry_pi",
        lambda: pytest.fail("numeric validation must happen before platform access"),
    )

    with pytest.raises(SystemExit) as error:
        picam.main(arguments)

    assert error.value.code == 2


def test_picam_altitude_requires_exact_nadir_confirmation(monkeypatch) -> None:
    monkeypatch.setattr(picam, "is_raspberry_pi", lambda: False)

    with pytest.raises(SystemExit, match="Pi-only"):
        picam.main(
            [
                "--altitude",
                "1.0",
                "--confirm-nadir-geometry",
                "NADIR_CALIBRATED",
            ]
        )
