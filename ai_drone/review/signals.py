"""Scalar telemetry mappings; validity and multi-instance sensors stay explicit."""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Signal:
    message: str
    filename: str
    field: str
    column: str
    factor: float = 1.0
    key: str | None = None
    label: str = ""
    panel: str = ""
    unit: str = ""


SIGNALS = (
    *(
        Signal(
            "ATTITUDE",
            "motion.csv",
            axis,
            f"{axis}_deg",
            180 / math.pi,
            axis,
            axis.title(),
            "Attitude",
            "°",
        )
        for axis in ("roll", "pitch", "yaw")
    ),
    *(
        Signal(
            "LOCAL_POSITION_NED",
            "motion.csv",
            field,
            f"{axis}_m",
            1,
            f"position_{axis}",
            f"EKF {axis}",
            "Estimated local position",
            "m",
        )
        for axis, field in (("north", "x"), ("east", "y"), ("down", "z"))
    ),
    *(
        Signal("LOCAL_POSITION_NED", "motion.csv", "v" + field, f"velocity_{axis}_m_s")
        for axis, field in (("north", "x"), ("east", "y"), ("down", "z"))
    ),
    Signal(
        "SCALED_PRESSURE",
        "environment.csv",
        "press_abs",
        "pressure_hpa",
        1,
        "pressure",
        "Pressure",
        "Barometer",
        "hPa",
    ),
    Signal(
        "SCALED_PRESSURE",
        "environment.csv",
        "temperature",
        "temperature_c",
        0.01,
        "temperature",
        "Barometer temperature",
        "Temperature",
        "°C",
    ),
)


def columns(filename: str) -> list[str]:
    """First declaration fixes the published CSV column order."""
    return list(
        dict.fromkeys(
            signal.column for signal in SIGNALS if signal.filename == filename
        )
    )


def for_message(message: str) -> tuple[Signal, ...]:
    return tuple(signal for signal in SIGNALS if signal.message == message)


# Keep the two wire schemas distinct: ArduPilot's historical "variance" names
# are not raw residuals or heading errors. Common ESTIMATOR_STATUS explicitly
# specifies dimensionless innovation test ratios (mavlink.io/messages/common).
ESTIMATOR_SIGNALS = (
    *(
        Signal(
            "EKF_STATUS_REPORT",
            "estimator.csv",
            field,
            field,
            1,
            "ekf_" + field,
            label,
            "Reported EKF diagnostics",
            "reported",
        )
        for field, label in (
            ("velocity_variance", "EKF velocity diagnostic"),
            ("pos_horiz_variance", "EKF horizontal-position diagnostic"),
            ("pos_vert_variance", "EKF vertical-position diagnostic"),
            ("compass_variance", "EKF compass/yaw diagnostic"),
            ("terrain_alt_variance", "EKF terrain-height diagnostic"),
            ("airspeed_variance", "EKF airspeed diagnostic"),
        )
    ),
    *(
        Signal(
            "ESTIMATOR_STATUS",
            "estimator.csv",
            field,
            field,
            1,
            "estimator_" + field,
            label,
            "Estimator innovation test ratios",
            "ratio",
        )
        for field, label in (
            ("vel_ratio", "Velocity innovation ratio"),
            ("pos_horiz_ratio", "Horizontal-position innovation ratio"),
            ("pos_vert_ratio", "Vertical-position innovation ratio"),
            ("mag_ratio", "Magnetometer innovation ratio"),
            ("hagl_ratio", "Terrain-height innovation ratio"),
            ("tas_ratio", "Airspeed innovation ratio"),
        )
    ),
)
