"""What the accelerometer reads while the motors are turning.

A stationary aircraft measures exactly one g however it is oriented, so the
magnitude of the accelerometer vector is self-checking: on a bench, any
sustained departure from `STANDARD_GRAVITY` is the sensor being wrong rather
than the aircraft moving.

This exists because that number decided two flights on 2026-08-21. With its
motors stopped the vehicle measured 9.79 m/s^2; with them turning at idle spin
it measured 10.65 m/s^2, while the reported vibration stayed at 0.17 and the
rangefinder stayed pinned at 0.02 m. EKF3 integrates that difference into a
vertical velocity, so the vehicle reported climbing at 4.27 m/s while sitting
on the floor, ALT_HOLD held the throttle down against the phantom climb, and
the aircraft never left the ground. The VIBE metric cannot see this: it
measures variance, and a rectified vibration shows up as a DC offset instead.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

STANDARD_GRAVITY = 9.80665

# Below this the difference is ordinary calibration error and sensor noise.
# The 2026-08-21 evening measurement was 0.85 m/s^2, four times over.
SIGNIFICANT_BIAS_MS2 = 0.20


def magnitude(x_mg: float, y_mg: float, z_mg: float) -> float:
    """Accelerometer magnitude in m/s^2 from the milligravity RAW_IMU fields."""

    scale = STANDARD_GRAVITY / 1000.0
    return math.sqrt(x_mg**2 + y_mg**2 + z_mg**2) * scale


def _mean(values: list[float]) -> float | None:
    return sum(values) / len(values) if values else None


@dataclass
class AccelBias:
    """Accelerometer magnitudes gathered with the motors stopped and turning."""

    still: list[float]
    turning: list[float]

    @classmethod
    def empty(cls) -> AccelBias:
        return cls(still=[], turning=[])

    def observe(self, x_mg: float, y_mg: float, z_mg: float, *, moving: bool) -> None:
        reading = magnitude(x_mg, y_mg, z_mg)
        if not math.isfinite(reading):
            return
        (self.turning if moving else self.still).append(reading)

    @property
    def still_mean(self) -> float | None:
        return _mean(self.still)

    @property
    def turning_mean(self) -> float | None:
        return _mean(self.turning)

    @property
    def bias(self) -> float | None:
        """How much more the sensor reads with the motors turning."""

        still, turning = self.still_mean, self.turning_mean
        if still is None or turning is None:
            return None
        return turning - still

    @property
    def is_significant(self) -> bool:
        bias = self.bias
        return bias is not None and abs(bias) >= SIGNIFICANT_BIAS_MS2

    def describe(self) -> str:
        still, turning, bias = self.still_mean, self.turning_mean, self.bias
        if bias is None:
            missing = "with the motors stopped" if still is None else "under power"
            return f"no accelerometer reading was captured {missing}"
        lines = [
            f"accelerometer magnitude: {still:.3f} m/s^2 stopped "
            f"({len(self.still)} samples), {turning:.3f} m/s^2 turning "
            f"({len(self.turning)} samples)",
            f"shift under power: {bias:+.3f} m/s^2 "
            f"against gravity at {STANDARD_GRAVITY:.3f} m/s^2",
        ]
        if self.is_significant:
            lines.append(
                "This is the fault that grounded the aircraft on 2026-08-21. EKF3 "
                "integrates it into a vertical velocity, so ALT_HOLD flies against "
                "a climb or a fall that is not happening. It is mechanical or "
                "electrical, not a software setting: look for an unbalanced or "
                "damaged propeller, a bent motor shaft, and whether the flight "
                "controller is hard-mounted to the frame."
            )
        else:
            lines.append(
                "Below the threshold that matters; the vertical estimate should "
                "hold up under power."
            )
        return "\n".join(lines)
