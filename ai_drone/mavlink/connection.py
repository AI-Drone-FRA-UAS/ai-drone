"""Consistent ArduPilot wire decoding for every maintained connection."""

from __future__ import annotations

import os
from typing import Any


def open_ardupilot_connection(device: str, **options: Any) -> Any:
    """Open with the MAVLink 2 decoder, which also accepts MAVLink 1 frames.

    Autodetection can cache V1 messages before switching to V2, then crash on
    V2 instance fields such as RAW_IMU.id. Select the complete dialect before
    receiving anything, including when pymavlink was imported earlier.
    """

    from pymavlink import mavutil

    os.environ["MAVLINK20"] = "1"
    return mavutil.mavlink_connection(device, dialect="ardupilotmega", **options)
