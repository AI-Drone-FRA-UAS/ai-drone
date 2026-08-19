#!/usr/bin/env python3
"""Live Rangefinder & Optical Flow (MTF-01P) Monitor & Kalibrierung.

Zeigt in Echtzeit alle Distanz- und Flow-Nachrichten des MTF-01P Sensors an.
"""

from __future__ import annotations

import time

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

PORT = "/dev/serial0"
BAUD = 115200


def main():
    print("=======================================================")
    print("  MTF-01P LiDAR / Rangefinder Live-Monitor & Kalibrierung")
    print("=======================================================")
    print("Verbinde mit Flight Controller...")

    try:
        master = mavutil.mavlink_connection(PORT, baud=BAUD)
    except Exception:
        master = mavutil.mavlink_connection(PORT, baud=921600)

    master.wait_heartbeat()
    print("Verbunden!")

    # Aktiviere die Sensordaten-Streams via MAV_CMD_SET_MESSAGE_INTERVAL (10 Hz)
    message_ids = (
        mavlink.MAVLINK_MSG_ID_RANGEFINDER,
        mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR,
        mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW,
        mavlink.MAVLINK_MSG_ID_OPTICAL_FLOW_RAD,
    )
    for msg_id in message_ids:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            100_000,  # 100,000 us = 10 Hz
            0,
            0,
            0,
            0,
            0,
        )

    # Backup Stream Request
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavlink.MAV_DATA_STREAM_EXTRA1,
        10,
        1,
    )
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavlink.MAV_DATA_STREAM_RAW_SENSORS,
        10,
        1,
    )

    print("\nLese Sensordaten aus... Hebe die Drohne mit der Hand an:")
    print("-------------------------------------------------------------------------")

    last_print = 0.0
    rf_dist = None
    ds_dist = None
    flow_dist = None
    flow_qual = None

    try:
        while True:
            msg = master.recv_match(blocking=True, timeout=0.2)
            if msg is not None:
                msg_type = msg.get_type()
                if msg_type == "RANGEFINDER":
                    rf_dist = float(msg.distance)
                elif msg_type == "DISTANCE_SENSOR":
                    orient = getattr(msg, "orientation", 25)
                    if orient == 25:
                        ds_dist = float(msg.current_distance) / 100.0
                elif msg_type == "OPTICAL_FLOW":
                    flow_dist = float(msg.ground_distance)
                    flow_qual = int(msg.quality)
                elif msg_type == "OPTICAL_FLOW_RAD":
                    flow_dist = float(msg.distance)
                    flow_qual = int(msg.quality)

            now = time.monotonic()
            if now - last_print >= 0.2:
                rf_str = f"{rf_dist * 100:5.1f} cm" if rf_dist is not None else "---"
                ds_str = f"{ds_dist * 100:5.1f} cm" if ds_dist is not None else "---"
                fl_str = (
                    f"{flow_dist * 100:5.1f} cm" if flow_dist is not None else "---"
                )
                q_str = f"Q:{flow_qual}" if flow_qual is not None else "Q:---"

                print(
                    f"\r[LiDAR] Rangefinder: {rf_str} | DistSensor: {ds_str} | FlowDist: {fl_str} ({q_str})   ",
                    end="",
                    flush=True,
                )
                last_print = now

    except KeyboardInterrupt:
        print("\n\nBeendet.")


if __name__ == "__main__":
    main()
