"""Legacy linear MAVLink takeoff/hover/land smoke test."""

from __future__ import annotations

import time

from pymavlink import mavutil
from pymavlink.dialects.v10 import ardupilotmega as mavlink

PORT = "/dev/serial0"
BAUD = 921600

TAKEOFF_ALT = 0.4
MAX_ALTITUDE = 0.5


def set_mode(master, mode_name):
    mode_mapping = master.mode_mapping()

    if mode_name not in mode_mapping:
        raise Exception(f"Mode {mode_name} not supported")

    mode_id = mode_mapping[mode_name]

    master.mav.set_mode_send(
        master.target_system,
        mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )

    print(f"Switching to {mode_name} mode...")
    time.sleep(2)


def request_position_stream(master):
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavlink.MAV_DATA_STREAM_POSITION,
        10,
        1,
    )


def get_relative_alt(master):
    while True:
        msg = master.recv_match(blocking=True, timeout=3)

        if msg is None:
            print("No MAVLink message received")
            return None

        msg_type = msg.get_type()
        print("Received:", msg_type)

        if msg_type == "GLOBAL_POSITION_INT":
            altitude = msg.relative_alt / 1000.0
            print(f"Altitude = {altitude:.2f} m")
            return altitude


def emergency_land(master):
    print("Switching to LAND mode")
    set_mode(master, "LAND")


def main():
    print("Connecting...")
    master = mavutil.mavlink_connection(PORT, baud=BAUD)

    print("Waiting for heartbeat...")
    master.wait_heartbeat()

    print("Connected")

    request_position_stream(master)

    print("Setting mode to guided...")
    set_mode(master, "GUIDED")

    print("Arming...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Armed")

    time.sleep(2)

    print(f"Taking off to {TAKEOFF_ALT} meters")

    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        TAKEOFF_ALT,
    )

    missing = 0

    while True:
        altitude = get_relative_alt(master)

        if altitude is None:
            missing += 1
            print(f"No altitude data ({missing}/10)")

            if missing >= 10:
                print("No altitude received. Landing.")
                emergency_land(master)
                break

            continue

        missing = 0

        if altitude >= TAKEOFF_ALT * 0.95:
            print("Target altitude reached")
            break

        if altitude > MAX_ALTITUDE:
            print("Altitude limit exceeded")
            emergency_land(master)
            break

        time.sleep(0.2)

    print("Hovering...")
    time.sleep(5)

    print("Landing...")
    set_mode(master, "LAND")

    print("Done")


if __name__ == "__main__":
    main()
