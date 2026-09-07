from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path

from pymavlink.dialects.v20 import ardupilotmega as mavlink


def _decode_raw_in_fresh_process(path: Path) -> dict:
    # mavutil's environment and global dialect are process-wide. Import it in
    # its default V1 state before exercising the production connection helper.
    environment = {
        name: value
        for name, value in os.environ.items()
        if name not in {"MAVLINK20", "MAVLINK09", "MAVLINK_DIALECT"}
    }
    process = subprocess.run(
        [
            sys.executable,
            "-c",
            """
import json
import sys
from pymavlink import mavutil
from ai_drone.mavlink.connection import open_ardupilot_connection

initial_version = mavutil.mavlink.WIRE_PROTOCOL_VERSION
connection = open_ardupilot_connection(
    sys.argv[1], notimestamps=True, source_system=42, source_component=190
)
messages = []
try:
    while (message := connection.recv_msg()) is not None:
        if message.get_type() != "BAD_DATA":
            messages.append({
                **message.to_dict(),
                "source_system": message.get_srcSystem(),
                "source_component": message.get_srcComponent(),
            })
    print(json.dumps({
        "initial_version": initial_version,
        "final_version": connection.WIRE_PROTOCOL_VERSION,
        "outbound_source": [connection.mav.srcSystem, connection.mav.srcComponent],
        "messages": messages,
    }))
finally:
    connection.close()
""",
            str(path),
        ],
        env=environment,
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )
    assert process.returncode == 0, process.stderr
    result = json.loads(process.stdout)
    assert result["initial_version"] == "1.0"
    assert result["final_version"] == "2.0"
    assert result["outbound_source"] == [42, 190]
    assert all(
        (message["source_system"], message["source_component"]) == (1, 1)
        for message in result["messages"]
    )
    return result


def test_mid_frame_startup_does_not_crash_raw_imu_instance_cache(
    tmp_path: Path,
) -> None:
    encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    imu = encoder.raw_imu_encode(1, 1, 1, 1, 1, 1, 1, 1, 1, 1, id=1)
    heartbeat = encoder.heartbeat_encode(
        mavlink.MAV_TYPE_QUADROTOR,
        mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,
        0,
        mavlink.MAV_STATE_STANDBY,
    )
    path = tmp_path / "mid-frame.raw"
    # This 100-byte synthetic stream reproduces the real capture's default
    # parser crash: cached V1 RAW_IMU lacks the later V2 instance dictionary.
    path.write_bytes(b"\x01" + imu.pack(encoder) * 2 + heartbeat.pack(encoder))

    result = _decode_raw_in_fresh_process(path)

    assert [message["mavpackettype"] for message in result["messages"]] == [
        "RAW_IMU",
        "RAW_IMU",
        "HEARTBEAT",
    ]
    assert [message["id"] for message in result["messages"][:2]] == [1, 1]
    assert result["messages"][-1]["autopilot"] == mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA


def test_v1_first_stream_retains_v2_only_messages(tmp_path: Path) -> None:
    encoder = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    heartbeat = encoder.heartbeat_encode(
        mavlink.MAV_TYPE_QUADROTOR,
        mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,
        0,
        mavlink.MAV_STATE_STANDBY,
    )
    obstacle = encoder.obstacle_distance_encode(
        1000, mavlink.MAV_DISTANCE_SENSOR_LASER, [100] * 72, 5, 20, 1500
    )
    path = tmp_path / "mixed.raw"
    path.write_bytes(
        heartbeat.pack(encoder, force_mavlink1=True)
        + obstacle.pack(encoder)
        + heartbeat.pack(encoder)
    )

    result = _decode_raw_in_fresh_process(path)

    assert [message["mavpackettype"] for message in result["messages"]] == [
        "HEARTBEAT",
        "OBSTACLE_DISTANCE",
        "HEARTBEAT",
    ]
    assert result["messages"][1]["distances"] == [100] * 72
