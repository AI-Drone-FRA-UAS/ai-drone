"""Download all DataFlash logs from the ArduPilot flight controller over MAVLink."""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path
from typing import Any

from ai_drone.mavlink.connection import open_ardupilot_connection
from ai_drone.mavlink.devices import (
    resolve_mavlink_endpoint,
)


def list_dataflash_logs(
    connection: Any, timeout_s: float = 6.0
) -> list[dict[str, Any]]:
    target_system = int(connection.target_system)
    target_component = int(connection.target_component)
    connection.mav.log_request_list_send(target_system, target_component, 0, 0xFFFF)
    deadline = time.monotonic() + timeout_s

    logs: list[dict[str, Any]] = []
    seen_ids: set[int] = set()

    while (remaining := deadline - time.monotonic()) > 0:
        msg = connection.recv_match(type="LOG_ENTRY", blocking=True, timeout=remaining)
        if msg is None:
            break
        if int(msg.num_logs) == 0:
            break
        log_id = int(msg.id)
        if log_id in seen_ids:
            continue
        seen_ids.add(log_id)
        logs.append(
            {
                "id": log_id,
                "size": int(msg.size),
                "time_utc": int(msg.time_utc),
                "last_log_num": int(msg.last_log_num),
                "num_logs": int(msg.num_logs),
            }
        )
        if log_id == int(msg.last_log_num):
            break

    logs.sort(key=lambda item: item["id"])
    return logs


def download_single_log(
    connection: Any,
    log_id: int,
    log_size: int,
    output_path: Path,
    chunk_size: int = 45000,
    max_retries_per_chunk: int = 5,
) -> bool:
    target_system = int(connection.target_system)
    target_component = int(connection.target_component)

    data = bytearray(log_size)
    ofs = 0
    start_time = time.monotonic()

    print(f"Downloading Log #{log_id} ({log_size:,} bytes)...")

    while ofs < log_size:
        req_len = min(chunk_size, log_size - ofs)
        retries = 0
        connection.mav.log_request_data_send(
            target_system, target_component, log_id, ofs, req_len
        )

        chunk_received = 0
        while chunk_received < req_len:
            msg = connection.recv_match(type="LOG_DATA", blocking=True, timeout=2.0)
            if not msg:
                retries += 1
                if retries > max_retries_per_chunk:
                    print(
                        f"Failed to receive chunk at ofs {ofs + chunk_received} after {retries} retries."
                    )
                    return False
                missing_ofs = ofs + chunk_received
                missing_len = req_len - chunk_received
                print(
                    f"  Timeout: retrying ofs {missing_ofs}, len {missing_len} (attempt {retries}/{max_retries_per_chunk})..."
                )
                connection.mav.log_request_data_send(
                    target_system, target_component, log_id, missing_ofs, missing_len
                )
                continue

            if int(msg.id) != log_id:
                continue

            msg_ofs = int(msg.ofs)
            msg_count = int(msg.count)
            if msg_ofs >= ofs and msg_ofs + msg_count <= ofs + req_len:
                data[msg_ofs : msg_ofs + msg_count] = bytes(msg.data[:msg_count])
                chunk_received = max(chunk_received, msg_ofs + msg_count - ofs)

        ofs += req_len
        pct = (ofs / log_size) * 100 if log_size else 100
        elapsed = time.monotonic() - start_time
        speed = ofs / elapsed / 1024 if elapsed > 0 else 0
        print(
            f"  Progress: {ofs:,}/{log_size:,} bytes ({pct:.1f}%) at {speed:.1f} KB/s",
            flush=True,
        )

    connection.mav.log_request_end_send(target_system, target_component)
    output_path.write_bytes(data)
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Pull DataFlash logs from flight controller."
    )
    parser.add_argument("--device", default=None, help="Serial device or endpoint")
    parser.add_argument(
        "--dest", default="artifacts/drone-logs", help="Destination folder"
    )
    parser.add_argument("--baud", type=int, default=115200)
    args = parser.parse_args()

    dest_dir = Path(args.dest)
    dest_dir.mkdir(parents=True, exist_ok=True)

    endpoint = resolve_mavlink_endpoint(args.device, prefer_stable=True)
    print(f"Connecting to FC at {endpoint}...")
    connection = open_ardupilot_connection(endpoint, baud=args.baud)
    hb = connection.wait_heartbeat(timeout=5)
    if not hb:
        print("Error: No heartbeat received from flight controller.")
        return 1

    print("Querying available DataFlash logs...")
    logs = list_dataflash_logs(connection)
    if not logs:
        print("No DataFlash logs found on flight controller.")
        connection.close()
        return 0

    print(f"Found {len(logs)} log(s) on flight controller:")
    for entry in logs:
        print(
            f"  Log #{entry['id']}: {entry['size']:,} bytes (UTC time: {entry['time_utc']})"
        )

    manifest: list[dict[str, Any]] = []

    try:
        for entry in logs:
            log_id = entry["id"]
            log_size = entry["size"]
            out_file = dest_dir / f"dataflash-log-{log_id}.bin"

            success = download_single_log(connection, log_id, log_size, out_file)
            if not success:
                print(f"Error downloading log #{log_id}")
                return 1

            sha256 = hashlib.sha256(out_file.read_bytes()).hexdigest()
            print(f"Saved: {out_file} (SHA256: {sha256[:16]}...)")
            manifest.append(
                {
                    "id": log_id,
                    "size_bytes": log_size,
                    "time_utc": entry["time_utc"],
                    "file": out_file.name,
                    "sha256": sha256,
                }
            )
    finally:
        connection.close()

    manifest_path = dest_dir / "manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"All {len(logs)} DataFlash logs pulled successfully to {dest_dir}/")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
