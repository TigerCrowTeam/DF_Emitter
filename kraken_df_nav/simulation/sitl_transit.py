"""Perform one bounded pre-detection GUIDED transit through MAVProxy."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import time

from pymavlink import mavutil

from ..df.geometry import GeoPoint, destination, distance_m


def run(connection: str, distance: float, bearing: float, timeout: float) -> dict:
    master = mavutil.mavlink_connection(connection, source_system=255, source_component=193)
    heartbeat = master.wait_heartbeat(timeout=30)
    if heartbeat is None:
        raise TimeoutError("no heartbeat for discovery transit")
    target_system, target_component = heartbeat.get_srcSystem(), heartbeat.get_srcComponent()
    position = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=15)
    if position is None:
        raise TimeoutError("no initial position for discovery transit")
    start = GeoPoint(position.lat/1e7, position.lon/1e7, position.relative_alt/1000)
    target = destination(start, bearing, distance)
    master.mav.command_int_send(
        target_system, target_component, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1, 0, 0, float("nan"),
        int(target.latitude*1e7), int(target.longitude*1e7), start.altitude_m)
    ack = master.recv_match(type="COMMAND_ACK", blocking=True, timeout=5,
                            condition="COMMAND_ACK.command==192")
    if ack is None or ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
        raise RuntimeError("discovery transit was not accepted")
    closest = distance
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        position = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1)
        if position is None:
            continue
        current = GeoPoint(position.lat/1e7, position.lon/1e7, position.relative_alt/1000)
        closest = min(closest, distance_m(current, target))
        if closest <= 5:
            master.close()
            return {"status": "PASS", "phase": "PRE_DETECTION_TRANSIT",
                    "bearing_deg": bearing % 360, "requested_distance_m": distance,
                    "start": start.__dict__, "target": target.__dict__,
                    "closest_distance_m": closest, "target_system": target_system,
                    "target_component": target_component, "ack_result": int(ack.result)}
    master.close()
    raise TimeoutError(f"discovery transit timed out; closest distance {closest:.1f} m")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", required=True)
    parser.add_argument("--distance", type=float, default=60)
    parser.add_argument("--bearing", type=float, default=90)
    parser.add_argument("--timeout", type=float, default=60)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    try:
        if not 10 <= args.distance <= 100:
            raise ValueError("discovery distance must be in [10, 100] m")
        report = run(args.connection, args.distance, args.bearing, args.timeout)
    except (OSError, RuntimeError, TimeoutError, ValueError) as exc:
        report = {"status": "FAIL", "phase": "PRE_DETECTION_TRANSIT", "error": str(exc)}
    Path(args.output).write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__": raise SystemExit(main())
