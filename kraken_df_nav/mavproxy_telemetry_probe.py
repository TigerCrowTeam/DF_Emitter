"""Read-only MAVProxy telemetry probe used by integration stage M1."""

from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from pymavlink import mavutil


REQUIRED = ("HEARTBEAT", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "ATTITUDE")


def run(connection: str, duration: float, heartbeat_timeout: float) -> dict:
    master = mavutil.mavlink_connection(connection, source_system=252,
                                        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    started = time.monotonic()
    heartbeat = master.wait_heartbeat(timeout=heartbeat_timeout)
    if heartbeat is None:
        raise TimeoutError("no heartbeat received through MAVProxy")
    first = time.monotonic()
    counts: Counter[str] = Counter({"HEARTBEAT": 1})
    last_seen = {"HEARTBEAT": first}
    max_gap = {name: 0.0 for name in REQUIRED}
    sample: dict[str, dict] = {"HEARTBEAT": {
        "source_system": heartbeat.get_srcSystem(),
        "source_component": heartbeat.get_srcComponent(),
        "base_mode": heartbeat.base_mode,
        "custom_mode": heartbeat.custom_mode,
    }}
    deadline = first + duration
    while time.monotonic() < deadline:
        message = master.recv_match(blocking=True, timeout=min(1.0, deadline-time.monotonic()))
        if message is None:
            continue
        name = message.get_type()
        if name == "BAD_DATA":
            continue
        now = time.monotonic()
        counts[name] += 1
        if name in REQUIRED:
            if name in last_seen:
                max_gap[name] = max(max_gap[name], now-last_seen[name])
            last_seen[name] = now
        if name == "GLOBAL_POSITION_INT":
            sample[name] = {"latitude": message.lat/1e7, "longitude": message.lon/1e7,
                            "relative_altitude_m": message.relative_alt/1000,
                            "heading_deg": None if message.hdg == 65535 else message.hdg/100,
                            "ground_speed_m_s": math.hypot(message.vx, message.vy)/100}
        elif name == "GPS_RAW_INT":
            sample[name] = {"fix_type": message.fix_type,
                            "satellites_visible": message.satellites_visible,
                            "eph_m": None if message.eph == 65535 else message.eph/100}
        elif name == "ATTITUDE":
            sample[name] = {"roll_deg": math.degrees(message.roll),
                            "pitch_deg": math.degrees(message.pitch)}
        elif name == "HEARTBEAT":
            sample[name] = {"source_system": message.get_srcSystem(),
                            "source_component": message.get_srcComponent(),
                            "base_mode": message.base_mode, "custom_mode": message.custom_mode}
    elapsed = time.monotonic()-first
    missing = [name for name in REQUIRED if not counts[name]]
    stale = [name for name in REQUIRED if name not in last_seen or time.monotonic()-last_seen[name] > 5]
    return {
        "schema_version": 1,
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "connection": connection,
        "read_only": True,
        "source_system": 252,
        "source_component": mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER,
        "heartbeat_wait_seconds": first-started,
        "observation_duration_seconds": elapsed,
        "required_messages": list(REQUIRED),
        "message_counts": dict(sorted(counts.items())),
        "required_message_rates_hz": {name: counts[name]/elapsed for name in REQUIRED},
        "required_max_gaps_seconds": max_gap,
        "latest_samples": sample,
        "missing_required_messages": missing,
        "stale_required_messages": stale,
        "status": "PASS" if not missing and not stale and elapsed >= duration-0.1 else "FAIL",
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", default="udpin:127.0.0.1:14560")
    parser.add_argument("--duration", type=float, default=300)
    parser.add_argument("--heartbeat-timeout", type=float, default=30)
    parser.add_argument("--output", default="logs/mavproxy_m1_telemetry.json")
    args = parser.parse_args()
    if args.duration <= 0:
        parser.error("--duration must be positive")
    try:
        report = run(args.connection, args.duration, args.heartbeat_timeout)
    except (OSError, TimeoutError) as exc:
        report = {"schema_version": 1, "generated_at": datetime.now(timezone.utc).isoformat(),
                  "connection": args.connection, "read_only": True, "status": "FAIL",
                  "error": str(exc)}
    path = Path(args.output)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
