"""Audit and summarize one saved MAVProxy-routed SITL run."""
from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
from importlib import metadata
from pathlib import Path
from typing import Any

from pymavlink import mavutil


REQUIRED_FILES = ("validation_report.json", "resolved_config.yaml", "preflight.json",
                  "mavproxy.log", "mav.tlog", "sitl.log", "prepare.log",
                  "telemetry_readiness.json", "application.log")


def _message_summary(path: Path) -> dict[str, Any]:
    log = mavutil.mavlink_connection(str(path))
    counts: dict[str, int] = defaultdict(int)
    first: dict[str, float] = {}
    last: dict[str, float] = {}
    heartbeat_times: list[float] = []
    while True:
        message = log.recv_match(blocking=False)
        if message is None:
            break
        key = f"{message.get_type()}:{message.get_srcSystem()}:{message.get_srcComponent()}"
        counts[key] += 1
        timestamp = getattr(message, "_timestamp", None)
        if timestamp is not None:
            first.setdefault(key, timestamp); last[key] = timestamp
            if message.get_type() == "HEARTBEAT" and message.get_srcSystem() == 1:
                heartbeat_times.append(timestamp)
    rates = {}
    for key, count in counts.items():
        span = last.get(key, 0)-first.get(key, 0)
        rates[key] = {"count": count, "span_seconds": round(span, 3),
                      "mean_rate_hz": round((count-1)/span, 3) if span > 0 else None}
    gaps = [b-a for a, b in zip(heartbeat_times, heartbeat_times[1:])]
    loss_intervals = [{"start_timestamp": a, "end_timestamp": b,
                       "duration_seconds": round(b-a, 3)}
                      for a, b in zip(heartbeat_times, heartbeat_times[1:]) if b-a > 2.5]
    return {"messages": rates, "autopilot_heartbeat": {
        "count": len(heartbeat_times), "maximum_gap_seconds": round(max(gaps), 3) if gaps else None,
        "link_loss_threshold_seconds": 2.5, "link_loss_intervals": loss_intervals}}


def audit(run_dir: Path) -> dict[str, Any]:
    failures = [f"missing {name}" for name in REQUIRED_FILES if not (run_dir/name).is_file()]
    report = json.loads((run_dir/"validation_report.json").read_text(encoding="utf-8"))
    config = report.get("resolved_config", {})
    events = ((report.get("mission_record") or {}).get("events") or [])
    waypoints = [event for event in events if event.get("type") == "waypoint"]
    required_waypoint = {"command", "target_system", "target_component",
                         "ack_result", "ack_result_code"}
    if any(not required_waypoint <= event.keys() for event in waypoints):
        failures.append("incomplete waypoint command evidence")
    for field in ("commands", "process_exit_codes", "all_children_stopped", "evidence_manifest"):
        if field not in report:
            failures.append(f"missing report field {field}")
    if report.get("status") != "PASS": failures.append("run status is not PASS")
    if not report.get("all_children_stopped"): failures.append("child cleanup not confirmed")
    mavlink = config.get("mavlink", {}); proxy = config.get("mavproxy", {})
    try:
        mavproxy_version = metadata.version("MAVProxy")
    except metadata.PackageNotFoundError:
        mavproxy_version = None
        failures.append("MAVProxy runtime version unavailable")
    result = {
        "schema_version": 1, "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": "PASS" if not failures else "FAIL", "simulation_only": True,
        "run_directory": str(run_dir.resolve()), "failures": failures,
        "versions": {"python": report.get("evidence_manifest", {}).get("python"),
                     "mavproxy": mavproxy_version,
                     "dependencies": report.get("evidence_manifest", {}).get("dependencies"),
                     "ardupilot_revision": report.get("ardupilot", {}).get("revision")},
        "endpoints": {"master": proxy.get("master_connection"),
                      "application": mavlink.get("connection"),
                      "preparation_udp_port": proxy.get("preparation_output_port"),
                      "operator_udp_port": proxy.get("operator_output_port")},
        "identities": {"mavproxy": [proxy.get("source_system"), proxy.get("source_component")],
                       "application": [mavlink.get("source_system"), mavlink.get("source_component")],
                       "target": [mavlink.get("expected_target_system"),
                                  mavlink.get("expected_target_component")]},
        "commands": waypoints, "process_exit_codes": report.get("process_exit_codes"),
        "resolved_config_saved": (run_dir/"resolved_config.yaml").is_file(),
        "message_traffic": _message_summary(run_dir/"mav.tlog"),
    }
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run_directory")
    parser.add_argument("--output", default="logs/mavproxy_m8_evidence_audit.json")
    args = parser.parse_args()
    result = audit(Path(args.run_directory))
    target = Path(args.output); target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__": raise SystemExit(main())
