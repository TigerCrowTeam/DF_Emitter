"""Audit final S7 and S8 evidence against the Stage S10 schema."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any


SITL_SCENARIOS = ("nominal", "kraken_loss", "mavlink_loss", "poor_geometry",
                  "multipath", "operator_interrupt")


def _require(value: Any, label: str, failures: list[str]) -> None:
    if value is None or value == "" or value == [] or value == {}:
        failures.append(label)


def audit_monte_carlo(path: Path) -> dict[str, Any]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    failures: list[str] = []
    manifest = payload.get("evidence_manifest", {})
    _require(manifest.get("source", {}).get("archive_identifier"), "source archive identifier", failures)
    _require(manifest.get("platform"), "platform version", failures)
    _require(manifest.get("python"), "Python version", failures)
    _require(manifest.get("dependencies"), "dependency versions", failures)
    trials = payload.get("trials", [])
    if len(trials) < 500:
        failures.append("at least 500 Monte Carlo trials")
    required_trial = ("resolved_config", "scenario_name", "classification", "seed",
                      "evidence_manifest_id", "actual_error_m", "state")
    required_audit = ("simulated_emitter_truth_validation_only", "kraken_generation_records",
                      "vehicle_telemetry_and_command_records", "mission_events",
                      "estimator_inputs", "final_scoring")
    for index, trial in enumerate(trials):
        for field in required_trial:
            if field not in trial:
                failures.append(f"trial {index}: {field}")
        audit = trial.get("audit", {})
        for field in required_audit:
            if field not in audit:
                failures.append(f"trial {index} audit: {field}")
        if trial.get("evidence_manifest_id") != manifest.get("source", {}).get("archive_identifier"):
            failures.append(f"trial {index}: source manifest reference mismatch")
        records = audit.get("kraken_generation_records", [])
        for record_index, record in enumerate(records):
            if record.get("outcome") == "measurement":
                if "clean_absolute_bearing_deg" not in record or "effects" not in record or "measurement" not in record:
                    failures.append(f"trial {index} measurement {record_index}: incomplete generation audit")
        events = audit.get("mission_events", [])
        if not any(event.get("type") == "state" for event in events):
            failures.append(f"trial {index}: missing state transitions")
        if not any(event.get("type") in {"waypoint", "safety_stop", "operator_abort"} for event in events):
            failures.append(f"trial {index}: missing command or terminal safety evidence")
    return {"artifact": str(path), "kind": "monte_carlo", "trial_count": len(trials),
            "passed": not failures, "failure_count": len(failures), "failures": failures}


def audit_sitl(root: Path) -> dict[str, Any]:
    results, failures = [], []
    for scenario in SITL_SCENARIOS:
        candidates = []
        for path in root.glob(f"{scenario}_seed42_*/validation_report.json"):
            payload = json.loads(path.read_text(encoding="utf-8"))
            if payload.get("status") == "PASS" and payload.get("evidence_manifest"):
                candidates.append((path.stat().st_mtime, path, payload))
        if not candidates:
            failures.append(f"{scenario}: no passing schema-complete SITL report")
            continue
        _, path, payload = max(candidates)
        item_failures = []
        for field in ("resolved_config", "scenario", "seed", "commands", "mission_record",
                      "evidence_manifest", "ardupilot", "status", "pass_reasons"):
            if field not in payload:
                item_failures.append(field)
        _require(payload.get("ardupilot", {}).get("revision"), "ArduPilot revision", item_failures)
        _require(payload.get("ardupilot", {}).get("sitl_parameters"), "SITL parameters", item_failures)
        events = (payload.get("mission_record") or {}).get("events", [])
        observations = (payload.get("mission_record") or {}).get("observations", [])
        contexts = [event for event in events if event.get("type") == "observation_context"]
        if len(contexts) != len(observations):
            item_failures.append("one telemetry/measurement context per recorded observation")
        if observations and not all("vehicle_state" in item and "measurement" in item for item in contexts):
            item_failures.append("complete observation contexts")
        if not any(event.get("type") == "state" for event in events):
            item_failures.append("mission state transitions")
        if scenario == "nominal":
            _require(payload.get("validation_metrics", {}).get("actual_error_m"),
                     "nominal actual error", item_failures)
            if not any(event.get("type") == "simulation_measurement" for event in events):
                item_failures.append("clean/injected/final Kraken measurement records")
            if not any(event.get("type") == "waypoint" for event in events):
                item_failures.append("MAVLink waypoint commands")
            if not any(event.get("type") == "estimate" for event in events):
                item_failures.append("estimator result")
        results.append({"scenario": scenario, "artifact": str(path),
                        "passed": not item_failures, "failures": item_failures})
        failures.extend(f"{scenario}: {failure}" for failure in item_failures)
    return {"artifact": str(root), "kind": "sitl", "scenario_count": len(results),
            "passed": not failures, "failure_count": len(failures), "failures": failures,
            "scenarios": results}


def run_audit(monte_carlo: Path, sitl_root: Path) -> dict[str, Any]:
    sections = [audit_monte_carlo(monte_carlo), audit_sitl(sitl_root)]
    return {"schema_version": 1, "generated_at": datetime.now(timezone.utc).isoformat(),
            "status": "PASS" if all(section["passed"] for section in sections) else "FAIL",
            "sections": sections,
            "required_separation": {
                "mission_receives_ground_truth": False,
                "validation_truth_location": "audit-only simulator records and post-mission scoring",
            }}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--monte-carlo", default="./logs/stage7_monte_carlo_report.json")
    parser.add_argument("--sitl-root", default="./logs/sitl")
    parser.add_argument("--output", default="./logs/stage10_evidence_audit.json")
    args = parser.parse_args()
    report = run_audit(Path(args.monte_carlo), Path(args.sitl_root))
    target = Path(args.output); target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
