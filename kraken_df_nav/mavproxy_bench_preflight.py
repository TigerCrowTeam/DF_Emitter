"""Fail-closed, non-actuating preflight for the M9 Cube bench campaign."""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path

from .config import load_config
from .mavproxy_preflight import check_ports


def run(config_path: Path, props_removed: bool) -> dict:
    checks: list[dict[str, object]] = []
    try:
        config = load_config(config_path)
    except (OSError, ValueError) as exc:
        return {"status": "BLOCKED", "error": str(exc), "checks": []}
    proxy = config.mavproxy
    master = proxy.master_connection
    device, separator, baud = master.rpartition(",")
    if not separator:
        device, baud = master, ""

    def add(name: str, passed: bool, detail: str) -> None:
        checks.append({"check": name, "passed": passed, "detail": detail})

    add("live_mode", config.mode == "LIVE", f"mode={config.mode}")
    add("props_removed_confirmation", props_removed,
        "operator confirmed propellers removed" if props_removed else "pass --confirm-props-removed")
    add("stable_serial_path", device.startswith("/dev/serial/by-id/") and "REPLACE" not in device,
        device)
    add("serial_device_present", Path(device).exists(), device)
    add("baud_rate", baud.isdigit() and 9_600 <= int(baud) <= 921_600,
        baud or "missing")
    add("kraken_endpoint_configured", "REPLACE" not in config.kraken.endpoint,
        config.kraken.endpoint)
    add("loopback_only", proxy.loopback_host == "127.0.0.1"
        and config.mavlink.connection.startswith("udpin:127.0.0.1:"),
        f"host={proxy.loopback_host}, app={config.mavlink.connection}")
    try:
        ports = check_ports(proxy.loopback_host, [proxy.poc_output_port,
                                                  proxy.preparation_output_port,
                                                  proxy.operator_output_port])
    except OSError as exc:
        ports = {"error": str(exc)}
    add("udp_ports_available", bool(ports) and
        all(value == "AVAILABLE" for value in ports.values()), str(ports))
    add("distinct_identities", (proxy.source_system, proxy.source_component)
        != (config.mavlink.source_system, config.mavlink.source_component),
        f"proxy={proxy.source_system}:{proxy.source_component}, "
        f"application={config.mavlink.source_system}:{config.mavlink.source_component}")
    add("expected_target_explicit", config.mavlink.expected_target_system is not None
        and config.mavlink.expected_target_component is not None,
        f"{config.mavlink.expected_target_system}:{config.mavlink.expected_target_component}")
    passed = all(bool(item["passed"]) for item in checks)
    return {"schema_version": 1, "generated_at": datetime.now(timezone.utc).isoformat(),
            "status": "READY_FOR_NON_ACTUATING_TELEMETRY_TEST" if passed else "BLOCKED",
            "non_actuating": True, "config": str(config_path.resolve()), "checks": checks,
            "next_action": ("start MAVProxy only, keep disarmed, and collect telemetry evidence"
                            if passed else "resolve every failed check; do not connect movement tooling")}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="kraken_df_nav/config.mavproxy_live.yaml")
    parser.add_argument("--confirm-props-removed", action="store_true")
    parser.add_argument("--output", default="logs/mavproxy_m9_bench_preflight.json")
    args = parser.parse_args()
    report = run(Path(args.config), args.confirm_props_removed)
    target = Path(args.output); target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["status"].startswith("READY") else 3


if __name__ == "__main__": raise SystemExit(main())
