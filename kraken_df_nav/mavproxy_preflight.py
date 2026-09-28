"""Validate a MAVProxy profile and ensure its local UDP ports are available."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import socket

import yaml

from .config import load_config


def check_ports(host: str, ports: list[int]) -> dict[int, str]:
    results = {}
    sockets = []
    try:
        for port in ports:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.bind((host, port))
            except OSError as exc:
                sock.close()
                results[port] = f"BUSY: {exc}"
            else:
                sockets.append(sock)
                results[port] = "AVAILABLE"
    finally:
        for sock in sockets:
            sock.close()
    return results


def run(config_path: str) -> dict:
    config = load_config(config_path)
    if not config.mavproxy.enabled:
        raise ValueError("selected configuration does not enable MAVProxy")
    proxy = config.mavproxy
    ports = [proxy.poc_output_port, proxy.preparation_output_port,
             proxy.operator_output_port]
    availability = check_ports(proxy.loopback_host, ports)
    busy = {str(k): v for k, v in availability.items() if v != "AVAILABLE"}
    return {
        "schema_version": 1, "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": "PASS" if not busy else "FAIL", "config_path": str(Path(config_path).resolve()),
        "route": {"master_owned_by": "MAVProxy", "application_connection": config.mavlink.connection,
                  "host": proxy.loopback_host, "ports": availability},
        "direct_autopilot_access_by_application": False,
        "busy_ports": busy, "resolved_config": asdict(config),
    }


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", default="kraken_df_nav/config.mavproxy_sitl.yaml")
    p.add_argument("--output")
    p.add_argument("--resolved-yaml")
    args = p.parse_args()
    try:
        report = run(args.config)
    except (OSError, ValueError) as exc:
        report = {"schema_version": 1, "generated_at": datetime.now(timezone.utc).isoformat(),
                  "status": "FAIL", "config_path": str(Path(args.config).resolve()),
                  "error": str(exc)}
    if args.output:
        Path(args.output).write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    if args.resolved_yaml and report["status"] == "PASS":
        Path(args.resolved_yaml).write_text(
            yaml.safe_dump(report["resolved_config"], sort_keys=False), encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__": raise SystemExit(main())
