from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess

import pytest


@pytest.mark.mavproxy_sitl
@pytest.mark.parametrize("scenario", ["nominal", "kraken_loss", "proxy_loss_movement"])
def test_real_mavproxy_ardupilot_route(tmp_path: Path, scenario: str):
    if os.environ.get("RUN_MAVPROXY_INTEGRATION") != "1":
        pytest.skip("set RUN_MAVPROXY_INTEGRATION=1 to run routed SITL")
    root = Path(__file__).parents[2]
    completed = subprocess.run([
        str(root / "demo" / "run_mavproxy_demo.sh"), "--headless", "--scenario", scenario,
        "--seed", "42", "--timeout", "180", "--output-root",
        str(tmp_path / "mavproxy-sitl"),
    ], cwd=root, text=True, capture_output=True, timeout=420)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    reports = list((tmp_path / "mavproxy-sitl").glob("*/validation_report.json"))
    assert len(reports) == 1
    report = json.loads(reports[0].read_text(encoding="utf-8"))
    assert report["status"] == "PASS"
    assert report["all_children_stopped"] is True
    events = (report.get("mission_record") or {}).get("events", [])
    if scenario == "nominal":
        assert report["loiter_confirmed"] is True
        assert report["validation_metrics"]["actual_error_m"] <= 10
    else:
        stops = [i for i, event in enumerate(events)
                 if event.get("type") in {"safety_stop", "operator_abort"}]
        assert stops
        assert not any(event.get("type") == "waypoint"
                       for event in events[stops[0] + 1:])
