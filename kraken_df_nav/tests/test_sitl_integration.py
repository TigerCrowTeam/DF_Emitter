from __future__ import annotations

import json
import os
from pathlib import Path
import subprocess

import pytest


@pytest.mark.sitl
def test_nominal_real_ardupilot_sitl_end_to_end(tmp_path: Path):
    if os.environ.get("RUN_SITL_INTEGRATION") != "1":
        pytest.skip("set RUN_SITL_INTEGRATION=1 to run real ArduPilot SITL")
    root = Path(__file__).parents[2]
    sim_vehicle = Path(os.environ.get(
        "ARDUPILOT_SIM_VEHICLE",
        root / "tools" / "ardupilot" / "Tools" / "autotest" / "sim_vehicle.py",
    ))
    if not sim_vehicle.is_file():
        pytest.fail(f"ArduPilot sim_vehicle.py not found: {sim_vehicle}")

    completed = subprocess.run([
        str(root / "run_simulation_poc.sh"), "--scenario", "nominal", "--seed", "42",
        "--sim-vehicle", str(sim_vehicle), "--timeout", "300",
        "--output-root", str(tmp_path / "sitl"),
    ], cwd=root, text=True, capture_output=True, timeout=420)
    assert completed.returncode == 0, completed.stdout + completed.stderr
    reports = list((tmp_path / "sitl").glob("*/validation_report.json"))
    assert len(reports) == 1
    report = json.loads(reports[0].read_text(encoding="utf-8"))
    assert report["status"] == "PASS"
    assert report["loiter_confirmed"] is True
    assert report["validation_metrics"]["actual_error_m"] <= 10
    assert report["all_children_stopped"] is True
