from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

from kraken_df_nav.sitl_orchestrator import _deep_merge, _score, _terminate, main


def test_deep_merge_preserves_unrelated_configuration():
    config = {"simulation": {"random_seed": 1, "digital_twin": {"dropout_probability": 0}},
              "rf": {"center_frequency_hz": 915_000_000}}
    _deep_merge(config, {"simulation": {"digital_twin": {"dropout_probability": 1}}})
    assert config["simulation"]["random_seed"] == 1
    assert config["simulation"]["digital_twin"]["dropout_probability"] == 1
    assert config["rf"]["center_frequency_hz"] == 915_000_000


def test_nominal_scoring_requires_complete_and_loiter():
    mission = {"events": [{"type": "state", "current": "COMPLETE"}]}
    metrics = {"actual_error_m": 2, "uncertainty_radius_m": 5, "rms_residual_m": 2,
               "best_crossing_angle_deg": 60, "accepted_observations": 7}
    assert _score("nominal", "success", 0, mission, True, metrics) == (True, [])
    passed, reasons = _score("nominal", "success", 0, mission, False, metrics)
    assert not passed and "LOITER was not confirmed" in reasons


def test_safety_scoring_rejects_post_stop_waypoint():
    mission = {"events": [
        {"type": "safety_stop"}, {"type": "state", "current": "ABORTED"},
        {"type": "waypoint"},
    ]}
    passed, reasons = _score("kraken_loss", "safe_stop", 1, mission, False)
    assert not passed
    assert "waypoint(s) followed a safety stop" in reasons[0]


def test_preflight_missing_sim_vehicle_has_distinct_exit(tmp_path: Path, capsys):
    config = tmp_path / "config.yaml"
    config.write_text("mode: SIMULATION\n", encoding="utf-8")
    exit_code = main(["--config", str(config), "--sim-vehicle", "definitely-not-installed-s8"])
    assert exit_code == 3
    assert "preflight failed" in capsys.readouterr().err


def test_terminate_stops_process_group():
    process = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"],
                               start_new_session=True)
    _terminate(process)
    assert process.poll() is not None


def test_launch_can_isolate_child_working_directory(tmp_path: Path):
    from kraken_df_nav.sitl_orchestrator import _launch
    child_dir = tmp_path / "child"
    child_dir.mkdir()
    process, handle = _launch(
        [sys.executable, "-c", "from pathlib import Path; print(Path.cwd())"],
        tmp_path / "child.log", child_dir,
    )
    assert process.wait(timeout=5) == 0
    handle.close()
    assert (tmp_path / "child.log").read_text(encoding="utf-8").strip() == str(child_dir)


def test_df_application_has_no_arm_or_disarm_calls():
    root = Path(__file__).parents[1]
    application_sources = [root / "main.py", root / "mavlink" / "connection.py",
                           root / "mission" / "state_machine.py"]
    text = "\n".join(path.read_text(encoding="utf-8") for path in application_sources)
    assert "arducopter_arm" not in text
    assert "arducopter_disarm" not in text
