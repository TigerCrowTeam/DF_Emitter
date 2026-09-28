from __future__ import annotations

from kraken_df_nav.config import AppConfig
from kraken_df_nav.monte_carlo import (
    _classification, _randomize, _summarize, run_campaign, run_trial,
)


def test_category_allocation_is_predeclared_60_24_16_split():
    counts = {name: sum(_classification(index, 500) == name for index in range(500))
              for name in ("PASS_EXPECTED", "DEGRADED_ALLOWED", "SAFE_FAILURE_EXPECTED")}
    assert counts == {"PASS_EXPECTED": 300, "DEGRADED_ALLOWED": 120,
                      "SAFE_FAILURE_EXPECTED": 80}


def test_randomization_is_seeded_and_changes_required_dimensions():
    first, meta_first = _randomize(AppConfig(), "DEGRADED_ALLOWED", 1234)
    second, meta_second = _randomize(AppConfig(), "DEGRADED_ALLOWED", 1234)
    assert first == second and meta_first == meta_second
    assert first.simulation.transmitter_lat != AppConfig().simulation.transmitter_lat
    assert first.simulation.start_lat != AppConfig().simulation.start_lat
    assert 0 <= meta_first["initial_heading_deg"] < 360
    assert first.simulation.digital_twin.confidence_error_scale_deg != 20


def test_small_campaign_is_repeatable_and_traceable():
    first = run_campaign(AppConfig(), trials=10, master_seed=99)
    second = run_campaign(AppConfig(), trials=10, master_seed=99)
    compact = lambda report: [(item["seed"], item["state"], item["actual_error_m"])
                              for item in report["trials"]]
    assert compact(first) == compact(second)
    assert all("resolved_config" in item and "seed" in item for item in first["trials"])
    assert first["total_trials"] == 10


def test_summary_contains_tail_and_safety_metrics():
    trial = run_trial(AppConfig(), "PASS_EXPECTED", seed=7, index=0)
    summary = _summarize([trial])
    assert set(summary["localization_error_m"]) == {"median", "p90", "p95", "worst"}
    assert "measurement_rejection_reasons" in summary
    assert "outlier_detection" in summary
    assert trial["unsafe_commands_after_stop"] == 0
