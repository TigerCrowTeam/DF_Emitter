from kraken_df_nav.scenarios import load_catalog, run_catalog


CATALOG = "kraken_df_nav/simulation_scenarios.yaml"
CONFIG = "kraken_df_nav/config.yaml"


def test_catalog_contains_unique_declared_roadmap_scenarios():
    catalog = load_catalog(CATALOG)
    assert len(catalog["scenarios"]) == 20
    assert {item["classification"] for item in catalog["scenarios"]} == {
        "PASS_EXPECTED", "DEGRADED_ALLOWED", "SAFE_FAILURE_EXPECTED"}
    assert all(isinstance(item["seed"], int) for item in catalog["scenarios"])


def test_every_deterministic_scenario_meets_declared_outcome():
    report = run_catalog(CATALOG, CONFIG)
    assert report["scenario_count"] == 20
    assert report["passed_count"] == 20
    assert report["failed_count"] == 0
    assert all(not item["false_success"] for item in report["results"])
    assert all(item["unsafe_commands_after_stop"] == 0 for item in report["results"])
    assert all(item["resolved_config"]["simulation"]["random_seed"] == item["seed"]
               for item in report["results"])


def test_scenario_summary_is_reproducible():
    first, second = run_catalog(CATALOG, CONFIG), run_catalog(CATALOG, CONFIG)
    keys = ("name", "state", "passed", "actual_error_m", "observations",
            "measurement_rejections", "waypoint_count", "false_success")
    compact = lambda report: [{key: item[key] for key in keys} for item in report["results"]]
    assert compact(first) == compact(second)
