from pathlib import Path

from kraken_df_nav.mavproxy_bench_preflight import run


def test_m9_template_fails_closed_without_hardware_confirmation():
    report = run(Path("kraken_df_nav/config.mavproxy_live.yaml"), props_removed=False)
    assert report["status"] == "BLOCKED"
    failed = {item["check"] for item in report["checks"] if not item["passed"]}
    assert "props_removed_confirmation" in failed
    assert "stable_serial_path" in failed
    assert "kraken_endpoint_configured" in failed
