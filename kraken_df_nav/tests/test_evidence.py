from __future__ import annotations

from pathlib import Path

from kraken_df_nav.evidence import environment_manifest, source_manifest


def test_source_manifest_is_stable_and_covers_runtime_sources():
    first, second = source_manifest(), source_manifest()
    assert first == second
    assert len(first["archive_identifier"]) == 64
    assert "kraken_df_nav/mission/state_machine.py" in first["files"]
    assert "run_simulation_poc.sh" in first["files"]


def test_environment_manifest_records_dependencies_and_platform():
    manifest = environment_manifest()
    assert manifest["python"]
    assert manifest["platform"]
    assert manifest["dependencies"]["numpy"]
    assert manifest["dependencies"]["pymavlink"]
