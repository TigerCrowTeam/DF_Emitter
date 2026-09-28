"""Reproducibility metadata and source-manifest helpers."""
from __future__ import annotations

import hashlib
from importlib import metadata
import platform
from pathlib import Path
import subprocess
from typing import Any


DEPENDENCIES = ("numpy", "PyYAML", "pymavlink", "MAVProxy", "pytest", "pexpect", "empy")


def source_manifest(project_root: Path | None = None) -> dict[str, Any]:
    root = project_root or Path(__file__).parents[1]
    paths = sorted([
        *root.glob("kraken_df_nav/**/*.py"), *root.glob("kraken_df_nav/*.yaml"),
        root / "kraken_df_nav" / "requirements.txt", root / "run_simulation_poc.sh",
        root / "demo" / "run_mavproxy_demo.sh", root / "demo" / "setup_mavproxy_gui.sh",
        root / "demo" / "mavproxy_emitter_marker.py", root / "demo" / "README.md",
        root / "pytest.ini",
    ])
    files, combined = {}, hashlib.sha256()
    for path in paths:
        if not path.is_file() or "__pycache__" in path.parts:
            continue
        relative = path.relative_to(root).as_posix()
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        files[relative] = digest
        combined.update(relative.encode("utf-8") + b"\0" + digest.encode("ascii") + b"\n")
    return {"algorithm": "sha256", "archive_identifier": combined.hexdigest(), "files": files}


def environment_manifest(project_root: Path | None = None) -> dict[str, Any]:
    versions = {}
    for dependency in DEPENDENCIES:
        try:
            versions[dependency] = metadata.version(dependency)
        except metadata.PackageNotFoundError:
            versions[dependency] = None
    return {
        "platform": platform.platform(), "python": platform.python_version(),
        "implementation": platform.python_implementation(), "dependencies": versions,
        "source": source_manifest(project_root),
    }


def ardupilot_revision(sim_vehicle_path: str | Path) -> str | None:
    path = Path(sim_vehicle_path).resolve()
    for parent in path.parents:
        if (parent / ".git").exists():
            completed = subprocess.run(
                ["git", "-C", str(parent), "rev-parse", "HEAD"],
                text=True, capture_output=True, timeout=10,
            )
            return completed.stdout.strip() if completed.returncode == 0 else None
    return None
