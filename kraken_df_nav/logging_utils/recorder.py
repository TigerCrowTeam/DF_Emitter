"""Machine-readable mission recording."""
from __future__ import annotations

import csv
from datetime import datetime, timezone
import json
from pathlib import Path
from typing import Any

from ..df.observation import Observation


class MissionRecorder:
    def __init__(self, directory: str, save_csv: bool = True, save_json: bool = True):
        # Microseconds prevent concurrent Monte Carlo workers from selecting the same files.
        stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.prefix = self.directory / f"mission_{stamp}"
        self.save_csv, self.save_json = save_csv, save_json
        self.events: list[dict[str, Any]] = []
        self.observations: list[Observation] = []
        self.final_report: dict[str, Any] | None = None

    def event(self, kind: str, **data: Any) -> None:
        item = {"timestamp": datetime.now(timezone.utc).isoformat(), "type": kind, **data}
        self.events.append(item)
        print(json.dumps(item, default=str))
        self.flush()

    def observation(self, observation: Observation) -> None:
        self.observations.append(observation)
        self.event("observation", **observation.to_dict())

    def set_final_report(self, **data: Any) -> None:
        """Persist the mission summary as an easy-to-find top-level JSON object."""
        self.final_report = {
            "generated_at": datetime.now(timezone.utc).isoformat(),
            **data,
        }
        self.event("final_report", **self.final_report)

    def flush(self) -> None:
        if self.save_json:
            self.prefix.with_suffix(".json").write_text(
                json.dumps({
                    "final_report": self.final_report,
                    "events": self.events,
                    "observations": [o.to_dict() for o in self.observations],
                }, indent=2, default=str), encoding="utf-8")
        if self.save_csv and self.observations:
            with self.prefix.with_suffix(".csv").open("w", newline="", encoding="utf-8") as handle:
                writer = csv.DictWriter(handle, fieldnames=list(self.observations[0].to_dict()))
                writer.writeheader()
                writer.writerows(o.to_dict() for o in self.observations)


class NullRecorder(MissionRecorder):
    def __init__(self):
        self.events, self.observations, self.final_report = [], [], None
    def event(self, kind: str, **data: Any) -> None:
        self.events.append({"timestamp": datetime.now(timezone.utc).isoformat(), "type": kind, **data})
    def observation(self, observation: Observation) -> None:
        self.observations.append(observation)
        self.event("observation", **observation.to_dict())
    def set_final_report(self, **data: Any) -> None:
        self.final_report = {"generated_at": datetime.now(timezone.utc).isoformat(), **data}
        self.event("final_report", **self.final_report)
    def flush(self) -> None: pass
