"""Read observations from recorder JSON or CSV."""
from __future__ import annotations

import csv
import json
from pathlib import Path

from ..df.observation import Observation
from ..kraken.client import BearingSource
from ..kraken.models import BearingMeasurement


def load_observations(path: str) -> list[Observation]:
    source = Path(path)
    if source.suffix.lower() == ".json":
        payload = json.loads(source.read_text(encoding="utf-8"))
        rows = payload.get("observations", payload) if isinstance(payload, dict) else payload
    elif source.suffix.lower() == ".csv":
        with source.open(newline="", encoding="utf-8") as handle:
            rows = list(csv.DictReader(handle))
    else:
        raise ValueError("replay input must be JSON or CSV")
    numeric = {"latitude", "longitude", "altitude_m", "vehicle_heading_deg", "relative_bearing_deg",
               "absolute_bearing_deg", "rssi_dbm", "confidence",
               "position_accuracy_m", "heading_accuracy_deg"}
    result = []
    for row in rows:
        converted = {k: (float(v) if k in numeric else v) for k, v in row.items()}
        if isinstance(converted.get("accepted"), str):
            converted["accepted"] = converted["accepted"].lower() == "true"
        result.append(Observation(**converted))
    return result


class ReplayBearingSource(BearingSource):
    """Sequential measurement adapter; positions remain available in observations."""
    def __init__(self, observations: list[Observation], frequency_hz: int = 915_000_000):
        self.observations, self.frequency_hz, self.index = observations, frequency_hz, 0
    def get_measurement(self) -> BearingMeasurement | None:
        if self.index >= len(self.observations): return None
        item = self.observations[self.index]; self.index += 1
        return BearingMeasurement(item.relative_bearing_deg, item.rssi_dbm, item.confidence, self.frequency_hz, 0)
