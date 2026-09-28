"""Direction-finding observation model."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime, timezone


@dataclass
class Observation:
    timestamp: str
    latitude: float
    longitude: float
    altitude_m: float
    vehicle_heading_deg: float
    relative_bearing_deg: float
    absolute_bearing_deg: float
    rssi_dbm: float
    confidence: float
    accepted: bool = True
    rejection_reason: str = ""
    position_accuracy_m: float = 0.0
    heading_accuracy_deg: float = 0.0

    @classmethod
    def now(cls, **kwargs: float) -> "Observation":
        return cls(timestamp=datetime.now(timezone.utc).isoformat(), **kwargs)

    def to_dict(self) -> dict[str, object]:
        return asdict(self)
