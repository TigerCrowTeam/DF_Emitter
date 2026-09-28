"""Transport-neutral bearing samples."""
from dataclasses import dataclass


@dataclass(frozen=True)
class BearingMeasurement:
    relative_bearing_deg: float
    rssi_dbm: float
    confidence: float
    frequency_hz: int
    timestamp: float
