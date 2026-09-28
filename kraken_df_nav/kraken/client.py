"""Bearing-source abstractions and Kraken HTTP adapters."""
from __future__ import annotations

from abc import ABC, abstractmethod
import json
import time
from typing import Any
from urllib.request import urlopen

from .models import BearingMeasurement


class BearingSource(ABC):
    @abstractmethod
    def get_measurement(self) -> BearingMeasurement | None: ...

    def close(self) -> None:
        pass


class KrakenBearingSource(BearingSource):
    """Poll a normalized JSON adapter or Kraken V2 ``DOA_value.html``.

    Kraken deployments expose data in different ways. This PoC deliberately expects
    a tiny normalized JSON adapter rather than claiming an undocumented native API:
    {"bearing_deg": 42, "confidence": .9, "rssi_dbm": -61,
     "frequency_hz": 915000000, "timestamp": 1710000000.0}
    Override ``parse_payload`` when integrating the actual site schema.
    """

    def __init__(self, endpoint: str, timeout_seconds: float = 2.0):
        if not endpoint:
            raise ValueError("kraken.endpoint is required for real Kraken modes")
        self.endpoint = endpoint
        self.timeout_seconds = timeout_seconds

    def parse_payload(self, payload: dict[str, object]) -> BearingMeasurement:
        return BearingMeasurement(
            relative_bearing_deg=float(payload["bearing_deg"]),
            rssi_dbm=float(payload["rssi_dbm"]),
            confidence=float(payload["confidence"]),
            frequency_hz=int(payload["frequency_hz"]),
            timestamp=float(payload.get("timestamp", time.time())),
        )

    def parse_legacy_csv(self, payload: str) -> BearingMeasurement:
        """Parse the first Kraken V2 legacy DoA row.

        Fields 0..4 are timestamp-ms, compass bearing, confidence, signal
        metric (called power_db by Kraken), and tuned frequency in hertz.
        A single decimator is required for an unambiguous polling endpoint.
        """
        rows = [line.strip() for line in payload.splitlines() if line.strip()]
        if not rows:
            raise ValueError("Kraken DoA endpoint returned no bearing rows")
        if len(rows) > 1:
            raise ValueError("Kraken DoA endpoint returned multiple decimators; configure a single DoA decimator")
        fields = [field.strip() for field in rows[0].split(",")]
        if len(fields) < 5:
            raise ValueError("Kraken legacy DoA row has fewer than five fields")
        return BearingMeasurement(
            relative_bearing_deg=float(fields[1]),
            confidence=float(fields[2]),
            rssi_dbm=float(fields[3]),
            frequency_hz=int(fields[4]),
            timestamp=float(fields[0]) / 1000.0,
        )

    def parse_response(self, raw: str) -> BearingMeasurement:
        stripped = raw.lstrip()
        if stripped.startswith(("{", "[")):
            payload: Any = json.loads(raw)
            if not isinstance(payload, dict):
                raise ValueError("normalized Kraken JSON payload must be an object")
            return self.parse_payload(payload)
        return self.parse_legacy_csv(raw)

    def get_measurement(self) -> BearingMeasurement | None:
        with urlopen(self.endpoint, timeout=self.timeout_seconds) as response:
            payload = response.read().decode("utf-8")
        return self.parse_response(payload)
