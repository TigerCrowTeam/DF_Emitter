"""Measurement and pose synchronization validation."""
from __future__ import annotations

import math
import time

from .models import BearingMeasurement


def validate_measurement(measurement: BearingMeasurement, target_hz: int, min_confidence: float,
                         tolerance_hz: int, now: float | None = None,
                         max_age_seconds: float | None = None) -> str | None:
    numeric = (measurement.relative_bearing_deg, measurement.rssi_dbm,
               measurement.confidence, measurement.frequency_hz, measurement.timestamp)
    if not all(math.isfinite(value) for value in numeric):
        return "measurement contains non-finite value"
    if measurement.confidence < min_confidence:
        return "confidence below threshold"
    if not 0 <= measurement.confidence <= 1:
        return "confidence outside [0, 1]"
    if abs(measurement.frequency_hz - target_hz) > tolerance_hz:
        return "measurement outside configured RF bandwidth"
    if not 0 <= measurement.relative_bearing_deg < 360:
        return "invalid relative bearing"
    if max_age_seconds is not None:
        current = time.time() if now is None else now
        age = current - measurement.timestamp
        if age > max_age_seconds:
            return "stale measurement"
        if age < -max_age_seconds:
            return "measurement timestamp too far in future"
    return None


class MeasurementGate:
    """Stateful duplicate/frozen detection layered on structural validation."""

    def __init__(self, max_frozen_samples: int = 20):
        self.max_frozen_samples = max_frozen_samples
        self.last_measurement: BearingMeasurement | None = None
        self.frozen_count = 0

    def validate(self, measurement: BearingMeasurement, *, target_hz: int,
                 min_confidence: float, tolerance_hz: int, now: float,
                 max_age_seconds: float, pose_timestamp: float,
                 max_pose_skew_seconds: float, position_accuracy_m: float,
                 max_position_accuracy_m: float, heading_accuracy_deg: float,
                 max_heading_accuracy_deg: float, ground_speed_m_s: float,
                 max_collection_speed_m_s: float) -> str | None:
        reason = validate_measurement(measurement, target_hz, min_confidence,
                                      tolerance_hz, now, max_age_seconds)
        if reason:
            return reason
        pose_values = (pose_timestamp, position_accuracy_m, heading_accuracy_deg, ground_speed_m_s)
        if not all(math.isfinite(value) for value in pose_values):
            return "vehicle pose contains non-finite value"
        if abs(measurement.timestamp - pose_timestamp) > max_pose_skew_seconds:
            return "bearing and vehicle pose are not time-synchronized"
        if position_accuracy_m > max_position_accuracy_m:
            return "vehicle position accuracy is insufficient"
        if heading_accuracy_deg > max_heading_accuracy_deg:
            return "vehicle heading accuracy is insufficient"
        if ground_speed_m_s > max_collection_speed_m_s:
            return "vehicle is moving during DF collection"
        previous = self.last_measurement
        self.last_measurement = measurement
        if previous is not None:
            if measurement == previous:
                return "duplicate measurement"
            difference = abs((measurement.relative_bearing_deg - previous.relative_bearing_deg + 180) % 360 - 180)
            self.frozen_count = self.frozen_count + 1 if difference < 1e-9 else 0
            if self.frozen_count + 1 >= self.max_frozen_samples:
                return "frozen bearing sequence"
        return None
