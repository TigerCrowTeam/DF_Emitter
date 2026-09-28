import math
import time

import pytest

from kraken_df_nav.config import AppConfig
from kraken_df_nav.df.estimator import WeightedLeastSquaresEstimator
from kraken_df_nav.df.geometry import GeoPoint
from kraken_df_nav.kraken.client import BearingSource, KrakenBearingSource
from kraken_df_nav.kraken.filtering import MeasurementGate, validate_measurement
from kraken_df_nav.kraken.models import BearingMeasurement
from kraken_df_nav.logging_utils.recorder import NullRecorder
from kraken_df_nav.mavlink.telemetry import VehicleState
from kraken_df_nav.mission.observation_planner import ObservationPlanner
from kraken_df_nav.mission.state_machine import Mission, MissionState
from kraken_df_nav.simulation.kraken_simulator import SimulatedVehicleController


NOW = 1_000.0


def measurement(**changes):
    values = dict(relative_bearing_deg=45.0, rssi_dbm=-60.0, confidence=.9,
                  frequency_hz=915_000_000, timestamp=NOW)
    values.update(changes)
    return BearingMeasurement(**values)


def gate_reason(item, gate=None, **changes):
    values = dict(target_hz=915_000_000, min_confidence=.7, tolerance_hz=1000,
        now=NOW, max_age_seconds=2, pose_timestamp=NOW, max_pose_skew_seconds=1,
        position_accuracy_m=1, max_position_accuracy_m=10,
        heading_accuracy_deg=1, max_heading_accuracy_deg=10,
        ground_speed_m_s=0, max_collection_speed_m_s=.5)
    values.update(changes)
    return (gate or MeasurementGate()).validate(item, **values)


@pytest.mark.parametrize(("field", "value"), [
    ("relative_bearing_deg", math.nan), ("rssi_dbm", math.inf),
    ("confidence", math.nan), ("timestamp", math.inf),
])
def test_non_finite_measurements_are_rejected(field, value):
    assert validate_measurement(measurement(**{field: value}), 915_000_000, .7, 1000,
                                NOW, 2) == "measurement contains non-finite value"


def test_invalid_bearing_is_not_hidden_by_http_parser():
    source = KrakenBearingSource("http://unused")
    parsed = source.parse_response(
        '{"bearing_deg": 400, "confidence": 0.9, "rssi_dbm": -60, '
        '"frequency_hz": 915000000, "timestamp": 1000}')
    assert parsed.relative_bearing_deg == 400
    assert gate_reason(parsed) == "invalid relative bearing"


@pytest.mark.parametrize(("item", "expected"), [
    (measurement(timestamp=990), "stale measurement"),
    (measurement(timestamp=1010), "measurement timestamp too far in future"),
    (measurement(confidence=.2), "confidence below threshold"),
    (measurement(confidence=1.2), "confidence outside [0, 1]"),
    (measurement(frequency_hz=900_000_000), "measurement outside configured RF bandwidth"),
])
def test_structural_age_frequency_and_confidence_rejections(item, expected):
    assert gate_reason(item) == expected


@pytest.mark.parametrize(("changes", "expected"), [
    ({"pose_timestamp": 998}, "bearing and vehicle pose are not time-synchronized"),
    ({"position_accuracy_m": 11}, "vehicle position accuracy is insufficient"),
    ({"heading_accuracy_deg": 11}, "vehicle heading accuracy is insufficient"),
    ({"ground_speed_m_s": 1}, "vehicle is moving during DF collection"),
])
def test_pose_synchronization_accuracy_and_motion_rejections(changes, expected):
    assert gate_reason(measurement(), **changes) == expected


def test_duplicate_and_frozen_sequences_are_stateful():
    duplicate_gate = MeasurementGate()
    item = measurement()
    assert gate_reason(item, duplicate_gate) is None
    assert gate_reason(item, duplicate_gate) == "duplicate measurement"

    frozen_gate = MeasurementGate(max_frozen_samples=3)
    assert gate_reason(measurement(timestamp=1000), frozen_gate) is None
    assert gate_reason(measurement(timestamp=1000.1), frozen_gate, pose_timestamp=1000.1) is None
    assert gate_reason(measurement(timestamp=1000.2), frozen_gate,
                       pose_timestamp=1000.2) == "frozen bearing sequence"


class RaisingSource(BearingSource):
    def get_measurement(self):
        raise ValueError("broken payload")


class MissingSource(BearingSource):
    def get_measurement(self):
        return None


def run_rejection(source):
    config = AppConfig()
    config.navigation.stabilization_seconds = 0
    vehicle = SimulatedVehicleController(GeoPoint(37, -122, 50))
    recorder = NullRecorder()
    mission = Mission(config, source, vehicle, WeightedLeastSquaresEstimator(),
                      ObservationPlanner(), recorder)
    mission.run(max_ticks=5, tick_seconds=0)
    return mission, vehicle, recorder


@pytest.mark.parametrize(("source", "text"), [
    (MissingSource(), "missing measurement"),
    (RaisingSource(), "malformed or unavailable measurement"),
])
def test_missing_and_malformed_measurements_are_logged_and_never_plan(source, text):
    mission, vehicle, recorder = run_rejection(source)
    assert not mission.observations and vehicle.command_count == 0
    assert any(event["type"] == "measurement_rejected" and text in event["reason"]
               for event in recorder.events)


def test_measurement_before_stabilization_is_rejected():
    class OneSource(BearingSource):
        def get_measurement(self):
            now = time.time()
            return BearingMeasurement(45, -60, .9, 915_000_000, now)

    config = AppConfig()
    vehicle = SimulatedVehicleController(GeoPoint(37, -122, 50))
    recorder = NullRecorder()
    mission = Mission(config, OneSource(), vehicle, WeightedLeastSquaresEstimator(),
                      ObservationPlanner(), recorder)
    mission.state = MissionState.COLLECTING_OBSERVATION
    mission.tick()
    assert not mission.observations and vehicle.command_count == 0
    assert any(event.get("reason") == "vehicle has not completed stabilization"
               for event in recorder.events)


def test_non_finite_vehicle_pose_is_invalid():
    state = VehicleState(37, -122, 50, math.nan, 0, 0, "GUIDED", timestamp=time.time())
    assert not state.navigation_valid()
