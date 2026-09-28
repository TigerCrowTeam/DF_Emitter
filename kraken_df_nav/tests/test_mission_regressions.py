from __future__ import annotations

import time

from kraken_df_nav.config import AppConfig
from kraken_df_nav.df.estimator import WeightedLeastSquaresEstimator
from kraken_df_nav.df.geometry import GeoPoint
from kraken_df_nav.kraken.client import BearingSource
from kraken_df_nav.kraken.models import BearingMeasurement
from kraken_df_nav.logging_utils.recorder import NullRecorder
from kraken_df_nav.mission.observation_planner import ObservationPlanner
from kraken_df_nav.mission.state_machine import Mission, MissionState
from kraken_df_nav.simulation.kraken_simulator import SimulatedVehicleController


class ParallelBearingSource(BearingSource):
    def __init__(self):
        self.index = 0

    def get_measurement(self):
        self.index += 1
        return BearingMeasurement(45, -60 - self.index * .01, .95, 915_000_000, time.time())


def test_invalid_parallel_geometry_cannot_produce_complete():
    config = AppConfig()
    config.navigation.stabilization_seconds = 0
    config.navigation.max_total_distance_m = 350
    vehicle = SimulatedVehicleController(GeoPoint(37, -122, 10))
    recorder = NullRecorder()
    mission = Mission(config, ParallelBearingSource(), vehicle,
        WeightedLeastSquaresEstimator(), ObservationPlanner(), recorder)

    mission.run(max_ticks=200, tick_seconds=0)

    assert mission.state in {MissionState.ABORTED, MissionState.PAUSED}
    assert mission.state is not MissionState.COMPLETE
    assert any(event["type"] == "estimator_invalid" for event in recorder.events)
    stop = next(i for i, event in enumerate(recorder.events) if event["type"] == "safety_stop")
    assert not any(event["type"] == "waypoint" for event in recorder.events[stop + 1:])


def test_successful_mission_records_expected_state_machine_path():
    from kraken_df_nav.simulation.kraken_simulator import SimulatedBearingSource
    from kraken_df_nav.simulation.transmitter import SimulatedTransmitter

    config = AppConfig()
    config.navigation.stabilization_seconds = 0
    transmitter = SimulatedTransmitter(37.0008, -121.9992)
    vehicle = SimulatedVehicleController(GeoPoint(37, -122, 10))
    recorder = NullRecorder()
    mission = Mission(config, SimulatedBearingSource(transmitter, vehicle, seed=7), vehicle,
        WeightedLeastSquaresEstimator(), ObservationPlanner(), recorder)
    mission.run(max_ticks=150, tick_seconds=0)
    states = [event["current"] for event in recorder.events if event["type"] == "state"]

    assert states[0] == "WAITING_FOR_SIGNAL"
    assert "COLLECTING_OBSERVATION" in states
    assert "PLANNING_MOVE" in states
    assert "MOVING" in states
    assert "ESTIMATING" in states
    assert states[-1] == "COMPLETE"
