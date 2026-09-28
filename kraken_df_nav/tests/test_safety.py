import time

from kraken_df_nav.config import AppConfig
from kraken_df_nav.df.estimator import WeightedLeastSquaresEstimator
from kraken_df_nav.df.geometry import GeoPoint
from kraken_df_nav.kraken.client import BearingSource
from kraken_df_nav.kraken.models import BearingMeasurement
from kraken_df_nav.logging_utils.recorder import NullRecorder
from kraken_df_nav.mavlink.telemetry import VehicleState
from kraken_df_nav.mission.observation_planner import ObservationPlanner
from kraken_df_nav.mission.state_machine import Mission, MissionState
from kraken_df_nav.simulation.kraken_simulator import SimulatedVehicleController


class FixedSource(BearingSource):
    def __init__(self, measurement=None):
        self.measurement = measurement

    def get_measurement(self):
        return self.measurement


class StateVehicle(SimulatedVehicleController):
    def __init__(self, state):
        super().__init__(GeoPoint(state.latitude, state.longitude, state.altitude_m))
        self.state = state

    def get_vehicle_state(self):
        return self.state

    def navigation_valid(self):
        return self.state.navigation_valid()


def mission_for(vehicle, source, action="ABORT"):
    config = AppConfig()
    config.navigation.stabilization_seconds = 0
    config.safety.invalid_data_action = action
    return Mission(config, source, vehicle, WeightedLeastSquaresEstimator(),
                   ObservationPlanner(), NullRecorder())


def valid_measurement(**changes):
    values = dict(relative_bearing_deg=45, rssi_dbm=-60, confidence=.95,
                  frequency_hz=915_000_000, timestamp=time.time())
    values.update(changes)
    return BearingMeasurement(**values)


def test_missing_measurement_does_not_create_waypoint():
    vehicle = SimulatedVehicleController(GeoPoint(37, -122, 50))
    mission = mission_for(vehicle, FixedSource(None))
    mission.run(max_ticks=8, tick_seconds=0)
    assert vehicle.target is None
    assert not mission.observations


def test_stale_navigation_aborts_without_waypoint():
    state = VehicleState(37, -122, 50, 0, 0, 0, "GUIDED", timestamp=time.time() - 60)
    vehicle = StateVehicle(state)
    mission = mission_for(vehicle, FixedSource(valid_measurement()))
    mission.run(max_ticks=5, tick_seconds=0)
    assert mission.state == MissionState.ABORTED
    assert vehicle.target is None


def test_excessive_attitude_collects_no_bearing():
    state = VehicleState(37, -122, 50, 0, 20, 0, "GUIDED", timestamp=time.time())
    vehicle = StateVehicle(state)
    mission = mission_for(vehicle, FixedSource(valid_measurement()))
    mission.run(max_ticks=8, tick_seconds=0)
    assert not mission.observations
    assert vehicle.target is None


def test_low_confidence_and_wrong_frequency_create_no_waypoint():
    for measurement in (valid_measurement(confidence=.1), valid_measurement(frequency_hz=800_000_000)):
        vehicle = SimulatedVehicleController(GeoPoint(37, -122, 50))
        mission = mission_for(vehicle, FixedSource(measurement))
        mission.run(max_ticks=8, tick_seconds=0)
        assert not mission.observations
        assert vehicle.target is None


def test_pause_action_prevents_waypoint_after_stale_navigation():
    state = VehicleState(37, -122, 50, 0, 0, 0, "GUIDED", timestamp=time.time() - 60)
    vehicle = StateVehicle(state)
    mission = mission_for(vehicle, FixedSource(valid_measurement()), "PAUSE")
    mission.run(max_ticks=5, tick_seconds=0)
    assert mission.state == MissionState.PAUSED
    assert vehicle.target is None


def test_terminal_states_remain_latched_under_extra_ticks():
    for action, expected in (("ABORT", MissionState.ABORTED),
                             ("PAUSE", MissionState.PAUSED)):
        state = VehicleState(37, -122, 50, 0, 0, 0, "GUIDED",
                             timestamp=time.time() - 60)
        vehicle = StateVehicle(state)
        mission = mission_for(vehicle, FixedSource(valid_measurement()), action)
        mission.run(max_ticks=5, tick_seconds=0)
        event_count = len(mission.recorder.events)
        for _ in range(20):
            mission.tick()
        assert mission.state is expected
        assert len(mission.recorder.events) == event_count
        assert vehicle.target is None
