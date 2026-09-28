import time

import pytest

from kraken_df_nav.config import AppConfig, VehicleTwinConfig, load_config
from kraken_df_nav.df.estimator import WeightedLeastSquaresEstimator
from kraken_df_nav.df.geometry import GeoPoint, distance_m
from kraken_df_nav.kraken.client import BearingSource
from kraken_df_nav.kraken.models import BearingMeasurement
from kraken_df_nav.logging_utils.recorder import NullRecorder
from kraken_df_nav.mission.observation_planner import ObservationPlanner
from kraken_df_nav.mission.state_machine import Mission, MissionState
from kraken_df_nav.simulation.kraken_simulator import SimulatedVehicleController


class Clock:
    def __init__(self, value=1000.0):
        self.value = value

    def __call__(self):
        return self.value


class FixedSource(BearingSource):
    def get_measurement(self):
        return BearingMeasurement(45, -60, .95, 915_000_000, time.time())


def make_mission(vehicle, clock=None):
    config = AppConfig()
    config.navigation.stabilization_seconds = 0
    config.simulation.required_observations = 7
    return Mission(config, FixedSource(), vehicle, WeightedLeastSquaresEstimator(),
                   ObservationPlanner(), NullRecorder(), clock=clock or time.monotonic)


def advance_to_moving(mission):
    for _ in range(5):
        mission.tick()
        if mission.state == MissionState.MOVING:
            return
    raise AssertionError(f"mission did not reach MOVING: {mission.state}")


def test_progressive_travel_and_arrival_delay():
    effects = VehicleTwinConfig(travel_fraction_per_update=.5, arrival_delay_updates=2)
    vehicle = SimulatedVehicleController(GeoPoint(37, -122), effects=effects)
    target = GeoPoint(37.0009, -122)
    vehicle.goto_location(target.latitude, target.longitude, 0)
    initial = distance_m(vehicle.point, target)
    assert vehicle.distance_to_target() == pytest.approx(initial)
    assert vehicle.distance_to_target() == pytest.approx(initial)
    assert vehicle.distance_to_target() == pytest.approx(initial / 2, rel=.01)
    assert vehicle.total_distance_m > 0


def test_seeded_gps_heading_drift_noise_and_latency_are_reproducible():
    effects = VehicleTwinConfig(gps_noise_std_m=2, gps_drift_m_per_sample=1,
        heading_bias_deg=5, heading_noise_std_deg=1, telemetry_latency_seconds=3)
    first = SimulatedVehicleController(GeoPoint(37, -122), 20, effects, seed=9, clock=lambda: 1000)
    second = SimulatedVehicleController(GeoPoint(37, -122), 20, effects, seed=9, clock=lambda: 1000)
    states_a = [first.get_vehicle_state() for _ in range(3)]
    states_b = [second.get_vehicle_state() for _ in range(3)]
    assert states_a == states_b
    assert states_a[0].timestamp == 997
    assert states_a[0].heading_deg != 20
    assert states_a[0].latitude != first.point.latitude
    assert first.records[-1]["true_latitude"] == first.point.latitude


def test_movement_attitude_and_tracking_error_are_modeled():
    effects = VehicleTwinConfig(travel_fraction_per_update=.5, moving_roll_deg=8,
        moving_pitch_deg=4, position_tracking_error_m=6)
    vehicle = SimulatedVehicleController(GeoPoint(37, -122), effects=effects)
    requested = GeoPoint(37.0009, -122)
    vehicle.goto_location(requested.latitude, requested.longitude, 0)
    state = vehicle.get_vehicle_state()
    assert state.roll_deg == 8 and state.pitch_deg == 4
    assert distance_m(vehicle.target, requested) == pytest.approx(6, rel=.02)


@pytest.mark.parametrize("effects", [
    VehicleTwinConfig(telemetry_loss_after_samples=1),
    VehicleTwinConfig(telemetry_packet_loss_probability=1),
])
def test_telemetry_loss_returns_no_state(effects):
    vehicle = SimulatedVehicleController(GeoPoint(37, -122), effects=effects)
    assert vehicle.get_vehicle_state() is None


@pytest.mark.parametrize("effects", [
    VehicleTwinConfig(heartbeat_loss_after_samples=1),
    VehicleTwinConfig(gps_loss_after_samples=1),
    VehicleTwinConfig(unexpected_mode_after_samples=1),
])
def test_navigation_faults_are_invalid(effects):
    vehicle = SimulatedVehicleController(GeoPoint(37, -122), effects=effects)
    state = vehicle.get_vehicle_state()
    if state is not None and effects.gps_loss_after_samples:
        assert state.gps_fix_type == 1
    if state is not None and effects.unexpected_mode_after_samples:
        assert state.flight_mode == "ALT_HOLD"
    assert not vehicle.navigation_valid()


def test_geofence_and_waypoint_acceptance_failure_refuse_commands():
    start = GeoPoint(37, -122)
    geofenced = SimulatedVehicleController(start, effects=VehicleTwinConfig(geofence_radius_m=20))
    with pytest.raises(RuntimeError, match="geofence"):
        geofenced.goto_location(37.001, -122, 0)
    failed = SimulatedVehicleController(start,
        effects=VehicleTwinConfig(waypoint_failure_probability=1))
    with pytest.raises(RuntimeError, match="acceptance failure"):
        failed.goto_location(37.0001, -122, 0)


def test_mission_duration_limit_aborts_without_waypoint():
    clock = Clock()
    vehicle = SimulatedVehicleController(GeoPoint(37, -122))
    mission = make_mission(vehicle, clock)
    mission.tick()
    clock.value += 601
    mission.tick()
    assert mission.state == MissionState.ABORTED
    assert vehicle.command_count == 0


def test_movement_timeout_prevents_later_commands():
    clock = Clock()
    vehicle = SimulatedVehicleController(GeoPoint(37, -122),
        effects=VehicleTwinConfig(travel_fraction_per_update=.1, arrival_delay_updates=20))
    mission = make_mission(vehicle, clock)
    mission.config.navigation.movement_timeout_seconds = 5
    advance_to_moving(mission)
    assert vehicle.command_count == 1
    clock.value += 6
    mission.tick()
    mission.tick()
    assert mission.state == MissionState.ABORTED
    assert vehicle.command_count == 1


def test_total_distance_limit_refuses_first_excessive_move():
    vehicle = SimulatedVehicleController(GeoPoint(37, -122))
    mission = make_mission(vehicle)
    mission.config.navigation.max_total_distance_m = 50
    advance_to_moving_or_abort = [mission.tick() for _ in range(5)]
    assert advance_to_moving_or_abort == [None] * 5
    assert mission.state == MissionState.ABORTED
    assert vehicle.command_count == 0
    assert any("maximum total mission distance" in event.get("reason", "")
               for event in mission.recorder.events)


def test_unexpected_mode_aborts_before_any_waypoint():
    vehicle = SimulatedVehicleController(GeoPoint(37, -122),
        effects=VehicleTwinConfig(unexpected_mode_after_samples=1))
    mission = make_mission(vehicle)
    mission.run(max_ticks=5, tick_seconds=0)
    assert mission.state == MissionState.ABORTED
    assert vehicle.command_count == 0


def test_nested_vehicle_config_loads_and_validates(tmp_path):
    path = tmp_path / "config.yaml"
    path.write_text("simulation:\n  vehicle_twin:\n    gps_noise_std_m: 3\n    travel_fraction_per_update: 0.25\n",
                    encoding="utf-8")
    config = load_config(path)
    assert config.simulation.vehicle_twin.gps_noise_std_m == 3
    assert config.simulation.vehicle_twin.travel_fraction_per_update == .25
    path.write_text("simulation:\n  vehicle_twin:\n    travel_fraction_per_update: 0\n", encoding="utf-8")
    with pytest.raises(ValueError, match="travel_fraction"):
        load_config(path)
