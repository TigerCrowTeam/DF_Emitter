from datetime import datetime, timezone

import pytest

from kraken_df_nav.config import AppConfig
from kraken_df_nav.df.estimator import WeightedLeastSquaresEstimator
from kraken_df_nav.df.geometry import GeoPoint, destination, distance_m, initial_bearing_deg
from kraken_df_nav.df.observation import Observation
from kraken_df_nav.mission.observation_planner import FixedGridPlanner, ObservationPlanner, RandomPlanner
from kraken_df_nav.logging_utils.recorder import NullRecorder
from kraken_df_nav.mission.state_machine import Mission, MissionState
from kraken_df_nav.planner_benchmark import run_benchmark
from kraken_df_nav.simulation.kraken_simulator import SimulatedBearingSource, SimulatedVehicleController
from kraken_df_nav.simulation.transmitter import SimulatedTransmitter


TX = GeoPoint(37.001, -121.999)


def observation(point, confidence=.95):
    bearing = initial_bearing_deg(point, TX)
    return Observation(datetime.now(timezone.utc).isoformat(), point.latitude, point.longitude,
                       50, 0, bearing, bearing, -60, confidence)


def test_information_planner_creates_useful_crossing_and_respects_limits():
    first = destination(TX, 210, 150)
    current = destination(TX, 300, 150)
    observations = [observation(first), observation(current)]
    planner = ObservationPlanner(60, 100, 500, 150)
    waypoint = planner.plan(current, observations, TX)
    predicted = initial_bearing_deg(waypoint.point, TX)
    crossing = max(planner._crossing_angle(predicted, item.absolute_bearing_deg)
                   for item in observations)
    assert 45 <= crossing <= 90
    assert distance_m(current, waypoint.point) <= 150.01
    assert distance_m(first, waypoint.point) <= 500.01


def test_planner_avoids_repeated_and_marked_unavailable_points():
    current = destination(TX, 240, 150)
    observations = [observation(destination(TX, 180, 150)), observation(current)]
    planner = ObservationPlanner(spacing_m=100, min_separation_m=25)
    first = planner.plan(current, observations, TX)
    planner.mark_unavailable(first.point)
    second = planner.plan(current, observations, TX)
    assert distance_m(first.point, second.point) >= 25
    assert all(distance_m(second.point, GeoPoint(item.latitude, item.longitude)) >= 25
               for item in observations)


def test_planner_stops_when_required_information_gain_is_unavailable():
    current = destination(TX, 240, 150)
    observations = [observation(destination(TX, 180, 150)), observation(current)]
    planner = ObservationPlanner(min_information_gain=100)
    with pytest.raises(ValueError, match="useful geometry improvement"):
        planner.plan(current, observations, TX)


def test_far_estimate_and_boundary_estimate_remain_bounded():
    origin = GeoPoint(37, -122)
    observations = [observation(origin)]
    planner = ObservationPlanner(spacing_m=100, max_search_radius_m=200,
                                 max_movement_distance_m=120)
    far_estimate = destination(origin, 45, 1000)
    waypoint = planner.plan(origin, observations, far_estimate)
    assert distance_m(origin, waypoint.point) <= 120.01
    assert distance_m(origin, waypoint.point) <= 200.01

    boundary_estimate = destination(origin, 90, 195)
    observations.append(observation(waypoint.point))
    next_waypoint = planner.plan(waypoint.point, observations, boundary_estimate)
    assert distance_m(origin, next_waypoint.point) <= 200.01


def test_seeded_random_baseline_is_reproducible():
    current = destination(TX, 240, 150)
    observations = [observation(current)]
    first = RandomPlanner(seed=10).plan(current, observations)
    second = RandomPlanner(seed=10).plan(current, observations)
    assert first.point == second.point


def test_fixed_grid_baseline_returns_bounded_point():
    current = GeoPoint(37, -122)
    item = observation(current)
    waypoint = FixedGridPlanner(spacing_m=100, max_search_radius_m=200,
                                max_movement_distance_m=150).plan(current, [item])
    assert distance_m(current, waypoint.point) <= 150.01


def test_information_planner_outperforms_baselines_in_small_seeded_benchmark():
    config = AppConfig()
    report = run_benchmark(config, seeds=5)
    summaries = {item["planner"]: item for item in report["summaries"]}
    selected = summaries["information_gain"]
    assert selected["completion_rate"] > summaries["random"]["completion_rate"]
    assert selected["completion_rate"] > summaries["fixed_grid"]["completion_rate"]
    assert selected["mean_distance_traveled_m"] < summaries["fixed_grid"]["mean_distance_traveled_m"]
    assert selected["false_success_rate"] == 0


def test_mission_uses_distinct_alternate_after_waypoint_refusal():
    class RejectFirstVehicle(SimulatedVehicleController):
        def __init__(self, start):
            super().__init__(start)
            self.attempted_points = []

        def goto_location(self, lat, lon, alt):
            self.attempted_points.append(GeoPoint(lat, lon, alt))
            if len(self.attempted_points) == 1:
                raise RuntimeError("simulated blocked candidate")
            super().goto_location(lat, lon, alt)

    config = AppConfig()
    config.navigation.stabilization_seconds = 0
    transmitter = SimulatedTransmitter(config.simulation.transmitter_lat,
                                       config.simulation.transmitter_lon)
    vehicle = RejectFirstVehicle(GeoPoint(config.simulation.start_lat,
        config.simulation.start_lon, config.simulation.altitude_m))
    source = SimulatedBearingSource(transmitter, vehicle, seed=7)
    recorder = NullRecorder()
    mission = Mission(config, source, vehicle, WeightedLeastSquaresEstimator(),
        ObservationPlanner(), recorder)
    mission.run(max_ticks=150, tick_seconds=0)
    assert mission.state == MissionState.COMPLETE
    assert distance_m(vehicle.attempted_points[0], vehicle.attempted_points[1]) >= 20
    assert any(event["type"] == "waypoint_rejected" for event in recorder.events)
