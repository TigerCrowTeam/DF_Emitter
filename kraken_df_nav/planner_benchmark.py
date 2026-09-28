"""Compare observation planners in deterministic offline missions."""
from __future__ import annotations

import argparse
from dataclasses import asdict, dataclass
import json
from pathlib import Path
from statistics import mean

from .config import AppConfig, load_config
from .df.estimator import WeightedLeastSquaresEstimator
from .df.geometry import GeoPoint, distance_m
from .logging_utils.recorder import NullRecorder
from .mission.observation_planner import FixedGridPlanner, ObservationPlanner, RandomPlanner
from .mission.state_machine import Mission, MissionState
from .simulation.kraken_simulator import SimulatedBearingSource, SimulatedVehicleController
from .simulation.transmitter import SimulatedTransmitter


@dataclass
class PlannerTrial:
    planner: str
    seed: int
    state: str
    actual_error_m: float | None
    uncertainty_radius_m: float | None
    observations: int
    distance_traveled_m: float
    waypoint_count: int
    mission_ticks: int
    best_crossing_angle_deg: float
    false_success: bool


def _planner(name: str, config: AppConfig, seed: int):
    arguments = (config.navigation.desired_cross_bearing_deg,
                 config.navigation.observation_spacing_m,
                 config.navigation.max_search_radius_m,
                 config.navigation.max_movement_distance_m,
                 config.navigation.planner_min_separation_m,
                 config.navigation.planner_min_information_gain)
    if name == "information_gain":
        return ObservationPlanner(*arguments)
    if name == "fixed_grid":
        return FixedGridPlanner(*arguments)
    if name == "random":
        return RandomPlanner(*arguments, seed=seed)
    raise ValueError(f"unknown planner: {name}")


def run_trial(config: AppConfig, planner_name: str, seed: int) -> PlannerTrial:
    transmitter = SimulatedTransmitter(config.simulation.transmitter_lat,
                                       config.simulation.transmitter_lon)
    vehicle = SimulatedVehicleController(GeoPoint(config.simulation.start_lat,
        config.simulation.start_lon, config.simulation.altitude_m),
        effects=config.simulation.vehicle_twin, seed=seed,
        arrival_tolerance_m=config.navigation.arrival_tolerance_m)
    source = SimulatedBearingSource(transmitter, vehicle, config.rf.center_frequency_hz,
        config.simulation.bearing_noise_std_deg,
        config.kraken.antenna_heading_offset_deg, seed,
        config.simulation.digital_twin)
    recorder = NullRecorder()
    mission = Mission(config, source, vehicle,
        WeightedLeastSquaresEstimator(config.df.outlier_rejection,
            config.df.outlier_threshold_m, config.df.max_leave_one_out_shift_m),
        _planner(planner_name, config, seed), recorder)
    result = mission.run(max_ticks=300, tick_seconds=0)
    error = None if result is None else distance_m(
        GeoPoint(transmitter.latitude, transmitter.longitude),
        GeoPoint(result.latitude, result.longitude))
    waypoints = [event for event in recorder.events if event["type"] == "waypoint"]
    traveled = 0.0
    previous = GeoPoint(config.simulation.start_lat, config.simulation.start_lon)
    for event in waypoints:
        point = GeoPoint(event["latitude"], event["longitude"])
        traveled += distance_m(previous, point)
        previous = point
    complete = mission.state == MissionState.COMPLETE
    return PlannerTrial(planner_name, seed, mission.state.name, error,
        None if result is None else result.uncertainty_radius_m,
        len(mission.observations), traveled, len(waypoints), mission.ticks_executed,
        0.0 if result is None else result.best_crossing_angle_deg,
        bool(complete and (error is None or error > config.df.target_error_radius_m)))


def run_benchmark(config: AppConfig, seeds: int = 20) -> dict[str, object]:
    config.navigation.stabilization_seconds = 0
    names = ("information_gain", "fixed_grid", "random")
    trials = [run_trial(config, name, seed) for name in names for seed in range(seeds)]
    summaries = []
    for name in names:
        group = [trial for trial in trials if trial.planner == name]
        estimates = [trial for trial in group if trial.actual_error_m is not None]
        summaries.append({
            "planner": name,
            "trial_count": len(group),
            "completion_rate": sum(trial.state == "COMPLETE" for trial in group) / len(group),
            "false_success_rate": sum(trial.false_success for trial in group) / len(group),
            "mean_actual_error_m": mean(trial.actual_error_m for trial in estimates) if estimates else None,
            "mean_distance_traveled_m": mean(trial.distance_traveled_m for trial in group),
            "mean_observations": mean(trial.observations for trial in group),
            "mean_waypoints": mean(trial.waypoint_count for trial in group),
            "mean_mission_ticks": mean(trial.mission_ticks for trial in group),
            "mean_best_crossing_angle_deg": mean(trial.best_crossing_angle_deg for trial in estimates) if estimates else 0,
        })
    return {"seeds_per_planner": seeds, "total_trials": len(trials),
            "summaries": summaries, "trials": [asdict(trial) for trial in trials]}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--output", default="./logs/planner_benchmark.json")
    args = parser.parse_args()
    if args.seeds < 1:
        parser.error("seeds must be positive")
    report = run_benchmark(load_config(args.config), args.seeds)
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"total_trials": report["total_trials"],
                      "summaries": report["summaries"]}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
