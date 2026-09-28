"""CLI entry point for Kraken DF navigation PoC."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

from .config import AppConfig, load_config
from .df.estimator import WeightedLeastSquaresEstimator
from .df.geometry import GeoPoint
from .kraken.client import KrakenBearingSource
from .logging_utils.recorder import MissionRecorder
from .logging_utils.replay import load_observations
from .mavlink.connection import PymavlinkVehicleController
from .mission.observation_planner import ObservationPlanner
from .mission.state_machine import Mission, MissionState
from .simulation.kraken_simulator import SimulatedBearingSource, SimulatedVehicleController
from .simulation.transmitter import SimulatedTransmitter
from .validation import validate_trials, write_validation_report


def make_estimator(config: AppConfig) -> WeightedLeastSquaresEstimator:
    return WeightedLeastSquaresEstimator(config.df.outlier_rejection, config.df.outlier_threshold_m,
                                         config.df.max_leave_one_out_shift_m)


def run_replay(config: AppConfig) -> int:
    if not config.replay.path:
        raise ValueError("replay.path is required in REPLAY mode")
    observations = load_observations(config.replay.path)
    result = make_estimator(config).estimate(observations)
    print(json.dumps(result.__dict__, indent=2))
    return 0 if result.uncertainty_radius_m <= config.df.target_error_radius_m else 2


def build_mission(config: AppConfig) -> Mission:
    recorder = MissionRecorder(config.logging.directory, config.logging.save_csv, config.logging.save_json)
    if config.mode == "SIMULATION" and not config.simulation.use_sitl:
        vehicle = SimulatedVehicleController(GeoPoint(config.simulation.start_lat, config.simulation.start_lon,
                                                       config.simulation.altitude_m),
                                             effects=config.simulation.vehicle_twin,
                                             seed=config.simulation.random_seed,
                                             arrival_tolerance_m=config.navigation.arrival_tolerance_m)
        transmitter = SimulatedTransmitter(config.simulation.transmitter_lat, config.simulation.transmitter_lon)
        source = SimulatedBearingSource(transmitter, vehicle, config.rf.center_frequency_hz,
                                        config.simulation.bearing_noise_std_deg,
                                        config.kraken.antenna_heading_offset_deg, config.simulation.random_seed,
                                        config.simulation.digital_twin,
                                        lambda record: recorder.event("simulation_measurement", **record))
    else:
        vehicle = PymavlinkVehicleController(config.mavlink.connection,
                                             config.mavlink.heartbeat_timeout_seconds,
                                             config.mavlink.telemetry_timeout_seconds,
                                             config.mavlink.source_system,
                                             config.mavlink.source_component,
                                             config.mavlink.expected_target_system,
                                             config.mavlink.expected_target_component)
        if config.mode == "SIMULATION":
            transmitter = SimulatedTransmitter(config.simulation.transmitter_lat, config.simulation.transmitter_lon)
            source = SimulatedBearingSource(transmitter, vehicle, config.rf.center_frequency_hz,
                                            config.simulation.bearing_noise_std_deg,
                                            config.kraken.antenna_heading_offset_deg, config.simulation.random_seed,
                                            config.simulation.digital_twin,
                                            lambda record: recorder.event("simulation_measurement", **record))
        else:
            source = KrakenBearingSource(config.kraken.endpoint, config.kraken.timeout_seconds)
    planner = ObservationPlanner(config.navigation.desired_cross_bearing_deg,
                                 config.navigation.observation_spacing_m,
                                 config.navigation.max_search_radius_m,
                                 config.navigation.max_movement_distance_m,
                                 config.navigation.planner_min_separation_m,
                                 config.navigation.planner_min_information_gain)
    return Mission(config, source, vehicle, make_estimator(config), planner, recorder)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    parser.add_argument("--validate", nargs="+", metavar="PATH",
                        help="validate recorded trial files or directories instead of running a mission")
    parser.add_argument("--truth-lat", type=float, help="known transmitter latitude for --validate")
    parser.add_argument("--truth-lon", type=float, help="known transmitter longitude for --validate")
    parser.add_argument("--report-output", help="optional JSON output path for --validate")
    args = parser.parse_args()
    config = load_config(args.config)
    if args.validate:
        if args.truth_lat is None or args.truth_lon is None:
            parser.error("--validate requires --truth-lat and --truth-lon")
        report = validate_trials(args.validate, GeoPoint(args.truth_lat, args.truth_lon),
                                 make_estimator(config), config.df.target_error_radius_m)
        print(write_validation_report(report, args.report_output))
        return 0 if report["trial_count"] and not report["failed_file_count"] else 2
    if config.mode == "REPLAY":
        return run_replay(config)
    mission = build_mission(config)
    result = mission.run()
    if mission.state == MissionState.COMPLETE and result:
        print(f"Transmitter estimate: {result.latitude:.7f}, {result.longitude:.7f}")
        print(f"Uncertainty radius: {result.uncertainty_radius_m:.1f} m")
        print(json.dumps([o.to_dict() for o in mission.observations], indent=2))
        return 0
    print(f"Mission ended in {mission.state.name}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
