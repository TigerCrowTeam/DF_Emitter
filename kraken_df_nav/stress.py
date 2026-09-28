"""Repeatable Monte Carlo stress testing for the offline DF mission."""
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
from .mission.observation_planner import ObservationPlanner
from .mission.state_machine import Mission, MissionState
from .simulation.kraken_simulator import SimulatedBearingSource, SimulatedVehicleController
from .simulation.transmitter import SimulatedTransmitter
from .validation import best_crossing_angle


@dataclass
class StressTrial:
    seed: int
    noise_std_deg: float
    state: str
    actual_error_m: float | None
    uncertainty_radius_m: float | None
    truth_inside_uncertainty: bool | None
    within_target: bool | None
    observations: int
    estimator_rejections: int
    rms_residual_m: float | None
    best_crossing_angle_deg: float


def run_trial(base: AppConfig, seed: int, noise_std_deg: float) -> StressTrial:
    tx = SimulatedTransmitter(base.simulation.transmitter_lat, base.simulation.transmitter_lon)
    vehicle = SimulatedVehicleController(GeoPoint(
        base.simulation.start_lat, base.simulation.start_lon, base.simulation.altitude_m))
    source = SimulatedBearingSource(tx, vehicle, base.rf.center_frequency_hz, noise_std_deg,
                                    base.kraken.antenna_heading_offset_deg, seed)
    planner = ObservationPlanner(
        base.navigation.desired_cross_bearing_deg, base.navigation.observation_spacing_m,
        base.navigation.max_search_radius_m, base.navigation.max_movement_distance_m)
    mission = Mission(base, source, vehicle,
                      WeightedLeastSquaresEstimator(base.df.outlier_rejection, base.df.outlier_threshold_m,
                                                     base.df.max_leave_one_out_shift_m),
                      planner, NullRecorder())
    result = mission.run(max_ticks=250, tick_seconds=0)
    truth = GeoPoint(tx.latitude, tx.longitude)
    error = None if result is None else distance_m(truth, GeoPoint(result.latitude, result.longitude))
    return StressTrial(
        seed=seed, noise_std_deg=noise_std_deg, state=mission.state.name,
        actual_error_m=error,
        uncertainty_radius_m=None if result is None else result.uncertainty_radius_m,
        truth_inside_uncertainty=None if result is None else error <= result.uncertainty_radius_m,
        within_target=None if result is None else error <= base.df.target_error_radius_m,
        observations=len(mission.observations),
        estimator_rejections=0 if result is None else result.rejected_count,
        rms_residual_m=None if result is None else result.rms_residual_m,
        best_crossing_angle_deg=best_crossing_angle([o.absolute_bearing_deg for o in mission.observations]),
    )


def run_stress(config: AppConfig, seeds: int = 20,
               noise_levels: tuple[float, ...] = (1, 2, 5, 10, 20)) -> dict[str, object]:
    # Stress runs should be fast and must still require the configured number of viewpoints.
    config.navigation.stabilization_seconds = 0
    trials = [run_trial(config, seed, noise) for noise in noise_levels for seed in range(seeds)]
    summaries = []
    for noise in noise_levels:
        group = [trial for trial in trials if trial.noise_std_deg == noise]
        completed = [trial for trial in group if trial.state == MissionState.COMPLETE.name]
        estimated = [trial for trial in group if trial.actual_error_m is not None]
        summaries.append({
            "noise_std_deg": noise,
            "trial_count": len(group),
            "completion_rate": len(completed) / len(group),
            "within_target_rate": sum(bool(t.within_target) for t in group) / len(group),
            "uncertainty_containment_rate": sum(bool(t.truth_inside_uncertainty) for t in group) / len(group),
            "mean_actual_error_m": mean(t.actual_error_m for t in estimated) if estimated else None,
            "max_actual_error_m": max((t.actual_error_m for t in estimated), default=None),
            "mean_uncertainty_radius_m": mean(t.uncertainty_radius_m for t in estimated) if estimated else None,
            "states": {state: sum(t.state == state for t in group) for state in sorted({t.state for t in group})},
        })
    return {
        "seeds_per_noise_level": seeds,
        "noise_levels_deg": list(noise_levels),
        "total_trials": len(trials),
        "target_error_radius_m": config.df.target_error_radius_m,
        "summaries": summaries,
        "trials": [asdict(trial) for trial in trials],
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--noise", type=float, nargs="+", default=[1, 2, 5, 10, 20])
    parser.add_argument("--output", default="./logs/stage3_stress_report.json")
    args = parser.parse_args()
    if args.seeds < 1 or any(noise < 0 for noise in args.noise):
        parser.error("seeds must be positive and noise levels must be non-negative")
    report = run_stress(load_config(args.config), args.seeds, tuple(args.noise))
    rendered = json.dumps(report, indent=2)
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("total_trials", "target_error_radius_m", "summaries")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
