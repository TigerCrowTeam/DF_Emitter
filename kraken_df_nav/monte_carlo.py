"""Final S7 randomized offline Monte Carlo validation campaign."""
from __future__ import annotations

import argparse
from collections import Counter
from copy import deepcopy
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import random
from statistics import mean, median
from typing import Any, Iterable

import numpy as np

from .config import AppConfig, load_config
from .df.estimator import WeightedLeastSquaresEstimator
from .df.geometry import GeoPoint, destination, distance_m
from .evidence import environment_manifest
from .logging_utils.recorder import NullRecorder
from .mission.observation_planner import FixedGridPlanner, ObservationPlanner, RandomPlanner
from .mission.state_machine import Mission, MissionState
from .simulation.kraken_simulator import SimulatedBearingSource, SimulatedVehicleController
from .simulation.transmitter import SimulatedTransmitter


CLASSIFICATIONS = ("PASS_EXPECTED", "DEGRADED_ALLOWED", "SAFE_FAILURE_EXPECTED")
S0_TARGETS = {
    "nominal_within_10m_rate_min": 0.80,
    "false_success_rate_max": 0.02,
    "completion_uncertainty_radius_m_max": 15.0,
    "completion_rms_residual_m_max": 12.0,
    "completion_best_crossing_angle_deg_min": 35.0,
    "observation_count_max": 9,
    "mission_duration_seconds_max": 600.0,
    "distance_traveled_m_max": 800.0,
    "uncertainty_containment_rate_min": 0.80,
    "uncertainty_containment_rate_max": 0.99,
}


def _percentile(values: Iterable[float], percentile: float) -> float | None:
    data = list(values)
    return None if not data else float(np.percentile(data, percentile))


def _classification(index: int, count: int) -> str:
    # Freeze a 60/24/16 split before randomization: 300/120/80 at the required 500 trials.
    position = index / count
    return ("PASS_EXPECTED" if position < .60 else
            "DEGRADED_ALLOWED" if position < .84 else "SAFE_FAILURE_EXPECTED")


def _randomize(base: AppConfig, classification: str, seed: int) -> tuple[AppConfig, dict[str, Any]]:
    rng, config = random.Random(seed), deepcopy(base)
    origin = GeoPoint(base.simulation.start_lat, base.simulation.start_lon)
    emitter = destination(origin, rng.uniform(0, 360), rng.uniform(40, 180))
    start = destination(emitter, rng.uniform(0, 360), rng.uniform(80, 180))
    config.simulation.transmitter_lat, config.simulation.transmitter_lon = emitter.latitude, emitter.longitude
    config.simulation.start_lat, config.simulation.start_lon = start.latitude, start.longitude
    initial_heading = rng.uniform(0, 360)
    config.simulation.random_seed = seed
    config.simulation.required_observations = rng.randint(7, 9)
    config.navigation.desired_cross_bearing_deg = rng.uniform(45, 80)
    config.navigation.observation_spacing_m = rng.uniform(70, 130)
    config.navigation.planner_min_separation_m = rng.uniform(15, 30)
    config.navigation.stabilization_seconds = 0
    digital, vehicle = config.simulation.digital_twin, config.simulation.vehicle_twin
    planner_name = "information_gain"
    fault_profile = "none"

    if classification == "PASS_EXPECTED":
        config.simulation.bearing_noise_std_deg = rng.uniform(.5, 2.5)
        digital.constant_bias_deg = rng.uniform(-.75, .75)
        digital.drift_deg_per_sample = rng.uniform(-.06, .06)
        digital.heading_error_deg = rng.uniform(-.5, .5)
        digital.dropout_probability = rng.uniform(0, .04)
        digital.delay_seconds = rng.uniform(0, .25)
        digital.stale_probability = rng.uniform(0, .01)
        digital.multipath_probability = rng.uniform(0, .025)
        digital.multipath_bias_deg = rng.uniform(4, 10)
        digital.multipath_persistence_samples = rng.randint(1, 2)
        digital.outlier_probability = rng.uniform(0, .025)
        digital.outlier_magnitude_deg = rng.uniform(35, 80)
        vehicle.gps_noise_std_m = rng.uniform(0, .7)
        vehicle.gps_drift_m_per_sample = rng.uniform(0, .015)
        vehicle.heading_bias_deg = rng.uniform(-.3, .3)
        vehicle.heading_noise_std_deg = rng.uniform(0, .25)
        vehicle.geofence_radius_m = rng.uniform(400, 500)
    elif classification == "DEGRADED_ALLOWED":
        config.simulation.bearing_noise_std_deg = rng.uniform(2.5, 7)
        digital.constant_bias_deg = rng.uniform(-4, 4)
        digital.drift_deg_per_sample = rng.uniform(-.35, .35)
        digital.heading_error_deg = rng.uniform(-3, 3)
        digital.dropout_probability = rng.uniform(.03, .15)
        digital.delay_seconds = rng.uniform(.2, 1.4)
        digital.stale_probability = rng.uniform(.02, .12)
        digital.multipath_probability = rng.uniform(.05, .25)
        digital.multipath_bias_deg = rng.uniform(10, 30) * rng.choice((-1, 1))
        digital.multipath_persistence_samples = rng.randint(1, 4)
        digital.outlier_probability = rng.uniform(.02, .10)
        digital.outlier_magnitude_deg = rng.uniform(40, 100) * rng.choice((-1, 1))
        vehicle.gps_noise_std_m = rng.uniform(.5, 3)
        vehicle.gps_drift_m_per_sample = rng.uniform(0, .08)
        vehicle.heading_bias_deg = rng.uniform(-2, 2)
        vehicle.heading_noise_std_deg = rng.uniform(.2, 1.2)
        vehicle.geofence_radius_m = rng.uniform(180, 450)
        planner_name = rng.choice(("information_gain", "information_gain", "random", "fixed_grid"))
    else:
        config.simulation.bearing_noise_std_deg = rng.uniform(2, 8)
        digital.constant_bias_deg = rng.uniform(-5, 5)
        digital.drift_deg_per_sample = rng.uniform(-.4, .4)
        digital.heading_error_deg = rng.uniform(-4, 4)
        digital.dropout_probability = rng.uniform(0, .12)
        digital.delay_seconds = rng.uniform(0, 1.5)
        digital.stale_probability = rng.uniform(0, .15)
        digital.multipath_probability = rng.uniform(.05, .3)
        digital.multipath_bias_deg = rng.uniform(15, 40) * rng.choice((-1, 1))
        digital.multipath_persistence_samples = rng.randint(2, 6)
        digital.outlier_probability = rng.uniform(.02, .12)
        digital.outlier_magnitude_deg = rng.uniform(45, 110) * rng.choice((-1, 1))
        vehicle.gps_noise_std_m = rng.uniform(.5, 3.5)
        vehicle.gps_drift_m_per_sample = rng.uniform(0, .1)
        vehicle.heading_bias_deg = rng.uniform(-3, 3)
        vehicle.heading_noise_std_deg = rng.uniform(.3, 1.5)
        fault_profile = rng.choice(("signal_loss", "stale_stream", "boundary",
                                    "telemetry_loss", "mode_change"))
        if fault_profile == "signal_loss":
            digital.dropout_probability = 1.0
            config.kraken.max_consecutive_rejections = rng.randint(4, 8)
        elif fault_profile == "stale_stream":
            digital.stale_probability = 1.0
            digital.stale_age_seconds = rng.uniform(5, 20)
            config.kraken.max_consecutive_rejections = rng.randint(4, 8)
        elif fault_profile == "boundary":
            vehicle.geofence_radius_m = rng.uniform(25, 60)
        elif fault_profile == "telemetry_loss":
            vehicle.telemetry_loss_after_samples = rng.randint(5, 10)
        else:
            vehicle.unexpected_mode_after_samples = rng.randint(5, 10)
        planner_name = rng.choice(("information_gain", "random", "fixed_grid"))

    digital.confidence_error_scale_deg = rng.uniform(16, 35)
    digital.confidence_range_scale_m = rng.uniform(700, 1600)
    config.validate()
    return config, {"initial_heading_deg": initial_heading, "planner": planner_name,
                    "fault_profile": fault_profile}


def _planner(config: AppConfig, name: str, seed: int) -> ObservationPlanner:
    args = (config.navigation.desired_cross_bearing_deg,
            config.navigation.observation_spacing_m,
            config.navigation.max_search_radius_m,
            config.navigation.max_movement_distance_m,
            config.navigation.planner_min_separation_m,
            config.navigation.planner_min_information_gain)
    if name == "random":
        return RandomPlanner(*args, seed=seed)
    if name == "fixed_grid":
        return FixedGridPlanner(*args)
    return ObservationPlanner(*args)


def run_trial(base: AppConfig, classification: str, seed: int, index: int) -> dict[str, Any]:
    config, randomized = _randomize(base, classification, seed)
    transmitter = SimulatedTransmitter(config.simulation.transmitter_lat,
                                       config.simulation.transmitter_lon)
    vehicle = SimulatedVehicleController(
        GeoPoint(config.simulation.start_lat, config.simulation.start_lon,
                 config.simulation.altitude_m), randomized["initial_heading_deg"],
        config.simulation.vehicle_twin, seed, arrival_tolerance_m=config.navigation.arrival_tolerance_m)
    source = SimulatedBearingSource(transmitter, vehicle, config.rf.center_frequency_hz,
        config.simulation.bearing_noise_std_deg, config.kraken.antenna_heading_offset_deg,
        seed, config.simulation.digital_twin)
    recorder = NullRecorder()
    mission = Mission(config, source, vehicle,
        WeightedLeastSquaresEstimator(config.df.outlier_rejection,
            config.df.outlier_threshold_m, config.df.max_leave_one_out_shift_m),
        _planner(config, randomized["planner"], seed), recorder)
    result = mission.run(max_ticks=500, tick_seconds=0)
    truth = GeoPoint(transmitter.latitude, transmitter.longitude)
    error = None if result is None else distance_m(truth, GeoPoint(result.latitude, result.longitude))
    completed = mission.state == MissionState.COMPLETE
    duration = mission.ticks_executed * .1
    stop_indices = [i for i, event in enumerate(recorder.events)
                    if event["type"] in {"safety_stop", "operator_abort"}]
    unsafe = (sum(event["type"] == "waypoint" for event in recorder.events[min(stop_indices) + 1:])
              if stop_indices else 0)
    completion_gates = bool(result is not None and error is not None and error <= 10
        and result.uncertainty_radius_m <= 15 and result.rms_residual_m <= 12
        and result.best_crossing_angle_deg >= 35
        and len(mission.observations) >= config.df.minimum_observations
        and len(mission.observations) <= 9 and duration <= 600
        and vehicle.total_distance_m <= 800)
    false_success = completed and not completion_gates
    measurement_reasons = Counter(event.get("reason", "unknown") for event in recorder.events
                                  if event["type"] == "measurement_rejected")
    safety_reasons = [event.get("reason", "unknown") for event in recorder.events
                      if event["type"] == "safety_stop"]

    measurement_records = [record for record in source.records if record.get("outcome") == "measurement"]
    detected, missed = 0, 0
    accepted_position = 0
    rejected_indices = set() if result is None else set(result.rejected_indices)
    for record, observation in zip(measurement_records, recorder.observations):
        injected = bool(record.get("effects", {}).get("outlier_deg"))
        if injected:
            caught = not observation.accepted or (observation.accepted and accepted_position in rejected_indices)
            detected += int(caught); missed += int(not caught)
        if observation.accepted:
            accepted_position += 1
    anomalous = bool(false_success or unsafe or (classification == "PASS_EXPECTED" and not completion_gates)
                     or (classification == "SAFE_FAILURE_EXPECTED" and completed))
    return {
        "trial_index": index, "seed": seed, "classification": classification,
        "scenario_name": f"randomized_{classification.lower()}",
        "randomized": randomized, "resolved_config": asdict(config), "state": mission.state.name,
        "completed": completed, "false_success": false_success, "anomalous": anomalous,
        "actual_error_m": error,
        "uncertainty_radius_m": None if result is None else result.uncertainty_radius_m,
        "truth_inside_uncertainty": None if result is None else error <= result.uncertainty_radius_m,
        "rms_residual_m": None if result is None else result.rms_residual_m,
        "best_crossing_angle_deg": None if result is None else result.best_crossing_angle_deg,
        "geometry_condition_number": None if result is None else result.geometry_condition_number,
        "observations": len(mission.observations), "ticks": mission.ticks_executed,
        "duration_seconds": duration, "distance_traveled_m": vehicle.total_distance_m,
        "estimator_rejections": 0 if result is None else result.rejected_count,
        "measurement_rejections": sum(measurement_reasons.values()),
        "measurement_rejection_reasons": dict(measurement_reasons),
        "outliers_injected": detected + missed, "outliers_detected": detected,
        "outliers_missed": missed, "safety_stop_reasons": safety_reasons,
        "unsafe_commands_after_stop": unsafe, "completion_gates_passed": completion_gates,
        "audit": {
            "simulated_emitter_truth_validation_only": {
                "latitude": transmitter.latitude, "longitude": transmitter.longitude,
            },
            "kraken_generation_records": source.records,
            "vehicle_telemetry_and_command_records": vehicle.records,
            "mission_events": recorder.events,
            "estimator_inputs": [observation.to_dict() for observation in mission.observations],
            "estimator_result": None if result is None else asdict(result),
            "final_scoring": {
                "state": mission.state.name, "actual_error_m": error,
                "false_success": false_success, "completion_gates_passed": completion_gates,
                "unsafe_commands_after_stop": unsafe,
            },
        },
    }


def _summarize(trials: list[dict[str, Any]]) -> dict[str, Any]:
    count = len(trials)
    estimated = [item for item in trials if item["actual_error_m"] is not None]
    completed = [item for item in trials if item["completed"]]
    aborts = [item for item in trials if item["state"] == "ABORTED"]
    pauses = [item for item in trials if item["state"] == "PAUSED"]
    errors = [item["actual_error_m"] for item in estimated]
    durations = [item["duration_seconds"] for item in trials]
    distances = [item["distance_traveled_m"] for item in trials]
    geometry = [item["best_crossing_angle_deg"] for item in estimated]
    uncertainty = [item["uncertainty_radius_m"] for item in estimated]
    rejection_reasons = Counter()
    abort_reasons = Counter()
    for item in trials:
        rejection_reasons.update(item["measurement_rejection_reasons"])
        abort_reasons.update(item["safety_stop_reasons"])
    injected = sum(item["outliers_injected"] for item in trials)
    detected = sum(item["outliers_detected"] for item in trials)
    return {
        "trial_count": count, "completion_rate": len(completed) / count,
        "false_success_rate": sum(item["false_success"] for item in trials) / count,
        "abort_rate": len(aborts) / count, "pause_rate": len(pauses) / count,
        "abort_pause_reasons": dict(abort_reasons),
        "localization_error_m": {"median": median(errors) if errors else None,
            "p90": _percentile(errors, 90), "p95": _percentile(errors, 95),
            "worst": max(errors, default=None)},
        "duration_seconds": {"mean": mean(durations), "p90": _percentile(durations, 90),
                             "p95": _percentile(durations, 95), "worst": max(durations)},
        "distance_traveled_m": {"mean": mean(distances), "p90": _percentile(distances, 90),
                                "p95": _percentile(distances, 95), "worst": max(distances)},
        "observation_count_distribution": dict(sorted(Counter(
            str(item["observations"]) for item in trials).items())),
        "estimator_rejection_count": sum(item["estimator_rejections"] for item in trials),
        "measurement_rejection_count": sum(item["measurement_rejections"] for item in trials),
        "measurement_rejection_reasons": dict(rejection_reasons),
        "outlier_detection": {"injected": injected, "detected": detected,
            "missed": injected - detected, "success_rate": None if not injected else detected / injected},
        "best_crossing_angle_deg": {"median": median(geometry) if geometry else None,
            "p05": _percentile(geometry, 5), "p95": _percentile(geometry, 95)},
        "reported_uncertainty_m": {"median": median(uncertainty) if uncertainty else None,
            "p90": _percentile(uncertainty, 90), "p95": _percentile(uncertainty, 95),
            "worst": max(uncertainty, default=None)},
        "uncertainty_containment_rate": (None if not estimated else
            sum(bool(item["truth_inside_uncertainty"]) for item in estimated) / len(estimated)),
    }


def run_campaign(base: AppConfig, trials: int = 500, master_seed: int = 20260828) -> dict[str, Any]:
    if trials < 1:
        raise ValueError("trial count must be positive")
    rng = random.Random(master_seed)
    seeds = rng.sample(range(1, 2_147_483_647), trials)
    results = [run_trial(base, _classification(index, trials), seed, index)
               for index, seed in enumerate(seeds)]
    categories = {name: _summarize([item for item in results if item["classification"] == name])
                  for name in CLASSIFICATIONS}
    nominal = [item for item in results if item["classification"] == "PASS_EXPECTED"]
    nominal_within = sum(item["actual_error_m"] is not None and item["actual_error_m"] <= 10
                         for item in nominal) / len(nominal)
    nominal_success = sum(item["completion_gates_passed"] for item in nominal) / len(nominal)
    overall = _summarize(results)
    containment = categories["PASS_EXPECTED"]["uncertainty_containment_rate"]
    target_results = {
        "nominal_within_10m_rate": nominal_within,
        "nominal_within_10m_pass": nominal_within >= S0_TARGETS["nominal_within_10m_rate_min"],
        "nominal_all_criteria_success_rate": nominal_success,
        "false_success_rate": overall["false_success_rate"],
        "false_success_pass": overall["false_success_rate"] <= S0_TARGETS["false_success_rate_max"],
        "nominal_uncertainty_containment_rate": containment,
        "containment_pass": containment is not None and
            S0_TARGETS["uncertainty_containment_rate_min"] <= containment <=
            S0_TARGETS["uncertainty_containment_rate_max"],
        "zero_unsafe_commands": not any(item["unsafe_commands_after_stop"] for item in results),
    }
    target_results["campaign_passed"] = all(value for key, value in target_results.items()
                                                  if key.endswith("_pass") or key == "zero_unsafe_commands")
    trace_seeds = [{"trial_index": item["trial_index"], "seed": item["seed"],
                    "classification": item["classification"], "state": item["state"],
                    "false_success": item["false_success"],
                    "resolved_config": item["resolved_config"]}
                   for item in results if item["anomalous"]]
    evidence = environment_manifest()
    for item in results:
        item["evidence_manifest_id"] = evidence["source"]["archive_identifier"]
        item["ardupilot"] = {"applicable": False, "reason": "offline Monte Carlo trial"}
    return {"schema_version": 2, "generated_at": datetime.now(timezone.utc).isoformat(),
            "master_seed": master_seed, "s0_targets": S0_TARGETS,
            "evidence_manifest": evidence,
            "total_trials": trials, "category_allocation": dict(Counter(
                item["classification"] for item in results)), "overall": overall,
            "categories": categories, "target_results": target_results,
            "failed_or_anomalous_trial_seeds": trace_seeds, "trials": results}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    parser.add_argument("--trials", type=int, default=500)
    parser.add_argument("--master-seed", type=int, default=20260828)
    parser.add_argument("--output", default="./logs/stage7_monte_carlo_report.json")
    args = parser.parse_args()
    if args.trials < 500:
        parser.error("the final S7 campaign requires at least 500 trials")
    report = run_campaign(load_config(args.config), args.trials, args.master_seed)
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2, default=str) + "\n", encoding="utf-8")
    print(json.dumps({"total_trials": report["total_trials"],
        "category_allocation": report["category_allocation"], "overall": report["overall"],
        "categories": report["categories"], "target_results": report["target_results"],
        "report": str(target.resolve())}, indent=2))
    return 0 if report["target_results"]["campaign_passed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
