"""Run deterministic simulation scenarios with declared expected outcomes."""
from __future__ import annotations

import argparse
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path

import yaml

from .config import AppConfig, load_config
from .df.estimator import WeightedLeastSquaresEstimator
from .df.geometry import GeoPoint, distance_m
from .kraken.client import BearingSource
from .kraken.models import BearingMeasurement
from .logging_utils.recorder import NullRecorder
from .mission.observation_planner import ObservationPlanner
from .mission.state_machine import Mission, MissionState
from .simulation.kraken_simulator import SimulatedBearingSource, SimulatedVehicleController
from .simulation.transmitter import SimulatedTransmitter


def apply_overrides(instance, values: dict) -> None:
    for key, value in values.items():
        if not hasattr(instance, key):
            raise ValueError(f"unknown scenario configuration key: {key}")
        current = getattr(instance, key)
        if hasattr(current, "__dataclass_fields__"):
            if not isinstance(value, dict):
                raise ValueError(f"scenario section {key} must be a mapping")
            apply_overrides(current, value)
        else:
            setattr(instance, key, value)


class ScriptedSource(BearingSource):
    def __init__(self, source: BearingSource, bearing_offsets: dict[int, float] | None = None,
                 constant_absolute_bearing_deg: float | None = None,
                 vehicle=None, interrupt_after: int = 0):
        self.source, self.bearing_offsets = source, bearing_offsets or {}
        self.constant_absolute_bearing_deg = constant_absolute_bearing_deg
        self.vehicle, self.interrupt_after, self.count = vehicle, interrupt_after, 0

    def get_measurement(self):
        self.count += 1
        if self.interrupt_after and self.count >= self.interrupt_after:
            raise KeyboardInterrupt
        measurement = self.source.get_measurement()
        if measurement is None:
            return None
        bearing = measurement.relative_bearing_deg + self.bearing_offsets.get(self.count, 0)
        if self.constant_absolute_bearing_deg is not None:
            state = self.vehicle.get_vehicle_state()
            bearing = self.constant_absolute_bearing_deg - state.heading_deg
        return BearingMeasurement(bearing % 360, measurement.rssi_dbm,
                                  measurement.confidence, measurement.frequency_hz,
                                  measurement.timestamp)

    def close(self):
        self.source.close()


def load_catalog(path: str | Path) -> dict:
    payload = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    if not isinstance(payload, dict) or not isinstance(payload.get("scenarios"), list):
        raise ValueError("scenario catalog must contain a scenarios list")
    names = [item.get("name") for item in payload["scenarios"]]
    if any(not name for name in names) or len(names) != len(set(names)):
        raise ValueError("every scenario requires a unique name")
    required = {"name", "description", "classification", "seed", "expected"}
    if any(not required.issubset(item) for item in payload["scenarios"]):
        raise ValueError("each scenario requires name, description, classification, seed, and expected")
    return payload


def run_scenario(base: AppConfig, definition: dict) -> dict[str, object]:
    config = deepcopy(base)
    apply_overrides(config, definition.get("overrides", {}))
    seed = int(definition["seed"])
    config.simulation.random_seed = seed
    config.navigation.stabilization_seconds = 0
    config.validate()
    transmitter = SimulatedTransmitter(config.simulation.transmitter_lat,
                                       config.simulation.transmitter_lon)
    vehicle = SimulatedVehicleController(GeoPoint(config.simulation.start_lat,
        config.simulation.start_lon, config.simulation.altitude_m),
        effects=config.simulation.vehicle_twin, seed=seed,
        arrival_tolerance_m=config.navigation.arrival_tolerance_m)
    raw_source = SimulatedBearingSource(transmitter, vehicle, config.rf.center_frequency_hz,
        config.simulation.bearing_noise_std_deg,
        config.kraken.antenna_heading_offset_deg, seed,
        config.simulation.digital_twin)
    hooks = definition.get("hooks", {})
    source = ScriptedSource(raw_source,
        {int(key): float(value) for key, value in hooks.get("bearing_offsets", {}).items()},
        hooks.get("constant_absolute_bearing_deg"), vehicle,
        int(hooks.get("operator_interrupt_after", 0)))
    recorder = NullRecorder()
    mission = Mission(config, source, vehicle,
        WeightedLeastSquaresEstimator(config.df.outlier_rejection,
            config.df.outlier_threshold_m, config.df.max_leave_one_out_shift_m),
        ObservationPlanner(config.navigation.desired_cross_bearing_deg,
            config.navigation.observation_spacing_m,
            config.navigation.max_search_radius_m,
            config.navigation.max_movement_distance_m,
            config.navigation.planner_min_separation_m,
            config.navigation.planner_min_information_gain), recorder)
    result = mission.run(max_ticks=int(definition.get("max_ticks", 400)), tick_seconds=0)
    truth = GeoPoint(transmitter.latitude, transmitter.longitude)
    error = None if result is None else distance_m(truth, GeoPoint(result.latitude, result.longitude))
    stop_indices = [index for index, event in enumerate(recorder.events)
                    if event["type"] in {"safety_stop", "operator_abort"}]
    unsafe_commands = (sum(event["type"] == "waypoint"
                           for event in recorder.events[min(stop_indices) + 1:])
                       if stop_indices else 0)
    false_success = bool(mission.state == MissionState.COMPLETE and (
        error is None or error > 10 or result.uncertainty_radius_m > 15
        or result.rms_residual_m > 12 or result.best_crossing_angle_deg < 35))
    expected = definition["expected"]
    outcome, reasons = expected["outcome"], []
    if outcome == "complete":
        if mission.state != MissionState.COMPLETE:
            reasons.append(f"expected COMPLETE, got {mission.state.name}")
        if error is None or error > float(expected.get("max_error_m", 10)):
            reasons.append(f"actual error {error} exceeds target")
    elif outcome == "safe_failure":
        if mission.state not in {MissionState.ABORTED, MissionState.PAUSED}:
            reasons.append(f"expected terminal safe failure, got {mission.state.name}")
    elif outcome == "degraded":
        if mission.state not in {MissionState.COMPLETE, MissionState.ABORTED, MissionState.PAUSED}:
            reasons.append(f"degraded scenario did not terminate: {mission.state.name}")
    else:
        reasons.append(f"unknown expected outcome: {outcome}")
    if false_success:
        reasons.append("false success detected")
    if unsafe_commands:
        reasons.append(f"{unsafe_commands} waypoint commands followed a safety stop")
    if expected.get("require_rejection") and not any(
            event["type"] == "measurement_rejected" for event in recorder.events):
        reasons.append("expected at least one measurement rejection")
    return {
        "name": definition["name"], "classification": definition["classification"],
        "description": definition["description"], "seed": seed,
        "expected": expected, "resolved_config": asdict(config),
        "state": mission.state.name, "passed": not reasons, "failure_reasons": reasons,
        "actual_error_m": error,
        "uncertainty_radius_m": None if result is None else result.uncertainty_radius_m,
        "rms_residual_m": None if result is None else result.rms_residual_m,
        "best_crossing_angle_deg": None if result is None else result.best_crossing_angle_deg,
        "observations": len(mission.observations),
        "measurement_rejections": sum(event["type"] == "measurement_rejected" for event in recorder.events),
        "waypoint_count": sum(event["type"] == "waypoint" for event in recorder.events),
        "unsafe_commands_after_stop": unsafe_commands, "false_success": false_success,
        "ticks": mission.ticks_executed, "events": recorder.events,
    }


def run_catalog(catalog_path: str | Path, base_config_path: str | Path) -> dict[str, object]:
    catalog, base = load_catalog(catalog_path), load_config(base_config_path)
    results = [run_scenario(base, definition) for definition in catalog["scenarios"]]
    return {"catalog": str(catalog_path), "base_config": str(base_config_path),
            "scenario_count": len(results), "passed_count": sum(item["passed"] for item in results),
            "failed_count": sum(not item["passed"] for item in results), "results": results}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--catalog", default=str(Path(__file__).with_name("simulation_scenarios.yaml")))
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.yaml")))
    parser.add_argument("--output", default="./logs/stage6_scenario_report.json")
    args = parser.parse_args()
    report = run_catalog(args.catalog, args.config)
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2, default=str) + "\n", encoding="utf-8")
    summary = {"scenario_count": report["scenario_count"], "passed_count": report["passed_count"],
               "failed_count": report["failed_count"],
               "results": [{key: item[key] for key in ("name", "state", "passed", "failure_reasons")}
                           for item in report["results"]]}
    print(json.dumps(summary, indent=2))
    return 0 if report["failed_count"] == 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
