"""Accuracy and uncertainty validation for recorded DF trials."""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
import json

from .df.estimator import WeightedLeastSquaresEstimator
from .df.geometry import GeoPoint, distance_m
from .logging_utils.replay import load_observations


@dataclass
class TrialValidation:
    path: str
    actual_error_m: float
    uncertainty_radius_m: float
    within_target: bool
    truth_inside_uncertainty: bool
    observations_logged: int
    accepted_observations: int
    estimator_accepted_count: int
    estimator_rejected_count: int
    rms_residual_m: float
    best_crossing_angle_deg: float
    geometry_condition_number: float
    max_leave_one_out_shift_m: float


def _crossing_angle(a: float, b: float) -> float:
    """Return the acute angle between two unoriented bearing lines."""
    difference = abs((a - b) % 180.0)
    return min(difference, 180.0 - difference)


def best_crossing_angle(bearings: list[float]) -> float:
    if len(bearings) < 2:
        return 0.0
    return max(_crossing_angle(a, b) for i, a in enumerate(bearings) for b in bearings[i + 1:])


def discover_trial_files(inputs: list[str]) -> list[Path]:
    """Resolve explicit files/directories, preferring JSON over duplicate CSV logs."""
    found: dict[str, Path] = {}
    for value in inputs:
        path = Path(value)
        if path.is_dir():
            candidates = sorted(path.glob("mission_*.json")) + sorted(path.glob("mission_*.csv"))
            if not candidates:
                candidates = sorted(path.glob("*.json")) + sorted(path.glob("*.csv"))
        else:
            candidates = [path]
        for candidate in candidates:
            if candidate.suffix.lower() not in {".json", ".csv"}:
                continue
            key = str(candidate.with_suffix(""))
            if key not in found or candidate.suffix.lower() == ".json":
                found[key] = candidate
    if not found:
        raise ValueError("no JSON or CSV trial files found")
    return sorted(found.values())


def validate_trials(inputs: list[str], truth: GeoPoint, estimator: WeightedLeastSquaresEstimator,
                    target_error_m: float) -> dict[str, object]:
    trials: list[TrialValidation] = []
    failures: list[dict[str, str]] = []
    for path in discover_trial_files(inputs):
        try:
            observations = load_observations(str(path))
            result = estimator.estimate(observations)
            error = distance_m(truth, GeoPoint(result.latitude, result.longitude))
            accepted_bearings = [o.absolute_bearing_deg for o in observations if o.accepted]
            trials.append(TrialValidation(
                path=str(path), actual_error_m=error,
                uncertainty_radius_m=result.uncertainty_radius_m,
                within_target=error <= target_error_m,
                truth_inside_uncertainty=error <= result.uncertainty_radius_m,
                observations_logged=len(observations),
                accepted_observations=len(accepted_bearings),
                estimator_accepted_count=result.accepted_count,
                estimator_rejected_count=result.rejected_count,
                rms_residual_m=result.rms_residual_m,
                best_crossing_angle_deg=result.best_crossing_angle_deg,
                geometry_condition_number=result.geometry_condition_number,
                max_leave_one_out_shift_m=result.max_leave_one_out_shift_m,
            ))
        except (OSError, TypeError, ValueError, json.JSONDecodeError) as exc:
            failures.append({"path": str(path), "error": str(exc)})

    count = len(trials)
    errors = [trial.actual_error_m for trial in trials]
    uncertainties = [trial.uncertainty_radius_m for trial in trials]
    return {
        "truth": {"latitude": truth.latitude, "longitude": truth.longitude},
        "target_error_m": target_error_m,
        "trial_count": count,
        "failed_file_count": len(failures),
        "target_pass_rate": sum(t.within_target for t in trials) / count if count else 0.0,
        "uncertainty_containment_rate": sum(t.truth_inside_uncertainty for t in trials) / count if count else 0.0,
        "mean_actual_error_m": sum(errors) / count if count else None,
        "max_actual_error_m": max(errors) if errors else None,
        "mean_uncertainty_radius_m": sum(uncertainties) / count if count else None,
        "trials": [asdict(trial) for trial in trials],
        "failures": failures,
    }


def write_validation_report(report: dict[str, object], output: str | None = None) -> str:
    rendered = json.dumps(report, indent=2)
    if output:
        Path(output).write_text(rendered + "\n", encoding="utf-8")
    return rendered
