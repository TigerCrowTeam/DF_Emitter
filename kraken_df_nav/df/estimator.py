"""Robust weighted least-squares bearing-line estimator."""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import numpy as np

from .geometry import GeoPoint, LocalFrame
from .observation import Observation


@dataclass
class EstimateResult:
    latitude: float
    longitude: float
    uncertainty_radius_m: float
    rms_residual_m: float
    accepted_count: int
    rejected_count: int
    used_indices: list[int] = field(default_factory=list)
    rejected_indices: list[int] = field(default_factory=list)
    best_crossing_angle_deg: float = 0.0
    geometry_condition_number: float = math.inf
    max_leave_one_out_shift_m: float = 0.0


class BearingEstimator:
    """Estimator interface suitable for replacement by a Bayesian implementation."""

    def estimate(self, observations: list[Observation]) -> EstimateResult:
        raise NotImplementedError


class WeightedLeastSquaresEstimator(BearingEstimator):
    def __init__(self, outlier_rejection: bool = True, outlier_threshold_m: float = 75.0,
                 max_leave_one_out_shift_m: float = 30.0):
        self.outlier_rejection = outlier_rejection
        self.outlier_threshold_m = outlier_threshold_m
        self.max_leave_one_out_shift_m = max_leave_one_out_shift_m

    @staticmethod
    def _solve(points: np.ndarray, bearings: np.ndarray,
               weights: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
        radians = np.radians(bearings)
        normals = np.column_stack((np.cos(radians), -np.sin(radians)))
        root_w = np.sqrt(weights)[:, None]
        a = normals * root_w
        b = np.sum(normals * points, axis=1) * root_w[:, 0]
        singular_values = np.linalg.svd(a, compute_uv=False)
        # matrix_rank alone accepts numerically non-identical but operationally
        # parallel bearings. Reject geometry with an extreme condition ratio.
        if len(singular_values) < 2 or singular_values[-1] <= singular_values[0] * 1e-3:
            raise ValueError("bearing geometry is singular or nearly parallel")
        condition = float(singular_values[0] / singular_values[-1])
        solution, _, _, _ = np.linalg.lstsq(a, b, rcond=None)
        residuals = np.abs(np.sum(normals * (solution - points), axis=1))
        return solution, residuals, normals, condition

    @staticmethod
    def _best_crossing_angle(bearings: np.ndarray) -> float:
        if len(bearings) < 2:
            return 0.0
        result = 0.0
        for index, first in enumerate(bearings):
            for second in bearings[index + 1:]:
                difference = abs((float(first) - float(second)) % 180.0)
                result = max(result, min(difference, 180.0 - difference))
        return result

    def estimate(self, observations: list[Observation]) -> EstimateResult:
        valid_pairs = [(index, observation) for index, observation in enumerate(observations)
                       if observation.accepted]
        valid = [observation for _, observation in valid_pairs]
        original_indices = np.array([index for index, _ in valid_pairs])
        if len(valid) < 2:
            raise ValueError("at least two accepted observations are required")
        numeric = [(o.latitude, o.longitude, o.absolute_bearing_deg, o.confidence,
                    o.position_accuracy_m, o.heading_accuracy_deg) for o in valid]
        if not all(math.isfinite(value) for row in numeric for value in row):
            raise ValueError("accepted observation contains non-finite value")
        if any(o.position_accuracy_m < 0 or o.heading_accuracy_deg < 0 for o in valid):
            raise ValueError("observation accuracy values must be non-negative")
        origin = GeoPoint(valid[0].latitude, valid[0].longitude, valid[0].altitude_m)
        frame = LocalFrame(origin)
        points = np.array([frame.to_enu(GeoPoint(o.latitude, o.longitude, o.altitude_m))[:2] for o in valid])
        bearings = np.array([o.absolute_bearing_deg for o in valid])
        weights = np.array([max(0.05, min(1.0, o.confidence)) ** 2 for o in valid])
        used = np.arange(len(valid))
        solution, residuals, normals, condition = self._solve(points, bearings, weights)

        rejected_positions: list[int] = []
        if self.outlier_rejection and len(valid) >= 4:
            # Generate pair and leave-one-out consensus candidates. Minimizing
            # median residual prevents several bad lines from pulling the ordinary
            # solution far enough to mask their own residuals.
            best_solution = solution
            best_score = float(np.median(residuals))
            all_radians = np.radians(bearings)
            all_normals = np.column_stack((np.cos(all_radians), -np.sin(all_radians)))
            candidates: list[np.ndarray] = []
            for omitted in range(len(valid)):
                trial_mask = np.arange(len(valid)) != omitted
                try:
                    trial, _, _, _ = self._solve(points[trial_mask], bearings[trial_mask], weights[trial_mask])
                except ValueError:
                    continue
                candidates.append(trial)
            for first in range(len(valid)):
                for second in range(first + 1, len(valid)):
                    pair = np.array([first, second])
                    try:
                        trial, _, _, _ = self._solve(points[pair], bearings[pair], weights[pair])
                    except ValueError:
                        continue
                    candidates.append(trial)
            for trial in candidates:
                trial_residuals = np.abs(np.sum(all_normals * (trial - points), axis=1))
                score = float(np.median(trial_residuals))
                if score < best_score:
                    best_solution, best_score = trial, score
            solution = best_solution
            residuals = np.abs(np.sum(all_normals * (solution - points), axis=1))
            median = float(np.median(residuals))
            mad = float(np.median(np.abs(residuals - median)))
            cutoff = max(self.outlier_threshold_m, median + 3.5 * max(1.0, 1.4826 * mad))
            mask = residuals <= cutoff
            if np.count_nonzero(mask) >= 2 and not np.all(mask):
                rejected_positions = used[~mask].tolist()
                used = used[mask]
                solution, residuals, normals, condition = self._solve(
                    points[mask], bearings[mask], weights[mask])
                points, bearings, weights = points[mask], bearings[mask], weights[mask]
            if len(used) * 2 <= len(valid):
                raise ValueError("inconsistent bearing clusters leave no supported majority")

        rms = float(np.sqrt(np.average(residuals**2, weights=weights)))
        # Approximate each angular measurement as cross-track error at estimated range.
        ranges = np.linalg.norm(points - solution, axis=1)
        angular_sigma_rad = np.radians(np.maximum(1.0, 8.0 * (1.0 - np.sqrt(weights))))
        position_sigma = np.array([valid[index].position_accuracy_m for index in used])
        heading_sigma_rad = np.radians(np.array([valid[index].heading_accuracy_deg for index in used]))
        line_sigma = np.sqrt(np.maximum(1.5, ranges * angular_sigma_rad) ** 2
                             + position_sigma ** 2 + (ranges * heading_sigma_rad) ** 2)
        information = normals.T @ np.diag(weights / line_sigma**2) @ normals
        try:
            covariance = np.linalg.inv(information)
            uncertainty = 2.4477 * math.sqrt(float(np.max(np.linalg.eigvalsh(covariance))))
        except np.linalg.LinAlgError as exc:
            raise ValueError("bearing geometry cannot produce an uncertainty estimate") from exc
        uncertainty = max(uncertainty, 2.0 * rms)
        loo_shifts = []
        if len(used) >= 3:
            for omitted in range(len(used)):
                mask = np.arange(len(used)) != omitted
                try:
                    trial, _, _, _ = self._solve(points[mask], bearings[mask], weights[mask])
                except ValueError:
                    continue
                loo_shifts.append(float(np.linalg.norm(trial - solution)))
        max_loo_shift = max(loo_shifts, default=0.0)
        if condition < 10 and max_loo_shift > self.max_leave_one_out_shift_m:
            raise ValueError("estimator is unstable or has inconsistent bearing clusters")
        estimate = frame.from_enu(float(solution[0]), float(solution[1]))
        used_original = original_indices[used].tolist()
        rejected_original = original_indices[rejected_positions].tolist()
        return EstimateResult(
            estimate.latitude, estimate.longitude, uncertainty, rms,
            len(used), len(valid) - len(used), used_original, rejected_original,
            self._best_crossing_angle(bearings), condition, max_loo_shift,
        )
