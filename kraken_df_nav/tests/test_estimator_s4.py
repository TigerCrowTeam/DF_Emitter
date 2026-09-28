from datetime import datetime, timezone
import math
import random

import pytest

from kraken_df_nav.df.estimator import WeightedLeastSquaresEstimator
from kraken_df_nav.df.geometry import GeoPoint, destination, distance_m, initial_bearing_deg
from kraken_df_nav.df.observation import Observation


TX = GeoPoint(37.001, -121.999)


def observation(point, error=0.0, confidence=.95, accepted=True,
                position_accuracy=0.0, heading_accuracy=0.0, target=TX):
    bearing = (initial_bearing_deg(point, target) + error) % 360
    return Observation(datetime.now(timezone.utc).isoformat(), point.latitude, point.longitude,
        50, 0, bearing, bearing, -60, confidence, accepted=accepted,
        position_accuracy_m=position_accuracy, heading_accuracy_deg=heading_accuracy)


def ring(errors=None, confidence=.95, position_accuracy=0, heading_accuracy=0):
    angles = [190, 235, 280, 325, 10, 55, 100]
    errors = errors or [0] * len(angles)
    return [observation(destination(TX, angle, 180), error, confidence,
                        position_accuracy=position_accuracy, heading_accuracy=heading_accuracy)
            for angle, error in zip(angles, errors)]


def test_exact_two_line_intersection_and_diagnostics():
    result = WeightedLeastSquaresEstimator().estimate([
        observation(destination(TX, 210, 150)),
        observation(destination(TX, 300, 150)),
    ])
    assert distance_m(TX, GeoPoint(result.latitude, result.longitude)) < .01
    assert result.best_crossing_angle_deg == pytest.approx(90, abs=.1)
    assert result.geometry_condition_number < 2
    assert result.max_leave_one_out_shift_m == 0


def test_seven_observation_solution_and_leave_one_out_sensitivity():
    result = WeightedLeastSquaresEstimator().estimate(ring([1, -1, .5, -.5, 1, -1, .25]))
    assert result.accepted_count == 7
    assert distance_m(TX, GeoPoint(result.latitude, result.longitude)) < 5
    assert result.max_leave_one_out_shift_m > 0
    assert result.best_crossing_angle_deg > 80


def test_low_confidence_reduces_influence_of_bad_line():
    points = [destination(TX, angle, 150) for angle in (200, 260, 320)]
    low = [observation(points[0]), observation(points[1]), observation(points[2], 18, .1)]
    high = [observation(points[0]), observation(points[1]), observation(points[2], 18, .99)]
    estimator = WeightedLeastSquaresEstimator(outlier_rejection=False,
                                               max_leave_one_out_shift_m=math.inf)
    low_result, high_result = estimator.estimate(low), estimator.estimate(high)
    low_error = distance_m(TX, GeoPoint(low_result.latitude, low_result.longitude))
    high_error = distance_m(TX, GeoPoint(high_result.latitude, high_result.longitude))
    assert low_error < high_error


def test_two_gross_outliers_are_rejected_by_consensus():
    observations = ring([0, 1, -1, 0, 80, -75, .5])
    result = WeightedLeastSquaresEstimator(True, 20).estimate(observations)
    assert result.rejected_count >= 2
    assert distance_m(TX, GeoPoint(result.latitude, result.longitude)) < 8


def test_persistent_multipath_subset_is_rejected():
    observations = ring([0, 0, 35, 35, 0, 0, 0])
    result = WeightedLeastSquaresEstimator(True, 20).estimate(observations)
    assert result.rejected_count >= 2
    assert distance_m(TX, GeoPoint(result.latitude, result.longitude)) < 8


def test_declared_pose_and_heading_uncertainty_inflate_result_uncertainty():
    clean = WeightedLeastSquaresEstimator().estimate(ring([1, -1, 1, -1, 1, -1, 1]))
    uncertain = WeightedLeastSquaresEstimator().estimate(
        ring([1, -1, 1, -1, 1, -1, 1], position_accuracy=5, heading_accuracy=5))
    assert uncertain.uncertainty_radius_m > clean.uncertainty_radius_m


def test_constant_bias_is_only_covered_when_declared_as_heading_uncertainty():
    biased = ring([8] * 7, heading_accuracy=8)
    result = WeightedLeastSquaresEstimator().estimate(biased)
    error = distance_m(TX, GeoPoint(result.latitude, result.longitude))
    assert error > 5
    assert result.uncertainty_radius_m >= error


def test_small_residual_does_not_hide_weak_geometry():
    observations = [observation(destination(TX, angle, 150)) for angle in (180, 182, 184)]
    result = WeightedLeastSquaresEstimator().estimate(observations)
    assert result.rms_residual_m < .1
    assert result.geometry_condition_number > 10
    assert result.uncertainty_radius_m > 20


def test_conflicting_equal_clusters_are_rejected():
    alternate = destination(TX, 90, 250)
    points = [destination(TX, angle, 180) for angle in (200, 260, 320, 20, 80, 140)]
    observations = [observation(point, target=TX if index < 3 else alternate)
                    for index, point in enumerate(points)]
    with pytest.raises(ValueError, match="inconsistent bearing clusters"):
        WeightedLeastSquaresEstimator(True, 20).estimate(observations)


def test_original_indices_are_preserved_across_input_and_estimator_rejection():
    ignored = observation(destination(TX, 150, 100), accepted=False)
    observations = [ignored, *ring([0, 0, 0, 80, 0, 0, 0])]
    result = WeightedLeastSquaresEstimator(True, 20).estimate(observations)
    assert 0 not in result.used_indices
    assert 4 in result.rejected_indices


def test_non_finite_accepted_observation_is_refused():
    observations = ring()
    observations[0].absolute_bearing_deg = math.nan
    with pytest.raises(ValueError, match="non-finite"):
        WeightedLeastSquaresEstimator().estimate(observations)


def test_uncertainty_contains_truth_in_most_seeded_nominal_trials():
    contained = 0
    for seed in range(100):
        randomizer = random.Random(seed)
        observations = ring([randomizer.gauss(0, 2) for _ in range(7)])
        result = WeightedLeastSquaresEstimator().estimate(observations)
        error = distance_m(TX, GeoPoint(result.latitude, result.longitude))
        contained += error <= result.uncertainty_radius_m
    assert contained >= 80
