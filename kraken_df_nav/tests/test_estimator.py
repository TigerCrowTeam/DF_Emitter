from datetime import datetime, timezone

import pytest

from kraken_df_nav.df.estimator import WeightedLeastSquaresEstimator
from kraken_df_nav.df.geometry import GeoPoint, destination, distance_m, initial_bearing_deg
from kraken_df_nav.df.observation import Observation


TX = GeoPoint(37.001, -121.999)


def obs(point, bearing_error=0, confidence=.95):
    bearing = (initial_bearing_deg(point, TX) + bearing_error) % 360
    return Observation(datetime.now(timezone.utc).isoformat(), point.latitude, point.longitude, 50, 0,
                       bearing, bearing, -60, confidence)


def test_weighted_line_intersection():
    observations = [obs(destination(TX, 210, 150), 1), obs(destination(TX, 300, 180), -1), obs(destination(TX, 80, 120), .5)]
    result = WeightedLeastSquaresEstimator().estimate(observations)
    assert distance_m(TX, GeoPoint(result.latitude, result.longitude)) < 8
    assert result.accepted_count == 3


def test_outlier_rejection():
    observations = [obs(destination(TX, angle, 150), err) for angle, err in [(200, 0), (260, 1), (320, -1), (30, 80)]]
    result = WeightedLeastSquaresEstimator(True, 20).estimate(observations)
    assert result.rejected_count >= 1
    assert distance_m(TX, GeoPoint(result.latitude, result.longitude)) < 10


def test_parallel_bearings_are_rejected_as_singular_geometry():
    points = [destination(TX, 180, 100), destination(TX, 180, 200)]
    observations = [obs(point) for point in points]
    with pytest.raises(ValueError, match="singular or nearly parallel"):
        WeightedLeastSquaresEstimator().estimate(observations)
