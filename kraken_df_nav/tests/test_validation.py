import json

from kraken_df_nav.df.estimator import WeightedLeastSquaresEstimator
from kraken_df_nav.df.geometry import GeoPoint, destination, initial_bearing_deg
from kraken_df_nav.df.observation import Observation
from kraken_df_nav.validation import best_crossing_angle, validate_trials


def test_best_crossing_angle_treats_bearings_as_lines():
    assert best_crossing_angle([5, 95, 185]) == 90


def test_validation_report_against_known_truth(tmp_path):
    truth = GeoPoint(37.001, -121.999)
    observations = []
    for index, angle in enumerate((210, 300, 80)):
        point = destination(truth, angle, 150)
        bearing = initial_bearing_deg(point, truth)
        observations.append(Observation(
            timestamp=f"2026-01-01T00:00:0{index}Z", latitude=point.latitude,
            longitude=point.longitude, altitude_m=50, vehicle_heading_deg=0,
            relative_bearing_deg=bearing, absolute_bearing_deg=bearing,
            rssi_dbm=-60, confidence=.95,
        ).to_dict())
    path = tmp_path / "trial.json"
    path.write_text(json.dumps({"observations": observations}), encoding="utf-8")

    report = validate_trials([str(path)], truth, WeightedLeastSquaresEstimator(), 10)

    assert report["trial_count"] == 1
    assert report["target_pass_rate"] == 1
    assert report["uncertainty_containment_rate"] == 1
    assert report["trials"][0]["actual_error_m"] < 0.1
    assert report["trials"][0]["best_crossing_angle_deg"] >= 80
