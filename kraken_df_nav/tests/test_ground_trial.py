import pytest

from kraken_df_nav.kraken.ground_trial import circular_mean, load_points


def test_circular_mean_wraps_across_north():
    result = circular_mean([359.0, 1.0], [1.0, 1.0])
    assert min(abs(result), abs(result - 360.0)) < 1e-9


def test_load_points_rejects_invalid_heading(tmp_path):
    path = tmp_path / "points.csv"
    path.write_text("name,latitude,longitude,array_heading_deg\nP1,37,-122,360\n",
                    encoding="utf-8")
    with pytest.raises(ValueError, match="invalid array heading"):
        load_points(str(path))


def test_load_points_accepts_optional_altitude(tmp_path):
    path = tmp_path / "points.csv"
    path.write_text("name,latitude,longitude,array_heading_deg\nP1,37,-122,45\n",
                    encoding="utf-8")
    assert load_points(str(path))[0]["altitude_m"] == 0.0
