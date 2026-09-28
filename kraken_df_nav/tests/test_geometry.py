import pytest

from kraken_df_nav.df.geometry import GeoPoint, LocalFrame, destination, distance_m, initial_bearing_deg


def test_enu_round_trip():
    origin = GeoPoint(37, -122, 10)
    point = GeoPoint(37.001, -121.999, 15)
    recovered = LocalFrame(origin).from_enu(*LocalFrame(origin).to_enu(point))
    assert recovered.latitude == pytest.approx(point.latitude, abs=1e-9)
    assert recovered.longitude == pytest.approx(point.longitude, abs=1e-9)
    assert recovered.altitude_m == pytest.approx(15)


def test_destination_distance_and_bearing():
    start = GeoPoint(37, -122)
    end = destination(start, 90, 100)
    assert distance_m(start, end) == pytest.approx(100, rel=1e-4)
    assert initial_bearing_deg(start, end) == pytest.approx(90, abs=0.01)
