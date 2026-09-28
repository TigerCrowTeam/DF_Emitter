from datetime import datetime, timezone

import pytest

from kraken_df_nav.df.geometry import GeoPoint, distance_m
from kraken_df_nav.df.observation import Observation
from kraken_df_nav.mission.observation_planner import ObservationPlanner


def test_planner_cross_bearing_and_limits():
    current = GeoPoint(37, -122, 50)
    observation = Observation(datetime.now(timezone.utc).isoformat(), 37, -122, 50, 0, 20, 20, -60, .9)
    planner = ObservationPlanner(60, 100, 500, 150)
    waypoint = planner.plan(current, [observation])
    assert distance_m(current, waypoint.point) == pytest.approx(100, rel=.01)
    assert "80.0 deg" in waypoint.rationale


def test_planner_caps_step():
    current = GeoPoint(37, -122)
    observation = Observation("now", 37, -122, 0, 0, 0, 0, -60, .9)
    waypoint = ObservationPlanner(spacing_m=300, max_movement_distance_m=120).plan(current, [observation])
    assert distance_m(current, waypoint.point) <= 120.01


def test_planner_refuses_when_no_waypoint_fits_search_radius():
    current = GeoPoint(37, -122)
    observation = Observation("now", 37, -122, 0, 0, 0, 0, -60, .9)
    planner = ObservationPlanner(spacing_m=100, max_search_radius_m=50, max_movement_distance_m=100)
    with pytest.raises(ValueError, match="no safe waypoint"):
        planner.plan(current, [observation])
