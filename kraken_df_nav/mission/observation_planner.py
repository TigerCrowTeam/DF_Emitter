"""Pure observation-position planning, independent of MAVLink."""
from __future__ import annotations

from dataclasses import dataclass
import math
import random

import numpy as np

from ..df.geometry import GeoPoint, destination, distance_m, initial_bearing_deg
from ..df.observation import Observation


@dataclass(frozen=True)
class PlannedWaypoint:
    point: GeoPoint
    rationale: str


class ObservationPlanner:
    def __init__(self, desired_cross_bearing_deg: float = 60, spacing_m: float = 100,
                 max_search_radius_m: float = 500, max_movement_distance_m: float = 200,
                 min_separation_m: float = 20, min_information_gain: float = .01):
        self.desired_cross_bearing_deg = desired_cross_bearing_deg
        self.spacing_m = spacing_m
        self.max_search_radius_m = max_search_radius_m
        self.max_movement_distance_m = max_movement_distance_m
        self.min_separation_m = min_separation_m
        self.min_information_gain = min_information_gain
        self.search_origin: GeoPoint | None = None
        self.unavailable: list[GeoPoint] = []

    @staticmethod
    def _information(bearings: list[float], confidences: list[float]) -> float:
        matrix = np.eye(2) * 1e-6
        for bearing, confidence in zip(bearings, confidences):
            radians = math.radians(bearing)
            normal = np.array([math.cos(radians), -math.sin(radians)])
            matrix += max(.05, min(1.0, confidence)) ** 2 * np.outer(normal, normal)
        _, logdet = np.linalg.slogdet(matrix)
        return float(logdet)

    @staticmethod
    def _crossing_angle(first: float, second: float) -> float:
        difference = abs((first - second) % 180)
        return min(difference, 180 - difference)

    def mark_unavailable(self, point: GeoPoint) -> None:
        self.unavailable.append(point)

    def plan(self, current: GeoPoint, observations: list[Observation], estimate: GeoPoint | None = None) -> PlannedWaypoint:
        if not observations:
            raise ValueError("an initial observation is required")
        self.search_origin = self.search_origin or GeoPoint(observations[0].latitude, observations[0].longitude)
        step = min(self.spacing_m, self.max_movement_distance_m)
        side = 1 if len(observations) % 2 else -1
        legacy_heading = (observations[-1].absolute_bearing_deg
                          + side * self.desired_cross_bearing_deg) % 360
        headings = [legacy_heading, (legacy_heading + 180) % 360]
        headings.extend(float(value) for value in range(0, 360, 30))
        # Preserve order while removing duplicate headings.
        headings = list(dict.fromkeys(round(value, 6) for value in headings))
        distances = [step, max(self.min_separation_m, step * .6)]
        target_hint = estimate or destination(
            GeoPoint(observations[0].latitude, observations[0].longitude),
            observations[0].absolute_bearing_deg, max(100, 2 * self.spacing_m))
        existing_bearings = [item.absolute_bearing_deg for item in observations]
        existing_confidences = [item.confidence for item in observations]
        current_information = self._information(existing_bearings, existing_confidences)
        candidate_specs = [(travel_bearing, destination(current, travel_bearing, candidate_distance),
                            candidate_distance)
                           for travel_bearing in headings for candidate_distance in distances]
        if estimate is not None:
            orbit_radius = min(self.spacing_m, self.max_search_radius_m * .8)
            for angle in range(0, 360, 30):
                orbit = destination(estimate, angle, orbit_radius)
                candidate_specs.append((initial_bearing_deg(current, orbit), orbit,
                                        distance_m(current, orbit)))
        ranked: list[tuple[float, float, float, GeoPoint, float]] = []
        for travel_bearing, candidate, candidate_distance in candidate_specs:
                if distance_m(current, candidate) > self.max_movement_distance_m + .01:
                    continue
                if distance_m(self.search_origin, candidate) > self.max_search_radius_m + .01:
                    continue
                if any(distance_m(candidate, GeoPoint(item.latitude, item.longitude))
                       < self.min_separation_m for item in observations):
                    continue
                if any(distance_m(candidate, unavailable) < self.min_separation_m
                       for unavailable in self.unavailable):
                    continue
                predicted_bearing = initial_bearing_deg(candidate, target_hint)
                new_information = self._information(
                    [*existing_bearings, predicted_bearing], [*existing_confidences, .9])
                information_gain = new_information - current_information
                best_crossing = max(self._crossing_angle(predicted_bearing, bearing)
                                    for bearing in existing_bearings)
                desired_score = 1 - abs(best_crossing - self.desired_cross_bearing_deg) / 90
                travel_cost = candidate_distance / max(step, 1)
                target_range_cost = distance_m(candidate, target_hint) / max(self.spacing_m, 1)
                score = (information_gain + .25 * desired_score
                         - .10 * travel_cost - .20 * target_range_cost)
                ranked.append((score, information_gain, travel_bearing,
                               candidate, best_crossing))
        if not ranked:
            raise ValueError("no safe waypoint within maximum search radius")
        score, gain, travel_bearing, candidate, crossing = max(ranked, key=lambda item: item[0])
        if gain < self.min_information_gain:
            raise ValueError("no candidate provides useful geometry improvement")
        return PlannedWaypoint(candidate,
            f"information-gain move at {travel_bearing % 360:.1f} deg; "
            f"predicted crossing {crossing:.1f} deg; gain {gain:.3f}")


class FixedGridPlanner(ObservationPlanner):
    """Simple compass-grid baseline for planner comparisons."""

    def plan(self, current: GeoPoint, observations: list[Observation], estimate: GeoPoint | None = None) -> PlannedWaypoint:
        if not observations:
            raise ValueError("an initial observation is required")
        self.search_origin = self.search_origin or GeoPoint(observations[0].latitude, observations[0].longitude)
        headings = (0, 90, 180, 270, 45, 135, 225, 315)
        for offset in range(len(headings)):
            heading = headings[(len(observations) - 1 + offset) % len(headings)]
            candidate = destination(self.search_origin, heading,
                                    min(self.spacing_m, self.max_search_radius_m))
            if distance_m(current, candidate) <= self.max_movement_distance_m + .01:
                if all(distance_m(candidate, GeoPoint(item.latitude, item.longitude)) >= self.min_separation_m
                       for item in observations):
                    return PlannedWaypoint(candidate, f"fixed-grid baseline at {heading:.1f} deg")
        raise ValueError("fixed-grid baseline has no valid candidate")


class RandomPlanner(ObservationPlanner):
    """Seeded random valid-point baseline for planner comparisons."""

    def __init__(self, *args, seed: int = 7, **kwargs):
        super().__init__(*args, **kwargs)
        self.random = random.Random(seed)

    def plan(self, current: GeoPoint, observations: list[Observation], estimate: GeoPoint | None = None) -> PlannedWaypoint:
        if not observations:
            raise ValueError("an initial observation is required")
        self.search_origin = self.search_origin or GeoPoint(observations[0].latitude, observations[0].longitude)
        for _ in range(100):
            heading = self.random.uniform(0, 360)
            travel = self.random.uniform(max(1, self.min_separation_m),
                                         min(self.spacing_m, self.max_movement_distance_m))
            candidate = destination(current, heading, travel)
            if distance_m(self.search_origin, candidate) > self.max_search_radius_m:
                continue
            if any(distance_m(candidate, GeoPoint(item.latitude, item.longitude)) < self.min_separation_m
                   for item in observations):
                continue
            return PlannedWaypoint(candidate, f"seeded-random baseline at {heading:.1f} deg")
        raise ValueError("seeded-random baseline has no valid candidate")
