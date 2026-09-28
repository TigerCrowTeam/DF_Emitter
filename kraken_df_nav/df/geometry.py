"""Small-area WGS84/local ENU geographic utilities."""
from __future__ import annotations

import math
from dataclasses import dataclass

EARTH_RADIUS_M = 6_378_137.0


@dataclass(frozen=True)
class GeoPoint:
    latitude: float
    longitude: float
    altitude_m: float = 0.0


class LocalFrame:
    """Azimuthal-equidistant-like ENU approximation for sub-kilometre work."""

    def __init__(self, origin: GeoPoint):
        self.origin = origin
        self._lat0 = math.radians(origin.latitude)
        self._lon0 = math.radians(origin.longitude)

    def to_enu(self, point: GeoPoint) -> tuple[float, float, float]:
        lat = math.radians(point.latitude)
        lon = math.radians(point.longitude)
        east = (lon - self._lon0) * EARTH_RADIUS_M * math.cos((lat + self._lat0) / 2)
        north = (lat - self._lat0) * EARTH_RADIUS_M
        return east, north, point.altitude_m - self.origin.altitude_m

    def from_enu(self, east: float, north: float, up: float = 0.0) -> GeoPoint:
        lat = self._lat0 + north / EARTH_RADIUS_M
        lon = self._lon0 + east / (EARTH_RADIUS_M * math.cos((lat + self._lat0) / 2))
        return GeoPoint(math.degrees(lat), math.degrees(lon), self.origin.altitude_m + up)


def distance_m(a: GeoPoint, b: GeoPoint) -> float:
    frame = LocalFrame(a)
    east, north, _ = frame.to_enu(b)
    return math.hypot(east, north)


def initial_bearing_deg(a: GeoPoint, b: GeoPoint) -> float:
    lat1, lat2 = math.radians(a.latitude), math.radians(b.latitude)
    dlon = math.radians(b.longitude - a.longitude)
    y = math.sin(dlon) * math.cos(lat2)
    x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dlon)
    return math.degrees(math.atan2(y, x)) % 360.0


def destination(point: GeoPoint, bearing_deg: float, distance: float) -> GeoPoint:
    angular = distance / EARTH_RADIUS_M
    bearing = math.radians(bearing_deg)
    lat1, lon1 = math.radians(point.latitude), math.radians(point.longitude)
    lat2 = math.asin(math.sin(lat1) * math.cos(angular) + math.cos(lat1) * math.sin(angular) * math.cos(bearing))
    lon2 = lon1 + math.atan2(math.sin(bearing) * math.sin(angular) * math.cos(lat1), math.cos(angular) - math.sin(lat1) * math.sin(lat2))
    return GeoPoint(math.degrees(lat2), ((math.degrees(lon2) + 540) % 360) - 180, point.altitude_m)
