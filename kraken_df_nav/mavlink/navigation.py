"""Navigation validation shared by controllers."""
from ..df.geometry import GeoPoint, distance_m


def validate_waypoint(current: GeoPoint, target: GeoPoint, max_movement_m: float) -> None:
    if not (-90 <= target.latitude <= 90 and -180 <= target.longitude <= 180):
        raise ValueError("invalid target coordinates")
    if distance_m(current, target) > max_movement_m + 0.01:
        raise ValueError("target exceeds maximum movement distance")
