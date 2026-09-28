"""Bearing transformations."""


def wrap_heading(degrees: float) -> float:
    return degrees % 360.0


def relative_to_absolute(relative_deg: float, vehicle_heading_deg: float, mounting_offset_deg: float = 0.0) -> float:
    return wrap_heading(relative_deg + vehicle_heading_deg + mounting_offset_deg)


def angular_difference_deg(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)
