"""Vehicle telemetry model."""
from dataclasses import dataclass
import math
import time


@dataclass
class VehicleState:
    latitude: float
    longitude: float
    altitude_m: float
    heading_deg: float
    roll_deg: float
    pitch_deg: float
    flight_mode: str
    gps_fix_type: int = 3
    satellites_visible: int = 10
    armed: bool = False
    timestamp: float = 0.0
    position_accuracy_m: float = 0.0
    heading_accuracy_deg: float = 0.0
    ground_speed_m_s: float = 0.0

    def navigation_valid(self, max_age_seconds: float = 5.0) -> bool:
        return (
            self.gps_fix_type >= 3
            and all(math.isfinite(value) for value in (
                self.latitude, self.longitude, self.altitude_m, self.heading_deg,
                self.roll_deg, self.pitch_deg, self.position_accuracy_m,
                self.heading_accuracy_deg, self.ground_speed_m_s, self.timestamp))
            and -90 <= self.latitude <= 90
            and -180 <= self.longitude <= 180
            and self.timestamp > 0
            and time.time() - self.timestamp <= max_age_seconds
        )
