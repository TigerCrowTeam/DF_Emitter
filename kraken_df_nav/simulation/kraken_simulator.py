"""Synthetic vehicle and Kraken source for complete offline missions."""
from __future__ import annotations

import math
import random
import time
from dataclasses import asdict
from typing import Callable

from ..config import DigitalTwinConfig, VehicleTwinConfig
from ..df.geometry import GeoPoint, distance_m, initial_bearing_deg
from ..kraken.client import BearingSource
from ..kraken.models import BearingMeasurement
from ..mavlink.connection import VehicleController
from ..mavlink.telemetry import VehicleState
from .transmitter import SimulatedTransmitter


class SimulatedVehicleController(VehicleController):
    def __init__(self, start: GeoPoint, heading_deg: float = 0,
                 effects: VehicleTwinConfig | None = None, seed: int = 7,
                 clock: Callable[[], float] = time.time, arrival_tolerance_m: float = 5.0):
        self.point = start
        self.origin = start
        self.heading = heading_deg
        self.target: GeoPoint | None = None
        self.loitered = False
        self.effects = effects or VehicleTwinConfig()
        self.random, self.clock = random.Random(seed), clock
        self.arrival_tolerance_m = arrival_tolerance_m
        self.state_samples = 0
        self.arrival_delay_remaining = 0
        self.total_distance_m = 0.0
        self.command_count = 0
        self.records: list[dict[str, object]] = []
        self._drift_north_m = self._drift_east_m = 0.0

    def connect(self) -> None: pass
    @staticmethod
    def _offset(point: GeoPoint, north_m: float, east_m: float) -> GeoPoint:
        lat = point.latitude + north_m / 111_320.0
        lon_scale = max(1e-9, 111_320.0 * math.cos(math.radians(point.latitude)))
        return GeoPoint(lat, point.longitude + east_m / lon_scale, point.altitude_m)

    def _moving(self) -> bool:
        return self.target is not None and distance_m(self.point, self.target) > self.arrival_tolerance_m

    def get_vehicle_state(self) -> VehicleState | None:
        self.state_samples += 1
        effects = self.effects
        if effects.telemetry_loss_after_samples and self.state_samples >= effects.telemetry_loss_after_samples:
            self.records.append({"type": "telemetry_loss", "sample": self.state_samples})
            return None
        if effects.telemetry_packet_loss_probability and self.random.random() < effects.telemetry_packet_loss_probability:
            self.records.append({"type": "telemetry_packet_loss", "sample": self.state_samples})
            return None
        self._drift_north_m += effects.gps_drift_m_per_sample
        noise_north = self.random.gauss(0, effects.gps_noise_std_m)
        noise_east = self.random.gauss(0, effects.gps_noise_std_m)
        reported = self._offset(self.point, self._drift_north_m + noise_north,
                                self._drift_east_m + noise_east)
        heading = (self.heading + effects.heading_bias_deg
                   + self.random.gauss(0, effects.heading_noise_std_deg)) % 360
        moving = self._moving()
        mode = "LOITER" if self.loitered else "GUIDED"
        if effects.unexpected_mode_after_samples and self.state_samples >= effects.unexpected_mode_after_samples:
            mode = effects.unexpected_mode
        gps_fix = 3
        if effects.gps_loss_after_samples and self.state_samples >= effects.gps_loss_after_samples:
            gps_fix = 1
        timestamp = self.clock() - effects.telemetry_latency_seconds
        if effects.heartbeat_loss_after_samples and self.state_samples >= effects.heartbeat_loss_after_samples:
            timestamp -= 3600
        state = VehicleState(reported.latitude, reported.longitude, reported.altitude_m,
            heading, effects.moving_roll_deg if moving else effects.stationary_roll_deg,
            effects.moving_pitch_deg if moving else effects.stationary_pitch_deg,
            mode, gps_fix_type=gps_fix,
            timestamp=timestamp,
            position_accuracy_m=abs(self._drift_north_m) + 2 * effects.gps_noise_std_m,
            heading_accuracy_deg=abs(effects.heading_bias_deg) + 2 * effects.heading_noise_std_deg,
            ground_speed_m_s=5.0 if moving else 0.0)
        self.records.append({"type": "telemetry", "sample": self.state_samples,
            "true_latitude": self.point.latitude, "true_longitude": self.point.longitude,
            "reported_latitude": reported.latitude, "reported_longitude": reported.longitude,
            "heading_deg": heading, "moving": moving, "flight_mode": mode,
            "gps_fix_type": gps_fix, "timestamp": timestamp})
        return state
    def goto_location(self, lat: float, lon: float, alt: float) -> None:
        if not self.navigation_valid(): raise RuntimeError("invalid simulated navigation")
        requested = GeoPoint(lat, lon, alt)
        if self.effects.geofence_radius_m and distance_m(self.origin, requested) > self.effects.geofence_radius_m:
            raise RuntimeError("simulated geofence refused waypoint")
        if self.effects.waypoint_failure_probability and self.random.random() < self.effects.waypoint_failure_probability:
            raise RuntimeError("simulated waypoint acceptance failure")
        self.target = requested
        if self.effects.position_tracking_error_m:
            self.target = self._offset(requested, self.effects.position_tracking_error_m, 0)
        self.arrival_delay_remaining = self.effects.arrival_delay_updates
        self.command_count += 1
        self.records.append({"type": "waypoint", "latitude": lat, "longitude": lon, "altitude_m": alt})
        if self.effects.travel_fraction_per_update >= 1 and self.arrival_delay_remaining == 0:
            self.total_distance_m += distance_m(self.point, self.target)
            self.point = self.target
    def distance_to_target(self) -> float | None:
        if self.target is None:
            return None
        if self.arrival_delay_remaining > 0:
            self.arrival_delay_remaining -= 1
            return distance_m(self.point, self.target)
        remaining = distance_m(self.point, self.target)
        if remaining > 0.05 and self.effects.travel_fraction_per_update < 1:
            fraction = self.effects.travel_fraction_per_update
            next_point = GeoPoint(
                self.point.latitude + (self.target.latitude - self.point.latitude) * fraction,
                self.point.longitude + (self.target.longitude - self.point.longitude) * fraction,
                self.point.altitude_m + (self.target.altitude_m - self.point.altitude_m) * fraction)
            self.total_distance_m += distance_m(self.point, next_point)
            self.point = next_point
            remaining = distance_m(self.point, self.target)
        return remaining
    def command_loiter(self) -> None: self.loitered = True
    def navigation_valid(self) -> bool:
        if self.effects.telemetry_loss_after_samples and self.state_samples >= self.effects.telemetry_loss_after_samples:
            return False
        if self.effects.heartbeat_loss_after_samples and self.state_samples >= self.effects.heartbeat_loss_after_samples:
            return False
        if self.effects.gps_loss_after_samples and self.state_samples >= self.effects.gps_loss_after_samples:
            return False
        if self.effects.unexpected_mode_after_samples and self.state_samples >= self.effects.unexpected_mode_after_samples:
            return False
        return True


class SimulatedBearingSource(BearingSource):
    def __init__(self, transmitter: SimulatedTransmitter, vehicle: VehicleController,
                 frequency_hz: int = 915_000_000, noise_std_deg: float = 2.0,
                 mounting_offset_deg: float = 0, seed: int = 7,
                 effects: DigitalTwinConfig | None = None,
                 record_callback: Callable[[dict[str, object]], None] | None = None,
                 clock: Callable[[], float] = time.time):
        self.transmitter, self.vehicle, self.frequency_hz = transmitter, vehicle, frequency_hz
        self.noise_std_deg, self.mounting_offset_deg = noise_std_deg, mounting_offset_deg
        self.random = random.Random(seed)
        self.effects = effects or DigitalTwinConfig()
        self.record_callback, self.clock = record_callback, clock
        self.records: list[dict[str, object]] = []
        self.sample_index = 0
        self.correlated_error = 0.0
        self.dropout_remaining = self.outlier_remaining = self.multipath_remaining = 0
        self.last_measurement: BearingMeasurement | None = None
        self.last_vehicle: GeoPoint | None = None

    def _record(self, **values: object) -> None:
        record = {"sample_index": self.sample_index, **values}
        self.records.append(record)
        if self.record_callback:
            self.record_callback(record)

    def _chance(self, probability: float) -> bool:
        return probability > 0 and self.random.random() < probability

    def get_measurement(self) -> BearingMeasurement | None:
        self.sample_index += 1
        effects = self.effects
        state = self.vehicle.get_vehicle_state()
        if state is None:
            self._record(outcome="dropout", reason="vehicle state unavailable")
            return None
        tx = GeoPoint(self.transmitter.latitude, self.transmitter.longitude)
        vehicle = GeoPoint(state.latitude, state.longitude)
        clean_absolute = initial_bearing_deg(vehicle, tx)
        distance = max(1.0, distance_m(vehicle, tx))
        moving = state.ground_speed_m_s > 0.5
        self.last_vehicle = vehicle

        if self.dropout_remaining > 0:
            self.dropout_remaining -= 1
            self._record(outcome="dropout", reason="burst dropout", clean_absolute_bearing_deg=clean_absolute)
            return None
        if self._chance(effects.burst_dropout_probability):
            self.dropout_remaining = max(0, effects.burst_dropout_samples - 1)
            self._record(outcome="dropout", reason="burst dropout started", clean_absolute_bearing_deg=clean_absolute)
            return None
        if self._chance(effects.dropout_probability):
            self._record(outcome="dropout", reason="intermittent dropout", clean_absolute_bearing_deg=clean_absolute)
            return None
        if self._chance(effects.malformed_probability):
            self._record(outcome="malformed", reason="simulated malformed payload", clean_absolute_bearing_deg=clean_absolute)
            raise ValueError("simulated malformed Kraken payload")
        if self.last_measurement and self._chance(effects.duplicate_probability):
            self._record(outcome="duplicate", clean_absolute_bearing_deg=clean_absolute,
                         measurement=asdict(self.last_measurement))
            return self.last_measurement

        components: dict[str, float] = {}
        components["gaussian_noise_deg"] = self.random.gauss(0, self.noise_std_deg)
        components["constant_bias_deg"] = effects.constant_bias_deg
        components["drift_deg"] = effects.drift_deg_per_sample * (self.sample_index - 1)
        components["heading_error_deg"] = effects.heading_error_deg
        components["range_noise_deg"] = self.random.gauss(0, effects.range_noise_deg_per_km * distance / 1000)
        innovation = self.random.gauss(0, effects.correlated_noise_std_deg)
        self.correlated_error = (effects.correlated_noise_coefficient * self.correlated_error
                                 + math.sqrt(1 - effects.correlated_noise_coefficient ** 2) * innovation)
        components["correlated_noise_deg"] = self.correlated_error

        if self.outlier_remaining > 0:
            self.outlier_remaining -= 1
            components["outlier_deg"] = effects.outlier_magnitude_deg
        elif self._chance(effects.outlier_burst_probability):
            self.outlier_remaining = max(0, effects.outlier_burst_samples - 1)
            components["outlier_deg"] = effects.outlier_magnitude_deg
        elif self._chance(effects.outlier_probability):
            components["outlier_deg"] = effects.outlier_magnitude_deg

        if self.multipath_remaining > 0:
            self.multipath_remaining -= 1
            components["multipath_deg"] = effects.multipath_bias_deg
        elif self._chance(effects.multipath_probability):
            self.multipath_remaining = effects.multipath_persistence_samples - 1
            components["multipath_deg"] = effects.multipath_bias_deg

        if moving:
            components["moving_penalty_deg"] = effects.moving_bearing_penalty_deg
        attitude_fraction = min(1.0, math.hypot(state.roll_deg, state.pitch_deg) / 45.0)
        components["attitude_penalty_deg"] = effects.attitude_bearing_penalty_deg * attitude_fraction
        total_error = sum(components.values())
        absolute = clean_absolute + total_error
        relative = (absolute - state.heading_deg - self.mounting_offset_deg) % 360
        rssi = self.transmitter.power_dbm - 20 * math.log10(distance) - 40 + self.random.gauss(0, 1)
        confidence = 0.98 - abs(total_error) / max(1.0, effects.confidence_error_scale_deg)
        confidence -= distance / max(1.0, effects.confidence_range_scale_m) * 0.1
        if moving:
            confidence -= effects.moving_confidence_penalty
        confidence -= effects.attitude_confidence_penalty * attitude_fraction
        confidence = max(0.0, min(0.99, confidence))
        frequency = self.frequency_hz
        if self._chance(effects.wrong_frequency_probability):
            frequency += effects.wrong_frequency_offset_hz
        timestamp = self.clock() - effects.delay_seconds
        if self._chance(effects.stale_probability):
            timestamp -= effects.stale_age_seconds
        measurement = BearingMeasurement(relative, rssi, confidence, frequency, timestamp)
        if self.last_measurement and self._chance(effects.frozen_probability):
            measurement = BearingMeasurement(self.last_measurement.relative_bearing_deg, rssi,
                                             confidence, frequency, timestamp)
        self.last_measurement = measurement
        self._record(outcome="measurement", clean_absolute_bearing_deg=clean_absolute,
                     transmitter_latitude=self.transmitter.latitude,
                     transmitter_longitude=self.transmitter.longitude,
                     vehicle_latitude=state.latitude, vehicle_longitude=state.longitude,
                     distance_m=distance, moving=moving, effects=components,
                     total_bearing_error_deg=total_error, measurement=asdict(measurement))
        return measurement
