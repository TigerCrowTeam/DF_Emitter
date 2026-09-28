"""Configuration loading and validation."""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml


@dataclass
class RFConfig:
    center_frequency_hz: int = 915_000_000
    bandwidth_hz: int = 2_400_000


@dataclass
class KrakenConfig:
    endpoint: str = ""
    min_confidence: float = 0.70
    antenna_heading_offset_deg: float = 0.0
    timeout_seconds: float = 2.0
    max_measurement_age_seconds: float = 2.0
    max_pose_measurement_skew_seconds: float = 1.0
    max_frozen_samples: int = 20
    max_consecutive_rejections: int = 20


@dataclass
class MavlinkConfig:
    connection: str = "udp:127.0.0.1:14550"
    heartbeat_timeout_seconds: float = 10.0
    telemetry_timeout_seconds: float = 5.0
    source_system: int = 252
    source_component: int = 191
    expected_target_system: int | None = None
    expected_target_component: int | None = None


@dataclass
class MAVProxyConfig:
    enabled: bool = False
    master_connection: str = ""
    loopback_host: str = "127.0.0.1"
    poc_output_port: int = 14560
    preparation_output_port: int = 14561
    operator_output_port: int = 14550
    source_system: int = 255
    source_component: int = 190
    map_enabled: bool = True
    console_enabled: bool = True


@dataclass
class DFConfig:
    target_error_radius_m: float = 10.0
    max_roll_deg: float = 5.0
    max_pitch_deg: float = 5.0
    minimum_observations: int = 2
    outlier_rejection: bool = True
    outlier_threshold_m: float = 20.0
    max_position_accuracy_m: float = 10.0
    max_heading_accuracy_deg: float = 10.0
    max_collection_speed_m_s: float = 0.5
    max_leave_one_out_shift_m: float = 30.0


@dataclass
class NavigationConfig:
    desired_cross_bearing_deg: float = 60.0
    observation_spacing_m: float = 100.0
    max_search_radius_m: float = 500.0
    max_movement_distance_m: float = 200.0
    arrival_tolerance_m: float = 5.0
    stabilization_seconds: float = 3.0
    command_altitude_m: float | None = None
    movement_timeout_seconds: float = 120.0
    max_mission_duration_seconds: float = 600.0
    max_total_distance_m: float = 800.0
    accepted_flight_modes: list[str] = field(default_factory=lambda: ["GUIDED", "LOITER"])
    planner_min_separation_m: float = 20.0
    planner_min_information_gain: float = 0.01
    planner_max_attempts: int = 3


@dataclass
class LoggingConfig:
    directory: str = "./logs"
    save_csv: bool = True
    save_json: bool = True


@dataclass
class DigitalTwinConfig:
    constant_bias_deg: float = 0.0
    drift_deg_per_sample: float = 0.0
    heading_error_deg: float = 0.0
    range_noise_deg_per_km: float = 0.0
    correlated_noise_std_deg: float = 0.0
    correlated_noise_coefficient: float = 0.0
    dropout_probability: float = 0.0
    burst_dropout_probability: float = 0.0
    burst_dropout_samples: int = 0
    delay_seconds: float = 0.0
    stale_probability: float = 0.0
    stale_age_seconds: float = 10.0
    duplicate_probability: float = 0.0
    frozen_probability: float = 0.0
    malformed_probability: float = 0.0
    wrong_frequency_probability: float = 0.0
    wrong_frequency_offset_hz: int = 1_000_000
    outlier_probability: float = 0.0
    outlier_magnitude_deg: float = 90.0
    outlier_burst_probability: float = 0.0
    outlier_burst_samples: int = 0
    multipath_probability: float = 0.0
    multipath_bias_deg: float = 35.0
    multipath_persistence_samples: int = 1
    confidence_error_scale_deg: float = 20.0
    confidence_range_scale_m: float = 1000.0
    moving_bearing_penalty_deg: float = 0.0
    moving_confidence_penalty: float = 0.0
    attitude_bearing_penalty_deg: float = 0.0
    attitude_confidence_penalty: float = 0.0


@dataclass
class VehicleTwinConfig:
    gps_noise_std_m: float = 0.0
    gps_drift_m_per_sample: float = 0.0
    heading_bias_deg: float = 0.0
    heading_noise_std_deg: float = 0.0
    moving_roll_deg: float = 0.0
    moving_pitch_deg: float = 0.0
    stationary_roll_deg: float = 0.0
    stationary_pitch_deg: float = 0.0
    travel_fraction_per_update: float = 1.0
    arrival_delay_updates: int = 0
    position_tracking_error_m: float = 0.0
    waypoint_failure_probability: float = 0.0
    telemetry_packet_loss_probability: float = 0.0
    telemetry_latency_seconds: float = 0.0
    telemetry_loss_after_samples: int = 0
    heartbeat_loss_after_samples: int = 0
    unexpected_mode_after_samples: int = 0
    unexpected_mode: str = "ALT_HOLD"
    gps_loss_after_samples: int = 0
    geofence_radius_m: float = 0.0


@dataclass
class SimulationConfig:
    transmitter_lat: float = 37.0008
    transmitter_lon: float = -121.9992
    start_lat: float = 37.0
    start_lon: float = -122.0
    altitude_m: float = 60.0
    bearing_noise_std_deg: float = 2.0
    random_seed: int = 7
    use_sitl: bool = False
    required_observations: int = 7
    digital_twin: DigitalTwinConfig = field(default_factory=DigitalTwinConfig)
    vehicle_twin: VehicleTwinConfig = field(default_factory=VehicleTwinConfig)


@dataclass
class ReplayConfig:
    path: str = ""


@dataclass
class SafetyConfig:
    invalid_data_action: str = "ABORT"  # ABORT or PAUSE


@dataclass
class AppConfig:
    mode: str = "SIMULATION"
    rf: RFConfig = field(default_factory=RFConfig)
    kraken: KrakenConfig = field(default_factory=KrakenConfig)
    mavlink: MavlinkConfig = field(default_factory=MavlinkConfig)
    mavproxy: MAVProxyConfig = field(default_factory=MAVProxyConfig)
    df: DFConfig = field(default_factory=DFConfig)
    navigation: NavigationConfig = field(default_factory=NavigationConfig)
    logging: LoggingConfig = field(default_factory=LoggingConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)
    replay: ReplayConfig = field(default_factory=ReplayConfig)
    safety: SafetyConfig = field(default_factory=SafetyConfig)

    def validate(self) -> None:
        self.mode = self.mode.upper()
        if self.mode not in {"SIMULATION", "KRAKEN_SITL", "LIVE", "REPLAY"}:
            raise ValueError(f"unsupported mode: {self.mode}")
        if not 0 <= self.kraken.min_confidence <= 1:
            raise ValueError("kraken.min_confidence must be in [0, 1]")
        if (self.kraken.max_measurement_age_seconds <= 0
                or self.kraken.max_pose_measurement_skew_seconds <= 0
                or self.kraken.max_frozen_samples < 2
                or self.kraken.max_consecutive_rejections < 1):
            raise ValueError("Kraken age/skew limits must be positive and frozen limit at least 2")
        if not 1 <= self.mavlink.source_system <= 255:
            raise ValueError("mavlink.source_system must be in [1, 255]")
        if not 1 <= self.mavlink.source_component <= 255:
            raise ValueError("mavlink.source_component must be in [1, 255]")
        for name in ("expected_target_system", "expected_target_component"):
            value = getattr(self.mavlink, name)
            if value is not None and not 1 <= value <= 255:
                raise ValueError(f"mavlink.{name} must be null or in [1, 255]")
        if self.mavproxy.enabled:
            proxy = self.mavproxy
            if proxy.loopback_host != "127.0.0.1":
                raise ValueError("mavproxy.loopback_host must be 127.0.0.1 during initial integration")
            expected_connection = f"udpin:{proxy.loopback_host}:{proxy.poc_output_port}"
            if self.mavlink.connection != expected_connection:
                raise ValueError(
                    f"MAVProxy mode requires mavlink.connection {expected_connection!r}; "
                    "the application must not open the autopilot master directly")
            ports = (proxy.poc_output_port, proxy.preparation_output_port,
                     proxy.operator_output_port)
            if any(not 1024 <= port <= 65535 for port in ports):
                raise ValueError("MAVProxy UDP ports must be in [1024, 65535]")
            if len(set(ports)) != len(ports):
                raise ValueError("MAVProxy UDP ports must be unique")
            if not proxy.master_connection:
                raise ValueError("mavproxy.master_connection is required when MAVProxy is enabled")
            if (self.mode == "SIMULATION" and self.simulation.use_sitl
                    and not proxy.master_connection.startswith(
                        ("tcp:127.0.0.1:", "udp:127.0.0.1:", "udpin:127.0.0.1:"))):
                raise ValueError("MAVProxy SITL master must be a loopback network endpoint")
            for name in ("source_system", "source_component"):
                if not 1 <= getattr(proxy, name) <= 255:
                    raise ValueError(f"mavproxy.{name} must be in [1, 255]")
        if self.navigation.observation_spacing_m <= 0:
            raise ValueError("observation spacing must be positive")
        if self.navigation.max_movement_distance_m <= 0:
            raise ValueError("maximum movement distance must be positive")
        if (self.navigation.movement_timeout_seconds <= 0
                or self.navigation.max_mission_duration_seconds <= 0
                or self.navigation.max_total_distance_m <= 0):
            raise ValueError("navigation time and total-distance limits must be positive")
        if self.navigation.planner_min_separation_m < 0 or self.navigation.planner_min_information_gain < 0:
            raise ValueError("planner separation and information gain must be non-negative")
        if self.navigation.planner_max_attempts < 1:
            raise ValueError("planner_max_attempts must be positive")
        if not self.navigation.accepted_flight_modes:
            raise ValueError("at least one accepted flight mode is required")
        if self.simulation.required_observations < 2:
            raise ValueError("simulation.required_observations must be at least 2")
        if (self.df.max_position_accuracy_m <= 0 or self.df.max_heading_accuracy_deg <= 0
                or self.df.max_collection_speed_m_s < 0 or self.df.max_leave_one_out_shift_m <= 0):
            raise ValueError("DF pose-accuracy limits must be positive and speed non-negative")
        twin = self.simulation.digital_twin
        probabilities = [value for name, value in vars(twin).items() if name.endswith("_probability")]
        if any(not 0 <= value <= 1 for value in probabilities):
            raise ValueError("digital-twin probabilities must be in [0, 1]")
        if not 0 <= twin.correlated_noise_coefficient < 1:
            raise ValueError("correlated_noise_coefficient must be in [0, 1)")
        if twin.burst_dropout_samples < 0 or twin.outlier_burst_samples < 0 or twin.multipath_persistence_samples < 1:
            raise ValueError("digital-twin burst lengths must be non-negative and persistence positive")
        vehicle = self.simulation.vehicle_twin
        if not 0 < vehicle.travel_fraction_per_update <= 1:
            raise ValueError("travel_fraction_per_update must be in (0, 1]")
        if any(not 0 <= value <= 1 for name, value in vars(vehicle).items() if name.endswith("_probability")):
            raise ValueError("vehicle-twin probabilities must be in [0, 1]")
        if any(getattr(vehicle, name) < 0 for name in (
                "arrival_delay_updates", "telemetry_loss_after_samples",
                "heartbeat_loss_after_samples", "unexpected_mode_after_samples",
                "gps_loss_after_samples", "geofence_radius_m")):
            raise ValueError("vehicle-twin counts and geofence radius must be non-negative")
        if self.safety.invalid_data_action.upper() not in {"ABORT", "PAUSE"}:
            raise ValueError("safety.invalid_data_action must be ABORT or PAUSE")


def _merge_dataclass(instance: Any, values: dict[str, Any]) -> None:
    for key, value in values.items():
        if not hasattr(instance, key):
            raise ValueError(f"unknown configuration key: {key}")
        current = getattr(instance, key)
        if hasattr(current, "__dataclass_fields__"):
            if not isinstance(value, dict):
                raise ValueError(f"configuration section {key} must be a mapping")
            _merge_dataclass(current, value)
        else:
            setattr(instance, key, value)


def load_config(path: str | Path) -> AppConfig:
    raw = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    config = AppConfig()
    for key, value in raw.items():
        current = getattr(config, key, None)
        if current is not None and hasattr(current, "__dataclass_fields__"):
            _merge_dataclass(current, value or {})
        elif hasattr(config, key):
            setattr(config, key, value)
        else:
            raise ValueError(f"unknown configuration section: {key}")
    config.validate()
    return config
