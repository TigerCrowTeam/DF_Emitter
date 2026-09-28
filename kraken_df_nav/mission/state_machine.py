"""Explicit autonomous DF mission state machine."""
from __future__ import annotations

from enum import Enum, auto
from dataclasses import asdict
import math
import time

from ..config import AppConfig
from ..df.estimator import EstimateResult, WeightedLeastSquaresEstimator
from ..df.geometry import GeoPoint, distance_m
from ..df.observation import Observation
from ..df.transforms import relative_to_absolute
from ..kraken.client import BearingSource
from ..kraken.filtering import MeasurementGate
from ..logging_utils.recorder import MissionRecorder
from ..mavlink.connection import VehicleController
from .observation_planner import ObservationPlanner


class MissionState(Enum):
    INITIALIZING = auto()
    WAITING_FOR_SIGNAL = auto()
    COLLECTING_OBSERVATION = auto()
    PLANNING_MOVE = auto()
    MOVING = auto()
    STABILIZING = auto()
    ESTIMATING = auto()
    PAUSED = auto()
    COMPLETE = auto()
    ABORTED = auto()


class Mission:
    def __init__(self, config: AppConfig, source: BearingSource, vehicle: VehicleController,
                 estimator: WeightedLeastSquaresEstimator, planner: ObservationPlanner,
                 recorder: MissionRecorder, clock=time.monotonic, wall_clock=time.time):
        self.config, self.source, self.vehicle = config, source, vehicle
        self.estimator, self.planner, self.recorder = estimator, planner, recorder
        self.state = MissionState.INITIALIZING
        self.observations: list[Observation] = []
        self.result: EstimateResult | None = None
        self._stable_since: float | None = None
        self._clock = clock
        self._wall_clock = wall_clock
        self._started_at = clock()
        self._move_started_at: float | None = None
        self._commanded_distance_m = 0.0
        self._measurement_gate = MeasurementGate(config.kraken.max_frozen_samples)
        self.ticks_executed = 0
        self._consecutive_measurement_rejections = 0

    def _measurement_rejected(self, reason: str, **details) -> None:
        self._consecutive_measurement_rejections += 1
        self.recorder.event("measurement_rejected", reason=reason,
                            consecutive_count=self._consecutive_measurement_rejections,
                            **details)
        if self._consecutive_measurement_rejections >= self.config.kraken.max_consecutive_rejections:
            self.fail_safe("maximum consecutive measurement rejections exceeded")
        else:
            self.transition(MissionState.WAITING_FOR_SIGNAL)

    def transition(self, state: MissionState) -> None:
        self.recorder.event("state", previous=self.state.name, current=state.name)
        self.state = state

    def fail_safe(self, reason: str) -> None:
        self.recorder.event("safety_stop", reason=reason)
        action = self.config.safety.invalid_data_action.upper()
        self.transition(MissionState.PAUSED if action == "PAUSE" else MissionState.ABORTED)

    def _stable(self, state) -> bool:
        attitude_ok = abs(state.roll_deg) <= self.config.df.max_roll_deg and abs(state.pitch_deg) <= self.config.df.max_pitch_deg
        if not attitude_ok:
            self._stable_since = None
            return False
        self._stable_since = self._stable_since or self._clock()
        return self._clock() - self._stable_since >= self.config.navigation.stabilization_seconds

    def _state_valid(self, state) -> bool:
        return (state is not None
                and state.navigation_valid(self.config.mavlink.telemetry_timeout_seconds)
                and state.flight_mode.upper() in {mode.upper() for mode in self.config.navigation.accepted_flight_modes})

    def _required_observations(self) -> int:
        """Simulation deliberately gathers several viewpoints for a richer demo."""
        if self.config.mode == "SIMULATION":
            return max(self.config.df.minimum_observations, self.config.simulation.required_observations)
        return self.config.df.minimum_observations

    def tick(self) -> None:
        # Terminal states are latched for the lifetime of a Mission instance.
        # Resuming requires constructing an explicit new run.
        if self.state in {MissionState.COMPLETE, MissionState.ABORTED, MissionState.PAUSED}:
            return
        if self._clock() - self._started_at > self.config.navigation.max_mission_duration_seconds:
            self.fail_safe("maximum mission duration exceeded")
            return
        if self.state == MissionState.INITIALIZING:
            self.vehicle.connect()
            self.transition(MissionState.WAITING_FOR_SIGNAL)
        elif self.state == MissionState.WAITING_FOR_SIGNAL:
            state = self.vehicle.get_vehicle_state()
            if not self._state_valid(state):
                self.fail_safe("invalid or stale vehicle navigation data")
            elif not self._stable(state):
                self.transition(MissionState.STABILIZING)
            else:
                self.transition(MissionState.COLLECTING_OBSERVATION)
        elif self.state == MissionState.STABILIZING:
            state = self.vehicle.get_vehicle_state()
            if not self._state_valid(state):
                self.fail_safe("navigation became invalid while stabilizing")
            elif self._stable(state):
                self.transition(MissionState.COLLECTING_OBSERVATION)
        elif self.state == MissionState.COLLECTING_OBSERVATION:
            state = self.vehicle.get_vehicle_state()
            if not self._state_valid(state) or not self.vehicle.navigation_valid():
                self.fail_safe("navigation invalid before DF collection"); return
            try:
                measurement = self.source.get_measurement()
            except Exception as exc:
                self._measurement_rejected(f"malformed or unavailable measurement: {exc}")
                return
            if measurement is None:
                self._measurement_rejected("missing measurement")
                return
            reason = self._measurement_gate.validate(
                measurement, target_hz=self.config.rf.center_frequency_hz,
                min_confidence=self.config.kraken.min_confidence,
                tolerance_hz=self.config.rf.bandwidth_hz // 2,
                now=self._wall_clock(), max_age_seconds=self.config.kraken.max_measurement_age_seconds,
                pose_timestamp=state.timestamp,
                max_pose_skew_seconds=self.config.kraken.max_pose_measurement_skew_seconds,
                position_accuracy_m=state.position_accuracy_m,
                max_position_accuracy_m=self.config.df.max_position_accuracy_m,
                heading_accuracy_deg=state.heading_accuracy_deg,
                max_heading_accuracy_deg=self.config.df.max_heading_accuracy_deg,
                ground_speed_m_s=state.ground_speed_m_s,
                max_collection_speed_m_s=self.config.df.max_collection_speed_m_s)
            if reason is None and (abs(state.roll_deg) > self.config.df.max_roll_deg
                                   or abs(state.pitch_deg) > self.config.df.max_pitch_deg):
                reason = "roll/pitch outside configured limits"
            if reason is None and self._stable_since is None:
                reason = "vehicle has not completed stabilization"
            absolute = (relative_to_absolute(measurement.relative_bearing_deg, state.heading_deg,
                                             self.config.kraken.antenna_heading_offset_deg)
                        if math.isfinite(measurement.relative_bearing_deg) and math.isfinite(state.heading_deg)
                        else math.nan)
            observation = Observation.now(latitude=state.latitude, longitude=state.longitude,
                altitude_m=state.altitude_m, vehicle_heading_deg=state.heading_deg,
                relative_bearing_deg=measurement.relative_bearing_deg, absolute_bearing_deg=absolute,
                rssi_dbm=measurement.rssi_dbm, confidence=measurement.confidence,
                accepted=reason is None, rejection_reason=reason or "",
                position_accuracy_m=state.position_accuracy_m,
                heading_accuracy_deg=state.heading_accuracy_deg)
            self.recorder.event("observation_context", vehicle_state=asdict(state),
                                measurement=asdict(measurement),
                                absolute_bearing_deg=absolute, accepted=reason is None,
                                rejection_reason=reason or "")
            self.recorder.observation(observation)
            if reason:
                self._measurement_rejected(reason,
                    measurement_timestamp=measurement.timestamp,
                    pose_timestamp=state.timestamp)
                return
            self._consecutive_measurement_rejections = 0
            self.observations.append(observation)
            self.transition(MissionState.ESTIMATING if len(self.observations) >= self.config.df.minimum_observations else MissionState.PLANNING_MOVE)
        elif self.state == MissionState.ESTIMATING:
            try:
                self.result = self.estimator.estimate(self.observations)
            except ValueError as exc:
                self.recorder.event("estimator_invalid", reason=str(exc))
                self.transition(MissionState.PLANNING_MOVE); return
            self.recorder.event("estimate", **self.result.__dict__)
            enough_viewpoints = len(self.observations) >= self._required_observations()
            if self.result.uncertainty_radius_m <= self.config.df.target_error_radius_m and enough_viewpoints:
                self.vehicle.command_loiter()
                self.recorder.set_final_report(
                    status="COMPLETE",
                    estimated_transmitter={
                        "latitude": self.result.latitude,
                        "longitude": self.result.longitude,
                    },
                    uncertainty_radius_m=self.result.uncertainty_radius_m,
                    rms_residual_m=self.result.rms_residual_m,
                    total_observations_logged=len(self.recorder.observations),
                    accepted_observations=len(self.observations),
                    estimator_accepted_count=self.result.accepted_count,
                    estimator_rejected_count=self.result.rejected_count,
                    target_error_radius_m=self.config.df.target_error_radius_m,
                    best_crossing_angle_deg=self.result.best_crossing_angle_deg,
                    geometry_condition_number=self.result.geometry_condition_number,
                    max_leave_one_out_shift_m=self.result.max_leave_one_out_shift_m,
                )
                self.transition(MissionState.COMPLETE)
            else:
                self.transition(MissionState.PLANNING_MOVE)
        elif self.state == MissionState.PLANNING_MOVE:
            state = self.vehicle.get_vehicle_state()
            if not self._state_valid(state) or not self.vehicle.navigation_valid():
                self.fail_safe("navigation invalid before waypoint planning"); return
            current = GeoPoint(state.latitude, state.longitude, state.altitude_m)
            estimate = None if self.result is None else GeoPoint(self.result.latitude, self.result.longitude)
            try:
                for attempt in range(1, self.config.navigation.planner_max_attempts + 1):
                    waypoint = self.planner.plan(current, self.observations, estimate)
                    altitude = self.config.navigation.command_altitude_m or state.altitude_m
                    movement = distance_m(current, waypoint.point)
                    if self._commanded_distance_m + movement > self.config.navigation.max_total_distance_m:
                        raise RuntimeError("maximum total mission distance would be exceeded")
                    try:
                        self.vehicle.goto_location(waypoint.point.latitude, waypoint.point.longitude, altitude)
                        break
                    except RuntimeError as exc:
                        self.recorder.event("waypoint_rejected", attempt=attempt, reason=str(exc),
                                            latitude=waypoint.point.latitude,
                                            longitude=waypoint.point.longitude)
                        if not self.vehicle.navigation_valid() or attempt >= self.config.navigation.planner_max_attempts:
                            raise
                        self.planner.mark_unavailable(waypoint.point)
            except (ValueError, RuntimeError) as exc:
                self.fail_safe(f"waypoint refused: {exc}"); return
            command_evidence = getattr(self.vehicle, "last_waypoint_result", {})
            self.recorder.event("waypoint", latitude=waypoint.point.latitude, longitude=waypoint.point.longitude,
                                altitude_m=altitude, rationale=waypoint.rationale,
                                **command_evidence)
            self._commanded_distance_m += movement
            self._move_started_at = self._clock()
            self.transition(MissionState.MOVING)
        elif self.state == MissionState.MOVING:
            state = self.vehicle.get_vehicle_state()
            if not self._state_valid(state) or not self.vehicle.navigation_valid():
                self.fail_safe("navigation invalid while moving"); return
            if (self._move_started_at is not None
                    and self._clock() - self._move_started_at > self.config.navigation.movement_timeout_seconds):
                self.fail_safe("movement timeout exceeded"); return
            remaining = self.vehicle.distance_to_target()
            if remaining is not None and remaining <= self.config.navigation.arrival_tolerance_m:
                self._stable_since = None
                self.transition(MissionState.STABILIZING)

    def run(self, max_ticks: int = 1000, tick_seconds: float = 0.1) -> EstimateResult | None:
        try:
            for _ in range(max_ticks):
                if self.state in {MissionState.COMPLETE, MissionState.ABORTED, MissionState.PAUSED}:
                    break
                self.ticks_executed += 1
                self.tick()
                time.sleep(tick_seconds)
            else:
                if self.state not in {MissionState.COMPLETE, MissionState.ABORTED, MissionState.PAUSED}:
                    self.fail_safe("maximum mission tick limit exceeded")
        except KeyboardInterrupt:
            self.transition(MissionState.ABORTED)
            self.recorder.event("operator_abort", reason="Ctrl+C")
        except Exception as exc:
            self.fail_safe(f"unhandled mission error: {exc}")
        finally:
            self.source.close(); self.vehicle.close(); self.recorder.flush()
        return self.result
