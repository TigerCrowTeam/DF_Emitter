"""Vehicle controller abstraction and pymavlink implementation."""
from __future__ import annotations

from abc import ABC, abstractmethod
import math
import time

from ..df.geometry import GeoPoint, distance_m
from .telemetry import VehicleState


class VehicleController(ABC):
    @abstractmethod
    def connect(self) -> None: ...
    @abstractmethod
    def get_vehicle_state(self) -> VehicleState | None: ...
    @abstractmethod
    def goto_location(self, lat: float, lon: float, alt: float) -> None: ...
    @abstractmethod
    def distance_to_target(self) -> float | None: ...
    @abstractmethod
    def command_loiter(self) -> None: ...
    @abstractmethod
    def navigation_valid(self) -> bool: ...
    def close(self) -> None: pass


class PymavlinkVehicleController(VehicleController):
    def __init__(self, connection: str, heartbeat_timeout_seconds: float = 10,
                 telemetry_timeout_seconds: float = 5, source_system: int = 252,
                 source_component: int = 191, expected_target_system: int | None = None,
                 expected_target_component: int | None = None):
        self.connection_string = connection
        self.heartbeat_timeout = heartbeat_timeout_seconds
        self.telemetry_timeout = telemetry_timeout_seconds
        self.source_system = source_system
        self.source_component = source_component
        self.expected_target_system = expected_target_system
        self.expected_target_component = expected_target_component
        self.master = None
        self.state: VehicleState | None = None
        self.target: GeoPoint | None = None
        self._attitude: dict[str, float] = {}
        self._gps: dict[str, float] = {}
        self._status: dict[str, object] = {}
        self.last_waypoint_result: dict[str, object] = {}

    def connect(self) -> None:
        try:
            from pymavlink import mavutil
        except ImportError as exc:
            raise RuntimeError("pymavlink is required for SITL/LIVE modes") from exc
        self.master = mavutil.mavlink_connection(
            self.connection_string, source_system=self.source_system,
            source_component=self.source_component)
        heartbeat = self.master.wait_heartbeat(timeout=self.heartbeat_timeout)
        if heartbeat is None:
            raise TimeoutError("no MAVLink heartbeat received")
        actual_system = heartbeat.get_srcSystem()
        actual_component = heartbeat.get_srcComponent()
        if (self.expected_target_system is not None
                and actual_system != self.expected_target_system):
            raise RuntimeError(f"unexpected target system {actual_system}; expected {self.expected_target_system}")
        if (self.expected_target_component is not None
                and actual_component != self.expected_target_component):
            raise RuntimeError(
                f"unexpected target component {actual_component}; expected {self.expected_target_component}")
        # pymavlink commonly leaves target_component at zero after heartbeat
        # discovery.  Preserve the observed, validated FC identity explicitly.
        self.master.target_system = actual_system
        self.master.target_component = actual_component
        self._status.update(
            flight_mode=mavutil.mode_string_v10(heartbeat),
            armed=bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED),
        )
        # Request common telemetry at 5 Hz. This does not arm or change flight mode.
        for message_id in (
            mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        ):
            self.master.mav.command_long_send(
                self.master.target_system, self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                message_id, 200_000, 0, 0, 0, 0, 0,
            )
        deadline = time.monotonic() + self.heartbeat_timeout
        required_gps = {"latitude", "longitude", "altitude_m", "heading_deg", "timestamp"}
        while time.monotonic() < deadline:
            self._drain(0.5)
            if (required_gps.issubset(self._gps)
                    and {"roll_deg", "pitch_deg"}.issubset(self._attitude)):
                return
        raise TimeoutError("MAVLink connected but complete initial navigation telemetry was not received")

    def _drain(self, wait_seconds: float = 0.25) -> None:
        if self.master is None:
            return
        deadline = time.monotonic() + wait_seconds
        while time.monotonic() < deadline:
            msg = self.master.recv_match(blocking=True, timeout=max(0.01, deadline - time.monotonic()))
            if msg is None:
                break
            name = msg.get_type()
            now = time.time()
            if name == "GLOBAL_POSITION_INT":
                self._gps.update(latitude=msg.lat / 1e7, longitude=msg.lon / 1e7,
                                 altitude_m=msg.relative_alt / 1000,
                                 heading_deg=(msg.hdg / 100 if msg.hdg != 65535 else math.nan),
                                 ground_speed_m_s=math.hypot(msg.vx, msg.vy) / 100,
                                 timestamp=now)
            elif name == "GPS_RAW_INT":
                accuracy = 0.0 if msg.eph == 65535 else msg.eph / 100
                self._gps.update(gps_fix_type=msg.fix_type, satellites_visible=msg.satellites_visible,
                                 position_accuracy_m=accuracy)
            elif name == "ATTITUDE":
                self._attitude.update(roll_deg=math.degrees(msg.roll), pitch_deg=math.degrees(msg.pitch))
            elif name == "HEARTBEAT":
                from pymavlink import mavutil
                self._status.update(flight_mode=mavutil.mode_string_v10(msg), armed=bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))

    def get_vehicle_state(self) -> VehicleState | None:
        self._drain()
        required = {"latitude", "longitude", "altitude_m", "heading_deg", "timestamp"}
        if not required.issubset(self._gps) or not {"roll_deg", "pitch_deg"}.issubset(self._attitude):
            return None
        values = {"gps_fix_type": 0, "satellites_visible": 0, "flight_mode": "UNKNOWN", "armed": False,
                  **self._gps, **self._attitude, **self._status}
        self.state = VehicleState(**values)
        return self.state

    def goto_location(self, lat: float, lon: float, alt: float) -> None:
        if self.master is None or not self.navigation_valid():
            raise RuntimeError("navigation is not valid; refusing waypoint")
        from pymavlink import mavutil
        if not (-90 <= lat <= 90 and -180 <= lon <= 180 and -100 <= alt <= 20_000):
            raise ValueError("obviously invalid waypoint")
        self.target = GeoPoint(lat, lon, alt)
        self._send_target()

    def _send_target(self) -> None:
        if self.master is None or self.target is None:
            raise RuntimeError("MAVLink target is unavailable")
        from pymavlink import mavutil
        # Each planned observation point is a discrete waypoint.  ArduPilot's
        # DO_REPOSITION command reliably starts a new WPNav leg in GUIDED mode;
        # raw SET_POSITION_TARGET messages are better suited to a continuous
        # offboard-control stream and can leave a completed leg stationary.
        self.master.mav.command_int_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION,
            0, 0, -1, 0, 0, math.nan,
            int(self.target.latitude * 1e7), int(self.target.longitude * 1e7),
            self.target.altitude_m,
        )
        command = mavutil.mavlink.MAV_CMD_DO_REPOSITION
        ack = None
        deadline = time.monotonic() + min(3.0, self.telemetry_timeout)
        while time.monotonic() < deadline:
            candidate = self.master.recv_match(type="COMMAND_ACK", blocking=True,
                                               timeout=max(0.01, deadline-time.monotonic()))
            if candidate is None:
                break
            if candidate.command == command:
                ack = candidate
                break
        if ack is None:
            raise RuntimeError("DO_REPOSITION acknowledgement timed out")
        result_name = mavutil.mavlink.enums["MAV_RESULT"][ack.result].name
        self.last_waypoint_result = {
            "command": "MAV_CMD_DO_REPOSITION",
            "target_system": self.master.target_system,
            "target_component": self.master.target_component,
            "ack_result": result_name,
            "ack_result_code": ack.result,
        }
        if ack.result not in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                              mavutil.mavlink.MAV_RESULT_IN_PROGRESS):
            raise RuntimeError(f"DO_REPOSITION rejected: {result_name}")
    def distance_to_target(self) -> float | None:
        state = self.get_vehicle_state()
        return None if state is None or self.target is None else distance_m(GeoPoint(state.latitude, state.longitude), self.target)

    def command_loiter(self) -> None:
        if self.master is None:
            raise RuntimeError("MAVLink not connected")
        mapping = self.master.mode_mapping() or {}
        mode_id = mapping.get("LOITER")
        if mode_id is None:
            raise RuntimeError("LOITER is unavailable for this vehicle")
        from pymavlink import mavutil
        self.master.mav.set_mode_send(self.master.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)

    def navigation_valid(self) -> bool:
        return self.state is not None and self.state.navigation_valid(self.telemetry_timeout)
