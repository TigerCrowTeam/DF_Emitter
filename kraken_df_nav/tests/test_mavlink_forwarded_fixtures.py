from __future__ import annotations

from types import SimpleNamespace

import pytest

from kraken_df_nav.df.geometry import GeoPoint
from kraken_df_nav.mavlink.connection import PymavlinkVehicleController


class Message(SimpleNamespace):
    def get_type(self):
        return self.kind


class FakeMav:
    def __init__(self):
        self.commands = []

    def command_int_send(self, *args):
        self.commands.append(args)


class FakeMaster:
    target_system = 1
    target_component = 1

    def __init__(self, messages):
        self.messages = list(messages)
        self.mav = FakeMav()

    def recv_match(self, **_kwargs):
        return self.messages.pop(0) if self.messages else None


def test_forwarded_packet_fixtures_update_cached_telemetry():
    controller = PymavlinkVehicleController("unused")
    controller.master = FakeMaster([
        Message(kind="GLOBAL_POSITION_INT", lat=370000000, lon=-1220000000,
                relative_alt=10000, hdg=9000, vx=300, vy=400),
        Message(kind="GPS_RAW_INT", eph=125, fix_type=6, satellites_visible=14),
        Message(kind="ATTITUDE", roll=0.1, pitch=-0.2),
    ])
    controller._drain(0.01)
    assert controller._gps["latitude"] == 37
    assert controller._gps["heading_deg"] == 90
    assert controller._gps["ground_speed_m_s"] == 5
    assert controller._gps["position_accuracy_m"] == 1.25
    assert controller._attitude["roll_deg"] == pytest.approx(5.72958, rel=1e-4)


def test_waypoint_acceptance_is_recorded_from_forwarded_ack():
    controller = PymavlinkVehicleController("unused")
    controller.master = FakeMaster([Message(kind="COMMAND_ACK", command=192, result=0)])
    controller.target = GeoPoint(37.0001, -122.0001, 10)
    controller._send_target()
    assert controller.last_waypoint_result["ack_result"] == "MAV_RESULT_ACCEPTED"
    assert controller.last_waypoint_result["target_system"] == 1


def test_waypoint_ack_timeout_is_a_refusal():
    controller = PymavlinkVehicleController("unused", telemetry_timeout_seconds=0.01)
    controller.master = FakeMaster([])
    controller.target = GeoPoint(37.0001, -122.0001, 10)
    with pytest.raises(RuntimeError, match="acknowledgement timed out"):
        controller._send_target()
