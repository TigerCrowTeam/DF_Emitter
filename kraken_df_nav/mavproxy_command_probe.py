"""M2 bounded movement and LOITER proof through a MAVProxy UDP route."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time

from pymavlink import mavutil

from .df.geometry import GeoPoint, destination, distance_m


def _validate_target(system: int, component: int, expected: tuple[int, int] = (1, 1)) -> None:
    if (system, component) != expected:
        raise RuntimeError(
            f"wrong target {system}:{component}; expected {expected[0]}:{expected[1]}")


def _ack_accepted(ack: dict) -> bool:
    return ack["result"] in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                             mavutil.mavlink.MAV_RESULT_IN_PROGRESS)


def run(connection: str, movement_m: float, timeout: float) -> dict:
    master = mavutil.mavlink_connection(
        connection, source_system=252,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    heartbeat = master.wait_heartbeat(timeout=30)
    if heartbeat is None:
        raise TimeoutError("no heartbeat received through MAVProxy")
    target_system, target_component = heartbeat.get_srcSystem(), heartbeat.get_srcComponent()
    _validate_target(target_system, target_component)
    mapping = master.mode_mapping() or {}
    guided, loiter = mapping.get("GUIDED"), mapping.get("LOITER")
    if guided is None or loiter is None:
        raise RuntimeError("vehicle does not advertise GUIDED and LOITER")

    # Harmless bidirectional proof: request GLOBAL_POSITION_INT at 5 Hz.
    master.mav.command_long_send(
        target_system, target_component, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200_000, 0, 0, 0, 0, 0)
    acknowledgements = []
    initial_msg = None
    ready_deadline = time.monotonic()+15
    while time.monotonic() < ready_deadline and initial_msg is None:
        msg = master.recv_match(type=["GLOBAL_POSITION_INT", "COMMAND_ACK"], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "COMMAND_ACK":
            acknowledgements.append({"command": msg.command, "result": msg.result})
        else:
            initial_msg = msg
    if initial_msg is None:
        raise TimeoutError("no initial position received")
    initial = GeoPoint(initial_msg.lat/1e7, initial_msg.lon/1e7, initial_msg.relative_alt/1000)
    target = destination(initial, 90, movement_m)
    target = GeoPoint(target.latitude, target.longitude, initial.altitude_m)

    master.mav.set_position_target_global_int_send(
        0, target_system, target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b110111111000, int(target.latitude*1e7), int(target.longitude*1e7), target.altitude_m,
        0, 0, 0, 0, 0, 0, 0, 0)
    movement_started = time.monotonic()
    closest = movement_m
    moved = 0.0
    final_position = initial
    reached = False
    unexpected_mode = None
    deadline = movement_started+timeout
    while time.monotonic() < deadline:
        msg = master.recv_match(type=["GLOBAL_POSITION_INT", "COMMAND_ACK", "HEARTBEAT"],
                                blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "COMMAND_ACK":
            acknowledgements.append({"command": msg.command, "result": msg.result})
        elif msg.get_type() == "GLOBAL_POSITION_INT":
            final_position = GeoPoint(msg.lat/1e7, msg.lon/1e7, msg.relative_alt/1000)
            moved = max(moved, distance_m(initial, final_position))
            closest = min(closest, distance_m(final_position, target))
            if closest <= 3:
                reached = True
                break
        elif msg.custom_mode != guided:
            unexpected_mode = msg.custom_mode
            break

    # SET_MODE has no required COMMAND_ACK here; confirm independently by heartbeat.
    master.mav.set_mode_send(target_system,
                             mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, loiter)
    loiter_confirmed = False
    mode_deadline = time.monotonic()+15
    while time.monotonic() < mode_deadline:
        msg = master.recv_match(type=["HEARTBEAT", "COMMAND_ACK"], blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == "COMMAND_ACK":
            acknowledgements.append({"command": msg.command, "result": msg.result})
        elif msg.custom_mode == loiter:
            loiter_confirmed = True
            break

    interval_acks = [a for a in acknowledgements
                     if a["command"] == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL]
    interval_accepted = any(_ack_accepted(a) for a in interval_acks)
    passed = (reached and moved >= 5 and loiter_confirmed and interval_accepted
              and unexpected_mode is None)
    return {
        "schema_version": 1, "generated_at": datetime.now(timezone.utc).isoformat(),
        "status": "PASS" if passed else "FAIL", "connection": connection,
        "source_identity": {"system": 252, "component": 191},
        "target_identity": {"system": target_system, "component": target_component},
        "message_rate_request": {"message": "GLOBAL_POSITION_INT", "rate_hz": 5,
                                 "acknowledgements": interval_acks,
                                 "accepted": interval_accepted},
        "position_target": {
            "message": "SET_POSITION_TARGET_GLOBAL_INT", "requested_distance_m": movement_m,
            "initial": initial.__dict__, "target": target.__dict__,
            "last_position": final_position.__dict__, "maximum_movement_observed_m": moved,
            "closest_distance_to_target_m": closest, "reached_within_3m": reached,
            "acknowledgement_semantics": "no normal COMMAND_ACK; result inferred from telemetry",
            "inferred_result": "ACCEPTED_AND_REACHED" if reached else "TIMED_OUT",
            "unexpected_mode_during_movement": unexpected_mode,
        },
        "loiter": {"message": "SET_MODE", "confirmed_by_heartbeat": loiter_confirmed,
                   "custom_mode": loiter},
        "all_command_acknowledgements": acknowledgements,
    }


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--connection", default="udpin:127.0.0.1:14560")
    p.add_argument("--movement-m", type=float, default=30)
    p.add_argument("--timeout", type=float, default=60)
    p.add_argument("--output", default="logs/mavproxy_m2_command_report.json")
    args = p.parse_args()
    if not 5 <= args.movement_m <= 50:
        p.error("--movement-m must be in [5, 50] for the M2 bounded SITL proof")
    try:
        report = run(args.connection, args.movement_m, args.timeout)
    except (OSError, RuntimeError, TimeoutError) as exc:
        report = {"schema_version": 1, "generated_at": datetime.now(timezone.utc).isoformat(),
                  "status": "FAIL", "connection": args.connection, "error": str(exc)}
    path = Path(args.output); path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if report["status"] == "PASS" else 2


if __name__ == "__main__": raise SystemExit(main())
