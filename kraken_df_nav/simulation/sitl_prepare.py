"""Prepare a localhost-only ArduCopter SITL instance for a movement test.

This deliberately separate harness arms and launches SITL. The DF application
itself never arms or takes off, including during this test.
"""
from __future__ import annotations

import argparse
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", default="tcp:127.0.0.1:5762")
    parser.add_argument("--altitude", type=float, default=60)
    args = parser.parse_args()
    if not args.connection.startswith(("tcp:127.0.0.1:", "udp:127.0.0.1:",
                                       "udpin:127.0.0.1:")):
        parser.error("this safety-limited helper only connects to localhost SITL")
    if not 2 <= args.altitude <= 120:
        parser.error("SITL takeoff altitude must be between 2 and 120 metres")

    master = mavutil.mavlink_connection(args.connection)
    initial_heartbeat = master.wait_heartbeat(timeout=15)
    if initial_heartbeat is None:
        raise TimeoutError("no SITL heartbeat received")
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL, 5, 1,
    )
    initial_position = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    initial_altitude = None if initial_position is None else initial_position.relative_alt / 1000
    if (initial_heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            and initial_altitude is not None and initial_altitude >= args.altitude - 2):
        print(f"SITL already ready at {initial_altitude:.1f} m")
        return 0
    mapping = master.mode_mapping() or {}
    guided = mapping.get("GUIDED")
    if guided is None:
        raise RuntimeError("connected vehicle does not advertise GUIDED mode")
    master.set_mode(guided)
    mode_deadline = time.monotonic() + 10
    while time.monotonic() < mode_deadline:
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if heartbeat is not None and heartbeat.custom_mode == guided:
            break
    else:
        raise TimeoutError("SITL did not enter GUIDED mode")
    if not initial_heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
        # Early SITL arm requests can be rejected while GPS/EKF initialization is
        # still finishing. Retry during a bounded readiness window instead of
        # disabling ArduPilot's normal arming checks.
        arm_deadline = time.monotonic() + 90
        next_arm_request = 0.0
        status_messages = []
        while time.monotonic() < arm_deadline:
            if time.monotonic() >= next_arm_request:
                master.arducopter_arm()
                next_arm_request = time.monotonic() + 5
            message = master.recv_match(type=["HEARTBEAT", "STATUSTEXT", "COMMAND_ACK"],
                                        blocking=True, timeout=1)
            if message is not None and message.get_type() == "STATUSTEXT":
                status_messages.append(message.text)
            if (message is not None and message.get_type() == "HEARTBEAT"
                    and message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                break
        else:
            detail = "; ".join(status_messages[-5:]) or "no status text received"
            raise TimeoutError(f"SITL did not arm: {detail}")
    # Initialization can return SITL to STABILIZE after the earlier mode
    # request. Reassert GUIDED after arming and verify it immediately before
    # issuing the guided takeoff command.
    ready_deadline = time.monotonic() + 90
    next_mode_request = next_arm_request = 0.0
    while time.monotonic() < ready_deadline:
        now = time.monotonic()
        if now >= next_mode_request:
            master.set_mode(guided)
            next_mode_request = now + 3
        heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if heartbeat is None:
            continue
        armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        if heartbeat.custom_mode == guided and armed:
            break
        if heartbeat.custom_mode == guided and not armed and now >= next_arm_request:
            master.arducopter_arm()
            next_arm_request = now + 3
    else:
        raise TimeoutError("SITL did not reach simultaneous GUIDED and armed readiness")
    heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=3)
    position = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=3)
    if heartbeat is not None:
        print(f"Pre-takeoff state: mode={mavutil.mode_string_v10(heartbeat)} "
              f"armed={bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)}")
    if position is not None:
        print(f"Pre-takeoff altitude: relative={position.relative_alt / 1000:.1f} m "
              f"MSL={position.alt / 1000:.1f} m")
    takeoff_deadline = time.monotonic() + 30
    takeoff_status = []
    takeoff_results = []
    accepted_result = None
    while time.monotonic() < takeoff_deadline and accepted_result is None:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, args.altitude,
        )
        retry_deadline = time.monotonic() + 5
        while time.monotonic() < retry_deadline:
            ack = master.recv_match(type=["COMMAND_ACK", "STATUSTEXT"], blocking=True, timeout=1)
            if ack is not None and ack.get_type() == "STATUSTEXT":
                takeoff_status.append(ack.text)
            elif (ack is not None and ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF
                  and ack.result in (mavutil.mavlink.MAV_RESULT_ACCEPTED,
                                     mavutil.mavlink.MAV_RESULT_IN_PROGRESS)):
                accepted_result = ack.result
                break
            elif ack is not None and ack.command == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                takeoff_results.append(str(ack.result))
    if accepted_result is None:
        detail = "; ".join(takeoff_status[-5:]) or "no status text received"
        if takeoff_results:
            detail += f"; MAV_RESULT sequence: {','.join(takeoff_results)}"
        raise RuntimeError(f"SITL did not accept takeoff: {detail}")
    print(f"SITL accepted takeoff command (MAV_RESULT {accepted_result})")
    deadline = time.monotonic() + 60
    last_altitude = None
    while time.monotonic() < deadline:
        position = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if position is not None:
            last_altitude = position.relative_alt / 1000
            if last_altitude >= args.altitude - 2:
                print(f"SITL ready in GUIDED at {last_altitude:.1f} m")
                return 0
    raise TimeoutError(f"SITL did not reach takeoff altitude; last altitude was {last_altitude!r} m")


if __name__ == "__main__":
    raise SystemExit(main())
