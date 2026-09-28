"""Process-owning ArduPilot + MAVProxy + DF application SITL runner."""

from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import time
from typing import Any

import yaml

from .config import load_config
from .evidence import ardupilot_revision, environment_manifest
from .mavproxy_preflight import run as route_preflight
from .sitl_orchestrator import (SCENARIOS, _deep_merge, _latest_mission, _launch,
                                _loiter_confirmed, _score, _terminate, _validation_metrics)


MAVPROXY_SCENARIOS: dict[str, dict[str, Any]] = {
    **SCENARIOS,
    "dramatic": {
        "expect": "success", "pre_detection_transit_m": 60,
        "pre_detection_transit_bearing_deg": 90, "overrides": {},
    },
    "proxy_loss_collection": {
        "expect": "safe_stop", "kill_proxy_state": "COLLECTING_OBSERVATION",
        "overrides": {},
    },
    "proxy_loss_movement": {
        "expect": "safe_stop", "kill_proxy_state": "MOVING", "overrides": {},
    },
    "master_link_loss": {
        "expect": "safe_stop", "kill_sitl_state": "MOVING", "overrides": {},
    },
    "mode_conflict": {
        "expect": "safe_stop", "inject_mode_state": "MOVING", "overrides": {},
    },
    "movement_conflict": {
        "expect": "safe_stop", "inject_move_state": "MOVING",
        "overrides": {"navigation": {"movement_timeout_seconds": 25}},
    },
    "wrong_target": {
        "expect": "safe_stop",
        "overrides": {"mavlink": {"expected_target_system": 2}},
    },
    "degraded_telemetry": {
        "expect": "success", "streamrate": 1, "overrides": {},
    },
    "interrupt_collection": {
        "expect": "safe_stop", "interrupt_state": "COLLECTING_OBSERVATION", "overrides": {},
    },
    "interrupt_movement": {
        "expect": "safe_stop", "interrupt_state": "MOVING", "overrides": {},
    },
    "interrupt_estimation": {
        "expect": "safe_stop", "interrupt_state": "ESTIMATING", "overrides": {},
    },
}


def _wait_tcp(host: str, port: int, process: subprocess.Popen, timeout: float) -> None:
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError("SITL exited before its MAVLink master became ready")
        try:
            with socket.create_connection((host, port), timeout=.5):
                return
        except OSError:
            time.sleep(.25)
    raise TimeoutError(f"SITL master tcp:{host}:{port} was not ready within {timeout}s")


def _gui_ready(python: Path) -> tuple[bool, str]:
    result = subprocess.run(
        [str(python), "-c", "import wx, cv2, PIL, matplotlib; "
         "from MAVProxy.modules.mavproxy_map import mp_slipmap; "
         "from MAVProxy.modules import mavproxy_console"],
        capture_output=True, text=True)
    return result.returncode == 0, (result.stderr or result.stdout).strip()


def _log_has_state(path: Path, state: str) -> bool:
    try:
        return f'"current": "{state}"' in path.read_text(encoding="utf-8")
    except OSError:
        return False


def _inject_mode(connection: str, mode: str = "ALT_HOLD") -> dict[str, Any]:
    from pymavlink import mavutil
    master = mavutil.mavlink_connection(connection, source_system=253, source_component=190)
    try:
        heartbeat = master.wait_heartbeat(timeout=5)
        if heartbeat is None:
            raise TimeoutError("fault injector received no heartbeat")
        mapping = master.mode_mapping() or {}
        if mode not in mapping:
            raise RuntimeError(f"fault-injection mode {mode} unavailable")
        master.mav.set_mode_send(heartbeat.get_srcSystem(),
                                 mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                                 mapping[mode])
        return {"action": "set_mode", "mode": mode, "source_system": 253,
                "target_system": heartbeat.get_srcSystem()}
    finally:
        master.close()


def _inject_move(connection: str) -> dict[str, Any]:
    from pymavlink import mavutil
    master = mavutil.mavlink_connection(connection, source_system=255, source_component=192)
    try:
        heartbeat = master.wait_heartbeat(timeout=5)
        position = master.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
        if heartbeat is None or position is None:
            raise TimeoutError("movement injector received incomplete telemetry")
        # Move roughly 35 m north of the current position, deliberately
        # conflicting with the application's active observation waypoint.
        master.mav.command_int_send(
            heartbeat.get_srcSystem(), heartbeat.get_srcComponent(),
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_DO_REPOSITION, 0, 0, -1, 0, 0, float("nan"),
            position.lat + 3144, position.lon, position.relative_alt / 1000,
        )
        return {"action": "reposition", "source_system": 255, "source_component": 192,
                "target_system": heartbeat.get_srcSystem(), "latitude_e7": position.lat + 3144,
                "longitude_e7": position.lon}
    finally:
        master.close()


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--scenario", choices=sorted(MAVPROXY_SCENARIOS), default="nominal")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--config", default=str(Path(__file__).with_name("config.mavproxy_sitl.yaml")))
    p.add_argument("--sim-vehicle", default="tools/ardupilot/Tools/autotest/sim_vehicle.py")
    p.add_argument("--vehicle", default="ArduCopter")
    p.add_argument("--timeout", type=float, default=720)
    p.add_argument("--readiness-timeout", type=float, default=60)
    p.add_argument("--output-root", default="./logs/mavproxy/sitl")
    p.add_argument("--wind-speed", type=float, default=3)
    p.add_argument("--wind-direction", type=float, default=225)
    p.add_argument("--wind-turbulence", type=float, default=.2)
    p.add_argument("--headless", action="store_true", help="disable map/console for automation")
    p.add_argument("--keep-open", action="store_true",
                   help="after a successful GUI run, keep map/console open until Ctrl+C")
    p.add_argument("--preflight-only", action="store_true")
    return p


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    app_python = Path(sys.executable)
    root = Path.cwd()
    gui_python = root/".venv-mavproxy/bin/python"
    proxy_python = app_python if args.headless else gui_python
    sim_vehicle = (shutil.which(args.sim_vehicle) if os.sep not in args.sim_vehicle else
                   str(Path(args.sim_vehicle).resolve()))
    preflight: dict[str, Any] = {
        "application_python": str(app_python), "mavproxy_python": str(proxy_python),
        "sim_vehicle": sim_vehicle, "config": str(Path(args.config).resolve()),
        "scenario": args.scenario, "headless": args.headless,
    }
    try:
        config = load_config(args.config)
        if sim_vehicle is None:
            raise ValueError("sim_vehicle.py not found")
        if not args.headless:
            if not proxy_python.is_file():
                raise ValueError("GUI environment missing; run ./demo/setup_mavproxy_gui.sh")
            ready, detail = _gui_ready(proxy_python)
            if not ready:
                raise ValueError(f"MAVProxy map/console dependencies are incomplete: {detail}")
        preflight["route"] = route_preflight(args.config)
        preflight["status"] = "READY" if preflight["route"]["status"] == "PASS" else "FAIL"
    except (OSError, ValueError) as exc:
        preflight.update(status="FAIL", error=str(exc))
    if args.preflight_only or preflight["status"] != "READY":
        print(json.dumps(preflight, indent=2, default=str))
        return 0 if preflight["status"] == "READY" else 3

    scenario = MAVPROXY_SCENARIOS[args.scenario]
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = Path(args.output_root)/f"{args.scenario}_seed{args.seed}_{stamp}"
    mission_dir = run_dir/"mission"
    run_dir.mkdir(parents=True, exist_ok=False); mission_dir.mkdir()
    raw = yaml.safe_load(Path(args.config).read_text()) or {}
    _deep_merge(raw, scenario["overrides"])
    raw["simulation"]["random_seed"] = args.seed
    raw["logging"]["directory"] = str(mission_dir.resolve())
    resolved_path = run_dir/"resolved_config.yaml"
    resolved_path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    config = load_config(resolved_path)
    (run_dir/"preflight.json").write_text(json.dumps(preflight, indent=2, default=str)+"\n")

    home = f"{config.simulation.start_lat},{config.simulation.start_lon},0,0"
    sitl_cmd = [str(app_python), str(sim_vehicle), "-v", args.vehicle, "-f", "quad",
                "--no-mavproxy", "--no-rebuild", "--wipe-eeprom", "--custom-location", home,
                "--param", f"SIM_WIND_SPD={args.wind_speed}",
                "--param", f"SIM_WIND_DIR={args.wind_direction%360}",
                "--param", f"SIM_WIND_TURB={args.wind_turbulence}"]
    proxy = config.mavproxy
    proxy_cmd = [str(proxy_python), "-m", "MAVProxy.mavproxy",
                 f"--master={proxy.master_connection}",
                 f"--out=udp:{proxy.loopback_host}:{proxy.poc_output_port}",
                 f"--out=udp:{proxy.loopback_host}:{proxy.preparation_output_port}",
                 f"--source-system={proxy.source_system}",
                 f"--source-component={proxy.source_component}",
                 f"--target-system={config.mavlink.expected_target_system}",
                 f"--target-component={config.mavlink.expected_target_component}",
                 f"--streamrate={scenario.get('streamrate', 5)}",
                 f"--state-basedir={run_dir.resolve()}", "--non-interactive"]
    if args.headless:
        proxy_cmd.append("--daemon")
    else:
        proxy_cmd.extend(["--map", "--console", "--load-module=mavproxy_emitter_marker"])
    prepare_cmd = [str(app_python), "-m", "kraken_df_nav.simulation.sitl_prepare",
                   "--connection", f"udpin:{proxy.loopback_host}:{proxy.preparation_output_port}",
                   "--altitude", str(config.simulation.altitude_m)]
    readiness_cmd = [str(app_python), "-m", "kraken_df_nav.mavproxy_telemetry_probe",
                     "--connection", config.mavlink.connection, "--duration", "2",
                     "--heartbeat-timeout", str(args.readiness_timeout),
                     "--output", str((run_dir/"telemetry_readiness.json").resolve())]
    transit_cmd = [str(app_python), "-m", "kraken_df_nav.simulation.sitl_transit",
                   "--connection", f"udpin:{proxy.loopback_host}:{proxy.preparation_output_port}",
                   "--distance", str(scenario.get("pre_detection_transit_m", 0)),
                   "--bearing", str(scenario.get("pre_detection_transit_bearing_deg", 90)),
                   "--output", str((run_dir/"pre_detection_transit.json").resolve())]
    app_cmd = [str(app_python), "-m", "kraken_df_nav.main", "--config", str(resolved_path.resolve())]
    commands = {"sitl": sitl_cmd, "mavproxy": proxy_cmd, "prepare": prepare_cmd,
                "pre_detection_transit": transit_cmd if scenario.get("pre_detection_transit_m") else None,
                "telemetry_readiness": readiness_cmd, "application": app_cmd}
    report: dict[str, Any] = {
        "schema_version": 1, "generated_at": datetime.now(timezone.utc).isoformat(),
        "scenario": args.scenario, "seed": args.seed, "expectation": scenario["expect"],
        "status": "ERROR", "run_directory": str(run_dir.resolve()), "preflight": preflight,
        "commands": commands, "resolved_config": asdict(config), "gui_enabled": not args.headless,
        "evidence_manifest": environment_manifest(),
        "ardupilot": {"revision": ardupilot_revision(sim_vehicle), "vehicle": args.vehicle,
                      "home": home, "wind": [args.wind_speed, args.wind_direction, args.wind_turbulence]},
    }
    processes: dict[str, subprocess.Popen | None] = {k: None for k in
        ("sitl", "mavproxy", "prepare", "transit", "readiness", "application")}
    handles = []
    interrupted = False
    fault_evidence: list[dict[str, Any]] = []
    started = time.monotonic()
    try:
        processes["sitl"], h = _launch(sitl_cmd, run_dir/"sitl.log", run_dir.resolve()); handles.append(h)
        _wait_tcp("127.0.0.1", 5760, processes["sitl"], args.readiness_timeout)
        module_path = str((root / "demo").resolve())
        existing_pythonpath = os.environ.get("PYTHONPATH")
        proxy_env = {**os.environ,
                     "KRAKEN_DF_MARKER_FILE": str((run_dir/"emitter_marker.json").resolve()),
                     "PYTHONPATH": (module_path if not existing_pythonpath else
                                    module_path + os.pathsep + existing_pythonpath)}
        processes["mavproxy"], h = _launch(proxy_cmd, run_dir/"mavproxy.log",
                                             env=proxy_env); handles.append(h)
        processes["prepare"], h = _launch(prepare_cmd, run_dir/"prepare.log"); handles.append(h)
        if processes["prepare"].wait(timeout=180) != 0:
            raise RuntimeError("SITL preparation through MAVProxy failed")
        if scenario.get("pre_detection_transit_m"):
            processes["transit"], h = _launch(transit_cmd, run_dir/"pre_detection_transit.log")
            handles.append(h)
            if processes["transit"].wait(timeout=90) != 0:
                raise RuntimeError("pre-detection straight transit failed")
            detection = {"phase": "EMITTER_DETECTED",
                         "message": "RF EMITTER DETECTED - STARTING DF SEARCH",
                         "location_known": False}
            (run_dir/"emitter_marker.json").write_text(
                json.dumps(detection, indent=2)+"\n", encoding="utf-8")
            report["emitter_detection"] = detection
            if not args.headless:
                time.sleep(0.5)
        processes["readiness"], h = _launch(readiness_cmd, run_dir/"telemetry_readiness.log"); handles.append(h)
        if processes["readiness"].wait(timeout=args.readiness_timeout+10) != 0:
            raise RuntimeError("complete routed telemetry was not ready")
        processes["application"], h = _launch(app_cmd, run_dir/"application.log"); handles.append(h)
        app_started = time.monotonic()
        fault_at = scenario.get("kill_sitl_after") or scenario.get("interrupt_after")
        state_fault_pending = True
        while processes["application"].poll() is None:
            elapsed = time.monotonic()-app_started
            if elapsed > args.timeout:
                os.killpg(processes["application"].pid, signal.SIGINT); interrupted = True
            if fault_at and elapsed >= fault_at:
                if "kill_sitl_after" in scenario:
                    _terminate(processes["sitl"]); processes["sitl"] = None
                else:
                    os.killpg(processes["application"].pid, signal.SIGINT); interrupted = True
                fault_at = None
            if state_fault_pending:
                app_log = run_dir/"application.log"
                if scenario.get("kill_proxy_state") and _log_has_state(app_log, scenario["kill_proxy_state"]):
                    fault_evidence.append({"action": "terminate_mavproxy",
                                           "trigger_state": scenario["kill_proxy_state"]})
                    _terminate(processes["mavproxy"]); state_fault_pending = False
                elif scenario.get("kill_sitl_state") and _log_has_state(app_log, scenario["kill_sitl_state"]):
                    fault_evidence.append({"action": "terminate_sitl",
                                           "trigger_state": scenario["kill_sitl_state"]})
                    _terminate(processes["sitl"]); state_fault_pending = False
                elif scenario.get("inject_mode_state") and _log_has_state(app_log, scenario["inject_mode_state"]):
                    evidence = _inject_mode(
                        f"udpin:{proxy.loopback_host}:{proxy.preparation_output_port}")
                    evidence["trigger_state"] = scenario["inject_mode_state"]
                    fault_evidence.append(evidence); state_fault_pending = False
                elif scenario.get("inject_move_state") and _log_has_state(app_log, scenario["inject_move_state"]):
                    evidence = _inject_move(
                        f"udpin:{proxy.loopback_host}:{proxy.preparation_output_port}")
                    evidence["trigger_state"] = scenario["inject_move_state"]
                    fault_evidence.append(evidence); state_fault_pending = False
                elif scenario.get("interrupt_state") and _log_has_state(app_log, scenario["interrupt_state"]):
                    os.killpg(processes["application"].pid, signal.SIGINT)
                    interrupted = True
                    fault_evidence.append({"action": "operator_sigint",
                                           "trigger_state": scenario["interrupt_state"]})
                    state_fault_pending = False
            time.sleep(.25)
        mission = _latest_mission(mission_dir)
        metrics = _validation_metrics(mission, config)
        final = {} if mission is None else (mission.get("final_report") or {})
        estimate = final.get("estimated_transmitter") or {}
        if "latitude" in estimate and "longitude" in estimate:
            marker = {"symbol": "redantenna", "label": "DF EMITTER FOUND",
                      "latitude": estimate["latitude"], "longitude": estimate["longitude"],
                      "source": "final_estimate_not_simulated_truth"}
            (run_dir/"emitter_marker.json").write_text(
                json.dumps(marker, indent=2)+"\n", encoding="utf-8")
            report["emitter_map_marker"] = marker
            if not args.headless:
                time.sleep(1.0)
        loiter = (scenario["expect"] == "success" and processes["mavproxy"].poll() is None
                  and _loiter_confirmed(
                      f"udpin:{proxy.loopback_host}:{proxy.preparation_output_port}", timeout=10))
        passed, reasons = _score(args.scenario, scenario["expect"],
                                 processes["application"].returncode, mission, loiter, metrics)
        report.update(status="PASS" if passed else "FAIL", pass_reasons=reasons,
                      loiter_confirmed=loiter, validation_metrics=metrics, mission_record=mission,
                      application_exit_code=processes["application"].returncode,
                      fault_injection_evidence=fault_evidence)
        if args.keep_open and not args.headless and report["status"] == "PASS":
            report["viewer_hold"] = {"enabled": True, "ended_by": None}
            print("\nEMITTER FOUND - map remains open; press Ctrl+C here to close the demo.\n",
                  flush=True)
            try:
                while processes["mavproxy"] is not None and processes["mavproxy"].poll() is None:
                    time.sleep(.5)
            except KeyboardInterrupt:
                report["viewer_hold"]["ended_by"] = "operator_ctrl_c"
            else:
                report["viewer_hold"]["ended_by"] = "mavproxy_window_closed"
    except KeyboardInterrupt:
        interrupted = True; report.update(status="INTERRUPTED", pass_reasons=["operator Ctrl+C"])
    except (OSError, RuntimeError, subprocess.TimeoutExpired, TimeoutError) as exc:
        report.update(status="ERROR", pass_reasons=[str(exc)])
    finally:
        for process in reversed(list(processes.values())):
            _terminate(process)
        for h in handles: h.close()
        report["duration_seconds"] = round(time.monotonic()-started, 3)
        report["process_exit_codes"] = {k: None if v is None else v.poll() for k,v in processes.items()}
        report["all_children_stopped"] = all(v is None or v.poll() is not None for v in processes.values())
        report["operator_signal_sent"] = interrupted
        (run_dir/"validation_report.json").write_text(json.dumps(report, indent=2, default=str)+"\n")
        print(json.dumps({k: report.get(k) for k in ("status", "scenario", "seed", "gui_enabled",
                         "duration_seconds", "all_children_stopped", "pass_reasons", "run_directory")}, indent=2))
    return 0 if report["status"] == "PASS" else (130 if report["status"] == "INTERRUPTED" else 2)


if __name__ == "__main__": raise SystemExit(main())
