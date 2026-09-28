"""One-command ArduPilot SITL proof-of-concept demonstration runner.

The orchestration harness owns SITL arming/takeoff.  The DF application is a
separate process and retains no arming or disarming capability.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
from typing import Any

import yaml

from .config import load_config
from .df.geometry import GeoPoint, distance_m
from .evidence import ardupilot_revision, environment_manifest


SCENARIOS: dict[str, dict[str, Any]] = {
    "nominal": {"expect": "success", "overrides": {}},
    "kraken_loss": {
        "expect": "safe_stop",
        "overrides": {"simulation": {"digital_twin": {"dropout_probability": 1.0}},
                      "kraken": {"max_consecutive_rejections": 5}},
    },
    "mavlink_loss": {"expect": "safe_stop", "kill_sitl_after": 8.0, "overrides": {}},
    "poor_geometry": {
        "expect": "safe_stop",
        "overrides": {"navigation": {"observation_spacing_m": 2.0,
                                      "planner_min_separation_m": 0.0,
                                      "max_total_distance_m": 12.0}},
    },
    "multipath": {
        "expect": "safe_stop",
        "overrides": {"df": {"target_error_radius_m": 1.0},
                      "navigation": {"max_total_distance_m": 250.0},
                      "simulation": {"bearing_noise_std_deg": 12.0,
                                      "digital_twin": {"multipath_probability": 1.0,
                                                       "multipath_bias_deg": 35.0,
                                                       "multipath_persistence_samples": 20}}},
    },
    "operator_interrupt": {"expect": "safe_stop", "interrupt_after": 8.0, "overrides": {}},
}


def _deep_merge(target: dict[str, Any], changes: dict[str, Any]) -> None:
    for key, value in changes.items():
        if isinstance(value, dict) and isinstance(target.get(key), dict):
            _deep_merge(target[key], value)
        else:
            target[key] = value


def _terminate(process: subprocess.Popen | None, timeout: float = 8.0) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=timeout)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)


def _launch(command: list[str], log_path: Path, cwd: Path | None = None,
            env: dict[str, str] | None = None) -> tuple[subprocess.Popen, Any]:
    handle = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT,
                               start_new_session=True, text=True, cwd=cwd, env=env)
    return process, handle


def _latest_mission(directory: Path) -> dict[str, Any] | None:
    paths = sorted(directory.glob("mission_*.json"), key=lambda item: item.stat().st_mtime)
    return json.loads(paths[-1].read_text(encoding="utf-8")) if paths else None


def _loiter_confirmed(connection: str, timeout: float = 10.0) -> bool:
    from pymavlink import mavutil
    master = mavutil.mavlink_connection(connection)
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            heartbeat = master.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if heartbeat is not None and mavutil.mode_string_v10(heartbeat).upper() == "LOITER":
                return True
        return False
    finally:
        master.close()


def _score(scenario: str, expectation: str, app_exit: int | None,
           mission: dict[str, Any] | None, loiter: bool,
           validation_metrics: dict[str, Any] | None = None) -> tuple[bool, list[str]]:
    events = [] if mission is None else mission.get("events", [])
    states = [event.get("current") for event in events if event.get("type") == "state"]
    complete = "COMPLETE" in states
    safely_stopped = any(state in {"ABORTED", "PAUSED"} for state in states)
    stop_index = next((i for i, event in enumerate(events)
                       if event.get("type") in {"safety_stop", "operator_abort"}), None)
    post_stop_waypoints = (0 if stop_index is None else
                           sum(event.get("type") == "waypoint" for event in events[stop_index + 1:]))
    reasons: list[str] = []
    if expectation == "success":
        if app_exit != 0 or not complete:
            reasons.append("nominal mission did not complete")
        if not loiter:
            reasons.append("LOITER was not confirmed")
        metrics = validation_metrics or {}
        if metrics.get("actual_error_m") is None or metrics["actual_error_m"] > 10:
            reasons.append("nominal actual error exceeds 10 m or is unavailable")
        if metrics.get("uncertainty_radius_m") is None or metrics["uncertainty_radius_m"] > 15:
            reasons.append("nominal uncertainty exceeds 15 m")
        if metrics.get("rms_residual_m") is None or metrics["rms_residual_m"] > 12:
            reasons.append("nominal residual exceeds 12 m")
        if (metrics.get("best_crossing_angle_deg") is None
                or metrics["best_crossing_angle_deg"] < 35):
            reasons.append("nominal crossing angle is below 35 degrees")
        if metrics.get("accepted_observations") is None or metrics["accepted_observations"] > 9:
            reasons.append("nominal observation count exceeds 9")
    else:
        if complete:
            reasons.append(f"{scenario} unexpectedly completed")
        if not safely_stopped:
            reasons.append("terminal ABORTED/PAUSED state was not recorded")
    if post_stop_waypoints:
        reasons.append(f"{post_stop_waypoints} waypoint(s) followed a safety stop")
    return not reasons, reasons


def _validation_metrics(mission: dict[str, Any] | None, config) -> dict[str, Any]:
    final = {} if mission is None else (mission.get("final_report") or {})
    estimate = final.get("estimated_transmitter") or {}
    error = None
    if "latitude" in estimate and "longitude" in estimate:
        error = distance_m(
            GeoPoint(config.simulation.transmitter_lat, config.simulation.transmitter_lon),
            GeoPoint(float(estimate["latitude"]), float(estimate["longitude"])),
        )
    return {
        "actual_error_m": error,
        "uncertainty_radius_m": final.get("uncertainty_radius_m"),
        "rms_residual_m": final.get("rms_residual_m"),
        "best_crossing_angle_deg": final.get("best_crossing_angle_deg"),
        "accepted_observations": final.get("accepted_observations"),
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", choices=sorted(SCENARIOS), default="nominal")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--config", default=str(Path(__file__).with_name("config.sitl.yaml")))
    parser.add_argument("--sim-vehicle", default="sim_vehicle.py")
    parser.add_argument("--vehicle", default="ArduCopter")
    parser.add_argument("--timeout", type=float, default=720.0)
    parser.add_argument("--output-root", default="./logs/sitl")
    parser.add_argument("--wind-speed", type=float, default=3.0,
                        help="SITL wind speed in m/s (default: mild 3 m/s)")
    parser.add_argument("--wind-direction", type=float, default=225.0)
    parser.add_argument("--wind-turbulence", type=float, default=0.2)
    parser.add_argument("--preflight-only", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    scenario = SCENARIOS[args.scenario]
    python = Path(sys.executable)
    sim_vehicle = (shutil.which(args.sim_vehicle) if os.sep not in args.sim_vehicle else
                   str(Path(args.sim_vehicle).resolve()))
    preflight = {
        "python": str(python), "sim_vehicle": sim_vehicle,
        "base_config": str(Path(args.config).resolve()), "scenario": args.scenario,
    }
    if sim_vehicle is None or not Path(args.config).is_file():
        missing = "sim_vehicle.py" if sim_vehicle is None else args.config
        print(f"S8 preflight failed: required dependency not found: {missing}", file=sys.stderr)
        return 3
    if args.preflight_only:
        print(json.dumps({"status": "READY", **preflight}, indent=2))
        return 0

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_dir = Path(args.output_root) / f"{args.scenario}_seed{args.seed}_{stamp}"
    mission_dir = run_dir / "mission"
    run_dir.mkdir(parents=True, exist_ok=False)
    mission_dir.mkdir()
    raw = yaml.safe_load(Path(args.config).read_text(encoding="utf-8")) or {}
    _deep_merge(raw, scenario["overrides"])
    raw.setdefault("simulation", {})["random_seed"] = args.seed
    raw.setdefault("logging", {})["directory"] = str(mission_dir.resolve())
    resolved_path = run_dir / "resolved_config.yaml"
    resolved_path.write_text(yaml.safe_dump(raw, sort_keys=False), encoding="utf-8")
    config = load_config(resolved_path)
    if not 0 <= args.wind_speed <= 20 or not 0 <= args.wind_turbulence <= 1:
        raise SystemExit("wind speed must be 0..20 m/s and turbulence 0..1")
    environment_path = run_dir / "sitl_environment.parm"
    environment_path.write_text(
        f"SIM_WIND_SPD {args.wind_speed}\n"
        f"SIM_WIND_DIR {args.wind_direction % 360}\n"
        f"SIM_WIND_TURB {args.wind_turbulence}\n", encoding="utf-8")
    # sim_vehicle's custom-location altitude is the initial field elevation,
    # not the requested flight height. Keep the local datum at zero and let
    # sitl_prepare command the configured relative takeoff altitude.
    home = f"{config.simulation.start_lat},{config.simulation.start_lon},0,0"
    sitl_command = [str(python), str(sim_vehicle), "-v", args.vehicle, "-f", "quad",
                    "--no-mavproxy", "--no-rebuild", "--wipe-eeprom",
                    "--custom-location", home,
                    "--param", f"SIM_WIND_SPD={args.wind_speed}",
                    "--param", f"SIM_WIND_DIR={args.wind_direction % 360}",
                    "--param", f"SIM_WIND_TURB={args.wind_turbulence}"]
    prepare_command = [str(python), "-m", "kraken_df_nav.simulation.sitl_prepare",
                       "--connection", config.mavlink.connection,
                       "--altitude", str(config.simulation.altitude_m)]
    app_command = [str(python), "-m", "kraken_df_nav.main", "--config", str(resolved_path)]
    report: dict[str, Any] = {
        "generated_at": datetime.now(timezone.utc).isoformat(), "scenario": args.scenario,
        "expectation": scenario["expect"], "seed": args.seed, "status": "ERROR",
        "preflight": preflight, "commands": {"sitl": sitl_command, "prepare": prepare_command,
                                               "application": app_command},
        "resolved_config": asdict(config), "run_directory": str(run_dir.resolve()),
        "evidence_manifest": environment_manifest(),
        "ardupilot": {
            "revision": ardupilot_revision(sim_vehicle),
            "vehicle": args.vehicle,
            "sitl_parameters": {
                "home": home, "wind_speed_m_s": args.wind_speed,
                "wind_direction_deg": args.wind_direction % 360,
                "wind_turbulence": args.wind_turbulence,
            },
        },
    }
    sitl = app = None
    handles: list[Any] = []
    interrupted = False
    started = time.monotonic()
    try:
        sitl, handle = _launch(sitl_command, run_dir / "sitl.log", run_dir.resolve()); handles.append(handle)
        prepare, handle = _launch(prepare_command, run_dir / "prepare.log"); handles.append(handle)
        prepare_exit = prepare.wait(timeout=120)
        report["prepare_exit_code"] = prepare_exit
        if prepare_exit != 0:
            raise RuntimeError("SITL preparation failed; see prepare.log and sitl.log")
        app, handle = _launch(app_command, run_dir / "application.log"); handles.append(handle)
        app_started = time.monotonic()
        fault_at = scenario.get("kill_sitl_after") or scenario.get("interrupt_after")
        while app.poll() is None:
            elapsed = time.monotonic() - app_started
            if elapsed > args.timeout:
                if not interrupted:
                    os.killpg(app.pid, signal.SIGINT); interrupted = True
                elif elapsed > args.timeout + 10:
                    _terminate(app)
            if fault_at and elapsed >= fault_at:
                if "kill_sitl_after" in scenario:
                    _terminate(sitl); sitl = None
                else:
                    os.killpg(app.pid, signal.SIGINT); interrupted = True
                fault_at = None
            time.sleep(0.25)
        report["application_exit_code"] = app.returncode
        mission = _latest_mission(mission_dir)
        validation_metrics = _validation_metrics(mission, config)
        loiter = (scenario["expect"] == "success" and sitl is not None
                  and _loiter_confirmed(config.mavlink.connection))
        passed, reasons = _score(args.scenario, scenario["expect"], app.returncode, mission,
                                 loiter, validation_metrics)
        report.update(status="PASS" if passed else "FAIL", pass_reasons=reasons,
                      loiter_confirmed=loiter, operator_signal_sent=interrupted,
                      validation_metrics=validation_metrics, mission_record=mission)
    except KeyboardInterrupt:
        interrupted = True
        report.update(status="INTERRUPTED", pass_reasons=["orchestrator received Ctrl+C"])
    except (RuntimeError, subprocess.TimeoutExpired, OSError) as exc:
        report.update(status="ERROR", pass_reasons=[str(exc)])
    finally:
        _terminate(app); _terminate(sitl)
        for handle in handles:
            handle.close()
        report["duration_seconds"] = round(time.monotonic() - started, 3)
        report["all_children_stopped"] = all(process is None or process.poll() is not None
                                                for process in (app, sitl))
        report["operator_signal_sent"] = interrupted
        (run_dir / "validation_report.json").write_text(
            json.dumps(report, indent=2, default=str), encoding="utf-8")
        print(json.dumps({key: report.get(key) for key in (
            "status", "scenario", "seed", "duration_seconds", "all_children_stopped",
            "loiter_confirmed", "pass_reasons", "run_directory")}, indent=2))
    return 0 if report["status"] == "PASS" else (130 if report["status"] == "INTERRUPTED" else 2)


if __name__ == "__main__":
    raise SystemExit(main())
