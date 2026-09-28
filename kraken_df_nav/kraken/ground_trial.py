"""Collect replay-compatible KrakenSDR observations from surveyed ground points."""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from ..df.observation import Observation
from ..df.transforms import relative_to_absolute
from .client import KrakenBearingSource
from .filtering import validate_measurement


def circular_mean(values: list[float], weights: list[float]) -> float:
    radians = [math.radians(value) for value in values]
    x = sum(weight * math.cos(value) for value, weight in zip(radians, weights))
    y = sum(weight * math.sin(value) for value, weight in zip(radians, weights))
    return math.degrees(math.atan2(y, x)) % 360.0


def load_points(path: str) -> list[dict[str, object]]:
    with Path(path).open(newline="", encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    required = {"name", "latitude", "longitude", "array_heading_deg"}
    if not rows or not required.issubset(rows[0]):
        raise ValueError(f"points CSV must contain: {', '.join(sorted(required))}")
    points = []
    for row in rows:
        latitude, longitude = float(row["latitude"]), float(row["longitude"])
        heading = float(row["array_heading_deg"])
        if not -90 <= latitude <= 90 or not -180 <= longitude <= 180:
            raise ValueError(f"invalid coordinates for point {row['name']!r}")
        if not 0 <= heading < 360:
            raise ValueError(f"invalid array heading for point {row['name']!r}")
        points.append({"name": row["name"], "latitude": latitude, "longitude": longitude,
                       "array_heading_deg": heading,
                       "altitude_m": float(row.get("altitude_m") or 0.0)})
    return points


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--points", required=True, help="surveyed observation-points CSV")
    parser.add_argument("--frequency-hz", required=True, type=int)
    parser.add_argument("--endpoint", default="http://127.0.0.1:8081/DOA_value.html")
    parser.add_argument("--samples-per-point", type=int, default=120)
    parser.add_argument("--interval", type=float, default=0.25)
    parser.add_argument("--min-confidence", type=float, default=0.30)
    parser.add_argument("--frequency-tolerance-hz", type=int, default=1000)
    parser.add_argument("--heading-offset-deg", type=float, default=0.0)
    parser.add_argument("--output", default="./logs/kraken_ground_trial.json")
    parser.add_argument("--no-prompt", action="store_true", help="do not pause before points (bench testing only)")
    args = parser.parse_args()
    if args.samples_per_point < 2 or args.interval < 0:
        parser.error("samples-per-point must be at least 2 and interval must be non-negative")
    if not 0 <= args.min_confidence <= 1 or args.frequency_tolerance_hz < 0:
        parser.error("invalid confidence or frequency tolerance")

    points, source = load_points(args.points), KrakenBearingSource(args.endpoint)
    observations, reports = [], []
    try:
        for point in points:
            if not args.no_prompt:
                input(f"Move to {point['name']} and align reference to {point['array_heading_deg']:.1f} deg; press Enter when stationary: ")
            accepted, rejected = [], []
            for sample_index in range(args.samples_per_point):
                try:
                    measurement = source.get_measurement()
                    if measurement is None:
                        rejected.append({"sample": sample_index, "reason": "no measurement"})
                    else:
                        reason = validate_measurement(measurement, args.frequency_hz,
                                                      args.min_confidence, args.frequency_tolerance_hz)
                        if reason:
                            rejected.append({"sample": sample_index, "reason": reason,
                                             "measurement": asdict(measurement)})
                        else:
                            accepted.append(measurement)
                except Exception as exc:
                    rejected.append({"sample": sample_index, "reason": str(exc)})
                if sample_index + 1 < args.samples_per_point:
                    time.sleep(args.interval)
            report = {**point, "requested_samples": args.samples_per_point,
                      "accepted_samples": len(accepted), "rejected_samples": rejected}
            if accepted:
                weights = [max(item.confidence, 1e-6) for item in accepted]
                relative = circular_mean([item.relative_bearing_deg for item in accepted], weights)
                absolute = relative_to_absolute(relative, float(point["array_heading_deg"]),
                                                args.heading_offset_deg)
                observation = Observation.now(latitude=float(point["latitude"]),
                    longitude=float(point["longitude"]), altitude_m=float(point["altitude_m"]),
                    vehicle_heading_deg=float(point["array_heading_deg"]),
                    relative_bearing_deg=relative, absolute_bearing_deg=absolute,
                    rssi_dbm=sum(item.rssi_dbm for item in accepted) / len(accepted),
                    confidence=sum(item.confidence for item in accepted) / len(accepted))
                observations.append(observation)
                report["observation"] = observation.to_dict()
            reports.append(report)
            print(f"{point['name']}: {len(accepted)}/{args.samples_per_point} samples accepted")
    finally:
        source.close()

    payload = {"generated_at": datetime.now(timezone.utc).isoformat(), "kind": "ground_trial",
               "frequency_hz": args.frequency_hz, "heading_offset_deg": args.heading_offset_deg,
               "observations": [item.to_dict() for item in observations], "point_reports": reports}
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(f"Saved {len(observations)}/{len(points)} observations to {target}")
    return 0 if len(observations) == len(points) and len(observations) >= 2 else 2


if __name__ == "__main__":
    raise SystemExit(main())
