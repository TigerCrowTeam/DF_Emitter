"""Stationary Kraken bearing probe that does not connect to MAVLink."""
from __future__ import annotations

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
from statistics import mean
import time

from .client import KrakenBearingSource


def circular_statistics(bearings: list[float]) -> tuple[float, float]:
    radians = [math.radians(value) for value in bearings]
    x, y = mean(math.cos(value) for value in radians), mean(math.sin(value) for value in radians)
    direction = math.degrees(math.atan2(y, x)) % 360
    resultant = min(1.0, math.hypot(x, y))
    deviation = math.degrees(math.sqrt(max(0.0, -2 * math.log(max(resultant, 1e-12)))))
    return direction, deviation


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="http://127.0.0.1:8081/DOA_value.html")
    parser.add_argument("--count", type=int, default=30)
    parser.add_argument("--interval", type=float, default=0.25)
    parser.add_argument("--output", default="./logs/kraken_stationary_probe.json")
    args = parser.parse_args()
    if args.count < 2 or args.interval < 0:
        parser.error("count must be at least 2 and interval must be non-negative")

    source = KrakenBearingSource(args.endpoint)
    samples, failures = [], []
    for index in range(args.count):
        try:
            measurement = source.get_measurement()
            if measurement is not None:
                samples.append(asdict(measurement))
        except Exception as exc:
            failures.append({"sample": index, "error": str(exc)})
        if index + 1 < args.count:
            time.sleep(args.interval)
    bearings = [sample["relative_bearing_deg"] for sample in samples]
    circular_mean, circular_std = circular_statistics(bearings) if bearings else (None, None)
    report = {
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "endpoint": args.endpoint,
        "requested_samples": args.count,
        "received_samples": len(samples),
        "failure_count": len(failures),
        "circular_mean_bearing_deg": circular_mean,
        "circular_stddev_deg": circular_std,
        "mean_confidence": mean(sample["confidence"] for sample in samples) if samples else None,
        "frequencies_hz": sorted({sample["frequency_hz"] for sample in samples}),
        "samples": samples,
        "failures": failures,
    }
    rendered = json.dumps(report, indent=2)
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(rendered + "\n", encoding="utf-8")
    print(rendered)
    return 0 if len(samples) == args.count else 2


if __name__ == "__main__":
    raise SystemExit(main())
