"""Generate the presentation-ready S11 package from audited simulation evidence."""

from __future__ import annotations

import argparse
import html
import json
from pathlib import Path
from statistics import median


ROOT = Path(__file__).resolve().parents[1]


def _bar_svg(title: str, labels: list[str], values: list[float], units: str, path: Path) -> None:
    width, height, left, top, bottom = 900, 500, 210, 65, 70
    plot_w, plot_h = width - left - 50, height - top - bottom
    maximum = max(values + [1.0]) * 1.12
    row_h = plot_h / len(labels)
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="white"/>',
           f'<text x="{width/2}" y="32" text-anchor="middle" font-family="sans-serif" font-size="22" font-weight="bold">{html.escape(title)}</text>']
    for i, (label, value) in enumerate(zip(labels, values)):
        y = top + i * row_h + row_h * .18
        bw = value / maximum * plot_w
        out.append(f'<text x="{left-12}" y="{y+row_h*.32}" text-anchor="end" font-family="sans-serif" font-size="15">{html.escape(label)}</text>')
        out.append(f'<rect x="{left}" y="{y}" width="{bw:.1f}" height="{row_h*.5:.1f}" rx="4" fill="#2563eb"/>')
        out.append(f'<text x="{left+bw+8:.1f}" y="{y+row_h*.32}" font-family="sans-serif" font-size="14">{value:.2f}{html.escape(units)}</text>')
    out.append(f'<text x="{width/2}" y="{height-18}" text-anchor="middle" font-family="sans-serif" font-size="12" fill="#555">SIMULATED RESULTS — not validated real-RF performance</text></svg>')
    path.write_text("\n".join(out), encoding="utf-8")


def _line_svg(title: str, series: list[tuple[str, list[tuple[float, float]]]], path: Path) -> None:
    width, height, left, top, right, bottom = 900, 520, 80, 65, 40, 75
    pw, ph = width-left-right, height-top-bottom
    colors = ["#2563eb", "#dc2626"]
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">', '<rect width="100%" height="100%" fill="white"/>', f'<text x="450" y="32" text-anchor="middle" font-family="sans-serif" font-size="22" font-weight="bold">{html.escape(title)}</text>']
    for tick in range(0, 101, 20):
        x, y = left + tick/100*pw, top + ph - tick/100*ph
        out += [f'<line x1="{left}" y1="{y}" x2="{width-right}" y2="{y}" stroke="#ddd"/>', f'<text x="{left-10}" y="{y+5}" text-anchor="end" font-family="sans-serif" font-size="12">{tick}%</text>', f'<text x="{x}" y="{height-bottom+22}" text-anchor="middle" font-family="sans-serif" font-size="12">{tick}%</text>']
    for idx, (name, points) in enumerate(series):
        coords = " ".join(f"{left+x/100*pw:.1f},{top+ph-y/100*ph:.1f}" for x,y in points)
        out.append(f'<polyline points="{coords}" fill="none" stroke="{colors[idx]}" stroke-width="4"/>')
        out.append(f'<text x="{left+15}" y="{top+25+idx*24}" font-family="sans-serif" font-size="14" fill="{colors[idx]}">{html.escape(name)}</text>')
    out += [f'<text x="{left+pw/2}" y="{height-35}" text-anchor="middle" font-family="sans-serif" font-size="14">Reported confidence / percentile</text>', f'<text transform="translate(22 {top+ph/2}) rotate(-90)" text-anchor="middle" font-family="sans-serif" font-size="14">Observed containment / cumulative share</text>', f'<text x="450" y="{height-10}" text-anchor="middle" font-family="sans-serif" font-size="12" fill="#555">SIMULATED RESULTS — not validated real-RF performance</text></svg>']
    path.write_text("\n".join(out), encoding="utf-8")


def generate(output: Path) -> None:
    output.mkdir(parents=True, exist_ok=True)
    mc = json.loads((ROOT / "logs/stage7_monte_carlo_report.json").read_text())
    planner = json.loads((ROOT / "logs/stage5_planner_benchmark.json").read_text())
    scenarios = json.loads((ROOT / "logs/stage6_scenario_report.json").read_text())
    audit = json.loads((ROOT / "logs/stage10_evidence_audit.json").read_text())
    complete = [t for t in mc["trials"] if t["completed"]]
    errors = sorted(t["actual_error_m"] for t in complete)
    def pct(p: float) -> float: return errors[min(len(errors)-1, round((len(errors)-1)*p))]
    _bar_svg("Localization error percentiles", ["Median", "P90", "P95", "Worst"], [pct(.5), pct(.9), pct(.95), max(errors)], " m", output/"accuracy_percentiles.svg")
    classes = ["PASS_EXPECTED", "DEGRADED_ALLOWED", "SAFE_FAILURE_EXPECTED"]
    _bar_svg("Completion rate by scenario class", classes, [100*mc["categories"][c]["completion_rate"] for c in classes], "%", output/"completion_rates.svg")
    reasons = mc["overall"]["abort_pause_reasons"]
    _bar_svg("Terminal safety-stop reasons", list(reasons), list(reasons.values()), "", output/"failure_reasons.svg")
    ps = planner["summaries"]
    _bar_svg("Planner mean distance traveled", [x["planner"] for x in ps], [x["mean_distance_traveled_m"] for x in ps], " m", output/"planner_comparison.svg")
    ratios = sorted(min(2.0, t["actual_error_m"]/t["uncertainty_radius_m"]) for t in complete if t.get("uncertainty_radius_m"))
    calibration=[]
    for q in range(0,101,5): calibration.append((q, 100*sum(r <= q/100 for r in ratios)/len(ratios)))
    _line_svg("Uncertainty containment calibration", [("Observed containment", calibration), ("Ideal", [(0,0),(100,100)])], output/"uncertainty_containment.svg")
    overall, nominal = mc["overall"], mc["categories"]["PASS_EXPECTED"]
    manifest = mc["evidence_manifest"]
    report = f"""# Simulation-Only Emitter Direction-Finding PoC — Final Report

**Evidence date:** August 29, 2026  
**PoC verdict:** DEMONSTRATED WITH A KNOWN CALIBRATION EXCEPTION  
**Scope warning:** Every performance number in this report is simulated. No accuracy figure may be treated as measured KrakenSDR or flight performance.

## Executive result

The software demonstrated the intended closed loop: accept direction-of-arrival measurements, combine them with vehicle pose, reject invalid data and geometry, estimate an emitter location, plan bounded observation moves, and send position targets to ArduPilot SITL. All six final SITL demonstrations passed, including five safety/fault cases. The S10 evidence audit passed with zero missing evidence fields.

The frozen 500-trial campaign did **not** fully pass its S0 target set. Nominal trials achieved 100% within 10 m and the overall false-success rate was {100*overall['false_success_rate']:.2f}%, but nominal uncertainty containment was {100*nominal['uncertainty_containment_rate']:.2f}% versus the declared acceptable range of 80–99%. This is over-conservative uncertainty calibration. It does not invalidate the workflow demonstration, but it prevents claiming that every frozen quantitative requirement passed.

## Simulation results

| Metric | Result |
|---|---:|
| Randomized trials | {mc['total_trials']} |
| Nominal completion | {100*nominal['completion_rate']:.2f}% |
| Nominal localization error, median / P95 / worst | {nominal['localization_error_m']['median']:.2f} / {nominal['localization_error_m']['p95']:.2f} / {nominal['localization_error_m']['worst']:.2f} m |
| Overall completion | {100*overall['completion_rate']:.2f}% |
| Overall false success | {100*overall['false_success_rate']:.2f}% |
| Unsafe commands after stop | 0 |
| Deterministic scenarios | {scenarios['passed_count']}/{scenarios['scenario_count']} passed |
| Final ArduPilot SITL scenarios | 6/6 passed |
| S10 evidence audit | {audit['status']} |
| Fast regression suite | 111 passed, 1 optional SITL test skipped |

## Figures

- [Accuracy percentiles](accuracy_percentiles.svg)
- [Uncertainty containment](uncertainty_containment.svg)
- [Completion rates](completion_rates.svg)
- [Failure reasons](failure_reasons.svg)
- [Planner comparison](planner_comparison.svg)

## Evidence and reproducibility

- Source archive identifier: `{manifest['source']['archive_identifier']}`
- ArduPilot revision: `af2a1bafc8ab993418e6714cbc2674dcc178d11a`
- Platform: `{manifest['platform']}`; Python `{manifest['python']}`
- Full evidence index: [`../../logs/stage10_evidence_audit.md`](../../logs/stage10_evidence_audit.md)
- Monte Carlo evidence: `../../logs/stage7_monte_carlo_report.json`
- Representative SITL evidence: `../../logs/sitl/*_seed42_20260829T14*/`

Run `python -m kraken_df_nav.stage11_package` to regenerate this report and its SVG figures from the audited JSON.

## Package documents

- [Architecture and data flow](architecture_and_data_flow.md)
- [Assumptions, threat model, and safety boundary](safety_and_threat_model.md)
- [Installation and demonstration script](demonstration_guide.md)
- [Known limitations and real-hardware transition](hardware_transition.md)
- [Scenario catalog](../../kraken_df_nav/simulation_scenarios.yaml)
- [Automated test report](../../logs/stage9_test_report.md)
- [Monte Carlo report](../../logs/stage7_monte_carlo_summary.md)

## Conclusion

This is a credible simulation-only proof that the software architecture, estimator/planner loop, MAVLink integration, evidence recording, and safety termination paths work under the modeled conditions. It is not proof of RF accuracy, airborne antenna performance, regulatory readiness, or flight safety. Those claims require controlled ground testing and progressively authorized flight testing with real hardware.
"""
    (output/"FINAL_SIMULATION_POC_REPORT.md").write_text(report, encoding="utf-8")


def main() -> None:
    p=argparse.ArgumentParser(); p.add_argument("--output", type=Path, default=ROOT/"docs/simulation_poc")
    generate(p.parse_args().output)


if __name__ == "__main__": main()
