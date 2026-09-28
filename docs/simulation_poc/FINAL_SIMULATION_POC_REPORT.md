# Simulation-Only Emitter Direction-Finding PoC — Final Report

**Evidence date:** August 29, 2026  
**PoC verdict:** DEMONSTRATED WITH A KNOWN CALIBRATION EXCEPTION  
**Scope warning:** Every performance number in this report is simulated. No accuracy figure may be treated as measured KrakenSDR or flight performance.

## Executive result

The software demonstrated the intended closed loop: accept direction-of-arrival measurements, combine them with vehicle pose, reject invalid data and geometry, estimate an emitter location, plan bounded observation moves, and send position targets to ArduPilot SITL. All six final SITL demonstrations passed, including five safety/fault cases. The S10 evidence audit passed with zero missing evidence fields.

The frozen 500-trial campaign did **not** fully pass its S0 target set. Nominal trials achieved 100% within 10 m and the overall false-success rate was 1.20%, but nominal uncertainty containment was 99.67% versus the declared acceptable range of 80–99%. This is over-conservative uncertainty calibration. It does not invalidate the workflow demonstration, but it prevents claiming that every frozen quantitative requirement passed.

## Simulation results

| Metric | Result |
|---|---:|
| Randomized trials | 500 |
| Nominal completion | 100.00% |
| Nominal localization error, median / P95 / worst | 1.14 / 3.78 / 6.64 m |
| Overall completion | 65.00% |
| Overall false success | 1.20% |
| Unsafe commands after stop | 0 |
| Deterministic scenarios | 20/20 passed |
| Final ArduPilot SITL scenarios | 6/6 passed |
| S10 evidence audit | PASS |
| Fast regression suite | 111 passed, 1 optional SITL test skipped |

## Figures

- [Accuracy percentiles](accuracy_percentiles.svg)
- [Uncertainty containment](uncertainty_containment.svg)
- [Completion rates](completion_rates.svg)
- [Failure reasons](failure_reasons.svg)
- [Planner comparison](planner_comparison.svg)

## Evidence and reproducibility

- Source archive identifier: `069e3974fc2738fec05303373a7c492b39c4f6b53fa8d0c51779fe246895915a`
- ArduPilot revision: `af2a1bafc8ab993418e6714cbc2674dcc178d11a`
- Platform: `Linux-7.0.0-30-generic-x86_64-with-glibc2.43`; Python `3.13.13`
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
