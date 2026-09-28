# MAVProxy Routed PoC Reproducibility

All results in this document are **simulation-only**. They do not establish real KrakenSDR or
flight accuracy.

## Reproduce the visual result

```bash
./demo/setup_mavproxy_gui.sh
./demo/run_mavproxy_demo.sh --preflight-only
./demo/run_mavproxy_demo.sh --scenario nominal --seed 42 --timeout 300
```

The dramatic presentation variant is `./demo/run_mavproxy_demo.sh`: 60 m pre-detection straight
transit, explicit detection announcement, DF maneuvers, and a final red-antenna estimate marker.

For CI or a machine without a display, add `--headless`. The launcher prints the isolated run
directory. That directory contains the exact resolved YAML, commands, source manifest, ArduPilot
revision, parameters, MAVProxy tlog and log, application/prepare/SITL logs, mission JSON/CSV,
process exit codes, validation metrics, and cleanup result.

## Evidence audit

```bash
.venv/bin/python -m kraken_df_nav.mavproxy_evidence_audit RUN_DIRECTORY \
  --output logs/mavproxy_m8_evidence_audit.json
```

The audit fails if required files, identities, command ACK evidence, process results, resolved
configuration, or cleanup confirmation are absent. It derives per-message counts and mean rates
and the maximum autopilot-heartbeat gap directly from `mav.tlog`.

## Reference result

- Routed run: `logs/mavproxy/sitl/nominal_seed42_20260829T201327Z/`
- Audit: `logs/mavproxy_m8_evidence_audit.json`
- MAVProxy: 1.8.74
- ArduPilot revision: `af2a1bafc8ab993418e6714cbc2674dcc178d11a`
- Application/MAVProxy/target identities: 255:191 / 254:190 / 1:1
- Position telemetry: 555 messages over 110.812 seconds (4.999 Hz)
- Maximum autopilot heartbeat gap: 1.010 seconds
- Result: COMPLETE, LOITER confirmed, 0.242 m simulated truth error

## Direct versus routed baseline

| Metric | Direct SITL | Routed MAVProxy SITL |
|---|---:|---:|
| Simulated truth error | 0.253 m | 0.242 m |
| Uncertainty radius | 3.064 m | 3.439 m |
| RMS residual | 0.618 m | 0.346 m |
| Best crossing angle | 81.805° | 81.060° |

Both pass the unchanged 10 m error target and uncertainty/residual/geometry gates. The routed
visual profile uses five viewpoints at 50 m spacing; the earlier direct baseline used seven.

## Shutdown-code note

MAVProxy 1.8.74 may return code 1 while unloading modules after requested termination. The
launcher separately verifies that its process exited and that no owned child remains. Application,
preparation, readiness, and mission status remain independently recorded.
