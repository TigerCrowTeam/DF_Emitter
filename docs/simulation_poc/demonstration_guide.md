# Installation and Demonstration Guide

All demonstrations below use simulated bearings. They do not measure real RF accuracy.

## One-command showcase

For the MAVProxy map/console routed showcase:

```bash
./demo/setup_mavproxy_gui.sh       # one-time setup
./demo/run_mavproxy_demo.sh --scenario nominal --seed 42 --timeout 300
```

For the more visual discovery narrative, use:

```bash
./demo/run_mavproxy_demo.sh
```

This version flies straight east for 60 m before the DF application starts, announces emitter
detection, executes the cross-bearing search, and plots a red antenna at the final estimated
emitter coordinates. The marker is derived from the estimator result, not simulator truth. The
map and console remain open after success; press `Ctrl+C` in the launch terminal when finished.

The second command owns SITL, MAVProxy, takeoff preparation, the DF application, validation, and
cleanup. It opens the MAVProxy map and console. Use `--headless` for automation. All RF bearings,
vehicle motion, and truth coordinates in this run are simulation-only.

For a complete presentation run:

```bash
./run_poc_showcase.sh
```

This performs preflight, the nominal closed-loop mission, a Kraken-loss safety demonstration,
the evidence audit, and the fast tests before printing the reports to open. Allow approximately
five minutes. For the shortest live demonstration, run only nominal plus verification:

```bash
./run_poc_showcase.sh --nominal-only
```

## Installation

```bash
python3 -m venv .venv
.venv/bin/pip install -r kraken_df_nav/requirements.txt
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git tools/ardupilot
git -C tools/ardupilot checkout af2a1bafc8ab993418e6714cbc2674dcc178d11a
git -C tools/ardupilot submodule update --init --recursive
(cd tools/ardupilot && ../../.venv/bin/python ./waf configure --board sitl)
(cd tools/ardupilot && ../../.venv/bin/python ./waf copter)
```

Exact Python dependency versions are recorded inside every S10 report and in `kraken_df_nav/requirements.txt`. Run the preflight check first:

```bash
./run_simulation_poc.sh --scenario nominal --seed 42 --sim-vehicle tools/ardupilot/Tools/autotest/sim_vehicle.py --preflight-only
```

For MAVProxy, run `./demo/setup_mavproxy_gui.sh`, then verify the complete GUI and routing environment:

```bash
./demo/run_mavproxy_demo.sh --preflight-only
```

After a routed run, generate its reproducibility audit with:

```bash
.venv/bin/python -m kraken_df_nav.mavproxy_evidence_audit \
  logs/mavproxy/sitl/nominal_seed42_TIMESTAMP \
  --output logs/mavproxy_m8_evidence_audit.json
```

## Presenter script

1. State: “This demonstration validates the software loop in simulation; it does not claim real RF or flight accuracy.”
2. Show `docs/simulation_poc/FINAL_SIMULATION_POC_REPORT.md` and the architecture diagram.
3. Run the nominal case:

   ```bash
   ./run_simulation_poc.sh --scenario nominal --seed 42 --sim-vehicle tools/ardupilot/Tools/autotest/sim_vehicle.py --timeout 300
   ```

4. Point out the `PASS`, `COMPLETE`, LOITER confirmation, estimate/error, and new isolated run directory.
5. Run one or all safety cases:

   ```bash
   for scenario in kraken_loss mavlink_loss poor_geometry multipath operator_interrupt; do
     ./run_simulation_poc.sh --scenario "$scenario" --seed 42 --sim-vehicle tools/ardupilot/Tools/autotest/sim_vehicle.py --timeout 240 || exit
   done
   ```

6. Explain that each expected abort is a passing safety result and show zero waypoint commands after stop.
7. Run the evidence audit and fast tests:

   ```bash
   .venv/bin/python -m kraken_df_nav.evidence_audit --monte-carlo logs/stage7_monte_carlo_report.json --sitl-root logs/sitl --output logs/stage10_evidence_audit.json
   .venv/bin/pytest -q kraken_df_nav/tests
   ```

8. Close with the known calibration exception and the real-hardware transition gates.
