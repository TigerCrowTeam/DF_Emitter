# KrakenSDR MAVLink DF Navigation PoC

This Python 3.10+ proof of concept combines existing KrakenSDR direction-of-arrival output with ArduPilot telemetry. It converts antenna-relative bearings to geographic bearing lines, plans cross-bearing observation positions, and commands `LOITER` after reaching the configured uncertainty target. It never arms or disarms the aircraft.

This is research software, not a flight-safety system. Test in simulation first. The operator and ArduPilot remain responsible for arming, flight mode, geofence, terrain/altitude limits, collision avoidance, link loss, and all failsafes.

## Architecture

- `kraken/`: transport-neutral `BearingSource`, normalized Kraken JSON adapter, and measurement validation.
- `mavlink/`: `VehicleController`, telemetry model, waypoint validation, and pymavlink details.
- `df/`: bearing transforms, WGS84/local-ENU conversion, observations, and robust weighted least squares.
- `mission/`: explicit state machine and MAVLink-independent cross-bearing planner.
- `simulation/`: deterministic synthetic transmitter, RF measurements, and offline vehicle.
- `logging_utils/`: JSON/CSV recorder and replay loader.

The estimator forms each bearing as a line in a local east/north frame. Confidence-squared weights are used in least squares; a leave-one-out robust seed and median/MAD threshold reject obvious outliers. The reported radius is an approximate 95% horizontal uncertainty derived from line geometry, confidence-derived angular error, and residuals. It is not a certified containment bound.

Estimator uncertainty also includes reported receiver-position accuracy and heading accuracy.
Pair and leave-one-out consensus candidates reduce the influence of multiple gross outliers.
The result reports RMS cross-track residual, accepted/rejected original observation indices,
best bearing-line crossing angle, weighted geometry condition number, and maximum
leave-one-observation-out position shift. A well-conditioned estimate is refused when that
sensitivity exceeds `df.max_leave_one_out_shift_m` (30 m by default).

The uncertainty radius uses the largest eigenvalue of the local horizontal covariance scaled
by 2.4477, with a floor of twice the weighted RMS residual. It is deliberately approximate:
an undeclared common bearing bias can move every line consistently and is not identifiable
from bearing intersections alone. Known calibration/heading uncertainty must be supplied in
the observations or addressed by a later bias model. Validation against hidden truth remains
required.

## Ubuntu setup

From the directory containing `kraken_df_nav`:

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r kraken_df_nav/requirements.txt
python -m pytest -q kraken_df_nav/tests
```

The default suite skips the several-minute real-SITL integration case. Run that case after
building ArduPilot with:

```bash
RUN_SITL_INTEGRATION=1 python -m pytest -q -m sitl \
  kraken_df_nav/tests/test_sitl_integration.py
```

Run the fully offline simulation:

```bash
python -m kraken_df_nav.main --config kraken_df_nav/config.yaml
```

The included deterministic scenario deliberately collects seven observations from seven viewpoints (six moves), then prints the estimate and uncertainty and writes timestamped JSON/CSV files under `./logs`. Change `simulation.required_observations` to use another count of at least two.

### Kraken digital twin

Advanced simulation effects live under `simulation.digital_twin`. They are disabled by
default, preserving the original Gaussian-noise scenario. A moderate example is provided in
`config.digital_twin.yaml`:

```bash
python -m kraken_df_nav.main --config kraken_df_nav/config.digital_twin.yaml
```

The typed model can combine calibration bias, sample drift, heading error, range-dependent
and time-correlated noise, confidence degradation, intermittent/burst dropouts, delay and
staleness, duplicate/frozen measurements, malformed data, wrong frequency, isolated/burst
outliers, persistent multipath, and movement/attitude penalties. Every generated result adds
a `simulation_measurement` audit event containing the clean bearing, applied effects, final
measurement, receiver pose, and validation-only transmitter truth. Mission components receive
only `BearingMeasurement`; they cannot access the truth fields through the bearing interface.

Use fixed `simulation.random_seed` values for reproducible scenarios. Create a separate YAML
file for each scenario rather than repeatedly editing the base configuration.

### Vehicle and navigation digital twin

Vehicle effects live under `simulation.vehicle_twin` and are disabled by default. The
moderate example adds small pose errors and progressive movement without requiring SITL:

```bash
python -m kraken_df_nav.main --config kraken_df_nav/config.vehicle_twin.yaml
```

The offline controller supports seeded GPS noise/drift, compass bias/noise, movement roll and
pitch, progressive travel, delayed arrival, position-tracking error, waypoint refusal,
simulated telemetry latency and packet loss, telemetry/heartbeat/GPS loss, unexpected modes,
and geofence refusal. It records true and reported poses separately for later scoring.

Mission-level protections also apply to ArduPilot SITL and live-controller modes:
`movement_timeout_seconds`, `max_mission_duration_seconds`, `max_total_distance_m`, and
`accepted_flight_modes`. The application still does not contain an arm or disarm operation.
Actual SITL wind injection and network impairment orchestration belong to the automated SITL
scenario runner in Stage S8; the deterministic controller provides fast coverage before then.

### Measurement gating and synchronization

`MeasurementGate` validates each sample immediately before an observation is accepted. The
default limits are configured under `kraken` and `df`:

```yaml
kraken:
  max_measurement_age_seconds: 2
  max_pose_measurement_skew_seconds: 1
  max_frozen_samples: 20
df:
  max_position_accuracy_m: 10
  max_heading_accuracy_deg: 10
  max_collection_speed_m_s: 0.5
```

The gate rejects non-finite or out-of-range fields, stale/future samples, wrong frequencies,
low confidence, pose skew, inadequate position/heading accuracy, motion, exact duplicates,
and frozen bearing sequences. Mission-state checks additionally require valid current
telemetry, an accepted flight mode, arrival, stabilization, and acceptable roll/pitch.

Every rejection produces a `measurement_rejected` event. When a structured measurement was
received, it is also preserved as an observation with `accepted: false` and a rejection reason.
Missing or malformed source output cannot form an observation but still leaves a rejection
event. Rejected samples are never appended to estimator inputs and cannot directly cause a
waypoint.

### Observation planner and comparison baselines

`ObservationPlanner` evaluates bounded candidates using predicted bearing-line information
gain, desired crossing geometry, travel cost, distance from the current emitter estimate,
minimum viewpoint separation, and unavailable-point exclusions. After an estimate exists it
also evaluates an orbit of candidates around that estimate, preventing the search from
drifting away while uncertainty remains high. A configurable minimum information gain stops
unproductive movement. The mission may try up to `planner_max_attempts` distinct candidates
when a vehicle refuses a waypoint, then aborts safely.

Run the deterministic comparison against fixed-grid and seeded-random baselines:

```bash
python -m kraken_df_nav.planner_benchmark \
  --config kraken_df_nav/config.yaml --seeds 20 \
  --output ./logs/stage5_planner_benchmark.json
```

The report includes completion and false-success rates, localization error, distance traveled,
observation/waypoint counts, mission ticks, and best crossing angle for every planner and seed.

### Deterministic scenario catalog

`simulation_scenarios.yaml` defines the named S6 scenarios, classifications, fixed seeds,
configuration overrides, scripted faults, and expected outcomes. Run the entire catalog with:

```bash
python -m kraken_df_nav.scenarios \
  --catalog kraken_df_nav/simulation_scenarios.yaml \
  --config kraken_df_nav/config.yaml \
  --output ./logs/stage6_scenario_report.json
```

The runner embeds each fully resolved configuration and complete event stream in the report.
It fails when required success is missed, a safety scenario does not terminate, a completed
result violates the accuracy/uncertainty/geometry gates, or any waypoint follows a safety
stop. Runs that exhaust their deterministic tick budget now transition to `ABORTED` rather
than remaining in an ambiguous active state. Sustained invalid Kraken data similarly reaches
a terminal safety stop after `kraken.max_consecutive_rejections`.

Run the repeatable Stage 3 Monte Carlo stress matrix (20 seeds at five bearing-noise levels):

```bash
python -m kraken_df_nav.stress --config kraken_df_nav/config.yaml
```

This writes full per-trial results to `./logs/stage3_stress_report.json` and prints a compact summary.

Run the frozen final S7 Monte Carlo campaign (500 trials) with:

```bash
python -m kraken_df_nav.monte_carlo \
  --trials 500 --master-seed 20260828 \
  --output ./logs/stage7_monte_carlo_report.json
```

The final runner preallocates trials as 60% nominal, 24% degraded, and 16% required safe
failures before drawing randomized configurations. It varies emitter/start geometry, initial
heading, planner/layout, bearing noise, bias and drift, heading/GPS errors, loss, delay,
staleness, multipath, confidence calibration, and boundaries. The report includes overall
and per-class tail metrics plus the full seed and resolved configuration for every trial.
Exit status zero means every frozen S0 campaign target passed; status 2 means the evidence was
saved but one or more targets failed.

### One-command ArduPilot SITL demonstration

Stage S8 provides a process-owning runner for the complete SITL demonstration. It requires
an ArduPilot source/build installation whose `sim_vehicle.py` is on `PATH`. Check that
dependency without starting anything:

```bash
./run_simulation_poc.sh --scenario nominal --seed 42 --preflight-only
```

Then run the nominal closed-loop demonstration:

```bash
./run_simulation_poc.sh --scenario nominal --seed 42
```

The runner starts ArduCopter with mild configurable wind, uses the separate localhost-only
`sitl_prepare` harness to establish GUIDED/armed/takeoff state, and only then starts the DF
application. The DF application itself still has no arm or disarm operation. Every run gets
an isolated directory under `logs/sitl/` with resolved configuration, SITL environment
parameters, SITL/preparation/application logs, the mission JSON/CSV, and an automatic
`validation_report.json`.

Run the required safety demonstrations with:

```bash
./run_simulation_poc.sh --scenario kraken_loss --seed 42
./run_simulation_poc.sh --scenario mavlink_loss --seed 42
./run_simulation_poc.sh --scenario poor_geometry --seed 42
./run_simulation_poc.sh --scenario multipath --seed 42
./run_simulation_poc.sh --scenario operator_interrupt --seed 42
```

The shell exit status is zero only when the scenario meets its declared outcome. Nominal
requires mission completion and a separately observed LOITER heartbeat. Safety scenarios
require an auditable `ABORTED`/`PAUSED` state, reject unexpected completion, and fail if any
waypoint appears after a safety stop. The runner terminates its child process groups after
success, failure, timeout, or Ctrl+C. Use `--sim-vehicle /path/to/sim_vehicle.py` if it is not
on `PATH`; use `--wind-speed`, `--wind-direction`, and `--wind-turbulence` to change the
recorded environmental disturbance.

The validated S8 campaign used ArduPilot revision
`af2a1bafc8ab993418e6714cbc2674dcc178d11a`. To reproduce that exact environment when
`sim_vehicle.py` is not on `PATH`, check out and build that revision, then pass its path:

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git tools/ardupilot
git -C tools/ardupilot checkout af2a1bafc8ab993418e6714cbc2674dcc178d11a
git -C tools/ardupilot submodule update --init --recursive
(cd tools/ardupilot && ../../.venv/bin/python ./waf configure --board sitl)
(cd tools/ardupilot && ../../.venv/bin/python ./waf copter)
./run_simulation_poc.sh --scenario nominal --seed 42 \
  --sim-vehicle tools/ardupilot/Tools/autotest/sim_vehicle.py
```

## Modes

### SIMULATION

The default uses a simulated vehicle and transmitter, so neither Kraken nor SITL is needed. Set `simulation.use_sitl: true` to generate synthetic bearings from the real position reported by ArduPilot SITL. Start SITL separately, place it in an appropriate already-armed test state if movement is desired, and leave the default connection as `udp:127.0.0.1:14550`.

The program will not arm it. ArduPilot must be in a mode that accepts `SET_POSITION_TARGET_GLOBAL_INT` (normally GUIDED). SITL UDP direction/port depends on how SITL was launched; adjust `mavlink.connection` if necessary.

### KRAKEN_SITL

Set `mode: KRAKEN_SITL`, configure the Kraken endpoint, and use the SITL UDP connection. Real bearings are combined with the SITL-reported pose.

### LIVE

Set `mode: LIVE`. A serial Cube Orange requires only configuration, for example:

```yaml
mavlink:
  connection: "/dev/serial/by-id/usb-CubePilot_CubeOrange_..."
```

If the link needs an explicit baud rate, pymavlink connection strings vary by deployment; a common URI form is `serial:/dev/ttyACM0:115200`. Prefer the stable `/dev/serial/by-id/` device path and verify the form against the installed pymavlink version. Ensure the user belongs to `dialout` and reconnect after adding group membership.

### REPLAY

Set the recorded JSON or CSV path and run normally:

```yaml
mode: REPLAY
replay:
  path: "./logs/mission_20260825T210000Z.json"
```

Replay runs accepted observations through the same estimator but issues no vehicle commands.

Validate one or more recorded field trials against surveyed transmitter coordinates:

```bash
python -m kraken_df_nav.main --validate ./logs \
  --truth-lat 37.0008000 --truth-lon -121.9992000 \
  --report-output ./logs/validation_report.json
```

The validator accepts JSON/CSV files or directories. When matching JSON and CSV mission
logs exist, it evaluates the JSON once. Its report includes per-trial actual error,
reported uncertainty, target/containment results, residuals, rejected observations, and
best bearing-line crossing angle, plus aggregate pass and containment rates. Use surveyed
ground truth and a meaningful number of independent trials before interpreting those rates.

## Kraken integration

`KrakenBearingSource` accepts either the normalized HTTP JSON adapter below or the
official KrakenSDR V2 legacy bearing endpoint at
`http://HOST:8081/DOA_value.html`:

```json
{
  "bearing_deg": 42.0,
  "confidence": 0.91,
  "rssi_dbm": -61.2,
  "frequency_hz": 915000000,
  "timestamp": 1787691600.0
}
```

`bearing_deg` is clockwise from the antenna array's reference direction. The V2 endpoint
must have exactly one active DoA decimator; the client refuses multiple rows rather than
silently choosing the wrong frequency. Its `power_db` signal metric is retained in the
PoC's historical `rssi_dbm` field. This project does not implement or replace Kraken's
MUSIC/DoA processing. Configure tuning in the existing Kraken stack; the `rf` values here
validate incoming measurements and document the intended 915 MHz setup.

Collect a stationary bearing-stability report without MAVLink:

```bash
python -m kraken_df_nav.kraken.probe \
  --endpoint http://127.0.0.1:8081/DOA_value.html \
  --count 120 --interval 0.25 \
  --output ./logs/kraken_stationary_probe.json
```

### Ground trial (no MAVLink)

Copy `ground_trial_points.csv` and replace every placeholder coordinate and heading with
surveyed values. Use seven well-separated points that create different bearing-line angles.
The heading is the clockwise true-north heading of the array's physical reference mark.
Then collect live samples at each point:

```bash
python -m kraken_df_nav.kraken.ground_trial \
  --points ./ground_trial_points.csv --frequency-hz 915000000 \
  --samples-per-point 120 --interval 0.25 \
  --output ./logs/kraken_ground_trial.json
```

The collector pauses before each point, never opens MAVLink, rejects wrong-frequency and
low-confidence samples, preserves rejection details, and writes replay-compatible JSON.
Replace the example frequency with the actual configured transmitter frequency. Replay or
validate the result using the commands above.

## Important configuration

- `kraken.min_confidence` and `antenna_heading_offset_deg`: sample gate and clockwise antenna-to-airframe heading offset.
- `df.max_roll_deg`, `max_pitch_deg`: attitude gates; `target_error_radius_m` defaults to a 10 m approximate 95% uncertainty radius.
- `navigation.desired_cross_bearing_deg`: default 60° oblique move rather than flying directly at the source.
- `observation_spacing_m`, `max_movement_distance_m`, `max_search_radius_m`: planner command bounds.
- `arrival_tolerance_m`, `stabilization_seconds`: arrival and dwell gates.
- `safety.invalid_data_action`: `ABORT` or `PAUSE`. Neither state generates further waypoints.
- `navigation.command_altitude_m`: `null` preserves current relative altitude; otherwise sets the commanded relative altitude.

Every observation—accepted or rejected—is logged with its rejection reason. Waypoints, state transitions, safety stops, and estimates are JSON events. A completed mission also writes a top-level `final_report` containing the estimated transmitter coordinates, uncertainty radius, residual, status, and observation counts. Ctrl+C closes the source/controller and flushes logs.

Example terminal result:

```text
Transmitter estimate: 37.0008006, -121.9992139
Uncertainty radius: 7.7 m
```

## Known limitations

- The Kraken HTTP schema is an adapter contract that must be mapped to the deployed Kraken output.
- Small-area ENU calculations are intended for the configured sub-kilometre search radius, not long-range/global navigation.
- Uncertainty is an engineering approximation and can be optimistic under multipath, biased calibration, platform reflections, time-correlated errors, or two-line geometry.
- The planner is intentionally simple and does not model wind, terrain, obstacles, battery, turn radius, or complete geofences.
- Routed GUIDED waypoints use `MAV_CMD_DO_REPOSITION`; the controller requires an accepted or
  in-progress ArduPilot acknowledgement and records the command result and target identity with
  every waypoint. ArduPilot remains the final authority and rejected or timed-out commands stop
  the mission safely.
- The PoC uses synchronous polling and assumes one vehicle and one bearing source.
