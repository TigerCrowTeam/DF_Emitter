# KrakenSDR Emitter Direction-Finding Navigation

## Project overview

This repository is a Python 3.10+ proof of concept (PoC) for locating a radio-frequency emitter with a KrakenSDR-equipped vehicle and an ArduPilot-controlled aircraft. It combines:

- direction-of-arrival (DoA) measurements from KrakenSDR;
- aircraft position, heading, attitude, speed, and flight-mode telemetry from ArduPilot over MAVLink;
- cross-bearing geometry and robust weighted least-squares estimation;
- an observation planner that requests new viewpoints when the current estimate is not good enough; and
- JSON/CSV evidence logs for replay and validation.

The application does **not** perform KrakenSDR signal processing. Kraken's own software produces the bearing. This application consumes that result, converts the antenna-relative bearing into a geographic bearing line, combines bearing lines from multiple locations, and estimates the emitter position.

This is research software, not a certified navigation, detect-and-avoid, or flight-safety system. Simulation results are not evidence of real RF accuracy or safe flight performance.

## What the application can do

The application supports four modes:

| Mode | Bearing source | Vehicle source | Commands a real aircraft? |
|---|---|---|---|
| `SIMULATION` | Synthetic | Offline vehicle or ArduPilot SITL | No |
| `KRAKEN_SITL` | Real KrakenSDR | ArduPilot SITL | No |
| `LIVE` | Real KrakenSDR | Real ArduPilot vehicle | Potentially yes |
| `REPLAY` | Recorded observations | None | No |

In a mission, the software can:

1. Receive a Kraken bearing through HTTP.
2. Read MAVLink heartbeat, GPS, global position, heading, velocity, roll, and pitch.
3. Reject stale, malformed, frozen, low-confidence, wrong-frequency, poorly synchronized, or unsafe-to-collect measurements.
4. Estimate the emitter location from two or more accepted viewpoints.
5. Report residuals, approximate 95% horizontal uncertainty, crossing angle, geometry condition, accepted/rejected observations, and leave-one-out sensitivity.
6. Plan bounded observation locations and send `MAV_CMD_DO_REPOSITION` requests.
7. Require a reposition command acknowledgement and stop if it is rejected or times out.
8. Request `LOITER` when the configured uncertainty target is reached.
9. Stop or pause on invalid navigation, repeated invalid bearings, unexpected flight mode, movement timeout, mission timeout, distance-limit violations, or operator interruption.
10. Save a timestamped mission record for later replay and surveyed-truth validation.

The application never arms or disarms the aircraft and does not intentionally change ArduPilot parameters or disable failsafes. Arming, takeoff, landing, emergency action, and final flight authority remain with the operator and ArduPilot.

## Software limitations

- The uncertainty radius is an engineering approximation, not a certified containment bound. A common bias shared by all bearings can shift every bearing line consistently and may not be detectable from intersections alone.
- The estimator uses a small-area local coordinate approximation and is intended for a sub-kilometre search, not long-range navigation.
- The planner does not model terrain, buildings, obstacles, other aircraft, battery state, wind/energy cost, turn radius, or a complete geofence. ArduPilot must independently enforce operational limits.
- The application is synchronous and supports one vehicle, one emitter, and one bearing source per process.
- `LIVE` mode is not a complete deployment service. There is no service manager, automatic restart policy, redundant link, high-availability controller, or independent watchdog supplied here.
- A `PAUSED` or `ABORTED` mission is latched. Resuming requires operator review and a new application run.
- The code consumes the Kraken V2 legacy `DOA_value.html` format or the documented normalized JSON object. A different Kraken firmware/API needs a tested adapter.
- The V2 legacy endpoint must expose exactly one active DoA decimator. Multiple rows are rejected as ambiguous.
- Incoming RF frequency is only validated against the configured center frequency and bandwidth. Radio tuning and MUSIC/DoA processing remain in the Kraken stack.
- Heading accuracy is not currently populated from a dedicated MAVLink accuracy message by the live controller. The quality of the vehicle heading solution therefore needs separate hardware validation.
- The PoC has no authentication for the Kraken HTTP endpoint. MAVLink 2 signing is recommended where all endpoints support it, but key provisioning is outside this repository.
- The implementation does not protect against deliberate RF deception, GNSS spoofing, a compromised host, unknown API changes, or malicious MAVLink traffic.
- Simulation currently reports conservative uncertainty containment and cannot be used to claim field accuracy.

## Hardware and physical limitations

The repository does not specify a production-ready aircraft or minimum CPU/RAM specification. Hardware must be selected and qualified for the intended airframe and operating environment. A real setup needs, at minimum:

- a five-channel KrakenSDR and its supported coherent antenna array;
- the existing Kraken DoA host/software, reachable from the application host by HTTP;
- an RF source operating legally at the configured test frequency;
- a companion/ground computer capable of Python 3.10+, Ethernet/Wi-Fi access to Kraken, and a reliable USB/serial connection to the flight controller;
- an ArduPilot-compatible flight controller (the current bench profile targets a Cube Orange);
- a calibrated GNSS and heading solution;
- an operator-owned RC/manual takeover path, independent power termination, and all required ArduPilot safety systems; and
- a surveyed test area and surveyed emitter/observation points for acceptance testing.

Real accuracy and reliability are limited by factors the simulator does not establish:

- antenna element placement, cable matching, array calibration, and array reference-heading error;
- airframe shadowing and reflections, platform coupling, rotor/ESC electrical noise, vibration, and attitude changes;
- multipath, non-line-of-sight propagation, terrain blockage, interference, transmitter bandwidth/polarization, range, and signal strength;
- GNSS error, magnetic interference, heading bias, timestamp alignment, network delay, and USB/serial link quality;
- companion-computer thermal limits, power stability, weather protection, and radio/network dropouts; and
- payload mass, centre of gravity, endurance, vibration isolation, antenna drag, and safe separation from propellers.

No simulated localization number should be treated as expected field accuracy. Determine accuracy with surveyed stationary and ground trials using the exact production antenna, airframe, firmware, frequency, and calibration procedure.

## Real-system architecture

The intended initial hardware route is:

```text
KrakenSDR + Kraken host --HTTP bearing--> DF application
                                               |
Cube/ArduPilot <--serial--> MAVProxy <--localhost UDP--+
                              |
                              +--> operator console/map (localhost only)
```

MAVProxy must be the only process that opens the Cube serial device. It forwards telemetry to the DF application on `127.0.0.1:14560` and may provide a separate operator display on `127.0.0.1:14550`. During initial integration, do not expose these UDP ports to a LAN.

## Prepare the software

Run these commands from the repository root:

```bash
python3.10 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r kraken_df_nav/requirements.txt
python -m pytest -q kraken_df_nav/tests
```

If the Cube serial device is not accessible, add the operator to the serial-access group and then log out and back in:

```bash
sudo usermod -aG dialout "$USER"
```

Do not run the application with `sudo`.

## Required validation before a live flight

Running the Python command is not, by itself, a valid hardware deployment. Use this progression:

1. **Offline and SITL:** pass the unit tests and the nominal/safety SITL scenarios.
2. **Stationary Kraken test:** measure bearing stability over azimuth, range, signal level, and platform powered/unpowered states.
3. **Surveyed ground trial:** collect bearings from separated, surveyed points and validate estimates against surveyed emitter truth.
4. **Real Kraken with SITL (`KRAKEN_SITL`):** use real RF bearings to move only a virtual aircraft. Exercise loss, stale/frozen data, wrong frequency, multipath, poor geometry, and operator abort.
5. **Cube bench gate:** remove propellers, restrain the vehicle, keep it disarmed, and complete [the M9 bench checklist](docs/mavproxy_m9_bench_checklist.md).
6. **HIL/restrained testing:** verify link loss, bad GNSS/heading/bearings, command rejection, geofence actions, manual takeover, and power termination.
7. **Authorized field test:** obtain airworthiness, spectrum, site, and flight approvals. Start with advisory-only results. Enable bounded reposition commands only after the measured hardware evidence satisfies predeclared acceptance limits.

## Configure a real installation

First locate the stable Cube serial path:

```bash
find /dev/serial/by-id -maxdepth 1 -type l -printf '%p -> %l\n'
```

Copy the safe template to a site-specific file. Do not commit that file if it will later contain operational or security-sensitive details.

```bash
cp kraken_df_nav/config.mavproxy_live.yaml site.live.yaml
```

Edit at least these values in `site.live.yaml`:

```yaml
mode: LIVE

rf:
  center_frequency_hz: 915000000  # replace with the legal, actual test frequency
  bandwidth_hz: 2400000

kraken:
  endpoint: "http://KRAKEN_HOST:8081/DOA_value.html"
  antenna_heading_offset_deg: 0   # measured clockwise array-reference-to-airframe offset

mavlink:
  connection: "udpin:127.0.0.1:14560"
  expected_target_system: 1       # verify from the real vehicle
  expected_target_component: 1

mavproxy:
  enabled: true
  master_connection: "/dev/serial/by-id/ACTUAL_CUBE_DEVICE,115200"
```

Also review, measure, and freeze every confidence, age/skew, attitude, GPS/heading-accuracy, waypoint-distance, altitude, duration, total-distance, flight-mode, and uncertainty limit. In particular, `navigation.command_altitude_m: null` preserves the current reported relative altitude; set a reviewed value only if the test plan requires it.

Verify the Kraken endpoint before introducing MAVLink:

```bash
curl --fail --max-time 2 http://KRAKEN_HOST:8081/DOA_value.html
python -m kraken_df_nav.kraken.probe \
  --endpoint http://KRAKEN_HOST:8081/DOA_value.html \
  --count 120 --interval 0.25 \
  --output ./logs/kraken_stationary_probe.json
```

The endpoint must return one legacy CSV row containing timestamp-ms, bearing, confidence, power, and frequency, or a JSON object like:

```json
{"bearing_deg": 42.0, "confidence": 0.91, "rssi_dbm": -61.2, "frequency_hz": 915000000, "timestamp": 1787691600.0}
```

## Bench preflight (non-actuating)

With propellers removed, the aircraft restrained, and the vehicle disarmed, run:

```bash
source .venv/bin/activate
python -m kraken_df_nav.mavproxy_bench_preflight \
  --config site.live.yaml \
  --confirm-props-removed
```

Do not continue unless every check passes and the status is `READY_FOR_NON_ACTUATING_TELEMETRY_TEST`. This check validates configuration and port/device availability; it does not prove that flight is safe.

Stop Mission Planner, QGroundControl, ModemManager, and any other process using the Cube serial device. MAVProxy must have exclusive ownership.

## Run the real application

There is intentionally no one-command live launcher: operator-controlled startup and verification are part of the hardware safety boundary. Use separate terminals.

### Terminal 1: start MAVProxy

Substitute the reviewed values from `site.live.yaml`:

```bash
source .venv/bin/activate
python -m MAVProxy.mavproxy \
  --master=/dev/serial/by-id/ACTUAL_CUBE_DEVICE,115200 \
  --out=udp:127.0.0.1:14560 \
  --out=udp:127.0.0.1:14550 \
  --source-system=254 \
  --source-component=190 \
  --target-system=1 \
  --target-component=1 \
  --streamrate=5 \
  --console --map
```

If GUI dependencies are unavailable, omit `--console --map` for a headless bench session. Before starting the DF process, verify the correct target identity, heartbeat, GPS fix, position, heading, attitude, flight mode, battery, and stable message rates. Confirm the vehicle is still disarmed during bench work.

### Terminal 2: start the DF mission

Only after completing the staged validation and the approved flight test card, place the aircraft in an operator-controlled, already-flying state and a mode that accepts reposition requests (normally `GUIDED`). The application will not arm or take off for you.

```bash
source .venv/bin/activate
python -m kraken_df_nav.main --config site.live.yaml
```

The process will collect a bearing, estimate when it has enough observations, request bounded observation moves as needed, and request `LOITER` when the uncertainty target is met. Press `Ctrl+C` to stop the application; then use the independent operator/RC procedure to manage or land the aircraft. Do not assume stopping the process commands RTL or LAND—it does not.

Logs are written to the configured `logging.directory` (default `./logs`) as JSON and CSV. Preserve the MAVProxy tlog, console output, application logs, vehicle parameters/version, Kraken configuration/version, calibration record, weather/site details, and surveyed truth for each trial.

## Validate real trial results

Given surveyed emitter coordinates:

```bash
python -m kraken_df_nav.main \
  --config site.live.yaml \
  --validate ./logs \
  --truth-lat SURVEYED_LATITUDE \
  --truth-lon SURVEYED_LONGITUDE \
  --report-output ./logs/validation_report.json
```

Use multiple independent trials across the operational envelope. Compare actual position error with reported uncertainty, rejected samples, residuals, crossing angles, geometry condition, and leave-one-out shift. Freeze acceptance criteria before the final campaign to avoid selecting limits after seeing the results.

## Operator stop and failure behavior

- `Ctrl+C` ends the process and records an operator abort, but does not command RTL, LAND, or disarm.
- Invalid data configured with `ABORT` or `PAUSE` stops new mission actions; neither state is an aircraft flight mode.
- Loss of the application does not replace ArduPilot's RC/GCS-loss, geofence, battery, EKF, GNSS, or vehicle failsafes.
- The operator must continuously retain an independent takeover and termination method.
- After any unexpected command, wrong target, stale navigation, Kraken anomaly, link loss, boundary violation, or disagreement between map and visual observation, terminate the test under the approved test card and preserve the evidence.

## Related documentation

- [Primary package README](kraken_df_nav/README.md)
- [M9 Cube bench checklist](docs/mavproxy_m9_bench_checklist.md)
- [MAVProxy vehicle interface contract](docs/mavproxy_vehicle_interface.md)
- [Known limitations and hardware transition](docs/simulation_poc/hardware_transition.md)
- [Safety and threat model](docs/simulation_poc/safety_and_threat_model.md)
- [Simulation demonstration guide](docs/simulation_poc/demonstration_guide.md)

