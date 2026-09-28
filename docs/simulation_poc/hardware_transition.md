# Known Limitations and Real-Hardware Transition

This package proves behavior only under modeled conditions.

## Known limitations

- No simulation validates KrakenSDR bearing accuracy, antenna calibration, platform reflections, rotor/ESC interference, vibration, RF propagation, or multipath realism.
- The frozen campaign missed its uncertainty-calibration ceiling: nominal containment was 99.67% against an 80–99% target. Uncertainty is slightly too conservative.
- The small-area coordinate approximation is not intended for long-range navigation.
- The planner omits terrain, obstacles, battery, turn radius, and full geofence planning.
- Routed waypoints now require and record `MAV_CMD_DO_REPOSITION` acknowledgements. Real-Cube
  command rejection and timeout behavior still require the disarmed M9 bench test.
- The synchronous single-vehicle/single-source design is a PoC, not a production architecture.

## Transition gates

1. Freeze the exact Kraken firmware/API, antenna geometry, frequency, calibration procedure, and timestamps.
2. Repeat stationary surveyed-angle tests over azimuth, range, signal strength, and platform operating states.
3. Run a walking/vehicle ground campaign at surveyed points; measure bias, dispersion, outliers, latency, and uncertainty containment.
4. Connect the real Kraken to SITL (`KRAKEN_SITL`) so real RF drives only a virtual aircraft; verify every fault and safety stop.
5. Add authenticated interfaces, health monitoring, waypoint ACK handling, watchdogs, and independent command/geofence limits.
6. Perform hardware-in-the-loop bench and restrained tests, including link loss, bad GNSS, bad heading, bad bearings, and operator takeover.
7. Obtain airworthiness, spectrum, site, and flight approvals; establish a test card, observers, termination criteria, and recovery plan.
8. Begin low-risk flight tests with the navigation output advisory-only. Enable bounded commands only after measured evidence satisfies newly frozen hardware requirements.

Simulation accuracy numbers must not be reused as expected field accuracy. Hardware acceptance thresholds must be declared before collecting the final field campaign.
