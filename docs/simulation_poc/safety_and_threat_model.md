# Assumptions, Threat Model, and Safety Boundary

All results discussed here are simulated.

## Assumptions

- One vehicle, one emitter, one Kraken bearing source, and a sub-kilometre search area.
- ArduPilot is configured, armed, airborne, and in a mode that accepts position targets before this application starts.
- ArduPilot remains the flight-control authority; this application never arms or disarms.
- Configuration values, antenna heading offset, frequency, coordinate frames, and clocks are correct.
- Terrain, obstacles, battery reserve, airspace, and regulatory compliance are handled outside this PoC.

## Modeled threats and failures

- Bearing noise, bias, drift, multipath/outliers, low confidence, loss, staleness, and frozen samples.
- Heading/GPS errors, telemetry loss, pose/bearing timestamp skew, excessive motion or attitude.
- Poor bearing-line geometry, estimator instability, geofence refusal, movement timeout, duration/distance exhaustion, and operator interrupt.
- MAVProxy loss during collection or movement, autopilot master-link loss, wrong target identity,
  competing-GCS mode/movement commands, degraded message rate, and process/port conflicts.

The model does not establish resistance to deliberate RF deception, cybersecurity compromise, GNSS spoofing, unknown Kraken API changes, antenna/platform coupling, rotor interference, terrain blockage, collision, or real wind/energy constraints.

## Safety boundary

The application may read telemetry and request bounded position targets. It may not arm, disarm, change mission-wide failsafes, override ArduPilot, or continue issuing waypoints after `ABORTED`, `PAUSED`, or another safety stop. ArduPilot and the human operator retain final authority.

MAVProxy exclusively owns the autopilot master link. The application sees only a loopback UDP
route. A lost route or MAVProxy restart makes the active mission terminal; cached state is never
automatically resumed. Source IDs are separated, target 1:1 is validated, and accepted command
ACKs are required. These controls reduce accidental authority conflicts but are not MAVLink
signing, authentication, or protection against a compromised local process.

Before flight, independent geofence, altitude, battery, link-loss, return/land behavior, obstacle clearance, manual takeover, and regulatory controls are required. A passing simulation is not authorization to fly.
