# Architecture and Data Flow

This document describes a simulation-only proof of concept.

```text
Emitter truth (simulator only) -> Kraken digital twin -> bearing measurement
                                                        |
ArduPilot SITL -> MAVLink telemetry -> pose/time gate ----+-> estimator
                                                              |
                                safety/state machine <---------+
                                      |                       |
                               bounded waypoint -> MAVLink    estimate
                                      |
                                ArduPilot SITL

Emitter truth -> post-mission scorer (never into mission logic)
All components -> evidence recorder -> JSON/CSV/audit
```

The bearing source produces body-relative direction, confidence, signal level, frequency, and timestamp. Telemetry supplies position, heading, attitude, speed, navigation accuracy, flight mode, and freshness. The observation gate rejects low-confidence, stale, desynchronized, moving, tilted, or inaccurate-pose samples. Accepted observations are transformed to true-north bearings.

The robust estimator intersects multiple bearing lines, rejects inconsistent observations, and reports coordinates, residual, geometry score, and an approximate uncertainty radius. The planner chooses a bounded next viewpoint for improved crossing geometry. The state machine is the sole owner of movement decisions and stops commands on invalid navigation, repeated bad measurements, rejected waypoints, time/distance limits, operator interruption, or terminal state.

## MAVProxy-routed SITL architecture

```text
ArduCopter SITL tcp:5760
          |
      MAVProxy 254:190
       /          \
UDP 14560       UDP 14561
DF app 255:191  preparation/independent validation
       |
target ArduPilot 1:1
```

MAVProxy is the only owner of the SITL master. The application consumes forwarded heartbeat,
position, GPS, and attitude messages and emits acknowledged `MAV_CMD_DO_REPOSITION` commands plus
terminal LOITER. Each consumer has a separate UDP output so telemetry reads do not compete.
`demo/run_mavproxy_demo.sh` owns every process group, readiness transition, run directory, and cleanup.

The routed demonstration is simulation-only. The simulated emitter truth is used only by the
bearing generator and post-run scorer; it is never supplied to the estimator or planner.

The simulator’s emitter coordinates are kept outside mission-facing measurement types. Only the post-mission validator uses truth to calculate actual error and containment.
