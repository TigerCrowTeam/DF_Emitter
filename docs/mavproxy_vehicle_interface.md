# MAVProxy M0 Vehicle Interface Contract

This is the frozen pre-integration boundary between the emitter direction-finding application and
the vehicle. `VehicleController` remains the abstraction used by mission logic. Introducing
MAVProxy must not add MAVProxy dependencies to the estimator, planner, geometry, bearing source,
or safety state machine.

## Controller operations

| Operation | Purpose | Current implementation |
|---|---|---|
| `connect()` | Establish link and obtain complete initial telemetry | Blocking pymavlink connection and heartbeat wait |
| `get_vehicle_state()` | Return the most recent complete vehicle state | Drains and combines MAVLink telemetry |
| `goto_location(lat, lon, alt)` | Request one bounded position target | `MAV_CMD_DO_REPOSITION` |
| `distance_to_target()` | Evaluate progress to the last requested target | Small-area geographic distance |
| `command_loiter()` | Request terminal LOITER behavior | MAVLink `SET_MODE` |
| `navigation_valid()` | Reject missing, invalid, or stale navigation | `VehicleState.navigation_valid()` |
| `close()` | Release owned resources | No-op for the current controller |

## Consumed MAVLink messages

| Message | Fields used |
|---|---|
| `HEARTBEAT` | flight mode and armed flag; establishes target system/component through pymavlink |
| `GLOBAL_POSITION_INT` | latitude, longitude, relative altitude, heading, north/east velocity, receipt time |
| `GPS_RAW_INT` | fix type, satellites visible, horizontal accuracy proxy (`eph`) |
| `ATTITUDE` | roll and pitch |

The current controller timestamps navigation at local receipt time. `heading_accuracy_deg` is not
currently populated from a dedicated MAVLink message and defaults to zero. M1 must preserve this
behavior for baseline comparison; any later accuracy-message improvement requires its own tests.

## Emitted MAVLink traffic

| Message/command | Purpose | Result evidence |
|---|---|---|
| `COMMAND_LONG / MAV_CMD_SET_MESSAGE_INTERVAL` | Request position, raw GPS, and attitude at 5 Hz | Telemetry arrival and freshness |
| `COMMAND_INT / MAV_CMD_DO_REPOSITION` | Request one GUIDED waypoint using global-relative-altitude coordinates | Required `COMMAND_ACK`, target identity, arrival, and timeout evidence |
| `SET_MODE` | Request LOITER | Independently observed LOITER heartbeat |

Every reposition command requires an accepted or in-progress `COMMAND_ACK`. Rejection or timeout
stops the mission; the mission record saves the command, target identity, numeric and named ACK.

## Frozen identity and authority

- Expected vehicle target: discovered from heartbeat, then validated explicitly during M2/M3.
- Routed PoC source system/component: 255:191. System 255 matches ArduPilot `MAV_GCS_SYSID`.
- MAVProxy source system/component: 254:190, kept distinct for independent packet sequences.
- Initial routing endpoint: loopback UDP port 14560.
- Optional operator/map output: loopback UDP port 14550.
- The PoC may request bounded observation movement and LOITER.
- The PoC may not arm, disarm, disable failsafes, or bypass ArduPilot.
- No waypoint may be emitted after `ABORTED`, `PAUSED`, or another safety stop.

## Compatibility rule

M1 and M2 may change transport ownership and identity handling, but they must not change mission
behavior or numerical algorithms. Direct SITL, offline simulation, replay, and existing tests must
remain usable while MAVProxy routing is developed.
