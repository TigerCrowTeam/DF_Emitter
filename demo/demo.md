# KrakenSDR Emitter Direction-Finding Demo

## What this demonstration shows

This proof of concept demonstrates how a drone could autonomously improve the estimated location of a radio-frequency emitter.

The demonstration uses ArduCopter Software in the Loop (SITL) instead of a physical aircraft and simulates KrakenSDR direction-of-arrival measurements. The navigation, MAVLink messaging, MAVProxy routing, mission logic, safety checks, and map display use the same architectural path intended for later hardware integration.

## Demonstration sequence

1. The launcher starts a simulated ArduCopter, MAVProxy, its console, and its map.
2. The aircraft flies approximately 60 meters in a straight line before detecting an emitter. This makes the transition from normal flight to direction-finding activity easy to see.
3. The system announces that an RF emitter has been detected. At this point, its location is still unknown.
4. The direction-finding application collects simulated bearing measurements containing a modest amount of realistic noise.
5. The application estimates which new observation locations will improve the crossing geometry between bearings.
6. Movement commands are sent to the simulated aircraft through MAVLink and routed by MAVProxy.
7. The aircraft moves, stabilizes, collects additional observations, and repeatedly improves the estimate.
8. When the estimate meets the configured completion criteria, the aircraft enters `LOITER` and the calculated emitter location is displayed on the MAVProxy map with a red antenna symbol.
9. The map and console remain open so the final result can be discussed. Press `Ctrl+C` in the launch terminal when the presentation is finished.

The final red antenna marker comes from the application's direction-finding estimate. It is not placed using the simulator's known emitter coordinates.

## What to point out during the presentation

- The initial straight flight represents an aircraft performing its normal mission before RF detection.
- The course changes after detection are selected to produce useful cross-bearing geometry, not simply to fly directly toward the emitter.
- MAVProxy provides the visible aircraft track and acts as the MAVLink routing layer between the application and simulated flight controller.
- The red antenna represents the application's final calculated emitter position.
- The final `LOITER` behavior shows that the application stops issuing search movements after completing the estimate.
- Run evidence and a validation report are written under `logs/mavproxy/sitl/` for later review.

## Run the demo

Open a terminal in the project directory and run:

```bash
./demo/run_mavproxy_demo.sh
```

Allow the MAVProxy console and map windows to open. Keep the original launch terminal visible because it reports the current phase and announces completion.

When the terminal displays:

```text
EMITTER FOUND - map remains open; press Ctrl+C here to close the demo.
```

the run has completed successfully. Review the aircraft track and red emitter marker, then press `Ctrl+C` in the original terminal to close the demonstration cleanly.

## Scope of the proof of concept

This demo validates the end-to-end software workflow without requiring a Cube flight controller, a physical drone, or live RF transmission. It does not yet validate real KrakenSDR measurement accuracy, antenna installation effects, onboard-computer performance, radio-link behavior, environmental multipath, or flight safety. Those items remain part of hardware and field validation.
