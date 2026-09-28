# M9 Cube Bench Integration Checklist

M9 is a hardware bench gate, not a flight authorization. Remove all propellers and restrain the
airframe before powering the Cube. Keep the vehicle disarmed throughout initial testing.

## 1. Configure without opening the serial link

1. Find the Cube's stable link:

   ```bash
   find /dev/serial/by-id -maxdepth 1 -type l -printf '%p -> %l\n'
   ```

2. Copy `kraken_df_nav/config.mavproxy_live.yaml` to a site-specific file that is not committed.
3. Replace the Cube path and Kraken endpoint. Do not use `/dev/ttyACM0`; enumeration can change.
4. Keep all MAVProxy outputs on `127.0.0.1`. Remote access requires a separately reviewed VPN and
   host firewall; never expose raw MAVLink UDP to an untrusted LAN.
5. Run the non-actuating gate:

   ```bash
   .venv/bin/python -m kraken_df_nav.mavproxy_bench_preflight \
     --config SITE_CONFIG.yaml --confirm-props-removed
   ```

Do not continue unless it reports `READY_FOR_NON_ACTUATING_TELEMETRY_TEST`.

## 2. Telemetry-only MAVProxy bench test

- Confirm propellers removed, vehicle restrained, transmitter safe, and an operator at the power
  disconnect.
- Stop Mission Planner, QGroundControl, ModemManager, and every other serial client. MAVProxy must
  be the exclusive Cube serial owner.
- Start MAVProxy with the exact master, identities, and separate loopback outputs printed by the
  site configuration. Do not start the DF application.
- Verify heartbeat, target system/component, GPS, attitude, battery, flight mode, parameter fetch,
  and stable message rates. Save the tlog, console log, parameters, versions, and process status.
- Confirm the vehicle remains disarmed and no servo/motor output changes.

## 3. Independent safety verification

Record the actual Cube parameter values and independently test:

- geofence enable/type/radius/altitude and fence action;
- altitude and mission-distance bounds;
- battery monitor, low/critical thresholds, and actions;
- RC loss, GCS/link loss, EKF/GNSS failure actions, and return/land behavior;
- physical/manual mode takeover and emergency motor stop policy;
- operator roles, verbal abort call, power disconnect, exclusion zone, and incident recovery.

The application must not modify these parameters. A second person should review the captured
parameter file against the approved test card.

## 4. Disarmed command-path tests

With propellers removed and the vehicle disarmed, verify that arming remains solely operator-owned
and that movement/mode requests are rejected or cannot actuate. Save target identities and command
ACKs. Never validate movement with the real airframe at this stage; use the existing ArduPilot SITL
or an approved non-actuating HIL/bench harness.

## 5. MAVLink signing and network policy

- Prefer MAVLink 2 signing when every required endpoint supports and has been provisioned for it.
- Generate and distribute signing keys outside this repository; never place keys in YAML, logs, or
  screenshots. Document key custody, rotation, revocation, and recovery.
- If signing cannot be used for this bench, keep traffic strictly on loopback and record that as a
  temporary compensating control. It is not acceptable for remote or flight operation.
- Default-deny inbound firewall policy; allow no external access to ports 14550, 14560, or 14561.
  Any remote operator path requires an authenticated VPN and a separate security review.

## 6. Real Kraken driving only SITL

Before connecting real bearings to a real Cube, run `KRAKEN_SITL`: real Kraken measurements and
the actual antenna/platform setup must drive only the virtual aircraft. Repeat nominal, loss,
stale/frozen data, wrong frequency, multipath, bad geometry, and operator-abort cases. Freeze the
Kraken firmware/API, antenna geometry, heading offset, timestamps, frequency, and measured error
model. Simulation accuracy numbers are not field-accuracy claims.

## Completion evidence

M9 passes only when the real Cube provides telemetry through MAVProxy while disarmed, no unintended
actuation occurs, all independent safety controls have signed-off evidence, and the real-Kraken
SITL campaign passes. Flight testing is a separate authorized activity.
