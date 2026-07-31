# MATDOG Station integration boundary

Status: `CURRENT_CANONICAL` documentary boundary; no runtime implementation.

## Current fork-main contract

At `MattRobotics/norma-core@32e3222c87016b7f5d7c1c1da497a4cea3e7b80a`,
Station is the sole serial-bus owner and the ST3215 driver owns the port.
MATDOG does not open a serial device directly. The command path is Station
queue -> Station command envelope -> ST3215 driver command -> transmitter ->
port worker. State contains per-servo samples and timestamps.

The MATDOG native calibrator provides:

- explicit `MATDOG_NATIVE_CALIBRATOR_ARM` profile selection and one profile
  per process run;
- exact 12-servo preflight and profile-scoped motor/goal corridors;
- RAM writes limited to TorqueEnable, Acceleration, GoalPosition, GoalSpeed,
  and TorqueLimit, with no EEPROM/reset/freeze sequence;
- command-result barriers, fresh telemetry/readback and timestamp checks;
- prerequisite restore in reverse order and mandatory global torque-OFF on
  success or failure;
- command audit through the Station/driver path.

The main pin uses strict all-home entry and a fixed 12 s motion timeout. The
PR #4 head is experimental evidence for restart-safe profile entry,
distance-aware timeouts (minimum 12 s, 40 tick/s expectation plus 5 s), and a
16-tick active-probe home settle while static/prerequisite tolerance remains
10 ticks. These PR behaviors are not labeled fork-main facts.

SO101 and ElRobot generic EEPROM calibration behavior is represented by two
independent `NORMACORE_GENERIC_REFERENCE` claims, each pointing to its own
pinned source file and locator. Neither behavior is applicable to the MATDOG
RAM-only contact path.

## Ownership decision left open

Three future alternatives remain `DECISION_REQUIRED`:

A. Station remains the operational bus owner.

B. A future MATDOG controller becomes the bus owner.

C. A broker provides one physical owner and multiple logical clients.

Milestone I does not choose among them. Any option must preserve single-owner
serialization, explicit authority, auditability, bounded queues, fresh state,
timeouts, failure cleanup and an emergency-stop contract before hardware use.

XGoLite corroborates only the abstract separation of command/state models,
protocol adapters, telemetry, watchdogs and a single-owner bus. Its transport,
registers, timing and physical constants do not cross this boundary.
