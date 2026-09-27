# CR2-C — Q0 hardware session

**Date:** 2026-09-27
**Branch:** `feat/calibration-readiness-v1`
**Authorization:** RECEIVED from operator
**Authorization phrase:** `AUTORIZZO CR2-C READ-ONLY Q0 HARDWARE SESSION`
**Execution status:** PRE-FLIGHT / NOT YET FLASHED / NOT YET CAPTURED

## Authorized scope

The operator explicitly authorized the CR2-C hardware session prepared in
`CR2C_Q0_HARDWARE_READONLY_RUNBOOK.md`.

The authorized objective is the first current-installation, Torque-OFF, no-motion acquisition of
12 q0 candidates.

Required enabling steps such as recovery verification, exact ROBOT_POWERED build provenance and
application-only flashing remain subject to their existing fail-closed script gates and must be
executed exactly as documented. Authorization does not waive any gate.

## Permanent constraints for this session

```text
NO Torque ON
NO GoalPosition
NO automatic motion
NO EEPROM write
NO PositionOffset write
NO CalibrationOfs
NO servo re-ID
NO merge to main
NO force push / history rewrite
SAFE_OFF remains independent
hardware_motion_authorized remains false
```

## Current state at authorization

CR2-A: OFFLINE VALIDATED
CR2-B: OFFLINE + BUILD VALIDATED
CR2-C: AUTHORIZED / PRE-FLIGHT

No hardware action had been performed by this CR2-C session at the time this log was created.

## Power-topology correction before execution

Before the operator executed the first CR2-C command block, they identified an incorrect
assumption in the initial runbook/guidance: MATDOG does not have an independently powered ESP32
with a switchable/dead servo rail.

Current validated hardware truth:

```text
battery + main fuse/disconnect + DALY KEY ON
  -> DALY protected B+/P- domain energized
  -> ESP32-S3 / TECNOIOT 5 V powered
  -> servo rail powered
  -> LED rail powered
```

This is the intended post-2026-09-24 wiring and is consistent with the existing G3
`ROBOT_POWERED` validation.

CR2-C safety was therefore corrected from:

```text
controller powered + servo rail 0 V
```

to:

```text
ROBOT_POWERED domain energized
+ charger disconnected
+ robot mechanically supported
+ all 13 installed servos VERIFIED_OFF before esptool reset/read/flash
+ no Torque ON / GoalPosition / motion path
```

No hardware command had been executed before this correction. The runbook was amended before
continuing.

## CR2-C.1 — pre-flash powered SAFE_OFF verification

**Result: PASS**

The operator executed the no-reset native USB CDC session with the robot in the validated
`ROBOT_POWERED` topology.

Observed before SAFE_OFF:

```text
SYSTEM health=BOOTING power_state=RUN mode=MAINTENANCE authority=NONE
profile=ROBOT_POWERED
runtime_resets=0
```

The following 13 installed servo IDs were then commanded through the existing de-escalation-only
`@SERVO SAFE_OFF` path:

```text
11 12 13 21 22 23 31 32 33 41 42 43 51
```

Every one returned:

```text
result=VERIFIED_OFF
```

Terminal result:

```text
CR2C_SAFE_OFF=PASS verified=13/13 ids=11,12,13,21,22,23,31,32,33,41,42,43,51
```

Final cached system state:

```text
SYSTEM health=READY power_state=RUN mode=MAINTENANCE authority=NONE
SERVO init=OK detected=ONLINE expected=REQUIRED result=PASS
runtime_resets=0
```

No servo motion was reported. No Torque ON, GoalPosition, EEPROM, PositionOffset or CalibrationOfs
operation was performed.

The powered bus is therefore in the required pre-esptool safe state for CR2-C.2 recovery backup.
