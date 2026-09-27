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
