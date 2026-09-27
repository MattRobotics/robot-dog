# MATDOG Calibration Readiness — CR0 / CR1 implementation record

**Date:** 2026-09-27  
**Branch:** `feat/calibration-readiness-v1`  
**Base:** `de187f541709b565ba6fbf576fdffcee90b690d4`  
**Scope:** offline/read-only readiness only  
**Hardware action:** NONE  
**Motion:** NONE  
**Torque ON:** NONE  
**GoalPosition:** NONE  
**EEPROM / PositionOffset / CalibrationOfs write:** NONE

## 1. CR0 — contract closure

The workstream first reconciled current repository truth rather than adding a write path.

Closed decisions:

1. Production `motorDirection` is current URDF / Geometry Compiler V5 hardware-contract data.
   It is resolved from the bound geometry profile. The legacy `DirectionEvidence` vocabulary
   remains available to historical replay / optional diagnostics but is not a production
   calibration prerequisite.
2. Full operational calibration does not mean commanding all 24 V5 geometric contacts.
   The 8 upper endpoints remain the executable physical contact-calibration set; the 16
   hip/lower endpoints remain diagnostic-only beyond the current URDF domain.
3. Future controlled calibration motion requires a session-scoped calibration-motion permit
   distinct from final `hardware_motion_authorized`. The latter remains false and is not used
   to bootstrap calibration.

New canonical document:
`05_Firmware/MATDOG_Controller/CALIBRATION_READINESS.md`.

New static gate:
`check_calibration_readiness_contract()`.

## 2. CR1 — formal current leg-population evidence

### 2.1 Gap found

The calibration domain already carried `LegPopulationEvidence.unexpected_count`, but
`evaluateLegPopulation()` did not consult it. A full 12-bit mask plus anomalous responders
could therefore evaluate PASS.

This was corrected first: nonzero `unexpected_count` now yields `PopulationVerdict::FAIL`,
with a regression test.

### 2.2 Producer

Added:

- `src/calibration/CalibrationPopulationEvidence.h`
- `src/calibration/CalibrationPopulationEvidence.cpp`
- `scripts/tests/test_calibration_population_evidence.cpp`

The producer consumes the existing structured `servo::CensusResult` and
`servo::PreflightResult`. It performs no scan and owns no transport.

A formal current PASS requires:

- an explicitly current observation bundle;
- complete canonical census range and internally coherent source counters;
- no unexpected or absent-by-design responder;
- no missing leg ID; ID 51 is not a leg-calibration slot;
- complete 12-joint preflight;
- 12 unique semantic leg slots derived from canonical joint name + expected physical-unit
  configuration, not a bus-id-only calibration identity;
- exact current model;
- readable zero offset;
- full persistent-profile match;
- torque OFF;
- raw position inside 0..4095;
- final `populationIsCurrentPass()` acceptance.

The existing `@SERVO PREFLIGHT 12/12 PASS` is not renamed or promoted automatically. CR1 has
no production orchestration call site; a later gate must establish same-session freshness and
submit the newly produced formal evidence to `CalibrationManager`.

### 2.3 Host-linkability prerequisite

`ServoPreflight.h` no longer includes the concrete bus transport header. It forward-declares
`ServoBus`; `ServoPreflight.cpp` owns the concrete include. This changes no device behaviour
and allows the structured result type to be consumed by host tests without Arduino/SCServo.

### 2.4 Audit / test integration

- CR1 producer added to calibration purity checks.
- Dedicated `check_calibration_population_evidence()` prevents transport/hardware ownership and
  pins the formal qualification gates.
- Producer is audit-forbidden from being called by `Controller.cpp` / `CommandRouter.cpp`
  during this foundation gate.
- `run_host_tests.sh` compiles and runs the new dedicated suite.

## 3. Validation status

GitHub has no Actions workflow for this repository. The connector can verify source/ref/diff
truth but cannot execute the repository's C++/Python test runner.

Therefore current status is deliberately:

```text
CR0  IMPLEMENTED / SOURCE-REVIEWED / AUDIT ADDED
CR1  IMPLEMENTED / TESTS ADDED / AUDIT ADDED
CR1  LOCAL HOST + STATIC EXECUTION REQUIRED
HARDWARE VALIDATION  NOT STARTED
FORMAL CURRENT H1    NOT CLAIMED
```

No CR2 q0 implementation should be treated as validated until this checkpoint passes locally.

## 4. Next gate

CR2 is the read-only q0 bootstrap:

```text
manual nominal URDF q=0 placement
-> torque confirmed OFF
-> repeated raw-position reads
-> current semantic joint + physical-unit identity
-> current Geometry V5 provenance
-> q0 candidate evidence
-> explicit later acceptance/promotion
```

CR2 must not add Torque ON, GoalPosition or automatic motion.
