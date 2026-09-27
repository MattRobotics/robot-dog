# MATDOG Calibration Readiness — CR2 read-only q0 bootstrap foundation

**Date:** 2026-09-27  
**Branch:** `feat/calibration-readiness-v1`  
**Scope:** offline/read-only q0 evidence foundation  
**Hardware action:** NONE  
**Motion:** NONE  
**Torque ON:** NONE  
**GoalPosition:** NONE  
**EEPROM / PositionOffset / CalibrationOfs write:** NONE

## 1. Why CR2 exists

The current installation has no accepted q0 evidence after the 2026-08-27 calibration reset.
The current contract is:

```text
operator manually aligns nominal URDF q=0
-> torque confirmed OFF
-> repeated read-only raw encoder observations
-> current q0 candidate
```

Raw centre 2048 is not q0 and must never be substituted for a measurement.

## 2. Existing implementation reused versus rejected

The superseded Station-era `matdog_digital_zero_calibration.py` was audited before new code was
designed.

### Reused only as measurement behaviour

Its read-only preflight established a useful measurement method:

- multiple samples rather than one snapshot;
- Torque OFF verified during capture;
- circular encoder handling around 0/4095;
- deterministic median;
- measured stability spread.

Those are generic measurement semantics, not calibration values.

### Explicitly not reused

The following remain historical/superseded and are not imported:

- old 12-servo IDs / joint-unit bindings;
- historical q0 values;
- historical PositionOffset values;
- the Station serial-ownership architecture;
- EEPROM unlock/write/relock execution;
- the target-displayed-2048 recentering model;
- historical q0/stability tolerances.

CR2 contains no PositionOffset or EEPROM path.

## 3. Domain-model correction

The previous `Q0Estimator` taxonomy contained only LF V25 historical estimators:

```text
FIXED_SCALE
AFFINE
```

Current direct read-only measurement at an operator-confirmed nominal URDF q=0 pose is a third,
different method, now represented explicitly as:

```text
Q0Estimator::MANUAL_ZERO_POSE
```

This prevents a current direct measurement from being mislabeled as historical affine evidence.

## 4. CR2-A pure candidate builder

Added:

```text
src/actuator/CalibrationQ0Bootstrap.h
src/actuator/CalibrationQ0Bootstrap.cpp
scripts/tests/test_calibration_q0_bootstrap.cpp
```

The module is in the actuator/calibration-geometry side because q0 evidence must bind to the exact
Geometry V5 provenance. It is nevertheless a pure reducer: no Arduino, no ServoBus, no UART, no
clock, no command surface and no hardware call.

### Inputs

- current `CalibrationGeometryProfile`;
- expected six-hash `GeometryProvenance`;
- formal current `LegPopulationEvidence`;
- `Q0BootstrapRequest`:
  - semantic `JointIdentity` including physical-unit label;
  - bus id as transport metadata only;
  - nonzero capture-session id;
  - explicit operator confirmation of nominal URDF q=0;
  - explicit stability-budget-presence flag and budget;
- 3..32 already-read `Q0CaptureSample` observations:
  - read success;
  - raw encoder tick;
  - observed TorqueEnable.

### Fail-closed gates

A candidate is refused if any of the following is true:

- population evidence is not a current PASS;
- geometry is unbound or its six-hash provenance differs;
- semantic/physical-unit identity is absent from the current profile;
- current profile bus metadata disagrees with the request;
- capture-session id is absent;
- manual q=0 confirmation is absent;
- stability budget was not explicitly supplied;
- sample count is outside 3..32;
- any read failed;
- any sample reports TorqueEnable != 0;
- any raw sample is outside unsigned 0..4095;
- circular spread exceeds the explicitly supplied stability budget.

### Successful output

CR2-A emits only:

```text
measured              = true
estimator             = MANUAL_ZERO_POSE
state                 = CANDIDATE
origin                = LIVE_SESSION
identity              = current semantic joint + physical unit
tick                  = measured circular median
geometry              = current Geometry V5 provenance tag
accepted_by_gate      = false
```

Bus id and capture-session id remain diagnostic/provenance metadata outside `JointIdentity`.

CR2-A has no path to:

```text
ACCEPTED
PROMOTED
JointTransformTable::admit()
Torque ON
GoalPosition
PositionOffset
EEPROM
```

Therefore `q0MayBeAppliedTo()` necessarily refuses every CR2-A result.

## 5. 2048 is deliberately not a candidate gate

A stable synthetic sample set centred at raw 3000 is expected to produce a candidate. This is a
test requirement, not a recommendation that 3000 is mechanically plausible.

The difference to raw centre is recorded in
`Q0Evidence.shift_from_digital_home_ticks` only as a diagnostic. The q0 plausibility window is
still **TO_DERIVE from current 12-joint measurements**; CR2 does not choose it in advance.

This preserves the bootstrap contract's required order:

```text
measure first
-> inspect current distribution / mechanics
-> derive plausibility gate
-> explicit acceptance
```

## 6. Circular encoder arithmetic versus target arithmetic

CR2 uses modulo/circular arithmetic only to reduce **encoder observations** around the 4095→0
boundary. It does not produce a motion target.

This does not weaken the permanent target invariant:

```text
GoalPosition = unsigned 0..4095
signed-wrap target semantics = forbidden
```

A dedicated test covers samples `4095, 0, 1` and expects median 0 with spread 1.

## 7. Same-session freshness boundary

CR2-A cannot prove wall-clock/session freshness because it receives immutable value objects.
Production use therefore remains forbidden.

CR2-B must later create one explicitly scoped **read-only evidence-capture transaction** that:

1. obtains current formal population evidence;
2. keeps the operator-confirmed nominal q=0 pose unchanged;
3. repeatedly reads current raw position + TorqueEnable from the one existing ServoBus;
4. labels those observations with one capture session;
5. invokes CR2-A only on that same-session bundle.

CR2-B may not acquire motion permission merely to perform reads and may not route through a live
motion session blocked by `hardware_motion_authorized=false`.

## 8. Audit and test integration

`check_calibration_q0_bootstrap()` enforces:

- no Arduino / ServoBus / UART / clock / actuator-write ownership in CR2-A;
- current population, current geometry, identity and bus cross-checks;
- explicit pose confirmation and stability budget;
- sample count/read/Torque-OFF/raw-domain/stability gates;
- output fixed at `CANDIDATE` / `LIVE_SESSION` / `accepted_by_gate=false`;
- no `PROMOTED`, no transform admission;
- no raw-centre acceptance condition;
- no production call from Controller, CommandRouter, ServoPreflight or ServoBus.

`run_host_tests.sh` now compiles and executes
`test_calibration_q0_bootstrap` against the real calibration domain and real generated Geometry
V5 profile implementation.

## 9. Validation status

Current status before the next synchronized local run:

```text
CR2-A SOURCE IMPLEMENTED
CR2-A TESTS ADDED
CR2-A STATIC AUDIT ADDED
CR2-A OFFLINE VALIDATED
CR2-B PRODUCTION READ-ONLY ORCHESTRATION TO_IMPLEMENT
FORMAL CURRENT H1 NOT CLAIMED
LIVE q0 NOT CAPTURED
HARDWARE MOTION AUTHORIZATION UNCHANGED / FALSE
```

No hardware action is authorized by this implementation.

## 10. Local validation closure

On the synchronized ASUS K53SV checkout at `5daefc73f824797d3052c75002a495bed21b58f3`:

```text
python3 scripts/static_audit.py
  Scanned 111 source files
  STATIC_AUDIT = PASS

bash scripts/tests/run_host_tests.sh
  CALIBRATION_Q0_BOOTSTRAP_TESTS = PASS
  checks_run=52 failures=0
  all other host suites PASS

git diff --check
  clean

git status --short --branch
  ## feat/calibration-readiness-v1
```

CR2-A is therefore **offline validated**. This does not constitute formal current H1, a live q0 capture, evidence promotion, hardware motion authorization, or any hardware action.
