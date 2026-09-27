# MATDOG Calibration Readiness Closure

**Status:** current architecture contract  
**Date:** 2026-09-27  
**Scope:** readiness for a future first controlled Full Calibration hardware session  
**Motion authorization:** **NONE** — this document does not authorize Torque ON, GoalPosition, contact probing, or any physical motion.

This document closes the architectural ambiguities that must be resolved before a production
calibration write path may be added. It does not replace the authoritative calibration state in
`06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml`; while
`calibration_reset.hardware_motion_authorized: false`, normal robot motion remains blocked.

## 1. Source precedence

Current calibration truth remains:

```text
state:                          CALIBRATION_RESET_PENDING_FULL_RECALIBRATION
all_joint_data_below_is_stale:  true
hardware_motion_authorized:     false
```

The current servo allocation, current URDF / collision geometry and the generated Geometry
Compiler V5 profile are current sources. LF V25 remains a historical hardware and regression
oracle only. Historical q0, historical physical-unit assignments and historical direction values
must never be promoted into current-installation calibration evidence.

## 2. Direction is current hardware-contract data

For the current mechanical architecture, `motorDirection` is part of the current URDF / Geometry
Compiler V5 contract, not a datum that normal recalibration measures.

The production rule is:

```text
q0              = current-installation calibration data; capture and promote
motorDirection  = current URDF / hardware-contract data; resolve from bound geometry
```

`JointTransform` therefore carries q0 but no stored direction copy. The current direction is
resolved through the bound `CalibrationGeometryProfile`, and a geometry-provenance change makes
previous transforms non-current.

`DIRECTION_VERIFY` remains an optional maintenance/development diagnostic. Its default budget is
zero. It is not a prerequisite for calibration acceptance, contact calibration, first stand, or
normal motion authorization.

A mechanical/topology change that invalidates the URDF contract requires updating and
re-validating the URDF/geometry source; it does not justify silently copying a historical
direction.

## 3. Full operational calibration does not mean 24 contact motions

Geometry Compiler V5 currently classifies the 24 leg endpoints as:

```text
8  upper-leg endpoints  EXECUTABLE_URDF_DOMAIN
16 hip/lower endpoints  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS
```

The sixteen hip/lower geometric contacts are evidence about the mechanism, not legal motion
targets under the current V5 policy. Full Calibration must not weaken `isExecutable()`, extend
the URDF limits, or command those diagnostic contacts merely to reproduce historical LF V25
behaviour.

For the current path, calibration completeness is defined by the evidence needed to construct a
safe current operational model:

- current identity / allocation and formal leg-population evidence for all 12 leg joints;
- current q0 for all 12 leg joints, captured at the nominal URDF q=0 pose with torque confirmed
  OFF and bound to physical-unit + geometry provenance;
- current direction resolved from the current URDF / Geometry V5 contract;
- physical contact calibration for the 8 executable upper-leg endpoints;
- current operational limits/envelopes for all 12 joints, with hip/lower envelopes derived
  conservatively without commanding beyond-URDF contacts;
- the required evidence lifecycle and explicit promotion to current operational evidence.

The hip/lower diagnostic contacts remain available for future model-validation/metrology work
under a separately reviewed safety plan. They are not a prerequisite for the first operational
calibration or first stand.

## 4. Calibration motion permit is not operational motion authorization

The current `hardware_motion_authorized` reset state represents the installed robot's
**operational** calibration/motion status. It must not be flipped merely to make a calibration
session executable.

A future controlled calibration motion path therefore needs a distinct, fail-closed,
session-scoped permit. Conceptually:

```text
CALIBRATION_MOTION_PERMIT
  != hardware_motion_authorized
```

The calibration permit is a temporary execution precondition for an explicitly authorized
calibration session. It must require the reviewed current prerequisites (including formal
population evidence, current q0/geometry prerequisites appropriate to the requested operation,
CALIBRATION actuator authority and operator authorization), and it expires with the session.

`hardware_motion_authorized` remains false throughout readiness work and throughout any
pre-calibration evidence collection. Transitioning it to true is a separate, reviewed
post-calibration state change after successful Full Calibration; it is not an entry condition
that may be used to bootstrap Full Calibration.

This separation removes the circular dependency:

```text
full recalibration needs controlled calibration motion
normal motion authorization must follow successful recalibration
therefore calibration motion cannot be represented by normal motion authorization
```

No implementation of the permit is added by this gate.

## 5. Permanent safety invariants

The readiness work must preserve all of the following:

- no EEPROM calibration write;
- no PositionOffset compensation;
- no CalibrationOfs / one-key-middle;
- GoalPosition raw domain is unsigned `0..4095`;
- signed-wrap target semantics are forbidden;
- SAFE_OFF remains independent of ordinary write policy and actuator authority;
- one ServoBus/UART owner; no duplicate transport ownership;
- historical replay can never authorize physical motion;
- `RESTORE INTENT != ABORT != SAFE_OFF`;
- authority loss fails closed and causes zero restore motion;
- LF V25 is a historical hardware/regression oracle, not the production execution architecture;
- no physical motion, Torque ON or contact probe occurs without explicit operator authorization.

## 6. Minimum readiness dependency chain

The lean dependency order is:

```text
contract closure
  -> formal current leg-population evidence producer
  -> read-only q0 bootstrap
  -> current promoted transforms
  -> V5 runtime binding
  -> single checked q<->raw target resolver
  -> persistence/promotion boundary
  -> minimal production actuator backend
  -> generic calibration endpoint executor
  -> session-scoped calibration motion permit
  -> hardware qualification
  -> 8 executable upper contact calibrations
  -> reviewed Full Calibration completion
  -> separate operational motion authorization
```

A production actuator backend is therefore not the first readiness step. Every prerequisite that
can be closed while the Controller remains physically unable to apply torque or GoalPosition
should be closed first.

## 7. Acceptance criteria for CR0

CR0 is complete when repository truth is internally consistent on these three points:

1. normal calibration does not require measuring `motorDirection`; `DIRECTION_VERIFY` is
   diagnostic-only;
2. Full Calibration does not require commanding the 16 hip/lower diagnostic contacts beyond the
   URDF domain;
3. future calibration motion authorization is explicitly distinct from final
   `hardware_motion_authorized`.

CR0 changes contracts and auditability only. It must not add a production backend, Torque ON,
GoalPosition, live calibration action command, geometry/transform admission, or any motion
authorization.

## 8. Implementation status

### CR0 — Contract Closure

**IMPLEMENTED on `feat/calibration-readiness-v1`.**

Repository contracts now agree that:
- production `motorDirection` comes from current URDF / Geometry V5;
- Full Calibration does not require the 16 beyond-URDF hip/lower contacts;
- controlled calibration motion will need a session-scoped permit distinct from final
  `hardware_motion_authorized`.

`check_calibration_readiness_contract()` protects these statements from documentation drift.

### CR1 — Formal Current Leg-Population Evidence Producer

**IMPLEMENTED / OFFLINE VALIDATED / NOT YET PRODUCTION-WIRED.**

`src/calibration/CalibrationPopulationEvidence.*` derives a formal
`LegPopulationEvidence` from the existing structured `ServoCensus` and `ServoPreflight`
results. It creates no bus transaction and no second census.

A current PASS requires:
- an explicitly current observation bundle rather than cached preflight data;
- canonical census range coverage and internally coherent census counters;
- no unexpected or absent-by-design responder;
- no missing leg ID (ID 51 remains outside the leg calibration population);
- exactly 12 unique semantic leg slots matched to the current canonical joint +
  expected physical-unit configuration;
- per-slot preflight PASS with the expected model, zero offset, full persistent-profile match,
  torque OFF and raw position inside 0..4095;
- final acceptance by `populationIsCurrentPass()`.

The domain gate itself now also rejects nonzero `unexpected_count`, closing the prior case in
which a complete 12-bit mask plus an anomalous responder could have evaluated PASS.

CR1 deliberately does **not** submit the result to `CalibrationManager` from production code
yet. Session freshness/orchestration is a later gate, and the existing 2026-09-26
`@SERVO PREFLIGHT 12/12 PASS` remains preflight evidence rather than being relabelled as formal
H1 evidence.

Offline validation was completed on the synchronized ASUS K53SV checkout on 2026-09-27 at
`55c036cb92d8039658309ef9fe6c3dc713ca22eb`: `python3 scripts/static_audit.py` PASS
(108 source files); the explicit host runner passed every suite, including
`CALIBRATION_POPULATION_EVIDENCE_TESTS = PASS` with 248 checks / 0 failures;
`git diff --check` was clean and the working tree remained clean.

### CR2 — Read-only q0 bootstrap

**CR2-A PURE FOUNDATION: IMPLEMENTED / TESTS ADDED / LOCAL OFFLINE VALIDATION PENDING.**

The current implementation adds `src/actuator/CalibrationQ0Bootstrap.*` and a dedicated host
suite. It does **not** read `ServoBus`, does not own a UART, does not issue any command, does not
bind the geometry into `Controller`, and does not create/admit a `JointTransform`.

CR2-A receives already-read samples and may produce only a per-joint `Q0Evidence`
`CANDIDATE`. A candidate requires:

- formal current leg-population evidence;
- the current bound Geometry V5 profile with matching provenance;
- semantic joint + current physical-unit identity, with bus ID checked only as transport metadata;
- a nonzero capture-session identifier;
- explicit operator confirmation that the mechanism is manually at nominal URDF q=0;
- an explicitly supplied stability budget;
- 3..32 successful samples;
- Torque OFF on every sample;
- every raw sample in unsigned encoder domain 0..4095;
- circular-median stability within the supplied budget.

`Q0Estimator::MANUAL_ZERO_POSE` distinguishes this current direct measurement from the two
historical LF V25 estimators. Raw 2048 is **not** an acceptance gate: its distance is recorded
only as a diagnostic. CR2-A cannot set `ACCEPTED`/`PROMOTED`, cannot admit a transform, and
cannot make q0 operational.

The old Station-mediated `matdog_digital_zero_calibration.py` is not reused as current runtime:
it belongs to the superseded pre-2026-08-27 architecture and contains a forbidden PositionOffset
write phase. CR2 reuses only the non-authoritative **measurement method** already demonstrated
there — repeated read-only samples, Torque OFF, circular median and stability spread — while
discarding its IDs, q0 values, offsets, EEPROM logic and tolerances.

**CR2-B SAME-SESSION READ-ONLY ORCHESTRATION: TO_IMPLEMENT.** A pure reducer cannot prove
wall-clock freshness. A later Controller-side read-only transaction must produce the formal
population evidence and q0 sample bundle inside one explicitly scoped capture session before
calling CR2-A. That future path still may not enable torque, command GoalPosition, move a joint or
write EEPROM.

Only after CR2-A passes the local host/static gate should CR2-B be designed.
