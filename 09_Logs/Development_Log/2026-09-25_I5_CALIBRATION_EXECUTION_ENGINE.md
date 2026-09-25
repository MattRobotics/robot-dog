# I5 — Calibration Execution Architecture

**Date:** 2026-09-25 · **Branch:** `feat/controller-nextgen-integration-v1`

Closes gate **I5** (V3 handoff §13), per explicit operator scoping received mid-gate after an
initial audit-and-stop (recorded below). This document reflects the operator's final instructions,
which are binding over the initial I5 audit's tentative proposals.

## Initial audit (before scoping instructions)

Read `CalibrationDomain.h` (492 lines), `CalibrationManager.h`/`.cpp`, and `ActuatorWritePolicy.h`.
Found: the domain model, evidence lifecycle, contact-witness gates, and the full 18-phase LF V25
sequence are already recovered and offline-tested (702 checks). `CalibrationManager` already
exposes the hooks a future execution engine would call (`noteExecutionPhase()`, `recordContact()`),
but nothing in production code calls them — the only thing that walks the 18 phases is logic
embedded in `test_calibration_domain.cpp`'s replay of the archived session.

Given the genuine ambiguity in what "advance the execution architecture" should concretely mean,
and the risk that extracting that replay walk into production code would either be exactly right or
exactly the kind of scope creep the V3 handoff warns against, this was surfaced to the operator
rather than decided unilaterally.

## Binding operator scoping (received, then implemented exactly)

The operator's instruction was explicit and is repeated here for the record, since it corrects a
stale assumption in `DEVELOPMENT_GATES.md` (fixed as step 1, below):

> Do NOT extract the historical LF V25 18-phase replay into a production `CalibrationReplayEngine`.
> V3 Section 15.11 is binding: LF V25 is an immutable historical oracle/replay source, and the old
> 18-state sequence must NOT become the new integrated Calibration Execution Architecture.

The instruction then specified nine concrete requirements (documentation correction, a generic
intent-based boundary driven by `CalibrationManager`/Geometry V5/Safe Actuator policy+runtime/fake
backends only, explicit types for session context / intent / target reference / result / restore
intent, no persistence, no live motion reachable, LF V25 test-only, ten named adversarial tests, and
an explicit permission to stop once the boundary was defined rather than invent persistence). Every
one of those requirements is addressed below.

## 1. Documentation correction

`DEVELOPMENT_GATES.md`'s calibration gate previously said:

> `TO_IMPLEMENT` — the calibration execution engine (the 18 recovered phases)

This conflated the historical oracle with the future architecture, which V3 §15.11 forbids. Fixed:
the entry now states the LF V25 18-phase sequence remains `HISTORICAL_REPLAY`/oracle, usable only in
regression tests, and records the new generic boundary as `IMPLEMENTED / OFFLINE TESTED` with a
pointer to this document. `ROADMAP.md` was not found to repeat the stale wording and needed no
change.

## 2–3. The generic, intent-based boundary

New files, `src/calibration/CalibrationExecutionEngine.{h,cpp}`. Neither references
`CalibrationPhase` anywhere — enforced by static audit, not just by convention (see below).

Layering (unchanged from `ActuatorWritePolicy.h`'s own diagram, now realized one level up):

```text
CalibrationManager          session lifecycle + evidence          [UNCHANGED]
     |  (session_active, origin, lease — a snapshot, never cached here)
     v
CalibrationExecutionEngine   <- THIS: intent -> operation routing
     |
     v
SafeActuatorPolicy           the decision core                    [UNCHANGED]
     |
     v
ActuatorRuntime               the offline runtime adapter          [UNCHANGED, I4]
     |
     v
ActuatorBackend                fake/test only — no production impl
```

Established types, per the operator's minimum list:

- **execution/session context** — `CalibrationExecutionContext` (`session_active`, `origin`,
  `lease`, `mode`): a snapshot the caller already holds, never cached or advanced by this class.
- **current calibration operation/intent** — `CalibrationIntent`
  (`NONE`/`CONTACT_PROBE`/`AUXILIARY_MOVE`/`DIRECTION_VERIFY`/`RESTORE`/`ABORT`), deliberately not
  the 18-phase sequence.
- **geometry-selected calibration-safe target/plan reference** — `CalibrationExecutionRequest`
  carries the endpoint *key* (`leg`/`joint`/`side`), the exact same reference
  `ActuatorCommand.endpoint_*` and `CalibrationGeometryProfile::findEndpoint()` already use. It
  deliberately does **not** carry a resolved numeric target: `target_urad` stays at its zero
  default, because computing one would require either the accepted raw↔q transform (absent) or
  reproducing `SafeActuatorPolicy::evaluateEndpointPlan()`'s own reviewed arithmetic — a second copy
  this design avoids by construction.
- **required authority/operation class** — `operationForIntent()`, a pure total function. No
  separate "required owner" concept was added: `SafeActuatorPolicy::operationPermittedForOwner()`
  already owns that table, and duplicating it here was rejected as exactly the kind of second copy
  the operator's instructions warn against.
- **execution result/fault** — `CalibrationExecutionResult` (`CalibrationExecutionOutcome` plus,
  when routed through the policy, the real `WriteDecision` and `ExecuteResult` — never a third,
  parallel fault taxonomy).
- **restore intent** — reuses the existing `RestorePlan` concept's meaning (`CalibrationDomain.h`:
  "State only. No motion is produced by this model.") via a dedicated, categorically non-executing
  `CalibrationIntent::RESTORE` path.
- **SAFE_OFF/de-escalation outcome** — inexpressible by construction, the same idiom
  `ActuatorWritePolicy.h` and `ActuatorRuntime.h` already use for it: there is no `CalibrationIntent`
  value for a torque removal, and static audit fails the build if this file ever names one.

`CalibrationExecutionEngine::execute()` calls `SafeActuatorPolicy::plan()`/`commit()` (via
`ActuatorRuntime::execute()`) unchanged — zero lines of `ActuatorWritePolicy.cpp` or
`ActuatorRuntime.cpp` were modified. It does not duplicate `CalibrationManager`: it takes a context
snapshot per call and never touches session lifecycle, evidence, or `CalibrationManager` itself.

## 4. Lifecycle rules preserved exactly

`RESTORE`, `ABORT` and an unrecognised/`NONE` intent are all categorically non-executing: `execute()`
returns for each of them **before** constructing an `ActuatorCommand` or touching
`policy_`/`runtime_`. This is what makes "authority loss → zero restore motion" true by construction
rather than by a checked exception — proven directly in the test suite by calling `RESTORE` with a
default-constructed (i.e. "lost") context and confirming the fake backend's call count stays zero,
then repeating with a fully live, valid session and confirming it is still zero. `ABORT` is treated
identically and produces a distinct outcome (`ABORT_IS_LIFECYCLE_NOT_MOTION` vs.
`RESTORE_ACKNOWLEDGED_NO_MOTION`) — the two are never collapsed.

Authority loss for the three *executable* intents (`CONTACT_PROBE`/`AUXILIARY_MOVE`/
`DIRECTION_VERIFY`) is handled by delegation, not reimplementation: every `plan()`/`commit()` call
re-reads the live arbiter, exactly as `ActuatorWritePolicy.cpp`'s own comment states ("Anything that
can change between the two — authority, generation, inhibit, mode — is read from the arbiter here,
never from the transaction").

## 5. Persistence/promotion — untouched, as instructed

No durable storage, no promotion transaction, no EEPROM semantics, no implicit persistent write
path. `CALIBRATION_SOURCE_PRECEDENCE.md` §7's `TO_DESIGN` marker (`PROVISIONING` vs. a separate
transaction) is unchanged.

## 6. Physical execution — unreachable, and now doubly audited

- `MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED` stays `0` (untouched).
- No production `ActuatorBackend` was created (none existed before this gate either — see I4).
- `target_urad`/`target_tick` are never computed from geometry by this class — see §2–3 above.
- New `check_calibration_execution_engine_boundaries()` in `scripts/static_audit.py`:
  - fails the build if `CalibrationExecutionEngine.h`/`.cpp` include `<Arduino.h>`, reference
    `Serial.`, `millis(`, `ServoBus`, `CalibrationPhase`, `safeOff` or `EnableTorque`;
  - fails the build if the identifier `CalibrationExecutionEngine` appears anywhere outside
    `src/calibration/` or the offline test suite — so neither `Controller.cpp` nor
    `CommandRouter.cpp` can wire it in without the audit catching it immediately.
  - `check_actuator_runtime_boundaries()` (I4) was extended to allow `src/calibration/` as a
    legitimate `ActuatorRuntime` consumer, since `#include` hides a transitive reference from a
    textual scan of `Controller.h` — the new engine-specific check above closes exactly that gap by
    auditing `CalibrationExecutionEngine` itself, not just what it depends on.
- Confirmed independently by the build: both hardware profiles compile to the exact same byte counts
  as the I4 baseline (978,195 B `USB_ONLY`, 978,751 B `ROBOT_POWERED`) — the linker dead-code-
  eliminates the entire engine because nothing references it.

## 7. LF V25 stays test/oracle-only

No file under `src/` references `CalibrationPhase`, `LfSessionState`, or any LF V25 archive path.
The 18-phase sequence continues to be exercised only by `test_calibration_domain.cpp`'s existing
replay (702 checks, unchanged). No `CalibrationReplayEngine` was created.

## 8. Adversarial offline tests

New `scripts/tests/test_calibration_execution_engine.cpp`, linking the real engine, the real
`SafeActuatorPolicy`, the real `ActuatorRuntime` and the real `ActuatorAuthorityArbiter` against a
fake backend. **72 checks, 0 failures.** Every fixture value was verified against the real compiled
geometry data (`CalibrationGeometryProfileData.h`) before writing the assertion, not assumed — all
72 checks passed on the first real compile-and-run. Covers all ten named cases:

| # | Case | Proof |
|---|---|---|
| 1 | authority loss → no restore command | `RESTORE` with a default (lost) context, then with a fully live one — zero backend calls either way |
| 2 | stale authority generation rejected | released-then-regranted lease, first (stale) lease → `REJECT_STALE_GENERATION` |
| 3 | diagnostic endpoint cannot become executable | LF HIP MIN_SIDE (`DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS` in the real compiled data) → `REJECT_ENDPOINT_NOT_EXECUTABLE` |
| 4 | stale/wrong geometry provenance rejected | tampered provenance hash → `REJECT_GEOMETRY_PROVENANCE`; unbound → `REJECT_NO_GEOMETRY_PROFILE` |
| 5 | historical replay cannot promote operational evidence | `origin=HISTORICAL_REPLAY` refused by this layer before `policy.plan()` is even called (`counters().plans == 0`) |
| 6 | no q0/current transform → no raw target | LF UPPER MIN_SIDE (executable, no parking, real `contact`/`urdf` bounds verified to admit a zero target) with an empty transform table → `REJECT_NO_ACCEPTED_TRANSFORM` |
| 7 | `CALIBRATION` contact probe allowed only under `CALIBRATION` | `DIAGNOSTICS`/`QC`/`PROVISIONING` owners → `REJECT_OPERATION_NOT_PERMITTED`; `CALIBRATION` reaches geometry-level decisions instead (tests 3/4/6) |
| 8 | `MOTION` cannot execute contact probe | `MOTION`+`RUN` → `REJECT_OPERATION_NOT_PERMITTED` |
| 9 | abort, restore intent and SAFE_OFF remain distinct | `RESTORE`/`ABORT` produce two different, separately named outcomes; SAFE_OFF has no `CalibrationIntent` value to test at all |
| 10 | unknown/unsupported execution intent fails closed | `NONE` and a corrupted enum value (200) both → `REJECT_UNKNOWN_INTENT`, policy never reached |

## 9. Where I5 stops

Per the operator's explicit instruction: no additional production behaviour is implemented beyond
the generic boundary above without either deciding the `TO_DESIGN` persistence question or requiring
current hardware/q0 evidence — neither of which this gate does. Remaining calibration work stays
explicitly:

- `TO_DESIGN` — persistence/promotion ownership (`PROVISIONING` vs. a separate transaction).
- `TO_IMPLEMENT` — direction measurement (`MEASURED_CANDIDATE`/`ACCEPTED`; no historical mechanism
  exists to recover it).
- `HARDWARE_TO_TEST` — everything requiring a live session: q0 capture, an admitted transform, an
  admitted limit, and therefore every path this gate's tests deliberately stopped short of `ACCEPT`
  for the three executable intents.

## Verification

- Offline suites: **13 host suites, 5695 checks** (previous baseline 12 / 5623, +72 from this gate).
- Static audit: **PASS, 85 files** (+3: the two engine files plus the new test suite), 0 findings.
- `USB_ONLY`: PASS — 978,195 B flash / 52,420 B RAM (unchanged from I4).
- `ROBOT_POWERED`: PASS — 978,751 B flash (unchanged). Default `USB_ONLY` artifact restored
  afterward.

## Outcome

`I5 = PASS`. Proceeding to `I6` — HostLink semantic layer.
