# MATDOG calibration — permanent maintenance capability requirement

**Registered:** 2026-09-22 · **Status:** ARCHITECTURAL REQUIREMENT — PERMANENT
**Audited against:** `feat/controller-calibration-bootstrap-v1` @ `a9d9e4b` (frozen)

---

## 1. The requirement

MATDOG calibration is **not** a one-shot procedure built to reach the first stand.
`CalibrationManager` and the future Calibration Execution Engine are a **permanent firmware
capability in `MAINTENANCE`**. The first stand is merely the first consumer of the current
accepted calibration.

The architecture must be able to support, **without being rewritten**:

```text
verify-only maintenance
q0 recapture
direction verification
contact verification / calibration
single-joint recalibration
single-leg recalibration
full-leg-system recalibration
post-servo-replacement calibration
post-mechanical-repair verification
geometry/model-vs-hardware diagnostics
```

None of these needs its UI or command surface today. What must hold today is that the **domain
model, session model and evidence lifecycle do not assume one global monolithic calibration**.

Two invalidation rules are part of the requirement:

| Event | Required consequence |
|---|---|
| A servo is replaced | Invalidate **that joint**. Independent evidence for other joints must not be forced to be discarded. |
| Geometry / URDF / profile changes | Invalidate **every** piece of evidence that depends on that provenance. |

### The layering, permanently

```text
MAINTENANCE
  → CalibrationManager
    → future Calibration Execution Engine
      → ActuatorAuthority::CALIBRATION
        → Safe Actuator Layer
          → ServoBus

SAFE_OFF — independent, outside all of it
```

### Geometry Compiler V5 keeps its role

Geometry Compiler V5 remains the **offline specialist compiler of calibration plans**, including
after ROS 2 / MoveIt are adopted for general pose, planning and motion. Those solve a different
problem: the Controller's contract is to consume a *prevalidated, provenance-bound calibration
geometry profile* and execute only plan primitives from it. A general-purpose planner does not
replace a compiler whose output is an authorization artifact.

### Scheduling is not removal

```text
NOT_REQUIRED_BEFORE_FIRST_STAND  !=  REMOVED_FROM_CALIBRATION
```

The decision in [`CALIBRATION_BOOTSTRAP.md`](CALIBRATION_BOOTSTRAP.md) §8 to defer some contact
probes past the first stand is a **scheduling** decision only:

- the **8 upper-leg contacts** remain part of final calibration;
- the **16 hip / lower-leg contacts** remain a maintenance and model-validation capability, to be
  enabled only when a dedicated geometry-safe plan exists for beyond-URDF travel.

---

## 2. Architecture audit — what already holds

Verified against the frozen tree, not assumed.

| # | Requirement | Status | Evidence |
|---|---|---|---|
| 1 | Evidence is per-joint, not global | **HOLDS** | `Q0Evidence`, `ContactEvidence`, `JointLimit`, `JointTransform` are all keyed by `JointIdentity` = leg + joint kind + **physical unit** |
| 2 | Servo replacement invalidates only that joint | **HOLDS** | `identityPermitsEvidenceReuse()` requires slot **and** physical unit. A replacement changes the unit label, so that joint's stored limit/transform stops matching while every other joint's record is untouched. Already covered by tests. |
| 3 | Sessions are leg-scoped, not system-wide | **HOLDS** | `CalibrationSessionStatus.leg`; `recordContact()` rejects evidence whose leg differs from the session's |
| 4 | No monolithic completion criterion | **HOLDS** | `completeSession()` gates on the reported execution phase, **not** on a contact count. `contacts_recorded` is a counter, never a gate — one contact and twenty-four take the same path. |
| 5 | Partial / staged calibration is expressible | **HOLDS** | The lifecycle `MEASURED → CANDIDATE → ACCEPTED → PROMOTED` is per-record; promotion is per-record, never global |
| 6 | Verify-without-promote is possible | **HOLDS** (by construction, see blocker B3) | A live session may record evidence and simply never promote it |
| 7 | The permanent layering is enforced, not documented | **HOLDS** | `static_audit.py` fails the build if `CalibrationManager` gains a transport, if `ServoBus` references the arbiter or the Safe Actuator Layer, or if the `@SERVO SAFE_OFF` branch consults authority or mode |
| 8 | Geometry stays an offline compiler; the device only consumes | **HOLDS** | `CalibrationGeometryProfile` has no mesh, no FK, no collision maths, no floating point; the audit enforces all four |
| 9 | Repeatable re-entry is not blocked | **HOLDS** | `reset()` clears session, transforms, limits and the parked flag; a new session may start from `NO_SESSION` at any time |

**No change to the frozen branch is required for items 1–9.** The bootstrap work did not close
any of the ten future capabilities.

---

## 3. Real blockers — documented, not fixed here

### B1 — Evidence is not bound to geometry provenance · **BLOCKER**

`JointLimit` and `JointTransform` each carry `identity`, `state` and `origin`. **Neither carries
any link to the `GeometryProvenance` it was derived under.**

Consequence, precisely:

- A URDF / mesh / profile change makes `CalibrationGeometryProfile::provenanceMatches()` fail, so
  no **new** plan-bound move is authorised. That half is fail-closed and correct.
- But limits and transforms already admitted **stay admitted**, and `POSITION_COMMAND` consults
  `limits_` with no geometry check at all — by design, because a joint bound was conceived as a
  machine measurement rather than a model fact.

That conception is incomplete. Both records *are* geometry-dependent once they exist:

- `JointTransform.q0_tick` is captured at "the nominal URDF q=0 pose" — a pose the URDF defines;
- a safe operational limit descends from a contact target that came from the V5 plan, and from a
  MODEL↔REAL comparison whose model half is the URDF.

So the requirement *"a geometry/URDF/profile change must invalidate all evidence that depends on
that provenance"* is **not met today**.

Smallest correct fix, **not implemented here**: give both records a geometry provenance
identifier, require it to match the bound profile in `admit()` **and** in `find()`, and make
`bindGeometry()` of a different profile drop non-matching entries. Roughly one field, two
predicates and their tests. It belongs to the phase that first produces a real limit or
transform — there is nothing to invalidate until then.

### B2 — The physical-unit ↔ joint ↔ bus binding is compile-time · **ACCEPTED DESIGN, needs a decision**

The binding lives in the generated `CalibrationGeometryProfileData.h` (reduced from
`MATDOG_SERVO_ALLOCATION.yaml`). The installed firmware's own `kCanonicalServos` table is weaker
still: `bus_id → joint` only, with **no physical unit field at all**.

Consequence: **post-servo-replacement calibration requires regenerating the profile and
rebuilding the firmware.** Joint-scoped invalidation still behaves correctly — the new unit
label simply matches nothing — but there is no runtime re-binding.

This may well be the right design: the allocation is reviewed configuration, and letting a
runtime command rewrite unit identity would undo the very protection that makes B2's
invalidation work. It is recorded here so it is an **accepted decision** rather than a surprise
discovered during a servo swap.

### B3 — No explicit session intent · **MINOR**

`CalibrationOrigin` distinguishes `HISTORICAL_REPLAY` from `LIVE_SESSION`, but there is no
`VERIFY_ONLY` vs `RECALIBRATE` intent. Verify-only maintenance is therefore achievable by
convention — record evidence, never promote it — but it is **not enforceable**: nothing stops a
session that was meant to verify from promoting.

Low severity today because promotion is an explicit act with its own gate. Recorded so a future
maintenance surface does not have to retrofit the concept into the lifecycle.

### B4 — The installed firmware cannot report unit identity · **BLOCKER for H0**

Covered in [`H0_LEG_PREFLIGHT_RUNBOOK.md`](H0_LEG_PREFLIGHT_RUNBOOK.md) §2. It is listed here
because it is the same root cause as B2: unit identity is not a runtime concept in the firmware.

---

## 4. Summary

```text
PERMANENT MAINTENANCE CALIBRATION CAPABILITY   REQUIRED, RECORDED
DOMAIN / SESSION / EVIDENCE SCOPING            joint- and leg-scoped: HOLDS
SERVO-REPLACEMENT JOINT-SCOPED INVALIDATION    HOLDS
GEOMETRY-CHANGE CASCADE INVALIDATION           BLOCKER B1 — not met
RUNTIME UNIT RE-BINDING                        BLOCKER B2 — compile-time by design
EXPLICIT VERIFY-ONLY INTENT                    BLOCKER B3 — minor
THE TEN FUTURE CAPABILITIES                    none closed off by current architecture
```

No code changed to register this requirement.
