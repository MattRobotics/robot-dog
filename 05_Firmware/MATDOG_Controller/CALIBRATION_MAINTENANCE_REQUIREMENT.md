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

### What a servo replacement actually invalidates

```text
q0              CURRENT INSTALLATION CALIBRATION DATA   INVALIDATED
motorDirection  CURRENT URDF / HARDWARE CONTRACT DATA   RETAINED
```

A same-type ST3215 mounted in the same orientation changes the physical unit, the
`PositionOffset` baseline and the raw q0 installation. It does **not** change the servo model,
the mounting orientation, the joint mechanical architecture, the URDF joint axes or the canonical
`motorDirection` — all of which were validated on real hardware and remain valid.

So **direction is not re-measured during normal calibration or after an equivalent servo
replacement.** It is read from the current URDF via `jointDirection()`, and `JointTransform`
carries no direction field at all. Direction is reconsidered only when the servo's physical
orientation, the transmission topology, the servo type or encoder convention, the URDF joint axis
or `motorDirection` change, or when contradictory hardware evidence appears — and every one of
those moves the geometry provenance tag, which invalidates the affected transforms automatically.

`DIRECTION_VERIFY` is retained as **OPTIONAL / DIAGNOSTIC ONLY**, never required for calibration
acceptance or stand authorization; `check_direction_is_contractual` fails the build if a
calibration path consults its budget.

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
| 6 | Verify-without-promote is possible | **HOLDS** (by construction; explicit intent deferred, B3) | A live session may record evidence and simply never promote it |
| 7 | The permanent layering is enforced, not documented | **HOLDS** | `static_audit.py` fails the build if `CalibrationManager` gains a transport, if `ServoBus` references the arbiter or the Safe Actuator Layer, or if the `@SERVO SAFE_OFF` branch consults authority or mode |
| 8 | Geometry stays an offline compiler; the device only consumes | **HOLDS** | `CalibrationGeometryProfile` has no mesh, no FK, no collision maths, no floating point; the audit enforces all four |
| 10 | Direction survives a servo replacement | **HOLDS** | Direction is resolved from the bound profile's URDF record, not stored as transform evidence. A replacement invalidates q0 and leaves direction readable and unchanged. |
| 9 | Repeatable re-entry is not blocked | **HOLDS** | `reset()` clears session, transforms, limits and the parked flag; a new session may start from `NO_SESSION` at any time |

**No change to the frozen branch is required for items 1–9.** The bootstrap work did not close
any of the ten future capabilities.

---

## 3. The blockers — B1 fixed, B2 confirmed, B3 deferred, B4 resolved as a semantic

### B1 — Evidence is not bound to geometry provenance · **FIXED 2026-09-22**

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
that provenance"* was **not met**. It is now.

#### What was implemented

`GeometryProvenanceTag` — a compact identity for one `GeometryProvenance`, FNV-1a over the six
SHA-256 hashes with field separators. It is an **identity tag, not a security digest**: the
question is accidental drift, not an adversary, and the six hashes remain the authority.

Both records carry one, and the two lookups are now different questions:

| Call | Answers |
|---|---|
| `findAny(joint)` | *is this on record?* — identity only. A superseded record stays here. |
| `find(joint, tag)` | *is this CURRENT operational evidence?* — identity **and** geometry. |

`admit()` refuses a record that does not name its geometry: one that cannot say which model it
was measured under could never be invalidated when that model changes. `find()` fails closed on
`kNoGeometryProvenance`, so an unbound policy matches nothing. `currentGeometryTag()` requires
`provenanceMatches()` — *some* geometry being loaded is not *the expected* geometry being loaded.

A record from another model produces `REJECT_EVIDENCE_GEOMETRY_MISMATCH`: distinguishable from
"no evidence at all", which is what makes the state diagnosable rather than merely safe.

#### The three axes stay separate

```text
physical unit identity   WHICH SERVO       JointIdentity      survives a geometry change
calibration provenance   HOW GOOD          state + origin     survives both
geometry provenance      WHICH MODEL       the tag            survives a servo change
```

A servo swap invalidates the first and leaves the other two intact. A URDF change invalidates
the third and leaves the other two intact. Neither cascades into the other.

#### No persistence migration

There is nothing to migrate: no accepted calibration evidence exists yet, so both tables start
empty on every boot. This fix landed **before** the first real limit or transform, which is the
only moment at which it could be done without a migration.

Enforced by `check_evidence_geometry_binding` and six mutations, each removing one guarantee and
required to be caught.

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

### B3 — No explicit session intent · **DEFERRED, by decision**

`CalibrationOrigin` distinguishes `HISTORICAL_REPLAY` from `LIVE_SESSION`, but there is no
`VERIFY_ONLY` vs `RECALIBRATE` intent. Verify-only maintenance is therefore achievable by
convention — record evidence, never promote it — but it is **not enforceable**: nothing stops a
session that was meant to verify from promoting.

**Deliberately not implemented now.** Promotion is already an explicit act with its own gate, so
nothing today can be promoted by accident, and inventing a session-intent enum before there is a
maintenance surface to consume it would be an abstraction with no consumer.

It is recorded as a **future Calibration Execution Engine / Maintenance requirement**: when the
first maintenance command surface is designed, `VERIFY_ONLY` versus `RECALIBRATE` should enter
the session model at that point rather than being retrofitted into the evidence lifecycle.

### B4 — Unit identity is not observable · **RESOLVED AS A SEMANTIC, not as a measurement**

An ST3215 exposes **no unit serial on the bus**. `M33` is a MATDOG label applied during the
2026-08-27 provisioning campaign; nothing the servo can say confirms it. So this was never going
to be solved by reading harder.

It is resolved by making the semantics explicit instead. `CanonicalServo` now carries the
expected physical unit from `MATDOG_SERVO_ALLOCATION.yaml`, the preflight reports it as
`expected_physical_unit`, and the static audit fails the build if that column is ever renamed to
suggest an observation. The only identity a servo can actually supply is *"it answered at this
address"*, and that is reported separately as `observed_bus_id`.

The binding itself is held by labelling and assembly discipline — which is how it was
established in the first place.

---

## 4. Summary

```text
PERMANENT MAINTENANCE CALIBRATION CAPABILITY   REQUIRED, RECORDED
DOMAIN / SESSION / EVIDENCE SCOPING            joint- and leg-scoped: HOLDS
SERVO-REPLACEMENT JOINT-SCOPED INVALIDATION    HOLDS
GEOMETRY-CHANGE CASCADE INVALIDATION           B1 — FIXED 2026-09-22
RUNTIME UNIT RE-BINDING                        B2 — compile-time BY DESIGN, confirmed
EXPLICIT VERIFY-ONLY INTENT                    B3 — DEFERRED to the Execution Engine
OBSERVABLE UNIT IDENTITY                       B4 — not observable; expected-vs-observed
                                                    semantics made explicit instead
THE TEN FUTURE CAPABILITIES                    none closed off by current architecture
```

The H0 preflight that consumes all of this is a **permanent MAINTENANCE capability**, not a
one-shot gate: re-runnable, holding no gate state, deciding nothing about motion. H0 is simply
its first consumer.
