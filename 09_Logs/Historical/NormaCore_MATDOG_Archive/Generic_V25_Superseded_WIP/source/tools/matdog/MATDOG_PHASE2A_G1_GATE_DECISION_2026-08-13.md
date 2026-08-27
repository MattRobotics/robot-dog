# MATDOG — Phase 2A G1 Gate Decision
## Reviewed architectural rulings — 2026-08-13

## Status

```text
P2A-G0 = CLOSED / PASS
P2A-G1 = CLOSED / PASS
BLOCKERS = 0
```

Gate authority: ChatGPT (architecture/gate owner) + Matteo.
Executor: Claude Code Opus (Phase 2A implementation engineer).
Later independent adversarial reviewer: Codex.

The audit these rulings resolve is preserved verbatim at:

```text
tools/matdog/MATDOG_PHASE2A_G0_G1_EXECUTOR_REPORT_2026-08-13.md
```

Base for this branch:

```text
MattRobotics/norma-core
origin/main
4a8ed6337261553b79c928975808d294c9ca723b
```

Immutable LF V25 oracle:

```text
release/matdog-lf-calibrator-v25
f87dd1fbc7e8100d275c74f9af448642f3429680
```

The G0 audit established that every LF calibrator source file is **byte-identical**
between that release ref and canonical main:

```text
matdog.rs             65c16f3c2f551b975570a89cf5e5934cf0ca48a2
matdog_test.rs        b6e6b6e767569081978d48d871300fe79af2d42e
mod.rs                63384ffe5632868fefbbb9e2c26338da329bb180
port.rs               ae86f1f44441ccfc4b50454e6171a3e3867eca53
bin/matdog_lf_freeze.rs  67cebee386f9b366ab380f456e98024b2b29b92a
```

---

## D1 — CONTACT SEMANTICS

**Ruling: preserve LF V25 behavior exactly.**

The historical sentence

```text
ContactConfirmed -> STOP ADVANCING IMMEDIATELY
```

is **clarified**, not changed, to:

```text
FINAL ACCEPTED CONTACT -> STOP ADVANCING IMMEDIATELY
```

### Rationale

The G1 audit (report §4.7, AMB-1) found that a detector-level `ContactConfirmed` is
not by itself the final accepted contact. On a fine metrology pass, LF V25 applies a
bounded friction/chamfer qualification **before** promoting a detector verdict to a
final accepted contact:

```text
matdog.rs:4015-4041   approach_with_scout          ContactConfirmed branch
matdog.rs:4123-4139   confirm_kinematic_plateau    adaptive branch
```

A confirmed contact shallower than the already physically measured coarse scout by
more than `FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS` (= `FINE_STEP_TICKS` = 8) is
qualified as a friction/chamfer plateau, and the probe takes one further bounded step.

This qualification is bounded by, and never widens:

```text
the mechanical guard (URDF limit + GUARD_OVERSHOOT_TICKS = 64)
the adaptive contact acceptance corridor
the coarse scout depth, which is itself a real prior hardware contact
```

Once a contact is **finally accepted**, LF V25 calls `stop_pressure()` — goal position
is set to the present position — and returns. That is the invariant.

### Binding constraints on G3

```text
MUST NOT remove the V25 friction plateau bypass
MUST NOT retune FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS
MUST NOT generalize the bypass beyond its current fine-pass scope
MUST NOT alter its behavior in any observable way
MUST NOT change any detector threshold
```

`HybridContactDetector`, `HybridContactConfig`, `contact_acceptance_bounds`,
`adaptive_contact_acceptance_bounds`, `probe_tracking_error_limit`,
`confirm_kinematic_plateau` and `stop_pressure` are carried forward behaviorally
unchanged.

The bypass must be carried forward **explicitly and named**, never inherited by
accident. G3 documentation must state that the two-stage semantics
(detector verdict → qualification → final accepted contact) is deliberate.

**AMB-1 is CLOSED.**

---

## D2 — PER-LEG GEOMETRY

**Ruling: `LegCalibrationSpec` must model each leg and each endpoint independently.**

### Rationale

The G1 audit (report §5 rows 33–35, AMB-2) established that all twelve entries of
`JOINT_SPECS` currently share three constant pairs:

```text
HIP_MIN_DELTA   -512   HIP_MAX_DELTA   +512
UPPER_MIN_DELTA -597   UPPER_MAX_DELTA +1394
LOWER_MIN_DELTA -1047  LOWER_MAX_DELTA +427
```

Only `motor_id` and `direction` vary per leg. Front/hind and left/right distinctions
survive **only** inside two hard-coded helpers:

```text
prerequisites_for()          front legs park the contralateral rear upper; hind legs park nothing
hip_upper_clearance_delta()  UPPER_85_DELTA on (LF,Max) and (RF,Min); UPPER_90_DELTA elsewhere
```

### Binding constraints

```text
Numerically equal current URDF limits MUST NOT imply structural
LF/RF/RH/LH geometry symmetry.

Each leg and each endpoint carries its own value, even when the values coincide today.

A mechanical lift of the shared constants into per-leg data is FORBIDDEN if it
represents the sharing itself as structural runtime truth.
```

Two domains must be explicitly separated in the spec:

```text
URDF executable joint domain
    the domain from which a GoalPosition may legally be derived

Geometry V5 contact/endstop evidence
    geometric contact prediction; NOT an executable authorization
```

`GEOMETRIC CONTACT != MOTION AUTHORIZATION` remains permanent.

FRONT/HIND and LEFT/RIGHT distinctions remain data/model driven. String naming
(`lf_`, `rf_`, `rh_`, `lh_`) must never be used to infer structural runtime truth
where model metadata already exists.

**AMB-2 is CLOSED as a design constraint on G2.**

---

## D3 — LF ORACLE HIERARCHY

**Ruling: three distinct oracles with three distinct roles. None is authoritative
outside its own role.**

```text
1. BEHAVIORAL REGRESSION ORACLE
   immutable release/matdog-lf-calibrator-v25
   f87dd1fbc7e8100d275c74f9af448642f3429680
   plus its tests (matdog_test.rs, 88 #[test])
   -> authority for: "did the refactor change behavior?"

2. RUNTIME LF CONTACT-WITNESS ORACLE
   the immutable V25 lf_reference_contact_ticks() values
        Hip   (2535, 1617)
        Upper (1443, 3442)
        Lower (3093, 1666)
   with LF_CONTACT_WITNESS_TOLERANCE_TICKS = 24
   -> authority for: the runtime contact-witness gate, LF only

3. PHYSICAL HARDWARE EVIDENCE ORACLE
   robot-dog
   06_Software/Matdog_Core/calibration/MATDOG_LF_V25_HARDWARE_EVIDENCE_2026-08-04.json
   approved LF hardware contact angles
        HIP   MIN -42.803   MAX +39.375
        UPPER MIN -53.525   MAX +122.607
        LOWER MIN -91.846   MAX +34.277
   -> authority for: the physical truth of the LF mechanical endstops

4. README TICK TABLES  (tools/matdog/README.md)
   documentary evidence only
   -> NEVER executable or runtime authority
```

### Binding constraints

```text
Do NOT rewrite one oracle to make it numerically identical to another.
```

The tick-level deltas the audit reported between oracle 2 and the README table
(Hip MAX 17, Upper MIN 4, Upper MAX 1, Lower MAX 8 ticks) are **expected**: they are
different quantities recorded at different stages of the same validated run. They all
lie inside the 24-tick witness tolerance. That is evidence of consistency, not of a
defect.

G4 uses **oracle 1** for behavioral preservation and **oracle 2** for runtime witness
equivalence. Oracle 3 remains the physical reference. Oracle 4 is never consulted by
code.

**AMB-3 is CLOSED.**

---

## D4 — 30/85/90 PREREQUISITE POSES

**Ruling: accepted as LF V25 historical prerequisite/parking behavior, for LF
regression only.**

```text
UPPER_30_DELTA   = 341     (~29.97 deg)   contralateral rear upper parking
UPPER_85_DELTA   = 967     (85.0 deg)     side-specific hip clearance
UPPER_90_DELTA   = 1024    (90.0 deg)     horizontal upper pose
LOWER_FOLDED_DELTA = -990                 folded lower pose for HIP sweep
```

### Binding constraints

```text
ACCEPTED   as LF V25 historical prerequisite/parking behavior, preserved for LF regression
NOT        declared Geometry V5 canonical truth
FORBIDDEN  as default RF/RH/LH prerequisites
```

Future other-leg prerequisites and parking remain Geometry-V5 / model driven.

In the G2 contract these are modeled as **spec-supplied prerequisite/parking
references carrying their own provenance**, not as engine constants. The LF spec
supplies exactly these values as historical LF behavior; no other leg's spec inherits
them.

This preserves `CANONICAL V5 != G4 REPLAY`: the legacy 30/50/90-degree replay context
does not re-enter the generic runtime or spec as structural truth.

**AMB-4 is CLOSED.**

---

## D5 — CI MIGRATION

**Ruling: MATDOG CI is an architectural/safety contract, not merely a test runner.**

`.github/workflows/matdog-native-calibrator-check.yml` performs a token-and-constant
grep gate over `matdog.rs`, `matdog_test.rs`, `port.rs`, `mod.rs` and
`matdog_lf_profile.py`. `.github/workflows/matdog-native-observer-check.yml` enforces
the external-observer authority boundary by forbidden token.

### Binding constraints

Every implementation-shape assertion that must change during G3 requires an explicit,
reviewed four-part mapping:

```text
old assertion
-> invariant protected
-> new assertion
-> proof that protection was not weakened
```

```text
NEVER make CI green by deleting a gate.
NEVER make CI green by broadly weakening a gate.
NEVER loosen a forbidden-token list to accommodate a refactor.
```

The workflow migration is a **separate reviewed diff** with per-assertion
justification. It may not be bundled opportunistically into a runtime change.

Assertions the audit classified as true safety invariants (the `forbidden_source`
EEPROM/register list, `HOME_TICK 2048`, `GUARD_OVERSHOOT_TICKS 64`, the port/module
token requirements, the observer forbidden-token list, the `matdog_v2` staleness
guard) are carried forward **verbatim**.

The audit further recommends extending the staleness guard to forbid reappearance of
per-leg duplicated engines. That recommendation is accepted in principle and belongs
to the G3/CI migration diff, not to G2.

**AMB-5 is CLOSED as a process constraint.**

---

## D6 — SHA PINS

**Ruling: pinned hashes track reviewed bytes, never implementation churn.**

Affected pins in `tools/matdog/matdog_v42_pinned_launcher.py`:

```text
EXPECTED_RUNNER_SHA256            9eccb4aa88c3496e6d4e986d9de2d5fea3d8185d1bd3ea4855d1d3aaa6945613
EXPECTED_OBSERVER_SHA256          b9521f97ed0a3cf4d7f39d8712c2fb7a060fa56bbbc2a10b8709742d6b0a5167
PINNED_STATION_SHA256             df4f6965d5c6b5eaecdc7f937391392dff0a1ca1cac166ab898d9a7c530f4651
PINNED_STATION_SOURCE_COMMIT      3f6a9099ea11f90da5981d9ea2cada1c7779878b
PINNED_STATION_ARTIFACT_ID        8869874935
PINNED_STATION_ARTIFACT_ZIP_SHA256 ec7fd93805e73ea0691638cf863eda30dc0bbf5212db7bc25032ed6b270560d6
```

### Binding constraints

```text
Update a pin ONLY after the new bytes have been reviewed and accepted.
NEVER mechanically re-hash merely because an implementation changed.
```

A launcher refusing to start because a pin no longer matches is **correct behavior**,
not a defect to be worked around.

**AMB-6 is CLOSED.**

---

## D7 — GEOMETRY PROVENANCE

**Ruling: no external/spec geometry data channel may become runtime-consumable
without the provenance gate and the target-domain gate landing in the same
architectural change.**

### Current state established by G1

`matdog.rs` derives every endpoint from constants compiled into the binary. There is
no external geometry channel at all today. The only provenance binding anywhere is
`urdf_sha256`, and it is applied **after** measurement, in the Python serializer.

The runtime is therefore *accidentally* safe: no DIAGNOSTIC or UNRESOLVED target can
enter because no external target can enter. **That accident ends the moment G2's
channel is implemented.**

### Binding constraints

```text
A stale or mismatched geometry profile MUST fail closed.

DIAGNOSTIC endpoints MUST have no conversion path to an executable GoalPosition.
UNRESOLVED endpoints MUST have no conversion path to an executable GoalPosition.

The provenance gate and the target-domain gate MUST be part of the same
architectural change that introduces the external data channel.
Never "channel first, gate later".
```

Permanent semantics:

```text
UNRESOLVED != PASS
DIAGNOSTIC != EXECUTABLE
GEOMETRIC CONTACT != MOTION AUTHORIZATION
```

The eight UNRESOLVED targets under the final 3 mm reference policy
(16 PASS / 0 FAIL / 8 UNRESOLVED / 0 motion authorizations):

```text
lf_hip_joint:min        lf_lower_leg_joint:min
rf_hip_joint:max        rf_lower_leg_joint:min
rh_hip_joint:max        rh_lower_leg_joint:min
lh_hip_joint:min        lh_lower_leg_joint:min
```

Canonical Geometry V5 semantic hashes that a provenance adapter must bind:

```text
endpoint  de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e
parking   67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139
combined  0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51
```

---

## D8 — HISTORICAL RF WORKTREE

**Ruling: forensic evidence only, permanently.**

```text
/home/matteo-manicardi/MATDOG/worktrees/norma-core-rf-calibrator
branch  matdog/rf-calibrator-from-lf-v25
HEAD    b2f7dac2eab7147917fccdfde702360da82ab7de
state   2 modified tracked files, 0 untracked  (EXPECTED_DIRTY_HISTORICAL_EVIDENCE)
```

Do not reset, clean, stash, checkout, delete, commit or merge it.

### Forbidden to copy

```text
RfSessionStateMachine and RfSessionState
duplicated RF engine and duplicated RF validators
    (validate_rf_role_observation / validate_rf_active_readback / validate_rf_session_snapshot)
duplicated RF corridors
    (rf_full_joint_corridor / rf_parking_corridor / rf_passive_corridor / rf_participant_corridor)
LF spans 935 / 2004 / 1435 as RF authority
    (rf_reference_span_ticks)
RF witness derived from the LF witness
    (RF_WITNESS_TOLERANCE_TICKS = LF_CONTACT_WITNESS_TOLERANCE_TICKS)
```

`rf_reference_span_ticks` is the §14 prohibition realized in code and is the primary
reason this worktree must never be merged wholesale.

### Potentially reusable concept only

The *structure* — not the values, not the types — of anchored second-contact
prediction:

```text
own-leg anchor  (the first really-measured contact on that same leg)
+ own/model-derived span  (never an LF span)
+ intersection with the existing geometry corridor
+ never widen that corridor
+ empty intersection fails BEFORE any forward motion
```

Also retained as discipline, not as code: production predicates must be exercised
directly by tests rather than duplicated in test-only mirrors.

---

## D9 — LIVE FK

**Ruling:**

```text
NO_DEPENDENCY_VERIFIED for Phase 2A
```

Evidence: a full sweep of norma-core `software/`, `tools/` and `.github/` for
`matdog_leg_fk_live|VISUAL_ZERO|DIGITAL_ZERO_CALIBRATED|MATDOG_JOINT_CALIBRATION|leg_fk|fk_live`
returned exactly one hit, a path reference inside a historical documentation file.
Zero hits in the calibrator, port gate, Station integration, profile tooling,
observer, launcher, freeze binary or any workflow.

The robot-dog contradiction:

```text
MATDOG_JOINT_CALIBRATION.yaml:5
    calibration_status: DIGITAL_ZERO_CALIBRATED_AND_VERIFIED

06_Software/Matdog_Core/kinematics/matdog_leg_fk_live.py:56
    EXPECTED_CALIBRATION_STATUS = "VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION"
```

remains **isolated and out of Phase 2A scope**. It is owned by the robot-dog
live-FK / visual-zero workflow.

```text
The established digital zero is UNCHANGED.
It MUST NOT be altered to make any test, model or oracle agree.
MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS = 96 remains the runtime protection.
```

---

## Scope of the authorization under which this document was produced

Authorized:

```text
create branch matdog/generic-v25-full-leg-engine from 4a8ed633...
create worktree /home/matteo-manicardi/MATDOG/worktrees/norma-core-generic-v25
preserve the G0/G1 report
record these rulings
produce the P2A-G2 design contract
one local documentation-only commit
```

Not authorized:

```text
any Rust / runtime / test / CI implementation
any push
any pull request
any modification to main, robot-dog or the historical RF worktree
G3 or anything beyond it
```
