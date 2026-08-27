# MATDOG — Phase 2A Entry Handoff
## Generic V25-Derived Full-Leg Calibration Engine
### Canonical post-Phase-2A0 state — 2026-08-11

## Purpose

This is the current operational entry handoff for:

```text
PHASE 2A
Generic V25-derived full-leg calibration engine
```

The real GitHub remotes and the real ASUS filesystem remain the implementation sources of truth. Re-verify them before changes.

## Closed predecessor

```text
PHASE 1B  CLOSED
PHASE 2A0 CLOSED
```

Geometry Compiler V5 was merged through `MattRobotics/robot-dog` PR #19.

```text
reviewed PR head:
2890daf0a8ac6103d3856f208a5f042528fc0da0

squash merge commit on robot-dog/main:
f07aa094a1b78c5670cc36ef3fdb349422a38955
```

## Expected repository entry state — verify, do not assume

```text
MattRobotics/robot-dog
main expected at handoff creation:
f07aa094a1b78c5670cc36ef3fdb349422a38955

MattRobotics/norma-core
main expected at handoff creation:
f47b1ba579c623139058a8b0118648015739ab10

immutable LF V25 release:
release/matdog-lf-calibrator-v25
f87dd1fbc7e8100d275c74f9af448642f3429680
```

The LF release must never be rewritten.

## Historical RF worktree — preserve as evidence

Verify on the ASUS:

```text
/home/matteo-manicardi/MATDOG/worktrees/norma-core-rf-calibrator
branch: matdog/rf-calibrator-from-lf-v25
historical checkpoint: b2f7dac2eab7147917fccdfde702360da82ab7de
```

It is evidence only. Do not reset/delete it and do not merge its duplicated RF architecture wholesale.

## Required Phase 2A architecture

```text
LegSessionStateMachine
+
LegCalibrationSpec
```

Conceptually:

```text
Generic V25 Full-Leg Engine
        |
        +-- LF LegCalibrationSpec
        +-- RF LegCalibrationSpec
        +-- RH LegCalibrationSpec
        +-- LH LegCalibrationSpec
```

Do not create independent LF/RF/RH/LH engines.

The shared engine owns behavior. Specs/profiles own leg-specific data.

## Phase 2A scope is offline only

Not authorized in this phase:

```text
physical servo movement
Station motion execution
serial probing
EEPROM writes
Position Offset changes
RF/RH/LH hardware calibration
persistent q0 freeze
```

Phase order:

```text
PHASE 2A — generic engine, offline
↓
PHASE 2B — final path/parking safety integration
↓
PHASE 2C — complete offline validation
↓
future hardware — RF -> RH -> LH
```

## Geometry Compiler V5 contract

Corrected canonical semantic hashes:

```text
endpoint
  de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e
parking
  67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139
combined
  0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51
```

Canonical V5:

```text
24/24 geometric contacts
24/24 empty canonical search contexts
24/24 endpoint/planner consistency
6 direct-target path obstructions
18 collision-free direct paths
```

Parking:

```text
18 NOT_NEEDED
6 FEASIBLE_1DOF_PLAN_FOUND
24/24 complete sequences
94 evaluated 1DOF candidates
0 requiring 2DOF
```

Permanent lesson from the independent audit:

```text
CANONICAL V5 != G4 REPLAY
```

Frozen G4 30/50/90-degree context is non-canonical replay evidence only. It must not re-enter the generic runtime/spec as structural truth.

## Safety-policy state — mandatory future input

Final external 3 mm reference policy:

```text
16 PASS
0 FAIL
8 UNRESOLVED
0 motion authorizations
```

All eight `UNRESOLVED` are outside the executable URDF target domain:

```text
lf_hip_joint:min
lf_lower_leg_joint:min
rf_hip_joint:max
rf_lower_leg_joint:min
rh_hip_joint:max
rh_lower_leg_joint:min
lh_hip_joint:min
lh_lower_leg_joint:min
```

Permanent contract:

```text
UNRESOLVED != PASS
DIAGNOSTIC != EXECUTABLE
GEOMETRIC CONTACT != MOTION AUTHORIZATION
```

The generic engine/spec/serializer/consumer must make it impossible for a diagnostic outside-limit target to silently become executable.

## LF V25 remains the only hardware oracle

Final hardware contacts:

```text
HIP   MIN -42.803°   MAX +39.375°
UPPER MIN -53.525°   MAX +122.607°
LOWER MIN -91.846°   MAX +34.277°
```

RF/RH/LH remain geometry-only until later hardware validation.

Never copy LF measured spans as RF/RH/LH endpoint coordinates.

Permanent detector rule for future hardware phases:

```text
ContactConfirmed -> STOP ADVANCING IMMEDIATELY
```

## Mandatory LF V25 classification before refactor

Before moving any materially relevant LF constant/helper/state transition, classify it as:

```text
A — truly generic calibration behavior
B — geometry/profile/spec data
C — global ST3215 hardware/safety parameter
D — historical LF-only evidence / witness
```

Review the mapping before refactor.

Do not alter a detector or safety threshold merely to make another leg pass.

## q0 / scale / affine distinctions

Keep separate:

```text
physical encoder/transmission angular scale
q0 offset
geometry/endstop mismatch
fitted affine diagnostic normalization
```

A fitted affine model must never erase raw model-vs-hardware disagreement.

LF V25 remains immutable historical evidence.

## Live-FK status mismatch — entry risk

A pre-existing contradiction remains in `robot-dog`:

```text
tracked state:
DIGITAL_ZERO_CALIBRATED_AND_VERIFIED

live-FK loader expectation:
VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION
```

Phase 2A G0 must determine whether the generic calibrator depends on this loader/contract.

If yes, resolve the semantic contradiction before relying on live FK. Do not change the established digital zero merely to make tests green.

## FRONT/HIND and geometry rules

FRONT and HIND are not interchangeable by convention. Prerequisites and parking must remain model/profile-driven.

## Stale-geometry provenance must fail closed

The eventual runtime/profile contract must bind calibration evidence to geometry provenance, conceptually including:

```text
robot-dog geometry revision
URDF SHA
collision-mesh manifest SHA
Geometry V5 semantic SHA
calibration software SHA
hardware evidence/log SHA
```

A stale or mismatched geometry profile must be rejected.

## Permanent ST3215 / Station constraints

```text
GoalPosition remains unsigned 0..4095
signed-wrap forbidden
Station remains sole serial owner during motion
```

No physical movement without successful offline gates and explicit current-session authorization.

No EEPROM Position Offset change without separate explicit authorization.

## Phase 2A gates

### P2A-G0 — read-only entry audit

Verify current remotes, PRs, branches, real ASUS worktrees, dirty/untracked state, immutable LF release, historical RF worktree, durable MATDOG workflows, Geometry V5 artifacts/hashes and live-FK dependency.

No modification before the G0 report is reviewed.

### P2A-G1 — LF V25 behavioral inventory

Read `matdog.rs`, tests, Station integration, profile/observer/freeze paths and classify relevant LF material A/B/C/D.

### P2A-G2 — generic spec contract

Design the minimal data-driven `LegCalibrationSpec` or equivalent. Preserve motor identity/direction, contact side/order, geometry corridors/bands, target-domain status, prerequisite/parking references, restore semantics and geometry provenance.

### P2A-G3 — generic engine foundation

Extract one shared state machine while preserving LF V25 behavior. No per-leg duplicate engines. No threshold changes.

### P2A-G4 — LF regression oracle

Instantiate LF through the new spec and prove offline behavioral preservation against immutable LF V25 source/evidence.

### P2A-G5 — RF/RH/LH offline specs

Instantiate geometry-driven specs only. Do not copy LF spans and do not claim hardware validation.

### P2A-G6 — fail-closed provenance/safety tests

Prove:

```text
diagnostic outside-limit target cannot become executable
UNRESOLVED cannot become accepted
stale Geometry V5 provenance rejected
wrong URDF/mesh/semantic SHA rejected
GoalPosition always unsigned
contact confirmation stops advance
EEPROM path unreachable in Phase 2A mode
```

### P2A-G7 — full offline validation

Run current CI-equivalent Rust/Station/MATDOG checks without hardware.

### P2A-G8 — independent adversarial review

Do not merge with any BLOCKER/MAJOR open.

### P2A-G9 — Draft PR / final handoff

Create a reviewable Draft PR only after offline gates. Merge requires separate explicit authorization.

## Stop conditions

Stop and report if any of these occur:

```text
LF V25 source/evidence changes unexpectedly
historical RF worktree is modified
Geometry V5 canonical provenance mismatches
per-leg duplicate state machines reappear
LF spans become other-leg coordinates
thresholds are changed to make another leg pass
DIAGNOSTIC becomes executable
UNRESOLVED becomes PASS/accepted
G4 replay context leaks into canonical runtime/spec
signed GoalPosition is proposed
EEPROM becomes reachable in Phase 2A
hardware/serial movement is attempted
live-FK contradiction is hidden by changing digital-zero state
FRONT/HIND are treated as identical by convention
fitted affine diagnostics erase raw disagreement
stale geometry provenance is accepted
force-push/destructive history rewrite is proposed
```

If an architectural deviation becomes necessary: stop, explain impact/risk, propose minimal alternatives and wait for Matteo approval.
