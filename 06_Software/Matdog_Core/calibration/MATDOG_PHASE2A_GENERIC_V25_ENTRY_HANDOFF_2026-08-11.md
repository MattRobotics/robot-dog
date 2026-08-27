# MATDOG — Phase 2A Entry Handoff

> ## ⚠️ SUPERSEDED DOCUMENTATION — STATION-MEDIATED PHASE
>
> This document describes the **Station-mediated, 12-servo** architecture and the pre-2026-08-27
> physical installation. It is **historical evidence**, not current operational truth.
>
> - The **ESP32-S3 coprocessor** now owns the ST3215 bus, not NormaCore Station.
> - The robot has **17 servos** (12 leg + 5 head/jaw), not 12.
> - **All calibration was RESET** on 2026-08-27; 14 of 17 servos were recoded to new bus IDs.
>
> Current: [ARCHITECTURE.md](../../../01_Docs/02_Architecture/ARCHITECTURE.md) ·
> [calibration reset](../../../09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md) ·
> [historical archive](../../../09_Logs/Historical/NormaCore_MATDOG_Archive/README.md)

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

Geometry Compiler V5 was merged through `robot-dog` PR #19.

```text
reviewed PR head:
2890daf0a8ac6103d3856f208a5f042528fc0da0

squash merge commit on robot-dog/main:
f07aa094a1b78c5670cc36ef3fdb349422a38955
```

Detailed closeout:

```text
09_Logs/Development_Log/2026-08-11_PHASE2A0_GEOMETRY_V5_CLOSEOUT.md
```

## Expected repository entry state — verify, do not assume

### robot-dog

```text
MattRobotics/robot-dog
main expected at handoff creation:
f07aa094a1b78c5670cc36ef3fdb349422a38955
```

### norma-core

```text
MattRobotics/norma-core
main expected at handoff creation:
f47b1ba579c623139058a8b0118648015739ab10
```

### immutable LF V25 oracle

```text
release/matdog-lf-calibrator-v25
f87dd1fbc7e8100d275c74f9af448642f3429680
```

The LF release must not be rewritten.

## Historical RF worktree — preserve as evidence

On the ASUS, verify and preserve:

```text
/home/matteo-manicardi/MATDOG/worktrees/norma-core-rf-calibrator
branch: matdog/rf-calibrator-from-lf-v25
historical checkpoint: b2f7dac2eab7147917fccdfde702360da82ab7de
```

This worktree is evidence only. Do not reset it, delete it or merge its duplicated RF architecture wholesale.

## Phase 2A target architecture

Required:

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

Forbidden target architecture:

```text
LfSessionStateMachine
RfSessionStateMachine
RhSessionStateMachine
LhSessionStateMachine
```

The shared engine owns behavior. The spec/profile owns leg-specific data.

## Phase 2A scope is offline only

Phase 2A does not authorize:

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
PHASE 2A
Generic V25-derived engine — OFFLINE
↓
PHASE 2B
final path/parking safety integration
↓
PHASE 2C
complete offline validation
↓
future hardware
RF -> RH -> LH
```

## Geometry Compiler V5 contract to consume

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

Permanent rule learned from the independent audit:

```text
CANONICAL V5 != G4 REPLAY
```

Frozen G4 is only a non-canonical replay/refactor oracle. Never reintroduce its legacy 30/50/90-degree context as canonical runtime/spec truth.

## Safety-policy state — keep visible

Final external reference policy:

```text
threshold = 3 mm
16 PASS
0 FAIL
8 UNRESOLVED
0 motion authorizations
```

All eight `UNRESOLVED` are outside executable URDF target domain:

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

The generic engine/spec/serializer/consumer must make it impossible for a diagnostic outside-limit target to silently become an executable command.

## LF V25 remains the only hardware oracle

Final LF hardware contacts:

```text
HIP   MIN -42.803°   MAX +39.375°
UPPER MIN -53.525°   MAX +122.607°
LOWER MIN -91.846°   MAX +34.277°
```

RF/RH/LH remain geometry-only until their own future hardware validation.

Never copy LF measured spans as mandatory RF/RH/LH coordinates.

Permanent detector rule:

```text
ContactConfirmed -> STOP ADVANCING IMMEDIATELY
```

A confirmed contact outside the model-predicted band must fail closed with controlled backoff, diagnostics, cleanup and verified torque-off when hardware phases eventually begin.

## Mandatory LF code classification before refactor

Before moving/refactoring any materially relevant LF V25 constant/helper/state transition, classify it as exactly one of:

```text
A — truly generic calibration behavior
B — geometry/profile/spec data
C — global ST3215 hardware/safety parameter
D — historical LF-only evidence/witness
```

Show and review the mapping before architecture edits.

Do not change detector thresholds merely to make another leg pass.

## q0 / scale / affine distinctions

The generalized engine must distinguish:

```text
1. physical encoder/transmission angular scale
2. q0 offset
3. geometry/endstop mismatch
4. fitted affine diagnostic normalization
```

A fitted affine model must never erase raw model-vs-hardware endpoint disagreement.

LF V25 is immutable historical evidence and must not be retrospectively rewritten.

## Live-FK status mismatch — entry risk

A pre-existing contradiction remains in `robot-dog`:

```text
MATDOG_JOINT_CALIBRATION.yaml:
DIGITAL_ZERO_CALIBRATED_AND_VERIFIED

matdog_leg_fk_live.py expects:
VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION
```

Classification:

```text
NON-BLOCKING for Phase 2A0
POTENTIAL Phase 2A / live-FK blocker
```

Phase 2A G0 must determine whether the generic engine actually depends on this live-FK loader/contract.

If not, isolate and defer it to the owning workflow.

If yes, stop and resolve the semantic contract explicitly before relying on live FK.

Do not change the established digital zero merely to make tests green.

## FRONT/HIND and geometry rules

FRONT and HIND are not interchangeable by convention. The current model has different front/hind hip placement; prerequisites and parking must remain model/profile-driven.

Do not infer runtime structural truth from string naming where model metadata already exists.

## Stale-geometry provenance must fail closed

The eventual generic runtime/profile contract must bind calibration evidence to geometry provenance, conceptually including:

```text
robot-dog geometry revision
URDF SHA
collision-mesh manifest SHA
Geometry V5 semantic SHA
calibration software SHA
hardware evidence/log SHA
```

A mismatched/stale geometry profile must be rejected, never silently consumed.

## Permanent ST3215 / Station constraints

```text
ST3215 GoalPosition remains unsigned 0..4095
signed-wrap forbidden
Station remains sole serial owner during motion
```

No physical movement without successful offline gates and explicit current-session authorization.

No EEPROM Position Offset change without separate explicit authorization.

## Recommended Phase 2A gates

### P2A-G0 — read-only entry audit

Verify before modification:

- `robot-dog` current remote main/PR/branches;
- `norma-core` current remote main/PR/branches;
- immutable LF V25 release;
- real ASUS worktrees and dirty/untracked state;
- historical RF worktree state;
- current MATDOG CI workflows;
- Geometry V5 canonical artifacts/hashes;
- live-FK mismatch ownership/dependency.

Report discrepancies and stop before modification if unexpected.

### P2A-G1 — LF V25 behavioral inventory

Read `matdog.rs`, tests, Station integration, profile/observer/freeze paths and classify relevant LF material A/B/C/D.

No refactor before review of this mapping.

### P2A-G2 — generic spec contract

Design the minimal `LegCalibrationSpec` or equivalent. Preserve target-domain status, geometry provenance, motor identity/direction, contact side/order, geometry corridors/bands, prerequisite/parking references and restore-plan semantics.

No hardware.

### P2A-G3 — generic engine foundation

Extract one shared `LegSessionStateMachine` while preserving LF V25 behavior. No per-leg duplicate engines and no detector threshold changes.

### P2A-G4 — LF regression oracle

Instantiate LF through the new spec and prove offline behavioral preservation against the immutable LF V25 source/evidence.

### P2A-G5 — RF/RH/LH offline specs

Instantiate geometry-driven data/specs only. Do not copy LF spans and do not label any of these legs hardware validated.

### P2A-G6 — fail-closed provenance/safety tests

Prove at minimum:

```text
diagnostic outside-limit target cannot become executable
UNRESOLVED cannot become accepted
stale Geometry V5 provenance rejected
wrong URDF/mesh/semantic SHA rejected
GoalPosition remains unsigned
contact confirmation stops further advance
no EEPROM path is reachable in Phase 2A mode
```

### P2A-G7 — full offline validation

Run current CI-equivalent Rust/Station/MATDOG checks without hardware.

### P2A-G8 — independent adversarial review

No merge if a BLOCKER/MAJOR remains.

### P2A-G9 — draft PR / final handoff

Create a reviewable Draft PR only after offline gates. Merge requires separate explicit authorization.

## Mandatory stop conditions

Stop and report if any of these occur:

```text
LF V25 source/evidence unexpectedly changes
historical RF worktree is modified
Geometry V5 canonical provenance mismatches
per-leg duplicate state machines reappear
LF spans become other-leg coordinates
detector thresholds are changed to make another leg pass
DIAGNOSTIC becomes executable
UNRESOLVED becomes PASS/accepted
G4 replay context leaks into canonical runtime/spec
signed GoalPosition is proposed
EEPROM becomes reachable in Phase 2A
hardware/serial motion is attempted
live-FK contradiction is hidden by changing digital-zero state
FRONT/HIND are treated as identical by convention
fitted affine diagnostics erase raw disagreement
stale geometry provenance is accepted
force-push/destructive history rewrite is proposed
```

If an architectural deviation is genuinely necessary: stop, explain impact/risk, propose the minimal alternatives and wait for Matteo's approval.
