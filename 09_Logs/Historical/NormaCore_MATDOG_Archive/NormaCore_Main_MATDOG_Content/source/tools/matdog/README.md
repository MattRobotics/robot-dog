# MATDOG calibrator — current canonical state

## Current development milestone — 2026-08-11

Geometry Compiler V5 / Phase 2A0 is now CLOSED and merged in `MattRobotics/robot-dog`.

Current development order:

```text
PHASE 2A
one generic V25-derived full-leg engine in norma-core
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

Current operational entry handoff:

```text
tools/matdog/MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md
```

The older:

```text
tools/matdog/MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md
```

is retained as historical/architectural evidence. It is no longer the current milestone entry point where it says Geometry Compiler is still the immediate next task.

## Sources of truth

Always verify live remote/local state before changes.

Expected entry checkpoints at handoff creation:

```text
MattRobotics/robot-dog
main after Geometry V5 merge:
f07aa094a1b78c5670cc36ef3fdb349422a38955

MattRobotics/norma-core
main:
f47b1ba579c623139058a8b0118648015739ab10

immutable LF V25 release:
release/matdog-lf-calibrator-v25
f87dd1fbc7e8100d275c74f9af448642f3429680
```

The real GitHub remotes and the real ASUS filesystem override stale handoff metadata if legitimate newer state exists.

## LF V25 — immutable hardware oracle

LF V25 remains the only mechanically hardware-validated MATDOG full-leg calibration flow.

```text
measurement sequence: 58/58 DONE
runner result: PASS
global torque OFF: verified
persistent stage: LF_STAGED
EEPROM transaction: LF_FROZEN
persistent profile: LF_FROZEN
```

Final LF evidence:

| Joint | Motor | Final MIN contact | Final MAX contact | Affine q0 before EEPROM |
|---|---:|---:|---:|---:|
| LF hip | M13 | 2535 | 1600 | 2067 |
| LF upper | M12 | 1439 | 3443 | 2040 |
| LF lower | M11 | 3093 | 1658 | 2074 |

Final hardware contact angles:

```text
HIP   MIN -42.803°   MAX +39.375°
UPPER MIN -53.525°   MAX +122.607°
LOWER MIN -91.846°   MAX +34.277°
```

Exact validated release:

```text
release/matdog-lf-calibrator-v25
f87dd1fbc7e8100d275c74f9af448642f3429680
```

The release branch must never be rewritten and LF V25 must not be retrospectively modified to fit later geometry or other legs.

RF/RH/LH are not hardware validated.

## Phase 2A target architecture

Required:

```text
LegSessionStateMachine
+
LegCalibrationSpec
```

not independent LF/RF/RH/LH state machines.

Conceptually:

```text
Generic V25 Full-Leg Engine
  -> LF LegCalibrationSpec
  -> RF LegCalibrationSpec
  -> RH LegCalibrationSpec
  -> LH LegCalibrationSpec
```

Before refactoring LF V25, classify every materially relevant constant/helper/state transition:

```text
A — truly generic calibration behavior
B — geometry/profile/spec data
C — global ST3215 hardware/safety parameter
D — historical LF-only evidence / witness
```

Review this mapping before moving logic.

Do not alter a detector/safety threshold merely to make another leg pass.

## Geometry Compiler V5 contract from robot-dog

Canonical merged `robot-dog` baseline immediately after PR #19:

```text
f07aa094a1b78c5670cc36ef3fdb349422a38955
```

Corrected V5 semantic hashes:

```text
endpoint
  de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e
parking
  67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139
combined
  0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51
```

Permanent semantic boundary:

```text
CANONICAL V5 != G4 REPLAY
```

Legacy G4 30/50/90-degree contexts are replay evidence only and must never re-enter the generic runtime/spec as canonical prerequisite truth.

## Safety-policy state to preserve

Final external reference policy:

```text
16 PASS
0 FAIL
8 UNRESOLVED
0 motion authorizations
3 mm threshold unchanged
```

All eight `UNRESOLVED` are outside the executable URDF target domain.

Permanent contract:

```text
UNRESOLVED != PASS
DIAGNOSTIC != EXECUTABLE
GEOMETRIC CONTACT != MOTION AUTHORIZATION
```

The generic engine must make it impossible for a diagnostic/outside-limit target to silently become executable.

## Historical RF worktree — preserve, do not continue as architecture

On the ASUS, verify and preserve:

```text
/home/matteo-manicardi/MATDOG/worktrees/norma-core-rf-calibrator
branch: matdog/rf-calibrator-from-lf-v25
historical checkpoint: b2f7dac2eab7147917fccdfde702360da82ab7de
```

It is evidence only. Do not reset/delete it and do not merge its duplicated RF state-machine design wholesale.

## Phase 2A is offline software-foundation work

Phase 2A does not authorize:

```text
physical servo motion
Station motion execution
direct serial probing
EEPROM writes
Position Offset changes
RF/RH/LH hardware calibration
persistent q0 freeze
```

Hardware remains gated behind later Phase 2B/2C offline validation and explicit current-session human authorization.

## Permanent ST3215 / Station rules

- Station remains the sole owner of the ST3215 serial adapter during motion.
- `GoalPosition` remains unsigned standard `0..4095`; signed-wrap is forbidden.
- Digital-home commissioning remains separate from mechanical endpoint calibration.
- Static joints are validated by real position drift/state integrity, not one isolated raw-speed sample.
- Every hardware contact must pass repeatability, model consistency and supervised evidence.
- EEPROM access starts only after complete measurement PASS, Station shutdown and verified serial release.
- EEPROM provisioning must be transactional: backup, unlock, write, action, readback, relock, rollback on failure.
- No per-leg exception may hide a generic detector problem.

## q0 / scale / affine distinction

The generalized system must distinguish:

```text
physical encoder/transmission scale
q0 offset
geometry/endstop mismatch
fitted affine diagnostic normalization
```

A fitted affine model must not erase raw model-vs-hardware disagreement.

LF measured spans remain evidence, never mandatory RF/RH/LH coordinates.

## Known entry risk: live-FK status mismatch

`robot-dog` contains a pre-existing contract mismatch:

```text
tracked state:
DIGITAL_ZERO_CALIBRATED_AND_VERIFIED

live-FK loader expects:
VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION
```

Phase 2A G0 must explicitly determine whether the generic calibrator depends on this loader/contract.

If it does, resolve the semantic contradiction before relying on live FK. Do not change the established digital zero merely to make tests green.

## Durable MATDOG CI

Current reusable checks on `norma-core/main` include:

```text
.github/workflows/matdog-native-calibrator-check.yml
.github/workflows/matdog-native-observer-check.yml
```

Use existing CI coverage before creating redundant workflows.

## Principal implementation files to inventory before refactor

```text
software/drivers/st3215/src/auto_calibrate/matdog.rs
software/drivers/st3215/src/auto_calibrate/matdog_test.rs
software/drivers/st3215/src/bin/matdog_lf_freeze.rs
tools/matdog/matdog_headless_auto_calibrate.py
tools/matdog/matdog_lf_profile.py
tools/matdog/matdog_native_observer_contract.py
```

## Current first gate

Do not begin by rewriting `matdog.rs`.

Start with a read-only G0 audit and a source-level LF V25 behavioral inventory. Only after the A/B/C/D mapping is reviewed may the generic-engine refactor begin.
