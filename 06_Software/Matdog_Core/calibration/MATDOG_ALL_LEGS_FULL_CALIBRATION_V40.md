# MATDOG All-Legs Full Calibration V40

Date: 2026-08-01  
Status: complete native implementation under offline verification  
Runtime token: `MATDOG_ALL_LEGS_FULL_V40`

## Purpose

V40 is the final native orchestration requested for the 12 ST3215 leg joints. A single press of **Auto Calibrate** executes all 24 mechanical-contact acquisitions and derives a software q=0 for every joint from both URDF endpoints.

Canonical order:

```text
LF → RF → RH → LH
```

For every leg:

```text
UPPER MIN → MAX
LOWER MIN → MAX
HIP MIN → MAX
URDF endpoint consistency gate
```

Only after all four leg gates pass:

```text
12 accepted software q=0 targets
→ all HIP joints
→ all UPPER joints
→ all LOWER joints
→ verify all 12 holds
→ verified global torque OFF
```

## Why final HOME is delayed

During the 24 contact acquisitions every stage restores the active geometry to the historical digital HOME and performs verified global torque OFF. A newly estimated q=0 is not applied immediately to a leg, because the next leg's restart-safe gate still expects the common digital reference.

V40 therefore stores the accepted q=0 estimates in memory and applies them only after LF, RF, RH and LH have all passed. This prevents a calibrated earlier leg from invalidating the startup assumptions of a later leg.

## Per-leg fail-closed rule

A leg must satisfy both-endpoint URDF consistency before the next leg starts:

```text
endpoint disagreement <= 24 ticks
estimated q=0 shift from 2048 <= 96 ticks
```

On failure:

```text
MODEL_ZERO_INCONSISTENT <LEG>
next leg not started
no calibrated HOME applied
global torque OFF verified
```

The acquired contacts remain in the Station log for diagnosis.

## Geometry policy

### Front legs

LF and RF use the existing ipsilateral rear-UPPER parking prerequisite from the validated profile table:

```text
LF active → park LH upper M42
RF active → park RH upper M32
```

LF retains the hardware-validated shared HIP stage:

```text
HIP MIN → HOME → HIP MAX → HOME
```

### Rear legs

The exact-mesh checkpoint of 2026-07-20 established that RH and LH do not require any front-leg parking. Non-active legs remain near HOME.

Rear HIP prerequisites remain:

```text
UPPER horizontal
LOWER folded/parallel
HIP probing joint
```

## Motion envelope

```text
TorqueLimit = 500
GoalSpeed   = 160
Acc         = 8
coarse step = 64 ticks
fine step   = 8 ticks
settle      = 900 ms
hard-current abort = 200
```

The higher speed and torque are intended to remove the under-seating ambiguity observed during the first LF HIP MAX pilot. They do not enlarge the URDF-derived contact corridors or mechanical guards.

## Permanent safety contract

- Station is the only serial owner.
- `GoalPosition` remains standard unsigned ST3215.
- Writes remain RAM-only: TorqueEnable, Acc, GoalPosition, GoalSpeed and TorqueLimit.
- No EEPROM, Position Offset, LOCK, reset, ResetCalibration, RegWrite, Action, Save or Freeze.
- Every contact stage ends with verified global torque OFF.
- Final all-joint placement ends with verified global torque OFF.
- Driver errors, status errors and hard current abort remain active.
- Any stage failure prevents the next stage from starting.

## Current hardware eligibility

The final V40 implementation can be reviewed and built offline immediately. Hardware execution remains intentionally blocked until both prerequisite leg checkpoints pass:

```text
LF_LEG_FULL_V38 hardware model-zero PASS
LH_LEG_FULL_V39 hardware model-zero PASS
```

After those two checkpoints, the remaining mirrored RF/RH behavior is already represented by the canonical profile table and V40 tests, but the first complete 24-contact run remains supervised with the robot supported, all legs free, operator present and master disconnect accessible.

## Repository branches

NormaCore:

```text
matdog/all-legs-full-calibration-v40
```

MATDOG records:

```text
matdog/all-legs-full-calibration-program-v40
```

No merge into `main` is authorized at this stage.
