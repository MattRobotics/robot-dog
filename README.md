# MATDOG — Custom Quadruped Robot

MATDOG is a custom quadruped robot developed by Matt Robotics in Italy. The repository is the engineering source of truth for the robot-specific stack: mechanical design, CAD/URDF, collision geometry, electronics, calibration evidence, kinematics, locomotion, validation and project decisions.

## System architecture

```text
MATDOG mechanical platform
→ 12 × Feetech ST3215 serial-bus servos
→ custom power distribution / protection
→ Waveshare Bus Servo Adapter
→ NormaCore Station / ST3215 runtime
→ MATDOG calibration + kinematics + locomotion software
→ future embedded controller / Jetson / IMU / perception
```

Repository responsibilities:

```text
MattRobotics/robot-dog
→ robot-specific truth: CAD, URDF, collision geometry, calibration evidence,
  kinematics, locomotion, validation reports and canonical handoffs

MattRobotics/norma-core
→ Station/ST3215 integration fork and native MATDOG calibration runtime
```

## Current canonical state — 2026-08-11

| Area | Status |
|---|---|
| Mechanical architecture / REV00 | Validated |
| Twelve-servo bus, mapping and directions | Validated |
| Digital-home commissioning / EEPROM readback | Validated for all 12 servos |
| Encoder↔radian conversion / read-only FK | Validated |
| LF mechanical calibration | **V25 hardware validated and frozen** |
| RF / RH / LH hardware calibration | Not yet validated |
| Geometry Compiler Phase 1B | **CLOSED** |
| Geometry Compiler V5 / Phase 2A0 | **CLOSED AND MERGED — PR #19** |
| Generic V25-derived full-leg engine / Phase 2A | **NEXT** |
| Phase 2B path/parking safety integration | Pending |
| Phase 2C complete offline validation | Pending |
| Stand-up / gait / locomotion | Pending post-calibration validation |

Canonical `main` immediately after the Phase 2A0 squash merge:

```text
f07aa094a1b78c5670cc36ef3fdb349422a38955
```

Merged PR:

```text
#19 — Geometry Compiler V5: deterministic pure geometry pipeline
reviewed head: 2890daf0a8ac6103d3856f208a5f042528fc0da0
merge commit:  f07aa094a1b78c5670cc36ef3fdb349422a38955
```

Detailed closeout:

```text
09_Logs/Development_Log/2026-08-11_PHASE2A0_GEOMETRY_V5_CLOSEOUT.md
```

## LF V25 — immutable hardware oracle

The only mechanically hardware-validated full-leg calibration flow remains LF V25.

Exact NormaCore release:

```text
release/matdog-lf-calibrator-v25
f87dd1fbc7e8100d275c74f9af448642f3429680
```

Final hardware contacts:

```text
HIP   MIN -42.803°   MAX +39.375°
UPPER MIN -53.525°   MAX +122.607°
LOWER MIN -91.846°   MAX +34.277°
```

Canonical LF record:

```text
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
```

RF/RH/LH are **not** hardware-confirmed and must not inherit LF spans as coordinates.

## Geometry Compiler V5 / Phase 2A0 — CLOSED

Phase 2A0 introduced the dedicated CAD-derived collision geometry and a deterministic pure-geometry compiler with explicit separation between geometry, replay evidence and external safety policy.

The final candidate underwent an independent adversarial audit. A real composition-root defect was found in the first candidate: frozen G4 replay contexts had contaminated the supposedly pure canonical V5 path analysis. The defect and all associated MAJOR/MINOR findings were remediated before merge.

Final finding status:

```text
B1 CLOSED
M1 CLOSED
M2 CLOSED
M3 CLOSED
m1-m5 CLOSED
NO NEW BLOCKER / MAJOR
```

Permanent rule:

```text
CANONICAL V5 != G4 REPLAY
```

Corrected canonical C/D semantic hashes:

```text
endpoint
  de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e
parking
  67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139
combined
  0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51
```

Final canonical results:

```text
24/24 geometric endpoint contacts
24/24 canonical endpoint contexts empty
24/24 endpoint/planner consistency
6 direct-target path obstructions
18 collision-free direct paths

parking:
18 NOT_NEEDED
6 FEASIBLE_1DOF_PLAN_FOUND
24/24 complete sequences
94 evaluated 1DOF candidates
0 requiring 2DOF
```

Final tests:

```text
241/241 geometry discovery PASS
58/58 adjacent offline calibration PASS
299/299 total PASS
```

## Safety-policy state carried forward

Geometry V5 intentionally does **not** equate geometry with permission to move.

Final external 3 mm policy:

```text
16 PASS
0 FAIL
8 UNRESOLVED
0 motion authorizations
```

All eight `UNRESOLVED` belong to `DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS`; none of the eight executable-URDF-domain endpoints is FAIL or UNRESOLVED.

Permanent interpretation:

```text
UNRESOLVED != PASS
DIAGNOSTIC != EXECUTABLE
GEOMETRIC CONTACT != MOTION AUTHORIZATION
```

The eight unresolved diagnostic endpoints remain explicit Phase 2B/2C safety inputs and must never be silently converted into executable targets.

## Next milestone — Phase 2A

Phase 2A builds **one generic V25-derived full-leg calibration engine** in `MattRobotics/norma-core`.

Target architecture:

```text
LegSessionStateMachine
+
LegCalibrationSpec

Generic V25 Full-Leg Engine
  → LF spec
  → RF spec
  → RH spec
  → LH spec
```

Not four copied per-leg state machines.

Before refactoring LF V25, every materially relevant constant/helper/state transition must be classified as:

```text
A — truly generic calibration behavior
B — geometry/profile/spec data
C — global ST3215 hardware/safety parameter
D — historical LF-only evidence
```

Phase 2A is offline software-foundation work only. No hardware motion, Station probing, serial ownership changes or EEPROM writes are part of the phase.

Canonical Phase 2A entry handoff:

```text
06_Software/Matdog_Core/calibration/
MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md
```

## Known next-phase risk: live-FK status mismatch

A pre-existing contract mismatch remains visible:

```text
tracked calibration status:
DIGITAL_ZERO_CALIBRATED_AND_VERIFIED

live-FK loader expectation:
VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION
```

It was verified as pre-existing and was not introduced by Geometry V5. Phase 2A must determine explicitly whether the generic calibrator depends on this loader. Do not change the established digital zero merely to make tests green.

## Robot definition

Canonical REV00 package:

```text
03_CAD/URDF/matt_robodog_rev00/
```

Coordinate convention:

```text
X = forward
Y = left
Z = up
units = metres and radians
right-handed frame
```

Servo mapping:

```text
LF: M13 hip, M12 upper, M11 lower
RF: M23 hip, M22 upper, M21 lower
RH: M33 hip, M32 upper, M31 lower
LH: M43 hip, M42 upper, M41 lower
```

Canonical leg order / trot diagonals:

```text
[LF, RF, RH, LH]
LF + RH
RF + LH
```

## Permanent calibration/control rules

- Station remains the sole ST3215 serial owner during motion.
- ST3215 `GoalPosition` remains unsigned `0..4095`; signed-wrap is forbidden.
- LF V25 release and hardware evidence remain immutable.
- No physical movement without successful offline gates and explicit current-session authorization.
- No EEPROM Position Offset write without separate explicit authorization.
- No detector threshold may be changed merely to make another leg pass.
- FRONT and HIND geometry must be derived from the actual model, not treated as identical by convention.
- Raw model-vs-hardware disagreement must remain visible even if an affine diagnostic is fitted.
- Stale/mismatched geometry provenance must fail closed.
- No merge into `main` without explicit authorization.
- No force-push or destructive removal of validation evidence.

## Canonical records

Start with:

```text
REPOSITORY_VERIFICATION_INDEX.md
06_Software/Matdog_Core/calibration/MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md
09_Logs/Development_Log/2026-08-11_PHASE2A0_GEOMETRY_V5_CLOSEOUT.md
09_Logs/Validation_Reports/Geometry_Compiler/README.md
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
```

Older Phase 1 / Phase 1B / RF experiment documents remain historical evidence. Where milestone direction conflicts, the newest explicitly canonical handoff governs.
