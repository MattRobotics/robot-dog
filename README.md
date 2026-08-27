# MATDOG — Custom Quadruped Robot

MATDOG is a custom quadruped robot developed by Matt Robotics in Italy. The repository is the engineering source of truth for the robot-specific stack: mechanical design, CAD/URDF, collision geometry, electronics, calibration evidence, kinematics, locomotion, validation and project decisions.

> ## ⚠️ CURRENT STATE — REASSEMBLY IN PROGRESS · CALIBRATION RESET REQUIRED
>
> As of **2026-08-27** all **17** servos have been bench-provisioned to `MATDOG_C018_V1` and are
> being **physically remounted** (12 leg servos, then 5 head/jaw servos). Consequently:
>
> - **All robot calibration must be redone from zero.** Old digital zero / q0 values are
>   **historical evidence, not active truth**.
> - **`PositionOffset = 0` is the baseline** on all 17 units and must stay that way. Never rewrite
>   `PositionOffset` to compensate mechanical mounting error. Never use `CalibrationOfs` or
>   one-key-middle.
> - **No hardware motion may be commanded from stale calibration.**
> - **Full recalibration must complete before any stand, gait or load-bearing attempt.**
>
> Read first: [**Calibration reset — 2026-08-27**](09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md)
> · [Handoff](09_Logs/Development_Log/2026-08-27_ST3215_CANONICAL_ARCHIVE_HANDOFF.md)
> · [ST3215 evidence index](09_Logs/ST3215_EVIDENCE_INDEX.md)

## System architecture

```text
MATDOG mechanical platform
→ 17 × Feetech ST-3215-C018 serial-bus servos (12 leg + 5 head/jaw)
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

## Current canonical state — 2026-08-27

| Area | Status |
|---|---|
| Mechanical architecture / REV00 | Validated — geometry unaffected by reassembly |
| Servo bus — now **17 servos** (12 leg + 5 head/jaw) | Allocation validated 2026-08-27 |
| Bench QC / Servo Quality Audit V6.1 | **COMPLETE** — 26/26 runs PASS · [report](09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md) |
| `MATDOG_C018_V1` persistent profile | **FROZEN** · [profile](01_Docs/02_Architecture/MATDOG_ST3215_C018_V1_PROFILE.md) |
| ST3215 provisioning | **17/17 PASS** · [campaign](09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/README.md) |
| Servo allocation — unit → joint → ID | **FIXED** · [allocation](06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml) |
| Physical reassembly | **IN PROGRESS** — legs then head |
| **Robot calibration (all joints)** | **⚠️ RESET — REQUIRED, NOT STARTED** · [reset](09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md) |
| Digital-home commissioning / old q0 | **HISTORICAL — superseded by reassembly** |
| LF mechanical calibration (V25) | **HISTORICAL ORACLE** — describes the previous installation |
| RF / RH / LH hardware calibration | Not yet validated |
| Head / jaw joints (5) | **NEVER CALIBRATED** |
| Encoder↔radian conversion / read-only FK | Validated as a method; **must be re-verified after recalibration** |
| Geometry Compiler Phase 1B | **CLOSED** |
| Geometry Compiler V5 / Phase 2A0 | **CLOSED AND MERGED — PR #19** |
| Generic V25-derived full-leg engine / Phase 2A | **NEXT** |
| Phase 2B path/parking safety integration | Pending |
| Phase 2C complete offline validation | Pending |
| Stand-up / gait / locomotion | Blocked on full recalibration |

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

## LF V25 — historical hardware oracle

> **⚠️ Superseded as current state by the 2026-08-27 reassembly.** The LF servos have been removed,
> bench-provisioned to `PositionOffset = 0` and remounted. The contacts and offsets below describe
> the **previous** installation. They remain the reference for what a correct calibration result
> looks like — they are **not** a description of the robot now, and must never be imported as
> current truth. See [calibration reset](09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md).

LF V25 remains the only mechanically hardware-validated full-leg calibration flow ever executed.

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

The frozen LF `PositionOffset` values (M11 `127`, M12 `851`, M13 `−486`) are **no longer present on
any servo** — all 17 units now hold `PositionOffset = 0`. Note also that these units were recoded:
M11 is now ID 52 (`NECK_PITCH`) and M13 is no longer allocated at all. Reading historical records by
bus ID against the current robot will produce **wrong joint associations** — see
[servo allocation](06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml).

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

## Next milestone — reassembly → full recalibration

The immediate milestone is **not** Phase 2A. It is physical:

```text
1. finish assembly        12 leg servos, then 5 head/jaw servos
                          hold physical RAW ~2048 while mounting;
                          mount horns and links directly in the calibration pose

2. full recalibration     from zero, on the new installation
                          re-capture digital q0 — measured, never imported
                          verify mapping and directions on all 17 joints

3. controlled bring-up    read-only FK verification
                          → supervised suspended motion
                          → gradual load transfer
```

Throughout: `PositionOffset = 0` stays the baseline. Never compensate mechanics by rewriting
EEPROM. Full policy: [calibration reset](09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md).

## Following milestone — Phase 2A

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

**This risk is now compounded by the 2026-08-27 calibration reset.** `robot.calibration_status`
still reads `DIGITAL_ZERO_CALIBRATED_AND_VERIFIED`, but that status is **stale** — the calibration
it describes no longer applies. The enum was deliberately left unchanged because six tools
hard-assert the exact string, and an additive `calibration_reset:` block now carries the
authoritative state.

> **Follow-up required:** introduce a proper reset enum together with updates to all six consumers,
> so the tools fail closed on stale calibration. Until then, treat
> `matdog_visual_zero_pose_probe.py`, `matdog_capture_visual_zero.py`, `matdog_apply_visual_zero.py`,
> `matdog_calibration_validate.py`, `matdog_live_joint_monitor.py` and `matdog_leg_fk_live.py` as
> **unsafe to run against hardware**.

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

Servo allocation — **17 units, current as of 2026-08-27**:

```text
        joint            bus ID   physical unit
LF      hip   / upper / lower      13 / 12 / 11    M22   / ELR01 / M33
RF      hip   / upper / lower      23 / 22 / 21    NEW01 / ELR03 / NEW03
RH      hip   / upper / lower      33 / 32 / 31    NEW06 / ELR02 / NEW05
LH      hip   / upper / lower      43 / 42 / 41    M43   / M42   / M41

HEAD    neck rotation              51              M31
        neck pitch                 52              M11
        head rotation              53              NEW04
        head pitch                 54              NEW02
        jaw                        55              ELR04
```

Authoritative machine-readable source:
[`06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml`](06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml)

> **This replaces the historical 12-servo mapping** (`LF: M13 hip, M12 upper, M11 lower`, …). Bus
> IDs were reassigned during provisioning — 14 of 17 units were recoded. Historical records keyed
> by bus ID will associate the **wrong joint** against the current robot.

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
- **`PositionOffset = 0` is the baseline on all 17 servos and must remain so.** Mechanical mounting
  error is corrected mechanically, never by rewriting `PositionOffset`.
- **`CalibrationOfs`, one-key-middle, factory reset and broadcast write are permanently forbidden.**
- The C018 model word is read from register **`0x03`** (expected `777`), never from `0x00`.
- No hardware motion may be commanded from stale calibration; full recalibration must complete
  before any stand, gait or load-bearing attempt.
- No detector threshold may be changed merely to make another leg pass.
- FRONT and HIND geometry must be derived from the actual model, not treated as identical by convention.
- Raw model-vs-hardware disagreement must remain visible even if an affine diagnostic is fitted.
- Stale/mismatched geometry provenance must fail closed.
- No merge into `main` without explicit authorization.
- No force-push or destructive removal of validation evidence.

## Canonical records

Start with the current phase:

| Record | Covers |
|---|---|
| [⚠️ Calibration reset — 2026-08-27](09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md) | **read before touching hardware** |
| [ST3215 canonical archive handoff](09_Logs/Development_Log/2026-08-27_ST3215_CANONICAL_ARCHIVE_HANDOFF.md) | current state, corrections, next steps |
| [ST3215 evidence index](09_Logs/ST3215_EVIDENCE_INDEX.md) | every artifact, in Git and on the ASUS archive |
| [MATDOG_C018_V1 profile](01_Docs/02_Architecture/MATDOG_ST3215_C018_V1_PROFILE.md) | canonical persistent servo profile |
| [Provisioning campaign 17/17](09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/README.md) | provisioning results and limits |
| [Bench QC V6.1](09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md) | servo quality audit |
| [Frozen bench tools](05_Firmware/ST3215_Bench_Tools/README.md) | QC, survey and provisioner freezes |
| [Servo allocation](06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml) | unit → joint → bus ID |

Geometry and prior phases:

```text
REPOSITORY_VERIFICATION_INDEX.md
06_Software/Matdog_Core/calibration/MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md
09_Logs/Development_Log/2026-08-11_PHASE2A0_GEOMETRY_V5_CLOSEOUT.md
09_Logs/Validation_Reports/Geometry_Compiler/README.md
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
```

Older Phase 1 / Phase 1B / RF experiment documents remain historical evidence. Where milestone direction conflicts, the newest explicitly canonical handoff governs.
