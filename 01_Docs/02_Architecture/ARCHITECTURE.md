# MATDOG Architecture

**Canonical as of 2026-08-27.** This document describes the **current** architecture only.
For the superseded Station-mediated phase see
[NormaCore MATDOG archive](../../09_Logs/Historical/NormaCore_MATDOG_Archive/README.md).

---

## Reading this document

Every claim below is tagged. Nothing is presented as built when it is not.

| Tag | Meaning |
|---|---|
| ✅ **VALIDATED** | implemented **and** exercised on real hardware |
| 🟦 **DECIDED** | current architectural decision; implementation partial or not started |
| ⬜ **TBD** | not yet decided or frozen |

---

## Purpose

MATDOG is a custom 17-DOF quadruped with a head. This repository is the single active engineering
source of truth for the whole robot: mechanics, electronics, geometry, firmware, calibration,
kinematics, locomotion and evidence.

---

## Runtime layers

```text
┌──────────────────────────────────────────────────────────────┐
│ HIGH-LEVEL HOST                                              │
│   ASUS workstation during development          ✅ current    │
│   Jetson-class onboard computer later          🟦 decided    │
│                                                              │
│   ROS 2 / MoveIt 2 integration                 🟦 decided    │
│   AI, vision, planning, UI, behaviours         🟦 decided    │
└──────────────────────────────────────────────────────────────┘
             ↓  high-level / joint / motion commands
             ↓  transport: ⬜ TBD (not frozen)
┌──────────────────────────────────────────────────────────────┐
│ DEDICATED MATDOG ESP32-S3 COPROCESSOR                        │
│   operational owner of the ST3215 bus          ✅ validated  │
│   direct / native ST3215 driver path           ✅ validated  │
│   provisioning / commissioning / QC utilities  ✅ validated  │
│   deterministic motion execution               🟦 decided    │
│   watchdog and safety                          🟦 decided    │
│   IMU / power / battery telemetry              🟦 decided    │
│   joint-to-servo conversion                    🟦 decided    │
│   operational gait / IK responsibilities       ⬜ TBD split  │
└──────────────────────────────────────────────────────────────┘
             ↓  Feetech serial bus, 1 Mbps
┌──────────────────────────────────────────────────────────────┐
│ 17 × Feetech ST-3215-C018                      ✅ validated  │
│   12 leg  +  5 head/jaw                                      │
│   profile MATDOG_C018_V1, PositionOffset = 0                 │
└──────────────────────────────────────────────────────────────┘
```

---

## Ownership rules

1. ✅ The **ESP32-S3 coprocessor is the operational owner of the ST3215 serial bus.**
2. 🟦 The high-level host issues joint/motion intent; it does not drive the servo bus directly.
3. 🟦 The host never sends raw encoder targets in normal operation.
4. ⬜ Whether joint→actuator conversion, IK and gait execution live host-side or ESP32-side is the
   **embedded split**, and it is not frozen. Conversion is currently expected on the ESP32-S3.
5. ✅ `GoalPosition` is unsigned `0..4095`; signed wrap is forbidden.
6. ✅ `PositionOffset = 0` is the baseline on all 17 servos and stays that way. Mechanical mounting
   error is corrected mechanically, never by rewriting `PositionOffset`.
7. ✅ `CalibrationOfs`, one-key-middle, factory reset and broadcast write are permanently forbidden.
8. ✅ The C018 model word is read from register `0x03` (expected `777`), never from `0x00`.
9. ✅ Digital-home commissioning and mechanical endpoint calibration remain separate programs.
10. ✅ Only hardware-validated calibration results may become persistent operational profiles.
11. 🟦 No hardware motion may be commanded from stale calibration — enforced by a machine gate, see
    [calibration reset](../../09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md).

---

## Where NormaCore Station stands now

> **Station is NOT the canonical servo-control requirement.**
> It is **not** a mandatory ST3215 owner, **not** a required calibration path, **not** a required
> provisioning path, and **not** the definitive MATDOG control architecture.

Station **may** remain, entirely optionally:

- historical development and reference technology;
- an optional telemetry / observation / inference / logging / replay component;
- transitional tooling where it is still convenient.

This is **not** a ban on reusing good upstream or native NormaCore code. The rule is narrower:
MATDOG-specific ownership and development no longer live there, and no current MATDOG operation
depends on Station being present.

### Why the premise changed

Station was originally designated sole bus owner, which forced MATDOG calibration development into
`MattRobotics/norma-core`. Direct ESP32-S3 ST3215 operation has since been demonstrated in
practice — the bench QC, source-signature survey and provisioner stack drove real servos with no
Station in the loop, ending in **17/17 units provisioned**. That removed the constraint.

Full transition record: [NormaCore MATDOG archive](../../09_Logs/Historical/NormaCore_MATDOG_Archive/README.md).

---

## Repository responsibilities

| Repository | Responsibility |
|---|---|
| **`MattRobotics/robot-dog`** | **Sole active MATDOG repository** — mechanics, CAD/URDF, electronics, ESP32-S3 firmware and bench tools, ST3215 QC/provisioning, calibration, geometry, kinematics, locomotion, ROS 2 integration, high-level software, evidence, historical archive |
| `MattRobotics/norma-core` | Reference/upstream fork only. `main` retained; **no active MATDOG development branches**; no new MATDOG feature development |
| `norma-core/norma-core` | Official upstream reference |

---

## What is implemented and hardware-validated today

| Capability | Evidence |
|---|---|
| Direct ESP32-S3 ↔ ST3215 serial operation | [Bench QC V6.1](../../09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md) |
| Servo quality audit, 26 runs | same |
| Read-only 71-byte full-state survey | [frozen tools](../../05_Firmware/ST3215_Bench_Tools/README.md) |
| Persistent profile `MATDOG_C018_V1` | [profile](MATDOG_ST3215_C018_V1_PROFILE.md) |
| Provisioning 17/17 — centre, offset→0, ID recode, cold verify | [campaign](../../09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/README.md) |
| Offline geometry compiler, URDF/collision pipeline | Geometry Compiler V5 / Phase 1B |

### Explicitly NOT yet implemented

| Item | Status |
|---|---|
| Host ↔ ESP32-S3 transport | ⬜ **TBD — not frozen.** Physical layer and protocol undecided |
| ROS 2 / MoveIt 2 integration | 🟦 intended high-level stack; **no integration exists yet** |
| Deterministic motion execution firmware | 🟦 decided; not written |
| Watchdog / safety firmware | 🟦 decided; not written |
| IMU / power / battery telemetry | 🟦 decided; not written |
| Gait / IK embedded split | ⬜ TBD |
| Jetson-class onboard host | 🟦 decided; not procured or integrated |
| Robot joint calibration | ⚠️ **RESET** — must be redone from zero |

> ROS 2 and MoveIt 2 are the **intended** high-level robotics stack. They are not integrated,
> and nothing in this repository currently depends on them.

---

## Current physical state

**Reassembly in progress.** All 17 servos were bench-provisioned and are being remounted — 12 leg
servos, then 5 head/jaw servos.

**All robot calibration is RESET.** Old digital zero and q0 values are historical evidence, not
active truth. Full recalibration must complete before any stand, gait or load-bearing attempt.

→ [Calibration reset](../../09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md)

---

## Development sequence

1. Finish assembly — 12 leg servos, then 5 head/jaw servos.
2. **Full recalibration from zero** on the new installation; q0 measured, never imported.
3. Verify mapping and directions on all 17 joints.
4. Controlled bring-up: read-only FK → supervised suspended motion → gradual load transfer.
5. Freeze the host ↔ ESP32-S3 transport and the embedded split.
6. Build the ESP32-S3 motion/safety/watchdog layer.
7. Recompute four-leg FK; regenerate and audit stand poses.
8. Single-foot trajectories, gait, walking.
9. ROS 2 / MoveIt 2 integration.
10. Jetson-class onboard host, IMU, estimator, perception and autonomy.

---

## Historical

The Station-mediated phase (Phases A–F, including the hardware-validated LF V25 oracle) is
preserved and indexed at
[`09_Logs/Historical/NormaCore_MATDOG_Archive/`](../../09_Logs/Historical/NormaCore_MATDOG_Archive/README.md).
None of it is current runtime or current calibration.
