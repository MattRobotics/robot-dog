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
│   ASUS Ubuntu workstation during development   ✅ current    │
│   Jetson Orin Nano Super onboard (final)       🟦 decided    │
│                                                              │
│   ROS 2 / MoveIt 2 high-level stack            🟦 decided    │
│   AI · vision · voice · planning · dashboard   🟦 decided    │
└──────────────────────────────────────────────────────────────┘
             ↓  native USB 2.0 Full-Speed / USB CDC   ✅ in use
             ↓  ESP32-S3 D− = GPIO19, D+ = GPIO20
             ↓  packet/command protocol over CDC      ⬜ TBD
┌──────────────────────────────────────────────────────────────┐
│ DEDICATED MATDOG ESP32-S3 MOTION COPROCESSOR                 │
│   ST3215 bus ownership                         ✅ validated  │
│   direct / native ST3215 driver path           ✅ validated  │
│   provisioning / commissioning / QC utilities  ✅ validated  │
│   deterministic servo control                  🟦 decided    │
│   gait execution                               🟦 decided    │
│   operational IK                               🟦 decided    │
│   IMU acquisition                              🟦 decided    │
│   battery / power telemetry                    🟦 decided    │
│   watchdog and safety                          🟦 decided    │
│   real-time motion execution                   🟦 decided    │
└──────────────────────────────────────────────────────────────┘
             ↓  UART   GPIO17 TX → driver RX
             ↓         GPIO18 RX ← driver TX
             ↓         shared GND
┌──────────────────────────────────────────────────────────────┐
│ Seeed Bus Servo Driver                         🟦 selected   │
│   provides the servo-bus electrical layer                    │
│   ESP32-S3 owns the ST3215 protocol and control              │
└──────────────────────────────────────────────────────────────┘
             ↓  Feetech serial bus, 1 Mbps
┌──────────────────────────────────────────────────────────────┐
│ 17 × Feetech ST-3215-C018                      ✅ validated  │
│   12 leg  +  5 head/jaw                                      │
│   profile MATDOG_C018_V1, PositionOffset = 0                 │
└──────────────────────────────────────────────────────────────┘
```

### Host ↔ coprocessor link — decided

| Property | Value |
|---|---|
| Physical transport | **native USB 2.0 Full-Speed / USB CDC** — decided, and already in use |
| ESP32-S3 USB pins | **D− = GPIO19**, **D+ = GPIO20** |
| Theoretical rate | 12 Mbit/s (USB 2.0 Full-Speed) |
| USB 3.x | neither required nor available on ESP32-S3 |
| Secondary channel | Wi-Fi command/diagnostic link — planned |
| Higher-level packet/command protocol | ⬜ **TBD** — the physical transport is frozen, the protocol carried over it is not |

The frozen bench tools already run over this link: the provisioning campaign used
`USBMode=hwcdc,CDCOnBoot=cdc` and enumerated as
`/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_…`.

> **Distinguish physical transport from protocol.** The transport is decided. The application-level
> packet format, command set and telemetry schema carried over USB CDC are still open.

---

## Ownership rules

1. ✅ The **ESP32-S3 coprocessor is the operational owner of the ST3215 serial bus.**
2. 🟦 The high-level host issues joint/motion intent; it does not drive the servo bus directly.
3. 🟦 The host never sends raw encoder targets in normal operation.
4. 🟦 The **compute responsibility split is decided**: joint→actuator conversion, operational IK,
   gait execution, IMU, power telemetry, watchdog/safety and real-time motion execution reside on
   the **ESP32-S3**. The high-level host owns ROS 2 / MoveIt 2, AI, vision, voice, planning, UI and
   semantic behaviours.
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

## Compute responsibility split — decided

| Jetson Orin Nano Super / high-level host | ESP32-S3 motion coprocessor |
|---|---|
| ROS 2 / MoveIt 2 high-level robotics stack | ST3215 bus ownership |
| AI | deterministic servo control |
| vision | gait execution |
| voice | operational IK |
| planning | IMU acquisition |
| dashboard / UI | battery / power telemetry |
| semantic and high-level behaviours | watchdog / safety |
|  | real-time motion execution |

**Implementation status is not the same as the decision.** Direct ST3215 ownership is
hardware-demonstrated. The runtime motion / gait / IK / watchdog stack is **decided but not yet
implemented** — see [validation scope](#validation-scope--precise-wording).

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

## Validation scope — precise wording

**VALIDATED** — implemented and exercised on real hardware:

- the ESP32-S3 can **exclusively own and directly operate** the ST3215 bus;
- the native/direct driver path is proven by the QC and provisioning campaigns;
- **17 servos were provisioned without Station** in the loop.

**DECIDED** — current architectural decision, implementation partial or absent:

- the ESP32-S3 is the **definitive operational servo-bus owner and motion coprocessor**;
- runtime motion, gait/IK, IMU, power telemetry and watchdog/safety responsibilities reside there;
- USB CDC is the primary host↔coprocessor link; Jetson Orin Nano Super is the final onboard host.

**NOT YET IMPLEMENTED**:

- the complete operational motion firmware;
- the final ROS 2 / MoveIt 2 integration;
- the complete Jetson onboard integration.

> The complete ESP32 operational runtime is **not** validated. Only bus ownership and the direct
> driver path are.

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
| Host ↔ ESP32-S3 **protocol** | ⬜ **TBD** — packet format, command set and telemetry schema. The **physical transport (USB CDC) is decided and in use** |
| ROS 2 / MoveIt 2 integration | 🟦 intended high-level stack; **no integration exists yet** |
| Deterministic motion execution firmware | 🟦 decided; not written |
| Watchdog / safety firmware | 🟦 decided; not written |
| IMU / power / battery telemetry | 🟦 decided; not written |
| Gait / IK / motion firmware | 🟦 decided to live on the ESP32-S3; **not written** |
| Jetson Orin Nano Super onboard host | 🟦 selected; not yet integrated |
| Robot joint calibration | ⚠️ **RESET** — must be redone from zero |

> ROS 2 and MoveIt 2 are the **intended** high-level robotics stack. They are not integrated,
> and nothing in this repository currently depends on them.

---

## Electronics

Current hardware decisions — servo-bus driver, power distribution, protection and cabling — are
recorded in [`04_Electronics/README.md`](../../04_Electronics/README.md).

Summary: **Seeed Bus Servo Driver** for the servo-bus electrical layer · **no CAN transceiver** ·
no dedicated Jetson DC/DC · no separate 5 V logic DC/DC · one removable external ATO main fuse ·
custom motor power busbar · locking 3D-printed cable housings · no bulk servo capacitor initially ·
no TVS initially.

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
5. Freeze the host ↔ ESP32-S3 **protocol** carried over the already-decided USB CDC transport.
6. Build the ESP32-S3 motion/safety/watchdog layer.
7. Recompute four-leg FK; regenerate and audit stand poses.
8. Single-foot trajectories, gait, walking.
9. ROS 2 / MoveIt 2 integration.
10. Jetson Orin Nano Super onboard host, IMU, estimator, perception and autonomy.

---

## Historical

The Station-mediated phase (Phases A–F, including the hardware-validated LF V25 oracle) is
preserved and indexed at
[`09_Logs/Historical/NormaCore_MATDOG_Archive/`](../../09_Logs/Historical/NormaCore_MATDOG_Archive/README.md).
None of it is current runtime or current calibration.
