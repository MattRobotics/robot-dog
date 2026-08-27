# MATDOG — Custom Quadruped Robot

A custom 17-DOF quadruped with an articulated head, developed by Matt Robotics in Italy. This
repository is the **single active engineering source of truth** for the whole robot.

> ## ⚠️ CURRENT STATE — REASSEMBLY IN PROGRESS · CALIBRATION RESET REQUIRED
>
> As of **2026-08-27** all 17 servos have been bench-provisioned and are being **physically
> remounted** (12 leg servos, then 5 head/jaw servos).
>
> - **All robot calibration must be redone from zero.** Old digital zero / q0 values are
>   **historical evidence, not active truth**.
> - **`PositionOffset = 0` is the baseline** on all 17 units and must stay that way.
> - **No hardware motion may be commanded from stale calibration** — this is machine-enforced.
> - **Full recalibration must complete before any stand, gait or load-bearing attempt.**
>
> → [**Calibration reset**](09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md)

---

## Current hardware

```text
high-level host      ASUS Ubuntu (dev) · Jetson Orin Nano Super (final onboard)
    ↓ native USB 2.0 Full-Speed / USB CDC        D− = GPIO19, D+ = GPIO20
ESP32-S3 motion coprocessor                      ← owns the ST3215 bus
    ↓ UART  GPIO17 TX → driver RX, GPIO18 RX ← driver TX, shared GND
Seeed Bus Servo Driver                           ← servo-bus electrical layer
    ↓ Feetech serial bus, 1 Mbps
17 × Feetech ST-3215-C018                        12 leg + 5 head/jaw
```

The **ESP32-S3 coprocessor exclusively owns and directly operates the servo bus** — demonstrated in
practice, not aspirational. The high-level host issues joint/motion intent; it does not drive the
bus directly.

**Compute split (decided):** the host owns ROS 2 / MoveIt 2, AI, vision, voice, planning and UI;
the ESP32-S3 owns bus control, deterministic servo control, gait execution, operational IK, IMU,
power telemetry, watchdog/safety and real-time motion execution.

Electronics decisions (Seeed driver, no CAN, single ATO main fuse, custom busbar, …):
[`04_Electronics/README.md`](04_Electronics/README.md).

## Current software direction

| Layer | Direction | Status |
|---|---|---|
| High-level | **ROS 2 / MoveIt 2**, AI, vision, voice, planning, UI | decided stack — **not yet integrated** |
| Coprocessor | ESP32-S3 deterministic servo/motion/gait/IK/safety/telemetry layer | **bus ownership validated**; motion & safety firmware **decided but not written** |
| Host link | native USB 2.0 Full-Speed / USB CDC | **decided and in use**; packet protocol still TBD |
| NormaCore Station | **optional / legacy / reference** | **not** a required actuation owner |

**Validated:** the ESP32-S3 can exclusively own and directly operate the ST3215 bus; the direct
driver path is proven by QC and provisioning; 17 servos were provisioned without Station.
**Decided but not implemented:** the complete operational motion firmware, the final
ROS 2 / MoveIt 2 integration, and the complete Jetson onboard integration.

Station was formerly designated sole owner of the ST3215 bus. It no longer is, and no current
MATDOG operation depends on it. Reusing good upstream NormaCore code remains fine.

→ [**ARCHITECTURE.md**](01_Docs/02_Architecture/ARCHITECTURE.md) — full current architecture, with
every claim tagged validated / decided / TBD.

## Validated

| Item | Result |
|---|---|
| [Bench QC / Servo Quality Audit V6.1](09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md) | 26/26 runs COMPLETE, checksum PASS, 0 protective stops |
| [`MATDOG_C018_V1` persistent profile](01_Docs/02_Architecture/MATDOG_ST3215_C018_V1_PROFILE.md) | **FROZEN** — identical fingerprint on all 17 units |
| [ST3215 provisioning](09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/README.md) | **17/17 PASS** — centre 2048 ±1, offset 0, cold-verified |
| [Provisioner V6 + frozen bench tools](05_Firmware/ST3215_Bench_Tools/README.md) | self-test 450/450, static audit PASS |
| [Servo allocation](06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml) | unit → joint → bus ID fixed for all 17 |
| Geometry Compiler V5 / Phase 1B | CLOSED — URDF, meshes and geometric endpoints remain valid |

Geometry is unaffected by the rebuild: it describes the design, not the build.

### Known limits

- **Profile EEPROM write path is NOT hardware-exercised** (`WRITE=0 SKIP=20` on all 22 sessions).
- Only 2 of 17 units were provisioned by the final V6 firmware (14 × V5, 1 × V4).
- Host ↔ ESP32-S3 **packet protocol** is TBD (the USB CDC transport itself is decided and in use).
- ROS 2 / MoveIt 2 is **not** integrated; the operational motion firmware is **not** written.
- The 3S power-load analysis covers 12 servos; the 17-servo budget needs re-evaluation.
- Parking / path-safety gate still `passed=False` on all four legs (pre-existing).
- Three LF geometric endpoints remain `HARDWARE_CONTRADICTED`; RF/RH/LH endpoints are model
  candidates, never hardware-confirmed.
- The 5 head/jaw joints have **never been calibrated**.

## Next

```text
1. finish assembly        12 leg servos, then 5 head/jaw servos
                          hold physical RAW ~2048 while mounting;
                          mount horns and links directly in the calibration pose

2. full recalibration     from zero, on the new installation
                          re-capture digital q0 — measured, never imported
                          verify mapping and directions on all 17 joints

3. controlled bring-up    read-only FK → supervised suspended motion
                          → gradual load transfer

4. then resume            stand, trajectories, gait, walking
```

---

## Robot definition

Canonical REV00 package: `03_CAD/URDF/matt_robodog_rev00/`

```text
front-to-rear hip spacing: 225 mm      X = forward
left-to-right hip spacing:  95 mm      Y = left
hip-to-knee segment:        90 mm      Z = up
knee-to-foot interface:    110 mm      metres and radians, right-handed
knee-to-contact frame:     118.1 mm
nominal body height:      ~150 mm
```

### Servo allocation — 17 units, current as of 2026-08-27

```text
        joint                 bus ID   physical unit
LF      hip / upper / lower   13/12/11  M22   / ELR01 / M33
RF      hip / upper / lower   23/22/21  NEW01 / ELR03 / NEW03
RH      hip / upper / lower   33/32/31  NEW06 / ELR02 / NEW05
LH      hip / upper / lower   43/42/41  M43   / M42   / M41
HEAD    neck rotation             51    M31
        neck pitch                52    M11
        head rotation             53    NEW04
        head pitch                54    NEW02
        jaw                       55    ELR04
```

Authoritative: [`MATDOG_SERVO_ALLOCATION.yaml`](06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml)

> **Replaces the historical 12-servo mapping.** 14 of 17 units were recoded during provisioning —
> historical records keyed by bus ID will associate the **wrong joint**.

Canonical leg order and trot diagonals: `[LF, RF, RH, LH]` · `LF+RH` · `RF+LH`

## Permanent calibration and control rules

- The **ESP32-S3 coprocessor** is the operational owner of the ST3215 bus; exactly one owner at a time.
- `GoalPosition` is unsigned `0..4095`; signed wrap is forbidden.
- **`PositionOffset = 0` is the baseline on all 17 servos.** Mechanical mounting error is corrected
  mechanically, never by rewriting `PositionOffset`.
- **`CalibrationOfs`, one-key-middle, factory reset and broadcast write are permanently forbidden.**
- The C018 model word is read from register `0x03` (expected `777`), never from `0x00`.
- No hardware motion from stale calibration; full recalibration precedes any stand or gait.
- No physical movement without successful offline gates and explicit current-session authorization.
- No EEPROM `PositionOffset` write without separate explicit authorization.
- `UNRESOLVED ≠ PASS` · `DIAGNOSTIC ≠ EXECUTABLE` · `GEOMETRIC CONTACT ≠ MOTION AUTHORIZATION`.
- No detector threshold may be changed merely to make another leg pass.
- FRONT and HIND geometry derive from the actual model, never assumed identical.
- Stale or mismatched geometry provenance must fail closed.
- No merge into `main` without explicit authorization; no force-push; no destructive removal of evidence.

## Repository structure

```text
01_Docs/        architecture and stable technical references
02_BOM/         components, suppliers and costs
03_CAD/         CAD, URDF, meshes and mechanical exports
04_Electronics/ wiring, power and servo mapping
05_Firmware/    ESP32-S3 bench tools (frozen) and future robot firmware
06_Software/    calibration, kinematics, gait and control
07_Media/       images, renders and videos
08_Tests/       repeatable validation procedures
09_Logs/        decisions, reports, evidence and historical archive
```

## Canonical records

| Record | Covers |
|---|---|
| [ARCHITECTURE.md](01_Docs/02_Architecture/ARCHITECTURE.md) | current architecture — start here |
| [⚠️ Calibration reset](09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md) | read before touching hardware |
| [ST3215 canonical archive handoff](09_Logs/Development_Log/2026-08-27_ST3215_CANONICAL_ARCHIVE_HANDOFF.md) | current state and next steps |
| [ST3215 evidence index](09_Logs/ST3215_EVIDENCE_INDEX.md) | every artifact, in Git and on the ASUS archive |
| [MATDOG_C018_V1 profile](01_Docs/02_Architecture/MATDOG_ST3215_C018_V1_PROFILE.md) | canonical persistent servo profile |
| [Frozen bench tools](05_Firmware/ST3215_Bench_Tools/README.md) | QC, survey and provisioner freezes |
| [Geometry Compiler](09_Logs/Validation_Reports/Geometry_Compiler/README.md) | offline geometry pipeline |
| [REPOSITORY_VERIFICATION_INDEX.md](REPOSITORY_VERIFICATION_INDEX.md) | verification index |

## History

MATDOG previously ran a **Station-mediated, 12-servo** architecture, with calibration development
split into `MattRobotics/norma-core`. That phase — including the hardware-validated **LF V25**
oracle — is preserved, classified and indexed in one place:

→ [**09_Logs/Historical/**](09_Logs/Historical/README.md)

Nothing in that archive is current runtime, current calibration, or authorization to move hardware.

---

Built and documented by Matt Robotics.
