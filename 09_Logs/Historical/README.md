# Historical index — superseded MATDOG phases

> **Nothing indexed here is current operational truth.**
> Current architecture: [`01_Docs/02_Architecture/ARCHITECTURE.md`](../../01_Docs/02_Architecture/ARCHITECTURE.md)

MATDOG has been through one major architectural change and one physical rebuild. Both invalidated
large amounts of previously-correct documentation. Rather than rewrite history, this index records
what is superseded, why, and where the current truth lives.

---

## The two events

### 1. Architecture change — Station → ESP32-S3 (2026-08-27)

NormaCore Station was originally the designated **sole owner** of the ST3215 serial bus. Direct
ESP32-S3 ST3215 control has since been demonstrated and selected instead.

**Consequence:** every document describing Station as the required bus owner, the required
calibration path or the required provisioning path is superseded as *current* architecture.

→ [NormaCore MATDOG archive](NormaCore_MATDOG_Archive/README.md)

### 2. Physical rebuild — 12 servos → 17, all remounted (2026-08-27)

All servos were removed, bench-provisioned to `PositionOffset = 0`, **14 of 17 recoded to new bus
IDs**, and remounted. Five head/jaw servos were added.

**Consequence:** every joint zero, q0, direction, limit and bus-ID mapping recorded before this
date describes an installation that no longer exists.

→ [Calibration reset](../Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md)

---

## Classification of Station-era material

Every occurrence of a Station-ownership or 12-servo claim in this repository was audited and
classified. Nothing was deleted.

### CURRENT — rewritten to the new architecture

| Document | Action |
|---|---|
| `README.md` | rewritten — 17 servos, ESP32-S3 ownership, Station optional |
| `01_Docs/02_Architecture/ARCHITECTURE.md` | rewritten — canonical current architecture |
| `04_Electronics/README.md` | rewritten — control chain and 17-servo mapping |
| `06_Software/Matdog_Core/kinematics/MATDOG_HARDWARE_SAFE_MODE_PREFLIGHT.md` | bus-ownership corrected; safety reasoning preserved; superseded banner |
| `01_Docs/01_Analysis/ST3215_Quadruped_3S_Power_Load_Analysis.md` | 17-servo re-evaluation caveat added |

### SUPERSEDED_DOCUMENTATION — banner added, content preserved

Unmistakably marked, never rewritten:

```text
06_Software/Matdog_Core/calibration/MATDOG_DIGITAL_ZERO_CALIBRATION.md
06_Software/Matdog_Core/calibration/MATDOG_MECHANICAL_ENDSTOP_CALIBRATION_PLAN.md
06_Software/Matdog_Core/calibration/MATDOG_MECHANICAL_ENDSTOP_GEOMETRY_CHECKPOINT_2026-07-20.md
06_Software/Matdog_Core/calibration/MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md
06_Software/Matdog_Core/calibration/MATDOG_CALIBRATION_CANONICAL_HANDOFF_PHASE1B_CLOSED_2026-08-09.md
06_Software/Matdog_Core/calibration/MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md
01_Docs/02_Architecture/Milestone_I/MATDOG_JOINT_AND_SERVO_CONTRACT.md
01_Docs/02_Architecture/Project_Reports/2026-06-25_MATDOG_Project_State_and_Next_Steps.md
```

### HISTORICAL_EVIDENCE — dated records, left at their original paths

Dated logs and validation reports keep their paths so provenance, hashes and inbound links stay
intact. Banners added where they read as operational instructions:

```text
09_Logs/Development_Log/2026-07-06_Visual_Zero_Calibration_and_Static_Control_Checkpoint.md
09_Logs/Development_Log/2026-07-10_C5R_POST_DIGITAL_ZERO_HANDOFF.md
09_Logs/Development_Log/2026-07-21_NATIVE_NORMACORE_CALIBRATOR_HANDOFF.md
09_Logs/Validation_Reports/2026-06-17_ST3215_Bus_Validation.md
09_Logs/Validation_Reports/2026-07-06_Visual_Zero_and_Static_Actuation_Validation.md
09_Logs/Calibration/C5_R_digital_recenter/          (2026-07-10 digital recenter)
09_Logs/Calibration/Digital_Zero/                   (2026-08-14 snapshots)
09_Logs/Calibration_Sessions/2026-07-02_initial_approx_reference.yaml
```

> Hashed evidence was **not moved.** Relocating it would break `.sha256` companions, manifest paths
> and inbound links. It is indexed as historical instead.

### CODE_DEPENDENCY_STILL_ACTIVE — documented, not silently rewritten

See [legacy Station code](#legacy-station-code-still-present) below.

---

## Legacy Station code still present

These Python tools genuinely still import or assume Station. They were **not** rewritten in this
pass — doing so would be an unreviewed behavioural change to hardware-touching code.

| File | Station dependency | Status |
|---|---|---|
| `06_Software/Matdog_Core/calibration/matdog_digital_zero_calibration.py` | Station-mediated ST3215 write path | **legacy/transitional** — superseded by the ESP32-S3 provisioner |
| `06_Software/Matdog_Core/calibration/matdog_endstop_station_readonly_watch.py` | Station read-only telemetry | **transitional** — Station remains valid as optional telemetry |
| `06_Software/Matdog_Core/calibration/matdog_visual_zero_pose_probe.py` | Station read path | **legacy** — pending recalibration rework |
| `06_Software/Matdog_Core/calibration/matdog_calibration_validate.py` | validates the Station-era config contract | **retained** — now fail-closed on stale calibration |

**All four are blocked from hardware by the calibration fail-closed gate.** They cannot acquire a
client, serial port or hardware handle while calibration state is
`CALIBRATION_RESET_PENDING_FULL_RECALIBRATION`.

### Are they still needed?

| Tool | Needed for the next phase? |
|---|---|
| `matdog_digital_zero_calibration.py` | **No.** The ESP32-S3 provisioner replaced it. Retained as reference for the recalibration rewrite. |
| `matdog_endstop_station_readonly_watch.py` | **Possibly.** Station stays valid as optional observation/telemetry. |
| `matdog_visual_zero_pose_probe.py` | **Needs rework.** The concept applies; the Station transport does not. |
| `matdog_calibration_validate.py` | **Yes.** It is the config-contract validator and now enforces the reset. |

---

## Archives

| Archive | Contents |
|---|---|
| [NormaCore MATDOG archive](NormaCore_MATDOG_Archive/README.md) | Phases A–F, LF V25 hardware oracle, generic V25 WIP, RF local-only work, MATDOG content from `norma-core/main` |

## Related

- [Canonical architecture](../../01_Docs/02_Architecture/ARCHITECTURE.md)
- [Calibration reset](../Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md)
- [ST3215 evidence index](../ST3215_EVIDENCE_INDEX.md)
