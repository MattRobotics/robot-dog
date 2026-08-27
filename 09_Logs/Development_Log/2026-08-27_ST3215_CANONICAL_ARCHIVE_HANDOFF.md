# Handoff — ST3215 bench phase closed, reassembly begins

**Date:** 2026-08-27
**Phase closed:** ST3215 bench qualification and provisioning
**Phase entered:** physical reassembly → full recalibration
**Branch:** `matdog/st3215-qc-provisioning-canonical-archive`

---

## Where MATDOG is right now

The servo bench phase is **complete**. Seventeen ST-3215-C018 units have been quality-audited,
characterized, provisioned to a single canonical persistent profile, and verified through a true
cold power cycle. They are now being physically remounted into the robot.

```text
Bench QC V6.1 (22 units)  →  allocation (17)  →  provisioning 17/17 PASS  →  REASSEMBLY (in progress)
                                                                                    ↓
                                                                       FULL RECALIBRATION (required, not started)
```

**The robot cannot move.** All joint calibration is reset and no motion is authorized until
recalibration completes.

---

## What is done and trustworthy

| Item | Status | Evidence |
|---|---|---|
| Bench QC V6.1 on 22 units | 26/26 `COMPLETE`, checksum `PASS` | [QC campaign](../Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md) |
| 18-unit blind/independent audit | complete, RAW provably unmutated | [`blind_audit/`](../Validation_Reports/ST3215_Bench_QC_2026-08-24/blind_audit/) |
| `MATDOG_C018_V1` profile | frozen, identical fingerprint on all 17 | [profile](../../01_Docs/02_Architecture/MATDOG_ST3215_C018_V1_PROFILE.md) |
| Provisioning | **17/17 PASS** | [provisioning campaign](../Validation_Reports/ST3215_Provisioning_2026-08-27/README.md) |
| Unit → joint → ID allocation | fixed and verified | [`MATDOG_SERVO_ALLOCATION.yaml`](../../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml) |
| Tool freezes | V6 provisioner + survey verified | [frozen tools](../../05_Firmware/ST3215_Bench_Tools/README.md) |
| Geometry / URDF | unchanged and still valid | Phase 1B, Geometry Compiler V5 |

Every one of these was re-verified from primary artifacts during this audit — not carried over
from prior summaries.

---

## What is NOT true, despite appearances

These are the corrections this handoff exists to make. Each is a claim that a reasonable reader
might otherwise get wrong.

### 1. The profile EEPROM write path has never run on hardware

All 22 sessions logged `DELTA_COUNT WRITE=0 SKIP=20` — **440 SKIP, 0 WRITE**. Every unit already
held the canonical values, so the provisioner correctly skipped all 20 registers.

The write branch is implemented and statically audited but carries **no hardware evidence**. Do
not call it hardware-validated.

The EEPROM write *mechanism* **was** exercised — `PositionOffset` (`0x1F`) ×17 and `Lock` (`0x37`)
×62. It is specifically the *canonical-profile register* path that is unproven.

### 2. "17/17 on V6" is imprecise

Only **2** units (ELR01, M33) were provisioned by V6. **14** ran V5 and **1** ran V4. V6 was built
mid-campaign at 14:03 UTC.

V6 is the final frozen deliverable and the version that fixed the ELR01 boundary blocker. The
accurate claim is: *17/17 pass the same acceptance contract; the frozen tool is V6; 2 units were
physically provisioned by it.*

### 3. QC V6.1 firmware was NOT EEPROM-write-incapable

Earlier documentation said it was. Wrong. The binary contains `@NORMALIZE_MATDOG <id>` →
`runNormalizeMatdog()` with five real EEPROM writes.

The campaign runner never invoked it — the command is absent from the runner's five-command
surface, no log contains it, and all 26 logs record `EEPROM_WRITES : NONE`. The RAW data stands.
The frozen source is **not** retroactively corrected; only the documentation is.

### 4. Old calibration is not current state

Digital zero, q0 and LF V25 results describe an installation that no longer exists. They are
preserved as historical evidence and must never be imported as truth.
See [calibration reset](../Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md).

### 5. Bus IDs changed

Fourteen units were recoded. M22 is now ID 13 (`LF_HIP`); M31 is now ID 51 (`NECK_ROTATION`).
Any historical record keyed by bus ID will associate the **wrong joint** against the current
robot.

---

## Count reconciliation — 12 / 18 / 22 / 17

Four numbers circulate. They are different cohorts, not inconsistent counts of one thing:

| Count | Cohort | Date |
|---:|---|---|
| 12 | `postimpact_definitive_v2` — M-series, **separate earlier campaign** | 2026-08-14 |
| 18 | Bench QC blind audit cohort (12 M + 6 NEW) | 2026-08-24 |
| 22 | Total Bench QC units (+4 ELR); 26 runs incl. 4 `_NORM` re-measurements | 2026-08-24/25 |
| 17 | Allocated into the robot | 2026-08-27 |

Not allocated: M12, M13, M21, M23, M32.

---

## The robot is now 17 joints, not 12

The repository previously described a 12-servo quadruped. MATDOG now has **17** joints:

```text
legs (12)  LF/RF/RH/LH × hip, upper, lower
head  (5)  NECK_ROTATION 51, NECK_PITCH 52, HEAD_ROTATION 53, HEAD_PITCH 54, JAW 55
```

The 5 head joints have **never been calibrated**. `MATDOG_JOINT_CALIBRATION.yaml` still contains
only the 12 leg joints — by design, since the head joints have no calibration data to record yet.

---

## Open items and risks

### Calibration status enum still asserts a stale value

`robot.calibration_status` still reads `DIGITAL_ZERO_CALIBRATED_AND_VERIFIED`. This was left
**deliberately unchanged** because six tools hard-assert that exact string and would change
runtime behaviour if it moved:

```text
matdog_visual_zero_pose_probe.py:132   matdog_capture_visual_zero.py:205
matdog_apply_visual_zero.py:134        matdog_calibration_validate.py:104
matdog_live_joint_monitor.py:105       matdog_leg_fk_live.py:101
```

An additive `calibration_reset:` block now carries the authoritative state, and a prominent banner
sits at the top of the file.

> **Follow-up required:** introduce a proper reset enum together with updates to all six consumers
> so they fail closed on stale calibration. Until then, treat those six tools as **unsafe to run
> against hardware**.

### NormaCore ST3215 model register defect — still open

```text
norma-core: software/drivers/st3215/src/protocol/memory.rs:46
            ModelNumber => (0x00, 2, "Model Number"),
```

The C018 exposes its model word at **`0x03`** (expected 777). The `0x00` declaration is a
source-map defect for this variant. MATDOG tooling reads `0x03` and is unaffected.

**Not fixed here.** This audit deliberately made no opportunistic changes to `norma-core`. Recorded
as a dependency risk for whoever generalizes the calibrator.

### Other open limits

| Limit | Status |
|---|---|
| Baud register write | not implemented in V1 — verification only |
| Gait / head / jaw runtime torque limits | TBD after recalibration |
| Parking / path-safety gate | still `passed=False` on all four legs (pre-existing, Phase 1B) |
| Three LF hardware-contradicted endpoints | still `UNKNOWN` (pre-existing) |
| RF/RH/LH geometric endpoints | `GEOMETRIC_ENDPOINT_CANDIDATE` — never hardware-confirmed |

---

## Next concrete steps

1. **Finish assembly** — 12 leg servos, then 5 head servos. Hold physical RAW ≈ 2048 while
   mounting; mount horns and links directly in the calibration pose.
2. **Full recalibration from zero** — no reuse of any pre-2026-08-27 calibration data. Re-capture
   digital q0 on the new installation; verify mapping and directions on all 17 joints.
3. **Controlled bring-up** — only after recalibration: read-only FK verification, then supervised
   suspended motion, then load transfer.

Throughout: `PositionOffset = 0` stays the baseline. Never compensate mechanics by rewriting
EEPROM.

---

## Entry points

- [ST3215 evidence index](../ST3215_EVIDENCE_INDEX.md) — everything, with locations and hashes
- [⚠️ Calibration reset](../Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md) — read before touching hardware
- [Provisioning campaign](../Validation_Reports/ST3215_Provisioning_2026-08-27/README.md)
- [Bench QC campaign](../Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md)
- [MATDOG_C018_V1 profile](../../01_Docs/02_Architecture/MATDOG_ST3215_C018_V1_PROFILE.md)
- [Frozen bench tools](../../05_Firmware/ST3215_Bench_Tools/README.md)
