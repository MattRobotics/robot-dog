# ST3215 provisioning campaign — 17/17 PASS

**Date:** 2026-08-27
**Tool:** MATDOG ST3215 Provisioner V1, freeze `build_freeze_20260827_v6`
**Profile applied:** [`MATDOG_C018_V1`](../../../01_Docs/02_Architecture/MATDOG_ST3215_C018_V1_PROFILE.md)
**Result:** 17 units provisioned, verified warm and cold, **17/17 PASS**

This directory is the authoritative evidence set for the ST3215 provisioning campaign that closes
the MATDOG bench phase. Every claim below is reproduced from the session artifacts committed here.

---

## Final allocation — physical unit → joint → bus ID

Machine-readable: [`06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml`](../../../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml)

| Unit | Joint | ID | Src ID | Recode | Old offset | Final offset | Final RAW | Err | Lock | Torque | Cold | TL after cold | FW | Session |
|---|---|---:|---:|---|---:|---:|---:|---:|---:|---|---|---:|---|---|
| **M22** | LF_HIP | 13 | 22 | RECODED | -891 | 0 | 2048 | +0 | 1 | OFF | PASS | 1000 | V5 | `M22__20260827_134852Z` |
| **ELR01** | LF_UPPER | 12 | 6 | RECODED | 2045 | 0 | 2048 | +0 | 1 | OFF | PASS | 1000 | V6 | `ELR01__20260827_140449Z` |
| **M33** | LF_LOWER | 11 | 33 | RECODED | -470 | 0 | 2049 | +1 | 1 | OFF | PASS | 1000 | V6 | `M33__20260827_140704Z` |
| **NEW01** | RF_HIP | 23 | 1 | RECODED | 85 | 0 | 2047 | -1 | 1 | OFF | PASS | 1000 | V4 | `NEW01__20260827_122920Z` |
| **ELR03** | RF_UPPER | 22 | 7 | RECODED | 0 | 0 | 2049 | +1 | 1 | OFF | PASS | 1000 | V5 | `ELR03__20260827_130419Z` |
| **NEW03** | RF_LOWER | 21 | 1 | RECODED | 85 | 0 | 2048 | +0 | 1 | OFF | PASS | 1000 | V5 | `NEW03__20260827_131754Z` |
| **NEW06** | RH_HIP | 33 | 1 | RECODED | 85 | 0 | 2047 | -1 | 1 | OFF | PASS | 1000 | V5 | `NEW06__20260827_132535Z` |
| **ELR02** | RH_UPPER | 32 | 1 | RECODED | -1900 | 0 | 2049 | +1 | 1 | OFF | PASS | 1000 | V5 | `ELR02__20260827_132834Z` |
| **NEW05** | RH_LOWER | 31 | 1 | RECODED | 85 | 0 | 2049 | +1 | 1 | OFF | PASS | 1000 | V5 | `NEW05__20260827_133055Z` |
| **M43** | LH_HIP | 43 | 43 | n/a (already target) | -740 | 0 | 2049 | +1 | 1 | OFF | PASS | 1000 | V5 | `M43__20260827_133313Z` |
| **M42** | LH_UPPER | 42 | 42 | n/a (already target) | 979 | 0 | 2048 | +0 | 1 | OFF | PASS | 1000 | V5 | `M42__20260827_133526Z` |
| **M41** | LH_LOWER | 41 | 41 | n/a (already target) | -1824 | 0 | 2049 | +1 | 1 | OFF | PASS | 1000 | V5 | `M41__20260827_133737Z` |
| **M31** | NECK_ROTATION | 51 | 31 | RECODED | -2021 | 0 | 2048 | +0 | 1 | OFF | PASS | 1000 | V5 | `M31__20260827_133936Z` |
| **M11** | NECK_PITCH | 52 | 11 | RECODED | 127 | 0 | 2049 | +1 | 1 | OFF | PASS | 1000 | V5 | `M11__20260827_134139Z` |
| **NEW04** | HEAD_ROTATION | 53 | 1 | RECODED | 85 | 0 | 2048 | +0 | 1 | OFF | PASS | 1000 | V5 | `NEW04__20260827_134334Z` |
| **NEW02** | HEAD_PITCH | 54 | 1 | RECODED | 85 | 0 | 2049 | +1 | 1 | OFF | PASS | 1000 | V5 | `NEW02__20260827_134520Z` |
| **ELR04** | JAW | 55 | 8 | RECODED | -1733 | 0 | 2049 | +1 | 1 | OFF | PASS | 1000 | V5 | `ELR04__20260827_134707Z` |

`Err` is the physical RAW centre error in ticks against the 2048 target; acceptance is ±1.

---

## Verified acceptance contract

Every clause below was re-verified from the committed session reports, not carried over from a
summary:

| Clause | Result |
|---|---|
| 17/17 PASS | ✅ 17 PASS |
| model word `0x03` = 777 | ✅ all 17 |
| `PositionOffset` = 0 | ✅ all 17 |
| physical RAW centre 2048 ±1 | ✅ all 17 (range 2047–2049, error −1…+1) |
| `Lock` = 1 | ✅ all 17 |
| Torque OFF at exit | ✅ all 17 |
| true cold cycle PASS | ✅ all 17 |
| final ID = target ID | ✅ all 17 |
| old ID silent after recode | ✅ 14 recoded, 3 already at target |
| `TorqueLimit` after cold = 1000 | ✅ all 17 (`OBSERVED_NOT_ASSERTED`) |
| canonical profile exact | ✅ single fingerprint `ae3ae5ce6ddda1c0…` on all 17 |

ID recode outcomes: **14 × `RECODED`** (target responds, old ID silent, state verified) and
**3 × `SKIPPED_SOURCE_EQUALS_TARGET`** (M41→41, M42→42, M43→43 already held their target ID).

---

## ⚠️ PROFILE EEPROM WRITE — NOT HARDWARE EXERCISED

> **The canonical-profile EEPROM write branch was never physically executed on a servo.**

Across the **entire** campaign — all 22 sessions including retries — every session logged:

```text
DELTA_COUNT WRITE=0 SKIP=20
```

Aggregated over the 22 sessions that is **440 SKIP and 0 WRITE**. Every unit already held all 20
canonical register values, so the provisioner correctly skipped each one. The write path is
implemented and statically audited, but it has **no hardware evidence** and must not be described
as hardware-validated.

### What *was* exercised on real hardware

| Operation | Evidence |
|---|---|
| EEPROM unlock → write → readback → relock | `PositionOffset` `0x1F` × 17, `Lock` `0x37` × 62 |
| Centering to physical RAW 2048 | all 17, `TorqueLimit` 300 / `GoalSpeed` 365 / `Acc` 50 |
| `PositionOffset` → 0 | all 17 |
| ID recode | 14 units |
| Warm verification | all 17 |
| True cold power cycle | all 17 |
| Acceptance gate ±1 | all 17 |

So the EEPROM write *mechanism* is hardware-proven; the *profile-register* write path is not.

---

## Session accounting — 22 directories, 17 authoritative

The campaign produced **22** session directories: 17 authoritative finals plus **5 retries**.

| Unit | Attempts | Authoritative session |
|---|---:|---|
| NEW01 | 4 | `NEW01__20260827_122920Z` (3 earlier attempts failed) |
| ELR03 | 2 | `ELR03__20260827_130419Z` |
| ELR01 | 2 | `ELR01__20260827_140449Z` |
| all other 14 units | 1 each | first attempt |

All five failures aborted with `ProvisionError: EXECUTE_RESULT PASS missing` — the evidence
contract refused to certify, which is the designed behaviour. Every failed session recovered with
torque OFF and EEPROM relocked.

Complete listing including retries: [`all_sessions_index.tsv`](all_sessions_index.tsv).
Authoritative finals: [`final_session_index.tsv`](final_session_index.tsv).

---

## Firmware heterogeneity — read this before citing "17/17 on V6"

The 17 authoritative sessions were **not** all executed with the V6 firmware. V6 was built and
frozen at 14:03 UTC, part-way through the campaign:

| Provisioner firmware | Units | Which |
|---|---:|---|
| **V4** | 1 | NEW01 |
| **V5** | 14 | ELR02, ELR03, ELR04, M11, M22, M31, M41, M42, M43, NEW02–NEW06 |
| **V6** | 2 | ELR01, M33 |

V6 is the **final frozen tool** and the version that resolved the ELR01 boundary blocker. For
units not near the position-domain boundary, V5 and V6 centering behave identically, and every
unit passed the same acceptance contract. But the accurate statement is:

> 17/17 units PASS the same acceptance contract; the frozen deliverable tool is V6; only 2 units
> were physically provisioned by V6.

Each session report pins its own firmware source and binary SHA256 with `match: true`.

---

## V5 → V6 — the ELR01 boundary blocker

ELR01 held an old `PositionOffset` of **2045**, placing its centre goal at displayed position
**3** — three ticks from the bottom of the `0..4095` domain.

### V5 failure (`ELR01__20260827_135044Z`) — correct refusal

```text
CENTER_PLAN OLD_OFFSET=2045 PRESENT=253 PHYSICAL_RAW=2298 CENTER_GOAL_DISPLAYED=3 TARGET_PHYSICAL_RAW=2048
MOTION_SUMMARY START=253 TARGET=3 FINAL=5 ERROR=2
CENTER_SETTLED DISPLAYED=5 PHYSICAL_RAW=2050 ERROR_TICKS=2 TOLERANCE=1
FAULT REASON=CENTER_STAGING_GOAL_OUT_OF_DOMAIN
RECOVERY TORQUE_OFF=OK EEPROM_LOCK=OK
EXECUTE_RESULT FAIL
```

Terminal deadband left the servo 2 ticks past target. V5's correction would have staged *below*
the approach point — at displayed −7, outside the domain. V5 refused rather than wrap, and
recovered cleanly. **This was correct behaviour, not a defect.**

### V6 boundary-safe centering

| Property | V6 behaviour |
|---|---|
| Normal staging | below the approach point |
| Low-boundary staging | **above** the approach point |
| Wrap 4095 ↔ 0 | never |
| Approach | domain-guarded |
| Acceptance | ±1, unchanged |
| Max corrections | 3 |
| Max bias | ±3 |
| `GoalPosition` | unsigned `0..4095` |

### V6 hardware validation on ELR01 (`ELR01__20260827_140449Z`)

```text
CENTER_SETTLED DISPLAYED=5 PHYSICAL_RAW=2050 ERROR_TICKS=2 TOLERANCE=1
CENTER_CORRECTION BEGIN ATTEMPT=1 OF=3 ERROR_TICKS=2 BIAS_TICKS=0  STAGING_DISPLAYED=15 APPROACH_DISPLAYED=3
CENTER_CORRECTION_SETTLED ATTEMPT=1 BIAS_TICKS=0  DISPLAYED=5 PHYSICAL_RAW=2050 ERROR_TICKS=2
CENTER_CORRECTION BEGIN ATTEMPT=2 OF=3 ERROR_TICKS=2 BIAS_TICKS=-2 STAGING_DISPLAYED=13 APPROACH_DISPLAYED=1
CENTER_CORRECTION_SETTLED ATTEMPT=2 BIAS_TICKS=-2 DISPLAYED=3 PHYSICAL_RAW=2048 ERROR_TICKS=0
STAGE_RESULT NAME=CENTERING PASS
```

Attempt 1 staged **above** the approach (15 → 3) with zero bias and still settled at +2. Attempt 2
applied bias **−2**, staged 13 → approached 1, and landed on physical RAW **2048**, error **0**.
No wrap occurred and the domain guard held throughout.

---

## Tool verification — reproduced during this audit

The V6 freeze was re-verified from scratch:

```text
build_freeze_20260827_v6/SHA256SUMS   4/4 OK
  matdog_st3215_provisioner_v1.ino          680b2715e6896c296c0fc63e7a72bc520a471977891478b2cca9b0f6472e2327
  matdog_st3215_provisioner_v1.py           222c4c6c9395226afe3b845021af362a85d825a1778f4530b0213e9c58747441
  matdog_st3215_provisioner_v1.ino.bin      3510997674b85e064c50d1e3b61abbcf5659b736f7811b1b40ada5d101241201
  matdog_st3215_provisioner_v1.ino.merged.bin  80dd639ddcd477b5d0277587b1b4b676dcdae32786cc0bc58a8efae18b28780a

--self-test        PASS (450/450 checks)
--static-audit     ok: true, hardware_freeze_blocked: false
--verify-firmware  source match: true, binary match: true
```

FQBN:

```text
esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi
```

---

## Characterization constants

Selected from the NEW01 characterization campaign and compiled into the firmware. Each session
logs them as `SELECTED_CONSTANT … SOURCE=NEW01_CHARACTERIZATION`:

| Constant | Value |
|---|---:|
| `PRIME_MAX_DELTA_TICKS` | 1 |
| `COLD_ABSENCE_DEBOUNCE_MS` | 1500 |
| `COLD_RETURN_STABLE_SAMPLES` | 10 |

Evidence: [`characterization_sessions/`](characterization_sessions/).

---

## Directory contents

```text
sessions/                    all 22 session directories (6 RAW 71-byte state snapshots,
                             report JSON, console log and SHA256SUMS each)
persistence_audit/           NEW01 and ELR03 persistence audits, ELR03 probe
characterization_sessions/   NEW01 characterization runs and the C018 reset/reboot audit
source_survey/               read-only source signature survey evidence
final_session_index.tsv      the 17 authoritative sessions
all_sessions_index.tsv       all 22 sessions including retries
SHA256SUMS.txt               manifest of everything in this directory
```

Per-session RAW state slots: `01_before`, `02_profile`, `03_centered_old_offset`,
`04_offset_zero`, `05_after_recode_warm`, `06_after_cold` — each a 71-byte full register dump.

---

## ⚠️ This is provisioning, not calibration

These 17 units are provisioned to a known persistent state with `PositionOffset = 0`. That is a
**servo-level** guarantee only. It says nothing about joint zero, mechanical mounting, or
direction conventions.

All robot calibration is **RESET** as of 2026-08-27 —
see [MATDOG_CALIBRATION_RESET_2026-08-27.md](../../Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md).

## Related

- [MATDOG_C018_V1 canonical profile](../../../01_Docs/02_Architecture/MATDOG_ST3215_C018_V1_PROFILE.md)
- [Bench QC V6.1](../ST3215_Bench_QC_2026-08-24/README.md)
- [Frozen bench tools](../../../05_Firmware/ST3215_Bench_Tools/README.md)
- [ST3215 evidence index](../../ST3215_EVIDENCE_INDEX.md)
