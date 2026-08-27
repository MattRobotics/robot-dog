# ST3215 bench tools — frozen

ESP32-S3 bench firmware and host runners used to qualify, characterize and provision MATDOG's
Feetech ST-3215-C018 servos. All three tools are **frozen**: the sources here are byte-for-byte
identical to what actually ran on hardware.

> These are **bench** tools. They are not robot flight firmware and are never run with the robot
> assembled and powered for motion. See [`05_Firmware/README.md`](../README.md).

---

## The three tools

| Tool | Purpose | Hardware writes |
|---|---|---|
| [Bench QC V6.1](Bench_QC_V6_1/) | Servo quality audit — motion, thermal, precision, load characterization | none during the campaign |
| [Source Signature Survey V1](Source_Signature_Survey_V1/) | Read-only 71-byte full-state capture | `TorqueEnable=0` only |
| [Provisioner V6](Provisioner_V6/) | Apply `MATDOG_C018_V1`, centre, zero offset, recode ID, verify cold | offset, ID, lock, torque |

Integrity manifest for everything in this directory: [`SHA256SUMS`](SHA256SUMS).

---

## Freeze policy

Every frozen tool in this directory carries:

1. original source, **byte-for-byte**
2. SHA256 of every artifact
3. provenance — where it ran and on what
4. command surface — the complete set of reachable commands
5. toolchain / FQBN
6. test result
7. hardware validation **scope**
8. an explicit statement of **what is not validated**

Sources are never retroactively corrected. Where documentation was wrong, the *documentation* is
corrected and the source is left historical.

### Compiled binaries are not committed

`.ino.bin` (~335 KB) and `.ino.merged.bin` (16 MB each) are **not** in Git. This repository has no
LFS rule for `.bin` and no release-asset workflow, and the binaries are reproducible from the
committed source plus the recorded FQBN.

They are preserved on the ASUS canonical archive:

```text
~/MATDOG/runtime/esp32/matdog_st3215_provisioner_v1/build_freeze_20260827_v6/
~/MATDOG/runtime/esp32/matdog_st3215_source_survey_v1/approved_stack_freeze_20260826/
```

Each freeze directory holds its own `SHA256SUMS`. Binary hashes are recorded in the per-tool
sections below and in every provisioning session report.

---

## 1. Bench QC V6.1

**Directory:** [`Bench_QC_V6_1/`](Bench_QC_V6_1/)
**Campaign:** [09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24](../../09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md)

| Artifact | SHA256 |
|---|---|
| `matdog_servo_commissioning.ino` | `74656fb9187fd2024f8251276b49676d8be9c6455f542c49500cfb30d25630cd` |
| `matdog_qc_runner.py` | `a143958034be46c9a36ce89fca2fb27fab2371e1af730abe2e3eaf265bc1245e` |
| `matdog_qc_unit.sh` | `450b19250acc87e1864a7eb6172ebc308bd583bbbebe9339602447a36d56a635` |

### Runner command surface — complete

```text
@SCAN <lo> <hi>   @READ <id>   @SAFE_OFF <id>   @DUMP_RAW <id>   @QC_FAST <id>
```

### ⚠️ EEPROM capability — corrected

The complete firmware **does** contain `@NORMALIZE_MATDOG <id>` → `runNormalizeMatdog()`, which
performs a real EEPROM unlock and five register writes. Earlier documentation calling this binary
"EEPROM-write-incapable" was wrong.

The campaign runner never invoked it: the command is not in the runner's surface, no campaign log
contains it, and all 26 logs record `EEPROM_WRITES : NONE`. Full analysis and evidence in the
[campaign report](../../09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md#-documentation-correction--normalize_matdog-and-eeprom-capability).

### Hardware validation scope

- **Validated:** 26 QC runs, all `COMPLETE`, checksum `PASS`, 0 performance events, 0 protective stops.
- **Not validated:** `@NORMALIZE_MATDOG` was never exercised by the campaign; it carries no campaign evidence.

RAW format: [`Bench_QC_V6_1/RAW_SCHEMA.md`](Bench_QC_V6_1/RAW_SCHEMA.md).

---

## 2. Source Signature Survey V1

**Directory:** [`Source_Signature_Survey_V1/`](Source_Signature_Survey_V1/)
**Freeze:** `approved_stack_freeze_20260826`

| Artifact | SHA256 |
|---|---|
| `matdog_st3215_source_survey_v1.ino` | `f37fa85a11e333634c18b4abbd991e24797b1891ec5d2e5432b1e4787e5a0c86` |
| `matdog_st3215_source_survey_v1.py` | `7cc77149cde24fe6bf0e8b0f6fef2c6b1bf42c1c45bad09cfb2075859cd05489` |
| `.ino.bin` (ASUS only) | `89ce2a5178b663d7a19b01275ad65662856ce7b7bc114198b5a3e535464f6a12` |

### Command surface — complete

```text
@SCAN 0 253      @SNAPSHOT71 <0..253>      @SAFE_OFF <0..253>
```

**Only permitted servo write:** `TorqueEnable = 0`, via the firmware-gated `@SAFE_OFF`. No EEPROM
write, no motion.

### What it captures

Full **71-byte** register state, model word read from **`0x03`** (expected 777), `PositionOffset`
decoded as int16 LE two's complement, and a SHA256 over the raw capture.

### Test result and scope

```text
RUNNER_SELF_TEST = PASS 225/225
STATIC_COMMAND_SURFACE = PASS
HARDWARE_SURVEY_EXECUTED_AT_FREEZE = NO
ESP32_UPLOAD_EXECUTED_AT_FREEZE = NO
SERIAL_OPENED_AT_FREEZE = NO
```

At freeze time the tool had been statically validated only. Survey evidence subsequently captured
on NEW01 is committed under
[`ST3215_Provisioning_2026-08-27/source_survey/`](../../09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/source_survey/).

Spec: [`MATDOG_ST3215_READONLY_SOURCE_SIGNATURE_SURVEY_SPEC_V1.md`](Source_Signature_Survey_V1/MATDOG_ST3215_READONLY_SOURCE_SIGNATURE_SURVEY_SPEC_V1.md).

---

## 3. Provisioner V6

**Directory:** [`Provisioner_V6/`](Provisioner_V6/)
**Freeze:** `build_freeze_20260827_v6`
**Campaign:** [09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27](../../09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/README.md)

| Artifact | SHA256 |
|---|---|
| `matdog_st3215_provisioner_v1.ino` | `680b2715e6896c296c0fc63e7a72bc520a471977891478b2cca9b0f6472e2327` |
| `matdog_st3215_provisioner_v1.py` | `222c4c6c9395226afe3b845021af362a85d825a1778f4530b0213e9c58747441` |
| `.ino.bin` (ASUS only) | `3510997674b85e064c50d1e3b61abbcf5659b736f7811b1b40ada5d101241201` |
| `.ino.merged.bin` (ASUS only) | `80dd639ddcd477b5d0277587b1b4b676dcdae32786cc0bc58a8efae18b28780a` |

### Toolchain

```text
esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi
```

### Command surface

```text
--unit LABEL         hardware mode: provision one physical unit
--self-test          offline; opens no serial port
--command-surface    print allowed commands; opens no port
--static-audit       audit every firmware write primitive; opens no port
--verify-firmware    verify firmware SHA256; opens no port
```

Explicitly **not implemented** in firmware: arbitrary `GoalPosition`, arbitrary ID write, baud
write, `CalibrationOfs`, factory reset, broadcast write, generic EEPROM write, host-exposed torque
on. The host supplies neither the source nor the target ID — firmware owns the allocation map.

### Test results — reproduced during the 2026-08-27 audit

```text
build_freeze_20260827_v6/SHA256SUMS   4/4 OK
--self-test        PASS (450/450 checks)
--static-audit     ok: true, hardware_freeze_blocked: false
--verify-firmware  source match: true, binary match: true
```

### Hardware validation scope

**Exercised on real hardware** (17 units):

- centering to physical RAW 2048 (`TorqueLimit` 300, `GoalSpeed` 365, `Acc` 50)
- `PositionOffset` → 0 (EEPROM write ×17)
- EEPROM unlock / relock (`Lock` `0x37` ×62)
- ID recode (×14)
- warm verification
- true cold power cycle
- acceptance gate ±1

### ⚠️ NOT hardware-validated

> **The canonical-profile EEPROM write branch was never physically executed.**

All 22 sessions logged `DELTA_COUNT WRITE=0 SKIP=20` — 440 SKIP, 0 WRITE. Every unit already held
the canonical values. The profile-register write path is implemented and statically audited but
has no hardware evidence.

### Firmware heterogeneity

Only **2** of the 17 units (ELR01, M33) were provisioned by V6; **14** by V5 and **1** by V4. V6
is the final frozen tool and the version that fixed the ELR01 low-boundary blocker. Details in the
[campaign report](../../09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/README.md#firmware-heterogeneity--read-this-before-citing-1717-on-v6).

### Companion tools

| File | Role |
|---|---|
| `matdog_st3215_characterize_v1.ino` / `.py` | characterization that produced `PRIME_MAX_DELTA_TICKS`, `COLD_ABSENCE_DEBOUNCE_MS`, `COLD_RETURN_STABLE_SAMPLES` |
| `matdog_st3215_persistence_audit_v1.py` | cold-persistence audit used on NEW01 and ELR03 |
| `MATDOG_ST3215_PROVISIONER_V1_DESIGN.md` | design record |
| `CHARACTERIZATION_DESIGN.md` | characterization design record |

---

## Related

- [MATDOG_C018_V1 canonical profile](../../01_Docs/02_Architecture/MATDOG_ST3215_C018_V1_PROFILE.md)
- [ST3215 evidence index](../../09_Logs/ST3215_EVIDENCE_INDEX.md)
- [Calibration reset — 2026-08-27](../../09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md)
