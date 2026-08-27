# ST3215 evidence index

Single entry point for every artifact produced by the MATDOG ST3215 bench phase — QC,
characterization, survey, provisioning — and a map of what lives in Git versus what is preserved
on the ASUS canonical archive.

**Phase closed:** 2026-08-27 · **Units provisioned:** 17/17 PASS · **Profile:** `MATDOG_C018_V1`

---

## Documents

| Document | What it covers |
|---|---|
| [MATDOG_C018_V1 canonical profile](../01_Docs/02_Architecture/MATDOG_ST3215_C018_V1_PROFILE.md) | the persistent EEPROM profile contract |
| [`MATDOG_ST3215_C018_V1.yaml`](../06_Software/Matdog_Core/config/MATDOG_ST3215_C018_V1.yaml) | machine-readable profile |
| [`MATDOG_SERVO_ALLOCATION.yaml`](../06_Software/Matdog_Core/config/MATDOG_SERVO_ALLOCATION.yaml) | unit → joint → bus ID for all 17 |
| [Bench QC V6.1 campaign](Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md) | QC results, count reconciliation, `@NORMALIZE_MATDOG` correction |
| [Provisioning campaign 17/17](Validation_Reports/ST3215_Provisioning_2026-08-27/README.md) | provisioning results, V5→V6 blocker, EEPROM-write caveat |
| [Frozen bench tools](../05_Firmware/ST3215_Bench_Tools/README.md) | freeze policy, hashes, command surfaces, validation scope |
| [RAW capture schema](../05_Firmware/ST3215_Bench_Tools/Bench_QC_V6_1/RAW_SCHEMA.md) | QC `.bin` binary format |
| [⚠️ Calibration reset](Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md) | why all calibration must be redone |
| [Handoff 2026-08-27](Development_Log/2026-08-27_ST3215_CANONICAL_ARCHIVE_HANDOFF.md) | current state and next steps |

---

## Frozen tools — all in Git

| Tool | Source SHA256 | Binary SHA256 (ASUS only) |
|---|---|---|
| Bench QC V6.1 firmware | `74656fb9187fd2024f8251276b49676d8be9c6455f542c49500cfb30d25630cd` | — |
| QC host runner | `a143958034be46c9a36ce89fca2fb27fab2371e1af730abe2e3eaf265bc1245e` | — |
| QC unit driver | `450b19250acc87e1864a7eb6172ebc308bd583bbbebe9339602447a36d56a635` | — |
| Source Survey V1 firmware | `f37fa85a11e333634c18b4abbd991e24797b1891ec5d2e5432b1e4787e5a0c86` | `89ce2a5178b663d7a19b01275ad65662856ce7b7bc114198b5a3e535464f6a12` |
| Source Survey V1 runner | `7cc77149cde24fe6bf0e8b0f6fef2c6b1bf42c1c45bad09cfb2075859cd05489` | — |
| Provisioner V6 firmware | `680b2715e6896c296c0fc63e7a72bc520a471977891478b2cca9b0f6472e2327` | `3510997674b85e064c50d1e3b61abbcf5659b736f7811b1b40ada5d101241201` |
| Provisioner V6 runner | `222c4c6c9395226afe3b845021af362a85d825a1778f4530b0213e9c58747441` | — |

Provisioner V6 merged image (ASUS only): `80dd639ddcd477b5d0277587b1b4b676dcdae32786cc0bc58a8efae18b28780a`

Manifest: [`05_Firmware/ST3215_Bench_Tools/SHA256SUMS`](../05_Firmware/ST3215_Bench_Tools/SHA256SUMS)

---

## Evidence in Git

| Path | Contents | Size |
|---|---|---:|
| `09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/campaign_logs/` | 26 QC transcripts | ~940 KB |
| `…/ST3215_Bench_QC_2026-08-24/manifest.tsv` | 26-row campaign manifest | — |
| `…/ST3215_Bench_QC_2026-08-24/RAW_BIN_SHA256SUMS.txt` | hashes of the out-of-Git RAW | — |
| `…/ST3215_Bench_QC_2026-08-24/blind_audit/` | 18-unit blind audit + analyzer scripts | ~1.1 MB |
| `09_Logs/Validation_Reports/ST3215_Provisioning_2026-08-27/sessions/` | all 22 sessions, 6 RAW state snapshots each | ~1.4 MB |
| `…/ST3215_Provisioning_2026-08-27/persistence_audit/` | NEW01 + ELR03 persistence audits, ELR03 probe | ~148 KB |
| `…/ST3215_Provisioning_2026-08-27/characterization_sessions/` | NEW01 characterization, C018 reset/reboot audit | ~256 KB |
| `…/ST3215_Provisioning_2026-08-27/source_survey/` | read-only survey evidence | ~32 KB |
| `09_Logs/Calibration/Digital_Zero/` | 2026-08-14 snapshots — **historical, superseded** | ~21 KB |

Each evidence directory carries its own `SHA256SUMS.txt`.

---

## Artifacts held outside Git

Preserved on the ASUS canonical archive. Not committed because this repository has no LFS rule for
`.bin` and no release-asset workflow.

| Artifact | ASUS path | Size | Hash manifest in Git |
|---|---|---:|---|
| QC RAW captures (26) | `~/MATDOG/runtime/esp32/qc_campaign/*.bin` | ~49 MB | [`RAW_BIN_SHA256SUMS.txt`](Validation_Reports/ST3215_Bench_QC_2026-08-24/RAW_BIN_SHA256SUMS.txt) |
| Provisioner freezes V1–V6 | `~/MATDOG/runtime/esp32/matdog_st3215_provisioner_v1/build_freeze_20260827_v{1..6}/` | ~100 MB | per-directory `SHA256SUMS` (V6 hashes above) |
| Survey approved freeze | `~/MATDOG/runtime/esp32/matdog_st3215_source_survey_v1/approved_stack_freeze_20260826/` | ~16 MB | own `SHA256SUMS` + `FREEZE_MANIFEST.txt` |
| Blind audit spatial bins | `~/MATDOG/runtime/esp32/qc_campaign_blind_audit_claude/blind_spatial_bins.csv` | 8.5 MB | derived; regenerable from RAW + committed scripts |

### Recovery

```bash
# QC RAW
cd ~/MATDOG/runtime/esp32/qc_campaign
sha256sum -c /path/to/repo/09_Logs/Validation_Reports/ST3215_Bench_QC_2026-08-24/RAW_BIN_SHA256SUMS.txt
```

```bash
# Provisioner V6 freeze
cd ~/MATDOG/runtime/esp32/matdog_st3215_provisioner_v1/build_freeze_20260827_v6
sha256sum -c SHA256SUMS
```

Binaries are reproducible from the committed sources using the recorded FQBN:

```text
esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi
```

---

## Integrity checks performed 2026-08-27

| Check | Result |
|---|---|
| Provisioner V6 freeze `SHA256SUMS` | 4/4 OK |
| Survey approved freeze `SHA256SUMS` | 3/3 OK |
| QC RAW vs `manifest.tsv` recorded hashes | 26/26 match |
| QC RAW vs blind audit `RAW_SHA256_BEFORE` | 18/18 match |
| QC RAW vs blind audit `RAW_SHA256_AFTER` | 18/18 match |
| Digital_Zero `.sha256` companions | 3/3 OK |
| Frozen QC sketch SHA256 | matches `74656fb9…` |
| Provisioner `--self-test` | PASS 450/450 |
| Provisioner `--static-audit` | `ok: true` |
| Provisioner `--verify-firmware` | source and binary match |

BEFORE/AFTER both matching proves the blind audit was read-only.

---

## Cohort reconciliation — 12 / 18 / 22 / 17

| Count | Cohort | Date |
|---:|---|---|
| 12 | `postimpact_definitive_v2` — M-series only, **separate earlier campaign** | 2026-08-14 |
| 18 | Bench QC blind audit cohort (12 M + 6 NEW) | 2026-08-24 |
| 22 | Total Bench QC units (18 + 4 ELR); 26 runs incl. 4 `_NORM` | 2026-08-24/25 |
| 17 | Allocated and provisioned into the robot | 2026-08-27 |

Not allocated: **M12, M13, M21, M23, M32**. Full explanation in the
[QC campaign report](Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md#count-reconciliation--12-18-22-17).

---

## Open limits

| Limit | Status |
|---|---|
| Canonical-profile EEPROM write branch | **NOT HARDWARE EXERCISED** — `WRITE=0 SKIP=20` on all 22 sessions |
| V6 firmware coverage | only 2 of 17 units provisioned by V6 (14 × V5, 1 × V4) |
| Baud register write | not implemented in V1 — verification only |
| NormaCore `ModelNumber` at `0x00` | **defect still open upstream**; C018 model word is at `0x03` |
| Gait / head / jaw runtime limits | TBD after recalibration |
| All robot joint calibration | **RESET** — see [calibration reset](Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md) |
