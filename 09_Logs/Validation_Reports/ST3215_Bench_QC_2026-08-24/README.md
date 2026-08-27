# Bench QC — ST3215 Servo Quality Audit V6.1

**Campaign dates:** 2026-08-24 (18 units) and 2026-08-25 (4 ELR units + 4 post-normalization re-runs)
**QC protocol:** V6.1 (`QC_PROTOCOL_VERSION = 61`)
**Frozen firmware:** `matdog_servo_commissioning.ino`
SHA256 `74656fb9187fd2024f8251276b49676d8be9c6455f542c49500cfb30d25630cd`
**Result:** 26/26 runs `COMPLETE`, checksum `PASS`, 0 performance events, 0 protective stops

This is the authoritative record of the MATDOG bench QC campaign. Tool sources are frozen under
[`05_Firmware/ST3215_Bench_Tools/Bench_QC_V6_1/`](../../../05_Firmware/ST3215_Bench_Tools/Bench_QC_V6_1/).

---

## ⚠️ Documentation correction — `@NORMALIZE_MATDOG` and EEPROM capability

Earlier MATDOG material described the QC V6.1 binary as *EEPROM-write-incapable*. **That
description is wrong and is corrected here.**

The complete frozen V6.1 firmware **does** contain an EEPROM-writing command:

```text
@NORMALIZE_MATDOG <id>   →   runNormalizeMatdog()
```

`runNormalizeMatdog()` performs a real EEPROM unlock and up to five register writes:

| # | Register | Change |
|---|---|---|
| 1 | `P` | 16 → 32 |
| 2 | `D` | 0 → 32 |
| 3 | `ProtectionCurrent` | 500 → 310 |
| 4 | `OverloadTorque` | 25 → 80 |
| 5 | `MaxTorque` | 500 → 1000 |

### The correct formulation

- The **measurement commands and campaign runner** used for the QC campaign did **not** invoke
  normalization.
- The **complete firmware** did contain a manually-triggered normalization capability.
- This does **not** automatically invalidate the RAW data, provided the logs show the campaign
  runner never used it.
- The frozen source remains **byte-for-byte historical**. It is **not** retroactively corrected.
- Provisioning is a **separate** tool — see [Provisioner V6](../ST3215_Provisioning_2026-08-27/README.md).

### Evidence that the campaign runner never used it

Three independent checks, all reproduced during this audit:

1. **The runner cannot send it.** `matdog_qc_runner.py` has a closed command surface of exactly
   five commands — `@SCAN`, `@READ`, `@SAFE_OFF`, `@DUMP_RAW`, `@QC_FAST`. `@NORMALIZE_MATDOG` is
   not among them and appears nowhere in the runner.
2. **No log contains it.** `grep NORMALIZE_MATDOG` across all 26 campaign logs → **0 matches**.
3. **Every log positively asserts no writes.** All 26 logs record `EEPROM_WRITES : NONE`.

The RAW dataset is therefore considered sound. The capability existed; the campaign did not
exercise it.

### The `_NORM` runs

Four runs are labelled `ELR0x_NORM` (2026-08-25, 20:00–20:09). These are QC **re-measurements**
of the four ELR units taken *after* a separate, manual normalization step performed outside the
campaign runner. Those four logs also record `EEPROM_WRITES : NONE`, because the normalization
happened in a different session — the `_NORM` run itself is pure measurement.

---

## Count reconciliation — 12, 18, 22, 17

Four different numbers appear across MATDOG material. They are **different cohorts from different
campaigns** and must not be flattened.

| Count | What it is | When | Where |
|---:|---|---|---|
| **12** | `postimpact_definitive_v2` dataset — the 12 pre-existing M-series servos (M11–M43) | 2026-08-14 | `09_Logs/Calibration_Sessions/2026-08-14_*_postimpact_definitive_v2_*.json` |
| **18** | Bench QC V6.1 **blind/independent audit** cohort = 12 M-series + 6 NEW | 2026-08-24 | [`blind_audit/`](blind_audit/) |
| **22** | **Total** Bench QC V6.1 units = 18 + 4 ELR | 2026-08-24 / 08-25 | [`manifest.tsv`](manifest.tsv) |
| **17** | Units **allocated and provisioned** into the robot | 2026-08-27 | [provisioning campaign](../ST3215_Provisioning_2026-08-27/README.md) |

Relationships:

```text
12  (postimpact, 2026-08-14)  ──┐
                                ├─→ 18 blind audit cohort (12 M + 6 NEW, 2026-08-24)
 6  NEW01..NEW06              ──┘
                                └─→ 22 total QC'd (18 + 4 ELR, 2026-08-25)
                                      └─→ 17 allocated  (22 − 5 M-units not used)
```

- The **12** postimpact runs are a *separate, earlier* campaign with a different tool
  (`matdog_postimpact_definitive_v2.py`). They are **not** Bench QC V6.1.
- The 4 ELR units were QC'd on 2026-08-25, **after** the blind audit was frozen, so they are not
  part of the 18-unit blind cohort.
- The 5 M-series units QC'd but **not** allocated: **M12, M13, M21, M23, M32**.
- Counting the 4 `_NORM` re-measurements, `manifest.tsv` holds **26** rows for **22** units.

---

## Campaign result

From [`manifest.tsv`](manifest.tsv), all 26 rows:

| Field | Value |
|---|---|
| `protocol` | 61 (V6.1) on every row |
| `execution` | `COMPLETE` × 26 |
| `checksum` | `PASS` × 26 |
| `performance_events` | 0 total |
| `protective_stops` | 0 total |

---

## RAW dataset — held outside Git

The 26 RAW `.bin` captures total **~49 MB** and are deliberately **not** committed. This
repository has no LFS policy for `.bin` and no release-asset workflow, so they are preserved on
the ASUS canonical archive with a committed hash manifest.

**Canonical location (ASUS):**

```text
~/MATDOG/runtime/esp32/qc_campaign/*.bin
```

**Manifest:** [`RAW_BIN_SHA256SUMS.txt`](RAW_BIN_SHA256SUMS.txt) — 26 entries.

### Integrity verified during this audit

| Check | Result |
|---|---|
| Recomputed SHA256 vs `manifest.tsv` recorded hashes | **26/26 match** |
| Recomputed vs blind audit `RAW_SHA256_BEFORE.txt` | **18/18 match** |
| Recomputed vs blind audit `RAW_SHA256_AFTER.txt` | **18/18 match** |

BEFORE and AFTER matching proves the blind audit was **read-only** and did not mutate the RAW.

### Recovery

To restore the dataset, copy the `.bin` files from the ASUS path above and verify with:

```bash
sha256sum -c RAW_BIN_SHA256SUMS.txt
```

The `.log` transcripts **are** committed here in [`campaign_logs/`](campaign_logs/) — they are
small and they carry the `EEPROM_WRITES : NONE` attestation.

---

## Blind / independent audit

An 18-unit anonymized audit run on 2026-08-24. Units were assigned S-codes by
`sha256(campaign_key || file_sha256)` rank, the anonymous report was frozen and hashed, and only
then were labels revealed.

```text
campaign key : a000bc3674f2b689cb6133a7ed7922bf276261ea10c1d214bde56618d24d4c96
frozen report: blind_report_ANONYMOUS.md
        sha256 5613911ee6c5651e08ad24065eafd0e04cb0ac6a04a381dc5b19a01e5e5b70c1
```

Contents in [`blind_audit/`](blind_audit/): frozen anonymous report, revealed label mapping,
per-domain metric CSVs, anomalies, ranking sensitivity, batch-effect check, and the analyzer
sources under [`blind_audit/scripts/`](blind_audit/scripts/).

> `blind_spatial_bins.csv` (8.5 MB) is **not** committed — it is a derived product regenerable
> from the RAW plus the committed scripts. It remains at
> `~/MATDOG/runtime/esp32/qc_campaign_blind_audit_claude/blind_spatial_bins.csv`.

---

## Tooling

| Component | File | SHA256 |
|---|---|---|
| QC firmware (frozen) | `matdog_servo_commissioning.ino` | `74656fb9187fd2024f8251276b49676d8be9c6455f542c49500cfb30d25630cd` |
| Host runner | `matdog_qc_runner.py` | `a143958034be46c9a36ce89fca2fb27fab2371e1af730abe2e3eaf265bc1245e` |
| Per-unit driver | `matdog_qc_unit.sh` | `450b19250acc87e1864a7eb6172ebc308bd583bbbebe9339602447a36d56a635` |

Sources: [`05_Firmware/ST3215_Bench_Tools/Bench_QC_V6_1/`](../../../05_Firmware/ST3215_Bench_Tools/Bench_QC_V6_1/)
RAW schema: [`RAW_SCHEMA.md`](../../../05_Firmware/ST3215_Bench_Tools/Bench_QC_V6_1/RAW_SCHEMA.md)

### Runner command surface — complete

```text
@SCAN <lo> <hi>     bus scan
@READ <id>          register read
@SAFE_OFF <id>      torque off
@DUMP_RAW <id>      RAW capture retrieval
@QC_FAST <id>       QC sequence execution
```

No other command is reachable from the runner.

---

## Related

- [MATDOG_C018_V1 canonical profile](../../../01_Docs/02_Architecture/MATDOG_ST3215_C018_V1_PROFILE.md)
- [Provisioning campaign 17/17](../ST3215_Provisioning_2026-08-27/README.md)
- [ST3215 evidence index](../../ST3215_EVIDENCE_INDEX.md)
