# MATDOG content merged into `norma-core/main`

Snapshot and classification of every MATDOG-specific file that was already merged into
`MattRobotics/norma-core` `main` before the migration. Captured so that retiring MATDOG
development from `norma-core` loses nothing, even though `main` itself is retained.

**Snapshot of:** `main` @ `4a8ed6337261553b79c928975808d294c9ca723b` · **Archived:** 2026-08-27

---

## Why this exists

The two retired branches were not the whole story. Several MATDOG pull requests were **merged into
`main`** during the Station-mediated phase:

| PR | Title | Result |
|---|---|---|
| #2 / #1 | native MATDOG M12 MIN calibrator foundation | MERGED |
| #3 | generalize MATDOG calibrator to 24 contact profiles | MERGED |
| #11 | **validate and freeze MATDOG LF calibrator V25** | MERGED |
| #13 | consolidate LF V25 and final repository policy | MERGED |
| #28 | archive RF failures and hand off | MERGED |
| #34 | freeze canonical geometry-first calibration architecture | MERGED |
| #35 | align canonical entry after Geometry V5 | MERGED |

So the MATDOG calibrator foundation lives on `main`, not only on the branches. This archive
captures it independently of `norma-core`'s future.

## Classification

| File | Classification |
|---|---|
| `software/drivers/st3215/src/auto_calibrate/matdog.rs` (182 K) | **ARCHIVE_REQUIRED** — the native calibrator core |
| `software/drivers/st3215/src/auto_calibrate/matdog_test.rs` (103 K) | **ARCHIVE_REQUIRED** — its test suite |
| `software/drivers/st3215/src/bin/matdog_lf_freeze.rs` (20 K) | **ARCHIVE_REQUIRED** — LF EEPROM freeze binary |
| `tools/matdog/matdog_headless_auto_calibrate.py` (86 K) | **ARCHIVE_REQUIRED** — headless calibration runner |
| `tools/matdog/matdog_lf_profile.py` (13 K) | **ARCHIVE_REQUIRED** — LF profile definition |
| `tools/matdog/matdog_native_observer_contract.py` | **ARCHIVE_REQUIRED** — observer safety contract |
| `tools/matdog/matdog_v42_pinned_launcher.py` | **ARCHIVE_REQUIRED** — pinned Station launcher |
| `tools/matdog/test_matdog_*.py` (4 files) | **ARCHIVE_REQUIRED** — test suites |
| `.github/workflows/matdog-native-calibrator-check.yml` | **ARCHIVE_REQUIRED** — CI contract |
| `.github/workflows/matdog-native-observer-check.yml` | **ARCHIVE_REQUIRED** — CI contract |
| `tools/matdog/MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md` | **ALREADY_PRESENT_IN_ROBOT_DOG** (also at `06_Software/Matdog_Core/calibration/`) |
| `tools/matdog/MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md` | **ALREADY_PRESENT_IN_ROBOT_DOG** |
| `tools/matdog/MATDOG_RF_CALIBRATOR_CLAUDE_HANDOFF_2026-08-05.md` | **OBSOLETE_REFERENCE** — RF work archived/closed |
| `tools/matdog/MATDOG_RF_CALIBRATOR_HANDOFF_STATE_2026-08-05.json` | **OBSOLETE_REFERENCE** |
| `tools/matdog/CLAUDE_PROMPT_RF_CALIBRATOR_2026-08-05.md` | **OBSOLETE_REFERENCE** |
| `tools/matdog/README.md` | **ARCHIVE_REQUIRED** — explains the tools directory |
| `tools/matdog/UPSTREAM_SYNC_2026-08-04.md` | **ARCHIVE_REQUIRED** — fork/upstream sync policy |
| `auto_calibrate/{calibrator,mod,elrobot,so101}.rs` | **UPSTREAM/GENERIC_NORMACORE** — *not* archived; not MATDOG-specific |

All files classified `ARCHIVE_REQUIRED` are in [`source/`](source/), byte-for-byte.

Nothing was classified `UNKNOWN`.

## What was deliberately not copied

The rest of NormaCore — Station, drivers, viewer, protobuf, examples, upstream tooling — is
**upstream/generic** and stays where it belongs. This archive holds the MATDOG-specific
intellectual and engineering content only, as instructed.

`auto_calibrate/calibrator.rs`, `mod.rs`, `elrobot.rs` and `so101.rs` are shared NormaCore
infrastructure. `matdog.rs` plugs into them but they are not MATDOG property, so they are
referenced here rather than copied.

## ⚠️ Classification of this content

Everything here is **STATION-MEDIATED** and **HISTORICAL**. It assumes NormaCore Station owns the
ST3215 serial bus — the architecture MATDOG has since replaced with a dedicated ESP32-S3
coprocessor. None of it is current MATDOG runtime, and none of it may command hardware.

`norma-core` `main` is retained permanently and untouched, so this snapshot is a convenience and a
safety net, not the only copy.

## Related

- [NormaCore MATDOG archive index](../README.md)
- [LF V25 historical hardware oracle](../LF_V25_Hardware_Oracle/README.md)
- [Canonical architecture](../../../../01_Docs/02_Architecture/ARCHITECTURE.md)
