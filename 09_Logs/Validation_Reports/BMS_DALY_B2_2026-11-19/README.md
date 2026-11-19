# DALY B2 19/11/2026: provenance and frozen RCA status

**Files in this folder:**
- `REPORT_B2.md`, `B2_30c894a_cold.read.txt` and `B2_30c894a_cold.detail.txt` are unchanged copies of the read-only acquisition run by Fable: a cold start of `30c894a` recorded from power-on, with no flash and no Motion command.
- Local original: `~/MATDOG/verification-artifacts/MATDOG_BMS_DALY_QON_20261119/`. Seal `SHA256SUMS.txt` `1c8303fc…`, 8 files, verified 19/11; copied here as `SHA256SUMS.local.txt`.
- Not published: `tools_used.sha256`, because it holds absolute host paths. Its hash `f084df6a…` is in the seal, and it names the same recorder and reader as on 18/11.
- Archive copy: `~/MATDOG/backups/verification-artifacts/session-20261119/MATDOG_BMS_DALY_QON_20261119.tar.gz` (`ecd5499b…`).

**Result:** CLEAN, 358/358 valid `@BMS STATUS` samples from 4 to 718 s, `fail_streak=0`, after 453.7 min off.

## RCA status, frozen 19/11/2026

| Item | State |
|---|---|
| A/B dev.5 against `30c894a` (18/11) | **INCONCLUSIVE** ([report](../BMS_DALY_AB_2026-11-18/REPORT_AB.md)) |
| B2, cold start of `30c894a` (19/11) | **CLEAN**, 358/358. It weakens the regression hypothesis: one-sided Fisher p = 0.5 |
| After the `1314bc4` flash, Motion HW sessions of 19/11 | 692 valid DALY readings, 0 errors, in the qualification session (≈ 44 min after the flash reset). The completion runs (07:24–07:33Z) also had 0 errors: no silence in ≈ 85 min |
| Cause of the earlier silences | **UNKNOWN** |
| BMS firmware change, causal fix | None. No solution is proven |

**Decision:** the active RCA is frozen.
- No new bisect or test campaign in this phase.
- Observation continues through the existing operational telemetry.
- The RCA reopens on a new documented anomaly.

The operational precaution stays as before: QON + 6 min and 150 valid polls before Motion after a power-on.
