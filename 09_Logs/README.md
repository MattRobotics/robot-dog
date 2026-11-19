# MATDOG logs and evidence

Stato corrente 19/11/2026: **Stable [`v1.0.2`](https://github.com/MattRobotics/robot-dog/releases/tag/v1.0.2)**, installata e qualificata sulla fixture. Firmware `1.0.2-dev.7` `1314bc411c58`, commit `1314bc4`, APP `dd16bd62…`.
- **Evidenze correnti:**
  - [qualifica HW del 19/11](Validation_Reports/Motion_1314bc4_HW_2026-11-19/README.md): catena 8/8, D1/D2, blocco D, S9;
  - [manifest della release](Validation_Reports/Motion_1314bc4_HW_2026-11-19/RELEASE_MANIFEST.md);
  - [evidenze, archivi locali e SHA256](Validation_Reports/Motion_1314bc4_HW_2026-11-19/EVIDENCE_MANIFEST.md).
- **Decisioni:**
  - [ADR-008](Architecture_Decisions/ADR-008_Motion_Timing_P0_Closure_and_Freeze.md): Motion Timing P0 CLOSED e congelato;
  - [DALY B2](Validation_Reports/BMS_DALY_B2_2026-11-19/README.md): RCA congelata, causa UNKNOWN.
- **Prima della Stable** (17–19/11):
  - [build integrata](Validation_Reports/Motion_Integrated_1314bc4_2026-11-19/REPORT.md);
  - [FP32 e P1](Validation_Reports/Motion_FP32_2026-11-18/REPORT.md);
  - [certificato `kSteer`](Validation_Reports/Motion_SteerSlow_Recert_2026-11-18/README.md);
  - [rollback `30c894a`](Validation_Reports/Motion_30c894a_HW_2026-11-18/README.md);
  - [A/B DALY](Validation_Reports/BMS_DALY_AB_2026-11-18/REPORT_AB.md);
  - [campagna P0 del 17/11](Validation_Reports/MATDOG_1_0_2_P0_2026-11-17/REPORT.md);
  - [handoff del 18/11](Development_Log/2026-11-18_MATDOG_SESSION_CLOSE_HANDOFF.md).

Storico 14–16/11/2026: branch operativo unificato e build diagnostiche `1.0.2-dev.8` ([handoff](Validation_Reports/Unified_Operational_2026-11-14/HANDOFF.md)). Ground Contact sensorless Fase 1 (13/11, [rapporto](Validation_Reports/Ground_Contact_Phase1_2026-11-13/REPORT.md)): diagnostica passiva inclusa nel firmware attuale, VERIFIED OFFLINE, mai provata sul robot. Storico 11/11: `1.0.2-dev.7` `b5610620d0ce` ([rapporto](Validation_Reports/MATDOG_DEV7_HW_2026-11-11/HW_REPORT.md)).

Storico 10/11/2026: era installata la build di sviluppo `1.0.2-dev.6` ACC 220 (`450d2c7`, APP `82122e06…`).
- **Hardware:** fixture Q1–Q8 qualificata; terra fuori protocollo. [Report HW](Validation_Reports/Dev6_ACC220_HW_2026-11-10/REPORT.md) · [evidenze e SHA256](Validation_Reports/Dev6_ACC220_HW_2026-11-10/EVIDENCE_MANIFEST.md).
- **Gemello MuJoCo V1, offline:** [rapporto M5](../06_Software/Matdog_MuJoCo/REPORT_M5.md).

Storico 07/11/2026: **prima Stable GitHub `v1.0.1`** = APP `1.0.1-dev.17` / `17db0efe2324` installata e collaudata, promossa senza ricompilazione — [report HW](Validation_Reports/Dev17_HW_2026-11-07/REPORT.md) · [manifest release](Validation_Reports/Dev17_HW_2026-11-07/RELEASE_MANIFEST.md) · [evidenze](Validation_Reports/Dev17_HW_2026-11-07/EVIDENCE_MANIFEST.md); rollback dev.16 ([report](Validation_Reports/Dev16_HW_2026-11-07/REPORT.md)). Indice completo: [Validation_Reports](Validation_Reports/README.md).

Storico 06/11/2026: [dev.15 — PARTIAL HW VALIDATION](Validation_Reports/Dev15_HW_2026-11-06/REPORT.md) · [manifest SHA256 grezzi locali](Validation_Reports/Dev15_HW_2026-11-06/EVIDENCE_MANIFEST.md) · [analisi video](Validation_Reports/Dev15_HW_2026-11-06/VIDEO_AUDIT.md) · [RCA timing 07/11](Validation_Reports/Dev15_HW_2026-11-06/TIMING_RCA_2026-11-07.md). Dev.7 pre-release storica → dev.12 04/11 → [dev.14](Validation_Reports/Dev14_HW_2026-11-05/REPORT.md) 05/11 → **dev.15 installata 06/11**, sorgente all'epoca solo locale, main non sincronizzato al device; **nessuna stable a quella data** (situazione poi superata da v1.0.1).

Historical scoped closeout (11/10/2026): [non-motion closeout](Validation_Reports/MATDOG_HW_NON_MOTION_CLOSEOUT_2026-10-11.md) · [Motion handoff](Development_Log/2026-10-12_MOTION_ENABLEMENT_HANDOFF.md).

Update 2026-10-31 (separate from the dated lines above): REV01-FIX geometry, physical properties and the Astra handoff were integrated in `main` by PR #57 (merge `887103e`, verified offline only) — [integration report](Validation_Reports/REV01_FIX_Integration_2026-10-30/REPORT.md) · [physical properties](Validation_Reports/REV01_FIX_Physical_Properties_2026-10-31/REPORT.md) · [**Astra handoff**](Validation_Reports/REV01_FIX_Alignment_2026-10-31/ASTRA_HANDOFF.md).

> **CHRONOLOGY AND EVIDENCE — NOT CURRENT-STATE AUTHORITY**
>
> Start with the [root README](../README.md) for today's hardware, firmware baseline, validation
> status and immediate milestone. Active architecture contracts live in
> [ARCHITECTURE.md](../01_Docs/02_Architecture/ARCHITECTURE.md).

This directory preserves dated engineering decisions, development chronology, calibration records
and validation evidence. A result remains evidence for its recorded date and scope; it does not
automatically describe the robot after a later rebuild or architecture change.

## Navigation

| Area | Purpose |
|---|---|
| [History index](HISTORY_INDEX.md) | Chronology June–November 2026; dated checkpoints, evidence and the published Stables v1.0.1 and v1.0.2 |
| [Open items](OPEN_ITEMS.md) | Current overview as of 19/11/2026 (Stable `v1.0.2`): closed, suspended and still to be qualified, next milestone; earlier dated states kept below it |
| [Commit provenance map](COMMIT_PROVENANCE_MAP.md) | PR #37–#41 merge SHAs; original SHA to recreated SHA for PR-2 and PR-3 (PR #38 merged with original SHAs; PR #41 is documentation only) |
| [External archives](EXTERNAL_ARCHIVES.md) | Deleted branches (last tips), remaining tags, the non-public archive status and recovery steps; items without evidence stay `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| [ST3215 evidence index](ST3215_EVIDENCE_INDEX.md) | Canonical navigation for bench QC, source survey, provisioning and frozen-tool evidence |
| [`Validation_Reports/`](Validation_Reports/README.md) | Dated hardware, software and model validation evidence; each claim is limited to its stated scope |
| [`Calibration/`](Calibration/) | Calibration records and the 2026-08-27 reset notice; pre-reset values are historical |
| [`Calibration_Sessions/`](Calibration_Sessions/) | Raw and derived records from dated calibration sessions |
| [`Development_Log/`](Development_Log/README.md) | Chronological handoffs and milestone records; their “next steps” are historical |
| [`Architecture_Decisions/`](Architecture_Decisions/README.md) | Dated ADRs; consult the canonical architecture before treating an older decision as active |
| [Historical index](Historical/README.md) | Explicitly superseded material and preserved engineering archives |

## Reading rules

- Validation means only that the documented test passed within its recorded scope.
- A dated handoff records what was current then; it does not own today's milestone.
- ST3215 provisioning of 17 allocated units is not a statement that 17 servos are physically
  installed today.
- Historical calibration and archived tools provide evidence or engineering reference, never
  hardware-motion authorization.
- Keep evidence at its existing path so hashes, manifests and inbound links remain intact.
