# MATDOG history index — June to October 2026

One navigable chronology. It **indexes** existing documents; it adds no new evidence and does not
change any recorded PASS/FAIL result. Where a document or report does not exist on `main`, the row
says so instead of describing it.

> **CHRONOLOGY — NOT CURRENT-STATE AUTHORITY.** Today's state: [root README](../README.md).
> Open blockers: [`OPEN_ITEMS.md`](OPEN_ITEMS.md). Commits and recreated SHAs:
> [`COMMIT_PROVENANCE_MAP.md`](COMMIT_PROVENANCE_MAP.md). Material outside `main`:
> [`EXTERNAL_ARCHIVES.md`](EXTERNAL_ARCHIVES.md).
> Standing limits: `MOTION_AUTHORIZED=0` · `RESTORE=NOT_IMPLEMENTED` · **DO NOT FLASH `main`**.

## How to read a row

| Column | Meaning |
|---|---|
| **Status** | One of the project vocabulary terms below, as the cited document records it for its own scope |
| **Documents** | Relative links; all resolve on `main` unless marked |
| **Evidence available** | What a reader can actually open on `main`. "Document only" = a written record without raw artifacts here. "Not on `main`" = referenced but absent |

Status vocabulary: **IMPLEMENTED** (code exists) · **OFFLINE TESTED** · **HARDWARE VALIDATED**
(for the stated scope only) · **ACCEPTANCE BLOCKED** · **DEFERRED** · **SUPERSEDED** · **HISTORICAL**.
A row marked `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` depends on material outside `main` that has not
been verified.

Documentation layers: **CURRENT** = root README, [`ARCHITECTURE.md`](../01_Docs/02_Architecture/ARCHITECTURE.md),
[`ROADMAP.md`](../01_Docs/02_Architecture/ROADMAP.md), Controller
[`DEVELOPMENT_GATES.md`](../05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md),
[`VALIDATION.md`](../05_Firmware/MATDOG_Controller/VALIDATION.md),
[`CHANGELOG.md`](../05_Firmware/MATDOG_Controller/CHANGELOG.md). Everything below the dated rows is
**HISTORICAL** unless its status says otherwise.

Per-area indexes: [ADRs](Architecture_Decisions/README.md) · [Development Log](Development_Log/README.md) ·
[Validation Reports](Validation_Reports/README.md) · [ST3215 evidence](ST3215_EVIDENCE_INDEX.md) ·
[Historical archive index](Historical/README.md).

## 1. Architecture stages

| Stage | Dates | Status | Documents | Evidence available |
|---|---|---|---|---|
| Station-mediated control, 12 servos | 2026-06 | **SUPERSEDED** (2026-08-27) | [ADR-001](Architecture_Decisions/ADR-001_Quadruped_Control_Architecture.md), [Project report 2026-06-25](../01_Docs/02_Architecture/Project_Reports/2026-06-25_MATDOG_Project_State_and_Next_Steps.md) | Document only |
| `robot-dog` as MATDOG reference repository | 2026-06-25 | Accepted; the Station bus-control part is **SUPERSEDED** | [ADR-002](Architecture_Decisions/ADR-002_MATDOG_Repository_and_Station_Integration.md) (annotation 2026-10-08) | Document only |
| URDF REV00 canonical kinematic baseline | 2026-06-30 | Accepted; OFFLINE | [ADR-003](Architecture_Decisions/ADR-003_URDF_REV00_Kinematic_Baseline.md), [dev log](Development_Log/2026-06-30_URDF_REV00_Kinematic_Baseline.md), [validation](Validation_Reports/2026-06-30_URDF_REV00_Kinematic_Validation.md) | URDF and meshes in `03_CAD/` |
| Three events: ESP32-S3 replaces Station, 12 servos → 17 allocated slots after the rebuild (13 physically installed today: 12 leg + neck ID 51), hardware decisions frozen | 2026-08-27 | Current architecture | [Historical index](Historical/README.md), [calibration reset](Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md), [ARCHITECTURE](../01_Docs/02_Architecture/ARCHITECTURE.md) | Documents |
| Third-party reverse-engineering material stays out of `main` | 2026-10-08 | Accepted | [ADR-004](Architecture_Decisions/ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md) | Manifest by path/SHA-256 only |

## 2. Mechanics, electronics and power

| Phase | Dates | Status | Documents | Evidence available |
|---|---|---|---|---|
| Servo mapping and BOM | not dated here | Reference | [`04_Electronics/`](../04_Electronics/README.md), [`02_BOM/`](../02_BOM/) | Files in those directories |
| 3S power-load analysis | not dated here | Analysis | [`ST3215_Quadruped_3S_Power_Load_Analysis.md`](../01_Docs/01_Analysis/ST3215_Quadruped_3S_Power_Load_Analysis.md) | Document |
| Power, KEY (DALY BMS), charging handoff | 2026-09-19 → 09-20 | HISTORICAL handoff | [2026-09-20 handoff](Development_Log/2026-09-20_MATDOG_POWER_KEY_CHARGING_NEXTGEN_HANDOFF.md), [power states](../04_Electronics/MATDOG_POWER_STATES_AND_CHARGING.md) | Document only (no 2026-09-19 log on `main`) |
| Charging, B-/P- bypass fix, post-rewire power gate A–E, external USB | 2026-09-22 → 09-24 | Post-rewire gate **PASS** (ROADMAP) | [2026-09-24 closeout](Development_Log/2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md), [`VALIDATION.md`](../05_Firmware/MATDOG_Controller/VALIDATION.md) | Document only |

## 3. Calibration, servo and kinematics (pre-rebuild and Geometry)

| Phase | Dates | Status | Documents | Evidence available |
|---|---|---|---|---|
| Visual zero, static actuation, live FK | 2026-07-02 → 07-07 | **HISTORICAL** (Station, 12 servos; pre-reset) | [dev log](Development_Log/2026-07-06_Visual_Zero_Calibration_and_Static_Control_Checkpoint.md), [validation](Validation_Reports/2026-07-06_Visual_Zero_and_Static_Actuation_Validation.md) | Raw sessions in [`Calibration_Sessions/`](Calibration_Sessions/) |
| C4-A…C4-F offline stand candidate and policies; digital zero; C5-R | 2026-07-08 → 07-10 | OFFLINE / HISTORICAL | [C5-R handoff](Development_Log/2026-07-10_C5R_POST_DIGITAL_ZERO_HANDOFF.md), [C4 reports](Validation_Reports/README.md) | JSON artifacts, [`Calibration/`](Calibration/) |
| Native NormaCore calibrator handoffs (M12 pilot, MIN/MAX) | 2026-07-21 → 07-28 | **HISTORICAL** | [07-21](Development_Log/2026-07-21_NATIVE_NORMACORE_CALIBRATOR_HANDOFF.md), [07-25](Development_Log/2026-07-25_NATIVE_NORMACORE_CALIBRATOR_HANDOFF.md), [07-28](Development_Log/2026-07-28_MATDOG_M12_MIN_MAX_HANDOFF.md) | Records in [`Calibration/`](Calibration/); archive in [`Historical/`](Historical/README.md) |
| LF V25 closeout and repository cleanup | 2026-08-04 | HISTORICAL | [log](Development_Log/2026-08-04_LF_V25_AND_REPOSITORY_CLEANUP.md) | Document |
| Geometry Compiler v1–v5, Phase 2A0 closeout | 2026-08-07 → 08-11 | OFFLINE; per-run status in the artifact index | [log](Development_Log/2026-08-11_PHASE2A0_GEOMETRY_V5_CLOSEOUT.md), [artifact index](Validation_Reports/Geometry_Compiler/README.md) | 85 artifacts on `main`. Pre-squash history of PR #19: tag `archive/2026-08-11/pr19-geometry-v5-pre-squash`, `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| ST3215 bench QC (26/26 runs) and provisioning (17/17 units) | 2026-08-24 → 08-27 | Historical bench evidence, **HARDWARE VALIDATED** for the bench scope only; 17/17 provisioning does **not** mean 17 servos are installed (13 are installed today: 12 leg + neck ID 51; 17 are the allocated slots) | [QC](Validation_Reports/ST3215_Bench_QC_2026-08-24/README.md), [provisioning](Validation_Reports/ST3215_Provisioning_2026-08-27/README.md), [handoff](Development_Log/2026-08-27_ST3215_CANONICAL_ARCHIVE_HANDOFF.md), [index](ST3215_EVIDENCE_INDEX.md) | Artifacts and checksums on `main` |
| Calibration reset after reassembly | 2026-08-27 | Current rule: all calibration redone from zero | [reset notice](Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md) | Document |
| Calibrator V1 H0 | 2026-08-28/29 | HISTORICAL | PR #22 (closed); tag `archive/2026-08-29/full-leg-calibrator-v1-h0` | `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |

## 4. Controller software and first hardware

| Phase | Dates | Status | Documents | Evidence available |
|---|---|---|---|---|
| Controller V0.1 (USB_ONLY), Sessions 1, 2, 2.1, 2.2 | 2026-09-15 | **HARDWARE VALIDATED** (USB_ONLY scope), tag `matdog-controller-v0.1.0` | [`VALIDATION.md`](../05_Firmware/MATDOG_Controller/VALIDATION.md) | Tagged commit on `main`. No dated log |
| ROBOT_POWERED preparation; G3 / G3.1 no-motion | 2026-09-17 → 09-18 | **G3 PASS (formal)**, G3.1 PASS | [G3 plan](../05_Firmware/MATDOG_Controller/G3_ROBOT_POWERED_VALIDATION_PLAN.md), [ROADMAP](../01_Docs/02_Architecture/ROADMAP.md) | Recorded in ROADMAP and `VALIDATION.md`; no dated log on `main` |
| NextGen integration I1–I9 (LED, source signature, actuator runtime, HostLink, Wi-Fi/OTA and Web decisions, freeze) | 2026-09-25 | IMPLEMENTED / OFFLINE TESTED | [Development Log index, 2026-09-25/26 group](Development_Log/README.md) | Logs |
| F0 final flash readiness gates, candidate V3 first flash | 2026-09-25 → 09-26 | Candidate hardware-validated for its scope, tag `matdog-controller-nextgen-hw-validated-v1` | [F0](Development_Log/2026-09-25_F0_FINAL_FLASH_READINESS.md), [V3 flash](Development_Log/2026-09-26_HARDWARE_VALIDATION_CANDIDATE_V3.md) | Logs; tagged commit |
| LED Status Manager V2 | 2026-09-26 | **HARDWARE VALIDATED** (focused), tag `matdog-led-v2-focused-hw-validated-v1` | [final](Development_Log/2026-09-26_LED_STATUS_MANAGER_V2_FINAL.md), [HW validation](Development_Log/2026-09-26_LED_STATUS_MANAGER_V2_HW_VALIDATION.md) | Logs; tagged commit |
| Calibration readiness CR0–CR3; CR2-C current-installation q0 | 2026-09-27 → 09-28 | CR2-C read-only capture **PASS**; CR3 offline | [CR0/CR1](Development_Log/2026-09-27_CALIBRATION_READINESS_CR0_CR1.md), [CR2-C session](Development_Log/2026-09-27_CR2C_Q0_HARDWARE_SESSION.md), [CR2-C evidence](Validation_Reports/Calibration_Q0_CR2C_2026-09-27/README.md), [CR3 logs](Development_Log/README.md) | Evidence package with checksums |

## 5. Calibration on hardware, persistence, recovery and thermal

| Phase | Dates | Status | Documents | Evidence available |
|---|---|---|---|---|
| Full Calibration generalized to four legs; 24-contact sequence geometry | 2026-09-29 → 09-30 | IMPLEMENTED / OFFLINE | [four-leg](Development_Log/2026-09-29_CALIBRATION_FULL_4LEG.md), [24-contact](Development_Log/2026-09-30_TRUE_24_CONTACT_FULL_CALIBRATION.md), [sequence geometry](Validation_Reports/Full_Calibration_Sequence_Geometry_2026-09-30/README.md) | Geometry artifacts |
| TRUE Full Calibration 24/24 | 2026-10-01 | **HARDWARE VALIDATED** for that session (`all_contact_calibrated=1`); RAM-only at the time; no envelope approved | [report](Validation_Reports/Full_Calibration_24_Contact_Hardware_2026-10-01/README.md), [status sync](Development_Log/2026-10-01_PROJECT_STATUS_DOCUMENTATION_SYNC.md) | Report, checksums, tables. PR #35, #36 |
| Flash layout M0 (offline), M0.1–M0.2 | 2026-10-02 | OFFLINE | [M0](Development_Log/2026-10-02_M0_FLASH_LAYOUT_OFFLINE.md), [M0.1](Development_Log/2026-10-02_M0_1_MIGRATION_WRITE_HARDENING.md), [M0.2](Development_Log/2026-10-02_M0_2_HARDWARE_ALIGNMENT.md), [runbook](../05_Firmware/MATDOG_Controller/FLASH_LAYOUT_M0_RUNBOOK.md) | Logs |
| M0.4 P1 closed; encrypted flash backup | 2026-10-03 | P1 closed; backup verified per the log | [P1](Development_Log/2026-10-03_M0_4_P1_CLOSED.md), [checkpoint](Development_Log/2026-10-03_M0_4_P1_CHECKPOINT.json), [`09_Backups/ESP32/`](../09_Backups/ESP32/README.md) | Manifest and receipt on `main`; encrypted archive is a GitHub Release asset |
| M0.4 P2 (layout migration) | 2026-10-03 | Prepared (log states "not executed" at its date). Execution report **not on `main`** | [P2 prepared](Development_Log/2026-10-03_M0_4_P2_PREPARED.md), [ROBOT_POWERED enablement](Development_Log/2026-10-03_M0_4_ROBOT_POWERED_ENABLEMENT.md) | `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` for the execution record |
| Post-ABORT recovery and thermal telemetry (offline) | 2026-10-03 | OFFLINE TESTED | [log](Development_Log/2026-10-03_POST_ABORT_THERMAL_OFFLINE_RELEASE.md), [runbook](../05_Firmware/MATDOG_Controller/POST_ABORT_RECOVERY_RUNBOOK.md) | Log |
| Calibration Persistence V1 (A/B NVS record, SAVE/ACK/RECONCILE, boot LOAD) | 2026-10-02 → 10-06 | **IMPLEMENTED** on `main`; SAVE/ACK and LOAD across a reset observed on the dev.3 candidate; no real power cycle; `RESTORE` NOT_IMPLEMENTED; hardware acceptance **ACCEPTANCE BLOCKED** | [PR-1 log](Development_Log/2026-10-08_PR1_PERSISTENCE_LAYOUT_V1_INTEGRATION.md), [dev.3 audit](Validation_Reports/MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md) | Code on `main`; audit document |

## 6. Wi-Fi, OTA and convergence lines

| Phase | Dates | Status | Documents | Evidence available |
|---|---|---|---|---|
| Wi-Fi / provisioning / OTA V1 → V3 | 2026-10-04 | IMPLEMENTED / OFFLINE TESTED; **not hardware tested**; no release | [V1 log](Development_Log/2026-10-04_WIFI_OTA_SHELLY_V1.md), [V3 offline](Validation_Reports/2026-10-04_WIFI_OTA_SHELLY_V3_OFFLINE.md), [runbook](../05_Firmware/MATDOG_Controller/WIFI_OTA_SHELLY_HARDWARE_RUNBOOK.md) | Documents; code on `main` since PR #39 |
| dev.1 convergence | 2026-10-05 | HISTORICAL record of the integration branch | [`Convergence/MATDOG_V0_2_DEV1.md`](../01_Docs/02_Architecture/Convergence/MATDOG_V0_2_DEV1.md) (quotes operator-machine paths) | Document. Branch `integration/matdog-controller-v0.2.0`: `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| Motion library G1 → G5-A (pure, host-tested, unwired) | G3 log dated 2026-09-28; other dates not established from `main` | Library **IMPLEMENTED**; execution suites and evidence **DEFERRED** (ADR-004); never hardware-authorized | [`src/motion/README.md`](../05_Firmware/MATDOG_Controller/src/motion/README.md), [CONTACT_STAND](../05_Firmware/MATDOG_Controller/src/motion/CONTACT_STAND.md), [STARTUP_TIMING](../05_Firmware/MATDOG_Controller/src/motion/STARTUP_TIMING.md), [STABILIZATION](../05_Firmware/MATDOG_Controller/src/motion/STABILIZATION.md), [PR-2 manifest](../05_Firmware/MATDOG_Controller/scripts/tests/motion_integration_manifest_pr2.json) | Sources and design notes on `main`. G3 report and G4/G4.1/G5-A/G35 evidence: **not on `main`**, `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| dev.2 boot census; hardware 13/13; calibration stopped at LF (block-read temperature artifact) | 2026-10-06 | dev.2 code on `main`; hardware result of the candidate | [log](Development_Log/2026-10-06_BOOT_SELFTEST_CENSUS_CORRECTION.md), [dev.2 audit](Validation_Reports/MATDOG_V0_2_DEV2_BOOT_SELFTEST_DELTA_AUDIT_2026-10-06.md) | Audit document; raw evidence root is on the operator machine (`EXTERNAL_ARCHIVE_PENDING_VERIFICATION`) |
| dev.3 thermal verdict from direct reads; 24/24; SAVE/ACK and LOAD across a reset; automatic recovery not triggered | 2026-10-06 | dev.3 code on `main`; candidate **not an accepted release** | [log](Development_Log/2026-10-06_THERMAL_DIRECT_VERDICT_DEV3.md), [dev.3 audit](Validation_Reports/MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md) | Audit document; raw evidence external |
| **Hardware Validation 2026-10-07 (dev.3)** | 2026-10-07 | Execution **COMPLETE**, acceptance **BLOCKED** (owner statement) | Referenced from the [root README](../README.md); no report on `main` | **Not on `main`**; not reconstructed. `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |

## 7. Repository convergence, PR #37 – #40 (2026-10-08)

Integration of **source** into `main`. Not a release, not a flash authorization, not a hardware
acceptance. Merge and recreated SHAs: [`COMMIT_PROVENANCE_MAP.md`](COMMIT_PROVENANCE_MAP.md).

| PR | Subject | Log |
|---|---|---|
| #37 | Flash layout safety notice (docs only) | [divergence notice](Development_Log/2026-10-08_FLASH_LAYOUT_DIVERGENCE_NOTICE.md), [safety notice](../05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md) |
| #38 | Persistence V1 and flash layout V1 (13 original SHAs preserved) | [PR-1](Development_Log/2026-10-08_PR1_PERSISTENCE_LAYOUT_V1_INTEGRATION.md) |
| #39 | Selective dev.1 integration; XGO extracts excluded | [PR-2](Development_Log/2026-10-08_PR2_SELECTIVE_DEV1_INTEGRATION.md), [ADR-004](Architecture_Decisions/ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md) |
| #40 | Selective dev.2/dev.3 source delta (7 commits recreated) | [PR-3](Development_Log/2026-10-08_PR3_DEV2_DEV3_SELECTIVE_INTEGRATION.md) |

## 8. Pull requests before #37, by period

Titles are paraphrased from the GitHub PR list (#15 is not described here); the repository documents above are the evidence.

| Period | PRs | Theme |
|---|---|---|
| before 2026-08-04 (dates not recorded here) | #1 | Repository foundation |
| before 2026-08-04 (dates not recorded here) | #2, #3, #5–#9 | Experiments, **ARCHIVED** |
| before 2026-08-04 (dates not recorded here) | #4 | Milestone I documentation |
| 2026-08-04 | #10–#12 | LF V25 closeout and cleanup |
| 2026-08-07 → 08-11 | #13, #14, #16–#19 | Canonical geometry-first architecture; Geometry Compiler phases to V5 |
| date not recorded here | #20 | Phase 2A0 close |
| 2026-08-27 → 08-28 | #21, #22 | ST3215 canonical archive; Calibrator V1 (closed) |
| 2026-09-05 | #23 | BNO085 viewer |
| 2026-09-15 → 09-24 | #24, #25, #26, #27 | Controller V0.1; alignment; ROBOT_POWERED; DALY KEY and power architecture |
| 2026-09-26 | #28, #29 | NextGen integration; LED V2 |
| 2026-09-27 → 09-29 | #30, #31, #32, #33 | CR0–CR2C; CR3 motion enablement; four-leg Full Calibration; current q0 |
| 2026-09-30 → 10-01 | #34 (closed), #35, #36 | Staged search (closed); TRUE 24-contact; status sync |

## 9. References that depend on material outside `main`

| Reference | Where | Class | Handling |
|---|---|---|---|
| G3 report permalink to commit `e170471…` | [`STARTUP_TIMING.md`](../05_Firmware/MATDOG_Controller/src/motion/STARTUP_TIMING.md) line 162; [PR-2 log](Development_Log/2026-10-08_PR2_SELECTIVE_DEV1_INTEGRATION.md) | **Archive-dependent, HISTORICAL.** The commit is not an ancestor of `main`; the link works only while a branch or tag still holds it. The PR-2 log calls it "immutable": that holds for the commit, not for the availability of the link | `STARTUP_TIMING.md` is SHA-256-pinned in the PR-2 manifest and is left unchanged. Provenance in [`COMMIT_PROVENANCE_MAP.md`](COMMIT_PROVENANCE_MAP.md); `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| Convergence audit report | [`FLASH_LAYOUT_SAFETY_NOTICE.md`](../05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md) Sources | Archive-dependent | Link replaced by provenance text; `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| dev.3 delta audit | three documents | Equivalent file is on `main` (blob-identical to the one at `b764c25`) | Relative links |
| Installed-firmware provenance commit `b3fd945` | several documents | Not an ancestor of `main` | SHA kept as text; recreated as `e028dd8` |
| Operator-machine paths in older logs and in the dev.1 convergence note | various | HISTORICAL, not resolvable from the repository | Left as recorded |
