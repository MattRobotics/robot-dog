# Development Log — index

Chronological handoffs and milestone records. A log states what was current **on its date**; its
"next steps" are historical. Status of a result is whatever the log itself records for its scope; this
index adds no status of its own. Current state: [root README](../../README.md). Whole-project
chronology: [`HISTORY_INDEX.md`](../HISTORY_INDEX.md).

Titles are quoted from each file (some are in Italian). Controller V0.1 / G2 / G3 history for
2026-09-01 – 2026-09-19 has no dated log here; it is recorded in
[`VALIDATION.md`](../../05_Firmware/MATDOG_Controller/VALIDATION.md) (Sessions 1, 2, 2.1, 2.2, dated
2026-09-15) and in the [ROADMAP](../../01_Docs/02_Architecture/ROADMAP.md). The checkpoint file
[`2026-10-03_M0_4_P1_CHECKPOINT.json`](2026-10-03_M0_4_P1_CHECKPOINT.json) accompanies the
M0.4 P1 closure log.


## Geometry, URDF and visual zero (June–July 2026)

| Log | Title |
|---|---|
| [`2026-06-30_URDF_REV00_Kinematic_Baseline`](2026-06-30_URDF_REV00_Kinematic_Baseline.md) | MATDOG URDF REV00 Kinematic Baseline Completed |
| [`2026-07-06_Visual_Zero_Calibration_and_Static_Control_Checkpoint`](2026-07-06_Visual_Zero_Calibration_and_Static_Control_Checkpoint.md) | 2026-07-06 - Visual-Zero Calibration and Static Control Checkpoint |
| [`2026-07-10_C5R_POST_DIGITAL_ZERO_HANDOFF`](2026-07-10_C5R_POST_DIGITAL_ZERO_HANDOFF.md) | MATDOG — C5-R Post-Digital-Zero Handoff |

## Native calibrator handoffs and cleanup (July–August 2026)

| Log | Title |
|---|---|
| [`2026-07-21_NATIVE_NORMACORE_CALIBRATOR_HANDOFF`](2026-07-21_NATIVE_NORMACORE_CALIBRATOR_HANDOFF.md) | MATDOG — Passaggio di consegna canonico |
| [`2026-07-25_NATIVE_NORMACORE_CALIBRATOR_HANDOFF`](2026-07-25_NATIVE_NORMACORE_CALIBRATOR_HANDOFF.md) | MATDOG — passaggio di consegna dopo il pilot nativo M12 MIN |
| [`2026-07-28_MATDOG_M12_MIN_MAX_HANDOFF`](2026-07-28_MATDOG_M12_MIN_MAX_HANDOFF.md) | MATDOG — handoff dopo chiusura LF_UPPER M12 MIN/MAX |
| [`2026-08-04_LF_V25_AND_REPOSITORY_CLEANUP`](2026-08-04_LF_V25_AND_REPOSITORY_CLEANUP.md) | MATDOG repository cleanup and LF V25 closeout |
| [`2026-08-11_PHASE2A0_GEOMETRY_V5_CLOSEOUT`](2026-08-11_PHASE2A0_GEOMETRY_V5_CLOSEOUT.md) | MATDOG — Phase 2A0 / Geometry Compiler V5 closeout |

## ST3215 canonical archive and rebuild (2026-08-27)

| Log | Title |
|---|---|
| [`2026-08-27_ST3215_CANONICAL_ARCHIVE_HANDOFF`](2026-08-27_ST3215_CANONICAL_ARCHIVE_HANDOFF.md) | Handoff — ST3215 bench phase closed, reassembly begins |

## Power, KEY, charging (September 2026)

| Log | Title |
|---|---|
| [`2026-09-20_MATDOG_POWER_KEY_CHARGING_NEXTGEN_HANDOFF`](2026-09-20_MATDOG_POWER_KEY_CHARGING_NEXTGEN_HANDOFF.md) | MATDOG — Power, KEY, Charging & NextGen Controller Architecture Handoff |
| [`2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT`](2026-09-24_MATDOG_POWER_CHARGING_USB_VALIDATION_CLOSEOUT.md) | MATDOG — Power, Charging & External USB Validation Closeout |

## NextGen integration I1–I9, F0, candidate V3 and LED V2 (2026-09-25/26)

| Log | Title |
|---|---|
| [`2026-09-25_F0_FINAL_FLASH_READINESS`](2026-09-25_F0_FINAL_FLASH_READINESS.md) | F0 — Final flash readiness gate |
| [`2026-09-25_F0_RERUN_CANDIDATE`](2026-09-25_F0_RERUN_CANDIDATE.md) | F0 (re-run) — Final flash readiness gate, against the integrated candidate |
| [`2026-09-25_F0_RERUN_CANDIDATE_V2`](2026-09-25_F0_RERUN_CANDIDATE_V2.md) | F0 (re-run V2) — Final flash readiness gate, against the I7/I8-hardened candidate |
| [`2026-09-25_I1_REPOSITORY_TRUTH_RECONCILIATION`](2026-09-25_I1_REPOSITORY_TRUTH_RECONCILIATION.md) | I1 — Repository truth reconciliation |
| [`2026-09-25_I2_LED_STATUS_MANAGER`](2026-09-25_I2_LED_STATUS_MANAGER.md) | I2 — LED Status Manager |
| [`2026-09-25_I3_SOURCE_SIGNATURE`](2026-09-25_I3_SOURCE_SIGNATURE.md) | I3 — Diagnostics/Maintenance: SOURCE_SIGNATURE |
| [`2026-09-25_I4_ACTUATOR_RUNTIME`](2026-09-25_I4_ACTUATOR_RUNTIME.md) | I4 — Safe Actuator runtime boundary |
| [`2026-09-25_I4_I5_CONTROLLER_WIRING`](2026-09-25_I4_I5_CONTROLLER_WIRING.md) | I4/I5 — Controller wiring: fail-closed infrastructure integration |
| [`2026-09-25_I5_CALIBRATION_EXECUTION_ENGINE`](2026-09-25_I5_CALIBRATION_EXECUTION_ENGINE.md) | I5 — Calibration Execution Architecture |
| [`2026-09-25_I6_HOSTLINK_AUDIT`](2026-09-25_I6_HOSTLINK_AUDIT.md) | I6 — HostLink semantic layer: audit |
| [`2026-09-25_I6_HOSTLINK_IMPLEMENTATION`](2026-09-25_I6_HOSTLINK_IMPLEMENTATION.md) | I6 — HostLink semantic layer: implementation |
| [`2026-09-25_I7_I8_HARDENING`](2026-09-25_I7_I8_HARDENING.md) | I7/I8 — network transport hardening before the one-flash candidate |
| [`2026-09-25_I7_I8_NETWORK_TRANSPORT_IMPLEMENTATION`](2026-09-25_I7_I8_NETWORK_TRANSPORT_IMPLEMENTATION.md) | I7/I8 — Network transport implementation (supersedes the earlier reconsiderations) |
| [`2026-09-25_I7_RECONSIDERED`](2026-09-25_I7_RECONSIDERED.md) | I7 — Wi-Fi/OTA: reconsidered under the objective change, scope unchanged |
| [`2026-09-25_I7_WIFI_OTA_AUDIT`](2026-09-25_I7_WIFI_OTA_AUDIT.md) | I7 — Wi-Fi / OTA transport + security: audit |
| [`2026-09-25_I8_RECONSIDERED`](2026-09-25_I8_RECONSIDERED.md) | I8 — Read-only Web foundation: reconsidered under the objective change |
| [`2026-09-25_I8_WEB_FOUNDATION_DEFERRED`](2026-09-25_I8_WEB_FOUNDATION_DEFERRED.md) | I8 — Read-only Web foundation: deferred |
| [`2026-09-25_I9_INTEGRATED_SOFTWARE_FREEZE`](2026-09-25_I9_INTEGRATED_SOFTWARE_FREEZE.md) | I9 — Integrated software freeze |
| [`2026-09-25_INTEGRATED_FREEZE_CANDIDATE`](2026-09-25_INTEGRATED_FREEZE_CANDIDATE.md) | Integrated software freeze — MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE |
| [`2026-09-25_INTEGRATED_FREEZE_CANDIDATE_V2`](2026-09-25_INTEGRATED_FREEZE_CANDIDATE_V2.md) | Integrated software freeze V2 — MATDOG NEXTGEN INTEGRATED HARDWARE VALIDATION CANDIDATE |
| [`2026-09-25_RECOVERY_BACKUP_AND_CANDIDATE_V3`](2026-09-25_RECOVERY_BACKUP_AND_CANDIDATE_V3.md) | Recovery-backup gate hardening + candidate V3 (real Wi-Fi/OTA credentials) |
| [`2026-09-26_HARDWARE_VALIDATION_CANDIDATE_V3`](2026-09-26_HARDWARE_VALIDATION_CANDIDATE_V3.md) | Hardware validation — MATDOG NEXTGEN INTEGRATED CANDIDATE V3, first flash |
| [`2026-09-26_LED_STATUS_MANAGER_V2_FINAL`](2026-09-26_LED_STATUS_MANAGER_V2_FINAL.md) | MATDOG LED Status Manager V2 final — 2026-09-26 |
| [`2026-09-26_LED_STATUS_MANAGER_V2_HW_VALIDATION`](2026-09-26_LED_STATUS_MANAGER_V2_HW_VALIDATION.md) | MATDOG LED Status Manager V2 — focused hardware validation |

## Calibration readiness CR0–CR3 (2026-09-27/28)

| Log | Title |
|---|---|
| [`2026-09-27_CALIBRATION_READINESS_CR0_CR1`](2026-09-27_CALIBRATION_READINESS_CR0_CR1.md) | MATDOG Calibration Readiness — CR0 / CR1 implementation record |
| [`2026-09-27_CALIBRATION_READINESS_CR2B_Q0_ACQUISITION`](2026-09-27_CALIBRATION_READINESS_CR2B_Q0_ACQUISITION.md) | MATDOG Calibration Readiness — CR2-B same-session read-only q0 acquisition |
| [`2026-09-27_CALIBRATION_READINESS_CR2_Q0_BOOTSTRAP`](2026-09-27_CALIBRATION_READINESS_CR2_Q0_BOOTSTRAP.md) | MATDOG Calibration Readiness — CR2 read-only q0 bootstrap foundation |
| [`2026-09-27_CR2C_Q0_HARDWARE_SESSION`](2026-09-27_CR2C_Q0_HARDWARE_SESSION.md) | CR2-C — Q0 hardware session |
| [`2026-09-27_CR3_M0_M2_ENTRY`](2026-09-27_CR3_M0_M2_ENTRY.md) | MATDOG CR3-M0 — Gap report and execution entry |
| [`2026-09-27_CR3_M3_BACKEND`](2026-09-27_CR3_M3_BACKEND.md) | MATDOG CR3-M3 — Minimal production ServoBus actuator backend |
| [`2026-09-27_CR3_Q0_EVIDENCE_PERSISTENCE`](2026-09-27_CR3_Q0_EVIDENCE_PERSISTENCE.md) | MATDOG CR3 — Persisted CR2-C q0 evidence package |
| [`2026-09-28_CR3_M5_PRODUCTION_COMPOSITION`](2026-09-28_CR3_M5_PRODUCTION_COMPOSITION.md) | MATDOG CR3-M5 — fail-closed production actuator composition |
| [`2026-09-28_CR3_M6_ENVELOPES_AND_SESSION_CLOSEOUT`](2026-09-28_CR3_M6_ENVELOPES_AND_SESSION_CLOSEOUT.md) | MATDOG CR3 — Priority 5/6 gates and offline session closeout |
| [`2026-09-28_CR3_OFFLINE_SESSION_ENTRY`](2026-09-28_CR3_OFFLINE_SESSION_ENTRY.md) | MATDOG CR3 — offline completion session entry |

## Full Calibration, four legs and 24 contacts (2026-09-29 – 2026-10-01)

| Log | Title |
|---|---|
| [`2026-09-29_CALIBRATION_FULL_4LEG`](2026-09-29_CALIBRATION_FULL_4LEG.md) | MATDOG — Full Calibration generalized to four legs (LF → RF → RH → LH, one build) |
| [`2026-09-29_FULL_CALIBRATION_V25_ORACLE_TRACEABILITY`](2026-09-29_FULL_CALIBRATION_V25_ORACLE_TRACEABILITY.md) | Full Calibration staged search — LF V25 hardware-oracle traceability (2026-09-29) |
| [`2026-09-30_FULL_CALIBRATION_24_CONTACT_V25_TRACEABILITY`](2026-09-30_FULL_CALIBRATION_24_CONTACT_V25_TRACEABILITY.md) | 24-contact Full Calibration — LF V25 hardware-oracle traceability (2026-09-30) |
| [`2026-09-30_TRUE_24_CONTACT_FULL_CALIBRATION`](2026-09-30_TRUE_24_CONTACT_FULL_CALIBRATION.md) | TRUE Full Calibration = 24 contacts — LF V25 full-leg state machine generalized (2026-09-30) |
| [`2026-10-01_PROJECT_STATUS_DOCUMENTATION_SYNC`](2026-10-01_PROJECT_STATUS_DOCUMENTATION_SYNC.md) | MATDOG — Current-status documentation synchronization (2026-10-01) |

## Flash layout M0 and migration (2026-10-02/03)

| Log | Title |
|---|---|
| [`2026-10-02_M0_1_MIGRATION_WRITE_HARDENING`](2026-10-02_M0_1_MIGRATION_WRITE_HARDENING.md) | MATDOG — M0.1: migration write and recovery hardening |
| [`2026-10-02_M0_2_HARDWARE_ALIGNMENT`](2026-10-02_M0_2_HARDWARE_ALIGNMENT.md) | MATDOG — M0.2: actual hardware integration and migration readiness |
| [`2026-10-02_M0_FLASH_LAYOUT_OFFLINE`](2026-10-02_M0_FLASH_LAYOUT_OFFLINE.md) | MATDOG M0 — Esito della pianificazione flash offline |
| [`2026-10-03_M0_4_P1_CLOSED`](2026-10-03_M0_4_P1_CLOSED.md) | MATDOG M0.4 — chiusura P1 e backup remoto verificato |
| [`2026-10-03_M0_4_P2_PREPARED`](2026-10-03_M0_4_P2_PREPARED.md) | MATDOG M0.4 — P2 preparata, non eseguita |
| [`2026-10-03_M0_4_ROBOT_POWERED_ENABLEMENT`](2026-10-03_M0_4_ROBOT_POWERED_ENABLEMENT.md) | MATDOG M0.4 — ROBOT_POWERED migration enablement |
| [`2026-10-03_POST_ABORT_THERMAL_OFFLINE_RELEASE`](2026-10-03_POST_ABORT_THERMAL_OFFLINE_RELEASE.md) | MATDOG — revisione offline recovery post-ABORT e telemetria termica |

## Wi-Fi/OTA V1, dev.2 and dev.3 (2026-10-04/06)

| Log | Title |
|---|---|
| [`2026-10-04_WIFI_OTA_SHELLY_V1`](2026-10-04_WIFI_OTA_SHELLY_V1.md) | MATDOG Wi-Fi / provisioning / OTA V1 — development log |
| [`2026-10-06_BOOT_SELFTEST_CENSUS_CORRECTION`](2026-10-06_BOOT_SELFTEST_CENSUS_CORRECTION.md) | 2026-10-06 — ROBOT_POWERED boot census correction (v0.2.0-dev.2) |
| [`2026-10-06_THERMAL_DIRECT_VERDICT_DEV3`](2026-10-06_THERMAL_DIRECT_VERDICT_DEV3.md) | 2026-10-06 — thermal verdict from direct reads (v0.2.0-dev.3) |

## Repository convergence (2026-10-08)

| Log | Title |
|---|---|
| [`2026-10-08_FLASH_LAYOUT_DIVERGENCE_NOTICE`](2026-10-08_FLASH_LAYOUT_DIVERGENCE_NOTICE.md) | MATDOG — Flash layout divergence: documentation mitigation (2026-10-08) |
| [`2026-10-08_PR1_PERSISTENCE_LAYOUT_V1_INTEGRATION`](2026-10-08_PR1_PERSISTENCE_LAYOUT_V1_INTEGRATION.md) | MATDOG — PR-1: Calibration Persistence V1 and flash layout V1 integration (2026-10-08) |
| [`2026-10-08_PR2_SELECTIVE_DEV1_INTEGRATION`](2026-10-08_PR2_SELECTIVE_DEV1_INTEGRATION.md) | MATDOG — PR-2: selective integration of the dev.1 line (2026-10-08) |
| [`2026-10-08_PR3_DEV2_DEV3_SELECTIVE_INTEGRATION`](2026-10-08_PR3_DEV2_DEV3_SELECTIVE_INTEGRATION.md) | MATDOG — PR-3: selective integration of the dev.2/dev.3 source delta (2026-10-08) |

Total: 64 logs.
