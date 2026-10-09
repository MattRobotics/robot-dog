# Open items and blockers

State as documented at `main` = `ddcd1cdb87eef29cc46996d6137d3f2b4c946f17` (after PR #37–#41;
firmware content as of PR #40, `9c2fe78…`; updated 2026-10-09). Every row is taken from a document in this repository; nothing here is new evidence.
Standing invariants: **`MOTION_AUTHORIZED=0`**, **`RESTORE=NOT_IMPLEMENTED`**, **DO NOT FLASH
`main`**. Current-state authority remains the [root README](../README.md).

## A. Firmware and hardware acceptance

| # | Item | State | Source |
|---|---|---|---|
| A1 | Hardware Validation 2026-10-07 of the dev.3 candidate | Execution COMPLETE, acceptance **BLOCKED** (owner statement). The report is not in the repository and is not reconstructed. Counts and the cause of the blocking defect are deliberately not quoted | [README](../README.md), [`FLASH_LAYOUT_SAFETY_NOTICE.md`](../05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md) |
| A2 | Release approval of `0.2.0-dev.3` | None. dev.3 is a candidate, not an accepted release. Integrated source on `main` is "Unreleased" | [`CHANGELOG.md`](../05_Firmware/MATDOG_Controller/CHANGELOG.md) |
| A3 | Is a `main` build equal to the installed firmware? | Not shown. Source files under `src/` match the dev.3 provenance commit (except one Markdown link repair), but commit, build ID and binary differ; no binary comparison was made | [PR-3 log](Development_Log/2026-10-08_PR3_DEV2_DEV3_SELECTIVE_INTEGRATION.md) |
| A4 | Flashing from `main` | **Forbidden** until a separately authorized procedure exists | [`FLASH_LAYOUT_SAFETY_NOTICE.md`](../05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md) |

## B. Calibration persistence

| # | Item | State |
|---|---|---|
| B1 | NVS record store (A/B), SAVE/ACK/RECONCILE, boot LOAD | IMPLEMENTED on `main` (PR #38); offline/host tested |
| B2 | SAVE/ACK and LOAD across a reset on the dev.3 candidate | Observed 2026-10-06 (dev.3 audit). A **real power cycle** was not performed (residual V-1) |
| B3 | Runtime calibration state | In RAM. Boot LOAD never admits a transform by itself |
| B4 | Automatic `RESTORE` | **NOT_IMPLEMENTED** |
| B5 | Hardware acceptance of persistence | **BLOCKED**; not promoted by any document |

## C. Motion and envelopes

| # | Item | State |
|---|---|---|
| C1 | Operational envelopes / JointLimits | BLOCKED; 0/12 admitted. See [`DEVELOPMENT_GATES.md`](../05_Firmware/MATDOG_Controller/DEVELOPMENT_GATES.md) |
| C2 | Stand / gait on hardware | BLOCKED; requires persistence acceptance, approved limits and independent motion-safety validation |
| C3 | Pure motion library G1–G5-A | On `main`, **unwired** (no path to the servo bus); hardware execution never authorized |
| C4 | Motion execution suites, fresh oracles, G35/G4/G4.1/G5-A evidence | DEFERRED by [ADR-004](Architecture_Decisions/ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md); recorded by path and SHA-256 only |

## D. Residuals recorded by the dev.2/dev.3 audits (unchanged)

T-1 block-read temperature artifact not explained · T-2 a direct read can also be spurious ·
T-3 abort verdicts keep `probe_failure=OVER_TEMPERATURE` at executor level · T-4 no periodic
unconditional direct sampling · P-1 no recovery to q0 across a reboot · P-2 automatic post-abort
recovery covered offline only · C-1 LF UPPER MIN margin of 1..3 ticks · V-1 real power cycle and
fixture/operator gates not run. Full text: [dev.3 audit, Residuals](Validation_Reports/MATDOG_V0_2_DEV3_THERMAL_DELTA_AUDIT_2026-10-06.md).

## E. Repository and archive items

| # | Item | State |
|---|---|---|
| E1 | Publish the 2026-10-07 report and the evidence behind the dev.2/dev.3 audits | Open; owner action |
| E2 | Non-public encrypted archive of the Git histories and original commits | Owner-reported: backup completed; R1 key restore PASS; R2 SHA-256 208/208 PASS. Verification artifacts are not in the repository. **No second independent physical copy is declared.** See [`EXTERNAL_ARCHIVES.md`](EXTERNAL_ARCHIVES.md) |
| E2a | Local cleanup of the operator workstation | Not complete / not reported; final result not yet available. Do not treat as done |
| E3 | Historical branches | **The 13 historical branches recorded in the consolidation inventory are deleted from GitHub** (11 inventoried at C1 plus the PR #40 and PR #41 work branches; not a claim to list every branch that ever existed). `docs/c2-final-sync` exists only while Draft PR #42 is open; target: `main` alone after the owner merges it and deletes the branch. Last tips in [`EXTERNAL_ARCHIVES.md`](EXTERNAL_ARCHIVES.md) §2; recovery steps in §6. Tags: 9 remain; 3 of them (`cc70718`, `2890daf`, `15f3fb8`) are not on `main` and their presence in the archive is `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| E4 | G3 report permalinks to commit `e170471…` in `STARTUP_TIMING.md` and the PR-2 log | Depend on a non-`main` commit whose only branch is deleted: treat as unavailable; the report is recoverable through the archive only. `STARTUP_TIMING.md` is pinned by the PR-2 manifest and left unchanged; any edit needs a separate authorization and the full gate |
| E5 | XGO material that was public on the deleted branches | Not on `main` (0 of 284 known blobs reachable from `main`, the 9 tags and the 41 PR refs). Removal from GitHub storage, caches, forks and clones **cannot be guaranteed**; any further step is an owner decision (ADR-004) |
| E6 | Full status synchronization of the 2026-10-01 snapshot in the root README | Partly done; the quoted snapshot stays labelled historical |
| E7 | Service / Provisioning / QC, Wi-Fi/OTA hardware tests, Web dashboard, IMU/BMS UI | FUTURE or NOT HARDWARE TESTED, per [ROADMAP](../01_Docs/02_Architecture/ROADMAP.md) |
