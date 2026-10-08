# Open items and blockers

State as documented at `main` = `9c2fe7868423f5267063285d7be93de00dc302db` (after PR #37–#40,
2026-10-08). Every row is taken from a document in this repository; nothing here is new evidence.
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
| E2 | Verification of the external backup of the repository, branches and evidence | `EXTERNAL_ARCHIVE_PENDING_VERIFICATION`; see [`EXTERNAL_ARCHIVES.md`](EXTERNAL_ARCHIVES.md) |
| E3 | Historical branches and tags not reachable from `main` | Kept; no deletion decided. Disposition table in [`EXTERNAL_ARCHIVES.md`](EXTERNAL_ARCHIVES.md) |
| E4 | G3 report permalinks to commit `e170471…` in `STARTUP_TIMING.md` and the PR-2 log | Depend on a non-`main` commit. `STARTUP_TIMING.md` is pinned by the PR-2 manifest and left unchanged; any edit needs a separate authorization and the full gate |
| E5 | XGO material already public on separate historical branches | Owner decision pending (ADR-004, PR-2 log). `main` history does not contain it |
| E6 | Full status synchronization of the 2026-10-01 snapshot in the root README | Partly done; the quoted snapshot stays labelled historical |
| E7 | Service / Provisioning / QC, Wi-Fi/OTA hardware tests, Web dashboard, IMU/BMS UI | FUTURE or NOT HARDWARE TESTED, per [ROADMAP](../01_Docs/02_Architecture/ROADMAP.md) |
