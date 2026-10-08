# External archives and references outside `main`

Purpose: tell a reader which referenced material is **on `main`**, which exists only on historical
branches/tags of this GitHub repository, and which lives outside GitHub. It is a reading aid, not a
backup attestation.

> **Status of every external archive: `EXTERNAL_ARCHIVE_PENDING_VERIFICATION`.**
> This document does not state that any backup exists, where it is, what it contains, or that it
> restores correctly. No backup path, backup hash or restore result is recorded here. Those facts
> will be added only after the operator verifies the archive and provides the evidence.

Rules that apply to this repository (see
[ADR-004](Architecture_Decisions/ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md)):

- Third-party reverse-engineering material (the 38 XGO extracts and the evidence derived from it)
  is **not** republished here. It is recorded only by path and SHA-256 in
  [`motion_integration_manifest_pr2.json`](../05_Firmware/MATDOG_Controller/scripts/tests/motion_integration_manifest_pr2.json).
- A permalink to a commit that is not an ancestor of `main` is provenance only. It may stop
  resolving when its branch is removed. Where equivalent content exists on `main`, documents link to
  it with a relative path.

Snapshot taken against `main` = `9c2fe7868423f5267063285d7be93de00dc302db`.

## A. Historical branches (GitHub) — reachability from `main`

| Branch | Tip | Commits not in `main` | Holds | Disposition for this documentation |
|---|---|---:|---|---|
| `feat/calibration-persistence-record-store-v1` | `7b258b3` | 0 | Persistence V1 line | Fully on `main` (PR #38). Nothing external |
| `integration/matdog-controller-v0.2.0` | `d7aa369` | 4 (`3f23439`, `1cec973`, `53aa962`, `d7aa369`) | dev.1 line incl. the XGO extracts | Source delta recreated on `main`; XGO excluded. `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| `backup/matdog-dev3-preservation-20261008` | `b764c25` | 11 | dev.2/dev.3 line, installed-firmware provenance `b3fd945` | Source delta recreated on `main`; original SHAs kept in [`COMMIT_PROVENANCE_MAP.md`](COMMIT_PROVENANCE_MAP.md). `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| `feat/g5a-stabilization-feasibility` | `e170471` | 64 | Divergent gait/G5-A line; G3 report `09_Logs/Development_Log/2026-09-28_G3_STARTUP_TIMED_STAND.md`; G4/G4.1/G5-A evidence | Pure motion library is on `main`; evidence DEFERRED. `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| `audit/matdog-v0.2.0-dev1-claude-handoff` | `b6d9957` | 5 | Convergence audit handoff (`_audit/…`, 363 files, includes XGO/G35 material) | Not imported. The audit report `MATDOG_FIRMWARE_CONVERGENCE_AUDIT_V0_2.md` is `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| `backup/history/gait-g41-pre-amend-6047e43` | `6047e43` | 57 | Pre-amend gait history incl. XGO files | Not imported. `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| `backup/history/calib-endstop-preflight-c6d7ba3` | `c6d7ba3` | 1 | Pre-rewrite calibration history | `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| `backup/history/calib-tolerance-pre-amend-b8ea0ba` | `b8ea0ba` | 1 | Pre-amend calibration history | `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| `backup/history/calib-zero-pose-pre-rebase-f4502ec` | `f4502ec` | 2 | Pre-rebase calibration history | `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| `backup/wifi-phy-a06314e-20261008` | `a06314e` | 1 | Wi-Fi PHY diagnostic, not integrated | `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| `backup/wip-m0-4-p2-20261008` | `43fa92a` | 1 | WIP M0.4 P2, not integrated | `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |

## B. Tags (GitHub)

| Tag | Commit | On `main`? | Note |
|---|---|---|---|
| `matdog-controller-v0.1.0` | `c54862f` | yes | Frozen Controller V0.1 |
| `matdog-controller-nextgen-hw-validated-v1` | `c45858c` | yes | NextGen candidate, hardware-validated for its scope |
| `matdog-led-v2-focused-hw-validated-v1` | `88062e1` | yes | LED V2 focused hardware validation |
| `backup-esp32-m0-20261002T173142Z-14c19f227594` | `2757604` | yes | Release carrying the encrypted ESP32 flash backup; manifest and receipt in [`09_Backups/ESP32/`](../09_Backups/ESP32/README.md) |
| `archive/2026-07-31/milestone-i-clean-room-spec` | `002f067` | yes | Milestone I spec snapshot |
| `archive/2026-07-31/calibration-sequence-upper-lower-hip-audit` | `cc70718` | no | `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| `archive/2026-08-11/pr19-geometry-v5-pre-squash` | `2890daf` | no | Pre-squash history of PR #19. `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| `archive/2026-08-29/full-leg-calibrator-v1-h0` | `15f3fb8` | no | Calibrator V1 H0. `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| `matdog-urdf-rev00-kinematic-baseline` | `2b6a3a0` | yes | URDF REV00 baseline; see [ADR-003](Architecture_Decisions/ADR-003_URDF_REV00_Kinematic_Baseline.md) |

## C. Material outside GitHub

| Item | Referred to by | Status |
|---|---|---|
| Hardware Validation 2026-10-07 report (dev.3): execution COMPLETE, acceptance BLOCKED | README, `FLASH_LAYOUT_SAFETY_NOTICE.md`, PR-3 log | Stated by the project owner. Report **not in the repository**. `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| Raw evidence roots of the dev.2/dev.3 audits (named inside the two audits) | the two delta audits | Operator machine. `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| M0.4 P2 execution report and 2026-10-03 layout migration records | [`2026-10-03_M0_4_P2_PREPARED`](Development_Log/2026-10-03_M0_4_P2_PREPARED.md) (prepared, not executed at that date) | Execution report not on `main`. `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| Local backup copies of the repository and of the evidence trees | none (not referenced by `main`) | Not documented here by design |
| Research repository `MattRobotics/xgolite-low-level-reconstruction` | ADR-004, PR-2 log | Separate repository; read-only provenance reference. Nothing imported |

## D. What still resolves without any archive

Everything linked from [`HISTORY_INDEX.md`](HISTORY_INDEX.md) by a relative path is on `main`. The
only two links that depend on a non-`main` commit are the G3 report permalinks in
[`STARTUP_TIMING.md`](../05_Firmware/MATDOG_Controller/src/motion/STARTUP_TIMING.md) (pinned by the
PR-2 manifest and deliberately unchanged) and in the
[PR-2 log](Development_Log/2026-10-08_PR2_SELECTIVE_DEV1_INTEGRATION.md) (historical record). Both name
commit `e170471…` and are listed in [`COMMIT_PROVENANCE_MAP.md`](COMMIT_PROVENANCE_MAP.md).
