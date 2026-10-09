# External archives and references outside `main`

Purpose: tell a reader which referenced material is **on `main`**, which was held only by GitHub
branches that have since been **deleted**, which still exists as GitHub tags, and which lives outside
GitHub. Snapshot: `main` = `ddcd1cdb87eef29cc46996d6137d3f2b4c946f17` (PR #41 merge), 2026-10-09.

Statements about the non-public archive come from the project owner and are marked **owner-reported**.
The verification artifacts are not in this repository and are not reproduced here. This document
publishes no key, passphrase, credential, archive path, archive hash or archive content.

## 1. Status summary (2026-10-09)

| Item | State | Basis |
|---|---|---|
| Encrypted local backup of the repository histories and of the original commits | **Completed** | owner-reported |
| R1 — key restore | **PASS** | owner-reported |
| R2 — SHA-256 verification | **PASS, 208/208** | owner-reported |
| Git histories and original commits of the deleted branches | Preserved in a **non-public** archive | owner-reported |
| Second, independent physical copy of that archive | **Not declared.** Do not assume one exists | — |
| Local cleanup of the operator workstation | **Not complete / not reported.** Final result not yet available | — |
| The 13 historical branches recorded in section 2 | **All deleted** from GitHub | verified by listing the remote heads on 2026-10-09: none of the 13 is present |
| Branch `docs/c2-final-sync` | Exists **only while the Draft PR #42 is open** | verified. The target state is `main` alone, after the owner merges PR #42 and deletes this branch (owner action) |
| GitHub tags | 9 remain (section 3) | verified |
| XGO material on `main` | **None**; 0 of the 284 known excluded blobs reachable from `main` | verified (section 5) |

"R1" and "R2" are the owner's names for the two restore-test steps; their detailed procedure and
outputs are outside this repository.

## 2. Deleted GitHub branches (13 records)

**Counting.** The 13 records below are the 11 branches inventoried at C1 plus 2 working branches that were
deleted after their merge: `integration/pr3-dev3-selective` (PR #40) and `docs/c1-documentation-consolidation`
(PR #41). The inventory covers the branches tracked by the consolidation; it does not claim to list every
branch that ever existed. The work branches of PR #37, #38 and #39 were also deleted after merge; those PRs
are merged, their commits are in `main` through the merge commits, and they are neither listed individually
nor reconstructed here. `docs/c2-final-sync` is not counted: it is the current Draft branch.

The refs below no longer exist on GitHub. "Last tip" is the tip last verified by the cloud session
before deletion (full SHA). "Not in `main`" is the number of commits on the branch that were not
ancestors of `main` at `9c2fe78…`.

| Deleted branch | Last tip | Not in `main` | What it held | Where the content is now |
|---|---|---:|---|---|
| `feat/calibration-persistence-record-store-v1` | `7b258b36c257bd455f135aee2667d035c4929544` | 0 | Persistence V1 line | **On `main`** (PR #38, original SHAs preserved) |
| `integration/pr3-dev3-selective` | `49e76fbe68dce0a15ff9c1d759a75bb0571f7b2e` | 0 | PR-3 working branch | **On `main`** (PR #40) |
| `docs/c1-documentation-consolidation` | `484056caa62d73134d89d8b878ab235a288d077d` | 0 | PR #41 working branch | **On `main`** (PR #41) |
| `integration/matdog-controller-v0.2.0` | `d7aa369631202aed70458d0c7ff2980534882058` | 4 (`3f23439`, `1cec973`, `53aa962`, `d7aa369`) | dev.1 line; **descends from `3f23439`, which introduced the 38 XGO extracts** | Source delta recreated on `main` (see [`COMMIT_PROVENANCE_MAP.md`](COMMIT_PROVENANCE_MAP.md)); originals in the non-public archive (owner-reported) |
| `backup/matdog-dev3-preservation-20261008` | `b764c25c9530350331dbaa59ea2d84ddab3a3f69` | 11 | dev.2/dev.3 line, installed-firmware provenance `b3fd945`; **XGO ancestry** | Source delta recreated on `main`; originals in the non-public archive (owner-reported) |
| `feat/g5a-stabilization-feasibility` | `e1704719979789cd9c9f18741e4546725558797e` | 64 | Divergent gait/G5-A line; G3 report; G4/G4.1/G5-A and G35 evidence (**XGO-derived paths**) | Pure motion library on `main`; evidence DEFERRED; originals in the non-public archive (owner-reported) |
| `audit/matdog-v0.2.0-dev1-claude-handoff` | `b6d99573755f85993c189227b6a3f890fb15fe20` | 5 | Convergence audit handoff (`_audit/…`, 363 files; **XGO/G35 paths**) | Not imported; in the non-public archive (owner-reported) |
| `backup/history/gait-g41-pre-amend-6047e43` | `6047e43aa6dceb67772c171958cc610339fa2a64` | 57 | Pre-amend gait history (**XGO-derived paths**) | Not imported; in the non-public archive (owner-reported) |
| `backup/history/calib-endstop-preflight-c6d7ba3` | `c6d7ba35e9140a4b29ab7c573dc51b91730c1c98` | 1 | Pre-rewrite calibration history | Not imported; in the non-public archive (owner-reported) |
| `backup/history/calib-tolerance-pre-amend-b8ea0ba` | `b8ea0ba2774f6682c9746fe7731011e5e794191c` | 1 | Pre-amend calibration history | Not imported; in the non-public archive (owner-reported) |
| `backup/history/calib-zero-pose-pre-rebase-f4502ec` | `f4502ecc5a8bd1c55d2b4e8c6255463d599f8cc4` | 2 | Pre-rebase calibration history | Not imported; in the non-public archive (owner-reported) |
| `backup/wifi-phy-a06314e-20261008` | `a06314e8f3647bfb0f66d0fa827fcd24f65ffaca` | 1 | Wi-Fi PHY diagnostic, not integrated | Not integrated; in the non-public archive (owner-reported) |
| `backup/wip-m0-4-p2-20261008` | `43fa92a0d17f9a0bfcd274420517224aad043f68` | 1 | WIP M0.4 P2, not integrated | Not integrated; in the non-public archive (owner-reported) |

Which of these commits are in the archive is as reported by the owner for "Git histories and original
commits"; this repository cannot check it. Dated logs and records written before the deletion
(for example the PR-1/PR-2/PR-3 logs, the 2026-10-02/03 M0 logs and the flash-layout divergence
notice) name these branches as they were; they are unchanged and are read together with this table.

## 3. Tags (still on GitHub)

| Tag | Commit | On `main`? | Note |
|---|---|---|---|
| `matdog-controller-v0.1.0` | `c54862f` | yes | Frozen Controller V0.1 |
| `matdog-controller-nextgen-hw-validated-v1` | `c45858c` | yes | NextGen candidate, hardware-validated for its scope |
| `matdog-led-v2-focused-hw-validated-v1` | `88062e1` | yes | LED V2 focused hardware validation |
| `backup-esp32-m0-20261002T173142Z-14c19f227594` | `2757604` | yes | Release carrying the encrypted ESP32 flash backup; manifest and receipt in [`09_Backups/ESP32/`](../09_Backups/ESP32/README.md) |
| `archive/2026-07-31/milestone-i-clean-room-spec` | `002f067` | yes | Milestone I spec snapshot |
| `matdog-urdf-rev00-kinematic-baseline` | `2b6a3a0` | yes | URDF REV00 baseline; see [ADR-003](Architecture_Decisions/ADR-003_URDF_REV00_Kinematic_Baseline.md) |
| `archive/2026-07-31/calibration-sequence-upper-lower-hip-audit` | `cc70718` | **no** | Held only by this tag. Whether it is also in the non-public archive: `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| `archive/2026-08-11/pr19-geometry-v5-pre-squash` | `2890daf` | **no** | Pre-squash history of PR #19. Same pending note |
| `archive/2026-08-29/full-leg-calibrator-v1-h0` | `15f3fb8` | **no** | Calibrator V1 H0. Same pending note |

## 4. Material outside GitHub

| Item | Referred to by | Status |
|---|---|---|
| Hardware Validation 2026-10-07 report (dev.3): execution COMPLETE, acceptance BLOCKED | README, `FLASH_LAYOUT_SAFETY_NOTICE.md`, PR-3 log | Stated by the project owner. Report **not in the repository**. `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| Raw evidence roots of the dev.2/dev.3 audits (named inside the two audits) | the two delta audits | Operator workstation. `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| M0.4 P2 execution report and 2026-10-03 layout migration records | [`2026-10-03_M0_4_P2_PREPARED`](Development_Log/2026-10-03_M0_4_P2_PREPARED.md) (prepared, not executed at that date) | Execution report not on `main`. `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |
| Non-public encrypted archive (section 1) | this document | Owner-reported complete; R1 PASS, R2 208/208 PASS; contents beyond "Git histories and original commits" are not described here |
| Research repository `MattRobotics/xgolite-low-level-reconstruction` | ADR-004, PR-2 log | Separate repository; read-only provenance reference. Nothing imported |

## 5. XGO material and public exposure

- **Not on `main`.** The 38 third-party XGO extracts and the evidence derived from them are excluded
  by [ADR-004](Architecture_Decisions/ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md);
  they are recorded only by path and SHA-256 in
  [`motion_integration_manifest_pr2.json`](../05_Firmware/MATDOG_Controller/scripts/tests/motion_integration_manifest_pr2.json).
  Checked on 2026-10-09: none of the 284 excluded blobs known to the project is reachable from `main`,
  from the 9 remaining tags or from the 41 `refs/pull/*/head` refs.
- **Cannot be guaranteed removed.** Those blobs were public on the branches in section 2 before the
  branches were deleted. Deleting a branch removes the ref, not copies that were already fetched,
  forks, clones, caches, previously viewed or linked commit URLs, or objects GitHub still stores.
  This repository cannot guarantee that the material is no longer obtainable. The cloud session that
  wrote this document can see only the refs listed by the GitHub remote; it has no visibility into GitHub's
  internal storage, caches or third-party copies, and it has not verified the archive or the restore tests.
- **Not republished.** The archive and the deleted branches that carry XGO ancestry (`integration/matdog-controller-v0.2.0`,
  `backup/matdog-dev3-preservation-20261008`, `feat/g5a-stabilization-feasibility`,
  `audit/matdog-v0.2.0-dev1-claude-handoff`, `backup/history/gait-g41-pre-amend-6047e43`) must not be
  pushed back to a public ref. Doing so would reintroduce the exposure.

## 6. Logical recovery instructions

For a deleted branch whose content is not on `main`. The archive format and the key-custody procedure
are held by the owner and are intentionally not described here.

1. Ask the owner for the archive and for the key under the owner's custody procedure. Do not look for
   either in this repository: none is published.
2. Verify the archive before use. Expected check: the SHA-256 listing shipped with the archive matches
   (the owner's R2 check reported 208/208 PASS). A mismatch means stop.
3. Work on a **scratch local clone**, never in a clone with a public push path to `MattRobotics/robot-dog`.
4. Locate the commit by its full SHA from section 2 (`git cat-file -t <sha>` in the restored
   repository). If it resolves, `git branch <name> <sha>` recreates a local ref for inspection.
5. To bring content into `main`, use selective recreation as in PR #39 and PR #40
   (patch application, new commit, `Original-SHA:` trailer) and respect ADR-004. Never merge or
   push a ref that descends from `3f23439` or contains the deferred XGO paths.
6. The three tags that are not on `main` (section 3) still resolve from GitHub; recovery from the
   archive is needed only if a tag is later removed and the owner confirms they are archived.

## 7. What still resolves without any archive

Everything linked from [`HISTORY_INDEX.md`](HISTORY_INDEX.md) by a relative path is on `main`. The
only two links that depend on a non-`main` commit are the G3 report permalinks in
[`STARTUP_TIMING.md`](../05_Firmware/MATDOG_Controller/src/motion/STARTUP_TIMING.md) (pinned by the
PR-2 manifest and deliberately unchanged) and in the
[PR-2 log](Development_Log/2026-10-08_PR2_SELECTIVE_DEV1_INTEGRATION.md) (historical record). Both name
commit `e170471…`, whose only branch (`feat/g5a-stabilization-feasibility`) has been deleted; **the
permalinks should now be treated as unavailable**, and the report can be recovered only through the
archive (section 6). See [`COMMIT_PROVENANCE_MAP.md`](COMMIT_PROVENANCE_MAP.md).
`b3fd945…` is the commit the **installed** firmware was built from. A build of `main` has another
source commit and build ID, and is not shown to be byte-identical to the installed image.
