# MATDOG — PR-3: selective integration of the dev.2/dev.3 source delta (2026-10-08)

**Scope:** code integration by selective recreation, checked offline in a cloud container. No
hardware, serial, flash, NVS, EEPROM or OTA access, no motion. `MOTION_AUTHORIZED=0`,
`RESTORE=NOT_IMPLEMENTED`. **DO NOT FLASH MAIN**; see
[`FLASH_LAYOUT_SAFETY_NOTICE.md`](../../05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md).

## Why recreation

The seven dev.2/dev.3 commits sit on top of `d7aa369`, whose ancestry contains `3f23439`, the
commit that introduced the third-party XGO decompiler extracts
([ADR-004](../Architecture_Decisions/ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md)).
Merging them would put those blobs into the history of `main`. Each delta was therefore applied as
a patch onto `main` (`git cherry-pick --no-commit`) and committed anew with the full `Original-SHA:`
in its message. The patch step is mechanical only: no original commit becomes an ancestor of
`main` (checked), and the new commits have `main` as their parent chain.

## Commit map

Base: `main` `e622a80d539c75a0cde4747419822fbdca7e34b5`.

| New | Original (full SHA) | Content | Files | Relation to the original |
|---|---|---|---:|---|
| `be94f78` | `bac652fe37705ecb5ecdd0521dc64db95926d47e` | dev.2 read-only boot servo census at ROBOT_POWERED startup | 10 | all byte-identical |
| `0f55120` | `d0ede365dbbdbf3940a31929e8e2e132faf9d49f` | dev.2 delta audit, log, changelog entry | 3 | 2 identical; `CHANGELOG.md` differs only in position of the added entry |
| `84fcf94` | `d67eaa1033ef2ea4c14fbefa97c9e46dfafe165b` | dev.2 qualification, flash, boot self-test and calibration stop (record) | 2 | identical |
| `1c9a144` | `62812cd66816b4b5506337bf7f3245ecce13a2ad` | dev.3 thermal verdict from direct reads only; identity `0.2.0-dev.3` | 11 | identical |
| `5574499` | `2f2324784d66bdc17e5a5c7cf39ca4c3dfc9a223` | host runner starts the post-abort recovery once | 2 | identical |
| `e028dd8` | `b3fd945bdaf37d192d97b605ac0f59b67f1dba45` | dev.3 delta audit, log, changelog entry (firmware provenance) | 3 | 2 identical; `CHANGELOG.md` differs only in position |
| `1d59dbe` | `b764c25c9530350331dbaa59ea2d84ddab3a3f69` | dev.3 qualification, flash, 24/24 calibration and persistence (record) | 2 | identical |

`CHANGELOG.md` is the only file that differs from its original: the added lines are identical, and
the dev.2 and dev.3 entries are placed above the PR-2 and PR-1 entries. The originals are not
ancestors of `main`; they stay preserved, unchanged, on
`backup/matdog-dev3-preservation-20261008` and `integration/matdog-controller-v0.2.0`.

Files excluded: none. The seven deltas touch no XGO, G35, Ghidra or evidence-tree path.

## Firmware correspondence with dev.3

Compared by tree and blob (mode and SHA) against the dev.3 tip `b764c25` and against the installed
provenance commit `b3fd945`.
- All 953 non-Markdown files of the tip: 950 are identical to dev.3; the remaining three are the
  two PR-2 test-gate scripts (`run_host_tests.sh`, `run_motion_convergence_tests.py`) and the
  PR-2 manifest, which are adaptations already on `main`. 278 non-Markdown files of dev.3 are
  absent: the deferred set declared in the manifest.
- Every file under `src/` is identical to `b764c25` and `b3fd945`, except
  `src/motion/STARTUP_TIMING.md`, a Markdown link repair made by PR-2.
- Checked blob by blob as identical: boot census (`ServoCensus.h`, `ServoBus.cpp`, `Controller.*`),
  thermal verdict (`ThermalConfirmation.*`), runner recovery (`calibration_hw_session.py`),
  configuration and identity (`BuildConfig.h`, `kFirmwareVersion = "0.2.0-dev.3"`), persistence
  (`CalibrationRecordStore.cpp`, `CalibrationRecordNvsBackend.cpp`, `CalibrationSaveMarker.cpp`),
  layout (`partitions.csv`, `matdog_layout.py`, `OtaLayoutContract.cpp`) and the flash scripts.

A build of `main` has another source commit and build ID than the installed image. Source
equivalence is not claimed to give an identical binary, and no binary comparison was made.
No pre-existing firmware anomaly was corrected.

## Closure check

The seven new commits add 83 objects (33 blobs). None of the 284 excluded blobs is reachable, no
path matches `xgo`, `G35_Pose`, `ghidra` or a deferred evidence tree, and a content scan finds no
decompiler or firmware-table marker, binary or secret. The added text contains no credential; the
device MAC address and local paths it quotes already appear in files on `main`.

## What is integrated

| Layer | State on `main` after PR-3 |
|---|---|
| Code | dev.2 boot census, dev.3 thermal verdict, runner recovery, identity `0.2.0-dev.3`, on top of PR-1 and PR-2 |
| Offline tests | see [`VALIDATION.md`](../../05_Firmware/MATDOG_Controller/VALIDATION.md), section PR-3 |
| Hardware results (historical, of the candidate) | the dev.2 and dev.3 delta audits: boot census 13/13; calibration stopped at LF under dev.2 (block-read temperature artifact); 24/24 under dev.3; SAVE/ACK and LOAD across a reset; automatic recovery not triggered; no real power cycle |
| Hardware acceptance / release acceptance | none. dev.3 Hardware Validation 2026-10-07: execution COMPLETE, acceptance BLOCKED. No release approved |
| Deferred | unchanged from PR-2 |

The 2026-10-07 validation report is not in the repository and is not reconstructed.

## Open

- Publish the 2026-10-07 report and the evidence behind the dev.2/dev.3 audits.
- Owner decision on the XGO material already public on other branches.
- A real power cycle of the persistence path and `RESTORE` remain open.
- Residuals recorded by the dev.2 and dev.3 audits (T-1…T-4, P-1, P-2, C-1, V-1) are unchanged.
