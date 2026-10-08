# MATDOG — PR-2: selective integration of the dev.1 line (2026-10-08)

**Scope:** code integration by merge and selective recreation, checked offline in a cloud
container. No hardware, serial, flash, NVS, EEPROM or OTA access. `MOTION_AUTHORIZED=0`.
**DO NOT FLASH MAIN**; see
[`FLASH_LAYOUT_SAFETY_NOTICE.md`](../../05_Firmware/MATDOG_Controller/FLASH_LAYOUT_SAFETY_NOTICE.md).

## Why a selective integration

The Integration dev.1 line (`d7aa369`) contains `3f23439`, which added 38 text files of
decompiler output from a third-party firmware image and artifacts derived from it. No licence or
authorization covering their redistribution is available (see
[ADR-004](../Architecture_Decisions/ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md)).
A full merge would put them into the history of `main`. Deleting them afterwards would not remove
them from that history.

## Git structure

| Item | Value |
|---|---|
| Base `main` | `fb02b8ed11ece11052f589d9222049f167336891` |
| Boundary merged | `13da04a290fc4e06d05636b5fd553476ed2d51e5`, 13 original commits `125d981` … `13da04a`, original SHAs preserved by a `--no-ff` merge commit |
| Recreated, new SHAs | three commits, each with `Original-SHA:` in its message (table below) |
| Not merged | `3f23439` and every descendant, including `d7aa369` and the dev.2/dev.3 line |

The original SHAs `1cec973`, `53aa962` and `d7aa369` are **not** ancestors of `main`. They stay
on `integration/matdog-controller-v0.2.0` and on the preservation branches. They are not claimed
as merged.

| Original | Recreated as | Files | Relation to the original |
|---|---|---|---|
| `1cec973` dev.1 identity and offline build isolation | selective recreation | 5 | all byte-identical |
| `53aa962` safety-gate extension | selective recreation | 8 | 5 byte-identical; `run_host_tests.sh` and `run_motion_convergence_tests.py` adapted; `motion_integration_manifest_pr2.json` new. `motion_convergence_sources.json` and `run_motion_fresh_oracle.py` not recreated |
| `d7aa369` persistence schema exposed through the owner boundary | selective recreation | 4 | all byte-identical |

## Merge conflicts

Two documentation conflicts, resolved inside the merge commit:
- `ROADMAP.md`: the PR-1 flash-safety note is kept; the superseded 2026-10-04 banner of the
  dev.1 line is dropped.
- `CHANGELOG.md`: the PR-1 entry and both dev.1 entries (Wi-Fi/OTA V3, post-abort recovery) are
  all kept.

## Excluded and deferred

All excluded artifacts are recorded by path and SHA-256 in
`05_Firmware/MATDOG_Controller/scripts/tests/motion_integration_manifest_pr2.json` as `DEFERRED`
(286 rows, no content).

| Class | Rows |
|---|---:|
| Third-party decompiler text (`xgo_static_extracts/`) | 38 |
| Other XGO-named artifacts, not individually classified | 9 |
| Pose audit tooling and G35 evidence bound to excluded XGO data | 69 |
| G4 / G4.1 / G5-A evidence, independence not reviewed | 97 |
| Audit tooling bound to the deferred evidence | 56 |
| Motion host tests bound to deferred oracles | 14 |
| Generators and one decision record | 3 |

Nothing deferred was validated by this integration. The sealed result of the integration branch
(20/20 motion qualification, 330/330 pinned files) is **not** claimed for `main`.

## Provenance cross-check (read-only)

`MattRobotics/xgolite-low-level-reconstruction` was read through a shallow clone; it was not
modified and nothing was copied from it.
- Its head `a1b34a8594e5bc76c76b1e3ddf89a3aef2b98298` is the commit named in the pose-audit
  README as the source snapshot.
- The firmware SHA-256 pinned in the excluded catalogue, `71032255ffac656c75234c2dcf6a40b307aefc753a92a04c1d0f707d67db6b0e`,
  appears in that repository, so both refer to the same firmware image.
- None of the 38 extracts exists there (0 of 38 by blob hash and by name).
- That repository has no `LICENSE` or `NOTICE`. Its README says third-party material whose
  redistribution is not appropriate is not to be committed. Its MATDOG transfer-boundary document
  says XGO values must not be transferred into MATDOG.

## What is integrated

| Layer | State on `main` after PR-2 |
|---|---|
| Code integrated | post-abort recovery and thermal acquisition, guarded release stages with DALY supervision, startup RF recovery, Wi-Fi/OTA V3 (TLS, ingest compiled out), charging presentation priority, dev.1 identity (`0.2.0-dev.1`), pure motion library G1–G5-A (`src/motion`, unwired) |
| Offline tests | see [`VALIDATION.md`](../../05_Firmware/MATDOG_Controller/VALIDATION.md), section PR-2 |
| Hardware validated | nothing new. The 2026-10-06 dev.2/dev.3 results belong to the dev.3 candidate, not to a `main` build |
| Deferred | the material in the previous section, and the motion execution suites that need it |

The motion library has no operational dispatch: the gate fails if any file outside `src/motion`
includes it. Standing limits: `MOTION_AUTHORIZED=0`, `RESTORE=NOT_IMPLEMENTED`, stand and gait
hardware not authorized.

## dev.3 status

The robot runs the dev.3 candidate (provenance `b3fd945`), which `main` does not contain.
Hardware Validation 2026-10-07: execution COMPLETE, acceptance BLOCKED (stated by the owner;
its report and result matrix are not in the repository and are not reconstructed here).

## Known inherited link defect

`05_Firmware/MATDOG_Controller/src/motion/STARTUP_TIMING.md` (line 162) links to
`09_Logs/Development_Log/2026-09-28_G3_STARTUP_TIMED_STAND.md`. That log exists only on
`feat/g5a-stabilization-feasibility` and `backup/history/gait-g41-pre-amend-6047e43`; it was never
on the integration branch, so the link is already dangling in the preserved original (`13da04a`).
It is the only broken relative link among the 210 checked in the Markdown files PR-2 changes. It
is left as is: the file is one of the 44 pinned `src/motion` sources, and editing it would change
its pinned SHA-256 and break byte-identity with the original.

## Open

- Owner decision on the already-public XGO material on other branches.
- PR-3 for dev.2/dev.3 must also be recreated selectively, because those commits descend from
  `3f23439`.
- Motion execution suites and evidence stay `DEFERRED` until their independence from the
  excluded data is shown or the data is regenerated clean-room.
- Publication of the 2026-10-07 validation report.
