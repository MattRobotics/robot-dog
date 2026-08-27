# NormaCore retirement — migration report

**Executed:** 2026-08-27 · **Result:** complete, 12/12 gates passed
**Outcome:** `MattRobotics/robot-dog` is now the sole active MATDOG repository.

---

## Branches retired

| Branch | Head SHA (verified against live remote) | Merge-base | Unique commits | Status |
|---|---|---|---:|---|
| `release/matdog-lf-calibrator-v25` | `f87dd1fbc7e8100d275c74f9af448642f3429680` | `32e3222c87016b7f5d7c1c1da497a4cea3e7b80a` | 92 | **deleted from remote** |
| `matdog/generic-v25-full-leg-engine` | `f4a87a443011f15e4f105858fec62fc15ca5a77c` | `4a8ed63…` (= `main`) | 16 | **deleted from remote** |
| `matdog/rf-calibrator-from-lf-v25` | `b2f7dac2eab7147917fccdfde702360da82ab7de` | — | 0 (local only) | local branch + worktree removed |

Both remote heads were verified with `git ls-remote` immediately before archiving and matched the
values in the task brief exactly.

## Archived locations

| Original | Archived to | Classification |
|---|---|---|
| `release/matdog-lf-calibrator-v25` | [`LF_V25_Hardware_Oracle/`](LF_V25_Hardware_Oracle/README.md) | HISTORICAL HARDWARE ORACLE |
| `matdog/generic-v25-full-leg-engine` | [`Generic_V25_Superseded_WIP/`](Generic_V25_Superseded_WIP/README.md) | SUPERSEDED WIP — never hardware validated |
| local uncommitted RF work | [`RF_Calibrator_Local_Only/`](RF_Calibrator_Local_Only/README.md) | SUPERSEDED WIP — never committed |
| MATDOG files on `norma-core/main` | [`NormaCore_Main_MATDOG_Content/`](NormaCore_Main_MATDOG_Content/README.md) | HISTORICAL — `main` retained |

Each carries `PROVENANCE.json`, the unique-commit list, a patch against the correct base, a file
inventory and `SHA256SUMS`.

## Archive verification

```text
LF_V25_Hardware_Oracle          21 files   SHA256SUMS OK
Generic_V25_Superseded_WIP      22 files   SHA256SUMS OK
NormaCore_Main_MATDOG_Content   22 files   SHA256SUMS OK
RF_Calibrator_Local_Only         5 files   SHA256SUMS OK
bundles                          2 files   SHA256SUMS OK
                                --------
                                72 files   0 FAILED
```

### Reconstruction proven before deletion

The LF V25 bundle was restored into a fresh repository and checked against the archived snapshot:

```text
restored head      f87dd1fbc7e8100d275c74f9af448642f3429680   matches
commits restored   92
matdog.rs sha256   c912f53c2a34c90ba8dec498c57426ccb6ebd1bc0a74c7aead91c4451a4b88b8
                   byte-identical to the archived source/ copy
```

## Migration gate — 12/12 PASS

| # | Gate | Result |
|---:|---|---|
| 1 | exact remote head SHA recorded | PASS |
| 2 | unique commit list recorded | PASS — 92 / 16 |
| 3 | unique diff preserved | PASS — 12 532 + 14 620 patch lines |
| 4 | source snapshot in robot-dog | PASS — 16 / 17 files |
| 5 | SHA256 archive manifest passes | PASS — 72 OK / 0 FAILED |
| 6 | hardware evidence links resolve | PASS — 0 broken |
| 7 | no dirty/untracked content only in a worktree | PASS — RF work archived and byte-verified |
| 8 | no stash contains unique work | PASS — 0 stashes |
| 9 | no open PR depends on the branches | PASS — 0 open PRs, 0 PRs headed by them |
| 10 | robot-dog archive committed and pushed | PASS |
| 11 | archive readable without the remote branch | PASS — source + patch + bundle |
| 12 | replacement location identified | PASS |

## MATDOG content preserved from `norma-core/main`

`main` is retained untouched, but its MATDOG-specific content was snapshotted anyway because the
branches were not the whole story — PRs #1, #2, #3, #11, #13, #28, #34 and #35 merged MATDOG work
directly into `main`. Twenty files archived and classified; the shared NormaCore infrastructure
(`calibrator.rs`, `mod.rs`, `elrobot.rs`, `so101.rs`) was deliberately **not** copied.

## Final repository state

```text
MattRobotics/norma-core
  refs/heads/main   4a8ed6337261553b79c928975808d294c9ca723b   UNCHANGED
  (no other branches)
```

- `main` untouched — same SHA before and after.
- No history rewritten. No force-push. No tags deleted (the `archive/2026-07-31/*` tags remain).
- Local worktrees `norma-core-generic-v25` and `norma-core-rf-calibrator` removed after archiving.
- Canonical `~/norma-core` checkout retained as the upstream/reference source.

## Governance from here

| Repository | Role |
|---|---|
| **`MattRobotics/robot-dog`** | **sole active MATDOG engineering repository** |
| `MattRobotics/norma-core` | reference/upstream fork; `main` retained; no active MATDOG development |

Reusing good upstream or native NormaCore code remains fine. The rule is that MATDOG-specific
ownership and development no longer live there.
