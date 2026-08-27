# RF calibrator — local-only uncommitted work

> # ⚠️ NEVER PUSHED · NEVER COMMITTED · EXISTED ONLY ON THE ASUS
> # SUPERSEDED DEVELOPMENT WIP · STATION-MEDIATED · NOT HARDWARE VALIDATED
> # DO NOT USE AS HARDWARE AUTHORIZATION

This archive exists because the audit found **1 176 lines of uncommitted work** that existed
nowhere except one local worktree — not on any remote, not in any commit, not in any stash.
Losing that worktree would have destroyed it permanently.

---

## Provenance

| Field | Value |
|---|---|
| Local branch | `matdog/rf-calibrator-from-lf-v25` |
| **On remote?** | **No — local-only.** `git ls-remote` returns nothing for it |
| Upstream tracking | none |
| Worktree | `~/MATDOG/worktrees/norma-core-rf-calibrator` |
| Branch head | `b2f7dac2eab7147917fccdfde702360da82ab7de` |
| Commits ahead of `origin/main` | 0 — the branch head is an ancestor of main |
| **Uncommitted changes** | **2 files, +1 176 / −14 lines** |
| Archived | 2026-08-27 |

Because the branch head carries no unique commits, **the uncommitted working-tree diff is the
entire unique content.**

## Contents

```text
rf_calibrator_uncommitted.patch   git diff of the working tree (1 414 patch lines)
working_copy/matdog.rs            full modified file, byte-for-byte
working_copy/matdog_test.rs       full modified file, byte-for-byte
unique_commits.txt                empty — no unique commits exist
SHA256SUMS                        integrity manifest
```

Modified files:

```text
software/drivers/st3215/src/auto_calibrate/matdog.rs        +1112 −14
software/drivers/st3215/src/auto_calibrate/matdog_test.rs     +78
```

## Restoration

```bash
# from a norma-core checkout at b2f7dac
git apply rf_calibrator_uncommitted.patch
```

Or copy `working_copy/*.rs` directly over
`software/drivers/st3215/src/auto_calibrate/`.

## Status and classification

This is **abandoned exploratory work** on adapting the LF V25 calibrator to the right-front leg,
under the Station-mediated architecture. It was:

- never committed;
- never pushed;
- never reviewed;
- never run against hardware;
- superseded twice over — first by the generic V25 engine attempt, then by the ESP32-S3
  architecture change.

The related RF history on the remote was already archived and closed (`norma-core` PRs #18–#28,
all CLOSED or superseded).

It is preserved for provenance and because discarding unreviewed engineering work without a record
is not acceptable — **not** because it is a development base.

## Related

- [NormaCore MATDOG archive index](../README.md)
- [LF V25 historical hardware oracle](../LF_V25_Hardware_Oracle/README.md)
- [Generic V25 superseded WIP](../Generic_V25_Superseded_WIP/README.md)
