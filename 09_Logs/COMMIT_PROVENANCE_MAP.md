# Commit provenance map — PR #37 to PR #40

Maps the commits cited in MATDOG documentation to what exists on `main`. Short or full SHAs in
older documents may name commits that live only on historical branches. A SHA that is **not an
ancestor of `main`** is kept here as provenance only: a permalink to it may become unavailable once the
refs (branches or tags) that make the commit reachable are removed, so this repository does not rely on
such a permalink
(where such commits are preserved: [`EXTERNAL_ARCHIVES.md`](EXTERNAL_ARCHIVES.md)).

Baseline of this map: `main` = `9c2fe7868423f5267063285d7be93de00dc302db` (PR #40 merge). `main` is now
`ddcd1cdb87eef29cc46996d6137d3f2b4c946f17` (PR #41 merge, documentation only).

> **2026-10-09:** every non-`main` GitHub branch named below has been **deleted**. The SHAs remain valid
> identifiers of the original commits; see [`EXTERNAL_ARCHIVES.md`](EXTERNAL_ARCHIVES.md) §2 for the last
> tips and §6 for recovery. "Where it lives" columns describe where the commit was held.
Verified with `git merge-base --is-ancestor` against `origin/main`.

## Integration PRs on `main`

| PR | Merge commit on `main` | Subject | Log |
|---|---|---|---|
| #37 | `a09cb76b428f46f0c6a77c049ab906da4dc4fed3` | Flash layout safety documentation (docs only) | [`2026-10-08_FLASH_LAYOUT_DIVERGENCE_NOTICE`](Development_Log/2026-10-08_FLASH_LAYOUT_DIVERGENCE_NOTICE.md) |
| #38 | `fb02b8ed11ece11052f589d9222049f167336891` | Calibration Persistence V1 and flash layout V1 (PR-1) | [`2026-10-08_PR1_…`](Development_Log/2026-10-08_PR1_PERSISTENCE_LAYOUT_V1_INTEGRATION.md) |
| #39 | `e622a80d539c75a0cde4747419822fbdca7e34b5` | Selective dev.1 integration, no XGO extracts (PR-2) | [`2026-10-08_PR2_…`](Development_Log/2026-10-08_PR2_SELECTIVE_DEV1_INTEGRATION.md) |
| #40 | `9c2fe7868423f5267063285d7be93de00dc302db` | Selective dev.2/dev.3 source convergence (PR-3) | [`2026-10-08_PR3_…`](Development_Log/2026-10-08_PR3_DEV2_DEV3_SELECTIVE_INTEGRATION.md) |
| #41 | `ddcd1cdb87eef29cc46996d6137d3f2b4c946f17` | Documentation and history consolidation (C1); Markdown only | [`HISTORY_INDEX.md`](HISTORY_INDEX.md) |

These four merges are **integration of source into `main`**. They are not a release approval; see
[`CHANGELOG.md`](../05_Firmware/MATDOG_Controller/CHANGELOG.md) ("Unreleased").

## PR #37 (docs) — original SHAs preserved

| Commit | Subject |
|---|---|
| `48506abd90383fffb1b6c63396d385fefbb30a08` | docs(firmware): add flash layout safety notice; main is not a flashable baseline |
| `2c3e8e9f49e33042acaf26b6e971c1e2077206b0` | docs: refine PR #37 |

## PR #38 / PR-1 — original SHAs preserved (`--no-ff` merge)

The persistence line was merged, not recreated. The 13 commits below are real ancestors of `main`
through merge `c358a3e5a44a7f79f3b815a3492368917a9860aa` (parents: previous `main` and `7b258b3`).

`4a46b872…` record/codec/A-B NVS store · `319e1605…` block store writes after uncertain SAVE ·
`44b4bcbc…` flash layout V1 with 5 MiB slots · `9d12431a…` dedicated `matdog_nvs` and SAVE marker ·
`ff0543c0…` durable caller acknowledgment · `90a2f62b…` integrate persistence (boot LOAD, STATUS,
SAVE, ACK, reconcile) · `e82cdab9…` attest current Q0 promotion · `be0c1297…` reject overflowing
USB lines · `ebb60783…` prepare M0 migration · `174aa041…` disable migration write retries ·
`27576044…` align assembled hardware · `b19a98e0…` qualify M0.4 · `7b258b36c257bd455f135aee2667d035c4929544`
close P1 with verified encrypted backup. Documentation commits on top: `471ebd3`, `7d15bdf`, `18e5be9`.

## PR #39 / PR-2 — mixed: one merged boundary, three recreated commits

**Merged with original SHAs** (merge `8093158c1cb07c71f6a363d6d4df311308aa46a7`, parents `fb02b8e`
and `13da04a290fc4e06d05636b5fd553476ed2d51e5`): 13 commits `125d981b…` (post-abort recovery,
thermal acquisition), `d4edd813…`, `036f339b…`, `466a58bf…`, `f24adfe3…`, `1a5e0085…` (RF startup
recovery), `1bf46222…`, `96656917…` (Wi-Fi provisioning and TLS OTA), `9489ebb4…`, `aee49bf3…`,
`7d120168…`, `751d43c9…` (charging presentation), `13da04a290fc4e06d05636b5fd553476ed2d51e5`
(pure motion library G1–G5-A).

**Recreated** (original is **not** an ancestor of `main`; new commit carries `Original-SHA:`):

| Original SHA (provenance only) | Recreated commit on `main` | Content |
|---|---|---|
| `1cec973b8137a2f5789f50ec087787df4113b09f` | `ffa1a9a143f1a4e72155ee4f4b6db09caff65a72` | dev.1 identity and offline build isolation |
| `53aa9620ec0cdd38d69edfa934d68a934962dd68` | `026e66fc48e374a7bf65e5e4c05f8210c12af901` | safety-gate extension; PR-2 manifest |
| `d7aa369631202aed70458d0c7ff2980534882058` | `44d72eeb44c34214ebcfccf2cb6d4b55051f2931` | persistence schema exposed through the owner boundary |

`3f23439` and every descendant (including `d7aa369`) were not merged: see
[ADR-004](Architecture_Decisions/ADR-004_Third-Party_Reverse_Engineering_Material_Boundary.md).

## PR #40 / PR-3 — seven commits recreated

Base `main` `e622a80`. Original is **not** an ancestor of `main`.

| Original SHA (provenance only) | Recreated commit on `main` | Content |
|---|---|---|
| `bac652fe37705ecb5ecdd0521dc64db95926d47e` | `be94f78978da52005cb7c13109216bf39be768bc` | dev.2 read-only boot servo census |
| `d0ede365dbbdbf3940a31929e8e2e132faf9d49f` | `0f551209a88e6be2888d38c7c82ef56adec67f51` | dev.2 delta audit, log, changelog |
| `d67eaa1033ef2ea4c14fbefa97c9e46dfafe165b` | `84fcf94623268e78d74ab453221ac49cac6f227e` | dev.2 qualification and boot self-test record |
| `62812cd66816b4b5506337bf7f3245ecce13a2ad` | `1c9a1449e5c415652ec5d8b06ab10a50611deed6` | dev.3 thermal verdict from direct reads; `0.2.0-dev.3` |
| `2f2324784d66bdc17e5a5c7cf39ca4c3dfc9a223` | `5574499c0ac793bdf428a1b3f7d89b838e2037ef` | runner starts post-abort recovery once |
| `b3fd945bdaf37d192d97b605ac0f59b67f1dba45` | `e028dd813545c895552df8fca4045f41d4fa0768` | dev.3 delta audit, log, changelog; **installed-firmware provenance** |
| `b764c25c9530350331dbaa59ea2d84ddab3a3f69` | `1d59dbee090efe55eddca579778d7f6b29b0f83c` | dev.3 qualification, flash, 24/24 and persistence record |

Documentation follow-ups on `main`: `1149810bb62d87aae651ecdec16c5584a084e50a`,
`49e76fbe68dce0a15ff9c1d759a75bb0571f7b2e`, `38cf519fed6af7c21168b053e18b7afb954a18c5`.

`b3fd945…` is the commit the **installed** firmware was built from. A build of `main` has another
source commit and build ID, and is not shown to be byte-identical to the installed image.

## Other SHAs cited in documentation that are not on `main`

| SHA | Meaning | Where it lives | Status |
|---|---|---|---|
| `e1704719979789cd9c9f18741e4546725558797e` | Tip of the divergent gait / G5-A line; holds `09_Logs/Development_Log/2026-09-28_G3_STARTUP_TIMED_STAND.md` (blob `513acfd998440e7d684f31d26257efefbd471c9f`) | formerly branch `feat/g5a-stabilization-feasibility` (deleted) | Preserved in the non-public archive (owner-reported). [`STARTUP_TIMING.md`](../05_Firmware/MATDOG_Controller/src/motion/STARTUP_TIMING.md) and the PR-2 log link to it by permalink. With the branch deleted, the permalink should be treated as unavailable; recovery only through the non-public archive |
| `3f23439` | Introduced the 38 XGO decompiler extracts | formerly `integration/matdog-controller-v0.2.0` (deleted) | Excluded by ADR-004. Not to be republished |
| `d7aa369…` | dev.1 tip | formerly the integration and preservation branches (deleted) | see PR-2 table |
| `a06314e` | Wi-Fi PHY diagnostic | formerly `backup/wifi-phy-a06314e-20261008` (deleted) | Not integrated |
| `43fa92a` | WIP M0.4 P2 | formerly `backup/wip-m0-4-p2-20261008` (deleted) | Not integrated |
| `b6d99573755f85993c189227b6a3f890fb15fe20` | Convergence audit handoff tip | formerly `audit/matdog-v0.2.0-dev1-claude-handoff` (deleted) | Contains XGO/G35 material and 87.6 MB of evidence. Not imported |
| `15f3fb8f…` | Full-leg calibrator V1 H0 (tag `archive/2026-08-29/full-leg-calibrator-v1-h0`) | tag, not on `main` | `EXTERNAL_ARCHIVE_PENDING_VERIFICATION` |

Many other hexadecimal strings in older documents are SHA-256 prefixes of files or commits from the
operator's machine or from other repositories; they are not commits of this repository and are not
listed here.
