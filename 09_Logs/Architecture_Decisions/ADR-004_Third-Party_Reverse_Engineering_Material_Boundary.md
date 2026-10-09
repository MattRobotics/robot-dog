# ADR-004 — Third-party reverse-engineering material stays out of MATDOG history

Date: 2026-10-08

## Status

Accepted (owner decision for PR-2, 2026-10-08; applied unchanged to PR-3). Does not supersede ADR-001 to ADR-003.

## Context

MATDOG research on the XGO Lite quadruped produced static-analysis material: decompiler text
extracted from a third-party firmware image and values derived from it. The research has its own
repository, `MattRobotics/xgolite-low-level-reconstruction`. That repository states that
vendor code, firmware packages and third-party material whose redistribution is not appropriate
are not to be committed, and that clean-room specifications stay separate from vendor code.

Neither repository contains a licence or an authorization that covers redistributing the
firmware-derived material. The `LICENSE` of `robot-dog` (MIT, Matt Robotics 2026) covers MATDOG's
own work only. That material was nevertheless committed to branches of `robot-dog` on
2026-09-29 and 2026-10-05, so it is already in the history of several public branches. It is not
on `main`.

## Decision

1. `main` does not receive: the 38 `xgo_static_extracts/` text files, the XGO trace, table,
   catalogue, revision-comparison and search-inventory JSON files, any other artifact derived from
   the firmware that has not been reviewed, or copies that reproduce them.
2. Provenance of excluded material is recorded by reference (repository, commit, path, SHA-256),
   never by content. The manifest `motion_integration_manifest_pr2.json` lists them as `DEFERRED`.
3. Integration from the Integration dev.1 line (PR-2) and from the dev.2/dev.3 line (PR-3) is
   selective. In PR-2, history up to the boundary commit `13da04a` is merged with its original
   SHAs; later commits are recreated as new commits. In PR-3 all seven commits are recreated. A
   recreated commit carries the original SHA in its message and leaves the third-party material
   out.
4. No automatic merge, cherry-pick or import from the reverse-engineering repository into
   `robot-dog`. MATDOG-original results enter only after a provenance and independence review.
5. A MATDOG component that really needs an excluded artifact is `DEFERRED` or `BLOCKED`. Tests are
   not weakened to hide this.

## Consequences

- The pure motion library (`src/motion`) is on `main`, unwired. Its execution suites, oracles and
  the G35/G4/G4.1/G5-A evidence are not: they are `DEFERRED`, not deleted. They remain on the
  preserved branches.
- Deleting a file from `main` later would not remove it from Git history; this is why the
  boundary is enforced at integration time.
- The material that is already public on other branches is a separate exposure. Handling it
  (including any history change) is an owner decision outside this ADR and is not performed here.
- Re-admitting deferred material needs verifiable authorization or a clean-room regeneration, and
  its own review.

## Application in PR-3

The seven dev.2/dev.3 commits descend from `3f23439`, so they are recreated as new commits whose
parent is `main` (full `Original-SHA:` in each message). Each delta was applied as a patch
(`git cherry-pick --no-commit`) and committed anew; no original commit becomes an ancestor of
`main`. Their deltas contain no excluded artifact. The PR-2 manifest, the DEFERRED declaration and the source/purity/wiring gate
are unchanged.

## Verification

The PR-2 gate `run_motion_convergence_tests.py` fails if any `DEFERRED` path is present in the
tree. The Git object closure of PR-2 was checked against the blob hashes of the excluded files
(0 reachable).

## Status update (2026-10-09)

Annotation only; the decision above is unchanged and the boundary is not weakened. The historical GitHub
branches that held the excluded material have been deleted. While Draft PR #42 is open its working branch
`docs/c2-final-sync` also exists; the target is `main` alone after it is merged and that branch is deleted. `main`, the 9 tags
and the 41 pull-request refs reach none of the 284 known excluded blobs. Deleting the branches does not
guarantee removal of material that was public before: it may persist in GitHub storage, caches, forks and
clones. The original commits are kept in a non-public archive (owner-reported) and must not be pushed to a
public ref. Details: [`EXTERNAL_ARCHIVES.md`](../EXTERNAL_ARCHIVES.md). Any further handling remains an
owner decision.
