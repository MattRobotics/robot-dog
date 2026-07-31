# MATDOG cross-repository baseline

Status: `CURRENT_CANONICAL`.  Freeze date: 2026-07-30.

## Exact pins

| Role | Repository | Immutable operational reference |
|---|---|---|
| MATDOG source and destination | `MattRobotics/robot-dog` | base `a6dc1184f56956dad696b3bcc24d74f375edb5b7` |
| NormaCore MATDOG fork main | `MattRobotics/norma-core` | `32e3222c87016b7f5d7c1c1da497a4cea3e7b80a` |
| NormaCore experiment | `MattRobotics/norma-core` PR #4 | head `b06cc2bf2e36fb5bbaae12e48c5998c7668862ef`, base `main`, open draft |
| XGoLite architecture reference | `MattRobotics/xgolite-low-level-reconstruction` | annotated tag `xgolite-static-closure-h2-2026-07-30`, commit `a1b34a8594e5bc76c76b1e3ddf89a3aef2b98298` |
| External NormaCore upstream | `norma-core/norma-core` | `NORMACORE_UPSTREAM_REFERENCE`; no operational gate |

The local NormaCore checkout at the fork-main SHA is a valid inspection
snapshot even if its `origin` points to the external upstream. No remote or
file in that checkout is changed by Milestone I.

## Repository boundaries

Robot-dog supplies the physical model, frame convention, joint/servo mapping,
directions, digital-zero record, historical failure record, and two measured
M12 contacts. NormaCore fork main supplies the Station/ST3215 ownership and
MATDOG native contact-profile implementation. PR #4 supplies experimental
restart-safe and distance-aware behavior only. XGoLite supplies architecture
patterns only.

The exact file-level pins, hashes, authority, scope, temporal state, parse
state, and interpretation state are in
`06_Software/Matdog_Core/milestone_i/registries/source_manifest.csv`.
Accepted source/claim identities and repository pins are frozen separately in
`foundation_expectations.json`, so loss of a row or a formally valid
substitution cannot pass on count alone.

## Inventory classification highlights

- The REV00 URDF, joint calibration YAML, digital-zero evidence and current
  M12 checkpoints are current/canonical or supporting evidence.
- The electronics mapping YAML remains supporting for ID topology but is
  superseded by the calibration YAML for direction/zero fields.
- The old C5 first-stand executor, signed-wrap targets, and post-failure
  mechanical-realignment instruction are historical/superseded after digital
  recenter. They are not command eligible.
- The old geometry YAML is supporting historical context; the REV00 URDF and
  newer kinematic contracts govern this foundation.
- The historical M11 direction blob is hash-pinned, non-parseable as YAML and
  consumed only as human-readable text at the registered line locator. Its
  bytes were not repaired or normalized.
- XGoLite's 55-claim closure (37 verified, 10 corroborated, 8 unknown) and its
  24 still-open proof items describe that repository, not MATDOG confidence.

The 24-profile expectation is generated from the constants and formulas of
the `N-MATDOG` source hash at the pinned NormaCore main commit. It is not a
second manually copied 24-row table.

## Remote-state observation

At preflight, fork PR #4 was `OPEN`, `DRAFT`, `MERGEABLE` with passing checks.
This state is an audit observation, not a dependency on future GitHub state.
The source registry pins the head SHA so later movement is detectable.
