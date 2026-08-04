# MATDOG repository verification index

**Verified:** 2026-08-04  
**Scope:** remote GitHub state after LF calibrator V25 completion

This file is the canonical entry point for repository verification. It distinguishes current sources from historical experiments and records the exact post-merge state.

## Current milestone

The left-front leg calibration completed successfully with MATDOG LF calibrator V25.

```text
58/58 sequence complete
6/6 LF mechanical contacts accepted
URDF affine gate PASS
supervised hardware-witness gate PASS
RAM q0 staging PASS
Station shutdown and serial release PASS
transactional EEPROM freeze PASS
persistent LF profile PASS
global torque OFF verified
```

Canonical LF result:

```text
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
```

## Canonical repositories and refs

```text
MattRobotics/robot-dog
  canonical branch: main
  role: CAD, URDF, project decisions, calibration evidence and roadmap

MattRobotics/norma-core
  canonical development branch: main
  role: Station/ST3215 integration and native calibration runtime

MattRobotics/norma-core
  immutable release branch: release/matdog-lf-calibrator-v25
  reviewed source head: f87dd1fbc7e8100d275c74f9af448642f3429680
  role: exact hardware-validated LF V25 source
```

## Merged implementation state

```text
robot-dog PR #10
  merged
  purpose: publish LF V25 result and replace stale project status
  merge commit: e09c42e22e51c947ba4814c3ec6af23813355258

norma-core PR #11
  merged
  purpose: validate and freeze MATDOG LF calibrator V25
  merge commit: ad9fdc1e13e8eaaa67193b38a99e4d69dd3a9337

norma-core PR #12
  merged
  purpose: remove release-only workflow files from main
  merge commit: 5c3d4b784a6448843bf8f13da1bf32529006f553
```

## Historical pull requests

The following are preserved for audit but are not valid development bases:

```text
robot-dog PRs #2, #3 and #5–#9
norma-core PRs #4–#10
```

They are closed and unmerged because their experimental content was superseded by the validated V25 architecture.

## Branch policy

Branches to retain:

```text
robot-dog/main
norma-core/main
norma-core/release/matdog-lf-calibrator-v25
```

The following remote branches are obsolete and must be deleted after confirming that their closed pull requests remain accessible:

```text
robot-dog/docs/matdog-lf-v25-closeout
robot-dog/matdog/lf-lower-m11-min-v28r-checkpoint
robot-dog/matdog/lf-lower-m11-max-preparation
robot-dog/matdog/full-calibration-program-v38
robot-dog/matdog/lh-full-calibration-program-v39
robot-dog/matdog/all-legs-full-calibration-program-v40

norma-core/matdog/lf-lower-m11-min-v28r-alignment
norma-core/matdog/full-calibration-v38
norma-core/matdog/lh-full-calibration-v39
norma-core/matdog/all-legs-full-calibration-v40
norma-core/matdog/lf-efficient-calibration-v41
norma-core/matdog/lf-q0-reconstruction-ci
norma-core/matdog/lf-q0-contract-v42
```

No future calibration work may branch from those refs.

## Workflow policy

Only reusable CI belongs on canonical `main` branches. Version-numbered or one-shot artifact workflows must not be copied into later-leg development.

Historical workflow runs are execution records, not source files. Failed, cancelled and duplicate runs may be deleted from the GitHub Actions interface after preserving the final PASS runs referenced by the canonical V25 evidence.

The immutable V25 release branch may retain the exact release-only workflow definitions required to reproduce the reviewed artifact. It must never be rewritten.

## Permanent calibration constraints

- NormaCore Station is the sole ST3215 serial owner during motion.
- ST3215 `GoalPosition` remains unsigned standard `0..4095`.
- Signed-wrap is forbidden.
- Digital-home commissioning remains separate from mechanical leg calibration.
- EEPROM provisioning occurs only after complete measurement PASS, Station shutdown and serial release.
- LF V25 must not be rerun unless LF mechanics, mounting, servo, URDF or calibration state changes.
- Later-leg work must generalize through data-driven leg profiles, not per-motor exceptions or copied versioned workflows.

## Next milestone

```text
branch from merged norma-core/main
→ generalize the validated V25 architecture to RF
→ preserve LF tests and evidence unchanged
→ perform supervised RF six-contact calibration
→ apply RF affine gate and transactional freeze
→ repeat for RH and LH
→ validate the complete twelve-joint persistent profile
```

## Verification checklist

A reviewer should confirm:

- `robot-dog/main` presents LF as calibrated and RF/RH/LH as pending;
- the LF final record matches the frozen NormaCore V25 source;
- NormaCore PRs #11 and #12 remain merged;
- `release/matdog-lf-calibrator-v25` exists and is unchanged;
- no open PR proposes an older V28–V42 calibration design;
- obsolete remote branches are absent;
- canonical main branches contain no one-shot version-numbered workflows;
- final PASS workflow runs and required evidence remain available.
