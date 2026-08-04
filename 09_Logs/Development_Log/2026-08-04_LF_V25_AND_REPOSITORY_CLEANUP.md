# MATDOG repository cleanup and LF V25 closeout

**Date:** 2026-08-04  
**Status:** post-merge closeout record

This log records the repository state after completion of the LF V25 milestone. The canonical reviewer entry point is:

```text
REPOSITORY_VERIFICATION_INDEX.md
```

## Canonical state

```text
robot project: MattRobotics/robot-dog main
Station integration: MattRobotics/norma-core main after PRs #11 and #12
frozen calibrator: MattRobotics/norma-core release/matdog-lf-calibrator-v25
canonical LF record: 06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
```

## Pull-request audit

### robot-dog

- PR #1 merged: initial project foundation.
- PR #4 merged: clean-room Milestone I foundation.
- PR #10 merged: LF V25 closeout, final result and current README.
- PRs #2, #3 and #5–#9 are closed, unmerged historical checkpoints.
- Historical PRs are audit records, not development bases.

### norma-core

- PRs #1–#3 merged: native MATDOG foundation and profile generalization.
- PRs #4–#10 closed, unmerged experimental checkpoints superseded by V25.
- PR #11 merged: validated LF V25 implementation.
- PR #12 merged: release-only workflow cleanup after the V25 merge.
- The exact reviewed implementation remains frozen on `release/matdog-lf-calibrator-v25`.

## Branch audit

Branches to retain:

```text
robot-dog/main
norma-core/main
norma-core/release/matdog-lf-calibrator-v25
```

Obsolete robot-dog branches:

```text
docs/matdog-lf-v25-closeout
matdog/lf-lower-m11-min-v28r-checkpoint
matdog/lf-lower-m11-max-preparation
matdog/full-calibration-program-v38
matdog/lh-full-calibration-program-v39
matdog/all-legs-full-calibration-program-v40
```

Obsolete norma-core branches:

```text
matdog/lf-lower-m11-min-v28r-alignment
matdog/full-calibration-v38
matdog/lh-full-calibration-v39
matdog/all-legs-full-calibration-v40
matdog/lf-efficient-calibration-v41
matdog/lf-q0-reconstruction-ci
matdog/lf-q0-contract-v42
```

Those refs must not be used for future development. Their closed pull requests preserve the audit trail after branch deletion.

## Workflow policy

The large number visible in GitHub Actions is principally run history, not a corresponding number of active workflow files.

Canonical `main` branches should contain only reusable CI. One-shot and version-numbered workflow files are historical implementation aids and must not be copied into RF/RH/LH development.

Final successful runs referenced by the V25 evidence should be retained. Failed, cancelled and duplicate historical runs may be removed from the GitHub Actions interface without changing source history.

The immutable release branch may retain the exact release-only workflow definitions used for the reviewed V25 artifact. It must not be rewritten.

## Development rule after closeout

New RF/RH/LH work must:

1. branch from merged `norma-core/main`;
2. preserve the V25 LF tests and evidence unchanged;
3. generalize through data-driven leg profiles rather than copied per-leg programs;
4. use one durable CI suite instead of version-numbered temporary workflows;
5. write canonical project results to `robot-dog`;
6. never rewrite `release/matdog-lf-calibrator-v25`.

## Post-merge conclusion

LF V25 is complete and frozen. The next implementation milestone is RF generalization from the merged NormaCore main branch. Repository reviewers should use `README.md`, `REPOSITORY_VERIFICATION_INDEX.md` and the canonical LF final record as the current sources of truth.
