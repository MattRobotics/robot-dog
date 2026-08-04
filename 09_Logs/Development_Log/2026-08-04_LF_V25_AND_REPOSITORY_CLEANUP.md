# MATDOG repository cleanup and LF V25 closeout

**Date:** 2026-08-04

## Canonical state

The LF calibration milestone is complete. Current sources of truth:

```text
robot project: MattRobotics/robot-dog main
Station integration: MattRobotics/norma-core main after PR #11
frozen calibrator: MattRobotics/norma-core release/matdog-lf-calibrator-v25
canonical LF record: 06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
```

## Pull-request audit

### robot-dog

- PR #1 merged: initial project foundation.
- PR #4 merged: clean-room Milestone I foundation.
- PRs #2, #3 and #5–#9 are closed, unmerged historical/experimental checkpoints.
- No open issues.
- No open pull requests before this closeout PR.

### norma-core

- PRs #1–#3 merged: native MATDOG foundation and 24-profile generalization.
- PRs #4–#10 closed, unmerged experimental checkpoints superseded by V25.
- PR #11 is the only current implementation candidate and contains the validated LF V25 flow.
- No open issues.

## Branch audit

Historical calibration branches are retained only until remote branch deletion can be performed with a GitHub ref-deletion capable client. They are not valid development bases.

### robot-dog branches superseded by this closeout

```text
matdog/lf-lower-m11-min-v28r-checkpoint
matdog/lf-lower-m11-max-preparation
matdog/full-calibration-program-v38
matdog/lh-full-calibration-program-v39
matdog/all-legs-full-calibration-program-v40
```

### norma-core branches superseded by V25

```text
matdog/lf-lower-m11-min-v28r-alignment
matdog/full-calibration-v38
matdog/lh-full-calibration-v39
matdog/all-legs-full-calibration-v40
matdog/lf-efficient-calibration-v41
matdog/lf-q0-reconstruction-ci
```

### branches to retain

```text
robot-dog/main
norma-core/main
norma-core/release/matdog-lf-calibrator-v25
```

The development branch `norma-core/matdog/lf-q0-contract-v42` may be deleted after PR #11 is merged and the release branch is confirmed.

## Workflow policy

The large number visible in GitHub Actions is the run counter/history, not a thousand active workflow files.

The V25 branch contains four deliberate MATDOG workflows:

```text
MATDOG Native Calibrator Offline Check
MATDOG Native Observer Check
MATDOG V42 Pinned Station Artifact
MATDOG LF Measurement and Freeze Artifact
```

They are retained through the PR #11 merge because each verifies a separate boundary:

- source/architecture and complete test/build gate;
- external observer non-authority contract;
- exact pinned Station identity;
- reproducible Station/provisioner artifact packaging.

Historical workflow runs are evidence and are not repository files. Deleting run history is optional account maintenance and does not clean source code. Future work should avoid creating one-off workflow files; temporary materializers must be deleted before merge, as done for V25.

## Development rule after closeout

New RF/RH/LH work must:

1. branch from merged `norma-core/main`;
2. preserve the V25 LF tests and evidence unchanged;
3. generalize through data-driven leg profiles rather than copy/paste workflows;
4. use one durable CI suite instead of version-numbered temporary workflows;
5. write canonical results to `robot-dog`, not raw hardware logs to `norma-core`;
6. never rewrite `release/matdog-lf-calibrator-v25`.
