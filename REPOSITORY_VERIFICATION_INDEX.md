# MATDOG repository verification index

**Scope:** canonical remote state after LF V25 closeout and repository cleanup.

## Current sources of truth

```text
MattRobotics/robot-dog
  active branch: main
  role: public MATDOG project source of truth

MattRobotics/norma-core
  active development branch: main
  immutable validated release: release/matdog-lf-calibrator-v25
  reviewed V25 source head: f87dd1fbc7e8100d275c74f9af448642f3429680
```

## Validation status

```text
LF V25: hardware validated, affine profile saved, EEPROM frozen
RF: not yet hardware validated
RH: not yet hardware validated
LH: not yet hardware validated
complete all-leg calibration: not yet validated
```

No V28–V42 or “all legs” experimental implementation is a current program. Future leg work must start from merged `norma-core/main` and generalize the proven V25 architecture through data-driven leg profiles.

## Canonical calibration architecture update — 2026-08-07

The current development contract for the next calibration phase is:

```text
06_Software/Matdog_Core/calibration/
MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md
```

That document supersedes older RF development prescriptions where they conflict with it. In particular it freezes the geometry-first three-phase plan:

```text
1. offline Geometry Compiler / 24 mesh-predicted contacts and safe paths
2. one generic V25-derived full-leg engine in norma-core
3. RF -> RH -> LH hardware completion
```

It also records the corrected q=0 policy: manual/visual home is only a seed; final q=0 must be derived from model geometry plus repeatable hardware contact evidence, staged and verified before any separately authorized EEPROM freeze.

The 2026-07-20 geometry checkpoint remains historical validated evidence for one collision-free path. Its +50°/+90° prerequisites and +30° rear parking are safe seeds, not permanent proof that those auxiliary poses are always necessary or minimal.

## Canonical records

```text
README.md
REPOSITORY_VERIFICATION_INDEX.md
06_Software/Matdog_Core/calibration/MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
06_Software/Matdog_Core/calibration/MATDOG_MECHANICAL_ENDSTOP_GEOMETRY_CHECKPOINT_2026-07-20.md
06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_PHASE1_COMPLETION_2026-08-07.md
09_Logs/Development_Log/2026-08-04_LF_V25_AND_REPOSITORY_CLEANUP.md
09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_PROFILE.json
09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_REPORT.md
```

## Geometry Compiler Phase 1 — closed 2026-08-07

`PASS_GEOMETRY_COMPILER_COMPLETE_WITH_EXPLICIT_MODEL_GAPS`, offline only,
24/24 endpoints processed, schema v3. See
`06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_PHASE1_COMPLETION_2026-08-07.md`
for the full record. This closes step 1 of the three-phase plan in the
canonical calibration architecture update above; step 2 (generic
V25-derived full-leg engine in norma-core) has not started.

## Repository hygiene policy

- `robot-dog` retains only `main` as an active remote branch after cleanup, except short-lived reviewed development/documentation branches required by branch protection.
- `norma-core` retains `main`, `release/matdog-lf-calibrator-v25`, and only the single active next-milestone/review branch when needed.
- Closed pull requests preserve the historical audit trail.
- Failed, cancelled, incomplete and superseded workflow runs may be deleted.
- Only successful V25 evidence and durable current CI are retained.
- Private external research material is excluded from the public MATDOG baseline.
