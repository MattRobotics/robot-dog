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

## Canonical records

```text
README.md
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
09_Logs/Development_Log/2026-08-04_LF_V25_AND_REPOSITORY_CLEANUP.md
```

## Repository hygiene policy

- `robot-dog` retains only `main` as an active remote branch after cleanup.
- `norma-core` retains only `main` and `release/matdog-lf-calibrator-v25` after cleanup.
- Closed pull requests preserve the historical audit trail.
- Failed, cancelled, incomplete and superseded workflow runs may be deleted.
- Only successful V25 evidence and durable current CI are retained.
- Private external research material is excluded from the public MATDOG baseline.
