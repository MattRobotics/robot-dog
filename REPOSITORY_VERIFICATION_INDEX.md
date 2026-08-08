# MATDOG repository verification index

**Scope:** canonical remote state after LF V25 closeout, Geometry Compiler Phase 1 completion/merge, repository cleanup and the 2026-08-08 search-envelope clarification.

## Current sources of truth

```text
MattRobotics/robot-dog
  active canonical branch: main
  canonical main SHA after Phase 1 merge:
  66f613034f3a64debc9bc45031eb4283a375e52f
  role: public MATDOG project source of truth

MattRobotics/norma-core
  active development branch: main
  canonical main SHA:
  f47b1ba579c623139058a8b0118648015739ab10
  immutable validated LF V25 release branch:
  release/matdog-lf-calibrator-v25
  reviewed V25 source head:
  f87dd1fbc7e8100d275c74f9af448642f3429680
```

GitHub remote and the real ASUS filesystem override stale chat summaries or historical handoff assumptions.

## Validation status

```text
LF V25: hardware validated, persistent profile saved, EEPROM frozen
Geometry Compiler Phase 1: COMPLETE + MERGED
Geometry profile: schema v3, 24/24 endpoints
Final Geometry Compiler test suite: 81/81 PASS
RF: not yet hardware validated
RH: not yet hardware validated
LH: not yet hardware validated
complete all-leg persistent calibration: not yet validated
Phase 2 generic V25-derived engine: NOT STARTED
Phase 3 RF/RH/LH hardware completion: NOT STARTED
```

No V28–V42 or historical “all legs” experimental implementation is a current program.

## Canonical three-phase calibration architecture

```text
1. offline Geometry Compiler / contact + path + parking + model-gap analysis
2. one generic V25-derived full-leg engine in norma-core
3. RF -> RH -> LH hardware completion
```

Phase 1 is closed as:

```text
PASS_GEOMETRY_COMPILER_COMPLETE_WITH_EXPLICIT_MODEL_GAPS
```

Merged through:

```text
MattRobotics/robot-dog PR #14
squash merge SHA:
66f613034f3a64debc9bc45031eb4283a375e52f
```

The generic Phase 2 engine has not started.

## Search-envelope clarification — 2026-08-08

The Geometry Compiler does not clamp FK to URDF limits. It evaluates poses with:

```text
enforce_limits=False
```

Phase 1 contact search nevertheless used an explicitly bounded margin:

```text
10 degrees beyond each URDF-declared endpoint
```

Therefore `NO_MODELED_ENDSTOP` has the strict meaning:

```text
no relevant same-leg mesh contact found inside the documented Phase 1 analysis envelope
```

It does **not** mean:

```text
the STL geometry never contacts at any larger angle
```

This is important because some original URDF limits were historical model inputs rather than independently proven physical endstop coordinates.

The v3 compiler demonstrably searched beyond the declared limits, including findings such as:

```text
LF hip MIN:   URDF -45.000 deg -> mesh -47.500 deg
LF lower MIN: URDF -92.000 deg -> mesh -97.957 deg
RF hip MAX:   URDF +45.000 deg -> mesh +47.500 deg
RF lower MIN: URDF -92.000 deg -> mesh -98.004 deg
```

LF 6/6 `MODEL_INCOMPLETE` remains valid because LF has direct V25 hardware contact angles and the mesh does not represent the same physical stopping event.

For RF/RH/LH, where hardware endpoint oracles do not yet exist, `NO_MODELED_ENDSTOP` must always be read with the bounded-envelope qualifier.

A targeted extended no-contact mesh sweep, using a physically justified wider angular domain independent from the old URDF endpoint as outer search anchor, is now the recommended **pre-Phase-2 geometry sanity check**.

Full clarification:

```text
06_Software/Matdog_Core/calibration/
MATDOG_GEOMETRY_COMPILER_ENVELOPE_CLARIFICATION_2026-08-08.md
```

## Corrected q=0 interpretation

Manual/visual home is only a commissioning seed and plausibility check.

Future generalization must keep separate:

```text
encoder/transmission scale
q0 offset
urdf_declared_limit
mesh-predicted collision/contact
hardware-measured contact
geometry/model mismatch
```

The historical LF V25 affine gate and operational freeze remain valid evidence for LF, but future legs must not use an affine scale fitted to map hardware contacts onto old URDF MIN/MAX values as circular proof that those URDF limits were physically exact.

## Parking / prerequisite interpretation

The 2026-07-20 geometry checkpoint remains validated evidence for one collision-free plan.

Historical values:

```text
HIP prerequisite: upper about +50 deg
LOWER prerequisite: upper about +90 deg
LF: park LH upper +30 deg
RF: park RH upper +30 deg
```

are validated seeds/reference poses, not permanent constants.

Phase 1 policy is:

```text
default = NO AUXILIARY PARKING
```

with parking derived from path collision analysis.

## Canonical records

```text
README.md
REPOSITORY_VERIFICATION_INDEX.md
06_Software/Matdog_Core/calibration/MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md
06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_PHASE1_COMPLETION_2026-08-07.md
06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_ENVELOPE_CLARIFICATION_2026-08-08.md
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
06_Software/Matdog_Core/calibration/MATDOG_MECHANICAL_ENDSTOP_GEOMETRY_CHECKPOINT_2026-07-20.md
09_Logs/Development_Log/2026-08-04_LF_V25_AND_REPOSITORY_CLEANUP.md
09_Logs/Validation_Reports/Geometry_Compiler/README.md
09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_PROFILE.json
09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_REPORT.md
```

## Current next step

Before writing the final Phase 2 generic hardware engine:

1. verify live `robot-dog/main` and `norma-core/main`;
2. perform the extended no-contact mesh sanity check described above;
3. preserve LF V25 as hardware oracle;
4. inspect the historical RF worktree only as evidence/prototype;
5. design one generic V25-derived engine in `norma-core`;
6. complete offline review before any new physical motion.

Historical RF worktree to preserve:

```text
/home/matteo-manicardi/MATDOG/worktrees/norma-core-rf-calibrator
branch: matdog/rf-calibrator-from-lf-v25
```

## Repository hygiene policy

- `robot-dog/main` is the canonical public state; short-lived focused review branches may exist temporarily.
- `norma-core` retains `main`, `release/matdog-lf-calibrator-v25`, and only explicitly justified active review/development branches.
- Closed pull requests preserve the historical audit trail.
- Historical validation reports are marked superseded rather than silently deleted.
- No force-push or destructive cleanup of evidence.
- No merge into `main` without explicit human authorization.
- Private external research material is excluded from the public MATDOG baseline.
