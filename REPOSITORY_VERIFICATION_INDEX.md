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
06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_PHASE1B_ADDENDUM_2026-08-08.md
06_Software/Matdog_Core/calibration/MATDOG_CALIBRATION_CANONICAL_HANDOFF_PHASE1B_CLOSED_2026-08-09.md
09_Logs/Development_Log/2026-08-04_LF_V25_AND_REPOSITORY_CLEANUP.md
09_Logs/Validation_Reports/Geometry_Compiler/README.md
```

Historical, retained, superseded for endpoint metrology:

```text
09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_PROFILE.json
09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_REPORT.md
```

## Geometry Compiler Phase 1 — closed 2026-08-07, endpoint metrology SUPERSEDED

`PASS_GEOMETRY_COMPILER_COMPLETE_WITH_EXPLICIT_MODEL_GAPS`, offline only,
24/24 endpoints processed, schema v3. See
`06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_PHASE1_COMPLETION_2026-08-07.md`
for the full record.

Phase 1's **endpoint metrology is superseded by Phase 1B** (below). The v3
artifacts are retained unchanged as historical evidence and must not be
deleted; they remain an accurate description of what the v3 policy could
observe. They are no longer the source of truth for joint endpoints.

## Geometry Compiler Phase 1B — adjacent revolute endstop metrology, 2026-08-08

Phase 1 reported LF 6/6 `MODEL_INCOMPLETE`. The cause was not missing STL
hardstop geometry. Two defects compounded:

```text
1. policy: `adjacent pair -> EXCLUDE` treated REVOLUTE hinges and FIXED
   structural attachments identically, making the real hardstop -- which
   lives ON the revolute parent/child pair -- unobservable by construction
2. mesh: the assembly STLs modelled the motor centre pins in nominal contact
   with the adjacent link, so adjacent pairs read INTERSECTING at every angle
```

Corrected policy, derived from URDF topology rather than a hard-coded list:

```text
parent-child REVOLUTE -> INCLUDE in collision analysis        (12 pairs)
parent-child FIXED    -> structural attachment -> EXCLUDE      (4 pairs)

ENDSTOP METROLOGY = active revolute parent-child pair
PATH SAFETY       = all other relevant collision pairs
```

Five collision meshes corrected for the motor-pin representation
(`base_link.stl`, `lf/rf/rh/lh_upper_leg_link.stl`). Canonical filenames,
same local frame/scale/coordinates; **`rev00` unchanged and the URDF
byte-identical**. All other STLs untouched.

Diagnostic evidence archived (not in this repository):

```text
/home/matteo-manicardi/MATDOG/_archive/geometry-diagnostics/GATE_A_ADJACENT_BASELINE_2026-08-08
/home/matteo-manicardi/MATDOG/_archive/geometry-diagnostics/GATE_B_MOTORPIN_SUPPORTED_2026-08-08
```

Full record:
`06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_PHASE1B_ADDENDUM_2026-08-08.md`

### Closure state

```text
Phase 1B: CLOSED
  PR #16 merged (squash) 2026-08-09 -> main 5b66044e225fcd921e44b98cc710f028da441a64
  PR #15 CLOSED, NEVER MERGED (superseded; head 751fe1eff44a2d97714438a040f04f4a8050ea04)
  remote branches after cleanup: main only
Phase 2: NEXT, NOT STARTED
```

PR #15 predates GATE A/GATE B. Its schema-v3 bounded-envelope clarification remains
valid **historically for v3** and is preserved in the closed PR. Its operational
recommendation (Phase 1 -> extended `NO_MODELED_ENDSTOP` audit -> Phase 2) is
**superseded and must not be used as the current plan**: schema v4 resolves 24/24
endpoints with 0 `NO_MODELED_ENDSTOP`, so that audit is no longer the pre-Phase-2
blocker.

Step 2 of the three-phase plan (generic V25-derived full-leg engine in norma-core)
has **not** started. The canonical Phase 2 entry conditions are recorded in
`06_Software/Matdog_Core/calibration/MATDOG_CALIBRATION_CANONICAL_HANDOFF_PHASE1B_CLOSED_2026-08-09.md`.

## Repository hygiene policy

- `robot-dog` retains only `main` as an active remote branch after cleanup, except short-lived reviewed development/documentation branches required by branch protection.
- `norma-core` retains `main`, `release/matdog-lf-calibrator-v25`, and only the single active next-milestone/review branch when needed.
- Closed pull requests preserve the historical audit trail.
- Failed, cancelled, incomplete and superseded workflow runs may be deleted.
- Only successful V25 evidence and durable current CI are retained.
- Private external research material is excluded from the public MATDOG baseline.
