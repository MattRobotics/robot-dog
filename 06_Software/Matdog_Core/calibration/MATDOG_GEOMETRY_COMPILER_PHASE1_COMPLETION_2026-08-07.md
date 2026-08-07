# MATDOG Geometry Compiler — Phase 1 completion record

**Date:** 2026-08-07
**Status:** `PASS_GEOMETRY_COMPILER_COMPLETE_WITH_EXPLICIT_MODEL_GAPS`
**Base robot-dog SHA:** `344675f4107683133fdb0f1f7171b54e05e6aa4b`
**Offline only:** no Station, no serial, no motor command, no EEPROM write, no norma-core change, no LF V25 hardware re-run.

This closes the "Immediate next task: Phase 1" recorded in
`MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md`. It does not modify
that document.

## 1. What was built

A reusable offline collision kernel (exact transformed AABB -> convex-hull
separating-axis mid-phase -> BVH/spatial-hash-accelerated triangle/triangle
narrow phase) plus a 24-endpoint (4 legs x 3 joints x 2 sides) contact
search, segment-scoped parking planner, manufacturing-tolerance
sensitivity analysis, and a deterministic hash-pinned machine-readable
profile + human-readable report generator:

```text
06_Software/Matdog_Core/calibration/
  matdog_geometry_mesh_kernel.py
  matdog_geometry_scene.py
  matdog_geometry_contact_search.py
  matdog_geometry_path_planner.py
  matdog_geometry_uncertainty.py
  matdog_geometry_profile.py
  matdog_geometry_report.py
  matdog_geometry_compiler.py
  tests/test_matdog_geometry_mesh_kernel.py
  tests/test_matdog_geometry_scene.py
  tests/test_matdog_geometry_checkpoint_regression.py
  tests/test_matdog_geometry_contact_search.py
  tests/test_matdog_geometry_path_planner.py
  tests/test_matdog_geometry_profile.py
  tests/test_matdog_geometry_uncertainty.py
```

## 2. Schema and final profile/report

```text
schema_version:            matdog.calibration_geometry_profile.v3
endpoints processed:       24
final profile:             09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_PROFILE.json
final profile sha256:      6b438285f7145b2cb4f9fc11f1e9b2342dfbb7360d83976ad9a25cd590a0b7c5
final report:               09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_REPORT.md
final report sha256:        de483529941ceee04e921086dd42cd66956a0749bfc39d161115650d619a4b50
content_sha256 (profile):   7d6c0577bde7ee31ae5672560406d65e096ab7de449d9436f3d43a8ce75172bb
URDF sha256:                 5e28da3dba10fd3f2ea6ebf6f5d6271157bda0b12b82d92aedbe3031643089ef
geometry compiler source sha256 (combined): ecd43602c95937d1746ea34a307a435c49a85ab1fdef8b4f8d3710411b088536
```

v3 corrects a v2 reconciliation bug: `contact_model_status` for a
hardware-oracle endpoint must be driven by MESH-vs-HARDWARE agreement, not
HARDWARE-vs-URDF agreement (a mesh contact can sit close to the declared
URDF limit yet be several degrees from where hardware actually stopped,
which means it is not the real endstop regardless of URDF proximity). v2
also had the sign wrong on 3 of 6 LF hardware-contact-angle derivations
(hip_min, upper_leg_min, lower_leg_min) from inconsistent manual delta
conventions; v3 stores the actual hardware contact angle instead of a
pre-computed delta, removing that class of error structurally.

## 3. Final 24-endpoint classification

```text
MODEL_INCOMPLETE:          6  (all LF -- see section 4)
MODEL_LIMIT_MISMATCH:      4  (rf_hip_max, rf_lower_leg_min, rh_lower_leg_min, lh_lower_leg_min)
NO_MODELED_ENDSTOP:        14
PATH_COLLISION_BEFORE_ENDPOINT: 0
UNINTENDED_SELF_COLLISION: 0
                            -- sum: 6+4+14 = 24
```

No endpoint required an automatic URDF revision; none was performed.

## 4. LF V25 hardware reconciliation — 6/6 = MODEL_INCOMPLETE

Direct hardware oracle (2026-08-04 V25 freeze, `MATDOG_LF_CALIBRATION_V25_FINAL.md`
+ archived per-motor profile) proves a real mechanical contact exists for
all six LF endpoints. None of the six collision-mesh findings (or absences)
corresponds to that real contact:

| Endpoint | Hardware contact | Mesh contact | mesh - hardware | Reason |
|---|---:|---:|---:|---|
| lf_hip_min | -42.803 deg | -47.500 deg | -4.70 deg | mesh contact found, but past the real hardware contact |
| lf_hip_max | +39.375 deg | none | - | no mesh contact at all; hardware proves a real one exists |
| lf_upper_leg_min | -53.525 deg | none | - | idem |
| lf_upper_leg_max | +122.607 deg | none | - | idem |
| lf_lower_leg_min | -91.846 deg | -97.957 deg | -6.11 deg | mesh contact found, but past the real hardware contact |
| lf_lower_leg_max | +34.277 deg | none | - | no mesh contact at all; hardware proves a real one exists |

Conclusion: the true LF endstop mechanism (servo/bracket internal limit)
is not represented in the current collision STL geometry for any of the
six joints. `MODEL_LIMIT_MISMATCH` (mesh vs. declared URDF) is retained as
an independent diagnostic flag on the two endpoints where a mesh contact
does exist (hip_min, lower_leg_min) but is not the primary
`contact_model_status`.

RF/RH/LH have no hardware oracle and were never auto-promoted to
`MODEL_INCOMPLETE`: their four `MODEL_LIMIT_MISMATCH` findings (mesh vs.
declared URDF only) stay as such, per the explicit regression test
`test_rf_rh_lh_without_hardware_oracle_never_auto_promoted_to_model_incomplete`.

## 5. Parking (segment-specific, default NO auxiliary parking)

```text
LF: REQUIRED, park LH at upper +30 deg (single park-before/restore-after
    for the whole LF sequence)
RF: REQUIRED, park RH at upper +30 deg (symmetric)
RH: NOT REQUIRED
LH: NOT REQUIRED
```

+30 deg is the smallest of the historical +30..+90 deg checkpoint seeds
and matches LF V25 hardware practice exactly (`station.log`: "Park LH
upper M42 once for the complete LF session" ... "Restore LH upper M42
once at end of LF calibration", session PASS 58/58).

## 6. Clearance semantics (EXACT / LOWER_BOUND / UNRESOLVED_FOR_THRESHOLD)

Every `PairCollisionResult` now carries a `clearance_kind` (`EXACT` when
the narrow phase found an actual closest triangle pair; `LOWER_BOUND` for
AABB/hull separation or a narrow-phase search-margin fallback). Path
segments carry a tri-state `clearance_gate_result`:
`PASS` / `FAIL` / `UNRESOLVED_FOR_THRESHOLD`. A `LOWER_BOUND` figure below
the 3mm pass bar resolves as `UNRESOLVED_FOR_THRESHOLD`, never a false
`FAIL` -- this is what the RH/LH "1.0mm" and LF/RF "0.117mm" residual
findings are (search-margin artifacts, not measured small gaps).

## 7. Manufacturing / assembly tolerance policy

`print_tolerance_m = +/-0.15mm` (PPA+CF) applied as a single part's
tolerance (Delta d) in the two-point clearance gradient, explicitly NOT
summed or RSS-combined with the second printed part in the same contact
pair (`tolerance_budget_note` on every endpoint's sensitivity record: a
2-part sum would be ~2x, RSS ~1.41x). Assembly-level tolerance (bushings,
screws, servo horn backlash) is explicitly `UNKNOWN`. Sensitivity is
pinned to the exact same contact pair that defines the endpoint (never
`worst_pair_at_pose`'s possibly-different pair).

## 8. Final validation gate

```text
Full test suite (tests/test_matdog_geometry_*.py), v3 code, definitive run:
  python3 -m unittest discover -s tests -p 'test_matdog_geometry_*.py'
  Ran 81 tests in 1084.393s
  OK -- 0 failures, 0 errors
  exit status: 0
  peak RSS: 680500 kB (~665 MB)

py_compile:  OK (all matdog_geometry_*.py and tests/test_matdog_geometry_*.py)
pyflakes:    OK (no unused imports/names)
```

Note: an earlier run against the same v3 code (before this final one) found
one real, legitimate test failure --
`TestModelLimitMismatch.test_deliberately_wrong_declared_limit_is_flagged_as_mismatch`
-- caused by the test incidentally reusing the `lf_hip_max` endpoint_id,
which (unlike the synthetic declared-limit override the test supplied)
now carries a real LF V25 hardware oracle entry keyed purely by
leg/joint_group/side; the classifier correctly used the hardware-vs-mesh
comparison instead of the test's intended mesh-vs-declared-URDF scenario.
Fixed by switching the test to `rf_hip_min` (RF has no hardware oracle),
which restores the test's original intent. No production code changed
for this fix, only the test.

```text
Full compiler run (24 endpoints, v3 classifier):
  exit status: 0
  elapsed: 29:22.90
  peak RSS: 478304 kB (~467 MB), well under the 3 GB target
```

## 9. Explicit confirmations

- HARDWARE NOT USED
- NORMA-CORE NOT MODIFIED
- LF V25 NOT MODIFIED (read-only: canonical doc + archived run data)
- Canonical URDF NOT MODIFIED
- Phase 2 NOT STARTED
