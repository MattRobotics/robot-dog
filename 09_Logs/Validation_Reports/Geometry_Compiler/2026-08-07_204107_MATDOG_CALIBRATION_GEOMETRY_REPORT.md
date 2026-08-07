# MATDOG Calibration Geometry Profile — Phase 1 report

schema_version: `matdog.calibration_geometry_profile.v3`
generation_timestamp_utc: `2026-08-07T20:41:07.524655+00:00`
robot_dog_commit_sha: `344675f4107683133fdb0f1f7171b54e05e6aa4b`
robot_dog_working_tree_dirty: `True`
content_sha256: `7d6c0577bde7ee31ae5672560406d65e096ab7de449d9436f3d43a8ce75172bb`
urdf_sha256: `5e28da3dba10fd3f2ea6ebf6f5d6271157bda0b12b82d92aedbe3031643089ef`

HARDWARE NOT USED. NORMA-CORE NOT MODIFIED. NO COMMIT/PUSH/PR/MERGE.

## LF V25 hardware reconciliation (read before the 24-endpoint table)

| Endpoint | Declared URDF | Hardware contact | Compiler mesh contact | Compiler contact pair | hw vs URDF | mesh vs hw | Contact model status | Corrective action |
|---|---:|---:|---:|---|---|---|---|---|
| lf_hip_max | +45.000 deg | +39.375 deg | - | - | INCOMPATIBLE | NO_MESH_CONTACT | **MODEL_INCOMPLETE** | none automatic: real stopping mechanism is not represented in the collision STL (servo/bracket internal limit, or a mesh contact that does not correspond to the real hardware contact); NEEDS_HUMAN_DECISION if a mesh model of that feature should be added |
| lf_hip_min | -45.000 deg | -42.803 deg | -47.500 deg | base_link <-> lf_upper_leg_link | INCOMPATIBLE | DISAGREES | **MODEL_INCOMPLETE** | none automatic: real stopping mechanism is not represented in the collision STL (servo/bracket internal limit, or a mesh contact that does not correspond to the real hardware contact); NEEDS_HUMAN_DECISION if a mesh model of that feature should be added |
| lf_lower_leg_max | +37.500 deg | +34.277 deg | - | - | INCOMPATIBLE | NO_MESH_CONTACT | **MODEL_INCOMPLETE** | none automatic: real stopping mechanism is not represented in the collision STL (servo/bracket internal limit, or a mesh contact that does not correspond to the real hardware contact); NEEDS_HUMAN_DECISION if a mesh model of that feature should be added |
| lf_lower_leg_min | -92.000 deg | -91.846 deg | -97.957 deg | lf_foot_link <-> lf_upper_leg_link | COMPATIBLE | DISAGREES | **MODEL_INCOMPLETE** | none automatic: real stopping mechanism is not represented in the collision STL (servo/bracket internal limit, or a mesh contact that does not correspond to the real hardware contact); NEEDS_HUMAN_DECISION if a mesh model of that feature should be added |
| lf_upper_leg_max | +122.500 deg | +122.607 deg | - | - | COMPATIBLE | NO_MESH_CONTACT | **MODEL_INCOMPLETE** | none automatic: real stopping mechanism is not represented in the collision STL (servo/bracket internal limit, or a mesh contact that does not correspond to the real hardware contact); NEEDS_HUMAN_DECISION if a mesh model of that feature should be added |
| lf_upper_leg_min | -52.500 deg | -53.525 deg | - | - | COMPATIBLE | NO_MESH_CONTACT | **MODEL_INCOMPLETE** | none automatic: real stopping mechanism is not represented in the collision STL (servo/bracket internal limit, or a mesh contact that does not correspond to the real hardware contact); NEEDS_HUMAN_DECISION if a mesh model of that feature should be added |

## 24 endpoints

| Leg | Joint | Side | Declared URDF | Mesh predicted contact | Delta | Contact pair | Clearance before contact | Contact model status | Path collision (if any) |
|---|---|---|---:|---:|---:|---|---:|---|---|
| LF | hip | max | +45.000 deg | - | - | - | - | **MODEL_INCOMPLETE** | - |
| LF | hip | min | -45.000 deg | -47.500 deg | -2.500 deg | base_link <-> lf_upper_leg_link | 0.0011 mm | **MODEL_INCOMPLETE** | - |
| LF | lower_leg | max | +37.500 deg | - | - | - | - | **MODEL_INCOMPLETE** | - |
| LF | lower_leg | min | -92.000 deg | -97.957 deg | -5.957 deg | lf_foot_link <-> lf_upper_leg_link | 0.0012 mm | **MODEL_INCOMPLETE** | - |
| LF | upper_leg | max | +122.500 deg | - | - | - | - | **MODEL_INCOMPLETE** | - |
| LF | upper_leg | min | -52.500 deg | - | - | - | - | **MODEL_INCOMPLETE** | - |
| LH | hip | max | +45.000 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |
| LH | hip | min | -45.000 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |
| LH | lower_leg | max | +37.500 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |
| LH | lower_leg | min | -92.000 deg | -97.957 deg | -5.957 deg | lh_foot_link <-> lh_upper_leg_link | 0.0012 mm | **MODEL_LIMIT_MISMATCH** | - |
| LH | upper_leg | max | +122.500 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |
| LH | upper_leg | min | -52.500 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |
| RF | hip | max | +45.000 deg | +47.500 deg | +2.500 deg | base_link <-> rf_upper_leg_link | 0.0011 mm | **MODEL_LIMIT_MISMATCH** | - |
| RF | hip | min | -45.000 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |
| RF | lower_leg | max | +37.500 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |
| RF | lower_leg | min | -92.000 deg | -98.004 deg | -6.004 deg | rf_foot_link <-> rf_upper_leg_link | 0.0002 mm | **MODEL_LIMIT_MISMATCH** | - |
| RF | upper_leg | max | +122.500 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |
| RF | upper_leg | min | -52.500 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |
| RH | hip | max | +45.000 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |
| RH | hip | min | -45.000 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |
| RH | lower_leg | max | +37.500 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |
| RH | lower_leg | min | -92.000 deg | -98.004 deg | -6.004 deg | rh_foot_link <-> rh_upper_leg_link | 0.0002 mm | **MODEL_LIMIT_MISMATCH** | - |
| RH | upper_leg | max | +122.500 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |
| RH | upper_leg | min | -52.500 deg | - | - | - | - | **NO_MODELED_ENDSTOP** | - |

## Mirror comparisons

### LF vs RF

| Joint | Side | LF contact | RF contact | Delta |
|---|---|---:|---:|---:|
| hip | min | -47.500 deg (MODEL_INCOMPLETE) | - (NO_MODELED_ENDSTOP) | - |
| hip | max | - (MODEL_INCOMPLETE) | +47.500 deg (MODEL_LIMIT_MISMATCH) | - |
| upper_leg | min | - (MODEL_INCOMPLETE) | - (NO_MODELED_ENDSTOP) | - |
| upper_leg | max | - (MODEL_INCOMPLETE) | - (NO_MODELED_ENDSTOP) | - |
| lower_leg | min | -97.957 deg (MODEL_INCOMPLETE) | -98.004 deg (MODEL_LIMIT_MISMATCH) | +0.047 deg |
| lower_leg | max | - (MODEL_INCOMPLETE) | - (NO_MODELED_ENDSTOP) | - |

### RH vs LH

| Joint | Side | RH contact | LH contact | Delta |
|---|---|---:|---:|---:|
| hip | min | - (NO_MODELED_ENDSTOP) | - (NO_MODELED_ENDSTOP) | - |
| hip | max | - (NO_MODELED_ENDSTOP) | - (NO_MODELED_ENDSTOP) | - |
| upper_leg | min | - (NO_MODELED_ENDSTOP) | - (NO_MODELED_ENDSTOP) | - |
| upper_leg | max | - (NO_MODELED_ENDSTOP) | - (NO_MODELED_ENDSTOP) | - |
| lower_leg | min | -98.004 deg (MODEL_LIMIT_MISMATCH) | -97.957 deg (MODEL_LIMIT_MISMATCH) | -0.047 deg |
| lower_leg | max | - (NO_MODELED_ENDSTOP) | - (NO_MODELED_ENDSTOP) | - |

### FRONT vs HIND

| Joint | Side | FRONT (LF) | HIND (RH) | Note |
|---|---|---:|---:|---|
| hip | min | -47.500 deg (MODEL_INCOMPLETE) | - (NO_MODELED_ENDSTOP) | DIFFERENT status |
| hip | max | - (MODEL_INCOMPLETE) | - (NO_MODELED_ENDSTOP) | DIFFERENT status |
| upper_leg | min | - (MODEL_INCOMPLETE) | - (NO_MODELED_ENDSTOP) | DIFFERENT status |
| upper_leg | max | - (MODEL_INCOMPLETE) | - (NO_MODELED_ENDSTOP) | DIFFERENT status |
| lower_leg | min | -97.957 deg (MODEL_INCOMPLETE) | -98.004 deg (MODEL_LIMIT_MISMATCH) | DIFFERENT status |
| lower_leg | max | - (MODEL_INCOMPLETE) | - (NO_MODELED_ENDSTOP) | DIFFERENT status |

Front hip Z ~= 0.0465 m, hind hip Z ~= 0.0265 m (20 mm difference, verified numerically in tests/test_matdog_geometry_scene.py); this does not change detector physics but can change prerequisite/parking/path clearance, per canonical handoff section 4.

## Parking

| Leg | Auxiliary parking | Parked leg | Angle | Reason |
|---|---|---|---:|---|
| LF | REQUIRED | lh | +30.000 deg | segment(s) ['upper probe positive (home prerequisite)', 'upper probe return to home', 'transition HIP prerequisite -> LOWER prerequisite', 'restore leg to home'] collide against lh at home (true mesh intersection); minimal seed parking 30 deg for lh, held for the whole lf sequence (single park-before/restore-after, matching the validated 2026-07-20 checkpoint and LF V25 hardware practice), resolves every true collision; residual NEEDS_HUMAN_DECISION margin finding (not a collision): minimum modelled clearance 0.1168 mm at 'hip probe return' is below the configured PASS bar (3.0000 mm); this is an UNRESOLVED_FOR_THRESHOLD lower bound (search margin, not a proven small gap) |
| LH | NOT REQUIRED | - | - | no true mesh collision in any segment with every other leg at home; NEEDS_HUMAN_DECISION margin finding (not a collision, parking not applicable): minimum modelled clearance 1.0000 mm at 'lower probe return' is below the configured PASS bar (3.0000 mm); this is an UNRESOLVED_FOR_THRESHOLD lower bound (search margin, not a proven small gap) |
| RF | REQUIRED | rh | +30.000 deg | segment(s) ['upper probe positive (home prerequisite)', 'upper probe return to home', 'transition HIP prerequisite -> LOWER prerequisite', 'restore leg to home'] collide against rh at home (true mesh intersection); minimal seed parking 30 deg for rh, held for the whole rf sequence (single park-before/restore-after, matching the validated 2026-07-20 checkpoint and LF V25 hardware practice), resolves every true collision; residual NEEDS_HUMAN_DECISION margin finding (not a collision): minimum modelled clearance 0.1168 mm at 'hip probe return' is below the configured PASS bar (3.0000 mm); this is an UNRESOLVED_FOR_THRESHOLD lower bound (search margin, not a proven small gap) |
| RH | NOT REQUIRED | - | - | no true mesh collision in any segment with every other leg at home; NEEDS_HUMAN_DECISION margin finding (not a collision, parking not applicable): minimum modelled clearance 1.0000 mm at 'lower probe return' is below the configured PASS bar (3.0000 mm); this is an UNRESOLVED_FOR_THRESHOLD lower bound (search margin, not a proven small gap) |

## Minimum modelled clearance per leg sequence

| Leg | Sequence PASS | Minimum modelled clearance |
|---|---|---:|
| LF | False | 0.1168 mm |
| LH | False | 1.0000 mm |
| RF | False | 0.1168 mm |
| RH | False | 1.0000 mm |

## MODEL_LIMIT_MISMATCH

- lh_lower_leg_min: declared -92.000 deg, mesh -97.957 deg, delta -5.957 deg
- rf_hip_max: declared +45.000 deg, mesh +47.500 deg, delta +2.500 deg
- rf_lower_leg_min: declared -92.000 deg, mesh -98.004 deg, delta -6.004 deg
- rh_lower_leg_min: declared -92.000 deg, mesh -98.004 deg, delta -6.004 deg

## PATH_COLLISION_BEFORE_ENDPOINT (cross-leg path obstructions)

(none)

## MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY (hardware disagrees, real stop not in collision STL)

- lf_hip_max: no same-leg mesh contact found in the analysis envelope, but LF V25 hardware proves a real contact exists at 39.38 deg -- the real stopping mechanism (servo/bracket internal limit, not the collision STL) is not represented in this model
- lf_hip_min: same-leg mesh contact at -47.50 deg does NOT match the LF V25 hardware contact at -42.80 deg (delta -4.70 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint. Separately, model_limit_mismatch=True vs the declared URDF limit (independent diagnostic flag, delta -2.50 deg)
- lf_lower_leg_max: no same-leg mesh contact found in the analysis envelope, but LF V25 hardware proves a real contact exists at 34.28 deg -- the real stopping mechanism (servo/bracket internal limit, not the collision STL) is not represented in this model
- lf_lower_leg_min: same-leg mesh contact at -97.96 deg does NOT match the LF V25 hardware contact at -91.85 deg (delta -6.11 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint. Separately, model_limit_mismatch=True vs the declared URDF limit (independent diagnostic flag, delta -5.96 deg)
- lf_upper_leg_max: no same-leg mesh contact found in the analysis envelope, but LF V25 hardware proves a real contact exists at 122.61 deg -- the real stopping mechanism (servo/bracket internal limit, not the collision STL) is not represented in this model
- lf_upper_leg_min: no same-leg mesh contact found in the analysis envelope, but LF V25 hardware proves a real contact exists at -53.53 deg -- the real stopping mechanism (servo/bracket internal limit, not the collision STL) is not represented in this model

## UNINTENDED_SELF_COLLISION

(none)

## NO_MODELED_ENDSTOP (no mesh contact found in envelope)

- lh_hip_max: declared +45.000 deg, envelope +35.000 deg..+55.000 deg
- lh_hip_min: declared -45.000 deg, envelope -55.000 deg..-35.000 deg
- lh_lower_leg_max: declared +37.500 deg, envelope +27.500 deg..+47.500 deg
- lh_upper_leg_max: declared +122.500 deg, envelope +112.500 deg..+132.500 deg
- lh_upper_leg_min: declared -52.500 deg, envelope -62.500 deg..-42.500 deg
- rf_hip_min: declared -45.000 deg, envelope -55.000 deg..-35.000 deg
- rf_lower_leg_max: declared +37.500 deg, envelope +27.500 deg..+47.500 deg
- rf_upper_leg_max: declared +122.500 deg, envelope +112.500 deg..+132.500 deg
- rf_upper_leg_min: declared -52.500 deg, envelope -62.500 deg..-42.500 deg
- rh_hip_max: declared +45.000 deg, envelope +35.000 deg..+55.000 deg
- rh_hip_min: declared -45.000 deg, envelope -55.000 deg..-35.000 deg
- rh_lower_leg_max: declared +37.500 deg, envelope +27.500 deg..+47.500 deg
- rh_upper_leg_max: declared +122.500 deg, envelope +112.500 deg..+132.500 deg
- rh_upper_leg_min: declared -52.500 deg, envelope -62.500 deg..-42.500 deg

## Unresolved assumptions / UNKNOWN

- lf_hip_max: MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY -- no same-leg mesh contact found in the analysis envelope, but LF V25 hardware proves a real contact exists at 39.38 deg -- the real stopping mechanism (servo/bracket internal limit, not the collision STL) is not represented in this model. NEEDS_HUMAN_DECISION: the real mechanical endstop is not represented in the current collision STL geometry.
- lf_hip_min: MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY -- same-leg mesh contact at -47.50 deg does NOT match the LF V25 hardware contact at -42.80 deg (delta -4.70 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint. Separately, model_limit_mismatch=True vs the declared URDF limit (independent diagnostic flag, delta -2.50 deg). NEEDS_HUMAN_DECISION: the real mechanical endstop is not represented in the current collision STL geometry.
- lf_lower_leg_max: MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY -- no same-leg mesh contact found in the analysis envelope, but LF V25 hardware proves a real contact exists at 34.28 deg -- the real stopping mechanism (servo/bracket internal limit, not the collision STL) is not represented in this model. NEEDS_HUMAN_DECISION: the real mechanical endstop is not represented in the current collision STL geometry.
- lf_lower_leg_min: MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY -- same-leg mesh contact at -97.96 deg does NOT match the LF V25 hardware contact at -91.85 deg (delta -6.11 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint. Separately, model_limit_mismatch=True vs the declared URDF limit (independent diagnostic flag, delta -5.96 deg). NEEDS_HUMAN_DECISION: the real mechanical endstop is not represented in the current collision STL geometry.
- lf_upper_leg_max: MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY -- no same-leg mesh contact found in the analysis envelope, but LF V25 hardware proves a real contact exists at 122.61 deg -- the real stopping mechanism (servo/bracket internal limit, not the collision STL) is not represented in this model. NEEDS_HUMAN_DECISION: the real mechanical endstop is not represented in the current collision STL geometry.
- lf_upper_leg_min: MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY -- no same-leg mesh contact found in the analysis envelope, but LF V25 hardware proves a real contact exists at -53.53 deg -- the real stopping mechanism (servo/bracket internal limit, not the collision STL) is not represented in this model. NEEDS_HUMAN_DECISION: the real mechanical endstop is not represented in the current collision STL geometry.
- lh_hip_max: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.
- lh_hip_min: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.
- lh_lower_leg_max: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.
- lh_lower_leg_min: MODEL_LIMIT_MISMATCH, mesh-predicted contact differs from the declared URDF limit by -0.103970 rad, with no direct hardware evidence available to explain it; NEEDS_HUMAN_DECISION on whether to revise the URDF limit (the compiler never does this automatically).
- lh_upper_leg_max: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.
- lh_upper_leg_min: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.
- manufacturing tolerance UNKNOWN: only the +/-0.15mm per-part print tolerance is modelled; assembly-level stack-up (bushings, screws, servo horn backlash, fit clearances) is not known and is not included in the sensitivity estimate.
- min_clearance_pass_m=3mm ('adequate clearance' threshold for path PASS) is a documented conservative default chosen for this compiler, not a project-mandated value; NEEDS_HUMAN_DECISION if a different bar is intended.
- narrow_phase_margin_m=1mm / grid_cell_size_m=5mm are compiler performance/precision parameters tuned for MATDOG's actual mm-scale collision meshes; they are configurable and reported here rather than hardcoded assumptions, per canonical handoff section 4.
- rf_hip_max: MODEL_LIMIT_MISMATCH, mesh-predicted contact differs from the declared URDF limit by 0.043633 rad, with no direct hardware evidence available to explain it; NEEDS_HUMAN_DECISION on whether to revise the URDF limit (the compiler never does this automatically).
- rf_hip_min: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.
- rf_lower_leg_max: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.
- rf_lower_leg_min: MODEL_LIMIT_MISMATCH, mesh-predicted contact differs from the declared URDF limit by -0.104788 rad, with no direct hardware evidence available to explain it; NEEDS_HUMAN_DECISION on whether to revise the URDF limit (the compiler never does this automatically).
- rf_upper_leg_max: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.
- rf_upper_leg_min: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.
- rh_hip_max: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.
- rh_hip_min: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.
- rh_lower_leg_max: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.
- rh_lower_leg_min: MODEL_LIMIT_MISMATCH, mesh-predicted contact differs from the declared URDF limit by -0.104788 rad, with no direct hardware evidence available to explain it; NEEDS_HUMAN_DECISION on whether to revise the URDF limit (the compiler never does this automatically).
- rh_upper_leg_max: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.
- rh_upper_leg_min: NO_MODELED_ENDSTOP -- no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. internal servo/bracket range), not from these collision meshes.

## Collision mesh manifest

| Link | SHA256 | Triangles | Degenerate dropped |
|---|---|---:|---:|
| base_link | `7b149643a6a71ae8ac37780085c021c5bff0c4491ebd42f1393150cb987e9e6e` | 105330 | 0 |
| lf_foot_link | `e43737c4cbc5d618431ecf2ce9ba20a4803f5a7b7f800974a6d8b2c7534091bc` | 9410 | 0 |
| lf_hip_link | `7d28351aba669cc1a8e11ad60b89db001f9689b728e4e8eb05849af69e1e4879` | 22992 | 0 |
| lf_lower_leg_link | `97f2bb775768fdbcef59638916eaef2cd34b21bf01e6197ef3900b0df85d7ae2` | 18096 | 0 |
| lf_upper_leg_link | `dfaf754764ded80743b00ae5d1dde208301ab5a3cebc9e29a38d8513dbe4cd93` | 39448 | 2 |
| lh_foot_link | `e43737c4cbc5d618431ecf2ce9ba20a4803f5a7b7f800974a6d8b2c7534091bc` | 9410 | 0 |
| lh_hip_link | `543f6f04a4dc45a0b176eba8db8ab7819ab20a328aa4f7fd9c9d4b77c1612094` | 22992 | 0 |
| lh_lower_leg_link | `97f2bb775768fdbcef59638916eaef2cd34b21bf01e6197ef3900b0df85d7ae2` | 18096 | 0 |
| lh_upper_leg_link | `dfaf754764ded80743b00ae5d1dde208301ab5a3cebc9e29a38d8513dbe4cd93` | 39448 | 2 |
| rf_foot_link | `e43737c4cbc5d618431ecf2ce9ba20a4803f5a7b7f800974a6d8b2c7534091bc` | 9410 | 0 |
| rf_hip_link | `ebd664b3db2065ac3341184a6b12700caca6cdece57adae72f935882ed9df4c0` | 22992 | 0 |
| rf_lower_leg_link | `c52ae662c09455f53db5bc8efc13332919942c7f938ada5a8c0e987f54ac3bc8` | 18094 | 0 |
| rf_upper_leg_link | `3c2508690ac89d006d25cabf3917ce52dc2c0b09e4ec5e051da8f71ff1a7e760` | 39448 | 2 |
| rh_foot_link | `e43737c4cbc5d618431ecf2ce9ba20a4803f5a7b7f800974a6d8b2c7534091bc` | 9410 | 0 |
| rh_hip_link | `2e9f31952e51b5748546c803c06d72316a3a776226f1f72775c1e8d383e1f325` | 22992 | 0 |
| rh_lower_leg_link | `c52ae662c09455f53db5bc8efc13332919942c7f938ada5a8c0e987f54ac3bc8` | 18094 | 0 |
| rh_upper_leg_link | `3c2508690ac89d006d25cabf3917ce52dc2c0b09e4ec5e051da8f71ff1a7e760` | 39448 | 2 |
