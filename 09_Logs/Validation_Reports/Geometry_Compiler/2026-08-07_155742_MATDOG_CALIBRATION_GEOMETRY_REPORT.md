# MATDOG Calibration Geometry Profile — Phase 1 report

schema_version: `matdog.calibration_geometry_profile.v1`
generation_timestamp_utc: `2026-08-07T15:57:42.407627+00:00`
robot_dog_commit_sha: `344675f4107683133fdb0f1f7171b54e05e6aa4b`
robot_dog_working_tree_dirty: `True`
content_sha256: `f4fc309f18fea34952d7e316d368c49edfb77c79cb0b4caca52fade2da2aa58d`
urdf_sha256: `5e28da3dba10fd3f2ea6ebf6f5d6271157bda0b12b82d92aedbe3031643089ef`

HARDWARE NOT USED. NORMA-CORE NOT MODIFIED. NO COMMIT/PUSH/PR/MERGE.

## 24 endpoints

| Leg | Joint | Side | Declared URDF | Mesh predicted contact | Delta | Contact pair | Clearance before contact | Result |
|---|---|---|---:|---:|---:|---|---:|---|
| LF | hip | max | +45.000 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| LF | hip | min | -45.000 deg | -47.500 deg | -2.500 deg | base_link <-> lf_upper_leg_link | 0.0011 mm | MESH_CONTACT_FOUND |
| LF | lower_leg | max | +37.500 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| LF | lower_leg | min | -92.000 deg | -97.957 deg | -5.957 deg | lf_foot_link <-> lf_upper_leg_link | 0.0012 mm | MESH_CONTACT_FOUND |
| LF | upper_leg | max | +122.500 deg | +73.281 deg | - | lf_foot_link <-> lh_foot_link | 0.0039 mm | UNINTENDED_COLLISION_BEFORE_ENDPOINT |
| LF | upper_leg | min | -52.500 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| LH | hip | max | +45.000 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| LH | hip | min | -45.000 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| LH | lower_leg | max | +37.500 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| LH | lower_leg | min | -92.000 deg | -97.957 deg | -5.957 deg | lh_foot_link <-> lh_upper_leg_link | 0.0012 mm | MESH_CONTACT_FOUND |
| LH | upper_leg | max | +122.500 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| LH | upper_leg | min | -52.500 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| RF | hip | max | +45.000 deg | +47.500 deg | +2.500 deg | base_link <-> rf_upper_leg_link | 0.0011 mm | MESH_CONTACT_FOUND |
| RF | hip | min | -45.000 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| RF | lower_leg | max | +37.500 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| RF | lower_leg | min | -92.000 deg | -98.004 deg | -6.004 deg | rf_foot_link <-> rf_upper_leg_link | 0.0002 mm | MESH_CONTACT_FOUND |
| RF | upper_leg | max | +122.500 deg | +73.281 deg | - | rf_foot_link <-> rh_foot_link | 0.0039 mm | UNINTENDED_COLLISION_BEFORE_ENDPOINT |
| RF | upper_leg | min | -52.500 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| RH | hip | max | +45.000 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| RH | hip | min | -45.000 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| RH | lower_leg | max | +37.500 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| RH | lower_leg | min | -92.000 deg | -98.004 deg | -6.004 deg | rh_foot_link <-> rh_upper_leg_link | 0.0002 mm | MESH_CONTACT_FOUND |
| RH | upper_leg | max | +122.500 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |
| RH | upper_leg | min | -52.500 deg | - | - | - | - | NO_MESH_CONTACT_IN_ENVELOPE |

## Mirror comparisons

### LF vs RF

| Joint | Side | LF contact | RF contact | Delta |
|---|---|---:|---:|---:|
| hip | min | -47.500 deg (MESH_CONTACT_FOUND) | - (NO_MESH_CONTACT_IN_ENVELOPE) | - |
| hip | max | - (NO_MESH_CONTACT_IN_ENVELOPE) | +47.500 deg (MESH_CONTACT_FOUND) | - |
| upper_leg | min | - (NO_MESH_CONTACT_IN_ENVELOPE) | - (NO_MESH_CONTACT_IN_ENVELOPE) | - |
| upper_leg | max | +73.281 deg (UNINTENDED_COLLISION_BEFORE_ENDPOINT) | +73.281 deg (UNINTENDED_COLLISION_BEFORE_ENDPOINT) | +0.000 deg |
| lower_leg | min | -97.957 deg (MESH_CONTACT_FOUND) | -98.004 deg (MESH_CONTACT_FOUND) | +0.047 deg |
| lower_leg | max | - (NO_MESH_CONTACT_IN_ENVELOPE) | - (NO_MESH_CONTACT_IN_ENVELOPE) | - |

### RH vs LH

| Joint | Side | RH contact | LH contact | Delta |
|---|---|---:|---:|---:|
| hip | min | - (NO_MESH_CONTACT_IN_ENVELOPE) | - (NO_MESH_CONTACT_IN_ENVELOPE) | - |
| hip | max | - (NO_MESH_CONTACT_IN_ENVELOPE) | - (NO_MESH_CONTACT_IN_ENVELOPE) | - |
| upper_leg | min | - (NO_MESH_CONTACT_IN_ENVELOPE) | - (NO_MESH_CONTACT_IN_ENVELOPE) | - |
| upper_leg | max | - (NO_MESH_CONTACT_IN_ENVELOPE) | - (NO_MESH_CONTACT_IN_ENVELOPE) | - |
| lower_leg | min | -98.004 deg (MESH_CONTACT_FOUND) | -97.957 deg (MESH_CONTACT_FOUND) | -0.047 deg |
| lower_leg | max | - (NO_MESH_CONTACT_IN_ENVELOPE) | - (NO_MESH_CONTACT_IN_ENVELOPE) | - |

### FRONT vs HIND

| Joint | Side | FRONT (LF) | HIND (RH) | Note |
|---|---|---:|---:|---|
| hip | min | -47.500 deg (MESH_CONTACT_FOUND) | - (NO_MESH_CONTACT_IN_ENVELOPE) | DIFFERENT result_kind |
| hip | max | - (NO_MESH_CONTACT_IN_ENVELOPE) | - (NO_MESH_CONTACT_IN_ENVELOPE) | same result_kind |
| upper_leg | min | - (NO_MESH_CONTACT_IN_ENVELOPE) | - (NO_MESH_CONTACT_IN_ENVELOPE) | same result_kind |
| upper_leg | max | +73.281 deg (UNINTENDED_COLLISION_BEFORE_ENDPOINT) | - (NO_MESH_CONTACT_IN_ENVELOPE) | DIFFERENT result_kind |
| lower_leg | min | -97.957 deg (MESH_CONTACT_FOUND) | -98.004 deg (MESH_CONTACT_FOUND) | same result_kind |
| lower_leg | max | - (NO_MESH_CONTACT_IN_ENVELOPE) | - (NO_MESH_CONTACT_IN_ENVELOPE) | same result_kind |

Front hip Z ~= 0.0465 m, hind hip Z ~= 0.0265 m (20 mm difference, verified numerically in tests/test_matdog_geometry_scene.py); this does not change detector physics but can change prerequisite/parking/path clearance, per canonical handoff section 4.

## Parking

| Leg | Auxiliary parking | Parked leg | Angle | Reason |
|---|---|---|---:|---|
| LF | REQUIRED | lh | - | NEEDS_HUMAN_DECISION: no seed parking angle in (30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0) for lh resolved the collision; a search over a wider/finer angle range or a different auxiliary leg may be required |
| LH | NOT REQUIRED | - | - | NEEDS_HUMAN_DECISION: no mesh intersection found anywhere in the sequence, but minimum modelled clearance 1.0000 mm at 'lower probe return' is below the configured PASS bar (3.0000 mm); this is a margin finding, not a collision, so auxiliary parking of another leg is not applicable |
| RF | REQUIRED | rh | - | NEEDS_HUMAN_DECISION: no seed parking angle in (30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0) for rh resolved the collision; a search over a wider/finer angle range or a different auxiliary leg may be required |
| RH | NOT REQUIRED | - | - | NEEDS_HUMAN_DECISION: no mesh intersection found anywhere in the sequence, but minimum modelled clearance 1.0000 mm at 'lower probe return' is below the configured PASS bar (3.0000 mm); this is a margin finding, not a collision, so auxiliary parking of another leg is not applicable |

## Minimum modelled clearance per leg sequence

| Leg | Sequence PASS | Minimum modelled clearance |
|---|---|---:|
| LF | False | 0.1168 mm |
| LH | False | 1.0000 mm |
| RF | False | 0.1168 mm |
| RH | False | 1.0000 mm |

## MODEL_LIMIT_MISMATCH

- lf_hip_min: declared -45.000 deg, mesh -47.500 deg, delta -2.500 deg
- lf_lower_leg_min: declared -92.000 deg, mesh -97.957 deg, delta -5.957 deg
- lh_lower_leg_min: declared -92.000 deg, mesh -97.957 deg, delta -5.957 deg
- rf_hip_max: declared +45.000 deg, mesh +47.500 deg, delta +2.500 deg
- rf_lower_leg_min: declared -92.000 deg, mesh -98.004 deg, delta -6.004 deg
- rh_lower_leg_min: declared -92.000 deg, mesh -98.004 deg, delta -6.004 deg

## Unintended collisions before endpoint

- lf_upper_leg_max: lf_foot_link <-> lh_foot_link at +73.281 deg
- rf_upper_leg_max: rf_foot_link <-> rh_foot_link at +73.281 deg

## No mesh contact found in envelope

- lf_hip_max: declared +45.000 deg, envelope +35.000 deg..+55.000 deg
- lf_lower_leg_max: declared +37.500 deg, envelope +27.500 deg..+47.500 deg
- lf_upper_leg_min: declared -52.500 deg, envelope -62.500 deg..-42.500 deg
- lh_hip_max: declared +45.000 deg, envelope +35.000 deg..+55.000 deg
- lh_hip_min: declared -45.000 deg, envelope -55.000 deg..-35.000 deg
- lh_lower_leg_max: declared +37.500 deg, envelope +27.500 deg..+47.500 deg
- lh_upper_leg_max: declared +122.500 deg, envelope +112.500 deg..+132.500 deg
- lh_upper_leg_min: declared -52.500 deg, envelope -62.500 deg..-42.500 deg
- rf_hip_min: declared -45.000 deg, envelope -55.000 deg..-35.000 deg
- rf_lower_leg_max: declared +37.500 deg, envelope +27.500 deg..+47.500 deg
- rf_upper_leg_min: declared -52.500 deg, envelope -62.500 deg..-42.500 deg
- rh_hip_max: declared +45.000 deg, envelope +35.000 deg..+55.000 deg
- rh_hip_min: declared -45.000 deg, envelope -55.000 deg..-35.000 deg
- rh_lower_leg_max: declared +37.500 deg, envelope +27.500 deg..+47.500 deg
- rh_upper_leg_max: declared +122.500 deg, envelope +112.500 deg..+132.500 deg
- rh_upper_leg_min: declared -52.500 deg, envelope -62.500 deg..-42.500 deg

## Unresolved assumptions / UNKNOWN

- lf: NEEDS_HUMAN_DECISION: no seed parking angle in (30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0) for lh resolved the collision; a search over a wider/finer angle range or a different auxiliary leg may be required
- lf_hip_max: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- lf_hip_min: MODEL_LIMIT_MISMATCH, mesh-predicted contact differs from the declared URDF limit by -0.043633 rad; NEEDS_HUMAN_DECISION on whether to revise the URDF limit (the compiler never does this automatically).
- lf_lower_leg_max: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- lf_lower_leg_min: MODEL_LIMIT_MISMATCH, mesh-predicted contact differs from the declared URDF limit by -0.103970 rad; NEEDS_HUMAN_DECISION on whether to revise the URDF limit (the compiler never does this automatically).
- lf_upper_leg_max: UNINTENDED_COLLISION_BEFORE_ENDPOINT against lf_foot_link/lh_foot_link even with the leg's determined parking context; NEEDS_HUMAN_DECISION on whether a different auxiliary pose or a mechanical redesign is required.
- lf_upper_leg_min: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- lh_hip_max: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- lh_hip_min: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- lh_lower_leg_max: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- lh_lower_leg_min: MODEL_LIMIT_MISMATCH, mesh-predicted contact differs from the declared URDF limit by -0.103970 rad; NEEDS_HUMAN_DECISION on whether to revise the URDF limit (the compiler never does this automatically).
- lh_upper_leg_max: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- lh_upper_leg_min: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- manufacturing tolerance UNKNOWN: only the +/-0.15mm per-part print tolerance is modelled; assembly-level stack-up (bushings, screws, servo horn backlash, fit clearances) is not known and is not included in the sensitivity estimate.
- min_clearance_pass_m=3mm ('adequate clearance' threshold for path PASS) is a documented conservative default chosen for this compiler, not a project-mandated value; NEEDS_HUMAN_DECISION if a different bar is intended.
- narrow_phase_margin_m=1mm / grid_cell_size_m=5mm are compiler performance/precision parameters tuned for MATDOG's actual mm-scale collision meshes; they are configurable and reported here rather than hardcoded assumptions, per canonical handoff section 4.
- rf: NEEDS_HUMAN_DECISION: no seed parking angle in (30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0) for rh resolved the collision; a search over a wider/finer angle range or a different auxiliary leg may be required
- rf_hip_max: MODEL_LIMIT_MISMATCH, mesh-predicted contact differs from the declared URDF limit by 0.043633 rad; NEEDS_HUMAN_DECISION on whether to revise the URDF limit (the compiler never does this automatically).
- rf_hip_min: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- rf_lower_leg_max: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- rf_lower_leg_min: MODEL_LIMIT_MISMATCH, mesh-predicted contact differs from the declared URDF limit by -0.104788 rad; NEEDS_HUMAN_DECISION on whether to revise the URDF limit (the compiler never does this automatically).
- rf_upper_leg_max: UNINTENDED_COLLISION_BEFORE_ENDPOINT against rf_foot_link/rh_foot_link even with the leg's determined parking context; NEEDS_HUMAN_DECISION on whether a different auxiliary pose or a mechanical redesign is required.
- rf_upper_leg_min: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- rh_hip_max: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- rh_hip_min: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- rh_lower_leg_max: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- rh_lower_leg_min: MODEL_LIMIT_MISMATCH, mesh-predicted contact differs from the declared URDF limit by -0.104788 rad; NEEDS_HUMAN_DECISION on whether to revise the URDF limit (the compiler never does this automatically).
- rh_upper_leg_max: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.
- rh_upper_leg_min: no mesh contact found within the analysis envelope around the declared URDF limit; that limit likely originates from a constraint this compiler does not model (e.g. servo internal range), not from these collision meshes.

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
