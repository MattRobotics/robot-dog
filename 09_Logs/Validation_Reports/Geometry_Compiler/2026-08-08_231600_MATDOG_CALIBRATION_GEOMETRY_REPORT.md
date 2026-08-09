# MATDOG Calibration Geometry Profile — Phase 1B report

schema_version: `matdog.calibration_geometry_profile.v4`
generation_timestamp_utc: `2026-08-08T23:16:00.075539+00:00`
robot_dog_commit_sha: `66f613034f3a64debc9bc45031eb4283a375e52f`
robot_dog_working_tree_dirty: `True`
content_sha256: `928ff09616a1ad80df029a99afe2061e41a0bedec6bfdc9b3f80f74af4867828`
urdf_sha256: `5e28da3dba10fd3f2ea6ebf6f5d6271157bda0b12b82d92aedbe3031643089ef`

HARDWARE NOT USED. NORMA-CORE NOT MODIFIED. NO COMMIT/PUSH/PR/MERGE.

## Pair policy in force (Phase 1B)

`phase1b_joint_aware_adjacency`, supersedes `phase1_v3_blanket_adjacent_exclusion`.

| concern | rule |
|---|---|
| REVOLUTE adjacent (12 pairs) | INCLUDE in collision analysis |
| FIXED adjacent (4 pairs) | structural attachment -> EXCLUDE from endstop metrology and path collision |
| NON adjacent | unchanged pre-existing clearance/path-safety policy |
| ENDSTOP METROLOGY | active revolute parent-child pair only (one pair per joint) |
| PATH SAFETY | all other relevant pairs; active pair excluded from its own sweep |
| clearance gate applies to | NON_ADJACENT only |

Clearance gate exclusion: REVOLUTE_ADJACENT -- their healthy resting state is sub-mm contact-fit separation (GATE B measured 0.0026-0.0372 mm at q=0), so the generic gate does not apply.

Evidence classes: `HARDWARE_CONFIRMED_CONTACT` (mesh agrees with a real hardware oracle), `HARDWARE_CONTRADICTED`, `GEOMETRIC_ENDPOINT_CANDIDATE` (model prediction awaiting hardware validation -- RF/RH/LH), `PATH_LIMITED`, `NO_MODELED_CONTACT`. An adjacent contact is never auto-promoted to "hardware hardstop".

## LF V25 hardware reconciliation (read before the 24-endpoint table)

| Endpoint | Declared URDF | Hardware contact | Compiler mesh contact | Compiler contact pair | hw vs URDF | mesh vs hw | Contact model status | Corrective action |
|---|---:|---:|---:|---|---|---|---|---|
| lf_hip_max | +45.000 deg | +39.375 deg | +45.230 deg | base_link <-> lf_hip_link | INCOMPATIBLE | DISAGREES | **MODEL_INCOMPLETE** | none automatic: real stopping mechanism is not represented in the collision STL (servo/bracket internal limit, or a mesh contact that does not correspond to the real hardware contact); NEEDS_HUMAN_DECISION if a mesh model of that feature should be added |
| lf_hip_min | -45.000 deg | -42.803 deg | -46.012 deg | base_link <-> lf_hip_link | INCOMPATIBLE | DISAGREES | **MODEL_INCOMPLETE** | none automatic: real stopping mechanism is not represented in the collision STL (servo/bracket internal limit, or a mesh contact that does not correspond to the real hardware contact); NEEDS_HUMAN_DECISION if a mesh model of that feature should be added |
| lf_lower_leg_max | +37.500 deg | +34.277 deg | +38.180 deg | lf_lower_leg_link <-> lf_upper_leg_link | INCOMPATIBLE | DISAGREES | **MODEL_INCOMPLETE** | none automatic: real stopping mechanism is not represented in the collision STL (servo/bracket internal limit, or a mesh contact that does not correspond to the real hardware contact); NEEDS_HUMAN_DECISION if a mesh model of that feature should be added |
| lf_lower_leg_min | -92.000 deg | -91.846 deg | -92.070 deg | lf_lower_leg_link <-> lf_upper_leg_link | COMPATIBLE | AGREES | **MODELED_ENDSTOP_CONTACT** | none: mesh contact corresponds to the real hardware contact within tolerance |
| lf_upper_leg_max | +122.500 deg | +122.607 deg | +121.875 deg | lf_hip_link <-> lf_upper_leg_link | COMPATIBLE | AGREES | **MODELED_ENDSTOP_CONTACT** | none: mesh contact corresponds to the real hardware contact within tolerance |
| lf_upper_leg_min | -52.500 deg | -53.525 deg | -52.039 deg | lf_hip_link <-> lf_upper_leg_link | COMPATIBLE | AGREES | **MODELED_ENDSTOP_CONTACT** | none: mesh contact corresponds to the real hardware contact within tolerance |

## 24 endpoints

| Leg | Joint | Side | Declared URDF | Mesh predicted contact | Delta | Contact pair | Clearance before contact | Contact model status | Evidence class | Path collision (if any) |
|---|---|---|---:|---:|---:|---|---:|---|---|---|
| LF | hip | max | +45.000 deg | +45.230 deg | +0.230 deg | base_link <-> lf_hip_link | - | **MODEL_INCOMPLETE** | HARDWARE_CONTRADICTED | - |
| LF | hip | min | -45.000 deg | -46.012 deg | -1.012 deg | base_link <-> lf_hip_link | - | **MODEL_INCOMPLETE** | HARDWARE_CONTRADICTED | base_link <-> lf_upper_leg_link @ -47.500 deg |
| LF | lower_leg | max | +37.500 deg | +38.180 deg | +0.680 deg | lf_lower_leg_link <-> lf_upper_leg_link | - | **MODEL_INCOMPLETE** | HARDWARE_CONTRADICTED | - |
| LF | lower_leg | min | -92.000 deg | -92.070 deg | -0.070 deg | lf_lower_leg_link <-> lf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | HARDWARE_CONFIRMED_CONTACT | lf_foot_link <-> lf_upper_leg_link @ -97.957 deg |
| LF | upper_leg | max | +122.500 deg | +121.875 deg | -0.625 deg | lf_hip_link <-> lf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | HARDWARE_CONFIRMED_CONTACT | - |
| LF | upper_leg | min | -52.500 deg | -52.039 deg | +0.461 deg | lf_hip_link <-> lf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | HARDWARE_CONFIRMED_CONTACT | - |
| LH | hip | max | +45.000 deg | +45.156 deg | +0.156 deg | base_link <-> lh_hip_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| LH | hip | min | -45.000 deg | -46.012 deg | -1.012 deg | base_link <-> lh_hip_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| LH | lower_leg | max | +37.500 deg | +38.180 deg | +0.680 deg | lh_lower_leg_link <-> lh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| LH | lower_leg | min | -92.000 deg | -92.070 deg | -0.070 deg | lh_lower_leg_link <-> lh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | lh_foot_link <-> lh_upper_leg_link @ -97.957 deg |
| LH | upper_leg | max | +122.500 deg | +121.875 deg | -0.625 deg | lh_hip_link <-> lh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| LH | upper_leg | min | -52.500 deg | -52.039 deg | +0.461 deg | lh_hip_link <-> lh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RF | hip | max | +45.000 deg | +46.012 deg | +1.012 deg | base_link <-> rf_hip_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | base_link <-> rf_upper_leg_link @ +47.500 deg |
| RF | hip | min | -45.000 deg | -45.230 deg | -0.230 deg | base_link <-> rf_hip_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RF | lower_leg | max | +37.500 deg | +37.547 deg | +0.047 deg | rf_lower_leg_link <-> rf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RF | lower_leg | min | -92.000 deg | -92.074 deg | -0.074 deg | rf_lower_leg_link <-> rf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | rf_foot_link <-> rf_upper_leg_link @ -98.004 deg |
| RF | upper_leg | max | +122.500 deg | +121.875 deg | -0.625 deg | rf_hip_link <-> rf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RF | upper_leg | min | -52.500 deg | -52.039 deg | +0.461 deg | rf_hip_link <-> rf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RH | hip | max | +45.000 deg | +46.012 deg | +1.012 deg | base_link <-> rh_hip_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RH | hip | min | -45.000 deg | -45.156 deg | -0.156 deg | base_link <-> rh_hip_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RH | lower_leg | max | +37.500 deg | +37.547 deg | +0.047 deg | rh_lower_leg_link <-> rh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RH | lower_leg | min | -92.000 deg | -92.074 deg | -0.074 deg | rh_lower_leg_link <-> rh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | rh_foot_link <-> rh_upper_leg_link @ -98.004 deg |
| RH | upper_leg | max | +122.500 deg | +121.875 deg | -0.625 deg | rh_hip_link <-> rh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RH | upper_leg | min | -52.500 deg | -52.039 deg | +0.461 deg | rh_hip_link <-> rh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |

## Mirror comparisons

### LF vs RF

| Joint | Side | LF contact | RF contact | Delta |
|---|---|---:|---:|---:|
| hip | min | -46.012 deg (MODEL_INCOMPLETE) | -45.230 deg (MODELED_ENDSTOP_CONTACT) | -0.781 deg |
| hip | max | +45.230 deg (MODEL_INCOMPLETE) | +46.012 deg (MODELED_ENDSTOP_CONTACT) | -0.781 deg |
| upper_leg | min | -52.039 deg (MODELED_ENDSTOP_CONTACT) | -52.039 deg (MODELED_ENDSTOP_CONTACT) | +0.000 deg |
| upper_leg | max | +121.875 deg (MODELED_ENDSTOP_CONTACT) | +121.875 deg (MODELED_ENDSTOP_CONTACT) | +0.000 deg |
| lower_leg | min | -92.070 deg (MODELED_ENDSTOP_CONTACT) | -92.074 deg (MODELED_ENDSTOP_CONTACT) | +0.004 deg |
| lower_leg | max | +38.180 deg (MODEL_INCOMPLETE) | +37.547 deg (MODELED_ENDSTOP_CONTACT) | +0.633 deg |

### RH vs LH

| Joint | Side | RH contact | LH contact | Delta |
|---|---|---:|---:|---:|
| hip | min | -45.156 deg (MODELED_ENDSTOP_CONTACT) | -46.012 deg (MODELED_ENDSTOP_CONTACT) | +0.855 deg |
| hip | max | +46.012 deg (MODELED_ENDSTOP_CONTACT) | +45.156 deg (MODELED_ENDSTOP_CONTACT) | +0.855 deg |
| upper_leg | min | -52.039 deg (MODELED_ENDSTOP_CONTACT) | -52.039 deg (MODELED_ENDSTOP_CONTACT) | +0.000 deg |
| upper_leg | max | +121.875 deg (MODELED_ENDSTOP_CONTACT) | +121.875 deg (MODELED_ENDSTOP_CONTACT) | +0.000 deg |
| lower_leg | min | -92.074 deg (MODELED_ENDSTOP_CONTACT) | -92.070 deg (MODELED_ENDSTOP_CONTACT) | -0.004 deg |
| lower_leg | max | +37.547 deg (MODELED_ENDSTOP_CONTACT) | +38.180 deg (MODELED_ENDSTOP_CONTACT) | -0.633 deg |

### FRONT vs HIND

| Joint | Side | FRONT (LF) | HIND (RH) | Note |
|---|---|---:|---:|---|
| hip | min | -46.012 deg (MODEL_INCOMPLETE) | -45.156 deg (MODELED_ENDSTOP_CONTACT) | DIFFERENT status |
| hip | max | +45.230 deg (MODEL_INCOMPLETE) | +46.012 deg (MODELED_ENDSTOP_CONTACT) | DIFFERENT status |
| upper_leg | min | -52.039 deg (MODELED_ENDSTOP_CONTACT) | -52.039 deg (MODELED_ENDSTOP_CONTACT) | same status |
| upper_leg | max | +121.875 deg (MODELED_ENDSTOP_CONTACT) | +121.875 deg (MODELED_ENDSTOP_CONTACT) | same status |
| lower_leg | min | -92.070 deg (MODELED_ENDSTOP_CONTACT) | -92.074 deg (MODELED_ENDSTOP_CONTACT) | same status |
| lower_leg | max | +38.180 deg (MODEL_INCOMPLETE) | +37.547 deg (MODELED_ENDSTOP_CONTACT) | DIFFERENT status |

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

(none)

## PATH_COLLISION_BEFORE_ENDPOINT (cross-leg path obstructions)

(none)

## MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY (hardware disagrees, real stop not in collision STL)

- lf_hip_max: active revolute pair contact at 45.23 deg does NOT match the LF V25 hardware contact at 39.38 deg (delta +5.86 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint
- lf_hip_min: active revolute pair contact at -46.01 deg does NOT match the LF V25 hardware contact at -42.80 deg (delta -3.21 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint
- lf_lower_leg_max: active revolute pair contact at 38.18 deg does NOT match the LF V25 hardware contact at 34.28 deg (delta +3.90 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint

## UNINTENDED_SELF_COLLISION

(none)

## NO_MODELED_ENDSTOP (no mesh contact found in envelope)

(none)

## Unresolved assumptions / UNKNOWN

- lf_hip_max: MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY -- active revolute pair contact at 45.23 deg does NOT match the LF V25 hardware contact at 39.38 deg (delta +5.86 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint. NEEDS_HUMAN_DECISION: the real mechanical endstop is not represented in the current collision STL geometry.
- lf_hip_min: MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY -- active revolute pair contact at -46.01 deg does NOT match the LF V25 hardware contact at -42.80 deg (delta -3.21 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint. NEEDS_HUMAN_DECISION: the real mechanical endstop is not represented in the current collision STL geometry.
- lf_lower_leg_max: MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY -- active revolute pair contact at 38.18 deg does NOT match the LF V25 hardware contact at 34.28 deg (delta +3.90 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint. NEEDS_HUMAN_DECISION: the real mechanical endstop is not represented in the current collision STL geometry.
- manufacturing tolerance UNKNOWN: only the +/-0.15mm per-part print tolerance is modelled; assembly-level stack-up (bushings, screws, servo horn backlash, fit clearances) is not known and is not included in the sensitivity estimate.
- min_clearance_pass_m=3mm ('adequate clearance' threshold for path PASS) is a documented conservative default chosen for this compiler, not a project-mandated value; NEEDS_HUMAN_DECISION if a different bar is intended.
- narrow_phase_margin_m=1mm / grid_cell_size_m=5mm are compiler performance/precision parameters tuned for MATDOG's actual mm-scale collision meshes; they are configurable and reported here rather than hardcoded assumptions, per canonical handoff section 4.

## Collision mesh manifest

| Link | SHA256 | Triangles | Degenerate dropped |
|---|---|---:|---:|
| base_link | `644a83e98fd116f3fc8e5d8792ca2b60b0bdb09bcafa4fc49140d641079e4b6b` | 153080 | 0 |
| lf_foot_link | `e43737c4cbc5d618431ecf2ce9ba20a4803f5a7b7f800974a6d8b2c7534091bc` | 9410 | 0 |
| lf_hip_link | `7d28351aba669cc1a8e11ad60b89db001f9689b728e4e8eb05849af69e1e4879` | 22992 | 0 |
| lf_lower_leg_link | `97f2bb775768fdbcef59638916eaef2cd34b21bf01e6197ef3900b0df85d7ae2` | 18096 | 0 |
| lf_upper_leg_link | `3c484b110a622274d1f8446b30329b8e16a0ff542d25da2a39334d41e8f4f169` | 29238 | 0 |
| lh_foot_link | `e43737c4cbc5d618431ecf2ce9ba20a4803f5a7b7f800974a6d8b2c7534091bc` | 9410 | 0 |
| lh_hip_link | `543f6f04a4dc45a0b176eba8db8ab7819ab20a328aa4f7fd9c9d4b77c1612094` | 22992 | 0 |
| lh_lower_leg_link | `97f2bb775768fdbcef59638916eaef2cd34b21bf01e6197ef3900b0df85d7ae2` | 18096 | 0 |
| lh_upper_leg_link | `3c484b110a622274d1f8446b30329b8e16a0ff542d25da2a39334d41e8f4f169` | 29238 | 0 |
| rf_foot_link | `e43737c4cbc5d618431ecf2ce9ba20a4803f5a7b7f800974a6d8b2c7534091bc` | 9410 | 0 |
| rf_hip_link | `ebd664b3db2065ac3341184a6b12700caca6cdece57adae72f935882ed9df4c0` | 22992 | 0 |
| rf_lower_leg_link | `c52ae662c09455f53db5bc8efc13332919942c7f938ada5a8c0e987f54ac3bc8` | 18094 | 0 |
| rf_upper_leg_link | `08fab5e3229280f21a52c6cfbfdab32f1e8345aae6e10648360ae30fe1e06b16` | 29238 | 0 |
| rh_foot_link | `e43737c4cbc5d618431ecf2ce9ba20a4803f5a7b7f800974a6d8b2c7534091bc` | 9410 | 0 |
| rh_hip_link | `2e9f31952e51b5748546c803c06d72316a3a776226f1f72775c1e8d383e1f325` | 22992 | 0 |
| rh_lower_leg_link | `c52ae662c09455f53db5bc8efc13332919942c7f938ada5a8c0e987f54ac3bc8` | 18094 | 0 |
| rh_upper_leg_link | `08fab5e3229280f21a52c6cfbfdab32f1e8345aae6e10648360ae30fe1e06b16` | 29238 | 0 |
