# MATDOG Calibration Geometry Profile — Phase 1B report

schema_version: `matdog.calibration_geometry_profile.v4`
generation_timestamp_utc: `2026-08-10T16:58:12.940039+00:00`
robot_dog_commit_sha: `e71876e80c23c370f9fecf36ddf15f152faf5eb3`
robot_dog_working_tree_dirty: `True`
content_sha256: `4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61`
urdf_sha256: `3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59`

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
| lf_hip_max | +45.000 deg | +39.375 deg | +45.223 deg | base_link <-> lf_hip_link | INCOMPATIBLE | DISAGREES | **MODEL_INCOMPLETE** | none automatic: real stopping mechanism is not represented in the collision STL (servo/bracket internal limit, or a mesh contact that does not correspond to the real hardware contact); NEEDS_HUMAN_DECISION if a mesh model of that feature should be added |
| lf_hip_min | -45.000 deg | -42.803 deg | -46.012 deg | base_link <-> lf_hip_link | INCOMPATIBLE | DISAGREES | **PATH_COLLISION_BEFORE_ENDPOINT** | none for this endpoint's own limit: resolve/park the path obstruction separately (see parking_plans), then re-evaluate |
| lf_lower_leg_max | +37.500 deg | +34.277 deg | +38.180 deg | lf_lower_leg_link <-> lf_upper_leg_link | INCOMPATIBLE | DISAGREES | **MODEL_INCOMPLETE** | none automatic: real stopping mechanism is not represented in the collision STL (servo/bracket internal limit, or a mesh contact that does not correspond to the real hardware contact); NEEDS_HUMAN_DECISION if a mesh model of that feature should be added |
| lf_lower_leg_min | -92.000 deg | -91.846 deg | -92.074 deg | lf_lower_leg_link <-> lf_upper_leg_link | COMPATIBLE | AGREES | **MODELED_ENDSTOP_CONTACT** | none: mesh contact corresponds to the real hardware contact within tolerance |
| lf_upper_leg_max | +122.500 deg | +122.607 deg | +121.875 deg | lf_hip_link <-> lf_upper_leg_link | COMPATIBLE | AGREES | **MODELED_ENDSTOP_CONTACT** | none: mesh contact corresponds to the real hardware contact within tolerance |
| lf_upper_leg_min | -52.500 deg | -53.525 deg | -52.133 deg | lf_hip_link <-> lf_upper_leg_link | COMPATIBLE | AGREES | **MODELED_ENDSTOP_CONTACT** | none: mesh contact corresponds to the real hardware contact within tolerance |

## 24 endpoints

| Leg | Joint | Side | Declared URDF | Mesh predicted contact | Delta | Contact pair | Clearance before contact | Contact model status | Evidence class | Path collision (if any) |
|---|---|---|---:|---:|---:|---|---:|---|---|---|
| LF | hip | max | +45.000 deg | +45.223 deg | +0.223 deg | base_link <-> lf_hip_link | - | **MODEL_INCOMPLETE** | HARDWARE_CONTRADICTED | - |
| LF | hip | min | -45.000 deg | -46.012 deg | -1.012 deg | base_link <-> lf_hip_link | - | **PATH_COLLISION_BEFORE_ENDPOINT** | PATH_LIMITED | base_link <-> lf_lower_leg_link @ -45.297 deg |
| LF | lower_leg | max | +37.500 deg | +38.180 deg | +0.680 deg | lf_lower_leg_link <-> lf_upper_leg_link | - | **MODEL_INCOMPLETE** | HARDWARE_CONTRADICTED | - |
| LF | lower_leg | min | -92.000 deg | -92.074 deg | -0.074 deg | lf_lower_leg_link <-> lf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | HARDWARE_CONFIRMED_CONTACT | lf_foot_link <-> lf_upper_leg_link @ -97.969 deg |
| LF | upper_leg | max | +122.500 deg | +121.875 deg | -0.625 deg | lf_hip_link <-> lf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | HARDWARE_CONFIRMED_CONTACT | - |
| LF | upper_leg | min | -52.500 deg | -52.133 deg | +0.367 deg | lf_hip_link <-> lf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | HARDWARE_CONFIRMED_CONTACT | - |
| LH | hip | max | +45.000 deg | +45.156 deg | +0.156 deg | base_link <-> lh_hip_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| LH | hip | min | -45.000 deg | -46.012 deg | -1.012 deg | base_link <-> lh_hip_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| LH | lower_leg | max | +37.500 deg | +38.180 deg | +0.680 deg | lh_lower_leg_link <-> lh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| LH | lower_leg | min | -92.000 deg | -92.074 deg | -0.074 deg | lh_lower_leg_link <-> lh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | lh_foot_link <-> lh_upper_leg_link @ -97.969 deg |
| LH | upper_leg | max | +122.500 deg | +121.875 deg | -0.625 deg | lh_hip_link <-> lh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| LH | upper_leg | min | -52.500 deg | -52.133 deg | +0.367 deg | lh_hip_link <-> lh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RF | hip | max | +45.000 deg | +46.012 deg | +1.012 deg | base_link <-> rf_hip_link | - | **PATH_COLLISION_BEFORE_ENDPOINT** | PATH_LIMITED | base_link <-> rf_lower_leg_link @ +45.297 deg |
| RF | hip | min | -45.000 deg | -45.223 deg | -0.223 deg | base_link <-> rf_hip_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RF | lower_leg | max | +37.500 deg | +38.180 deg | +0.680 deg | rf_lower_leg_link <-> rf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RF | lower_leg | min | -92.000 deg | -92.074 deg | -0.074 deg | rf_lower_leg_link <-> rf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | rf_foot_link <-> rf_upper_leg_link @ -97.973 deg |
| RF | upper_leg | max | +122.500 deg | +121.875 deg | -0.625 deg | rf_hip_link <-> rf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RF | upper_leg | min | -52.500 deg | -52.133 deg | +0.367 deg | rf_hip_link <-> rf_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RH | hip | max | +45.000 deg | +46.012 deg | +1.012 deg | base_link <-> rh_hip_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RH | hip | min | -45.000 deg | -45.156 deg | -0.156 deg | base_link <-> rh_hip_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RH | lower_leg | max | +37.500 deg | +38.180 deg | +0.680 deg | rh_lower_leg_link <-> rh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RH | lower_leg | min | -92.000 deg | -92.074 deg | -0.074 deg | rh_lower_leg_link <-> rh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | rh_foot_link <-> rh_upper_leg_link @ -97.973 deg |
| RH | upper_leg | max | +122.500 deg | +121.875 deg | -0.625 deg | rh_hip_link <-> rh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |
| RH | upper_leg | min | -52.500 deg | -52.133 deg | +0.367 deg | rh_hip_link <-> rh_upper_leg_link | - | **MODELED_ENDSTOP_CONTACT** | GEOMETRIC_ENDPOINT_CANDIDATE | - |

## Mirror comparisons

### LF vs RF

| Joint | Side | LF contact | RF contact | Delta |
|---|---|---:|---:|---:|
| hip | min | -46.012 deg (PATH_COLLISION_BEFORE_ENDPOINT) | -45.223 deg (MODELED_ENDSTOP_CONTACT) | -0.789 deg |
| hip | max | +45.223 deg (MODEL_INCOMPLETE) | +46.012 deg (PATH_COLLISION_BEFORE_ENDPOINT) | -0.789 deg |
| upper_leg | min | -52.133 deg (MODELED_ENDSTOP_CONTACT) | -52.133 deg (MODELED_ENDSTOP_CONTACT) | +0.000 deg |
| upper_leg | max | +121.875 deg (MODELED_ENDSTOP_CONTACT) | +121.875 deg (MODELED_ENDSTOP_CONTACT) | +0.000 deg |
| lower_leg | min | -92.074 deg (MODELED_ENDSTOP_CONTACT) | -92.074 deg (MODELED_ENDSTOP_CONTACT) | +0.000 deg |
| lower_leg | max | +38.180 deg (MODEL_INCOMPLETE) | +38.180 deg (MODELED_ENDSTOP_CONTACT) | +0.000 deg |

### RH vs LH

| Joint | Side | RH contact | LH contact | Delta |
|---|---|---:|---:|---:|
| hip | min | -45.156 deg (MODELED_ENDSTOP_CONTACT) | -46.012 deg (MODELED_ENDSTOP_CONTACT) | +0.855 deg |
| hip | max | +46.012 deg (MODELED_ENDSTOP_CONTACT) | +45.156 deg (MODELED_ENDSTOP_CONTACT) | +0.855 deg |
| upper_leg | min | -52.133 deg (MODELED_ENDSTOP_CONTACT) | -52.133 deg (MODELED_ENDSTOP_CONTACT) | +0.000 deg |
| upper_leg | max | +121.875 deg (MODELED_ENDSTOP_CONTACT) | +121.875 deg (MODELED_ENDSTOP_CONTACT) | +0.000 deg |
| lower_leg | min | -92.074 deg (MODELED_ENDSTOP_CONTACT) | -92.074 deg (MODELED_ENDSTOP_CONTACT) | +0.000 deg |
| lower_leg | max | +38.180 deg (MODELED_ENDSTOP_CONTACT) | +38.180 deg (MODELED_ENDSTOP_CONTACT) | +0.000 deg |

### FRONT vs HIND

| Joint | Side | FRONT (LF) | HIND (RH) | Note |
|---|---|---:|---:|---|
| hip | min | -46.012 deg (PATH_COLLISION_BEFORE_ENDPOINT) | -45.156 deg (MODELED_ENDSTOP_CONTACT) | DIFFERENT status |
| hip | max | +45.223 deg (MODEL_INCOMPLETE) | +46.012 deg (MODELED_ENDSTOP_CONTACT) | DIFFERENT status |
| upper_leg | min | -52.133 deg (MODELED_ENDSTOP_CONTACT) | -52.133 deg (MODELED_ENDSTOP_CONTACT) | same status |
| upper_leg | max | +121.875 deg (MODELED_ENDSTOP_CONTACT) | +121.875 deg (MODELED_ENDSTOP_CONTACT) | same status |
| lower_leg | min | -92.074 deg (MODELED_ENDSTOP_CONTACT) | -92.074 deg (MODELED_ENDSTOP_CONTACT) | same status |
| lower_leg | max | +38.180 deg (MODEL_INCOMPLETE) | +38.180 deg (MODELED_ENDSTOP_CONTACT) | DIFFERENT status |

Front hip Z ~= 0.0465 m, hind hip Z ~= 0.0265 m (20 mm difference, verified numerically in tests/test_matdog_geometry_scene.py); this does not change detector physics but can change prerequisite/parking/path clearance, per canonical handoff section 4.

## Parking

| Leg | Auxiliary parking | Parked leg | Angle | Reason |
|---|---|---|---:|---|
| LF | REQUIRED | lh | +30.000 deg | segment(s) ['upper probe positive (home prerequisite)', 'upper probe return to home', 'transition HIP prerequisite -> LOWER prerequisite', 'restore leg to home'] collide against lh at home (true mesh intersection); minimal seed parking 30 deg for lh, held for the whole lf sequence (single park-before/restore-after, matching the validated 2026-07-20 checkpoint and LF V25 hardware practice), resolves every true collision; residual NEEDS_HUMAN_DECISION margin finding (not a collision): minimum modelled clearance 0.1014 mm at 'hip probe return' is below the configured PASS bar (3.0000 mm); this is an UNRESOLVED_FOR_THRESHOLD lower bound (search margin, not a proven small gap) |
| LH | NOT REQUIRED | - | - | no true mesh collision in any segment with every other leg at home; NEEDS_HUMAN_DECISION margin finding (not a collision, parking not applicable): minimum modelled clearance 1.0000 mm at 'lower probe return' is below the configured PASS bar (3.0000 mm); this is an UNRESOLVED_FOR_THRESHOLD lower bound (search margin, not a proven small gap) |
| RF | REQUIRED | rh | +30.000 deg | segment(s) ['upper probe positive (home prerequisite)', 'upper probe return to home', 'transition HIP prerequisite -> LOWER prerequisite', 'restore leg to home'] collide against rh at home (true mesh intersection); minimal seed parking 30 deg for rh, held for the whole rf sequence (single park-before/restore-after, matching the validated 2026-07-20 checkpoint and LF V25 hardware practice), resolves every true collision; residual NEEDS_HUMAN_DECISION margin finding (not a collision): minimum modelled clearance 0.1014 mm at 'hip probe return' is below the configured PASS bar (3.0000 mm); this is an UNRESOLVED_FOR_THRESHOLD lower bound (search margin, not a proven small gap) |
| RH | NOT REQUIRED | - | - | no true mesh collision in any segment with every other leg at home; NEEDS_HUMAN_DECISION margin finding (not a collision, parking not applicable): minimum modelled clearance 1.0000 mm at 'lower probe return' is below the configured PASS bar (3.0000 mm); this is an UNRESOLVED_FOR_THRESHOLD lower bound (search margin, not a proven small gap) |

## Minimum modelled clearance per leg sequence

| Leg | Sequence PASS | Minimum modelled clearance |
|---|---|---:|
| LF | False | 0.1014 mm |
| LH | False | 1.0000 mm |
| RF | False | 0.1014 mm |
| RH | False | 1.0000 mm |

## MODEL_LIMIT_MISMATCH

(none)

## PATH_COLLISION_BEFORE_ENDPOINT (cross-leg path obstructions)

- lf_hip_min: base_link <-> lf_lower_leg_link at -45.297 deg -- path obstruction found at -45.30 deg, before this joint's active revolute pair makes contact in the envelope (or it never does); this is a path/parking prerequisite, not this joint's own designed limit
- rf_hip_max: base_link <-> rf_lower_leg_link at +45.297 deg -- path obstruction found at 45.30 deg, before this joint's active revolute pair makes contact in the envelope (or it never does); this is a path/parking prerequisite, not this joint's own designed limit

## MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY (hardware disagrees, real stop not in collision STL)

- lf_hip_max: active revolute pair contact at 45.22 deg does NOT match the LF V25 hardware contact at 39.38 deg (delta +5.85 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint
- lf_lower_leg_max: active revolute pair contact at 38.18 deg does NOT match the LF V25 hardware contact at 34.28 deg (delta +3.90 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint

## UNINTENDED_SELF_COLLISION

(none)

## NO_MODELED_ENDSTOP (no mesh contact found in envelope)

(none)

## Unresolved assumptions / UNKNOWN

- lf_hip_max: MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY -- active revolute pair contact at 45.22 deg does NOT match the LF V25 hardware contact at 39.38 deg (delta +5.85 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint. NEEDS_HUMAN_DECISION: the real mechanical endstop is not represented in the current collision STL geometry.
- lf_hip_min: PATH_COLLISION_BEFORE_ENDPOINT against base_link/lf_lower_leg_link even with the leg's determined parking context; this joint's own designed limit could not be searched past this obstruction. NEEDS_HUMAN_DECISION on whether a different auxiliary pose or a mechanical redesign is required.
- lf_lower_leg_max: MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY -- active revolute pair contact at 38.18 deg does NOT match the LF V25 hardware contact at 34.28 deg (delta +3.90 deg) -- this mesh event is not the real mechanical endstop; recorded as a diagnostic collision only, not the endpoint. NEEDS_HUMAN_DECISION: the real mechanical endstop is not represented in the current collision STL geometry.
- manufacturing tolerance UNKNOWN: only the +/-0.15mm per-part print tolerance is modelled; assembly-level stack-up (bushings, screws, servo horn backlash, fit clearances) is not known and is not included in the sensitivity estimate.
- min_clearance_pass_m=3mm ('adequate clearance' threshold for path PASS) is a documented conservative default chosen for this compiler, not a project-mandated value; NEEDS_HUMAN_DECISION if a different bar is intended.
- narrow_phase_margin_m=1mm / grid_cell_size_m=5mm are compiler performance/precision parameters tuned for MATDOG's actual mm-scale collision meshes; they are configurable and reported here rather than hardcoded assumptions, per canonical handoff section 4.
- rf_hip_max: PATH_COLLISION_BEFORE_ENDPOINT against base_link/rf_lower_leg_link even with the leg's determined parking context; this joint's own designed limit could not be searched past this obstruction. NEEDS_HUMAN_DECISION on whether a different auxiliary pose or a mechanical redesign is required.

## Collision mesh manifest

| Link | SHA256 | Triangles | Degenerate dropped |
|---|---|---:|---:|
| base_link | `0a485e7a1101d457f317b664e52a7a4ef061382e6ecc2c7c87a333c81d5b466f` | 22044 | 0 |
| lf_foot_link | `e4d35cd4107bd7fad2d3fe511926ec4fc5211f15f978581e469787266ebe5357` | 2248 | 0 |
| lf_hip_link | `b2e430aa99998791879f1ab80c46787a310c80f77a3df9116d1084e0a0e27d97` | 6016 | 0 |
| lf_lower_leg_link | `bbb40400571fab554395d5671ad86b1b106ec3fe24fce733ef0be179a739508a` | 12336 | 0 |
| lf_upper_leg_link | `0830fc10d8873b6a45f2f58c2ca61f08a7f38092984a96b911ac6beed274c30c` | 3802 | 0 |
| lh_foot_link | `e4d35cd4107bd7fad2d3fe511926ec4fc5211f15f978581e469787266ebe5357` | 2248 | 0 |
| lh_hip_link | `5350010e3623ec36301d2c0592e3209e1b06da83e823215caf55cca5aafe86da` | 6018 | 0 |
| lh_lower_leg_link | `bbb40400571fab554395d5671ad86b1b106ec3fe24fce733ef0be179a739508a` | 12336 | 0 |
| lh_upper_leg_link | `0830fc10d8873b6a45f2f58c2ca61f08a7f38092984a96b911ac6beed274c30c` | 3802 | 0 |
| rf_foot_link | `18dfa7908e7410cc2920d5d37b0c0cce13cb2c683f765e7d073b3d5777c00d33` | 2248 | 0 |
| rf_hip_link | `83a7622f990228e6c2b6bb0431e682fc462a76feb3ce2fe190b7553be879dc33` | 6016 | 0 |
| rf_lower_leg_link | `6b40148edad22670c420d1bcca2325c8befd72e27f4390569a292250df3e96c8` | 12352 | 0 |
| rf_upper_leg_link | `fbf43f047a943188a6c7b9a7a3a45763c22ddb35704a517b7b47b03dcb68cb91` | 3806 | 0 |
| rh_foot_link | `18dfa7908e7410cc2920d5d37b0c0cce13cb2c683f765e7d073b3d5777c00d33` | 2248 | 0 |
| rh_hip_link | `04b205e9be3683b5238a035970b56ee319e4e7fca55ba7860a19fdb549c4efea` | 6018 | 0 |
| rh_lower_leg_link | `6b40148edad22670c420d1bcca2325c8befd72e27f4390569a292250df3e96c8` | 12352 | 0 |
| rh_upper_leg_link | `fbf43f047a943188a6c7b9a7a3a45763c22ddb35704a517b7b47b03dcb68cb91` | 3806 | 0 |
