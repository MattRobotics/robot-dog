# MATDOG Geometry Compiler V5 — architecture and schema migration

**Date:** 2026-08-10<br>
**Branch:** `matdog/geometry-compiler-v5-collision-baseline`<br>
**Status:** G5 model audit PASS; normative G6 mapping approved before V5 serialization<br>
**Historical provenance:** schema v1/v2/v3/v4 and frozen G4 artifacts remain immutable

Frozen G4 content SHA256:

```text
4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61
```

## G5 model-first hard gate

The current URDF unambiguously selects the intended actuators using:

```text
joint.type == revolute
AND exactly one hardware/motorId
AND exactly one hardware/motorDirection in {-1,+1}
```

Verified: 17 unique links, 16 unique joints, one root, 12 bounded revolute
joints with complete and unique motor metadata, four fixed terminal joints,
and four topology-derived independent branches of three actuators. Names such
as `lf`, `hip`, `upper` and `lower` are presentation labels only. No new URDF
metadata is required for REV00; the metadata STOP condition does not apply.

## Approved additive V5 architecture

```text
matdog_geometry_model_v5.py
  URDF topology, joints, limits, axes, motor metadata
  collision filenames, per-mesh scales and collision origins
  topology-derived branches and pair relations

matdog_geometry_scene_v5.py
  pure FK/collision scene over the model
  world_from_collision = world_from_link @ link_from_collision_origin
  validated Phase1B mesh kernel reused without semantic changes

matdog_geometry_contact_search_v5.py
  pure active-pair endpoint search
  independent PATH_OBSTRUCTION search
  explicit context pose; no mandatory legacy prerequisite

matdog_geometry_profile_v5.py / matdog_geometry_report_v5.py
  pure geometry serialization and reporting

matdog_geometry_path_planner_v5.py
  topology-driven geometric path/parking search

matdog_geometry_safety_policy.py
  external policy consumer; historical 3 mm lives only here

matdog_geometry_hardware_reconciler.py
  separate consumer of immutable LF evidence and V5 geometry

matdog_geometry_compiler_v5.py
  composition root, process workers, parent-only serialization
```

The seven Phase1B sources pinned by G4 are not modified. V5 is additive.

```text
geometric endpoint contact
!= path obstruction / reachability
!= hardware evidence
!= safety-policy acceptance
```

Path relation metadata is topology-derived and independent of the outcome:
`same_branch`, `body_vs_branch`, `cross_branch`, or `body_internal`.

## Schema decision

The canonical pure-geometry artifact is:

```text
matdog.calibration_geometry_profile.v5
```

This is a semantic change, not a milestone-only bump: v4 embeds LF hardware
and the historical 3 mm policy. V5 removes them while preserving v4 exactly.

Top-level V5 shape:

```text
schema_version
semantic_content_sha256
generation_metadata                 # excluded from semantic hash
provenance
model
geometry_pair_policy
analysis_parameters
endpoint_searches
path_plans                           # added by G9, raw geometry only
geometry_unknowns
```

Separate artifacts consume the geometry profile:

```text
V5 geometry + LF evidence -> Hardware Reconciliation report
V5 geometry + policy      -> Safety Policy report
```

## Normative v4 → V5 mapping

Every relevant v4 field has exactly one migration class.

### Top level and provenance

| v4 field | Class | V5 destination |
|---|---|---|
| `schema_version` | KEEP | same field, value `.v5` |
| `generation_timestamp_utc` | RESTRUCTURE | `generation_metadata.generated_at_utc`, non-semantic |
| `robot_dog_commit_sha` | RESTRUCTURE | `provenance.repository.commit_sha` |
| `robot_dog_working_tree_dirty` | RESTRUCTURE | `generation_metadata.working_tree_dirty`, non-semantic |
| `urdf` | RESTRUCTURE | `provenance.urdf`, actual input path |
| `collision_mesh_manifest` | RESTRUCTURE | `provenance.collision_meshes`, with URDF transforms |
| `pair_policy` | RESTRUCTURE | pure `geometry_pair_policy` only |
| `geometry_compiler` | RESTRUCTURE | `provenance.geometry_compiler` |
| `numerical_parameters` | RESTRUCTURE | geometry-only `analysis_parameters` |
| `manufacturing_tolerance` | REMOVE FROM PURE GEOMETRY | optional external tolerance artifact |
| `endpoints` | RESTRUCTURE | `endpoint_searches` |
| `parking_plans` | RESTRUCTURE | topology-driven raw path plans |
| `lf_v25_hardware_reconciliation` | MOVE TO HARDWARE RECONCILER | entire block |
| `unresolved_assumptions` | RESTRUCTURE | layer-owned structured unknowns |
| `content_sha256` | RENAME | `semantic_content_sha256` |

### URDF and collision meshes

| v4 field | Class | V5 destination |
|---|---|---|
| `urdf.relative_path` | KEEP | actual input path, not a required constant |
| `urdf.sha256` | KEEP | provenance |
| `collision_mesh_manifest.<link>.stl_relative_path` | KEEP | effective URDF filename |
| `.sha256` | KEEP | mesh provenance |
| `.triangle_count` | KEEP | loader result |
| `.degenerate_triangles_dropped` | KEEP | loader result |

V5 adds `scale_xyz`, `origin_xyz`, and `origin_rpy` for every collision
element. Resolved absolute paths are non-portable generation metadata only.

### Pair policy

| v4 field | Class | V5 destination |
|---|---|---|
| `policy_version` | RENAME | `geometry_pair_policy.version` |
| `supersedes` | RESTRUCTURE | provenance lineage |
| `rules.revolute_adjacent` | RENAME | pure geometric role |
| `rules.fixed_adjacent` | RENAME | pure geometric role |
| `rules.non_adjacent` | RENAME | pure geometric role |
| `endstop_metrology` | RENAME | `active_pair_contact_search_rule` |
| `path_safety` | RENAME | `path_obstruction_pair_rule` |
| `clearance_gate_applies_to` | MOVE TO SAFETY POLICY | external gate scope |
| `clearance_gate_excluded_from` | MOVE TO SAFETY POLICY | external gate scope |
| `revolute_adjacent_pairs` | KEEP | topology-derived |
| `fixed_adjacent_pairs` | KEEP | topology-derived |
| `revolute_adjacent_pair_count` | KEEP | derived validation |
| `fixed_adjacent_pair_count` | KEEP | derived validation |
| `active_pair_by_joint` | KEEP | topology-derived |

The pure V5 obstruction rule does not use `safety` in its name.

### Compiler provenance

| v4 field | Class | V5 destination |
|---|---|---|
| `geometry_compiler.source_file_sha256` | KEEP | all pure-geometry sources influencing output |
| `geometry_compiler.source_combined_sha256` | KEEP | canonical digest of that manifest |

Hardware Reconciler and Safety Policy record their own source/input digests.

### Numerical parameters

| v4 field | Class | V5 destination |
|---|---|---|
| `coarse_step_rad` | KEEP | contact/path search |
| `envelope_margin_rad` | KEEP | search domain |
| `bisection_resolution_rad` | KEEP | contact/path search |
| `max_bisection_iterations` | KEEP | contact/path search |
| `path_step_rad` | KEEP | raw path geometry |
| `parking_path_step_rad` | KEEP | raw parking geometry |
| `narrow_phase_margin_m` | KEEP | collision kernel |
| `grid_cell_size_m` | KEEP | collision kernel |
| `max_narrow_phase_candidate_pairs` | KEEP | bounded resource guard |
| `sensitivity_step_rad` | KEEP | optional raw derivative |
| `min_gradient_m_per_rad` | KEEP | numerical stability |
| `min_clearance_pass_m` | MOVE TO SAFETY POLICY | legacy 3 mm |
| `parking_seed_angles_deg` | REMOVE FROM PURE GEOMETRY | forbidden as canonical V5 truth |
| `model_limit_mismatch_threshold_rad` | REMOVE FROM PURE GEOMETRY | retain raw delta only |

`clearance_sample_stride` is added when used because it changes sampling.

### Manufacturing tolerance

| v4 field | Class | Destination |
|---|---|---|
| `manufacturing_tolerance.print_tolerance_m` | REMOVE FROM PURE GEOMETRY | external tolerance analysis |
| `.assembly_tolerance_note` | REMOVE FROM PURE GEOMETRY | external engineering UNKNOWN |

### Endpoint

| v4 field | Class | V5 destination |
|---|---|---|
| `endpoint_id` | RESTRUCTURE | identity `{joint_name, limit_side}`; string is presentation |
| `leg` | RESTRUCTURE | topology-derived `articulated_branch_id` |
| `joint_group` | REMOVE FROM PURE GEOMETRY | name-derived label |
| `side` | RENAME | `limit_side` |
| `joint_name` | KEEP | model identity |
| `servo_id` | RENAME | `model.joints[].motor_id`, read from URDF |
| `urdf_declared_limit_rad` | RENAME | `declared_limit_rad` |
| `urdf_lower_rad` | RESTRUCTURE | `model.joints[].limit.lower_rad` |
| `urdf_upper_rad` | RESTRUCTURE | `model.joints[].limit.upper_rad` |
| `prerequisite_pose_rad` | RESTRUCTURE | effective `search_context.joint_positions_rad`, never mandatory |
| `other_legs_pose_rad` | RESTRUCTURE | same complete context; remove “other legs” concept |
| `active_revolute_pair` | KEEP | topology-derived pair |
| `pair_class` | KEEP | topology-derived |
| `endpoint_evidence_class` | RESTRUCTURE | independent contact/path outcomes; hardware moves out |
| `result_kind` | RENAME | `GEOMETRIC_CONTACT_FOUND` / `NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN` |
| `mesh_predicted_contact_rad` | RENAME | `geometric_contact.angle_rad` |
| `delta_from_declared_rad` | RENAME | `geometric_contact.minus_declared_limit_rad` |
| `model_limit_mismatch` | REMOVE FROM PURE GEOMETRY | raw delta is sufficient |
| `contact_link_pair` | RENAME | `geometric_contact.link_pair` |
| `is_cross_leg_contact` | RESTRUCTURE | topology relation for contact and obstruction separately |
| `clearance_before_contact_m` | RESTRUCTURE | raw `{status,value_m,kind}` if evaluated |
| `bracket_clear_rad` | RESTRUCTURE | `geometric_contact.bracket.clear_rad` |
| `bracket_contact_rad` | RESTRUCTURE | `geometric_contact.bracket.contact_rad` |
| `contact_model_status` | RESTRUCTURE | independent contact/obstruction outcomes, no mixed verdict |
| `contact_model_status_reason` | RESTRUCTURE | geometry reason codes only |
| `path_collision_angle_rad` | RESTRUCTURE | `path_obstruction.angle_rad` |
| `path_collision_link_pair` | RESTRUCTURE | `path_obstruction.link_pair` |
| `hardware_evidence_note` | MOVE TO HARDWARE RECONCILER | forbidden in pure profile |
| `hardware_vs_urdf_status` | MOVE TO HARDWARE RECONCILER | forbidden in pure profile |
| `mesh_vs_hardware_status` | MOVE TO HARDWARE RECONCILER | forbidden in pure profile |
| `numerical_search` | RESTRUCTURE | explicit contact/path search provenance |
| `sensitivity` | RESTRUCTURE | raw derivative only; tolerance moves out |

Mixed v4 evidence values:

```text
HARDWARE_CONFIRMED_CONTACT -> Hardware Reconciler
HARDWARE_CONTRADICTED      -> Hardware Reconciler
GEOMETRIC_ENDPOINT_CANDIDATE -> pure contact outcome
NO_MODELED_CONTACT           -> pure no-contact outcome
PATH_LIMITED                 -> independent PATH_OBSTRUCTION outcome
```

### Endpoint numerical search

| v4 field | Class | V5 destination |
|---|---|---|
| `coarse_step_rad` | KEEP | per-result provenance |
| `bisection_resolution_rad` | KEEP | per-result provenance |
| `bisection_iterations` | KEEP | per-result measurement |
| `analysis_envelope_rad` | RENAME | `search_domain_rad` |

### Sensitivity

| v4 field | Class | V5 destination |
|---|---|---|
| `contact_link_pair` | KEEP | pinned pair |
| `finite_difference_step_rad` | KEEP | numerical provenance |
| `clearance_near_m` / `clearance_near_kind` | KEEP | raw geometry |
| `clearance_far_m` / `clearance_far_kind` | KEEP | raw geometry |
| `gradient_m_per_rad` | KEEP | raw geometry |
| `gradient_stable` | RESTRUCTURE | `COMPUTED`, `NOT_APPLICABLE`, `UNRESOLVED` |
| `unstable_reason` | RENAME | `status_reason` |
| `tolerance_used_m` | REMOVE FROM PURE GEOMETRY | external input |
| `tolerance_budget_note` | REMOVE FROM PURE GEOMETRY | external assumption |
| `estimated_uncertainty_rad` | REMOVE FROM PURE GEOMETRY | tolerance-derived output |

### Parking plan

| v4 field | Class | V5 destination |
|---|---|---|
| `parking_plans.<leg-key>` | RESTRUCTURE | topology branch collection |
| `leg` | RESTRUCTURE | `articulated_branch_id` |
| `auxiliary_parking_required` | RESTRUCTURE | explicit geometry outcome enum |
| `reason` | RESTRUCTURE | geometry-only reason code/details |
| `parked_leg` | RESTRUCTURE | implicated/parked branch ID |
| `parking_angle_rad` | RESTRUCTURE | joint-position map supporting 1-DOF/2-DOF |
| `active_leg_sequence_passed` | MOVE TO SAFETY POLICY | v4 includes 3 mm gate |
| `active_leg_sequence_min_clearance_m` | KEEP | raw clearance plus kind/completeness |
| `park_path_passed` | MOVE TO SAFETY POLICY | v4 includes gate |
| `park_path_min_clearance_m` | KEEP | raw clearance plus kind/completeness |
| `segments` | RESTRUCTURE | complete `path_in`, `task_path`, `path_out` |

### Path segment

| v4 field | Class | V5 destination |
|---|---|---|
| `description` | KEEP | presentation label |
| `joint_name` | RESTRUCTURE | moved-joint set, supports 2-DOF |
| `start_rad` / `end_rad` | RESTRUCTURE | full start/end configurations |
| `passed` | MOVE TO SAFETY POLICY | composite threshold verdict |
| `has_true_collision` | RENAME | `intersects` / `collision_free` |
| `min_clearance_m` / `min_clearance_kind` | KEEP | raw geometry |
| `clearance_gate_result` | MOVE TO SAFETY POLICY | threshold verdict |
| `first_collision_angle_rad` | RESTRUCTURE | first obstruction configuration |
| `first_collision_pair` | RENAME | `first_obstruction.link_pair` |
| `sample_count` | KEEP | numerical provenance |

### LF hardware reconciliation

The entire container and every listed field are **MOVE TO HARDWARE
RECONCILER**:

```text
endpoint_id
urdf_declared_limit_rad
hardware_contact_rad
compiler_contact_model_status
compiler_mesh_predicted_contact_rad
compiler_contact_link_pair
hardware_vs_urdf_status
mesh_vs_hardware_status
hardware_evidence_note
required_corrective_action
```

The reconciler records the geometry semantic hash, evidence-dataset hash and
reconciler source hash, and never writes adjusted geometry truth.

## Semantic hash rule

`semantic_content_sha256` is canonical sorted JSON excluding the hash itself
and the complete `generation_metadata` container. Timestamp, runtime, worker
count, dirty state, benchmark environment and output path are non-semantic.
Model, geometry, search parameters, deterministic result order and source
hashes are semantic. Workers 1 and 4 must have the same semantic hash.

## G4 → G7 same-new-geometry oracle

Match each endpoint by `{joint_name, limit_side}` and compare:

```text
active_revolute_pair
v4 result_kind -> V5 pure contact outcome
mesh_predicted_contact_rad -> geometric_contact.angle_rad
contact_link_pair
bracket_clear_rad / bracket_contact_rad
analysis_envelope_rad -> search_domain_rad
bisection resolution and iterations
delta_from_declared_rad
path_collision_angle_rad -> independent path_obstruction.angle_rad
path_collision_link_pair -> independent path_obstruction.link_pair
```

Angles must agree within the predeclared bisection resolution when the G4
context pose and parameters are replayed. That pose is an oracle-harness input,
not mandatory V5 truth.

Do not equality-gate hardware/policy verdicts, `model_limit_mismatch`, parking
`passed`, the 3 mm gate, LF reconciliation or the v4 content hash. The two
legacy `MODEL_INCOMPLETE` and two path-limited cases remain valid geometric
contacts. The latter additionally retain an independent preceding obstruction.

Any unexplained geometric difference is a mandatory STOP.

## Required V5 test boundaries

```text
model-first selection and renamed-model invariance
collision filename, per-mesh scale and collision origin
motorId and motorDirection from URDF
q=0 12/12 active-pair separation
pure search without hardware imports
schema forbidden hardware/policy fields
semantic hash determinism and runtime exclusions
frozen v1-v4 artifact hashes
G4 oracle: 24 contacts and 6 path events
separate immutable Hardware Reconciler
separate reproducible 3 mm Safety Policy
topology-driven 1-DOF then 2-DOF parking
NO FEASIBLE PARKING outcome
workers=1 vs workers=4 semantic equality
parent-only artifact writes
PATH_OBSTRUCTION never automatically labelled cross-leg
```

## Decisions not reopened

Approved meshes, visual/collision separation, Phase1B, schema v1-v4, G4 oracle,
pure geometry/hardware/safety separation, topology-first structure, four
processes under 6 GiB with zero swap, fixed candidate caps, hardware isolation,
norma-core isolation and no merge to main remain mandatory.
