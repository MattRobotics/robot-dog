# MATDOG Geometry Compiler V5 — final architecture

**Date:** 2026-08-11<br>
**Branch:** `matdog/geometry-compiler-v5-collision-baseline`<br>
**Base SHA:** `e71876e80c23c370f9fecf36ddf15f152faf5eb3`<br>
**Status:** G5-G11 implemented and validated; G12 artifact set complete<br>
**Frozen G4 content SHA256:** `4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61`

## Architectural result

Geometry Compiler V5 is a model-first, pure-geometry pipeline. The URDF and
its referenced collision meshes are the geometry authority. Joint or link
names remain opaque presentation identifiers; topology and URDF metadata
drive actuator selection, branch construction, collision relationships,
endpoint search and parking.

The implementation preserves four independent meanings:

```text
geometric endpoint contact
!= path obstruction / reachability
!= hardware evidence
!= safety-policy acceptance
```

No layer is allowed to promote one of these meanings into another. In
particular, a path obstruction is classified from topology as
`same_branch`, `body_vs_branch`, `cross_branch` or `body_internal`; it is not
automatically a cross-leg event.

## Model authority and hard gates

The current REV00 URDF unambiguously selects the intended 12 actuators with:

```text
joint.type == revolute
AND exactly one hardware/motorId
AND exactly one hardware/motorDirection in {-1,+1}
```

The verified model contains 17 unique links, 16 unique joints, one root, four
topology-derived linear articulated branches, 12 bounded revolute joints and
four fixed terminal joints. Motor IDs are unique; all axes are finite and
nonzero; all limits are finite and ordered. Branch and within-branch ordering
are derived deterministically from topology and motor metadata, without
parsing `lf`, `rf`, `rh`, `lh`, `hip`, `upper` or `lower`.

Every run is gated by the following immutable inputs:

- URDF SHA256
  `3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59`;
- the approved 17-file collision manifest in
  `03_CAD/URDF/matt_robodog_rev00/SHA256SUMS.txt`;
- 119,696 loaded collision triangles;
- 12/12 active revolute parent-child pairs separated at q=0;
- the frozen G4 file and recomputed legacy content hash;
- semantic and execution source manifests.

The input fingerprint is checked before computation, in each spawned worker,
after computation and again before publication. A source or input change is a
hard failure. Canonical output paths are repository-bound and no-clobber.

## Component boundaries

| Component | Responsibility |
|---|---|
| `matdog_geometry_model_v5.py` | Parse URDF topology, axes, limits, hardware mapping, collision filenames, scale/origin and topology relations. |
| `matdog_geometry_scene_v5.py` | Pure FK and collision scene; apply each URDF collision transform. |
| `matdog_geometry_contact_search_v5.py` | Search active-pair contact and independent path obstruction in an explicit pose context. |
| `matdog_geometry_g4_oracle_v5.py` | Reconstruct the frozen G4 replay tasks and compare the same-new-geometry result. |
| `matdog_geometry_profile_v5.py` | Build, validate and hash pure V5 geometry profiles. |
| `matdog_geometry_report_v5.py` | Render geometry-only human-readable reports. |
| `matdog_geometry_path_planner_v5.py` | Validate full paths and search topology-driven 1-DOF, then 2-DOF, parking. |
| `matdog_geometry_process_workers_v5.py` | Execute deterministic isolated contact/parking batches in one or four processes. |
| `matdog_geometry_compiler_v5.py` | Compose model, contact search, oracle and endpoint profile without writing from workers. |
| `matdog_geometry_full_runner_v5.py` | Run the integrated endpoint/parking pipeline, enforce resource and provenance gates, compare C/D and publish one parent-owned bundle. |
| `matdog_geometry_hardware_reconciler.py` | Consume immutable LF evidence separately; never alter geometry truth. |
| `matdog_geometry_safety_policy.py` | Apply the historical 3 mm clearance policy as a separate offline consumer. |

The validated Phase1B mesh kernel is reused without changing its geometric
semantics. Historical schema-v1 through schema-v4 sources and artifacts remain
immutable.

## Pure-geometry schemas

The canonical profile schema is:

```text
matdog.calibration_geometry_profile.v5
```

Its semantic payload includes model, collision provenance, pair policy,
analysis parameters, deterministic endpoint searches, path plans and source
hashes. The complete `generation_metadata` container is excluded from
`semantic_content_sha256`; only timestamps, runtime, worker count, dirty-state
and materialization/audit paths live there.

Parking schema `matdog.geometry_path_parking.v2` makes the input profile's
semantic hash the semantic anchor. The exact input path and file SHA are kept
as non-semantic materialization audit metadata. This preserves exact
provenance while allowing equivalent workers=1 and workers=4 files to have the
same parking and combined semantic content. Frozen parking-v1 remains readable
and unchanged.

Hardware, hardware-derived values, manufacturing tolerance and acceptance
thresholds are forbidden from the pure profile. The normative v4-to-V5 field
mapping is recorded in
`MATDOG_GEOMETRY_COMPILER_V5_ARCHITECTURE_SCHEMA_MIGRATION_2026-08-10.md`.

## Contact and path semantics

The G4-to-G7 oracle matches all 24 endpoints by `{joint_name, limit_side}` and
checks active pair, contact result, angle, link pair, bracket, search domain,
bisection data, declared-limit delta and independent path event. The final
replay is PASS:

```text
24/24 geometric contacts found
6 path obstructions retained
2 path obstructions precede endpoint contact
0 unexplained geometric regressions
```

The two legacy `MODEL_INCOMPLETE` LF cases (`lf_hip_max` and
`lf_lower_leg_max`) are still ordinary geometric contacts; their disagreement
with hardware exists only in the Hardware Reconciler. The two legacy
`PATH_COLLISION_BEFORE_ENDPOINT` cases (`lf_hip_min` and `rf_hip_max`) retain
both a geometric endpoint contact and a preceding `body_vs_branch`
obstruction. Four lower-leg obstruction events occur after contact and remain
diagnostic evidence.

## Geometry-driven parking

Parking starts from the validated q=0 configuration and uses current model
topology to identify relevant movable joints. It first tests the direct task
path, then a canonical 1-DOF grid and finally a bounded 2-DOF grid if needed.
Every accepted plan contains complete `path_in`, `task_path`, `task_return`
and `path_out` validation; collision-free means no mesh intersection, not a
clearance-policy verdict.

Final canonical outcomes:

```text
24 endpoint plans
18 NOT_NEEDED
6 FEASIBLE_1DOF_PLAN_FOUND
0 FEASIBLE_2DOF_PLAN_FOUND
0 NO_FEASIBLE_PARKING_FOUND_IN_DECLARED_SEARCH_DOMAIN
24/24 complete geometric sequences
8 EXECUTABLE_URDF_DOMAIN targets
16 DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS targets
94 evaluated 1-DOF candidates
0 evaluated 2-DOF candidates
```

An outside-limit target is diagnostic geometry, never authorization to move a
robot. The integrated pure-geometry artifacts explicitly record
`external_safety_policy_applied: false`.

## Hardware and safety consumers

The immutable LF V25 evidence SHA256 is
`6eae3201a00b5299550028d5b4e1e73d67520deccf5a85e548f3b07b1777cab4`.
The separate 2-degree Hardware Reconciler reports 3 AGREES and 3 DISAGREES
across six LF endpoints; RF/RH/LH remain geometry-only. It does not modify a
mesh, endpoint, limit or profile.

The separate historical 3 mm safety-policy replay reports 16 PASS, 0 FAIL and
8 UNRESOLVED lower-bound cases. It grants no motion authorization. The frozen
G4 policy replay remains exact, with 0/4 legacy active sequences accepted.
Clearance is retained as a measured value; the threshold is not geometry.

## Deterministic process architecture

The only supported worker counts are one and four. Both modes execute the
same batch functions and result-validation boundary. The 24 endpoints are
partitioned into 12 deterministic topology-derived joint batches, each
containing `min` then `max`; four-process completion order is discarded and
the parent sorts by `canonical_endpoint_index`.

```text
start method:          spawn
worker writes:         0
parent publications:   1 bundle
OMP_NUM_THREADS:       1
OPENBLAS_NUM_THREADS:  1
MKL_NUM_THREADS:       1
NUMEXPR_NUM_THREADS:   1
four-worker memory:    MemoryMax=6 GiB, MemorySwapMax=0
CPU affinity:          physical cores represented by CPU 0-3
```

The four-process deployment run is accepted only if it matches the one-worker
reference for endpoint, parking and combined semantic payloads, G4 oracle,
input manifest, semantic-source manifest and execution-source manifest. It
does so exactly. Different timestamps, worker metadata, runtimes, paths and
file SHAs are expected non-semantic materialization differences.

## Canonical deployment artifacts

The workers=4 `D` bundle under
`09_Logs/Validation_Reports/Geometry_Compiler/2026-08-11_072224_MATDOG_GEOMETRY_V5_BENCHMARK_D_W4_*`
is the canonical deployment output. Its semantic identities are:

```text
endpoint profile:  cad2f194c49d063b5a09ae4602b9acf61a701de48791e5d1aae04f1439db1211
parking v2:        3cda03c2c02ba5fbe6def7821ce72d4aa9e4ca66e7ca8655a2b8f0e92dde297c
combined profile:  e99e2b65ea8d032f94b5d1aa815432a292c1771766e556fc7115dc7a1f5de73e
run manifest file: 96ba79875f19c37952e9c981946e5a7df4136b86e1fb5b1c13ffbe12d891b57e
```

The matching one-worker `C` bundle is retained as the determinism oracle.

## Remaining legitimate unknowns

- Geometry-only contacts for RF/RH/LH have no hardware validation and must
  not be labelled hardware-confirmed.
- Sixteen geometric targets lie outside their declared URDF limits and are
  diagnostic-only.
- Lower-bound clearance values do not prove an exact clearance; eight remain
  unresolved under the external 3 mm policy.
- Collision candidate-pair and AABB-survivor totals are not instrumented.
- Performance on the target Jetson Orin Nano Super is not measured here; this
  host run is an algorithm/resource baseline, not cycle-accurate emulation.
- The URDF has no separate generic `calibratable` tag. For REV00, bounded
  revolute plus complete motor metadata selects exactly the approved 12; a
  future powered-but-noncalibratable joint would require explicit metadata.

## Permanent boundaries

No hardware, Station, serial, servo or EEPROM access occurred. No geometry or
threshold was fitted to hardware. No candidate cap was raised. No
`norma-core` or historical RF NormaCore worktree was touched. No historical
v1-v4 artifact or LF V25 evidence was modified. No merge to `main` is part of
this work.
