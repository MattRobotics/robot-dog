# MATDOG Geometry Compiler V5 — final architecture

**Date:** 2026-08-11<br>
**Branch:** `matdog/geometry-compiler-v5-collision-baseline`<br>
**Base SHA:** `e71876e80c23c370f9fecf36ddf15f152faf5eb3`<br>
**Status:** G5-G12 implemented and validated; corrected post-independent-audit
candidate<br>
**Canonical bundle:** `2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4_*`<br>
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
| `matdog_geometry_g4_oracle_v5.py` | Own the **non-canonical** frozen-G4 replay end to end: build replay tasks, execute them via `run_g4_replay_v5`, and compare 34 checks per endpoint. Deliberately has no profile builder and no artifact writer, and is excluded from the canonical semantic source manifest. |
| `matdog_geometry_profile_v5.py` | Build, validate and hash pure V5 geometry profiles. |
| `matdog_geometry_report_v5.py` | Render geometry-only human-readable reports. |
| `matdog_geometry_path_planner_v5.py` | Validate full paths and search topology-driven 1-DOF, then 2-DOF, parking. |
| `matdog_geometry_process_workers_v5.py` | Execute deterministic isolated contact/parking batches in one or four processes. |
| `matdog_geometry_compiler_v5.py` | Compose model, contact search and the **canonical context-free** endpoint profile without writing from workers. It does not import the G4 oracle and has no parameter that could accept a G4 profile. |
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

Two independent executions exist and must never be conflated.

**Canonical pure V5** searches every endpoint with an empty, model-derived
`q=0` context and a `DIRECT_TO_GEOMETRIC_TARGET` path domain — that is, from
`q=0` to the endpoint's own geometric contact angle, or to its declared limit
when no contact exists. An obstruction lying *beyond* the target is therefore
not a canonical path-obstruction result.

```text
24/24 canonical endpoint search contexts empty
24/24 geometric contacts found
6  canonical direct-target path obstructions
18 canonical collision-free direct paths
```

Four of the six are `same_branch` and two are `cross_branch`;
`PATH_OBSTRUCTION` remains topology-classified and is not automatically
cross-leg.

**The non-canonical G4 replay** is a separate execution
(`run_g4_replay_v5`) that reproduces the frozen schema-v4 context and the
historical `FULL_ENDPOINT_ENVELOPE` path domain. It exists as refactor
evidence only; its artifacts carry
`artifact_role: NONCANONICAL_G4_REPLAY_ORACLE` and
`canonical_profile_eligible: false`, and the API owns no profile builder or
writer. Its result is PASS with 24/24 contacts, 6 replay path events and 2
replay events preceding contact — those legacy figures describe the replay,
not canonical V5.

`compare_g4_g7` evaluates 34 named checks per endpoint: endpoint identity,
ordered active pair, replay and path context, contact status/angle/pair, both
contact brackets, declared limit and declared-limit delta, G4 analysis
envelope, G7 search start and domain, coarse step, bisection resolution,
maximum and actual iterations, and the full parallel path battery including
reconstructed path brackets and iteration count, path record shape,
path-precedes-contact and the legacy G4 classification cross-check. Outcome
acceptance is `absolute difference <= max(G4, G7 bisection resolution)`;
observed maxima are `6.998e-12 rad` for contact and `5.749e-12 rad` for path,
recorded as a non-binding `tight_replay_diagnostic`. Four explicit
representation exceptions are recorded in every oracle artifact.

The two legacy `MODEL_INCOMPLETE` LF cases (`lf_hip_max` and
`lf_lower_leg_max`) are still ordinary geometric contacts; their disagreement
with hardware exists only in the Hardware Reconciler.

## Endpoint/planner path consistency

`validate_endpoint_path_plan_consistency` is a fail-closed hard gate. The
endpoint layer and the parking baseline sweep the identical equal-subdivision
grid (`intervals = ceil(|target| / step)`) from `q=0` to the same target, then
bisect the first transition independently. The gate rejects a non-empty
canonical context, target or coarse-step or domain mismatch, a planner
baseline that is not the same direct sweep, an obstruction-status
disagreement, a link-pair or relation disagreement, a refined-angle
disagreement beyond the declared resolution, and any refinement bracket that
does not lie on the path interpolation or exceeds its declared width.

```text
24/24 consistent
6  obstructed
18 collision-free
max precise/refined delta: 3.6703973194107675e-13 rad
```

A further gate, `_compare_canonical_contacts_to_replay`, proves that removing
the legacy context did not move any active-pair contact:
`max_contact_angle_delta_rad = 6.907807659217724e-12`, accepted against the
declared bisection resolution.

## Geometry-driven parking

Parking starts from the validated q=0 configuration and uses current model
topology to identify relevant movable joints. It first tests the direct task
path, then a canonical 1-DOF grid and finally a bounded 2-DOF grid if needed.
Every accepted plan contains complete `path_in`, `task_path`, `task_return`
and `path_out` validation; collision-free means no mesh intersection, not a
clearance-policy verdict. `task_return` and `path_out` are proven by exact
reversal of the same sampled configuration set as their forward segments and
record `reverse_validation_of`.

The planner additionally bisects its first obstruction in joint space
(`obstruction_bisection_resolution_rad = 0.0001`, maximum 40 iterations) and
serializes clear/contact progress, both poses, pair, relation, resolution and
iteration count. `validate_parking_artifact` then proves that evidence against
its own contract before the artifact is accepted. Refinement is evidence, not a
behavioural change: the refined blocking pair equals the raw sampled blocking
pair for all six obstructed endpoints, so movable-joint selection, parking
configurations and candidate counts are unchanged.

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
four-worker memory:    MemoryMax=6 GiB, MemorySwapMax=0   (cgroup v2 enforced)
CPU affinity:          CPU 0-3 process sched affinity, inherited by spawn
                       workers; NOT a cgroup cpuset
```

Memory, swap and OOM are cgroup-v2 enforced and hard-gated. CPU pinning is a
process scheduler mask applied with `taskset`; `cpuset.cpus.effective` is
`null` and the manifests record this honestly as
`cpu_affinity: PROCESS_SCHED_AFFINITY_INHERITED_BY_SPAWN_WORKERS` and
`cpuset_cpus_effective_role: TELEMETRY_ONLY`. The runner validates
`os.sched_getaffinity(0)` against the required mask independently of cgroup
mode.

Run manifests use schema `matdog.geometry_compiler_v5.integrated_run.v2`, are
self-hashing (`manifest_content_sha256`) and are published **last** as a bundle
validity marker (`bundle_validity.marker: RUN_MANIFEST_PUBLISHED_LAST`). A
directory of data artifacts without its manifest is not a valid bundle.

The four-process deployment run is accepted only if it matches the one-worker
reference for endpoint, parking and combined semantic payloads, the G4 replay
oracle payload, the input manifest, the canonical semantic-source manifest,
the G4 replay source manifest and the execution-source manifest. It does so
exactly. Different timestamps, worker metadata, runtimes, repository
materialization state, paths and file SHAs are expected non-semantic
materialization differences.

## Canonical deployment artifacts

The corrected workers=4 `D` bundle under
`09_Logs/Validation_Reports/Geometry_Compiler/2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4_*`
is the canonical deployment output. Its semantic identities are:

```text
endpoint profile:  de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e
parking v2:        67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139
combined profile:  0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51
run manifest file: 0db86e633599f63a769dbba75db3a54c1e6470e128c007eb24c465f92a428b17
```

The matching corrected one-worker `C` bundle
(`…REMEDIATION_BENCHMARK_C_W1_*`, run manifest
`b86b5d35678df4510989ea49fb5d42acaea2e4838226d7017456399dfceb0d81`) is
retained as the determinism oracle.

### Superseded pre-audit candidate

The earlier `2026-08-11_072224_MATDOG_GEOMETRY_V5_BENCHMARK_*` bundle is
**SUPERSEDED PRE-AUDIT V5 CANDIDATE EVIDENCE**. It is preserved unchanged for
provenance and its semantic hashes are **not** canonical:

```text
SUPERSEDED endpoint  cad2f194c49d063b5a09ae4602b9acf61a701de48791e5d1aae04f1439db1211
SUPERSEDED parking   3cda03c2c02ba5fbe6def7821ce72d4aa9e4ca66e7ca8655a2b8f0e92dde297c
SUPERSEDED combined  e99e2b65ea8d032f94b5d1aa815432a292c1771766e556fc7115dc7a1f5de73e
```

It was produced by a pipeline whose composition root reused one G4-context
replay as both replay evidence and the canonical profile. An independent
adversarial audit identified that defect; this document describes the
corrected architecture.

## Remaining legitimate unknowns

- Geometry-only contacts for RF/RH/LH have no hardware validation and must
  not be labelled hardware-confirmed.
- Sixteen geometric targets lie outside their declared URDF limits and are
  diagnostic-only.
- Lower-bound clearance values do not prove an exact clearance; eight remain
  unresolved under the external 3 mm policy.
- Collision candidate-pair and AABB-survivor totals are not instrumented.
- `_atomic_json` in `matdog_geometry_compiler_v5.py` is dead after the oracle
  writer moved out. It is deliberately retained: that file is inside the
  corrected C/D canonical-semantic and execution source manifests, so removing
  it would invalidate corrected C/D provenance and require a rerun. C/D
  validity takes priority over cosmetic cleanup.
- The live-FK calibration-status mismatch in
  `test_matdog_leg_fk_live.py` is pre-existing on `main`, non-blocking for
  PR #19, and remains a separate next-phase / live-FK issue.
- No CI workflow exists in this repository. CI is recommended before Phase 2A
  but is deliberately not part of this work.
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
