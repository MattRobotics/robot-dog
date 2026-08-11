# MATDOG Geometry Compiler V5 — corrected final validation report

**Date:** 2026-08-11<br>
**Worktree:** `/home/matteo-manicardi/MATDOG/worktrees/robot-dog-geometry-v5`<br>
**Branch:** `matdog/geometry-compiler-v5-collision-baseline`<br>
**Base SHA:** `e71876e80c23c370f9fecf36ddf15f152faf5eb3`<br>
**Status:** post-independent-audit corrected candidate<br>
**Canonical deployment output:** corrected benchmark D, workers=4, prefix
`2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4_*`

This report supersedes
`2026-08-11_072224_MATDOG_GEOMETRY_V5_FINAL_VALIDATION_REPORT.md`. That
document and its whole `2026-08-11_072224` bundle are retained unchanged as
**SUPERSEDED PRE-AUDIT V5 CANDIDATE EVIDENCE**; none of its semantic hashes
are canonical.

## Why this report exists

An independent adversarial audit of PR #19 found a composition-root defect.
The integrated runner required the frozen schema-v4 G4 profile and reused the
*same* analyses both as replay evidence and as the canonical "pure geometry"
endpoint profile. The canonical artifact therefore carried the legacy
`30 / 50 / 90` degree prerequisite context on 20 of 24 endpoints, and its
`path_obstruction` layer contradicted the pure `q=0` parking results inside
the same combined profile on four endpoints.

Endpoint *contact* angles were never affected — an active revolute
parent/child pair depends only on its own joint — but the path layer and the
claim of purity were. The remediation separates the two executions, makes the
canonical context structurally empty, redefines the canonical path domain, and
adds a fail-closed consistency gate between the endpoint and planner layers.

## 1. Canonical pure V5 geometry

The canonical pipeline uses model-derived `q=0` context only.

```text
canonical endpoint search contexts empty: 24/24
canonical task type:  EndpointSearchTaskV5, context_pose_rad = {}
canonical path domain mode: DIRECT_TO_GEOMETRIC_TARGET
```

`run_geometry_compiler_v5` no longer accepts a G4 profile argument and
`matdog_geometry_compiler_v5.py` no longer imports the oracle module.
`_assert_canonical_context_free` gates the tasks before execution and the
returned analyses after execution; the integrated runner re-gates 24/24
context-freedom on the built profile independently.

## 2. G4 replay is separate and non-canonical

`run_g4_replay_v5` is a distinct API that returns only analyses, comparison
evidence and a runtime. It owns no profile builder and no artifact writer, so
replay context has no path into canonical serialization. `G4ReplayTask`
hard-rejects the canonical path-domain mode; canonical tasks cannot carry the
historical envelope mode.

Both corrected replay artifacts report:

```text
status:                      PASS
artifact_role:               NONCANONICAL_G4_REPLAY_ORACLE
canonical_profile_eligible:  false
canonical_use:               NON_CANONICAL_REPLAY_EVIDENCE_ONLY
endpoints:                   24
geometric contacts:          24
replay path events:          6
replay events preceding contact: 2
```

The legacy `30 / 50 / 90` degree seeds now exist **only** inside this replay
evidence. The canonical profile contains none of them.

## 3. G4 oracle actual coverage

`compare_g4_g7` evaluates **34 named checks per endpoint**, all true for all 24
in both corrected C and D:

```text
endpoint_identity, active_pair, replay_context, path_context
contact_status, contact_angle, contact_pair
contact_bracket_clear, contact_bracket_contact
contact_search_start, contact_search_domain, g4_analysis_envelope
contact_coarse_step, contact_bisection_resolution
contact_max_bisection_iterations, contact_bisection_iterations
declared_limit, declared_limit_delta
path_analysis_present, path_endpoint_identity, path_status
path_angle, path_pair, path_record_shape, g4_path_record_shape
path_bracket_clear, path_bracket_contact
path_search_domain, path_coarse_step, path_bisection_resolution
path_max_bisection_iterations, path_bisection_iterations
path_precedes_contact, g4_path_classification
```

Acceptance for outcome quantities remains `absolute difference <=
max(G4, G7 bisection resolution)`, i.e. the declared numerical resolution. It
was **not** loosened and **not** tightened to force a PASS. The observed tight
deltas are recorded as `tight_replay_diagnostic` and are explicitly marked
`DIAGNOSTIC_ONLY_DOES_NOT_TIGHTEN_ACCEPTANCE`:

```text
max contact abs delta: 6.997957768817287e-12 rad
max path abs delta:    5.748956866113986e-12 rad
```

Explicit representation exceptions, recorded in every oracle artifact:

- G4 `analysis_envelope_rad` is a symmetric reporting envelope around the
  declared limit while both algorithms sweep `q=0` to the directional far
  bound; the oracle validates the G4 envelope and compares that far bound to
  the complete G7 search domain.
- G4 does not serialize a path relation; G7 relation presence/absence is
  validated but equality is not claimed.
- G4 does not serialize the path clear-side bracket or path bisection count.
  For all six frozen interior full-coarse-step events these are
  deterministically reconstructed from side, path contact bound, coarse step,
  resolution and maximum iterations before comparison. The reconstruction
  fail-closes: if an event is not a full-coarse-step interior event, both
  checks evaluate false.
- For a hypothetical G4 no-contact record G4 has no result pair whereas V5
  retains the searched active pair; the ordered active pair is compared
  instead. Frozen G4 contains 24 contacts, so this branch is not exercised.

## 4. Canonical path domain

```text
canonical_path_domain: Q0_TO_GEOMETRIC_CONTACT_OR_DECLARED_LIMIT
```

The canonical path-obstruction search runs from `q=0` to the endpoint's own
geometric target — the contact angle, or the declared limit when no contact
exists — over the **same equal-subdivision grid** the parking baseline uses
(`intervals = ceil(|target| / step)`). Both layers then bisect the first
transition independently.

**Semantic consequence, stated explicitly:** an obstruction lying *beyond* the
target is not a canonical path-obstruction result. The pre-remediation
headline "6 path obstructions / 2 precede contact" was produced under the
legacy full-envelope domain and legacy context. It is **not** canonical V5
truth and must not be carried forward. The historical full-envelope view is
preserved separately in the G4 replay evidence (section 2).

## 5. Canonical endpoint/planner consistency

`validate_endpoint_path_plan_consistency` is a fail-closed hard gate run
inside the integrated runner. It rejects a non-empty canonical context, a
target mismatch between layers, a coarse-step or domain mismatch, a planner
baseline that is not the same `q=0` direct sweep, an obstruction-status
disagreement, a link-pair or relation disagreement, a refined-angle
disagreement beyond the declared resolution, and any refinement bracket that
does not lie on the path interpolation or exceeds its declared width.

```text
endpoint_count:                     24
consistent_endpoint_count:          24
obstructed_count:                    6
collision_free_count:               18
max_precise_to_refined_delta_rad:   3.6703973194107675e-13
coarse_step_rad:                    0.017453292519943295
```

The delta is nine orders of magnitude inside the `1e-4 rad` declared
resolution.

## 6. Canonical path results

From the corrected `2026-08-11_131818` D bundle. The endpoint layer and the
planner baseline now agree on **all** 24 endpoints — symmetric difference of
the obstructed sets is empty, and every obstructed endpoint agrees on link
pair and relation.

| Endpoint | Relation | First blocking pair |
|---|---|---|
| `lf_upper_leg_joint:max` | `cross_branch` | `lf_foot_link ↔ lh_foot_link` |
| `rf_upper_leg_joint:max` | `cross_branch` | `rf_foot_link ↔ rh_foot_link` |
| `lf_lower_leg_joint:min` | `same_branch` | `lf_hip_link ↔ lf_lower_leg_link` |
| `rf_lower_leg_joint:min` | `same_branch` | `rf_hip_link ↔ rf_lower_leg_link` |
| `rh_lower_leg_joint:min` | `same_branch` | `rh_hip_link ↔ rh_lower_leg_link` |
| `lh_lower_leg_joint:min` | `same_branch` | `lh_hip_link ↔ lh_lower_leg_link` |

```text
canonical direct-target obstructions: 6
canonical collision-free direct paths: 18
```

All six are obstructions *within* the task-to-target domain. `PATH_OBSTRUCTION`
remains topology-classified and is not automatically cross-leg: four of six are
`same_branch`.

A further gate proves the de-contextualization did not move geometry.
`_compare_canonical_contacts_to_replay` compares every canonical `q=0` contact
against its frozen-context replay counterpart:

```text
endpoint_count:                 24
matching_endpoint_count:        24
max_contact_angle_delta_rad:    6.907807659217724e-12
acceptance_basis:               MAX_DECLARED_BISECTION_RESOLUTION
```

## 7. Git commit and working-tree state are non-semantic

`commit_sha` and `working_tree_dirty` now live in
`generation_metadata.repository_materialization`, which is excluded from
`semantic_content_sha256`. Previously `provenance.repository.commit_sha` was
inside the semantic payload, which made the canonical identity irreproducible
across any commit. `provenance` no longer contains a `repository` key.

## 8. Semantic provenance

The canonical semantic payload is anchored on:

- `provenance.urdf.sha256` and relative path;
- `provenance.collision_meshes` — per-link STL path, SHA256, triangle count,
  dropped degenerates, scale and origin;
- `provenance.geometry_compiler.source_file_sha256` — the nine canonical
  semantic source files, plus their combined digest;
- the complete model record, pair policy, analysis parameters, deterministic
  endpoint searches and path plans.

The G4 oracle module is deliberately **excluded** from the canonical semantic
source manifest, so oracle changes cannot perturb canonical geometry identity.
It has its own seven-file replay manifest, and the eleven-file execution
manifest covers both.

## 9. CPU affinity

Process-level, not cgroup-level, and stated as such:

```text
cpu_affinity:                PROCESS_SCHED_AFFINITY_INHERITED_BY_SPAWN_WORKERS
cpuset_cpus_effective_role:  TELEMETRY_ONLY
observed process affinity:   [0, 1, 2, 3]
```

The runner validates `os.sched_getaffinity(0)` against the required mask
independently of cgroup mode; spawned workers inherit the mask.

## 10. Memory, swap and OOM

Genuinely cgroup-v2 enforced and hard-gated (`memory_swap_oom:
CGROUP_V2_VALIDATED`):

```text
memory.max        6442450944 bytes (6 GiB)
memory.swap.max   0
swap peak         0
oom / oom_kill    0 / 0
```

## 11. Corrected C/D semantic identity

| Semantic artifact | C W1 | D W4 | Result |
|---|---|---|---|
| Endpoint profile | `de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e` | same | PASS |
| Parking v2 | `67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139` | same | PASS |
| Combined profile | `0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51` | same | PASS |

```text
corrected C run manifest SHA256: b86b5d35678df4510989ea49fb5d42acaea2e4838226d7017456399dfceb0d81
corrected D run manifest SHA256: 0db86e633599f63a769dbba75db3a54c1e6470e128c007eb24c465f92a428b17
run manifest schema:             matdog.geometry_compiler_v5.integrated_run.v2
```

D's seven determinism comparisons are all true, including the new
`g4_replay_oracle_semantic_payload_equal` and `g4_replay_source_manifest_equal`.
Run manifests are now self-hashing (`manifest_content_sha256`) and are
published **last** as a bundle validity marker
(`bundle_validity.marker: RUN_MANIFEST_PUBLISHED_LAST`); a bundle without its
manifest is not a valid bundle.

## 12. Corrected C/D resource results

| Benchmark | Workers | Wall | cgroup tree peak | Envelope use |
|---|---:|---:|---:|---:|
| C | 1 | 3,360.21 s | 466,382,848 B (444.777344 MiB) | 7.238% |
| D | 4 | 1,691.92 s | 1,798,238,208 B (1,714.933594 MiB) | 27.912% |

```text
C/D = 1.986034x
```

Both runs: `spawn` process pool for D, `OMP/OPENBLAS/MKL/NUMEXPR_NUM_THREADS=1`,
affinity `[0,1,2,3]`, `MemoryMax` 6 GiB, `MemorySwapMax` 0, zero swap, zero OOM,
zero worker writes, one parent bundle publication.

The corrected runs are slower than the superseded ones because they now execute
two independent endpoint passes — the canonical `q=0` pass and the separate
non-canonical G4 replay — plus planner obstruction refinement.

```text
C: canonical endpoint 522.733140 s | G4 replay 531.316916 s | parking 2276.664515 s
D: canonical endpoint 248.817139 s | G4 replay 261.024330 s | parking 1152.318666 s
```

Candidate-pair and AABB-survivor totals remain explicitly `NOT_INSTRUMENTED`.

## 13. Superseded pre-audit hashes

The following are **SUPERSEDED** and are not canonical:

```text
endpoint  cad2f194c49d063b5a09ae4602b9acf61a701de48791e5d1aae04f1439db1211
parking   3cda03c2c02ba5fbe6def7821ce72d4aa9e4ca66e7ca8655a2b8f0e92dde297c
combined  e99e2b65ea8d032f94b5d1aa815432a292c1771766e556fc7115dc7a1f5de73e
```

The whole `2026-08-11_072224` bundle is preserved unchanged for provenance and
is labelled **SUPERSEDED PRE-AUDIT V5 CANDIDATE EVIDENCE** in the artifact
index. Its own reports carry a superseded banner pointing here.

## Separate hardware reconciliation

Regenerated against the corrected canonical combined profile.

```text
input geometry semantic SHA256: 0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51
LF evidence file SHA256:        6eae3201a00b5299550028d5b4e1e73d67520deccf5a85e548f3b07b1777cab4
reconciliation content SHA256:  0af31e9dbcae0aae22978faeaf12a668c8d402abf921d2d63cea49f11195e70c

AGREES:                 3
DISAGREES:              3
NO_GEOMETRIC_CONTACT:   0
geometry-only endpoints: 18
geometry_modified:      false
```

Contact angles are unchanged from the superseded run — they are
context-independent — so the 3/3 split is unchanged. The
`path_obstruction_context` did change, as a direct and expected consequence of
the corrected canonical path domain:

| Endpoint | Superseded context | Corrected context |
|---|---|---|
| `lf_hip_joint:min` | `base_link ↔ lf_lower_leg_link`, precedes=true | `NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN` |
| `lf_upper_leg_joint:max` | `NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN` | `lf_foot_link ↔ lh_foot_link`, precedes=true |
| `lf_lower_leg_joint:min` | `lf_upper_leg_link ↔ lf_foot_link`, precedes=false | `lf_hip_link ↔ lf_lower_leg_link`, precedes=true |

RF/RH/LH remain geometry-only and must not be described as hardware-confirmed.
No evidence is mirrored to another assembly and no geometry was adjusted.

## Separate safety policy

Regenerated against the corrected canonical parking artifact. The external
3 mm threshold is unchanged.

```text
input parking semantic SHA256: 67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139
safety semantic SHA256:        e5cb2a4c33082c59c6f5d381f90fcc19680cc899e326d13c9c9129932bde5d08
threshold_m:                   0.003

PASS:                              16
FAIL:                               0
UNRESOLVED lower-bound cases:       8
outside-limit cases that pass:      8
motion authorizations granted:      0
geometry mutated:               false
frozen G4 3 mm replay:          exact, 0/4 legacy active sequences accepted
```

`clearance_policy_accepted` is `true` for the 16 PASS rows and `null` for all 8
UNRESOLVED rows — UNRESOLVED can never become PASS. Every row, including all
eight outside-limit clearance PASSes, carries
`motion_authorization: NOT_GRANTED_OFFLINE_EVIDENCE_ONLY`. A clearance PASS is
limited to sampled clearance evidence and never means an outside-limit target
is safe or authorized for execution.

## Path and parking result

Unchanged by the remediation, and independently reverified:

```text
24 endpoint plans
18 NOT_NEEDED
6  FEASIBLE_1DOF_PLAN_FOUND
0  FEASIBLE_2DOF_PLAN_FOUND
0  NO_FEASIBLE_PARKING_FOUND_IN_DECLARED_SEARCH_DOMAIN
24/24 complete geometric sequences
94 evaluated 1-DOF candidates
0  evaluated 2-DOF candidates
8  EXECUTABLE_URDF_DOMAIN targets
16 DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS targets
```

Planner obstruction refinement is new evidence, not a behavioural change: the
refined blocking pair equals the raw sampled blocking pair for all six
obstructed endpoints, so movable-joint selection, parking configurations and
candidate counts are identical to the superseded run.

An outside-limit target is diagnostic geometry, never authorization to move a
robot. The integrated pure-geometry artifacts record
`external_safety_policy_applied: false`.

## Frozen geometry authority

```text
URDF SHA256:            3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59
G4 file SHA256:         f1b059a58c51508345ec583cc421cf9ca66ec8e9ac5547649c05bbf755e5c5fa
G4 content SHA256:      4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61
LF evidence SHA256:     6eae3201a00b5299550028d5b4e1e73d67520deccf5a85e548f3b07b1777cab4
approved collision meshes: 17
loaded triangles:          119,696
q=0 separated active pairs: 12/12
```

All 36 entries of `03_CAD/URDF/matt_robodog_rev00/SHA256SUMS.txt` validate
against current bytes. No collision or visual geometry changed in this
finalization.

## 14. Pre-existing live-FK calibration-status mismatch

`06_Software/Matdog_Core/kinematics/tests/test_matdog_leg_fk_live.py` reports
four failing methods across four legs. The tracked
`MATDOG_JOINT_CALIBRATION.yaml` declares
`calibration_status: DIGITAL_ZERO_CALIBRATED_AND_VERIFIED` while
`matdog_leg_fk_live.py` requires
`VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION`.

This is **PRE-EXISTING** and byte-identical on `main @ e71876e8`. It is
**NON-BLOCKING for PR #19** and remains a separate next-phase / live-FK issue.
It was deliberately not changed to manufacture a green suite. PR #19 touches
that YAML only to update the pinned URDF SHA.

## 15. CI

**RECOMMENDED BUT NON-BLOCKING**, and deliberately **not implemented in this
task**. The repository currently has no `.github/workflows`. The offline
geometry and adjacent calibration suites are CI-viable and should be wired up
before Phase 2A.

## Remaining unknowns

- RF/RH/LH contacts have no hardware oracle and remain geometric candidates.
- Sixteen targets lie outside declared URDF limits and are diagnostic-only.
- Eight clearance lower bounds remain unresolved under the external 3 mm policy.
- Candidate-pair and AABB-survivor counts are not instrumented.
- Target Jetson Orin Nano Super performance remains unmeasured; this host run
  is an algorithm/resource baseline, not cycle-accurate emulation.
- The URDF has no generic `calibratable` tag; for REV00 bounded revolute plus
  complete motor metadata selects exactly the approved 12.
- `_atomic_json` in `matdog_geometry_compiler_v5.py` is dead after the oracle
  writer was removed. It was deliberately **not** deleted: that file is in the
  corrected C/D canonical-semantic and execution source manifests, so removing
  it would invalidate corrected C/D provenance and force a forbidden rerun.
  C/D validity takes priority over cosmetic cleanup.

## Permanent constraint declaration

```text
NO HARDWARE USED
NO STATION / SERIAL / SERVO / EEPROM USED
NO GEOMETRY FITTED TO HARDWARE
NO THRESHOLD RELAXED
NO COLLISION CANDIDATE CAP RAISED
NO LF V25 EVIDENCE MODIFIED
NO HISTORICAL v1-v4 ARTIFACT MODIFIED
NO 2026-08-11_072224 ARTIFACT BUNDLE OVERWRITTEN OR DELETED
NO NORMA-CORE MODIFIED
NO HISTORICAL RF NORMACORE WORKTREE MODIFIED
NO FORCE-PUSH
NO MERGE TO MAIN PERFORMED
```

Publication is limited to the development branch for human review. PR #19
remains OPEN and DRAFT. No merge is authorized.
