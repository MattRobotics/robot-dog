# MATDOG Geometry Compiler V5 — final validation report

> **STATUS: SUPERSEDED PRE-AUDIT V5 CANDIDATE EVIDENCE.**
> This document describes the pre-audit V5 candidate. An independent adversarial
> audit of PR #19 found that the integrated runner reused one frozen-G4-context
> replay as both replay evidence and the canonical "pure geometry" endpoint
> profile, so the published canonical artifact carried legacy 30/50/90 degree
> context on 20 of 24 endpoints and its path-obstruction layer contradicted the
> pure q=0 parking results.
>
> The semantic hashes quoted below (`cad2f194…`, `3cda03c2…`, `e99e2b65…`) are
> **not canonical**. The corrected canonical bundle is
> `2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4_*`; see
> `2026-08-11_132758_MATDOG_GEOMETRY_V5_FINAL_VALIDATION_REPORT.md`.
>
> This file is retained unchanged in substance for provenance. Nothing in the
> `2026-08-11_072224` bundle was overwritten or deleted.

**Date:** 2026-08-11<br>
**Worktree:** `/home/matteo-manicardi/MATDOG/worktrees/robot-dog-geometry-v5`<br>
**Branch:** `matdog/geometry-compiler-v5-collision-baseline`<br>
**Base/initial HEAD:** `e71876e80c23c370f9fecf36ddf15f152faf5eb3`<br>
**Overall result:** **G0-G11 PASS; G12 artifact set complete**<br>
**Canonical deployment output:** benchmark D, workers=4

## Recovery of the interrupted turn

The model-capacity interruption affected only the conversational turn. The G10
process launched before it had already completed successfully (`232/232`,
`OK`); no process remained live, no output was ambiguous and no partial
canonical artifact existed. Branch, worktree, HEAD, checkpoint, G4, approved
meshes and frozen G0-G9B artifacts were all reverified before work resumed.

The persistent context checkpoint SHA256 is:

```text
fcf4941595fc11a417e1e9a8de725e010b66d2c54e0ab95c8160183eec70411a
```

## Gate status

| Gate | Status | Accepted evidence |
|---|---|---|
| G0 — local/remote/filesystem truth | PASS | Dedicated worktree/branch/base verified; main checkout clean; 17 staged-source meshes matched 17/17 before placement. |
| G1 — visual/collision separation | PASS | URDF collision filenames point to `meshes/collision/`; visual filenames and visual mesh bytes remain unchanged. |
| G2 — mesh/frame/integrity | PASS | Exactly 17 approved collision files; SHA 17/17; finite, nondegenerate, watertight and frame-consistent; 119,696 triangles. |
| G3 — q=0 hard gate | PASS | 12/12 active revolute parent-child pairs are `SEPARATED_NARROW`. |
| G4 — new-geometry reference replay | PASS / FROZEN | 24 contacts; 20 legacy modeled, 2 legacy model-incomplete, 2 legacy path-before-endpoint; file/content provenance frozen. |
| G5 — pure-geometry architecture | PASS | Model/scene/search/profile composition is URDF- and topology-driven; no name parsing or hardware truth in core. |
| G6 — schema decision | PASS | Additive pure schema v5 and exhaustive v4-to-V5 migration mapping; v1-v4 untouched. |
| G7 — same-new-geometry oracle | PASS | 24/24 endpoint comparisons and all six path events match G4 within `1e-4 rad`; no regression. |
| G8 — Hardware Reconciler | PASS | Separate immutable LF consumer; 3 AGREES, 3 DISAGREES; geometry modified=false. |
| G9 — path/parking | PASS | 24/24 complete sequences; 18 direct, 6 feasible 1-DOF; no geometric infeasibility. |
| G9B — safety separation | PASS | External 3 mm policy reproduced without geometry mutation: 16 PASS, 0 FAIL, 8 UNRESOLVED; no motion authorization. |
| G10 — offline tests | PASS | 262/262 approved calibration tests pass; broader pre-existing kinematics mismatch separately disclosed. |
| G11 — deterministic processes | PASS | Real C/D semantic equality; four spawn workers, parent-only write, 6 GiB/zero-swap/four-core contract. |
| G12 — artifacts and review handoff | ARTIFACT PASS | Required artifact set and reports are complete. Commit, push and Draft PR occur after this pre-publication report is frozen and are recorded in the final session handoff; no merge. |

## Frozen geometry authority

```text
URDF SHA256:
3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59

G4 reference profile file SHA256:
f1b059a58c51508345ec583cc421cf9ca66ec8e9ac5547649c05bbf755e5c5fa

G4 content SHA256:
4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61

Approved collision mesh count: 17
Loaded triangle count: 119,696
q=0 separated active pairs: 12/12
```

The approved mesh manifest is
`03_CAD/URDF/matt_robodog_rev00/SHA256SUMS.txt`. All 36 entries in that file
(visual plus collision assets) validate against current bytes.

## G4 versus final V5 geometry

The G4 replay and final V5 both find all 24 active-pair contacts. The oracle
matches contact status, angle, pair, bracket, search domain, bisection record,
declared-limit delta and independent path event for every endpoint. Maximum
observed deltas are `6.998e-12 rad` for contact and `5.749e-12 rad` for path,
versus the declared `1e-4 rad` tolerance.

The semantic distinction is deliberate:

```text
geometric endpoint contact: active parent-child meshes intersect
path obstruction:           another relevant pair blocks a sampled path
hardware evidence:          measured LF V25 endpoint, separate consumer
safety policy:              external acceptance rule over measured clearance
```

The legacy `MODEL_INCOMPLETE` endpoints `lf_hip_max` and
`lf_lower_leg_max` are not missing modeled contacts. V5 records their contacts
as geometry; the hardware disagreement moves to the Hardware Reconciler.

The legacy `PATH_COLLISION_BEFORE_ENDPOINT` endpoints `lf_hip_min` and
`rf_hip_max` retain a valid endpoint contact plus an earlier
`body_vs_branch` obstruction. Four lower-leg events are `same_branch` and
occur after endpoint contact. `PATH_OBSTRUCTION` is therefore not synonymous
with cross-leg.

## Final path and parking result

All 24 q=0 start configurations are valid. Six direct task paths are
obstructed and are resolved by one topology-selected parking joint; the other
18 need no auxiliary parking.

| Endpoint | Target domain | Initial blocker relation | Selected 1-DOF parking configuration |
|---|---|---|---|
| `lf_upper_leg_joint:max` | executable | `cross_branch`, `lf_foot_link ↔ lh_foot_link` | `lh_upper_leg_joint=0.610865238198 rad` |
| `lf_lower_leg_joint:min` | diagnostic outside limits | `same_branch`, `lf_hip_link ↔ lf_lower_leg_link` | `lf_upper_leg_joint=1.119919603363 rad` |
| `rf_upper_leg_joint:max` | executable | `cross_branch`, `rf_foot_link ↔ rh_foot_link` | `rh_upper_leg_joint=0.610865238198 rad` |
| `rf_lower_leg_joint:min` | diagnostic outside limits | `same_branch`, `rf_hip_link ↔ rf_lower_leg_link` | `rf_upper_leg_joint=1.119919603363 rad` |
| `rh_lower_leg_joint:min` | diagnostic outside limits | `same_branch`, `rh_hip_link ↔ rh_lower_leg_link` | `rh_upper_leg_joint=1.628973968528 rad` |
| `lh_lower_leg_joint:min` | diagnostic outside limits | `same_branch`, `lh_hip_link ↔ lh_lower_leg_link` | `lh_upper_leg_joint=1.628973968528 rad` |

All four segments (`path_in`, `task_path`, `task_return`, `path_out`) are
collision-free for every selected plan. No 2-DOF candidate was needed. These
are geometric reachability results only. Sixteen endpoint targets lie outside
their URDF limits and remain diagnostic, not executable motion commands.

## Separate hardware reconciliation

The final G12 reconciliation consumes the canonical D combined profile and
the unchanged LF evidence dataset:

```text
geometry profile semantic SHA256:
e99e2b65ea8d032f94b5d1aa815432a292c1771766e556fc7115dc7a1f5de73e

LF evidence file SHA256:
6eae3201a00b5299550028d5b4e1e73d67520deccf5a85e548f3b07b1777cab4

reconciliation content SHA256:
a21325694c8506bfd906a36f29acf609fb9833a81e1d10ab3aa7672fe4d26e23
```

At the fixed 2-degree agreement threshold: 3 LF endpoints AGREE, 3 DISAGREE,
0 lack geometric contact. The 18 non-LF endpoints remain geometry-only. No
evidence is mirrored to another assembly and no geometry is adjusted.

## Separate safety policy

The final G12 safety artifact consumes D parking-v2 semantic SHA256
`3cda03c2c02ba5fbe6def7821ce72d4aa9e4ca66e7ca8655a2b8f0e92dde297c`.
Its own semantic SHA256 is
`6179d82b7edaecd39420903c88c94deb1676338ec84e18889ae3d0cb03fffbe7`.

With the unchanged external 3 mm threshold it reports:

```text
PASS:                              16
FAIL:                               0
UNRESOLVED lower-bound cases:       8
outside-limit cases that pass:      8
motion authorizations granted:      0
geometry mutated:               false
```

The exact frozen G4 policy replay is retained. A clearance PASS is limited to
sampled clearance evidence and never means an outside-limit target is safe or
authorized for execution.

## Determinism and canonical hashes

| Semantic artifact | C W1 | D W4 | Result |
|---|---|---|---|
| Endpoint profile | `cad2f194c49d063b5a09ae4602b9acf61a701de48791e5d1aae04f1439db1211` | same | PASS |
| Parking v2 | `3cda03c2c02ba5fbe6def7821ce72d4aa9e4ca66e7ca8655a2b8f0e92dde297c` | same | PASS |
| Combined profile | `e99e2b65ea8d032f94b5d1aa815432a292c1771766e556fc7115dc7a1f5de73e` | same | PASS |

G4 oracle, input manifest, semantic-source manifest and execution-source
manifest are also exact matches. D uses 12 topology-derived batches over four
spawned processes; workers write zero files, and the parent canonically sorts
24/24 results before one bundle publication.

## A/B/C/D and resources

| Benchmark | Workers | Wall | Peak memory |
|---|---:|---:|---:|
| A | 1 | 3,567.00 s | 541 MB historical |
| B | 1 | 809.33 s | 451.34375 MiB frozen GNU RSS |
| C | 1 | 2,716.84 s | 445.484375 MiB cgroup tree |
| D | 4 | 1,707.39 s | 1,712.636719 MiB cgroup tree |

```text
A/B = 4.407349x
B/C = 0.297894x (not a software speed-up; C/B = 3.356900x)
C/D = 1.591224x
A/D = 2.089154x
```

D uses `27.87495%` of the 6 GiB envelope, with zero swap, zero OOM events and
observed affinity `[0,1,2,3]`. Candidate caps were unchanged. Candidate-pair
and AABB-survivor totals remain explicitly `NOT_INSTRUMENTED`.

## Tests

The final approved offline calibration result is 262/262 PASS: 204 geometry
tests plus 58 other offline calibration tests. An independent 82-test V5 audit
also passes. A broader kinematics diagnostic has 16 subtest errors from a
pre-existing calibration-status mismatch; it is documented in the test
summary and was not changed to manufacture a green result.

## Final artifact map

| Requirement | Artifact |
|---|---|
| Approved 17-mesh manifest | `03_CAD/URDF/matt_robodog_rev00/SHA256SUMS.txt` |
| G4 frozen provenance | `2026-08-10_164419_MATDOG_NEW_GEOMETRY_G0_G4_BASELINE.md` and frozen profile/report/time |
| v4-to-V5 migration | `MATDOG_GEOMETRY_COMPILER_V5_ARCHITECTURE_SCHEMA_MIGRATION_2026-08-10.md` |
| V5 pure endpoint profile/report | D `*_ENDPOINT_PROFILE.json` / `*_ENDPOINT_REPORT.md` |
| V5 combined geometry profile/report | D `*_COMBINED_PROFILE.json` / `*_COMBINED_REPORT.md` |
| V5 path/parking | D `*_PATH_PARKING.json` / `*_PATH_PARKING.md` |
| G4/G7 oracle | D `*_G4_G7_ORACLE.json` / `*_G4_G7_ORACLE.md` |
| LF Hardware Reconciler | `*_G12_LF_HARDWARE_RECONCILIATION.json` / `.md` |
| External safety policy | `*_G12_EXTERNAL_SAFETY_POLICY.json` / `.md` |
| Determinism and W1/W4 | `2026-08-11_072224_MATDOG_GEOMETRY_V5_DETERMINISM_REPORT.md` |
| A/B/C/D | `2026-08-11_072224_MATDOG_GEOMETRY_V5_ABCD_PERFORMANCE_REPORT.md` |
| Tests | `2026-08-11_072224_MATDOG_GEOMETRY_V5_TEST_SUMMARY.md` |
| Architecture | `MATDOG_GEOMETRY_COMPILER_V5_FINAL_ARCHITECTURE_2026-08-11.md` |
| Final validation | this file |

The D run-manifest file SHA256 is
`96ba79875f19c37952e9c981946e5a7df4136b86e1fb5b1c13ffbe12d891b57e`.
All eight D artifact hashes and all eight C artifact hashes independently
validate.

## Change scope

The branch contains only the approved Geometry Compiler V5 work:

- 17 dedicated collision meshes plus URDF collision references and manifest;
- filename-only URDF provenance updates in three active calibration/kinematic
  contracts and the FK loader;
- additive V5 model, scene, contact, profile, report, G4 oracle, process,
  parking, reconciler, safety and integrated-runner modules;
- additive offline tests and the explicitly approved Phase1B expectation
  update for visual/collision separation;
- frozen G0-G9B evidence, final C/D bundles and G12 reports;
- artifact/URDF documentation and persistent context checkpoint.

Historical v1-v4 artifacts and validated Phase1B source files remain
byte-identical. The LF evidence file remains at its approved hash.

## Remaining unknowns

- RF/RH/LH contacts have no hardware oracle and remain geometric candidates.
- Sixteen targets are outside declared limits and diagnostic-only.
- Eight clearance lower bounds remain unresolved under the external 3 mm
  policy.
- Candidate-pair and AABB-survivor counts are not instrumented.
- Target Jetson performance remains unmeasured.
- The unrelated live-FK calibration-status test mismatch remains for its
  owning workflow.

## Permanent constraint declaration

```text
NO HARDWARE USED
NO STATION / SERIAL / SERVO / EEPROM USED
NO GEOMETRY FITTED TO HARDWARE
NO THRESHOLD RELAXED
NO COLLISION CANDIDATE CAP RAISED
NO LF V25 EVIDENCE MODIFIED
NO HISTORICAL v1-v4 ARTIFACT MODIFIED
NO NORMA-CORE MODIFIED
NO HISTORICAL RF NORMACORE WORKTREE MODIFIED
NO FORCE-PUSH
NO MERGE TO MAIN PERFORMED
```

This report is intentionally frozen before the commit that contains it. Its
own commit SHA and resulting Draft PR URL cannot be embedded without a
self-reference cycle; commit, push and Draft PR confirmation are therefore
recorded in the final session handoff. Publication is limited to the
development branch for human review. No merge is authorized.
