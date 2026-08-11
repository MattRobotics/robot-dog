# MATDOG — Geometry Compiler V5 controlled context checkpoint

**Checkpoint date:** 2026-08-10  
**Purpose:** authoritative continuity record before any G5 modification  
**Scope:** completed G0-G4 plus approved G5-G12 execution contract  
**G5 status at checkpoint creation:** **NOT STARTED**

This file is the persistent source of truth for resuming the Geometry Compiler
V5 task after context compaction. Repository files and frozen artifacts take
precedence over conversational memory whenever they are more precise.

Upstream authority documents:

- `/home/matteo-manicardi/Downloads/MATDOG_CODEX_GEOMETRY_COMPILER_V5_EXECUTION_BRIEF_2026-08-10.md`
- `/home/matteo-manicardi/.codex/attachments/b8ece2c4-c4c0-4a50-81c9-9b845a4e581c/pasted-text.txt`
- `09_Logs/Validation_Reports/Geometry_Compiler/2026-08-10_164419_MATDOG_NEW_GEOMETRY_G0_G4_BASELINE.md`

## Authority labels used in this checkpoint

- **APPROVED DECISION** — accepted design or execution boundary; do not reopen
  without new explicit Matteo authorization.
- **VERIFIED FACT** — directly measured or checked in the current worktree.
- **FROZEN BASELINE** — immutable comparison/provenance input for later gates.
- **CURRENT IMPLEMENTATION STATE** — what the existing Phase1B code currently
  does; this may be refactored only within the approved contract.
- **OPEN UNKNOWN** — legitimate unresolved result; never hide or force to PASS.
- **STOP CONDITION** — halt immediately and report before proceeding.

---

## [VERIFIED FACT] Repository, branch, worktree and base

```text
repository:       MattRobotics/robot-dog
origin:           https://github.com/MattRobotics/robot-dog.git
branch:           matdog/geometry-compiler-v5-collision-baseline
worktree:         /home/matteo-manicardi/MATDOG/worktrees/robot-dog-geometry-v5
primary checkout: /home/matteo-manicardi/MATDOG/github/robot-dog
hostname:         matteo-manicardi-K53SV
local HEAD:       e71876e80c23c370f9fecf36ddf15f152faf5eb3
origin/main:      e71876e80c23c370f9fecf36ddf15f152faf5eb3
expected base:    origin/main @ e71876e80c23c370f9fecf36ddf15f152faf5eb3
```

At G0, the default remote branch was `main`, the only remote branch was
`main`, and there were zero open PRs. The operational worktree was clean before
G1. The primary checkout is clean after the authorized mesh transfer.

## [APPROVED DECISION] Permanent Git and scope boundaries

```text
NO merge to main without explicit Matteo authorization
NO force-push
NO history rewrite or destructive Git history operation
NO deletion or overwrite of v1/v2/v3/v4 artifacts
commit and push only matdog/geometry-compiler-v5-collision-baseline
final PR must be Draft
```

The task may proceed autonomously through G5-G12 only while all hard gates and
architectural decisions remain unambiguous.

## [APPROVED DECISION] Execution style

```text
Use Ultra-level reasoning.
Do not perform speculative cleanup or unrelated refactors.
Do not redesign MATDOG beyond the accepted architectural contract.
Prefer measured evidence over assumptions.
Maintain an explicit audit trail for decisions, provenance and measurements.
```

---

## [APPROVED DECISION] Phase 1B is CLOSED

```text
PHASE 1B: CLOSED
G0: PASS
G1: PASS
G2: PASS
G3: PASS
G4: PASS / ACCEPTED AND FROZEN
Benchmark B: ACCEPTED AND FROZEN
```

Do not reopen Phase 1B and do not recreate G0-G4 unless a direct integrity
check detects an unexpected change. Historical schema v4 remains immutable
provenance for the old geometry, not a numerical equality target for the new
geometry.

## [VERIFIED FACT] Completed G0-G4

### G0 — local/remote/filesystem truth: PASS

```text
source staged meshes: 17 exactly
source path:
/home/matteo-manicardi/MATDOG/github/robot-dog/03_CAD/URDF/matt_robodog_rev00/meshes/

SHA256:    17/17 exact
bytes:     5,986,228 / 5,986,228
triangles: 119,696 / 119,696
destination before G1: absent and safe
mapping: canonical and unambiguous by removing _collision
discrepancies: none
```

`NO MODIFICATIONS PERFORMED YET` was explicitly reported before G1.

### G1 — visual/collision separation: PASS

The approved staging meshes were copied byte-for-byte into:

```text
03_CAD/URDF/matt_robodog_rev00/meshes/collision/
```

They were renamed to canonical link filenames and verified 17/17 before the
untracked staging source copies were removed. The detailed files directly
under `meshes/` were not modified by G1 and remain visual geometry.

Only 17 URDF values changed:

```text
<visual>    -> meshes/<link>.stl
<collision> -> meshes/collision/<link>.stl
```

The URDF remains named:

```text
matt_robodog_rev00.urdf
```

Changing collision representation does not create a new mechanical revision.
Collision origins, mesh scales, joint origins, axes, limits, inertials and all
non-collision-path XML remained byte-identical. Evidence:

```text
pre-change URDF SHA256:
5e28da3dba10fd3f2ea6ebf6f5d6271157bda0b12b82d92aedbe3031643089ef

updated URDF SHA256:
3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59

updated URDF normalized by replacing meshes/collision/ with meshes/:
5e28da3dba10fd3f2ea6ebf6f5d6271157bda0b12b82d92aedbe3031643089ef
```

### G2 — mesh/frame/integrity: PASS

```text
17/17 binary STL count/size consistency
17/17 exact SHA256
17/17 production RobotScene load
17/17 finite vertices and normals
0 degenerate triangles across 119,696 triangles
17/17 watertight; every undirected edge incidence = 2
17/17 consistent winding/stored normals
17/17 positive signed volume orientation
17/17 URDF scale = 0.001 0.001 0.001
17/17 collision origin = identity
local AABBs and units sane
feet centred
upper-leg baked frame confirmed
handedness/symmetry checked
```

The complete G2 AABBs and symmetry measurements are frozen in the G0-G4
baseline report listed below.

### G3 — q=0 hard gate: PASS

```text
base <-> hip:    4/4 SEPARATED_NARROW
hip <-> upper:   4/4 SEPARATED_NARROW
upper <-> lower: 4/4 SEPARATED_NARROW
TOTAL:           12/12 SEPARATED
```

Production Phase1B collision settings and candidate caps were used unchanged.
No tolerance relaxation occurred.

### G4 — new-geometry Phase1B reference replay: ACCEPTED AND FROZEN

```text
geometry:                   approved 17 dedicated collision meshes
collision triangles:        119,696
algorithm:                  unchanged validated Phase1B
workers:                    1
result kind:                24/24 MESH_CONTACT_FOUND
MODELED_ENDSTOP_CONTACT:    20
MODEL_INCOMPLETE:           2
PATH_COLLISION_BEFORE_ENDPOINT: 2
NO_MODELED_ENDSTOP:         0
content SHA256:
4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61
```

The G4 content hash was recomputed independently and matched. Comparing the
profile's URDF, collision mesh and compiler-source manifests with the live
files returned no mismatches.

---

## [FROZEN BASELINE] Approved 17 collision mesh manifest

Do not alter, replace, retessellate, rename or fit these meshes to historical
v4 or hardware endpoint numbers.

| Canonical collision filename | Triangles | Bytes | SHA256 |
|---|---:|---:|---|
| `base_link.stl` | 22,044 | 1,102,284 | `0a485e7a1101d457f317b664e52a7a4ef061382e6ecc2c7c87a333c81d5b466f` |
| `lf_foot_link.stl` | 2,248 | 112,484 | `e4d35cd4107bd7fad2d3fe511926ec4fc5211f15f978581e469787266ebe5357` |
| `lf_hip_link.stl` | 6,016 | 300,884 | `b2e430aa99998791879f1ab80c46787a310c80f77a3df9116d1084e0a0e27d97` |
| `lf_lower_leg_link.stl` | 12,336 | 616,884 | `bbb40400571fab554395d5671ad86b1b106ec3fe24fce733ef0be179a739508a` |
| `lf_upper_leg_link.stl` | 3,802 | 190,184 | `0830fc10d8873b6a45f2f58c2ca61f08a7f38092984a96b911ac6beed274c30c` |
| `lh_foot_link.stl` | 2,248 | 112,484 | `e4d35cd4107bd7fad2d3fe511926ec4fc5211f15f978581e469787266ebe5357` |
| `lh_hip_link.stl` | 6,018 | 300,984 | `5350010e3623ec36301d2c0592e3209e1b06da83e823215caf55cca5aafe86da` |
| `lh_lower_leg_link.stl` | 12,336 | 616,884 | `bbb40400571fab554395d5671ad86b1b106ec3fe24fce733ef0be179a739508a` |
| `lh_upper_leg_link.stl` | 3,802 | 190,184 | `0830fc10d8873b6a45f2f58c2ca61f08a7f38092984a96b911ac6beed274c30c` |
| `rf_foot_link.stl` | 2,248 | 112,484 | `18dfa7908e7410cc2920d5d37b0c0cce13cb2c683f765e7d073b3d5777c00d33` |
| `rf_hip_link.stl` | 6,016 | 300,884 | `83a7622f990228e6c2b6bb0431e682fc462a76feb3ce2fe190b7553be879dc33` |
| `rf_lower_leg_link.stl` | 12,352 | 617,684 | `6b40148edad22670c420d1bcca2325c8befd72e27f4390569a292250df3e96c8` |
| `rf_upper_leg_link.stl` | 3,806 | 190,384 | `fbf43f047a943188a6c7b9a7a3a45763c22ddb35704a517b7b47b03dcb68cb91` |
| `rh_foot_link.stl` | 2,248 | 112,484 | `18dfa7908e7410cc2920d5d37b0c0cce13cb2c683f765e7d073b3d5777c00d33` |
| `rh_hip_link.stl` | 6,018 | 300,984 | `04b205e9be3683b5238a035970b56ee319e4e7fca55ba7860a19fdb549c4efea` |
| `rh_lower_leg_link.stl` | 12,352 | 617,684 | `6b40148edad22670c420d1bcca2325c8befd72e27f4390569a292250df3e96c8` |
| `rh_upper_leg_link.stl` | 3,806 | 190,384 | `fbf43f047a943188a6c7b9a7a3a45763c22ddb35704a517b7b47b03dcb68cb91` |
| **TOTAL** | **119,696** | **5,986,228** | **17/17 exact** |

Historical collision set: 472,020 triangles. Approved reduction: 74.64%.

---

## [FROZEN BASELINE] G4 artifacts and provenance

Artifact directory:

```text
/home/matteo-manicardi/MATDOG/worktrees/robot-dog-geometry-v5/
09_Logs/Validation_Reports/Geometry_Compiler/
```

| Artifact | File SHA256 |
|---|---|
| `2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_PROFILE.json` | `f1b059a58c51508345ec583cc421cf9ca66ec8e9ac5547649c05bbf755e5c5fa` |
| `2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_REPORT.md` | `140c0aa5a06e3d2855222a28cf06d8de417a391938e954607e1b91b49e4e580c` |
| `2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_BENCHMARK_B_TIME.txt` | `02e89f8431ebd94735964654964b1b726e9efb4f6175b917ffff8ebef689ea78` |
| `2026-08-10_164419_MATDOG_NEW_GEOMETRY_G0_G4_BASELINE.md` | `53ba3165b5398ef179afb1abfdfe6036bc4f8b2f749d316b73a4a4e11d017920` |

G4 profile provenance:

```text
profile content SHA256:
4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61

URDF input SHA256:
3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59

Phase1B combined source SHA256:
066079118fb2cd0f59671ae5afa566b001bdada57395690cc64a7e8c866d2ee0
```

Frozen Phase1B source file hashes:

| Source | SHA256 |
|---|---|
| `matdog_geometry_compiler.py` | `fa23b952ddb8dc6249b0a66319cfcbf6d5a2c69c0b4d66d9ca754405c9e86ef6` |
| `matdog_geometry_contact_search.py` | `3c330e0565f8e296eb55981fc221a4b043e0cee6eb2caafb868e5e04fbe997bf` |
| `matdog_geometry_mesh_kernel.py` | `f7119fd93b913968ea342269b4369546ecbcad2e8731e7d4d8fabd4ba08c017c` |
| `matdog_geometry_path_planner.py` | `a0d12f98dc2a89fedc13a5a4e4f8d5d062bfa523e1b39967b036fa851b0ccc5b` |
| `matdog_geometry_profile.py` | `d1175234cb0362d581b392df9c9eb12d2fd7d59d7dd2c49977c0a6f3a949c62a` |
| `matdog_geometry_scene.py` | `6c5043e053453f5215f9ebab5ade9ea781703119b884f2719d78910e77bbb2c9` |
| `matdog_geometry_uncertainty.py` | `6f11b9577e71d695e6723f4e3aa23453dad707e0200dfe0c0f00f8d090de4152` |

Do not overwrite these G4 artifacts when producing G7/V5 outputs.

---

## [FROZEN BASELINE] G4 endpoint/contact summary

All 24 active parent-child revolute geometric contacts were found. The legacy
Phase1B status can additionally incorporate hardware or path semantics; those
legacy classifications are not pure-geometry truth.

| Endpoint | Geometric contact (deg) | Active contact pair | Frozen legacy status |
|---|---:|---|---|
| `lf_hip_min` | `-46.011719` | `base_link ↔ lf_hip_link` | `PATH_COLLISION_BEFORE_ENDPOINT` |
| `lf_hip_max` | `+45.222656` | `base_link ↔ lf_hip_link` | `MODEL_INCOMPLETE` |
| `lf_upper_leg_min` | `-52.132813` | `lf_hip_link ↔ lf_upper_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `lf_upper_leg_max` | `+121.875000` | `lf_hip_link ↔ lf_upper_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `lf_lower_leg_min` | `-92.074219` | `lf_upper_leg_link ↔ lf_lower_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `lf_lower_leg_max` | `+38.179688` | `lf_upper_leg_link ↔ lf_lower_leg_link` | `MODEL_INCOMPLETE` |
| `rf_hip_min` | `-45.222656` | `base_link ↔ rf_hip_link` | `MODELED_ENDSTOP_CONTACT` |
| `rf_hip_max` | `+46.011719` | `base_link ↔ rf_hip_link` | `PATH_COLLISION_BEFORE_ENDPOINT` |
| `rf_upper_leg_min` | `-52.132813` | `rf_hip_link ↔ rf_upper_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `rf_upper_leg_max` | `+121.875000` | `rf_hip_link ↔ rf_upper_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `rf_lower_leg_min` | `-92.074219` | `rf_upper_leg_link ↔ rf_lower_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `rf_lower_leg_max` | `+38.179688` | `rf_upper_leg_link ↔ rf_lower_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `rh_hip_min` | `-45.156250` | `base_link ↔ rh_hip_link` | `MODELED_ENDSTOP_CONTACT` |
| `rh_hip_max` | `+46.011719` | `base_link ↔ rh_hip_link` | `MODELED_ENDSTOP_CONTACT` |
| `rh_upper_leg_min` | `-52.132813` | `rh_hip_link ↔ rh_upper_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `rh_upper_leg_max` | `+121.875000` | `rh_hip_link ↔ rh_upper_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `rh_lower_leg_min` | `-92.074219` | `rh_upper_leg_link ↔ rh_lower_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `rh_lower_leg_max` | `+38.179688` | `rh_upper_leg_link ↔ rh_lower_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `lh_hip_min` | `-46.011719` | `base_link ↔ lh_hip_link` | `MODELED_ENDSTOP_CONTACT` |
| `lh_hip_max` | `+45.156250` | `base_link ↔ lh_hip_link` | `MODELED_ENDSTOP_CONTACT` |
| `lh_upper_leg_min` | `-52.132813` | `lh_hip_link ↔ lh_upper_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `lh_upper_leg_max` | `+121.875000` | `lh_hip_link ↔ lh_upper_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `lh_lower_leg_min` | `-92.074219` | `lh_upper_leg_link ↔ lh_lower_leg_link` | `MODELED_ENDSTOP_CONTACT` |
| `lh_lower_leg_max` | `+38.179688` | `lh_upper_leg_link ↔ lh_lower_leg_link` | `MODELED_ENDSTOP_CONTACT` |

G4 evidence classes:

```text
17 GEOMETRIC_ENDPOINT_CANDIDATE
3  HARDWARE_CONFIRMED_CONTACT
2  HARDWARE_CONTRADICTED
2  PATH_LIMITED
```

### [FROZEN BASELINE] Two legacy `MODEL_INCOMPLETE` LF cases

These are not missing geometric contacts. They are geometry/hardware
disagreements and must become ordinary geometric-contact records in the V5
pure-geometry profile, with hardware comparison moved to the reconciler.

```text
lf_hip_max
geometry: +45.222656 deg (~+45.223)
LF V25 hardware: +39.375 deg

lf_lower_leg_max
geometry: +38.179688 deg (~+38.180)
LF V25 hardware: +34.277 deg
```

### [FROZEN BASELINE] Two legacy `PATH_COLLISION_BEFORE_ENDPOINT` cases

Both still have active-pair geometric contacts, but a path obstruction occurs
earlier along the legacy task path:

```text
lf_hip_min
geometric active-pair contact: -46.011719 deg
earlier path obstruction: base_link ↔ lf_lower_leg_link @ -45.296875 deg

rf_hip_max
geometric active-pair contact: +46.011719 deg
earlier path obstruction: base_link ↔ rf_lower_leg_link @ +45.296875 deg
```

### [FROZEN BASELINE] All six recorded G4 path events

| Endpoint | Path event |
|---|---|
| `lf_hip_min` | `base_link ↔ lf_lower_leg_link @ -45.296875°` |
| `rf_hip_max` | `base_link ↔ rf_lower_leg_link @ +45.296875°` |
| `lf_lower_leg_min` | `lf_foot_link ↔ lf_upper_leg_link @ -97.968750°` |
| `lh_lower_leg_min` | `lh_foot_link ↔ lh_upper_leg_link @ -97.968750°` |
| `rf_lower_leg_min` | `rf_foot_link ↔ rf_upper_leg_link @ -97.972656°` |
| `rh_lower_leg_min` | `rh_foot_link ↔ rh_upper_leg_link @ -97.972656°` |

Only the first two occur before the active-pair endpoint and therefore drive
the two legacy path-limited endpoint statuses. The four lower-leg path events
occur beyond the active-pair endpoint and remain diagnostic path evidence.

### [FROZEN BASELINE] Legacy G4 parking output

```text
LF: park LH upper at +30 deg; park path PASS; sequence passed=False;
    minimum reported clearance 0.101410 mm LOWER_BOUND
RF: park RH upper at +30 deg; park path PASS; sequence passed=False;
    minimum reported clearance 0.101410 mm LOWER_BOUND
RH: auxiliary parking not required; sequence passed=False;
    minimum reported clearance 1.000000 mm LOWER_BOUND
LH: auxiliary parking not required; sequence passed=False;
    minimum reported clearance 1.000000 mm LOWER_BOUND
```

The four `passed=False` results are legacy 3 mm safety-policy findings, not
residual true mesh intersections after selected parking. Preserve this G4
provenance; do not reinterpret it as V5 geometry failure.

---

## [APPROVED DECISION] Fundamental semantic separation

The following are separate facts and must never be conflated:

```text
geometric endpoint contact
!=
path reachability / path obstruction
!=
hardware contact evidence
!=
safety-policy acceptance
```

Definitions:

- **Geometric endpoint contact:** collision geometry reports contact on the
  active revolute parent-child pair within a declared search domain.
- **Path obstruction:** another collision pair blocks or obstructs a task path;
  it does not erase an active-pair contact that exists later in the domain.
- **Hardware evidence:** immutable empirical LF V25 measurements compared in a
  separate Hardware Reconciler; never geometry-core truth.
- **Safety policy:** an external rule over feasibility/clearance, such as the
  historical 3 mm acceptance bar; never the definition of geometric validity.

### [APPROVED DECISION] Path obstruction is not automatically cross-leg

The legacy label `PATH_COLLISION_BEFORE_ENDPOINT` must not be interpreted as
automatically cross-leg. For example:

```text
base_link ↔ lf_lower_leg_link
base_link ↔ rf_lower_leg_link
```

are body-vs-active-branch obstructions, not collisions between two legs. V5
must use a geometry-neutral `PATH_OBSTRUCTION` concept and may independently
classify relation as, for example:

```text
same_branch
body_vs_branch
cross_branch
```

Relationship classification is reporting metadata and must not determine
endpoint geometry.

---

## [APPROVED DECISION] V5 pure-geometry architecture

```text
Python
= algorithms

URDF
= robot definition
  links and topology
  joint types
  parent / child
  joint origins
  axes
  limits
  motorId / motorDirection
  collision mesh paths
  per-mesh scales
  collision origins

collision STL
= collision geometry

hardware evidence
= separate immutable empirical dataset and Hardware Reconciler

safety policy
= separate external acceptance layer
```

The Geometry Compiler V5 core must be pure geometry. Hardware evidence and
safety policy may consume geometry output but must not determine it.

## [APPROVED DECISION] Structural selection rules

The intended 12 actuated/calibratable revolute joints must be selected from
actual URDF/model metadata. Do not infer structural semantics from names such
as `lf_`, `rf_`, `rh_`, `lh_`, `hip`, `upper` or `lower`; names may be used
only as presentation labels.

If current URDF metadata cannot unambiguously select exactly the intended 12
joints, stop and propose the minimal explicit MATDOG URDF metadata contract.
Do not invent a hidden Python naming convention.

---

## [CURRENT IMPLEMENTATION STATE] Phase1B code before G5

The frozen Phase1B production source files remain byte-identical to the G4
source manifest. G5 code changes have not started.

Known coupling/duplication to inspect and remove or isolate coherently:

- `matdog_geometry_scene.py` contains model-shape assumptions including global
  `URDF_MESH_SCALE`, `LEG_IDS`, joint-group/name conventions and
  `SERVO_ID_BY_JOINT`; it currently rejects non-global scale and non-identity
  collision origins rather than honoring per-model data.
- `matdog_geometry_contact_search.py` contains legacy LF V25 hardware evidence,
  endpoint/hardware classification and prerequisite conventions.
- `matdog_geometry_path_planner.py` contains mandatory legacy prerequisite
  poses, fixed parking seed angles `30,40,50,60,70,80,90` degrees and the 3 mm
  legacy clearance acceptance threshold.
- `matdog_geometry_profile.py` serializes schema v4, including hardware-coupled
  evidence/status fields and LF reconciliation in the geometry profile.
- `matdog_geometry_compiler.py` orchestrates named leg groups, legacy parking,
  endpoint search, uncertainty and v4 serialization sequentially.
- `matdog_urdf_fk.py` exposes a hardcoded canonical URDF SHA constant used by
  parts of the broader kinematics stack; the V5 geometry core must not require
  a hardcoded URDF hash as model truth.
- `matdog_geometry_mesh_kernel.py` is the validated collision kernel and should
  be changed only where required for model-driven transforms, deterministic
  instrumentation or measured optimization; do not change collision semantics
  casually.

Before each material change, re-read the exact implementation, separate
structural model truth from algorithm and make the smallest coherent refactor.

---

## [APPROVED DECISION] G5 — pure-geometry refactor requirements

Remove from geometry-core structural truth, where technically possible:

```text
hardcoded canonical URDF SHA requirement
global URDF_MESH_SCALE assumption
SERVO_ID_BY_JOINT duplicate table
LEG_IDS / JOINT_GROUPS as required structural truth
LF_V25_HARDWARE_EVIDENCE in geometry core
mandatory +50 deg / +90 deg prerequisite assumptions
mandatory fixed 30/40/50/60/70/80/90 deg parking seed table
3 mm clearance threshold as geometry truth
mandatory historical parking seed list as geometry truth
```

Read from URDF/model instead:

```text
links
joint topology and types
parent / child
collision paths
per-mesh scale
collision origin
joint origin
axis
limits
motorId
motorDirection
```

G5 must preserve frozen geometry/search behaviour wherever the underlying
algorithm is intentionally unchanged. It must not preserve hardware/safety
coupling merely to reproduce schema-v4 verdicts.

---

## [APPROVED DECISION] G6 — schema decision and migration mapping

Do not bump schema merely because the milestone is named V5. Removing
hardware-dependent semantics from the canonical geometry profile is a real
semantic change and is expected to require a new schema.

Before implementing canonical V5 serialization, produce a concise but complete
field-by-field mapping:

```text
v4 -> proposed V5
```

Classify every relevant v4 field as exactly one of:

```text
KEEP
RENAME
RESTRUCTURE
MOVE TO HARDWARE RECONCILER
MOVE TO SAFETY POLICY
REMOVE FROM PURE GEOMETRY
```

Acceptable pure-geometry concepts include:

```text
GEOMETRIC_CONTACT_FOUND
NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN
PATH_OBSTRUCTION
geometric clearance
contact pair
search domain
numerical resolution
model provenance
```

Pure-geometry output must not contain hardware verdicts/fields such as:

```text
HARDWARE_CONFIRMED_CONTACT
HARDWARE_CONTRADICTED
mesh_vs_hardware_status
hardware_vs_urdf_status
```

Those fields move to the Hardware Reconciler. Preserve all v1/v2/v3/v4
artifacts byte-for-byte. If the mapping exposes a genuinely unresolved
architectural decision, stop before choosing a design.

### [APPROVED DECISION] Schema-v4 provenance rule

`matdog.calibration_geometry_profile.v4` and its historical artifacts remain
immutable evidence of Phase1B and the old geometry/policy. Do not overwrite,
reinterpret or use old-geometry v4 endpoint equality as a V5 gate. A new schema
must coexist with, not rewrite, v1-v4 provenance.

---

## [APPROVED DECISION] G7 — same-new-geometry oracle

Run V5 on the same approved 17 collision meshes. The primary refactor oracle is:

```text
G4 = new geometry + unchanged validated Phase1B algorithm
G7 = same new geometry + V5 pure-geometry algorithm

where search is intentionally unchanged:
G7 geometric result ≈ G4 geometric result
within the predeclared numerical search resolution
```

Do not compare against historical old-geometry v4 endpoint equality. Explain
every G4/G7 difference and classify it as:

```text
expected semantic separation
expected representation/schema difference
intentional algorithm change
unexpected regression
```

Any unexpected geometric regression is a stop condition. The two legacy
`MODEL_INCOMPLETE` and two path-limited cases still contain valid G4 geometric
active-pair contacts; pure geometry must preserve those contacts unless an
explicit algorithm change is justified.

---

## [APPROVED DECISION] G8 — separate Hardware Reconciler

LF V25 is immutable external evidence:

```text
HIP MIN       -42.803 deg
HIP MAX       +39.375 deg
UPPER MIN     -53.525 deg
UPPER MAX    +122.607 deg
LOWER MIN     -91.846 deg
LOWER MAX      +34.277 deg
```

Implement/extract a separate layer and output:

```text
Geometry V5 profile
+ LF V25 hardware evidence
= hardware reconciliation report
```

The reconciler may compare geometry and hardware. It must never modify the
geometry profile, meshes, endpoints, URDF, contact search or thresholds to
improve agreement. RF/RH/LH remain geometry-only predictions until future
hardware validation.

---

## [APPROVED DECISION] G9 — geometry-driven path and parking

The G4 legacy-policy replay stays frozen. Canonical V5 must not use fixed
parking seeds `30,40,50,60,70,80,90` degrees as geometry truth.

Required deterministic search strategy:

1. Validate starting/home configuration.
2. Identify the first blocking pair.
3. Identify relevant movable articulated branch/joints.
4. Perform deterministic 1-DOF escape search within URDF limits.
5. Only if 1-DOF is insufficient, perform controlled 2-DOF search on implicated
   joints.
6. Validate path into the parking configuration.
7. Validate the complete calibration/task path.
8. Validate return path.

Fixed optimization order:

```text
1. zero mesh intersection
2. maximize minimum geometric clearance
3. minimize displacement from home
4. deterministic tie-break
```

A correct result may be:

```text
NO FEASIBLE PARKING FOUND IN DECLARED SEARCH DOMAIN
```

Do not force PASS and do not modify geometry merely to obtain parking.

## [APPROVED DECISION] G9B — geometry feasibility vs clearance vs safety

```text
geometric feasibility = intersection / no intersection
clearance             = measured geometric quantity
safety policy         = external acceptance rule
```

The historical 3 mm gate may remain documented and reproducible as a
legacy/reference external safety policy. It must not define geometric truth.
Do not reduce/change thresholds merely to obtain PASS.

---

## [APPROVED DECISION] G10 — offline test contract

Add strong tests proving at minimum:

```text
changing a URDF collision filename changes loaded collision geometry
without Python edits

per-mesh URDF scale is honored
collision origin is honored
motorId comes from URDF/model
motorDirection comes from URDF/model
actuated/calibratable joints are selected model-first
V5 geometry core does not import LF hardware evidence
Hardware Reconciler is separate from geometry core
safety policy is separate from geometry core
PATH_OBSTRUCTION is not automatically cross-leg
historical 3 mm is not required for geometry feasibility
two identical V5 runs have identical semantic content except explicitly
non-semantic timestamp/runtime fields
workers=1 and workers=4 have identical semantic content
v1/v2/v3/v4 artifacts are untouched
```

Preserve and run all relevant pre-existing geometry tests. Do not alter old
tests only to hide an unexpected regression; update expectations only for
explicitly approved semantic/schema changes.

---

## [APPROVED DECISION] G11 — deterministic process parallelism

Canonical deployment/benchmark profile:

```text
workers:              4
worker model:         processes, not Python threads, for CPU-bound work
physical-core target: one worker per physical core
OMP_NUM_THREADS:      1
OPENBLAS_NUM_THREADS: 1
MKL_NUM_THREADS:      1
NUMEXPR_NUM_THREADS:  1
MemoryMax:            6 GiB process tree where supported
MemorySwapMax:        0 where supported
```

Prefer coarse independent articulated-branch/task partitioning. If fixed leg
names are removed, partition deterministically by independent top-level
articulated branches derived from topology.

```text
workers -> calculate isolated deterministic results
parent  -> collect -> canonical sort -> serialize exactly once
```

Workers must not concurrently write canonical artifacts. Do not increase
collision candidate caps because more RAM is available. Parallelism may change
time but must not change semantic geometry:

```text
workers=1 semantic content == workers=4 semantic content
```

Exclude only explicitly non-semantic timestamp/runtime/benchmark metadata from
that comparison.

---

## [FROZEN BASELINE / REQUIRED] A/B/C/D performance matrix

```text
A — HISTORICAL
old geometry + validated Phase1B compiler
1 worker
wall: 3567 s = 59m27s
peak RSS: 541 MB historical record

B — MESH EFFECT (FROZEN)
new geometry + unchanged validated Phase1B algorithm
1 worker
wall: 809.33 s = 13m29.33s
peak RSS: 462176 KiB = 451.34375 MiB = 473.268224 MB decimal
swap: 0
exit: 0
endpoints: 24/24

C — SOFTWARE EFFECT (NOT STARTED)
new geometry + final V5 pure-geometry compiler
1 worker

D — DEPLOYMENT PROFILE (NOT STARTED)
new geometry + final V5 pure-geometry compiler
4 workers
6 GiB process-tree envelope
swap 0 where supported
```

Required calculations:

```text
mesh speed-up     = A / B = 4.40735x (already frozen)
software speed-up = B / C
parallel speed-up = C / D
total speed-up    = A / D
```

Normalize memory units before RAM percentage comparisons. Report where
instrumented:

```text
wall time
peak process-tree RSS
swap
triangle count
grid entries
candidate pairs
AABB-surviving pairs
worker count
content hash
```

Phase1B did not expose grid-entry, unique-candidate or AABB-survivor counters
for B. Do not materially instrument or rerun the frozen reference algorithm
merely to obtain those historical counters. The B systemd user-manager
post-run `Memory peak: 256.0K` was implausible and is not canonical; GNU
`time -v` peak RSS `462176 KiB` is the accepted B measurement. Swap was zero.

## [APPROVED DECISION] Resource discipline

The target is deterministic efficient geometry processing suitable for later
Jetson Orin Nano Super deployment, not cycle-accurate emulation.

```text
do not optimize blindly
do not trade correctness for speed
do not raise caps merely because RAM is available
prefer better data structures
prefer less duplicated geometry work
prefer coarse process parallelism
prefer deterministic aggregation
record resource usage explicitly
```

Any optimization that changes geometry semantics is an algorithm change and
must be separately justified against G4/G7.

---

## [APPROVED DECISION] G12 — final artifacts and Draft PR

Produce all of:

```text
approved 17-mesh manifest
G4 frozen reference replay provenance
V5 pure geometry profile
V5 geometry report
V5 path/parking report
separate LF hardware reconciliation report
v4 -> V5 schema migration note
determinism report
workers=1 vs workers=4 comparison
A/B/C/D performance report
test summary
architectural summary
```

Then intentionally commit and push only:

```text
matdog/geometry-compiler-v5-collision-baseline
```

Prepare a Draft PR for human review. Do not merge.

Final report must include:

```text
gate-by-gate status
files changed
tests executed/results
G4 vs G7 geometric comparison
schema v4 -> V5 decision
path/parking outcome
Hardware Reconciler outcome
workers 1 vs 4 determinism
benchmark A/B/C/D
resource usage
commits
pushed branch
Draft PR
remaining UNKNOWNs

NO HARDWARE USED
NO NORMA-CORE MODIFIED
NO MERGE PERFORMED
```

---

## [STOP CONDITION] Mandatory immediate stops

Stop and report before continuing if any occurs:

```text
frozen G4 provenance/content hash changed unexpectedly
any approved collision mesh hash changed unexpectedly
12/12 q=0 revolute separation no longer holds
any new mesh becomes nonfinite, degenerate or non-watertight unexpectedly
historical v1/v2/v3/v4 artifact changes/deletion/overwrite
visual geometry accidentally changes
joint origins, axes, limits or inertials change without separately approved reason
URDF metadata cannot unambiguously identify the intended 12 actuated joints
pure geometry requires LF hardware evidence
V5 core imports/uses LF hardware data to define geometry output
a geometric endpoint is altered to reproduce LF hardware or historical v4
geometry, search or threshold is fitted to hardware
a tolerance/safety rule is relaxed merely to obtain PASS
fixed collision candidate caps are raised merely because RAM is available
workers=1 and workers=4 produce different semantic geometry
an unexpected G4 vs G7 geometric regression appears
resource envelope cannot be respected safely
norma-core is touched
historical RF NormaCore worktree is touched
hardware, Station, serial, servo or EEPROM access occurs
merge to main is attempted
a material architectural decision is required but not resolved by the accepted
contract/brief
context compaction continuity verification fails before G5
```

Do not hide, reinterpret or automatically resolve a stop condition.

## [APPROVED DECISION] Permanent hardware and repository isolation

```text
NO hardware access
NO Station
NO serial
NO servo commands or motion
NO EEPROM
NO LF V25 evidence alteration
NO geometry fitting to hardware
NO norma-core changes
NO historical RF NormaCore worktree changes
NO main merge
```

---

## [CURRENT IMPLEMENTATION STATE] Modified and untracked files

Status captured immediately before creating this checkpoint:

```text
## matdog/geometry-compiler-v5-collision-baseline...origin/main
 M 03_CAD/URDF/matt_robodog_rev00/README.md
 M 03_CAD/URDF/matt_robodog_rev00/SHA256SUMS.txt
 M 03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf
 M 09_Logs/Validation_Reports/Geometry_Compiler/README.md
?? 03_CAD/URDF/matt_robodog_rev00/meshes/collision/
?? 09_Logs/Validation_Reports/Geometry_Compiler/2026-08-10_164419_MATDOG_NEW_GEOMETRY_G0_G4_BASELINE.md
?? 09_Logs/Validation_Reports/Geometry_Compiler/2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_BENCHMARK_B_TIME.txt
?? 09_Logs/Validation_Reports/Geometry_Compiler/2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_PROFILE.json
?? 09_Logs/Validation_Reports/Geometry_Compiler/2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_REPORT.md
```

The new checkpoint file itself becomes an additional untracked file after that
snapshot:

```text
?? 06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_V5_CONTEXT_CHECKPOINT_2026-08-10.md
```

Exact untracked collision meshes:

```text
03_CAD/URDF/matt_robodog_rev00/meshes/collision/base_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/lf_foot_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/lf_hip_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/lf_lower_leg_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/lf_upper_leg_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/lh_foot_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/lh_hip_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/lh_lower_leg_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/lh_upper_leg_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/rf_foot_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/rf_hip_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/rf_lower_leg_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/rf_upper_leg_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/rh_foot_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/rh_hip_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/rh_lower_leg_link.stl
03_CAD/URDF/matt_robodog_rev00/meshes/collision/rh_upper_leg_link.stl
```

There are no Phase1B core source modifications yet. Historical v1-v4 artifacts
and `norma-core` have no diff.

---

## [CURRENT IMPLEMENTATION STATE] Important paths for G5

Robot model and assets:

```text
03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf
03_CAD/URDF/matt_robodog_rev00/README.md
03_CAD/URDF/matt_robodog_rev00/SHA256SUMS.txt
03_CAD/URDF/matt_robodog_rev00/meshes/
03_CAD/URDF/matt_robodog_rev00/meshes/collision/
```

Geometry core/refactor surface:

```text
06_Software/Matdog_Core/calibration/matdog_geometry_compiler.py
06_Software/Matdog_Core/calibration/matdog_geometry_scene.py
06_Software/Matdog_Core/calibration/matdog_geometry_mesh_kernel.py
06_Software/Matdog_Core/calibration/matdog_geometry_contact_search.py
06_Software/Matdog_Core/calibration/matdog_geometry_path_planner.py
06_Software/Matdog_Core/calibration/matdog_geometry_uncertainty.py
06_Software/Matdog_Core/calibration/matdog_geometry_profile.py
06_Software/Matdog_Core/calibration/matdog_geometry_report.py
06_Software/Matdog_Core/kinematics/matdog_urdf_fk.py
```

Relevant tests:

```text
06_Software/Matdog_Core/calibration/tests/test_matdog_geometry_scene.py
06_Software/Matdog_Core/calibration/tests/test_matdog_geometry_mesh_kernel.py
06_Software/Matdog_Core/calibration/tests/test_matdog_geometry_contact_search.py
06_Software/Matdog_Core/calibration/tests/test_matdog_geometry_path_planner.py
06_Software/Matdog_Core/calibration/tests/test_matdog_geometry_profile.py
06_Software/Matdog_Core/calibration/tests/test_matdog_geometry_phase1b_policy.py
06_Software/Matdog_Core/calibration/tests/test_matdog_geometry_checkpoint_regression.py
06_Software/Matdog_Core/calibration/tests/test_matdog_geometry_uncertainty.py
```

Historical/architectural provenance:

```text
06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_PHASE1B_ADDENDUM_2026-08-08.md
06_Software/Matdog_Core/calibration/MATDOG_CALIBRATION_CANONICAL_HANDOFF_PHASE1B_CLOSED_2026-08-09.md
06_Software/Matdog_Core/calibration/MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md
09_Logs/Validation_Reports/Geometry_Compiler/README.md
09_Logs/Validation_Reports/Geometry_Compiler/2026-08-10_164419_MATDOG_NEW_GEOMETRY_G0_G4_BASELINE.md
```

---

## [CURRENT IMPLEMENTATION STATE] Completed vs not started

Completed:

```text
brief read and accepted
G0 local/remote/filesystem truth
G1 mesh transfer + URDF visual/collision separation
G2 integrity/frame/symmetry audit
G3 q=0 12/12 hard gate
G4 same-new-geometry Phase1B replay
Benchmark B
G0-G4 persistent baseline report
URDF asset README and SHA256SUMS update
Geometry Compiler artifact index update
this pre-G5 context checkpoint
```

Not started:

```text
G5 pure-geometry source refactor
G6 v4 -> proposed V5 field mapping and schema implementation
G7 V5 pure-geometry run/oracle comparison
G8 separate Hardware Reconciler implementation/output
G9 geometry-driven path/parking implementation/output
G9B external safety-policy separation implementation
G10 new V5 test contract and full regression validation
G11 4-process deterministic parallel implementation
Benchmark C
Benchmark D
A/B/C/D final report
G12 final artifact set
commit
push
Draft PR
```

---

## [APPROVED DECISION] Decisions that must not be reconsidered

```text
The 17 approved collision meshes are the geometry to retain.
Detailed meshes remain VISUAL geometry.
meshes/collision/ remains COLLISION geometry.
The mechanical revision remains matt_robodog_rev00.
Phase 1B is closed.
G4 and Benchmark B are accepted/frozen.
Historical old-geometry v4 endpoint equality is not a V5 target.
G4 same-new-geometry is the G7 software-refactor oracle.
Pure geometry, hardware evidence and safety policy are separate layers.
The two legacy MODEL_INCOMPLETE LF records contain valid geometric contacts.
The two legacy path-limited records contain valid geometric contacts plus
earlier path obstructions.
PATH_OBSTRUCTION is not automatically cross-leg.
LF hardware evidence never changes geometry.
RF/RH/LH remain geometry-only until hardware validation.
The historical 3 mm threshold is not geometric truth.
The legacy fixed parking seed list is not geometric truth.
Model structure must come from URDF metadata, not link/joint name parsing.
Workers=1 and workers=4 must be semantically identical.
Process parallelism uses four workers and one numerical thread each.
The 6 GiB process-tree and zero-swap contract remains in force.
Candidate caps are not raised merely for available memory.
No hardware, norma-core or main merge is authorized.
```

---

## [OPEN UNKNOWN] Legitimate unresolved items

These are not failures and must not be silently “fixed”:

- Whether current URDF metadata selects exactly the intended 12 actuated
  revolute joints without added metadata. Verify first in G5; if not, stop and
  propose the minimal explicit contract.
- The final V5 schema field topology after the mandatory v4-to-V5 mapping.
- G7 runtime, memory and exact semantic content hash.
- Any intentional G4/G7 representation differences; each must be classified.
- Geometry-driven parking feasibility and selected configurations within the
  declared deterministic search domain.
- Benchmark C and D values and software/parallel/total speed-ups.
- Whether useful grid/candidate/AABB counters can be added to V5 without
  material benchmark distortion.
- Assembly-level tolerance, bushings, screws, servo horn backlash and fit
  clearances remain unmodelled.
- G3 proves mesh separation at q=0, but very small model clearances are not a
  physical assembly-clearance specification.
- LF geometry/hardware disagreement remains legitimate for `lf_hip_max`,
  `lf_lower_leg_max` and the path-limited `lf_hip_min`; do not fit it away.
- RF/RH/LH have no hardware oracle.
- The G4 legacy parking sequences remain `passed=False` under the external
  3 mm reference policy even though selected parking removes true
  intersections.

---

## [STOP CONDITION] Post-compaction continuity protocol before G5

After context compaction and before any G5 modification:

1. Re-read this Markdown file completely from disk.
2. Verify `pwd` equals the dedicated worktree path.
3. Verify branch equals `matdog/geometry-compiler-v5-collision-baseline`.
4. Verify local `HEAD` and `origin/main` remain
   `e71876e80c23c370f9fecf36ddf15f152faf5eb3` unless an explicitly authorized
   in-branch commit has since been made.
5. Verify the G4 profile file SHA256 remains
   `f1b059a58c51508345ec583cc421cf9ca66ec8e9ac5547649c05bbf755e5c5fa`.
6. Parse the G4 profile and verify content SHA256 remains
   `4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61`.
7. Verify all 17 approved mesh hashes still match this checkpoint.
8. Verify the recorded G0-G4 modified/untracked files remain present and no
   unexpected external modifications appeared.
9. Verify historical v1-v4 artifacts, Phase1B frozen source and `norma-core`
   remain untouched.
10. Only after all continuity checks pass may G5 code modification begin.

If the context cannot be compacted, this checkpoint cannot be read back fully,
its SHA cannot be verified, or any post-compaction continuity check fails:

```text
STOP BEFORE G5 AND REPORT THE FAILURE
```
