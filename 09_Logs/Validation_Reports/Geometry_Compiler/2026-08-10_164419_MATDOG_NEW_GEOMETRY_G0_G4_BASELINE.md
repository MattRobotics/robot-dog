# MATDOG — approved new collision geometry, G0-G4 baseline

**Date:** 2026-08-10  
**Branch:** `matdog/geometry-compiler-v5-collision-baseline`  
**Base:** `origin/main @ e71876e80c23c370f9fecf36ddf15f152faf5eb3`  
**Scope:** G0 through G4 only; Geometry Compiler V5 refactor not started.

No hardware, Station, serial, servo, EEPROM or `norma-core` access occurred.
No commit, push, PR or merge occurred.

## G0 — local/remote/filesystem truth: PASS

| Item | Observed |
|---|---|
| Hostname | `matteo-manicardi-K53SV` |
| Worktree | `/home/matteo-manicardi/MATDOG/worktrees/robot-dog-geometry-v5` |
| Branch | `matdog/geometry-compiler-v5-collision-baseline` |
| Local HEAD | `e71876e80c23c370f9fecf36ddf15f152faf5eb3` |
| `origin/main` | `e71876e80c23c370f9fecf36ddf15f152faf5eb3` |
| Remote state | default `main`; remote branch `main` only; zero open PRs |
| Operational worktree | clean before G1 |
| Staging source | exactly 17 untracked `*_collision.stl` files in the primary checkout mesh directory |
| Manifest verification | SHA256 17/17, 5,986,228/5,986,228 bytes, 119,696/119,696 triangles |
| Destination before G1 | absent and safe |
| Discrepancies | none |

`NO MODIFICATIONS PERFORMED YET` was reported before G1 began.

## G1 — visual/collision separation: PASS

The 17 approved staging files were copied byte-for-byte into
`03_CAD/URDF/matt_robodog_rev00/meshes/collision/`, renamed to canonical
link filenames and reverified 17/17 before the staging copies were removed.
The primary checkout returned to a clean state; the bytes remain in the
verified worktree destinations.

Only the 17 `<collision><geometry><mesh filename>` values changed in
`matt_robodog_rev00.urdf`, from `meshes/<link>.stl` to
`meshes/collision/<link>.stl`. All 17 `<visual>` paths remain unchanged.
Replacing `meshes/collision/` with `meshes/` in the updated URDF reproduces
the pre-change file byte-for-byte:

```text
normalized updated URDF SHA256 = 5e28da3dba10fd3f2ea6ebf6f5d6271157bda0b12b82d92aedbe3031643089ef
HEAD URDF SHA256               = 5e28da3dba10fd3f2ea6ebf6f5d6271157bda0b12b82d92aedbe3031643089ef
updated URDF SHA256            = 3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59
```

This proves collision origins, scales, joint origins, axes, limits, inertials
and all non-collision-path XML remained byte-identical.

## Approved 17-mesh manifest

| File | Triangles | Bytes | SHA256 |
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
| **Total** | **119,696** | **5,986,228** | **17/17 exact** |

Historical collision triangles: 472,020. Reduction: 74.64%.

## G2 — mesh/frame/integrity: PASS

The audit parsed the raw binary STL records independently of the production
loader, then loaded the same meshes through `RobotScene`.

| Check | Result |
|---|---|
| Binary STL size/count consistency | 17/17 |
| Exact SHA256 | 17/17 |
| Production loader | 17/17 |
| Finite vertices and stored normals | 17/17 |
| Degenerate triangles | 0 across 119,696 |
| Watertight manifold edge incidence | 17/17; every undirected edge has incidence 2 |
| Winding/stored-normal consistency | 17/17; zero non-positive face-normal dot products |
| Signed volume orientation | positive for all 17 |
| URDF collision mesh scale | 17/17 `0.001 0.001 0.001` |
| URDF collision origin | 17/17 identity |
| Units/frame comparison | collision AABBs align with detailed visual link frames |

### Local AABBs in STL millimetres

```text
base_link             min[-132.800003,-58.000000,-0.000000] max[132.800003,58.000000,79.026276]
lf_foot_link          min[-14.895466,-6.950000,0.003599]     max[14.904511,6.950000,29.803553]
lf_hip_link           min[-27.500000,-11.232409,-11.250000] max[27.500000,74.000000,33.500000]
lf_lower_leg_link     min[-16.140760,-29.000000,-47.782745] max[119.998444,26.000000,11.250000]
lf_upper_leg_link     min[-19.507025,-23.712168,-100.467369] max[14.603555,20.325642,10.467365]
lh_foot_link          min[-14.895466,-6.950000,0.003599]     max[14.904511,6.950000,29.803553]
lh_hip_link           min[-27.500000,-11.232409,-11.250000] max[27.500000,74.000000,33.500000]
lh_lower_leg_link     min[-16.140760,-29.000000,-47.782745] max[119.998444,26.000000,11.250000]
lh_upper_leg_link     min[-19.507025,-23.712168,-100.467369] max[14.603555,20.325642,10.467365]
rf_foot_link          min[-14.895466,-6.950000,0.003599]     max[14.904511,6.950000,29.803553]
rf_hip_link           min[-27.500000,-74.000000,-11.250000] max[27.500000,11.232409,33.500000]
rf_lower_leg_link     min[-16.140905,-26.000000,-47.782745] max[119.998444,29.000000,11.250000]
rf_upper_leg_link     min[-19.507025,-20.325642,-100.467369] max[14.603555,23.712168,10.467365]
rh_foot_link          min[-14.895466,-6.950000,0.003599]     max[14.904511,6.950000,29.803553]
rh_hip_link           min[-27.500000,-74.000000,-11.250000] max[27.500000,11.232409,33.500000]
rh_lower_leg_link     min[-16.140905,-26.000000,-47.782745] max[119.998444,29.000000,11.250000]
rh_upper_leg_link     min[-19.507025,-20.325642,-100.467369] max[14.603555,23.712168,10.467365]
```

Frame/symmetry findings:

- all four feet have the same surface vertex set and are centred at
  `x=0.004523 mm`, `y=0 mm` by AABB midpoint;
- upper-leg AABB midpoint Z is `-45.000002 mm` on all four links, confirming
  the baked upper-leg frame; its maximum AABB delta from the detailed visual
  mesh is 0.065281 mm;
- same-side front/hind upper, lower and foot surfaces are identical;
- left/right reflected AABBs agree, while independent tessellations have a
  maximum reflected vertex-set Hausdorff distance of 0.309156 mm; winding and
  handedness remain correct on both sides;
- the simplified foot retains the visual mesh X/Z placement and centre while
  intentionally reducing Y extent from 25.5 mm to 13.9 mm;
- the largest collision-vs-visual AABB changes are the approved simplifications:
  2.0 mm at the base top and 5.8 mm in foot Y extent, not frame rotations or
  unit errors.

## G3 — q=0 active revolute gate: PASS

Production Phase1B boolean collision settings and candidate caps were used
unchanged.

| Pair class | Result |
|---|---:|
| base ↔ hip | 4/4 `SEPARATED_NARROW` |
| hip ↔ upper | 4/4 `SEPARATED_NARROW` |
| upper ↔ lower | 4/4 `SEPARATED_NARROW` |
| **Total** | **12/12 separated** |

Runtime for this gate was 4.11 s, peak RSS 121,964 KiB, swap 0.

## G4 — unchanged Phase1B reference replay: COMPLETE

This is a diagnostic/reference replay for the approved new geometry. It is
not the V5 canonical profile and historical v4 endpoint equality was not used
as a gate.

Artifacts:

- `2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_PROFILE.json`
- `2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_REPORT.md`
- `2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_BENCHMARK_B_TIME.txt`

Integrity:

| Item | SHA256 |
|---|---|
| Profile content (timestamp excluded) | `4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61` |
| Profile file | `f1b059a58c51508345ec583cc421cf9ca66ec8e9ac5547649c05bbf755e5c5fa` |
| Report file | `140c0aa5a06e3d2855222a28cf06d8de417a391938e954607e1b91b49e4e580c` |
| Benchmark time file | `02e89f8431ebd94735964654964b1b726e9efb4f6175b917ffff8ebef689ea78` |
| URDF input | `3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59` |
| Phase1B source combined | `066079118fb2cd0f59671ae5afa566b001bdada57395690cc64a7e8c866d2ee0` |

The content hash was recomputed independently and matches. The profile's
URDF, mesh and compiler-source manifests were compared with the live files;
`find_geometry_mismatches` returned `[]`.

### Endpoint outcome

| Classification | Count |
|---|---:|
| `MESH_CONTACT_FOUND` result kind | 24/24 |
| `MODELED_ENDSTOP_CONTACT` | 20 |
| `MODEL_INCOMPLETE` | 2 |
| `PATH_COLLISION_BEFORE_ENDPOINT` | 2 |
| `NO_MODELED_ENDSTOP` | 0 |

Evidence classes: 17 `GEOMETRIC_ENDPOINT_CANDIDATE`, 3
`HARDWARE_CONFIRMED_CONTACT`, 2 `HARDWARE_CONTRADICTED`, 2 `PATH_LIMITED`.
The exact 24 endpoint angles and contact pairs are frozen in the profile and
human-readable reference report.

Six path events were recorded:

| Endpoint | Path event |
|---|---|
| `lf_hip_min` | `base_link ↔ lf_lower_leg_link @ -45.296875°` |
| `rf_hip_max` | `base_link ↔ rf_lower_leg_link @ +45.296875°` |
| `lf_lower_leg_min` | `lf_foot_link ↔ lf_upper_leg_link @ -97.968750°` |
| `lh_lower_leg_min` | `lh_foot_link ↔ lh_upper_leg_link @ -97.968750°` |
| `rf_lower_leg_min` | `rf_foot_link ↔ rf_upper_leg_link @ -97.972656°` |
| `rh_lower_leg_min` | `rh_foot_link ↔ rh_upper_leg_link @ -97.972656°` |

Legacy-policy parking output:

| Active leg | Auxiliary parking | Sequence status | Minimum reported clearance |
|---|---|---|---:|
| LF | LH upper +30°; park path PASS | `passed=False` | 0.101410 mm lower bound |
| RF | RH upper +30°; park path PASS | `passed=False` | 0.101410 mm lower bound |
| RH | not required | `passed=False` | 1.000000 mm lower bound |
| LH | not required | `passed=False` | 1.000000 mm lower bound |

The `passed=False` values are residual legacy 3 mm clearance-policy findings,
not residual true mesh intersections after the selected parking. They are
preserved without tolerance relaxation.

## Benchmark B — mesh effect

Configuration:

```text
geometry:             approved new 17-mesh collision baseline
algorithm:            unchanged validated Phase1B
workers:              1
OMP/BLAS/MKL/NumExpr: 1 thread
cgroup MemoryMax:     6,442,450,944 bytes (6 GiB)
cgroup MemorySwapMax: 0
```

| Metric | A — historical | B — new mesh effect |
|---|---:|---:|
| Collision triangles | 472,020 | 119,696 |
| Wall time | 59:27 (3,567 s) | 13:29.33 (809.33 s) |
| Peak RSS | 541 MB historical record | 462,176 KiB = 451.344 MiB |
| Swap | not reported | 0 |
| Exit/endpoints | 24/24 | 24/24 |

Measured mesh speed-up `A / B = 4.40735x`; wall-time reduction 77.31%.

The cgroup limits were verified as `memory.max=6442450944` and
`memory.swap.max=0`, then applied to the benchmark service. The user systemd
manager's post-run `Memory peak: 256.0K` summary was physically implausible and
is not used; GNU `time -v` measured the single compiler process at 462,176 KiB.
Because B is single-process, that is the relevant process-tree peak RSS.

Phase1B does not expose counters for grid entries, unique candidate pairs or
AABB-surviving triangle pairs. They are recorded as **not instrumented** for B;
the Phase1B sources were intentionally kept byte-identical rather than adding
counter overhead to this isolation run.

## Stop state

G0-G4 are complete. No G5 architectural refactor has started. No commit, push,
Draft PR or merge has been performed.
