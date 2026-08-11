# MATDOG Geometry Compiler V5 — workers 1/4 determinism report

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
> `2026-08-11_132758_MATDOG_GEOMETRY_V5_DETERMINISM_REPORT.md`.
>
> This file is retained unchanged in substance for provenance. Nothing in the
> `2026-08-11_072224` bundle was overwritten or deleted.

**Date:** 2026-08-11<br>
**Result:** **PASS**<br>
**Reference run:** C, workers=1<br>
**Deployment run:** D, workers=4<br>
**Runner SHA256:** `c9479b93491e31aac836cb2a12aa58b1e44ffd1bb87275f0c7a6f7f31dcaca85`

## Acceptance rule

Workers may change elapsed time and explicitly non-semantic generation
metadata. They must not change model inputs, search parameters, result order,
geometry, path/parking content, source provenance or the frozen G4 replay.

The four-process run is accepted only when all seven comparisons below are
exactly true:

| Comparison | D result |
|---|---|
| Endpoint profile semantic payload | `true` |
| Parking-v2 semantic payload | `true` |
| Combined-profile semantic payload | `true` |
| G4/G7 oracle payload | `true` |
| Input file manifest | `true` |
| Semantic source manifest | `true` |
| Execution source manifest | `true` |

The D run manifest records `determinism.status: PASS` and pins the exact C run
manifest SHA256
`db3f8093a8bcf37cb53a9250349a86d7ce1c7dda0aa5ea2d6bf9e18c1f319c04`.

## Run identity

| Property | C — workers=1 | D — workers=4 |
|---|---|---|
| Prefix | `2026-08-11_072224_MATDOG_GEOMETRY_V5_BENCHMARK_C_W1` | `2026-08-11_072224_MATDOG_GEOMETRY_V5_BENCHMARK_D_W4` |
| Worker model | Same local batch function | `spawn` process pool |
| Batch strategy | 12 topology-derived joint batches, min then max | Same; dynamically scheduled |
| Canonical aggregation | `canonical_endpoint_index` | Same |
| Worker writes | 0 | 0 |
| Parent bundle publications | 1 | 1 |
| CPU affinity observed | `[0,1,2,3]` | `[0,1,2,3]` |
| MemoryMax | 6 GiB | 6 GiB |
| MemorySwapMax | 0 | 0 |
| Run-manifest file SHA256 | `db3f8093a8bcf37cb53a9250349a86d7ce1c7dda0aa5ea2d6bf9e18c1f319c04` | `96ba79875f19c37952e9c981946e5a7df4136b86e1fb5b1c13ffbe12d891b57e` |
| GNU-time file SHA256 | `dd44c4025af7765ada62acf6042503214db439d3d9bef1356aec6017df325101` | `a9b8c9fcfb6294ea158d4d88404989f8566b87ae14a1ce7d45bfa1c9cd9855b0` |

Both commands set `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`,
`MKL_NUM_THREADS` and `NUMEXPR_NUM_THREADS` to one before numerical imports.
Both were constrained with cgroup v2 and `/usr/bin/taskset --cpu-list 0-3`;
the runner independently rejected any observed affinity other than CPU 0-3.

## Semantic identity

| Artifact | Schema | C semantic SHA256 | D semantic SHA256 | Result |
|---|---|---|---|---|
| Endpoint profile | `matdog.calibration_geometry_profile.v5` | `cad2f194c49d063b5a09ae4602b9acf61a701de48791e5d1aae04f1439db1211` | same | PASS |
| Parking | `matdog.geometry_path_parking.v2` | `3cda03c2c02ba5fbe6def7821ce72d4aa9e4ca66e7ca8655a2b8f0e92dde297c` | same | PASS |
| Combined profile | `matdog.calibration_geometry_profile.v5` | `e99e2b65ea8d032f94b5d1aa815432a292c1771766e556fc7115dc7a1f5de73e` | same | PASS |

Independent validation recalculated each semantic digest, rebuilt the
endpoint-to-parking-to-combined provenance chain and reconstructed the combined
profile from its endpoint and parking inputs. All checks passed.

The G4/G7 oracle JSON is byte-identical between C and D, file SHA256
`8a1b685e627b7c1dad2aedaf8cb6db87169a2ad1527b81368f8977ec4dc56311`.
It also matches the frozen G7 oracle.

## Expected file-level differences

Equivalent semantic artifacts need not have identical complete file hashes.
The C/D differences in endpoint, parking and combined JSON are confined to
`generation_metadata`:

- endpoint: six leaf values;
- parking-v2: seven leaf values;
- combined: 14 leaf values.

These values are timestamps, worker count, phase runtime, working-tree/audit
metadata and exact materialization path/file SHA. They are excluded from the
semantic hashes by schema. Endpoint, oracle and combined Markdown reports are
byte-identical. Parking reports differ only where they display the exact
materialized endpoint path and file SHA.

## Complete artifact integrity

Each prefix contains exactly ten files: eight canonical artifacts recorded in
the run manifest, one run manifest and one GNU-time record. No staging
directory or partial output remains.

| Artifact | C file SHA256 | D file SHA256 |
|---|---|---|
| Endpoint profile | `e9fa23d5c5ee98c56d622064b7ed7577ea9867f6db498909afbfc61d11156079` | `3f447177078f8fdc7571e037e09788b5404956704aadf0efe3efee7b4762e62b` |
| Endpoint report | `9f7cd25ef7f91078956dcf798b18f9f2ca7668e3d43ed30841a469040f188a98` | same |
| Oracle JSON | `8a1b685e627b7c1dad2aedaf8cb6db87169a2ad1527b81368f8977ec4dc56311` | same |
| Oracle report | `0d6084fe11aca86ca9200fc50d1fc212ea2946eba44f9209dfb9b2075aa19e38` | same |
| Parking JSON | `18fb81d05c7f4eb35d35d36880a8dd60627c1d98e982dafa2b8df52f2f519f65` | `5703b24cf05e3dc8a6b199842c77af89c9688cc9a381d1976ee6049d5a3c1eb6` |
| Parking report | `93b1b2fbcc9c25fefae7168ab9200b130f1f1541e06e197758a485e1a543fbb9` | `20ee1e0b537384c0fe9907541b10fe02666b40b9f49bbff378d66b5f775cba7d` |
| Combined profile | `06f5d1e7f46db1cf0f6084ca75f36a27d76c18e25d26c8addc53a4967e7b41d1` | `1c112a1af7af484cebc77736155118a924f327364a72e6bdb590980c70786dea` |
| Combined report | `26fe99b64fddc1b719983c4a8b1323472a902217188776b08dc3296965e3187c` | same |

All 16 manifest-recorded SHA256 values were independently recomputed from the
published bytes and passed.

## Geometric result equality

Both runs contain exactly:

```text
q=0 active pairs separated:             12/12
canonical endpoint coverage:            24/24
geometric contacts:                     24
path obstructions:                       6
path obstructions preceding contact:     2
parking NOT_NEEDED:                     18
parking FEASIBLE_1DOF_PLAN_FOUND:        6
complete geometric sequences:           24/24
executable URDF-domain targets:           8
diagnostic outside-limit targets:        16
evaluated 1-DOF candidates:              94
evaluated 2-DOF candidates:               0
collision triangles:                119,696
```

The maximum G4/G7 contact-angle delta is `6.998e-12 rad`; the maximum path
event delta is `5.749e-12 rad`, both below the predeclared `1e-4 rad`
tolerance.

## Resource result

| Metric | C — workers=1 | D — workers=4 |
|---|---:|---:|
| GNU wall | 2,716.84 s | 1,707.39 s |
| Integrated internal wall | 2,716.355623 s | 1,706.912483 s |
| Endpoint compute | 531.505577 s | 280.597768 s |
| Parking compute | 2,162.539309 s | 1,402.905607 s |
| Cgroup process-tree peak | 467,124,224 B = 445.484375 MiB | 1,795,829,760 B = 1,712.636719 MiB |
| 6 GiB quota used | 7.251% | 27.875% |
| Swap peak | 0 | 0 |
| OOM / OOM kill | 0 / 0 | 0 / 0 |
| GNU-time maximum RSS | 473,072 KiB | 457,312 KiB |

For the multi-process run, the cgroup peak is the canonical process-tree
memory measurement; GNU-time maximum RSS is not a sum across the process
tree. D remains 4,646,621,184 bytes below the 6 GiB limit and produced no swap
or OOM event. The exact remaining-byte figure is intentionally not used as an
acceptance metric; the manifest's observed peak and configured limit are the
authoritative values.

## Conclusion

Workers=1 and workers=4 produce identical semantic geometry and identical
frozen-oracle results. The four-process deployment profile satisfies the 6 GiB
process-tree, zero-swap, four-physical-core and parent-only-publication
contracts. G11 determinism is PASS.
