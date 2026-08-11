# MATDOG Geometry Compiler V5 — corrected workers 1/4 determinism report

**Date:** 2026-08-11<br>
**Result:** **PASS**<br>
**Reference run:** corrected C, workers=1<br>
**Deployment run:** corrected D, workers=4<br>
**Run manifest schema:** `matdog.geometry_compiler_v5.integrated_run.v2`

Supersedes `2026-08-11_072224_MATDOG_GEOMETRY_V5_DETERMINISM_REPORT.md`, whose
semantic hashes are no longer canonical.

## Acceptance rule

Workers may change elapsed time and explicitly non-semantic generation
metadata. They must not change model inputs, search parameters, result order,
geometry, path/parking content, source provenance or the separated G4 replay.

The four-process run is accepted only when all seven comparisons are exactly
true:

| Comparison | D result |
|---|---|
| Endpoint profile semantic payload | `true` |
| Parking-v2 semantic payload | `true` |
| Combined-profile semantic payload | `true` |
| G4 replay oracle semantic payload | `true` |
| Input file manifest | `true` |
| Canonical semantic source manifest | `true` |
| G4 replay source manifest | `true` |
| Execution source manifest | `true` |

The G4 replay oracle payload is compared with only `replay_execution_provenance.workers`
excluded — that single field is materialization, everything else in the replay
evidence must match exactly.

## Run identity

| Property | corrected C — workers=1 | corrected D — workers=4 |
|---|---|---|
| Prefix | `2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_C_W1` | `2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4` |
| Worker model | Same local batch function | `spawn` process pool |
| Batch strategy | 12 topology-derived joint batches, min then max | Same; dynamically scheduled |
| Canonical aggregation | `canonical_endpoint_index` | Same |
| Worker writes | 0 | 0 |
| Parent bundle publications | 1 | 1 |
| CPU affinity observed | `[0,1,2,3]` | `[0,1,2,3]` |
| MemoryMax | 6,442,450,944 | 6,442,450,944 |
| MemorySwapMax | 0 | 0 |
| cgroup tree peak | 466,382,848 B | 1,798,238,208 B |
| Swap peak / OOM / OOM-kill | 0 / 0 / 0 | 0 / 0 / 0 |
| Run-manifest file SHA256 | `b86b5d35678df4510989ea49fb5d42acaea2e4838226d7017456399dfceb0d81` | `0db86e633599f63a769dbba75db3a54c1e6470e128c007eb24c465f92a428b17` |
| GNU-time file SHA256 | `f97dfe8018c12f9605dd6bd1b9d802a6e620ed294a52cde62036af5161fea265` | `0dcdb02039f9335f459a2cc4d947c16fa5a08f221e8f25f54d691c7fc40f84a6` |

Both commands set `OMP_NUM_THREADS`, `OPENBLAS_NUM_THREADS`,
`MKL_NUM_THREADS` and `NUMEXPR_NUM_THREADS` to one before numerical imports.
Both were constrained with cgroup v2 and `/usr/bin/taskset --cpu-list 0-3`; the
runner independently rejects any observed process affinity other than CPU 0-3,
and does so regardless of whether cgroup mode is requested.

## Corrected semantic identity

| Artifact | Schema | C semantic SHA256 | D semantic SHA256 | Result |
|---|---|---|---|---|
| Endpoint profile | `matdog.calibration_geometry_profile.v5` | `de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e` | same | PASS |
| Parking | `matdog.geometry_path_parking.v2` | `67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139` | same | PASS |
| Combined profile | `matdog.calibration_geometry_profile.v5` | `0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51` | same | PASS |

D records `determinism.status: PASS` and pins the exact corrected C run-manifest
SHA256 `b86b5d35678df4510989ea49fb5d42acaea2e4838226d7017456399dfceb0d81`.

## What changed versus the superseded determinism report

Git commit SHA and working-tree dirty state are no longer inside the semantic
payload. They moved to `generation_metadata.repository_materialization`, which
is excluded from `semantic_content_sha256`. Previously
`provenance.repository.commit_sha` was hashed, which made the canonical
identity irreproducible across any commit — a rerun on byte-identical geometry
after a merge would have produced different hashes for a non-geometric reason.

Semantic provenance is now anchored purely on URDF SHA, per-link collision mesh
manifest, canonical semantic source hashes, and the model/algorithm/result
content itself.

## Bundle validity

Run manifests are self-hashing (`manifest_content_sha256`) and are published
**last**, after every data artifact, as an explicit validity marker:

```text
bundle_validity.marker:                RUN_MANIFEST_PUBLISHED_LAST
bundle_validity.manifest_required:     true
bundle_validity.artifact_count:        8
bundle_validity.artifact_hash_algorithm: SHA256
```

A directory containing data artifacts but no manifest is not a valid bundle and
is rejected. Loading a reference bundle validates the manifest schema, its
self-hash, the exact expected artifact key set, per-artifact SHA256, that every
member is in the manifest's own directory, and that no two members alias.

## Expected file-level differences

Equivalent semantic artifacts need not have identical complete file hashes. The
corrected C/D differences in endpoint, parking, combined and oracle JSON are
confined to non-semantic materialization: timestamps, worker count, phase
runtimes, repository materialization metadata, audit paths and file SHAs.

## Deterministic process architecture

```text
start method:          spawn
worker writes:         0
parent publications:   1 bundle
OMP_NUM_THREADS:       1
OPENBLAS_NUM_THREADS:  1
MKL_NUM_THREADS:       1
NUMEXPR_NUM_THREADS:   1
memory contract:       cgroup v2, MemoryMax=6 GiB, MemorySwapMax=0
CPU affinity:          process sched affinity 0-3, inherited by spawn workers
```

The 24 endpoints are partitioned into 12 deterministic topology-derived joint
batches, each containing `min` then `max`. Four-process completion order is
discarded; the parent sorts by `canonical_endpoint_index` and re-verifies each
result's identity against its expected index before aggregation.
