# MATDOG Geometry Compiler V5 — A/B/C/D performance report

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
> `2026-08-11_132758_MATDOG_GEOMETRY_V5_ABCD_PERFORMANCE_REPORT.md`.
>
> This file is retained unchanged in substance for provenance. Nothing in the
> `2026-08-11_072224` bundle was overwritten or deleted.

**Date:** 2026-08-11<br>
**Result:** benchmark matrix complete<br>
**Canonical deployment profile:** D, workers=4

## Matrix

| Benchmark | Geometry / software scope | Workers | Wall time | Canonical peak memory | Swap | Endpoints |
|---|---|---:|---:|---:|---:|---:|
| A — historical | Old geometry + validated Phase1B compiler | 1 | 3,567.00 s = 59:27.00 | 541 MB, historical record | not recorded | 24 |
| B — mesh effect, frozen | Approved new geometry + unchanged Phase1B compiler | 1 | 809.33 s = 13:29.33 | 462,176 KiB = 451.34375 MiB | 0 | 24/24 |
| C — software effect | Approved new geometry + final integrated V5 | 1 | 2,716.84 s = 45:16.84 | 467,124,224 B = 445.484375 MiB cgroup process-tree | 0 | 24/24 |
| D — deployment | Same final integrated V5 | 4 processes | 1,707.39 s = 28:27.39 | 1,795,829,760 B = 1,712.636719 MiB cgroup process-tree | 0 | 24/24 |

A and B are frozen records. They were not rerun or materially instrumented.
C and D execute the same final integrated command, parameters and semantic
sources; only worker count and D's determinism reference differ.

## Required ratios

```text
mesh effect:       A / B = 4.407349x
software ratio:    B / C = 0.297894x
parallel speed-up: C / D = 1.591224x
total speed-up:    A / D = 2.089154x
```

The software ratio is not a speed-up: C takes `3.356900x` the wall time of B,
or `+235.69%`. This must not be hidden or relabelled. It also is not a
like-for-like kernel regression measurement: B is a Phase1B/schema-v4 endpoint
compiler with four legacy parking records, whereas C is the final V5
integrated endpoint plus topology-driven parking pipeline for all 24
endpoints, with repeated path validation, provenance gates and artifact
materialization.

Process parallelism reduces C-to-D wall time by `37.1553%`. The final D run is
`52.1337%` faster than historical A end to end.

## Final V5 workload

| Metric | C | D |
|---|---:|---:|
| Collision triangles | 119,696 | 119,696 |
| Grid entries serialized | 94 | 94 |
| Evaluated 1-DOF candidates | 94 | 94 |
| Evaluated 2-DOF candidates | 0 | 0 |
| Collision candidate pairs | `NOT_INSTRUMENTED` | `NOT_INSTRUMENTED` |
| AABB-surviving pairs | `NOT_INSTRUMENTED` | `NOT_INSTRUMENTED` |
| Endpoint contacts | 24 | 24 |
| Path obstruction events | 6 | 6 |
| Parking results | 18 direct + 6 feasible 1-DOF | same |

Phase1B did not expose grid-entry, unique-candidate or AABB-survivor counters
for B. The frozen algorithm was deliberately not changed or rerun merely to
obtain them. Candidate and AABB totals remain explicitly uninstrumented rather
than estimated.

## Phase timings

| Metric | C — workers=1 | D — workers=4 | C/D |
|---|---:|---:|---:|
| Endpoint compute | 531.505577 s | 280.597768 s | 1.894190x |
| Parking compute | 2,162.539309 s | 1,402.905607 s | 1.541472x |
| Integrated internal wall | 2,716.355623 s | 1,706.912483 s | 1.591385x |
| External GNU wall | 2,716.84 s | 1,707.39 s | 1.591224x |

The external GNU wall is the canonical value for the A/B/C/D ratios.

## Memory and resource envelope

Memory values are normalized before comparison:

```text
B: 451.343750 MiB  (GNU-time RSS, accepted frozen measurement)
C: 445.484375 MiB  (cgroup v2 process-tree peak)
D: 1712.636719 MiB (cgroup v2 process-tree peak)
```

C is `5.859375 MiB` below B, but the instrumentation differs: B is the
accepted single-process GNU-time maximum RSS and C is cgroup process-tree
peak. They are close enough to report, not to over-interpret.

D uses `3.844437x` C's process-tree peak and `27.87495%` of the configured
6 GiB limit. It remained `4,646,621,184 B` below MemoryMax. Both C and D
recorded zero swap peak and zero OOM/OOM-kill events.

For D, GNU-time's `457,312 KiB` maximum RSS describes only a single process
maximum and is not the canonical multi-process peak. The cgroup v2 value
`1,795,829,760 B` covers the process tree and is authoritative.

## Reproducibility and integrity

The complete command line and GNU-time counters are retained in:

- `2026-08-11_072224_MATDOG_GEOMETRY_V5_BENCHMARK_C_W1_TIME.txt`;
- `2026-08-11_072224_MATDOG_GEOMETRY_V5_BENCHMARK_D_W4_TIME.txt`.

The run manifests are:

- C file SHA256:
  `db3f8093a8bcf37cb53a9250349a86d7ce1c7dda0aa5ea2d6bf9e18c1f319c04`;
- D file SHA256:
  `96ba79875f19c37952e9c981946e5a7df4136b86e1fb5b1c13ffbe12d891b57e`.

Both runs were pinned to CPU 0-3, a set representing four distinct physical
cores on this host, with all numerical-library thread counts set to one. Both
ran with `MemoryMax=6442450944`, `MemorySwapMax=0`; D additionally hard-gated
workers=4 and exact C semantic equality.

## Conclusion

The approved mesh replacement accounts for the frozen A-to-B improvement.
The broader final V5 software scope costs more than B in single-worker mode.
Deterministic four-process execution recovers a 1.591224x C-to-D improvement
while staying far below the 6 GiB envelope and preserving exact semantic
geometry. D is the accepted deployment benchmark.
