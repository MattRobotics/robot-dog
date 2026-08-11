# MATDOG Geometry Compiler V5 — corrected A/B/C/D performance report

**Date:** 2026-08-11<br>
**Canonical runs:** corrected C (workers=1) and corrected D (workers=4),
prefix `2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_*`

Supersedes `2026-08-11_072224_MATDOG_GEOMETRY_V5_ABCD_PERFORMANCE_REPORT.md`.
A and B are retained as historical single-process baselines and were not rerun;
only C and D are canonical.

## Wall clock and memory

| Benchmark | Workers | Wall | Peak memory | Basis |
|---|---:|---:|---:|---|
| A | 1 | 3,567.00 s | 541 MB | historical |
| B | 1 | 809.33 s | 451.34375 MiB | frozen GNU RSS |
| C (corrected) | 1 | 3,360.21 s | 466,382,848 B — 444.777344 MiB | cgroup v2 tree peak |
| D (corrected) | 4 | 1,691.92 s | 1,798,238,208 B — 1,714.933594 MiB | cgroup v2 tree peak |

```text
C/D = 1.986034x
A/D = 2.108256x
```

D uses `27.91233%` of the 6 GiB envelope, with zero swap, zero OOM events and
observed process affinity `[0,1,2,3]`.

## Why corrected C/D are slower than the superseded runs

The superseded `072224` C/D executed one endpoint pass and reused it for both
the replay oracle and the canonical profile. That reuse was the audited
composition-root defect. The corrected pipeline executes **two independent
endpoint passes**:

```text
                      canonical q=0    non-canonical G4 replay    parking
corrected C (W1)        522.733140 s          531.316916 s     2276.664515 s
corrected D (W4)        248.817139 s          261.024330 s     1152.318666 s
```

Parking also now performs joint-space bisection refinement of each first
obstruction, which produces the evidence the endpoint/planner consistency gate
validates. The additional cost buys architectural separation and a fail-closed
cross-layer proof; it is not a regression in the geometry kernel.

Internal integrated wall times recorded in the run manifests:

```text
corrected C integrated_internal_wall: 3359.696038 s
corrected D integrated_internal_wall: 1691.412311 s
```

## Resource contract

Identical for both corrected runs and hard-gated by the runner:

```text
MemoryMax        6442450944 bytes (6 GiB)   cgroup v2 enforced
MemorySwapMax    0                          cgroup v2 enforced
swap peak        0
OOM / OOM-kill   0 / 0
numeric threads  OMP/OPENBLAS/MKL/NUMEXPR = 1
CPU affinity     [0,1,2,3] process sched affinity, inherited by spawn workers
worker writes    0
parent publications 1 bundle
```

`cpuset.cpus.effective` is `null` in both runs: CPU pinning is a process
scheduler mask applied with `taskset` and inherited by spawned workers, not a
cgroup cpuset. The manifests state this explicitly as
`cpu_affinity: PROCESS_SCHED_AFFINITY_INHERITED_BY_SPAWN_WORKERS` and
`cpuset_cpus_effective_role: TELEMETRY_ONLY`. Memory, swap and OOM are the
cgroup-enforced quantities.

## Not measured

- Collision candidate-pair and AABB-survivor totals remain explicitly
  `NOT_INSTRUMENTED`.
- Target Jetson Orin Nano Super performance is not measured. This host run is an
  algorithm and resource baseline, not cycle-accurate emulation.
- No candidate cap was raised and no threshold was relaxed to obtain these
  figures.
