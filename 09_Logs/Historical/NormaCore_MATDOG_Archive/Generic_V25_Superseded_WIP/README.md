# Generic V25 full-leg engine — superseded development WIP

> # ⚠️ SUPERSEDED DEVELOPMENT WIP
> # STATION-MEDIATED IMPLEMENTATION
> # NOT CURRENT RUNTIME
> # NOT FINAL GENERIC CALIBRATOR
> # DO NOT USE AS HARDWARE AUTHORIZATION

Archive of the unfinished attempt to generalize LF V25 into a data-driven four-leg calibration
engine, from `MattRobotics/norma-core` before its branch was retired.

---

## Provenance

Machine-readable: [`PROVENANCE.json`](PROVENANCE.json)

| Field | Value |
|---|---|
| Source repository | `MattRobotics/norma-core` |
| Original branch | `matdog/generic-v25-full-leg-engine` |
| Head commit | `f4a87a443011f15e4f105858fec62fc15ca5a77c` |
| Head tree | `e011b9410c1bd0c501884085b5d3c0824eef463d` |
| Merge-base with `main` | `4a8ed6337261553b79c928975808d294c9ca723b` (= `main` head) |
| Unique commits | **16** |
| `main` ahead by | 0 — branched directly off current `main` |
| Files changed | 17 (2 Rust + 15 markdown) |
| **Hardware validated** | **NO — offline only** |
| Archived | 2026-08-27 |

Head SHA verified directly against the live remote at archive time.

## Contents

```text
source/                              17 files at branch head, byte-for-byte
generic_v25_vs_main.patch            full diff vs main (13 681 insertions, 244 deletions)
unique_commits.txt                   all 16 unique commits
file_inventory.txt                   per-file diffstat
PROVENANCE.json                      machine-readable provenance
SHA256SUMS                           integrity manifest
../bundles/generic_v25_unique.bundle git bundle carrying the 16 commits
```

## How it evolved from LF V25

LF V25 was a single hard-coded left-front state machine. The generic engine aimed to split it into:

```text
LegSessionStateMachine  +  LegCalibrationSpec  →  one engine, four leg profiles (LF/RF/RH/LH)
```

explicitly **not** four copied per-leg state machines. Every constant, helper and state transition
was to be classified as: **A** generic behaviour · **B** geometry/profile/spec data · **C** global
ST3215 hardware/safety parameter · **D** historical LF-only evidence.

## What was completed

| Gate | Status |
|---|---|
| G0 / G1 audit | complete — frozen audit and spec contract |
| G2 generic spec contract | complete, revised through **four** independent review rounds |
| G2 GoalPosition call-graph closure | complete |
| G2 architecture-gate consistency | complete |
| G3A structural foundation | complete |
| G3B goal authority extraction | complete |
| G3B LF-context and session-context requirements | complete |
| G3C rejected-profile cleanup | complete |

Twelve of the 17 files are G1/G2/G3 review and decision documents — the branch is unusually
well-documented relative to how much executable code it produced.

## What was NOT completed

- No end-to-end generic calibration run, offline or on hardware.
- RF, RH and LH leg profiles were never finished or validated.
- **Zero hardware execution.** Nothing here was ever run against a servo.
- Phase 2B path/parking safety gate never integrated — it remains `passed=False` on all four legs.
- Phase 2C offline validation never completed.

## ⚠️ Do not promote these values

No constant, span, offset or profile value from this branch may be promoted into the new
calibration. It never produced a validated measurement. Its q0 handling assumes the pre-reassembly
installation and the Station-mediated architecture, both of which are gone.

## What may still be useful

As an **engineering reference for algorithms and structure only**:

- the spec/state-machine split concept (`LegSessionStateMachine` + `LegCalibrationSpec`);
- the A/B/C/D classification discipline for separating generic behaviour from leg data;
- goal-authority extraction — a single owner for `GoalPosition` writes;
- session-context and LF-context preconditions before any write;
- the rejected-profile cleanup semantics;
- the G2 review documents, which record why several designs were rejected.

Those ideas transfer to the ESP32-S3 architecture. The **implementation** does not — it is built
on Station ownership.

## Related

- [NormaCore MATDOG archive index](../README.md)
- [LF V25 historical hardware oracle](../LF_V25_Hardware_Oracle/README.md)
- [Canonical architecture](../../../../01_Docs/02_Architecture/ARCHITECTURE.md)
