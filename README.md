# MATDOG — Custom Quadruped Robot

MATDOG is a custom quadruped robot developed by Matt Robotics in Italy. The project covers the complete robot stack: mechanical design, 3D-printed structure, power distribution, twelve Feetech ST3215 serial-bus servos, CAD/URDF-derived kinematics, mechanical calibration, locomotion, embedded control and future perception/autonomy.

This repository is the public engineering source of truth for the MATDOG platform. The current development strategy is **geometry-first, evidence-driven and explicitly split between offline model analysis and hardware calibration**.

## Current project state — 8 August 2026

The calibration architecture was reorganized after LF V25 into three explicit phases:

```text
Phase 1 — OFFLINE GEOMETRY COMPILER          COMPLETE + MERGED
Phase 2 — GENERIC V25-DERIVED HARDWARE ENGINE NEXT, NOT STARTED
Phase 3 — HARDWARE CALIBRATION RF/RH/LH       NOT STARTED
```

Canonical `robot-dog/main` after Phase 1:

```text
66f613034f3a64debc9bc45031eb4283a375e52f
```

Phase 1 was merged through:

```text
PR #14 — feat(calib): complete offline geometry compiler phase 1
```

Final Phase 1 status:

```text
PASS_GEOMETRY_COMPILER_COMPLETE_WITH_EXPLICIT_MODEL_GAPS
```

### Current validated-state matrix

| Area | Status |
|---|---|
| Mechanical architecture and REV00 CAD/URDF | Validated baseline |
| Twelve-servo bus, sparse IDs, mapping and directions | Validated |
| Digital-home commissioning and EEPROM readback | Validated for all 12 servos |
| Encoder-to-radian conversion and live read-only FK | Validated for all four legs |
| LF mechanical end-stop calibration | **V25 hardware validated and frozen** |
| Geometry Compiler Phase 1 | **Complete: 24/24 endpoint analyses, schema v3** |
| LF model-vs-hardware reconciliation | **Complete: 6/6 MODEL_INCOMPLETE for endpoint metrology** |
| RF/RH/LH hardware end-stop calibration | Not yet hardware validated |
| Generic V25-derived full-leg calibration engine | Phase 2 — next task |
| Complete 12-joint persistent profile | Not yet complete |
| Stand-up and locomotion | Pending post-calibration regeneration and validation |
| Embedded/autonomy integration | Planned |

> **Important:** LF V25 is still the only full-leg mechanical calibration validated on real hardware. Geometry Compiler results for RF/RH/LH are offline model findings, not hardware calibration results.

---

## Repository responsibilities

```text
MattRobotics/robot-dog
→ MATDOG source of truth for CAD, URDF, collision meshes, electronics,
  calibration evidence, Geometry Compiler, kinematics, locomotion,
  validation reports and project decisions

MattRobotics/norma-core
→ Station/ST3215 runtime and the native MATDOG hardware-calibration engine
```

Private research material is intentionally excluded from this public repository and is not part of the MATDOG runtime or public technical baseline.

---

# Phase 1 — Geometry Compiler

Phase 1 built an offline Geometry Compiler that reuses the canonical URDF, collision STL meshes and FK/collision infrastructure to analyze:

```text
4 legs × 3 joints × 2 sides = 24 endpoint cases
```

The compiler separates three concepts that must not be conflated:

1. the **URDF-declared joint limit**;
2. the **first relevant collision/contact represented by the collision meshes**;
3. the **real hardware mechanical contact**, where hardware evidence exists.

These are stored and compared independently.

Canonical Phase 1 completion record:

```text
06_Software/Matdog_Core/calibration/
MATDOG_GEOMETRY_COMPILER_PHASE1_COMPLETION_2026-08-07.md
```

Final machine-readable profile:

```text
09_Logs/Validation_Reports/Geometry_Compiler/
2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_PROFILE.json
```

Final human-readable report:

```text
09_Logs/Validation_Reports/Geometry_Compiler/
2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_REPORT.md
```

### Final Phase 1 endpoint classification

| Status | Count | Meaning |
|---|---:|---|
| `MODEL_INCOMPLETE` | 6 | LF hardware proves a real endpoint exists, but current collision mesh does not represent the same physical event |
| `MODEL_LIMIT_MISMATCH` | 4 | Mesh contact exists but is materially displaced from the URDF-declared limit; no hardware oracle yet for that leg |
| `NO_MODELED_ENDSTOP` | 14 | No same-leg mesh contact was found inside the configured bounded analysis envelope |
| `PATH_COLLISION_BEFORE_ENDPOINT` | 0 | No final endpoint remained blocked by an earlier cross-leg path collision after parking planning |
| `UNINTENDED_SELF_COLLISION` | 0 | No final endpoint was classified as an incidental same-leg collision near home |

### Final validation

```text
81/81 Geometry Compiler tests PASS
0 failures
0 errors
py_compile PASS
pyflakes PASS
24/24 endpoint compiler run PASS
compiler peak RSS ≈ 478 MB
full test-suite peak RSS ≈ 665 MB
```

No Station, serial, servo command, torque command or EEPROM write is performed by the Geometry Compiler.

---

# Important Geometry Compiler search-envelope semantics

The Geometry Compiler does **not** clamp FK evaluation to the URDF joint limits.

`RobotScene` evaluates geometry with:

```text
enforce_limits=False
```

and endpoint search deliberately continues beyond the URDF-declared limit.

Current Phase 1 setting:

```text
DEFAULT_ENVELOPE_MARGIN_RAD = 10 deg
```

For each endpoint the search sweeps from q=0 toward:

```text
MAX side: urdf_declared_max + 10 deg
MIN side: urdf_declared_min - 10 deg
```

Therefore Phase 1 did **not** simply stop at the old URDF limits.

Concrete examples prove that the search really crossed those limits:

```text
LF hip MIN:
URDF declared = -45.000 deg
mesh contact  = -47.500 deg

LF lower MIN:
URDF declared = -92.000 deg
mesh contact  = -97.957 deg

RF hip MAX:
URDF declared = +45.000 deg
mesh contact  = +47.500 deg

RF lower MIN:
URDF declared = -92.000 deg
mesh contact  = -98.004 deg
```

However, **NO_MODELED_ENDSTOP is a bounded statement**. It means:

> no relevant same-leg mesh contact was found between q=0 and 10 degrees beyond the declared URDF limit in the searched direction.

It does **not** prove that the STL geometry would never collide at a much larger angle.

Examples of the actual no-contact envelopes include:

```text
hip MAX:       searched through +55.0 deg for a +45.0 deg URDF limit
hip MIN:       searched through -55.0 deg for a -45.0 deg URDF limit
upper MAX:     searched through +132.5 deg for a +122.5 deg URDF limit
upper MIN:     searched through -62.5 deg for a -52.5 deg URDF limit
lower MAX:     searched through +47.5 deg for a +37.5 deg URDF limit
```

This distinction is now a permanent interpretation rule.

Canonical clarification:

```text
06_Software/Matdog_Core/calibration/
MATDOG_GEOMETRY_COMPILER_ENVELOPE_CLARIFICATION_2026-08-08.md
```

### Why LF remains `MODEL_INCOMPLETE`

The bounded-envelope caveat does **not** invalidate the LF result.

LF has direct V25 hardware evidence for all six real contacts. For example:

```text
LF hip MIN hardware contact   ≈ -42.803 deg
LF hip MIN mesh collision     ≈ -47.500 deg

LF lower MIN hardware contact ≈ -91.846 deg
LF lower MIN mesh collision   ≈ -97.957 deg
```

For the other four LF endpoints no same-leg mesh collision is present at the real hardware contact angle or anywhere through the 10-degree-beyond-URDF envelope.

A hypothetical collision much farther away would still not represent the physical event at which LF hardware actually stops. Therefore LF 6/6 `MODEL_INCOMPLETE` remains the correct endpoint-metrology conclusion.

For RF/RH/LH, where there is no hardware oracle yet, `NO_MODELED_ENDSTOP` must remain explicitly bounded to the searched envelope.

### Pre-Phase-2 geometry sanity check

Before Phase 2 hardware-engine implementation is treated as final, the project should perform a targeted **extended mesh-contact audit** for the endpoints currently carrying `NO_MODELED_ENDSTOP`.

Purpose:

- characterize whether a collision eventually exists farther beyond the original URDF limits;
- make that search range independent from the original hand-entered URDF endpoint values;
- distinguish "no contact inside Phase 1 envelope" from "no relevant mesh contact over a wider physically meaningful angular domain";
- avoid accidentally using the old URDF limits as hidden anchors in future geometry reasoning.

This is an offline model-characterization task only. It does not authorize hardware motion and does not invalidate the already-completed Phase 1 deliverable.

---

# LF V25 — hardware oracle and frozen reference

On 4 August 2026 the complete left-front leg calibration completed successfully.

```text
58/58 sequence complete
6/6 LF mechanical contacts accepted
supervised hardware-witness gate PASS
RAM q0 staging PASS
Station shutdown and serial release PASS
transactional EEPROM freeze PASS
persistent LF profile PASS
global torque OFF verified
```

Canonical record:

```text
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
```

Exact validated NormaCore source:

```text
branch: release/matdog-lf-calibrator-v25
source head: f87dd1fbc7e8100d275c74f9af448642f3429680
implementation PR: MattRobotics/norma-core#11
```

### LF final hardware contacts

| Joint | Motor | MIN contact | MAX contact | q0 candidate before EEPROM | Final displayed q0 |
|---|---:|---:|---:|---:|---:|
| LF hip | M13 | 2535 | 1600 | 2067 | 2048 |
| LF upper | M12 | 1439 | 3443 | 2040 | 2051 |
| LF lower | M11 | 3093 | 1658 | 2074 | 2046 |

Frozen ST3215 Position Offsets:

| Motor | Previous | Frozen |
|---|---:|---:|
| M11 | 101 | 127 |
| M12 | 859 | 851 |
| M13 | -505 | -486 |

LF V25 is a frozen hardware reference and must not be rerun unless LF mechanics, servo, mounting, URDF or calibration state changes.

### Important interpretation of the historical affine gate

LF V25 successfully produced and verified the operational LF calibration state. That hardware evidence remains valid.

For future generalization, however, the project must **not** use an affine scale fitted so that measured MIN/MAX contacts map by construction onto old URDF MIN/MAX values as proof that the old URDF limits were physically exact.

The generalized architecture keeps separate:

```text
encoder/transmission scale
q0 offset
urdf_declared_limit
mesh-predicted collision/contact
hardware-measured contact
geometry/model mismatch
```

Manual/photo home is only a commissioning seed and plausibility check, not the final mathematical definition of q=0.

---

# Parking and path planning

Phase 1 confirmed that parking is an output of collision/path planning, not a permanent convention.

Default policy:

```text
NO AUXILIARY PARKING unless geometry proves it is needed
```

Current Phase 1 result:

| Active leg | Auxiliary parking |
|---|---|
| LF | park LH upper at +30° |
| RF | park RH upper at +30° |
| RH | none required in analyzed sequence |
| LH | none required in analyzed sequence |

The +30° LF/LH solution agrees with the 2026-07-20 geometry checkpoint and with the real LF V25 hardware session.

Historical +30°/+50°/+90° values remain search seeds/reference evidence, not universal constants.

---

# Robot definition

Canonical REV00 package:

```text
03_CAD/URDF/matt_robodog_rev00/
```

The package contains the canonical URDF, visual/collision meshes, workbook, URDF Studio project and integrity manifest.

Coordinate convention:

```text
X = forward
Y = left
Z = up
units = metres and radians
right-handed frame
```

### Servo mapping

```text
LF: M13 hip, M12 upper, M11 lower
RF: M23 hip, M22 upper, M21 lower
RH: M33 hip, M32 upper, M31 lower
LH: M43 hip, M42 upper, M41 lower
```

Direction mapping:

```text
LF: hip -1, upper +1, lower -1
RF: hip -1, upper -1, lower +1
RH: hip +1, upper -1, lower +1
LH: hip +1, upper +1, lower -1
```

Canonical leg order and trot diagonals:

```text
[LF, RF, RH, LH]
LF + RH
RF + LH
```

### Front / hind geometry is not interchangeable

Front and hind hip geometry differ in Z by about 20 mm:

```text
front hip Z ≈ 0.0465 m
hind hip Z  ≈ 0.0265 m
```

LF/RF and RH/LH may support mirror comparisons where demonstrated, but FRONT and HIND must not be treated as geometrically interchangeable by convention.

---

# Permanent calibration and control rules

- Station is the sole ST3215 serial owner during motion.
- ST3215 `GoalPosition` remains unsigned standard `0..4095`; signed-wrap is forbidden.
- Digital-home commissioning remains separate from mechanical endpoint calibration.
- `urdf_declared_limit`, mesh contact and hardware contact are separate quantities.
- A cross-leg path collision must never be relabeled as the probed joint's mechanical endpoint.
- `NO_MODELED_ENDSTOP` means no contact in the documented analysis envelope, not proof of no contact at any possible angle.
- LF V25 remains the immutable full-leg hardware oracle.
- Manual/photo home is a seed, not final q=0 metrology.
- EEPROM writes are allowed only after complete measurement PASS, verified Station shutdown/serial release and explicit authorization.
- EEPROM provisioning must be transactional with backup, readback, relock and rollback.
- RF/RH/LH must be generalized from the V25 architecture through a generic data-driven engine, not by reviving old version-numbered programs.
- No hardware movement is authorized by offline Geometry Compiler results alone.

---

# Development roadmap

## Phase 1 — Offline Geometry Compiler

- [x] Canonical geometry-first architecture frozen
- [x] Memory-bounded collision kernel
- [x] 24-endpoint analysis
- [x] Separate endstop-contact and path-collision policies
- [x] Segment-scoped parking planner
- [x] EXACT / LOWER_BOUND / UNRESOLVED clearance semantics
- [x] Pair-pinned sensitivity / uncertainty model
- [x] LF V25 6/6 hardware reconciliation
- [x] Deterministic schema-v3 profile/report + hashes
- [x] 81/81 final tests PASS
- [x] Merge PR #14 into `main`
- [x] Repository cleanup and local/remote alignment
- [ ] Extended no-contact mesh sweep independent of the old URDF-limit ±10° envelope — **pre-Phase-2 sanity check**

## Phase 2 — Generic V25-derived calibration engine in NormaCore

Next engineering phase, after the extended offline geometry sanity check.

Goals:

- [ ] Audit live `norma-core/main`, LF V25 release and historical RF worktree
- [ ] Compare LF V25 validated behavior with the historical RF prototype
- [ ] Define generic `LegCalibrationSpec` / full-leg state-machine architecture
- [ ] Consume the Geometry Compiler profile for path/parking/staleness/model-gap information
- [ ] Preserve `ContactConfirmed -> STOP ADVANCING IMMEDIATELY`
- [ ] Preserve `coarse scout → backoff → fine1 → backoff → fine2 → repeatability`
- [ ] Guarantee torque-OFF and serial-release behavior on all failure/exit paths
- [ ] Complete offline tests and human review
- [ ] No hardware movement during Phase 2 offline development without separate authorization

Historical RF worktree retained for comparison only:

```text
/home/matteo-manicardi/MATDOG/worktrees/norma-core-rf-calibrator
branch: matdog/rf-calibrator-from-lf-v25
```

It is evidence/prototype history, not an automatically accepted Phase 2 base.

## Phase 3 — Hardware calibration

Planned order:

```text
RF → RH → LH
```

- [ ] RF full-leg hardware calibration
- [ ] Model-vs-real reconciliation
- [ ] RF q0 / persistent profile
- [ ] RH full-leg hardware calibration
- [ ] RH q0 / persistent profile
- [ ] LH full-leg hardware calibration
- [ ] LH q0 / persistent profile
- [ ] Complete 12-joint profile

## Post-calibration locomotion

- [ ] Recompute four-leg FK with complete frozen profiles
- [ ] Regenerate HOME → LOW_STAND → NOMINAL_STAND
- [ ] Repeat collision/contact/support audit with calibrated limits
- [ ] Supervised suspended stand
- [ ] Gradual load transfer and nominal stand
- [ ] Single-foot swing trajectory
- [ ] Trot in place
- [ ] First slow walking tests

## Embedded integration and autonomy

- [ ] Battery and smart-BMS integration
- [ ] Embedded motion-controller evaluation
- [ ] Jetson integration
- [ ] IMU, estimator and watchdog integration
- [ ] Depth vision, object detection, voice and autonomous behaviour

---

# Canonical references

Start here when reviewing the current calibration architecture:

```text
06_Software/Matdog_Core/calibration/MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md
06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_PHASE1_COMPLETION_2026-08-07.md
06_Software/Matdog_Core/calibration/MATDOG_GEOMETRY_COMPILER_ENVELOPE_CLARIFICATION_2026-08-08.md
06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md
06_Software/Matdog_Core/calibration/MATDOG_MECHANICAL_ENDSTOP_GEOMETRY_CHECKPOINT_2026-07-20.md
09_Logs/Validation_Reports/Geometry_Compiler/README.md
REPOSITORY_VERIFICATION_INDEX.md
```

---

# Repository structure

```text
01_Docs/        architecture and stable technical references
02_BOM/         components, suppliers and costs
03_CAD/         CAD, URDF, meshes and mechanical exports
04_Electronics/ wiring, power and servo mapping
05_Firmware/    future low-level firmware
06_Software/    calibration, kinematics, gait and control
07_Media/       images, renders and videos
08_Tests/       repeatable validation procedures
09_Logs/        decisions, reports and historical evidence
```

# Development policy

- `main` is the canonical public state of `robot-dog`.
- New work starts from verified current `main`.
- At most one clearly named active branch should exist for a focused milestone unless explicit parallel work is justified.
- No force-push or destructive cleanup of evidence.
- Historical reports remain preserved and are marked superseded rather than silently deleted.
- No merge into `main` without explicit human authorization.
- GitHub remote and the real ASUS filesystem override stale chat summaries or handoff assumptions.

---

Built and documented by Matt Robotics.
