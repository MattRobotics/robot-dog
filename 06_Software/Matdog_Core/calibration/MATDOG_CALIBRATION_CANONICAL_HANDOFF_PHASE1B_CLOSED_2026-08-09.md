# MATDOG — canonical handoff: Phase 1B CLOSED, Phase 2 NEXT

**Date:** 2026-08-09
**State:** Phase 1B **CLOSED** and merged. Phase 2 is **NEXT** and **NOT STARTED**.

This document is the operational entry point for Phase 2. It supersedes
`MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md` where the two conflict on endpoint
metrology; that earlier handoff remains valid for everything it says about the three-phase
plan structure and the q=0 policy.

---

## 1. Repository state at closure

### robot-dog

```text
final main                5b66044e225fcd921e44b98cc710f028da441a64
PR #16                    MERGED (squash) 2026-08-09
                          feat(calib): complete adjacent endstop metrology Phase 1B
                          head was 2d39180307ecbcf273517751f7d58da29f47dc48 (24 files)
PR #15                    CLOSED, NEVER MERGED (superseded)
                          head 751fe1eff44a2d97714438a040f04f4a8050ea04
remote branches           main only
local branches            main only
```

Deleted after proof of preservation (identical trees, no worktree, no uncommitted state):
`matdog/geometry-adjacent-endstop-phase1b`, `matdog/geometry-compiler-phase1`,
`docs/phase1-readme-geometry-clarification-2026-08-08`. No logs, artifacts or closed PRs
were deleted.

### norma-core

```text
main                       f47b1ba579c623139058a8b0118648015739ab10
release/matdog-lf-calibrator-v25   f87dd1fbc7e8100d275c74f9af448642f3429680
```

Both branches are intentional. The release branch is the **immutable LF V25 source**; its
divergence from `main` is expected and is not repository dirt. It must not be merged, rebased
or fast-forwarded to resemble `main`.

### Historical RF worktree — preserved evidence, NOT the Phase 2 base

```text
path      /home/matteo-manicardi/MATDOG/worktrees/norma-core-rf-calibrator
branch    matdog/rf-calibrator-from-lf-v25  @ b2f7dac2eab7147917fccdfde702360da82ab7de
state     intentionally dirty (2 modified files, dated 2026-08-07)
          software/drivers/st3215/src/auto_calibrate/matdog.rs
          software/drivers/st3215/src/auto_calibrate/matdog_test.rs
```

These modifications are **historical evidence**. Do not clean, reset, stash, commit, checkout
or delete them. **Phase 2 does not start from this worktree** — it starts from merged
`norma-core` `main`.

---

## 2. Phase 1B validated result

| item | result |
|---|---|
| GATE A | **PASS** |
| GATE B | **SUPPORTED** |
| adjacency | **12 REVOLUTE** included, **4 FIXED** excluded (URDF-derived) |
| endstop metrology | active revolute parent-child pair, one per joint |
| path safety | every other relevant pair; active pair excluded from its own sweep |
| clearance gate | generic 3 mm gate applies to **NON_ADJACENT only** |
| collision paths | boolean intersection and near-miss clearance are separate paths/margins |
| compiler | **24/24**, exit 0, 59:27, peak RSS 541 MB |
| final full suite | **122/122**, exit 0, 43:21, peak RSS 842 MB |
| classification | **21** `MODELED_ENDSTOP_CONTACT` / **3** `MODEL_INCOMPLETE` / **0** `NO_MODELED_ENDSTOP` |

Targeted suites: `phase1b_policy` 39/39, `contact_search` 15/15, `scene` 18/18,
`mesh_kernel` 21/21, `uncertainty` 4/4.

### Canonical geometry hashes

```text
base_link.stl          644a83e98fd116f3fc8e5d8792ca2b60b0bdb09bcafa4fc49140d641079e4b6b
lf_upper_leg_link.stl  3c484b110a622274d1f8446b30329b8e16a0ff542d25da2a39334d41e8f4f169
lh_upper_leg_link.stl  3c484b110a622274d1f8446b30329b8e16a0ff542d25da2a39334d41e8f4f169
rf_upper_leg_link.stl  08fab5e3229280f21a52c6cfbfdab32f1e8345aae6e10648360ae30fe1e06b16
rh_upper_leg_link.stl  08fab5e3229280f21a52c6cfbfdab32f1e8345aae6e10648360ae30fe1e06b16

URDF                   5e28da3dba10fd3f2ea6ebf6f5d6271157bda0b12b82d92aedbe3031643089ef
                       byte-identical to pre-Phase-1B; rev00 unchanged
```

### Schema v4 artifacts — current canonical

```text
09_Logs/Validation_Reports/Geometry_Compiler/2026-08-08_231600_...
  PROFILE.json             file c2e980bfc49e80b957fc0a326b1ac98724fc1d4ac16121da19fcf6ab2be3eff6
                           content_sha256 928ff09616a1ad80df029a99afe2061e41a0bedec6bfdc9b3f80f74af4867828
  REPORT.md                file 7774af1c41df2859fed89bfb468d4e04be95352158fa7a199493bf823775a858
  COLLISION_MESH_HASH_MANIFEST.json  17904a647d6dd357586765829d24dbd0b09c3b1fb976a2084eb9b1f1537aac03
```

The manifest `.md` companion is regenerated alongside. v1/v2/v3 artifacts are **retained**;
v3 is superseded for endpoint metrology, not deleted.

### Historical path regressions — preserved

```text
base_link       <-> lf_upper_leg_link   -47.5000 deg
lf_upper_leg_link <-> lf_foot_link      -97.9570 deg
```

Both reproduce exactly and are now classified as **PATH** events, not endpoints; in each case
the joint's own articulation contact occurs at a smaller |q|.

---

## 3. Open engineering items — visible, not blocking Phase 1B closure

### 3.1 Three LF hardware contradictions

```text
lf_hip_min        mesh - hardware = -3.2090 deg
lf_hip_max                          +5.8555 deg
lf_lower_leg_max                    +3.9023 deg
```

State: **`HARDWARE_CONTRADICTED`, physical cause UNKNOWN.**

These must **not** be called *missing endpoint*, *agreement*, *fixed* or *resolved*. Phase 1B
**did** find geometric contacts for all three; the model simply stops later than the hardware
did, and why is unidentified. They are the same three endpoints that showed no far-field contact
in GATE A.

They do **not** reopen Phase 1B, and they must **not** silently block offline Phase 2
construction. They **do** constrain how geometry predictions may be interpreted before hardware.

### 3.2 Other open items

- **RF/RH/LH have no hardware oracle.** Their endpoints are `GEOMETRIC_ENDPOINT_CANDIDATE` —
  predicted search targets and bounds, never measured hardstops.
- **Adjacent hardstop sensitivity: `NOT_APPLICABLE`** with the current global
  clearance-gradient method (a revolute pair's minimum clearance is the angle-invariant
  motor-pin fit, not the contact feature). A hardstop-surface-local method is required.
- **Assembly tolerance unmodelled** (bushings, screws, servo backlash).
- **Parking: `REVALIDATED / UNCHANGED FROM v3`, `passed=False` 4/4.** Not a PASS, and not
  introduced by Phase 1B — bit-identical to v3, already failing there on non-adjacent
  clearance. Acceptable for Phase 1B closure; **not** acceptable to ignore before Phase 2
  hardware execution (see §4, Phase 2B).

### 3.3 Superseded planning — do not carry forward

The PR #15 recommendation *Phase 1 → extended `NO_MODELED_ENDSTOP` sweep → Phase 2* is
**superseded**. Schema v4 gives 24/24 active-pair endpoint localization with 0
`NO_MODELED_ENDSTOP`, so that sweep is no longer the pre-Phase-2 blocker. The v3
bounded-envelope clarification remains valid historically for v3 and is preserved in closed
PR #15.

---

## 4. Phase 2 — entry plan (NOT STARTED)

### Phase 2A — offline engine generalization

1. Start from current merged `MattRobotics/norma-core` **`main`**.
2. **Do not** continue from the historical RF worktree as the development base.
3. Generalize the proven LF V25 architecture through **data-driven leg profiles**: LF, RF, RH, LH.
4. Preserve the permanent contracts:
   - Station is the **sole serial owner** during motion;
   - ST3215 `GoalPosition` unsigned `0..4095`, **no signed wrap**;
   - contact acquisition **RAM-only** until an explicit persistence gate;
   - **no Position Offset / EEPROM change** without a separately authorized transactional
     backup → readback → rollback flow;
   - **torque OFF** on all hardware exits and failure paths.
5. Consume schema-v4 outputs correctly: RF/RH/LH endpoints are `GEOMETRIC_ENDPOINT_CANDIDATE`,
   i.e. predicted search targets/bounds, **not** hardware-confirmed stops. Never silently
   overwrite URDF or q0 from them.
6. Preserve **LF V25 as the immutable hardware oracle**. Do not re-run or re-zero LF merely
   because three modeled endpoints disagree.

### Phase 2B — path / parking pre-hardware gate

Before **any** new real-leg calibration motion:

- resolve or explicitly redesign the parking/path-safety planning whose gate remains
  `passed=False` on all four legs;
- validate prerequisites and parking for the leg being calibrated using the Phase 1B path
  semantics — active revolute contact is **endstop metrology**, every other relevant contact
  remains **path safety**;
- **no motion is authorized merely because an endpoint candidate exists.**

### Phase 2C — offline validation

Unit tests; deterministic leg-profile mapping; FRONT/HIND geometry handling; mirror logic; safe
prerequisites; path/parking checks; failure and abort paths; torque-off contracts; and **no
hardware imports or access in offline tests**.

### Phase 3 — hardware sequence

Only after Phase 2 offline review **and a new explicit hardware authorization**:

```text
RF -> validate/freeze -> RH -> validate/freeze -> LH -> validate/freeze
   -> complete 12-joint persistent profile
```

Hardware legs are **not** batch-approved in advance. Each leg is its own evidence gate.

---

## 5. Diagnostic evidence archive

```text
/home/matteo-manicardi/MATDOG/_archive/geometry-diagnostics/
  GATE_A_ADJACENT_BASELINE_2026-08-08     (210 files + SHA256SUMS.txt)
  GATE_B_MOTORPIN_SUPPORTED_2026-08-08    (25 files + SHA256SUMS.txt)
```

Retained in full, with their originals preserved under `/tmp` for the session lifetime.

---

## 6. Boundary

No hardware, Station, serial, servo motion, torque or EEPROM access was involved in Phase 1B or
in this closeout. `norma-core` was not modified. The historical RF worktree was not touched.
No force-push, no history rewrite, no branch-protection bypass.

**Phase 1B CLOSED. Phase 2 NEXT. Phase 2 NOT STARTED.**
