# MATDOG PHASE 2A — G0/G1 EXECUTOR REPORT

---

## 1. Executive result

```text
G0 micro-preflight:                 VERIFIED
G1 inventory:                       COMPLETE
repository modifications performed: NONE
hardware actions performed:         NONE

EXECUTOR_ASSESSMENT: READY_FOR_CHATGPT_REVIEW
```

No BLOCKER found. The immutable LF V25 oracle is intact and **byte-identical** to canonical `norma-core/main` for every calibrator source file. G0 continued automatically into G1 per §9.

Three findings require an architectural decision before G2/G3 and are raised in §13:

1. **The permanent detector rule has a bounded, deliberate exception in current LF V25 code** (`friction plateau bypass`) where `ContactConfirmed` does **not** stop advancing. §15's rule as literally stated is not what the code implements.
2. **`JOINT_SPECS` makes all four legs geometrically identical** (same MIN/MAX deltas for HIP/UPPER/LOWER); only `motor_id` and `direction` differ per leg. Front/hind and left/right distinctions currently survive only in two hard-coded helper functions, not in data.
3. **Three mutually non-identical tick-level LF references exist** (in-code witness constants, `tools/matdog/README.md` evidence table, and the degree-level `MATDOG_LF_V25_HARDWARE_EVIDENCE_2026-08-04.json`). They agree only *within* the 24-tick witness tolerance, not exactly. G4 must declare which is the regression oracle.

---

## 2. G0 evidence

### 2.1 Path resolution

`norma-core` is **not** at `/home/matteo-manicardi/MATDOG/github/norma-core`. It resolves to the documented fallback:

```text
robot-dog:  /home/matteo-manicardi/MATDOG/github/robot-dog
norma-core: /home/matteo-manicardi/norma-core
```

`/home/matteo-manicardi/MATDOG/github/` contains only `robot-dog`.

### 2.2 robot-dog

| Item | Observed | Expected | Class |
|---|---|---|---|
| remote | `https://github.com/MattRobotics/robot-dog.git` | — | VERIFIED |
| current branch | `main` | — | VERIFIED |
| local HEAD | `e71876e80c23c370f9fecf36ddf15f152faf5eb3` (`docs(calib): make Phase 1B handoff provenance non-self-referential (#18)`, 2026-08-09) | — | VERIFIED (see note) |
| `origin/main` | `bd5aa8edbd903531885a3439b29a7303009e838b` | `bd5aa8e…` | **VERIFIED** |
| `ls-remote refs/heads/main` | `bd5aa8edbd903531885a3439b29a7303009e838b` | `bd5aa8e…` | **VERIFIED** |
| remote heads | `main` only (+3 tags) | only `main` | VERIFIED |
| `docs/phase2a0-closeout-alignment` remote | absent | absent | VERIFIED |
| working tree | clean, 0 untracked | clean | VERIFIED |

**Note (characterized precisely, not a blocker):** local `main` is **behind `origin/main` by 2 commits**. This is a local checkout lag, not a divergence — `e71876e` is an ancestor of `bd5aa8e`. All G1 reading of robot-dog content was done from `origin/main` blobs via `git show`, never from the stale working tree.

**Worktrees:**

```text
/home/matteo-manicardi/MATDOG/github/robot-dog                 e71876e [main]
/home/matteo-manicardi/MATDOG/worktrees/robot-dog-geometry-v5  2890daf [matdog/geometry-compiler-v5-collision-baseline]
```

The second worktree was **not named in the brief**. Reported as required: it is **clean, 0 untracked**, HEAD `2890daf0a8ac6103d3856f208a5f042528fc0da0` (`fix(geometry): separate canonical V5 from G4 replay`). Its local branch's remote-tracking ref is `gone` — consistent with the branch having been deleted remotely after merge. `2890daf` is **not** an ancestor of `origin/main`; this is expected because PR #19 was **squash-merged** as `f07aa094a1b78c5670cc36ef3fdb349422a38955` (stated in the handoff). Classification: **VERIFIED** (stale local ref + preserved review worktree, harmless).

### 2.3 norma-core

| Item | Observed | Expected | Class |
|---|---|---|---|
| remotes | `origin` → `MattRobotics/norma-core`; `upstream` → `norma-core/norma-core` (push `DISABLED`) | — | VERIFIED |
| current branch | `main` | — | VERIFIED |
| local HEAD | `f47b1ba579c623139058a8b0118648015739ab10` (`docs(matdog): freeze canonical geometry-first calibration architecture`) | — | VERIFIED (see note) |
| `origin/main` | `4a8ed6337261553b79c928975808d294c9ca723b` (`docs(matdog): align canonical entry after Geometry V5 (#35)`) | `4a8ed63…` | **VERIFIED** |
| `ls-remote refs/heads/main` | `4a8ed6337261553b79c928975808d294c9ca723b` | `4a8ed63…` | **VERIFIED** |
| `origin/release/matdog-lf-calibrator-v25` | `f87dd1fbc7e8100d275c74f9af448642f3429680` | `f87dd1f…` | **VERIFIED** |
| `ls-remote refs/heads/release/…v25` | `f87dd1fbc7e8100d275c74f9af448642f3429680` | `f87dd1f…` | **VERIFIED** |
| remote heads | `main`, `release/matdog-lf-calibrator-v25` only | exactly these two | **VERIFIED** |
| `docs/matdog-phase2a-entry-alignment` remote | absent | absent | VERIFIED |
| working tree | clean, 0 untracked | clean | VERIFIED |

**Note:** local `main` is **behind `origin/main` by 1 commit** (`f47b1ba` is the handoff-documented entry SHA; `4a8ed63` adds the Phase 2A entry handoff + README update). Same handling as robot-dog.

`upstream/dev` and `upstream/main` remote-tracking refs exist (upstream fork, push disabled). Harmless; classified VERIFIED.

### 2.4 LF V25 oracle integrity — the critical check

Blob-level comparison, `origin/release/matdog-lf-calibrator-v25` vs `origin/main`:

```text
IDENTICAL  software/drivers/st3215/src/auto_calibrate/matdog.rs        65c16f3c2f551b975570a89cf5e5934cf0ca48a2
IDENTICAL  software/drivers/st3215/src/auto_calibrate/matdog_test.rs   b6e6b6e767569081978d48d871300fe79af2d42e
IDENTICAL  software/drivers/st3215/src/auto_calibrate/mod.rs           63384ffe5632868fefbbb9e2c26338da329bb180
IDENTICAL  software/drivers/st3215/src/port.rs                         ae86f1f44441ccfc4b50454e6171a3e3867eca53
IDENTICAL  software/drivers/st3215/src/bin/matdog_lf_freeze.rs         67cebee386f9b366ab380f456e98024b2b29b92a
IDENTICAL  .github/workflows/matdog-native-calibrator-check.yml        ccc4b75610d291971e4a6f708418ce09f36cbf1b
DIFFERENT  .github/workflows/matdog-native-observer-check.yml          5f1be2c4… (release) -> 48589179… (main)
```

**The entire LF runtime, its tests, the port gate and the freeze binary are unchanged between the immutable oracle and canonical main.** This is the strongest available evidence that the LF oracle has not drifted.

Full `release → main` delta (name-status), for completeness:

```text
D  .github/workflows/matdog-lf-freeze-artifact.yml
D  .github/workflows/matdog-v42-pinned-station.yml
M  .github/workflows/matdog-native-observer-check.yml
M  README.md
A  software/station/examples/arm2dog-py/**            (10 files)
A  tools/matdog/**                                    (7 docs/handoffs)
```

Two completed one-shot workflows were removed on main; no runtime change. Classification: **VERIFIED**.

Working-tree blobs vs `origin/main` blobs (all inspected files): **SAME**, except `tools/matdog/README.md` (differs) and `MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md` (absent locally) — both fully explained by the 1-commit lag. Every Rust source and every `tools/matdog/*.py` file in the working tree is byte-identical to canonical main, so `Read`-tool inspection of the working tree is canonical-faithful.

### 2.5 Canonical entry handoff — cross-repository

Present in **both** repositories, as expected:

```text
norma-core:  tools/matdog/MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md
             blob cbe72b944922bda5f6fd0111f6998acdd934163b   (362 lines)
robot-dog:   06_Software/Matdog_Core/calibration/MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md
             blob 4dd97e7ea416c5b7f836fdf0383e21eea1e26a47   (419 lines)
```

**The two copies are not byte-identical.** Full diff reviewed. The robot-dog copy is a **superset/expanded variant**: it adds a pointer to `09_Logs/Development_Log/2026-08-11_PHASE2A0_GEOMETRY_V5_CLOSEOUT.md`, an explicit *"Forbidden target architecture"* block naming `LfSessionStateMachine`/`RfSessionStateMachine`/…, an explicit live-FK classification block (`NON-BLOCKING for Phase 2A0` / `POTENTIAL Phase 2A / live-FK blocker`), the explicit `threshold = 3 mm` line, and expanded G0 bullet lists. Everything else is rewording.

**No canonical value contradicts**: Geometry V5 semantic hashes, V5 result counts, the 3 mm policy counts, the eight UNRESOLVED targets, LF hardware contact angles, the ST3215 constraints, the A/B/C/D definitions and the G0–G9 gate list are identical in both. Classification: **VERIFIED** (documentation variance, not a contract conflict).

### 2.6 Historical RF worktree

```text
path:            /home/matteo-manicardi/MATDOG/worktrees/norma-core-rf-calibrator
branch:          matdog/rf-calibrator-from-lf-v25
HEAD:            b2f7dac2eab7147917fccdfde702360da82ab7de  ("ci: remove completed final pruning workflow", 2026-08-05)
git common dir:  /home/matteo-manicardi/norma-core/.git
git dir:         /home/matteo-manicardi/norma-core/.git/worktrees/norma-core-rf-calibrator

dirty (tracked):
  M software/drivers/st3215/src/auto_calibrate/matdog.rs
  M software/drivers/st3215/src/auto_calibrate/matdog_test.rs
untracked:       (none)

diffstat vs b2f7dac:
  matdog.rs        | 1112 +++++++++++++++-
  matdog_test.rs   |   78 ++
  2 files changed, 1176 insertions(+), 14 deletions(-)
```

HEAD matches the expected historical checkpoint exactly; the modified file set matches the expected set exactly; nothing extra is dirty or untracked.

Classification: **EXPECTED_DIRTY_HISTORICAL_EVIDENCE**. Not touched.

### 2.7 Hardware / process inspection (read-only)

```text
/dev/ttyUSB*   : none
/dev/ttyACM*   : none
/dev/ttyAMA*   : none
lsof on serial : nothing
```

Processes matching `matdog|station|st3215|servo|calibrat`:

```text
3464  /home/matteo-manicardi/bin/devtunnel host matdog-github-mcp.eun1
3833  /home/matteo-manicardi/.local/bin/claude remote-control --name "ASUS - MATDOG" --spawn=session
```

Both are session/tooling infrastructure matching only on the literal string `matdog`/`MATDOG`. **No MATDOG calibrator, no Station, no ST3215 process, and no serial hardware is present.** Nothing was stopped or interacted with.

### 2.8 G0 classification summary

| # | Item | Class |
|---|---|---|
| 1 | robot-dog `origin/main` = `bd5aa8e` (local + ls-remote) | VERIFIED |
| 2 | robot-dog local checkout 2 commits behind (ancestor, clean) | VERIFIED |
| 3 | robot-dog extra clean worktree `robot-dog-geometry-v5` @ `2890daf`, remote-tracking gone (squash-merged as `f07aa09`) | VERIFIED |
| 4 | norma-core resolved at fallback path `/home/matteo-manicardi/norma-core` | VERIFIED |
| 5 | norma-core `origin/main` = `4a8ed63` (local + ls-remote) | VERIFIED |
| 6 | norma-core local checkout 1 commit behind (ancestor, clean) | VERIFIED |
| 7 | LF release ref = `f87dd1f` (local + ls-remote) | VERIFIED |
| 8 | LF calibrator sources byte-identical release ↔ main | VERIFIED |
| 9 | Only `main` + `release/…v25` exist remotely on norma-core | VERIFIED |
| 10 | `upstream/dev`, `upstream/main` stale remote-tracking refs | VERIFIED (harmless) |
| 11 | Cross-repo handoff copies textually differ, semantically identical | VERIFIED |
| 12 | RF worktree @ `b2f7dac`, exactly 2 expected dirty files | EXPECTED_DIRTY_HISTORICAL_EVIDENCE |
| 13 | No serial devices, no MATDOG/Station/ST3215 processes | VERIFIED |
| 14 | Geometry V5 canonical semantic hashes present on robot-dog `origin/main` | VERIFIED |

**No BLOCKER. G0 → G1 continued automatically.**

---

## 3. Sources inspected

**norma-core (canonical `origin/main` content):**

- `software/drivers/st3215/src/auto_calibrate/matdog.rs` (4948 lines — read in full across regions: constants, tables, profile builders, RAM/goal gates, corridors, detector, state machine, evidence/affine derivation, orchestration, calibrator methods, helpers)
- `software/drivers/st3215/src/auto_calibrate/matdog_test.rs` (2992 lines, 88 `#[test]` — full test-name inventory extracted)
- `software/drivers/st3215/src/auto_calibrate/mod.rs` (189 lines, full)
- `software/drivers/st3215/src/port.rs` (2129 lines — MATDOG regions: arming gate, thermal supervisor, forced torque-off, EEPROM cache)
- `software/drivers/st3215/src/bin/matdog_lf_freeze.rs` (596 lines, full)
- `software/drivers/st3215/src/state.rs` (calibration API surface)
- `tools/matdog/matdog_lf_profile.py` (354 lines, full)
- `tools/matdog/matdog_native_observer_contract.py` (49 lines, full)
- `tools/matdog/matdog_v42_pinned_launcher.py` (76 lines, full)
- `tools/matdog/matdog_headless_auto_calibrate.py` (2201 lines — contract constants and class/function map)
- `tools/matdog/README.md`, `MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md`
- `.github/workflows/matdog-native-calibrator-check.yml` (full)
- `.github/workflows/matdog-native-observer-check.yml` (full)

**robot-dog (canonical `origin/main` content):**

- `06_Software/Matdog_Core/calibration/MATDOG_PHASE2A_GENERIC_V25_ENTRY_HANDOFF_2026-08-11.md`
- `06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml`
- `06_Software/Matdog_Core/calibration/MATDOG_LF_V25_HARDWARE_EVIDENCE_2026-08-04.json`
- `06_Software/Matdog_Core/kinematics/matdog_leg_fk_live.py` (status contract line)
- Geometry V5 artifact/hash presence sweep across `06_Software/` and `09_Logs/`

**Historical RF worktree:** structural diff of `matdog.rs` (added consts/enums/structs/fns) and full diff of `matdog_test.rs`.

---

## 4. LF state-machine reconstruction

### 4.1 Two arming sentinels, three run paths

`MATDOG_NATIVE_CALIBRATOR_ARM` selects the path (`matdog.rs:546 active_profile`):

| Arm value | Path | Notes |
|---|---|---|
| `LF_LEG_STATE_MACHINE` | `run_lf_full_calibration` → `run_lf_state_machine` | 58 steps; the hardware-validated V25 flow |
| `LF_HIP_M13_MIN_MAX` | `run_lf_hip_min_max` | 20 steps; shared-geometry HIP MIN+MAX |
| one of 24 `{LEG}_{JOINT}_M{id}_{SIDE}` | `run_profile` → `run()` | 16 steps; single contact. **All HIP variants blocked** by `hardware_profile_allowed` |

`hardware_profile_allowed` (`matdog.rs:536`) rejects every isolated HIP profile with `HIP_HARDWARE_BLOCK_REASON`. Only the two LF sequences may reach HIP hardware.

### 4.2 `LfSessionState` — 18 states, strictly linear + universal `Cleanup` escape

```text
Preflight
  └─> InitialRecovery
       └─> Parking
            └─> UpperMin ─> UpperMax ─> UpperHorizontal
                 └─> LowerMin ─> LowerMax ─> LowerFolded
                      └─> HipMin ─> HipMax
                           └─> Diagnostics
                                └─> ReturnHip ─> ReturnLowerHeld ─> ReturnUpper
                                     └─> RestoreParking ─> Cleanup ─> TorqueOff

any state ──────────────────────────────────────────────────────────> Cleanup
```

`transition()` (`matdog.rs:1066`) permits only: self-transition, the exact adjacent pair list above, **or `Cleanup` from anywhere**. Anything else is `Err("invalid LF state transition")`. Every non-`Cleanup` entry additionally runs `validate_transition_entry`.

### 4.3 Per-state detail

| State | Entry: required held set | Required prev-active | Active motor allowed | Action | Detector | Transition out | Timeout / failure | Backoff | Restore | Next | A/B/C/D |
|---|---|---|---|---|---|---|---|---|---|---|---|
| `Preflight` | `{}` | — | none | exact-ID-set wait; verified global torque OFF; q=0 normalization of all 12; build session from entry positions | n/a | after session created | any error → `Cleanup` | n/a | n/a | `InitialRecovery` | A |
| `InitialRecovery` | `{}` | — | `11,12,13` | re-home LF joints only if `lf_initial_recovery_needed` (>16 ticks from HOME or speed >4) | n/a | roles verified | `Cleanup` | n/a | n/a | `Parking` | A |
| `Parking` | `{}` | — | `42` | `prepare_motor(42)`; move to `UPPER_30_DELTA` (LH upper); promote to held | n/a | `StableTargetGate` (4 samples / 400 ms / ≤10 ticks / speed ≤4) | `move_lf_session_motor_to` deadline → Err → `Cleanup` | n/a | n/a | `UpperMin` | A behavior / **B target** |
| `UpperMin` | `{42}` | — | `12` | `prepare_motor(12)`; baseline → coarse scout → backoff → fine 1 → backoff → fine 2 | **active** | contact pair complete | guard/stall/tracking → Err → `Cleanup` | `BACKOFF_TICKS`=96, current must recover below threshold | n/a | `UpperMax` | A |
| `UpperMax` | `{42}` | M12 | `12` | `stop_pressure` at MIN contact, then same 6-phase search on the opposite side | **active** | pair complete | as above | as above | n/a | `UpperHorizontal` | A |
| `UpperHorizontal` | `{42}` | M12 | `12` | move M12 **directly from MAX contact** to `UPPER_90_DELTA`; promote to held | n/a | stable hold | as above | n/a | n/a | `LowerMin` | A / **B pose** |
| `LowerMin` | `{12,42}` | — | `11` | `prepare_motor(11)`; 6-phase search | **active** | pair complete | as above | as above | n/a | `LowerMax` | A |
| `LowerMax` | `{12,42}` | M11 | `11` | opposite side | **active** | pair complete | as above | as above | n/a | `LowerFolded` | A |
| `LowerFolded` | `{12,42}` | M11 | `11` | move M11 to `LOWER_FOLDED_DELTA`; promote to held | n/a | stable hold | as above | n/a | n/a | `HipMin` | A / **B pose** |
| `HipMin` | `{11,12,42}` | — | `13` | `prepare_motor(13)`; 6-phase search using `lf_hip_sequence_profile(Min)` | **active** | pair complete | as above | as above | n/a | `HipMax` | A |
| `HipMax` | `{11,12,42}` | M13 | `13` | opposite side | **active** | pair complete | as above | as above | n/a | `Diagnostics` | A |
| `Diagnostics` | `{11,12,42}` | M13 | `13` | derive `ModelZeroEstimate` + `AffineJointCalibration` + contact-witness for all 3 joints; log evidence; **freeze gate** | n/a | all 3 `accepted` | any joint rejected → Err → `Cleanup` (no motion) | n/a | n/a | `ReturnHip` | A gate / **D witness** |
| `ReturnHip` | `{11,12,42}` | M13 | `13` | move M13 to `outcome.joints[0].affine.estimated_zero_tick`; hold | n/a | stable hold | as above | n/a | **staged q0** | `ReturnLowerHeld` | A |
| `ReturnLowerHeld` | `{11,12,13,42}` | — | `11` | release M11 hold; move M11 to affine staged q0; hold | n/a | stable hold | as above | n/a | staged q0 | `ReturnUpper` | A |
| `ReturnUpper` | `{11,12,13,42}` | — | `12` | release M12 hold; move M12 to affine staged q0; hold | n/a | stable hold | as above | n/a | staged q0 | `RestoreParking` | A |
| `RestoreParking` | `{11,12,13,42}` | — | `42` | release M42 hold; move M42 back to `HOME_TICK` | n/a | stable hold | as above | n/a | parking undo | `Cleanup` | A |
| `Cleanup` | `{}` (bypasses hold check) | — | none | sync-write `TorqueEnable=0` to **all 12**; verify every motor reports torque OFF; clear holds | n/a | all verified OFF | any motor still ON → Err | n/a | n/a | `TorqueOff` | **C** |
| `TorqueOff` | `{}` | — | none | terminal | n/a | — | — | — | — | — | **C** |

`Diagnostics` is the only state where `transition()` does **not** clear `self.active` (`matdog.rs:1101`) — M13 must remain the recorded active motor to satisfy `Diagnostics → ReturnHip`'s `required_previous_active`.

### 4.4 Continuous role supervision (runs on every telemetry frame)

`validate_lf_session_snapshot` (`matdog.rs:1482`) re-checks the exact 12-ID set and then, for every motor except the one being moved, `validate_lf_role_observation`:

| Role | Invariant |
|---|---|
| `ActivelyCommanded{t}` / `ContactProbe{t}` | torque ON, `torque_limit == 500`, `goal == t`, present **and** target inside `lf_participant_corridor` |
| `ActivelyHeld{t}` | all of the above **plus** positional error ≤ `STATIC_TOLERANCE_TICKS` (10) |
| `PassiveTorqueOffSafe{corridor}` | torque **must be OFF**; position inside state-dependent corridor |
| `NonParticipatingTorqueOff{entry}` | torque **must be OFF**; drift from session-entry tick ≤ `NON_PARTICIPATING_MAX_DRIFT_TICKS` (16) |

Plus, for every role: telemetry age ≤ 3 s and non-zero stamp; `status == 0`; no driver error; `current < 200`; `temperature_limit == 70` exactly and `temperature ≤ limit`.

### 4.5 Contact-side search (the 6-phase unit, `measure_lf_contact_side_efficient`)

```text
1. stop_pressure at previous contact (if second side of a pair)
2. acquire_moving_current_baseline_forward   -> BaselineStats{median, MAD}
3. approach_with_scout(COARSE_STEP_TICKS=64, scout=None)   -> coarse_scout_tick   [DISCARDED as metrology]
4. backoff_and_verify(coarse, BACKOFF_TICKS=96)            -> current must fall below threshold
5. approach_with_scout(FINE_STEP_TICKS=8, scout=coarse)    -> first_tick
6. backoff_and_verify(first)
7. approach_with_scout(FINE_STEP_TICKS=8, scout=coarse)    -> second_tick
8. repeatability_spread(first, second)
   contact_tick = circular_midpoint_tick(first, second)
```

### 4.6 Detector outcomes (`HybridContactDetector::observe`, `matdog.rs:1850`) — the safety core

```text
HardAbort         -> abort_with_global_torque_off()      IMMEDIATE, all 12 motors
EarlyStall        -> stop_pressure(); Err                fail closed (stall outside acceptance corridor)
ContactConfirmed  -> stop_pressure(); return position    STOP ADVANCING
ContactSuspected  -> keep sampling (needs 3 consecutive)
FreeMotion        -> continue
guard reached     -> Err before any command is issued    fail closed
```

`stop_pressure(motor_id, present_position)` sets `GoalPosition := present position` — the probe stops pushing at the tick where contact was confirmed. This is the correct realisation of §15's rule.

`HardAbort` triggers on any of: driver error, non-zero status, torque OFF, `torque_limit != 500`, `goal != commanded`, `current >= 200`.

`ContactConfirmed` requires **all** of: ≥3 consecutive qualifying samples, `travel ≥ 24` ticks from start, `progress ≤ 2` ticks/sample, `velocity ≤ 10`, target still ahead in probe direction, goal error above the settle tolerance, `target_samples_seen > 4`, **and** position inside the adaptive acceptance corridor. Outside the corridor the same evidence yields `EarlyStall` instead — fail closed. **Current is computed (`_current_supports_contact`) but deliberately not used as a gate**; contact is kinematic, not current-based (test: `current_rise_without_kinematic_stall_is_not_contact`).

### 4.7 ⚠ The one exception to "ContactConfirmed → STOP ADVANCING IMMEDIATELY"

In `approach_with_scout` (`matdog.rs:4015-4041`), when a fine pass (`coarse_scout_tick = Some`) receives `ContactConfirmed`:

```rust
if !fine_contact_reproduces_coarse_depth(observation.position, scout, probe_sign) {
    info!("... friction plateau bypass ...");
    continue 'approach_steps;      // <-- no stop_pressure(); the probe advances one more step
}
```

A confirmed contact **shallower than the coarse scout by more than `FINE_STEP_TICKS` (8)** is reclassified as a friction/chamfer plateau and the probe **keeps advancing**. The identical bypass exists on the adaptive `confirm_kinematic_plateau` branch (`matdog.rs:4123-4139`).

**Bounding that does apply:** the mechanical guard is still enforced on every step, the adaptive acceptance corridor is still enforced, and the reference depth is a *previously physically measured* coarse contact on the same joint/side — so the probe is re-approaching a depth hardware has already reached. It is not "push until an LF span is reached".

**But** it is a real, deliberate deviation from §15 as literally worded, it is the only place in LF V25 where `ContactConfirmed` does not stop advance, and G2/G3 must carry it forward *explicitly* rather than accidentally. Raised as **AMB-1** in §13.

---

## 5. A/B/C/D master classification

Legend — Owner: `SM` = `LegSessionStateMachine`, `SPEC` = `LegCalibrationSpec`, `SAFETY` = global ST3215 safety layer, `EVID` = historical regression evidence, `CI` = CI-only oracle.
HW impact: NONE / LOW / MEDIUM / HIGH / CRITICAL.

| ID | Symbol | File | Semantic role | Cls | Rationale | LF dependency | Future owner | Preserve exactly? | HW impact | Risk if misclassified | Consumers | Test/CI witness | Open question |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 01 | `MATDOG_MOTOR_IDS` | matdog.rs:22 | canonical 12-ID robot topology | **C** | whole-robot identity, not per-leg; gates dispatch, sync-write, torque-off | none | SAFETY | YES | CRITICAL | wrong robot accepted; partial torque-off | mod.rs, port.rs, all gates | `exact_matdog_id_set_is_required`; `exact_motor_topologies_dispatch_without_overlap` | — |
| 02 | `MATDOG_ARM_ENV` | matdog.rs:23 | explicit arming env var | **C** | motion-authority switch | none | SAFETY | YES | CRITICAL | unarmed motion | mod.rs, port.rs | `unsupported_arming_values_are_rejected` | should Phase 2A add a leg-scoped arm token? |
| 03 | `HOME_TICK = 2048` | matdog.rs:25 | digital zero / q=0 | **C** | ST3215 unsigned midpoint + established digital zero | none | SAFETY (+ SPEC reference) | YES | CRITICAL | q0 convention break; EEPROM offset math wrong | everything | CI `expected_constants` pins 2048 | — |
| 04 | `TICKS_PER_REVOLUTION = 4096` | matdog.rs:26 | encoder resolution | **C** | ST3215 hardware constant | none | SAFETY | YES | HIGH | degree conversion + wrap math wrong | `ticks_to_degrees`, wrap helpers | `wrap_math_is_local_but_goal_targets_remain_unsigned` | — |
| 05 | `protocol::MAX_ANGLE_STEP` | protocol | unsigned ceiling 4095 | **C** | signed-wrap prohibition | none | SAFETY | YES | CRITICAL | signed GoalPosition | every tick constructor | `wrap_math_is_local_but_goal_targets_remain_unsigned` | — |
| 06 | `GUARD_OVERSHOOT_TICKS = 64` | matdog.rs:27 | guard beyond URDF limit | **A** | uniform algorithmic margin, leg-independent | none | SM | YES | CRITICAL | probe overtravel past mechanical stop | `build_profile` | `every_guard_extends_only_64_ticks_beyond_its_urdf_limit`; CI pins 64 | should it become per-joint spec data? currently uniform |
| 07 | `BASELINE_TRAVEL_TICKS = 64` | matdog.rs:28 | current-baseline travel | **A** | measurement mechanics | none | SM | YES | MEDIUM | bad threshold → false/late contact | baseline acquisition | `robust_current_baseline_uses_median_and_mad` | — |
| 08 | `TORQUE_LIMIT = 500` | matdog.rs:34 | commanded torque ceiling | **C** | global ST3215 energy cap (half of range) | none | SAFETY | YES | CRITICAL | mechanical damage on contact | RAM gate, every readback validator | `v38_motion_envelope_is_faster_but_keeps_bounded_contact_guards` | should it become per-joint later? not now |
| 09 | `GOAL_SPEED = 160` | matdog.rs:35 | commanded speed | **C** | global motion envelope (V38) | none | SAFETY | YES | HIGH | impact energy at contact | RAM gate, `prepare_motor` | `v38_motion_envelope_is_faster_but_keeps_bounded_contact_guards` | — |
| 10 | `ACCELERATION = 8` | matdog.rs:36 | commanded accel | **C** | global motion envelope | none | SAFETY | YES | HIGH | as above | RAM gate | same | — |
| 11 | `COARSE_STEP_TICKS = 64` | matdog.rs:37 | scouting increment | **A** | search mechanics | none | SM | YES | MEDIUM | overshoot per step | `run`, `measure_*` | `coarse_scout_is_persisted_but_cannot_change_fine_metrology_or_q0` | — |
| 12 | `FINE_STEP_TICKS = 8` | matdog.rs:38 | metrology increment | **A** | search mechanics | none | SM | YES | MEDIUM | metrology resolution loss | `measure_*` | `v38_repeatability_compares_two_identical_fine_approaches_not_the_coarse_scout` | — |
| 13 | `BACKOFF_TICKS = 96` | matdog.rs:39 | retreat after contact | **A** | pressure release | none | SM | YES | HIGH | sustained load on endstop | `backoff_and_verify` | — | — |
| 14 | `STATIC_TOLERANCE_TICKS = 10` | matdog.rs:40 | hold/settle gate | **A** | uniform static criterion | none | SM | YES | HIGH | undetected hold drift | holds, gates, corridors | `actively_held_static_role_uses_position_error_not_instantaneous_speed` | — |
| 15 | `OUTSIDE_CORRIDOR_SETTLE_TOLERANCE_TICKS = 16` | matdog.rs:45 | settle tol outside corridor | **A** | bounded continuation, derived from V36 M13 evidence | **D-derived** | SM | YES | HIGH | premature/late contact outside corridor | detector | `lf_hip_max_v36_1981_settle_continues_once_but_real_stall_and_contact_remain_bounded` | value justified by one LF observation — generic? |
| 16 | `PROBE_TRACKING_ERROR_FLOOR_TICKS` (=16) | matdog.rs:49 | outer tracking floor | **A** | kept equal to #15 by construction | D-derived | SM | YES | HIGH | tracking/plateau misjudged | `probe_tracking_error_limit` | `v24_m13_fine_tracking_uses_detector_consistent_global_floor` (CI-required) | — |
| 17 | `PROBE_HOME_TOLERANCE_TICKS = 16` | matdog.rs:53 | probe-at-home tol | **A** | backlash/prereq-load allowance | D-derived | SM | YES | MEDIUM | false startup accept | startup envelopes, `run` | `probe_home_tolerance_is_scoped_to_startup_home_endpoint_and_active_probe_returns` | — |
| 18 | `PROBE_PASSIVE_RESTORE_DRIFT_TICKS = 32` | matdog.rs:57 | passive displacement tol | **A** | reverse-recovery physics | D-derived | SM | YES | MEDIUM | undetected passive motion | `home_hold_tolerance`, parking corridor | `probe_reverse_recovery_accepts_observed_2031_then_requires_final_rehome` | — |
| 19 | `STARTUP_PREREQUISITE_HOME_SETTLE_TICKS = 16` | matdog.rs:62 | startup endpoint widening | **A** | restart-safety only | D-derived (M42 2037) | SM | YES | MEDIUM | weakened live hold gate | `startup_prerequisite_bounds` | `startup_prerequisite_home_endpoint_accepts_observed_m42_2037_without_weakening_target` | — |
| 20 | `STARTUP_HOME_RECOVERY_LIMIT_TICKS = 64` | matdog.rs:63 | recovery distance cap | **A** | bounded recovery | none | SM | YES | MEDIUM | unbounded startup motion | startup recovery | `startup_v10_pose_classifies_m42_as_valid_prerequisite_residue` | — |
| 21 | `REPEATABILITY_TOLERANCE_TICKS = 16` | matdog.rs:64 | fine-to-fine spread cap | **A** | metrology acceptance | none | SM | YES | MEDIUM | poor contact accepted | `repeatability_spread` | `repeatability_uses_circular_distance_and_preserves_unsigned_goals` | — |
| 22 | `BASELINE_MIN_SAMPLES = 6` | matdog.rs:65 | baseline sample floor | **A** | statistics quality | none | SM | YES | LOW | noisy threshold | baseline acquisition | `robust_current_baseline_uses_median_and_mad` | — |
| 23 | `MINIMUM_CONTACT_TRAVEL_TICKS = 24` | matdog.rs:66 | min travel before contact | **A** | rejects stall-at-start | none | SM | YES | HIGH | contact declared without motion | detector config | `detector_confirms_only_persistent_stall_inside_profile_corridor` | — |
| 24 | `TARGET_STARTUP_SAMPLES = 4` | matdog.rs:67 | ignore first N samples | **A** | command-latency masking | none | SM | YES | MEDIUM | false contact on command edge | detector | `direction_generic_detector_confirms_stall_in_both_tick_directions` | — |
| 25 | `CONTACT_SETTLE_WINDOW = 900 ms` | matdog.rs:68 | per-step settle window | **A** | search timing | none | SM | YES | MEDIUM | truncated evidence | `approach_with_scout` | — | — |
| 26 | `HARD_CURRENT_ABORT_RAW = 200` | matdog.rs:69 | hard current abort | **C** | global ST3215 protection | none | SAFETY | YES | **CRITICAL** | motor/mechanism damage | detector, `ensure_observation_safe`, role validator | `hard_abort_inputs_are_direction_independent` | — |
| 27 | `EXPECTED_TEMPERATURE_LIMIT_C = 70` + `MAX_TEMPERATURE_LIMIT_ADDRESS 0x0D` | matdog.rs:70-71 | thermal contract | **C** | EEPROM-configured protection must be exactly 70 | none | SAFETY | YES | **CRITICAL** | thermal runaway | `validate_matdog_temperature`, port.rs supervisor | `matdog_temperature_contract_requires_exact_70c_limit_and_rejects_over_limit` | — |
| 28 | `COMMAND_TIMEOUT 5 s`, `TELEMETRY_TIMEOUT 2 s`, `MAX_TELEMETRY_AGE 3 s` | matdog.rs:72-74 | liveness bounds | **C** | stale-telemetry fail-closed | none | SAFETY | YES | **CRITICAL** | commanding on stale state | every wait/validate | CI requires `MAX_TELEMETRY_AGE` token | — |
| 29 | `MOTION_TIMEOUT 12 s`, `MIN_EXPECTED_MOTION_TICKS_PER_SECOND 80`, `MOTION_SETTLE_MARGIN 5 s` | matdog.rs:75-82 | distance-scaled deadline | **A** | motion mechanics | D-derived (M12 MAX return) | SM | YES | MEDIUM | premature abort / runaway wait | `motion_timeout_for_distance` | `motion_timeout_covers_observed_m12_max_return`; `motion_timeout_keeps_short_moves_fast_and_scales_for_every_profile` | — |
| 30 | `LF_ALLOWED [11,12,13,42]` | matdog.rs:84 | LF motor allowlist **incl. LH parking joint** | **B** | leg-scoped participant set | LF | SPEC | YES | HIGH | cross-leg command leak | RAM gate, `is_lf_full_sequence` | `armed_motor_allowlists_are_leg_scoped_and_include_front_parking_joint` | — |
| 31 | `RF_ALLOWED [21,22,23,32]` | matdog.rs:85 | RF allowlist incl. RH upper | **B** | same | none | SPEC | N/A | HIGH | as above | `Leg::allowed_motor_ids` | same test | — |
| 32 | `RH_ALLOWED [31,32,33]`, `LH_ALLOWED [41,42,43]` | matdog.rs:86-87 | hind allowlists, **3 motors, no parking joint** | **B** | encodes front/hind asymmetry | none | SPEC | N/A | HIGH | wrong parking assumption for hind | same | same | hind parking strategy undefined for hardware |
| 33 | `HIP_MIN_DELTA −512` / `HIP_MAX_DELTA +512` | matdog.rs:89-90 | URDF hip limits | **B** | geometry data | **shared by all 4 legs** | SPEC | YES for LF | HIGH | wrong endpoint target | `JOINT_SPECS` | `directions_transform_q_limits_into_leg_specific_unsigned_ticks` | see AMB-2 |
| 34 | `UPPER_MIN_DELTA −597` / `UPPER_MAX_DELTA +1394` | matdog.rs:91-92 | URDF upper limits | **B** | geometry data | shared by all 4 legs | SPEC | YES for LF | HIGH | as above | `JOINT_SPECS` | `lf_upper_m12_max_profile_matches_reviewed_geometry` | see AMB-2 |
| 35 | `LOWER_MIN_DELTA −1047` / `LOWER_MAX_DELTA +427` | matdog.rs:93-94 | URDF lower limits | **B** | geometry data | shared by all 4 legs | SPEC | YES for LF | HIGH | as above | `JOINT_SPECS` | `lf_lower_profiles_use_horizontal_upper_and_exact_unsigned_numbers` | see AMB-2 |
| 36 | `UPPER_30_DELTA = 341` | matdog.rs:95 | contralateral-rear parking pose | **B** | prerequisite geometry | LF/RF only | SPEC | YES | HIGH | rear leg collision during front calibration | `prerequisites_for`, parking corridor | `front_profiles_park_only_the_ipsilateral_rear_upper`; CI pins 341 | is this a legacy 30° replay artefact? see AMB-4 |
| 37 | `UPPER_90_DELTA = 1024` | matdog.rs:96 | horizontal upper pose | **B** | prerequisite geometry | LF | SPEC | YES | HIGH | LOWER search geometry wrong | `prerequisites_for`, `UpperHorizontal` | CI pins 1024 | as above |
| 38 | `UPPER_85_DELTA = 967` | matdog.rs:97 | side-specific hip clearance | **B** | **front/hind + side asymmetry** | LF MAX / RF MIN | SPEC | YES | HIGH | hip contact geometry wrong | `hip_upper_clearance_delta` | `hip_prerequisites_are_compact_and_side_specific`; CI pins 967 | why 85° only on those two arms? |
| 39 | `LOWER_FOLDED_DELTA = −990` | matdog.rs:98 | folded-lower pose for HIP | **B** | prerequisite geometry | LF | SPEC | YES | HIGH | hip sweep collision | `prerequisites_for`, `LowerFolded` | `lf_hip_sequence_uses_one_horizontal_parallel_pose_for_both_contacts` | — |
| 40 | `CONTACT_ACCEPTANCE_INNER_TICKS = 64` | matdog.rs:99 | inner edge of acceptance | **A** | corridor construction | none | SM | YES | **CRITICAL** | contact accepted far from model | `contact_acceptance_bounds` | `contact_acceptance_corridors_match_model_inner_boundary_and_guard` | — |
| 41 | `ADAPTIVE_FINE_SCOUT_TICKS = 32` | matdog.rs:102 | corridor widening away from guard | **A** | adaptive corridor, never widens toward guard | none | SM | YES | **CRITICAL** | corridor widened toward mechanical stop | `adaptive_contact_acceptance_bounds` | `v41_adaptive_fine_corridor_accepts_observed_hip_max_plateau_without_moving_guard` | — |
| 42 | `FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS` (= 8) | matdog.rs:106 | friction-plateau bypass threshold | **A** | governs the §4.7 exception | D-derived | SM | YES | **CRITICAL** | probe keeps advancing after real contact | `approach_with_scout` bypass | `fine_contact_scout_depth_gate_is_direction_independent_and_bounded` | **AMB-1** |
| 43 | `AFFINE_SCALE_MIN/MAX_PERMILLE 850/1150` | matdog.rs:107-108 | affine span acceptance band | **A** | global sanity band on encoder/transmission scale | none | SM (gate) | YES | HIGH | bad scale accepted → wrong staged q0 | `derive_affine_joint_calibration` | `affine_gate_accepts_real_span_while_fixed_scale_stays_diagnostic` | ±15 % is wide — intentional? |
| 44 | `KINEMATIC_PLATEAU_SAMPLES 3`, `..._POSITION_SPAN_TICKS 3` | matdog.rs:109-110 | adaptive plateau confirmation | **A** | detector mechanics | none | SM | YES | HIGH | plateau mistaken for contact | `confirm_kinematic_plateau` | `v41_adaptive_fine_corridor_accepts_…` | — |
| 45 | `MODEL_ZERO_ENDPOINT_CONSISTENCY_TICKS = 24` | matdog.rs:111 | fixed-scale disagreement limit | **A** (diagnostic) | **explicitly diagnostic only** — does not gate acceptance | none | SM (diagnostic) | YES | MEDIUM | raw model-vs-hardware disagreement erased | `derive_model_zero` | `model_zero_gate_rejects_endpoint_disagreement_even_when_midpoint_is_near_2048`; `affine_gate_accepts_real_span_while_fixed_scale_stays_diagnostic` | §18: this is the raw-disagreement record — must survive |
| 46 | `LF_CONTACT_WITNESS_TOLERANCE_TICKS = 24` | matdog.rs:112 | LF witness tolerance | **D** | compares against LF measured ticks | **LF-only** | EVID | LF only | HIGH | LF ticks become other-leg law | `lf_contact_witness_accepted` | `lf_contact_witness_gate_is_uniform_and_rejects_v24_m12_cable_obstruction`; CI pins 24 | **must not generalize** |
| 47 | `lf_reference_contact_ticks()` → Hip (2535,1617), Upper (1443,3442), Lower (3093,1666) | matdog.rs:1979 | LF hardware witness ticks | **D** | pure historical LF measurement | **LF-only** | EVID | LF only | HIGH | **RF/RH/LH endpoints copied from LF** | `derive_joint_evidence` | `historical_contacts_use_affine_and_uniform_witness_freeze_gate` (CI-required) | **AMB-3** (differs from README/JSON) |
| 48 | `MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS = 96` | matdog.rs:113 | max q0 shift from digital home | **C** | protects the established digital zero | none | SAFETY | YES | **CRITICAL** | digital zero silently redefined | affine + fixed gates, `full_sequence_joint_goal_allowed` | `model_zero_solver_recovers_exact_urdf_home_without_fitting_encoder_scale` | — |
| 49 | `LF_TRANSITION_SETTLED_SAMPLES 4`, `LF_TRANSITION_SETTLE_WINDOW 400 ms` | matdog.rs:114-115 | hold-promotion dwell | **A** | anti-single-sample promotion | none | SM | YES | HIGH | hold promoted on transient | `StableTargetGate` | `final_hold_promotion_requires_stable_fresh_dwell_not_one_crossing_sample` | — |
| 50 | `LF_HELD_MAX_SPEED_RAW = 4` | matdog.rs:116 | quiescence threshold | **A** | settle criterion | none | SM | YES | HIGH | moving joint treated as held | `StableTargetGate`, `lf_initial_recovery_needed` | same | — |
| 51 | `NON_PARTICIPATING_MAX_DRIFT_TICKS = 16` | matdog.rs:117 | drift cap for the other 8 motors | **C** | whole-robot supervision | none | SAFETY | YES | **CRITICAL** | unnoticed motion elsewhere on robot | `validate_lf_role_observation` | `nonparticipating_torque_off_uses_real_position_drift_not_instantaneous_speed` | — |
| 52 | `HIP_HARDWARE_BLOCK_REASON` + `hardware_profile_allowed` | matdog.rs:118,536 | blocks isolated HIP profiles | **C** | motion-authority policy | LF exemptions | SAFETY | YES | **CRITICAL** | unreviewed HIP hardware motion | `active_profile` | `isolated_hip_hardware_profiles_are_blocked_but_lower_is_allowed`; `lf_hip_combined_sequence_is_the_only_unblocked_hip_hardware_path` | generalize per-leg in G2? |
| 53 | `Leg`, `JointKind`, `ContactSide` enums | matdog.rs:122-179 | identity vocabulary | **A** | already generic | none | SM + SPEC | YES | NONE | — | everywhere | — | — |
| 54 | `JointSpec{leg,kind,name,motor_id,direction,min_delta,max_delta}` | matdog.rs:182 | per-joint record | **B** | the de-facto proto-spec | none | **SPEC** | YES | HIGH | wrong motor/direction → wrong physical joint | `JOINT_SPECS`, everything | `directions_transform_q_limits_into_leg_specific_unsigned_ticks` | direct ancestor of `LegCalibrationSpec` |
| 55 | `JOINT_SPECS[12]` | matdog.rs:214 | complete robot joint table | **B** | data, not behavior | none | SPEC | YES | HIGH | robot mis-modelled | `spec_for` | `profile_table_covers_exactly_24_unique_contacts` | **AMB-2** |
| 56 | `direction: i8` per joint | matdog.rs:186 | encoder sign per joint | **B** | genuinely leg/joint-specific | none | SPEC | YES | **CRITICAL** | probe drives away from the intended endstop | `tick_for_delta`, `build_profile` | `directions_transform_q_limits_into_leg_specific_unsigned_ticks` | — |
| 57 | `JointSpec::tick_for_delta` | matdog.rs:200 | q-delta → unsigned tick, range-checked | **A** | generic conversion enforcing unsignedness | none | SM | YES | **CRITICAL** | signed wrap | `build_profile`, `static_target` | `wrap_math_is_local_but_goal_targets_remain_unsigned` | — |
| 58 | `ContactProfile` | matdog.rs:332 | per-contact runtime bundle | **A**+**B** mix | behavior fields + geometry fields intermixed | LF specials | split SM/SPEC | YES | HIGH | spec/engine boundary drawn wrong | all run paths | `profile_table_covers_exactly_24_unique_contacts` | the key G2 split |
| 59 | `build_profile` | matdog.rs:415 | derives guard/baseline/probe_sign from spec | **A** | generic derivation | none | SM | YES | **CRITICAL** | guard misplaced | everywhere | `every_guard_extends_only_64_ticks_beyond_its_urdf_limit` | — |
| 60 | `probe_sign = direction × q_sign` | matdog.rs:422 | probing tick direction | **A** | generic rule | none | SM | YES | **CRITICAL** | wrong-direction probing | detector, guards, backoff | `direction_generic_detector_confirms_stall_in_both_tick_directions` | — |
| 61 | `prerequisites_for` | matdog.rs:372 | per-(leg,joint,side) prerequisite poses | **B** data in **A** shape | **hard-codes front/hind asymmetry in a match arm** | LF/RF park rear | SPEC (data) driven by SM | YES | **CRITICAL** | collision during calibration | all run paths | `front_profiles_park_only_the_ipsilateral_rear_upper`; `prerequisites_are_unique_and_never_include_the_probe_motor` | must become model/profile-driven (§20) |
| 62 | `hip_upper_clearance_delta` | matdog.rs:363 | 90° vs 85° per (leg,side) | **B** data in **A** shape | explicit front/hind + left/right asymmetry table | LF/RF | SPEC | YES | **CRITICAL** | hip clearance wrong | `prerequisites_for` | `hip_prerequisites_are_compact_and_side_specific` | must become data |
| 63 | `prerequisite_restore_order` | matdog.rs:406 | reverse-order restore, probe excluded | **A** | generic restore rule | none | SM | YES | **CRITICAL** | rear parking released too early → collision | `restore_prerequisites` (via `held_targets` pop) | `front_lower_restore_order_keeps_rear_parking_until_active_leg_is_home` (CI-required) | currently `#[cfg(test)]`-only helper mirroring production LIFO |
| 64 | `LF_HIP_SEQUENCE_ARM_VALUE` / `LF_FULL_SEQUENCE_ARM_VALUE` | matdog.rs:100-101 | LF arming sentinels | **D** | LF-only entry tokens | LF-only | EVID / SPEC-id | YES | HIGH | wrong sequence armed | `profile_for_arm_value` | `lf_full_sequence_is_one_explicit_hardware_arm_with_union_goal_gate` | generic naming in G2 |
| 65 | `lf_hip_sequence_profile`, `lf_full_sequence_profile`, `is_lf_*` | matdog.rs:465-497 | LF sequence constructors/predicates | **A** shape, **D** binding | generic sequence concept hard-bound to LF | LF-only | SM (generalized) | behavior YES | HIGH | per-leg duplication (the RF trap) | run dispatch, RAM gate | `lf_hip_combined_sequence_is_the_only_unblocked_hip_hardware_path` | **primary G3 target** |
| 66 | `all_profiles()` → 24 | matdog.rs:499 | full contact enumeration | **A** | generic | none | SM | YES | LOW | profile coverage loss | `profile_for_arm_value` | `profile_table_covers_exactly_24_unique_contacts` (CI-required) | — |
| 67 | `ram_write_allowed_for_profile` | matdog.rs:561 | RAM register/value/motor allowlist | **C** | the motion-authority gate | LF branch inside | SAFETY | YES | **CRITICAL** | arbitrary RAM writes while armed | port.rs | `armed_ram_gate_restricts_registers_values_motors_and_goal_windows` (CI-required); `only_ram_motion_registers_are_allowlisted` | LF branch must generalize without weakening |
| 68 | `armed_goal_target_allowed` | matdog.rs:731 | GoalPosition corridor gate | **C** | bounds every commandable target | LF branches | SAFETY | YES | **CRITICAL** | target outside mechanical corridor | port.rs | `full_lf_port_gate_allows_only_exact_home_normalization_for_non_participants` | — |
| 69 | universal `target == HOME_TICK` allowance | matdog.rs:736 | any canonical motor may be sent to 2048 | **C** | uniform q=0 normalization | none | SAFETY | YES | HIGH | 12-motor motion authority is broader than one leg | startup normalization | `startup_home_goal_gate_is_global_and_exact_home_only_for_non_profile_joints`; `full_lf_q0_normalization_has_no_distance_admission_window` | acceptable for a per-leg engine? |
| 70 | `lf_full_joint_corridor`, `lf_parking_corridor`, `lf_passive_corridor`, `lf_participant_corridor` | matdog.rs:666-729 | state-dependent tick corridors | **A** shape, **B** values | corridor concept generic; bounds from geometry | LF-bound | SM + SPEC | YES | **CRITICAL** | motion outside mechanical envelope | role validation, `set_active` | `active_readback_cannot_cross_the_strict_mechanical_guard` | — |
| 71 | `TickCorridor` | matdog.rs:1011 | inclusive tick interval | **A** | generic | none | SM | YES | NONE | — | corridors | — | — |
| 72 | `is_exact_matdog_motor_set` | matdog.rs:792 | exact-set check | **C** | robot identity | none | SAFETY | YES | **CRITICAL** | wrong topology accepted | mod.rs dispatch, run paths | `exact_matdog_id_set_is_required` | — |
| 73 | `contact_acceptance_bounds` / `adaptive_…` | matdog.rs:800-834 | model corridor for contact | **A** | generic, guard-anchored | none | SM | YES | **CRITICAL** | contact accepted outside model | detector | `contact_acceptance_corridors_match_model_inner_boundary_and_guard` | — |
| 74 | `ContactState` enum | matdog.rs:878 | detector verdicts | **A** | generic | none | SM | YES | **CRITICAL** | verdict semantics lost | `approach_with_scout` | `v19_m13_2405_is_early_stall_not_contact` | — |
| 75 | `HybridContactConfig` (`max_progress 2`, `max_velocity 10`, `persistence 3`) | matdog.rs:1755-1775 | detector tuning | **A** | generic detector policy | none | SM | YES | **CRITICAL** | false/missed contact | detector | `detector_confirms_only_persistent_stall_inside_profile_corridor` | — |
| 76 | `HybridContactDetector::observe` | matdog.rs:1850 | the contact detector | **A** | the core generic algorithm | none | SM | **YES, exactly** | **CRITICAL** | the central safety behavior | approach loops | `current_rise_without_kinematic_stall_is_not_contact`; `hard_abort_inputs_are_direction_independent` | includes §4.7 exception |
| 77 | `BaselineStats` + `contact_threshold()` = median + max(4·MAD, 5) | matdog.rs:1727-1752 | robust current baseline | **A** | generic statistics | none | SM | YES | MEDIUM | used for backoff recovery check only | `backoff_and_verify`, logs | `robust_current_baseline_uses_median_and_mad` | — |
| 78 | `StableTargetGate` | matdog.rs:887 | dwell-based settle gate | **A** | generic | none | SM | YES | HIGH | premature hold | LF moves | `final_hold_promotion_requires_stable_fresh_dwell_not_one_crossing_sample` | — |
| 79 | `MotorObservation` | matdog.rs:920 | telemetry snapshot | **C** | the safety observation contract | none | SAFETY | YES | **CRITICAL** | missing safety field | everything | `observation_reads_required_live_registers_and_error_state` | — |
| 80 | `validate_matdog_temperature` | matdog.rs:934 | thermal validation | **C** | global protection | none | SAFETY | YES | **CRITICAL** | thermal event | roles, `ensure_*` | `matdog_temperature_contract_requires_exact_70c_limit_and_rejects_over_limit` | — |
| 81 | `LfSessionState` (18) | matdog.rs:951 | LF choreography | **A** structure, **D** naming | ordering is generic; names/order are LF-specific | LF | SM | behavior YES | **CRITICAL** | duplicated per-leg machines (forbidden) | everything | `lf_state_machine_runs_the_full_simulated_path_with_runtime_roles` | **CI pins `ReturnHip` textually** — see §7 |
| 82 | `LfMotorRole` (5) | matdog.rs:1023 | per-motor role vocabulary | **A** | generic and reusable | none | SM | YES | **CRITICAL** | role confusion → unsupervised motion | role validation | `non_participating_and_held_role_failures_are_detected_from_simulated_telemetry` | — |
| 83 | `LfSessionStateMachine` | matdog.rs:1032 | the engine | **A** | **this is `LegSessionStateMachine`** | LF-bound | SM | YES | **CRITICAL** | the whole Phase 2A goal | everything | `lf_state_model_rejects_wrong_actuator_hold_and_missing_prerequisite` | — |
| 84 | `transition()` adjacency table | matdog.rs:1066 | legal transitions | **A** structure, **B** order | order is leg-geometry-driven | LF | SM (+SPEC order) | YES | **CRITICAL** | illegal sequence | all | `lf_state_model_rejects_wrong_actuator_hold_and_missing_prerequisite` | order must become spec data |
| 85 | universal `→ Cleanup` escape | matdog.rs:1068 | fail-closed terminal | **C** | every state can reach verified torque-OFF | none | SAFETY | **YES, exactly** | **CRITICAL** | unreachable safe stop | all | `every_lf_state_can_fail_into_the_same_verified_cleanup_terminal`; `production_global_cleanup_executes_command_and_fresh_readback_from_every_lf_state` | — |
| 86 | `validate_transition_entry` required-hold sets | matdog.rs:1109 | per-state hold preconditions | **A** rule, **B** sets | motor IDs are LF | LF | SM + SPEC | YES | **CRITICAL** | moving without prerequisite holds | transitions | `lf_state_model_rejects_wrong_actuator_hold_and_missing_prerequisite` | — |
| 87 | `active_motor_allowed` | matdog.rs:1166 | one active motor per state | **A** rule, **B** IDs | invariant is generic | LF IDs | SM + SPEC | YES | **CRITICAL** | multiple joints advancing | `set_active` | `production_snapshot_verifier_checks_m11_hold_while_m12_is_active` | — |
| 88 | `hold()` / `release()` | matdog.rs:1243-1288 | held-target lifecycle | **A** rule, **B** pairs | promotion requires a completed matching active move | LF pairs | SM + SPEC | YES | **CRITICAL** | phantom hold | LF moves | `final_hold_promotion_requires_stable_fresh_dwell_not_one_crossing_sample` | — |
| 89 | `role_for` | matdog.rs:1290 | role resolution incl. non-participants | **A** | generic | LF ID list | SM | YES | **CRITICAL** | unsupervised motor | snapshot validation | `non_participating_and_held_role_failures_…` | — |
| 90 | `complete_verified_cleanup` | matdog.rs:1352 | terminal transition | **C** | verified end state | none | SAFETY | YES | **CRITICAL** | session ends torque-ON | `global_torque_off_verified` | `global_torque_off_cleanup_is_exact` | — |
| 91 | `validate_lf_role_observation` / `_active_readback` / `_session_snapshot` | matdog.rs:1378-1506 | continuous 12-motor supervision | **C** | whole-robot safety supervision | LF-named | SAFETY | **YES, exactly** | **CRITICAL** | undetected unsafe state | every frame | `production_snapshot_verifier_replays_m23_2140_and_m11_2059_runtime_paths` | rename ≠ behavior change |
| 92 | `StartupRole`, `StartupEntryPlan`, `startup_*_bounds`, `startup_envelope` | matdog.rs:1509-1630 | restart-safe entry classification | **A** | generic restart safety | LF/profile-bound | SM | YES | **CRITICAL** | motion from unknown pose | `run`, `run_lf_*` | `startup_envelopes_match_exhaustive_oracle_for_all_profiles_and_ticks`; `startup_wrong_profile_residue_is_rejected` | — |
| 93 | `ModelZeroEstimate` + `derive_model_zero` | matdog.rs:1937,2223 | fixed-scale q0 + raw disagreement | **A** (diagnostic) | preserves raw model-vs-hardware disagreement | none | SM | YES | MEDIUM | §18 violated — disagreement erased | diagnostics, logs | `model_zero_solver_recovers_exact_urdf_home_without_fitting_encoder_scale` | must remain visible |
| 94 | `AffineJointCalibration` + `derive_affine_joint_calibration` | matdog.rs:1951,2000 | fitted affine q0 + scale | **A** | authoritative for staging | none | SM | YES | HIGH | fitted model hides disagreement | staging targets, profile record | `v41_affine_solver_accepts_complete_observed_lf_contact_set` | §18 tension: authoritative yet "diagnostic normalization" |
| 95 | `derive_joint_evidence` → `accepted: affine.accepted && contact_witness_accepted` | matdog.rs:2094 | the freeze gate | **A** gate + **D** term | witness term is LF-only | **LF-bound** | SM (gate) / EVID (term) | YES | **CRITICAL** | LF witness applied to RF/RH/LH | freeze gate | CI requires the exact token `accepted: affine.accepted && contact_witness_accepted` | **the single most important A/D split** |
| 96 | `lf_machine_profile_record` → `MATDOG_LF_PROFILE_V1\|…` | matdog.rs:2110 | machine record for the serializer | **A** shape, **D** name | the cross-language contract | LF-named | SM + serializer | YES | HIGH | serializer breaks silently | `matdog_lf_profile.py` | CI requires the exact prefix in both files | format must version on generalization |
| 97 | `joint_degree_evidence` | matdog.rs:2149 | human evidence line incl. residuals | **A** | diagnostics | LF-labelled | SM | NO (text) | NONE | evidence readability | logs | `degree_diagnostics_preserve_lf_direction_sign_range_and_endpoint_residuals` | — |
| 98 | `circular_midpoint_tick`, `signed_tick_delta`, `circular_distance`, `directional_progress`, `advance_tick`, `passed_guard`, `crossed_home`, `speed_magnitude`, `median` | matdog.rs:2205,4869-4948 | wrap-safe tick math | **C** | ST3215 unsigned arithmetic | none | SAFETY | YES | **CRITICAL** | signed wrap / wrong guard | everywhere | `wrap_math_is_local_but_goal_targets_remain_unsigned`; `repeatability_uses_circular_distance_…` | — |
| 99 | `is_allowed_matdog_ram_register`, `validate_ram_write` | matdog.rs:2251-2276 | RAM-only allowlist | **C** | RAM-only contract | none | SAFETY | YES | **CRITICAL** | EEPROM reachable | all writes | `only_ram_motion_registers_are_allowlisted`; `canonical_matdog_source_has_no_eeprom_reset_offset_regwrite_action_or_freeze_path` (CI-required) | — |
| 100 | `global_torque_off_writes` + `global_torque_off_verified` | matdog.rs:2278,4411 | sync-write OFF to all 12 + readback | **C** | the terminal safety action | none | SAFETY | **YES, exactly** | **CRITICAL** | robot left energized | all run paths, abort | `global_torque_off_cleanup_is_exact` | — |
| 101 | `abort_with_global_torque_off` | matdog.rs:4239 | immediate abort | **C** | fail-closed | none | SAFETY | YES | **CRITICAL** | abort without de-energizing | `HardAbort` | `hard_abort_inputs_are_direction_independent` | — |
| 102 | `stop_pressure` | matdog.rs:4249 | goal := present position | **A** | the "stop advancing" primitive | none | SM | **YES, exactly** | **CRITICAL** | sustained push on endstop | contact/stall paths | `detector_confirms_only_persistent_stall_inside_profile_corridor` | — |
| 103 | `backoff_and_verify` + `crossed_home` check | matdog.rs:4211 | retreat + current recovery + home-cross refusal | **A** | controlled backoff | none | SM | YES | **CRITICAL** | backoff crosses q=0 into opposite domain | search unit | — | — |
| 104 | `restore_prerequisites` (LIFO pop + torque OFF each) | matdog.rs:3604 | reverse-order restore | **A** | keeps rear parking last | none | SM | YES | **CRITICAL** | rear leg released too early | `run` | `front_lower_restore_order_keeps_rear_parking_until_active_leg_is_home` | — |
| 105 | `normalize_all_matdog_joints_to_q0` + `verify_uniform_startup_home_snapshot` | matdog.rs:3698,3614 | uniform 12-joint q=0 entry | **A** | one uniform rule, no distance window | none | SM | YES | **CRITICAL** | motion from unknown robot pose | `run_lf_state_machine` | `full_lf_normalizes_all_twelve_joints_before_creating_strict_session_roles`; `full_lf_q0_normalization_has_no_distance_admission_window` | — |
| 106 | `prepare_motor` | matdog.rs:3875 | prime at observed tick then torque ON | **A** | avoids step-change on enable | none | SM | YES | **CRITICAL** | jerk on torque enable | all paths | `startup_probe_and_prerequisite_corridors_are_restart_safe` | — |
| 107 | `ensure_observation_safe` / `_fresh` / `_temperature_safe` | matdog.rs:4632-4722 | per-frame safety | **C** | fail-closed frame validation | LF role hook | SAFETY | YES | **CRITICAL** | acting on unsafe/stale state | everywhere | `production_snapshot_verifier_replays_…` | — |
| 108 | `check_stop` / `stop_requested` | matdog.rs:4724 | operator abort | **C** | operator authority | none | SAFETY | YES | **CRITICAL** | unstoppable session | all loops | — | — |
| 109 | `next_phase` / `publish_progress` / `total_steps` 16 / 20 / **58** | matdog.rs:4732-4763 | progress telemetry | **A** + **D** | 58 is the LF witness step count | LF | SM / EVID | 58 = LF only | LOW | external runner asserts 58 | Station, headless runner | runner `EXPECTED_FULL_TOTAL_STEPS = 58` | must version per leg |
| 110 | `next_command_id` (app_start_id, nonce, counter) | matdog.rs:4887 | command scoping | **C** | prevents cross-session command confusion | none | SAFETY | YES | HIGH | stale command accepted | all writes | `command_ids_are_scoped_and_monotonic`; `command_result_and_ram_readback_are_matched_exactly` | — |
| 111 | `mod.rs: calibration_kind` / `AUTO_CALIBRATE_SUPPORTED_IDS` | mod.rs:39,51 | robot-type dispatch | **C** | topology gate vs SO101/ElRobot | none | SAFETY | YES | **CRITICAL** | MATDOG path on wrong robot | `calibrate()` | `exact_motor_topologies_dispatch_without_overlap` | — |
| 112 | `mod.rs: matdog_calibrator_is_armed` / `matdog_armed_ram_write_allowed` | mod.rs:15-21 | driver↔port bridge | **C** | authority boundary | none | SAFETY | YES | **CRITICAL** | gate bypass | port.rs | CI requires both tokens in `mod.rs` | — |
| 113 | `port.rs: matdog_command_allowed_with` / `matdog_armed_command_allowed` | port.rs:80,146 | rejects non-MATDOG-RAM commands while armed | **C** | outermost motion-authority gate | none | SAFETY | **YES, exactly** | **CRITICAL** | Station/other client commands during calibration | Station command path | CI requires all three port tokens | — |
| 114 | `port.rs` thermal supervisor + `force_matdog_motor_torque_off` | port.rs:44-76,454 | independent thermal kill | **C** | second, independent safety layer | none | SAFETY | YES | **CRITICAL** | thermal event unhandled if driver hangs | port loop | — | not covered by the LF test file |
| 115 | `bin/matdog_lf_freeze.rs` (whole binary) | — | EEPROM Offset transaction | **C** + **D** | separate operator tool; LF-only motor list | `LF_MOTORS [11,12,13]` | SAFETY (out of scope) / EVID | YES | **CRITICAL** | EEPROM reachable in Phase 2A | operator only | `offset_delta_matches_displayed_position_convention` | **must stay unreachable from the Phase 2A path** |
| 116 | `COMMIT_TOKEN = "LF_FREEZE_COMMIT"` | freeze:15 | explicit EEPROM authorization | **C** | human authorization gate | LF-named | SAFETY | YES | **CRITICAL** | accidental EEPROM write | freeze binary | argument parser refuses without it | — |
| 117 | `matdog_lf_profile.py` (`MATDOG_LF_PROFILE_V1`, stage/finalize) | tools | evidence → staged/frozen profile | **A** shape, **D** LF binding | serializer boundary | LF joints hard-coded | SPEC serializer | YES | HIGH | staged q0 wrong at EEPROM time | freeze binary, global index | `test_matdog_lf_profile.py`; CI requires `estimated_q0_tick={record['q0_affine']}` | uses **affine** q0, never fixed — CI enforces |
| 118 | `matdog_native_observer_contract.py` | tools | external observer, **must not** duplicate motion policy | **C** | authority boundary | none | SAFETY (external) | YES | HIGH | external policy shadow-fork | launcher, runner | observer workflow forbids `M42`, `HOME_TICK`, corridors, `EXPECTED_ACTIVE_TORQUE_LIMIT`, … | — |
| 119 | `matdog_v42_pinned_launcher.py` (runner/observer/Station SHA pins) | tools | provenance pinning | **C** | reviewed-artifact enforcement | pins LF-era SHAs | provenance adapter | pins must update | HIGH | unreviewed runner executed | operator entry | `test_matdog_v42_pinned_launcher.py` | pins are **stale by construction** after any G2 edit |
| 120 | `matdog_headless_auto_calibrate.py` (`EXPECTED_BUS_SERIAL`, `EXPECTED_ACTIVE_TORQUE_LIMIT=500`, `EXPECTED_FULL_TOTAL_STEPS=58`, `verify_station_process`) | tools | fail-closed external runner | **C** + **D** | safety invariants + LF step-count witness | 58, bus serial | SAFETY / EVID | 58 = LF only | HIGH | external gate breaks on generalization | Station | `test_matdog_headless_auto_calibrate.py` | 58 must become per-spec |

---

## 6. Constant inventory (grouped)

**Motion / search mechanics** — *generic policy (A)*
`GUARD_OVERSHOOT_TICKS 64` · `BASELINE_TRAVEL_TICKS 64` · `COARSE_STEP_TICKS 64` · `FINE_STEP_TICKS 8` · `BACKOFF_TICKS 96` · `STATIC_TOLERANCE_TICKS 10` · `CONTACT_ACCEPTANCE_INNER_TICKS 64` · `ADAPTIVE_FINE_SCOUT_TICKS 32` · `MINIMUM_CONTACT_TRAVEL_TICKS 24` · `CONTACT_SETTLE_WINDOW 900 ms`

**Detector / current behavior** — *A, except where noted*
`HybridContactConfig{max_progress 2, max_velocity 10, persistence 3}` · `BaselineStats::contact_threshold = median + max(4·MAD, 5)` · `TARGET_STARTUP_SAMPLES 4` · `KINEMATIC_PLATEAU_SAMPLES 3` · `KINEMATIC_PLATEAU_POSITION_SPAN_TICKS 3` · `FINE_CONTACT_SCOUT_LAG_TOLERANCE_TICKS 8` · **`HARD_CURRENT_ABORT_RAW 200` → C**

**Timeouts** — *C (liveness) / A (motion)*
C: `COMMAND_TIMEOUT 5 s` · `TELEMETRY_TIMEOUT 2 s` · `MAX_TELEMETRY_AGE 3 s`
A: `MOTION_TIMEOUT 12 s` · `MIN_EXPECTED_MOTION_TICKS_PER_SECOND 80` · `MOTION_SETTLE_MARGIN 5 s`

**Repeatability / settle** — *A*
`REPEATABILITY_TOLERANCE_TICKS 16` · `BASELINE_MIN_SAMPLES 6` · `LF_TRANSITION_SETTLED_SAMPLES 4` · `LF_TRANSITION_SETTLE_WINDOW 400 ms` · `LF_HELD_MAX_SPEED_RAW 4` · `OUTSIDE_CORRIDOR_SETTLE_TOLERANCE_TICKS 16` · `PROBE_TRACKING_ERROR_FLOOR_TICKS 16` · `PROBE_HOME_TOLERANCE_TICKS 16` · `PROBE_PASSIVE_RESTORE_DRIFT_TICKS 32` · `STARTUP_PREREQUISITE_HOME_SETTLE_TICKS 16` · `STARTUP_HOME_RECOVERY_LIMIT_TICKS 64`

**Motor identities** — *C (robot) / B (leg)*
C: `MATDOG_MOTOR_IDS` · `AUTO_CALIBRATE_SUPPORTED_IDS` · `NON_PARTICIPATING_MAX_DRIFT_TICKS 16`
B: `LF_ALLOWED [11,12,13,42]` · `RF_ALLOWED [21,22,23,32]` · `RH_ALLOWED [31,32,33]` · `LH_ALLOWED [41,42,43]` · `JointSpec.motor_id`
D: `LF_MOTORS [11,12,13]` (freeze binary)

**Motor directions** — *B*
`JointSpec.direction` per joint: LF (−1, +1, −1) · RF (−1, −1, +1) · RH (+1, −1, +1) · LH (+1, +1, −1) for (Hip, Upper, Lower). Derived: `probe_sign = direction × q_sign` (A).

**Endpoint / contact values** — *B (URDF) / C (frame)*
B: `HIP_MIN/MAX_DELTA ∓512` · `UPPER_MIN/MAX_DELTA −597/+1394` · `LOWER_MIN/MAX_DELTA −1047/+427` — **identical for all four legs**
C: `HOME_TICK 2048` · `TICKS_PER_REVOLUTION 4096` · `MAX_ANGLE_STEP 4095` · `MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS 96`

**Historical witnesses** — *D. Must never generalize.*
`lf_reference_contact_ticks`: Hip (2535, 1617) · Upper (1443, 3442) · Lower (3093, 1666)
`LF_CONTACT_WITNESS_TOLERANCE_TICKS 24` · `LF_HIP_SEQUENCE_ARM_VALUE` · `LF_FULL_SEQUENCE_ARM_VALUE` · `total_steps 58` · `EXPECTED_BUS_SERIAL/BUS_SERIAL_DEFAULT "5B14114953"` · launcher SHA-256 pins · robot-dog approved angles (−42.803/+39.375, −53.525/+122.607, −91.846/+34.277)
**The LF spans 935 / 2004 / 1435 do NOT appear anywhere in canonical LF V25 source.** They appear only in the historical RF worktree (§11).

**Geometry / prerequisite poses** — *B*
`UPPER_30_DELTA 341` · `UPPER_90_DELTA 1024` · `UPPER_85_DELTA 967` · `LOWER_FOLDED_DELTA −990` · the `prerequisites_for` table · the `hip_upper_clearance_delta` table

**Restore behavior** — *A*
LIFO `held_targets` pop → move to `HOME_TICK` → verified torque OFF each; probe re-prime/re-home after restore; `PROBE_PASSIVE_RESTORE_DRIFT_TICKS 32`; explicit refusal if backoff `crossed_home`.

**Safety / torque behavior** — *C*
`TORQUE_LIMIT 500` · `GOAL_SPEED 160` · `ACCELERATION 8` · `EXPECTED_TEMPERATURE_LIMIT_C 70` · `MAX_TEMPERATURE_LIMIT_ADDRESS 0x0D` · RAM allowlist `{TorqueEnable, Acc, GoalPosition, GoalSpeed, TorqueLimit}` · `global_torque_off_writes` (all 12, value `[0]`) · `COMMIT_TOKEN "LF_FREEZE_COMMIT"` · port.rs thermal supervisor (`0.5 s` sampling, `3` confirmation reads, `50 ms` delay)

---

## 7. Test / CI inventory

`matdog_test.rs` contains **88 `#[test]`**. Classification of the materially relevant ones:

### GENERIC_BEHAVIOR_TEST — should survive unchanged; changing them could conceal an LF regression

| Test | Invariant protected | Survives? | Migration? |
|---|---|---|---|
| `direction_generic_detector_confirms_stall_in_both_tick_directions` | detector is sign-independent | YES | none |
| `detector_confirms_only_persistent_stall_inside_profile_corridor` | 3-sample persistence + corridor | YES | none |
| `current_rise_without_kinematic_stall_is_not_contact` | contact is kinematic, not current | YES | none |
| `v19_m13_2405_is_early_stall_not_contact` | outside-corridor stall ≠ contact | YES | none |
| `fine_contact_scout_depth_gate_is_direction_independent_and_bounded` | governs the §4.7 bypass | YES | none |
| `contact_acceptance_corridors_match_model_inner_boundary_and_guard` | corridor construction | YES | none |
| `v41_adaptive_fine_corridor_accepts_observed_hip_max_plateau_without_moving_guard` | adaptive widening never toward guard | YES | none |
| `robust_current_baseline_uses_median_and_mad` | baseline robustness | YES | none |
| `every_guard_extends_only_64_ticks_beyond_its_urdf_limit` | guard rule | YES | leg loop may widen |
| `motion_timeout_keeps_short_moves_fast_and_scales_for_every_profile` | deadline scaling | YES | none |
| `startup_envelopes_match_exhaustive_oracle_for_all_profiles_and_ticks` | exhaustive startup oracle | YES | none |
| `startup_wrong_profile_residue_is_rejected` | restart safety | YES | none |
| `final_hold_promotion_requires_stable_fresh_dwell_not_one_crossing_sample` | dwell gate | YES | none |
| `active_readback_cannot_cross_the_strict_mechanical_guard` | guard on readback | YES | none |
| `prerequisites_are_unique_and_never_include_the_probe_motor` | prerequisite sanity | YES | none |
| `coarse_scout_is_persisted_but_cannot_change_fine_metrology_or_q0` | scout is not metrology | YES | none |
| `v38_repeatability_compares_two_identical_fine_approaches_not_the_coarse_scout` | repeatability definition | YES | none |

### GLOBAL_SAFETY_TEST — CRITICAL, must not weaken

`matdog_temperature_contract_requires_exact_70c_limit_and_rejects_over_limit` · `hard_abort_inputs_are_direction_independent` · `only_ram_motion_registers_are_allowlisted` · `armed_ram_gate_restricts_registers_values_motors_and_goal_windows` · `global_torque_off_cleanup_is_exact` · `every_lf_state_can_fail_into_the_same_verified_cleanup_terminal` · `production_global_cleanup_executes_command_and_fresh_readback_from_every_lf_state` · `canonical_matdog_source_has_no_eeprom_reset_offset_regwrite_action_or_freeze_path` · `wrap_math_is_local_but_goal_targets_remain_unsigned` · `exact_matdog_id_set_is_required` · `command_ids_are_scoped_and_monotonic` · `command_result_and_ram_readback_are_matched_exactly` · `observation_reads_required_live_registers_and_error_state` · `nonparticipating_torque_off_remains_fail_closed_on_torque_or_real_drift` · `isolated_hip_hardware_profiles_are_blocked_but_lower_is_allowed` · `full_lf_port_gate_allows_only_exact_home_normalization_for_non_participants`

### LF_REGRESSION_ORACLE — LF-only; must be preserved as LF evidence, **must not become generic**

`validated_m12_min_profile_preserves_hardware_pilot_numbers` · `lf_upper_m12_max_profile_matches_reviewed_geometry` · `historical_contacts_use_affine_and_uniform_witness_freeze_gate` · `lf_contact_witness_gate_is_uniform_and_rejects_v24_m12_cable_obstruction` · `affine_gate_accepts_real_span_while_fixed_scale_stays_diagnostic` · `v41_affine_solver_accepts_complete_observed_lf_contact_set` · `current_lf_hardware_evidence_proves_upper_zero_but_requires_stronger_lower_and_hip_recheck` · `v24_m13_fine_tracking_uses_detector_consistent_global_floor` · `lf_hip_max_v36_1981_settle_continues_once_but_real_stall_and_contact_remain_bounded` · `v38_2026_08_01_failure_is_coarse_loading_not_mechanical_change` · `probe_home_tolerance_covers_observed_m13_settle_without_weakening_static_gate` · `probe_reverse_recovery_accepts_observed_2031_then_requires_final_rehome` · `startup_prerequisite_home_endpoint_accepts_observed_m42_2037_without_weakening_target` · `startup_probe_home_endpoint_accepts_observed_m11_2059_without_weakening_guard_side` · `startup_v10_pose_classifies_m42_as_valid_prerequisite_residue` · `motion_timeout_covers_observed_m12_max_return` · `production_snapshot_verifier_replays_m23_2140_and_m11_2059_runtime_paths` · `degree_diagnostics_preserve_lf_direction_sign_range_and_endpoint_residuals`

### IMPLEMENTATION_SHAPE_TEST — ⚠ will legitimately need deliberate migration in G3

| Test | Shape it pins | Why migration is needed |
|---|---|---|
| `lf_state_machine_runs_the_full_simulated_path_with_runtime_roles` | `LfSessionStateMachine` type + all 18 `LfSessionState` variants | type/variant names change under a generic engine |
| `lf_state_model_rejects_wrong_actuator_hold_and_missing_prerequisite` | LF hold sets by literal motor ID | IDs must come from spec |
| `production_snapshot_verifier_checks_m11_hold_while_m12_is_active` | LF motor IDs | as above |
| `initial_recovery_skips_safe_passive_m11_2059_and_moves_only_when_needed` | `InitialRecovery` semantics + LF IDs | as above |
| `full_lf_final_order_stages_m13_m11_m12_then_restores_m42` | exact LF return order | order becomes spec data |
| `full_lf_startup_home_normalization_is_uniform_for_all_canonical_joints` | LF-named normalization | rename only |
| `full_lf_normalizes_all_twelve_joints_before_creating_strict_session_roles` | LF-named | rename only |
| `lf_full_sequence_is_one_explicit_hardware_arm_with_union_goal_gate` | `LF_LEG_STATE_MACHINE` arm token | token becomes per-leg |
| `lf_hip_sequence_*` (4 tests) | `LF_HIP_M13_MIN_MAX` shape | as above |
| `ordered_profile_table_lists_upper_then_lower_then_hip` | table iteration order | order becomes spec data |
| `profile_table_covers_exactly_24_unique_contacts` | 24-profile enumeration | should survive |

### PROFILE_DATA_TEST
`directions_transform_q_limits_into_leg_specific_unsigned_ticks` · `hip_prerequisites_are_compact_and_side_specific` · `front_profiles_park_only_the_ipsilateral_rear_upper` · `armed_motor_allowlists_are_leg_scoped_and_include_front_parking_joint` · `lf_lower_profiles_use_horizontal_upper_and_exact_unsigned_numbers` · `front_lower_restore_order_keeps_rear_parking_until_active_leg_is_home`
These are the tests that **should become spec-parameterized** rather than deleted.

### OBSERVER_BOUNDARY_TEST
`test_matdog_native_observer_contract.py` + the `matdog-native-observer-check.yml` forbidden-token gate. Protects: the external observer must not reconstruct native motion policy. **CRITICAL to preserve.**

### FREEZE_PATH_TEST
`accepted_endpoint_q0_is_used_only_for_transactional_staging` · `canonical_matdog_source_has_no_eeprom_reset_offset_regwrite_action_or_freeze_path` · `offset_delta_matches_displayed_position_convention` · `circular_distance_handles_wrap` · `test_matdog_lf_profile.py`

### POTENTIALLY_STALE_TEST
- `prerequisite_restore_order` is `#[cfg(test)]`-only; production restore is a LIFO `pop()` in `restore_prerequisites`. The CI-required token `prerequisite_restore_order` therefore pins a **test-only mirror** of the production rule. Low risk today, but the two could silently diverge.
- `matdog_v42_pinned_launcher.py` SHA pins (runner `9eccb4aa…`, observer `b9521f97…`, Station `df4f6965…`, artifact `8869874935`) are **stale by construction** the moment G2 edits any pinned file. Not stale today; guaranteed to break on generalization.

### CI as architecture — assertions that will need deliberate migration

`matdog-native-calibrator-check.yml` performs a **token-and-constant grep gate** over the sources. Classification of what it encodes:

| Assertion group | Type | Verdict |
|---|---|---|
| `forbidden_source`: `EepromRegister`, `RamRegister::Lock`, `ST3215Request::`, `reg_write: Some`, `reset*: Some`, `freeze_calibration: Some`, `action: Some`, `Offset.address` | **true safety invariant** | keep verbatim — this is what makes EEPROM unreachable |
| `expected_constants`: `HOME_TICK 2048`, `GUARD_OVERSHOOT_TICKS 64` | **true safety invariant** | keep |
| `expected_constants`: `UPPER_30_DELTA 341`, `UPPER_85_DELTA 967`, `UPPER_90_DELTA 1024` | **implementation-shape / B data pinned in CI** | ⚠ must migrate when these become spec data |
| `expected_constants`: `LF_CONTACT_WITNESS_TOLERANCE_TICKS 24` | **LF regression witness** | keep, but scope to LF |
| `required_source`: `MATDOG LF CONTACT WITNESS:`, `MATDOG LF URDF FREEZE GATE: PASS`, `MATDOG_LF_PROFILE_V1\|joint=` | **LF-specific log/format strings** | ⚠ migrate; these are `LF`-named contracts |
| `required_source`: `accepted: affine.accepted && contact_witness_accepted` | **exact source line pinned** | ⚠ highest-friction assertion in the repo — any refactor of `derive_joint_evidence` breaks CI |
| return-section parse: finds `transition_lf_state(LfSessionState::ReturnHip)` … `RestoreParking`, requires `hip_staged_q0`/`lower_staged_q0`/`upper_staged_q0` and `affine.estimated_zero_tick`, **forbids** `fixed_scale.estimated_zero_tick` | **safety invariant expressed as implementation shape** | the *rule* (affine stages, fixed-scale never does) must survive; the *textual anchors* (`LfSessionState::ReturnHip`, the three variable names) must migrate |
| `required_tests`: 10 exact test-function names | **implementation shape** | ⚠ renaming any of these breaks CI |
| `required_port` / `required_module` tokens | **true safety invariant** | keep |
| `stale`: refuses `matdog_v2.rs` / `matdog_v2_test.rs` | anti-duplication guard | keep — and consider extending it to forbid `Rf/Rh/LhSessionStateMachine` |
| `forbidden_other`: `matdog_pilot_is_armed`, `matdog_armed_motor_ids`, `write.motor_id != 12` | stale-symbol guard | keep |
| observer workflow forbidden tokens (`M42`, `HOME_TICK`, corridors, `EXPECTED_ACTIVE_TORQUE_LIMIT`, …) | **true authority invariant** | keep verbatim |

**Net:** MATDOG CI is not merely a test runner — it is a **source-shape contract**. G2/G3 cannot be merged without a coordinated, reviewed workflow migration. That migration must be a *reviewed diff*, never an opportunistic loosening.

---

## 8. Station / observer / profile / freeze / restore dependency map

```text
Station (software/station/bin/station)
  │   NOTE: no direct textual reference to matdog/auto_calibrate in station/bin.
  │   Coupling is through the st3215 driver crate + normfs queues, not source calls.
  ▼
ST3215 port loop  (software/drivers/st3215/src/port.rs)
  ├── SOLE SERIAL OWNER during all calibration motion
  ├── matdog_armed_command_allowed()  ──> auto_calibrate::matdog_armed_ram_write_allowed()
  │       rejects every non-MATDOG-RAM command while armed
  ├── independent thermal supervisor ──> force_matdog_motor_torque_off()
  └── eeprom_cache (read-side only for MATDOG)
        ▼
auto_calibrate/mod.rs
  ├── calibration_kind()  -> Matdog | So101 | Elrobot | Unsupported   (exact-set dispatch)
  ├── matdog_calibrator_is_armed()          -> matdog::active_profile()
  └── matdog_armed_ram_write_allowed()      -> matdog::armed_ram_write_allowed()
        ▼
auto_calibrate/matdog.rs
  ├── active_profile()  <- env MATDOG_NATIVE_CALIBRATOR_ARM
  ├── auto_calibrate()  -> spawns one of:
  │        run_lf_full_calibration -> run_lf_state_machine   (58 steps)
  │        run_lf_hip_min_max      -> run_lf_hip_min_max     (20 steps)
  │        run_profile             -> run()                  (16 steps)
  ├── MatdogRamOnlyCalibrator
  │     ├── comm.send_tx(TxEnvelope)                 (writes: RAM only)
  │     ├── watch::Receiver<InferenceState>          (reads: telemetry)
  │     ├── comm.update_calibration_progress(...)    -> Station progress UI
  │     └── comm.{set,get,clear}_calibration_stop    -> operator abort
  └── emits stdout/log:
        "MATDOG LF EVIDENCE: ..."            (human)
        "MATDOG_LF_PROFILE_V1|joint=..."     (machine)   ─────┐
                                                              │
tools/matdog/matdog_v42_pinned_launcher.py                    │
  ├── SHA-256-verifies runner + observer, pins Station SHA     │
  └── installs observer -> matdog_headless_auto_calibrate      │
        ├── verify_station_process()  (Station identity/provenance)
        ├── parse_frame(): exact bus, exact 12 IDs, unsigned 12-bit goal, full RAM image
        ├── FrameContract.validate_common(): app identity, freshness, driver/status/current, thermal
        │     (NativeAuthorityFrameContract adds NOTHING — deliberate)
        └── writes Station log ────────────────────────────────┤
                                                               ▼
tools/matdog/matdog_lf_profile.py   stage
  ├── records_from_log()   parses MATDOG_LF_PROFILE_V1 records (needs all 3 LF joints)
  ├── verify_urdf()        re-derives min/max tick deltas from the real URDF, ±1 tick
  ├── sha256(urdf)         -> urdf_sha256                    <-- only provenance binding today
  └── emits profile.json (LF_STAGED) + MATDOG_LF_STAGE_V1 plan
          estimated_q0_tick = q0_AFFINE   (never q0_fixed — CI-enforced)
                                                               ▼
        [ STATION MUST BE STOPPED — serial adapter released ]
                                                               ▼
software/drivers/st3215/src/bin/matdog_lf_freeze.rs
  ├── requires --commit LF_FREEZE_COMMIT
  ├── opens the serial port DIRECTLY (separate from Station)
  ├── preconditions per motor: torque OFF, at staged q0 ±10, mode==0, max_temp==70, lock==1
  ├── writes EepromRegister::Offset with verify + retry + full rollback
  └── emits MATDOG_LF_FREEZE_RESULT_V1 (LF_FROZEN)
                                                               ▼
tools/matdog/matdog_lf_profile.py   finalize
  ├── cross-checks freeze bus_serial + urdf_sha256 against the staged profile
  ├── requires final displayed position == 2048 ±10 per motor
  └── updates MATDOG_GLOBAL_PROFILE_V1 { legs: LF/RF/RH/LH, frozen_legs, status }
```

**Boundaries G2/G3 must preserve:**

1. **port.rs is the only serial owner** while armed; the calibrator never touches a port.
2. **The observer must stay policy-free** — enforced by a forbidden-token CI gate.
3. **The machine-record string is the cross-language contract** between Rust and Python.
4. **The EEPROM transaction is a separate binary, a separate process, and requires Station to have released the adapter.** It is unreachable from the Phase 2A path.
5. **`MATDOG_GLOBAL_PROFILE_V1` already has RF/RH/LH slots** (`null`) — the natural place for per-leg spec/provenance to land.

---

## 9. Geometry V5 / provenance boundary

**Present today:**

| Mechanism | Where | Binds |
|---|---|---|
| `urdf_sha256` | `matdog_lf_profile.py` stage/finalize; `matdog_lf_freeze.rs` stage/backup/result | the exact URDF file |
| URDF limit re-derivation ±1 tick | `matdog_lf_profile.py: verify_urdf` | source deltas ↔ real URDF |
| `calibrator_commit` | `matdog_lf_profile.py` CLI arg | free-text, **unvalidated** |
| runner + observer SHA-256 | `matdog_v42_pinned_launcher.py` | reviewed tool bytes |
| Station executable SHA-256 + artifact id + source commit | same | the running Station binary |
| Station process identity/provenance | `matdog_headless_auto_calibrate.py: verify_station_process` | the live process |
| `bus_serial` | runner + profile + freeze | the physical bus |

**Geometry V5 artifacts** live entirely in **robot-dog** and are confirmed present on `origin/main`:

```text
de205209…  endpoint  -> 20 files   (incl. V5 final architecture, Phase 2A handoff, V5 benchmark JSONs)
67c58430…  parking   -> 18 files
0a772234…  combined  -> 16 files
```

**Gaps (nothing in norma-core consumes any of these):**

```text
MISSING  robot-dog geometry revision binding
MISSING  collision-mesh manifest SHA
MISSING  Geometry V5 endpoint/parking/combined semantic SHA
MISSING  hardware evidence/log SHA
MISSING  validation of calibrator_commit
MISSING  any target-domain status field (EXECUTABLE vs DIAGNOSTIC/UNRESOLVED)
MISSING  any transport of the 8 UNRESOLVED targets into the runtime/spec
```

**Critical structural observation:** `matdog.rs` derives every endpoint target from **`JOINT_SPECS` constants compiled into the binary**, not from any geometry artifact. The only URDF cross-check happens *after the fact*, in the Python serializer. So today there is **no mechanism at all** by which stale Geometry V5 provenance could be detected before motion — and equally, no mechanism by which a DIAGNOSTIC/UNRESOLVED target could enter, because the runtime has no channel for external targets whatsoever.

That is *accidentally* safe. G6's fail-closed provenance requirement is entirely unbuilt, and it becomes a *real* requirement the moment G2 introduces an external spec source. **G2 must not open that channel without the provenance gate landing in the same change.**

No G4-replay artefact (30/50/90° legacy context) was found in norma-core runtime or CI. The `UPPER_30_DELTA 341` / `UPPER_90_DELTA 1024` / `UPPER_85_DELTA 967` constants are prerequisite poses — see AMB-4.

---

## 10. Live-FK dependency finding

```text
NO_DEPENDENCY_VERIFIED
```

**Evidence.** Sweep across `software/`, `tools/` and `.github/` in norma-core (`*.rs`, `*.py`, `*.yml`, `*.yaml`, `*.toml`, `*.md`) for `matdog_leg_fk_live|VISUAL_ZERO|DIGITAL_ZERO_CALIBRATED|MATDOG_JOINT_CALIBRATION|leg_fk|fk_live`:

```text
tools/matdog/MATDOG_CALIBRATION_CANONICAL_HANDOFF_2026-08-07.md:569
    06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml
```

**One hit, in a historical documentation file, as a path reference.** Zero hits in the calibrator, the port gate, the Station integration, the profile tooling, the observer, the launcher, the freeze binary, or any workflow.

**The contradiction itself is confirmed present and unchanged in robot-dog:**

```text
06_Software/Matdog_Core/calibration/MATDOG_JOINT_CALIBRATION.yaml:5
    calibration_status: DIGITAL_ZERO_CALIBRATED_AND_VERIFIED

06_Software/Matdog_Core/kinematics/matdog_leg_fk_live.py:56
    EXPECTED_CALIBRATION_STATUS = "VISUAL_ZERO_CAPTURED_PENDING_LIVE_VALIDATION"
```

It also affects `matdog_apply_visual_zero.py:26`, `matdog_live_joint_monitor.py:106,110`, `matdog_visual_zero_pose_probe.py:46` — all robot-dog Python tooling. `matdog_calibration_validate.py:105` is the one script aligned with the YAML.

**Conclusion:** the mismatch is entirely confined to robot-dog live-FK / visual-zero tooling. Per §19, it is **isolated and deferred** to that owning workflow. Not fixed. Digital-zero status not touched.

---

## 11. Historical RF forensic comparison

The RF worktree (`b2f7dac` + 1176 uncommitted insertions) added:

```text
RF_FULL_SEQUENCE_ARM_VALUE = "RF_LEG_STATE_MACHINE"
RF_WITNESS_TOLERANCE_TICKS = LF_CONTACT_WITNESS_TOLERANCE_TICKS

enum   RfSessionState            (incl. HipDown / HipUp)
struct RfSessionStateMachine     { new, transition, validate_transition_entry,
                                   active_motor_allowed, set_active, update_active_target,
                                   clear_active, hold, release, role_for, record_contacts,
                                   record_diagnostics, has_complete_evidence,
                                   trace_summary, complete_verified_cleanup }

fn rf_full_sequence_profile / is_rf_full_sequence / rf_full_sequence_goal_allowed
fn rf_full_joint_corridor / rf_parking_corridor / rf_passive_corridor / rf_participant_corridor
fn rf_contact_state
fn validate_rf_role_observation / validate_rf_active_readback / validate_rf_session_snapshot
fn rf_reference_span_ticks / rf_expected_second_contact_region
fn rf_contact_witness_deviation / rf_contact_witness_accepted
fn derive_rf_joint_evidence
fn position_inside_expected_region
methods: transition_rf_state, set_rf_active, record_rf_contacts, record_rf_diagnostics,
         verify_rf_session_others_except, measure_rf_joint_pair_efficient,
         measure_rf_hip_pair_physical_order, measure_rf_second_contact_side_with_anchor,
         approach_with_scout_and_expected_region
```

### 🔴 Must NOT survive Phase 2A

1. **`RfSessionState` + `RfSessionStateMachine`** — this is *precisely* the forbidden target architecture. A complete second copy of the engine, with `validate_rf_*` mirroring `validate_lf_*` line-for-line (visible in the diff: the LF validators are removed and re-added alongside RF twins). Duplicated **A-behavior**.
2. **`rf_reference_span_ticks`** — verbatim:
   ```rust
   fn rf_reference_span_ticks(joint: JointKind) -> u16 {
       match joint {
           JointKind::Hip   => 935,
           JointKind::Upper => 2004,
           JointKind::Lower => 1435,
       }
   }
   ```
   These are **exactly the LF historical spans named in §14**, promoted into an RF acceptance criterion. The accompanying test is explicit: `let reference = rf_reference_span_ticks(JointKind::Hip); // 935, LF V25 hardware-measured`. This is the §14 prohibition realized in code. **It must not be copied forward in any form.**
3. **`rf_contact_witness_accepted` / `RF_WITNESS_TOLERANCE_TICKS`** — attempted specialization of **C-safety** by aliasing the LF witness tolerance onto an unvalidated leg.
4. **`rf_full_joint_corridor` / `rf_parking_corridor` / `rf_passive_corridor` / `rf_participant_corridor`** — four duplicated corridor functions where one spec-parameterized function belongs.

### 🟡 Genuine B-data candidates (correct instinct, wrong mechanism)

- **RF HIP physical ordering** (`HipDown` before `HipUp`, from real M23 encoder-direction evidence). The *fact* that contact order is physically constrained and differs per leg is a real B-data finding. Encoding it as new enum variants is the wrong mechanism; it belongs in the spec's contact ordering.
- **`rf_full_sequence_profile` / `RF_ALLOWED` scoping** — pure B data.

### 🟢 Lessons worth retaining without adopting the engine

1. **Anchor-based second-contact prediction with hard-fail on empty intersection.** `rf_expected_second_contact_region` predicts the second contact from the *first really-measured contact on that same leg* plus a span, then **intersects** with the existing geometric corridor and **never widens it**; an empty intersection is `Err` **before** any forward motion (test: `rf_anchor_region_empty_intersection_fails_closed_before_second_side_forward_movement`). The *structure* — relative-to-own-anchor, intersect-never-widen, fail-closed-before-motion — is architecturally sound and a genuine improvement over a purely static corridor. **The reference span it uses must come from that leg's own geometry, never from LF.**
2. **`position_inside_expected_region` used inside the production predicate**, not duplicated in tests — good discipline worth keeping.
3. **The test comment on `rf_witness_accepts_translated_pair_with_correct_span_not_absolute_ticks`** documents a real insight: comparing *spans* rather than absolute ticks avoids the mirror-symmetry trap. The insight is right; sourcing the span from LF is wrong.

**Authority:** this worktree is never more authoritative than immutable LF V25. It was **read only**; nothing was modified, reset, cleaned, stashed, or committed.

---

## 12. Safety-critical invariants G2/G3 must preserve

### CRITICAL

1. `GoalPosition` is unsigned `0..=4095` on every path; wrap math is local only.
2. Every state can transition to `Cleanup`, and `Cleanup` performs a **verified** global torque-OFF of all 12 motors with fresh readback.
3. `HARD_CURRENT_ABORT_RAW = 200` aborts with immediate global torque-OFF.
4. `EXPECTED_TEMPERATURE_LIMIT_C = 70` exactly; the port-level thermal supervisor is an **independent** second layer.
5. Stale telemetry (`MAX_TELEMETRY_AGE = 3 s`, zero stamp) fails closed.
6. RAM allowlist is exactly `{TorqueEnable, Acc, GoalPosition, GoalSpeed, TorqueLimit}`; no `EepromRegister`, `RamRegister::Lock`, `reg_write`, `action`, `reset*`, `freeze_calibration` in the calibrator path.
7. `port.rs` rejects every non-MATDOG-RAM command while armed; Station remains sole serial owner.
8. Guard = URDF limit + 64 ticks; **never** crossed, by target or by readback.
9. Contact acceptance corridors may widen only *away* from the guard, never toward it.
10. All 12 motors are supervised every frame; non-participants must be torque-OFF and drift ≤ 16 ticks.
11. Exactly one motor advances per state.
12. `stop_pressure` (goal := present position) on contact/stall.
13. Holds require a completed, dwell-verified matching active move.
14. Prerequisite restore is LIFO — rear parking released last.
15. `MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS = 96` bounds any q0 shift from the established digital zero.
16. Isolated HIP profiles remain blocked.
17. EEPROM is reachable only from a separate binary, requiring `--commit LF_FREEZE_COMMIT`, with Station stopped.
18. The external observer must not reconstruct native motion policy.

### HIGH

19. `TORQUE_LIMIT = 500` / `GOAL_SPEED = 160` / `ACCELERATION = 8` motion envelope.
20. Contact requires ≥ 24 ticks travel, 3 persistent samples, low velocity, target ahead, inside corridor.
21. Current is diagnostic, not a contact gate.
22. Coarse scout is discarded as metrology; only the two fine passes produce the contact.
23. Backoff must show current recovery and must not cross home.
24. Uniform q=0 normalization with no distance admission window.
25. Restart-safe entry classification before any motion.
26. Staged q0 uses the **affine** estimate; fixed-scale is diagnostic and must never stage.
27. LF witness ticks/tolerance are LF-only.
28. Operator stop is honoured at every phase boundary and inside every wait loop.

### MEDIUM

29. Distance-scaled motion deadline with 12 s floor.
30. Repeatability spread ≤ 16 ticks between two identical fine passes.
31. Median+MAD baseline with ≥ 6 samples.
32. Fixed-scale endpoint disagreement is **recorded**, never erased (§18).
33. Command IDs scoped and monotonic; results matched exactly.

### LOW

34. Progress/step telemetry to Station (58/20/16).
35. Human-readable evidence lines preserve direction sign, range and endpoint residuals.

---

## 13. Ambiguities / risks / blockers

**AMB-1 — `ContactConfirmed` does not always stop advance. Severity: HIGH.**
*Evidence:* `matdog.rs:4015-4041` and `4123-4139` — on a fine pass, a confirmed contact shallower than the coarse scout by more than `FINE_STEP_TICKS` triggers `continue 'approach_steps` **without** `stop_pressure`. *Impact:* §15 and the handoff state the rule with no exception; the code has a deliberate, evidence-driven, bounded exception. If G3 "preserves LF V25 behavior exactly", it preserves this exception; if G3 implements §15 literally, it **changes hardware-validated behavior**. *May G2 proceed?* **Yes** — G2 is spec design. **G3 must not start until ChatGPT rules on this.**

**AMB-2 — `JOINT_SPECS` makes all four legs geometrically identical. Severity: HIGH.**
*Evidence:* all 12 entries use the same `HIP_MIN/MAX_DELTA`, `UPPER_MIN/MAX_DELTA`, `LOWER_MIN/MAX_DELTA`; only `motor_id` and `direction` vary. Front/hind and left/right distinctions survive **only** in the `prerequisites_for` match arms and `hip_upper_clearance_delta`. *Impact:* §20's "FRONT ≠ HIND" is currently true only in two hard-coded helpers. Any G2 spec that mechanically lifts `JOINT_SPECS` into per-leg data would *encode symmetry as structural runtime truth* — exactly what §20 forbids. *May G2 proceed?* Yes, **provided** the spec models per-leg limits as independent fields even where today's values coincide.

**AMB-3 — three non-identical tick-level LF references. Severity: MEDIUM.**
*Evidence:*

| Joint / side | `lf_reference_contact_ticks` (code, D) | `tools/matdog/README.md` (final evidence) | Δ |
|---|---:|---:|---:|
| Hip MIN | 2535 | 2535 | 0 |
| Hip MAX | 1617 | 1600 | **17** |
| Upper MIN | 1443 | 1439 | 4 |
| Upper MAX | 3442 | 3443 | 1 |
| Lower MIN | 3093 | 3093 | 0 |
| Lower MAX | 1666 | 1658 | 8 |

Plus `MATDOG_LF_V25_HARDWARE_EVIDENCE_2026-08-04.json`, which records only approved **angles** (−42.803/+39.375, −53.525/+122.607, −91.846/+34.277). All deltas are within `LF_CONTACT_WITNESS_TOLERANCE_TICKS = 24`, so nothing currently fails. *Impact:* G4 must prove "offline behavioral preservation against immutable LF V25 source/evidence" — and there are three candidate oracles that are not identical. *May G2 proceed?* Yes. **G4 must not start until ChatGPT declares the authoritative oracle.**

**AMB-4 — `UPPER_30_DELTA / UPPER_85_DELTA / UPPER_90_DELTA` vs the G4-replay prohibition. Severity: MEDIUM.**
*Evidence:* 341 ticks = 29.97°, 967 = 85.0°, 1024 = 90.0°. §16 forbids legacy 30/50/90° replay context becoming runtime truth. These are *prerequisite/parking poses*, not endpoint contexts, and are pinned by CI. No 50° constant exists. *Impact:* they are numerically the same family of round degree values the audit warned about; if Geometry V5 parking data ever supersedes them, they must be replaced by model output, not retained by inertia. *May G2 proceed?* Yes — G2 should model them as spec-supplied parking/prerequisite references with a provenance link, not as engine constants.

**AMB-5 — CI pins exact source lines and test names. Severity: MEDIUM.**
*Evidence:* the calibrator workflow requires the literal strings `accepted: affine.accepted && contact_witness_accepted`, `transition_lf_state(LfSessionState::ReturnHip)`, `MATDOG_LF_PROFILE_V1|joint=`, `MATDOG LF URDF FREEZE GATE: PASS`, plus 10 exact test-function names and 6 exact constant values. *Impact:* G3 cannot be green without a coordinated workflow change; there is a real temptation to loosen the gate rather than migrate it. *May G2 proceed?* Yes. The workflow migration must be a separate reviewed diff with a per-assertion justification.

**AMB-6 — launcher SHA pins guaranteed to break. Severity: LOW.**
`matdog_v42_pinned_launcher.py` pins runner/observer/Station SHA-256. Any G2 edit invalidates them. Correct behavior (refuse to launch), but the update must be deliberate and reviewed, not a mechanical re-hash.

**AMB-7 — `prerequisite_restore_order` is test-only. Severity: LOW.**
CI requires the token; production uses a LIFO `pop()` in `restore_prerequisites`. They agree today but are not the same code path.

**AMB-8 — cross-repo handoff copies differ textually. Severity: LOW.**
Semantically equivalent (§2.5). Worth converging to one canonical text so future G0 runs get a byte-identical check.

**AMB-9 — robot-dog `origin/main` has advanced past the handoff's stated entry SHA. Severity: LOW.**
Handoff says `f07aa094…`; canonical main is `bd5aa8e…` (PR #20, "close Phase 2A0 and align Phase 2A entry"). Consistent forward progress, matching the brief's stated expectation. Not a contradiction.

### Blockers

**None.** No stop condition from §32 was triggered:
LF release unchanged ✓ · canonical LF source unmodified and byte-identical to the release ✓ · RF worktree intact with exactly its expected evidence ✓ · Geometry V5 hashes present and matching ✓ · no per-leg duplicated state machines in canonical main ✓ · no LF spans used as other-leg coordinates in canonical code ✓ · no detector thresholds altered ✓ · no DIAGNOSTIC treated as executable ✓ · no UNRESOLVED treated as PASS ✓ · no G4 replay presented as canonical ✓ · no signed GoalPosition ✓ · EEPROM unreachable from the Phase 2A path ✓ · no hardware/serial activity ✓ · live-FK contradiction reported, not hidden ✓ · FRONT/HIND asymmetry present and flagged ✓ · affine fitting does not erase raw disagreement ✓ · no destructive git activity ✓.

---

## 14. Proposed G2 boundary — DESIGN INPUT ONLY

*No code. No files. Input for architecture review only.*

**`LegCalibrationSpec` — minimal concerns that appear to belong here**

- leg identity + label
- per-joint: `kind`, `joint_name`, `motor_id`, `direction`, `min_delta`, `max_delta` — **as independent per-leg fields**, never shared constants (AMB-2)
- participant allowlist (incl. whether a contralateral parking joint exists — front/hind asymmetric)
- prerequisite pose table per `(joint, side)`, including the 90°/85° hip-clearance distinction, **as data**
- parking reference and its corridor
- contact ordering (joint order, side order, and any physically constrained order such as the RF hip evidence)
- restore plan / staging order
- target-domain status per endpoint: `EXECUTABLE` vs `DIAGNOSTIC` vs `UNRESOLVED` — a **required** field with no default, so the 8 UNRESOLVED targets cannot silently become executable
- geometry provenance block (see below)
- optional, LF-only: hardware-witness reference + tolerance — `Option<…>`, absent for RF/RH/LH, so an unvalidated leg **cannot** inherit an LF witness

**`LegSessionStateMachine` — behavior**

- the state vocabulary and role model (`ActivelyCommanded` / `ActivelyHeld` / `ContactProbe` / `PassiveTorqueOffSafe` / `NonParticipatingTorqueOff`)
- transition legality driven by **spec-supplied ordering**, plus the universal `→ Cleanup` escape as a structural invariant
- per-state required-hold and single-active-motor enforcement, resolved through the spec
- the 6-phase contact-side search unit
- `HybridContactDetector` unchanged, **including an explicit decision on AMB-1**
- corridor construction (guard-anchored, adaptive widening away from guard only)
- backoff / restore / staging / cleanup choreography
- `ModelZeroEstimate` (raw disagreement, preserved) and `AffineJointCalibration` (staging authority) — both, never one

**Global ST3215 safety layer**

- `MATDOG_MOTOR_IDS`, exact-set enforcement, topology dispatch
- unsigned tick arithmetic and the `0..=4095` ceiling
- `TORQUE_LIMIT` / `GOAL_SPEED` / `ACCELERATION` envelope
- `HARD_CURRENT_ABORT_RAW`, thermal contract, both supervision layers
- telemetry freshness and health
- RAM allowlist, the port arming gate, the EEPROM prohibition
- global verified torque-OFF and the abort path
- `MODEL_ZERO_MAX_SHIFT_FROM_DIGITAL_HOME_TICKS` (digital-zero protection)
- operator stop

**Historical LF oracle / regression evidence**

- `lf_reference_contact_ticks` + `LF_CONTACT_WITNESS_TOLERANCE_TICKS`, scoped to the LF spec only
- LF arm tokens, `total_steps = 58`, `bus_serial`
- the ~18 LF_REGRESSION_ORACLE tests, retained verbatim
- the robot-dog approved angles + evidence JSON

**Geometry V5 provenance adapter**

- carries geometry revision, URDF SHA, collision-mesh manifest SHA, V5 endpoint/parking/combined semantic SHAs, calibration-software SHA, hardware-evidence SHA
- **fails closed** on any mismatch, before any spec is accepted
- refuses to expose a non-`EXECUTABLE` target to the engine at all — the boundary should be *type-level*, so a `DIAGNOSTIC`/`UNRESOLVED` endpoint has no path to becoming a `GoalPosition`
- must land **in the same change** as any external spec source, never after it (§9)

---

## 15. Final repository-integrity proof

Re-run read-only after the entire inventory. State is byte-for-byte identical to the start of the session.

```text
robot-dog        HEAD e71876e80c23c370f9fecf36ddf15f152faf5eb3  branch main   status: clean, 0 untracked
norma-core       HEAD f47b1ba579c623139058a8b0118648015739ab10  branch main   status: clean, 0 untracked
RF worktree      HEAD b2f7dac2eab7147917fccdfde702360da82ab7de  branch matdog/rf-calibrator-from-lf-v25
                 status:  M matdog.rs
                          M matdog_test.rs         (unchanged from session start)
geometry-v5 wt   HEAD 2890daf0a8ac6103d3856f208a5f042528fc0da0  branch matdog/geometry-compiler-v5-collision-baseline
                 status: clean, 0 untracked

refs/remotes/origin/main                              (norma-core) 4a8ed6337261553b79c928975808d294c9ca723b
refs/remotes/origin/release/matdog-lf-calibrator-v25  (norma-core) f87dd1fbc7e8100d275c74f9af448642f3429680
refs/heads/matdog/rf-calibrator-from-lf-v25           (norma-core) b2f7dac2eab7147917fccdfde702360da82ab7de
refs/remotes/origin/main                              (robot-dog)  bd5aa8edbd903531885a3439b29a7303009e838b

reflog HEAD@{0}: norma-core "checkout: moving from main to main"  (pre-existing, predates this session)
reflog HEAD@{0}: robot-dog  "merge origin/main: Fast-forward"     (pre-existing, predates this session)
```

```text
robot-dog tracked modifications caused by this session:            NONE
norma-core tracked modifications caused by this session:           NONE
historical RF worktree modifications caused by this session:       NONE
geometry-v5 worktree modifications caused by this session:         NONE
untracked files created in any repository:                         NONE
branches created:                                                  NONE
commits created:                                                   NONE
pushes:                                                            NONE
fetches / prunes / gc / maintenance:                               NONE
index writes:                                                      NONE (every git call used --no-optional-locks)
cargo invocations (test/check/build/fmt):                          NONE
hardware / serial actions:                                         NONE
processes started, stopped or signalled:                           NONE
EEPROM / servo / Station state changes:                            NONE
```

**Full disclosure of every write performed during this session (all outside both repositories):**

1. `/tmp/claude-1000/-home-matteo-manicardi-MATDOG/…/scratchpad/handoff_normacore.md`
2. `/tmp/claude-1000/-home-matteo-manicardi-MATDOG/…/scratchpad/handoff_robotdog.md`
   — two `git show` blob dumps written to the session scratchpad solely to `diff` the cross-repository handoff copies (§2.5).
3. `/home/matteo-manicardi/.claude/plans/matdog-phase-2a-synthetic-badger.md`
   — the plan-mode artifact the harness requires; records that no implementation is authorized.

None of these is inside `robot-dog`, `norma-core`, or any worktree. No repository file, index, ref, config or generated artifact was created, modified or deleted.

---

## EXECUTOR_ASSESSMENT

```text
READY_FOR_CHATGPT_REVIEW
```

G0 is coherent with no BLOCKER; G1 is complete. `P2A-G1` is **not** declared closed — that is ChatGPT's and Matteo's call.

Three items should be ruled on before the corresponding gates open: **AMB-1** before G3, **AMB-2** during G2 spec design, **AMB-3** before G4.

**STOP.** No G2 work, no `LegCalibrationSpec`, no branch, no worktree, no refactor, no test edit, no PR — awaiting explicit post-review authorization.
