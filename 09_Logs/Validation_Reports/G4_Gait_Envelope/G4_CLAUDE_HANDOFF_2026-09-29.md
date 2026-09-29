# G4 Claude handoff — 2026-09-29

**G4 IS NOT COMPLETE.** This is an INCOMPLETE_CHECKPOINT requested by the user because quota is low. Do not restart the research or treat the checkpoint as final acceptance. No new research or large validation was launched after the checkpoint request.

## 1. Repository, commits and preservation

- Worktree: `/home/matteo-manicardi/MATDOG/worktrees/robot-dog-gait-engine`
- Branch: `feat/gait-engine-offline-v1`
- Accepted G4 baseline: `c4befbe90b3121d60ba1b9ba09dc1082364c8d88`
- Code/artifact checkpoint SHA: `aa621bff14ff16bab428d5eeec58787677e8224d`.
- This handoff and its JSON are committed in a following documentation commit. Its self-referential SHA cannot be embedded in its own content; obtain final handoff HEAD with `git log -1 --format=%H -- 09_Logs/Validation_Reports/G4_Gait_Envelope/G4_CLAUDE_HANDOFF_2026-09-29.md`. No amend/rewrite is needed.
- No live G4 survey/refinement/oracle/lifecycle jobs remained at checkpoint inspection. All persisted JSON files parse, and both full case files have `complete: true` with planned counts met.
- The original G4 request, including all 27 final questions, is copied verbatim to `G4_ORIGINAL_REQUEST.txt`. The superseding checkpoint instruction is `G4_CHECKPOINT_REQUEST.txt`. Those requests supersede the old G3.5 resumes.

Local G4 commits before the handoff carrier:

```text
71b777ba439c768b530c0a05a090b2386cff6051 feat(motion): add normalized Cartesian gait and gated locomotion lifecycle
06a7b855dd1fd4a92f32836efec2f899b1acb61f fix(motion): isolate gait lifecycle from immutable G3 gate contract
aa621bff14ff16bab428d5eeec58787677e8224d G4: checkpoint gait audit tooling and completed survey evidence
```

The first commit temporarily appended MotionState values. The second restored the accepted G3 files byte-for-byte and isolated the coordinator state in `LocomotionState`; this is intentional preserved history. No accepted G1/G2/G3/G3.5 commit was rewritten.

Complete added/modified file list through the code/artifact checkpoint:

```text
M	05_Firmware/MATDOG_Controller/scripts/tests/run_motion_host_tests.sh
A	05_Firmware/MATDOG_Controller/scripts/tests/test_gait.cpp
M	05_Firmware/MATDOG_Controller/scripts/tests/test_motion_oracle.py
A	05_Firmware/MATDOG_Controller/src/motion/Gait.cpp
A	05_Firmware/MATDOG_Controller/src/motion/Gait.h
A	05_Firmware/MATDOG_Controller/src/motion/Locomotion.cpp
A	05_Firmware/MATDOG_Controller/src/motion/Locomotion.h
A	06_Software/Matdog_Core/gait_audit/README.md
A	06_Software/Matdog_Core/gait_audit/bridge.cpp
A	06_Software/Matdog_Core/gait_audit/core.py
A	06_Software/Matdog_Core/gait_audit/lifecycle_audit.py
A	06_Software/Matdog_Core/gait_audit/oracle.py
A	06_Software/Matdog_Core/gait_audit/refine.py
A	06_Software/Matdog_Core/gait_audit/render.py
A	06_Software/Matdog_Core/gait_audit/survey.py
A	06_Software/Matdog_Core/gait_audit/xgo_evidence.py
A	09_Logs/Validation_Reports/G4_Gait_Envelope/G4_CHECKPOINT_REQUEST.txt
A	09_Logs/Validation_Reports/G4_Gait_Envelope/G4_ORIGINAL_REQUEST.txt
A	09_Logs/Validation_Reports/G4_Gait_Envelope/checkpoint_logs/matdog-g4-full.log
A	09_Logs/Validation_Reports/G4_Gait_Envelope/checkpoint_logs/matdog-g4-lifecycle.log
A	09_Logs/Validation_Reports/G4_Gait_Envelope/checkpoint_logs/matdog-g4-manifest.log
A	09_Logs/Validation_Reports/G4_Gait_Envelope/checkpoint_logs/matdog-g4-oracle.log
A	09_Logs/Validation_Reports/G4_Gait_Envelope/checkpoint_logs/matdog-g4-pose-regression.log
A	09_Logs/Validation_Reports/G4_Gait_Envelope/checkpoint_logs/matdog-g4-refine.log
A	09_Logs/Validation_Reports/G4_Gait_Envelope/checkpoint_logs/matdog-g4-sanitizer.log
A	09_Logs/Validation_Reports/G4_Gait_Envelope/checkpoint_logs/matdog-g4-screen.log
A	09_Logs/Validation_Reports/G4_Gait_Envelope/checkpoint_logs/matdog-g4-static.log
A	09_Logs/Validation_Reports/G4_Gait_Envelope/checkpoint_logs/targeted_checkpoint_validation.log
A	09_Logs/Validation_Reports/G4_Gait_Envelope/definitions.json
A	09_Logs/Validation_Reports/G4_Gait_Envelope/envelope_grid.csv
A	09_Logs/Validation_Reports/G4_Gait_Envelope/full_cases.json
A	09_Logs/Validation_Reports/G4_Gait_Envelope/lifecycle_audit.json
A	09_Logs/Validation_Reports/G4_Gait_Envelope/oracle_results.json
A	09_Logs/Validation_Reports/G4_Gait_Envelope/refinement_full.json
A	09_Logs/Validation_Reports/G4_Gait_Envelope/refinement_screen.json
A	09_Logs/Validation_Reports/G4_Gait_Envelope/screen.json
A	09_Logs/Validation_Reports/G4_Gait_Envelope/xgo_architecture.json
```

This final handoff commit additionally adds this Markdown file, `handoff_state.json`, and `checkpoint_manifest.json`. Final known status is checked after that commit.

## 2. Implemented architecture and exact gait definitions

**VERIFIED_FROM_SOURCE + host/oracle evidence**, not physical validation:

- `Gait.h/.cpp`: fixed-size, deterministic C++17, namespace `matdog::motion`, no normal-operation allocation, exceptions, RTTI, mutable globals or framework/device dependency.
- Canonical joint/leg order is **LF RF RH LH**, hip/upper/lower URDF radians.
- Unwrapped cycle count `s` determines world anchors; fractional phase is `fract(s+offset)`. Finite nonnegative cycles are bounded to 1,000,000 as a numerical/resource guard. Near-equality boundaries are restored within eight input ULPs, not a physical time tolerance.
- WALK offsets `[0, .5, .75, .25]`; swing order **RH, RF, LH, LF**. Duty beta is `[.75,1)`, local phase `< beta` is stance; otherwise swing. At most one leg swings. Open-loop X sway is allowed only for beta > .75, and is parameterized independently.
- TROT offsets `[0,.5,0,.5]`, pairing **LF+RH** and **RF+LH**. Duty is `[.5,1)`. TROT dynamic stability is always **NOT YET PROVEN**.
- Duty values screened: WALK `.75,.8,.9`; TROT `.5,.6,.8`. WALK beta .75 uses zero sway. Other WALK screen defaults use 4 mm X sway. These are study choices, not approved operation parameters.
- Geometric parameters: body height, lift, body X/Y displacement per cycle, yaw/cycle, duty, X sway, numerical Jacobian conditioning threshold. Period is separate. Default values are illustrative, not approved.
- Stance uses immutable **world G2 physical-contact-reference anchors**, not foot_link origins. Initial anchors are canonical C4 contacts. Subsequent touchdown anchors use the nominal SE(2) body transform at touchdown time plus beta/2, applied to canonical XY foot positions.
- Swing parameter `u=(local_phase-beta)/(1-beta)`. XY interpolation uses `10u^3-15u^4+6u^5`; Z is `64*h*u^3*(1-u)^3`. Both endpoints have zero velocity and acceleration. Exact lift/touch boundaries permit ground contact; interior swing contact is prohibited by the offline policy.
- Body is level (roll=pitch=0), flat +Z ground only. Constant per-cycle planar twist is integrated with the SE(2) exponential and stable small-yaw series. Turning comes from rigid transforms, never per-leg joint offsets.
- WALK X sway alternates positive/negative plateaus in quarter cycles; quintic changes occur during all-stance gaps of length beta-.75. This is an open-loop geometric study, not stabilization.
- `solveGait` uses G2 contact IK, previous valid q and branch identifiers, exact eccentric geometry, URDF joint limits and analytic contact Jacobian/Hessian derivatives. It reports infinity-norm Jacobian condition. qdot scales 1/T and qddot 1/T². No actuator limit comparison exists.
- C++ returns kinematic candidates; `assessGait` is the explicit external collision/contact/support assessment boundary. The core does not secretly implement mesh collision checks.

## 3. Lifecycle and watchdog

`Locomotion` owns the unchanged `StandTransition`; only a successfully completed G3 acquisition/stand path establishes STAND. It exposes separate `LocomotionState` values OFF, IDLE, STAND_TRANSITION, STAND, STOPPING, GAIT_START, WALK, TROT. REST_GROUND is not a startup path. No public completion event can forge STAND.

Start: one period of four-contact preparation from canonical 150 mm STAND to chosen height and initial sway; two periods of quintic phase-speed acceleration, covering one geometric cycle; then steady WALK/TROT. Stop: finish the current cycle; decelerate through one final planned cycle over two periods; place future touchdowns at canonical terminal contacts; recenter height/sway over one more period with contacts locked. Never retarget an in-flight swing. A stop during preparation returns via four-contact hold. Restart retains the terminal world transform. DISABLE/FAULT cancels target generation, with no physical braking claim.

Watchdog uses caller time only. Finite nonnegative monotonic `now`, non-future stamps, strictly increasing sequence and nondecreasing stamps are required. Age equal to timeout is fresh; greater is stale. Invalid, stale, zero, or mode/parameter/period-changing command requests semantic stop; a changed command is not applied mid-cycle. Explicit restart from STAND is needed. Backward/nonfinite sample time fails explicitly.

Exact `GaitStatus` order: `OK, INVALID_PARAMETER, NONFINITE, IK_UNREACHABLE, JOINT_LIMIT, CONTACT_INVALID, BRANCH_CHANGE, ILL_CONDITIONED, COLLISION, SUPPORT_INVALID, STATE_ERROR, CANCELLED`.
Python classification names include `KINEMATICALLY_VALID`, `JOINT_LIMIT_BLOCKED`, `NUMERICALLY_ILL_CONDITIONED`, `GROUND_COLLISION`, `SELF_COLLISION`, and `CONTACT_INVALID`. Screen failures can have more than one category. A failure never enables target.valid.

## 4. XGO evidence — already inspected, do not redo

`xgo_architecture.json` indexes 40 pinned files with hashes. Read-only archive `/tmp/matdog-g35-xgo` is origin/main `a1b34a8594e5bc76c76b1e3ddf89a3aef2b98298`. Original checkout `/home/matteo-manicardi/robotics-reverse/xgolite-low-level-reconstruction` had unrelated local untracked `analysis/`, `references/intake/`, `tools/` and was left untouched.

Inspected G/G2/G2.1 scheduler, phase/leg maps, writer formulas/continuity, commands, Cartesian interpolation, body-before-IK, runtime tick/task binding, G2.1 semantic reconciliation and H2 transfer boundary. G2 stronger evidence supersedes G historical address/timing uncertainties. Requested 2 ms delay is proven; actual cadence/jitter are not. Host gait names are documentary VERIFIED; full physical mode/leg binding remains CORROBORATED. Units/signs/zeros remain unbound.

Transferred concepts: command/state separation, phase slots and offsets, Cartesian construction, body transform before IK, explicit lifecycle. MATDOG differs intentionally: elapsed-time normalized modulo preserving overshoot, C2 swing and timed joins, independent freshness contract. XGO's local zero-command phase reset and smoothing do not prove the complete MATDOG stopping contract. No bound XGO locomotion watchdog proof was found; host read timeouts are not one.

No XGO geometry, angles, physical zeros/signs, limits, stride, lift, duty, period/frequency, gains, body height, velocity/acceleration limits or IMU tuning was copied. No 1.5x conversion rule. No XGO firmware executed.

## 5. Oracle and targeted validation

**VERIFIED_FROM_ARTIFACT** `oracle_results.json`:

```json
{
  "status": "PASS",
  "lifecycle": {
    "max_qdot_error": 2.7869795195256675e-07,
    "max_qddot_error": 1.809259968050484e-05,
    "frames": 46
  },
  "sample_count": 3872,
  "numerical_ik_legs": 16,
  "metrics": {
    "max_body": 5.5077470362263625e-17,
    "max_target_m": 5.551115123125783e-17,
    "max_urdf_contact_m": 1.1959531388233124e-16,
    "max_qdot_error": 1.9325782263379665e-07,
    "max_qddot_error": 6.76232944840649e-06,
    "max_numeric_ik_rad": 1.5987211554602254e-14,
    "max_periodic_q_rad": 2.7755575615628914e-15,
    "max_world_stance_drift_m": 1.689731085869606e-16
  },
  "scope": "Kinematic oracle. Mesh/support validity is a separate survey gate."
}
```

The oracle uses independent matrix-exponential SE(2), polynomial evaluation, canonical URDF FK, numerical contact IK and finite differences. C2 lifecycle joins have jerk discontinuities, so the oracle uses Richardson extrapolation to remove the O(h) acceleration-difference bias. It also checks deterministic reruns, periodic q, signed planar commands, and T vs 2T scaling. Final aggregate 0.5/1/2/4-second derivative tables remain pending; `definitions.json` lists intended periods but the saved oracle explicitly validates 1 vs 2 seconds.

Current checkpoint quick build: **148,291 checks, zero failures**, strict C++17 flags, `-fno-exceptions -fno-rtti`. `git diff --check` passes. G3.5 artifact manifest passes for 80 artifacts with one historical generator revision. Existing G3 startup, StandTransition and MotionState files have no net difference from accepted baseline.

Saved logs are under `checkpoint_logs/`:

- `matdog-g4-static.log`: completed **STATIC_AUDIT PASS**, 148 source files. Static audit invokes the full existing host runner including G1/G2/G3 and G4. Its log has no source SHA; it began before final small G4 guard additions. Treat final-current-source static validation as pending.
- `matdog-g4-pose-regression.log`: **28 tests PASS** after restoring unchanged G3 state files.
- `matdog-g4-sanitizer.log`: earlier ASan+UBSan test passed with **147,606 checks**. The final 148,291-check code still needs its final sanitizer run.
- `matdog-g4-oracle.log`: latest completed independent oracle pass, matching the saved oracle JSON. Later additions were input guards; final source-pinned revalidation remains pending.
- `matdog-g4-manifest.log` is a preserved HISTORICAL failure from the temporary MotionState edit; it is superseded by the current manifest PASS in `targeted_checkpoint_validation.log`. Do not mistake the historical failure for unresolved G3 regression.
- The current short validation is explicitly recorded in `targeted_checkpoint_validation.log`.

## 6. Completed surveys and exact scope

**VERIFIED_FROM_ARTIFACT**, all previously running jobs finished:

| Artifact | Completed cases | Scope |
|---|---:|---|
| screen.json / envelope_grid.csv | 2044 | coarse screen, no self-collision check, early exit |
| refinement_screen.json | 308 | finer axial, pure lateral/yaw and stress screen, early exit |
| full_cases.json | 16 / 16 planned | full mesh, all saved cycle samples |
| refinement_full.json | 10 / 10 planned | full mesh, all saved cycle samples |
| lifecycle_audit.json | 2 × 141 frames | full mesh STAND→gait→STAND traces |

Broad screen passing count: 344. Refined screen passing count: 46. Full sampled steady cases passing: 19 of 26. "Screen passing" must never be promoted to full mesh passing. Failed screen metrics only cover the prefix before first failure. Full-cycle metrics cover every saved sample.

Full validation uses unchanged G3.5 model: all 17 canonical meshes, all 120 nonadjacent pairs, FCL triangle collision/distance plus solid-containment checks. Sixteen directly adjacent assembly interfaces remain excluded. Ground/contact tolerance is 1 µm; declared support patch band is 10 µm. CAD/URDF aggregate COM uses all inertials. WALK requires strictly positive support margin. TROT static support results remain recorded diagnostics, with dynamics unproven.

Primary study heights: 80,100,120,140,150 mm. Additional stress heights from refinement: 60, 160, 180, 200, 210, 220, 240, 300 mm. No engineering minimum height is established; no micrometre-height gait was used.

Largest/smallest currently **full sampled passing** body displacement per cycle, across the tested duty/lift combinations (not a continuous envelope; see IDs for full parameters):

| Type | Height mm | Min X mm/cycle | Max X mm/cycle | Passing IDs |
|---|---:|---:|---:|---|
| WALK | 80 | -10.0 | 15.0 | 18, 3012, 4000 |
| WALK | 100 | -12.5 | 10.0 | 222, 287, 3032 |
| WALK | 120 | -15.0 | 20.0 | 416, 426, 3053 |
| WALK | 140 | none | none | none |
| WALK | 150 | none | none | none |
| TROT | 80 | -50.0 | 50.0 | 1040, 1046, 3069, 3084 |
| TROT | 100 | -60.0 | 60.0 | 1230, 1238, 3092, 3105 |
| TROT | 120 | -30.0 | 30.0 | 3116, 3125 |
| TROT | 140 | none | none | none |
| TROT | 150 | none | none | none |

Lateral and yaw: implemented kinematically and tested with both signs and combined commands. The broad and refinement grids did not establish a nonzero full-mesh-valid lateral/yaw operating interval. Contact tilt/finite-mesh ground intersection invalidates many such screen cases. Do not call this proof that physical turning/lateral locomotion is impossible.

Some coarse-screen candidates fail denser validation: e.g. WALK id34 (+20 mm, 80 mm body, duty .9) fails support; WALK id226 (+40 mm, 100 mm body) fails ground policy; TROT id1311 (100 mm body, +10 mm, 10 mm lift, duty .6) fails contact policy at 201 samples. Do not use that TROT record as a validated representative.

## 7. Best current cases, margins and derivatives

Useful dense WALK representative: **id287**, 100 mm body, +10 mm/cycle, 10 mm lift, duty .8, 4 mm X sway, period 1 s, 201 cycle frames, full sampled PASS. Other full passing records and parameters are machine-readable in `handoff_state.json.full_case_summary`.

TROT has several full sampled passing 81-frame sets (e.g. ids1046,1238,3084,3105); they remain dynamically uncertified. Selection and denser validation of a final representative are **INCOMPLETE**. The current `render.py` hardcodes id1311's failed dense TROT case by parameter matching, so it MUST be changed or explicitly labeled as a failed candidate before presenting "validated" visuals. Rendering has not been run.

Aggregate minima/maxima over currently full passing sampled steady cases (**VERIFIED_FROM_ARTIFACT**):

```json
{
  "min_joint_margin_rad": 0.3673548117577856,
  "min_self_separation_m": 0.013787832260131826,
  "min_nonfoot_ground_m": 0.0024115076710135436,
  "max_stance_drift_m": 1.3878871967638603e-16,
  "max_condition": 5.092195667381843,
  "min_walk_support_margin_m": 0.00012042026361750412
}
```

No selected IK branch changes are recorded in the full passing cases. No nonadjacent self-collision was detected in those cases. These statements do not cover unsampled motion or excluded adjacent assembly interfaces.

Per-joint peak qdot/qddot at period 1 s, canonical LF/RF/RH/LH hip/upper/lower order:

- id287: parameters `{"type": 0.0, "height_m": 0.1, "lift_m": 0.01, "duty": 0.8, "advance_x_m": 0.01, "advance_y_m": 0.0, "yaw_rad": 0.0, "sway_x_m": 0.004, "max_condition": 10000.0}`
  - peak qdot rad/s: `[2.6335182661875036e-16, 2.710926095877125, 2.3152164429215203, 2.1172721610187465e-16, 2.710926095877125, 2.3152164429215203, 1.629810745562208e-16, 2.137520694942177, 2.1674338617032394, 0.0, 2.137520694942176, 2.1674338617032385]`
  - peak qddot rad/s²: `[8.251972148391522e-15, 163.33777964675392, 75.54790315260081, 5.806886548138435e-15, 163.33777964675392, 75.54790315260081, 9.62575291931403e-15, 124.5199002623875, 72.93390759951967, 0.0, 124.51990026238747, 72.93390759951967]`

- id1238: parameters `{"type": 1.0, "height_m": 0.1, "lift_m": 0.005, "duty": 0.5, "advance_x_m": 0.06, "advance_y_m": 0.0, "yaw_rad": 0.0, "sway_x_m": 0.0, "max_condition": 10000.0}`
  - peak qdot rad/s: `[4.6377113888276966e-17, 1.5355478037543115, 0.7612486631046379, 6.162975822039155e-33, 1.5355478037543093, 0.7612486631046368, 3.033808582304675e-17, 1.2759940002818506, 0.9101196037839585, 6.087581834357677e-17, 1.2759940002818493, 0.9101196037839585]`
  - peak qddot rad/s²: `[3.543619690603408e-16, 14.517961119138887, 9.11916763327716, 1.8488927466117464e-32, 14.517961119138874, 9.119167633277149, 6.786174508667584e-16, 13.609203056195094, 10.570214490546716, 6.559406901219926e-16, 13.609203056195092, 10.570214490546714]`

- id3105: parameters `{"type": 1.0, "height_m": 0.1, "lift_m": 0.005, "duty": 0.6, "advance_x_m": 0.045, "advance_y_m": 0.0, "yaw_rad": 0.0, "sway_x_m": 0.0, "max_condition": 10000.0}`
  - peak qdot rad/s: `[3.27306206818894e-17, 1.5684795123058708, 0.8524301683441815, 2.1362789858726928e-17, 1.5684795123058697, 0.8524301683441813, 6.899754499207233e-17, 1.3308477552250746, 0.9857042915527747, 7.692209626782708e-17, 1.3308477552250746, 0.9857042915527743]`
  - peak qddot rad/s²: `[9.136379255011065e-16, 17.64730502380612, 12.539889306586272, 8.941324067019127e-16, 17.64730502380611, 12.539889306586266, 1.0603397669793102e-15, 16.834810351873635, 13.501913567193967, 9.750426384722479e-16, 16.83481035187364, 13.501913567193967]`

These are semantic requirements only. q remains unchanged, qdot scales 1/T, qddot 1/T² under the tested time law. No actuator-rate, torque or physical safety conclusion follows. The conditioning threshold is a numerical rejection policy, not a hardware limit.

## 8. Critical unresolved lifecycle/contact issue

**VERIFIED_FROM_ARTIFACT:** kinematic start/stop joins are continuous and end in canonical q with zero qdot/qddot, but neither saved complete 100 mm study-height lifecycle passes the full mesh/contact policy. Keep steady-cycle and lifecycle classifications separate. No autonomous physical walking route is validated.

G2 locks the canonical eccentric-cylinder contact reference at world Z=0. The canonical tessellated foot mesh can protrude roughly a micrometre below that analytic surface at certain orientations. Holding the reference fixed therefore does not guarantee the lowest triangle is above the unchanged -1 µm threshold. Near a slow final touchdown, an interior swing can also enter the <=1 µm undeclared-contact band before the exact contact-mode boundary. This is not resolved by successful IK, zero anchor drift, or the 10 µm declared support patch band.

Do NOT widen tolerances, move targets, collapse contact into foot_link, change G2 semantics or invent physical compliance. Evaluate the evidence and explicit contact-mode contract before deciding any later resolution. The current checkpoint only records rejection.

Exact saved failures (time in seconds, period 1 s, stop request 3.2 s, terminal stand 7 s):

### WALK lifecycle

5 failing frames of 141. Worst mesh-ground Z: -1.21632700582e-06 m.

- t=0.15: GROUND_PENETRATION:rh_foot_link, GROUND_PENETRATION:lh_foot_link
- t=0.25: GROUND_PENETRATION:rh_foot_link, GROUND_PENETRATION:lh_foot_link
- t=4.25: UNDECLARED_GROUND_CONTACT:rh_foot_link
- t=6.75: GROUND_PENETRATION:rh_foot_link, GROUND_PENETRATION:lh_foot_link
- t=6.85: GROUND_PENETRATION:rh_foot_link, GROUND_PENETRATION:lh_foot_link

### TROT lifecycle

13 failing frames of 141. Worst mesh-ground Z: -1.18300045751e-06 m.

- t=0.15: GROUND_PENETRATION:rh_foot_link, GROUND_PENETRATION:lh_foot_link
- t=0.2: GROUND_PENETRATION:rh_foot_link, GROUND_PENETRATION:lh_foot_link
- t=0.25: GROUND_PENETRATION:rh_foot_link, GROUND_PENETRATION:lh_foot_link
- t=5.65: UNDECLARED_GROUND_CONTACT:rh_foot_link
- t=5.7: UNDECLARED_GROUND_CONTACT:rh_foot_link
- t=5.75: UNDECLARED_GROUND_CONTACT:rh_foot_link
- t=5.8: UNDECLARED_GROUND_CONTACT:rh_foot_link
- t=5.85: UNDECLARED_GROUND_CONTACT:rh_foot_link
- t=5.9: UNDECLARED_GROUND_CONTACT:rh_foot_link
- t=5.95: UNDECLARED_GROUND_CONTACT:rh_foot_link
- t=6.75: GROUND_PENETRATION:rh_foot_link, GROUND_PENETRATION:lh_foot_link
- t=6.8: GROUND_PENETRATION:rh_foot_link, GROUND_PENETRATION:lh_foot_link
- t=6.85: GROUND_PENETRATION:rh_foot_link, GROUND_PENETRATION:lh_foot_link

## 9. Remaining acceptance work and provenance

**INCOMPLETE**, not optional:

- Finalize WALK and TROT sampled envelopes without claiming a continuous or physical envelope
- Choose FULL_MESH_VALIDATED representative parameter sets; current dense TROT representative fails
- Separate steady-cycle acceptance from complete STAND->gait->STAND acceptance
- Resolve or explicitly retain strict contact/mesh lifecycle failures without changing G2/tolerance arbitrarily
- Complete support/COM and per-joint derivative reports for chosen sets and periods
- Generate and inspect deterministic canonical mesh side/top/contact-phase visuals and cycle CSVs
- Complete post-implementation XGO comparison and all 27 original final questions
- Add final offline artifact tests/provenance manifest and review code/API
- Run final current-source regressions/static audit/sanitizers and clean local commits

Also answer explicit follow-ups: which representatives are FULL_MESH_VALIDATED; whether the entire STAND→WALK/TROT→STAND path passes; which failures are only near-boundary tolerance/contact-mode failures; what is still numerical research rather than a physically approved parameter. The original 27 questions remain required in the final G4 report. No final G4 REPORT.md or final validation_results.json has been created.

The saved `definitions.json` includes source hashes captured during the full run. Some tools/core input guards were edited afterward. Exact current mismatches are recorded in `handoff_state.json.definition_provenance_mismatches`; do not silently relabel old artifacts as generated by later code. Preserve original evidence and add explicit revalidation if required. A final source/artifact manifest and tests are still needed. `checkpoint_manifest.json` is only an inventory of bytes at this handoff, not a claim that the milestone is accepted.

An ad-hoc cheap 200-interval TROT height/lift probe had no saved authoritative output and its process handle expired. Do not infer its result. It launched before the checkpoint request; no job remains. It need not be rediscovered unless useful for selecting the final representative.

## 10. Next commands, in order

Begin by reading preserved results. The long commands below are for Claude's continuation after review, not actions performed during this checkpoint. Do not rerun expensive completed searches merely for completeness.

```sh
cd /home/matteo-manicardi/MATDOG/worktrees/robot-dog-gait-engine
git status --short --branch; git rev-parse HEAD; git log -6 --oneline
cat 09_Logs/Validation_Reports/G4_Gait_Envelope/G4_CLAUDE_HANDOFF_2026-09-29.md
cat 09_Logs/Validation_Reports/G4_Gait_Envelope/G4_ORIGINAL_REQUEST.txt
/tmp/matdog-g35-env/bin/python 06_Software/Matdog_Core/pose_audit/artifact_manifest.py --check
g++ -std=c++17 -O1 -Wall -Wextra -Werror -fno-exceptions -fno-rtti 05_Firmware/MATDOG_Controller/scripts/tests/test_gait.cpp 05_Firmware/MATDOG_Controller/src/motion/*.cpp -o /tmp/matdog-g4-test && /tmp/matdog-g4-test
# Review completed artifacts, select validated representatives, and correct/label render selection before rendering.
OPENBLAS_NUM_THREADS=1 /tmp/matdog-g35-env/bin/python 06_Software/Matdog_Core/gait_audit/render.py
# After final implementation/acceptance edits, run the final gates below (not during this checkpoint).
OPENBLAS_NUM_THREADS=1 /tmp/matdog-g35-env/bin/python 06_Software/Matdog_Core/gait_audit/oracle.py
OPENBLAS_NUM_THREADS=1 /tmp/matdog-g35-env/bin/python 06_Software/Matdog_Core/pose_audit/test_pose_audit.py
PATH=/tmp/matdog-g35-env/bin:$PATH python3 05_Firmware/MATDOG_Controller/scripts/static_audit.py
g++ -std=c++17 -g -O1 -Wall -Wextra -Werror -fno-exceptions -fno-rtti -fsanitize=address,undefined -fno-omit-frame-pointer 05_Firmware/MATDOG_Controller/scripts/tests/test_gait.cpp 05_Firmware/MATDOG_Controller/src/motion/*.cpp -o /tmp/matdog-g4-sanitizer
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1 /tmp/matdog-g4-sanitizer
git diff --check; git status --short --branch
```

Do not rerun `survey.py --stage screen`, `survey.py --stage full`, `refine.py` or `lifecycle_audit.py` unless source changes actually invalidate their saved results or a specific acceptance gap requires new samples. Do not repeat XGO reverse engineering. Rendering is pending and has the representative-selection warning above. Final tests should be performed after the remaining code/acceptance decisions, not repeatedly during review.

Environment: `/tmp/matdog-g35-env` with pinned G3.5 dependencies; `OPENBLAS_NUM_THREADS=1` used for most analyses. Host bridge builds content-addressed disposable shared libraries under `/tmp/matdog-g4-*.so`. No on-device execution exists.

## 11. Safety and handoff status

**G4 IS NOT COMPLETE.** All valid partial work, completed survey outputs, failures and logs are preserved. No reset, clean, discard, amend, rebase, push, merge or cherry-pick occurred. No hardware/serial/ServoBus/ST3215/torque/flashing/EEPROM/raw-tick/q0/motorDirection/calibration operation occurred. The separate calibration branch/worktree and original XGO checkout were not modified. G3 startup remains active; BODY_ONLY REST_GROUND→autonomous STAND remains unvalidated.
