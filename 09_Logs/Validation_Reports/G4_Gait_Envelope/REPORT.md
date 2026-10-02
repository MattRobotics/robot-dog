# G4 — MATDOG normalized gait scheduler, Cartesian WALK/TROT and offline gait envelope

**Scope: OFFLINE kinematic / geometric study. No hardware, serial device, actuator bus, servo command, flashing, persistent-memory write or calibration change was used or created.**
Baseline: `c4befbe90b3121d60ba1b9ba09dc1082364c8d88` (G3.5.1). Checkpoint: `aa621bf`/`b06558f`. This report completes the checkpoint; it does not restart it. All numbers below are read from the saved artifacts by `build_report.py`.

## 0. Verdict

* **Implemented and independently checked:** a pure C++17 normalized gait core (WALK, TROT, world-locked stance, C2 Cartesian swing, SE(2) body motion, semantic q/qdot/qddot, gated start/stop lifecycle, caller-time command watchdog, explicit failure statuses), an independent Python oracle, and an offline full-mesh/contact/support audit. TROT is **KINEMATICALLY STUDIED, DYNAMICALLY UNCERTIFIED**.
* **Review gate:** the checkpoint's 19-of-26 count and all six aggregates are reproduced by an independent per-frame predicate (section 2). Per-case `complete` is `not errors`, which here coincides with that predicate; the top-level `complete` only means "all planned cases were evaluated".
* **Steady-state:** under the unchanged strict policy 19 of the 26 saved cases pass at their saved sampling (WALK 9/12, TROT 10/14). That tier is **sampling-density sensitive**: the strict policy is a point test on a finite sample, and the fraction of those passes that survive denser sampling falls from 19 to 2 (section 5). Representatives therefore carry additional gates (section 4).
* **Lifecycle:** there is **no** complete FULL-MESH-valid STAND → WALK → STAND or STAND → TROT → STAND. The strict failures are 1.0–1.7 micrometres. They are produced by a deterministic 5.8 micrometre offset between the G2 analytic foot-contact cylinder and the canonical foot mesh, which no continuous start/stop can avoid under the strict policy, plus a sampling-sensitive contact-transition band (section 6). Rejection is **kept**; no tolerance, target, G2 semantic or compliance was changed.
* **Envelope:** reported as **maximum tested passing** values on finite grids, in separate tiers (strict, strict-density-robust, diagnostic). Not a physical maximum, global limit or continuous valid box.
* **Nothing in G4 is approved for the physical robot.** All gait parameters are study values. Semantic derivatives are not compared with any actuator limit.

## 1. What changed after the checkpoint (new files only; preserved raw evidence is byte-identical)

The checkpoint artifacts (`full_cases.json`, `refinement_full.json`, `screen.json`, `refinement_screen.json`, `envelope_grid.csv`, `lifecycle_audit.json`, `oracle_results.json`, `definitions.json`, `xgo_architecture.json`, logs, handoff) are unchanged; `test_gait_audit.py` and `artifact_manifest.py --check` verify their hashes against `checkpoint_manifest.json`. The C++ gait/locomotion sources are unchanged from the checkpoint. `core.py` gained an atomic library build (a concurrent-worker race loaded a partial `.so` once), `oracle.py` an `--output` option (the final run is `oracle_results_final.json`), and `render.py` now refuses any case that is not an audited representative. See `gait_audit/README.md` for the tool table.

`definitions.json` recorded a Gait.cpp hash older than the current file (input guards were added after the full run). `revalidate_saved.py` recomputes all 35 saved full-mesh cases with the current core: the largest difference in q, qdot, qddot or body pose is **0.0** (bit-identical), so the saved evidence did not need regeneration.

## 2. Review gate: what "complete" meant, and the aggregates

`survey.py` sets per-case `complete = not errors` where `errors` is the union of classified per-frame failures (for TROT the diagnostic-only SUPPORT_INVALID is dropped). The top-level `complete` of `full_cases.json`/`refinement_full.json` only states that the planned number of cases was evaluated. `audit_gate.py` therefore does not use either flag: a case passes only if every one of these holds on its saved frames — full-mesh record of the right length; classification KINEMATICALLY_VALID with no first failure; no frame error (TROT: only the diagnostic support flag tolerated); joint margin > 0, condition <= limit, contact residual <= 1e-9 m; no IK branch change; WALK support margin > 0; non-foot ground clearance > 1 um and nonadjacent separation > 0; foot mesh never below -1 um and a swing foot away from its boundary above +1 um; stance contact drift <= 1e-9 m. Result: **19 of 26 pass** (WALK 9/12, TROT 10/14). The 7 failing cases are 34 (WALK), 226 (WALK), 925 (WALK), 1311 (TROT), 1435 (TROT), 1441 (TROT), 1948 (TROT). No case disagrees with its raw flag.

| Quantity | Checkpoint handoff | Independent recomputation |
|---|---|---|
| Passing cases | 19 of 26 | 19 of 26 |
| Min joint-limit margin | 0.367355 rad | 0.367355 rad |
| Min nonadjacent self-separation | 13.788 mm | 13.788 mm |
| Min non-foot ground clearance | 2.412 mm | 2.412 mm |
| Max stance-contact drift | 1.39e-16 m (consecutive frames) | 1.16e-16 m (from touchdown, per run) |
| Max Jacobian condition | 5.092 | 5.092 |
| Min WALK support margin | 0.1204 mm | 0.1204 mm (id 18) |
| IK branch changes | none recorded | 0 |

The stance-drift definitions differ (the checkpoint measured consecutive frames, the audit measures each stance run from its touchdown); both are numerical zero (~1e-16 m). These aggregates are over the *strict-saved-sampling* passes only.

## 3. Definitions (VERIFIED_FROM_SOURCE; unchanged from the checkpoint)

* **Scheduler.** Unwrapped cycle count `s`; each leg's local phase is `fract(s + offset)`; physical time enters only through the caller's period (`cycles/s = 1/T`). Order LF RF RH LH. Phase boundaries are restored within eight input ULPs (a numerical rule, not a physical tolerance); cycles are bounded to 1e6 (resource guard).
* **WALK.** Offsets `[0, 1/2, 3/4, 1/4]`, swing order RH → RF → LH → LF, duty in [0.75, 1). One leg swings at a time; rear/front support triangles alternate; the all-stance gaps (duty - 0.75) allow an *open-loop* fore/aft body shift ("X sway"). The sway is a study parameter, not stabilization.
* **TROT.** Offsets `[0, 1/2, 0, 1/2]`; diagonal pairs LF+RH and RF+LH; duty in [0.5, 1). Static support margin is recorded as a diagnostic only.
* **Stance.** Each stance foot is an immutable *world* point: the canonical G2 physical-contact reference (not the foot_link origin); a new anchor is placed from the nominal planar body transform at touchdown + duty/2.
* **Swing.** XY by the quintic `10u^3 - 15u^4 + 6u^5`; Z by `64 h u^3 (1-u)^3`; zero velocity and acceleration at both ends, so q is C2 across lift-off/touchdown.
* **Body.** Level (roll = pitch = 0), flat ground, SE(2) twist integrated with the exact exponential (series near zero yaw). Turning is a rigid transform, never per-leg offsets.
* **IK.** Existing G2 contact IK with the previous solution as seed and branch reference; analytic contact Jacobian/Hessian give qdot/qddot; failures are explicit `GaitStatus` values and never clipped.
* **Lifecycle and watchdog** are described in section 7 and in the README.

## 4. Evidence tiers and representatives

| Tier | Meaning |
|---|---|
| SCREEN | saved strict screens: early exit, no self-collision |
| FULL_MESH at saved sampling | 19 cases; independent audit pass |
| SWEEP-SAFE | also no sub -1 um mesh height anywhere in the continuous stance-pitch sweep of the case (section 6) |
| FULL_MESH_VALIDATED (representative) | new 400-interval full-mesh pass of the audit, sweep-safe, and a ground/support pass at 1600 intervals |
| DIAGNOSTIC | micrometre foot_link events quarantined; never acceptance |

Five saved passes are not sweep-safe (1230, 1238, 3053, 3069, 3084): they pass their 81 sampled frames but a continuous pitch sweep crosses a facet window below -1 um.

All four checkpoint TROT candidates (ids 1046, 1238, 3084, 3105) pass at 81 frames and **fail a 201-frame full-mesh recheck** (`dense_representatives.json`): 1238 and 3084 by foot penetration of 1.03 um, 1046 and 3105 only by the undeclared-contact band. **TROT id1311 is a failed 201-frame candidate and is not a representative**; the old `render.py` selection of it is removed. The WALK candidate id287 passes at 401 frames but fails the ground screen at 800 and 1600 intervals (band events only, no penetration), so it is labelled density-fragile.

A targeted search (560 parameter sets, 80-120 mm, duty 0.5-0.9, both signs; `robust_search.py`) kept 102 at 400 intervals and **24 at 1600 intervals** (ground and support only). Four of them were then checked on the full mesh at 400 intervals.

| Name | Id | Status | Height | Advance X | Lift | Duty | Sway X | Frames | Min joint margin | Min self-sep | Min non-foot ground | Min support margin | Weakest |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| walk | 357 | FULL_MESH_VALIDATED | 80 mm | +10 mm/cycle | 10 mm | 0.8 | 4 mm | 401 | 0.367 rad | 13.79 mm | 2.50 mm | 1.59 mm | phase 0.052, 3 feet |
| walk_100mm | 287 | FULL_MESH_SAMPLED_PASS_DENSITY_FRAGILE | 100 mm | +10 mm/cycle | 10 mm | 0.8 | 4 mm | 401 | 0.601 rad | 13.79 mm | 2.65 mm | 2.05 mm | phase 0.052, 3 feet |
| trot | 61 | FULL_MESH_VALIDATED | 80 mm | +20 mm/cycle | 10 mm | 0.8 | 0 mm | 401 | 0.388 rad | 13.79 mm | 2.50 mm | -9.95 mm (TROT: diagnostic only) | phase 0.805, 2 feet |
| trot_120mm | 309 | FULL_MESH_VALIDATED | 120 mm | +20 mm/cycle | 10 mm | 0.7 | 0 mm | 401 | 0.785 rad | 13.79 mm | 2.79 mm | -1.09 mm (TROT: diagnostic only) | phase 0.702, 2 feet |

* Representatives are sampled finite sets. They are `FULL_MESH_VALIDATED` only in the sense above. There is no density-robust TROT at 100 mm in the searched grid; the 100 mm TROT passes of the checkpoint are not density-robust.
* TROT support margin is negative in the two-foot diagonal phases by construction (COM off the diagonal line): this is the reason TROT is dynamic and is not rejected by it.
* No IK branch change in any representative or any strict-saved pass.

### Dense-sampling sensitivity of the strict policy

| Intervals per cycle | 80 | 200 | 400 | 800 | 1600 |
|---|---|---|---|---|---|
| Of the 19 saved passes, still passing the ground/support screen | 19 | 8 | 4 | 3 | 2 |

The strict policy forbids a *swing* foot's mesh within 1 um of the ground away from the exact lift-off/touchdown event. Any continuous foot must pass through that band when it leaves or reaches the ground, so a sampled test hits the band once the sampling interval is shorter than the time spent in it (a few ms at 5-10 mm lift). A pass at one sampling density is therefore not a property of the trajectory. A swing foot can only enter or leave the band when `delta` at its lift-off/touchdown pitch is below 1 um (section 6). Measured at the boundary feet: the density-robust sets (ids 18, 4000, 357, 61, 29, 309) have `delta` of at least 1.20 um at every boundary foot, the density-fragile ids 287 and 222 as low as 0.27 um and the checkpoint TROT id 1046 0.67 um.

## 5. Gait envelope (maximum TESTED passing; sampled finite grids)

Primary heights 80, 100, 120, 140, 150 mm; additional stress heights 60, 160, 180, 200, 210, 220, 240, 300 mm (refinement) and 90, 110 mm (robust search). Duty factors evaluated: WALK 0.75, 0.8, 0.9; TROT 0.5, 0.6, 0.7, 0.8. Lift 5-60 mm (envelope grids), 5/10 mm (robust search).

| Case | Strict, saved sampling: passing X mm/cycle | of which sweep-safe ids | Strict, density-robust screen: X mm/cycle | DIAGNOSTIC forward: max tested / first block (mm) | DIAGNOSTIC backward: max tested / first block (mm) |
|---|---|---|---|---|---|
| WALK 80 mm | -10, 10, 15 | 18, 3012, 4000 | -10, 10 | 200 / 180 (SUPPORT_INVALID) | 10 / 100 (SUPPORT_INVALID) |
| WALK 90 mm | none | none | -10 | none / not reached | none / not reached |
| WALK 100 mm | -12.5, -10, 10 | 222, 287, 3032 | none | 250 / 100 (SUPPORT_INVALID) | 12.5 / 100 (SUPPORT_INVALID) |
| WALK 120 mm | -15, -10, 20 | 416, 426 | none | 200 / 140 (SUPPORT_INVALID) | 20 / 100 (SUPPORT_INVALID) |
| WALK 140 mm | none | none | none | 160 / 180 (JOINT_LIMIT_BLOCKED) | 40 / 100 (SUPPORT_INVALID) |
| WALK 150 mm | none | none | none | 100 / 120 (JOINT_LIMIT_BLOCKED) | none / 100 (SUPPORT_INVALID) |
| TROT 80 mm | -50, -40, 40, 50 | 1040, 1046 | -30, -20, 20, 30 | 250 / 300 (GROUND_COLLISION) | 250 / 300 (GROUND_COLLISION) |
| TROT 90 mm | none | none | -20, 20 | none / not reached | none / not reached |
| TROT 100 mm | -60, -45, 45, 60 | 3092, 3105 | none | 300 / not reached | 300 / not reached |
| TROT 120 mm | -30, 30 | 3116, 3125 | -20, 20 | 300 / not reached | 300 / not reached |
| TROT 140 mm | none | none | none | 200 / 250 (JOINT_LIMIT_BLOCKED) | 200 / 250 (JOINT_LIMIT_BLOCKED) |
| TROT 150 mm | none | none | none | 140 / 160 (JOINT_LIMIT_BLOCKED) | 140 / 160 (JOINT_LIMIT_BLOCKED) |

Reading the table:

* **Strict columns** are the answer under the unchanged policy; they are small and sampling-sensitive. **140 and 150 mm have no strict full-mesh pass in the selected sets. This is not a statement that locomotion at 140/150 mm is impossible** — see the diagnostic columns: the same heights pass the kinematic diagnostic out to 100-200 mm/cycle.
* **What limits the strict envelope.** Of the 2352 strict screen cases (390 pass), the first failure is: FOOT_LINK_GROUND_PENETRATION: 1380, SUPPORT_INVALID: 263, CONTACT_INVALID: 255, IK_UNREACHABLE: 42, FOOT_LINK_GROUND_PENETRATION + SUPPORT_INVALID: 16, JOINT_LIMIT_BLOCKED: 3, FOOT_LINK_GROUND_PENETRATION + FOOT_LINK_MISSING_MESH_SUPPORT: 2, NUMERICALLY_ILL_CONDITIONED: 1. No strict-screen case first fails on a non-foot link touching the ground. About 70 percent of all strict rejections are a foot_link ground event, in pure fore/aft cases at most 5.20 um deep in the diagnostic re-screen (bounded by the 5.78 um mesh offset). The strict envelope is therefore set mostly by the micrometre foot-mesh policy, not by robot kinematics.
* **Diagnostic columns** (`diag_*.py`, NOT acceptance, no self-collision beyond +/-100 mm/cycle): 1245 of 2352 saved screen cases pass once micrometre foot events are quarantined. TROT shows no kinematic block at any primary height out to 100 mm/cycle; the grid edge, not the robot, was the limit. Extended to 300 mm/cycle with 10 mm lift: joint limits stop TROT at 150 mm body height near 160 mm/cycle and at 140 mm near 250 mm/cycle; WALK is stopped by joint limits or IK reach at 140/150 mm; the 80 mm bodies reach ground contact at 250-300 mm/cycle. WALK's remaining blocks are mostly SUPPORT_INVALID, i.e. the fixed 4 mm open-loop sway is not suited to every stride and sign (backward WALK fails support almost everywhere): a limit of that *study parameter*. The diagnostic support margin is not monotone in stride.
* Full-mesh frontier with self-collision for 19 largest-stride diagnostic cases (+/-100 mm/cycle, every primary height): 18 of 19 KINEMATICALLY_VALID (the other is a WALK support failure at 100 mm); minimum nonadjacent separation 10.9 mm (WALK 80 mm) and 12.1 mm (TROT 80 mm).
* **Lateral and yaw** are implemented kinematically and tested with both signs and combined commands. The strict screens found only 2 screen-level passes with nonzero lateral or yaw motion and **no** full-mesh-valid nonzero interval. In the diagnostic re-screen only 36 of 600 pass, all with y <= 1 mm/cycle or yaw <= 0.005 rad/cycle; the dominant block (255 cases) is the unchanged G2 contact-IK `CONTACT_INVALID` (nominal-strip contact tilt), and foot penetration exceeds 5.8 um (tilted contact axis) in 321 cases. This is **IMPLEMENTED KINEMATICALLY, FULL-MESH ENVELOPE NOT YET ESTABLISHED**; it is not evidence that MATDOG cannot strafe or turn.
* The robust-search survivors and the diagnostic maxima are parameter-set dependent (lift, duty, sway), not a box.

## 6. Analytic cylinder versus tessellated mesh: classification of the rejection

Everything here uses the unchanged G2 contact reference, the unchanged 1 um tolerance and the unchanged canonical meshes.

**Mechanism (VERIFIED_FROM_ARTIFACT, reproduced to 6.1e-12 um).** With the analytic reference on the ground the lowest mesh vertex is at `delta(phi) = R + min_v u(phi).(v - c)` from the ground, where `c = (0, 0, R)` is the analytic cylinder centre, R = 14.9 mm and `u` the world-up vector in foot_link for foot pitch `phi`. `delta` is a function of the foot pitch only. It reproduces every saved stance foot (largest error 6.1e-12 um).

* The mesh rings are exact circles (residual 0) but their centre is offset by (dx, dz) = (4.535, 3.576) um, magnitude **5.775 um**, from the analytic cylinder axis. A rigid offset gives `delta` an amplitude of 5.775 um.
* Tessellation only *raises* the mesh: the residual of `delta` against the ideal offset circle lies in [-0.0010, 11.86] um. It never adds penetration beyond the offset, but it makes `delta` jagged (changes of 2 um within 1 degree of pitch), which is why pass/fail flips between nearby samples.
* Over all pitches `delta` ranges from -5.775 to 16.265 um, and is below -1 um for **40.8 percent** of pitch angles. Windows in the relevant range (pitch deg): [-67.92, -63.91]; [-63.63, -61.66]; [-60.83, -59.41]; [-58.45, -54.95]; [-54.86, -53.84]; [-53.66, -52.74]; [-51.23, -50.56]; [-50.01, -49.49]; [-48.7, -48.47].

**Why a continuous start/stop cannot avoid it.** With rear contacts locked in the world, a level body and a given height, the rear foot pitch is a continuous function of body height and fore/aft shift. The canonical STAND has rear pitch -51.52 deg (`delta` = -0.38 um, passes). Reaching a 100 mm or 80 mm body requires sweeping the pitch through the window [-51.23, -50.56] deg, where `delta` falls to -1.262 um, whatever the time law. The intermediate value theorem makes this unavoidable for any continuous path with locked rear contacts; it needs only about 1 mm of descent. The same sweep appears in the stop. A 5 ms probe confirms the worst case at the predicted value:

| Lifecycle (5 ms, ground policy) | Result | Penetration instances | Undeclared-contact instances | Worst mesh z [um] | Rear foot pitch range [deg] |
|---|---|---|---|---|---|
| WALK_100mm_id287 | FAIL | 88 | 7 | -1.262 | -51.5 .. -35.2 |
| WALK_80mm_id357 | FAIL | 76 | 5 | -1.260 | -51.5 .. -33.6 |
| TROT_100mm_id3105 | FAIL | 130 | 85 | -1.704 | -56.2 .. -33.0 |
| TROT_80mm_id61 | FAIL | 95 | 2 | -1.261 | -51.5 .. -35.4 |

**The saved 141-frame full-mesh lifecycles** (WALK: 5 of 141 frames fail, worst mesh-ground z -1.216327e-06 m; TROT: 13 of 141 frames fail, worst mesh-ground z -1.183000e-06 m). Every failing frame, with analytic contact and mesh height measured against the ground:

| Gait | t [s] | State | Foot | Role | Analytic contact z [um] | Mesh min z [um] | Mesh - analytic [um] | Foot pitch [deg] | Policy event |
|---|---|---|---|---|---|---|---|---|---|
| WALK | 0.15 | GAIT_START | RH | STANCE | -0.000 | -1.216 | -1.216 | -51.03 | GROUND_PENETRATION |
| WALK | 0.15 | GAIT_START | LH | STANCE | -0.000 | -1.216 | -1.216 | -51.03 | GROUND_PENETRATION |
| WALK | 0.25 | GAIT_START | RH | STANCE | -0.000 | -1.144 | -1.144 | -49.69 | GROUND_PENETRATION |
| WALK | 0.25 | GAIT_START | LH | STANCE | -0.000 | -1.144 | -1.144 | -49.69 | GROUND_PENETRATION |
| WALK | 4.25 | STOPPING | RH | SWING | +0.090 | +0.005 | -0.085 | -39.25 | UNDECLARED_GROUND_CONTACT |
| WALK | 6.75 | STOPPING | RH | STANCE | +0.000 | -1.144 | -1.144 | -49.69 | GROUND_PENETRATION |
| WALK | 6.75 | STOPPING | LH | STANCE | +0.000 | -1.144 | -1.144 | -49.69 | GROUND_PENETRATION |
| WALK | 6.85 | STOPPING | RH | STANCE | +0.000 | -1.216 | -1.216 | -51.03 | GROUND_PENETRATION |
| WALK | 6.85 | STOPPING | LH | STANCE | +0.000 | -1.216 | -1.216 | -51.03 | GROUND_PENETRATION |
| TROT | 0.15 | GAIT_START | RH | STANCE | +0.000 | -1.183 | -1.183 | -51.08 | GROUND_PENETRATION |
| TROT | 0.15 | GAIT_START | LH | STANCE | +0.000 | -1.183 | -1.183 | -51.08 | GROUND_PENETRATION |
| TROT | 0.20 | GAIT_START | RH | STANCE | +0.000 | -1.036 | -1.036 | -50.58 | GROUND_PENETRATION |
| TROT | 0.20 | GAIT_START | LH | STANCE | +0.000 | -1.036 | -1.036 | -50.58 | GROUND_PENETRATION |
| TROT | 0.25 | GAIT_START | RH | STANCE | +0.000 | -1.121 | -1.121 | -49.87 | GROUND_PENETRATION |
| TROT | 0.25 | GAIT_START | LH | STANCE | +0.000 | -1.121 | -1.121 | -49.87 | GROUND_PENETRATION |
| TROT | 5.65 | STOPPING | RH | SWING | +0.518 | +0.833 | +0.315 | -41.12 | UNDECLARED_GROUND_CONTACT |
| TROT | 5.70 | STOPPING | RH | SWING | +0.091 | +0.385 | +0.294 | -41.13 | UNDECLARED_GROUND_CONTACT |
| TROT | 5.75 | STOPPING | RH | SWING | +0.011 | +0.292 | +0.280 | -41.14 | UNDECLARED_GROUND_CONTACT |
| TROT | 5.80 | STOPPING | RH | SWING | +0.001 | +0.273 | +0.272 | -41.14 | UNDECLARED_GROUND_CONTACT |
| TROT | 5.85 | STOPPING | RH | SWING | +0.000 | +0.269 | +0.269 | -41.14 | UNDECLARED_GROUND_CONTACT |
| TROT | 5.90 | STOPPING | RH | SWING | +0.000 | +0.267 | +0.267 | -41.14 | UNDECLARED_GROUND_CONTACT |
| TROT | 5.95 | STOPPING | RH | SWING | +0.000 | +0.267 | +0.267 | -41.14 | UNDECLARED_GROUND_CONTACT |
| TROT | 6.75 | STOPPING | RH | STANCE | +0.000 | -1.121 | -1.121 | -49.87 | GROUND_PENETRATION |
| TROT | 6.75 | STOPPING | LH | STANCE | +0.000 | -1.121 | -1.121 | -49.87 | GROUND_PENETRATION |
| TROT | 6.80 | STOPPING | RH | STANCE | +0.000 | -1.036 | -1.036 | -50.58 | GROUND_PENETRATION |
| TROT | 6.80 | STOPPING | LH | STANCE | +0.000 | -1.036 | -1.036 | -50.58 | GROUND_PENETRATION |
| TROT | 6.85 | STOPPING | RH | STANCE | +0.000 | -1.183 | -1.183 | -51.08 | GROUND_PENETRATION |
| TROT | 6.85 | STOPPING | LH | STANCE | +0.000 | -1.183 | -1.183 | -51.08 | GROUND_PENETRATION |

* Penetration rows are stance feet whose analytic reference is exactly on the ground; the mesh is 1.04-1.22 um below it, at the rear foot pitch predicted by `delta`. These all occur in the four-contact height-change segments (start preparation, final recenter).
* Undeclared-contact rows are *swing* feet within 1 um of the ground while the *mesh* is still above it (0.005-0.83 um). In the stop, the last decelerating cycle ends exactly at a lift-off/touchdown boundary with phase rate approaching zero, so the foot dwells in the band for about 0.3 s before the touchdown event. This is a contact-mode semantic issue (and a property of the stop time law), not a penetration.

**Evidence summary and classification.** A (genuine geometric incompatibility), B (representation / tessellation sensitivity), C (contact-mode boundary semantics), D (combination), E (insufficient evidence):

* The penetration events are explained to the stated precision by a *deterministic, rigid* 5.78 um registration offset between two chosen representations, not by random tessellation error (tessellation never adds penetration). For the pair {G2 analytic reference, 1 um policy, canonical mesh} this is a real incompatibility: no continuous path through certain foot pitches satisfies both.
* The band events (undeclared contact) are not penetrations; they come from the sampled band rule and the zero-rate stop ending (C), and their count grows with sampling density.
* Tessellation sensitivity (B) is real but secondary: it makes the pass/fail boundary jagged and sample dependent.
* **Verdict: D, a mixed cause — dominated by a deterministic analytic-versus-mesh registration mismatch, amplified by tessellation jaggedness, plus a contact-mode boundary issue.** This is evidence about the *model contract*, not about the physical robot: the evidence does not show that a real foot would ever be unable to stand or walk, and it also does not show the micrometre contact is physically acceptable. The physical meaning of a 1 um mesh/ground distance is below what the model resolves, so it is not claimed either way.
* Possible later resolution (NOT applied here): an explicit G2-level reconciliation of the contact reference with the mesh (or a stated contact-transition contract), then re-validation with continuous (swept) checks. This is a decision for a future milestone because it changes accepted contact semantics.

## 7. Lifecycle and watchdog (VERIFIED_FROM_SOURCE; joins verified by the oracle)

* **Start.** From the completed G3 canonical STAND only (`Locomotion` owns an unchanged `StandTransition`; G3 `MotionState`/startup files are byte-identical to the baseline; `REST_GROUND` is not a startup path). One period of four-contact preparation to the chosen height and initial sway, two periods of quintic phase-speed acceleration covering one geometric cycle, then steady gait. Joint velocity and acceleration are zero at the preparation and terminal joins.
* **Stop.** Finish the current cycle; decelerate through one final planned cycle over two periods with future touchdowns placed at the canonical terminal contacts; recenter height and shift over one more period with contacts locked. An in-flight swing is never retargeted. A stop during preparation returns by a four-contact hold. DISABLE/FAULT cancels target generation; it is not a physical braking claim.
* **Watchdog.** Caller time only: finite, monotonic `now`; non-future stamps; strictly increasing sequence; age == timeout is fresh, > timeout is stale. Invalid, stale, zero, or mode/geometry/period-changing commands request a semantic stop; a changed command is not applied mid-cycle and needs an explicit restart from STAND. Backward or nonfinite sample time returns an explicit failure.
* **Result.** Kinematically continuous start/stop joins (C2 phase and body laws, end in canonical q with zero qdot/qddot). **Mesh/contact validity is not established for either complete lifecycle** (section 6). Steady-cycle acceptance and lifecycle acceptance remain separate. No autonomous physical walking route is validated.

## 8. Semantic joint derivatives

Time law: q depends only on the normalized phase, so q is unchanged by the period; qdot scales as 1/T and qddot as 1/T^2 (T = 0.5, 1, 2, 4 s). Verified for every representative at every period: q differs by exactly 0.0 between periods; peaks times T and T^2 are invariant to 1e-12; and an independent central finite difference of the produced q matches the analytic qdot and qddot (relative error < 1e-7 in the table data). Hip joints stay at zero for pure fore/aft gaits. **Semantic requirements only: no comparison with ST3215 or any actuator limit was made.** The peak per joint for all 12 joints is in `derivative_tables.json`.

| Representative | T [s] | max abs q [rad] | peak qdot upper [rad/s] | peak qdot lower [rad/s] | peak qddot upper [rad/s^2] | peak qddot lower [rad/s^2] | peak qdot hip [rad/s] |
|---|---|---|---|---|---|---|---|
| walk (id 357) | 0.5 | 1.771 | 6.320 | 4.280 | 759.46 | 326.17 | 7.5e-16 |
| walk (id 357) | 1 | 1.771 | 3.160 | 2.140 | 189.87 | 81.54 | 3.8e-16 |
| walk (id 357) | 2 | 1.771 | 1.580 | 1.070 | 47.47 | 20.39 | 1.9e-16 |
| walk (id 357) | 4 | 1.771 | 0.790 | 0.535 | 11.87 | 5.10 | 9.4e-17 |
| walk_100mm (id 287) | 0.5 | 1.537 | 5.422 | 4.630 | 653.35 | 302.19 | 6.0e-16 |
| walk_100mm (id 287) | 1 | 1.537 | 2.711 | 2.315 | 163.34 | 75.55 | 3.0e-16 |
| walk_100mm (id 287) | 2 | 1.537 | 1.355 | 1.158 | 40.83 | 18.89 | 1.5e-16 |
| walk_100mm (id 287) | 4 | 1.537 | 0.678 | 0.579 | 10.21 | 4.72 | 7.5e-17 |
| trot (id 61) | 0.5 | 1.750 | 5.872 | 4.587 | 328.14 | 291.36 | 7.5e-16 |
| trot (id 61) | 1 | 1.750 | 2.936 | 2.294 | 82.04 | 72.84 | 3.8e-16 |
| trot (id 61) | 2 | 1.750 | 1.468 | 1.147 | 20.51 | 18.21 | 1.9e-16 |
| trot (id 61) | 4 | 1.750 | 0.734 | 0.573 | 5.13 | 4.55 | 9.4e-17 |
| trot_120mm (id 309) | 0.5 | 1.310 | 3.011 | 3.597 | 113.75 | 153.61 | 3.5e-16 |
| trot_120mm (id 309) | 1 | 1.310 | 1.505 | 1.798 | 28.44 | 38.40 | 1.8e-16 |
| trot_120mm (id 309) | 2 | 1.310 | 0.753 | 0.899 | 7.11 | 9.60 | 8.8e-17 |
| trot_120mm (id 309) | 4 | 1.310 | 0.376 | 0.450 | 1.78 | 2.40 | 4.4e-17 |

The checkpoint's per-joint peaks at T = 1 s for ids 287, 1238 and 3105 remain in `handoff_state.json`. The independent oracle also validates the lifecycle derivatives with Richardson-extrapolated differences across the C2 joins (jerk steps).

## 9. XGO architecture comparison (no physical value transferred)

Evidence: 40 pinned files at origin/main `a1b34a8594e5bc76c76b1e3ddf89a3aef2b98298` (read-only archive; the original checkout was not touched). Re-used architecturally: normalized phase and per-leg phase slots, WALK as quarter-spaced slots and TROT as alternating pairs, stance/swing states, command/state separation, Cartesian target construction with body transform before IK, explicit start/stop lifecycle. Differences by design: elapsed-time normalized modulo (XGO increments per call and resets at strict `phase > period`), C2 quintic swing and timed joins (XGO is linear), an independent freshness contract (no bound XGO locomotion command-expiry proof was found; host read timeouts are not one), a stop that places terminal contacts in the world (XGO's zero-command phase reset does not prove a world-locked terminal stand), and zero command = semantic stop (not mark-time). **Not transferred:** geometry, joint angles, zeros, signs, joint limits, stride, lift, duty, period, gains, body height, velocity/acceleration limits, IMU tuning; there is no 1.5x conversion rule; no XGO firmware was executed. All MATDOG values were derived from the canonical URDF/G2 model or are explicit study parameters.

## 10. Validation on the final source (ALL PASSED)

Run by `validate.py` on the current source at HEAD `49aeae2b25770b361f4c9a2a926457703c315e0e` (worktree clean before the run: True). Independent oracle: 3872 samples, 16 independently solved legs, lifecycle 46 frames.

| Gate | Result | Summary |
|---|---|---|
| g4_cpp_unit_build | PASS |  |
| g4_cpp_unit_tests | PASS | GAIT_HOST = PASS: 148291 checks, 0 failures |
| g4_independent_oracle | PASS | "status": "PASS", |
| saved_frames_vs_current_core | PASS | REVALIDATE_SAVED PASS 35 cases; worst 0.0 |
| g4_artifact_tests | PASS | Ran 15 tests in 1.505s; OK |
| g35_pose_audit_tests | PASS | Ran 28 tests in 15.967s; OK |
| g35_artifact_manifest_check | PASS | ARTIFACT_MANIFEST OK 80 artifacts; historical generator revisions: 1 |
| host_motion_runner_g1_g2_g3_g35_g4 | PASS | GAIT_HOST = PASS: 148291 checks, 0 failures |
| static_audit | PASS | STATIC_AUDIT = PASS |
| g4_asan_ubsan_build | PASS |  |
| g4_asan_ubsan_run | PASS | GAIT_HOST = PASS: 148291 checks, 0 failures |
| git_diff_check_worktree | PASS |  |
| git_diff_check_vs_baseline | PASS |  |
| g1_g2_g3_motion_sources_unchanged_vs_baseline | PASS | only Gait.cpp/.h and Locomotion.cpp/.h added; no accepted motion source modified |

Oracle metrics (`oracle_results_final.json`; identical in kind to the checkpoint's `oracle_results.json`):

```json
{
 "max_body": 5.5077470362263625e-17,
 "max_target_m": 5.551115123125783e-17,
 "max_urdf_contact_m": 1.1959531388233124e-16,
 "max_qdot_error": 1.9325782263379665e-07,
 "max_qddot_error": 6.76232944840649e-06,
 "max_numeric_ik_rad": 1.5987211554602254e-14,
 "max_periodic_q_rad": 2.7755575615628914e-15,
 "max_world_stance_drift_m": 1.689731085869606e-16
}
```

The artifact manifest (`artifact_manifest.json`) is written after this report and checked with `artifact_manifest.py --check`; it records the preserved-file check, input-hash checks and the historical-generator note for `definitions.json`.

## 11. Answers to the 34 questions

1. **Scheduler.** Normalized cycle count with per-leg phase `fract(s + offset)`, a stance/swing split by duty, caller-supplied physical period, world-locked anchors and a quintic swing; a `Locomotion` coordinator (OFF, IDLE, STAND_TRANSITION, STAND, STOPPING, GAIT_START, WALK, TROT) with a caller-time watchdog (sections 3, 7).
2. **WALK offsets.** `[0, 0.5, 0.75, 0.25]` for LF RF RH LH: swing order RH, RF, LH, LF with one leg in the air at a time; this alternates rear and front support triangles and leaves all-stance gaps of (duty - 0.75) for the open-loop fore/aft shift. Chosen from MATDOG's canonical leg order and the quasi-static support analysis, not copied from XGO's values.
3. **TROT offsets.** `[0, 0.5, 0, 0.5]`: LF+RH and RF+LH swing together (diagonal pairs from the canonical coordinates).
4. **Duty factors.** WALK 0.75, 0.8, 0.9; TROT 0.5, 0.6, 0.7, 0.8. WALK duty is restricted to [0.75, 1) and TROT to [0.5, 1) by the footfall pattern.
5. **Stance in world coordinates.** Each stance foot is an immutable world point: the canonical G2 physical-contact-reference anchor; later anchors come from the nominal planar body transform at touchdown + duty/2 (never the foot_link origin).
6. **Max stance drift.** About 1e-16 m: 1.39e-16 (checkpoint, consecutive frames), 1.16e-16 m (audit, per run), 1.69e-16 m (oracle). Numerical zero.
7. **Swing trajectory.** XY quintic `10u^3 - 15u^4 + 6u^5`; Z `64 h u^3 (1-u)^3`; zero velocity and acceleration at both ends; exact endpoints; configurable lift.
8. **Body heights.** 80, 100, 120, 140, 150 mm primary; 60, 90, 110, 160, 180, 200, 210, 220, 240, 300 mm additionally. No engineering minimum height is established, and the 1.001 um G3.5 pure-foot boundary was not used.
9. **Forward/backward stride.** Maximum tested passing values by tier are in the section 5 table. Strict, saved sampling: WALK 80 mm -10..+15, 100 mm -12.5..+10, 120 mm -15..+20 mm/cycle; TROT 80 mm -50..+50, 100 mm -60..+60, 120 mm -30..+30 mm/cycle (sweep-safe subsets are smaller). Density-robust strict screen: WALK 80 mm +/-10 (90 mm -10); TROT 80 mm +/-20..30, 90 mm +/-20, 120 mm +/-20. Nothing at 140/150 mm in the strict tiers. Diagnostic: much larger (table). These are maxima over finite grids, not limits.
10. **Lateral.** Implemented kinematically (both signs); strict screens found no full-mesh-valid nonzero lateral interval (2 screen-level passes); diagnostic passes only at 1 mm/cycle; dominant block is the G2 contact tilt validity. Not established, not disproved.
11. **Yaw/cycle.** Implemented (both signs, combined with translation); strict full-mesh: none established. Diagnostic: only 0.001 rad/cycle (and 0.005 rad/cycle with foot penetration up to 9.7 um) pass; 0.01 rad/cycle and above did not pass (G2 contact `CONTACT_INVALID` or foot ground events deeper than 10 um). Not established, not disproved.
12. **What limits each envelope.** Strict envelope: foot_link micrometre ground events (about 70 percent of first failures), then static WALK support, G2 contact validity and the sampling-density effect. Diagnostic kinematic envelope: joint limits and IK reach at large strides and at 140-150 mm, ground contact at 250-300 mm/cycle on 80 mm bodies, WALK support of the fixed open-loop sway. Self-collision was not the first limit in any tested case.
13. **Min joint-limit margin.** 0.367 rad over the 19 strict passes (0.367 / 0.601 / 0.388 / 0.785 rad for the four representatives); 0.052 rad on the WALK 150 mm +100 mm/cycle diagnostic frontier case.
14. **Min self-separation.** 13.79 mm (nonadjacent pairs; 19 strict passes and all representatives); 10.9 mm on the diagnostic frontier. Sixteen directly adjacent assembly interfaces remain excluded; no contact between nonadjacent meshes was detected in any passing case.
15. **Min non-foot ground clearance.** 2.41 mm (19 strict passes); 2.50-2.79 mm for the representatives.
16. **Min quasi-static WALK support margin.** 0.120 mm (id 18); 1.59 mm (id 357) and 2.05 mm (id 287) for the WALK representatives. The margin is the COM projection's distance inside the finite-mesh support hull and is quasi-static only.
17. **Weakest phases.** WALK is weakest in the three-foot phases (one leg in swing), never four-foot; e.g. id 18 at phase 0.2375, id 357 at 0.0525. No WALK frame has fewer than three declared supports. TROT has two-foot diagonal phases by design (static margin negative; dynamics unproven).
18. **Why TROT is not dynamically certified.** The COM projection is off the two-foot support line in swing phases and no dynamics, contact force, friction, actuator or state-estimation model exists; a static polygon cannot certify it. `dynamicStabilityProven` stays false and the artifacts mark TROT NOT_YET_PROVEN.
19. **Peak qdot/qddot.** Section 8; at T = 1 s the largest are in the upper-leg joints: 3.16 rad/s and 190 rad/s^2 (WALK id 357), 2.94 rad/s and 82 rad/s^2 (TROT id 61), 1.80 rad/s and 38 rad/s^2 (TROT id 309, lower leg). Semantic only.
20. **Scaling with period.** q unchanged, qdot proportional to 1/T, qddot proportional to 1/T^2, verified at 0.5, 1, 2, 4 s.
21. **IK branch changes.** None in any strict pass, any representative or the diagnostic frontier cases.
22. **STAND -> gait -> STAND.** Start/stop contracts in section 7; kinematic joins are C2 and end at canonical q. Mesh/contact validity of the complete lifecycle is not established (section 6).
23. **Timeout.** Section 7: stale, invalid, zero or mode-changing commands request a semantic stop of the planned stop path; equality at expiry is fresh; no physical stop is claimed.
24. **XGO concepts reused.** Section 9 (phase slots, pair patterns, state/command separation, Cartesian-then-IK order with body correction, lifecycle).
25. **XGO values not transferred.** Geometry, angles, zeros, signs, limits, stride, lift, duty, period, gains, height, velocity/acceleration limits, IMU tuning; no 1.5x rule.
26. **Tuning candidates.** All of them: height, lift, duty, advance X/Y, yaw per cycle, sway, period, the Jacobian-condition threshold (a numerical rejection policy), start/stop durations and the watchdog timeout. Nothing is an approved physical parameter.
27. **Safest next milestone before hardware walking.** A **G4.1 contact-contract reconciliation**, offline: decide and document how the G2 analytic foot reference, the canonical mesh and the contact-transition band relate (including a swept/continuous check rather than point sampling), then re-run the strict lifecycle. In parallel but separately: approve an actuator-limit source before any derivative comparison, and build a rigid-body dynamics study before any TROT claim. Hardware walking must wait for the separate calibration workstream.
28. **Steady-state FULL_MESH_VALIDATED sets.** Representatives (sampled finite sets): WALK 80 mm id 357, TROT 80 mm id 61, TROT 120 mm id 309 (full mesh at 400 intervals, sweep-safe, 1600-interval ground screen). The fourth full-mesh check, robust-search id 29 (TROT 80 mm, +20 mm/cycle, 5 mm lift, duty 0.8), passes the same gates and was not needed as a representative. Density-fragile: WALK 100 mm id 287. At the checkpoint's saved sampling, 19 cases pass, 14 of them sweep-safe.
29. **Only screen/kinematic candidates.** Every other tested set: the strict screen passes (390 screen-level), the other 21 robust-search survivors (ground/support screen at 1600 intervals only; WALK ids 18 and 4000 also have saved-sampling full-mesh passes but no dense full-mesh record), all diagnostic passes (1245 of 2352 screen cases), the checkpoint TROT ids 1046/1238/3084/3105 and WALK 222, 3012, 3032, 3053 and others at saved sampling only. The checkpoint's 140/150 mm cases and all lateral/yaw cases are not full-mesh validated.
30. **Complete FULL-MESH-valid STAND -> WALK -> STAND?** No.
31. **Complete FULL-MESH-valid STAND -> TROT -> STAND?** No.
32. **What blocks each lifecycle.** WALK: five of 141 frames; rear feet 1.14-1.22 um below the ground in the start preparation (t = 0.15, 0.25 s) and the recenter (6.75, 6.85 s), plus one swing foot within 1 um of the ground (4.25 s). TROT: 13 of 141; the same rear-foot penetration at six frames (1.04-1.18 um) and seven undeclared-contact frames (5.65-5.95 s) from the zero-rate stop ending. A 5 ms probe of four parameter sets reproduces a 1.26 um worst case (1.70 um for TROT id 3105).
33. **Measured discrepancy.** Mesh-minus-analytic height of stance feet: -0.50 to -1.00 um (sampled) and -0.57 to -1.05 um (continuous pitch sweep) for the strict passes; -1.22 um (WALK) and -1.18 um (TROT) at the saved lifecycle failures; -1.26 um over the continuous start sweep; theoretical range of the foot mesh -5.78 to +16.27 um; rigid mesh-circle offset 5.775 um. Reproduced from the foot mesh alone to 6.1e-12 um.
34. **Cause.** Mixed (D): a deterministic analytic-versus-mesh registration mismatch (dominant; for the model contract an unavoidable incompatibility at certain foot pitches), amplified by tessellation sensitivity, together with a contact-mode boundary effect for the undeclared-contact band. Insufficient evidence on the physical meaning of micrometre contact; none is claimed.

## 12. Limits of what is claimed

* Finite sampled grids; point tests are not continuous clearance; a pass at one sampling density is not a property of the trajectory (section 4).
* Flat ground, level body, planar motion only; no IMU, dynamics, friction, compliance, actuator, timing or mass-property validation beyond the URDF inertials.
* The diagnostic tier is explicitly not acceptance; it exists only to expose what else limits strides.
* Candidate rejection of the lifecycles is retained; G2, the tolerance and the targets were not changed. The pose-graph edges accepted in G3.5 were validated by sampling and may share this continuous-sweep sensitivity; that was not examined.
* Only offline software was run. G3 startup remains active; BODY_ONLY REST_GROUND to autonomous STAND remains unvalidated.
