# G4.1 — Contact model reconciliation and continuous lifecycle validation

**Scope: OFFLINE software only. No hardware, serial device, actuator bus, servo command, flashing, persistent-memory write or calibration change was used or created. The separate calibration worktree was only read (hashes).**
Start: G4 final `3ce85f92e2b5381b65e327dd371b2f164308f326`. No accepted G1-G4 file or evidence was changed (checked by the `accepted_g1_g4_files_unchanged_vs_g4_head` gate). G4.1 adds new tools, one pure C++ unit (`ContactMode`) with its test, and new evidence only.

## 0. Verdict

* **Root cause (1).** Both STLs of the foot (collision and visual) contain the nominal 14.9 mm tread circle **translated rigidly by (4.534, 3.577) um, 5.775 um in magnitude**, from the G2 analytic reference. It is a property of the CAD-to-foot_link registration, not of tessellation or numerics.
* **Complete lifecycles.** Under the G4.1 contact contract the four reference gaits — WALK 357, WALK 287, TROT 61, TROT 309 — each complete STAND → gait → STAND with **no failed check and no unresolved check** (section 6). Terminal STAND is canonical and every certified margin is positive. TROT remains kinematically studied and dynamically uncertified.
* **This is not a tolerance change.** The legacy G4 rule (mesh within 1 um) **still rejects** all four lifecycles on the unchanged G2 (worst -1.262 um) and that rejection is recorded, not hidden. What changed is the contract: the tread is judged on the analytic surface, the mesh/analytic difference is bounded in closed form and declared, and contact modes follow the schedule.
* **Selected correction.** A2 (policy: analytic tread authority) is applied in G4.1 on the unchanged G2. The evidence additionally supports A1 (re-register the G2 reference to the mesh, "G2.1"): it removes the penetration exactly. A1 changes an accepted cross-milestone contract, so it is **evaluated as a prototype and NOT applied; it needs approval** (migration strategy in section 4).
* **Not claimed.** Hardware-approved parameters, dynamic stability, actuator limits, lateral/yaw contact, or physical truth of a micrometre foot contact.

## 1. Root cause of the 5.775 um discrepancy

Five things were separated (`root_cause.json`):

| Layer | Finding |
|---|---|
| Nominal CAD geometry | G2 YAML `cad_validation`: R = 14.9 mm, centre z = 14.9 mm, tread 13.9 mm, fillet 2 mm. G2 uses exactly these values: the analytic reference **is** the nominal CAD tread |
| Ideal analytic contact | the G2 cylinder; contact point = centre + R down; no mesh involved |
| Tessellated collision mesh | y=-4.95 mm: 94 vertices, max gap 6.55 deg, chord sagitta <= 24.3 um; y=+5.05 mm: 482 vertices, max gap 4.57 deg, chord sagitta <= 11.9 um. Tread vertices lie on or inside the nominal-radius circle translated by (4.534, 3.577) um: maximum radial excess **0.0006 um**, 327 vertices on it to 0.01 um. Extents against the nominal circle (um): x_min_um=+4.53, x_max_um=+4.51, z_min_um=+3.60, z_max_um=+3.55 |
| Actual lowest point | always a vertex (287 distinct lowest vertices over pitch), never an edge or face; mesh-minus-analytic height `delta(phi)` ranges -5.775 .. 16.265 um and is below -1 um for 40.8 percent of pitch angles |
| Numerical error | float64 vs long double 2.6e-12 um; float32 STL quantisation 5.0e-07 um: negligible |

* The **visual** STL carries the same translation (4.535, 3.581) um (difference from the collision STL under 0.01 um). Two independently exported meshes agree, so the offset is real CAD-to-foot_link registration and not an export artefact. The G2 YAML audit used the visual STL and rejected a *centroid* offset because the centroid depends on tessellation; the circle translation does not.
* `delta(phi) = u.d + (non-negative tessellation term)`, with `u` the world-up vector in foot_link for foot pitch `phi` and `d` the translation. So the mesh sits up to 5.775 um below the analytic reference at the worst pitch; with the tessellation term (up to 10.8 um, because tessellation inscribes a polygon in the translated circle) it sits up to 16.265 um above it.
* All four feet have identical collision geometry in foot_link (max vertex difference 0 um: True); the discrepancy of each foot is `delta` at that foot's pitch (G4: rear feet at -51 deg, delta = -1.2 um).
* The geometric cause of the translation itself (why the CAD wheel axis is 5.8 um from the nominal point in the foot frame) cannot be determined from the repository; the URDF foot joint origin is specified to 0.1 mm. This is an **open question for the CAD owner**, not something the offline model can settle.

## 2. Canonical geometry and provenance

Geometry consistent: **True**. URDF sha256 `3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59`.

* `SHA256SUMS.txt` verifies every URDF/mesh file. The G2 YAML, the G3.5 pose library and the G4 `definitions.json` record this URDF hash and it matches. The G1 `matdog_motion_geometry_export.py --check` ("Geometry V5 / live URDF") and the G2 `matdog_contact_stand_export.py --check` pass.
* The calibration worktree (read only, HEAD `174aa0415eb85e4ecc32a69ac5134bc6b0b9bd4c`) has 0 differing URDF/STL files and an identical foot contact YAML. Geometry V5 and the Full Calibration provenance consume the same URDF and meshes; they define no different foot or contact geometry, and calibration-side URDF-limit/q0 findings do not touch the foot.
* **Material discrepancy found:** the G2 YAML mesh audit (hash `e43737...`, lowest z 16.7 um, `all_four_foot_meshes_identical`) is of the **visual** foot STL; the G3.5 and G4 collision checks use the **collision** STL (different file, tessellation and `delta`). The two never contradicted each other numerically (same translation) but were different objects. Impact: none on G1-G4 results; recorded for the CAD/geometry owner.
* No canonical geometry file was modified.

## 3. Alternatives (measured on the exact G4 lifecycle foot motions, WALK 357 and TROT 61, 5 ms)

| Alternative | Reference shift vs G2 [um] | Mesh minus reference, stance feet [um] | Samples below -1 um (WALK 357) | (TROT 61) | Max reference rate [um/s] |
|---|---|---|---|---|---|
| A0 status quo (G2 nominal, strict mesh test) | +0.00 .. +0.00 | -1.260 .. +5.35 | 76 | 99 | 0 |
| A1 registered reference (candidate G2.1) | -1.32 .. +2.72 | +0.000 .. +3.02 | 0 | 0 | 19 |
| A2 analytic tread authority (policy only) | +0.00 .. +0.00 | -1.260 .. +5.35 | 76 | 99 | 0 |
| A3 exact mesh support function | -1.26 .. +5.48 | +0.000 .. +0.00 | 0 | 0 | 454 |
| A4 circumscribing cylinder | -5.78 .. -5.78 | +4.515 .. +11.12 | 0 | 0 | 0 |

Reference shift = change of the contact-reference bottom relative to G2; "mesh minus reference" is the lowest mesh height above that reference for stance feet.

| Alternative | Geometric justification | Mathematical consistency | Impact on G1-G4 and reference data | IK / derivatives | Collision / contact classification | Cost | Regression needed |
|---|---|---|---|---|---|---|---|
| A0 status quo | G2 nominal CAD tread | consistent for IK; inconsistent with the mesh by up to 5.775 um | none | none | rejects every lifecycle; sampling-density dependent | none | none |
| A1 registered reference (G2.1) | mesh tread = nominal circle translated; two STLs agree | exact: the mesh never goes below it, all-pitch bound >= 0 | **changes accepted G2** (YAML, `FootContactData.h`, C4 stand, goldens) and everything derived (G3 stand, G3.5 pose data) | q change <= 4.6e-05 rad, qdot <= 9.0e-05 rad/s (2.8e-05 relative), qddot <= 2.5e-03 rad/s^2 (1.3e-05 relative); stand 4.5e-05 rad; G1 unaffected | penetration removed exactly; mesh stays <= 3.02 um above the reference (inside the 10 um patch band); band events remain (semantic) | constants only in firmware; the offline re-solve is a 3x3 Newton per leg | regenerate and re-run G2, G3, G3.5, G4 gates |
| A2 analytic tread authority | G2 YAML states the cylinder is the stable contact geometry and the mesh is for collision | closed-form `z >= 0` for the swing foot; mesh bounded by the declared U = 5.775 um | **none** (offline policy only; v1 stays available) | none | tread judged analytically; everything else on the mesh | none | G4.1 gates |
| A3 exact mesh support function | the tessellated mesh is not the real tread (chord sagitta up to 24 um on the coarse ring) | continuous but only piecewise smooth: reference rate up to 488 um/s with slope kinks | changes G2 semantics | non-smooth derivatives | trivially no penetration | per-frame min over the foot vertices | full |
| A4 circumscribing cylinder | conservative | exact no-penetration | changes G2 constants | none | stance feet hover up to 11.3 um > 10 um patch band (MISSING_MESH_SUPPORT) | none | full |
| A5 widen the tolerance | none | none | none | none | needs 1.260 um here, still sampling dependent | none | REJECTED |

A z-only or x-only offset cannot fix the problem: the difference is `u.d`, a sinusoid in pitch that needs both components (arbitrary offsets are rejected).

Files embedding the G2 radius (what A1 would regenerate, first 40):
  - `MATDOG_Controller/scripts/tests/test_contact_stand_oracle.py`
  - `MATDOG_Controller/src/motion/CONTACT_STAND.md`
  - `MATDOG_Controller/src/motion/FootContactData.h`
  - `Matdog_Core/contact_audit/alternatives.py`
  - `Matdog_Core/contact_audit/test_contact_audit.py`
  - `Matdog_Core/gait_audit/contact_discrepancy.py`
  - `Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml`
  - `Matdog_Core/kinematics/tests/test_matdog_foot_contact.py`

**Selection.** Evidence: the translation is registered and consistent (A1 is the geometry-consistent mapping); but adopting it changes an accepted cross-milestone contract, which this milestone is not authorised to do. A2 needs no contract change and is justified by the G2 YAML's own statement. G4.1 therefore applies A2 and demonstrates A1; both are audited below.

## 4. Contact contract v2, impact on G1-G4, migration

Full text in `contact_audit/README.md`. In short: reference = G2 analytic cylinder (unchanged); modes STANCE / SWING / LIFT_OFF / TOUCHDOWN are declared by the schedule (`ContactMode`, 68 C++ checks) and never inferred from proximity; stance tread height 0 (IK exact), swing tread height `64 h u^3 (1-u)^3 >= 0` in closed form; the mesh-minus-reference height over **all** pitches is bounded exactly by `R - max|v-c|` (= -5.775 um for G2, >= 0 for the registered reference) and that bound is the declared uncertainty `U = 5.775 um`, never a pass/fail dial; non-foot links keep the strict 1 um mesh test; support margin from the G2 central strip ends over the stance set common to both neighbouring samples.

| Contract | Impact |
|---|---|
| G1 FK/IK | none (foot_link origin unchanged) |
| G2 contact reference, YAML, `FootContactData.h`, C4 stand | **unchanged**; A1 would version them as G2.1 |
| G3 startup gate, `MotionState`, stand targets | unchanged |
| G3.5 pose library, collision policy | unchanged; its 1 um mesh test is kept as policy v1 |
| G4 sources, evidence, representatives | unchanged and byte-identical; G4 `artifact_manifest.py --check` still passes |
| New | `ContactMode.h/.cpp` (additive), `test_contact_mode.cpp`, host-runner line, G4.1 tools and evidence |

**Migration if A1 is approved:** version `G2.1` beside `G2` (centre and radius from `root_cause.json`, fit provenance recorded); regenerate `FootContactData.h`, the C4 stand export and goldens as `G2.1` and keep the `G2` goldens as versioned evidence; re-run the G2/G3/G3.5/G4 gates and re-version the pose and transition evidence; the expected size of the change is the table above.

## 5. Contact-mode, touchdown and lift-off semantics

* `LIFT_OFF` and `TOUCHDOWN` are the exact schedule instants (phase = duty and phase = 0): foot velocity and acceleration are zero (quintic swing), height 0. In the WALK 357 lifecycle there are 24 scheduler events (lift-offs and touchdowns).
* The 1 um band of G4 was a numerical proxy for "touching". On the analytic surface the swing foot height is `>= 0` for every `t`, so an undeclared tread contact is impossible, not merely unobserved. The band events that the legacy rule reports (e.g. the stop's 0.3 s dwell within 1 um before touchdown, or the 150-190 samples that remain even on the registered reference) are the **planned approach to the scheduled touchdown** and its mirror at lift-off; they are not penetration, and penetration (negative analytic height) remains a failure.
* Undeclared contact is still checked wherever it can occur: every non-foot link and the non-tread foot geometry, against the mesh, with the 1 um band.

## 6. Continuous verification and lifecycle results

Evidence level of every quantity (2 ms grid, 3501 samples over 7 s per lifecycle; start from the canonical 150 mm STAND):

| Quantity | Status (WALK 357, G2 nominal) |
|---|---|
| joint_limit_margin | CONTINUOUSLY_VERIFIED_BOUNDED |
| ik_branch | CONTINUOUSLY_VERIFIED_BOUNDED |
| derivatives | SAMPLED_ONLY |
| tread_analytic_stance_exact | CONTINUOUSLY_VERIFIED_ANALYTIC |
| tread_analytic_swing_nonnegative | CONTINUOUSLY_VERIFIED_ANALYTIC |
| tread_mesh_vs_reference | CONTINUOUSLY_VERIFIED_ANALYTIC |
| nonfoot_ground_clearance | CONTINUOUSLY_VERIFIED_BOUNDED |
| self_separation | CONTINUOUSLY_VERIFIED_BOUNDED |
| support_margin_walk | CONTINUOUSLY_VERIFIED_BOUNDED |

For TROT: joint_limit_margin: CONTINUOUSLY_VERIFIED_BOUNDED, ik_branch: CONTINUOUSLY_VERIFIED_BOUNDED, derivatives: SAMPLED_ONLY, tread_analytic_stance_exact: CONTINUOUSLY_VERIFIED_ANALYTIC, tread_analytic_swing_nonnegative: CONTINUOUSLY_VERIFIED_ANALYTIC, tread_mesh_vs_reference: CONTINUOUSLY_VERIFIED_ANALYTIC, nonfoot_ground_clearance: CONTINUOUSLY_VERIFIED_BOUNDED, self_separation: CONTINUOUSLY_VERIFIED_BOUNDED, support_margin_trot: DIAGNOSTIC_ONLY_DYNAMICS_NOT_PROVEN. The TROT support margin is diagnostic only (dynamics not proven).

* `CONTINUOUSLY_VERIFIED_ANALYTIC`: closed form, valid for all `t` (tread stance exactness, swing height >= 0, mesh-minus-reference bound over all pitches; scope: foot axis tilt 0).
* `CONTINUOUSLY_VERIFIED_BOUNDED`: sampled value minus an interpolation-error bound (`A h^2 / 8` for C2 quantities, Lipschitz for separation) whose constant is **estimated** from the dense samples with a 1.5 safety factor. This is **not a formal proof**; it is confirmed by a 0.5 ms re-run (below).
* `SAMPLED_ONLY`: derivative magnitudes and steps (the G4 oracle covers the analytic C2 joins).
* `PASS_WITH_SAMPLED_ONLY_ITEMS` means no check failed and no check is unresolved, but the derivative item is only sampled (the lifecycle is not claimed fully continuous in that respect).
* For the unchanged G2 reference, `tread_mesh_vs_reference` is verified **against the declared uncertainty** `U`, which by construction equals the measured offset (its exact content is the all-pitch bound -5.775 um); for the registered reference the same bound is >= 0, a statement with no allowance at all. The first is therefore a model-uncertainty statement, the second a geometric one.

| Case | Contact reference | Verdict | Final state | Terminal contacts vs initial [m] | Joint margin (certified) [rad] | Non-foot ground clearance (certified) | Min self-separation (certified) | WALK support margin (certified) | Tread mesh-minus-reference, sampled min [um] | all-pitch exact bound [um] | Legacy G4 rule: penetration / band samples |
|---|---|---|---|---|---|---|---|---|---|---|---|
| WALK 357 | G2 nominal (unchanged) | PASS_WITH_SAMPLED_ONLY_ITEMS | STAND | 0.0e+00 | 0.305 | 2.43 mm | 8.55 mm | 1.09 mm | -1.262 | -5.775 | 100 / 11 |
| WALK 357 | registered candidate | PASS_WITH_SAMPLED_ONLY_ITEMS | STAND | 0.0e+00 | 0.305 | 2.43 mm | 8.55 mm | 1.09 mm | +0.000 | +0.000 | 0 / 173 |
| WALK 287 | G2 nominal (unchanged) | PASS_WITH_SAMPLED_ONLY_ITEMS | STAND | 0.0e+00 | 0.305 | 2.63 mm | 8.47 mm | 1.58 mm | -1.262 | -5.775 | 112 / 16 |
| WALK 287 | registered candidate | PASS_WITH_SAMPLED_ONLY_ITEMS | STAND | 0.0e+00 | 0.305 | 2.63 mm | 8.47 mm | 1.58 mm | +0.000 | +0.000 | 0 / 174 |
| TROT 61 | G2 nominal (unchanged) | PASS_WITH_SAMPLED_ONLY_ITEMS | STAND | 0.0e+00 | 0.305 | 2.43 mm | 7.51 mm | -3.00 mm (diagnostic) | -1.262 | -5.775 | 138 / 3 |
| TROT 61 | registered candidate | PASS_WITH_SAMPLED_ONLY_ITEMS | STAND | 0.0e+00 | 0.305 | 2.43 mm | 7.51 mm | -3.00 mm (diagnostic) | +0.000 | +0.000 | 0 / 152 |
| TROT 309 | G2 nominal (unchanged) | PASS_WITH_SAMPLED_ONLY_ITEMS | STAND | 0.0e+00 | 0.305 | 2.77 mm | 9.13 mm | -2.70 mm (diagnostic) | -1.262 | -5.775 | 200 / 187 |
| TROT 309 | registered candidate | PASS_WITH_SAMPLED_ONLY_ITEMS | STAND | 0.0e+00 | 0.305 | 2.77 mm | 9.13 mm | -2.70 mm (diagnostic) | +0.000 | +0.000 | 0 / 187 |

Convergence confirmation (certified bound at 2 ms never exceeds, and fine sampling never finds a worse minimum than, the 0.5 ms sampled minimum):

| Case | Quantity | Certified lower bound at 2 ms | Sampled minimum at 0.5 ms | Bound <= fine sample |
|---|---|---|---|---|
| WALK_357 | non-foot ground clearance | 0.0024332 | 0.00248361 | yes |
| WALK_357 | self-separation | 0.00855158 | 0.0137878 | yes |
| WALK_357 | joint margin (rad) | 0.304777 | 0.304919 | yes |
| WALK_357 | WALK support margin | 0.00109207 | 0.0010971 | yes |
| WALK_287 | non-foot ground clearance | 0.0026341 | 0.00264493 | yes |
| WALK_287 | self-separation | 0.00846997 | 0.0137878 | yes |
| WALK_287 | joint margin (rad) | 0.304797 | 0.304919 | yes |
| WALK_287 | WALK support margin | 0.00157994 | 0.00158489 | yes |
| TROT_61 | non-foot ground clearance | 0.00242774 | 0.00247009 | yes |
| TROT_61 | self-separation | 0.0075114 | 0.0137878 | yes |
| TROT_61 | joint margin (rad) | 0.304852 | 0.304919 | yes |
| TROT_309 | non-foot ground clearance | 0.00277344 | 0.00277577 | yes |
| TROT_309 | self-separation | 0.00913362 | 0.0137878 | yes |
| TROT_309 | joint margin (rad) | 0.304889 | 0.304919 | yes |

Per-state minima (sampled, G2 nominal; the start state is the descent from the canonical 150 mm STAND, STOPPING includes the recentre):

| Case | Lifecycle state | Samples | Min joint margin [rad] | Min non-foot ground [mm] | Min self-separation [mm] | Min declared-set support margin [mm] |
|---|---|---|---|---|---|---|
| WALK 357 | GAIT_START | 1500 | 0.305 | 2.50 | 13.79 | 1.65 |
| WALK 357 | WALK | 101 | 0.367 | 2.50 | 13.79 | 1.55 |
| WALK 357 | STOPPING | 1899 | 0.305 | 2.48 | 13.79 | 1.10 |
| WALK 357 | STAND | 1 | 0.305 | 2.87 | 13.79 | 98.61 |
| WALK 287 | GAIT_START | 1500 | 0.305 | 2.65 | 13.79 | 2.12 |
| WALK 287 | WALK | 101 | 0.601 | 2.65 | 13.79 | 2.02 |
| WALK 287 | STOPPING | 1899 | 0.305 | 2.64 | 13.79 | 1.59 |
| WALK 287 | STAND | 1 | 0.305 | 2.87 | 13.79 | 98.61 |
| TROT 61 | GAIT_START | 1500 | 0.305 | 2.50 | 13.79 | -2.14 |
| TROT 61 | TROT | 101 | 0.478 | 2.50 | 13.79 | 98.61 |
| TROT 61 | STOPPING | 1899 | 0.305 | 2.47 | 13.79 | -2.99 |
| TROT 61 | STAND | 1 | 0.305 | 2.87 | 13.79 | 98.61 |
| TROT 309 | GAIT_START | 1500 | 0.305 | 2.79 | 13.79 | -1.35 |
| TROT 309 | TROT | 101 | 0.785 | 2.79 | 13.79 | 98.61 |
| TROT 309 | STOPPING | 1899 | 0.305 | 2.78 | 13.79 | -2.69 |
| TROT 309 | STAND | 1 | 0.305 | 2.87 | 13.79 | 98.61 |

Time a swing foot spends within 1 um of the ground on the analytic surface (the interval the legacy band rule treated as undeclared contact; it is the planned approach, and it is longest in the zero-rate stop ending):

| Case | Time with a swing foot within 1 um of the ground (analytic) | Longest continuous interval |
|---|---|---|
| WALK 357 | 354 ms | 270 ms |
| WALK 287 | 350 ms | 274 ms |
| TROT 61 | 310 ms | 270 ms |
| TROT 309 | 342 ms | 300 ms |

Per-foot discrepancy over the lifecycle motions (all four feet share one collision geometry; they differ only in the pitch they pass through):

| Case | Foot | Pitch range [deg] | Mesh minus analytic, stance [um] | Samples below -1 um (5 ms) |
|---|---|---|---|---|
| WALK 357 | LF | -43.5 .. -10.5 | -0.497 .. +4.95 | 0 |
| WALK 357 | RF | -43.5 .. -10.1 | -0.497 .. +5.35 | 0 |
| WALK 357 | RH | -51.5 .. -33.6 | -1.260 .. +2.80 | 38 |
| WALK 357 | LH | -51.5 .. -35.8 | -1.260 .. +2.79 | 38 |
| WALK 287 | LF | -43.5 .. -17.0 | -0.497 .. +4.79 | 0 |
| WALK 287 | RF | -43.5 .. -16.7 | -0.497 .. +4.79 | 0 |
| WALK 287 | RH | -51.5 .. -35.2 | -1.262 .. +2.73 | 44 |
| WALK 287 | LH | -51.5 .. -37.1 | -1.262 .. +2.76 | 44 |
| TROT 61 | LF | -43.5 .. -11.3 | -0.497 .. +5.49 | 0 |
| TROT 61 | RF | -43.5 .. -10.5 | -0.497 .. +5.50 | 0 |
| TROT 61 | RH | -51.5 .. -35.4 | -1.261 .. +2.80 | 59 |
| TROT 61 | LH | -51.5 .. -34.1 | -1.261 .. +2.80 | 40 |
| TROT 309 | LF | -43.5 .. -25.0 | -0.497 .. +2.53 | 0 |
| TROT 309 | RF | -43.5 .. -24.3 | -0.497 .. +2.53 | 0 |
| TROT 309 | RH | -51.5 .. -40.0 | -1.261 .. +2.37 | 93 |
| TROT 309 | LH | -51.5 .. -38.7 | -1.261 .. +2.79 | 62 |

### 7. Complete WALK lifecycle

WALK 357 (80 mm, +10 mm/cycle, 10 mm lift, duty 0.8, 4 mm sway) and WALK 287 (100 mm) complete STAND → WALK → STAND on both contact configurations: no failed or unresolved check; terminal contacts equal the initial contacts in the body frame and the final joint state is the canonical STAND with zero qdot/qddot; no IK branch change; joint margin, non-foot ground clearance, self-separation and quasi-static support margin are positive with certified lower bounds in the table. The transition from the canonical 150 mm STAND to 80 and 100 mm sweeps the rear foot pitch through the windows where G4's mesh test fell below -1 um (worst -1.262 um): that is exactly the legacy rejection and it is inside the declared uncertainty `U`. This verdict is **conditional on the stated contract** (analytic tread, declared `U`) and on the estimated bound constants.

### 8. Complete TROT lifecycle

TROT 61 (80 mm, +20 mm/cycle, duty 0.8) and TROT 309 (120 mm, duty 0.7) complete STAND → TROT → STAND with the same checks. Static support margin is negative in the diagonal phases (diagnostic). **Dynamic stability remains NOT YET PROVEN**; nothing here certifies the physical gait.

## 9. Remaining failures, caveats and limits

* The legacy 1 um mesh rule still fails (G2 nominal: penetration to -1.262 um; registered: no penetration but band events). Retained, not hidden.
* The bound constants are estimated; derivative continuity is sampled (plus the G4 oracle).
* The tread invariant is proved for foot axis tilt 0 (pure fore/aft, flat, level). Lateral and yaw motion tilt the contact axis (G2 nominal strip tilt validity) and are **not** covered.
* Study parameters only; none is promoted to a hardware setting. G4's density-robustness findings about the legacy rule stand; the v2 results do not depend on sampling density for the analytic items.
* The physical truth of the foot (tread diameter, run-out, hub registration, elastomer compliance, ground friction) is unmeasured: the model resolves 5.8 um, the URDF foot joint origin is given to 0.1 mm, and a real rubber tread deforms by far more.
* The cause of the 5.775 um CAD registration is unknown (open question for the CAD owner).

## 10. Regression results and source versions (ALL PASSED)

Run by `validate.py` at HEAD `e69531bdb815acc7307ed9fa3863249808bb7e39` (worktree clean before the run: True). Accepted G4 evidence is byte-identical to the G4 final head; the G4 and G3.5 manifests pass.

| Gate | Result | Summary |
|---|---|---|
| test_gait_build_strict_cpp17 | PASS |  |
| test_gait_run | PASS | GAIT_HOST = PASS: 148291 checks, 0 failures |
| test_contact_mode_build_strict_cpp17 | PASS |  |
| test_contact_mode_run | PASS | CONTACT_MODE_HOST = PASS: 68 checks, 0 failures |
| g4_independent_oracle_current_source | PASS | "status": "PASS", |
| g4_artifact_tests | PASS | Ran 15 tests in 1.394s; OK |
| g4_artifact_manifest_check | PASS | ARTIFACT_MANIFEST OK 57 artifacts; historical generator revisions: 4 |
| g35_pose_audit_tests | PASS | Ran 28 tests in 14.885s; OK |
| g35_artifact_manifest_check | PASS | ARTIFACT_MANIFEST OK 80 artifacts; historical generator revisions: 1 |
| geometry_provenance_recheck | PASS | } |
| g41_contact_tests | PASS | Ran 14 tests in 12.069s; OK |
| lifecycle_replay_bit_identical_2ms | PASS | REPLAY_COMPARE PASS |
| host_motion_runner_g1_g2_g3_g35_g4_g41 | PASS | CONTACT_MODE_HOST = PASS: 68 checks, 0 failures |
| static_audit | PASS | STATIC_AUDIT = PASS |
| test_gait_asan_ubsan_build | PASS |  |
| test_gait_asan_ubsan_run | PASS | GAIT_HOST = PASS: 148291 checks, 0 failures |
| test_contact_mode_asan_ubsan_build | PASS |  |
| test_contact_mode_asan_ubsan_run | PASS | CONTACT_MODE_HOST = PASS: 68 checks, 0 failures |
| git_diff_check_worktree | PASS |  |
| git_diff_check_vs_g4_head | PASS |  |
| accepted_g1_g4_files_unchanged_vs_g4_head | PASS | only G4.1 additions, the host-runner line and the host-oracle include allowlist changed; G1/G2/G3/G3.5/G4 sources, geometry and evidence untouched |

## 11. Outstanding before hardware commissioning

1. A decision on A1 (G2.1) versus the policy-only contract, with CAD-owner input on the 5.775 um registration; versioned regeneration if A1 is chosen.
2. Physical measurement of the foot tread (profile, run-out, position relative to the servo-driven frame) and a compliance/contact model; the offline model cannot resolve micrometres of a rubber tread.
3. An approved actuator-limit source and the semantic-derivative comparison (still not done); timing, torque and thermal limits.
4. A rigid-body dynamics and stabilisation study before any TROT claim; BNO085/state estimation integration.
5. Lateral and yaw contact (axis tilt) validated under the contact contract.
6. Completion and approval of the separate calibration workstream (q0, joint-direction mapping, persistence), the stand authorisation (none exists) and the G3 REST_GROUND route (BODY_ONLY REST_GROUND to STAND remains unvalidated).
7. An independent safety review; hardware commissioning needs explicit authorisation. Nothing in G4.1 authorises any physical motion.
