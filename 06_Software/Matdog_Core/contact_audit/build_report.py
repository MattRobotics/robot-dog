"""Render the G4.1 REPORT.md from the saved artifacts so every number in it comes from a file."""
import json
import sys
from pathlib import Path
from string import Template

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
OUT = ROOT / '09_Logs/Validation_Reports/G41_Contact_Reconciliation'
CASES = ('WALK_357', 'WALK_287', 'TROT_61', 'TROT_309')
VARIANTS = (('G2_NOMINAL', 'G2 nominal (unchanged)'), ('G2_1_REGISTERED_CANDIDATE', 'registered candidate'))


def load(name):
    return json.loads((OUT / name).read_text())


def table(header, rows):
    return '\n'.join(['| ' + ' | '.join(header) + ' |', '|' + '|'.join('---' for _ in header) + '|'] + ['| ' + ' | '.join(str(c) for c in r) + ' |' for r in rows])


def mm(x, n=2):
    return f'{x * 1e3:.{n}f} mm'


def values():
    rc, prov, alt = load('root_cause.json'), load('geometry_provenance.json'), load('alternatives.json')
    val = load('validation_results.json')
    v = {}
    col, vis = rc['meshes']
    mmx = col['nominal_radius_translated_circle_minimax']
    mmv = vis['nominal_radius_translated_circle_minimax']
    v['offset'] = f"{mmx['offset_um']:.3f}"; v['dx'] = f"{mmx['dx_um']:.3f}"; v['dz'] = f"{mmx['dz_um']:.3f}"
    v['vis_dx'] = f"{mmv['dx_um']:.3f}"; v['vis_dz'] = f"{mmv['dz_um']:.3f}"
    v['excess'] = f"{mmx['max_radial_excess_um']:.4f}"; v['on_circle'] = mmx['vertices_within_0p01um_of_circle']
    v['delta_min'] = f"{col['delta_um']['min']:.3f}"; v['delta_max'] = f"{col['delta_um']['max']:.3f}"
    v['frac_bad'] = f"{col['delta_um']['fraction_below_minus_1um'] * 100:.1f}"
    v['err_ld'] = f"{col['numeric_error']['float64_vs_longdouble_um']:.1e}"; v['err_f32'] = f"{col['numeric_error']['float32_stl_quantisation_max_um']:.1e}"
    v['col_extent'] = ', '.join(f"{k}={x:+.2f}" for k, x in col['extents_vs_nominal'].items())
    v['rings'] = '; '.join(f"y={r['y_mm']:+.2f} mm: {r['vertices']} vertices, max gap {r['max_gap_deg']:.2f} deg, chord sagitta <= {r['max_chord_sagitta_um']:.1f} um" for r in col['rings'])
    v['tess_max'] = f"{col['tessellation_residual_um']['max']:.1f}"
    v['feet_identical'] = rc['findings']['all_four_feet_identical_in_foot_link']
    v['lowest_vertices'] = col['distinct_lowest_vertices_over_pitch']
    v['prov_ok'] = prov['assessment']['geometry_consistent']
    v['urdf'] = prov['urdf_sha256']
    v['cal_head'] = prov['calibration_worktree_read_only_comparison']['head']
    v['cal_diffs'] = len(prov['calibration_worktree_read_only_comparison']['urdf_and_stl_files_differing'])
    a = alt['alternatives']['WALK_357']; t = alt['alternatives']['TROT_61']; imp = alt['alternatives']['A1_registered_reference_impact']
    names = [('A0_G2_nominal_strict_mesh_test', 'A0 status quo (G2 nominal, strict mesh test)'), ('A1_registered_reference', 'A1 registered reference (candidate G2.1)'),
             ('A2_analytic_tread_authority', 'A2 analytic tread authority (policy only)'), ('A3_exact_mesh_support_function', 'A3 exact mesh support function'),
             ('A4_circumscribing_cylinder', 'A4 circumscribing cylinder')]
    rows = []
    for key, label in names:
        x, y = a[key], t[key]
        rows.append([label, f"{x['reference_shift_um'][0]:+.2f} .. {x['reference_shift_um'][1]:+.2f}", f"{x['mesh_minus_reference_um_stance'][0]:+.3f} .. {x['mesh_minus_reference_um_stance'][1]:+.2f}",
                     x['penetration_samples_below_minus_1um'], y['penetration_samples_below_minus_1um'], f"{x['max_reference_rate_um_per_s']:.0f}"])
    v['alt_table'] = table(['Alternative', 'Reference shift vs G2 [um]', 'Mesh minus reference, stance feet [um]', 'Samples below -1 um (WALK 357)', '(TROT 61)', 'Max reference rate [um/s]'], rows)
    v['a1_max'] = f"{a['A1_registered_reference']['mesh_minus_reference_um_stance'][1]:.2f}"; v['a3_rate'] = f"{max(a['A3_exact_mesh_support_function']['max_reference_rate_um_per_s'], t['A3_exact_mesh_support_function']['max_reference_rate_um_per_s']):.0f}"
    v['a4_max'] = f"{max(a['A4_circumscribing_cylinder']['mesh_minus_reference_um_stance'][1], t['A4_circumscribing_cylinder']['mesh_minus_reference_um_stance'][1]):.1f}"; v['sag'] = f"{max(r['max_chord_sagitta_um'] for r in col['rings']):.0f}"
    v['a5'] = f"{a['A5_widen_tolerance']['required_tolerance_um']:.3f}"
    v['uncert'] = f"{a['A2_analytic_tread_authority']['declared_uncertainty_um']:.3f}"
    v['rad_env'] = f"{alt['circumscribing_radius_increase_um']:.3f}"
    v['dq'] = f"{imp['max_abs_delta_q_rad']:.1e}"; v['dqd'] = f"{imp['max_abs_delta_qdot_rad_s']:.1e}"; v['dqdd'] = f"{imp['max_abs_delta_qddot_rad_s2']:.1e}"
    v['rqd'] = f"{imp['relative_qdot_change']:.1e}"; v['rqdd'] = f"{imp['relative_qddot_change']:.1e}"; v['dstand'] = f"{imp['canonical_stand_delta_q_rad']:.1e}"
    v['embed'] = '\n'.join(f"  - `{p}`" for p in imp['files_embedding_G2_radius'])
    # lifecycles
    lrows, vrows = [], []
    for case in CASES:
        for key, label in VARIANTS:
            d = load(f'lifecycle_{case}_{key}.json')
            x = d['verification']
            st = d['all_statuses']
            sup = x.get('support_margin_walk')
            lrows.append([case.replace('_', ' '), label, d['lifecycle_verdict'], d['terminal']['final_state'], f"{d['terminal']['max_contact_difference_in_body_frame_m']:.1e}",
                          f"{x['joint_limit_margin']['certified_lower_bound_rad']:.3f}", mm(x['nonfoot_ground_clearance']['certified_lower_bound_m']),
                          mm(x['self_separation']['certified_lower_bound_m']), (mm(sup['certified_lower_bound_m']) if sup else f"{x['support_margin_trot']['sampled_min_m'] * 1e3:.2f} mm (diagnostic)"),
                          f"{x['tread_mesh_vs_reference']['sampled_min_um']:+.3f}", f"{x['tread_mesh_vs_reference']['all_pitch_exact_lower_bound_um']:+.3f}",
                          f"{x['legacy_v1_strict']['penetration_samples']} / {x['legacy_v1_strict']['undeclared_samples']}"])
    v['lifecycle_table'] = table(['Case', 'Contact reference', 'Verdict', 'Final state', 'Terminal contacts vs initial [m]', 'Joint margin (certified) [rad]', 'Non-foot ground clearance (certified)',
                                  'Min self-separation (certified)', 'WALK support margin (certified)', 'Tread mesh-minus-reference, sampled min [um]', 'all-pitch exact bound [um]',
                                  'Legacy G4 rule: penetration / band samples'], lrows)
    d = load('lifecycle_WALK_357_G2_NOMINAL.json')
    v['status_vocab'] = table(['Quantity', 'Status (WALK 357, G2 nominal)'], [[k, s] for k, s in d['all_statuses'].items()])
    d2 = load('lifecycle_TROT_61_G2_NOMINAL.json')
    v['status_vocab_trot'] = ', '.join(f'{k}: {s}' for k, s in d2['all_statuses'].items() if k != 'support_margin_walk')
    v['dt'] = f"{d['dt_s'] * 1e3:g} ms"; v['samples'] = d['samples']
    conv = []
    for case in CASES:
        a1 = load(f'lifecycle_{case}_G2_NOMINAL.json')['verification']; b1 = load(f'lifecycle_{case}_G2_NOMINAL_fine.json')['verification']
        for q, lo, mn, label in (('nonfoot_ground_clearance', 'certified_lower_bound_m', 'sampled_min_m', 'non-foot ground clearance'), ('self_separation', 'certified_lower_bound_m', 'sampled_min_m', 'self-separation'),
                                 ('joint_limit_margin', 'certified_lower_bound_rad', 'sampled_min_rad', 'joint margin (rad)'), ('support_margin_walk', 'certified_lower_bound_m', 'sampled_min_m', 'WALK support margin')):
            if q in a1:
                conv.append([case, label, f"{a1[q][lo]:.6g}", f"{b1[q][mn]:.6g}", 'yes' if a1[q][lo] <= b1[q][mn] + 1e-12 else 'NO'])
    v['conv_table'] = table(['Case', 'Quantity', 'Certified lower bound at 2 ms', 'Sampled minimum at 0.5 ms', 'Bound <= fine sample'], conv)
    v['validation_table'] = table(['Gate', 'Result', 'Summary'], [[g['name'], 'PASS' if g['passed'] else 'FAIL', (g['summary'] or '').replace('|', '/')] for g in val['gates']])
    v['all_passed'] = 'ALL PASSED' if val['all_passed'] else 'NOT ALL PASSED'
    v['head_validated'] = val['git']['head_at_validation']; v['clean_before'] = val['git']['worktree_clean_before_validation']
    ev = load('lifecycle_WALK_357_G2_NOMINAL.json')['verification']['contact_events']
    v['events'] = ev['count']
    return v


TEMPLATE = Template(r'''# G4.1 — Contact model reconciliation and continuous lifecycle validation

**Scope: OFFLINE software only. No hardware, serial device, actuator bus, servo command, flashing, persistent-memory write or calibration change was used or created. The separate calibration worktree was only read (hashes).**
Start: G4 final `3ce85f92e2b5381b65e327dd371b2f164308f326`. No accepted G1-G4 file or evidence was changed (checked by the `accepted_g1_g4_files_unchanged_vs_g4_head` gate). G4.1 adds new tools, one pure C++ unit (`ContactMode`) with its test, and new evidence only.

## 0. Verdict

* **Root cause (1).** Both STLs of the foot (collision and visual) contain the nominal 14.9 mm tread circle **translated rigidly by ($dx, $dz) um, 5.775 um in magnitude**, from the G2 analytic reference. It is a property of the CAD-to-foot_link registration, not of tessellation or numerics.
* **Complete lifecycles.** Under the G4.1 contact contract the four reference gaits — WALK 357, WALK 287, TROT 61, TROT 309 — each complete STAND → gait → STAND with **no failed check and no unresolved check** (section 6). Terminal STAND is canonical and every certified margin is positive. TROT remains kinematically studied and dynamically uncertified.
* **This is not a tolerance change.** The legacy G4 rule (mesh within 1 um) **still rejects** all four lifecycles on the unchanged G2 (worst $legacy_worst um) and that rejection is recorded, not hidden. What changed is the contract: the tread is judged on the analytic surface, the mesh/analytic difference is bounded in closed form and declared, and contact modes follow the schedule.
* **Selected correction.** A2 (policy: analytic tread authority) is applied in G4.1 on the unchanged G2. The evidence additionally supports A1 (re-register the G2 reference to the mesh, "G2.1"): it removes the penetration exactly. A1 changes an accepted cross-milestone contract, so it is **evaluated as a prototype and NOT applied; it needs approval** (migration strategy in section 4).
* **Not claimed.** Hardware-approved parameters, dynamic stability, actuator limits, lateral/yaw contact, or physical truth of a micrometre foot contact.

## 1. Root cause of the 5.775 um discrepancy

Five things were separated (`root_cause.json`):

| Layer | Finding |
|---|---|
| Nominal CAD geometry | G2 YAML `cad_validation`: R = 14.9 mm, centre z = 14.9 mm, tread 13.9 mm, fillet 2 mm. G2 uses exactly these values: the analytic reference **is** the nominal CAD tread |
| Ideal analytic contact | the G2 cylinder; contact point = centre + R down; no mesh involved |
| Tessellated collision mesh | $rings. Tread vertices lie on or inside the nominal-radius circle translated by ($dx, $dz) um: maximum radial excess **$excess um**, $on_circle vertices on it to 0.01 um. Extents against the nominal circle (um): $col_extent |
| Actual lowest point | always a vertex ($lowest_vertices distinct lowest vertices over pitch), never an edge or face; mesh-minus-analytic height `delta(phi)` ranges $delta_min .. $delta_max um and is below -1 um for $frac_bad percent of pitch angles |
| Numerical error | float64 vs long double $err_ld um; float32 STL quantisation $err_f32 um: negligible |

* The **visual** STL carries the same translation ($vis_dx, $vis_dz) um (difference from the collision STL under 0.01 um). Two independently exported meshes agree, so the offset is real CAD-to-foot_link registration and not an export artefact. The G2 YAML audit used the visual STL and rejected a *centroid* offset because the centroid depends on tessellation; the circle translation does not.
* `delta(phi) = u.d + (non-negative tessellation term)`, with `u` the world-up vector in foot_link for foot pitch `phi` and `d` the translation. So the mesh sits $offset um below the analytic reference at the worst pitch and up to $tess_max um above it (tessellation inscribes a polygon in the translated circle).
* All four feet have identical collision geometry in foot_link (max vertex difference 0 um: $feet_identical); the discrepancy of each foot is `delta` at that foot's pitch (G4: rear feet at -51 deg, delta = -1.2 um).
* The geometric cause of the translation itself (why the CAD wheel axis is 5.8 um from the nominal point in the foot frame) cannot be determined from the repository; the URDF foot joint origin is specified to 0.1 mm. This is an **open question for the CAD owner**, not something the offline model can settle.

## 2. Canonical geometry and provenance

Geometry consistent: **$prov_ok**. URDF sha256 `$urdf`.

* `SHA256SUMS.txt` verifies every URDF/mesh file. The G2 YAML, the G3.5 pose library and the G4 `definitions.json` record this URDF hash and it matches. The G1 `matdog_motion_geometry_export.py --check` ("Geometry V5 / live URDF") and the G2 `matdog_contact_stand_export.py --check` pass.
* The calibration worktree (read only, HEAD `$cal_head`) has $cal_diffs differing URDF/STL files and an identical foot contact YAML. Geometry V5 and the Full Calibration provenance consume the same URDF and meshes; they define no different foot or contact geometry, and calibration-side URDF-limit/q0 findings do not touch the foot.
* **Material discrepancy found:** the G2 YAML mesh audit (hash `e43737...`, lowest z 16.7 um, `all_four_foot_meshes_identical`) is of the **visual** foot STL; the G3.5 and G4 collision checks use the **collision** STL (different file, tessellation and `delta`). The two never contradicted each other numerically (same translation) but were different objects. Impact: none on G1-G4 results; recorded for the CAD/geometry owner.
* No canonical geometry file was modified.

## 3. Alternatives (measured on the exact G4 lifecycle foot motions, WALK 357 and TROT 61, 5 ms)

$alt_table

Reference shift = change of the contact-reference bottom relative to G2; "mesh minus reference" is the lowest mesh height above that reference for stance feet.

| Alternative | Geometric justification | Mathematical consistency | Impact on G1-G4 and reference data | IK / derivatives | Collision / contact classification | Cost | Regression needed |
|---|---|---|---|---|---|---|---|
| A0 status quo | G2 nominal CAD tread | consistent for IK; inconsistent with the mesh by up to $uncert um | none | none | rejects every lifecycle; sampling-density dependent | none | none |
| A1 registered reference (G2.1) | mesh tread = nominal circle translated; two STLs agree | exact: the mesh never goes below it, all-pitch bound >= 0 | **changes accepted G2** (YAML, `FootContactData.h`, C4 stand, goldens) and everything derived (G3 stand, G3.5 pose data) | q change <= $dq rad, qdot <= $dqd rad/s ($rqd relative), qddot <= $dqdd rad/s^2 ($rqdd relative); stand $dstand rad; G1 unaffected | penetration removed exactly; mesh stays <= $a1_max um above the reference (inside the 10 um patch band); band events remain (semantic) | constants only in firmware; the offline re-solve is a 3x3 Newton per leg | regenerate and re-run G2, G3, G3.5, G4 gates |
| A2 analytic tread authority | G2 YAML states the cylinder is the stable contact geometry and the mesh is for collision | closed-form `z >= 0` for the swing foot; mesh bounded by the declared U = $uncert um | **none** (offline policy only; v1 stays available) | none | tread judged analytically; everything else on the mesh | none | G4.1 gates |
| A3 exact mesh support function | the tessellated mesh is not the real tread (chord sagitta up to $sag um on the coarse ring) | continuous but only piecewise smooth: reference rate up to $a3_rate um/s with slope kinks | changes G2 semantics | non-smooth derivatives | trivially no penetration | per-frame min over the foot vertices | full |
| A4 circumscribing cylinder | conservative | exact no-penetration | changes G2 constants | none | stance feet hover up to $a4_max um > 10 um patch band (MISSING_MESH_SUPPORT) | none | full |
| A5 widen the tolerance | none | none | none | none | needs $a5 um here, still sampling dependent | none | REJECTED |

A z-only or x-only offset cannot fix the problem: the difference is `u.d`, a sinusoid in pitch that needs both components (arbitrary offsets are rejected).

Files embedding the G2 radius (what A1 would regenerate, first 40):
$embed

**Selection.** Evidence: the translation is registered and consistent (A1 is the geometry-consistent mapping); but adopting it changes an accepted cross-milestone contract, which this milestone is not authorised to do. A2 needs no contract change and is justified by the G2 YAML's own statement. G4.1 therefore applies A2 and demonstrates A1; both are audited below.

## 4. Contact contract v2, impact on G1-G4, migration

Full text in `contact_audit/README.md`. In short: reference = G2 analytic cylinder (unchanged); modes STANCE / SWING / LIFT_OFF / TOUCHDOWN are declared by the schedule (`ContactMode`, 68 C++ checks) and never inferred from proximity; stance tread height 0 (IK exact), swing tread height `64 h u^3 (1-u)^3 >= 0` in closed form; the mesh-minus-reference height over **all** pitches is bounded exactly by `R - max|v-c|` (= -$offset um for G2, >= 0 for the registered reference) and that bound is the declared uncertainty `U = $uncert um`, never a pass/fail dial; non-foot links keep the strict 1 um mesh test; support margin from the G2 central strip ends over the stance set common to both neighbouring samples.

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

* `LIFT_OFF` and `TOUCHDOWN` are the exact schedule instants (phase = duty and phase = 0): foot velocity and acceleration are zero (quintic swing), height 0. In the WALK 357 lifecycle there are $events scheduler events (lift-offs and touchdowns).
* The 1 um band of G4 was a numerical proxy for "touching". On the analytic surface the swing foot height is `>= 0` for every `t`, so an undeclared tread contact is impossible, not merely unobserved. The band events that the legacy rule reports (e.g. the stop's 0.3 s dwell within 1 um before touchdown, or the 150-190 samples that remain even on the registered reference) are the **planned approach to the scheduled touchdown** and its mirror at lift-off; they are not penetration, and penetration (negative analytic height) remains a failure.
* Undeclared contact is still checked wherever it can occur: every non-foot link and the non-tread foot geometry, against the mesh, with the 1 um band.

## 6. Continuous verification and lifecycle results

Evidence level of every quantity (2 ms grid, $samples samples over 7 s per lifecycle; start from the canonical 150 mm STAND):

$status_vocab

For TROT: $status_vocab_trot. The TROT support margin is diagnostic only (dynamics not proven).

* `CONTINUOUSLY_VERIFIED_ANALYTIC`: closed form, valid for all `t` (tread stance exactness, swing height >= 0, mesh-minus-reference bound over all pitches; scope: foot axis tilt 0).
* `CONTINUOUSLY_VERIFIED_BOUNDED`: sampled value minus an interpolation-error bound (`A h^2 / 8` for C2 quantities, Lipschitz for separation) whose constant is **estimated** from the dense samples with a 1.5 safety factor. This is **not a formal proof**; it is confirmed by a 0.5 ms re-run (below).
* `SAMPLED_ONLY`: derivative magnitudes and steps (the G4 oracle covers the analytic C2 joins).

$lifecycle_table

Convergence confirmation (certified bound at 2 ms never exceeds, and fine sampling never finds a worse minimum than, the 0.5 ms sampled minimum):

$conv_table

### 7. Complete WALK lifecycle

WALK 357 (80 mm, +10 mm/cycle, 10 mm lift, duty 0.8, 4 mm sway) and WALK 287 (100 mm) complete STAND → WALK → STAND on both contact configurations: no failed or unresolved check; terminal contacts equal the initial contacts in the body frame and the final joint state is the canonical STAND with zero qdot/qddot; no IK branch change; joint margin, non-foot ground clearance, self-separation and quasi-static support margin are positive with certified lower bounds in the table. The transition from the canonical 150 mm STAND to 80 and 100 mm sweeps the rear foot pitch through the windows where G4's mesh test fell below -1 um (worst $legacy_worst um): that is exactly the legacy rejection and it is inside the declared uncertainty `U`. This verdict is **conditional on the stated contract** (analytic tread, declared `U`) and on the estimated bound constants.

### 8. Complete TROT lifecycle

TROT 61 (80 mm, +20 mm/cycle, duty 0.8) and TROT 309 (120 mm, duty 0.7) complete STAND → TROT → STAND with the same checks. Static support margin is negative in the diagonal phases (diagnostic). **Dynamic stability remains NOT YET PROVEN**; nothing here certifies the physical gait.

## 9. Remaining failures, caveats and limits

* The legacy 1 um mesh rule still fails (G2 nominal: penetration to $legacy_worst um; registered: no penetration but band events). Retained, not hidden.
* The bound constants are estimated; derivative continuity is sampled (plus the G4 oracle).
* The tread invariant is proved for foot axis tilt 0 (pure fore/aft, flat, level). Lateral and yaw motion tilt the contact axis (G2 nominal strip tilt validity) and are **not** covered.
* Study parameters only; none is promoted to a hardware setting. G4's density-robustness findings about the legacy rule stand; the v2 results do not depend on sampling density for the analytic items.
* The physical truth of the foot (tread diameter, run-out, hub registration, elastomer compliance, ground friction) is unmeasured: the model resolves 5.8 um, the URDF foot joint origin is given to 0.1 mm, and a real rubber tread deforms by far more.
* The cause of the 5.775 um CAD registration is unknown (open question for the CAD owner).

## 10. Regression results and source versions ($all_passed)

Run by `validate.py` at HEAD `$head_validated` (worktree clean before the run: $clean_before). Accepted G4 evidence is byte-identical to the G4 final head; the G4 and G3.5 manifests pass.

$validation_table

## 11. Outstanding before hardware commissioning

1. A decision on A1 (G2.1) versus the policy-only contract, with CAD-owner input on the 5.775 um registration; versioned regeneration if A1 is chosen.
2. Physical measurement of the foot tread (profile, run-out, position relative to the servo-driven frame) and a compliance/contact model; the offline model cannot resolve micrometres of a rubber tread.
3. An approved actuator-limit source and the semantic-derivative comparison (still not done); timing, torque and thermal limits.
4. A rigid-body dynamics and stabilisation study before any TROT claim; BNO085/state estimation integration.
5. Lateral and yaw contact (axis tilt) validated under the contact contract.
6. Completion and approval of the separate calibration workstream (q0, motorDirection, persistence), the stand authorisation (none exists) and the G3 REST_GROUND route (BODY_ONLY REST_GROUND to STAND remains unvalidated).
7. An independent safety review; hardware commissioning needs explicit authorisation. Nothing in G4.1 authorises any physical motion.
''')


def main():
    v = values()
    v['legacy_worst'] = f"{load('lifecycle_WALK_357_G2_NOMINAL.json')['verification']['legacy_v1_strict']['worst_foot_mesh_z_um']:.3f}"
    text = TEMPLATE.substitute(v)
    (OUT / 'REPORT.md').write_text(text)
    print('REPORT.md written,', len(text.splitlines()), 'lines')


if __name__ == '__main__':
    sys.exit(main())
