"""Quantify the G2 analytic-cylinder vs canonical-foot-mesh contact discrepancy.

Reads preserved saved frames only; no survey is repeated. G2 contact semantics, the
1 um ground tolerance and the canonical meshes are unchanged and used as-is.

For a foot whose analytic G2 contact reference lies exactly on the ground, the lowest mesh
vertex sits at  delta(phi) = R + min_v u(phi).(v - c)  relative to the ground, where
c = (0,0,R) is the analytic cylinder centre in foot_link, R its radius and u the world-up
vector in foot_link, u = (sin phi, 0, cos phi) for foot pitch phi. delta depends only on
the rigid foot pitch.
"""
import json
import sys
from pathlib import Path
import numpy as np
from core import ROOT
sys.path.insert(0, str(ROOT / '06_Software/Matdog_Core/pose_audit'))
from model import Model, LEGS, TOL
OUT = ROOT / '09_Logs/Validation_Reports/G4_Gait_Envelope'
R = 0.0149  # G2 rigid cylinder radius (MATDOG_FOOT_CONTACT_GEOMETRY.yaml); asserted below
UM = 1e6


def pitch_deg(tf):
    u = tf[:3, :3].T @ np.array([0., 0., 1.])
    return float(np.degrees(np.arctan2(u[0], u[2]))), float(np.degrees(np.arcsin(abs(u[1]))))


class Foot:
    def __init__(self, model):
        from matdog_foot_contact import load_foot_contact_model
        g = load_foot_contact_model(ROOT)
        assert abs(g.cylinder_radius_m - R) < 1e-15 and np.allclose(g.cylinder_center_in_foot_link_m, [0, 0, R])
        assert np.allclose(g.cylinder_axis_in_foot_link_unit, [0, 1, 0])
        meshes = [model.meshes[l + '_foot_link'].vertices for l in LEGS]
        key = lambda v: v[np.lexsort(v.T[::-1])]
        spread = max(float(np.abs(key(meshes[0]) - key(v)).max()) for v in meshes)
        assert spread < 1e-12, 'foot meshes must coincide in foot_link: %g' % spread
        self.v = meshes[0]
        self.x = self.v[:, 0]
        self.z = self.v[:, 2] - R

    def delta(self, phi_deg):
        phi = np.radians(np.atleast_1d(phi_deg))
        return (R + np.min(np.sin(phi)[:, None] * self.x[None] + np.cos(phi)[:, None] * self.z[None], axis=1))

    def ring_fits(self):
        yr = np.round(self.v[:, 1], 6)
        rings = []
        for y in np.unique(yr):
            q = self.v[yr == y]
            P = np.c_[q[:, 0], q[:, 2]]
            if len(P) < 20:
                continue
            A = np.c_[2 * P, np.ones(len(P))]
            sol = np.linalg.lstsq(A, (P ** 2).sum(1), rcond=None)[0]
            cx, cz = sol[:2]
            rad = float(np.sqrt(sol[2] + cx ** 2 + cz ** 2))
            res = np.hypot(*(P - [cx, cz]).T) - rad
            rings.append(dict(y_mm=float(y * 1e3), vertices=len(P), centre_dx_um=float(cx * UM), centre_dz_um=float((cz - R) * UM),
                              radius_minus_R_um=float((rad - R) * UM), residual_pk_pk_um=float(np.ptp(res) * UM)))
        return rings


def mesh_characterization(foot):
    rings = [r for r in foot.ring_fits() if abs(r['radius_minus_R_um']) < 50 and r['residual_pk_pk_um'] < 5]
    dx = float(np.median([r['centre_dx_um'] for r in rings])); dz = float(np.median([r['centre_dz_um'] for r in rings]))
    phis = np.arange(-180, 180, 0.01)
    d = foot.delta(phis) * UM
    circle = dx * np.sin(np.radians(phis)) + dz * np.cos(np.radians(phis))  # ideal ring with the fitted centre offset
    bad = phis[d < -TOL * UM]
    sweeps = {}
    for name, lo, hi in (('rear_start_sweep', -51.6, -38.9), ('front_start_sweep', -43.6, -20.6)):
        k = (phis >= lo) & (phis <= hi)
        sweeps[name] = dict(pitch_deg=[lo, hi], delta_min_um=float(d[k].min()), delta_max_um=float(d[k].max()),
                            fraction_below_minus_1um=float((d[k] < -1).mean()))
    runs = np.split(bad, np.where(np.diff(bad) > 0.011)[0] + 1) if len(bad) else []
    return dict(
        usable_fit_rings=len(rings), mesh_circle_centre_offset_um=dict(dx=dx, dz=dz, magnitude=float(np.hypot(dx, dz))),
        radius_minus_G2_R_um=float(np.median([r['radius_minus_R_um'] for r in rings])),
        full_tessellation_note='tread rings carry irregular 94/482-vertex triangulation; fillet rings 55 vertices (6.545 deg)',
        delta_um=dict(min=float(d.min()), max=float(d.max()), at_min_pitch_deg=float(phis[d.argmin()])),
        ideal_offset_circle_um=dict(min=float(circle.min()), max=float(circle.max())),
        tessellation_residual_um=dict(min=float((d - circle).min()), max=float((d - circle).max())),
        fraction_of_pitch_angles_below_minus_1um=float((d < -1).mean()),
        pitch_intervals_below_minus_1um_deg=[[round(float(r[0]), 2), round(float(r[-1]), 2)] for r in runs if -70 < r[0] < 0],
        lifecycle_start_sweeps=sweeps)


def frames_of(case):
    return case['frames']


def stance_pitch_runs(model, case):
    """Per-leg stance runs of foot pitch (deg) from saved frames; each run is one continuous contact."""
    runs = {l: [] for l in LEGS}; cur = {l: None for l in LEGS}
    for f in case['frames']:
        tf = model.fk(f['q'], np.array(f['body']))
        for i, l in enumerate(LEGS):
            if f['leg_phase'][i][1]:
                if cur[l] is None:
                    cur[l] = []; runs[l].append(cur[l])
                cur[l].append(pitch_deg(tf[l + '_foot_link'])[0])
            else:
                cur[l] = None
    return runs


def pitch_interval_min_delta_um(foot, runs):
    """Worst mesh-minus-analytic height over every pitch a stance run sweeps through (exact mesh function)."""
    worst = np.inf
    for rs in runs.values():
        for run in rs:
            grid = np.append(np.arange(min(run), max(run), 0.002), max(run))
            worst = min(worst, float(foot.delta(grid).min()) * UM)
    return worst


def analyze(model, foot):
    tfs = {}
    out = dict(steady=[], lifecycle=[])
    maxerr = 0.0
    names = (('full_cases.json', 'full'), ('refinement_full.json', 'refi'))
    for fname, tag in names:
        for case in json.loads((OUT / fname).read_text())['cases']:
            runs = {l: [] for l in LEGS}; cur = {l: None for l in LEGS}
            dmin = None; worst_foot = 0.0
            for f in case['frames']:
                tf = model.fk(f['q'], np.array(f['body']))
                for i, l in enumerate(LEGS):
                    stance = bool(f['leg_phase'][i][1])
                    phi, tilt = pitch_deg(tf[l + '_foot_link'])
                    meas = (f['ground_min_m'][l + '_foot_link'] - f['contacts_world_m'][i][2]) * UM
                    if abs(tilt) < 1e-6 and f['contacts_world_m'][i][2] < 1e-9 and stance:
                        maxerr = max(maxerr, abs(meas - float(foot.delta(phi)[0]) * UM))
                    if stance:
                        dmin = meas if dmin is None else min(dmin, meas)
                        if cur[l] is None: cur[l] = []; runs[l].append(cur[l])
                        cur[l].append(phi)
                    else:
                        cur[l] = None
            cont = []
            for l in LEGS:
                for run in runs[l]:
                    grid = np.append(np.arange(min(run), max(run), 0.002), max(run))
                    cont.append(float(foot.delta(grid).min()) * UM)
            p = case['parameters']
            out['steady'].append(dict(
                source=tag, id=case['id'], type='WALK' if p['type'] == 0 else 'TROT', height_m=p['height_m'], advance_x_m=p['advance_x_m'],
                raw_classification=case['classification'], sampled_min_stance_delta_um=dmin,
                stance_pitch_range_deg=[min(min(r) for rs in runs.values() for r in rs), max(max(r) for rs in runs.values() for r in rs)],
                pitch_interval_min_delta_um=min(cont),
                pitch_interval_check='FAIL_BELOW_-1um' if min(cont) < -TOL * UM else 'OK'))
    for case in json.loads((OUT / 'lifecycle_audit.json').read_text())['cases']:
        rows = []
        for f in case['frames']:
            bad = [e for e in f['geometric_errors']]
            if not bad:
                continue
            tf = model.fk(f['q'], np.array(f['body']))
            for i, l in enumerate(LEGS):
                phi, _ = pitch_deg(tf[l + '_foot_link'])
                ana = f['contacts_world_m'][i][2] * UM; mesh = f['ground_min_m'][l + '_foot_link'] * UM
                stance = l in f['active_feet']
                mine = [e for e in bad if e.endswith(l + '_foot_link')]
                if mine or (stance and mesh - ana < 0):
                    rows.append(dict(time_s=f['time_s'], state=f['state'], leg=l, role='STANCE' if stance else 'SWING', analytic_contact_z_um=ana,
                                     mesh_min_z_um=mesh, mesh_minus_analytic_um=mesh - ana, pitch_deg=phi,
                                     predicted_delta_um=float(foot.delta(phi)[0]) * UM, failure=mine))
        out['lifecycle'].append(dict(type=case['type'], failure_counts=case['failure_counts'], rows=rows,
                                     worst_mesh_ground_um=min(r['mesh_min_z_um'] for r in rows)))
    out['max_abs_error_measured_vs_predicted_um'] = maxerr
    return out


def main():
    model = Model(); foot = Foot(model)
    mesh = mesh_characterization(foot)
    data = analyze(model, foot)
    bad_cases = [s for s in data['steady'] if s['pitch_interval_check'] != 'OK']
    result = dict(
        purpose='Why the strict 1 um mesh/contact policy rejects the saved lifecycles and some steady cases',
        unchanged=['G2 analytic eccentric-cylinder contact reference', '1 um ground tolerance', 'canonical collision meshes'],
        foot_mesh=mesh, measured_vs_prediction=dict(max_abs_error_um=data['max_abs_error_measured_vs_predicted_um'],
            meaning='delta(phi) computed from the foot mesh alone reproduces every saved stance mesh-minus-analytic offset'),
        steady_cases=data['steady'],
        steady_sampled_pass_but_pitch_interval_fail=[s['id'] for s in bad_cases if s['raw_classification'] == ['KINEMATICALLY_VALID']],
        lifecycle=data['lifecycle'])
    (OUT / 'contact_discrepancy.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    print(json.dumps(dict(foot_mesh=mesh, max_err=data['max_abs_error_measured_vs_predicted_um'],
                          sampled_pass_but_interval_fail=result['steady_sampled_pass_but_pitch_interval_fail']), indent=1))


if __name__ == '__main__':
    main()
