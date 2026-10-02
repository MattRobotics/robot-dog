"""Quantitative comparison of contact-model alternatives on the exact G4 lifecycle foot motions.

For each alternative the contact reference bottom sits at some height b(phi) relative to the G2 reference bottom for a
foot at pitch phi; the mesh lowest point sits at delta(phi) above the G2 bottom. The mesh-minus-reference height is
delta - b, the IK reference shift is b, and smoothness is how b varies along the motion.

  A0  G2 nominal reference, mesh ground test at 1 um           b = 0
  A1  registered reference (nominal circle translated by d)    b = u.d - dR
  A2  G2 nominal reference, tread judged analytically          b = 0 (policy change only)
  A3  exact mesh support function as the reference             b = delta(phi)
  A4  circumscribing cylinder (all vertices inside)            b = -(Renv - R)
  A5  widen the strict tolerance                               b = 0, tolerance = worst observed penetration (REJECTED)
Nothing here changes an accepted contract; the registered reference exists only in G4.1 tools.
"""
import sys
import time
import numpy as np
from common import OUT, UM, LEGS, save, delta_um
from core import params, lifecycle
from survey import FIELDS  # noqa: F401  gait_audit survey first
import contact_model as cm
import lifecycle_v2 as lc


def foot_pitches(ctx, frames, q_override=None):
    out = np.zeros((len(frames), 4))
    for k, f in enumerate(frames):
        q = f['q'] if q_override is None else q_override[k]
        tf = ctx.model.fk(q, np.array(f['body']))
        for i, l in enumerate(LEGS):
            u = tf[l + '_foot_link'][:3, :3].T @ np.array([0.0, 0.0, 1.0])
            out[k, i] = np.degrees(np.arctan2(u[0], u[2]))
    return out


def main():
    ctx = lc.Context()
    nom, reg = ctx.nominal, ctx.registered
    tread = ctx.treads['nominal'].all
    d_vec = (reg.center - nom.center)[[0, 2]]
    dR = reg.radius - nom.radius
    wxz = np.hypot(tread[:, 0] - nom.center[0], tread[:, 2] - nom.center[2])
    r_env = float(wxz.max())
    results = {}
    for name in ('WALK_357', 'TROT_61'):
        p = lc.CASES[name]
        times = np.round(np.arange(0, 7.0 + 1e-9, 0.005), 9)
        frames = lifecycle(ctx.core, p, times)
        pit = foot_pitches(ctx, frames)
        dl = np.stack([delta_um(tread, nom.center, nom.radius, pit[:, i]) for i in range(4)], axis=1)           # mesh lowest above G2 bottom (um)
        u = np.stack([np.sin(np.radians(pit)), np.cos(np.radians(pit))], axis=-1)
        b1 = (u @ d_vec) * UM - dR * UM
        b3 = dl
        b4 = np.full_like(dl, -(r_env - nom.radius) * UM)
        st = np.array([[f['leg_phase'][i][1] != 0 or f['leg_phase'][i][2] != 0 for i in range(4)] for f in frames])
        dt = 0.005

        def stats(b):
            rel = dl - b
            return dict(reference_shift_um=[float(b.min()), float(b.max())], mesh_minus_reference_um_all=[float(rel.min()), float(rel.max())],
                        mesh_minus_reference_um_stance=[float(rel[st].min()), float(rel[st].max())], max_reference_rate_um_per_s=float(np.abs(np.diff(b, axis=0)).max() / dt),
                        penetration_samples_below_minus_1um=int((rel < -1.0).sum()))
        results[name] = {
            'A0_G2_nominal_strict_mesh_test': dict(**stats(np.zeros_like(dl)), role='reproduces the G4 rejection'),
            'A1_registered_reference': dict(**stats(b1), role='candidate G2.1 contract'),
            'A2_analytic_tread_authority': dict(**stats(np.zeros_like(dl)), declared_uncertainty_um=float(np.hypot(*d_vec) * UM), role='policy only, G2 constants unchanged'),
            'A3_exact_mesh_support_function': dict(**stats(b3), role='mesh is the reference'),
            'A4_circumscribing_cylinder': dict(**stats(b4), radius_increase_um=float((r_env - nom.radius) * UM), role='conservative envelope'),
            'A5_widen_tolerance': dict(required_tolerance_um=float(-(dl[st].min())), role='REJECTED: arbitrary, would still be sampling dependent'),
        }
    # impact on IK and derivatives of the registered reference (A1): re-solve the lifecycle joint angles against it
    p = lc.CASES['WALK_357']
    times = np.round(np.arange(0, 7.0 + 1e-9, 0.005), 9)
    frames = lifecycle(ctx.core, p, times)
    t0 = time.time()
    q1 = np.array([lc.resolve_to_reference(ctx, f['q'], np.array(f['body']), f['feet'], reg) for f in frames])
    solve_s = (time.time() - t0) / len(frames)
    q0 = np.array([f['q'] for f in frames])
    dq = q1 - q0
    dqdot = np.gradient(dq, times, axis=0)
    dqddot = np.gradient(dqdot, times, axis=0)
    qd0 = np.array([f['qdot'] for f in frames]); qdd0 = np.array([f['qddot'] for f in frames])
    # canonical STAND: how far the G2 stand targets move
    stand = frames[0]
    qs = lc.resolve_to_reference(ctx, stand['q'], np.array(stand['body']), stand['feet'], reg)
    # accepted data that embeds the G2 reference constants (what a contract change would have to regenerate)
    import subprocess
    grep = subprocess.run(['grep', '-rIl', '--include=*.py', '--include=*.cpp', '--include=*.h', '--include=*.yaml', '--include=*.json', '--include=*.md', '0.0149', str(lc.cm.__file__).rsplit('/06_Software', 1)[0] + '/06_Software',
                           str(lc.cm.__file__).rsplit('/06_Software', 1)[0] + '/05_Firmware'], capture_output=True, text=True).stdout.split()
    results['A1_registered_reference_impact'] = dict(
        max_abs_delta_q_rad=float(np.abs(dq).max()), max_abs_delta_qdot_rad_s=float(np.abs(dqdot).max()), max_abs_delta_qddot_rad_s2=float(np.abs(dqddot).max()),
        relative_qdot_change=float(np.abs(dqdot).max() / np.abs(qd0).max()), relative_qddot_change=float(np.abs(dqddot).max() / np.abs(qdd0).max()),
        canonical_stand_delta_q_rad=float(np.abs(qs - np.array(stand['q'])).max()),
        newton_seconds_per_frame=float(solve_s), contact_point_shift_um_bound=float(np.hypot(*d_vec) * UM + abs(dR) * UM),
        files_embedding_G2_radius=sorted(set(x.split('/06_Software/')[-1] if '/06_Software/' in x else x.split('/05_Firmware/')[-1] for x in grep))[:40])
    save('alternatives.json', dict(
        purpose='Compare contact-model alternatives on the exact G4 lifecycle foot motions (WALK 357 and TROT 61, 5 ms)',
        registered_offset_um=float(np.hypot(*d_vec) * UM), registered_radius_change_nm=float(dR * 1e9), circumscribing_radius_increase_um=float((r_env - nom.radius) * UM),
        alternatives=results))
    for n in ('WALK_357', 'TROT_61'):
        for k, v in results[n].items():
            print(n, k, {kk: vv for kk, vv in v.items() if kk != 'role'})
    print(results['A1_registered_reference_impact'])


if __name__ == '__main__':
    sys.exit(main())
