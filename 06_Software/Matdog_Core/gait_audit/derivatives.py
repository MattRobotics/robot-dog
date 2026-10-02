"""Semantic joint derivative tables for the selected representatives at T = 0.5, 1, 2, 4 s.

Semantic URDF radians only: nothing here is compared with an actuator limit.
q must be independent of the period; qdot scales as 1/T and qddot as 1/T^2. Besides the
production core's analytic derivatives, a central finite difference of the produced q (in
time, per period) independently checks every table row at interior cycle points.
"""
import json
import numpy as np
from core import Core, params, FIELDS
from survey import phases, OUT, save

PERIODS = (0.5, 1.0, 2.0, 4.0)
JOINTS = [f'{l}_{j}' for l in ('lf', 'rf', 'rh', 'lh') for j in ('hip', 'upper', 'lower')]


def table(core, p, n=400):
    rows = {}
    previous = None
    ss = phases(p, n)
    base_q = None
    for T in PERIODS:
        peak_q = np.zeros(12); peak_qd = np.zeros(12); peak_qdd = np.zeros(12); qs = []
        previous = None
        for s in ss:
            f = core.frame(p, s, period=T, previous=previous)
            assert 'q' in f, (p, s, f)
            previous = f
            qs.append(f['q'])
            peak_q = np.maximum(peak_q, abs(f['q'])); peak_qd = np.maximum(peak_qd, abs(f['qdot'])); peak_qdd = np.maximum(peak_qdd, abs(f['qddot']))
        qs = np.array(qs)
        if base_q is None:
            base_q = qs
        # independent time finite differences of q at interior points
        fd_err_v = fd_err_a = 0.0
        for s in (1.123, 1.373, 1.623, 1.873):
            eps = 2e-5
            c = core.frame(p, s, period=T)
            a = core.frame(p, s - eps, period=T, previous=c); b = core.frame(p, s + eps, period=T, previous=c)
            dt = eps * T
            fd_v = (b['q'] - a['q']) / (2 * dt)
            fd_a = (b['qdot'] - a['qdot']) / (2 * dt)
            fd_err_v = max(fd_err_v, float(np.max(abs(fd_v - c['qdot'])) / max(1e-9, np.max(abs(c['qdot'])))))
            fd_err_a = max(fd_err_a, float(np.max(abs(fd_a - c['qddot'])) / max(1e-9, np.max(abs(c['qddot'])))))
        rows[T] = dict(period_s=T, samples=len(ss), peak_abs_q_rad=peak_q.tolist(), peak_qdot_rad_s=peak_qd.tolist(), peak_qddot_rad_s2=peak_qdd.tolist(),
                       max_q_difference_vs_T0p5_rad=float(np.max(abs(qs - base_q))),
                       finite_difference_relative_error=dict(qdot=fd_err_v, qddot=fd_err_a))
    ref = rows[1.0]
    scaling = {}
    for T, r in rows.items():
        qd = np.array(r['peak_qdot_rad_s']); qdd = np.array(r['peak_qddot_rad_s2'])
        scaling[T] = dict(
            max_rel_dev_qdot_times_T=float(np.max(abs(qd * T - np.array(ref['peak_qdot_rad_s']))) / np.max(np.array(ref['peak_qdot_rad_s']))),
            max_rel_dev_qddot_times_T2=float(np.max(abs(qdd * T * T - np.array(ref['peak_qddot_rad_s2']))) / np.max(np.array(ref['peak_qddot_rad_s2']))))
        assert r['max_q_difference_vs_T0p5_rad'] < 1e-12
        assert scaling[T]['max_rel_dev_qdot_times_T'] < 1e-9 and scaling[T]['max_rel_dev_qddot_times_T2'] < 1e-9
        assert r['finite_difference_relative_error']['qdot'] < 1e-4 and r['finite_difference_relative_error']['qddot'] < 5e-3
    return dict(parameters=dict(zip(FIELDS, p)), periods=[rows[T] for T in PERIODS], scaling_check={str(k): v for k, v in scaling.items()})


def main():
    reps = json.loads((OUT / 'representatives.json').read_text())['representatives']
    core = Core()
    out = {}
    for r in reps:
        p = np.array([r['parameters'][k] for k in FIELDS])
        out[r['name']] = dict(id=r['id'], **table(core, p))
        print(r['name'], 'ok', flush=True)
    save('derivative_tables.json', dict(scope='Semantic URDF-radian requirements; q is period independent, qdot ~ 1/T, qddot ~ 1/T^2. No actuator comparison.',
                                        joint_order=JOINTS, cases=out))


if __name__ == '__main__':
    main()
