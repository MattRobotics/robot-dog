"""Independent verification of the G5-A frame conventions and the tilted contact IK, plus the STAND tilt envelope.

Independent means: URDF forward kinematics and the G2 Python contact model from the G3.5 offline `Model` (world frame,
ground normal +Z, no base-frame normal), numpy/scipy numerical solvers, and quaternion algebra written separately from the
C++ code. Nothing here reuses the C++ IK; it is only compared against it.
"""
import json
import sys
import numpy as np
from scipy.optimize import least_squares
from core import Lib, OUT, ROOT, LEGS
sys.path.insert(0, str(ROOT / '06_Software/Matdog_Core/pose_audit'))
from model import Model  # noqa: E402

DEG = np.pi / 180


def rx(a): c, s = np.cos(a), np.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def ry(a): c, s = np.cos(a), np.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def rz(a): c, s = np.cos(a), np.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def quat_from_matrix(R):
    """Shepperd's method, independent of the C++ and of the test composition."""
    t = np.trace(R)
    if t > 0:
        s = 2 * np.sqrt(t + 1); q = [s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    else:
        i = int(np.argmax(np.diag(R))); j, k = (i + 1) % 3, (i + 2) % 3
        s = 2 * np.sqrt(1 + R[i, i] - R[j, j] - R[k, k]); q = [0, 0, 0, 0]
        q[0] = (R[k, j] - R[j, k]) / s; q[1 + i] = s / 4; q[1 + j] = (R[j, i] + R[i, j]) / s; q[1 + k] = (R[k, i] + R[i, k]) / s
    return np.array(q)


def pose(roll, pitch, yaw, height):
    T = np.eye(4); T[:3, :3] = rz(yaw) @ ry(pitch) @ rx(roll); T[:3, 3] = [0, 0, height]; return T


def frames(lib, model):
    rows = []
    # 1. quaternion -> tilt over a dense set including yaw and the q/-q ambiguity
    worst = 0.0
    rng = np.random.default_rng(11)
    for _ in range(2000):
        r, p, y = rng.uniform(-.5, .5), rng.uniform(-.5, .5), rng.uniform(-np.pi, np.pi)
        q = quat_from_matrix(rz(y) @ ry(p) @ rx(r))
        for sign in (1, -1):
            t = lib.tilt(sign * q)
            worst = max(worst, abs(t[0] - r), abs(t[1] - p))
    # 2. sign conventions against the URDF geometry itself (not against the library helpers)
    q0 = np.zeros(12)
    tf_roll = model.fk(q0, pose(.1, 0, 0, .2)); tf_flat = model.fk(q0, pose(0, 0, 0, .2))
    tf_pitch = model.fk(q0, pose(0, .1, 0, .2))
    z = lambda tf, link: tf[link][2, 3]
    conv = dict(
        positive_roll_left_hip_rises=bool(z(tf_roll, 'lf_hip_link') > z(tf_flat, 'lf_hip_link') and z(tf_roll, 'rf_hip_link') < z(tf_flat, 'rf_hip_link')),
        positive_pitch_front_goes_down=bool(z(tf_pitch, 'lf_hip_link') < z(tf_flat, 'lf_hip_link') and z(tf_pitch, 'lh_hip_link') > z(tf_flat, 'lh_hip_link')),
        lf_hip_is_left_and_front=bool(tf_flat['lf_hip_link'][1, 3] > 0 and tf_flat['lf_hip_link'][0, 3] > 0),
        rh_hip_is_right_and_rear=bool(tf_flat['rh_hip_link'][1, 3] < 0 and tf_flat['rh_hip_link'][0, 3] < 0))
    # 3. the IMU orientation of a base_link pose, read back through the contract, equals the pose roll/pitch
    pose_rt = max(max(abs(lib.tilt(quat_from_matrix(pose(r, p, y, .2)[:3, :3]))[0] - r), abs(lib.tilt(quat_from_matrix(pose(r, p, y, .2)[:3, :3]))[1] - p))
                  for r, p, y in [(.1, .05, 1.0), (-.2, .3, -2.0), (.0, -.25, 3.0)])
    return dict(quaternion_tilt_max_error_rad=worst, samples=4000, conventions=conv, pose_to_quaternion_to_tilt_max_error_rad=pose_rt,
                all_conventions_hold=all(conv.values()) and worst < 1e-12 and pose_rt < 1e-12)


def world_contacts(model, q, T):
    return model.contacts(model.fk(q, T))


def axis_tilt(model, q, T):
    tf = model.fk(q, T)
    return [float(np.arcsin(min(1, abs(tf[l + '_foot_link'][2, 1])))) for l in LEGS]


def independent_solution(model, T, targets, q0):
    """Damped least squares on the URDF/G2-Python world contact; legs are independent (4 x 3 unknowns)."""
    lo, hi = model.limits[:, 0], model.limits[:, 1]
    res = least_squares(lambda q: (world_contacts(model, q, T) - targets).ravel(), np.clip(q0, lo + 1e-9, hi - 1e-9), bounds=(lo, hi), gtol=1e-14, ftol=1e-14, xtol=1e-14)
    return res.x, float(np.abs(res.fun).max())


def exists_any(model, T, targets, rng, starts=40):
    """Multi-start search for ANY in-limit joint solution (no branch preference), to separate solver misses from true infeasibility."""
    lo, hi = model.limits[:, 0], model.limits[:, 1]
    best = np.inf
    for leg in range(4):
        sl = slice(3 * leg, 3 * leg + 3)
        ok = False
        for _ in range(starts):
            q = rng.uniform(lo, hi)
            def f(x):
                qq = q.copy(); qq[sl] = x
                return (world_contacts(model, qq, T)[leg] - targets[leg])
            r = least_squares(f, q[sl], bounds=(lo[sl], hi[sl]), gtol=1e-13, ftol=1e-13, xtol=1e-13)
            if np.abs(r.fun).max() < 1e-8:
                qq = q.copy(); qq[sl] = r.x; ok = True; break
        if not ok:
            return False
    return True


def ik_and_envelope(lib, model):
    contacts, seeds, height, _ = lib.stand()
    rng = np.random.default_rng(5)
    checks = []
    for roll, pitch in [(0, 0), (2, 0), (-2, 0), (0, 2), (0, -2), (3, 3), (-3, 3), (3, -3), (5, 0), (0, 5), (0, -4), (8, 0), (-8, 0), (0, 10), (4, 4)]:
        r, p = roll * DEG, pitch * DEG
        c = lib.compensate(r, p)
        if c['status'] != 'OK':
            checks.append(dict(roll_deg=roll, pitch_deg=pitch, status=c['status'])); continue
        T = pose(r, p, 0, height)
        q = c['joints'].ravel()
        residual = float(np.abs(world_contacts(model, q, T) - contacts).max())
        qi, resi = independent_solution(model, T, contacts, seeds.ravel())
        lim = float(np.min(np.minimum(q - model.limits[:, 0], model.limits[:, 1] - q)))
        checks.append(dict(roll_deg=roll, pitch_deg=pitch, status='OK', independent_contact_residual_m=residual, max_joint_difference_vs_independent_solver_rad=float(np.abs(q - qi).max()),
                           independent_solver_residual_m=resi, cpp_margin_rad=float(c['margin']), independent_margin_rad=lim, max_axis_tilt_deg=float(max(axis_tilt(model, q, T)) / DEG),
                           cpp_drift_m=float(c['drift']), condition=float(c['condition']), iterations=c['iterations']))
    # envelope: single-axis limits at 0.25 deg resolution, and a coarse 2-D map
    def limit_scan(axis, sign):
        last_ok, first_fail = 0.0, None
        for a in np.arange(.25, 25.01, .25):
            r, p = (sign * a * DEG, 0) if axis == 'roll' else (0, sign * a * DEG)
            c = lib.compensate(r, p)
            if c['status'] != 'OK':
                first_fail = (a, c['failed_leg'], c['leg_status'], r, p); break
            last_ok = a
        return last_ok, first_fail
    env = {}
    for axis in ('roll', 'pitch'):
        for sign in (1, -1):
            ok, ff = limit_scan(axis, sign)
            entry = dict(max_tested_ok_deg=ok)
            if ff:
                a, leg, why, r, p = ff
                T = pose(r, p, 0, height)
                anyfree = exists_any(model, T, contacts, rng)
                # same target without the nominal-strip requirement (edge-biased contact allowed): is the geometry reachable at all?
                c2 = lib.compensate(r, p, strip=False)
                entry.update(first_failure_deg=a, failed_leg=LEGS[leg], cause=why, any_in_limit_solution_exists_independent=bool(anyfree),
                             solves_if_edge_biased_contact_allowed=c2['status'] == 'OK')
            env[f'{axis}_{"+" if sign > 0 else "-"}'] = entry
    grid_r = np.arange(-12, 12.1, 1.0); grid_p = np.arange(-10, 16.1, 1.0)
    grid = np.zeros((len(grid_p), len(grid_r)), dtype=int)
    for i, p in enumerate(grid_p):
        for j, r in enumerate(grid_r):
            grid[i, j] = lib.compensate(r * DEG, p * DEG)['status'] == 'OK'
    return dict(checks=checks, single_axis_envelope=env, map=dict(roll_deg=grid_r.tolist(), pitch_deg=grid_p.tolist(), feasible=grid.tolist()),
                worst_independent_contact_residual_m=max(c.get('independent_contact_residual_m', 0) for c in checks),
                worst_joint_difference_rad=max(c.get('max_joint_difference_vs_independent_solver_rad', 0) for c in checks))


def main():
    lib, model = Lib(), Model()
    f = frames(lib, model)
    (OUT / 'frames_oracle.json').write_text(json.dumps(f, indent=2, default=lambda o: o.item() if hasattr(o, 'item') else float(o)) + '\n')
    k = ik_and_envelope(lib, model)
    (OUT / 'ik_oracle.json').write_text(json.dumps(k, indent=2, default=lambda o: o.item() if hasattr(o, 'item') else float(o)) + '\n')
    print('frames', f['all_conventions_hold'], f['conventions'])
    print('ik worst residual', k['worst_independent_contact_residual_m'], 'joint diff', k['worst_joint_difference_rad'])
    for name, e in k['single_axis_envelope'].items():
        print(name, e)


if __name__ == '__main__':
    sys.exit(main())
