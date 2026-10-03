"""Leg rigid-body dynamics from the canonical URDF masses and inertias (offline analysis, SI units).

Recursive Newton-Euler for the 3-DOF serial leg chain (hip about X, upper and lower about Y, foot rigidly attached) with a
base that may accelerate (gravity is modelled as an upward base acceleration). Ground reaction forces are applied through the
G2 contact reference. This is a rigid-body model: no servo dynamics, backlash, compliance, friction or cable loads.
URDF inertials come from CAD; the robot's real mass distribution (battery, wiring, head) is NOT verified.
"""
import xml.etree.ElementTree as ET
import numpy as np
import yaml
from core import ROOT, LEGS

URDF = ROOT / '03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf'
G = 9.80665


def _v(s):
    return np.array([float(x) for x in s.split()])


def rot(axis, a):
    ax = np.asarray(axis, dtype=float); c, s = np.cos(a), np.sin(a)
    if ax[0]: return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


class Robot:
    def __init__(self, mass_scale=1.0):
        root = ET.parse(URDF).getroot()
        self.links = {}
        for l in root.findall('link'):
            i = l.find('inertial'); m = float(i.find('mass').get('value')); c = _v(i.find('origin').get('xyz'))
            a = i.find('inertia'); I = np.array([[float(a.get('ixx')), float(a.get('ixy')), float(a.get('ixz'))],
                                                 [float(a.get('ixy')), float(a.get('iyy')), float(a.get('iyz'))],
                                                 [float(a.get('ixz')), float(a.get('iyz')), float(a.get('izz'))]])
            self.links[l.get('name')] = (m * mass_scale, c, I * mass_scale)
        self.joints = {j.get('name'): j for j in root.findall('joint')}
        g = yaml.safe_load((ROOT / '06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml').read_text())['rigid_cylinder']
        self.cyl_c, self.cyl_axis, self.cyl_r = np.array(g['center_in_foot_link_m']), np.array(g['axis_in_foot_link_unit']), float(g['radius_m'])
        self.total_mass = sum(m for m, _, _ in self.links.values())
        self.chain = {}
        for leg in LEGS:
            ch = []
            for jn, ln in ((f'{leg}_hip_joint', f'{leg}_hip_link'), (f'{leg}_upper_leg_joint', f'{leg}_upper_leg_link'), (f'{leg}_lower_leg_joint', f'{leg}_lower_leg_link'), (f'{leg}_foot_joint', f'{leg}_foot_link')):
                j = self.joints[jn]
                ch.append(dict(origin=_v(j.find('origin').get('xyz')), axis=_v(j.find('axis').get('xyz')) if j.get('type') == 'revolute' else None, link=ln, joint=jn))
            self.chain[leg] = ch

    def fk(self, leg, q):
        """Per-link (R, p) in base_link and the joint-frame list used by the dynamics."""
        R, p, out = np.eye(3), np.zeros(3), []
        for k, e in enumerate(self.chain[leg]):
            p = p + R @ e['origin']
            if e['axis'] is not None:
                R = R @ rot(e['axis'], q[k])
            out.append((R.copy(), p.copy()))
        return out

    def link_com(self, leg, q):
        res = []
        for e, (R, p) in zip(self.chain[leg], self.fk(leg, q)):
            m, c, _ = self.links[e['link']]
            res.append((m, p + R @ c))
        return res

    def contact_point(self, leg, q, normal=(0, 0, 1)):
        R, p = self.fk(leg, q)[-1]
        a = R @ self.cyl_axis; n = np.asarray(normal, dtype=float); n = n / np.linalg.norm(n)
        down = -(n - a * (a @ n)); down /= np.linalg.norm(down)
        return p + R @ self.cyl_c + self.cyl_r * down

    def contact_jacobian(self, leg, q, normal=(0, 0, 1), h=1e-7):
        J = np.zeros((3, 3))
        for j in range(3):
            qp, qm = np.array(q, dtype=float), np.array(q, dtype=float); qp[j] += h; qm[j] -= h
            J[:, j] = (self.contact_point(leg, qp, normal) - self.contact_point(leg, qm, normal)) / (2 * h)
        return J

    def rnea(self, leg, q, qd, qdd, base_accel):
        """Craig RNEA written in base-aligned coordinates (all vectors expressed in base_link); base angular motion is zero."""
        ch = self.chain[leg]
        w_prev, al_prev, a_prev = np.zeros(3), np.zeros(3), np.asarray(base_accel, dtype=float)
        R = np.eye(3); p = np.zeros(3)
        data = []; k = 0
        for e in ch:
            o = R @ e['origin']                      # origin offset in base coordinates
            a_o = a_prev + np.cross(al_prev, o) + np.cross(w_prev, np.cross(w_prev, o))
            p = p + o
            if e['axis'] is not None:
                z = R @ e['axis']                    # joint axis in base coordinates (before rotating about it, axis is invariant)
                R = R @ rot(e['axis'], q[k])
                w = w_prev + z * qd[k]
                al = al_prev + z * qdd[k] + np.cross(w_prev, z * qd[k])
                zi = z; k += 1
            else:
                w, al, zi = w_prev, al_prev, None
            m, c, I = self.links[e['link']]
            cw = R @ c
            a_c = a_o + np.cross(al, cw) + np.cross(w, np.cross(w, cw))
            Iw = R @ I @ R.T
            F = m * a_c
            Nn = Iw @ al + np.cross(w, Iw @ w)
            data.append(dict(F=F, N=Nn, cw=cw, o=o, z=zi))
            w_prev, al_prev, a_prev = w, al, a_o
        # backward
        f_next = np.zeros(3); n_next = np.zeros(3); tau = np.zeros(3); idx = 2
        o_next = np.zeros(3)
        for i in range(len(ch) - 1, -1, -1):
            d = data[i]
            f = d['F'] + f_next
            n = d['N'] + np.cross(d['cw'], d['F']) + n_next + np.cross(o_next, f_next)
            if d['z'] is not None:
                tau[idx] = d['z'] @ n; idx -= 1
            f_next, n_next, o_next = f, n, d['o']
        return tau

    def leg_torque(self, leg, q, qd, qdd, base_accel_world, foot_force_world=None, R_wb=None):
        """Actuator torques of one leg. foot_force_world is the ground reaction on the foot (N, world frame)."""
        Rw = np.eye(3) if R_wb is None else R_wb
        a0 = Rw.T @ (np.array([0, 0, G]) + np.asarray(base_accel_world, dtype=float))
        tau = self.rnea(leg, q, qd, qdd, a0)
        if foot_force_world is not None and np.any(foot_force_world):
            normal = Rw.T @ np.array([0, 0, 1.0])
            tau = tau - self.contact_jacobian(leg, q, normal).T @ (Rw.T @ np.asarray(foot_force_world, dtype=float))
        return tau

    # ---- validation helpers (independent of the RNEA) ----
    def potential(self, leg, q):
        return sum(m * G * c[2] for m, c in self.link_com(leg, q))

    def kinetic(self, leg, q, qd, h=1e-6):
        T = 0.0
        com0 = self.link_com(leg, q); com1 = self.link_com(leg, np.asarray(q) + h * np.asarray(qd)); com_1 = self.link_com(leg, np.asarray(q) - h * np.asarray(qd))
        for i, e in enumerate(self.chain[leg]):
            m = com0[i][0]; v = (com1[i][1] - com_1[i][1]) / (2 * h)
            R, _ = self.fk(leg, q)[i]; Rp, _ = self.fk(leg, np.asarray(q) + h * np.asarray(qd))[i]; Rm, _ = self.fk(leg, np.asarray(q) - h * np.asarray(qd))[i]
            S = (Rp - Rm) / (2 * h) @ R.T      # skew(omega)
            w = np.array([S[2, 1], S[0, 2], S[1, 0]])
            Iw = R @ self.links[e['link']][2] @ R.T
            T += 0.5 * m * v @ v + 0.5 * w @ Iw @ w
        return T


def com_world(robot, q12, R_wb, t_wb):
    """Whole-body COM in world from the URDF inertials for joint vector q12 (LF RF RH LH x hip/upper/lower)."""
    m0, c0, _ = robot.links['base_link']
    acc, mass = m0 * (R_wb @ c0 + t_wb), m0
    for i, leg in enumerate(LEGS):
        for m, c in robot.link_com(leg, q12[3 * i:3 * i + 3]):
            acc = acc + m * (R_wb @ c + t_wb); mass += m
    return acc / mass, mass


def strip_ends(robot, leg, q, R_wb, t_wb):
    """World positions of the G2 central support strip ends of one foot (level ground normal)."""
    R, p = robot.fk(leg, q)[-1]
    normal = R_wb.T @ np.array([0, 0, 1.0])
    a = R @ robot.cyl_axis
    down = -(normal - a * (a @ normal)); down /= np.linalg.norm(down)
    pt = p + R @ robot.cyl_c + robot.cyl_r * down
    half = a * 0.5 * 0.0099
    return R_wb @ (pt - half) + t_wb, R_wb @ (pt + half) + t_wb


def distribute_vertical(points_xy, target_xy, weight):
    """Minimum-norm non-negative vertical forces with sum = weight and centre of pressure at target_xy; None if infeasible.

    This is a STATIC load-sharing ASSUMPTION (the true distribution of a compliant, statically indeterminate stance is not known)."""
    from scipy.optimize import minimize
    P = np.asarray(points_xy, dtype=float); n = len(P)
    if n == 0:
        return None
    A = np.vstack([np.ones(n), P[:, 0], P[:, 1]]); b = np.array([weight, weight * target_xy[0], weight * target_xy[1]])
    res = minimize(lambda f: f @ f, np.full(n, weight / n), jac=lambda f: 2 * f, constraints=[dict(type='eq', fun=lambda f: A @ f - b, jac=lambda f: A)],
                   bounds=[(0, None)] * n, method='SLSQP', options=dict(ftol=1e-14, maxiter=200))
    if not res.success or np.abs(A @ res.x - b).max() > 1e-7 * max(1, weight):
        return None
    return res.x
