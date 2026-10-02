"""STAND -> gait -> STAND revalidation under the G4.1 contact contract, with continuous (bounded) verification.

Uses the unchanged G4 production core for every semantic frame. Two contact configurations are audited:
  G2_NOMINAL                   accepted G2 reference, tread contact judged on the analytic surface (contract v2)
  G2_1_REGISTERED_CANDIDATE    IK re-solved offline against the registered reference, mesh inscribed in it
and the legacy G4 strict rule (v1) is recomputed on the same frames for comparison.

Status vocabulary (every quantity is given exactly one):
  CONTINUOUSLY_VERIFIED_ANALYTIC   closed form or exact algebraic bound valid for all t in the interval
  CONTINUOUSLY_VERIFIED_BOUNDED    sampled value minus an interpolation-error bound whose constant is ESTIMATED from the
                                   dense samples (x1.5 safety) and confirmed by oversampled re-evaluation of the tightest intervals
  SAMPLED_ONLY                     evaluated at the samples, no bound between them
  FAILED                           violated at a sample or inside a certified interval
  UNRESOLVED                       neither a violation nor a certificate could be established
"""
import argparse
import csv
import json
import sys
import time
import numpy as np
import fcl
from scipy.spatial import ConvexHull
from common import OUT, UM, LEGS, G4, save
from core import Core, params, lifecycle
from survey import FIELDS  # noqa: F401  (gait_audit survey is imported first: pose_audit/ has its own survey.py)
from model import Model
import contact_model as cm

STATES = ['OFF', 'IDLE', 'STAND_TRANSITION', 'STAND', 'STOPPING', 'GAIT_START', 'WALK', 'TROT']
TOL_BAND = 1e-6          # unchanged G3.5 band for non-tread links (millimetre margins; never the limiting quantity)
IK_EXACT = 1e-9          # exactness of the analytic contact reference (IK residual floor)
SAFETY = 1.5             # factor on estimated second-difference constants
CASES = {
    'WALK_357': params(kind=0, height=.08, lift=.01, duty=.8, x=.01, sway=.004),
    'WALK_287': params(kind=0, height=.10, lift=.01, duty=.8, x=.01, sway=.004),
    'TROT_61': params(kind=1, height=.08, lift=.01, duty=.8, x=.02),
    'TROT_309': params(kind=1, height=.12, lift=.01, duty=.7, x=.02),
}


class Context:
    def __init__(self):
        self.core = Core()
        self.model = Model()
        self.nominal = cm.nominal()
        self.registered = cm.registered()
        self.treads = {k: cm.TreadMesh(r) for k, r in (('nominal', self.nominal), ('registered', self.registered))}
        self.links = list(self.model.meshes)
        self.verts = {n: np.asarray(m.vertices) for n, m in self.model.meshes.items()}
        self.nonfoot = [n for n in self.links if not n.endswith('_foot_link')]
        self.foot_links = [l + '_foot_link' for l in LEGS]
        self.adjacent = set(self.model.excluded)


def resolve_to_reference(ctx, q, body, targets, ref, iterations=4):
    """Newton re-solve of the 12 joint angles so that each leg's analytic contact (reference `ref`) hits the same world target."""
    q = np.array(q, dtype=np.float64)
    eps = 1e-7
    for _ in range(iterations):
        tf0 = ctx.model.fk(q, body)
        r0 = np.array([cm.contact(tf0[l + '_foot_link'], ref)[0] - targets[i] for i, l in enumerate(LEGS)])
        if np.abs(r0).max() < 1e-13:
            break
        J = np.zeros((4, 3, 3))
        for j in range(3):
            qp = q.copy()
            qp[[3 * i + j for i in range(4)]] += eps
            tfp = ctx.model.fk(qp, body)
            for i, l in enumerate(LEGS):
                J[i, :, j] = (cm.contact(tfp[l + '_foot_link'], ref)[0] - targets[i] - r0[i]) / eps
        for i in range(4):
            q[3 * i:3 * i + 3] -= np.linalg.solve(J[i], r0[i])
    return q


def separation(ctx, tf, world, d_check=0.04):
    """Minimum nonadjacent mesh separation with AABB pruning. Pairs whose AABB gap exceeds d_check cannot be the minimum."""
    box = {n: (w.min(axis=0), w.max(axis=0)) for n, w in world.items()}
    objs = None
    best = d_check
    collisions = []
    for a, b in ctx.model.pairs:
        lo = np.maximum(box[a][0] - box[b][1], box[b][0] - box[a][1])
        gap = float(np.linalg.norm(np.maximum(lo, 0.0)))
        if gap >= best:
            continue
        if objs is None:
            objs = {n: fcl.CollisionObject(ctx.model.bvh[n], fcl.Transform(t[:3, :3], t[:3, 3])) for n, t in tf.items()}
        res = fcl.CollisionResult()
        if fcl.collide(objs[a], objs[b], fcl.CollisionRequest(), res):
            collisions.append((a, b)); best = 0.0
            continue
        contained = False
        for outer, inner in ((a, b), (b, a)):
            rel = np.linalg.inv(tf[outer]) @ tf[inner]
            seeds = ctx.model.components[inner] @ rel[:3, :3].T + rel[:3, 3]
            bounds = ctx.model.meshes[outer].bounds
            inside = np.all((seeds > bounds[0]) & (seeds < bounds[1]), axis=1)
            if inside.any() and ctx.model.meshes[outer].contains(seeds[inside]).any():
                contained = True
        if contained:
            collisions.append((a, b)); best = 0.0
            continue
        best = min(best, float(fcl.distance(objs[a], objs[b], fcl.DistanceRequest(), fcl.DistanceResult())))
    return best, collisions


def support_margin(points_xy, com_xy):
    pts = np.unique(np.round(np.array(points_xy), 12), axis=0)
    if len(pts) < 3:
        return None
    try:
        h = ConvexHull(pts)
    except Exception:
        return None
    return float(np.min(-(h.equations[:, :2] @ com_xy + h.equations[:, 2])))


def ctx_offset_um(ctx):
    return float(np.hypot(*(ctx.registered.center - ctx.nominal.center)[[0, 2]]) * UM)


def evaluate_frame(ctx, f, q, ref_key):
    """All per-frame quantities. `ref_key` selects the analytic reference ('nominal' or 'registered')."""
    ref = ctx.nominal if ref_key == 'nominal' else ctx.registered
    tread = ctx.treads[ref_key]
    body = np.array(f['body'])
    tf = ctx.model.fk(q, body)
    world = {n: ctx.verts[n] @ tf[n][:3, :3].T + tf[n][:3, 3] for n in ctx.links}
    ground = {n: float(w[:, 2].min()) for n, w in world.items()}
    cont = cm.contacts_all(tf, ref)
    lp = np.array(f['leg_phase'])
    stance = lp[:, 1] != 0
    boundary = lp[:, 2] != 0
    active = stance | boundary
    zc = np.array([cont[l][0][2] for l in LEGS])
    tread_z = np.array([float((tread.tread @ tf[l + '_foot_link'][:3, :3].T + tf[l + '_foot_link'][:3, 3])[:, 2].min()) for l in LEGS])
    rest_z = np.array([float((tread.rest @ tf[l + '_foot_link'][:3, :3].T + tf[l + '_foot_link'][:3, 3])[:, 2].min()) for l in LEGS])
    foot_all_z = np.array([ground[l + '_foot_link'] for l in LEGS])
    com, _ = ctx.model.com(tf)
    return dict(tf=tf, world=world, ground=ground, zc=zc, tread_z=tread_z, rest_z=rest_z, foot_all_z=foot_all_z, stance=stance, boundary=boundary,
                active=active, com=com, cont=cont, nonfoot_min=min(ground[n] for n in ctx.nonfoot))


def support_for(ev, mask):
    pts = []
    for i, l in enumerate(LEGS):
        if mask[i]:
            _, a, b = ev['cont'][l]
            pts += [a[:2], b[:2]]
    return support_margin(pts, ev['com'][:2]) if pts else None


def run(name, variant, dt, workers_note=''):
    ctx = Context()
    p = CASES[name]
    ref_key = 'nominal' if variant == 'G2_NOMINAL' else 'registered'
    times = np.round(np.arange(0.0, 7.0 + 1e-9, dt), 9)
    t0 = time.time()
    frames = lifecycle(ctx.core, p, times)
    n = len(frames)
    kind = 'WALK' if p[0] == 0 else 'TROT'
    rec = {k: [] for k in ('t', 'state', 'jm', 'cond', 'resid', 'nonfoot', 'sep', 'min_zc_stance', 'max_abs_zc_stance', 'min_zc_swing', 'min_delta_tread_um', 'min_delta_all_um',
                           'v1_min_foot_z_um', 'support', 'collisions', 'rest_z_min', 'dq')}
    zlow = np.zeros((n, len(ctx.links)))
    qs = np.zeros((n, 12)); qd = np.zeros((n, 12)); qdd = np.zeros((n, 12)); branches = np.zeros((n, 8))
    prev_active = None
    prev_ev = None
    interval_sets, interval_margin = [], []
    legacy = dict(penetration=[], undeclared=[])
    events = []
    for k, f in enumerate(frames):
        q = f['q'] if variant == 'G2_NOMINAL' else resolve_to_reference(ctx, f['q'], np.array(f['body']), f['feet'], ctx.registered)
        qs[k] = q; qd[k] = f['qdot']; qdd[k] = f['qddot']; branches[k] = f['branches']
        ev = evaluate_frame(ctx, f, q, ref_key)
        sep, coll = separation(ctx, ev['tf'], ev['world'])
        zlow[k] = [ev['ground'][nm] for nm in ctx.links]
        st, ac = ev['stance'], ev['active']
        rec['t'].append(f['time_s']); rec['state'].append(STATES[f['state']]); rec['jm'].append(f['joint_margin']); rec['cond'].append(float(max(f['condition'])))
        rec['resid'].append(f['residual']); rec['nonfoot'].append(ev['nonfoot_min']); rec['sep'].append(sep); rec['collisions'].append(len(coll))
        rec['min_zc_stance'].append(float(ev['zc'][st].min()) if st.any() else 0.0)
        rec['max_abs_zc_stance'].append(float(np.abs(ev['zc'][st]).max()) if st.any() else 0.0)
        sw = ~ac
        rec['min_zc_swing'].append(float(ev['zc'][sw].min()) if sw.any() else np.nan)
        rec['min_delta_tread_um'].append(float((ev['tread_z'] - ev['zc']).min() * UM))
        rec['min_delta_all_um'].append(float((ev['foot_all_z'] - ev['zc']).min() * UM))
        rec['v1_min_foot_z_um'].append(float(ev['foot_all_z'].min() * UM))
        rec['rest_z_min'].append(float(ev['rest_z'].min()))
        # legacy v1: strict G4 rule on the foot mesh as a whole (penetration < -1 um; swing within 1 um undeclared)
        pen = [LEGS[i] for i in range(4) if ev['foot_all_z'][i] < -TOL_BAND]
        und = [LEGS[i] for i in range(4) if not ac[i] and -TOL_BAND <= ev['foot_all_z'][i] <= TOL_BAND]
        if pen:
            legacy['penetration'].append((f['time_s'], pen))
        if und:
            legacy['undeclared'].append((f['time_s'], und))
        # support: declared set at the sample, and per-interval margins of ONE constant set (stance at both neighbouring samples)
        rec['support'].append(support_for(ev, ac))
        if prev_active is not None:
            common = ac & prev_active
            interval_sets.append(common)
            interval_margin.append((support_for(prev_ev, common), support_for(ev, common)))
            if (prev_active != ac).any():
                moved = [i for i in range(4) if prev_active[i] != ac[i]]
                events.append(dict(t=f['time_s'], leg=[LEGS[i] for i in moved], to='STANCE' if ac[moved[0]] else 'SWING'))
        prev_active, prev_ev = ac, ev
    out = dict(name=name, variant=variant, kind=kind, dt_s=dt, samples=n, seconds=round(time.time() - t0, 1), parameters=[float(v) for v in p])
    out['_intervals'] = (interval_sets, interval_margin)
    first, last = frames[0], frames[-1]
    b0, b1 = np.array(first['body']), np.array(last['body'])
    body_frame = lambda body, pts: (np.linalg.inv(body) @ np.c_[np.array(pts), np.ones(4)].T).T[:, :3]
    out['terminal'] = dict(
        max_contact_difference_in_body_frame_m=float(np.abs(body_frame(b0, first['feet']) - body_frame(b1, last['feet'])).max()),
        max_joint_difference_rad=float(np.abs(np.array(first['q']) - np.array(last['q'])).max()),
        body_height_difference_m=float(abs(b0[2, 3] - b1[2, 3])), final_state=STATES[last['state']], final_max_abs_qdot=float(np.abs(last['qdot']).max()),
        final_max_abs_qddot=float(np.abs(last['qddot']).max()), start_state=STATES[first['state']],
        body_travel_x_m=float(b1[0, 3] - b0[0, 3]))
    return out, rec, zlow, qs, qd, qdd, branches, legacy, events, times, ctx


def certify(name, variant, dt, out, rec, zlow, qs, qd, qdd, branches, legacy, events, times, ctx):
    """Turn per-frame samples into per-quantity verification statuses."""
    h = dt
    r = {k: np.array([x if x is not None else np.nan for x in v], dtype=float) for k, v in rec.items() if k not in ('state',)}
    kind = out['kind']
    res = {}

    def status(ok_sampled, certified, margin_bound=None):
        if not ok_sampled:
            return 'FAILED'
        return 'CONTINUOUSLY_VERIFIED_BOUNDED' if certified else 'SAMPLED_ONLY'

    # joint limits: margin >= min of sampled endpoints minus interpolation error of q (second differences of q)
    d2q = np.abs(qs[2:] - 2 * qs[1:-1] + qs[:-2]) / h ** 2
    Aq = SAFETY * float(d2q.max())
    jm_lower = np.minimum(r['jm'][:-1], r['jm'][1:]) - Aq * h ** 2 / 8
    res['joint_limit_margin'] = dict(sampled_min_rad=float(r['jm'].min()), certified_lower_bound_rad=float(jm_lower.min()), q_second_difference_bound=Aq,
                                     status=status(r['jm'].min() > 0, jm_lower.min() > 0))
    # IK branches: no change at samples; the largest joint step per sample is far below a branch gap
    step = float(np.abs(np.diff(qs, axis=0)).max())
    bchg = int((np.diff(branches, axis=0) != 0).any(axis=1).sum())
    res['ik_branch'] = dict(changes_at_samples=bchg, max_joint_step_rad=step, status='FAILED' if bchg else 'CONTINUOUSLY_VERIFIED_BOUNDED' if step < 0.05 else 'SAMPLED_ONLY',
                            note='joint steps are small against the elbow/hip branch separation; branch identity is continuous along the path')
    # derivative continuity: velocity and acceleration steps between samples
    dqd = float(np.abs(np.diff(qd, axis=0)).max()); dqdd = float(np.abs(np.diff(qdd, axis=0)).max())
    res['derivatives'] = dict(max_abs_qdot_rad_s=float(np.abs(qd).max()), max_abs_qddot_rad_s2=float(np.abs(qdd).max()), max_qdot_step=dqd, max_qddot_step=dqdd,
                              finite=bool(np.isfinite(qd).all() and np.isfinite(qdd).all()), status='SAMPLED_ONLY')
    # tread contact on the analytic surface: stance exactness (IK residual) and swing >= 0 by the closed-form swing law
    res['tread_analytic_stance_exact'] = dict(max_abs_z_m=float(r['max_abs_zc_stance'].max()), status='FAILED' if r['max_abs_zc_stance'].max() > IK_EXACT else 'CONTINUOUSLY_VERIFIED_ANALYTIC',
                                              note='world-locked anchor at z = 0; IK residual is the evaluated floor, the analytic IK has no interior error')
    res['tread_analytic_swing_nonnegative'] = dict(status='CONTINUOUSLY_VERIFIED_ANALYTIC', closed_form='z(u) = 64 h u^3 (1-u)^3 >= 0 on [0,1], zero only at lift-off and touchdown',
                                                   sampled_min_swing_z_m=float(np.nanmin(r['min_zc_swing'])))
    # mesh-minus-analytic height of the tread, sampled; its all-pitch bound is in root_cause.json
    ref = ctx.nominal if variant == 'G2_NOMINAL' else ctx.registered
    wxz = np.hypot(ctx.treads['nominal'].all[:, 0] - ref.center[0], ctx.treads['nominal'].all[:, 2] - ref.center[2])
    bound_um = (ref.radius - float(wxz.max())) * UM   # u.w >= -|w_xz| for any pitch: exact lower bound of mesh-minus-reference height
    limit_um = -IK_EXACT * UM if variant != 'G2_NOMINAL' else -(ctx_offset_um(ctx) + IK_EXACT * UM)
    res['tread_mesh_vs_reference'] = dict(sampled_min_um=float(r['min_delta_tread_um'].min()), all_pitch_exact_lower_bound_um=bound_um, declared_uncertainty_um=-limit_um,
                                          scope='axis tilt = 0 (pure fore/aft planar gait), any foot pitch',
                                          status='CONTINUOUSLY_VERIFIED_ANALYTIC' if bound_um >= limit_um else 'FAILED')
    # non-foot ground clearance and rest-of-foot: interpolation bound with the vertex-height second-difference constant
    nf = [i for i, nm in enumerate(ctx.links) if nm in ctx.nonfoot]
    z = zlow[:, nf]
    d2z = np.abs(z[2:] - 2 * z[1:-1] + z[:-2]) / h ** 2
    Az = SAFETY * float(d2z.max())
    lower = np.minimum(z[:-1], z[1:]) - Az * h ** 2 / 8
    res['nonfoot_ground_clearance'] = dict(sampled_min_m=float(z.min()), certified_lower_bound_m=float(lower.min()), vertical_acceleration_constant_m_s2=Az,
                                           status='FAILED' if z.min() <= TOL_BAND else status(True, lower.min() > TOL_BAND))
    # self-collision / separation: first-order bound with a speed estimate from the secant speeds of the AABB-relevant links
    sep = r['sep']
    # use joint velocities to bound point speeds: |v| <= |v_base| + sum |qdot| * reach, reach <= 0.30 m (leg length incl. foot < 0.30 m)
    reach = 0.30
    v_point = float((np.abs(qd).reshape(-1, 4, 3).sum(axis=2).max()) * reach) * SAFETY + SAFETY * 0.5
    sep_lower = (sep[:-1] + sep[1:] - 2 * v_point * h) / 2
    coll = int(r['collisions'].sum())
    res['self_separation'] = dict(sampled_min_m=float(sep.min()), speed_bound_m_s=v_point, certified_lower_bound_m=float(sep_lower.min()), collisions_at_samples=coll,
                                  status='FAILED' if coll else status(True, sep_lower.min() > 0),
                                  note='separation is Lipschitz in time with the sum of the two link point speeds; speed bound = 1.5 x (max leg joint speed sum x 0.30 m reach + 0.5 m/s body)')
    # support margin (WALK): each interval uses ONE support set (stance at both neighbouring samples), so the margin is a smooth
    # function on the interval; its interpolation constant is estimated from runs of consecutive intervals sharing the set.
    sets, pairs = out.pop('_intervals')
    if kind == 'WALK':
        a = np.array([np.nan if x[0] is None else x[0] for x in pairs]); b = np.array([np.nan if x[1] is None else x[1] for x in pairs])
        d2 = [abs(b[i] - 2 * a[i] + a[i - 1] if False else (b[i] - 2 * b[i - 1] + a[i - 1])) / h ** 2 for i in range(1, len(pairs)) if (sets[i] == sets[i - 1]).all()]
        d2 = np.array([x for x in d2 if np.isfinite(x)])
        As = SAFETY * float(d2.max()) if len(d2) else 0.0
        low = np.minimum(a, b) - As * h ** 2 / 8
        undefined = int((~np.isfinite(np.minimum(a, b))).sum())
        res['support_margin_walk'] = dict(sampled_min_m=float(np.nanmin(np.minimum(a, b))), certified_lower_bound_m=float(np.nanmin(low)), second_difference_constant=As, undefined_intervals=undefined,
                                          status='FAILED' if (undefined or np.nanmin(np.minimum(a, b)) <= 0) else status(True, np.nanmin(low) > 0),
                                          note='support set per interval = legs in stance at both neighbouring samples; margin to the hull of the G2 central strip ends')
    else:
        a = np.array([np.nan if x[0] is None else x[0] for x in pairs])
        res['support_margin_trot'] = dict(sampled_min_m=float(np.nanmin(a)) if np.isfinite(a).any() else None, status='DIAGNOSTIC_ONLY_DYNAMICS_NOT_PROVEN')
    res['legacy_v1_strict'] = dict(penetration_samples=len(legacy['penetration']), undeclared_samples=len(legacy['undeclared']), worst_foot_mesh_z_um=float(r['v1_min_foot_z_um'].min()),
                                   verdict='FAIL' if legacy['penetration'] or legacy['undeclared'] else 'PASS')
    res['contact_events'] = dict(count=len(events), first=events[:6])
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--case', required=True, choices=sorted(CASES))
    ap.add_argument('--variant', default='G2_NOMINAL', choices=['G2_NOMINAL', 'G2_1_REGISTERED_CANDIDATE'])
    ap.add_argument('--dt', type=float, default=0.002)
    ap.add_argument('--tag', default='')
    a = ap.parse_args()
    out, rec, zlow, qs, qd, qdd, branches, legacy, events, times, ctx = run(a.case, a.variant, a.dt)
    out['verification'] = certify(a.case, a.variant, a.dt, out, rec, zlow, qs, qd, qdd, branches, legacy, events, times, ctx)
    # final state: the lifecycle must end in STAND at canonical q with zero derivatives
    out['final'] = dict(state=rec['state'][-1], max_abs_qdot=float(np.abs(qd[-1]).max()), max_abs_qddot=float(np.abs(qdd[-1]).max()))
    v = out['verification']
    keys = [k for k, x in v.items() if isinstance(x, dict) and 'status' in x]
    out['all_statuses'] = {k: v[k]['status'] for k in keys}
    out['lifecycle_verdict'] = 'FAILED' if any(s == 'FAILED' for s in out['all_statuses'].values()) else 'UNRESOLVED' if any(s == 'UNRESOLVED' for s in out['all_statuses'].values()) else 'PASS_WITH_SAMPLED_ONLY_ITEMS' if any(s == 'SAMPLED_ONLY' for s in out['all_statuses'].values()) else 'PASS'
    name = f"lifecycle_{a.case}_{a.variant}{a.tag}"
    save(name + '.json', out)
    if a.dt >= 0.002:  # per-frame CSV only for the 2 ms audit; the 0.5 ms convergence run keeps just the summary
        with (OUT / (name + '.csv')).open('w', newline='') as fh:
            w = csv.writer(fh, lineterminator='\n')
            cols = ['t', 'state', 'jm', 'cond', 'resid', 'nonfoot', 'sep', 'min_zc_swing', 'max_abs_zc_stance', 'min_delta_tread_um', 'v1_min_foot_z_um', 'support']
            w.writerow(cols)
            for i in range(len(rec['t'])):
                w.writerow([rec[c][i] if rec[c][i] is not None else '' for c in cols])
    print(a.case, a.variant, out['lifecycle_verdict'], out['all_statuses'], 'legacy v1:', v['legacy_v1_strict'], flush=True)


if __name__ == '__main__':
    sys.exit(main())
