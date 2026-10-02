"""Dense ground-policy probe of STAND -> gait -> STAND for representative parameter sets.

Complements the preserved 141-frame full-mesh lifecycle_audit.json with 5 ms sampling of the same
kinematic lifecycle (ground and support policy only; no self-collision). It records what blocks each
lifecycle under the unchanged strict policy and where the rear foot pitch sits relative to the
sub -1 um facet windows of the canonical foot mesh. It does not alter any policy.
"""
import json
import sys
import numpy as np
from core import Core, params, lifecycle, ROOT
from survey import save, OUT
sys.path.insert(0, str(ROOT / '06_Software/Matdog_Core/pose_audit'))
from model import Model, LEGS
from contact_discrepancy import Foot, pitch_deg

CASES = {
    'WALK_100mm_id287': params(kind=0, height=.1, lift=.01, duty=.8, x=.01, sway=.004),
    'WALK_80mm_id357': params(kind=0, height=.08, lift=.01, duty=.8, x=.01, sway=.004),
    'TROT_100mm_id3105': params(kind=1, height=.1, lift=.005, duty=.6, x=.045),
    'TROT_80mm_id61': params(kind=1, height=.08, lift=.01, duty=.8, x=.02),
}
STATES = ['OFF', 'IDLE', 'STAND_TRANSITION', 'STAND', 'STOPPING', 'GAIT_START', 'WALK', 'TROT']


def run(name, p, core, model, foot):
    times = np.arange(0, 7.0 + 1e-9, 0.005)
    frames = lifecycle(core, p, times)
    rows = []; worst = 0.0; rear = []
    for f in frames:
        active = [l for i, l in enumerate(LEGS) if f['leg_phase'][i, 1] or f['leg_phase'][i, 2]]
        a = model.evaluate(f['q'], f['body'], 'FOOT_SUPPORT', active, full=False)
        errs = [e for e in a['errors'] if p[0] == 0 or e != 'SUPPORT_INVALID']
        tf = model.fk(f['q'], f['body'])
        rear.append(pitch_deg(tf['rh_foot_link'])[0])
        for e in errs:
            leg = e.split(':')[-1]
            rows.append(dict(time_s=f['time_s'], state=STATES[f['state']], error=e.split(':')[0], link=leg,
                             mesh_min_z_um=a['ground_min_m'].get(leg, float('nan')) * 1e6 if leg in a['ground_min_m'] else None))
        worst = min(worst, min(a['ground_min_m'][l + '_foot_link'] for l in LEGS))
    kinds = {}
    for r in rows:
        kinds[r['error']] = kinds.get(r['error'], 0) + 1
    by_state = {}
    for r in rows:
        by_state[r['state']] = by_state.get(r['state'], 0) + 1
    start = [r for r in rows if r['state'] == 'GAIT_START' and r['error'] == 'GROUND_PENETRATION']
    return dict(name=name, parameters=[float(v) for v in p], samples=len(frames), strict_pass=not rows,
                failing_error_instances=kinds, failing_by_state=by_state,
                worst_foot_mesh_z_um=worst * 1e6,
                start_prep_penetration_instances=len(start),
                rear_pitch_range_deg=[min(rear), max(rear)],
                first_failures=rows[:6])


def main():
    core = Core(); model = Model(); foot = Foot(model)
    out = [run(n, p, core, model, foot) for n, p in CASES.items()]
    save('lifecycle_probe.json', dict(
        scope='5 ms ground/support-policy probe of the kinematic lifecycle; supplementary to lifecycle_audit.json (141-frame full mesh)',
        cases=out))
    for r in out:
        print(r['name'], 'PASS' if r['strict_pass'] else 'FAIL', r['failing_error_instances'], r['failing_by_state'], 'worst %.3f um' % r['worst_foot_mesh_z_um'], 'rear pitch', [round(v, 2) for v in r['rear_pitch_range_deg']])


if __name__ == '__main__':
    main()
