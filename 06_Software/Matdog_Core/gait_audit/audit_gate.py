"""Review gate: independently re-derive full-mesh pass/fail from saved per-frame evidence.

Reads the preserved raw survey artifacts (never modified) and writes a derived audit.
The per-case `complete` flag in the raw files is only `not errors`; the top-level
`complete` flag only means every planned case was evaluated. Neither is used here as
the pass predicate. A case passes only when every explicit criterion below holds.
"""
import hashlib
import json
import sys
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
OUT = ROOT / '09_Logs/Validation_Reports/G4_Gait_Envelope'
SOURCES = ('full_cases.json', 'refinement_full.json')
GROUND_TOL_M = 1e-6          # unchanged G3.5 policy
RESIDUAL_TOL_M = 1e-9
DRIFT_TOL_M = 1e-9
PREDICATE = [
    'validation == FULL_SAMPLED_MESH and len(frames) == geometry_samples > 0',
    'classification == [KINEMATICALLY_VALID] and first_failure is null',
    'no frame carries an error (TROT: only the diagnostic-only SUPPORT_INVALID is tolerated)',
    'every frame: joint margin > 0, Jacobian condition <= max_condition, contact residual <= 1e-9 m',
    'no IK branch change between consecutive frames (recomputed from frames)',
    'WALK only: modeled support margin > 0 in every frame',
    'non-foot link-ground clearance > 1 um and nonadjacent separation > 0 in every frame',
    'foot mesh never below -1 um; a swing foot away from its lift/touch boundary stays above +1 um',
    'stance contact drift from its touchdown point <= 1e-9 m (recomputed per stance run)',
]


def load():
    cases = []
    for name in SOURCES:
        raw = json.loads((OUT / name).read_text())
        assert raw['complete'] and len(raw['cases']) == raw['planned_cases'], name
        for case in raw['cases']:
            cases.append((name, case))
    return cases


def audit_case(source, case):
    p = case['parameters']
    walk = p['type'] == 0
    frames = case['frames']
    failures = []
    if case['validation'] != 'FULL_SAMPLED_MESH' or not frames or len(frames) != case['geometry_samples']:
        failures.append('NOT_A_COMPLETE_FULL_MESH_RECORD')
    if case['classification'] != ['KINEMATICALLY_VALID'] or case['first_failure'] is not None:
        failures.append('RAW_CLASSIFICATION_' + '+'.join(case['classification']))
    allowed = [] if walk else ['SUPPORT_INVALID']
    margin, cond, resid = [], [], []
    nonfoot, foot_low, swing_foot, sep, support, drift = [], [], [], [], [], []
    trot_support_fail = 0
    for f in frames:
        bad = [e for e in f['errors'] if e not in allowed]
        if bad or f['classification']:
            failures.append('FRAME_ERROR@%.6f:%s' % (f['phase'], ','.join(bad or f['classification'])))
        if 'SUPPORT_INVALID' in f['errors']:
            trot_support_fail += 1
        margin.append(f['joint_margin_rad']); cond.append(max(f['condition'])); resid.append(f['residual_m'])
        sep.append(f['min_separation']['distance_m'])
        gm = f['ground_min_m']
        nonfoot.append(min(v for k, v in gm.items() if not k.endswith('_foot_link')))
        lp = np.array(f['leg_phase'])
        for i, leg in enumerate(('lf', 'rf', 'rh', 'lh')):
            z = gm[leg + '_foot_link']; foot_low.append(z)
            if not lp[i, 1] and not lp[i, 2]:
                swing_foot.append(z)
        if walk and not (f['support_margin_m'] is not None and f['support_margin_m'] > 0):
            failures.append('WALK_SUPPORT@%.6f' % f['phase'])
        if f['support_margin_m'] is not None:
            support.append((f['support_margin_m'], f['phase'] % 1, len(f['active_feet'])))
    if min(margin) <= 0: failures.append('JOINT_MARGIN')
    if max(cond) > p['max_condition']: failures.append('CONDITION')
    if max(resid) > RESIDUAL_TOL_M: failures.append('CONTACT_RESIDUAL')
    if min(nonfoot) <= GROUND_TOL_M: failures.append('NONFOOT_GROUND')
    if min(sep) <= 0: failures.append('SELF_SEPARATION')
    if min(foot_low) < -GROUND_TOL_M: failures.append('FOOT_PENETRATION')
    if swing_foot and min(swing_foot) <= GROUND_TOL_M: failures.append('SWING_FOOT_NEAR_GROUND')
    branches = np.array([f['branches'] for f in frames]).reshape(len(frames), 4, 2)
    changes = int(np.any(branches[1:] != branches[:-1], axis=2).sum())
    if changes: failures.append('BRANCH_CHANGE')
    # Stance drift: worst displacement of any stance contact from the first point of its run.
    worst = 0.0
    for i in range(4):
        start = None
        for f in frames:
            lp = f['leg_phase'][i]
            if lp[1]:
                c = np.array(f['contacts_world_m'][i])
                if start is None: start = c
                worst = max(worst, float(np.linalg.norm(c - start)))
            else:
                start = None
    if worst > DRIFT_TOL_M: failures.append('STANCE_DRIFT')
    peak_qdot = np.max(np.abs([f['qdot'] for f in frames]), axis=0)
    peak_qddot = np.max(np.abs([f['qddot'] for f in frames]), axis=0)
    weakest = min(support) if support else None
    return dict(
        source=source, id=case['id'], type='WALK' if walk else 'TROT', parameters=p,
        resolution_intervals=case['resolution_intervals'], frames=len(frames),
        raw_complete_flag=case['complete'], raw_classification=case['classification'],
        FULL_MESH_VALIDATED=not failures, failures=failures,
        recomputed=dict(
            min_joint_margin_rad=min(margin), max_condition=max(cond), max_contact_residual_m=max(resid),
            min_self_separation_m=min(sep), min_nonfoot_ground_m=min(nonfoot), min_foot_ground_m=min(foot_low),
            min_swing_foot_ground_m=min(swing_foot) if swing_foot else None,
            max_stance_drift_m=worst, branch_changes=changes,
            min_support_margin_m=weakest[0] if weakest else None,
            weakest_phase=weakest[1] if weakest else None, weakest_support_count=weakest[2] if weakest else None,
            trot_frames_with_static_support_failure=None if walk else trot_support_fail,
            peak_qdot_rad_s=peak_qdot.tolist(), peak_qddot_rad_s2=peak_qddot.tolist()),
        raw_metrics_agree=abs(case['metrics']['min_joint_margin_rad'] - min(margin)) < 1e-12 and abs(case['metrics']['min_self_separation_m'] - min(sep)) < 1e-12,
        dynamic_stability=case['dynamic_stability'])


def build():
    audits = [audit_case(s, c) for s, c in load()]
    ok = [a for a in audits if a['FULL_MESH_VALIDATED']]
    walk = [a for a in ok if a['type'] == 'WALK']
    def agg(group):
        r = [a['recomputed'] for a in group]
        sm = [x['min_support_margin_m'] for x in r if x['min_support_margin_m'] is not None]
        return dict(
            cases=len(group),
            min_joint_margin_rad=min(x['min_joint_margin_rad'] for x in r),
            min_self_separation_m=min(x['min_self_separation_m'] for x in r),
            min_nonfoot_ground_m=min(x['min_nonfoot_ground_m'] for x in r),
            max_stance_drift_m=max(x['max_stance_drift_m'] for x in r),
            max_condition=max(x['max_condition'] for x in r),
            total_branch_changes=sum(x['branch_changes'] for x in r))
    summary = dict(
        total_cases=len(audits), full_mesh_validated=len(ok),
        by_type={t: dict(total=sum(a['type'] == t for a in audits), validated=sum(a['type'] == t for a in ok)) for t in ('WALK', 'TROT')},
        aggregate_all_validated=agg(ok), aggregate_walk=agg(walk) if walk else None,
        aggregate_trot=agg([a for a in ok if a['type'] == 'TROT']),
        min_walk_support_margin_m=min(a['recomputed']['min_support_margin_m'] for a in walk),
        min_walk_support_margin_case=min(walk, key=lambda a: a['recomputed']['min_support_margin_m'])['id'],
        failed_cases=[dict(id=a['id'], type=a['type'], failures=a['failures'][:3]) for a in audits if not a['FULL_MESH_VALIDATED']],
        raw_flag_disagreements=[a['id'] for a in audits if a['raw_complete_flag'] != a['FULL_MESH_VALIDATED']],
        raw_metric_disagreements=[a['id'] for a in audits if not a['raw_metrics_agree']])
    inputs = {n: hashlib.sha256((OUT / n).read_bytes()).hexdigest() for n in SOURCES}
    return dict(
        purpose='Independent re-derivation of the full-mesh pass predicate from preserved per-frame evidence',
        scope='Sampled finite grids only: maximum tested passing, not a physical or continuous limit',
        predicate=PREDICATE, thresholds=dict(ground_tol_m=GROUND_TOL_M, residual_tol_m=RESIDUAL_TOL_M, drift_tol_m=DRIFT_TOL_M),
        raw_semantics=dict(
            top_level_complete='all planned cases were evaluated; says nothing about validity',
            case_complete='survey.py: complete = not errors, errors being the union of classified frame failures (TROT drops SUPPORT_INVALID)'),
        input_sha256=inputs, summary=summary, cases=audits)


def main():
    result = build()
    (OUT / 'full_case_audit.json').write_text(json.dumps(result, indent=2, allow_nan=False) + '\n')
    s = result['summary']
    print(json.dumps({k: v for k, v in s.items()}, indent=1))
    return 0


if __name__ == '__main__':
    sys.exit(main())
