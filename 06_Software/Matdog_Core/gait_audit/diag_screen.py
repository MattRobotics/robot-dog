"""Diagnostic re-screen of the saved broad and refinement grids that quarantines foot-mesh micrometre contact.

NOT an acceptance gate and NOT a validation. Every saved screen case is re-evaluated at its original
sampling with the same code, but a foot_link ground event of at most the declared 10 um contact-patch
band (PENETRATION up to PATCH_TOL, or the +/-1 um undeclared-contact band) is recorded as
FOOT_MESH_MICROMETRE instead of GROUND_COLLISION / CONTACT_INVALID. The point is to separate what limits
the sampled envelope in the robot's kinematics (IK, joint limits, conditioning, non-foot ground contact,
static support) from the strict micrometre foot-mesh policy. Self-collision is omitted, as in the original
screen. Every other ground event keeps its normal classification.
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import survey
from core import FIELDS
from survey import OUT, phases, save
from model import PATCH_TOL


def diag(task):
    idx, p, n = task
    p = np.array(p); old = None; cats = set(); frames = 0; max_foot_pen = 0.0; margin = None; cond = 0.0
    for s in phases(p, n):
        f = survey._core.frame(p, s, previous=old)
        if 'q' not in f:
            cats.add(f['status']); break
        old = f; frames += 1
        margin = f['joint_margin'] if margin is None else min(margin, f['joint_margin']); cond = max(cond, max(f['condition']))
        active = [l for i, l in enumerate(survey.LEGS) if f['leg_phase'][i, 1] or f['leg_phase'][i, 2]]
        a = survey._model.evaluate(f['q'], f['body'], 'FOOT_SUPPORT', active, full=False)
        errors = list(a['errors'])
        if p[0] == 0 and (a['support_margin_m'] is None or a['support_margin_m'] <= 0) and 'SUPPORT_INVALID' not in errors:
            errors.append('SUPPORT_INVALID')
        kept = []
        for e in errors:
            kind, _, link = e.partition(':')
            if link.endswith('_foot_link'):
                z = a['ground_min_m'][link]
                if kind == 'GROUND_PENETRATION' and -z <= PATCH_TOL:
                    max_foot_pen = max(max_foot_pen, -z); cats.add('FOOT_MESH_MICROMETRE'); continue
                if kind == 'UNDECLARED_GROUND_CONTACT':
                    cats.add('FOOT_MESH_MICROMETRE'); continue
            kept.append(e)
        cats.update(survey.classify(kept, p[0]))
    quarantined = cats - {'FOOT_MESH_MICROMETRE'}
    return dict(id=idx, parameters=dict(zip(FIELDS, p)), frames=frames, categories=sorted(cats), max_foot_penetration_um=max_foot_pen * 1e6,
                kinematic_diagnostic_pass=not quarantined, min_joint_margin_rad=margin, max_condition=cond)


def main():
    tasks = []
    for name, n in (('screen.json', 40), ('refinement_screen.json', 80)):
        for c in json.loads((OUT / name).read_text())['cases']:
            tasks.append((c['id'], [c['parameters'][k] for k in FIELDS], n))
    with ProcessPoolExecutor(max_workers=3, initializer=survey.initialize) as pool:
        rows = list(pool.map(diag, tasks, chunksize=8))
    save('kinematic_diagnostic_screen.json', dict(
        scope='DIAGNOSTIC re-screen: foot-link ground events within the 10 um patch band are quarantined as FOOT_MESH_MICROMETRE. Not acceptance; no self-collision check.',
        quarantine_threshold_um=PATCH_TOL * 1e6, cases=rows))
    print(len(rows), sum(r['kinematic_diagnostic_pass'] for r in rows), 'diagnostic passes;',
          'worst quarantined foot penetration %.3f um' % max(r['max_foot_penetration_um'] for r in rows))


if __name__ == '__main__':
    sys.exit(main())
