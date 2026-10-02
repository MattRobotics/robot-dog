"""Sampling-density sensitivity of the strict ground/contact policy (ground and support checks only).

The unchanged 1 um policy forbids a swing foot's mesh within 1 um of the ground away from the exact
lift-off/touchdown boundary. Any continuous swing foot leaves and reaches the ground, so a sampled
check must hit that band once the sampling interval is shorter than the band's duration. This tool
re-evaluates the cases that passed the saved sampling at several sampling densities. Self-collision
is omitted (clearances are tens of millimetres); nothing here replaces a full-mesh validation.
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import survey
from core import FIELDS
from survey import OUT, phases, save

DENSITIES = (80, 200, 400, 800, 1600)


def probe(task):
    case_id, p, densities = task
    p = np.array(p)
    result = {}
    for n in densities:
        old = None; failing = []; frames = 0; band_hits = 0
        for s in phases(p, n):
            f = survey._core.frame(p, s, previous=old)
            if 'q' not in f:
                failing.append(dict(phase=s, errors=[f['status']])); break
            old = f
            a, why = survey.check_frame(f, p, False)
            frames += 1
            kinds = [e.split(':')[0] for e in a['errors'] if not (e == 'SUPPORT_INVALID' and p[0] == 1)]
            if kinds:
                band_hits += any(k == 'UNDECLARED_GROUND_CONTACT' for k in kinds)
                failing.append(dict(phase=s, errors=[e for e in a['errors'] if not (e == 'SUPPORT_INVALID' and p[0] == 1)],
                                    foot_mesh_min_z_um={k[:2]: round(v * 1e6, 3) for k, v in a['ground_min_m'].items() if k.endswith('foot_link')}))
        penetration = sum(any(e.startswith('GROUND_PENETRATION') for e in fr['errors']) for fr in failing)
        result[str(n)] = dict(frames=frames, failing_frames=len(failing), passes=not failing, undeclared_contact_frames=band_hits,
                              penetration_frames=penetration, first_failures=failing[:3])
    return dict(id=case_id, densities=result)


def main():
    audit = json.loads((OUT / 'full_case_audit.json').read_text())
    sweep = {s['id']: s for s in json.loads((OUT / 'contact_discrepancy.json').read_text())['steady_cases']}
    tasks = []
    for a in audit['cases']:
        if a['FULL_MESH_VALIDATED']:
            tasks.append((a['id'], [a['parameters'][k] for k in FIELDS], DENSITIES))
    with ProcessPoolExecutor(max_workers=4, initializer=survey.initialize) as pool:
        rows = list(pool.map(probe, tasks))
    by_id = {r['id']: r for r in rows}
    summary = {str(n): dict(cases=len(rows), passing=sum(r['densities'][str(n)]['passes'] for r in rows)) for n in DENSITIES}
    for r in rows:
        r['type'] = next(a['type'] for a in audit['cases'] if a['id'] == r['id'])
        r['pitch_sweep_check'] = sweep[r['id']]['pitch_interval_check']
    save('dense_ground_probe.json', dict(
        scope='Ground-contact and support policy only, cases that passed the saved sampling; evidence of sampling-density sensitivity, not a validation',
        densities=list(DENSITIES), pass_counts_by_density=summary, cases=rows))
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
