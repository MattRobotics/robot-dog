"""DIAGNOSTIC full-mesh frontier of the micrometre-quarantined screen (self-collision included).

The kinematic diagnostic screen omits self-collision. This evaluates, with the same full canonical mesh
model and the same 80 intervals as the original full sets, the largest-stride diagnostic passes at each
primary height and direction. Foot-link ground events within the 10 um patch band are quarantined, so the result says what limits the stride once the micrometre foot-mesh
policy is set aside. It is NOT acceptance and does not alter the strict policy or any saved result.
"""
import json
import sys
from concurrent.futures import ProcessPoolExecutor
import survey
from core import FIELDS
from survey import OUT, save
from model import PATCH_TOL

_original = survey.check_frame


def quarantining_check(f, p, full):
    a, why = _original(f, p, full)
    kept, quarantined = [], []
    for e in a['errors']:
        kind, _, link = e.partition(':')
        if link.endswith('_foot_link') and ((kind == 'GROUND_PENETRATION' and -a['ground_min_m'][link] <= PATCH_TOL) or kind == 'UNDECLARED_GROUND_CONTACT'):
            quarantined.append(e)
        else:
            kept.append(e)
    a['quarantined'] = quarantined
    a['errors'] = kept
    return a, survey.classify(kept, p[0])


def init():
    survey.initialize()
    survey.check_frame = quarantining_check


def frontier():
    rows = json.loads((OUT / 'kinematic_diagnostic_screen.json').read_text())['cases']
    pick = {}
    for r in rows:
        p = r['parameters']
        if not r['kinematic_diagnostic_pass'] or p['advance_y_m'] != 0 or p['yaw_rad'] != 0 or p['height_m'] not in (.08, .1, .12, .14, .15):
            continue
        d = 1 if p['advance_x_m'] > 0 else -1
        if p['advance_x_m'] == 0:
            continue
        key = (p['type'], p['height_m'], d)
        rank = (abs(p['advance_x_m']), -p['lift_m'], -abs(p['duty'] - (.8 if p['type'] == 0 else .6)))
        if key not in pick or rank > pick[key][0]:
            pick[key] = (rank, r)
    return [v[1] for k, v in sorted(pick.items())]


def main():
    tasks = [(r['id'], [r['parameters'][k] for k in FIELDS], True, 80) for r in frontier()]
    results = []
    with ProcessPoolExecutor(max_workers=4, initializer=init) as pool:
        for r in pool.map(survey.evaluate, tasks):
            results.append(r)
            print('frontier', r['id'], r['parameters']['type'], r['parameters']['height_m'], r['parameters']['advance_x_m'], r['classification'], flush=True)
            save('diag_frontier_full.json', dict(
                scope='DIAGNOSTIC full-mesh frontier with foot-link micrometre events quarantined ; not acceptance',
                planned_cases=len(tasks), complete=len(results) == len(tasks), quarantine_threshold_um=PATCH_TOL * 1e6, cases=results))


if __name__ == '__main__':
    sys.exit(main())
