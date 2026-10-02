"""Targeted search for sampling-density-robust gait parameter sets (new samples, saved separately).

Motivation: the saved full-mesh passes are sampled sets; the strict policy is sampling-density
sensitive (see dense_ground_probe.py). This looks for parameter sets whose ground-contact and support
checks still pass at dense sampling, so that a representative does not rest on a sampling coincidence.

Stage 1: ground/support checks at 400 intervals with early exit.
Stage 2: survivors at 1600 intervals with early exit.
Stage 3 (dense_representatives.py / select_representatives.py): full mesh on the final choices.
Self-collision is omitted in stages 1-2 (a screen), exactly as in the original screen stage.
"""
import itertools
import sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
import survey
from core import params, FIELDS
from survey import OUT, phases, save


def grid():
    cases = []
    for h, lift, duty, x in itertools.product((.08, .09, .1, .11, .12), (.005, .01), (.5, .6, .7, .8), (-.04, -.03, -.02, -.01, .01, .02, .03, .04)):
        cases.append(params(kind=1, height=h, lift=lift, duty=duty, x=x))
    for h, lift, duty, x in itertools.product((.08, .09, .1, .11, .12), (.005, .01), (.75, .8, .9), (-.02, -.015, -.01, -.005, .005, .01, .015, .02)):
        cases.append(params(kind=0, height=h, lift=lift, duty=duty, x=x, sway=.004 if duty > .75 else 0))
    return [list(map(float, p)) for p in cases]


def first_failure(task):
    idx, p, n = task
    p = np.array(p); old = None; count = 0
    for s in phases(p, n):
        f = survey._core.frame(p, s, previous=old)
        if 'q' not in f:
            return dict(id=idx, passes=False, frames=count, failure=f['status'], phase=s)
        old = f; count += 1
        a, why = survey.check_frame(f, p, False)
        if why:
            return dict(id=idx, passes=False, frames=count, failure=why, phase=s)
    return dict(id=idx, passes=True, frames=count, failure=None, phase=None)


def main():
    cases = grid()
    with ProcessPoolExecutor(max_workers=4, initializer=survey.initialize) as pool:
        s1 = list(pool.map(first_failure, [(i, p, 400) for i, p in enumerate(cases)], chunksize=4))
        alive = [r['id'] for r in s1 if r['passes']]
        print('stage1', len(cases), '->', len(alive), flush=True)
        s2 = list(pool.map(first_failure, [(i, cases[i], 1600) for i in alive], chunksize=1))
    robust = [r['id'] for r in s2 if r['passes']]
    rows = []
    for i, p in enumerate(cases):
        rows.append(dict(id=i, parameters=dict(zip(FIELDS, p)), stage1_400=s1[i]['passes'], stage1_failure=s1[i]['failure'],
                         stage2_1600=next((r['passes'] for r in s2 if r['id'] == i), None)))
    save('robust_search.json', dict(
        scope='New targeted ground/support screen at 400 and 1600 intervals; no self-collision; not a validation',
        planned=len(cases), stage1_survivors=len(alive), stage2_survivors=len(robust), robust_ids=robust, cases=rows))
    print('stage2', len(alive), '->', len(robust))
    for i in robust:
        p = cases[i]; print(i, 'WALK' if p[0] == 0 else 'TROT', *[round(v, 4) for v in p[1:8]])


if __name__ == '__main__':
    sys.exit(main())
