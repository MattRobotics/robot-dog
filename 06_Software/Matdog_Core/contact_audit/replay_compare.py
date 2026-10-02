"""Deterministic replay: re-run the 2 ms lifecycle audits from scratch and require bit-identical per-frame results."""
import csv
import sys
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from common import OUT
import lifecycle_v2 as lc

COLS = {'t': 't', 'jm': 'jm', 'cond': 'cond', 'resid': 'resid', 'nonfoot': 'nonfoot', 'sep': 'sep', 'min_delta_tread_um': 'min_delta_tread_um', 'v1_min_foot_z_um': 'v1_min_foot_z_um', 'support': 'support'}


def one(case):
    out, rec, *_ = lc.run(case, 'G2_NOMINAL', 0.002)
    with (OUT / f'lifecycle_{case}_G2_NOMINAL.csv').open() as fh:
        rows = list(csv.DictReader(fh))
    worst = 0.0
    for k, col in COLS.items():
        saved = np.array([float(r[col]) if r[col] != '' else np.nan for r in rows])
        now = np.array([np.nan if v is None else v for v in rec[k]], dtype=float)
        if saved.shape != now.shape:
            return case, float('inf')
        same = (saved == now) | (np.isnan(saved) & np.isnan(now))
        worst = max(worst, 0.0 if same.all() else float(np.nanmax(np.abs(saved - now))))
    return case, worst


def main():
    with ProcessPoolExecutor(max_workers=4) as pool:
        results = list(pool.map(one, ['WALK_357', 'WALK_287', 'TROT_61', 'TROT_309']))
    for case, worst in results:
        print('REPLAY', case, 'max difference', worst)
    ok = all(w == 0.0 for _, w in results)
    print('REPLAY_COMPARE', 'PASS' if ok else 'FAIL')
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
