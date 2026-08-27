"""FASE C4/C5 — spatial binning (512/64/32/16 ticks) and cross-speed hotspots."""
import csv, sys
from collections import defaultdict
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import qc_common as q
from qc_metrics import velocity

GROUPS = {3: "SLOW", 4: "SLOW", 11: "MEDIUM", 10: "TRANSFER"}
BINS = (512, 64, 32, 16)
MIN_N = 4                 # samples required in a bin
MIN_SPAN_FRAC = 0.5       # fraction of the bin width that must actually be traversed


def cruise_mask(pos, t, k=10):
    v = velocity(pos, t, k)
    av = np.abs(v)
    if np.isfinite(av).sum() < 5:
        return np.zeros(len(pos), dtype=bool), np.nan
    plateau = np.nanpercentile(av, 90)
    c = np.isfinite(av) & (av >= 0.6 * plateau)
    idx = np.flatnonzero(c)
    m = np.zeros(len(pos), dtype=bool)
    if idx.size >= 5:
        m[idx[0]:idx[-1] + 1] = True
        m &= np.isfinite(av)
    return m, plateau


def main():
    rows = []
    for sc in q.scodes():
        d = q.load_raw(sc)
        segs = q.segment_moves(d)
        meas = [(s, e) for (s, e) in segs
                if d["goal_pos"][s] != q.GOAL_SENTINEL_POS
                and not (d["goal_spd"][s] == 100 and d["goal_acc"][s] == 10)]

        for mi, (s, e) in enumerate(meas):
            ph = int(d["phase"][s])
            if ph not in GROUPS:
                continue
            pos = d["pos"][s:e].astype(np.float64)
            t = d["t_us"][s:e]
            target = int(d["goal_pos"][s])
            direction = 1 if target > pos[0] else (-1 if target < pos[0] else 0)
            if direction == 0 or abs(target - pos[0]) < 64:
                continue
            cm, plateau = cruise_mask(pos, t)
            if cm.sum() < 8:
                continue
            ld = np.abs(d["load"][s:e]).astype(np.float64)
            cu = np.abs(d["current"][s:e]).astype(np.float64)
            ci = np.flatnonzero(cm)
            p, tt, l, c = pos[ci], t[ci], ld[ci], cu[ci]

            for bw in BINS:
                b = (p // bw).astype(np.int64)
                order = np.argsort(b, kind="stable")
                bs, ps, ts, ls, cs = b[order], p[order], tt[order], l[order], c[order]
                edges = np.flatnonzero(np.diff(bs)) + 1
                for a, z in zip(np.concatenate([[0], edges]),
                                np.concatenate([edges, [len(bs)]])):
                    n = z - a
                    if n < MIN_N:
                        continue
                    span = ps[a:z].max() - ps[a:z].min()
                    dur = (ts[a:z].max() - ts[a:z].min()) / 1e6
                    if span < MIN_SPAN_FRAC * bw or dur <= 0:
                        continue
                    rows.append(dict(
                        scode=sc, group=GROUPS[ph], phase=ph, move_idx=mi,
                        direction=direction, bin_w=bw, bin_idx=int(bs[a]),
                        pos_lo=int(bs[a]) * bw, pos_hi=(int(bs[a]) + 1) * bw - 1,
                        n=int(n), span=float(span), dwell_ms=float(dur * 1000),
                        speed_tps=float(span / dur),
                        load_med=float(np.median(ls[a:z])),
                        load_p95=float(np.percentile(ls[a:z], 95)),
                        cur_med=float(np.median(cs[a:z])),
                        cur_p95=float(np.percentile(cs[a:z], 95)),
                        idx_start=int(s + ci[order[a]]), idx_end=int(s + ci[order[z - 1]]),
                        t_start_us=int(ts[a:z].min()), t_end_us=int(ts[a:z].max())))
        print(f"{sc} binned", flush=True)

    # ---- population robust baseline at identical (group, direction, bin) ----
    key = lambda r: (r["group"], r["direction"], r["bin_w"], r["bin_idx"])
    pool = defaultdict(list)
    for r in rows:
        pool[key(r)].append(r)

    for k, rs in pool.items():
        for metric in ("speed_tps", "load_med", "cur_med", "dwell_ms"):
            vals = np.array([r[metric] for r in rs], dtype=np.float64)
            med = np.median(vals)
            mad = np.median(np.abs(vals - med))
            floor = max(mad, 0.01 * abs(med), 1e-6)
            for r, v in zip(rs, vals):
                r[f"{metric}_pop_med"] = round(float(med), 4)
                r[f"{metric}_z"] = round(float(0.6745 * (v - med) / floor), 3)
                r[f"{metric}_relpct"] = round(float(100.0 * (v - med) / med), 3) if med else np.nan
        for r in rs:
            r["pop_n"] = len(rs)

    keys = sorted({k for r in rows for k in r})
    keys = ["scode", "group", "direction", "bin_w", "bin_idx", "pos_lo", "pos_hi"] + \
           [k for k in keys if k not in ("scode", "group", "direction", "bin_w",
                                         "bin_idx", "pos_lo", "pos_hi")]
    with open(q.AUDIT / "blind_spatial_bins.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader(); w.writerows(rows)
    print(f"\nwrote blind_spatial_bins.csv ({len(rows)} bins, "
          f"{len(pool)} population cells)")


if __name__ == "__main__":
    main()
