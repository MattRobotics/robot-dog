"""
Focal-load anomaly detector.

Rationale: the speed-led hotspot rule in qc_hotspots.py under-detects. A local
mechanical resistance shows up most reliably as EXTRA OUTPUT (load) at a fixed
encoder position, because the position controller compensates and partly hides
the speed deficit. Load excess is measured against the population at the SAME
16-tick position, so protocol/common-mode effects cancel.

The detection threshold is derived from the pooled population distribution of
the statistic itself, not chosen by hand.
"""
import csv, sys
from collections import defaultdict
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import qc_common as q

MIN_PASSES = 4          # independent traversals of the position required
MIN_WIDTH_TICKS = 48    # a real mechanical feature spans more than one bin


def main():
    bins = list(csv.DictReader(open(q.AUDIT / "blind_spatial_bins.csv")))
    prof = defaultdict(lambda: defaultdict(list))
    spd = defaultdict(lambda: defaultdict(list))
    for b in bins:
        if b["bin_w"] != "16":
            continue
        p = int(float(b["pos_lo"]))
        prof[b["scode"]][p].append(float(b["load_med"]) - float(b["load_med_pop_med"]))
        spd[b["scode"]][p].append(float(b["speed_tps_relpct"]))

    pooled = np.array([np.mean(v) for d in prof.values()
                       for v in d.values() if len(v) >= MIN_PASSES])
    med, mad = np.median(pooled), np.median(np.abs(pooled - np.median(pooled)))
    sd = 1.4826 * mad
    thresh = float(np.percentile(pooled, 99.5))
    print(f"pooled per-position mean load excess (n={pooled.size}): "
          f"median={med:.2f} robust_sd={sd:.2f} p99={np.percentile(pooled,99):.2f} "
          f"p99.5={thresh:.2f} max={pooled.max():.2f}")
    print(f"detection threshold = p99.5 of the pooled population = {thresh:.2f} "
          f"(= {(thresh-med)/sd:.1f} robust SD)\n")

    out = []
    for sc in q.scodes():
        pts = sorted(p for p, v in prof[sc].items() if len(v) >= MIN_PASSES)
        hot = [p for p in pts if np.mean(prof[sc][p]) >= thresh]
        runs, cur = [], []
        for p in hot:
            if cur and p == cur[-1] + 16:
                cur.append(p)
            else:
                if cur:
                    runs.append(cur)
                cur = [p]
        if cur:
            runs.append(cur)
        for r in runs:
            width = r[-1] + 16 - r[0]
            exc = [np.mean(prof[sc][p]) for p in r]
            peak = r[int(np.argmax(exc))]
            sp = [np.mean(spd[sc][p]) for p in r]
            out.append(dict(
                scode=sc, pos_lo=r[0], pos_hi=r[-1] + 15, width_ticks=width,
                n_positions=len(r), passes=int(np.min([len(prof[sc][p]) for p in r])),
                peak_pos=peak, peak_excess=round(float(max(exc)), 1),
                mean_excess=round(float(np.mean(exc)), 1),
                speed_rel_pct=round(float(np.mean(sp)), 1),
                deg_lo=round(r[0] / 4096 * 360, 1), deg_hi=round((r[-1] + 16) / 4096 * 360, 1),
                verdict=("FOCAL_CONFIRMED" if width >= MIN_WIDTH_TICKS and len(r) >= 3
                         else "FOCAL_WEAK")))

    with open(q.AUDIT / "blind_focal_load.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(out[0].keys()))
        w.writeheader(); w.writerows(out)

    print(f"{'unit':5s} {'pos range':>13s} {'width':>6s} {'npos':>5s} {'passes':>7s} "
          f"{'peak@':>6s} {'peak':>6s} {'mean':>6s} {'spd%':>6s} {'deg':>13s} {'verdict':>16s}")
    for r in out:
        print(f"{r['scode']:5s} {str(r['pos_lo'])+'-'+str(r['pos_hi']):>13s} "
              f"{r['width_ticks']:6d} {r['n_positions']:5d} {r['passes']:7d} "
              f"{r['peak_pos']:6d} {r['peak_excess']:6.1f} {r['mean_excess']:6.1f} "
              f"{r['speed_rel_pct']:6.1f} {str(r['deg_lo'])+'-'+str(r['deg_hi']):>13s} "
              f"{r['verdict']:>16s}")
    print("\nunits with NO focal load region:",
          " ".join(s for s in q.scodes() if not any(r["scode"] == s for r in out)))


if __name__ == "__main__":
    main()
