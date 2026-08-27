"""FASE D/M11 — composite scoring with explicit weights + sensitivity analysis."""
import csv, itertools, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import qc_common as q

# Feature -> (direction, description). direction=+1 means "higher is worse".
# per-feature resolution floor: the smallest difference that is physically
# meaningful. Without it, a feature whose MAD is 0 (most units identical)
# produces an unbounded z and dominates every score.
FLOOR = {"hs_crossspeed": 1.0, "focal_width": 16.0, "focal_peak": 5.0,
         "min_zones_lt50": 1.0, "min_dir_asym": 2.0, "prec_mae": 0.5,
         "hyst_med": 1.0, "med_offtarget_n": 1.0, "max_tps_deficit": 2.0}

FEATURES = {
    "hs_crossspeed":    (+1, "cross-speed repeatable spatial hotspots"),
    "focal_width":      (+1, "width (ticks) of CONFIRMED focal load regions"),
    "focal_peak":       (+1, "peak focal load excess in a CONFIRMED region"),
    "min_zones_lt50":   (+1, "min-speed zones below 50% of population travel"),
    "min_dir_asym":     (+1, "min-speed directional asymmetry (ticks)"),
    "prec_mae":         (+1, "precision mean absolute error (ticks)"),
    "hyst_med":         (+1, "median directional hysteresis (ticks)"),
    "med_offtarget_n":  (+1, "45-deg medium moves settling >3 ticks off target"),
    "max_tps_deficit":  (+1, "MAX speed deficit vs population median (%)"),
}
# metrics a 50% output cap is expected to move on its own -> excluded from the
# mechanical-evidence view so a configuration difference cannot masquerade as damage
CAP_CONTAMINATED = {"prec_mae", "hyst_med", "min_zones_lt50", "min_dir_asym",
                    "max_tps_deficit", "med_offtarget_n"}

SCHEMES = {
    "A_spatial_led":   dict(hs_crossspeed=3.0, focal_width=3.0, focal_peak=3.0, min_zones_lt50=1.5,
                            min_dir_asym=1.0, prec_mae=1.0, hyst_med=0.5,
                            med_offtarget_n=1.0, max_tps_deficit=1.0),
    "B_uniform":       {k: 1.0 for k in FEATURES},
    "C_precision_led": dict(hs_crossspeed=1.0, focal_width=1.0, focal_peak=1.0, min_zones_lt50=1.0,
                            min_dir_asym=1.0, prec_mae=3.0, hyst_med=3.0,
                            med_offtarget_n=2.0, max_tps_deficit=2.0),
    "D_mechanical_only": {k: (0.0 if k in CAP_CONTAMINATED else 1.0) for k in FEATURES},
}


def kendall_tau(a, b):
    n, num = len(a), 0
    for i, j in itertools.combinations(range(n), 2):
        num += np.sign(a[i] - a[j]) * np.sign(b[i] - b[j])
    return num / (n * (n - 1) / 2)


def main():
    from collections import defaultdict
    units = {r["scode"]: r for r in csv.DictReader(open(q.AUDIT / "blind_metrics.csv"))}
    bins = list(csv.DictReader(open(q.AUDIT / "blind_spatial_bins.csv")))
    minp = list(csv.DictReader(open(q.AUDIT / "blind_min_probes.csv")))

    # focal load excess: worst 16-tick position, averaged over all independent passes
    focal = defaultdict(lambda: defaultdict(list))
    for b in bins:
        if b["bin_w"] == "16":
            focal[b["scode"]][int(b["pos_lo"])].append(
                float(b["load_med"]) - float(b["load_med_pop_med"]))
    focal_best = {}
    for sc, d in focal.items():
        vals = [(np.mean(v), p, len(v)) for p, v in d.items() if len(v) >= 4]
        focal_best[sc] = max(vals) if vals else (0.0, -1, 0)

    poptrav = defaultdict(list)
    for r in minp:
        poptrav[(r["zone"], r["direction"])].append(float(r["directional_travel"]))

    focalrows = list(csv.DictReader(open(q.AUDIT / "blind_focal_load.csv")))
    fw, fp = defaultdict(float), defaultdict(float)
    for r in focalrows:
        if r["verdict"] == "FOCAL_CONFIRMED":
            fw[r["scode"]] += float(r["width_ticks"])
            fp[r["scode"]] = max(fp[r["scode"]], float(r["peak_excess"]))

    rows = []
    for sc in q.scodes():
        u = units[sc]
        ex, pos, npass = focal_best.get(sc, (0.0, -1, 0))
        nlow = sum(1 for r in minp if r["scode"] == sc and float(r["directional_travel"])
                   < 0.5 * np.median(poptrav[(r["zone"], r["direction"])]))
        rows.append(dict(
            scode=sc,
            hs_crossspeed=float(u["hs_crossspeed"]),
            focal_width=fw[sc], focal_peak=fp[sc],
            focal_excess_any=round(float(ex), 2), focal_pos=pos, focal_passes=npass,
            min_zones_lt50=float(nlow),
            min_dir_asym=float(u["min_dir_asym"]),
            prec_mae=float(u["prec_mae"]),
            hyst_med=float(u["hyst_med"]),
            med_offtarget_n=float(u["med_offtarget_n"]),
            max_tps=float(u["max_tps"]),
            load_ceiling=500.0 if float(u["max_load"]) <= 600 else 1000.0))

    popmax = np.median([r["max_tps"] for r in rows])
    for r in rows:
        r["max_tps_deficit"] = round(100.0 * (popmax - r["max_tps"]) / popmax, 2)

    # Normalised excess over the population median, scaled by a spread that
    # cannot collapse to zero. Only "worse than typical" accrues score.
    for f, (sign, _) in FEATURES.items():
        v = np.array([r[f] for r in rows], float)
        p50, p90 = np.percentile(v, 50), np.percentile(v, 90)
        scale = max(p90 - p50, FLOOR[f])
        for r, x in zip(rows, v):
            r[f"z_{f}"] = round(float(max(0.0, sign * (x - p50) / scale)), 3)

    for name, w in SCHEMES.items():
        tot = sum(w.values()) or 1.0
        for r in rows:
            r[f"score_{name}"] = round(sum(w[f] * r[f"z_{f}"] for f in FEATURES) / tot, 4)

    order = {n: [r["scode"] for r in sorted(rows, key=lambda r: -r[f"score_{n}"])]
             for n in SCHEMES}
    ranks = {n: {s: i + 1 for i, s in enumerate(o)} for n, o in order.items()}
    borda = {r["scode"]: sum(ranks[n][r["scode"]] for n in SCHEMES) for r in rows}
    for r in rows:
        for n in SCHEMES:
            r[f"rank_{n}"] = ranks[n][r["scode"]]
        r["rank_borda"] = sorted(borda, key=lambda s: borda[s]).index(r["scode"]) + 1
        rk = [ranks[n][r["scode"]] for n in SCHEMES]
        r["rank_min"], r["rank_max"] = min(rk), max(rk)
        r["rank_spread"] = max(rk) - min(rk)
        r["rank_stable"] = r["rank_spread"] <= 3

    rows.sort(key=lambda r: r["rank_borda"])
    keys = list(rows[0].keys())
    with open(q.AUDIT / "ranking_sensitivity.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader(); w.writerows(rows)

    print("=== composite scores under 4 weightings (higher = more anomalous) ===")
    print(f"{'unit':5s} {'A_spat':>7s} {'B_unif':>7s} {'C_prec':>7s} {'D_mech':>7s} "
          f"{'rA':>3s} {'rB':>3s} {'rC':>3s} {'rD':>3s} {'borda':>5s} {'spread':>6s} {'stable':>6s} {'cap':>5s}")
    for r in rows:
        print(f"{r['scode']:5s} {r['score_A_spatial_led']:7.3f} {r['score_B_uniform']:7.3f} "
              f"{r['score_C_precision_led']:7.3f} {r['score_D_mechanical_only']:7.3f} "
              f"{r['rank_A_spatial_led']:3d} {r['rank_B_uniform']:3d} "
              f"{r['rank_C_precision_led']:3d} {r['rank_D_mechanical_only']:3d} "
              f"{r['rank_borda']:5d} {r['rank_spread']:6d} {str(r['rank_stable']):>6s} "
              f"{int(r['load_ceiling']):5d}")

    print("\n=== Kendall tau between weightings ===")
    names = list(SCHEMES)
    scs = [r["scode"] for r in rows]
    for a, b in itertools.combinations(names, 2):
        ta = [ranks[a][s] for s in scs]; tb = [ranks[b][s] for s in scs]
        print(f"  {a:20s} vs {b:20s} tau = {kendall_tau(ta, tb):+.3f}")

    print("\n=== raw feature values ===")
    fs = list(FEATURES)
    print(f"{'unit':5s} "+' '.join(f'{f[:13]:>13s}' for f in fs)+f" {'focal_pos':>9s}")
    for r in rows:
        print(f"{r['scode']:5s} "+' '.join(f'{r[f]:13.2f}' for f in fs)+f" {r['focal_pos']:9d}")


if __name__ == "__main__":
    main()
