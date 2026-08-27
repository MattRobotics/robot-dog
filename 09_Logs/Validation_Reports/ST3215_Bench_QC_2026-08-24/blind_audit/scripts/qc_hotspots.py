"""FASE C5 — hotspot clustering with boundary rejection and cross-speed intersection."""
import csv, sys
from collections import defaultdict
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import qc_common as q

Z_SPEED = -3.0
Z_STRONG = -5.0


def load(name, numeric):
    rows = list(csv.DictReader(open(q.AUDIT / name)))
    for r in rows:
        for k in numeric:
            try:
                r[k] = float(r[k])
            except (ValueError, KeyError):
                r[k] = np.nan
    return rows


def main():
    bins = load("blind_spatial_bins.csv",
                ["direction", "bin_w", "bin_idx", "pos_lo", "pos_hi", "n", "span",
                 "dwell_ms", "speed_tps", "load_med", "cur_med", "pop_n",
                 "speed_tps_z", "speed_tps_relpct", "load_med_z", "cur_med_z",
                 "dwell_ms_z", "speed_tps_pop_med", "load_med_pop_med",
                 "cur_med_pop_med", "idx_start", "idx_end", "t_start_us",
                 "t_end_us", "move_idx"])
    moves = load("blind_move_metrics.csv", ["move_idx", "start_pos", "target"])
    mv = {(m["scode"], int(m["move_idx"])): m for m in moves}

    # a bin touching the commanded start or target holds accel/decel -> never a hotspot
    for b in bins:
        m = mv[(b["scode"], int(b["move_idx"]))]
        b["boundary"] = (b["pos_lo"] <= m["start_pos"] <= b["pos_hi"] or
                         b["pos_lo"] <= m["target"] <= b["pos_hi"])

    cand = [b for b in bins
            if b["bin_w"] in (32.0, 64.0) and b["pop_n"] >= 16
            and not b["boundary"] and b["speed_tps_z"] <= Z_SPEED
            and b["group"] in ("SLOW", "MEDIUM", "TRANSFER")]

    # cluster spatially adjacent candidate bins within one traversal
    groups = defaultdict(list)
    for b in cand:
        groups[(b["scode"], b["group"], b["direction"], b["bin_w"], int(b["move_idx"]))].append(b)

    clusters = []
    for k, bs in groups.items():
        bs.sort(key=lambda r: r["bin_idx"])
        run = [bs[0]]
        for b in bs[1:]:
            if b["bin_idx"] == run[-1]["bin_idx"] + 1:
                run.append(b)
            else:
                clusters.append((k, run)); run = [b]
        clusters.append((k, run))

    out = []
    for (sc, grp, dirn, bw, mi), run in clusters:
        lo = min(b["pos_lo"] for b in run)
        hi = max(b["pos_hi"] for b in run)
        out.append(dict(
            scode=sc, group=grp, direction=int(dirn), bin_w=int(bw), move_idx=mi,
            n_bins=len(run), pos_lo=int(lo), pos_hi=int(hi),
            raw_idx_start=int(min(b["idx_start"] for b in run)),
            raw_idx_end=int(max(b["idx_end"] for b in run)),
            t_start_us=int(min(b["t_start_us"] for b in run)),
            t_end_us=int(max(b["t_end_us"] for b in run)),
            worst_speed_z=round(min(b["speed_tps_z"] for b in run), 2),
            speed_tps=round(float(np.mean([b["speed_tps"] for b in run])), 1),
            baseline_tps=round(float(np.mean([b["speed_tps_pop_med"] for b in run])), 1),
            deficit_pct=round(float(np.mean([b["speed_tps_relpct"] for b in run])), 1),
            load_med=round(float(np.mean([b["load_med"] for b in run])), 1),
            load_baseline=round(float(np.mean([b["load_med_pop_med"] for b in run])), 1),
            load_z=round(float(np.mean([b["load_med_z"] for b in run])), 2),
            cur_med=round(float(np.mean([b["cur_med"] for b in run])), 2),
            cur_baseline=round(float(np.mean([b["cur_med_pop_med"] for b in run])), 2),
        ))

    # cross-speed / cross-direction repeatability on a common 64-tick grid
    cov = defaultdict(set)
    for c in out:
        for g in range(c["pos_lo"] // 64, c["pos_hi"] // 64 + 1):
            cov[(c["scode"], g)].add((c["group"], c["direction"]))
    for c in out:
        gs = range(c["pos_lo"] // 64, c["pos_hi"] // 64 + 1)
        ctx = set().union(*[cov[(c["scode"], g)] for g in gs])
        c["seen_in_groups"] = "|".join(sorted({g for g, _ in ctx}))
        c["seen_in_directions"] = "|".join(sorted({str(d) for _, d in ctx}))
        c["n_independent_passes"] = len(ctx)
        c["cross_speed"] = ("SLOW" in {g for g, _ in ctx} and
                            "MEDIUM" in {g for g, _ in ctx})
        c["bidirectional"] = len({d for _, d in ctx}) > 1

        strong = c["worst_speed_z"] <= Z_STRONG
        if c["n_bins"] >= 2 and c["cross_speed"] and c["n_independent_passes"] >= 3:
            c["confidence"] = "HIGH"
        elif c["n_bins"] >= 2 and (c["cross_speed"] or c["n_independent_passes"] >= 3):
            c["confidence"] = "MEDIUM"
        elif c["n_bins"] >= 2 or strong:
            c["confidence"] = "LOW"
        else:
            c["confidence"] = "REJECTED_SINGLE_BIN"

    out.sort(key=lambda c: (c["scode"], c["pos_lo"]))
    keys = list(out[0].keys())
    with open(q.AUDIT / "blind_anomalies.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader(); w.writerows(out)

    print(f"{len(cand)} candidate bins -> {len(out)} clusters")
    kept = [c for c in out if c["confidence"] != "REJECTED_SINGLE_BIN"]
    print(f"{len(kept)} survive the >=2-adjacent-bins / strong-z filter\n")
    per = defaultdict(lambda: defaultdict(int))
    for c in out:
        per[c["scode"]][c["confidence"]] += 1
    print(f"{'unit':6s} {'HIGH':>5s} {'MED':>5s} {'LOW':>5s} {'rejected':>9s}")
    for sc in q.scodes():
        p = per[sc]
        print(f"{sc:6s} {p['HIGH']:5d} {p['MEDIUM']:5d} {p['LOW']:5d} "
              f"{p['REJECTED_SINGLE_BIN']:9d}")


if __name__ == "__main__":
    main()
