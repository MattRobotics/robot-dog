"""FASE B/C — per-move motion metrics + telemetry artifact accounting."""
import csv, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import qc_common as q

SLOW = (3, 4)
MEDIUM = (11,)
MAXP = (6, 7)
PREC = (8,)
MINP = (5,)


def velocity(pos, t_us, k):
    """Central difference over +/-k samples -> tick/s. NaN where a gap intrudes."""
    n = len(pos)
    v = np.full(n, np.nan)
    if n < 2 * k + 1:
        return v
    dp = pos[2 * k:].astype(np.float64) - pos[:-2 * k].astype(np.float64)
    dt = (t_us[2 * k:] - t_us[:-2 * k]).astype(np.float64) / 1e6
    ok = dt > 0
    seg = np.full(n - 2 * k, np.nan)
    seg[ok] = dp[ok] / dt[ok]
    # invalidate windows that span a telemetry hole
    gap = np.diff(t_us) > 6000
    if gap.any():
        csum = np.concatenate([[0], np.cumsum(gap)])
        spans = csum[2 * k:] - csum[:-2 * k]
        seg[spans > 0] = np.nan
    v[k:n - k] = seg
    return v


def move_metrics(d, s, e, k):
    pos = d["pos"][s:e].astype(np.float64)
    t = d["t_us"][s:e]
    ld = np.abs(d["load"][s:e]).astype(np.float64)
    cu = np.abs(d["current"][s:e]).astype(np.float64)
    sp = d["speed"][s:e].astype(np.float64)
    vo = d["volt"][s:e].astype(np.float64)
    target = int(d["goal_pos"][s])
    start = pos[0]
    final = pos[-1]
    direction = 1 if target > start else (-1 if target < start else 0)
    t0 = t[0]

    m = dict(n=e - s, start_pos=int(start), target=target, final_pos=int(final),
             direction=direction, residual=int(final - target),
             abs_residual=abs(int(final - target)),
             elapsed_ms=(t[-1] - t0) / 1000.0)

    moved = np.flatnonzero(np.abs(pos - start) >= 2)
    m["first_motion_ms"] = (t[moved[0]] - t0) / 1000.0 if moved.size else np.nan
    m["total_travel"] = float(pos.max() - pos.min())

    v = velocity(pos, t, k)
    av = np.abs(v)
    if np.isfinite(av).sum() < 5:
        m.update(cruise_v_med=np.nan, cruise_v_cv=np.nan, cruise_n=0,
                 cruise_ms=np.nan, cruise_span=np.nan, cruise_v_lin=np.nan,
                 plateau_v=np.nan, accel_ms=np.nan,
                 decel_ms=np.nan, stick_slip=np.nan, dwell_max_ms=np.nan,
                 backtrack_n=0, backtrack_max=0.0, overshoot=0.0,
                 settle_ms=np.nan, load_med=np.nan, load_p95=np.nan,
                 load_max=float(ld.max()), cur_med=np.nan, cur_p95=np.nan,
                 cur_max=float(cu.max()), volt_min=float(vo.min()),
                 volt_med=float(np.median(vo)), speedreg_ratio=np.nan)
        return m

    plateau = np.nanpercentile(av, 90)
    m["plateau_v"] = float(plateau)
    cruise = np.isfinite(av) & (av >= 0.6 * plateau)
    idx = np.flatnonzero(cruise)
    if idx.size >= 5:
        lo, hi = idx[0], idx[-1]
        cm = np.zeros(len(pos), dtype=bool)
        cm[lo:hi + 1] = True
        cm &= np.isfinite(av)
    else:
        cm = np.isfinite(av)
        lo, hi = 0, len(pos) - 1

    cv_vals = av[cm]
    m["cruise_n"] = int(cm.sum())
    m["cruise_ms"] = float((t[hi] - t[lo]) / 1000.0)
    m["cruise_v_med"] = float(np.median(cv_vals)) if cv_vals.size else np.nan
    m["cruise_v_cv"] = float(np.std(cv_vals) / np.mean(cv_vals)) if cv_vals.size and np.mean(cv_vals) > 0 else np.nan
    # quantization-free cruise speed: total encoder span over total cruise time.
    # (central differences at MAX quantize to whole ticks per window and would
    #  fabricate ~4% direction asymmetries -- do NOT use them for MAX.)
    m["cruise_span"] = float(abs(pos[hi] - pos[lo]))
    m["cruise_v_lin"] = (float(m["cruise_span"] / (m["cruise_ms"] / 1000.0))
                         if m["cruise_ms"] > 0 else np.nan)
    m["accel_ms"] = float((t[lo] - t0) / 1000.0)
    m["decel_ms"] = float((t[-1] - t[hi]) / 1000.0)

    # stick-slip and dwell inside cruise
    m["stick_slip"] = float((cv_vals < 0.10 * plateau).mean()) if cv_vals.size else np.nan
    pc = pos[lo:hi + 1]
    tc = t[lo:hi + 1]
    if pc.size > 2:
        chg = np.flatnonzero(np.diff(pc) != 0)
        if chg.size:
            edges = np.concatenate([[0], chg + 1, [len(pc)]])
            dwell = np.diff(tc[np.clip(edges, 0, len(pc) - 1)]) / 1000.0
            m["dwell_max_ms"] = float(dwell.max())
        else:
            m["dwell_max_ms"] = float((tc[-1] - tc[0]) / 1000.0)
    else:
        m["dwell_max_ms"] = np.nan

    # backtracking against the commanded direction, >= 2 ticks
    if direction != 0 and pc.size > 1:
        dp = np.diff(pc) * direction
        neg = dp < 0
        runs, cur = [], 0.0
        for i, isneg in enumerate(neg):
            if isneg:
                cur += -dp[i]
            elif cur:
                runs.append(cur); cur = 0.0
        if cur:
            runs.append(cur)
        runs = [r for r in runs if r >= 2]
        m["backtrack_n"] = len(runs)
        m["backtrack_max"] = float(max(runs)) if runs else 0.0
    else:
        m["backtrack_n"], m["backtrack_max"] = 0, 0.0

    m["overshoot"] = float(max(0.0, (pos.max() - target) if direction > 0
                              else (target - pos.min()) if direction < 0 else 0.0))
    near = np.flatnonzero(np.abs(pos - final) <= 3)
    m["settle_ms"] = float((t[-1] - t[near[0]]) / 1000.0) if near.size else np.nan

    m["load_med"] = float(np.median(ld[cm])) if cm.any() else np.nan
    m["load_p95"] = float(np.percentile(ld[cm], 95)) if cm.any() else np.nan
    m["load_max"] = float(ld.max())
    m["cur_med"] = float(np.median(cu[cm])) if cm.any() else np.nan
    m["cur_p95"] = float(np.percentile(cu[cm], 95)) if cm.any() else np.nan
    m["cur_max"] = float(cu.max())
    m["volt_min"] = float(vo.min())
    m["volt_med"] = float(np.median(vo))

    # PresentSpeed vs encoder-derived velocity (independent channels)
    ok = cm & np.isfinite(v) & (np.abs(sp) > 5) & (np.abs(v) > 1)
    m["speedreg_ratio"] = float(np.median(np.abs(sp[ok]) / np.abs(v[ok]))) if ok.sum() >= 5 else np.nan
    return m


def artifact_runs(x, win=25, thresh=5):
    """Deviation from a rolling median -> lengths of consecutive anomalous runs."""
    n = len(x)
    if n < 2 * win + 1:
        return np.array([]), np.zeros(n, dtype=bool)
    from numpy.lib.stride_tricks import sliding_window_view
    med = np.median(sliding_window_view(x, 2 * win + 1), axis=1)
    dev = np.zeros(n, dtype=bool)
    dev[win:n - win] = np.abs(x[win:n - win] - med) > thresh
    runs, cur = [], 0
    for b in dev:
        if b:
            cur += 1
        elif cur:
            runs.append(cur); cur = 0
    if cur:
        runs.append(cur)
    return np.array(runs), dev


def main():
    moverows, unitrows = [], []
    for sc in q.scodes():
        d = q.load_raw(sc)
        segs = q.segment_moves(d)
        meas = [(s, e) for (s, e) in segs
                if d["goal_pos"][s] != q.GOAL_SENTINEL_POS
                and not (d["goal_spd"][s] == 100 and d["goal_acc"][s] == 10)]

        for i, (s, e) in enumerate(meas):
            ph = int(d["phase"][s])
            k = 2 if ph in MAXP else (3 if ph in PREC else 10)
            m = move_metrics(d, s, e, k)
            m.update(scode=sc, move_idx=i, phase=ph, phase_name=q.PHASE[ph],
                     goal_spd=int(d["goal_spd"][s]), goal_acc=int(d["goal_acc"][s]),
                     idx_start=s, idx_end=e,
                     t_start_us=int(d["t_us"][s]), t_end_us=int(d["t_us"][e - 1]))
            moverows.append(m)

        # ---- static phases ----
        u = dict(scode=sc)
        for ph, tag in ((1, "pre"), (9, "post")):
            k = d["phase"] == ph
            p = d["pos"][k]; c = np.abs(d["current"][k]); l = np.abs(d["load"][k])
            u[f"static_{tag}_n"] = int(k.sum())
            u[f"static_{tag}_p2p"] = int(p.max() - p.min())
            u[f"static_{tag}_std"] = round(float(p.std()), 4)
            u[f"static_{tag}_drift"] = int(p[-1] - p[0])
            u[f"static_{tag}_cur_max"] = int(c.max())
            u[f"static_{tag}_load_max"] = int(l.max())
            u[f"static_{tag}_volt_med"] = float(np.median(d["volt"][k]))
            u[f"static_{tag}_temp_med"] = float(np.median(d["temp"][k]))

        # ---- telemetry artifacts ----
        tr, tdev = artifact_runs(d["temp"].astype(np.float64), 25, 5)
        vr, vdev = artifact_runs(d["volt"].astype(np.float64), 25, 8)
        for name, runs in (("temp", tr), ("volt", vr)):
            u[f"{name}_spike_single"] = int((runs == 1).sum())
            u[f"{name}_run_2_24"] = int(((runs >= 2) & (runs < 25)).sum())
            u[f"{name}_run_ge25"] = int((runs >= 25).sum())
            u[f"{name}_run_max"] = int(runs.max()) if runs.size else 0
        u["temp_clean_max"] = int(d["temp"][~tdev].max())
        u["temp_clean_min"] = int(d["temp"][~tdev].min())
        u["temp_rise"] = u["static_post_temp_med"] - u["static_pre_temp_med"]
        u["volt_clean_min"] = int(d["volt"][~vdev].min())
        u["volt_clean_max"] = int(d["volt"][~vdev].max())
        u["volt_med_all"] = float(np.median(d["volt"]))

        # load saturation WITH low motion = the only stall-like signature
        vall = velocity(d["pos"], d["t_us"], 3)
        sat = np.abs(d["load"]) >= 1000
        u["load_sat_n"] = int(sat.sum())
        u["load_sat_lowspeed_n"] = int((sat & np.isfinite(vall) & (np.abs(vall) < 100)).sum())
        u["cur_max_all"] = int(np.abs(d["current"]).max())
        u["pos_reach_min"] = int(d["pos"].min())
        u["pos_reach_max"] = int(d["pos"].max())
        unitrows.append(u)
        print(f"{sc} moves={len(meas)} ok", flush=True)

    def dump(rows, name):
        keys = sorted({k for r in rows for k in r})
        keys = ["scode"] + [k for k in keys if k != "scode"]
        with open(q.AUDIT / name, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader(); w.writerows(rows)

    dump(moverows, "blind_move_metrics.csv")
    dump(unitrows, "blind_unit_telemetry.csv")
    print(f"\nwrote blind_move_metrics.csv ({len(moverows)} moves), blind_unit_telemetry.csv")


if __name__ == "__main__":
    main()
