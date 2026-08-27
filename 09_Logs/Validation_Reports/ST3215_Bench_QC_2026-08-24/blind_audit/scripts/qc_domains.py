"""FASE C2/C5/C6/C7 + D — per-unit domain metrics and robust population z-scores."""
import csv, sys
from collections import defaultdict
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
import qc_common as q

NUM = ["move_idx", "phase", "goal_spd", "goal_acc", "start_pos", "target",
       "final_pos", "direction", "residual", "abs_residual", "elapsed_ms",
       "first_motion_ms", "total_travel", "cruise_v_med", "cruise_v_cv",
       "cruise_n", "cruise_ms", "plateau_v", "accel_ms", "decel_ms",
       "stick_slip", "dwell_max_ms", "backtrack_n", "backtrack_max",
       "cruise_span", "cruise_v_lin",
       "overshoot", "settle_ms", "load_med", "load_p95", "load_max",
       "cur_med", "cur_p95", "cur_max", "volt_min", "volt_med", "n",
       "speedreg_ratio", "idx_start", "idx_end", "t_start_us", "t_end_us"]


def load(name, num):
    rows = list(csv.DictReader(open(q.AUDIT / name)))
    for r in rows:
        for k in num:
            if k in r:
                try:
                    r[k] = float(r[k])
                except ValueError:
                    r[k] = np.nan
    return rows


def nmed(v):
    v = np.asarray([x for x in v if np.isfinite(x)], dtype=float)
    return float(np.median(v)) if v.size else np.nan


def main():
    mrows = load("blind_move_metrics.csv", NUM)
    tele = {r["scode"]: r for r in load("blind_unit_telemetry.csv", [])}
    anom = load("blind_anomalies.csv", ["n_bins", "n_independent_passes"])

    by = defaultdict(list)
    for r in mrows:
        by[r["scode"]].append(r)

    minrows, precrows, units = [], [], []
    for sc in q.scodes():
        ms = sorted(by[sc], key=lambda r: r["move_idx"])
        u = dict(scode=sc)

        def sel(phases, direction=None):
            return [m for m in ms if int(m["phase"]) in phases
                    and (direction is None or int(m["direction"]) == direction)]

        # ---------- SLOW / MEDIUM ----------
        for tag, ph in (("slow", (3, 4)), ("med", (11,)), ("xfer", (10,))):
            g = sel(ph)
            u[f"{tag}_v"] = nmed([m["cruise_v_lin"] for m in g])
            u[f"{tag}_v_up"] = nmed([m["cruise_v_med"] for m in g if m["direction"] > 0])
            u[f"{tag}_v_dn"] = nmed([m["cruise_v_med"] for m in g if m["direction"] < 0])
            u[f"{tag}_v_cv"] = nmed([m["cruise_v_cv"] for m in g])
            u[f"{tag}_resid"] = nmed([m["abs_residual"] for m in g])
            u[f"{tag}_resid_max"] = float(np.nanmax([m["abs_residual"] for m in g])) if g else np.nan
            u[f"{tag}_offtarget_n"] = int(sum(1 for m in g if m["abs_residual"] > 3))
            u[f"{tag}_first_motion"] = nmed([m["first_motion_ms"] for m in g])
            u[f"{tag}_backtrack"] = int(sum(int(m["backtrack_n"]) for m in g))
            u[f"{tag}_stickslip"] = nmed([m["stick_slip"] for m in g])
            u[f"{tag}_dwell_max"] = float(np.nanmax([m["dwell_max_ms"] for m in g])) if g else np.nan
            u[f"{tag}_load"] = nmed([m["load_med"] for m in g])
            u[f"{tag}_cur"] = nmed([m["cur_med"] for m in g])
            u[f"{tag}_settle"] = nmed([m["settle_ms"] for m in g])
        u["updown_asym_slow"] = abs(u["slow_v_up"] - u["slow_v_dn"])
        u["updown_asym_med"] = abs(u["med_v_up"] - u["med_v_dn"])

        # ---------- MAX ----------
        dn = sel((6,)); up = sel((7,))
        u["max_tps_dn"] = nmed([m["cruise_v_lin"] for m in dn])
        u["max_tps_up"] = nmed([m["cruise_v_lin"] for m in up])
        u["max_tps"] = nmed([m["cruise_v_lin"] for m in dn + up])
        u["max_asym_pct"] = (100 * abs(u["max_tps_up"] - u["max_tps_dn"]) /
                             np.nanmean([u["max_tps_up"], u["max_tps_dn"]]))
        u["max_rpm"] = u["max_tps"] / 4096 * 60
        u["max_s_per_60deg"] = 682.667 / u["max_tps"]
        u["max_accel_ms"] = nmed([m["accel_ms"] for m in dn + up])
        u["max_decel_ms"] = nmed([m["decel_ms"] for m in dn + up])
        u["max_overshoot"] = nmed([m["overshoot"] for m in dn + up])
        u["max_load"] = nmed([m["load_med"] for m in dn + up])
        u["max_cur"] = nmed([m["cur_med"] for m in dn + up])
        u["max_cur_peak"] = float(np.nanmax([m["cur_max"] for m in dn + up])) if dn + up else np.nan
        u["max_volt_min"] = float(np.nanmin([m["volt_min"] for m in dn + up])) if dn + up else np.nan
        u["max_volt_med"] = nmed([m["volt_med"] for m in dn + up])
        u["max_tps_per_volt"] = u["max_tps"] / (u["max_volt_med"] / 10.0)
        u["max_resid"] = nmed([m["abs_residual"] for m in dn + up])

        # ---------- PRECISION / HYSTERESIS (phase 8) ----------
        prec = sel((8,))
        zones = defaultdict(dict)
        for m in prec:
            zones[int(m["target"])]["above" if m["direction"] < 0 else "below"] = m
        errs, hyst = [], []
        for z, pair in sorted(zones.items()):
            ea = pair["above"]["residual"] if "above" in pair else np.nan
            eb = pair["below"]["residual"] if "below" in pair else np.nan
            h = ea - eb if np.isfinite(ea) and np.isfinite(eb) else np.nan
            if np.isfinite(ea): errs.append(ea)
            if np.isfinite(eb): errs.append(eb)
            if np.isfinite(h): hyst.append(h)
            precrows.append(dict(scode=sc, zone=z, err_from_above=ea, err_from_below=eb,
                                 hysteresis=h,
                                 overshoot_above=pair.get("above", {}).get("overshoot", np.nan),
                                 overshoot_below=pair.get("below", {}).get("overshoot", np.nan),
                                 settle_above=pair.get("above", {}).get("settle_ms", np.nan),
                                 settle_below=pair.get("below", {}).get("settle_ms", np.nan)))
        errs = np.array(errs, dtype=float)
        u["prec_n_zones"] = len(zones)
        u["prec_mae"] = float(np.mean(np.abs(errs))) if errs.size else np.nan
        u["prec_max_abs_err"] = float(np.max(np.abs(errs))) if errs.size else np.nan
        u["prec_bias"] = float(np.mean(errs)) if errs.size else np.nan
        u["prec_repeat_sd"] = float(np.std(errs)) if errs.size else np.nan
        u["hyst_med"] = nmed(hyst)
        u["hyst_max"] = float(np.max(np.abs(hyst))) if hyst else np.nan
        u["prec_settle"] = nmed([m["settle_ms"] for m in prec])
        u["prec_overshoot"] = nmed([m["overshoot"] for m in prec])

        # ---------- MIN-SPEED PROBES (phase 5) ----------
        mp = sel((5,))
        zone_ctx = None
        for m in ms:
            if int(m["phase"]) == 10 and int(m["goal_spd"]) == 600 and int(m["target"]) % 512 == 256:
                zone_ctx = int(m["target"])
            if int(m["phase"]) != 5:
                continue
            dirn = 1 if m["target"] > m["start_pos"] else -1
            travel = (m["total_travel"] if dirn > 0 else m["total_travel"])
            eff = abs(m["final_pos"] - m["start_pos"])
            minrows.append(dict(
                scode=sc, zone=zone_ctx, direction=dirn, cmd_speed=int(m["goal_spd"]),
                start=int(m["start_pos"]), goal=int(m["target"]),
                final=int(m["final_pos"]), directional_travel=eff,
                travel_span=m["total_travel"],
                travel_per_s=eff / (m["elapsed_ms"] / 1000.0),
                first_motion_ms=m["first_motion_ms"], v_med=m["cruise_v_med"],
                backtrack_n=int(m["backtrack_n"]), backtrack_max=m["backtrack_max"],
                dwell_max_ms=m["dwell_max_ms"], stick_slip=m["stick_slip"],
                load_med=m["load_med"], cur_med=m["cur_med"],
                idx_start=int(m["idx_start"]), t_start_us=int(m["t_start_us"])))
        tv = [abs(m["final_pos"] - m["start_pos"]) for m in mp]
        tp = [abs(m["final_pos"] - m["start_pos"]) for m in mp if m["target"] > m["start_pos"]]
        tn = [abs(m["final_pos"] - m["start_pos"]) for m in mp if m["target"] < m["start_pos"]]
        u["min_n"] = len(mp)
        u["min_travel_med"] = nmed(tv)
        u["min_travel_min"] = float(np.min(tv)) if tv else np.nan
        u["min_travel_up"] = nmed(tp)
        u["min_travel_dn"] = nmed(tn)
        u["min_dir_asym"] = abs(u["min_travel_up"] - u["min_travel_dn"])
        u["min_zero_response"] = int(sum(1 for t in tv if t < 1))
        u["min_first_motion"] = nmed([m["first_motion_ms"] for m in mp])
        u["min_backtrack"] = int(sum(int(m["backtrack_n"]) for m in mp))
        u["min_dwell_max"] = float(np.nanmax([m["dwell_max_ms"] for m in mp])) if mp else np.nan
        u["min_v"] = nmed([m["cruise_v_med"] for m in mp])

        # ---------- static + telemetry ----------
        for k, v in tele[sc].items():
            if k != "scode":
                try:
                    u[k] = float(v)
                except ValueError:
                    u[k] = v

        # ---------- hotspot burden ----------
        a = [x for x in anom if x["scode"] == sc]
        u["hs_high"] = sum(1 for x in a if x["confidence"] == "HIGH")
        u["hs_med"] = sum(1 for x in a if x["confidence"] == "MEDIUM")
        u["hs_low"] = sum(1 for x in a if x["confidence"] == "LOW")
        u["hs_rejected"] = sum(1 for x in a if x["confidence"] == "REJECTED_SINGLE_BIN")
        u["hs_crossspeed"] = sum(1 for x in a if x["cross_speed"] == "True"
                                 and x["confidence"] != "REJECTED_SINGLE_BIN")
        units.append(u)

    # ---------- robust population z ----------
    zkeys = [k for k in units[0] if k != "scode" and isinstance(units[0][k], float)]
    for k in zkeys:
        vals = np.array([u[k] for u in units], dtype=float)
        if not np.isfinite(vals).any():
            continue
        med = np.nanmedian(vals)
        mad = np.nanmedian(np.abs(vals - med))
        floor = max(mad, 1e-9)
        for u, v in zip(units, vals):
            u[f"z_{k}"] = round(float(0.6745 * (v - med) / floor), 3) if mad > 0 else 0.0

    def dump(rows, name, first=("scode",)):
        keys = list(first) + sorted({k for r in rows for k in r} - set(first))
        with open(q.AUDIT / name, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader(); w.writerows(rows)

    dump(units, "blind_metrics.csv")
    dump(minrows, "blind_min_probes.csv")
    dump(precrows, "blind_precision.csv")
    print(f"wrote blind_metrics.csv ({len(units)} units, {len(units[0])} columns), "
          f"blind_min_probes.csv ({len(minrows)}), blind_precision.csv ({len(precrows)})")


if __name__ == "__main__":
    main()
