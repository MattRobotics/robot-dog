#!/usr/bin/env python3
"""Locomotion vNext (2026-11-13): one twin walk of a build per scenario, m6_gait's metrics plus the effective
foot clearance of every full swing, from the same simulation.

m6_gait.run gives the travel (achieved speed against the law), transmitted support (weight on law-swing feet,
samples without any loaded foot), slip, joint error/speed/duty, STOP. Added here, per full swing of the
measured window (swings cut by the window edges are left out):
  gap   : signed distance cylinder-floor of the swing foot (mj_geomDistance on the recorded state, base and
          joints; < 0 below the plane), i.e. the effective geometric clearance, never the commanded law;
  core  : the minimum gap over the central half of the swing (u .25-.75);
  share : the share of the swing with gap > 0.3 mm (out of contact) and > 1 mm;
  edges : the swing fraction spent at the floor (gap <= 0.3 mm) before the first and after the last free sample;
  load  : the share of the swing with the foot's normal force > CONTACT_N (contact with force).
Advance budget (mu_rca's read-only probe): COM advance per cycle by gait phase (which leg is in the law's swing,
or four stance), along the instantaneous heading, and the slip of the loaded stance feet (velocity of the tread's
material point at the contact reference, i.e. sliding, not rolling) along the heading, per phase.
Counterfactual (diagnostic only): --cf swing_contact=0 (law-swing feet do not collide; mu_rca.install).

Usage: vnext_eval.py <cosim> <out dir> <label> <scenario ...> [--variant dev7_ground] [--set k=v ...]
       [--fast-law period:stride_mm:height_mm] [--cf name=value] [--npz]  (--npz: also the raw arrays, for diagnosis)
"""
import json
import pathlib
import sys

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve()
sys.path.insert(0, str(HERE.parents[2]))  # Matdog_MuJoCo
sys.path.insert(0, str(HERE.parents[1] / "fast40"))

import dynamics  # noqa: E402
import m6_dev7  # noqa: E402
import m6_gait  # noqa: E402
import twin  # noqa: E402

FREE_M = 3e-4

# vNext CRAWL prototype (the SLOW slot of its build): 8 s cycle, top stride 80 mm, i.e. 10 mm/s
CRAWL_PERIOD, CRAWL_TOP = 8.0, .080
held = m6_gait.held
m6_gait.SCENARIOS["crawl_fwd"] = ("SLOW", .120, held(1.0, 49.0, f"DRIVE {CRAWL_TOP / CRAWL_PERIOD:.17g} 0 0") + [(49.2, "WALKSTOP")], 66.0, (17.0, 49.0))
m6_gait.SCENARIOS["crawl_stop"] = ("SLOW", .120, [(1.0, "STOPFRONT 12 0.85")] + held(1.0, 40.0, f"DRIVE {CRAWL_TOP / CRAWL_PERIOD:.17g} 0 0"), 30.0, None)
for k in range(8):
    t = 25.0 + k * CRAWL_PERIOD / 8
    m6_gait.SCENARIOS[f"crawl_stopcyc_{k}"] = ("SLOW", .120, held(1.0, t, f"DRIVE {CRAWL_TOP / CRAWL_PERIOD:.17g} 0 0") + [(t, "STOP")], t + 6.0, None)
m6_gait.SCENARIOS["crawl_left"] = ("SLOW", .120, held(1.0, 49.0, "DRIVE 0 0.0025 0") + [(49.2, "WALKSTOP")], 58.0, (17.0, 49.0))
m6_gait.SCENARIOS["crawl_yaw"] = ("SLOW", .120, held(1.0, 49.0, "DRIVE 0 0 0.02") + [(49.2, "WALKSTOP")], 58.0, (17.0, 49.0))


def clearance(rows, tw, window):
    rec = tw.result()
    t = rec["t"]
    n = len(rows)
    idx = np.clip(np.ceil(t / twin.RELEASE_S - 1e-9).astype(int) - 1, 0, n - 1)
    stance = np.array([[int(r[f"stance{l}"]) for l in range(4)] for r in rows])[idx]
    law_z = np.array([[float(r[f"foot{l}_z"]) for l in range(4)] for r in rows])[idx]
    a, b = window
    m = (t >= a) & (t <= b) & (stance >= 0).all(axis=1)
    wi = np.flatnonzero(m)
    dk = mujoco.MjData(tw.m)
    out = []
    for l in range(4):
        sw = (stance[:, l] == 0) & m
        e = np.flatnonzero(np.diff(np.r_[0, sw.astype(int), 0]))
        for i0, i1 in zip(e[::2], e[1::2]):
            if i1 - i0 < 50 or i0 <= wi[0] or i1 > wi[-1]:
                continue
            gap = np.empty(i1 - i0)
            for k, i in enumerate(range(i0, i1)):
                dk.qpos[0:3], dk.qpos[3:7], dk.qpos[7:19] = rec["pos"][i], rec["quat"][i], rec["q"][i]
                mujoco.mj_kinematics(tw.m, dk)
                gap[k] = mujoco.mj_geomDistance(tw.m, dk, tw.floor, tw.feet[l], 0.05, None)
            u = np.linspace(0, 1, len(gap))
            core = (u >= .25) & (u <= .75)
            free = np.flatnonzero(gap > FREE_M)
            out.append({"leg": "LF RF RH LH".split()[l], "t0": float(t[i0]), "duration_s": float(t[i1 - 1] - t[i0]),
                        "gap_at_law_peak_mm": 1e3 * float(gap[int(np.argmax(law_z[i0:i1, l]))]),
                        "gap_core_min_mm": 1e3 * float(gap[core].min()), "gap_max_mm": 1e3 * float(gap.max()),
                        "free_share": float(np.mean(gap > FREE_M)), "above_1mm_share": float(np.mean(gap > 1e-3)),
                        "floor_before_first_free": float(u[free[0]]) if len(free) else 1.0,
                        "floor_after_last_free": float(1 - u[free[-1]]) if len(free) else 1.0,
                        "loaded_share": float(np.mean(rec["fz"][i0:i1, l] > dynamics.CONTACT_N))})
    agg = {}
    for grp, legs in (("front", ("LF", "RF")), ("hind", ("RH", "LH"))):
        S = [s for s in out if s["leg"] in legs]
        if S:
            agg[grp] = {k: float(np.median([s[k] for s in S])) for k in S[0] if k not in ("leg", "t0")}
            agg[grp]["n"] = len(S)
            agg[grp]["gap_core_min_mm_worst"] = float(min(s["gap_core_min_mm"] for s in S))
    return {"swings": out, "by_group": agg}


def budget(rows, tw, window, probe):
    rec = tw.result()
    t = rec["t"]
    n = len(rows)
    idx = np.clip(np.ceil(t / twin.RELEASE_S - 1e-9).astype(int) - 1, 0, n - 1)
    st = np.array([[int(r[f"stance{l}"]) for l in range(4)] for r in rows])[idx]
    wc = np.array([float(r["walk_cycles"]) for r in rows])[idx]
    a, b = window
    m = (t >= a) & (t <= b) & (st >= 0).all(axis=1)
    P = probe.arrays()
    k = min(len(t), len(P["probe_vcom"]))
    m, st, wc = m[:k], st[:k], wc[:k]
    yaw = np.array([np.arctan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] ** 2 + q[3] ** 2)) for q in rec["quat"][:k]])
    h = np.c_[np.cos(yaw), np.sin(yaw)]
    dt = float(np.median(np.diff(t)))
    cyc = wc[m][-1] - wc[m][0]
    vcom = (P["probe_vcom"][:k, :2] * h).sum(1)
    vslip = (P["probe_vref"][:k, :, :2] * h[:, None, :]).sum(2)
    loaded = P["probe_F"][:k, :, 2] > dynamics.CONTACT_N
    nsw = (st == 0).sum(1)
    lab = np.where(nsw == 0, 4, np.argmax(st == 0, 1))
    lab[nsw > 1] = 5
    names = ("LF", "RF", "RH", "LH", "4-stance", "multi")
    on, oF = P["probe_other_n"][:k], P["probe_other_F"][:k]
    hit = m & (on > 0) & (oF[:, 2] > dynamics.CONTACT_N)  # a non-foot part of the robot loaded on the floor
    out = {"cycles": float(cyc), "com_mm_per_cycle": 1e3 * float(vcom[m].sum()) * dt / cyc, "by_phase": {},
           "non_foot_floor_contact_share": float(hit.sum() / max(1, m.sum())), "non_foot_floor_fz_max_n": float(oF[m, 2].max()) if m.any() else 0.0}
    for j, nm in enumerate(names):
        sel = m & (lab == j)
        if not sel.any():
            continue
        L = loaded & (st == 1) & sel[:, None]
        out["by_phase"][nm] = {"com_mm_per_cycle": 1e3 * float(vcom[sel].sum()) * dt / cyc,
                               "stance_slip_mm_s_mean": 1e3 * float(vslip[L].mean()) if L.any() else None,
                               "stance_slip_mm_per_cycle_sum": 1e3 * float(vslip[L].sum()) * dt / cyc}
    return out


def main():
    argv = sys.argv[1:]
    sets = [argv[i + 1] for i, x in enumerate(argv) if x == "--set"]
    cfs = dict(argv[i + 1].split("=") for i, x in enumerate(argv) if x == "--cf")
    variant = next((argv[i + 1] for i, x in enumerate(argv) if x == "--variant"), "dev7_ground")
    if "--fast-law" in argv:
        per, stride_mm, height_mm = (float(x) for x in argv[argv.index("--fast-law") + 1].split(":"))
        m6_gait.FAST_LAW.update(period=per, stride=stride_mm * 1e-3, height=height_mm * 1e-3)
        m6_gait.SCENARIOS.update(m6_gait.fast_scenarios(m6_gait.FAST_LAW))
    pos = [x for i, x in enumerate(argv) if not x.startswith("--") and (i == 0 or not argv[i - 1].startswith("--"))]
    binary, out_dir, label, names = pathlib.Path(pos[0]), pathlib.Path(pos[1]), pos[2], pos[3:]
    import mu_rca
    mu_rca._sim_modules()
    probe = mu_rca.Probe()
    mu_rca.install(probe, cfs)
    _, v = m6_dev7.variant_params(variant, sets)
    out_dir.mkdir(parents=True, exist_ok=True)
    for name in names:
        probe.__init__()
        rep, rows, tw = m6_gait.run(binary, out_dir, label, name, v, write=False, keep=True)
        rep["cf"] = cfs
        window = m6_gait.SCENARIOS[name][4]
        if window and "travel" in rep:  # no measured window when the walk faulted or never started
            rep["clearance"] = clearance(rows, tw, window)
            rep["budget"] = budget(rows, tw, window, probe)
            bd = rep["budget"]
            print(f"  {name} {label} non-foot floor contact {bd['non_foot_floor_contact_share']:.4f} (max {bd['non_foot_floor_fz_max_n']:.2f} N) | budget COM {bd['com_mm_per_cycle']:6.1f} mm/cycle: " + " ".join(
                f"{k} {x['com_mm_per_cycle']:6.1f} (slip {x['stance_slip_mm_per_cycle_sum']:6.1f})" for k, x in bd["by_phase"].items()), flush=True)
            g = rep["clearance"]["by_group"]
            print(f"  {name} {label} clearance " + " | ".join(
                f"{k}: peak {x['gap_at_law_peak_mm']:5.2f} core-min {x['gap_core_min_mm']:5.2f} (worst {x['gap_core_min_mm_worst']:5.2f}) "
                f"free {x['free_share']:.2f} >1mm {x['above_1mm_share']:.2f} floor@start {x['floor_before_first_free']:.2f} @end {x['floor_after_last_free']:.2f} "
                f"loaded {x['loaded_share']:.2f} n {x['n']}" for k, x in g.items()), flush=True)
        if "--npz" in argv:
            rec = tw.result()
            keys = ("t_s", "walk_cycles", "body_x", "body_y", "body_z", "body_yaw", "imu_roll", "imu_pitch")
            rowarr = {f"row_{k}": np.array([float(r[k]) for r in rows]) for k in keys}
            rowarr["row_stance"] = np.array([[int(r[f"stance{l}"]) for l in range(4)] for r in rows])
            rowarr["row_raw"] = np.array([[int(r[f"raw{j}"]) for j in range(12)] for r in rows])
            rowarr["row_foot"] = np.array([[[float(r[f"foot{l}_{a}"]) for a in "xyz"] for l in range(4)] for r in rows])
            np.savez_compressed(out_dir / f"{name}__{label}.npz", **{f"rec_{k}": np.asarray(x) for k, x in rec.items()}, **probe.arrays(), **rowarr)
        (out_dir / f"{name}__{label}.json").write_text(json.dumps(rep, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")


if __name__ == "__main__":
    main()
