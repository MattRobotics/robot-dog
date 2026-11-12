#!/usr/bin/env python3
"""M6 gait study: one Motion build (dev.7 or a candidate patch, gait_study/build_variant.sh)
walked in closed loop by the corrected twin over a fixed scenario set, with comparative metrics.

Metrics come from the twin's own physics (never from the commanded law):
- travel: base displacement per gait cycle (x, y in the heading frame at the start) and yaw per cycle,
  against the law's own displacement per cycle;
- lateral body excursion: peak-to-peak of the base's lateral position about its linear trend;
- feet: real swing clearance of the contact reference (p10/p50/p90) and the share of the swing with
  a normal force > DRAG_N; slip of loaded feet; support margin of the loaded feet (p05, min);
- joints: p95 |error| (measured - commanded, ticks), p95 |speed| (step/s), p95 |duty| (per mille);
- STOP: verified or fault, and the time to verification.

Usage: m6_gait.py <cosim> <out dir> <label> [scenario ...] [--variant dev7_ground] [--set key=value ...]
       [--fast-law period_s:stride_mm:height_mm] (the FAST law under test for the fast_* scenarios; default dev.7 2.5:80:105)
"""
import dataclasses
import json
import math
import pathlib
import sys
import time

import numpy as np

import cosim
import dynamics
import m6_dev7
import servo
import twin

JOY = 0.2  # joystick renewal (s)


def held(t0, t1, cmd):
    """A held stick: the command at t0 and its renewals until t1."""
    return [(t0, cmd)] + [(("renew", JOY), cmd)] * int((t1 - t0) / JOY)


# name: (mode, height m, events, duration s, measured window [from, to] s)
def scenarios():
    s = {}
    for mode, h, vx in (("SLOW", .120, .020), ("STANDARD", .110, .030), ("FAST", .105, .032)):
        m = mode.lower()
        s[f"fwd_{m}"] = (mode, h, held(1.0, 31.0, f"DRIVE {vx} 0 0") + [(31.2, "WALKSTOP")], 40.0, (11.0, 31.0))
        # the trigger first: a command due after the renewals would drop them (cosim.simulate)
        s[f"fwd_{m}_stop"] = (mode, h, [(1.0, "STOPFRONT 12 0.85")] + held(1.0, 30.0, f"DRIVE {vx} 0 0"), 22.0, None)
        # STOP statistics: 8 STOPs spread over one gait cycle of the steady forward walk
        period = {"SLOW": 5.0, "STANDARD": 3.0, "FAST": 2.5}[mode]
        for k in range(8):
            t = 14.0 + k * period / 8
            s[f"stopcyc_{m}_{k}"] = (mode, h, held(1.0, t, f"DRIVE {vx} 0 0") + [(t, "STOP")], t + 5.0, None)
    # mode changes inside a forward walk (law shape changes in four-contact windows)
    # (each SELECT between two held segments: a timed command due after pending renewals drops them)
    s["modes_cycle"] = ("SLOW", .120, held(1.0, 12.0, "DRIVE 0.020 0 0") + [(12.0, "SELECT STANDARD")] + held(12.0, 24.0, "DRIVE 0.030 0 0")
                        + [(24.0, "SELECT FAST")] + held(24.0, 34.0, "DRIVE 0.032 0 0") + [(34.0, "SELECT SLOW")] + held(34.0, 47.0, "DRIVE 0.020 0 0")
                        + [(47.2, "WALKSTOP")], 56.0, (2.0, 47.0))
    s["back_standard"] = ("STANDARD", .110, held(1.0, 31.0, "DRIVE -0.01666 0 0") + [(31.2, "WALKSTOP")], 40.0, (11.0, 31.0))
    s["back_fast_stop"] = ("FAST", .105, held(1.0, 13.0, "DRIVE -0.02 0 0") + [(13.0, "STOP")], 18.0, None)
    s["left_standard"] = ("STANDARD", .110, held(1.0, 31.0, "DRIVE 0 0.00666 0") + [(31.2, "WALKSTOP")], 40.0, (13.0, 31.0))
    s["right_fast"] = ("FAST", .105, held(1.0, 31.0, "DRIVE 0 -0.0048 0") + [(31.2, "WALKSTOP")], 40.0, (13.0, 31.0))
    s["left_slow"] = ("SLOW", .120, held(1.0, 41.0, "DRIVE 0 0.004 0") + [(41.2, "WALKSTOP")], 50.0, (16.0, 41.0))
    s["yaw_standard"] = ("STANDARD", .110, held(1.0, 31.0, "DRIVE 0 0 0.04") + [(31.2, "WALKSTOP")], 40.0, (13.0, 31.0))
    s["yaw_standard_stop"] = ("STANDARD", .110, held(1.0, 16.0, "DRIVE 0 0 -0.04") + [(16.0, "STOP")], 21.0, None)
    s["diag_standard"] = ("STANDARD", .110, held(1.0, 31.0, "DRIVE 0.0137 -0.0027 0") + [(31.2, "WALKSTOP")], 40.0, (13.0, 31.0))
    # continuous direction changes: forward -> diagonal -> lateral -> diagonal back -> back (the HW lever sweep)
    sweep = []
    for k, (vx, vy) in enumerate(((.03, 0), (.0137, -.0027), (.0105, .0048), (0, .0048), (-.0119, -.0026), (-.0167, 0), (0, -.0067))):
        sweep += held(1.0 + 6.0 * k, 7.0 + 6.0 * k, f"DRIVE {vx} {vy} 0")
    s["sweep_standard"] = ("STANDARD", .110, sweep + [(43.2, "WALKSTOP")], 50.0, (1.0, 43.0))
    return s


# FAST study (2026-11-12): the FAST law under test (dev.7 by default; --fast-law period:stride_mm:height_mm)
# and its JoystickSpec per cycle (dev.7 FAST: lateral 12 mm, yaw .08 rad, backward 50 mm).
FAST_LAW = {"period": 2.5, "stride": .080, "height": .105}
FAST_JOY = {"lateral": .012, "yaw": .08, "back": .050}


def fast_scenarios(law, joy=FAST_JOY):
    """Every direction of the FAST law at its own published ranges (cosim DRIVE in m/s, rad/s)."""
    P, h = law["period"], law["height"]
    top, lat, yaw, back = law["stride"] / P, joy["lateral"] / P, joy["yaw"] / P, joy["back"] / P
    # full precision: the published range is the same double quotient (a 6-digit rounding can exceed it)
    d = lambda fx, fy=0.0, fw=0.0: f"DRIVE {fx * (top if fx >= 0 else back):.17g} {fy * lat:.17g} {fw * yaw:.17g}"
    walk = lambda cmd, t1=31.0, w=(11.0, 31.0): ("FAST", h, held(1.0, t1, cmd) + [(t1 + .2, "WALKSTOP")], t1 + 9.0, w)
    s = {"fast_fwd": walk(d(1)), "fast_fwd_32": walk(f"DRIVE {min(.032, top):.6f} 0 0"),
         "fast_fwd_stop": ("FAST", h, [(1.0, "STOPFRONT 12 0.85")] + held(1.0, 30.0, d(1)), 22.0, None),
         "fast_back": walk(d(-1)), "fast_back_stop": ("FAST", h, held(1.0, 13.0, d(-1)) + [(13.0, "STOP")], 18.0, None),
         "fast_left": walk(d(0, 1), w=(13.0, 31.0)), "fast_right": walk(d(0, -1), w=(13.0, 31.0)),
         "fast_yaw": walk(d(0, 0, 1), w=(13.0, 31.0)),
         "fast_yaw_stop": ("FAST", h, held(1.0, 16.0, d(0, 0, -1)) + [(16.0, "STOP")], 21.0, None),
         "fast_diag": walk(d(.5, .5), w=(13.0, 31.0)), "fast_diag_yaw": walk(d(.5, 0, .5), w=(13.0, 31.0))}
    for k in range(8):
        t = 14.0 + k * P / 8
        s[f"fast_stopcyc_{k}"] = ("FAST", h, held(1.0, t, d(1)) + [(t, "STOP")], t + 5.0, None)
    # dense STOP sweep: 16 instants over one cycle, half a period later than stopcyc (other releases)
    for k in range(16):
        t = 14.0 + (k + .5) * P / 16
        s[f"fast_stopdense_{k:02d}"] = ("FAST", h, held(1.0, t, d(1)) + [(t, "STOP")], t + 5.0, None)
    s["fast_diag_stop"] = ("FAST", h, held(1.0, 16.0, d(.5, .5)) + [(16.0, "STOP")], 21.0, None)
    # continuous direction changes at the FAST ranges (the fractions of sweep_standard)
    sweep = []
    for k, (fx, fy) in enumerate(((1, 0), (.457, -.405), (.35, .72), (0, .72), (-.714, -.39), (-1, 0), (0, -1))):
        sweep += held(1.0 + 6.0 * k, 7.0 + 6.0 * k, d(fx, fy))
    s["fast_sweep"] = ("FAST", h, sweep + [(43.2, "WALKSTOP")], 50.0, (1.0, 43.0))
    s["fast_modes_cycle"] = ("SLOW", .120, held(1.0, 12.0, "DRIVE 0.020 0 0") + [(12.0, "SELECT STANDARD")] + held(12.0, 24.0, "DRIVE 0.030 0 0")
                             + [(24.0, "SELECT FAST")] + held(24.0, 36.0, d(1)) + [(36.0, "SELECT SLOW")] + held(36.0, 49.0, "DRIVE 0.020 0 0")
                             + [(49.2, "WALKSTOP")], 58.0, (2.0, 49.0))
    return s


SCENARIOS = scenarios()
SCENARIOS.update(fast_scenarios(FAST_LAW))


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def run(binary, out_dir, label, name, v, write=True):
    mode, height, events, duration, window = SCENARIOS[name]
    ev = ([(0.5, f"SELECT {mode}")] if mode != "SLOW" else []) + events
    params = dataclasses.replace(servo.Params(), **v.get("params", {}))
    imu = tuple(1e-3 * x for x in v["imu_offset_mrad"]) if "imu_offset_mrad" in v else None
    started = time.time()
    rows, tw, replies, columns = cosim.simulate(binary, ev, duration, params, v.get("mu", 0.8), v.get("unit"), height, None, imu, v.get("model"))
    rec = tw.result()
    t, n = rec["t"], len(rows)
    stance = np.array([[int(r[f"stance{l}"]) for l in range(4)] for r in rows])
    idx = np.clip(np.ceil(t / twin.RELEASE_S - 1e-9).astype(int) - 1, 0, n - 1)
    st = stance[idx]
    walking = (st >= 0).all(axis=1)
    rep = {"scenario": name, "label": label, "mode": mode, "params": dataclasses.asdict(params), "definition": v, "fast_law": dict(FAST_LAW),
           "refusals": [r for r in replies if r[2] != "OK -"], "wall_s": round(time.time() - started, 1),
           "faults": [(r["t_s"], r["reason"]) for r in rows if r["state"] == "FAULT"][:1],
           "imu_reasons": {k: sum(r["imu_reason"] == k for r in rows) for k in {r["imu_reason"] for r in rows}},
           "states": sorted({(r["state"], r["reason"]) for r in rows})}
    if window:
        a, b = window
        m = (t >= a) & (t <= b) & walking
        if m.sum() > 100:
            wc = np.array([float(r["walk_cycles"]) for r in rows])[idx]
            cyc = wc[m][-1] - wc[m][0]
            yaw0 = yaw_of(rec["quat"][m][0])
            c, s_ = math.cos(yaw0), math.sin(yaw0)
            d = rec["pos"][m][-1, :2] - rec["pos"][m][0, :2]
            dxy = np.array([c * d[0] + s_ * d[1], -s_ * d[0] + c * d[1]])
            yaw = np.unwrap([yaw_of(q) for q in rec["quat"][m]])
            law = np.array([[float(r["body_x"]), float(r["body_y"])] for r in rows])[idx][m]
            law_yaw = np.unwrap(np.array([float(r["body_yaw"]) for r in rows])[idx][m])
            ld = law[-1] - law[0]
            # lateral excursion: base y in the start heading frame about its linear trend (and the law's)
            p = rec["pos"][m][:, :2] - rec["pos"][m][0, :2]
            lat = -s_ * p[:, 0] + c * p[:, 1]
            tt = t[m]
            res = lat - np.polyval(np.polyfit(tt, lat, 1), tt)
            lres = (law[:, 1] - law[0, 1]) - np.polyval(np.polyfit(tt, law[:, 1] - law[0, 1], 1), tt)
            rep["travel"] = {"cycles": float(cyc), "twin_mm_per_cycle": (1e3 * dxy / cyc).tolist(), "law_mm_per_cycle": (1e3 * ld / cyc).tolist(),
                             "twin_yaw_deg_per_cycle": float(np.degrees(yaw[-1] - yaw[0]) / cyc), "law_yaw_deg_per_cycle": float(np.degrees(law_yaw[-1] - law_yaw[0]) / cyc),
                             "twin_speed_mm_s": (1e3 * dxy / (tt[-1] - tt[0])).tolist(), "twin_yaw_deg_s": float(np.degrees(yaw[-1] - yaw[0]) / (tt[-1] - tt[0])),
                             "lateral_excursion_p2p_mm": [1e3 * float(np.ptp(res)), 1e3 * float(np.ptp(lres))]}
            # feet
            clear, drag = [], []
            for l in range(4):
                sw = (st[:, l] == 0) & m
                e = np.flatnonzero(np.diff(np.r_[0, sw.astype(int), 0]))
                for i0, i1 in zip(e[::2], e[1::2]):
                    if i1 - i0 >= 20:
                        clear.append(1e3 * float(rec["feet"][i0:i1, l, 2].max()))
                        drag.append(float(np.mean(rec["fz"][i0:i1, l] > dynamics.DRAG_N)))
            loaded = rec["fz"] > dynamics.CONTACT_N
            marg = np.array([dynamics.support_margin(rec["feet"][i][loaded[i]], rec["com"][i][:2]) for i in np.flatnonzero(m)[::10]])
            marg = marg[np.isfinite(marg)]
            slip = 0.0
            for l in range(4):
                on = loaded[:, l] & m
                e = np.flatnonzero(np.diff(np.r_[0, on.astype(int), 0]))
                for i0, i1 in zip(e[::2], e[1::2]):
                    seg = rec["feet"][i0:i1, l, :2]
                    if len(seg) > 50:
                        slip = max(slip, float(np.max(np.linalg.norm(seg - seg[0], axis=1))))
            rep["feet"] = {"swings": len(clear), "clearance_mm_p10_p50_p90": [float(np.percentile(clear, q)) for q in (10, 50, 90)] if clear else None,
                           "dragging_share_p50_p90": [float(np.percentile(drag, q)) for q in (50, 90)] if drag else None,
                           "support_margin_mm_p05_min": [1e3 * float(np.percentile(marg, 5)), 1e3 * float(marg.min())] if len(marg) else None,
                           "slip_max_mm": 1e3 * slip}
            # joints (release grid): error = read at release k+1 - frame k
            k = np.array([i for i in range(n - 1) if rows[i]["state"] == "WALKING" and a <= i * twin.RELEASE_S <= b])
            cmd = np.array([[int(rows[i][f"raw{j}"]) for j in range(12)] for i in k])
            fb = np.array([[int(rows[i + 1][f"fb_raw{j}"]) for j in range(12)] for i in k])
            spd = np.abs(np.array([[int(rows[i + 1][f"fb_speed{j}"]) for j in range(12)] for i in k]))
            duty = np.abs(np.array([[int(rows[i + 1][f"fb_load{j}"]) for j in range(12)] for i in k]))
            err = np.abs(fb - cmd)
            rep["joints"] = {g: {"err_p95": float(np.percentile(err[:, idx_], 95)), "speed_p95": float(np.percentile(spd[:, idx_], 95)),
                                 "duty_p95": float(np.percentile(duty[:, idx_], 95))} for g, idx_ in (("HIP", [0, 3, 6, 9]), ("UPPER", [1, 4, 7, 10]), ("LOWER", [2, 5, 8, 11]))}
            hips = (np.array([[int(rows[i + 1][f"fb_raw{j}"]) for j in (0, 3, 6, 9)] for i in k]) - 2048) * twin.TICK
            lo, hi = dynamics.box("kHeight")
            rep["hip_margin_kheight_min_ticks"] = float(np.min(np.minimum(hips - lo[[0, 3, 6, 9]], hi[[0, 3, 6, 9]] - hips)) / twin.TICK)
            rp = np.array([dynamics.quat_rpy(q) for q in rec["quat"][m]]) * 1e3
            rep["roll_pitch_p2p_mrad"] = np.ptp(rp, axis=0).tolist()
    stop_cmd = next((i for i, (w, c) in enumerate(ev) if c in ("STOP",) or c.startswith("STOPFRONT")), None)
    if stop_cmd is not None:
        n0 = next((i for i, r in enumerate(rows) if r["stop_active"] == "1"), None)
        if n0 is not None:
            ver = next((i for i in range(n0, n) if rows[i]["stop_verified"] == "1"), None)
            flt = next((i for i in range(n0, n) if rows[i]["state"] == "FAULT"), None)
            legs_in_swing = [l for l in range(4) if stance[n0 - 1, l] == 0]
            rep["stop"] = {"t_s": n0 * twin.RELEASE_S, "verified_s": None if ver is None else (ver - n0) * twin.RELEASE_S,
                           "fault": None if flt is None else rows[flt]["reason"], "fault_s": None if flt is None else (flt - n0) * twin.RELEASE_S,
                           "legs_in_swing": legs_in_swing, "drive": rows[n0]["drive_speed_step_s"]}
        else:
            rep["stop"] = {"t_s": None, "note": "STOP never became active"}
    if write:
        out_dir.mkdir(parents=True, exist_ok=True)
        (out_dir / f"{name}__{label}.json").write_text(json.dumps(rep, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    show(rep)
    return rep


def show(r):
    line = f"{r['scenario']:20s} {r['label']:24s} fault {r['faults']} refusals {len(r['refusals'])}"
    if "travel" in r:
        tr = r["travel"]
        line += (f" | travel/cycle {tr['twin_mm_per_cycle'][0]:6.1f},{tr['twin_mm_per_cycle'][1]:6.1f} (law {tr['law_mm_per_cycle'][0]:6.1f},{tr['law_mm_per_cycle'][1]:6.1f}) mm"
                 f" yaw {tr['twin_yaw_deg_per_cycle']:5.2f} (law {tr['law_yaw_deg_per_cycle']:5.2f}) deg | lat p2p {tr['lateral_excursion_p2p_mm'][0]:5.1f} (law {tr['lateral_excursion_p2p_mm'][1]:5.1f})")
    if "feet" in r:
        f = r["feet"]
        line += (f" | clear p50 {f['clearance_mm_p10_p50_p90'][1]:5.2f} drag {f['dragging_share_p50_p90'][0]:.2f} | support p05 {f['support_margin_mm_p05_min'][0]:5.1f}"
                 f" min {f['support_margin_mm_p05_min'][1]:5.1f} | slip {f['slip_max_mm']:4.1f} | hip {r['hip_margin_kheight_min_ticks']:5.1f}")
        j = r["joints"]
        line += " | err/spd/duty p95 " + " ".join(f"{g[0]} {j[g]['err_p95']:.0f}/{j[g]['speed_p95']:.0f}/{j[g]['duty_p95']:.0f}" for g in j)
    if "stop" in r:
        line += f" | STOP {r['stop']}"
    print(line, flush=True)


def main():
    argv = sys.argv[1:]
    sets = []
    while "--set" in argv:
        i = argv.index("--set")
        sets.append(argv[i + 1])
        del argv[i:i + 2]
    base = "baseline"
    if "--variant" in argv:
        i = argv.index("--variant")
        base = argv[i + 1]
        del argv[i:i + 2]
    if "--fast-law" in argv:
        i = argv.index("--fast-law")
        per, stride_mm, height_mm = (float(x) for x in argv[i + 1].split(":"))
        FAST_LAW.update(period=per, stride=stride_mm * 1e-3, height=height_mm * 1e-3)
        SCENARIOS.update(fast_scenarios(FAST_LAW))
        del argv[i:i + 2]
    binary, out_dir, label, names = pathlib.Path(argv[0]), pathlib.Path(argv[1]), argv[2], argv[3:] or list(SCENARIOS)
    variant, v = m6_dev7.variant_params(base, sets)
    for name in names:
        run(binary, out_dir, label, name, v)


if __name__ == "__main__":
    main()
