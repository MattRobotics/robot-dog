#!/usr/bin/env python3
"""M6: the twin against the 1.0.2-dev.7 ground session (b5610620d0ce, 11/11/2026), read only.

Episodes are fixed windows of the session (controller seconds), each replayed in closed
loop (cosim.simulate: the installed SimpleMotion, fw_bridge built from b561062) from
STANDING at the walk's height with the mode the hardware had selected. The walk's own
commands are replayed as in m5_compare (in-walk commands at the same walk_cycles, the
joystick on its three axes and renewed); STAND commands between walks are not (the twin
starts at the walk height). `hold` replays the M1 static HOLDs (STAND 120, then 110, 105,
120 at the hardware's delays). Hardware snapshots are observed on the twin as the
controller observed the robot (m5_compare.observe).

Usage: m6_dev7.py <cosim binary> <out dir> <hw session dir> <episode|hold|all> [...]
       [--variant NAME] [--set key=value ...] [--tag TAG]
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
import m2_stand_hold
import m5_compare
import m5_hw
import servo
import twin

# name: (from s, to s, height m, mode); every window starts after the previous walk ended
EPISODES = {
    "S1_fast_back_stop": (2588.0, 2604.0, .105, "FAST"),
    "S2_slow_fwd_stop": (2991.0, 3011.0, .120, "SLOW"),
    "S3_standard_fwd_stop": (3023.0, 3047.0, .110, "STANDARD"),
    "S4_fast_fwd_stop": (3059.0, 3085.0, .105, "FAST"),
    "S5_standard_yawcw_stop": (3145.5, 3164.0, .110, "STANDARD"),
    "S6_slow_modechanges_stop": (3259.0, 3323.0, .110, "SLOW"),
    "S7_standard_fwd_stop": (3407.5, 3422.0, .110, "STANDARD"),
    "W1_slow_fwd_ramp": (2490.5, 2520.0, .120, "SLOW"),
    "W2_standard_fwd": (2521.0, 2545.0, .110, "STANDARD"),
    "W3_fast_fwd_diag_right": (2550.5, 2588.0, .105, "FAST"),
    "W4_standard_left": (3111.5, 3129.0, .110, "STANDARD"),
    "W5_standard_yawccw": (3129.5, 3145.5, .110, "STANDARD"),
    "W6_slow_back": (3177.5, 3202.0, .120, "SLOW"),
    "W7_standard_back": (3215.5, 3255.0, .110, "STANDARD"),
    "W8_standard_fwd_diag_right": (3336.5, 3364.0, .110, "STANDARD"),
    "W9_standard_back_diag_right": (3364.5, 3407.0, .110, "STANDARD"),
    # fixture (F3b/F3c, legs unloaded): body welded at the hardware HOLD attitude (IMU minus mount)
    "F_standard_fwd": (1026.0, 1057.0, .110, "STANDARD", "fixture"),
    "F_fast_fwd": (1315.4, 1351.0, .105, "FAST", "fixture"),
}
# M1 (ground block 1): STAND 120 reached 2345.9 (19:31:26), STAND 110 at 2389.472, 105 at 2427.397, 120 at 2466.213
HOLD_CMDS = ((2389.472, .110), (2427.397, .105), (2466.213, .120))
HOLD_FROM, HOLD_TO, HOLD_T0 = 2345.9, 2490.0, 2342.644  # STAND 120 accepted at HOLD_T0
VARIANTS = dict(m5_compare.VARIANTS)
# The corrected ground twin of REPORT_M6.md: contact without creep, body without the Jetson (2.807 kg
# weighed), the IMU offset measured on the dev.7 ground HOLDs (m5_imu_tilt), load-dependent gear friction
VARIANTS["dev7_ground"] = {"model": {"noslip": 5, "base_mass_kg": 1.707}, "imu_offset_mrad": (2.0, 38.9),
                           "params": {"gear_efficiency": 0.5, "gear_friction_v0_rad_s": 0.02}}
ROWS = False  # --rows: also write the twin's release records (<stem>_twin.csv)


def variant_params(name, sets):
    v = dict(VARIANTS[name])
    for kv in sets:
        key, value = kv.split("=")
        if key == "mu":
            v["mu"] = float(value)
        elif key in ("imu_roll_mrad", "imu_pitch_mrad"):  # IMU mounted turned from base_link (m5_imu_tilt)
            off = list(v.get("imu_offset_mrad", (0.0, 0.0)))
            off[key == "imu_pitch_mrad"] = float(value)
            v["imu_offset_mrad"] = tuple(off)
        elif key in ("noslip", "base_mass_kg"):
            v["model"] = dict(v.get("model", {}), **{key: float(value)})
        elif key.startswith("com_"):  # com_x / com_y / com_z (m)
            shift = list(v.get("model", {}).get("com_shift_m", (0.0, 0.0, 0.0)))
            shift["xyz".index(key[-1])] = float(value)
            v["model"] = dict(v.get("model", {}), com_shift_m=tuple(shift))
        else:
            v["params"] = dict(v.get("params", {}), **{key: float(value)})
        name += f"+{key}{value}"
    return name, v


def window(boot, cmds, t_from, t_to):
    snaps = [s for s in boot if t_from * 1e3 <= s["now_ms"] <= t_to * 1e3]
    inside = [c for c in cmds if t_from * 1e3 <= c["t_ms"] <= t_to * 1e3 and c["kind"] not in ("STAND", "PRONE")]
    return snaps, inside


def stop_outcomes(snaps, cmds, rows, fire):
    """Per STOP command: hardware verification interval (s after the STOP) and the twin's."""
    out = []
    for k, c in enumerate(cmds):
        if c["kind"] != "STOP":
            continue
        t0 = c["t_ms"]
        after = [s for s in snaps if s["now_ms"] > t0]
        first = next((s for s in after if s["state"] == "STOPPED" or s["state"] == "FAULT"), None)
        before = [s for s in after if first is None or s["now_ms"] < first["now_ms"]]
        hw = {"state": first and first["state"], "reason": first and first["reason"],
              "verified_s": [((before[-1]["now_ms"] if before else t0) - t0) / 1e3, (first["now_ms"] - t0) / 1e3] if first else None}
        tw = {"fired_s": fire.get(k)}
        if k in fire:
            n0 = int(round(fire[k] / twin.RELEASE_S))
            ver = next((n for n in range(n0, len(rows)) if rows[n]["stop_verified"] == "1"), None)
            flt = next((n for n in range(n0, len(rows)) if rows[n]["state"] == "FAULT"), None)
            tw.update(verified_s=None if ver is None else (ver - n0 + 1) * twin.RELEASE_S,
                      fault_s=None if flt is None else (flt - n0 + 1) * twin.RELEASE_S, fault_reason=None if flt is None else rows[flt]["reason"],
                      state_after_3s=rows[min(n0 + 150, len(rows) - 1)]["state"])
        out.append({"t_s": t0 / 1e3, "hardware": hw, "twin": tw})
    return out


def swing_proxy(pairs, rows):
    """Leg load in swing vs stance, both observed the same way: per hardware snapshot and leg,
    the twin law's stance flag at the read instant of the leg's LOWER (t_sim - age), and the
    |PresentLoad| (per mille) of UPPER and LOWER of that leg on the robot and on the twin."""
    stance = np.array([[int(r[f"stance{l}"]) for l in range(4)] for r in rows])
    out = {"swing": {"hw": [], "sim": []}, "stance": {"hw": [], "sim": []}}
    for x in pairs:
        if not x["hw_label"].startswith(("WALKING", "CHANGE")):
            continue
        for l in range(4):
            j = 3 * l + 2
            n = int(math.ceil((x["t_sim"] - x["age_ms"][j] / 1e3) / twin.RELEASE_S - 1e-9)) - 1
            if not 0 <= n < len(rows) or stance[n, l] < 0:
                continue
            key = "stance" if stance[n, l] else "swing"
            out[key]["hw"].append(float(np.mean(np.abs(x["hw_duty"][[j - 1, j]]))))
            out[key]["sim"].append(float(np.mean(np.abs(x["sim_duty"][[j - 1, j]]))))
    return {k: {"n": len(v["hw"]), "hw_p50_p90": m5_compare.p(np.array(v["hw"]), (50, 90)), "sim_p50_p90": m5_compare.p(np.array(v["sim"]), (50, 90))}
            for k, v in out.items()}


def leg_samples(pairs, rows, rec):
    """Per walking snapshot and leg, at the read instant of the leg's LOWER (t_sim - age): the law's
    stance flag, the time since the leg's last law lift-off (s, swing only), the signed joint-space
    duty of UPPER and LOWER on the robot and on the twin, and the twin foot's normal force (N)."""
    stance = np.array([[int(r[f"stance{l}"]) for l in range(4)] for r in rows])
    out = []
    for x in pairs:
        if not x["hw_label"].startswith(("WALKING", "CHANGE")):
            continue
        for l in range(4):
            j = 3 * l + 2
            t_read = x["t_sim"] - x["age_ms"][j] / 1e3
            n = int(math.ceil(t_read / twin.RELEASE_S - 1e-9)) - 1
            if not 0 <= n < len(rows) or stance[n, l] < 0:
                continue
            k = n
            while k > 0 and stance[k - 1, l] == 0:
                k -= 1
            i = int(np.clip(np.searchsorted(rec["t"], t_read), 0, len(rec["t"]) - 1))
            out.append([x["now_ms"], l, int(stance[n, l]), None if stance[n, l] else (n - k + 1) * twin.RELEASE_S,
                        float(x["hw_duty"][j - 1]), float(x["hw_duty"][j]), float(x["sim_duty"][j - 1]), float(x["sim_duty"][j]),
                        float(rec["fz"][i, l]), 1e3 * float(rec["feet"][i, l, 2])])
    return out


def twin_feet(rows, rec):
    """The twin's real foot motion in swing (never the commanded lift): per swing, the highest
    clearance of the contact reference over the floor and the share of the swing with a
    normal force > DRAG_N; the support margin of the loaded feet (p05, min) while walking."""
    t = rec["t"]
    stance = np.array([[int(r[f"stance{l}"]) for l in range(4)] for r in rows])
    idx = np.clip(np.ceil(t / twin.RELEASE_S - 1e-9).astype(int) - 1, 0, len(rows) - 1)
    st = stance[idx]
    clear, drag = [], []
    for l in range(4):
        sw = st[:, l] == 0
        edges = np.flatnonzero(np.diff(np.r_[0, sw.astype(int), 0]))
        for a, b in zip(edges[::2], edges[1::2]):
            if b - a < 20:
                continue
            z = rec["feet"][a:b, l, 2]
            clear.append(1e3 * float(z.max()))
            drag.append(float(np.mean(rec["fz"][a:b, l] > dynamics.DRAG_N)))
    walking = (st >= 0).all(axis=1)
    loaded = rec["fz"] > dynamics.CONTACT_N
    margin = [dynamics.support_margin(rec["feet"][i][loaded[i]], rec["com"][i][:2]) for i in np.flatnonzero(walking)[::10]]
    margin = np.array([m for m in margin if np.isfinite(m)])
    return {"swings": len(clear), "clearance_mm_p10_p50_p90": m5_compare.p(np.array(clear), (10, 50, 90)),
            "dragging_share_p50_p90": m5_compare.p(np.array(drag), (50, 90)),
            "support_margin_mm_p05_min": [1e3 * float(np.percentile(margin, 5)), 1e3 * float(margin.min())] if len(margin) else None}


def yaw_rates(snaps, pairs, rec):
    """Body yaw rate while WALKING: hardware IMU quaternion (1 Hz) and twin base, fitted per walk segment."""
    def yaw(q):
        w, x, y, z = q
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    hw = [(s["quat_ms"] / 1e3, yaw(s["quat"])) for s in snaps if s["state"] == "WALKING" and s["quat"]]
    sim_t = np.array([x["t_sim"] for x in pairs if x["hw_label"].startswith("WALKING")])
    out = {}
    if len(hw) > 3:
        t, y = np.array(hw).T
        out["hw_deg_s"] = float(np.degrees(np.polyfit(t, np.unwrap(y), 1)[0]))
    if len(sim_t) > 3:
        m = (rec["t"] >= sim_t.min()) & (rec["t"] <= sim_t.max())
        y = np.unwrap([yaw(q) for q in rec["quat"][m]])
        out["sim_deg_s"] = float(np.degrees(np.polyfit(rec["t"][m], y, 1)[0]))
        out["sim_travel_mm_s"] = (1e3 * np.polyfit(rec["t"][m], rec["pos"][m, :2], 1)[0]).tolist()
    return out


def run_episode(binary, out_dir, hw_dir, name, variant, v, tag=""):
    t_from, t_to, height, mode, *where = EPISODES[name]
    files, snaps, trace_cmds = m5_hw.load([hw_dir])
    boot = max(m5_hw.boots(snaps), key=len)
    cmds = m5_hw.commands(trace_cmds, boot)
    sel, inside = window(boot, cmds, t_from, t_to)
    attitude = None
    if where == ["fixture"]:
        hold = np.array([dynamics.quat_rpy(s["quat"]) for s in boot if s["state"] == "STANDING" and s["quat"]
                         and (t_from - 5) * 1e3 <= s["now_ms"] < inside[0]["t_ms"]])
        attitude = tuple(float(x) for x in hold.mean(axis=0) - 1e-3 * np.array(v.get("imu_offset_mrad", (0.0, 0.0))))
    ev, owner = m5_compare.events(sel, inside, mode)
    if mode != "SLOW":
        ev = [(0.5, f"SELECT {mode}")] + ev
    duration = m5_compare.LEAD_S + (sel[-1]["now_ms"] - inside[0]["t_ms"]) / 1e3 + 3.0
    params = dataclasses.replace(servo.Params(), **v.get("params", {}))
    imu_offset = tuple(1e-3 * x for x in v["imu_offset_mrad"]) if "imu_offset_mrad" in v else None
    started = time.time()
    rows, tw, replies, columns = cosim.simulate(binary, ev, duration, params, v.get("mu", 0.8), v.get("unit"), height, attitude, imu_offset, v.get("model"))
    rec = tw.result()
    if mode != "SLOW":
        replies = replies[1:]
    q0, direction = m2_stand_hold.gen1()
    fire, refused = m5_compare.fired(replies, owner)
    pairs = m5_compare.observe(sel, inside, rows, rec, fire, q0, direction, imu_offset)
    lo, hi = dynamics.box("kHeight")
    report = {"episode": name, "window_s": [t_from, t_to], "height_m": height, "mode": mode, "variant": variant, "definition": v,
              "params": dataclasses.asdict(params), "wall_s": round(time.time() - started, 1),
              "events": [[str(w), c] for w, c in ev if not cosim.renewal((w, c))], "twin_refusals": refused, "pairs": len(pairs),
              "stops": stop_outcomes(sel, inside, rows, fire), "table": m5_compare.compare(pairs, lo, hi),
              "fixture_attitude_mrad": None if attitude is None else [1e3 * a for a in attitude],
              "swing_load": swing_proxy(pairs, rows), "twin_feet": twin_feet(rows, rec) if attitude is None else None, "yaw": yaw_rates(sel, pairs, rec),
              "twin_faults": [(r["t_s"], r["reason"]) for r in rows if r["state"] == "FAULT"][:1],
              "hardware_faults": [(s["now_ms"], s["reason"]) for s in sel if s["state"] == "FAULT"][:1],
              "timeline": m5_compare.timeline(sel, rows, inside, fire)}
    out_dir.mkdir(parents=True, exist_ok=True)
    stem = f"{name}__{variant}{tag}"
    if ROWS:
        with open(out_dir / f"{stem}_twin.csv", "w") as f:
            keys = ["t_s"] + columns + [f"fb_{k}{i}" for k in ("raw", "speed", "load") for i in range(12)]
            f.write(",".join(keys) + "\n")
            for r in rows:
                f.write(",".join(str(r[k]) for k in keys) + "\n")
    with open(out_dir / f"{stem}_legs.csv", "w") as f:
        f.write("now_ms,leg,stance,swing_s,hw_upper,hw_lower,sim_upper,sim_lower,sim_fz_n,sim_foot_z_mm\n")
        for row in leg_samples(pairs, rows, rec):
            f.write(",".join("" if v is None else f"{v:.6g}" for v in row) + "\n")
    (out_dir / f"{stem}.json").write_text(json.dumps(report, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    summarize(report)
    return report


def summarize(r):
    print(f"{r['episode']:28s} {r['variant']:30s} pairs {r['pairs']:3d} twin fault {r['twin_faults']} refusals {len(r['twin_refusals'])}", flush=True)
    for s in r["stops"]:
        print(f"   STOP {s['t_s']:.2f}: HW {s['hardware']['state']} in {s['hardware']['verified_s']} s | twin verified {s['twin'].get('verified_s')} s "
              f"fault {s['twin'].get('fault_reason')} at {s['twin'].get('fault_s')}", flush=True)
    for lab, row in r["table"].items():
        if not lab.startswith(("WALKING", "CHANGE", "STOPPED", "STANDING")):
            continue
        line = f"   {lab:22s} n {row['samples']:3d}"
        for g in m5_hw.GROUPS:
            x = row[g]
            line += f" | {g[0]} err p95 {x['abs_err_hw'][1]:4.0f}/{x['abs_err_sim'][1]:4.0f} duty p95 {x['abs_duty_hw'][1]:4.0f}/{x['abs_duty_sim'][1]:4.0f}"
        line += f" | hip min {row['hip_margin_min_hw']:5.1f}/{row['hip_margin_min_sim']:5.1f}"
        print(line, flush=True)
    sl = r["swing_load"]
    print(f"   leg UL load swing HW/sim p50 {sl['swing']['hw_p50_p90'] and round(sl['swing']['hw_p50_p90'][0])}/{sl['swing']['sim_p50_p90'] and round(sl['swing']['sim_p50_p90'][0])} "
          f"stance {sl['stance']['hw_p50_p90'] and round(sl['stance']['hw_p50_p90'][0])}/{sl['stance']['sim_p50_p90'] and round(sl['stance']['sim_p50_p90'][0])} | "
          f"twin swing clearance p10/50/90 {r['twin_feet'] and r['twin_feet']['clearance_mm_p10_p50_p90']} drag {r['twin_feet'] and r['twin_feet']['dragging_share_p50_p90']} | yaw {r['yaw']}", flush=True)


def run_hold(binary, out_dir, hw_dir, variant, v, tag=""):
    """M1: the twin's static HOLDs at 120/110/105/120 mm vs the hardware's, per joint (joint space)."""
    files, snaps, _ = m5_hw.load([hw_dir])
    boot = max(m5_hw.boots(snaps), key=len)
    ev = [(t - HOLD_T0, f"HEIGHT {h}") for t, h in HOLD_CMDS]
    params = dataclasses.replace(servo.Params(), **v.get("params", {}))
    imu_offset = tuple(1e-3 * x for x in v["imu_offset_mrad"]) if "imu_offset_mrad" in v else None
    rows, tw, replies, columns = cosim.simulate(binary, ev, HOLD_TO - HOLD_T0, params, v.get("mu", 0.8), v.get("unit"), .120, None, imu_offset, v.get("model"))
    rec = tw.result()
    q0, direction = m2_stand_hold.gen1()
    holds, out = [], {"variant": variant, "definition": v, "params": dataclasses.asdict(params), "refusals": [r for r in replies if r[2] != "OK -"], "holds": []}
    edges = [HOLD_FROM] + [t for t, _ in HOLD_CMDS] + [HOLD_TO]
    heights = [.120] + [h for _, h in HOLD_CMDS]
    for a, b, h in zip(edges, edges[1:], heights):
        hw = [s for s in boot if a * 1e3 <= s["now_ms"] <= b * 1e3 and s["state"] == "STANDING" and s["phase"] == "HOLD" and s["torque"]]
        hw = hw[2:]  # 2 s after the arrival
        if not hw:
            continue
        ta, tb = hw[0]["now_ms"] / 1e3 - HOLD_T0, hw[-1]["now_ms"] / 1e3 - HOLD_T0
        n = [k for k in range(len(rows)) if ta <= k * twin.RELEASE_S <= tb and rows[k]["state"] == "STANDING"]
        cmd = np.array([[int(rows[k][f"raw{i}"]) for i in range(12)] for k in n])
        fb = np.array([[int(rows[k][f"fb_raw{i}"]) for i in range(12)] for k in n])
        fbl = np.array([[int(rows[k][f"fb_load{i}"]) for i in range(12)] for k in n])
        hw_err = np.median([direction * (s["raw"] - s["cmd"]) for s in hw], axis=0)
        hw_load = np.median([-direction * s["load"] for s in hw], axis=0)
        sim_err = np.median(fb[1:] - cmd[:-1], axis=0)   # the read before release k+1 against the frame of release k
        sim_load = np.median(-fbl, axis=0)                # fb_load = PresentLoad convention (-duty): joint-space duty = -fb_load
        rp = np.array([dynamics.quat_rpy(s["quat"]) for s in hw if s["quat"]]) * 1e3
        out["holds"].append({"height_m": h, "hw_window_s": [hw[0]["now_ms"] / 1e3, hw[-1]["now_ms"] / 1e3], "hw_n": len(hw),
                             "err_hw": hw_err.tolist(), "err_sim": sim_err.tolist(), "duty_hw": hw_load.tolist(), "duty_sim": sim_load.tolist(),
                             "hw_roll_pitch_mrad_median": np.median(rp, axis=0).tolist() if len(rp) else None})
        print(f"HOLD {h * 1e3:.0f} mm n {len(hw)}  joint-space err (t) HW/twin and duty (permille) HW/twin", flush=True)
        for g, idx in m5_hw.GROUPS.items():
            print(f"   {g:5s} err HW {np.round(hw_err[idx]).astype(int)} twin {np.round(sim_err[idx]).astype(int)} | "
                  f"duty HW {np.round(hw_load[idx]).astype(int)} twin {np.round(sim_load[idx]).astype(int)}", flush=True)
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / f"M1_hold__{variant}{tag}.json").write_text(json.dumps(out, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    return out


def main():
    argv = sys.argv[1:]
    opt, sets = {}, []
    while "--set" in argv:
        i = argv.index("--set")
        sets.append(argv[i + 1])
        del argv[i:i + 2]
    for key in ("--variant", "--tag"):
        if key in argv:
            i = argv.index(key)
            opt[key] = argv[i + 1]
            del argv[i:i + 2]
    binary, out_dir, hw_dir, what = pathlib.Path(argv[0]), pathlib.Path(argv[1]), argv[2], argv[3:]
    variant, v = variant_params(opt.get("--variant", "baseline"), sets)
    tag = opt.get("--tag", "")
    global ROWS
    ROWS = "--rows" in what
    what = [w for w in what if w != "--rows"]
    names = list(EPISODES) if what == ["all"] else what
    for name in names:
        if name == "hold":
            run_hold(binary, out_dir, hw_dir, variant, v, tag)
        else:
            run_episode(binary, out_dir, hw_dir, name, variant, v, tag)


if __name__ == "__main__":
    main()
