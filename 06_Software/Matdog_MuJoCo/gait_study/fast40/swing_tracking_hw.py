#!/usr/bin/env python3
"""FAST 40 study, priority 2 (2026-11-13): is the twin's swing-leg servo lag real? dev.7 hardware vs twin.

The M6 ground episodes of the dev.7 session (m6_dev7.EPISODES, hardware read-only) are replayed in the
corrected twin exactly as m6_dev7.run_episode does; every hardware snapshot is observed on the twin the same
way (m5_compare.observe: each joint at its own read age). For each walking snapshot, leg and joint, the
leg's law phase at that joint's read instant (stance, or swing progress u) and the measured-minus-commanded
error (ticks) on the robot and on the twin. Output: per mode, leg pair (front/hind) and joint, in mid swing
(u .25-.75) and in stance: n, signed median and p90 of |error|, hardware and twin.

The hardware logs hold no foot height and no foot force (REPORT_M6 §2): this compares only the joint lag
that, in the twin, keeps the swing foot on the floor.

Usage: swing_tracking_hw.py <dev.7 cosim> <hardware dir> <out.json> [episode ...]
"""
import dataclasses
import json
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))  # Matdog_MuJoCo

import cosim
import m2_stand_hold
import m5_compare
import m5_hw
import m6_dev7
import servo
import twin

JOINT = ("HIP", "UPPER", "LOWER")


def samples(binary, hw_dir, name, v):
    t_from, t_to, height, mode, *where = m6_dev7.EPISODES[name]
    _, snaps, trace_cmds = m5_hw.load([hw_dir])
    boot = max(m5_hw.boots(snaps), key=len)
    cmds = m5_hw.commands(trace_cmds, boot)
    sel, inside = m6_dev7.window(boot, cmds, t_from, t_to)
    ev, owner = m5_compare.events(sel, inside, mode)
    if mode != "SLOW":
        ev = [(0.5, f"SELECT {mode}")] + ev
    duration = m5_compare.LEAD_S + (sel[-1]["now_ms"] - inside[0]["t_ms"]) / 1e3 + 3.0
    params = dataclasses.replace(servo.Params(), **v.get("params", {}))
    imu = tuple(1e-3 * x for x in v["imu_offset_mrad"])
    rows, tw, replies, _ = cosim.simulate(binary, ev, duration, params, v.get("mu", 0.8), v.get("unit"), height, None, imu, v.get("model"))
    rec = tw.result()
    if mode != "SLOW":
        replies = replies[1:]
    q0, direction = m2_stand_hold.gen1()
    fire, _ = m5_compare.fired(replies, owner)
    pairs = m5_compare.observe(sel, inside, rows, rec, fire, q0, direction, imu)
    stance = np.array([[int(r[f"stance{l}"]) for l in range(4)] for r in rows])
    out = []
    for x in pairs:
        if not x["hw_label"].startswith("WALKING") or not x["torque"]:
            continue
        for l in range(4):
            for k in range(3):
                j = 3 * l + k
                n = int(math.ceil((x["t_sim"] - x["age_ms"][j] / 1e3) / twin.RELEASE_S - 1e-9)) - 1
                if not 0 <= n < len(rows) or stance[n, l] < 0:
                    continue
                u = None
                if stance[n, l] == 0:
                    a = n
                    while a > 0 and stance[a - 1, l] == 0:
                        a -= 1
                    b = n
                    while b + 1 < len(rows) and stance[b + 1, l] == 0:
                        b += 1
                    u = (n - a + .5) / (b - a + 1)
                out.append({"episode": name, "mode": x["hw_label"].split()[1], "leg": l, "joint": JOINT[k], "u": u,
                            "hw_err": float(x["hw_err"][j]), "sim_err": float(x["sim_err"][j]), "age_ms": float(x["age_ms"][j])})
    return out


def main():
    binary, hw_dir, out = sys.argv[1], sys.argv[2], sys.argv[3]
    names = sys.argv[4:] or [n for n, e in m6_dev7.EPISODES.items() if len(e) == 4]
    _, v = m6_dev7.variant_params("dev7_ground", [])
    allx = []
    for name in names:
        s = samples(binary, hw_dir, name, v)
        allx += s
        print(f"{name}: {len(s)} joint samples", flush=True)
    table = {}
    for mode in ("SLOW", "STANDARD", "FAST"):
        for pair, legs in (("front", (0, 1)), ("hind", (2, 3))):
            for jn in JOINT:
                for phase in ("mid_swing", "stance"):
                    sel = [x for x in allx if x["mode"] == mode and x["leg"] in legs and x["joint"] == jn
                           and ((phase == "stance" and x["u"] is None) or (phase == "mid_swing" and x["u"] is not None and .25 <= x["u"] <= .75))]
                    if not sel:
                        continue
                    h = np.array([x["hw_err"] for x in sel])
                    s = np.array([x["sim_err"] for x in sel])
                    table[f"{mode} {pair} {jn} {phase}"] = {"n": len(sel), "hw_median": float(np.median(h)), "sim_median": float(np.median(s)),
                                                           "hw_abs_p90": float(np.percentile(np.abs(h), 90)), "sim_abs_p90": float(np.percentile(np.abs(s), 90))}
    json.dump({"episodes": names, "variant": "dev7_ground", "samples": allx, "table": table}, open(out, "w"), indent=1)
    for k, r in table.items():
        print(f"{k:34s} n {r['n']:3d} | median HW {r['hw_median']:6.0f} twin {r['sim_median']:6.0f} | |err| p90 HW {r['hw_abs_p90']:5.0f} twin {r['sim_abs_p90']:5.0f}")


if __name__ == "__main__":
    main()
