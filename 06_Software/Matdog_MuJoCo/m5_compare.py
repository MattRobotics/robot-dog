#!/usr/bin/env python3
"""M5: the twin against a hardware run, through the hardware's own sampling.

The run's commands (m5_hw) are replayed in closed loop (cosim.simulate: production
SimpleMotion + twin) from STANDING at the run's height: a command given while
WALKING fires at the same walk_cycles, any other at the same delay from the previous
command. Each hardware snapshot is then observed on the twin as the controller
observed the robot: the frame written at that instant, each joint's position and
load `age_ms` earlier, the IMU at its own sampling time. Snapshots are aligned on
walk_cycles while walking, else on the delay from the last command. Errors are
measured - commanded in joint-space ticks; duty is the PresentLoad in joint space
(per mille). Simulation against hardware: neither replaces the other.

Usage: m5_compare.py <cosim binary> <out dir> <name> <hw file|dir> [...]
       [--from S] [--to S] [--mode SLOW] [--fixture] [--variant NAME]
"""
import csv
import dataclasses
import json
import math
import pathlib
import sys

import numpy as np

import cosim
import dynamics
import m2_stand_hold
import m5_hw
import servo
import twin

LEAD_S = 1.5  # twin HOLD before the first command (the run's own HOLD is not replayed)
STRIDES_MM = {"SLOW": (50, 75, 100), "STANDARD": (60, 75, 90), "FAST": (60, 70, 80)}  # WALK presets, SimpleMotion.cpp kModes
# One motivated physical correction at a time (name -> {"params": {...}, "mu": .., "unit": [...], "imu_offset_mrad": (roll, pitch)}).
VARIANTS = {"baseline": {},
            # IMU mounted turned from base_link: on the ground HOLDs of dev.4 (08/11, traces 1-3, 184 distinct snapshots,
            # m5_imu_tilt.py) the IMU tilt exceeds the tilt of the four-feet plane from the encoders by pitch 22.9 mrad
            # (median; p5-p95 21.8-37.0), roll -0.5
            "imu_mount": {"imu_offset_mrad": (-0.5, 22.9)}}


def episode(snaps, cmds, t_from, t_to):
    """Snapshots and commands of one replayable stretch: from the STANDING HOLD before the first
    command in [t_from, t_to] (s) up to the next STAND or PRONE (the twin cannot replay those)."""
    inside = [c for c in cmds if t_from * 1e3 <= c["t_ms"] <= t_to * 1e3]
    while inside and inside[0]["kind"] in ("STAND", "PRONE"):
        inside = inside[1:]
    stop = next((k for k, c in enumerate(inside) if c["kind"] in ("STAND", "PRONE")), len(inside))
    end = inside[stop]["t_ms"] - 1 if stop < len(inside) else t_to * 1e3
    inside = inside[:stop]
    hold = [s for s in snaps if s["now_ms"] < inside[0]["t_ms"] and s["state"] == "STANDING" and s["phase"] == "HOLD"]
    sel = [s for s in snaps if hold[-1]["now_ms"] - 2000 <= s["now_ms"] <= end]
    return sel, inside, hold[-1]


def walk_phase_at(snaps, t_ms):
    """walk_cycles at a command given while WALKING: interpolated between the snapshots around it,
    or extrapolated at the walk's own rate when the next snapshot is no longer WALKING (a STOP)."""
    before = [s for s in snaps if s["now_ms"] <= t_ms]
    after = [s for s in snaps if s["now_ms"] > t_ms]
    if not before or before[-1]["state"] != "WALKING":
        return None
    a = before[-1]
    if after and after[0]["state"] == "WALKING" and after[0]["walk_cycles"] >= a["walk_cycles"]:
        b = after[0]
        return float(np.interp(t_ms, [a["now_ms"], b["now_ms"]], [a["walk_cycles"], b["walk_cycles"]]))
    prev = [s for s in before[:-1] if s["state"] == "WALKING" and s["walk_cycles"] < a["walk_cycles"]]
    rate = (a["walk_cycles"] - prev[-1]["walk_cycles"]) / (a["now_ms"] - prev[-1]["now_ms"]) if prev else 0.0
    return float(a["walk_cycles"] + rate * (t_ms - a["now_ms"]))


def events(snaps, cmds, mode):
    """cosim events and, per event, the index of the hardware command it replays: in-walk commands on
    walk_cycles, the others on the delay from the previous command (the first after LEAD_S)."""
    out, owner, selected = [], [], None
    for k, c in enumerate(cmds):
        wc = walk_phase_at(snaps, c["t_ms"])
        when = ("cycles", wc) if wc is not None else (LEAD_S if k == 0 else ("after", (c["t_ms"] - cmds[k - 1]["t_ms"]) / 1e3))
        kind = c["kind"]
        if kind == "WALK":  # label 'WALK <mode> <stride> mm'; before dev.6 'WALK STANDARD <stride> mm' was the SLOW law
            before = [s for s in snaps if s["now_ms"] <= c["t_ms"]]
            m = (before[-1]["selected"] if before else None) or mode
            words = (c["label"] or "").split()
            preset = STRIDES_MM[m].index(int(words[2])) if len(words) >= 4 and words[2].isdigit() else 2
            # the twin's selection follows the hardware's: re-selected only when it differs (a STOP latch refuses it)
            add = ([(when, f"SELECT {m}"), (("after", 0.0), f"WALK {preset}")] if m != selected else [(when, f"WALK {preset}")])
            selected = m
        elif kind == "SELECT":
            add = [(when, f"SELECT {c['arg']}")]
            selected = c["arg"]
        elif kind in ("WALKSTOP", "STOP"):
            add = [(when, kind)]
        elif kind == "DRIVE":
            vx, vy, yaw = (tuple(c["arg"]) + (0.0,))[:3]
            if abs(vy) > 5e-4 or abs(yaw) > 1e-3:
                raise SystemExit(f"command at {c['t_ms'] / 1e3:.3f} s: lateral {vy * 1e3:.1f} mm/s, yaw {yaw:.3f} rad/s, "
                                 "not reproducible (cosim DRIVE is frontal only)")
            add = [(when, f"DRIVE {vx:.6f}")]
        else:
            raise SystemExit(f"command {kind} at {c['t_ms'] / 1e3:.3f} s not reproducible inside an episode")
        out += add
        owner += [k] * len(add)
    return out, owner


def fired(replies, owner):
    """Twin time at which each hardware command fired (its first event) and the twin's refusals."""
    fire, refused = {}, []
    for (t, cmd, reply), k in zip(replies, owner):
        fire.setdefault(k, t)
        if reply != "OK -":
            refused.append({"t": t, "command": cmd, "reply": reply})
    return fire, refused


def observe(snaps, cmds, rows, rec, fire, q0, direction, imu_offset=None):
    """Paired samples: each hardware snapshot and the twin observed the same way."""
    n_rows = len(rows)
    sim_wc = np.array([float(r["walk_cycles"]) for r in rows])
    sim_walk = np.array([r["state"] == "WALKING" for r in rows])
    t_rel = np.arange(n_rows) * twin.RELEASE_S  # TICK time of release n (its frame applies after it)
    seg_sim = np.cumsum(np.r_[sim_walk[0], sim_walk[1:] & ~sim_walk[:-1]]) * sim_walk  # 1-based walking segment
    rec_t = rec["t"]
    out, seg_hw, was_walking = [], 0, False
    for s in snaps:
        walking = s["state"] == "WALKING"
        seg_hw += walking and not was_walking
        was_walking = walking
        t_sim = None
        if walking:
            m = seg_sim == seg_hw
            if m.any() and sim_wc[m][0] <= s["walk_cycles"] <= sim_wc[m][-1]:
                t_sim = float(np.interp(s["walk_cycles"], sim_wc[m], t_rel[m]))
        else:
            last = [i for i, c in enumerate(cmds) if c["t_ms"] <= s["now_ms"]]
            if last and last[-1] in fire:
                t_sim = fire[last[-1]] + (s["now_ms"] - cmds[last[-1]]["t_ms"]) / 1e3
            elif not last:
                t_sim = LEAD_S - (cmds[0]["t_ms"] - s["now_ms"]) / 1e3
        if t_sim is None or t_sim < 0.3 or t_sim > t_rel[-1]:
            continue
        n = min(max(int(math.ceil(t_sim / twin.RELEASE_S - 1e-9)) - 1, 0), n_rows - 1)
        r = rows[n]
        cmd_q = (np.array([int(r[f"raw{i}"]) for i in range(12)]) - 2048) * twin.TICK
        at = np.clip(np.searchsorted(rec_t, t_sim - s["age_ms"] / 1e3), 0, len(rec_t) - 1)
        q = np.rint(rec["q"][at, np.arange(12)] / twin.TICK) * twin.TICK
        duty = 1e3 * rec["duty"][at, np.arange(12)]
        iq = int(np.clip(np.searchsorted(rec_t, t_sim - (s["now_ms"] - (s["quat_ms"] or s["now_ms"])) / 1e3), 0, len(rec_t) - 1))
        out.append({
            "now_ms": s["now_ms"], "t_sim": t_sim, "hw_label": m5_hw.label(s), "sim_label": label_row(r),
            "hw_reason": s["reason"], "sim_reason": r["reason"], "hw_wc": s["walk_cycles"], "sim_wc": float(r["walk_cycles"]),
            "hw_err": direction * (s["raw"] - s["cmd"]), "sim_err": (q - cmd_q) / twin.TICK,
            "hw_duty": -direction * s["load"], "sim_duty": duty, "hw_q": direction * (s["raw"] - q0) * twin.TICK, "sim_q": q,
            "cmd_diff": (direction * (s["cmd"] - q0) * twin.TICK - cmd_q) / twin.TICK,  # alignment check: same law, same phase
            "stance": [int(r[f"stance{l}"]) for l in range(4)],
            "hw_rp": np.array(dynamics.quat_rpy(s["quat"])) * 1e3 if s["quat"] else np.full(2, np.nan),
            "sim_rp": np.array(dynamics.quat_rpy(cosim.imu_quaternion(rec["quat"][iq], imu_offset))) * 1e3, "age_ms": s["age_ms"], "torque": s["torque"]})
    return out


def label_row(r):
    if r["state"] == "WALKING":
        a, b = r["law_mode"], r["target_mode"]
        return f"WALKING {a}" if a == b else f"CHANGE {a}->{b}"
    return f"{r['state']} {r['phase']}"


def margins(q, lo, hi):
    """Hip distance to the kHeight box edge (ticks, negative = outside), per hip."""
    hips = np.array(m5_hw.GROUPS["HIP"])
    return np.minimum(q[..., hips] - lo[hips], hi[hips] - q[..., hips]) / twin.TICK


def p(a, qs=(50, 95, 100)):
    return [float(np.percentile(a, v)) for v in qs] if len(a) else None


def compare(pairs, lo, hi, hw_label_only=False):
    """Per hardware label: hardware vs twin distributions and paired differences."""
    table = {}
    for name in dict.fromkeys(x["hw_label"] for x in pairs):
        sel = [x for x in pairs if x["hw_label"] == name and x["torque"]]
        if not sel:
            continue
        same = float(np.mean([x["sim_label"] == name or (name == "WALKING None" and x["sim_label"].startswith("WALKING")) for x in sel]))
        cd = np.abs(np.array([x["cmd_diff"] for x in sel]))
        row = {"samples": len(sel), "twin_same_label": same, "cmd_diff_abs_p50_p95_ticks": {g: p(cd[:, idx].ravel(), (50, 95)) for g, idx in m5_hw.GROUPS.items()}}
        for g, idx in m5_hw.GROUPS.items():
            he = np.array([x["hw_err"][idx] for x in sel])
            se = np.array([x["sim_err"][idx] for x in sel])
            hd = np.array([x["hw_duty"][idx] for x in sel])
            sd = np.array([x["sim_duty"][idx] for x in sel])
            row[g] = {"abs_err_hw": p(np.abs(he).ravel()), "abs_err_sim": p(np.abs(se).ravel()),
                      "err_rmse_sim_minus_hw": float(np.sqrt(np.mean((se - he) ** 2))), "err_bias_sim_minus_hw": float(np.mean(se - he)),
                      "abs_duty_hw": p(np.abs(hd).ravel()), "abs_duty_sim": p(np.abs(sd).ravel()),
                      "duty_rmse_sim_minus_hw": float(np.sqrt(np.mean((sd - hd) ** 2))), "duty_bias_sim_minus_hw": float(np.mean(sd - hd))}
        hm = margins(np.array([x["hw_q"] for x in sel]), lo, hi)
        sm = margins(np.array([x["sim_q"] for x in sel]), lo, hi)
        row["hip_margin_min_hw"], row["hip_margin_min_sim"] = float(hm.min()), float(sm.min())
        row["hip_margin_p05_hw"], row["hip_margin_p05_sim"] = float(np.percentile(hm.min(axis=1), 5)), float(np.percentile(sm.min(axis=1), 5))
        row["hip_outside_pct_hw"], row["hip_outside_pct_sim"] = float(100 * np.mean(hm.min(axis=1) < 0)), float(100 * np.mean(sm.min(axis=1) < 0))
        hrp = np.array([x["hw_rp"] for x in sel])
        srp = np.array([x["sim_rp"] for x in sel])
        ok = np.isfinite(hrp).all(axis=1)
        if ok.sum() > 2:
            dh, ds = hrp[ok] - hrp[ok].mean(axis=0), srp[ok] - srp[ok].mean(axis=0)
            row["roll_pitch_p2p_hw"] = np.ptp(hrp[ok], axis=0).tolist()
            row["roll_pitch_p2p_sim"] = np.ptp(srp[ok], axis=0).tolist()
            row["roll_pitch_mean_hw"] = hrp[ok].mean(axis=0).tolist()
            row["roll_pitch_mean_sim"] = srp[ok].mean(axis=0).tolist()
            row["roll_pitch_rmse_demeaned"] = np.sqrt(np.mean((ds - dh) ** 2, axis=0)).tolist()
        table[name] = row
    return table


def timeline(snaps, rows, cmds, fire_t):
    """Motion states of both, from each command: hardware between two snapshots, twin per release."""
    hw = m5_hw.transitions(snaps)
    sim, prev = [], None
    for n, r in enumerate(rows):
        key = (r["state"], r["phase"], r["reason"])
        if key != prev:
            sim.append({"t": n * twin.RELEASE_S, "state": r["state"], "phase": r["phase"], "reason": r["reason"]})
            prev = key
    return {"hardware": hw, "twin": sim, "commands": [dict(c, twin_fired_s=fire_t.get(i)) for i, c in enumerate(cmds)]}


def main():
    argv = sys.argv[1:]
    opt = {}
    for key in ("--from", "--to", "--mode", "--variant"):
        if key in argv:
            i = argv.index(key)
            opt[key] = argv[i + 1]
            del argv[i:i + 2]
    fixture = "--fixture" in argv
    argv = [a for a in argv if a != "--fixture"]
    binary, out_dir, name, sources = pathlib.Path(argv[0]), pathlib.Path(argv[1]), argv[2], argv[3:]
    variant = opt.get("--variant", "baseline")
    v = VARIANTS[variant]
    files, snaps, trace_cmds = m5_hw.load(sources)
    boot = m5_hw.boots(snaps)[-1] if len(m5_hw.boots(snaps)) == 1 else max(m5_hw.boots(snaps), key=len)
    cmds = m5_hw.commands(trace_cmds, boot)
    t_from, t_to = float(opt.get("--from", 0)), float(opt.get("--to", 1e9))
    sel, ep_cmds, hold = episode(boot, cmds, t_from, t_to)
    height = round(hold["height_mm"]) / 1000
    attitude = None
    if fixture:
        hold_rp = np.array([dynamics.quat_rpy(s["quat"]) for s in sel if s["state"] == "STANDING" and s["quat"] and s["now_ms"] < ep_cmds[0]["t_ms"]])
        attitude = tuple(float(x) for x in hold_rp.mean(axis=0) - 1e-3 * np.array(v.get("imu_offset_mrad", (0.0, 0.0))))  # body = IMU - mount
    ev, owner = events(sel, ep_cmds, opt.get("--mode", "SLOW"))
    duration = LEAD_S + (sel[-1]["now_ms"] - ep_cmds[0]["t_ms"]) / 1e3 + 3.0
    params = dataclasses.replace(servo.Params(), **v.get("params", {}))
    print(f"{name}: {len(files)} file(s), episode {sel[0]['now_ms'] / 1e3:.2f}-{sel[-1]['now_ms'] / 1e3:.2f} s, {len(sel)} snapshots, "
          f"height {height * 1e3:.0f} mm, {'fixture ' + str(np.round(np.array(attitude) * 1e3, 1)) + ' mrad' if fixture else 'ground'}, variant {variant}", flush=True)
    for e, k in zip(ev, owner):
        print(f"  hw {ep_cmds[k]['t_ms'] / 1e3:9.3f} s {ep_cmds[k]['kind']:8s} -> twin {e}", flush=True)
    imu_offset = tuple(1e-3 * x for x in v["imu_offset_mrad"]) if "imu_offset_mrad" in v else None
    rows, tw, replies, columns = cosim.simulate(binary, ev, duration, params, v.get("mu", 0.8), v.get("unit"), height, attitude, imu_offset)
    rec = tw.result()
    q0, direction = m2_stand_hold.gen1()
    fire_t, refused = fired(replies, owner)
    pairs = observe(sel, ep_cmds, rows, rec, fire_t, q0, direction, imu_offset)
    lo, hi = dynamics.box("kHeight")
    table = compare(pairs, lo, hi)
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = f"{name}__{variant}"
    with open(out_dir / f"{tag}_twin.csv", "w", newline="") as f:
        feedback = [f"fb_{k}{i}" for k in ("raw", "speed", "load") for i in range(12)]
        w = csv.DictWriter(f, fieldnames=["t_s"] + columns + feedback)
        w.writeheader()
        w.writerows(rows)
    with open(out_dir / f"{tag}_pairs.csv", "w", newline="") as f:
        w = csv.writer(f)
        cols = ["now_ms", "t_sim", "hw_label", "sim_label", "hw_wc", "sim_wc"]
        w.writerow(cols + [f"{k}_{n}" for k in ("hw_err", "sim_err", "hw_duty", "sim_duty", "hw_q", "sim_q", "age_ms") for n in m5_hw.SLOT_NAMES]
                   + ["hw_roll", "hw_pitch", "sim_roll", "sim_pitch"])
        for x in pairs:
            w.writerow([x[c] for c in cols] + [f"{v:.5g}" for k in ("hw_err", "sim_err", "hw_duty", "sim_duty", "hw_q", "sim_q", "age_ms") for v in x[k]]
                       + [f"{v:.4g}" for v in (*x["hw_rp"], *x["sim_rp"])])
    report = {"name": name, "variant": variant, "variant_definition": v, "sources": [str(f) for f in files],
              "episode_sources": {k: sum(s["source"] == k for s in sel) for k in dict.fromkeys(s["source"] for s in sel)},
              "episode_now_s": [sel[0]["now_ms"] / 1e3, sel[-1]["now_ms"] / 1e3], "height_m": height,
              "conditions": "fixture (body welded, rigid; attitude from the hardware HOLD)" if fixture else "ground (flat, mu 0.8)",
              "fixture_attitude_mrad": None if attitude is None else [1e3 * a for a in attitude],
              "events": [[str(w), c] for w, c in ev], "twin_refusals": refused, "pairs": len(pairs), "rates": m5_hw.rates(sel),
              "table": table, "timeline": timeline(sel, rows, ep_cmds, fire_t),
              "twin_faults": [(r["t_s"], r["reason"]) for r in rows if r["state"] == "FAULT"][:1],
              "hardware_faults": [(s["now_ms"], s["reason"]) for s in sel if s["state"] == "FAULT"][:1]}
    (out_dir / f"{tag}.json").write_text(json.dumps(report, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")
    print(f"  pairs {len(pairs)}; twin refusals {refused}; twin fault {report['twin_faults']}; hardware fault {report['hardware_faults']}")
    for lab, row in table.items():
        line = f"  {lab:24s} n {row['samples']:3d} same {row['twin_same_label']:.2f} cmd|d| p95 " + "/".join(f"{v[1]:.0f}" for v in row["cmd_diff_abs_p50_p95_ticks"].values())
        for g in m5_hw.GROUPS:
            r = row[g]
            line += f" | {g} |err| p95 hw {r['abs_err_hw'][1]:5.1f} sim {r['abs_err_sim'][1]:5.1f} rmse {r['err_rmse_sim_minus_hw']:5.1f}" \
                    f" |duty| p95 hw {r['abs_duty_hw'][1]:4.0f} sim {r['abs_duty_sim'][1]:4.0f}"
        line += f" | hip margin min hw {row['hip_margin_min_hw']:5.1f} sim {row['hip_margin_min_sim']:5.1f}"
        if "roll_pitch_p2p_hw" in row:
            line += f" | roll/pitch p2p hw {row['roll_pitch_p2p_hw'][0]:4.0f}/{row['roll_pitch_p2p_hw'][1]:4.0f} sim {row['roll_pitch_p2p_sim'][0]:4.0f}/{row['roll_pitch_p2p_sim'][1]:4.0f}"
        print(line, flush=True)


if __name__ == "__main__":
    main()
