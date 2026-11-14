#!/usr/bin/env python3
"""M6 summary: every variant of a run folder against the dev.7 hardware, episode by episode.

Per variant: the 7 ground STOPs (hardware verification interval vs the twin's verification
or fault), and over the walking labels the twin/hardware ratio of the p95 |error| and p95
|duty| per joint group, the hip margin from kHeight, roll/pitch peak-to-peak, the yaw rate,
the twin's real swing clearance and dragging share. Ratios are geometric means over the
walking labels with >= 5 snapshots.

Usage: m6_summary.py <run dir> [--variants a,b] [--json out.json]
"""
import collections
import json
import math
import pathlib
import sys

import numpy as np

GROUPS = ("HIP", "UPPER", "LOWER")
# episodes walked in the fore/aft regime only (kHeight is their STOP box; steered walks use kSteer*)
FIXTURE = "--fixture" in sys.argv  # summarize the fixture episodes alone
FRONTAL = ("S1_", "S2_", "S3_", "S4_", "S6_", "S7_", "W1_", "W2_", "W6_", "W7_", "F_")


def load(run_dir):
    out = collections.defaultdict(dict)
    for f in sorted(pathlib.Path(run_dir).glob("*.json")):
        if f.name.startswith(("M1_", "F_")) and not FIXTURE:
            continue
        if FIXTURE and not f.name.startswith("F_"):
            continue
        r = json.loads(f.read_text())
        out[r["variant"]][r["episode"]] = r
    return out


def walking_rows(r, min_n=5):
    return {k: v for k, v in r["table"].items() if k.startswith(("WALKING", "CHANGE")) and v["samples"] >= min_n}


def gmean(x):
    x = [v for v in x if v and v > 0 and math.isfinite(v)]
    return float(np.exp(np.mean(np.log(x)))) if x else None


def summarize(episodes):
    stops, err, duty, hip, rp, yaw, clear, drag, cmd = [], {g: [] for g in GROUPS}, {g: [] for g in GROUPS}, [], [], [], [], [], []
    speed = {g: [] for g in GROUPS}
    for name, r in sorted(episodes.items()):
        for s in r["stops"]:
            hw, tw = s["hardware"], s["twin"]
            stops.append({"episode": name, "hw": hw["verified_s"], "hw_state": hw["state"], "twin_verified_s": tw.get("verified_s"),
                          "twin_fault": tw.get("fault_reason"), "twin_fault_s": tw.get("fault_s")})
        for lab, row in walking_rows(r).items():
            for g in GROUPS:
                err[g].append(row[g]["abs_err_sim"][1] / max(row[g]["abs_err_hw"][1], 1.0))
                duty[g].append(row[g]["abs_duty_sim"][1] / max(row[g]["abs_duty_hw"][1], 8.0))
                if "abs_speed_hw" in row[g]:
                    speed[g].append(row[g]["abs_speed_sim"][1] / max(row[g]["abs_speed_hw"][1], 10.0))
            if name.startswith(FRONTAL):
                hip.append((name, lab, row["hip_margin_min_hw"], row["hip_margin_min_sim"]))
            if "roll_pitch_p2p_hw" in row:
                rp.append((row["roll_pitch_p2p_hw"], row["roll_pitch_p2p_sim"]))
            cmd.append(max(v[1] for v in row["cmd_diff_abs_p50_p95_ticks"].values()))
        y = r["yaw"]
        if "hw_deg_s" in y and "sim_deg_s" in y and abs(y["hw_deg_s"]) > 0.5:
            yaw.append((name, y["hw_deg_s"], y["sim_deg_s"]))
        f = r["twin_feet"]
        if f and f["clearance_mm_p10_p50_p90"]:
            clear.append(f["clearance_mm_p10_p50_p90"][1])
            drag.append(f["dragging_share_p50_p90"][0])
    verified = sum(s["twin_verified_s"] is not None and not s["twin_fault"] for s in stops)
    within = sum(s["twin_verified_s"] is not None and not s["twin_fault"] and bool(s["hw"]) and s["hw"][0] - 0.1 <= s["twin_verified_s"] <= s["hw"][1] + 0.1 for s in stops)
    hw_hip, sim_hip = [h[2] for h in hip], [h[3] for h in hip]
    return {"stops": stops, "stops_verified": verified, "stops_in_hw_interval": within, "stops_total": len(stops),
            "err_ratio": {g: gmean(err[g]) for g in GROUPS}, "duty_ratio": {g: gmean(duty[g]) for g in GROUPS},
            "speed_ratio": {g: gmean(speed[g]) for g in GROUPS},
            "hip_margin_min": [min(hw_hip) if hw_hip else None, min(sim_hip) if sim_hip else None],
            "hip_margin_mean_of_min": [float(np.mean(hw_hip)) if hw_hip else None, float(np.mean(sim_hip)) if sim_hip else None],
            "roll_pitch_p2p_ratio": [gmean([s[0] / max(h[0], 1) for h, s in rp]), gmean([s[1] / max(h[1], 1) for h, s in rp])],
            "yaw": yaw, "swing_clearance_mm_p50_median": float(np.median(clear)) if clear else None,
            "swing_dragging_share_median": float(np.median(drag)) if drag else None,
            "cmd_alignment_p95_max_ticks": float(max(cmd)) if cmd else None, "episodes": len(episodes)}


def main():
    argv = [a for a in sys.argv[1:] if a != "--fixture"]
    out = None
    if "--json" in argv:
        i = argv.index("--json"); out = argv[i + 1]; del argv[i:i + 2]
    only = None
    if "--variants" in argv:
        i = argv.index("--variants"); only = argv[i + 1].split(","); del argv[i:i + 2]
    data = load(argv[0])
    report = {}
    for variant, episodes in sorted(data.items()):
        if only and variant not in only:
            continue
        s = summarize(episodes)
        report[variant] = s
        print(f"\n== {variant}  ({s['episodes']} episodes)")
        print(f"   STOPs verified {s['stops_verified']}/{s['stops_total']}, inside the hardware interval (+-0.1 s) {s['stops_in_hw_interval']}")
        for x in s["stops"]:
            hw = f"[{x['hw'][0]:.2f}, {x['hw'][1]:.2f}]" if x["hw"] else "-"
            print(f"     {x['episode']:28s} HW {x['hw_state']} {hw} s | twin " +
                  (f"FAULT {x['twin_fault']} at +{x['twin_fault_s']:.2f} s" if x["twin_fault"] else f"verified +{x['twin_verified_s']}"))
        print("   walking twin/HW p95 |err| " + " ".join(f"{g} {s['err_ratio'][g]:.2f}" for g in GROUPS) +
              " | p95 |duty| " + " ".join(f"{g} {s['duty_ratio'][g]:.2f}" for g in GROUPS) +
              (" | p95 |speed| " + " ".join(f"{g} {s['speed_ratio'][g]:.2f}" for g in GROUPS) if s["speed_ratio"]["HIP"] else ""))
        print(f"   fore/aft walks, hip margin from kHeight: min HW {s['hip_margin_min'][0]:.1f} twin {s['hip_margin_min'][1]:.1f} t; mean of label minima HW {s['hip_margin_mean_of_min'][0]:.1f} twin {s['hip_margin_mean_of_min'][1]:.1f}")
        f2 = lambda x, n=2: "-" if x is None else f"{x:.{n}f}"
        print(f"   roll/pitch p2p twin/HW {f2(s['roll_pitch_p2p_ratio'][0])}/{f2(s['roll_pitch_p2p_ratio'][1])}; swing clearance p50 {f2(s['swing_clearance_mm_p50_median'])} mm, "
              f"dragging {f2(s['swing_dragging_share_median'])}; frame alignment p95 max {f2(s['cmd_alignment_p95_max_ticks'], 0)} t")
        for name, h, t in s["yaw"]:
            print(f"     yaw {name:28s} HW {h:+.2f} twin {t:+.2f} deg/s")
    if out:
        pathlib.Path(out).write_text(json.dumps(report, indent=1, default=str) + "\n")


if __name__ == "__main__":
    main()
