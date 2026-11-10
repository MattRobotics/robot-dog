#!/usr/bin/env python3
"""STOP_MEASURED_OUTSIDE_BOX during WALK: the hips, separated by source of evidence.

Usage: m4_hip_stop.py <cosim binary> <nominal cl dir> <out dir> <hardware trace json> [...]

1. HW (real): measured hip angle (gen-1 map) against kHeight in dashboard traces of
   ground walks, while walking and during the STOPs that followed (sparse samples).
2. Twin (simulated): the same quantities in the nominal closed-loop runs (cosim.py),
   from the feedback SimpleMotion read.
3. Attribution: each STOP_MEASURED_OUTSIDE_BOX scenario re-run closed loop with one
   change: hips with a doubled torque constant (unit 2, about the HW hip deflection),
   foot friction 0.5, or the ideal actuator. Diagnostic variants, not the nominal model.
The firmware check (SimpleMotion.cpp generateStop) is reported, never modified.
"""
import json
import multiprocessing
import pathlib
import sys

import numpy as np

import cosim
import dynamics
import m3_replays
import servo
import twin
from m2_stand_hold import gen1

HIPS = (0, 3, 6, 9)
BOX_SCENARIOS = ("cl_stop_front_slow", "cl_stop_front_standard", "cl_stop_front_fast", "cl_stop_lateswing_slow")
VARIANTS = {  # name: (servo overrides, foot friction, unit per slot)
    "hips_x2": ({}, 0.8, [2.0 if i in HIPS else 1.0 for i in range(12)]),
    "mu_0.5": ({}, 0.5, None),
    "ideal_actuator": (m3_replays.VARIANTS["ideal_actuator"], 0.8, None),
}


def margin_ticks(q):
    lo, hi = dynamics.box()
    return np.minimum(q - lo, hi - q) / twin.TICK


def hardware(paths):
    q0, direction = gen1()
    out = []
    for path in paths:
        samples = [s["status"] for s in json.loads(pathlib.Path(path).read_text())["samples"]]
        for label, phases in (("walk", ("WALK_STANDARD",)), ("stop_after_walk", ("STOP",))):
            rows = [s for s in samples if s.get("phase") in phases and s.get("torque_active")]
            if label == "stop_after_walk" and not any(s.get("phase") == "WALK_STANDARD" for s in samples):
                rows = []
            if not rows:
                continue
            raw = np.array([[j["raw"] for j in s["joints"][:12]] for s in rows], float)
            cmd = np.array([[j["commanded_raw"] for j in s["joints"][:12]] for s in rows], float)
            q = direction * (raw - q0) * twin.TICK
            dev = np.abs(raw - cmd)[:, HIPS]
            margin = margin_ticks(q)[:, HIPS]
            out.append({"trace": str(path), "segment": label, "samples": len(rows),
                        "reasons": sorted({f"{s.get('state')}/{s.get('reason')}" for s in rows}),
                        "hip_abs_deviation_ticks_p50_p95_max": [float(np.percentile(dev, 50)), float(np.percentile(dev, 95)), float(dev.max())],
                        "hip_margin_min_ticks": {twin.SLOT_NAMES[h]: float(margin[:, k].min()) for k, h in enumerate(HIPS)}})
    return out


def twin_closed_loop(csv_path):
    t = twin.read_csv(csv_path)
    walking = t["state"] == "WALKING"
    fb = np.stack([t[f"fb_raw{i}"].astype(int) for i in range(12)], axis=1)
    cmd = np.stack([t[f"raw{i}"].astype(int) for i in range(12)], axis=1)
    q = (fb - 2048) * twin.TICK
    dev = np.abs(fb - cmd)[walking][:, HIPS]
    margin = margin_ticks(q)[:, HIPS]
    fault = np.where(t["state"] == "FAULT")[0]
    stop = np.where(t["stop_active"] == "1")[0]
    return {"walk_hip_abs_deviation_ticks_p50_p95_max": [float(np.percentile(dev, 50)), float(np.percentile(dev, 95)), float(dev.max())],
            "walk_hip_margin_min_ticks": {twin.SLOT_NAMES[h]: float(margin[walking][:, k].min()) for k, h in enumerate(HIPS)},
            "stop_t_s": float(t["t_s"][stop[0]]) if len(stop) else None,
            "fault": [float(t["t_s"][fault[0]]), str(t["reason"][fault[0]])] if len(fault) else None,
            "hip_margin_at_fault_ticks": {twin.SLOT_NAMES[h]: float(margin[fault[0], k]) for k, h in enumerate(HIPS)} if len(fault) else None}


def variant_run(job):
    binary, out_dir, scenario, variant = job
    overrides, mu, unit = VARIANTS[variant]
    report = cosim.run(binary, pathlib.Path(out_dir), scenario, servo.Params(**{**servo.Params().__dict__, **overrides}), mu, variant, unit)
    return scenario, variant, report


def main():
    binary, cl_dir, out_dir = map(pathlib.Path, sys.argv[1:4])
    traces = sys.argv[4:]
    out_dir.mkdir(parents=True, exist_ok=True)
    result = {"hardware_real": hardware(traces),
              "twin_nominal_closed_loop": {s: twin_closed_loop(cl_dir / f"{s}.csv") for s in BOX_SCENARIOS},
              "attribution": {}}
    jobs = [(str(binary), str(out_dir), s, v) for s in BOX_SCENARIOS for v in VARIANTS]
    with multiprocessing.Pool(4) as pool:
        for scenario, variant, report in pool.imap_unordered(variant_run, jobs):
            stats = twin_closed_loop(out_dir / f"{scenario}__{variant}.csv")
            result["attribution"].setdefault(scenario, {})[variant] = {
                "final": f"{report['final_state']} ({report['final_reason']})", "fault": stats["fault"],
                "stop_verified": report["stop_verified"], "walk_hip_abs_deviation_ticks_p50_p95_max": stats["walk_hip_abs_deviation_ticks_p50_p95_max"],
                "walk_hip_margin_min_ticks": stats["walk_hip_margin_min_ticks"], "hip_margin_at_fault_ticks": stats["hip_margin_at_fault_ticks"]}
    (out_dir / "m4_hip_stop.json").write_text(json.dumps(result, indent=1) + "\n")
    print("== HW (real, dashboard samples)")
    for h in result["hardware_real"]:
        print(f"   {pathlib.Path(h['trace']).parent.name[:28]:28s} {h['segment']:16s} n={h['samples']:3d} |dev| p50/p95/max {h['hip_abs_deviation_ticks_p50_p95_max']} "
              f"margin min {h['hip_margin_min_ticks']} {h['reasons']}")
    print("== twin nominal closed loop")
    for s, v in result["twin_nominal_closed_loop"].items():
        print(f"   {s:26s} |dev| {[round(x, 1) for x in v['walk_hip_abs_deviation_ticks_p50_p95_max']]} walk margin min "
              f"{ {k: round(x, 1) for k, x in v['walk_hip_margin_min_ticks'].items()} } fault {v['fault']}")
    print("== attribution (closed loop, one change)")
    for s, vs in result["attribution"].items():
        for v, r in vs.items():
            print(f"   {s:26s} {v:15s} {r['final']:40s} stop_verified {r['stop_verified']}  |dev| {[round(x, 1) for x in r['walk_hip_abs_deviation_ticks_p50_p95_max']]}")


if __name__ == "__main__":
    main()
