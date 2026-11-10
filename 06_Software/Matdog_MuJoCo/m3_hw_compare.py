#!/usr/bin/env python3
"""Twin vs hardware on the one law walked on the ground: SLOW (WALK100), dev.4,
2026-11-08 (verification-artifacts/MATDOG_1_0_2_DEV4_GROUND_20261108; the SLOW
law of the candidate is dev.4's bit for bit). The hardware ran closed loop (IMU
correction, at its envelope); the twin replays open loop the references of the
rig with the IMU following the commanded attitude. Distributions, not paired
samples: the dashboard trace samples every ~0.45 s.

Usage: m3_hw_compare.py <dev.4 matdog-motion-trace json> <sequences dir> <out dir>
"""
import dataclasses
import json
import math
import pathlib
import sys

import numpy as np

import dynamics
import m3_replays
import servo
import twin

IDEAL = m3_replays.VARIANTS["ideal_actuator"]


def hardware(path):
    samples = [s["status"] for s in json.loads(pathlib.Path(path).read_text())["samples"]
               if s["status"].get("state") == "WALKING" and s["status"].get("phase") == "WALK_STANDARD"]
    err = np.array([[j["raw"] - j["commanded_raw"] for j in s["joints"][:12]] for s in samples], float)
    load = np.array([[-(j["load"] & 0x3FF) if j["load"] & 0x400 else j["load"] for j in s["joints"][:12]] for s in samples], float)
    rp = np.array([dynamics.quat_rpy(s["telemetry"]["quaternion"]) for s in samples])
    return {"samples": len(samples), "abs_err": np.abs(err), "abs_duty": np.abs(load), "roll": 1e3 * rp[:, 0], "pitch": 1e3 * rp[:, 1],
            "applied_pitch_mrad": [min(s["imu_correction"]["applied_mrad"][1] for s in samples),
                                   max(s["imu_correction"]["applied_mrad"][1] for s in samples)]}


def simulated(seq, overrides):
    params = dataclasses.replace(servo.Params(), **overrides)
    tw = dynamics.Twin(params)
    rec = tw.run(seq)
    steady = np.where((seq["state"] == "WALKING") & (seq["t"] >= 0))[0]
    at = np.minimum(np.searchsorted(rec["t"], seq["t"][steady] - 1e-9), len(rec["t"]) - 1)
    rp = np.array([dynamics.quat_rpy(q) for q in rec["quat"][at]])
    report = dynamics.metrics(seq, rec, params)["segments"]["walk_SLOW"]
    return {"abs_err": np.abs(rec["release_end_q"][steady] - seq["q"][steady]) / servo.TICK,
            "abs_duty": 1e3 * np.abs(rec["duty"][at]), "roll": 1e3 * rp[:, 0], "pitch": 1e3 * rp[:, 1],
            "travel": dynamics.metrics(seq, rec, params).get("walk_travel"), "segment": report}


def stats(a):
    return [float(np.percentile(a, 50)), float(np.percentile(a, 95)), float(np.max(a))]


def main():
    trace, seq_dir, out_dir = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2]), pathlib.Path(sys.argv[3])
    out_dir.mkdir(parents=True, exist_ok=True)
    hw = hardware(trace)
    seq = twin.sequence(seq_dir / "imufollows_walk_slow.csv")
    runs = {"hardware dev.4": hw}
    for name, overrides in (("twin nominal", {}), ("twin bench_alt", m3_replays.VARIANTS["bench_alt"]),
                            ("twin p_only", m3_replays.VARIANTS["p_only"]), ("twin ideal actuator", IDEAL)):
        runs[name] = simulated(seq, overrides)
        print(f"{name} done", flush=True)
    groups = {"HIP": [0, 3, 6, 9], "UPPER": [1, 4, 7, 10], "LOWER": [2, 5, 8, 11]}
    table = {}
    for name, r in runs.items():
        row = {"roll_p2p_mrad": float(np.ptp(r["roll"])), "pitch_p2p_mrad": float(np.ptp(r["pitch"]))}
        for g, idx in groups.items():
            row[f"{g}_abs_err_p50_p95_max_ticks"] = stats(r["abs_err"][:, idx])
            row[f"{g}_abs_duty_p50_p95_max_permille"] = stats(r["abs_duty"][:, idx])
        if "travel" in r:
            row["travel"] = r["travel"]
            row["swing_foot_dragging_pct"] = r["segment"]["swing_foot_dragging_pct"]
        table[name] = row
    print(f"{'':22s} {'roll p-p':>9s} {'pitch p-p':>9s} | " + " | ".join(f"{g} |err| p50/p95/max  |duty| p50/p95/max" for g in groups))
    for name, row in table.items():
        cells = " | ".join("{:4.0f}/{:4.0f}/{:4.0f}  {:4.0f}/{:4.0f}/{:4.0f}".format(*row[f"{g}_abs_err_p50_p95_max_ticks"], *row[f"{g}_abs_duty_p50_p95_max_permille"]) for g in groups)
        print(f"{name:22s} {row['roll_p2p_mrad']:9.1f} {row['pitch_p2p_mrad']:9.1f} | {cells}")
    result = {"hardware": {"source": str(trace), "samples": hw["samples"], "applied_pitch_correction_mrad": hw["applied_pitch_mrad"],
                           "note": "closed loop on the robot; dashboard samples ~0.45 s; load = PresentLoad"},
              "twin_sequence": "imufollows_walk_slow (open loop, IMU following the commanded attitude)", "table": table}
    (out_dir / "m3_hw_compare.json").write_text(json.dumps(result, indent=1, default=float) + "\n")


if __name__ == "__main__":
    main()
