#!/usr/bin/env python3
"""FAST 40 RCA on the twin: STOPs requested inside the front swing, where the offline StopPlan proof is tight.

One forward FAST walk at the law's top speed (corrected twin dev7_ground, so the IMU correction rides the
envelope as on the dev.7 robot); the leg phases of that walk give the instants at which LF is at swing
progress u (default .60 .65 .70 .75); one STOP is then walked at each instant. Per STOP it writes, release by
release from the request to verification, for the front UPPERs and the hind HIPs: commanded and measured
angle, the active StopPlan end point (the bridge records it while a STOP is active), the kHeight bounds and
the margins (ticks; negative = outside), plus the runtime verdict. Before the request it also records the
tracking error (measured - commanded) of the swinging front UPPER over the last 0.2 s.

Usage: twin_stop_rca.py <cosim> <fast law period:stride_mm:height_mm> <out.json> [u ...]
"""
import json
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))  # Matdog_MuJoCo

import cosim
import dynamics
import m6_dev7
import m6_gait
import servo
import twin

JOINTS = {"LF_UPPER": 1, "RF_UPPER": 4, "RH_HIP": 6, "LH_HIP": 9}


def walk_rows(binary, events, duration, v, height):
    import dataclasses
    params = dataclasses.replace(servo.Params(), **v.get("params", {}))
    imu = tuple(1e-3 * x for x in v["imu_offset_mrad"])
    rows, tw, replies, _ = cosim.simulate(binary, [(0.5, "SELECT FAST")] + events, duration, params, v.get("mu", 0.8), v.get("unit"), height, None, imu, v.get("model"))
    return rows, replies


def main():
    binary, law, out = sys.argv[1], sys.argv[2], sys.argv[3]
    us = [float(x) for x in sys.argv[4:]] or [.60, .65, .70, .75]
    per, stride_mm, height_mm = (float(x) for x in law.split(":"))
    m6_gait.FAST_LAW.update(period=per, stride=stride_mm * 1e-3, height=height_mm * 1e-3)
    sc = m6_gait.fast_scenarios(m6_gait.FAST_LAW)
    _, v = m6_dev7.variant_params("dev7_ground", [])
    lo, hi = dynamics.box("kHeight")
    drive = sc["fast_fwd"][2][0][1]
    # 1) the walk alone: LF swing windows (stance0 == 0) after the start ramp
    rows, _ = walk_rows(binary, m6_gait.held(1.0, 20.0, drive), 20.0, v, height_mm * 1e-3)
    t = np.array([float(r["t_s"]) for r in rows])
    sw = np.array([r["stance0"] == "0" for r in rows])
    starts = [i for i in range(1, len(rows)) if sw[i] and not sw[i - 1] and t[i] > 12.0]
    i0 = starts[0]
    i1 = i0 + int(np.argmax(~sw[i0:]))
    t0, t1 = t[i0], t[i1]
    result = {"law": law, "lf_swing_s": [t0, t1], "stops": []}
    for u in us:
        ts = round((t0 + u * (t1 - t0)) / twin.RELEASE_S) * twin.RELEASE_S
        rows, replies = walk_rows(binary, m6_gait.held(1.0, ts, drive) + [(ts, "STOP")], ts + 4.0, v, height_mm * 1e-3)
        tt = np.array([float(r["t_s"]) for r in rows])
        k0 = int(np.searchsorted(tt, ts))
        cmd = (np.array([[int(r[f"raw{j}"]) for j in range(12)] for r in rows]) - 2048) * twin.TICK
        fb = (np.array([[int(r[f"fb_raw{j}"]) for j in range(12)] for r in rows]) - 2048) * twin.TICK
        active = np.array([r["stop_active"] == "1" for r in rows])
        ends = np.array([[float(r[f"stop_end{j}"]) for j in range(12)] for r in rows])
        ka = [k for k in range(k0, len(rows)) if active[k]]
        rec = {"u_request": u, "t_request": ts, "states": sorted({(r["state"], r["reason"]) for r in rows[k0:]}),
               "verified": any(r["stop_verified"] == "1" for r in rows[k0:]),
               "imu_pitch_mrad_at_request": 1e3 * float(rows[k0]["imu_pitch"]), "imu_reason_at_request": rows[k0]["imu_reason"],
               "plan_s": float(rows[ka[0]]["stop_plan_s"]) if ka else None, "joints": {}}
        pre = slice(max(0, k0 - 10), k0)
        for name, j in JOINTS.items():
            m = lambda q: float(min(q - lo[j], hi[j] - q) / twin.TICK)
            rec["joints"][name] = {
                "cmd_margin_at_request": m(cmd[k0, j]), "meas_margin_at_request": m(fb[k0, j]),
                "tracking_ticks_before_request": float(np.max(np.abs(fb[pre, j] - cmd[pre, j])) / twin.TICK),
                "plan_end_margin": m(ends[ka[0], j]) if ka else None,
                "min_cmd_margin_during_stop": min(m(q) for q in cmd[k0:, j]),
                "min_meas_margin_during_stop": min(m(q) for q in fb[k0:, j]),
                "meas_minus_cmd_at_request_ticks": float((fb[k0, j] - cmd[k0, j]) / twin.TICK)}
        result["stops"].append(rec)
        print(f"u {u:.2f} t {ts:.2f}: verified {rec['verified']} imu pitch {rec['imu_pitch_mrad_at_request']:.1f} mrad ({rec['imu_reason_at_request']}) plan {rec['plan_s']} s | "
              + " | ".join(f"{n} cmd {d['cmd_margin_at_request']:.0f} meas {d['meas_margin_at_request']:.0f} end {d['plan_end_margin'] if d['plan_end_margin'] is None else round(d['plan_end_margin'], 1)} "
                           f"min meas {d['min_meas_margin_during_stop']:.1f}" for n, d in rec["joints"].items()), flush=True)
    json.dump(result, open(out, "w"), indent=1)


if __name__ == "__main__":
    main()
