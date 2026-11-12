#!/usr/bin/env python3
"""FAST 40 study, priority 2 (2026-11-13): why the twin's swing clearance is far below the law's lift.

One straight walk of a build at the FAST law's top speed (corrected twin dev7_ground, as m6_gait). For
every swing in the steady window and every 1 ms twin sample, the height above the floor of the foot's
contact reference (the lowest point of the tread cylinder, twin.contact_reference = firmware referenceM)
in five versions, each one change away from the previous:
  law   : the law's foot target z (bridge foot<l>_z: 64 lift u^3 (1-u)^3);
  nom   : MuJoCo FK of the COMMANDED joints (RAW of the release) on the law's body: base at the law's
          height, level (the commanded joints carry the IMU roll/pitch correction, so nom - law is the
          correction plus any FK/law mismatch);
  cmd   : the same commanded joints on the twin's SIMULATED base pose (cmd - nom: body height, pitch, roll);
  act   : the twin's real foot (cmd -> act: servo tracking, with the floor's reaction while in contact).
Per swing: the peak of each version and their values at the law's peak instant (u = .5); the base height,
pitch and roll at u = .5 against the law; the foot's normal force (share of the swing above DRAG_N, the
last loaded sample after lift-off, the first before touchdown); the per-joint tracking error at u = .5.

Usage: foot_lift_rca.py <cosim> <fast law period:stride_mm:height_mm> <out.json> [--set key=value ...]
       (--set as m6_gait/m6_dev7: imu_pitch_mrad=0 removes the IMU mount offset, a diagnostic only)
"""
import dataclasses
import json
import pathlib
import sys

import mujoco
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))  # Matdog_MuJoCo

import cosim
import dynamics
import m6_dev7
import m6_gait
import servo
import twin

LEGS = ("LF", "RF", "RH", "LH")


def rpy(q):
    return np.array(dynamics.quat_rpy(q))


def main():
    binary, law, out = sys.argv[1], sys.argv[2], sys.argv[3]
    sets = [sys.argv[i + 1] for i, x in enumerate(sys.argv) if x == "--set"]
    per, stride_mm, height_mm = (float(x) for x in law.split(":"))
    m6_gait.FAST_LAW.update(period=per, stride=stride_mm * 1e-3, height=height_mm * 1e-3)
    sc = m6_gait.fast_scenarios(m6_gait.FAST_LAW)
    mode, height, events, duration, window = sc["fast_fwd"]
    _, v = m6_dev7.variant_params("dev7_ground", sets)
    params = dataclasses.replace(servo.Params(), **v.get("params", {}))
    imu = tuple(1e-3 * x for x in v["imu_offset_mrad"])
    rows, tw, replies, _ = cosim.simulate(binary, [(0.5, "SELECT FAST")] + events, duration, params, v.get("mu", 0.8), v.get("unit"), height, None, imu, v.get("model"))
    rec = tw.result()
    t = rec["t"]
    n = len(rows)
    idx = np.clip(np.ceil(t / twin.RELEASE_S - 1e-9).astype(int) - 1, 0, n - 1)  # release whose frame is in force
    stance = np.array([[int(r[f"stance{l}"]) for l in range(4)] for r in rows])[idx]
    raw = np.array([[int(r[f"raw{j}"]) for j in range(12)] for r in rows])
    qcmd = (raw - 2048) * twin.TICK
    law_feet = np.array([[[float(r[f"foot{l}_{a}"]) for a in "xyz"] for l in range(4)] for r in rows])
    body_z = np.array([float(r["body_z"]) for r in rows])
    corr = np.array([[float(r["imu_roll"]), float(r["imu_pitch"])] for r in rows])
    a, b = window
    m = (t >= a) & (t <= b) & (stance >= 0).all(axis=1)
    # FK on a scratch copy of the twin's model
    dk = mujoco.MjData(tw.m)

    def fk(pos, quat, q):
        dk.qpos[0:3], dk.qpos[3:7], dk.qpos[7:19] = pos, quat, q
        mujoco.mj_kinematics(tw.m, dk)
        return twin.contact_reference(dk, tw.centers, tw.feet)[:, 2]

    level = np.array([1.0, 0, 0, 0])
    swings = []
    stance_nom = []  # nom z of stance feet: checks that the FK and the law share the floor
    for l in range(4):
        sw = (stance[:, l] == 0) & m
        e = np.flatnonzero(np.diff(np.r_[0, sw.astype(int), 0]))
        for i0, i1 in zip(e[::2], e[1::2]):
            if i1 - i0 < 50:
                continue
            seg = np.arange(i0, i1)
            k = idx[seg]
            z_law = law_feet[k, l, 2]
            z_nom = np.array([fk([0, 0, body_z[kk]], level, qcmd[kk])[l] for kk in k])
            z_cmd = np.array([fk(rec["pos"][i], rec["quat"][i], qcmd[kk])[l] for i, kk in zip(seg, k)])
            z_act = rec["feet"][seg, l, 2]
            fz = rec["fz"][seg, l]
            j = int(np.argmax(z_law))  # the law's peak instant (u = .5)
            i_mid = seg[j]
            loaded = fz > dynamics.CONTACT_N
            free = np.flatnonzero(~loaded)
            ang = rpy(rec["quat"][i_mid]) * 1e3
            hip, upp, low = 3 * l, 3 * l + 1, 3 * l + 2
            swings.append({
                "leg": LEGS[l], "t0": float(t[i0]), "t1": float(t[i1 - 1]), "duration_s": float(t[i1 - 1] - t[i0]),
                "peak_mm": {"law": 1e3 * float(z_law.max()), "nom": 1e3 * float(z_nom.max()), "cmd": 1e3 * float(z_cmd.max()), "act": 1e3 * float(z_act.max())},
                "at_law_peak_mm": {"law": 1e3 * float(z_law[j]), "nom": 1e3 * float(z_nom[j]), "cmd": 1e3 * float(z_cmd[j]), "act": 1e3 * float(z_act[j])},
                "u_of_act_peak": float(np.argmax(z_act) / max(1, len(seg) - 1)),
                "base_at_law_peak": {"z_mm_minus_law": 1e3 * float(rec["pos"][i_mid][2] - body_z[idx[i_mid]]), "roll_mrad": float(ang[0]), "pitch_mrad": float(ang[1]),
                                     "imu_correction_roll_pitch_mrad": (1e3 * corr[idx[i_mid]]).tolist()},
                "contact": {"drag_share": float(np.mean(fz > dynamics.DRAG_N)), "loaded_share": float(np.mean(loaded)),
                            "first_free_s": float(t[seg[free[0]]] - t[i0]) if len(free) else None,
                            "last_free_s": float(t[i1 - 1] - t[seg[free[-1]]]) if len(free) else None, "fz_max_n": float(fz.max())},
                "tracking_at_law_peak_ticks": {"hip": float((rec["q"][i_mid][hip] - qcmd[idx[i_mid]][hip]) / twin.TICK),
                                               "upper": float((rec["q"][i_mid][upp] - qcmd[idx[i_mid]][upp]) / twin.TICK),
                                               "lower": float((rec["q"][i_mid][low] - qcmd[idx[i_mid]][low]) / twin.TICK)}})
        st = (stance[:, l] == 1) & m
        for i in np.flatnonzero(st)[::97]:
            stance_nom.append(1e3 * float(fk([0, 0, body_z[idx[i]]], level, qcmd[idx[i]])[l]))
    walk = [r for r in rows if r["state"] == "WALKING"]
    result = {"law": law, "sets": sets, "imu_offset_mrad": list(v["imu_offset_mrad"]), "window_s": [a, b], "swings": swings,
              "stance_nom_z_mm_p05_p50_p95": np.percentile(stance_nom, [5, 50, 95]).tolist() if stance_nom else None,
              "faults": [(r["t_s"], r["reason"]) for r in rows if r["state"] == "FAULT"][:1], "walking_releases": len(walk)}
    json.dump(result, open(out, "w"), indent=1)
    med = lambda f: float(np.median([f(s) for s in swings])) if swings else float("nan")
    print(f"{law} sets {sets}: swings {len(swings)} | peak law {med(lambda s: s['peak_mm']['law']):.2f} nom {med(lambda s: s['peak_mm']['nom']):.2f} "
          f"cmd {med(lambda s: s['peak_mm']['cmd']):.2f} act {med(lambda s: s['peak_mm']['act']):.2f} mm | at u=.5: nom {med(lambda s: s['at_law_peak_mm']['nom']):.2f} "
          f"cmd {med(lambda s: s['at_law_peak_mm']['cmd']):.2f} act {med(lambda s: s['at_law_peak_mm']['act']):.2f} | drag {med(lambda s: s['contact']['drag_share']):.2f} "
          f"loaded {med(lambda s: s['contact']['loaded_share']):.2f} | stance nom z p50 {result['stance_nom_z_mm_p05_p50_p95'][1] if stance_nom else float('nan'):.2f} mm", flush=True)
    for leg in LEGS:
        S = [s for s in swings if s["leg"] == leg]
        if S:
            print(f"  {leg}: peak law {np.median([s['peak_mm']['law'] for s in S]):.2f} nom {np.median([s['peak_mm']['nom'] for s in S]):.2f} cmd {np.median([s['peak_mm']['cmd'] for s in S]):.2f} "
                  f"act {np.median([s['peak_mm']['act'] for s in S]):.2f} | base dz {np.median([s['base_at_law_peak']['z_mm_minus_law'] for s in S]):.2f} mm pitch {np.median([s['base_at_law_peak']['pitch_mrad'] for s in S]):.1f} "
                  f"roll {np.median([s['base_at_law_peak']['roll_mrad'] for s in S]):.1f} corr {np.median([s['base_at_law_peak']['imu_correction_roll_pitch_mrad'][1] for s in S]):.1f} mrad | "
                  f"track U {np.median([s['tracking_at_law_peak_ticks']['upper'] for s in S]):.0f} L {np.median([s['tracking_at_law_peak_ticks']['lower'] for s in S]):.0f} H {np.median([s['tracking_at_law_peak_ticks']['hip'] for s in S]):.0f} | "
                  f"loaded {np.median([s['contact']['loaded_share'] for s in S]):.2f} drag {np.median([s['contact']['drag_share'] for s in S]):.2f}", flush=True)


if __name__ == "__main__":
    main()
