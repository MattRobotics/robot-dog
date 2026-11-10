#!/usr/bin/env python3
"""Ground STOP RCA: the closed-loop STOPs during WALK (cosim.py), release by release.

Usage: rca_ground_stop.py <cl dir> <out dir> [scenario ...]

Each recorded run is replayed with the nominal twin (the frames of cosim.py, checked release
by release against the recorded feedback) to recover what the record lacks: foot forces, COM,
attitude, duty. Around STOP and fault it reports, in ticks and permille:
- per joint, the margin to the STOP box (kHeight for a STOP from WALK; > 0 inside), measured
  and commanded, and when the measured one first went outside;
- error to the reference, duty, foot forces and heights, COM in the support polygon, roll/pitch;
- for STOP_NOT_SETTLED, generateStop's settle criteria per joint after the plan end.
Values are the twin's at each release boundary; SimpleMotion reads two servos per release, so
its view of a joint can lag up to 120 ms. Reads only: the firmware check is unchanged.
"""
import json
import multiprocessing
import pathlib
import sys

import mujoco
import numpy as np

import dynamics
import m4_hip_stop
import servo
import twin

SCENARIOS = ("cl_stop_front_slow", "cl_stop_front_standard", "cl_stop_front_fast", "cl_stop_slow_to_standard",
             "cl_stop_lateswing_slow", "cl_stop_lateswing_standard", "cl_stop_lateswing_fast")
LOADED_TICKS, MAX_PERMILLE, MAX_SPEED, CREEP_TICKS, DWELL_S = 48, 400, 50, 1, 0.5  # SimpleMotion.cpp:27,43,303-310


def replay(path):
    """The nominal twin, or the m4_hip_stop variant named by a `__<variant>` suffix."""
    table = twin.sequence(path)
    fb = np.stack([table[f"fb_raw{i}"].astype(int) for i in range(12)], axis=1)
    variant = pathlib.Path(path).stem.partition("__")[2]
    overrides, mu, unit = m4_hip_stop.VARIANTS[variant] if variant else ({}, 0.8, None)
    tw = dynamics.Twin(servo.Params(**{**servo.Params().__dict__, **overrides}), mu=mu, unit=unit)
    tw.begin((fb[0] - 2048) * twin.TICK, 480, 13)
    root = tw.m.body("base_link").id
    s = {k: [] for k in ("q", "base", "fz", "com", "feet", "rpy", "duty", "force")}
    same = 0
    for k in range(len(fb)):  # state at the start of release k = the feedback of release k
        same += int(np.array_equal(2048 + np.rint(tw.d.qpos[7:19] / twin.TICK).astype(int), fb[k]))
        mujoco.mj_rnePostConstraint(tw.m, tw.d)
        s["q"].append(tw.d.qpos[7:19].copy())
        s["base"].append(tw.d.qpos[:7].copy())
        s["fz"].append(tw.d.cfrc_ext[tw.foot_bodies, 5].copy())
        s["com"].append(tw.d.subtree_com[root].copy())
        s["feet"].append(twin.contact_reference(tw.d, tw.centers, tw.feet))
        s["rpy"].append(dynamics.quat_rpy(tw.d.qpos[3:7]))
        s["duty"].append(1e3 * tw.servos.duty(tw.d.qpos[7:19]))
        s["force"].append(foot_forces(tw))
        tw.release(table["q"][k], int(table["drive_speed_step_s"][k]), int(table["drive_acc_100step_s2"][k]),
                   np.full(12, table["torque_on"][k] == "12"), record=False)
    return table, {k: np.array(v) for k, v in s.items()}, same, tw.m


def foot_forces(tw):
    """Ground force on each foot (N, world frame), from the contact solver."""
    out, f6 = np.zeros((4, 3)), np.zeros(6)
    for c in range(tw.d.ncon):
        con = tw.d.contact[c]
        for l, g in enumerate(tw.feet):
            if g in (con.geom1, con.geom2):
                mujoco.mj_contactForce(tw.m, tw.d, c, f6)
                world = con.frame.reshape(3, 3).T @ f6[:3]  # force of geom1 on geom2
                out[l] += world if con.geom2 == g else -world
    return out


def commanded_feet(model, base, q):
    """Foot contact points of the commanded joints placed on the actual body pose."""
    d = mujoco.MjData(model)
    d.qpos[:7], d.qpos[7:19] = base, q
    mujoco.mj_kinematics(model, d)
    return twin.contact_reference(d, twin.site_ids(model, "foot_cylinder_center"),
                                  np.array([model.geom(f"{l}_foot_cylinder").id for l in twin.LEGS]))


def support(s, k):
    on = s["fz"][k] > dynamics.CONTACT_N
    return 1e3 * dynamics.support_margin(s["feet"][k][on], s["com"][k][:2]), "".join(l[0].upper() + l[1].upper() for l, c in zip(twin.LEGS, on) if c)


def settle(table, s, k0, k1):
    """generateStop's settle criteria per joint over releases k0..k1 (true values, every release)."""
    goal = table["q"][k1]
    err = np.abs((np.stack([table[f"fb_raw{i}"].astype(int) for i in range(12)], axis=1)[k0:k1 + 1] - 2048) * twin.TICK - goal) / twin.TICK
    load = np.abs(np.stack([table[f"fb_load{i}"].astype(int) for i in range(12)], axis=1)[k0:k1 + 1])
    speed = np.abs(np.stack([table[f"fb_speed{i}"].astype(int) for i in range(12)], axis=1)[k0:k1 + 1])
    raw = np.stack([table[f"fb_raw{i}"].astype(int) for i in range(12)], axis=1)[k0:k1 + 1]
    fail = {"error>48": err > LOADED_TICKS, "load>400": load > MAX_PERMILLE, "speed>50": speed > MAX_SPEED}
    resting = ~(fail["error>48"] | fail["load>400"] | fail["speed>50"]).any(axis=1)
    best, start, creep = 0, None, np.zeros(12, int)
    for j in range(len(raw)):  # the dwell: every RAW within 1 tick of its value at the dwell start
        if not resting[j]:
            start = None
            continue
        if start is None:
            start = j
        moved = np.abs(raw[j] - raw[start]) > CREEP_TICKS
        if moved.any():
            creep += moved
            start = j
        best = max(best, j - start)
    return {"releases": int(k1 - k0 + 1),
            "fail_pct_by_joint": {c: {twin.SLOT_NAMES[i]: round(100 * float(m[:, i].mean()), 1) for i in range(12) if m[:, i].any()} for c, m in fail.items()},
            "resting_pct": round(100 * float(resting.mean()), 1),
            "creep_resets_by_joint": {twin.SLOT_NAMES[i]: int(creep[i]) for i in range(12) if creep[i]},
            "longest_dwell_s": round(best * twin.RELEASE_S, 2), "dwell_needed_s": DWELL_S,
            "load_max_permille": {twin.SLOT_NAMES[i]: int(load[:, i].max()) for i in np.argsort(-load.max(axis=0))[:3]},
            "error_max_ticks": {twin.SLOT_NAMES[i]: round(float(err[:, i].max()), 1) for i in np.argsort(-err.max(axis=0))[:3]},
            "raw_drift_ticks_last_1s": {twin.SLOT_NAMES[i]: int(d) for i, d in enumerate(raw[-1] - raw[max(0, len(raw) - 51)]) if abs(d) > 1}}


def walk_stop(table, s, same, model):
    """The gait's own stop (walkStop): WALK_STOPPING, WALK_SETTLE, then the STAND it reaches."""
    t, reason, state = table["t"].astype(float), table["reason"], table["state"]
    lo, hi = dynamics.box("kHeight")
    margin = np.minimum(s["q"] - lo, hi - s["q"]) / twin.TICK
    k0 = int(np.flatnonzero(reason == "WALK_STOPPING")[0])
    reached = np.flatnonzero((state == "STANDING") & (np.arange(len(t)) > k0))
    k1 = int(reached[0]) if len(reached) else len(t) - 1
    load = np.abs(np.stack([table[f"fb_load{i}"].astype(int) for i in range(12)], axis=1))
    err = np.abs(s["q"][k1] - table["q"][k1]) / twin.TICK
    m, feet = support(s, len(t) - 1)
    hips = (0, 3, 6, 9)
    return {"replay_identical_releases": f"{same}/{len(t)}", "end": f"{state[-1]} {reason[-1]}", "walkstop_t_s": float(t[k0]),
            "standing_after_s": round(float(t[k1] - t[k0]), 2), "faults": sorted(set(reason[state == "FAULT"])),
            "hip_margin_min_ticks_stopping_to_standing": {twin.SLOT_NAMES[i]: round(float(margin[k0:k1 + 1, i].min()), 1) for i in hips},
            "hip_outside_box_pct_stopping_to_standing": {twin.SLOT_NAMES[i]: round(100 * float((margin[k0:k1 + 1, i] < 0).mean()), 1) for i in hips if (margin[k0:k1 + 1, i] < 0).any()},
            "at_standing_load_max_permille": {twin.SLOT_NAMES[i]: int(load[k1, i]) for i in np.argsort(-load[k1])[:3]},
            "at_standing_error_max_ticks": {twin.SLOT_NAMES[i]: round(float(err[i]), 1) for i in np.argsort(-err)[:3]},
            "end_feet_in_contact": feet, "end_com_support_margin_mm": round(m, 1),
            "end_roll_pitch_mrad": [round(1e3 * float(a), 1) for a in s["rpy"][-1]]}


def analyse(job):
    name, path = job
    table, s, same, model = replay(path)
    if "WALK_STOPPING" in set(table["reason"]) and not (table["stop_active"] == "1").any():
        return name, walk_stop(table, s, same, model)
    t, state = table["t"].astype(float), table["state"]
    lo, hi = dynamics.box("kHeight")
    margin = np.minimum(s["q"] - lo, hi - s["q"]) / twin.TICK
    cmd_margin = np.minimum(table["q"] - lo, hi - table["q"]) / twin.TICK
    k_stop = int(np.flatnonzero(table["stop_active"] == "1")[0])
    fault = np.flatnonzero(state == "FAULT")
    k_end = int(fault[0]) if len(fault) else len(t) - 1
    walking = np.flatnonzero(state[:k_stop] == "WALKING")
    stance = [int(table[f"stance{l}"][k_stop - 1]) for l in range(4)]
    out = {"replay_identical_releases": f"{same}/{len(t)}",
           "mode": f"{table['law_mode'][k_stop - 1]} vx {float(table['vx_m_s'][k_stop - 1]):.3f} m/s drive {table['drive_speed_step_s'][k_stop - 1]}/{table['drive_acc_100step_s2'][k_stop - 1]}",
           "swing_legs_at_stop": [twin.LEGS[l].upper() for l in range(4) if not stance[l]],
           "t_stop_s": float(t[k_stop]), "stop_plan_s": float(table["stop_plan_s"][k_stop]),
           "end": f"{state[k_end]} {table['reason'][k_end]}", "t_end_s": float(t[k_end]), "end_after_stop_s": round(float(t[k_end] - t[k_stop]), 2)}
    # the STOP box, before and after the STOP request
    out["walk_outside_box_pct_by_joint"] = {twin.SLOT_NAMES[i]: round(100 * float((margin[walking, i] < 0).mean()), 1) for i in range(12) if (margin[walking, i] < 0).any()}
    out["walk_margin_min_ticks_by_joint"] = {twin.SLOT_NAMES[i]: round(float(margin[walking, i].min()), 1) for i in range(12) if margin[walking, i].min() < 8}
    out["walk_cmd_margin_min_ticks"] = round(float(cmd_margin[walking].min()), 1)
    out["at_stop_margin_ticks"] = {twin.SLOT_NAMES[i]: round(float(margin[k_stop, i]), 1) for i in range(12) if margin[k_stop, i] < 8}
    after = margin[k_stop:k_end + 1]
    out["stop_margin_min_ticks"] = {twin.SLOT_NAMES[i]: round(float(after[:, i].min()), 1) for i in range(12) if after[:, i].min() < 4}
    out["stop_cmd_margin_min_ticks"] = round(float(cmd_margin[k_stop:k_end + 1].min()), 1)
    outside = np.flatnonzero((after < 0).any(axis=0))
    out["stop_first_outside_s_after_stop"] = {twin.SLOT_NAMES[i]: round(float(t[k_stop + np.flatnonzero(after[:, i] < 0)[0]] - t[k_stop]), 2) for i in outside}
    if "OUTSIDE_BOX" in out["end"]:
        window = margin[max(k_stop, k_end - 5):k_end + 1]  # the last read of every joint is within 6 releases
        culprits = np.flatnonzero((window < 0).any(axis=0))
        out["fault_joints"] = {}
        for i in culprits:
            first = k_end
            while first > 0 and margin[first - 1, i] < 0:
                first -= 1
            out["fault_joints"][twin.SLOT_NAMES[i]] = {
                "margin_min_last_6_releases_ticks": round(float(window[:, i].min()), 1),
                "cmd_margin_at_fault_ticks": round(float(cmd_margin[k_end - 1, i]), 1),
                "error_to_reference_ticks": round(float((s["q"][k_end, i] - table["q"][k_end - 1, i]) / twin.TICK), 1),
                "duty_permille": round(float(s["duty"][k_end, i])),
                "outside_continuously_since_s_after_stop": round(float(t[first] - t[k_stop]), 2),
                "outside_pct_of_walk": out["walk_outside_box_pct_by_joint"].get(twin.SLOT_NAMES[i], 0.0)}
    # body: attitude, COM, contacts, actuator
    rpy = 1e3 * s["rpy"]
    for label, k in (("at_stop", k_stop), ("at_end", k_end)):
        m, feet = support(s, k)
        out[f"{label}_body"] = {"roll_pitch_mrad": [round(float(rpy[k, 0]), 1), round(float(rpy[k, 1]), 1)],
                                "imu_correction_applied_mrad": [round(1e3 * float(table["imu_roll"][k]), 1), round(1e3 * float(table["imu_pitch"][k]), 1)],
                                "feet_in_contact": feet, "fz_n": [round(float(f), 1) for f in s["fz"][k]],
                                "foot_height_mm": [round(1e3 * float(p[2]), 1) for p in s["feet"][k]],
                                "com_support_margin_mm": round(m, 1), "com_z_mm": round(1e3 * float(s["com"][k, 2]), 1)}
    sm = [support(s, k)[0] for k in range(k_stop, k_end + 1)]
    out["stop_com_support_margin_min_mm"] = round(float(min(sm)), 1)
    out["stop_roll_pitch_range_mrad"] = [[round(float(rpy[k_stop:k_end + 1, a].min()), 1), round(float(rpy[k_stop:k_end + 1, a].max()), 1)] for a in (0, 1)]
    out["stop_duty_max_permille"] = {twin.SLOT_NAMES[i]: round(float(np.abs(s["duty"][k_stop:k_end + 1, i]).max())) for i in np.argsort(-np.abs(s["duty"][k_stop:k_end + 1]).max(axis=0))[:3]}
    if "NOT_SETTLED" in out["end"]:
        k_plan = k_stop + int(np.ceil(out["stop_plan_s"] / twin.RELEASE_S))
        out["settle_after_plan_end"] = settle(table, s, k_plan, k_end)
        k = k_end - 1  # the rest that never verified: where each foot is held, and where it is commanded
        f = s["force"][k]
        cmd = commanded_feet(model, s["base"][k], table["q"][k])
        out["end_foot_force_n"] = {twin.LEGS[l].upper(): {"fx_fy_fz": [round(float(v), 2) for v in f[l]],
                                                          "tangential_over_normal": round(float(np.hypot(*f[l, :2]) / max(f[l, 2], 1e-9)), 2),
                                                          "commanded_minus_actual_foot_mm": {"horizontal": round(1e3 * float(np.hypot(*(cmd[l, :2] - s["feet"][k][l, :2]))), 1),
                                                                                             "vertical": round(1e3 * float(cmd[l, 2] - s["feet"][k][l, 2]), 1)}} for l in range(4)}
    return name, out


def main():
    cl_dir, out_dir = map(pathlib.Path, sys.argv[1:3])
    out_dir.mkdir(parents=True, exist_ok=True)
    jobs = [(n, str(cl_dir / f"{n}.csv")) for n in (sys.argv[3:] or SCENARIOS)]
    with multiprocessing.Pool(4) as pool:
        result = dict(pool.map(analyse, jobs))
    (out_dir / "rca_ground_stop.json").write_text(json.dumps(result, indent=1) + "\n")
    for name, r in result.items():
        if "walkstop_t_s" in r:
            print(f"== {name}: {r}")
            continue
        print(f"== {name}: {r['mode']}, swing {r['swing_legs_at_stop']}, STOP {r['t_stop_s']:.2f} s (plan {r['stop_plan_s']:.2f} s) -> {r['end']} +{r['end_after_stop_s']} s  [{r['replay_identical_releases']}]")
        for key in ("walk_margin_min_ticks_by_joint", "walk_outside_box_pct_by_joint", "walk_cmd_margin_min_ticks", "at_stop_margin_ticks", "stop_margin_min_ticks",
                    "stop_cmd_margin_min_ticks", "stop_first_outside_s_after_stop", "fault_joints", "at_stop_body", "at_end_body", "stop_com_support_margin_min_mm",
                    "stop_roll_pitch_range_mrad", "stop_duty_max_permille", "settle_after_plan_end", "end_foot_force_n"):
            if key in r:
                print(f"   {key}: {r[key]}")


if __name__ == "__main__":
    main()
