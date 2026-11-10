#!/usr/bin/env python3
"""Milestone 3: physics replays of SLOW, STANDARD, FAST, mode changes, joystick
and STOP, with the servo model, and their sensitivity to the unmeasured
parameters. Simulation only; never a hardware result.

Usage: m3_replays.py <sequences dir> <out dir> [suite ...]
Suites: walks, stops, hip, sensitivity, lateswing (default: the first four). One JSON per run in
<out>/runs/, a summary in <out>/m3_<suite>.json.
"""
import dataclasses
import json
import pathlib
import sys
import time

import numpy as np

import dynamics
import servo
import twin

NOMINAL = servo.Params()
VARIANTS = {  # one change at a time from NOMINAL; status of each in servo.PARAMS / the report
    "nominal": {},
    "torque_limit_500": {"torque_limit": 0.5},
    "volt_9.6": {"volt": 9.6},
    "bench_alt": {"ff_frac": 0.60, "tau_m_s": 0.015, "friction_frac": 0.03},
    "p_only": {"ff_frac": 0.0},
    "tau_m_10ms": {"tau_m_s": 0.010},
    "tau_m_35ms": {"tau_m_s": 0.035},
    "friction_0.08": {"friction_frac": 0.08},
    "stall_x0.7": {"stall_torque_nm_12v": 0.7 * NOMINAL.stall_torque_nm_12v},
    "brake_9231": {"brake_cap_step_s2": 9231.0},
    "brake_8400": {"brake_cap_step_s2": 8400.0},
    "brake_6000": {"brake_cap_step_s2": 6000.0},
    "brake_4000": {"brake_cap_step_s2": 4000.0},
    "latency_3ms": {"latency_s": 0.003},
    "deadband_3": {"deadband_ticks": 3.0},
    # diagnostic reference, not a servo hypothesis: a stiff actuator without lag
    "ideal_actuator": {"k_duty_per_tick": 0.2, "ff_frac": 1.0, "tau_m_s": 0.003, "friction_frac": 0.0,
                       "deadband_ticks": 0.0, "latency_s": 0.0, "torque_limit": 10.0},
}
FRICTION_VARIANTS = {"mu_0.5": 0.5, "mu_1.0": 1.0}

WALKS = ["imufollows_walk_slow", "imufollows_walk_standard", "imufollows_walk_fast",
         "preset_slow_standard_fast_slow", "joystick_full_slow_fast_slow", "imufollows_preset_slow_standard_fast_slow",
         "imufollows_joystick_full_slow_fast_slow", "joystick_steps_release_standard", "joystick_steps_release_fast"]
STOPS = ["stop_front_midswing_slow", "stop_front_midswing_standard", "stop_front_midswing_fast",
         "stop_front_lateswing_slow", "stop_front_lateswing_standard", "stop_front_lateswing_fast",
         "imufollows_stop_front_midswing_fast", "preset_fast_stop_midswing", "preset_slow_to_fast_stop_during_change"]
HIP = [f"stop_standard_k{k}" for k in range(8)] + [f"stop_slow_to_standard_t{t}" for t in range(3, 10)]


def run(seq_dir, out_dir, name, variant="nominal", mu=0.8):
    params = dataclasses.replace(NOMINAL, **VARIANTS.get(variant, {}))
    seq = twin.sequence(seq_dir / f"{name}.csv")
    started = time.time()
    tw = dynamics.Twin(params, mu=mu)
    rec = tw.run(seq)
    report = dynamics.metrics(seq, rec, params)
    report.update(sequence=name, variant=variant, mu=mu, wall_s=round(time.time() - started, 1),
                  params={k: getattr(params, k) for k in servo.PARAMS} | {"tau_full_nm": params.tau_full,
                          "kp_nm_rad": params.kp, "armature_kgm2": params.armature, "damping_nms": params.damping,
                          "friction_nm": params.friction})
    tag = f"{name}__{variant}" + ("" if mu == 0.8 else f"__mu{mu}")
    (out_dir / "runs").mkdir(parents=True, exist_ok=True)
    (out_dir / "runs" / f"{tag}.json").write_text(json.dumps(report, indent=1, default=float) + "\n")
    stops = report["stops"]
    s = stops[0] if stops else None
    line = f"{tag:70s} {report['wall_s']:5.1f}s"
    if s:
        line += (f" STOP over {s['overshoot_beyond_plan_end_max_ticks']:5.1f}t ({s['overshoot_joint']}) box {s['kheight_out_max_ticks']:5.1f}t ({s['kheight_out_joint']})"
                 f" verified {s['verified_after_s']} duty {s['duty_max_permille']:4.0f} acc {s['body_acc_max_m_s2']:4.1f} slip {s['slip_max_mm']:4.1f}mm")
    print(line, flush=True)
    return report


def summary(reports):
    rows = []
    for r in reports:
        row = {"sequence": r["sequence"], "variant": r["variant"], "mu": r["mu"], "segments": {}}
        for name, seg in r["segments"].items():
            row["segments"][name] = {k: seg[k] for k in ("seconds", "track_max_ticks", "track_max_joint", "duty_max_permille",
                                                          "duty_max_joint", "duty_at_limit_pct", "pitch_mrad", "roll_mrad",
                                                          "pitch_minus_commanded_mrad", "support_margin_min_mm", "support_margin_p05_mm", "slip_max_mm",
                                                          "stance_foot_without_contact_pct", "swing_foot_dragging_pct", "other_ground_contact", "body_acc_max_m_s2")}
        row["stops"] = [{k: s[k] for k in ("t_s", "plan_s", "drive", "fastest_joint_at_stop", "fastest_speed_step_s",
                                             "overshoot_beyond_plan_end_max_ticks", "overshoot_joint", "kheight_out_max_ticks",
                                             "kheight_out_joint", "verified_after_s", "duty_max_permille", "duty_at_limit",
                                             "body_acc_max_m_s2", "pitch_change_mrad", "slip_max_mm", "support_margin_min_mm",
                                             "other_ground_contact")} for s in r["stops"]]
        row["stop_lh_hip_kheight_out_ticks"] = [s["kheight_out_ticks"][9] for s in r["stops"]]
        if "walk_travel" in r:
            row["walk_travel"] = r["walk_travel"]
        rows.append(row)
    return rows


def main():
    seq_dir, out_dir = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    suites = sys.argv[3:] or ["walks", "stops", "hip", "sensitivity"]
    out_dir.mkdir(parents=True, exist_ok=True)
    for suite in suites:
        reports = []
        if suite == "walks":
            reports += [run(seq_dir, out_dir, n) for n in WALKS]
            reports += [run(seq_dir, out_dir, "imufollows_preset_slow_standard_fast_slow", v) for v in ("torque_limit_500", "volt_9.6")]
            reports += [run(seq_dir, out_dir, "imufollows_preset_slow_standard_fast_slow", mu=m) for m in FRICTION_VARIANTS.values()]
        elif suite == "stops":
            reports += [run(seq_dir, out_dir, n) for n in STOPS]
        elif suite == "hip":
            for v in ("nominal", "stall_x0.7", "torque_limit_500"):
                reports += [run(seq_dir, out_dir, n, v) for n in HIP]
        elif suite == "lateswing":  # added after the first stops suite: late front swing, with braking caps
            for n in ("stop_front_lateswing_slow", "stop_front_lateswing_standard", "stop_front_lateswing_fast"):
                reports += [run(seq_dir, out_dir, n, v) for v in ("nominal", "brake_8400", "brake_4000", "tau_m_35ms", "torque_limit_500")]
        elif suite == "sensitivity":
            for n in ("stop_front_midswing_slow", "stop_front_midswing_standard", "stop_front_midswing_fast"):
                reports += [run(seq_dir, out_dir, n, v) for v in VARIANTS if v != "nominal"]
                reports += [run(seq_dir, out_dir, n, mu=m) for m in FRICTION_VARIANTS.values()]
        (out_dir / f"m3_{suite}.json").write_text(json.dumps(summary(reports), indent=1, default=float) + "\n")


if __name__ == "__main__":
    main()
