#!/usr/bin/env python3
"""Markdown tables of the twin's results for the report.

Usage: report_tables.py <out dir of the runs>   (m1, m2, identify, m3 runs/, cl/)
"""
import glob
import json
import pathlib
import sys

SLOTS = [f"{leg}_{joint}" for leg in ("LF", "RF", "RH", "LH") for joint in ("HIP", "UPPER", "LOWER")]


def load(path):
    return json.loads(pathlib.Path(path).read_text())


def f(v, n=1):
    return "—" if v is None else f"{v:.{n}f}"


def main():
    out = pathlib.Path(sys.argv[1])
    runs = {pathlib.Path(p).stem: load(p) for p in sorted(glob.glob(str(out / "m3" / "runs" / "*.json")))}
    print("### Open loop: walks (nominal unless noted)\n")
    print("| run | segment | s | track p95 / max (tick) | duty max (‰) | pitch (mrad) | roll (mrad) | drag % | slip max (mm) | margin p05 (mm) | travel twin/law (mm) |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for tag, r in runs.items():
        if r["stops"] and not tag.startswith(("preset_", "joystick", "imufollows_preset", "imufollows_joystick", "imufollows_walk")):
            continue
        travel = r.get("walk_travel")
        for name, s in r["segments"].items():
            if not name.startswith(("walk_", "change_")):
                continue
            print(f"| {tag} | {name} | {s['seconds']:.0f} | {s['track_p95_ticks']:.0f} / {s['track_max_ticks']:.0f} | {s['duty_max_permille']:.0f} | "
                  f"{s['pitch_mrad'][0]:.0f}…{s['pitch_mrad'][1]:.0f} | {s['roll_mrad'][0]:.0f}…{s['roll_mrad'][1]:.0f} | "
                  f"{s['swing_foot_dragging_pct']:.1f} | {s['slip_max_mm']:.1f} | {s['support_margin_p05_mm']:.1f} | "
                  f"{(f'{travel['twin_mm']:.0f}/{travel['law_mm']:.0f}' if travel else '—')} |")
    print("\n### Open loop: STOP (every run with a STOP)\n")
    print("| run | drive | plan s | fastest joint (step/s) | its overshoot beyond plan end (tick) | kHeight out (tick, joint) | LH_HIP out | verified after (s) | duty max (‰) | body acc max (m/s²) | slip (mm) |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for tag, r in runs.items():
        for s in r["stops"]:
            fastest = SLOTS.index(s["fastest_joint_at_stop"])
            print(f"| {tag} | {s['drive'][0]}/{s['drive'][1]} | {s['plan_s']:.2f} | {s['fastest_joint_at_stop']} {s['fastest_speed_step_s']:.0f} | "
                  f"{s['overshoot_ticks'][fastest]:+.1f} | {s['kheight_out_max_ticks']:.1f} {s['kheight_out_joint']} | "
                  f"{s['kheight_out_ticks'][9]:.1f} | {f(s['verified_after_s'], 2)} | {s['duty_max_permille']:.0f} | {s['body_acc_max_m_s2']:.1f} | {s['slip_max_mm']:.1f} |")
    cl = {pathlib.Path(p).stem: load(p) for p in sorted(glob.glob(str(out / "cl" / "*.json")))}
    print("\n### Closed loop (SimpleMotion fed by the twin)\n")
    print("| scenario | final state (reason) | first fault (t, reason) | measured outside kHeight at the fault (tick) | IMU applied pitch / roll (mrad) | walk segments: track p95 (tick), duty max (‰), drag % | travel twin/law (mm) |")
    print("|---|---|---|---|---|---|---|")
    for tag, r in cl.items():
        segs = "; ".join(f"{n} {s['track_p95_ticks']:.0f}, {s['duty_max_permille']:.0f}, {s['swing_foot_dragging_pct']:.0f}"
                         for n, s in r["segments"].items() if n.startswith(("walk_", "change_")))
        travel = r.get("walk_travel")
        fb = r.get("fault_feedback") or {}
        print(f"| {tag} | {r['final_state']} ({r['final_reason']}) | {r['first_fault'] or '—'} | {fb.get('kheight_out_ticks', '—')} | "
              f"{r['imu_pitch_applied_mrad'][0]:.1f}…{r['imu_pitch_applied_mrad'][1]:.1f} / {r['imu_roll_applied_mrad'][0]:.1f}…{r['imu_roll_applied_mrad'][1]:.1f} | {segs} | "
              f"{(f'{travel['twin_mm']:.0f}/{travel['law_mm']:.0f}' if travel else '—')} |")


if __name__ == "__main__":
    main()
