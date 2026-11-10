#!/usr/bin/env python3
"""Closed loop: the production SimpleMotion (fw_bridge/cosim) driven by the MuJoCo
twin. Each release the twin's encoders (rig RAW map), speeds, loads (duty) and
base orientation (as the IMU quaternion) are offered to SimpleMotion, whose frame
goes back to the servo model. SimpleMotion itself reads two servos per release
(each servo every 120 ms), as on the robot; the read sees the state at the
release boundary without bus latency, and the IMU is the exact base orientation
(no noise, latency or mounting error). Simulation only.

Usage: cosim.py <cosim binary> <out dir> [scenario ...]
"""
import csv
import dataclasses
import json
import pathlib
import subprocess
import sys
import time

import numpy as np

import dynamics
import servo
import twin

# name: (events [(t_s, command)], duration_s); t = 0 at STANDING 150 mm
SCENARIOS = {
    "cl_walk_slow": ([(1.0, "SELECT SLOW"), (1.0, "WALK 2")], 45.0),
    "cl_walk_standard": ([(1.0, "SELECT STANDARD"), (1.0, "WALK 2")], 35.0),
    "cl_walk_fast": ([(1.0, "SELECT FAST"), (1.0, "WALK 2")], 35.0),
    "cl_modes_slow_standard_fast_slow": ([(1.0, "SELECT SLOW"), (1.0, "WALK 2"), (12.0, "SELECT STANDARD"),
                                          (24.0, "SELECT FAST"), (34.0, "SELECT SLOW")], 50.0),
    "cl_stop_front_slow": ([(1.0, "SELECT SLOW"), (1.0, "WALK 2"), (1.0, "STOPFRONT 8")], 22.0),
    "cl_stop_front_standard": ([(1.0, "SELECT STANDARD"), (1.0, "WALK 2"), (1.0, "STOPFRONT 8")], 20.0),
    "cl_stop_front_fast": ([(1.0, "SELECT FAST"), (1.0, "WALK 2"), (1.0, "STOPFRONT 8")], 20.0),
    "cl_stop_slow_to_standard": ([(1.0, "SELECT SLOW"), (1.0, "WALK 2"), (12.0, "SELECT STANDARD"), (16.0, "STOP")], 24.0),
    "cl_stop_lateswing_slow": ([(1.0, "SELECT SLOW"), (1.0, "WALK 2"), (1.0, "STOPFRONT 8 0.85")], 22.0),
    "cl_stop_lateswing_standard": ([(1.0, "SELECT STANDARD"), (1.0, "WALK 2"), (1.0, "STOPFRONT 8 0.85")], 20.0),
    "cl_stop_lateswing_fast": ([(1.0, "SELECT FAST"), (1.0, "WALK 2"), (1.0, "STOPFRONT 8 0.85")], 20.0),
}


class Bridge:
    def __init__(self, binary):
        self.p = subprocess.Popen([str(binary)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1)
        self.columns = self.ask("HEADER").split(",")[1:]

    def ask(self, line):
        self.p.stdin.write(line + "\n")
        self.p.stdin.flush()
        return self.p.stdout.readline().rstrip("\n")

    def record(self, reply):
        assert reply.startswith("R "), reply
        return dict(zip(self.columns, reply[2:].split(" ")[0].split(",")))

    def close(self):
        self.p.stdin.close()
        self.p.wait(timeout=10)


def run(binary, out_dir, name, params=None, mu=0.8, tag="", unit=None):
    """One closed-loop scenario; `unit` scales a servo's torque constant (servo.apply_to_model)."""
    params = params or servo.Params()
    events, duration = SCENARIOS[name]
    bridge = Bridge(binary)
    standing = bridge.record(bridge.ask("STAND 0.15"))
    raw0 = np.array([int(standing[f"raw{i}"]) for i in range(12)])
    tw = dynamics.Twin(params, mu=mu, unit=unit)
    tw.begin((raw0 - 2048) * twin.TICK, int(standing["drive_speed_step_s"]), int(standing["drive_acc_100step_s2"]))
    rows, replies, pending = [], [], list(events)
    started = time.time()
    for n in range(int(round(duration / twin.RELEASE_S))):
        t = n * twin.RELEASE_S
        while pending and pending[0][0] <= t + 1e-9:
            replies.append((t, pending[0][1], bridge.ask(pending.pop(0)[1])))
        d = tw.d
        q, qd = d.qpos[7:19], d.qvel[6:18]
        raw = 2048 + np.rint(q / twin.TICK).astype(int)
        speed = np.rint(qd / twin.TICK).astype(int)
        # PresentLoad: the duty with the servo's sign convention, opposite to (goal - raw):
        # settledUnderLoad requires (commandedRaw - raw > 0) == load bit 10 (rc.11 HOLD: err -21, load +176).
        load = -np.rint(1e3 * tw.servos.duty(q)).astype(int)
        quat = d.qpos[3:7]
        line = "TICK " + " ".join(map(str, np.concatenate([raw, speed, load]))) + " " + " ".join(repr(float(v)) for v in quat)
        rec = bridge.record(bridge.ask(line))
        rec["t_s"] = repr(t + twin.RELEASE_S)
        for i in range(12):  # the feedback SimpleMotion read for this release
            rec[f"fb_raw{i}"], rec[f"fb_speed{i}"], rec[f"fb_load{i}"] = raw[i], speed[i], load[i]
        rows.append(rec)
        goal = (np.array([int(rec[f"raw{i}"]) for i in range(12)]) - 2048) * twin.TICK
        on = np.full(12, int(rec["torque_on"]) == 12)
        tw.release(goal, int(rec["drive_speed_step_s"]), int(rec["drive_acc_100step_s2"]), on)
    bridge.close()
    out_dir.mkdir(parents=True, exist_ok=True)
    tag = name + (f"__{tag}" if tag else "")
    path = out_dir / f"{tag}.csv"
    with open(path, "w", newline="") as f:
        feedback = [f"fb_{k}{i}" for k in ("raw", "speed", "load") for i in range(12)]
        w = csv.DictWriter(f, fieldnames=["t_s"] + bridge.columns + feedback)
        w.writeheader()
        w.writerows(rows)
    seq = twin.sequence(path)
    report = dynamics.metrics(seq, tw.result(), params)
    faults = [(r["t_s"], r["reason"]) for r in rows if r["state"] == "FAULT"]
    lo, hi = dynamics.box()
    fault_feedback = None
    if faults:
        k = next(i for i, r in enumerate(rows) if r["state"] == "FAULT")
        q_fb = (np.array([rows[k][f"fb_raw{i}"] for i in range(12)]) - 2048) * twin.TICK
        out = np.maximum(lo - q_fb, q_fb - hi) / twin.TICK
        fault_feedback = {"kheight_out_ticks": {twin.SLOT_NAMES[i]: float(out[i]) for i in range(12) if out[i] > 0},
                          "speed_step_s": [int(rows[k][f"fb_speed{i}"]) for i in range(12)],
                          "load_permille": [int(rows[k][f"fb_load{i}"]) for i in range(12)]}
    report.update(scenario=name, tag=tag, mu=mu, wall_s=round(time.time() - started, 1), commands=[list(c) for c in replies],
                  first_fault=faults[0] if faults else None, fault_feedback=fault_feedback, final_state=rows[-1]["state"], final_phase=rows[-1]["phase"],
                  final_reason=rows[-1]["reason"], stop_verified=any(r["stop_verified"] == "1" for r in rows),
                  imu_pitch_applied_mrad=[1e3 * float(min(float(r["imu_pitch"]) for r in rows)), 1e3 * float(max(float(r["imu_pitch"]) for r in rows))],
                  imu_roll_applied_mrad=[1e3 * float(min(float(r["imu_roll"]) for r in rows)), 1e3 * float(max(float(r["imu_roll"]) for r in rows))],
                  imu_reasons={k: sum(1 for r in rows if r["imu_reason"] == k) for k in {r["imu_reason"] for r in rows}})
    (out_dir / f"{tag}.json").write_text(json.dumps(report, indent=1, default=float) + "\n")
    walk = {k: v for k, v in report["segments"].items() if k.startswith(("walk_", "change_"))}
    print(f"{tag:45s} {report['wall_s']:6.1f}s final {report['final_state']}/{report['final_phase']} ({report['final_reason']}) "
          f"fault {report['first_fault']} stop_verified {report['stop_verified']}", flush=True)
    if fault_feedback:
        print(f"   at the fault, measured outside kHeight (ticks): {fault_feedback['kheight_out_ticks']}", flush=True)
    for k, v in walk.items():
        print(f"   {k:28s} track p95 {v['track_p95_ticks']:5.1f} max {v['track_max_ticks']:6.1f}  duty max {v['duty_max_permille']:4.0f}  "
              f"pitch {v['pitch_mrad'][0]:6.1f}..{v['pitch_mrad'][1]:5.1f} roll {v['roll_mrad'][0]:6.1f}..{v['roll_mrad'][1]:5.1f}  "
              f"drag {v['swing_foot_dragging_pct']:4.1f}%  slip {v['slip_max_mm']:4.1f} mm", flush=True)
    if "walk_travel" in report:
        print(f"   travel twin {report['walk_travel']['twin_mm']:.0f} mm vs law {report['walk_travel']['law_mm']:.0f} mm", flush=True)
    for s in report["stops"]:
        print(f"   STOP t {s['t_s']:.2f} drive {s['drive']} plan {s['plan_s']:.2f}s fastest {s['fastest_joint_at_stop']} {s['fastest_speed_step_s']:.0f} step/s: "
              f"overshoot {s['overshoot_beyond_plan_end_max_ticks']:.1f} t ({s['overshoot_joint']}), kHeight out {s['kheight_out_max_ticks']:.1f} t ({s['kheight_out_joint']}), "
              f"verified after {s['verified_after_s']}", flush=True)
    return report


def main():
    binary, out_dir = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    for name in sys.argv[3:] or list(SCENARIOS):
        run(binary, out_dir, name)


if __name__ == "__main__":
    main()
