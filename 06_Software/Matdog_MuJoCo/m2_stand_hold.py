#!/usr/bin/env python3
"""Milestone 2 check under load: the twin holding STAND 150 against the rc.11
hardware HOLD (2026-10-30, VH: weight on the four legs, fall-arrest slack,
TorqueLimit 1000, 11.3 V).

Usage: m2_stand_hold.py <rc.11 joints_timeseries.csv> <out dir>

The commanded RAW of the HOLD is mapped to URDF radians with the gen-1 transform
(q0 of scripts/tests/test_preenable_bus.cpp, encoder_direction of
CalibrationGeometryProfileData.h): raw = q0 + dir * ticks(q). The twin holds that
pose on a flat floor with the servo model; per joint it reports the static error
q_cmd - q (ticks, joint space) and the duty (PresentLoad, per mille, joint space:
-dir * load), against the hardware medians over the HOLD.
"""
import csv
import json
import pathlib
import re
import sys

import mujoco
import numpy as np

import dynamics
import servo
import twin

FW = twin.FW_TREE
BUS_IDS = (13, 12, 11, 23, 22, 21, 33, 32, 31, 43, 42, 41)  # slots LF RF RH LH x hip upper lower


def gen1():
    q0 = re.search(r"static const uint16_t q0\[12\]=\{([^}]*)\}", (FW / "scripts/tests/test_preenable_bus.cpp").read_text())
    q0 = np.array([int(v) for v in q0.group(1).split(",")])
    rows = re.findall(r"\}, (\d+), (-?1), EncoderDirectionSource", (FW / "src/actuator/CalibrationGeometryProfileData.h").read_text())
    direction = {int(bus): int(sign) for bus, sign in rows}
    return q0, np.array([direction[b] for b in BUS_IDS])


def hardware_hold(path, q0, direction):
    with open(path, newline="") as f:
        rows = [r for r in csv.DictReader(f) if r["state"] == "STANDING" and r["phase"] == "HOLD" and r["energized"] == "1"]
    cmd, err, load = np.zeros(12), np.zeros(12), np.zeros(12)
    for s in range(12):
        mine = [r for r in rows if int(r["slot"]) == s]
        cmd[s] = np.median([int(r["cmd"]) for r in mine])
        err[s] = direction[s] * np.median([int(r["cmd"]) - int(r["raw"]) for r in mine])
        load[s] = -direction[s] * np.median([int(r["load"]) for r in mine])
    q_cmd = direction * (cmd - q0) * servo.TICK
    return q_cmd, err, load, len(rows) // 12, float(np.median([float(r["v_dv"]) for r in rows])) / 10


def simulate(q_cmd, params, hold_s=4.0):
    tw = dynamics.Twin(params)
    tw.place(q_cmd)
    servos = servo.Servos(params, q_cmd)
    servos.hold(q_cmd)
    servos.command(0.0, q_cmd, 480, 13)
    errs, duties, now = [], [], 0.0
    for n in range(int(hold_s / tw.dt)):
        servos.step(now, tw.dt)
        tw.d.ctrl[:] = servos.torque(tw.d.qpos[7:19])
        mujoco.mj_step(tw.m, tw.d)
        now += tw.dt
        if now > hold_s - 1.0:
            errs.append((q_cmd - tw.d.qpos[7:19]) / servo.TICK)
            duties.append(servos.duty(tw.d.qpos[7:19]))
    return np.mean(errs, axis=0), 1e3 * np.mean(duties, axis=0), float(tw.d.qpos[2])


def main():
    hw_csv, out_dir = pathlib.Path(sys.argv[1]), pathlib.Path(sys.argv[2])
    out_dir.mkdir(parents=True, exist_ok=True)
    q0, direction = gen1()
    q_cmd, hw_err, hw_load, samples, volt = hardware_hold(hw_csv, q0, direction)
    nominal = servo.Params(volt=volt)
    variants = {
        "nominal": nominal,
        "stall_x0.8": servo.Params(volt=volt, stall_torque_nm_12v=0.8 * nominal.stall_torque_nm_12v),
        "stall_x1.2": servo.Params(volt=volt, stall_torque_nm_12v=1.2 * nominal.stall_torque_nm_12v),
        "friction_0.00": servo.Params(volt=volt, friction_frac=0.0),
        "friction_0.06": servo.Params(volt=volt, friction_frac=0.06),
    }
    loaded = np.abs(hw_load) >= 100  # the four joints the hardware loads most
    result = {"hardware": {"source": str(hw_csv), "hold_samples_per_joint": samples, "volt": volt,
                           "error_ticks": hw_err.tolist(), "duty_permille": hw_load.tolist(),
                           "duty_per_tick": float(np.median(hw_load[loaded] / hw_err[loaded]))},
              "q_cmd_rad": q_cmd.tolist(), "variants": {}}
    print(f"rc.11 HOLD: {samples} samples/joint at {volt:.1f} V; duty/error on loaded joints {result['hardware']['duty_per_tick']:.2f} permille/tick")
    print(f"{'joint':10s} {'HW err':>7s} {'HW duty':>8s} | " + " | ".join(f"{k:>16s}" for k in variants))
    sims = {}
    for name, p in variants.items():
        e, dmil, z = simulate(q_cmd, p)
        sims[name] = (e, dmil)
        result["variants"][name] = {"error_ticks": e.tolist(), "duty_permille": dmil.tolist(), "base_z_m": z,
                                    "tau_full_nm": p.tau_full, "friction_nm": p.friction,
                                    "loaded_joints_mean_abs_error_ticks": float(np.mean(np.abs(e[loaded]))),
                                    "sum_abs_duty_permille": float(np.sum(np.abs(dmil)))}
    for s in range(12):
        print(f"{twin.SLOT_NAMES[s]:10s} {hw_err[s]:7.1f} {hw_load[s]:8.0f} | " +
              " | ".join(f"{sims[k][0][s]:6.1f} t {sims[k][1][s]:6.0f}" for k in variants))
    hw_mean = float(np.mean(np.abs(hw_err[loaded])))
    print(f"loaded joints {[twin.SLOT_NAMES[s] for s in np.where(loaded)[0]]}: HW mean |err| {hw_mean:.1f} ticks, "
          f"sum |duty| HW {np.sum(np.abs(hw_load)):.0f} permille")
    for name in variants:
        v = result["variants"][name]
        print(f"  {name:14s} mean |err| {v['loaded_joints_mean_abs_error_ticks']:5.1f} ticks, sum |duty| {v['sum_abs_duty_permille']:6.0f} permille")
    result["hardware"]["loaded_joints"] = [twin.SLOT_NAMES[s] for s in np.where(loaded)[0]]
    result["hardware"]["loaded_mean_abs_error_ticks"] = hw_mean
    result["hardware"]["sum_abs_duty_permille"] = float(np.sum(np.abs(hw_load)))
    (out_dir / "m2_stand_hold.json").write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
