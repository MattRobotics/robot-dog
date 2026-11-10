#!/usr/bin/env python3
"""IMU tilt against the tilt the encoders give, on the ground, per STANDING HOLD snapshot (M5).

With the four feet on a flat floor, the body's roll and pitch follow from the measured
joints alone: the plane through the four contact references (twin geometry = firmware
FK, encoders through the gen-1 map) is the floor. The IMU (BNO085 quaternion, read by
the firmware as R_world_from_base, ImuAttitude.cpp) gives the same angles if it is
mounted aligned with base_link and the floor is level. The difference is the IMU
mounting offset plus floor slope plus what the encoders cannot see (link and bracket
flex). Ground logs only: on the fixture the feet carry nothing (--from: the controller
time, s, at which the robot stands on the floor). The heading (IMU yaw, deg) of each
group tells a mounting offset (fixed in the body) from a floor slope (turns with it).

Usage: m5_imu_tilt.py <dashboard trace .json | http.jsonl> [...] [--from S]
"""
import sys

import mujoco
import numpy as np

import dynamics
import m2_stand_hold
import m5_hw
import twin


def feet_plane_tilt(model, data, centers, feet, q):
    """(roll, pitch) of base_link standing on a level plane through the four feet at joints q."""
    mujoco.mj_resetData(model, data)
    data.qpos[3:7] = [1, 0, 0, 0]
    data.qpos[7:19] = q
    mujoco.mj_kinematics(model, data)
    p = twin.contact_reference(data, centers, feet)
    _, b, c = np.linalg.lstsq(np.c_[np.ones(4), p[:, 0], p[:, 1]], p[:, 2], rcond=None)[0]
    return np.array([-np.arctan(c), np.arctan(b)])


def main():
    q0, direction = m2_stand_hold.gen1()
    model, data = twin.load_model()
    centers = twin.site_ids(model, "foot_cylinder_center")
    feet = np.array([model.geom(f"{l}_foot_cylinder").id for l in twin.LEGS])
    args = sys.argv[1:]
    t_from = float(args.pop(args.index("--from") + 1)) * 1e3 if "--from" in args else -np.inf
    args = [a for a in args if a != "--from"]
    rows = []  # traces exported one after the other overlap: each controller instant once (m5_hw.merge)
    for s in m5_hw.load(args)[1]:
        if s["state"] == "STANDING" and s["phase"] == "HOLD" and s["torque"] and s["quat"] and s["now_ms"] >= t_from:
            imu = np.array(dynamics.quat_rpy(s["quat"]))
            w, x, y, z = s["quat"]
            yaw = np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
            rows.append((s["source"], s["height_mm"], 1e3 * (imu - feet_plane_tilt(model, data, centers, feet, direction * (s["raw"] - q0) * twin.TICK)), yaw))
    for source, h in dict.fromkeys((r[0], r[1]) for r in rows):
        d = np.array([r[2] for r in rows if r[0] == source and r[1] == h])
        yaw = np.array([r[3] for r in rows if r[0] == source and r[1] == h])
        print(f"{source:34s} HOLD {h:5.0f} mm n {len(d):3d}: IMU - encoders roll {np.median(d[:, 0]):6.1f} pitch {np.median(d[:, 1]):6.1f} mrad"
              f" (heading {np.median(yaw):6.1f} deg, {yaw.min():.0f}..{yaw.max():.0f})")
    d = np.array([r[2] for r in rows])
    if len(d):
        print(f"all: n {len(d)}, roll {np.median(d[:, 0]):.1f} (p5 {np.percentile(d[:, 0], 5):.1f}, p95 {np.percentile(d[:, 0], 95):.1f}), "
              f"pitch {np.median(d[:, 1]):.1f} (p5 {np.percentile(d[:, 1], 5):.1f}, p95 {np.percentile(d[:, 1], 95):.1f}) mrad")


if __name__ == "__main__":
    main()
