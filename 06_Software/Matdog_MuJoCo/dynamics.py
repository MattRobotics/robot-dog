"""Physics replay of SimpleMotion references in the MuJoCo twin, and its metrics.

Open loop: each release (20 ms) the twin receives the frame SimpleMotion sent on
the scripted rig (12 goals + drive {speed, ACC}); the servo model of servo.py
tracks it on the MuJoCo body with contacts. The twin's state never reaches
SimpleMotion (no IMU or encoder feedback), so divergences accumulate exactly as
an uncorrected plant would let them. Results are simulation, never hardware.
"""
import math
import pathlib
import re

import mujoco
import numpy as np

import servo
import twin

HERE = pathlib.Path(__file__).resolve().parent
COLLISION_H = HERE.parents[1] / "05_Firmware/MATDOG_Controller/src/simple/CollisionData.h"
CONTACT_N = 0.1        # a foot is in contact above this normal force (N)
DRAG_N = 2.0           # a swing foot carrying more than this drags on the floor (N)
VERIFY_TICKS, VERIFY_LOAD, VERIFY_SPEED = 48, 0.400, 50   # SimpleMotion STOP verification (handoff section 4)


def box(name="kHeight"):
    text = COLLISION_H.read_text()
    match = re.search(r"constexpr Box " + name + r"=\{\{([^}]*)\},\{([^}]*)\}\};", text)
    lo, hi = (np.array([float(v) for v in g.split(",")]) for g in match.groups())
    return lo, hi


def quat_rpy(q):
    w, x, y, z = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def cross2(a, b):
    return a[0] * b[1] - a[1] * b[0]


def support_margin(points, com):
    """Signed distance (m) of the COM from the support polygon of `points` (xy):
    > 0 inside, < 0 outside. One or two points: the distance to them, negative."""
    pts = np.unique(np.round(points[:, :2], 9), axis=0)
    if len(pts) < 3:
        if len(pts) == 0:
            return -np.inf
        if len(pts) == 1:
            return -float(np.linalg.norm(com - pts[0]))
        a, b = pts
        u = np.clip(np.dot(com - a, b - a) / np.dot(b - a, b - a), 0, 1)
        return -float(np.linalg.norm(com - (a + u * (b - a))))
    # convex hull (monotone chain)
    pts = pts[np.lexsort((pts[:, 1], pts[:, 0]))]

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and cross2(out[-1] - out[-2], p - out[-2]) <= 0:
                out.pop()
            out.append(p)
        return out
    hull = np.array(half(pts)[:-1] + half(pts[::-1])[:-1])
    inside, best = True, np.inf
    for i in range(len(hull)):
        a, b = hull[i], hull[(i + 1) % len(hull)]
        edge = b - a
        cross = cross2(edge, com - a)
        if cross < 0:
            inside = False
        u = np.clip(np.dot(com - a, edge) / np.dot(edge, edge), 0, 1)
        best = min(best, float(np.linalg.norm(com - (a + u * edge))))
    return best if inside else -best


class Twin:
    def __init__(self, params, mu=0.8, unit=None):
        self.p = params
        self.m, self.d = twin.load_model()
        self.feet = np.array([self.m.geom(f"{l}_foot_cylinder").id for l in twin.LEGS])
        self.floor = self.m.geom("floor").id
        self.m.geom_friction[self.feet, 0] = mu
        self.m.geom_friction[self.floor, 0] = mu
        self.dofs = np.arange(6, 18)
        self.unit = np.ones(12) if unit is None else np.asarray(unit, float)
        servo.apply_to_model(self.m, params, self.dofs, self.unit)
        self.centers = twin.site_ids(self.m, "foot_cylinder_center")
        self.foot_bodies = np.array([self.m.body(f"{l}_foot_link").id for l in twin.LEGS])
        self.base = self.m.body("base_link").id
        self.dt = self.m.opt.timestep

    def place(self, q):
        """Joints at q, body level, lowest contact reference on the floor."""
        mujoco.mj_resetData(self.m, self.d)
        twin.set_base(self.d, (0, 0, 0.3))
        self.d.qpos[7:19] = q
        mujoco.mj_kinematics(self.m, self.d)
        low = twin.contact_reference(self.d, self.centers, self.feet)[:, 2].min()
        self.d.qpos[2] -= low
        mujoco.mj_forward(self.m, self.d)

    def begin(self, q0, speed, acc, record_dt=0.001):
        """Feet on the floor at q0, servos holding it with the drive {speed, acc}."""
        self.place(q0)
        self.servos = servo.Servos(self.p, q0, self.unit)
        self.servos.hold(self.d.qpos[7:19])
        self.servos.command(0.0, q0, speed, acc)
        self.every = int(round(record_dt / self.dt))
        self.rec = {k: [] for k in ("t", "q", "qd", "duty", "pos", "quat", "vel", "com", "feet", "fz", "other")}
        self.release_end = []
        self.now = 0.0

    def advance(self, seconds, record=True):
        for s in range(int(round(seconds / self.dt))):
            self.servos.step(self.now, self.dt)
            self.d.ctrl[:] = self.servos.torque(self.d.qpos[7:19])
            mujoco.mj_step(self.m, self.d)
            self.now += self.dt
            if record and (s + 1) % self.every == 0:
                rec = self.rec
                mujoco.mj_rnePostConstraint(self.m, self.d)
                rec["t"].append(self.now)
                rec["q"].append(self.d.qpos[7:19].copy())
                rec["qd"].append(self.d.qvel[6:18].copy())
                rec["duty"].append(self.servos.duty(self.d.qpos[7:19]))
                rec["pos"].append(self.d.qpos[0:3].copy())
                rec["quat"].append(self.d.qpos[3:7].copy())
                rec["vel"].append(self.d.qvel[0:3].copy())
                rec["com"].append(self.d.subtree_com[self.base].copy())
                rec["feet"].append(twin.contact_reference(self.d, self.centers, self.feet))
                rec["fz"].append(self.d.cfrc_ext[self.foot_bodies, 5].copy())
                nc = self.d.ncon
                g1, g2 = self.d.contact.geom1[:nc], self.d.contact.geom2[:nc]
                rec["other"].append(float(np.any(~np.isin(np.where(g1 == self.floor, g2, g1), self.feet))))

    def release(self, goal, speed, acc, torque_on=None, record=True):
        """One Motion release: a SYNC_WRITE frame, then 20 ms of physics."""
        if torque_on is not None and not np.array_equal(torque_on, self.servos.torque_on):
            self.servos.torque_on = np.asarray(torque_on, bool)  # SAFE_OFF: no drive, no back-EMF
            servo.apply_to_model(self.m, self.p, self.dofs, self.unit, self.servos.torque_on)
        self.servos.command(self.now, goal, speed, acc)
        self.advance(twin.RELEASE_S, record)
        self.release_end.append(self.d.qpos[7:19].copy())

    def result(self):
        out = {k: np.array(v) for k, v in self.rec.items()}
        out["release_end_q"] = np.array(self.release_end)
        return out

    def run(self, seq, settle_s=1.5, record_dt=0.001):
        """Open loop: hold frame 0 for settle_s, then replay every release of `seq`."""
        speed = seq["drive_speed_step_s"].astype(int)
        acc = seq["drive_acc_100step_s2"].astype(int)
        self.begin(seq["q"][0], speed[0], acc[0], record_dt)
        self.now = seq["t"][0] - twin.RELEASE_S - settle_s
        self.advance(settle_s, record=False)
        for k in range(len(seq["q"])):
            self.release(seq["q"][k], speed[k], acc[k])
        return self.result()


def segments(seq):
    """Label each release: settle/height, walk_<MODE>, change_<A>_to_<B>, stop, hold."""
    labels = []
    for k in range(len(seq["q"])):
        if seq["stop_active"][k] == "1":
            labels.append("stop")
        elif seq["state"][k] == "WALKING":
            a, b = seq["law_mode"][k], seq["target_mode"][k]
            labels.append(f"walk_{a}" if a == b else f"change_{a}_to_{b}")
        else:
            labels.append(str(seq["phase"][k]).lower())
    return np.array(labels)


def metrics(seq, rec, params, unit=None):
    """Per segment and per STOP: the divergences of handoff section 5."""
    lo, hi = box("kHeight")
    labels = segments(seq)
    t_rel = seq["t"]
    # frame k is applied over (t_k - 20 ms, t_k]: the release tracked at each record
    rel_index = np.clip(np.searchsorted(t_rel, rec["t"] - 1e-9), 0, len(t_rel) - 1)
    lab = labels[rel_index]
    rp = np.array([quat_rpy(q) for q in rec["quat"]])
    loaded = rec["fz"] > CONTACT_N
    margin = np.full(len(rec["t"]), np.inf)  # every 5 ms
    for i in range(0, len(rec["t"]), 5):
        margin[i] = support_margin(rec["feet"][i][loaded[i]], rec["com"][i][:2])
    track = (rec["release_end_q"] - seq["q"]) / servo.TICK
    expected_pitch = seq["imu_pitch"].astype(float)
    expected_roll = seq["imu_roll"].astype(float)
    acc = np.gradient(rec["vel"], rec["t"], axis=0)
    kernel = np.ones(10) / 10
    acc_f = np.stack([np.convolve(acc[:, i], kernel, mode="same") for i in range(3)], axis=1)
    acc_norm = np.linalg.norm(acc_f, axis=1)

    def slip(mask):
        """Largest horizontal excursion of a loaded foot within one continuous loaded interval (mm)."""
        worst = 0.0
        for l in range(4):
            on = loaded[:, l] & mask
            start = None
            for i in range(len(on)):
                if on[i] and start is None:
                    start = i
                if (not on[i] or i == len(on) - 1) and start is not None:
                    seg = rec["feet"][start:i + 1, l, :2]
                    worst = max(worst, float(np.max(np.linalg.norm(seg - seg[0], axis=1))) if len(seg) else 0.0)
                    start = None
        return 1e3 * worst

    report = {"segments": {}, "stops": []}
    for name in dict.fromkeys(labels):
        rel = labels == name
        m = lab == name
        if not m.any():
            continue
        tr = np.abs(track[rel])
        j = int(np.argmax(tr.max(axis=0)))
        duty = np.abs(rec["duty"][m])
        stance_unloaded = swing_loaded = feet_checked = 0
        if name.startswith(("walk_", "change_")):
            for k in np.where(rel)[0]:
                i = min(np.searchsorted(rec["t"], t_rel[k] - 1e-9), len(rec["t"]) - 1)
                for l in range(4):
                    feet_checked += 1
                    if seq[f"stance{l}"][k] == "1" and not loaded[i, l]:
                        stance_unloaded += 1
                    if seq[f"stance{l}"][k] == "0" and rec["fz"][i, l] > DRAG_N:
                        swing_loaded += 1
        report["segments"][name] = {
            "seconds": float(np.sum(rel) * twin.RELEASE_S),
            "track_max_ticks": float(tr.max()), "track_max_joint": twin.SLOT_NAMES[j],
            "track_p95_ticks": float(np.percentile(tr, 95)),
            "duty_max_permille": float(1e3 * duty.max()), "duty_max_joint": twin.SLOT_NAMES[int(np.argmax(duty.max(axis=0)))],
            "duty_at_limit_pct": float(100 * np.mean(duty >= params.torque_limit - 1e-9)),
            "torque_max_nm": float(duty.max() * params.tau_full),
            "pitch_mrad": [float(1e3 * rp[m, 1].min()), float(1e3 * rp[m, 1].max())],
            "roll_mrad": [float(1e3 * rp[m, 0].min()), float(1e3 * rp[m, 0].max())],
            "pitch_minus_commanded_mrad": float(1e3 * np.max(np.abs(rp[m, 1] - expected_pitch[rel_index[m]]))),
            "support_margin_min_mm": float(1e3 * margin[m][np.isfinite(margin[m])].min()),
            "support_margin_p05_mm": float(1e3 * np.percentile(margin[m][np.isfinite(margin[m])], 5)),
            "slip_max_mm": slip(m),
            "stance_foot_without_contact_pct": float(100 * stance_unloaded / max(1, feet_checked)),
            "swing_foot_dragging_pct": float(100 * swing_loaded / max(1, feet_checked)),
            "other_ground_contact": bool(rec["other"][m].max() > 0),
            "body_acc_max_m_s2": float(acc_norm[m].max()),
        }
    walk = np.array([lab_.startswith(("walk_", "change_")) for lab_ in labels])
    if walk.any():
        k0, k1 = np.where(walk)[0][[0, -1]]
        i0 = np.searchsorted(rec["t"], t_rel[k0])
        i1 = min(np.searchsorted(rec["t"], t_rel[k1]), len(rec["t"]) - 1)
        law = float(seq["body_x"][k1]) - float(seq["body_x"][k0])
        report["walk_travel"] = {"law_mm": 1e3 * law, "twin_mm": 1e3 * float(rec["pos"][i1, 0] - rec["pos"][i0, 0]),
                                 "lateral_drift_mm": 1e3 * float(rec["pos"][i1, 1] - rec["pos"][i0, 1]),
                                 "yaw_drift_mrad": 1e3 * float(2 * math.atan2(rec["quat"][i1, 3], rec["quat"][i1, 0]) - 2 * math.atan2(rec["quat"][i0, 3], rec["quat"][i0, 0]))}

    starts = [k for k in range(len(labels)) if labels[k] == "stop" and (k == 0 or labels[k - 1] != "stop")]
    for k in starts:
        ts = t_rel[k] - twin.RELEASE_S  # the STOP frame is applied from here
        i0 = np.searchsorted(rec["t"], ts)
        after = slice(i0, len(rec["t"]))
        end = np.array([float(seq[f"stop_end{i}"][k]) for i in range(12)])
        q_at = rec["q"][i0]
        qd_at = rec["qd"][i0]
        direction = np.sign(qd_at)
        over = np.max((rec["q"][after] - end) * direction, axis=0) / servo.TICK
        box_out = np.maximum(lo - rec["q"][after], rec["q"][after] - hi).max(axis=0) / servo.TICK
        goal = seq["q"][-1]
        ok = (np.all(np.abs(rec["q"] - goal) / servo.TICK <= VERIFY_TICKS, axis=1) &
              np.all(np.abs(rec["duty"]) <= VERIFY_LOAD, axis=1) &
              np.all(np.abs(rec["qd"]) / servo.TICK <= VERIFY_SPEED, axis=1))
        verified = None
        w = int(0.5 / (rec["t"][1] - rec["t"][0]))
        for i in range(i0, len(rec["t"]) - w):
            if ok[i] and ok[i:i + w].all() and np.max(np.abs(rec["q"][i + w] - rec["q"][i])) / servo.TICK <= 1:
                verified = float(rec["t"][i] - ts)
                break
        window = (rec["t"] >= ts) & (rec["t"] <= ts + 1.0)
        settle = (rec["t"] >= ts) & (rec["t"] <= ts + 2.0)
        stop_speed = np.abs(qd_at) / servo.TICK
        report["stops"].append({
            "t_s": float(ts), "plan_s": float(seq["stop_plan_s"][k]), "drive": [int(seq["drive_speed_step_s"][k]), int(seq["drive_acc_100step_s2"][k])],
            "fastest_joint_at_stop": twin.SLOT_NAMES[int(np.argmax(stop_speed))], "fastest_speed_step_s": float(stop_speed.max()),
            "overshoot_beyond_plan_end_max_ticks": float(over.max()), "overshoot_joint": twin.SLOT_NAMES[int(np.argmax(over))],
            "overshoot_ticks": [float(v) for v in over],
            "kheight_out_max_ticks": float(box_out.max()), "kheight_out_joint": twin.SLOT_NAMES[int(np.argmax(box_out))],
            "kheight_out_ticks": [float(v) for v in box_out],
            "verified_after_s": verified,
            "duty_max_permille": float(1e3 * np.abs(rec["duty"][window]).max()),
            "duty_at_limit": bool(np.abs(rec["duty"][window]).max() >= params.torque_limit - 1e-9),
            "body_acc_max_m_s2": float(acc_norm[window].max()),
            "pitch_change_mrad": float(1e3 * np.ptp(rp[window, 1])), "roll_change_mrad": float(1e3 * np.ptp(rp[window, 0])),
            "slip_max_mm": slip(settle),
            "support_margin_min_mm": float(1e3 * margin[settle][np.isfinite(margin[settle])].min()),
            "other_ground_contact": bool(rec["other"][settle].max() > 0),
            "final_error_max_ticks": float(np.max(np.abs(rec["q"][-1] - goal)) / servo.TICK),
        })
    return report
