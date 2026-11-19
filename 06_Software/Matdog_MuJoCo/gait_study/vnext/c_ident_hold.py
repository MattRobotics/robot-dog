#!/usr/bin/env python3
"""Offline identifiability of the ST3215 static compliance from the dev.7 ground HOLDs (11/11/2026), read only.

Data: the four HOLD windows of m6_dev7 (STAND 120, 110, 105, 120 mm; m5_hw snapshots, 2 s after arrival), joint
space through the gen-1 map (m2_stand_hold.gen1: q = dir (raw - q0) tick, error e = dir (cmd - raw), duty
u = -dir load, as m5_compare).

Static model, per window w, on the canonical MuJoCo geometry (twin.load_model, base mass as the session's robot):
    bias_k(q_w) = sum_l (J_l^T F_l)_k + [k actuated] (kappa_type(k) u_k + phi_k),    k = 0..17
    bias = gravity generalized force (zero velocity); J_l the translational Jacobian at foot l's contact reference
    (twin.contact_reference on the measured joints); F_l the floor reaction (world frame, floor normal +z);
    kappa the N*m per permille of duty (the servo's torque per PresentLoad unit, unknown); phi_k a bounded
    torque the duty does not explain (gear/closed-chain static friction, deadband, unmodelled terms).
The floor is the plane of the four measured contact references (flat, level floor; the IMU, mount offset removed,
is reported only as a check). F_z >= 0, |F_x|, |F_y| <= MU F_z.

`extract` (MuJoCo venv): per window the medians and the geometry -> npz.  `solve` (python3 with scipy): rank and
conditioning, least squares (phi = 0), the minimum friction bound making the data consistent, and the interval of
each kappa (LP) for friction bounds phi_max; C = tick / (kappa * KP), KP the regulator's slope (permille/tick).
Diagnostic only: no firmware, model or candidate is changed.

`quality` (MuJoCo venv): per window validity, read age, speed, RAW drift, cmd changes, voltage, temperature, IMU
age; the regulator law (joint-space error -> PresentLoad) and the current register against the duty.

Usage: c_ident_hold.py quality <hw session dir>
       c_ident_hold.py extract <hw session dir> <out.npz> [base_mass_kg=1.707] [com_shift_mm=x,y,z]
       c_ident_hold.py solve <npz> [<npz> ...]
"""
import json
import math
import pathlib
import sys

import numpy as np

HERE = pathlib.Path(__file__).resolve()
TICK = 2 * math.pi / 4096
KP = 8.0  # permille per tick: the regulator law read from the HOLD telemetry (u = KP (|e| + 1), 0 for |e| <= 1)
MU = 1.0  # friction cone used as a bound only (generous; the twin uses 0.8)
TYPE = np.array([0, 1, 2] * 4)  # HIP, UPPER, LOWER per slot
NAMES = ("HIP", "UPPER", "LOWER")


def hold_windows(hw_dir, with_current=False):
    """The four HOLD windows of m6_dev7 (2 s after arrival), m5_hw snapshots (+ the current register if asked)."""
    sys.path.insert(0, str(HERE.parents[2]))
    import m5_hw
    import m6_dev7
    if with_current:
        orig = m5_hw.snapshot
        def snapshot(s, host, source):
            o = orig(s, host, source)
            o["current"] = np.array([x.get("current", -1) for x in s["joints"][:12]], float)
            return o
        m5_hw.snapshot = snapshot
    files, snaps, _ = m5_hw.load([hw_dir])
    boot = max(m5_hw.boots(snaps), key=len)
    edges = [m6_dev7.HOLD_FROM] + [t for t, _ in m6_dev7.HOLD_CMDS] + [m6_dev7.HOLD_TO]
    heights = [120] + [int(round(h * 1e3)) for _, h in m6_dev7.HOLD_CMDS]
    return [(h, [s for s in boot if a * 1e3 <= s["now_ms"] <= b * 1e3 and s["state"] == "STANDING" and s["phase"] == "HOLD" and s["torque"]][2:])
            for a, b, h in zip(edges, edges[1:], heights)]


def quality(hw_dir):
    import collections
    import m2_stand_hold
    wins = hold_windows(hw_dir, with_current=True)
    q0, direction = m2_stand_hold.gen1()
    law, cur = collections.Counter(), collections.defaultdict(list)
    for h, hw in wins:
        raw = np.array([s["raw"] for s in hw]); cmd = np.array([s["cmd"] for s in hw]); age = np.array([s["age_ms"] for s in hw])
        spd = np.array([s["speed"] for s in hw]); qa = [s["now_ms"] - s["quat_ms"] for s in hw if s["quat_ms"]]
        print(f"HOLD {h} mm: n {len(hw)} valid {all(s['valid'] for s in hw)} read age med/max {np.median(age):.0f}/{age.max():.0f} ms | speed != 0: "
              f"{int((spd != 0).sum())}/{spd.size} | RAW range max {int((raw.max(0) - raw.min(0)).max())} tick | cmd range max {int((cmd.max(0) - cmd.min(0)).max())} tick | "
              f"V {np.mean([np.mean(s['voltage_dv']) for s in hw]) / 10:.2f} | temp {min(min(s['temp']) for s in hw)}-{max(max(s['temp']) for s in hw)} C | "
              f"IMU age med {np.median(qa):.0f} ms | dt med {np.median(np.diff([s['now_ms'] for s in hw])):.0f} ms")
        for s in hw:
            e = direction * (s["cmd"] - s["raw"]); u = -direction * s["load"]
            for j in range(12):
                law[(int(e[j]), int(u[j]))] += 1
                cur[int(abs(u[j]))].append(s["current"][j])
    print("regulator law, joint space: error e = q_cmd - q (ticks) -> duty u (permille) x count")
    by = collections.defaultdict(list)
    for (e, u), c in law.items():
        by[e].append(f"{u:+d}x{c}")
    for e in sorted(by):
        print(f"  e {e:+3d}: {', '.join(sorted(by[e]))}")
    print("current register against |duty|: " + ", ".join(f"{u}:{np.median(c):.0f}(n{len(c)})" for u, c in sorted(cur.items())))


def extract(hw_dir, out, args):
    sys.path.insert(0, str(HERE.parents[2]))
    import mujoco
    import m2_stand_hold
    import m6_dev7
    import twin
    opt = dict(a.split("=", 1) for a in args)
    base_mass = float(opt.get("base_mass_kg", 1.707))
    shift = np.array([float(x) for x in opt.get("com_shift_mm", "0,0,0").split(",")]) * 1e-3
    m, d = twin.load_model()
    b = m.body("base_link").id
    m.body_inertia[b] *= base_mass / m.body_mass[b]
    m.body_mass[b] = base_mass
    m.body_ipos[b] += shift
    mujoco.mj_setConst(m, d)
    centers = twin.site_ids(m, "foot_cylinder_center")
    feet = np.array([m.geom(f"{l}_foot_cylinder").id for l in twin.LEGS])
    foot_bodies = np.array([m.body(f"{l}_foot_link").id for l in twin.LEGS])
    q0, direction = m2_stand_hold.gen1()
    res = dict(mass_kg=float(m.body_subtreemass[0]), com_shift_mm=(shift * 1e3).tolist(), windows=[])
    arrays = {}
    for w, (h, hw) in enumerate(hold_windows(hw_dir)):
        raw = np.array([s["raw"] for s in hw]); cmd = np.array([s["cmd"] for s in hw]); load = np.array([s["load"] for s in hw])
        q = direction * (np.median(raw, 0) - q0) * TICK
        e = direction * (cmd - raw)  # ticks, joint space, per snapshot
        u = -direction * load        # permille, joint space, per snapshot
        # floor = plane of the four contact references (body frame), found iteratively (the reference depends on the normal)
        n = np.array([0, 0, 1.0])
        for _ in range(5):
            d.qpos[:] = 0; d.qpos[3] = 1; d.qpos[7:19] = q
            mujoco.mj_kinematics(m, d)
            p = twin.contact_reference(d, centers, feet, normal=n)
            A = np.c_[p[:, :2], np.ones(4)]
            (ca, cb, cc), *_ = np.linalg.lstsq(A, p[:, 2], rcond=None)
            n = np.array([-ca, -cb, 1.0]) / math.hypot(1, math.hypot(ca, cb))
        plane_res = float(np.abs(A @ [ca, cb, cc] - p[:, 2]).max())
        # base rotation taking the body-frame floor normal n to world +z (no yaw)
        z = np.array([0, 0, 1.0]); v = np.cross(n, z); s_, c_ = np.linalg.norm(v), n @ z
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + K + K @ K * ((1 - c_) / s_ ** 2) if s_ > 1e-12 else np.eye(3)
        quat = np.zeros(4); mujoco.mju_mat2Quat(quat, R.flatten())
        d.qpos[:] = 0; d.qpos[3:7] = quat; d.qpos[7:19] = q; d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        bias = d.qfrc_bias.copy()
        pw = twin.contact_reference(d, centers, feet)  # world frame, normal +z
        Jt = np.zeros((m.nv, 12))
        for l in range(4):
            jp = np.zeros((3, m.nv)); mujoco.mj_jac(m, d, jp, None, pw[l], foot_bodies[l])
            Jt[:, 3 * l:3 * l + 3] = jp.T
        # IMU check: body roll/pitch from the quaternion minus the mount offset vs the feet plane
        quats = [s["quat"] for s in hw if s["quat"]]
        imu = None
        if quats:
            qm = np.median(np.array(quats, float), 0); qm /= np.linalg.norm(qm)
            Ri = np.zeros(9); mujoco.mju_quat2Mat(Ri, qm); Ri = Ri.reshape(3, 3)
            g_imu = Ri.T @ z  # world up in the IMU frame
            off = np.array(m6_dev7.VARIANTS["dev7_ground"]["imu_offset_mrad"]) * 1e-3
            imu = dict(roll_pitch_mrad_imu=[1e3 * math.atan2(g_imu[1], g_imu[2]), 1e3 * -math.asin(max(-1, min(1, g_imu[0])))],
                       mount_offset_mrad=(off * 1e3).tolist())
        res["windows"].append(dict(height_mm=h, n=len(hw), plane_residual_mm=1e3 * plane_res,
                                   floor_normal_body=n.tolist(), body_roll_pitch_mrad=[1e3 * math.atan2(n[1], n[2]), 1e3 * -math.asin(n[0])],
                                   imu=imu, e_median=np.median(e, 0).tolist(), u_median=np.median(u, 0).tolist(),
                                   u_range=(u.max(0) - u.min(0)).tolist(), q_rad=q.tolist(), contact_world_mm=(1e3 * pw).tolist(),
                                   com_world_mm=(1e3 * d.subtree_com[0]).tolist()))
        arrays[f"w{w}_Jt"] = Jt; arrays[f"w{w}_bias"] = bias; arrays[f"w{w}_u"] = u; arrays[f"w{w}_e"] = e
    np.savez(out, meta=json.dumps(res), **arrays)
    print(json.dumps({k: v for k, v in res.items() if k != "windows"}), flush=True)
    for w in res["windows"]:
        print(f"  HOLD {w['height_mm']} mm n {w['n']}: floor plane residual {w['plane_residual_mm']:.2f} mm, body roll/pitch on the feet "
              f"{w['body_roll_pitch_mrad'][0]:.1f}/{w['body_roll_pitch_mrad'][1]:.1f} mrad, IMU {w['imu']['roll_pitch_mrad_imu'] if w['imu'] else None} "
              f"| e {[int(x) for x in w['e_median']]} | u {[int(x) for x in w['u_median']]} | COM {[round(x, 1) for x in w['com_world_mm']]}", flush=True)


def system(z, types=(0, 1, 2), shared=True, use=(0, 1, 2), windows=None):
    """Equalities A x = b, x = [F_w (12) , phi_w (12)] per window + kappa (1 shared or one per type in `types`)."""
    meta = json.loads(str(z["meta"]))
    W = range(len(meta["windows"])) if windows is None else windows
    nk = 1 if shared else len(types)
    blocks, rhs = [], []
    nW = len(W)
    for i, w in enumerate(W):
        Jt, bias, u = z[f"w{w}_Jt"], z[f"w{w}_bias"], np.median(z[f"w{w}_u"], 0)
        A = np.zeros((18, 24 * nW + nk))
        A[:, 24 * i:24 * i + 12] = Jt
        for k in range(12):
            if TYPE[k] not in use:  # this joint's torque is left free (no information)
                continue
            A[6 + k, 24 * i + 12 + k] = 1.0
            A[6 + k, 24 * nW + (0 if shared else types.index(TYPE[k]))] = u[k]
        rows = [r for r in range(18) if r < 6 or TYPE[r - 6] in use]
        blocks.append(A[rows]); rhs.append(bias[rows])
    return np.vstack(blocks), np.concatenate(rhs), nW, nk


def solve(paths):
    from scipy.optimize import linprog
    out = {}
    for path in paths:
        z = np.load(path, allow_pickle=False)
        meta = json.loads(str(z["meta"]))
        print(f"== {path}: mass {meta['mass_kg']:.3f} kg, COM shift {meta['com_shift_mm']} mm")
        rep = out.setdefault(str(path), {})
        for label, kw in (("kappa shared, UPPER+LOWER equations (HIP free)", dict(shared=True, use=(1, 2))),
                          ("kappa UPPER / LOWER separate (HIP free)", dict(shared=False, types=(1, 2), use=(1, 2))),
                          ("kappa shared, all 12 joints (HIP with its duty)", dict(shared=True, use=(0, 1, 2))),
                          ("kappa HIP / UPPER / LOWER separate", dict(shared=False, types=(0, 1, 2), use=(0, 1, 2)))):
            A, b, nW, nk = system(z, **kw)
            ncol = A.shape[1]
            # rank and conditioning with phi = 0 (columns of F and kappa only)
            keep = [c for c in range(ncol) if not (c < 24 * nW and (c % 24) >= 12)]
            A0 = A[:, keep]
            sc = np.linalg.norm(A0, axis=0); sc[sc == 0] = 1
            sv = np.linalg.svd(A0 / sc, compute_uv=False)
            rank = int((sv > sv[0] * 1e-9).sum())
            # identifiability of kappa with phi = 0: is any kappa column inside the span of the F columns?
            kcols = list(range(A0.shape[1] - nk, A0.shape[1]))
            AF = (A0 / sc)[:, :A0.shape[1] - nk]
            Pf = AF @ np.linalg.pinv(AF)
            kappa_ident = [float(np.linalg.norm((np.eye(len(b)) - Pf) @ (A0 / sc)[:, c])) for c in kcols]
            x, *_ = np.linalg.lstsq(A0, b, rcond=None)
            r = A0 @ x - b
            kap = x[-nk:]
            # LP: minimum friction bound making the data consistent (kappa free >= 0)
            nv = ncol + 1  # + t (phi bound)
            bounds = [(None, None)] * ncol + [(0, None)]
            for c in range(ncol - nk, ncol):
                bounds[c] = (0, None)
            Aub, bub = [], []
            for i in range(nW):
                for l in range(4):
                    fx, fy, fz = 24 * i + 3 * l, 24 * i + 3 * l + 1, 24 * i + 3 * l + 2
                    row = np.zeros(nv); row[fz] = -1; Aub.append(row); bub.append(0)
                    for c in (fx, fy):
                        for sgn in (1, -1):
                            row = np.zeros(nv); row[c] = sgn; row[fz] = -MU; Aub.append(row); bub.append(0)
                for k in range(12):
                    c = 24 * i + 12 + k
                    for sgn in (1, -1):
                        row = np.zeros(nv); row[c] = sgn; row[-1] = -1; Aub.append(row); bub.append(0)
            Aub, bub = np.array(Aub), np.array(bub)
            Aeq = np.c_[A, np.zeros(len(b))]
            cobj = np.zeros(nv); cobj[-1] = 1
            lp = linprog(cobj, A_ub=Aub, b_ub=bub, A_eq=Aeq, b_eq=b, bounds=bounds, method="highs")
            phi_min = float(lp.x[-1]) if lp.status == 0 else None
            intervals = {}
            for pm in sorted({round(phi_min, 4) if phi_min is not None else 0, 0.27, 0.30, 0.35, 0.40, 0.50}):
                if phi_min is None or pm < phi_min - 1e-9:
                    continue
                bnds = list(bounds); bnds[-1] = (0, pm + 1e-9)
                iv = []
                for j in range(nk):
                    lohi = []
                    for sgn in (1, -1):
                        cj = np.zeros(nv); cj[ncol - nk + j] = sgn
                        s = linprog(cj, A_ub=Aub, b_ub=bub, A_eq=Aeq, b_eq=b, bounds=bnds, method="highs")
                        lohi.append(sgn * s.fun if s.status == 0 else (0.0 if sgn == 1 else math.inf) if s.status == 3 else None)
                    iv.append(lohi)
                intervals[f"{pm:.3f}"] = iv
            names = ["shared"] if nk == 1 else [NAMES[t] for t in kw.get("types", (0, 1, 2))]
            C = lambda k: TICK / (k * KP) if k and k > 0 and math.isfinite(k) else (math.inf if k == 0 else 0.0)
            print(f"  [{label}] equations {A0.shape[0]} unknowns(F,kappa) {A0.shape[1]} rank {rank} cond {sv[0] / sv[rank - 1]:.3g} "
                  f"smallest s.v. {sv[-1]:.2e} | kappa distance from the F span (0 = not identifiable) {[round(v, 4) for v in kappa_ident]}")
            print(f"     LS (phi=0): kappa {dict(zip(names, np.round(kap, 6).tolist()))} N*m/permille -> C {dict(zip(names, [round(C(k), 4) for k in kap]))} rad/N*m | "
                  f"residual rms {math.sqrt(np.mean(r ** 2)):.4f}, max {np.abs(r).max():.4f}")
            print(f"     minimum friction bound for consistency (with contact cone): {phi_min if phi_min is None else round(phi_min, 4)} N*m")
            for pm, iv in intervals.items():
                print(f"     phi_max {pm} N*m: " + "; ".join(f"{n_} kappa [{lo:.5f}, {hi:.5f}] -> C [{C(hi):.4f}, {C(lo):.4f}]" if lo is not None and hi is not None else f"{n_} infeasible"
                                                   for n_, (lo, hi) in zip(names, iv)))
            # leave one window out (least squares, phi = 0): stability of kappa across postures
            loo = []
            for drop in range(nW):
                keepw = [w for w in range(nW) if w != drop]
                A1, b1, nW1, _ = system(z, windows=keepw, **kw)
                k1 = [c for c in range(A1.shape[1]) if not (c < 24 * nW1 and (c % 24) >= 12)]
                x1, *_ = np.linalg.lstsq(A1[:, k1], b1, rcond=None)
                loo.append([round(C(k), 4) for k in x1[-nk:]])
            print(f"     leave-one-window-out LS C: {loo}")
            if nk == 1 and 0 in kw.get("use", ()):
                for i, w in enumerate(meta["windows"]):
                    F = x[12 * i:12 * i + 12].reshape(4, 3)  # LS columns: F of each window, then kappa
                    P = np.array(w["contact_world_mm"]) * 1e-3; c = np.array(w["com_world_mm"]) * 1e-3
                    Wn = meta["mass_kg"] * 9.81
                    M = np.vstack([np.ones(4), P[:, 0], P[:, 1]]); fmn = M.T @ np.linalg.solve(M @ M.T, [Wn, Wn * c[0], Wn * c[1]])
                    print(f"     HOLD {w['height_mm']} mm F_z LS (LF RF RH LH) {np.round(F[:, 2], 1).tolist()} N vs minimum-norm model {np.round(fmn, 1).tolist()} N")
            rep[label] = dict(leave_one_out_C=loo, equations=int(A0.shape[0]), unknowns=int(A0.shape[1]), rank=rank, cond=float(sv[0] / sv[rank - 1]),
                              kappa_span_distance=kappa_ident, ls_kappa=kap.tolist(), ls_residual_rms=float(math.sqrt(np.mean(r ** 2))),
                              phi_min=phi_min, intervals=intervals)
    return out


if __name__ == "__main__":
    if sys.argv[1] == "quality":
        sys.path.insert(0, str(HERE.parents[2]))
        quality(sys.argv[2])
    elif sys.argv[1] == "extract":
        extract(sys.argv[2], sys.argv[3], sys.argv[4:])
    else:
        result = solve(sys.argv[2:])
        pathlib.Path(sys.argv[2]).with_suffix(".solve.json").write_text(json.dumps(result, indent=1) + "\n")
