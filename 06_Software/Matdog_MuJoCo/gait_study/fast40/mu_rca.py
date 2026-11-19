#!/usr/bin/env python3
"""RCA 01 (2026-11-13): why the twin walks the FAST law backwards at foot friction mu 1.0. Diagnostics only.

run: one fast_fwd walk of a build through m6_gait.run (same twin dev7_ground, same scenario, same metrics: the
report must equal the archived one), with dynamics.Twin.probe sampling every 1 ms record:
  - per foot, its floor contacts summed in the WORLD frame: mj_contactForce gives the force in the contact
    frame (rows of contact.frame: normal from geom1 to geom2, tangent 1, tangent 2) acting on geom2; the force
    on the foot is +frame^T f if the foot is geom2, -frame^T f if geom1 (checked against cfrc_ext);
  - their count, friction, condim, penetration (contact dist), and the friction use |Ft| / (mu Fn);
  - the velocity of the foot's contact reference (lowest tread point, twin.contact_reference): slip when loaded;
  - non-foot floor contacts (geoms, force); the servo profile setpoint and goal; COM velocity; solver iterations.
Everything goes to an npz (the bridge rows' columns used here, the twin record, the probe).

Counterfactuals (--cf, diagnostics only, one condition each, never a configuration):
  swing_mu=<mu>   a foot in the law's swing has friction <mu> (floor friction 0: a contact takes the larger of
                  its two geoms' friction, so the foot's own value applies), stance feet keep the run's mu.
  timestep=<s>    the physics step (numerical check of integrator and contact discretization).
  solref=<s>      the contact time constant of feet and floor (model default 0.004): sensitivity to the contact
                  stiffness behind the ~200 Hz chatter, never a calibration.
  swing_contact=0 a foot in the law's swing does not collide with the floor ("ghost" swing: the three stance feet
                  carry the body, and the foot's geometric distance, < 0 below the floor, is the clearance deficit;
                  at touchdown the foot collides again from wherever it is).

analyse: the tables of the RCA from those npz (numpy only); plot: its figures (matplotlib, any python).

Usage: mu_rca.py run <cosim> <period:stride_mm:height_mm> <label> <out.npz> [--fast-joy l:y:b] [--set k=v ...]
                 [--cf name=value] [--compare archived.json]
       mu_rca.py analyse <out.json> <runs dir> <cf dir>
       mu_rca.py plot <out dir> <runs dir> <contacts dir>
       mu_rca.py footlift <cf name=value> <foot_lift_rca.py arguments>   (C inside a counterfactual)
       mu_rca.py clearance <foot_lift_rca JSON ...>   (the clearance table, full swings only)
       mu_rca.py contacts <cosim> <period:stride_mm:height_mm> <mu> <from s> <until s> <out.npz>   (per contact point)
"""
import json
import math
import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))  # Matdog_MuJoCo


def _sim_modules():
    """The twin's modules, only for `run` (analyse and plot read npz files)."""
    global mujoco, cosim, dynamics, m6_dev7, m6_gait, twin, servo
    import mujoco
    import servo
    import cosim
    import dynamics
    import m6_dev7
    import m6_gait
    import twin

ROW_KEYS = (["t_s", "walk_cycles", "drive_speed_step_s", "drive_acc_100step_s2", "body_x", "body_y", "body_z", "body_yaw", "imu_roll", "imu_pitch"]
            + [f"raw{j}" for j in range(12)] + [f"stance{l}" for l in range(4)] + [f"foot{l}_{a}" for l in range(4) for a in "xyz"])


class Probe:
    def __init__(self):
        self.s = {k: [] for k in ("F", "n", "mu", "dim", "dist", "vref", "use", "Fext", "other_F", "other_n", "sp", "goal", "vcom", "niter", "ctrl")}
        self.other_geoms = set()
        self.f6 = np.zeros(6)
        self.v6 = np.zeros(6)
        self.swing_mu = None  # counterfactual swing_mu: (mu_swing, mu_stance, stance flags of the release in force)
        self.ghost = None  # counterfactual swing_contact=0: the feet's own contype
        # counterfactual swing_leg_contact=0 (2026-11-13): the whole distal leg of a law-swing leg (lower-leg hull and
        # foot) does not collide. swing_contact=0 frees only the foot cylinder, and at the end of the front swings
        # the sunken leg's lower-leg hull then touched the floor (RCA 01 D4: 23.7 % of the samples, ~14 N)
        self.ghost_leg = None
        self.stance = np.ones(4, int)

    def __call__(self, tw):
        m, d, s = tw.m, tw.d, self.s
        nc = d.ncon
        feet = list(tw.feet)
        F, n = np.zeros((4, 3)), np.zeros(4)
        mu, dim, dist = np.full(4, np.nan), np.zeros(4), np.full(4, np.nan)
        oF, on = np.zeros(3), 0
        for i in range(nc):
            g1, g2 = int(d.contact.geom1[i]), int(d.contact.geom2[i])
            if tw.floor not in (g1, g2):
                continue
            mujoco.mj_contactForce(m, d, i, self.f6)
            fw = d.contact.frame[i].reshape(3, 3).T @ self.f6[:3]  # force on geom2, world
            other = g2 if g1 == tw.floor else g1
            f_on = fw if other == g2 else -fw
            if other in feet:
                l = feet.index(other)
                F[l] += f_on
                n[l] += 1
                mu[l] = d.contact.friction[i][0]
                dim[l] = d.contact.dim[i]
                dist[l] = np.nanmin([dist[l], d.contact.dist[i]])
            else:
                oF += f_on
                on += 1
                self.other_geoms.add(m.geom(other).name or f"geom{other}")
        ref = tw.rec["feet"][-1]
        vref = np.zeros((4, 3))
        for l, g in enumerate(feet):
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_GEOM, g, self.v6, 0)  # [rot, lin] at the geom center, world axes
            vref[l] = self.v6[3:] + np.cross(self.v6[:3], ref[l] - d.geom_xpos[g])
        ft = np.linalg.norm(F[:, :2], axis=1)
        use = np.where(F[:, 2] > dynamics.CONTACT_N, ft / (np.nan_to_num(mu, nan=1.0) * np.maximum(F[:, 2], 1e-9)), np.nan)
        mujoco.mj_subtreeVel(m, d)
        s["F"].append(F); s["n"].append(n); s["mu"].append(mu); s["dim"].append(dim); s["dist"].append(dist)
        s["vref"].append(vref); s["use"].append(use); s["Fext"].append(d.cfrc_ext[tw.foot_bodies, 3:6].copy())
        s["other_F"].append(oF); s["other_n"].append(on)
        s["sp"].append(tw.servos.sp.copy()); s["goal"].append(tw.servos.goal.copy())
        s["vcom"].append(d.subtree_linvel[tw.base].copy()); s["niter"].append(int(np.sum(d.solver_niter)))
        s["ctrl"].append(d.ctrl.copy())

    def arrays(self):
        return {f"probe_{k}": np.array(v) for k, v in self.s.items()}


def install(probe, cf):
    """dynamics.Twin with the probe (and a counterfactual) attached; physics untouched unless --cf."""
    base_init, base_release = dynamics.Twin.__init__, dynamics.Twin.release

    def init(self, *a, **k):
        base_init(self, *a, **k)
        self.probe = probe
        if "swing_mu" in cf:
            probe.swing_mu = (float(cf["swing_mu"]), float(self.m.geom_friction[self.floor, 0]))
            self.m.geom_friction[self.floor, 0] = 0.0
        if "timestep" in cf:
            self.m.opt.timestep = self.dt = float(cf["timestep"])
        if "solref" in cf:
            self.m.geom_solref[list(self.feet) + [self.floor], 0] = float(cf["solref"])
        if cf.get("swing_contact") == "0":
            probe.ghost = self.m.geom_contype[self.feet].copy()
        if "stance_gain" in cf or "swing_gain" in cf:
            # counterfactual stance_gain / swing_gain (2026-11-13): the servo P gain multiplied on the joints of the
            # law-stance / law-swing legs only (diagnostic: compliance of the support against tracking in flight)
            probe.gain = (float(cf.get("stance_gain", 1)), float(cf.get("swing_gain", 1)))
            if not getattr(servo.Servos, "_kmul_patched", False):
                base_duty = servo.Servos.duty

                def duty(srv, q):
                    k = getattr(srv, "kmul", None)
                    if k is None:
                        return base_duty(srv, q)
                    saved = srv.sp
                    srv.sp = q + (saved - q) * k  # error scaled by the per-joint factor (deadband on the scaled error)
                    try:
                        return base_duty(srv, q)
                    finally:
                        srv.sp = saved
                servo.Servos.duty, servo.Servos._kmul_patched = duty, True
        if cf.get("swing_leg_contact") == "0":
            legs = [[g for g in range(self.m.ngeom) if self.m.geom_bodyid[g] in (self.m.body(f"{l}_lower_leg_link").id, self.m.body(f"{l}_foot_link").id)]
                    for l in twin.LEGS]
            probe.ghost_leg = (legs, {g: int(self.m.geom_contype[g]) for gs in legs for g in gs})

    def release(self, goal, speed, acc, torque_on=None, record=True):
        if probe.swing_mu is not None:
            mu_sw, mu_st = probe.swing_mu
            self.m.geom_friction[self.feet, 0] = np.where(probe.stance == 0, mu_sw, mu_st)
        if probe.ghost is not None:
            self.m.geom_contype[self.feet] = np.where(probe.stance == 0, 0, probe.ghost)
        if getattr(probe, "gain", None) is not None and getattr(self, "servos", None) is not None:
            self.servos.kmul = np.repeat(np.where(probe.stance == 1, probe.gain[0], probe.gain[1]), 3)
        if probe.ghost_leg is not None:
            legs, own = probe.ghost_leg
            for l, gs in enumerate(legs):
                for g in gs:
                    self.m.geom_contype[g] = 0 if probe.stance[l] == 0 else own[g]
        return base_release(self, goal, speed, acc, torque_on, record)

    bridge_record = cosim.Bridge.record

    def record(self, reply):  # the stance flags of the release in force reach the counterfactual through the bridge rows
        r = bridge_record(self, reply)
        if "stance0" in r:
            probe.stance = np.array([int(r[f"stance{l}"]) for l in range(4)])
        return r

    dynamics.Twin.__init__, dynamics.Twin.release, cosim.Bridge.record = init, release, record


def run(argv):
    _sim_modules()
    cfs = dict(argv[i + 1].split("=") for i, x in enumerate(argv) if x == "--cf")
    compare = next((argv[i + 1] for i, x in enumerate(argv) if x == "--compare"), None)
    sets = [argv[i + 1] for i, x in enumerate(argv) if x == "--set"]
    if "--fast-joy" in argv:
        i = argv.index("--fast-joy")
        lat_mm, yaw_rad, back_mm = (float(x) for x in argv[i + 1].split(":"))
        m6_gait.FAST_JOY.update(lateral=lat_mm * 1e-3, yaw=yaw_rad, back=back_mm * 1e-3)
    binary, law, label, out = argv[1], argv[2], argv[3], pathlib.Path(argv[4])
    per, stride_mm, height_mm = (float(x) for x in law.split(":"))
    m6_gait.FAST_LAW.update(period=per, stride=stride_mm * 1e-3, height=height_mm * 1e-3)
    m6_gait.SCENARIOS.update(m6_gait.fast_scenarios(m6_gait.FAST_LAW, m6_gait.FAST_JOY))
    _, v = m6_dev7.variant_params("dev7_ground", sets)
    probe = Probe()
    install(probe, cfs)
    captured = {}
    simulate = cosim.simulate

    def capture(*a, **k):
        captured["out"] = simulate(*a, **k)
        return captured["out"]
    cosim.simulate = capture
    rep = m6_gait.run(pathlib.Path(binary), out.parent, label, "fast_fwd", v, write=False)
    rows, tw, _, _ = captured["out"]
    rec = tw.result()
    arrays = {f"rec_{k}": v_ for k, v_ in rec.items()}
    arrays.update(probe.arrays())
    for key in ROW_KEYS:
        arrays[f"row_{key}"] = np.array([float(r[key]) for r in rows])
    arrays["row_state"] = np.array([r["state"] for r in rows])
    arrays["row_imu_reason"] = np.array([r["imu_reason"] for r in rows])
    for k in ("raw", "speed", "load"):
        arrays[f"row_fb_{k}"] = np.array([[int(r[f"fb_{k}{j}"]) for j in range(12)] for r in rows])
    meta = {"label": label, "law": law, "sets": sets, "cf": cfs, "definition": v, "mu": v.get("mu", 0.8), "binary": str(binary),
            "fw_tree": str(twin.FW_TREE), "other_floor_geoms": sorted(probe.other_geoms), "mujoco": mujoco.__version__,
            "opt": {"timestep": tw.m.opt.timestep, "noslip": int(tw.m.opt.noslip_iterations), "cone": int(tw.m.opt.cone), "impratio": float(tw.m.opt.impratio),
                    "integrator": int(tw.m.opt.integrator), "iterations": int(tw.m.opt.iterations), "tolerance": float(tw.m.opt.tolerance)},
            "foot_friction": tw.m.geom_friction[tw.feet].tolist(), "floor_friction": tw.m.geom_friction[tw.floor].tolist(),
            "foot_condim": tw.m.geom_condim[tw.feet].tolist(), "floor_condim": int(tw.m.geom_condim[tw.floor]),
            "foot_priority": tw.m.geom_priority[tw.feet].tolist(), "floor_priority": int(tw.m.geom_priority[tw.floor]),
            "foot_solref": tw.m.geom_solref[tw.feet].tolist(), "foot_solimp": tw.m.geom_solimp[tw.feet].tolist(), "npair": int(tw.m.npair),
            "warnings": {mujoco.mjtWarning(i).name: int(tw.d.warning[i].number) for i in range(len(tw.d.warning))}, "report": rep}
    if compare:
        ref = json.load(open(compare))
        mine = json.loads(json.dumps(rep, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)))
        skip = {"wall_s", "label", "fast_joy"}
        diff = [k for k in set(ref) | set(mine) if k not in skip and ref.get(k) != mine.get(k)]
        meta["compare"] = {"archived": compare, "differing_keys": diff}
        print(f"COMPARE {compare}: {'IDENTICAL' if not diff else 'DIFFERENT ' + str(diff)} (ignored {sorted(skip)})", flush=True)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, **arrays)
    out.with_suffix(".json").write_text(json.dumps(meta, indent=1, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)) + "\n")


def contacts(argv):
    """B, the first divergence at physics-step resolution: the fast_fwd walk up to <until> s (identical to the full
    run up to there), and from <from> s every floor contact POINT at every step: foot, normal force, tangential
    force, |Ft| / (mu Fn) of that point's own cone, slip speed of the foot at the point. Same twin, no counterfactual."""
    _sim_modules()
    import dataclasses
    import servo
    binary, law, mu, t_from, until, out = argv[1], argv[2], float(argv[3]), float(argv[4]), float(argv[5]), pathlib.Path(argv[6])
    per, stride_mm, height_mm = (float(x) for x in law.split(":"))
    m6_gait.FAST_LAW.update(period=per, stride=stride_mm * 1e-3, height=height_mm * 1e-3)
    _, height, events, _, _ = m6_gait.fast_scenarios(m6_gait.FAST_LAW)["fast_fwd"]
    _, v = m6_dev7.variant_params("dev7_ground", [f"mu={mu}"])
    params = dataclasses.replace(servo.Params(), **v.get("params", {}))
    rec, f6, v6 = [], np.zeros(6), np.zeros(6)

    def probe(tw):
        m, d = tw.m, tw.d
        if tw.now < t_from - 1e-9:
            return
        feet = list(tw.feet)
        for i in range(d.ncon):
            g1, g2 = int(d.contact.geom1[i]), int(d.contact.geom2[i])
            if tw.floor not in (g1, g2):
                continue
            other = g2 if g1 == tw.floor else g1
            mujoco.mj_contactForce(m, d, i, f6)
            frame = d.contact.frame[i].reshape(3, 3)
            mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_GEOM, other, v6, 0)
            vp = v6[3:] + np.cross(v6[:3], d.contact.pos[i] - d.geom_xpos[other])
            vt = vp - frame[0] * (vp @ frame[0])
            rec.append((tw.now, feet.index(other) if other in feet else -1, f6[0], float(np.hypot(f6[1], f6[2])), float(d.contact.friction[i][0]),
                        float(d.contact.dist[i]), float(np.linalg.norm(vt)), *d.contact.pos[i]))
    base_init, base_begin = dynamics.Twin.__init__, dynamics.Twin.begin

    def init(self, *a, **k):
        base_init(self, *a, **k)
        self.probe = probe

    def begin(self, q0, speed, acc, record_dt=0.001):
        base_begin(self, q0, speed, acc, record_dt=self.dt)  # every physics step (records only read the state)
    dynamics.Twin.__init__, dynamics.Twin.begin = init, begin
    imu = tuple(1e-3 * x for x in v["imu_offset_mrad"])
    rows, tw, _, _ = cosim.simulate(binary, [(0.5, "SELECT FAST")] + events, until, params, v["mu"], None, height, None, imu, v.get("model"))
    r = tw.result()
    stance = np.array([[int(row[f"stance{l}"]) for l in range(4)] for row in rows])
    np.savez_compressed(out, contacts=np.array(rec), t=r["t"], pos=r["pos"], q=r["q"], stance=stance, row_t=np.array([float(x["t_s"]) for x in rows]))


LEG = ("LF", "RF", "RH", "LH")
TICK = 2 * math.pi / 4096
WINDOW = (11.0, 31.0)  # m6_gait fast_fwd measured window
DT = 1e-3  # record interval


def load(path):
    """An npz of `run` with derived series: release index in force, law stance flags, yaw/roll/pitch, heading."""
    z = np.load(path)
    D = {k: z[k] for k in z.files}
    D["meta"] = json.loads(pathlib.Path(path).with_suffix(".json").read_text())
    t = D["rec_t"]
    D["idx"] = np.clip(np.ceil(t / 0.02 - 1e-9).astype(int) - 1, 0, len(D["row_t_s"]) - 1)
    D["st"] = np.stack([D[f"row_stance{l}"] for l in range(4)], 1)[D["idx"]].astype(int)
    w, x, y, zq = D["rec_quat"].T
    D["yaw"] = np.unwrap(np.arctan2(2 * (w * zq + x * y), 1 - 2 * (y * y + zq * zq)))
    D["roll"] = np.arctan2(2 * (w * x + y * zq), 1 - 2 * (x * x + y * y))
    D["pitch"] = np.arcsin(np.clip(2 * (w * y - zq * x), -1, 1))
    D["h"] = np.stack([np.cos(D["yaw"]), np.sin(D["yaw"])], 1)  # instantaneous heading
    D["m"] = (t >= WINDOW[0]) & (t <= WINDOW[1]) & (D["st"] >= 0).all(1)
    return D


def heading(D, v):
    """x component of world xy vectors along the instantaneous heading."""
    return v[..., 0] * D["h"][:, 0, None] + v[..., 1] * D["h"][:, 1, None] if v.ndim == 3 else v[:, 0] * D["h"][:, 0] + v[:, 1] * D["h"][:, 1]


def audit(D):
    """A: the twin speed of m6_gait (base origin, endpoints, start heading) against the alternatives."""
    t, m = D["rec_t"], D["m"]
    i = np.flatnonzero(m)
    i0, i1 = i[0], i[-1]
    c, s = math.cos(D["yaw"][i0]), math.sin(D["yaw"][i0])
    T = t[i1] - t[i0]
    start = lambda d: d[0] * c + d[1] * s
    wc = D["row_walk_cycles"][D["idx"]]
    per_cycle = []
    k = np.flatnonzero(np.diff(np.floor(wc[i0:i1 + 1]))) + i0 + 1  # cycle boundaries inside the window
    for a, b in zip(k[:-1], k[1:]):
        per_cycle.append(1e3 * start(D["rec_com"][b] - D["rec_com"][a]))
    return {"window_s": [float(t[i0]), float(t[i1])], "gaps": int(np.sum(np.diff(i) > 1)), "cycles": float(wc[i1] - wc[i0]),
            "m6_gait_mm_s": float(D["meta"]["report"]["travel"]["twin_speed_mm_s"][0]),
            "base_origin_mm_s": 1e3 * start(D["rec_pos"][i1] - D["rec_pos"][i0]) / T,
            "com_mm_s": 1e3 * start(D["rec_com"][i1] - D["rec_com"][i0]) / T,
            "int_base_vel_mm_s": 1e3 * np.sum(D["rec_vel"][i0:i1, 0] * c + D["rec_vel"][i0:i1, 1] * s) * DT / T,
            "int_com_vel_mm_s": 1e3 * np.sum(D["probe_vcom"][i0:i1, 0] * c + D["probe_vcom"][i0:i1, 1] * s) * DT / T,
            "int_com_vel_inst_heading_mm_s": 1e3 * np.sum(heading(D, D["probe_vcom"])[i0:i1]) * DT / T,
            "yaw_change_deg": float(np.degrees(D["yaw"][i1] - D["yaw"][i0])),
            "com_dx_per_full_cycle_mm": per_cycle, "negative_cycles": int(np.sum(np.array(per_cycle) < 0)),
            "force_vs_cfrc_ext_max_abs_n": float(np.abs(D["probe_F"] - D["probe_Fext"]).max()),
            "sum_fz_mean_n": float(D["probe_F"][m, :, 2].sum(1).mean()), "other_floor_contacts_max": int(D["probe_other_n"][m].max()),
            "solver_iter_p50_max": [float(np.median(D["probe_niter"][m])), int(D["probe_niter"][m].max())],
            "contact_mu": sorted({float(x) for x in np.unique(D["probe_mu"][m]) if np.isfinite(x)}),
            "contact_dim": sorted({int(x) for x in np.unique(D["probe_dim"][m]) if x > 0})}


def phases(D):
    """B: COM displacement per cycle by gait phase (which leg is in the law's swing), and per foot and phase the
    contact: loaded share, normal and heading-frame tangential force, impulse, slip, friction use."""
    m, st = D["m"], D["st"]
    wc = D["row_walk_cycles"][D["idx"]]
    i = np.flatnonzero(m)
    cyc = wc[i[-1]] - wc[i[0]]
    vcom = heading(D, D["probe_vcom"])
    nsw = (st == 0).sum(1)
    lab = np.where(nsw == 0, 4, np.argmax(st == 0, 1))
    lab[nsw > 1] = 5
    names = LEG + ("4-stance", "multi")
    com = {names[k]: {"dx_mm_per_cycle": 1e3 * float(np.sum(vcom[m & (lab == k)])) * DT / cyc, "s_per_cycle": float(np.sum(m & (lab == k))) * DT / cyc}
           for k in range(6) if np.any(m & (lab == k))}
    F = D["probe_F"]
    Fn, Fx = F[..., 2], heading(D, F)
    vx, vt = heading(D, D["probe_vref"]), np.linalg.norm(D["probe_vref"][..., :2], axis=2)
    loaded = Fn > 0.1
    feet = {}
    for l in range(4):
        for phase, code in (("swing", 0), ("stance", 1)):
            sel = m & (st[:, l] == code)
            L = sel & loaded[:, l]
            u = D["probe_use"][L, l]
            feet[f"{LEG[l]} {phase}"] = {
                "loaded_share": float(L.sum() / max(1, sel.sum())), "fn_mean_n": float(Fn[sel, l].mean()), "fx_mean_n": float(Fx[sel, l].mean()),
                "impulse_x_ns_per_cycle": float(np.sum(Fx[sel, l]) * DT / cyc),
                "slip_vx_loaded_mm_s": 1e3 * float(vx[L, l].mean()) if L.any() else None,
                "sliding_share_loaded": float(np.mean(vt[L, l] > 2e-3)) if L.any() else None,
                "use_p50_p90": [float(np.nanpercentile(u, 50)), float(np.nanpercentile(u, 90))] if L.any() else None}
    return {"cycles": float(cyc), "com_dx_mm_per_cycle": 1e3 * float(np.sum(vcom[m])) * DT / cyc, "by_phase": com, "feet": feet}


def joints(D):
    """B/C: front and hind UPPER/LOWER error split in profile (setpoint - goal) and P loop (q - setpoint), duty."""
    m, st = D["m"], D["st"]
    goal, sp, q, duty = D["probe_goal"], D["probe_sp"], D["rec_q"], D["rec_duty"]
    # saturation against the run's own duty clamp (D5 raises it to 2); above_unit_share: |duty| >= 1, the
    # standard clamp, which in D5 means "beyond the normal range", not saturation (Codex review, 2026-11-13)
    lim = float(D["meta"]["report"]["params"].get("torque_limit", 1.0))
    out = {}
    for pair, legs in (("front", (0, 1)), ("hind", (2, 3))):
        for k, jn in ((1, "UPPER"), (2, "LOWER")):
            for phase, code in (("swing", 0), ("stance", 1)):
                sel = [(m & (st[:, l] == code), 3 * l + k) for l in legs]
                cat = lambda f: np.concatenate([f(s, j) for s, j in sel])
                tot, prof = cat(lambda s, j: q[s, j] - goal[s, j]) / TICK, cat(lambda s, j: sp[s, j] - goal[s, j]) / TICK
                loop, du = cat(lambda s, j: q[s, j] - sp[s, j]) / TICK, cat(lambda s, j: np.abs(duty[s, j]))
                p = lambda x: [float(np.percentile(np.abs(x), 50)), float(np.percentile(np.abs(x), 95))]
                out[f"{pair} {jn} {phase}"] = {"total_abs_p50_p95": p(tot), "profile_abs_p50_p95": p(prof), "loop_abs_p50_p95": p(loop),
                                               "duty_permille_p50_p95": [1e3 * float(np.percentile(du, 50)), 1e3 * float(np.percentile(du, 95))],
                                               "saturated_share": float(np.mean(du >= .999 * lim)), "above_unit_share": float(np.mean(du >= .999)),
                                               "duty_clamp": lim}
    return out


def body(D):
    m = D["m"]
    return {"z_mean_mm": 1e3 * float(D["rec_pos"][m, 2].mean()), "z_law_mean_mm": 1e3 * float(D["row_body_z"][D["idx"]][m].mean()),
            "pitch_mean_p2p_mrad": [1e3 * float(D["pitch"][m].mean()), 1e3 * float(np.ptp(D["pitch"][m]))],
            "roll_p2p_mrad": 1e3 * float(np.ptp(D["roll"][m])), "vz_rms_mm_s": 1e3 * float(np.std(D["rec_vel"][m, 2]))}


def chatter(D):
    """The 1 ms contact state: loaded-feet count shares, sum Fz percentiles and spectrum, on/off transitions."""
    m, st = D["m"], D["st"]
    Fz = D["probe_F"][..., 2]
    L = Fz > 0.1
    tot = Fz.sum(1)
    x = tot[m] - tot[m].mean()
    f = np.fft.rfftfreq(len(x), DT)
    P = np.abs(np.fft.rfft(x)) ** 2
    trans = {}
    for l in range(4):
        for phase, code in (("stance", 1), ("swing", 0)):
            s = m & (st[:, l] == code)
            trans[f"{LEG[l]} {phase}"] = float(np.sum(np.abs(np.diff((L[:, l] & s).astype(int)))) / (s.sum() * DT))
    return {"loaded_feet_count_share_0_to_4": [float(np.mean(L[m].sum(1) == k)) for k in range(5)],
            "sum_fz_p01_p50_p99_n": [float(np.percentile(tot[m], q)) for q in (1, 50, 99)],
            "sum_fz_spectrum_peak_hz": float(f[np.argmax(P * (f > 5))]), "sum_fz_power_above_100hz": float(P[f >= 100].sum() / P[f > 0].sum()),
            "on_off_transitions_per_s": trans}


def stance_triangle(D):
    """COM margin to the triangle of the three LAW-stance feet (twin contact references) while one leg swings.
    Geometric availability only: it says nothing on which feet carry the body (Codex review, 2026-11-13: at
    11.010 s of B66 mu 1.0 the margin is +14 mm while the only loaded foot is the law-swing RF). The support
    actually transmitted is in support()."""
    import dynamics as dyn
    m, st = D["m"], D["st"]
    out = {}
    for l in range(4):
        sel = np.flatnonzero(m & (st[:, l] == 0) & ((st == 0).sum(1) == 1))[::5]
        mg = np.array([dyn.support_margin(D["rec_feet"][i][st[i] == 1], D["rec_com"][i][:2]) for i in sel])
        out[LEG[l]] = {"p05_p50_p95_mm": [1e3 * float(np.percentile(mg, q)) for q in (5, 50, 95)], "share_negative": float(np.mean(mg < 0))}
    return out


def support(D):
    """Support actually transmitted in the twin (1 ms samples, Fz > 0.1 N), against the law's schedule: share of
    samples with no loaded foot (m6_gait drops them as non-finite margins), share in which a law-swing foot is
    loaded, the weight fraction carried by law-swing feet, and per single-swing leg the same with the
    law-tripod geometric margin for comparison."""
    import dynamics as dyn
    m, st = D["m"], D["st"]
    Fz = D["probe_F"][..., 2]
    L = Fz > 0.1
    tot = Fz.sum(1)
    swingload = np.where(st == 0, Fz, 0).sum(1)
    one = m & ((st == 0).sum(1) == 1)
    per = {}
    for l in range(4):
        sel = one & (st[:, l] == 0)
        if not sel.any():
            continue
        per[LEG[l]] = {"swing_foot_loaded_share": float(np.mean(L[sel, l])),
                       "law_stance_feet_all_unloaded_share": float(np.mean(~(L[sel] & (st[sel] == 1)).any(1))),
                       "weight_fraction_on_swing_foot_mean": float(np.mean(Fz[sel, l] / np.maximum(tot[sel], 1e-9)))}
    sel = np.flatnonzero(m)[::10]
    mg = np.array([dyn.support_margin(D["rec_feet"][i][L[i]], D["rec_com"][i][:2]) for i in sel])
    return {"samples": int(m.sum()), "no_loaded_foot_samples": int(np.sum(m & ~L.any(1))),
            "law_swing_foot_loaded_share": float(np.mean((L & (st == 0)).any(1)[m & (st == 0).any(1)])),
            "weight_fraction_on_law_swing_feet_mean": float(np.mean(swingload[m] / np.maximum(tot[m], 1e-9))),
            "loaded_feet_margin_mm_p05_p50": [1e3 * float(np.nanpercentile(mg, q)) for q in (5, 50)],
            "loaded_feet_margin_nonfinite_share": float(np.mean(~np.isfinite(mg))), "single_swing": per}


def clearance(argv):
    """Clearance table (median over the full swings of the window) from foot_lift_rca JSONs."""
    rows = []
    for p in argv[1:]:
        d = json.loads(pathlib.Path(p).read_text())
        for grp, legs in (("front", ("LF", "RF")), ("hind", ("RH", "LH"))):
            S = [s for s in d["swings"] if s["leg"] in legs]
            if not S:
                continue
            med = lambda f: float(np.median([f(s) for s in S]))
            u5 = " ".join(f"{k} {med(lambda s, k=k: s['at_law_peak_mm'][k]):6.2f}" for k in ("law", "nom", "cmd", "actnom", "act", "geom"))
            pk = " ".join(f"{k} {med(lambda s, k=k: s['peak_mm'][k]):6.2f}" for k in ("law", "nom", "cmd", "actnom", "act", "geom"))
            gap = [s["geom_gap_share"]["above_0_1mm"] for s in S]
            rows.append(f"{pathlib.Path(p).stem:17s} {grp:5s} n {len(S):2d} u=.5: {u5} | peak: {pk} | loaded {med(lambda s: s['contact']['loaded_share']):.2f} "
                        f"gap>.1 p50 {np.median(gap):.2f} max {np.max(gap):.2f} | geom peak max {max(s['peak_mm']['geom'] for s in S):.3f} | "
                        f"body dz {med(lambda s: s['base_at_law_peak']['z_mm_minus_law']):5.2f} pitch {med(lambda s: s['base_at_law_peak']['pitch_mrad']):6.1f} "
                        f"|roll| {med(lambda s: abs(s['base_at_law_peak']['roll_mrad'])):4.1f} | U {med(lambda s: s['tracking_at_law_peak_ticks']['upper']):4.0f} "
                        f"L {med(lambda s: s['tracking_at_law_peak_ticks']['lower']):4.0f} | sync {max(s.get('act_record_offset_max_mm', float('nan')) for s in S):.3f} mm "
                        f"| truncated out {d.get('truncated_swings_left_out', '-')}")
    print("median over full swings (mm); u=.5 = law peak instant; loaded = Fz>0.1 N share of the swing; gap = share of the swing "
          "with geom distance > 0.1 mm (p50 and max over swings); versions on different bodies: not additive")
    print("\n".join(rows))


def swings(D):
    """Every swing of the run: start, leg, COM advance, foot advance (heading frame), contact and UPPER state."""
    t, st = D["rec_t"], D["st"]
    out = []
    for l in range(4):
        e = np.flatnonzero(np.diff(np.r_[0, (st[:, l] == 0).astype(int), 0]))
        for i0, i1 in zip(e[::2], e[1::2]):
            if i1 - i0 <= 50:
                continue
            hx, hy = D["h"][i0]
            Fn = D["probe_F"][i0:i1, l, 2]
            L = Fn > 0.1
            vx = D["probe_vref"][i0:i1, l, :2] @ [hx, hy]
            up = 3 * l + 1
            out.append({"t0": float(t[i0]), "leg": LEG[l], "com_dx_mm": 1e3 * float((D["rec_com"][i1 - 1] - D["rec_com"][i0])[:2] @ [hx, hy]),
                        "foot_advance_mm": 1e3 * float((D["rec_feet"][i1 - 1, l] - D["rec_feet"][i0, l])[:2] @ [hx, hy]),
                        "loaded_share": float(L.mean()), "fn_mean_n": float(Fn.mean()), "slip_vx_loaded_mm_s": 1e3 * float(vx[L].mean()) if L.any() else None,
                        "upper_err_p50_ticks": float(np.median((D["rec_q"][i0:i1, up] - D["probe_goal"][i0:i1, up]) / TICK)),
                        "upper_saturated_share": float(np.mean(np.abs(D["rec_duty"][i0:i1, up]) >= .999))})
    return sorted(out, key=lambda s: s["t0"])


def divergence(A, B):
    """B: first instants at which two runs of the same build (mu .8 / 1.0) differ, and the contact that causes it."""
    t = A["rec_t"]
    n = min(len(t), len(B["rec_t"]))
    first = lambda x, th: float(t[np.argmax(x > th)]) if np.any(x > th) else None
    dpos = np.linalg.norm(B["rec_pos"][:n] - A["rec_pos"][:n], axis=1)
    dfoot = np.linalg.norm(B["rec_feet"][:n, :, :2] - A["rec_feet"][:n, :, :2], axis=2).max(1)
    dq = np.abs(B["rec_q"][:n] - A["rec_q"][:n]).max(1) / TICK
    raw = lambda D: np.stack([D[f"row_raw{j}"] for j in range(12)], 1)
    rn = min(len(A["row_t_s"]), len(B["row_t_s"]))
    rd = np.flatnonzero(np.any(raw(A)[:rn] != raw(B)[:rn], axis=1))
    ratio = np.nan_to_num(A["probe_use"][:n], nan=0) * A["meta"]["mu"]  # |Ft| / Fn in the mu .8 run
    k = int(np.argmax((ratio >= A["meta"]["mu"] * .999).any(1)))
    legs = [LEG[l] for l in np.flatnonzero(ratio[k] >= A["meta"]["mu"] * .999)]
    return {"walk_start_s": float(t[np.argmax((A["st"] >= 0).all(1))]),
            "dpos_mm_first_above": {f"{th:g}": first(dpos, th * 1e-3) for th in (1e-4, 1e-3, 1e-2, 0.1, 1)},
            "dfoot_xy_mm_first_above": {f"{th:g}": first(dfoot, th * 1e-3) for th in (1e-4, 1e-3, 1e-2, 0.1, 1)},
            "dq_ticks_first_above": {f"{th:g}": first(dq, th) for th in (0.01, 0.1, 1, 10)},
            "first_command_difference_s": float(A["row_t_s"][rd[0]]) if len(rd) else None,
            "first_cone_saturation_low_mu": {"t_s": float(t[k]), "legs": legs, "law_stance": A["st"][k].tolist(), "fn_n": A["probe_F"][k, :, 2].tolist(),
                                             "ft_over_fn_low_mu": ratio[k].tolist(), "ft_over_fn_high_mu": (np.nan_to_num(B["probe_use"][k], nan=0) * B["meta"]["mu"]).tolist()}}


def analyse(argv):
    out, runs, cfdir = pathlib.Path(argv[1]), pathlib.Path(argv[2]), pathlib.Path(argv[3])
    res = {"runs": {}, "counterfactuals": {}, "divergence": {}}
    data = {}
    for p in sorted(runs.glob("*.npz")) + sorted(cfdir.glob("*.npz")):
        D = data[p.stem] = load(p)
        r = {"meta": {k: D["meta"][k] for k in ("label", "law", "sets", "cf", "mu", "binary", "fw_tree", "opt", "foot_friction", "floor_friction",
                                                "foot_condim", "floor_condim", "foot_priority", "floor_priority", "npair", "warnings", "other_floor_geoms", "compare") if k in D["meta"]},
             "audit": audit(D), "phases": phases(D), "joints": joints(D), "body": body(D), "chatter": chatter(D), "stance_triangle": stance_triangle(D),
             "support": support(D)}
        if p.parent == runs:
            r["swings"] = swings(D)
        (res["runs"] if p.parent == runs else res["counterfactuals"])[p.stem] = r
        print(f"{p.stem:22s} speed {r['audit']['m6_gait_mm_s']:7.2f} mm/s (COM {r['audit']['com_mm_s']:7.2f}) | COM dx/cycle {r['phases']['com_dx_mm_per_cycle']:6.1f} mm: "
              + " ".join(f"{k} {v['dx_mm_per_cycle']:6.1f}" for k, v in r["phases"]["by_phase"].items())
              + f" | front swing loaded {np.mean([r['phases']['feet'][f'{x} swing']['loaded_share'] for x in ('LF', 'RF')]):.2f} Fn {np.mean([r['phases']['feet'][f'{x} swing']['fn_mean_n'] for x in ('LF', 'RF')]):5.1f}"
              + f" | front UPPER swing loop p50 {r['joints']['front UPPER swing']['loop_abs_p50_p95'][0]:4.0f} sat {r['joints']['front UPPER swing']['saturated_share']:.2f}", flush=True)
    for b in ("B66", "dev7"):
        if f"{b}_mu08" in data and f"{b}_mu10" in data:
            res["divergence"][b] = divergence(data[f"{b}_mu08"], data[f"{b}_mu10"])
    out.write_text(json.dumps(res, indent=1) + "\n")


def plot(argv):
    """Figure 1: B66 mu .8 / 1.0 from the walk start (COM advance, front swing foot load and front UPPER duty,
    20 ms means: the 1 ms values carry the ~200 Hz stick-slip chatter); figure 2: the first divergence (log |dpos|,
    per contact point |Ft| / (mu Fn) of RH and LF from <contacts dir>); figure 3: COM advance per cycle by phase,
    every run and counterfactual (from <out dir>/analysis.json)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out, runs, cdir = pathlib.Path(argv[1]), pathlib.Path(argv[2]), pathlib.Path(argv[3])
    A, B = load(runs / "B66_mu08.npz"), load(runs / "B66_mu10.npz")
    mean20 = lambda x: np.convolve(np.nan_to_num(x), np.ones(20) / 20, mode="same")
    t = A["rec_t"]
    i0 = int(np.argmax(t >= 3.5))
    fig, ax = plt.subplots(3, 1, figsize=(11, 8.5), sharex=True)
    for D, name, col in ((A, "μ 0.8", "tab:blue"), (B, "μ 1.0", "tab:red")):
        x = np.cumsum(heading(D, D["probe_vcom"])) * DT
        ax[0].plot(t[i0:], 1e3 * (x[i0:] - x[i0]), color=col, label=f"F40T_JCB66 {name}")
        for l, ls in ((0, "-"), (1, "--")):
            sw = D["st"][i0:, l] == 0
            ax[1].plot(t[i0:], np.where(sw, mean20(D["probe_F"][:, l, 2])[i0:], np.nan), color=col, ls=ls, lw=1, label=f"{LEG[l]} in swing, {name}")
            ax[2].plot(t[i0:], np.where(sw, mean20(1e3 * np.abs(D["rec_duty"][:, 3 * l + 1]))[i0:], np.nan), color=col, ls=ls, lw=1, label=f"{LEG[l]} UPPER in swing, {name}")
    law = A["row_body_x"][A["idx"]]
    ax[0].plot(t[i0:], 1e3 * (law[i0:] - law[i0]), color="gray", ls=":", label="law body_x")
    for a in ax:
        for l in (0, 1):
            e = np.flatnonzero(np.diff(np.r_[0, (A["st"][:, l] == 0).astype(int), 0]))
            for s0, s1 in zip(e[::2], e[1::2]):
                if t[s0] >= 3.5:
                    a.axvspan(t[s0], t[s1 - 1], color="0.9", zorder=0)
        a.axvline(4.520, color="k", lw=.8, ls="--")
        a.axvline(6.46, color="k", lw=.8)
    ax[0].set_ylabel("COM advance (mm)\nheading frame")
    ax[1].set_ylabel("front swing foot\nnormal force (N, 20 ms mean)")
    ax[2].set_ylabel("front UPPER |duty|\nin swing (‰, 20 ms mean)")
    ax[2].set_xlabel("time (s); grey: front-leg swings; dashed: first divergence 4.520 s; solid: first front swing 6.46 s")
    ax[0].set_ylim(-80, 420)
    ax[0].legend(fontsize=8, loc="upper left")
    ax[1].legend(fontsize=7, ncol=2, loc="upper right")
    ax[2].legend(fontsize=7, ncol=2, loc="lower right")
    ax[0].set_xlim(3.5, 16)
    fig.suptitle("RCA 01 — twin dev7_ground, fast_fwd, F40T_JCB66: friction 0.8 vs 1.0 (simulation only)")
    fig.tight_layout()
    fig.savefig(out / "fig1_timeline_B66.png", dpi=130)
    fig, ax = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
    n = min(len(t), len(B["rec_t"]))
    sl = slice(int(np.argmax(t >= 4.40)), int(np.argmax(t >= 4.90)))
    ax[0].semilogy(t[sl], 1e3 * np.linalg.norm(B["rec_pos"][:n] - A["rec_pos"][:n], axis=1)[sl] + 1e-9, label="|Δ base position| (mm)")
    ax[0].semilogy(t[sl], 1e3 * np.linalg.norm(B["rec_feet"][:n, :, :2] - A["rec_feet"][:n, :, :2], axis=2).max(1)[sl] + 1e-9, label="max |Δ foot xy| (mm)")
    ax[0].legend(fontsize=8)
    for mu, name, col in (("08", "μ 0.8", "tab:blue"), ("10", "μ 1.0", "tab:red")):
        c = np.load(cdir / f"B66_mu{mu}.npz")["contacts"]  # t, foot, Fn, |Ft|, mu, dist, slip, pos
        for l, ls in ((2, "-"), (0, "--")):
            pts = c[(c[:, 1] == l) & (c[:, 0] >= t[sl][0]) & (c[:, 0] <= t[sl][-1])]
            for tt in (np.unique(pts[:, 0]),):
                light = np.array([pts[pts[:, 0] == x][np.argmin(pts[pts[:, 0] == x][:, 2])] for x in tt])  # the lighter point
                ax[1].plot(light[:, 0], light[:, 3] / np.maximum(light[:, 4] * light[:, 2], 1e-12), color=col, ls=ls, lw=1,
                           label=f"{LEG[l]} ({'swing' if l == 2 else 'stance'}) lighter point, {name}")
    ax[1].set_ylabel("|Ft| / (μ Fn), per point")
    ax[1].set_ylim(0, 1.05)
    ax[1].legend(fontsize=7, ncol=2, loc="lower left")
    for a in ax:
        a.axvline(4.520, color="k", lw=.8, ls="--")
    ax[1].set_xlabel("time (s); RH in swing from 4.38 s (first swing of the walk, foot dragged)")
    fig.suptitle("First divergence μ 0.8 / 1.0 (F40T_JCB66): a contact point of the dragged RH swing foot reaches the 0.8 cone", fontsize=10)
    fig.tight_layout()
    fig.savefig(out / "fig2_first_divergence.png", dpi=130)
    res = json.loads((out / "analysis.json").read_text())
    runs_ = {**res["runs"], **res["counterfactuals"]}
    order = ["B66_mu08", "B66_mu10", "B66_mu10_swing08", "B66_mu08_swing10", "B66_mu10_swing0", "B66_mu10_ghost", "B66_mu10_tl2", "B66_mu10_imu0",
             "B66_mu10_dt025", "B66_mu10_noslip20", "B66_mu08_solref008", "B66_mu10_solref008", "dev7_mu08", "dev7_mu10", "dev7_mu10_swing08"]
    order = [k for k in order if k in runs_]
    parts = ("LF", "RF", "RH", "LH", "4-stance")
    fig, ax = plt.subplots(figsize=(13, 5.5))
    for j, k in enumerate(order):
        pos = neg = 0.0
        for c, part in zip(("tab:red", "tab:orange", "tab:green", "tab:olive", "tab:gray"), parts):
            v = runs_[k]["phases"]["by_phase"].get(part, {"dx_mm_per_cycle": 0})["dx_mm_per_cycle"]
            base = pos if v >= 0 else neg
            ax.bar(j, v, bottom=base, color=c, label=part if j == 0 else None)
            pos, neg = (pos + v, neg) if v >= 0 else (pos, neg + v)
        ax.plot(j, runs_[k]["phases"]["com_dx_mm_per_cycle"], "k_", ms=18, mew=2)
    ax.axhline(0, color="k", lw=.6)
    ax.set_xticks(range(len(order)), [k.replace("_", "\n") for k in order], fontsize=8)
    ax.set_ylabel("COM advance per cycle (mm), by phase; black: net")
    ax.legend(fontsize=8, title="leg in swing", ncol=5)
    fig.suptitle("Where the advance is lost: COM advance per cycle by gait phase (twin, fast_fwd; cf = single-condition counterfactuals)")
    fig.tight_layout()
    fig.savefig(out / "fig3_phase_budget.png", dpi=130)


def footlift(argv):
    """foot_lift_rca.py unchanged, inside a counterfactual: footlift <name=value> <foot_lift_rca arguments>."""
    _sim_modules()
    import gait_study.fast40.foot_lift_rca as fl  # noqa: E402 (Matdog_MuJoCo is on sys.path)
    install(Probe(), dict([argv[1].split("=")]))
    sys.argv = ["foot_lift_rca.py"] + argv[2:]
    fl.main()


if __name__ == "__main__":
    {"run": run, "analyse": analyse, "plot": plot, "footlift": footlift, "contacts": contacts, "clearance": clearance}.get(sys.argv[1], lambda _: sys.exit(__doc__))(sys.argv[1:])
