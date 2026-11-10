#!/usr/bin/env python3
"""See the MATDOG twin: interactive 3D viewer on the desktop, MP4 or PNG offscreen.

Every replay is recomputed with the nominal servo model (deterministic, no tuning):
  closed loop  results/cl/*.csv (cosim.py): SimpleMotion's frames re-applied from the same
               start; each release is checked against the feedback SimpleMotion read (fb_raw)
  open loop    sequences/*.csv (fw_bridge/export_sequences), 1.5 s settle on frame 0; the
               delivered fixed-IMU CSVs only as diagnostic reference
  stand150     the SimpleMotion STAND 150 mm pose (sequences/stand_poses.csv) held 5 s
Overlay tags: [SIM] MuJoCo result, [FW] production SimpleMotion output (real code, simulated
inputs), [HW] measured on the robot, [IPOTESI] assumed parameter.

  view.py --list
  view.py <replay> [<replay> ...]       viewer: SPACE play/pause, LEFT/RIGHT one release,
                                        UP/DOWN speed x2 /2, F first fault, R restart,
                                        N/P next/previous replay, TAB MuJoCo panel; mouse: camera
  view.py <replay> --mp4 <file> [--start s] [--end s] [--fps 50]    (MUJOCO_GL=egl)
  view.py <replay> --png <file> --at s [s ...]
  view.py <sequence.csv> --kinematic    joints imposed, no physics (milestone 1)
A replay is a catalog name, a CSV path or stand:<height m>; data root $MATDOG_TWIN_DATA
(default ~/MATDOG/verification-artifacts/MATDOG_MUJOCO_TWIN_20261110).
"""
import argparse
import dataclasses
import json
import os
import pathlib
import struct
import subprocess
import time
import zlib

import mujoco
import mujoco.viewer
import numpy as np

import dynamics
import servo
import twin

DATA = pathlib.Path(os.environ.get("MATDOG_TWIN_DATA", "~/MATDOG/verification-artifacts/MATDOG_MUJOCO_TWIN_20261110")).expanduser()
CATALOG = {  # name: (source, title)
    "stand150": ("stand:0.15", "STAND 150 mm: posa SimpleMotion tenuta 5 s (anello aperto)"),
    "modi": ("results/cl/cl_modes_slow_standard_fast_slow.csv", "WALK SLOW>STANDARD>FAST>SLOW, anello chiuso"),
    "stop_fuori_box": ("results/cl/cl_stop_front_standard.csv", "STOP a meta' swing anteriore, STANDARD: STOP_MEASURED_OUTSIDE_BOX"),
    "stop_fuori_box_slow": ("results/cl/cl_stop_lateswing_slow.csv", "STOP a fine swing anteriore, SLOW: STOP_MEASURED_OUTSIDE_BOX"),
    "stop_non_assestato": ("results/cl/cl_stop_lateswing_fast.csv", "STOP a fine swing anteriore, FAST: STOP_NOT_SETTLED"),
    "modi_anello_aperto": ("sequences/imufollows_preset_slow_standard_fast_slow.csv", "WALK SLOW>STANDARD>FAST>SLOW, anello aperto, IMU coerente"),
    "csv_imu_fisso": ("sequences/preset_slow_standard_fast_slow.csv", "CSV consegnato, IMU fisso (deriva artificiale): solo diagnostico"),
    "stop_fuori_box_anche_x2": ("results/m4/cl_stop_front_standard__hips_x2.csv",
                                "STOP a meta' swing anteriore, STANDARD, anche 2x rigide (deviazione HIP come HW): STOP_MEASURED_OUTSIDE_BOX"),
}
# Diagnostic variants of m4_hip_stop.py: the replay must use the servo model that produced the CSV.
VARIANT_UNIT = {"__hips_x2": [2.0 if i in (0, 3, 6, 9) else 1.0 for i in range(12)]}
HIPS = (0, 3, 6, 9)
HW_HIP_MARGIN_MIN = 4.3  # ticks: measured hip margin to kHeight in ground STOPs after walking (dev.4 08/11, dev16 07/11), m4_hip_stop.py
W, H = 1280, 720
try:  # rc.11 STAND150 HOLD, measured |q_cmd - q| per joint (ticks): m2_stand_hold.py
    HW_HOLD = json.loads((DATA / "results" / "m2" / "m2_stand_hold.json").read_text())["hardware"]["error_ticks"]
except (OSError, KeyError, ValueError):
    HW_HOLD = None


@dataclasses.dataclass
class Replay:
    name: str
    title: str
    kind: str                  # closed, open, stand, kinematic
    t: np.ndarray              # s, per release
    qpos: np.ndarray
    qvel: np.ndarray
    q_cmd: np.ndarray          # commanded joints of the release (rad)
    fz: np.ndarray             # foot normal force (N)
    duty: np.ndarray           # PresentLoad equivalent (fraction)
    info: list                 # per release: firmware fields
    check: str = ""            # closed loop: replay vs the feedback SimpleMotion read
    hip: int = 0               # hip shown in the panel and the plot
    fault_k: int = -1
    stop_k: int = -1
    variant: str = ""          # a diagnostic servo variant, never the nominal model


def resolve(name):
    if name in CATALOG:
        source, title = CATALOG[name]
        return name, (source if source.startswith("stand:") else str(DATA / source)), title
    return (name if name.startswith("stand:") else pathlib.Path(name).stem), name, name


def compute(name):
    label, source, title = resolve(name)
    unit = next((u for tag, u in VARIANT_UNIT.items() if tag in pathlib.Path(source).stem), None)
    tw = dynamics.Twin(servo.Params(), unit=unit)
    snaps = {k: [] for k in ("qpos", "qvel", "fz", "duty")}

    def snapshot():
        mujoco.mj_rnePostConstraint(tw.m, tw.d)
        snaps["qpos"].append(tw.d.qpos.copy())
        snaps["qvel"].append(tw.d.qvel.copy())
        snaps["fz"].append(tw.d.cfrc_ext[tw.foot_bodies, 5].copy())
        snaps["duty"].append(tw.servos.duty(tw.d.qpos[7:19]))

    check = ""
    if source.startswith("stand:"):
        h = float(source.split(":")[1])
        stand = twin.read_csv(DATA / "sequences" / "stand_poses.csv")
        row = int(np.argmin(np.abs(stand["h_nominal_m"].astype(float) - h)))
        q = (np.array([int(stand[f"raw{i}"][row]) for i in range(12)]) - 2048) * twin.TICK
        n, kind = 250, "stand"
        t = np.arange(1, n + 1) * twin.RELEASE_S
        q_cmd = np.tile(q, (n, 1))
        info = [{"state": "STANDING", "phase": "HOLD", "reason": "STAND_REACHED", "law_mode": "-", "target_mode": "-",
                 "drive": "480/13", "stop_active": "0", "stop_verified": "0", "torque_on": "12"}] * n
        tw.begin(q, 480, 13)
        for _ in range(n):
            tw.release(q, 480, 13, record=False)
            snapshot()
    else:
        header = pathlib.Path(source).read_text().split("\n", 1)[0]
        table = twin.sequence(source) if "raw0" in header else twin.delivered(source)
        n = len(table["q"])
        t, q_cmd = table["t"], table["q"]
        fields = ("state", "phase", "reason", "law_mode", "target_mode", "stop_active", "stop_verified", "torque_on")
        info = [{f: str(table[f][k]) if f in table else "-" for f in fields} for k in range(n)]
        for k, row in enumerate(info):
            row["drive"] = f"{table['drive_speed_step_s'][k]}/{table['drive_acc_100step_s2'][k]}"
        speed = table["drive_speed_step_s"].astype(int)
        acc = table["drive_acc_100step_s2"].astype(int)
        if "fb_raw0" in table:  # closed loop: same start and frames as cosim.py
            kind = "closed"
            fb = np.stack([table[f"fb_raw{i}"].astype(int) for i in range(12)], axis=1)
            tw.begin((fb[0] - 2048) * twin.TICK, 480, 13)
            mismatches = 0
            for k in range(n):
                mismatches += int(np.any(2048 + np.rint(tw.d.qpos[7:19] / twin.TICK).astype(int) != fb[k]))
                tw.release(q_cmd[k], speed[k], acc[k], np.full(12, table["torque_on"][k] == "12"), record=False)
                snapshot()
            check = f"replay = cosim registrata: {n - mismatches}/{n} release identiche"
        else:
            kind = "open"
            tw.begin(q_cmd[0], speed[0], acc[0])
            tw.advance(1.5, record=False)
            for k in range(n):
                tw.release(q_cmd[k], speed[k], acc[k], record=False)
                snapshot()
    rep = Replay(label, title, kind, np.asarray(t, float), np.array(snaps["qpos"]), np.array(snaps["qvel"]), np.asarray(q_cmd),
                 np.array(snaps["fz"]), np.array(snaps["duty"]), info, check)
    rep.variant = "[IPOTESI diagnostica] anche con costante di coppia x2" if unit is not None else ""
    states = [row["state"] for row in info]
    rep.fault_k = states.index("FAULT") if "FAULT" in states else -1
    stops = [k for k, row in enumerate(info) if row["stop_active"] == "1"]
    rep.stop_k = stops[0] if stops else -1
    margin = hip_margin(rep.qpos[:, 7:19])
    at = rep.fault_k - 1 if rep.fault_k > 0 else int(np.argmin(margin.min(axis=1)))
    rep.hip = HIPS[int(np.argmin(margin[at]))]
    return rep


def kinematic(name):
    label, source, title = resolve(name)
    table = twin.sequence(source)
    tw = dynamics.Twin(servo.Params())
    tw.place(table["q"][0])
    n = len(table["q"])
    qpos = np.tile(tw.d.qpos, (n, 1))
    qpos[:, 7:19] = table["q"]
    info = [{"state": str(table["state"][k]), "phase": str(table["phase"][k]), "reason": "-", "law_mode": str(table["law_mode"][k]),
             "target_mode": str(table["target_mode"][k]), "drive": "-", "stop_active": str(table["stop_active"][k]),
             "stop_verified": "-", "torque_on": "-"} for k in range(n)]
    return Replay(label, title + " (giunti imposti)", "kinematic", table["t"], qpos, np.zeros((n, tw.m.nv)), table["q"],
                  np.zeros((n, 4)), np.zeros((n, 12)), info)


def wrap(text, width=40):
    lines, line = [], ""
    for word in text.split():
        if line and len(line) + 1 + len(word) > width:
            lines.append(line)
            line = "     " + word
        else:
            line = f"{line} {word}" if line else word
    return "\n".join(lines + [line])


def hip_margin(q):
    """Distance of each hip from the kHeight bound (ticks, > 0 inside), per row."""
    lo, hi = dynamics.box()
    return (np.minimum(q - lo, hi - q) / twin.TICK)[:, HIPS]


class Painter:
    """Overlay texts, the hip figure and scene markers, shared by viewer and offscreen."""

    def __init__(self, model):
        self.centers = twin.site_ids(model, "foot_cylinder_center")
        self.feet = np.array([model.geom(f"{l}_foot_cylinder").id for l in twin.LEGS])
        self.hip_joints = [model.joint(twin.JOINTS[h]).id for h in HIPS]
        self.fig = mujoco.MjvFigure()

    def texts(self, rep, k, playback=""):
        row, t = rep.info[k], rep.t[k]
        roll, pitch = dynamics.quat_rpy(rep.qpos[k, 3:7])
        q = rep.qpos[k, 7:19]
        err = np.abs(rep.q_cmd[k] - q) / twin.TICK
        j = int(np.argmax(err))
        load = np.abs(rep.duty[k])
        jl = int(np.argmax(load))
        contacts = "  ".join(f"{l.upper()} {f:4.1f}" if f > dynamics.CONTACT_N else f"{l.upper()}  -- " for l, f in zip(twin.LEGS, rep.fz[k]))
        source = {"closed": "[FW] SimpleMotion 450d2c7 (Motion=ae70e03)\n     in anello chiuso col gemello",
                  "open": "[FW] frame esportati dal banco host\n     anello aperto",
                  "stand": "[FW] posa STAND 150 di SimpleMotion\n     tenuta dal gemello",
                  "kinematic": "[FW] frame esportati\n     giunti imposti, senza fisica"}[rep.kind]
        title = ("MATDOG - SIMULAZIONE MuJoCo, NON HARDWARE\n" + wrap(rep.title) + "\n" + source + "\n"
                 "[IPOTESI] servo identificato a vuoto;\n"
                 "     ACC>50, carico, attrito 0.8: non misurati" + (f"\n{wrap(rep.variant)}" if rep.variant else "") +
                 (f"\n{wrap(rep.check)}" if rep.check else ""))
        fault = f"{rep.info[rep.fault_k]['reason']} a {rep.t[rep.fault_k]:.2f} s" if 0 <= rep.fault_k <= k else "-"
        stop = "-" if row["stop_active"] != "1" else ("attivo, verificato" if row["stop_verified"] == "1" else "attivo, non verificato")
        h = HIPS.index(rep.hip)
        margin = hip_margin(q[None, :])[0, h]
        margin_cmd = hip_margin(rep.q_cmd[k][None, :])[0, h]
        name = twin.SLOT_NAMES[rep.hip]
        labels, values = "assetto [SIM]\nappoggi Fz [SIM]\n|cmd-mis| max [SIM]\ncarico max [SIM]", (
            f"roll {1e3 * roll:+6.1f}  pitch {1e3 * pitch:+6.1f} mrad\n{contacts} N\n"
            f"{err[j]:5.1f} tick  {twin.SLOT_NAMES[j]}\n{1e3 * load[jl]:5.0f} permille  {twin.SLOT_NAMES[jl]} (STOP: <=400)")
        if rep.kind == "stand" and HW_HOLD:  # the real sag of a STAND150 HOLD next to the simulated one
            pairs = [f"{twin.SLOT_NAMES[i]} {abs(HW_HOLD[i]):.0f} / {err[i]:.0f}" for i in (2, 5, 7, 10)]
            labels += "\n|cmd-mis| [HW] / [SIM]\n(HW: HOLD rc.11)"
            values += f"\n{pairs[0]}   {pairs[1]}\n{pairs[2]}   {pairs[3]}"
        return [
            (mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT, title, None),
            (mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPRIGHT,
             "t [SIM]\nMotion [FW]\nprofilo [FW]\ndrive [FW]\nSTOP [FW]\nFAULT [FW]",
             f"{t:7.2f} s {playback}\n{row['state']} / {row['phase']}\n{row['law_mode']} > {row['target_mode']}\n"
             f"{row['drive']}\n{stop}\n{fault}"),
            (mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMLEFT, labels, values),
            (mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_BOTTOMRIGHT,
             f"{name} misurata [SIM]\n{name} comandata [FW]\ndistanza kHeight [SIM]\nminimo osservato [HW]",
             f"{q[rep.hip] / twin.TICK:+7.1f} tick\n{rep.q_cmd[k][rep.hip] / twin.TICK:+7.1f} tick\n"
             f"{margin:+6.1f} tick (comandata {margin_cmd:+5.1f})\n{HW_HIP_MARGIN_MIN:+6.1f} tick (STOP verificati a terra)"),
        ]

    def figure(self, rep, k, window=6.0):
        """Hip distance from the kHeight bound over the last `window` s (ticks, > 0 inside)."""
        fig = self.fig
        k0 = max(0, int(np.searchsorted(rep.t, rep.t[k] - window)))
        sl = slice(k0, k + 1)
        h = HIPS.index(rep.hip)
        meas = np.clip(hip_margin(rep.qpos[sl, 7:19])[:, h], -25, 25)  # a fall after SAFE_OFF leaves the scale
        cmd = np.clip(hip_margin(rep.q_cmd[sl])[:, h], -25, 25)
        x = rep.t[sl]
        step = max(1, len(x) // 300)
        # The legend sits top right over the plot: two entries only, above the data (margin <= 16.3 tick)
        ymin, ymax = -30.0, 50.0
        lines = [("misurata [SIM]", x[::step], meas[::step], (1.0, 0.55, 0.1)),
                 ("comandata [FW]", x[::step], cmd[::step], (0.75, 0.75, 0.75)),
                 ("", [x[0], x[-1]], [0.0, 0.0], (0.95, 0.15, 0.15)),
                 ("", [x[0], x[-1]], [HW_HIP_MARGIN_MIN] * 2, (0.2, 0.85, 0.3))]
        for kk, rgb in ((rep.stop_k, (0.3, 0.8, 1.0)), (rep.fault_k, (1.0, 0.1, 0.6))):  # STOP, FAULT
            if k0 <= kk <= k:
                lines.append(("", [rep.t[kk]] * 2, [ymin, ymax], rgb))
        fig.linepnt[:] = 0
        for i, (label, xs, ys, rgb) in enumerate(lines):
            n = len(xs)
            fig.linedata[i][: 2 * n] = np.column_stack([xs, ys]).ravel()
            fig.linepnt[i] = n
            fig.linename[i] = label
            fig.linergb[i] = rgb
        fig.title = f"{twin.SLOT_NAMES[rep.hip]}: distanza da kHeight (tick, >0 dentro)"
        fig.xlabel = "t (s)   rosso: kHeight   verde: min. HW"
        fig.range[0] = [x[0], max(x[-1], x[0] + 0.1)]
        fig.range[1] = [ymin, ymax]
        fig.flg_legend = 1
        fig.gridsize = [5, 9]
        fig.figurergba[3] = 0.6
        return fig

    def markers(self, scn, data, rep, k):
        """Hip shown (red outside kHeight, yellow under the HW minimum, else green) and feet (green in contact)."""
        def sphere(pos, radius, rgba):
            if scn.ngeom < scn.maxgeom:
                mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_SPHERE, [radius, 0, 0], pos, np.eye(3).ravel(), rgba)
                scn.ngeom += 1
        h = HIPS.index(rep.hip)
        margin = hip_margin(rep.qpos[k][None, 7:19])[0, h]
        rgba = [0.9, 0.1, 0.1, 0.9] if margin < 0 else [0.95, 0.8, 0.1, 0.9] if margin < HW_HIP_MARGIN_MIN else [0.2, 0.8, 0.3, 0.9]
        anchor = data.xanchor[self.hip_joints[h]]
        flag = anchor + [0.0, 0.0, 0.06]  # the anchor is inside the body: a flag above it
        if scn.ngeom < scn.maxgeom:  # initGeom first: the connector sets only the shape, a stale material id crashes mjr_render
            mujoco.mjv_initGeom(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).ravel(), rgba)
            mujoco.mjv_connector(scn.geoms[scn.ngeom], mujoco.mjtGeom.mjGEOM_CAPSULE, 0.002, anchor, flag)
            scn.ngeom += 1
        sphere(flag, 0.012, rgba)
        feet = twin.contact_reference(data, self.centers, self.feet)
        for l in range(4):
            sphere(feet[l], 0.008, [0.2, 0.9, 0.3, 0.9] if rep.fz[k, l] > dynamics.CONTACT_N else [0.55, 0.55, 0.55, 0.6])


def figure_rect(width, height):
    """Bottom right, above the bottom texts, scaled to the window."""
    w, h = int(min(460, 0.38 * width)), int(min(320, 0.45 * height))
    return mujoco.MjrRect(width - w - 8, max(118, int(0.2 * height)), w, h)


def pose(model, data, rep, k):
    data.qpos[:] = rep.qpos[k]
    data.qvel[:] = rep.qvel[k]
    mujoco.mj_forward(model, data)


def track(cam, model, hip=0):
    """Follow the body from the side of the hip shown, three-quarter view."""
    side_x, side_y = (1 if hip in (0, 3) else -1), (1 if hip in (0, 9) else -1)  # LF, RF front; LF, LH left
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = model.body("base_link").id
    cam.distance, cam.elevation = 0.85, -18.0
    cam.azimuth = float(np.degrees(np.arctan2(-side_y, -0.6 * side_x)))


def write_png(path, rgb):
    """RGB uint8 image to PNG with the standard library only."""
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)
    pathlib.Path(path).write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0)) +
                                   chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


class Offscreen:
    def __init__(self, model):
        self.m, self.d = model, mujoco.MjData(model)
        self.gl = mujoco.GLContext(W, H)
        self.gl.make_current()
        self.con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)
        mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, self.con)
        self.scn = mujoco.MjvScene(model, maxgeom=4000)
        self.cam, self.opt = mujoco.MjvCamera(), mujoco.MjvOption()
        track(self.cam, model)
        self.paint = Painter(model)
        self.rect = mujoco.MjrRect(0, 0, W, H)

    def render(self, rep, k, playback=""):
        if getattr(self, "_hip", None) != rep.hip:
            track(self.cam, self.m, rep.hip)
            self._hip = rep.hip
        pose(self.m, self.d, rep, k)
        mujoco.mjv_updateScene(self.m, self.d, self.opt, None, self.cam, mujoco.mjtCatBit.mjCAT_ALL, self.scn)
        self.paint.markers(self.scn, self.d, rep, k)
        mujoco.mjr_render(self.rect, self.scn, self.con)
        for font, grid, a, b in self.paint.texts(rep, k, playback):
            mujoco.mjr_overlay(font, grid, self.rect, a, b, self.con)
        if rep.kind != "kinematic":
            mujoco.mjr_figure(figure_rect(W, H), self.paint.figure(rep, k), self.con)
        img = np.empty((H, W, 3), np.uint8)
        mujoco.mjr_readPixels(img, None, self.rect, self.con)
        return np.flipud(img)


def export_mp4(rep, path, start, end, fps):
    ks = [k for k in range(len(rep.t)) if start <= rep.t[k] <= end]
    speed = fps * twin.RELEASE_S  # video speed: one frame per 20 ms release
    off = Offscreen(mujoco.MjModel.from_xml_path(str(twin.MODEL_XML)))
    enc = subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}",
                            "-r", f"{fps:g}", "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "23",
                            "-movflags", "+faststart", str(path)], stdin=subprocess.PIPE)
    for k in ks:
        enc.stdin.write(np.ascontiguousarray(off.render(rep, k, f"(video x{speed:g})")).tobytes())
    enc.stdin.close()
    if enc.wait():
        raise SystemExit("ffmpeg failed")
    print(f"{path}: {len(ks)} frames, {rep.t[ks[0]]:.2f}-{rep.t[ks[-1]]:.2f} s simulated, {len(ks) / fps:.1f} s of video")


def viewer(names, kinematic_only, demo_seconds, show_ui):
    model = mujoco.MjModel.from_xml_path(str(twin.MODEL_XML))
    data = mujoco.MjData(model)
    paint = Painter(model)
    ui = {"i": 0, "k": 0, "play": True, "speed": 1.0, "load": True}
    replays = {}

    def key(code):
        if code == 32:
            ui["play"] = not ui["play"]
        elif code in (262, 263) and not ui["play"]:
            ui["k"] += 1 if code == 262 else -1
        elif code in (264, 265):
            ui["speed"] = min(8.0, ui["speed"] * 2) if code == 265 else max(0.125, ui["speed"] / 2)
        elif code == ord("R"):
            ui["k"] = 0
        elif code == ord("F") and names[ui["i"]] in replays and replays[names[ui["i"]]].fault_k >= 0:
            ui["k"], ui["play"] = max(0, replays[names[ui["i"]]].fault_k - 50), False
        elif code in (ord("N"), ord("P")):
            ui["i"] = (ui["i"] + (1 if code == ord("N") else -1)) % len(names)
            ui["load"], ui["k"] = True, 0

    with mujoco.viewer.launch_passive(model, data, key_callback=key, show_left_ui=show_ui, show_right_ui=False) as v:
        track(v.cam, model)
        started, clock, carry = time.time(), time.time(), 0.0
        while v.is_running() and (demo_seconds <= 0 or time.time() - started < demo_seconds):
            name = names[ui["i"]]
            if ui["load"]:
                v.set_texts((mujoco.mjtFont.mjFONT_BIG, mujoco.mjtGridPos.mjGRID_TOPLEFT, f"calcolo di {name} ...", None))
                v.sync()
                if name not in replays:
                    replays[name] = kinematic(name) if kinematic_only else compute(name)
                track(v.cam, model, replays[name].hip)
                ui["load"], clock, carry = False, time.time(), 0.0
            rep = replays[name]
            now = time.time()
            if ui["play"]:
                carry += (now - clock) * ui["speed"] / twin.RELEASE_S
                ui["k"] += int(carry)
                carry -= int(carry)
            clock = now
            ui["k"] = int(np.clip(ui["k"], 0, len(rep.t) - 1))
            k = ui["k"]
            with v.lock():
                pose(model, data, rep, k)
                v.user_scn.ngeom = 0
                paint.markers(v.user_scn, data, rep, k)
            v.set_texts(paint.texts(rep, k, f"x{ui['speed']:g}" + ("" if ui["play"] else " PAUSA")))
            if rep.kind != "kinematic" and v.viewport is not None:
                v.set_figures((figure_rect(v.viewport.width, v.viewport.height), paint.figure(rep, k)))
            v.sync()
            time.sleep(1 / 60)
    # launch_passive renders in a daemon thread that destroys its GL context after close():
    # exiting before it finishes crashes the process (segfault or GLXBadDrawable on SwapBuffers).
    deadline = time.time() + 5
    while v._sim() is not None and time.time() < deadline:
        time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("replays", nargs="*")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--kinematic", action="store_true")
    ap.add_argument("--mp4")
    ap.add_argument("--png")
    ap.add_argument("--at", type=float, nargs="*", default=[0.0])
    ap.add_argument("--start", type=float, default=-1e9)
    ap.add_argument("--end", type=float, default=1e9)
    ap.add_argument("--fps", type=float, default=50, help="50 = real time (one frame per release), 25 = half speed")
    ap.add_argument("--demo-seconds", type=float, default=0, help="close the viewer after this many seconds (self-test)")
    ap.add_argument("--ui", action="store_true", help="show MuJoCo's left panel (TAB toggles it in the viewer)")
    args = ap.parse_args()
    if args.list or not args.replays:
        print(f"data: {DATA}")
        for name, (source, title) in CATALOG.items():
            present = source.startswith("stand:") or (DATA / source).exists()
            print(f"  {name:22s} {title}  [{source}{'' if present else ', MISSING'}]")
        return
    if args.mp4 or args.png:
        rep = kinematic(args.replays[0]) if args.kinematic else compute(args.replays[0])
        if rep.check:
            print(rep.check)
        if args.mp4:
            export_mp4(rep, args.mp4, args.start, args.end, args.fps)
        else:
            off = Offscreen(mujoco.MjModel.from_xml_path(str(twin.MODEL_XML)))
            for at in args.at:
                k = int(np.clip(np.searchsorted(rep.t, at - 1e-9), 0, len(rep.t) - 1))
                out = args.png.replace(".png", f"_t{rep.t[k]:.2f}.png")
                write_png(out, off.render(rep, k))
                print(out)
        return
    viewer(args.replays, args.kinematic, args.demo_seconds, args.ui)


if __name__ == "__main__":
    main()
