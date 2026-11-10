"""Shared helpers of the MATDOG MuJoCo twin: model, slot map, replay data, foot contact."""
import csv
import math
import pathlib

import mujoco
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
MODEL_XML = HERE / "generated" / "matdog.xml"
TICK = 2 * math.pi / 4096  # simple::kTickRad
LEGS = ("lf", "rf", "rh", "lh")  # firmware slot order
JOINTS = tuple(f"{l}_{j}_joint" for l in LEGS for j in ("hip", "upper_leg", "lower_leg"))
SLOT_NAMES = tuple(f"{l.upper()}_{j}" for l in LEGS for j in ("HIP", "UPPER", "LOWER"))
FOOT_RADIUS = 0.0149  # MATDOG_FOOT_CONTACT_GEOMETRY.yaml rigid_cylinder.radius_m (checked by build_model.py)
RELEASE_S = 0.02  # simple::kPeriodUs


def load_model():
    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    data = mujoco.MjData(model)
    qadr = np.array([model.jnt_qposadr[model.joint(j).id] for j in JOINTS])
    dadr = np.array([model.jnt_dofadr[model.joint(j).id] for j in JOINTS])
    assert list(qadr) == list(range(7, 19)) and list(dadr) == list(range(6, 18)), "slot order != qpos order"
    assert [model.actuator(i).name for i in range(model.nu)] == list(JOINTS)
    return model, data


def site_ids(model, suffix):
    return np.array([model.site(f"{l}_{suffix}").id for l in LEGS])


def contact_reference(data, center_sites, cylinder_geoms, normal=(0.0, 0.0, 1.0)):
    """Lowest point of each foot cylinder along -normal (the firmware's referenceM).

    Same definition as FootContact.cpp contactForwardKinematics, evaluated on the
    MuJoCo frames: radialDown = -(n - a (a.n)) / |...|, reference = center + r radialDown.
    """
    n = np.asarray(normal, float)
    axes = data.geom_xmat[cylinder_geoms].reshape(-1, 3, 3)[:, :, 2]  # geom z = foot cylinder axis
    perpendicular = n - axes * (axes @ n)[:, None]
    radial = -perpendicular / np.linalg.norm(perpendicular, axis=1)[:, None]
    return data.site_xpos[center_sites] + FOOT_RADIUS * radial


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    return {k: np.array([r[k] for r in rows]) for k in rows[0]}


def numeric(table, key):
    return table[key].astype(float)


def sequence(path):
    """A fw_bridge/export_sequences CSV: per release RAW (rig map) -> URDF radians."""
    t = read_csv(path)
    raw = np.stack([t[f"raw{i}"].astype(int) for i in range(12)], axis=1)
    t["raw"] = raw
    t["q"] = (raw - 2048) * TICK
    t["t"] = numeric(t, "t_s")
    return t


def delivered(path):
    """A delivered mujoco_replay CSV (q printed with 6 significant digits): exact RAW back."""
    t = read_csv(path)
    q = np.stack([numeric(t, f"q{i}_rad") for i in range(12)], axis=1)
    ticks = q / TICK
    raw = np.rint(ticks).astype(int) + 2048
    assert np.max(np.abs(ticks - np.rint(ticks))) < 0.05, "delivered q is not on the tick grid"  # 6 digits: <= 0.007 tick
    t["raw"], t["q"], t["t"] = raw, (raw - 2048) * TICK, numeric(t, "t_s")
    return t


def set_base(data, pos=(0, 0, 0), yaw=0.0, rotation=None):
    data.qpos[0:3] = pos
    if rotation is None:
        data.qpos[3:7] = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
    else:
        mujoco.mju_mat2Quat(data.qpos[3:7], np.ascontiguousarray(rotation, float).reshape(9))


def law_body_pose(table, k):
    """world_from_base_link of release k as the firmware solves it: the law's pose
    headed by the applied level correction (SimpleMotion.cpp headed() and the
    translation that keeps the nominal clearance over the same gait XY)."""
    yaw, roll, pitch = (float(table[c][k]) for c in ("body_yaw", "imu_roll", "imu_pitch"))
    t = [float(table[c][k]) for c in ("body_x", "body_y", "body_z")]
    cy, sy, cr, sr, cp, sp = math.cos(yaw), math.sin(yaw), math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    tilt = np.array([[cp, sp * sr, sp * cr], [0, cr, -sr], [-sp, cp * sr, cp * cr]])  # Ry(pitch) Rx(roll)
    r = tilt @ np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return np.array([t[0] + t[2] * r[0, 2], t[1] + t[2] * r[1, 2], t[2] * r[2, 2]]), r
