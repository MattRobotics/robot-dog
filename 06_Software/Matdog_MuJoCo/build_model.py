#!/usr/bin/env python3
"""Derive the MATDOG MuJoCo model (MJCF) from the canonical REV01-FIX URDF.

Inputs, read and hash-verified, never modified:
  - 03_CAD/URDF/matt_robodog_rev00/ (URDF, 17 STL, SHA256SUMS.txt): the
    canonical REV01-FIX package (the directory keeps the rev00 name);
  - 06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml: the
    analytic foot cylinder, cross-checked against the firmware's
    src/motion/FootContactData.h.
Outputs: generated/matdog.xml and generated/PROVENANCE.json.

Kinematics, masses, CoM and inertia tensors are copied from the URDF. Visual
geometry is the canonical STL set. Everything the URDF does not define (floor,
contact and friction parameters, collision simplification) is a MODEL
ASSUMPTION listed in PROVENANCE.json. Actuator properties (armature, back-EMF
damping, friction) belong to the servo model (servo.py) and are set at run time.
"""
import hashlib
import json
import pathlib
import re
import sys
import xml.etree.ElementTree as ET

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[1]
URDF_DIR = REPO / "03_CAD/URDF/matt_robodog_rev00"
URDF = URDF_DIR / "matt_robodog_rev00.urdf"
FOOT_YAML = REPO / "06_Software/Matdog_Core/kinematics/MATDOG_FOOT_CONTACT_GEOMETRY.yaml"
FOOT_DATA_H = REPO / "05_Firmware/MATDOG_Controller/src/motion/FootContactData.h"
OUT = HERE / "generated"

# The REV01-FIX identity the firmware geometry is generated from (LegGeometryData.h).
CANONICAL_URDF_SHA256 = "ce63034e727e82ee82267fb97cea01cd6e171e850cb2d42cb42cf763ee3706f7"
LEGS = ("lf", "rf", "rh", "lh")  # firmware slot order; per leg hip, upper, lower
JOINTS = tuple(f"{l}_{j}_joint" for l in LEGS for j in ("hip", "upper_leg", "lower_leg"))

# MODEL ASSUMPTIONS (not in the URDF). Each is reported in PROVENANCE.json.
ASSUMPTIONS = {
    "timestep_s": (0.0005, "integration step; the servo model runs at the same rate"),
    "integrator": ("implicitfast", "joint damping (back-EMF) integrated implicitly"),
    "floor": ("rigid plane z=0", "flat, level terrain; no compliance besides MuJoCo soft contact"),
    "foot_friction": (0.8, "rubber tread on a typical floor: HYPOTHESIS, swept 0.5-1.0 by the replay tools"),
    "friction_cone": ("elliptic, impratio 10", "reduces the soft-contact creep of planted feet"),
    "contact_solref": ("0.004 1", "stiff soft contact (~0.1 mm static penetration at nominal load)"),
    "foot_collision": ("analytic cylinder from MATDOG_FOOT_CONTACT_GEOMETRY.yaml",
                       "radius and tread width as declared; the 2 mm end fillet is NOT modelled (sharp rim)"),
    "other_collision": ("convex hull of each canonical STL, floor only",
                        "detects body/leg ground contact; no self-collision (Geometry V5 owns it)"),
}


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def verify_package():
    sums = {}
    for line in (URDF_DIR / "SHA256SUMS.txt").read_text().splitlines():
        digest, name = line.split()
        actual = sha256(URDF_DIR / name)
        if actual != digest:
            sys.exit(f"REFUSED: {name} sha256 {actual} != SHA256SUMS {digest}")
        sums[name] = digest
    if sums["matt_robodog_rev00.urdf"] != CANONICAL_URDF_SHA256:
        sys.exit("REFUSED: the URDF is not the REV01-FIX identity of LegGeometryData.h")
    return sums


def foot_contact():
    """Cylinder of the canonical YAML, equal to the firmware's FootContactData.h."""
    text = FOOT_YAML.read_text()

    def vec(key):
        return [float(v) for v in re.search(key + r":\s*\[([^\]]+)\]", text).group(1).split(",")]

    def num(key):
        return float(re.search(key + r":\s*([-0-9.eE]+)", text).group(1))

    yaml = {"center": vec("center_in_foot_link_m"), "axis": vec("axis_in_foot_link_unit"),
            "radius": num("radius_m"), "tread": num("total_tread_width_m"),
            "fillet": num("end_fillet_radius_m"), "support": num("central_rigid_support_width_m")}
    header = re.search(r"kContact = \{\s*\{([^}]+)\},\s*\{([^}]+)\},\s*([^}]+)\}", FOOT_DATA_H.read_text())
    fw_center = [float(v) for v in header.group(1).split(",")]
    fw_axis = [float(v) for v in header.group(2).split(",")]
    fw_radius, fw_tread, fw_fillet, fw_support = [float(v) for v in header.group(3).split(",")[:4]]
    same = (fw_center == yaml["center"] and fw_axis == yaml["axis"] and fw_radius == yaml["radius"] and
            fw_tread == yaml["tread"] and fw_fillet == yaml["fillet"] and fw_support == yaml["support"])
    if not same or yaml["axis"] != [0.0, 1.0, 0.0]:
        sys.exit("REFUSED: foot contact YAML and FootContactData.h disagree (or axis is not foot Y)")
    return yaml


def xyz(element, attribute="xyz"):
    return [float(v) for v in element.get(attribute, "0 0 0").split()]


def fmt(values):
    return " ".join(repr(float(v)) for v in values)


def parse_urdf():
    root = ET.parse(URDF).getroot()
    links, joints = {}, {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if xyz(inertial.find("origin"), "rpy") != [0, 0, 0]:
            sys.exit(f"REFUSED: {link.get('name')} inertial rpy is not zero")
        i = inertial.find("inertia").attrib
        mesh = link.find("visual/geometry/mesh")
        if mesh.get("filename") != link.find("collision/geometry/mesh").get("filename"):
            sys.exit(f"REFUSED: {link.get('name')} visual and collision meshes differ")
        links[link.get("name")] = {
            "com": xyz(inertial.find("origin")), "mass": float(inertial.find("mass").get("value")),
            # MuJoCo fullinertia order: ixx iyy izz ixy ixz iyz
            "inertia": [float(i[k]) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")],
            "mesh": mesh.get("filename"), "scale": xyz(mesh, "scale")}
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        if xyz(origin, "rpy") != [0, 0, 0]:
            sys.exit(f"REFUSED: {joint.get('name')} origin rpy is not zero")
        entry = {"type": joint.get("type"), "parent": joint.find("parent").get("link"),
                 "child": joint.find("child").get("link"), "pos": xyz(origin)}
        if entry["type"] == "revolute":
            limit = joint.find("limit")
            entry.update(axis=xyz(joint.find("axis")), range=[float(limit.get("lower")), float(limit.get("upper"))],
                         effort=float(limit.get("effort")), velocity=float(limit.get("velocity")))
        joints[joint.get("name")] = entry
    if sorted(j for j, e in joints.items() if e["type"] == "revolute") != sorted(JOINTS):
        sys.exit("REFUSED: unexpected revolute joint set")
    return links, joints


def mjcf(links, joints, foot):
    children = {}
    for name, joint in joints.items():
        children.setdefault(joint["parent"], []).append((name, joint))
    order = {name: k for k, name in enumerate(JOINTS)}
    lines = []

    def body(link, joint_name, joint, depth):
        pad = "  " * depth
        data = links[link]
        pos = fmt(joint["pos"]) if joint else "0 0 0.25"
        lines.append(f'{pad}<body name="{link}" pos="{pos}">')
        if joint is None:
            lines.append(f'{pad}  <freejoint name="root"/>')
        elif joint["type"] == "revolute":
            lines.append(f'{pad}  <joint name="{joint_name}" axis="{fmt(joint["axis"])}" range="{fmt(joint["range"])}"/>')
        lines.append(f'{pad}  <inertial pos="{fmt(data["com"])}" mass="{data["mass"]!r}" fullinertia="{fmt(data["inertia"])}"/>')
        lines.append(f'{pad}  <geom class="visual" mesh="{link}"/>')
        if link.endswith("_foot_link"):
            leg = link[:2]
            # MuJoCo cylinder: size = radius, half length along the geom z axis (= foot Y).
            lines.append(f'{pad}  <geom name="{leg}_foot_cylinder" class="foot" type="cylinder" pos="{fmt(foot["center"])}" '
                         f'zaxis="{fmt(foot["axis"])}" size="{foot["radius"]!r} {foot["tread"] / 2!r}"/>')
            lines.append(f'{pad}  <site name="{leg}_foot_origin" pos="0 0 0"/>')
            lines.append(f'{pad}  <site name="{leg}_foot_cylinder_center" pos="{fmt(foot["center"])}"/>')
        else:
            lines.append(f'{pad}  <geom class="hull" mesh="{link}"/>')
        if joint is None:
            lines.append(f'{pad}  <site name="base_origin" pos="0 0 0"/>')
        for child_joint, child in sorted(children.get(link, []), key=lambda c: order.get(c[0], 99)):
            body(child["child"], child_joint, child, depth + 1)
        lines.append(f"{pad}</body>")

    body("base_link", None, None, 2)
    meshes = "\n".join(f'    <mesh name="{name}" file="{data["mesh"]}" scale="{fmt(data["scale"])}"/>'
                       for name, data in links.items())
    actuators = "\n".join(f'    <motor name="{j}" joint="{j}" gear="1"/>' for j in JOINTS)
    sensors = "\n".join(f'    <jointpos name="{j}_pos" joint="{j}"/>' for j in JOINTS)
    meshdir = pathlib.Path("../../..") / URDF_DIR.relative_to(REPO)
    mu = ASSUMPTIONS["foot_friction"][0]
    return f"""<mujoco model="matdog_rev01fix">
  <!-- DERIVED FILE, DO NOT EDIT: generated by 06_Software/Matdog_MuJoCo/build_model.py from
       {URDF.relative_to(REPO)} (sha256 {CANONICAL_URDF_SHA256}) and
       {FOOT_YAML.relative_to(REPO)}. Model assumptions: generated/PROVENANCE.json.
       Joint order = firmware slots LF, RF, RH, LH x hip, upper, lower (qpos[7:19]). -->
  <compiler angle="radian" meshdir="{meshdir.as_posix()}" autolimits="true"/>
  <option timestep="{ASSUMPTIONS['timestep_s'][0]}" integrator="implicitfast" cone="elliptic" impratio="10"/>
  <visual><global offwidth="1280" offheight="720"/><quality shadowsize="4096"/></visual>
  <default>
    <geom solref="0.004 1"/>
    <default class="visual"><geom type="mesh" contype="0" conaffinity="0" group="2" rgba="0.18 0.18 0.2 1"/></default>
    <default class="hull"><geom type="mesh" contype="1" conaffinity="0" group="3" rgba="0.8 0.3 0.3 0.3"/></default>
    <default class="foot"><geom contype="1" conaffinity="0" group="3" condim="3" friction="{mu} 0.005 0.0001" rgba="0.1 0.6 0.9 0.6"/></default>
    <site size="0.004" group="4" rgba="1 0.5 0 1"/>
  </default>
  <asset>
{meshes}
    <texture name="grid" type="2d" builtin="checker" rgb1="0.82 0.84 0.86" rgb2="0.72 0.74 0.76" width="512" height="512"/>
    <material name="grid" texture="grid" texrepeat="20 20" reflectance="0.05"/>
  </asset>
  <worldbody>
    <light pos="0.4 -0.6 1.2" dir="-0.3 0.5 -1" diffuse="0.8 0.8 0.8" castshadow="true"/>
    <geom name="floor" type="plane" size="0 0 0.05" material="grid" contype="0" conaffinity="1" condim="3" friction="{mu} 0.005 0.0001"/>
{chr(10).join(lines)}
  </worldbody>
  <actuator>
{actuators}
  </actuator>
  <sensor>
{sensors}
  </sensor>
</mujoco>
"""


def main():
    sums = verify_package()
    foot = foot_contact()
    links, joints = parse_urdf()
    OUT.mkdir(exist_ok=True)
    text = mjcf(links, joints, foot)
    (OUT / "matdog.xml").write_text(text)
    import mujoco  # only to prove the generated file compiles and to record the version
    model = mujoco.MjModel.from_xml_path(str(OUT / "matdog.xml"))
    provenance = {
        "generator": {"path": str(pathlib.Path(__file__).relative_to(REPO)), "sha256": sha256(pathlib.Path(__file__))},
        "mujoco_version": mujoco.__version__,
        "inputs": {
            "urdf_package": {"path": str(URDF_DIR.relative_to(REPO)), "sha256sums": sums},
            "foot_contact_yaml": {"path": str(FOOT_YAML.relative_to(REPO)), "sha256": sha256(FOOT_YAML)},
            "foot_contact_firmware": {"path": str(FOOT_DATA_H.relative_to(REPO)), "sha256": sha256(FOOT_DATA_H)},
        },
        "output": {"path": "generated/matdog.xml", "sha256": sha256(OUT / "matdog.xml")},
        "copied_from_urdf": ["joint origins, axes and limits", "link masses, CoM, full inertia tensors", "mesh files and scale"],
        "not_from_urdf_model_assumptions": {k: {"value": v[0], "note": v[1]} for k, v in ASSUMPTIONS.items()},
        "actuators": "12 torque motors (gear 1); torque, armature, back-EMF damping and friction come from servo.py",
        "model_summary": {"nbody": model.nbody, "njnt": model.njnt, "nu": model.nu, "ngeom": model.ngeom,
                          "total_mass_kg": float(sum(model.body_mass))},
    }
    (OUT / "PROVENANCE.json").write_text(json.dumps(provenance, indent=2) + "\n")
    print(f"generated/matdog.xml  bodies={model.nbody} joints={model.njnt} actuators={model.nu} "
          f"mass={sum(model.body_mass):.4f} kg  mujoco {mujoco.__version__}")


if __name__ == "__main__":
    main()
