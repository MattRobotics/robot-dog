"""Offline SI mass properties. No actuator, firmware, serial or network imports."""
from __future__ import annotations

from dataclasses import dataclass, replace
import importlib.util
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

PENDING = "PENDING_MEASUREMENT"
ROOT = Path(__file__).resolve().parents[3]
URDF_REL = "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf"
KIN = ROOT / "06_Software/Matdog_Core/kinematics"


def load_pure_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


# These two historical modules only define pure functions when imported.
fk = load_pure_module("matdog_rev01_historical_fk", KIN / "matdog_urdf_fk.py")
c4e = load_pure_module("matdog_rev01_historical_c4e",
                       KIN / "matdog_offline_static_stability_support_polygon.py")


def number(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Expected finite number, got {value!r}")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("Nonfinite value")
    return value


def vector(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("Expected three SI coordinates")
    return tuple(number(v) for v in value)


def matrix(value):
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError("Expected 3x3 tensor")
    return tuple(vector(row) for row in value)


IDENTITY = ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))


def transpose(a):
    return tuple(tuple(a[j][i] for j in range(3)) for i in range(3))


def multiply(a, b):
    return tuple(tuple(math.fsum(a[i][k] * b[k][j] for k in range(3))
                       for j in range(3)) for i in range(3))


def matvec(a, v):
    return tuple(math.fsum(a[i][k] * v[k] for k in range(3)) for i in range(3))


def add(a, b):
    return tuple(a[i] + b[i] for i in range(3))


def subtract(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def rotate_tensor(tensor, rotation):
    return multiply(multiply(rotation, tensor), transpose(rotation))


def rotation(rpy):
    return fk._rotation_from_rpy(*vector(rpy))


def principal_moments(tensor):
    """Deterministic Jacobi eigenvalues of a symmetric 3x3 tensor."""
    a = [list(row) for row in matrix(tensor)]
    scale = max(abs(v) for row in a for v in row)
    for _ in range(64):
        p, q = max(((0, 1), (0, 2), (1, 2)), key=lambda ij: abs(a[ij[0]][ij[1]]))
        if abs(a[p][q]) <= max(scale * 1e-15, 1e-300):
            break
        angle = .5 * math.atan2(2 * a[p][q], a[q][q] - a[p][p])
        c, s = math.cos(angle), math.sin(angle)
        app, aqq, apq = a[p][p], a[q][q], a[p][q]
        a[p][p] = c*c*app - 2*s*c*apq + s*s*aqq
        a[q][q] = s*s*app + 2*s*c*apq + c*c*aqq
        a[p][q] = a[q][p] = 0.
        for k in range(3):
            if k != p and k != q:
                akp, akq = a[k][p], a[k][q]
                a[k][p] = a[p][k] = c*akp - s*akq
                a[k][q] = a[q][k] = s*akp + c*akq
    else:
        raise ValueError("Eigenvalue solver did not converge")
    return tuple(sorted(a[i][i] for i in range(3)))


def validate_tensor(tensor):
    tensor = matrix(tensor)
    scale = max(abs(v) for row in tensor for v in row)
    tolerance = max(scale * 1e-12, 1e-300)
    if any(abs(tensor[i][j] - tensor[j][i]) > tolerance
           for i in range(3) for j in range(3)):
        raise ValueError("Asymmetric inertia tensor")
    moments = principal_moments(tensor)
    if moments[0] < -tolerance:
        raise ValueError("Negative principal moment")
    if moments[2] > moments[0] + moments[1] + tolerance:
        raise ValueError("Principal moments violate physical triangle inequality")
    return {"principal_moments_kg_m2": moments,
            "symmetric": True, "positive_semidefinite": True,
            "positive_definite": moments[0] > tolerance,
            "triangle_inequality": True}


@dataclass(frozen=True)
class Body:
    name: str
    mass: float
    com: tuple
    inertia: tuple
    evidence: str = "HISTORICAL_REV00"

    def __post_init__(self):
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("Body name required")
        mass = number(self.mass)
        if mass <= 0:
            raise ValueError("Mass must be positive; omission is never a zero body")
        com, inertia = vector(self.com), matrix(self.inertia)
        validate_tensor(inertia)
        # Remove only symmetry roundoff within the validated relative tolerance.
        # A materially asymmetric input was already rejected above.
        inertia = tuple(tuple(inertia[i][j] if i == j else
                              .5*inertia[i][j] + .5*inertia[j][i]
                              for j in range(3)) for i in range(3))
        object.__setattr__(self, "mass", mass)
        object.__setattr__(self, "com", com)
        object.__setattr__(self, "inertia", inertia)


def load_baseline(path=ROOT / URDF_REL):
    """URDF tensors are about the inertial origin, in the inertial axes."""
    root = ET.parse(path).getroot()
    bodies = {}
    for link in root.findall("link"):
        name = link.attrib["name"]
        node = link.find("inertial")
        if node is None or name in bodies:
            raise ValueError(f"Missing/duplicate inertia: {name}")
        origin = node.find("origin")
        xyz = tuple(float(v) for v in origin.attrib["xyz"].split())
        rpy = tuple(float(v) for v in origin.attrib.get("rpy", "0 0 0").split())
        a = node.find("inertia").attrib
        i = tuple(tuple(float(a[key]) for key in row) for row in
                  (("ixx", "ixy", "ixz"), ("ixy", "iyy", "iyz"), ("ixz", "iyz", "izz")))
        bodies[name] = Body(name, float(node.find("mass").attrib["value"]),
                            vector(xyz), rotate_tensor(i, rotation(rpy)))
    return bodies


def link_transforms(path, names, pose):
    joints = fk.load_urdf_joints(path)
    required = {n for n, j in joints.items() if j.joint_type != "fixed"}
    if set(pose) != required:
        raise ValueError("Explicit pose must contain exactly all movable URDF joints")
    pose = {n: number(v) for n, v in pose.items()}
    # Validate parent uniqueness, connectivity and cycles before historical FK.
    parents = {}
    for j in joints.values():
        if j.child_link in parents or j.parent_link not in names or j.child_link not in names:
            raise ValueError("Invalid URDF link tree")
        parents[j.child_link] = j.parent_link
    if "base_link" not in names or "base_link" in parents or len(parents) != len(names)-1:
        raise ValueError("Expected one rooted URDF tree with base_link as root")
    for name in names:
        seen, current = set(), name
        while current != "base_link":
            if current in seen or current not in parents:
                raise ValueError("Disconnected/cyclic URDF")
            seen.add(current)
            current = parents[current]
    return {name: fk.forward_kinematics(path, "base_link", name, pose).tip_transform
            for name in names}


def transform_body(body, transform):
    r = tuple(tuple(transform[i][j] for j in range(3)) for i in range(3))
    t = tuple(transform[i][3] for i in range(3))
    return replace(body, com=add(matvec(r, body.com), t),
                   inertia=rotate_tensor(body.inertia, r))


def aggregate(bodies):
    """All bodies must share axes. Output inertia is about aggregate CoM."""
    bodies = list(bodies)
    if not bodies or len({b.name for b in bodies}) != len(bodies):
        raise ValueError("Nonempty, uniquely named body collection required")
    mass = math.fsum(b.mass for b in bodies)
    com = tuple(math.fsum(b.mass * b.com[k] for b in bodies) / mass for k in range(3))
    # I_C = sum(R I_i R^T + m_i (||d_i||² E - d_i d_i^T)).
    distances = [(b, subtract(b.com, com)) for b in bodies]
    inertia = tuple(tuple(math.fsum(
        b.inertia[i][j] + b.mass * (
            (math.fsum(v*v for v in d) if i == j else 0.) - d[i]*d[j])
        for b, d in distances) for j in range(3)) for i in range(3))
    validate_tensor(inertia)
    return Body("aggregate", mass, com, inertia, "OFFLINE_COMPUTED")


def at_pose(links, pose, path=ROOT / URDF_REL):
    transforms = link_transforms(path, links, pose)
    return aggregate(transform_body(b, transforms[n]) for n, b in links.items())


def body_dict(body):
    return {"mass_kg": body.mass, "com_m": body.com,
            "inertia_at_com_kg_m2": body.inertia, "evidence": body.evidence}


def missing(value, prefix=""):
    if value == PENDING or value is None:
        return [prefix]
    if isinstance(value, dict):
        return [p for k, v in value.items() for p in missing(v, f"{prefix}.{k}".strip("."))]
    if isinstance(value, (list, tuple)):
        return [p for i, v in enumerate(value) for p in missing(v, f"{prefix}[{i}]" )]
    return []


def evidence(value):
    if not isinstance(value, str) or not value.strip() or value == PENDING:
        raise ValueError("Explicit evidence/coverage statement required")
    return value


def resolve_budget(baseline, budget):
    """Replace full links or attach disjoint additions. Never subtract unknown mass.

    All numerical results remain REV01_PENDING, including fully populated offline
    scenarios. Real assembly qualification is outside this calculator's authority.
    """
    if budget.get("schema") != 1 or budget.get("readiness") != "REV01_PENDING":
        raise ValueError("Schema 1 and REV01_PENDING required")
    if budget.get("units") != {"mass": "kg", "length": "m", "angle": "rad", "inertia": "kg*m^2"}:
        raise ValueError("SI units required; convert explicitly before import")
    evidence(budget["assembly_revision"])
    reviews = budget["link_reviews"]
    records = budget["components"]
    if {r["link_name"] for r in reviews} != set(baseline) or len(reviews) != len(baseline):
        raise ValueError("One review for each of the 17 baseline links required")
    if len({r["id"] for r in records}) != len(records):
        raise ValueError("Duplicate component ID")
    issues = [p for k in ("inventory_review", "assembly_configuration_description")
              for p in missing(budget.get(k), k)]
    for k in ("inventory_review", "assembly_configuration_description"):
        if not missing(budget.get(k), k):
            evidence(budget[k])
    active = []
    for r in records:
        disposition = r["disposition"]
        if disposition == PENDING:
            issues.extend(missing(r, "components." + r["id"]))
            continue
        if disposition in {"absent", "included_in_link"}:
            fields = ("evidence",) if disposition == "absent" else ("evidence", "parent_link", "coverage_note")
        elif disposition == "add":
            fields = ("parent_link", "mass_kg", "com_in_parent_m", "inertia_at_com_component_kg_m2",
                      "component_rpy_in_parent_rad", "evidence", "coverage_note")
        elif disposition == "replace_link":
            fields = ("parent_link", "mass_kg", "com_in_parent_m", "inertia_at_com_component_kg_m2",
                      "component_rpy_in_parent_rad", "evidence", "coverage_note")
        else:
            raise ValueError("Disposition must be pending, absent, included_in_link, add or replace_link")
        pending = [p for k in fields for p in missing(r.get(k), f"components.{r['id']}.{k}")]
        issues.extend(pending)
        if pending:
            continue
        evidence(r["evidence"])
        if disposition != "absent":
            if r["parent_link"] not in baseline:
                raise ValueError("Unknown mounting/replacement link")
            evidence(r["coverage_note"])
        active.append(r)
    for r in reviews:
        if r["decision"] == PENDING or r["evidence"] == PENDING:
            issues.extend(missing(r, "link_reviews." + r["link_name"]))
        elif r["decision"] not in {"retain_historical_offline", "replace_complete_link"}:
            raise ValueError("Invalid link review decision")
        else:
            evidence(r["evidence"])
    if issues:
        return {"readiness": "REV01_PENDING", "complete": False,
                "pending_fields": sorted(set(issues)), "result": None, "links": None,
                "historical_reference_mass_kg": math.fsum(b.mass for b in baseline.values()),
                "hardware_verified": False, "command_eligible": False}
    links = dict(baseline)
    replacements = {}
    additions = {}
    for r in active:
        if r["disposition"] not in {"add", "replace_link"}:
            continue
        parent = r["parent_link"]
        body = Body(r["id"], number(r["mass_kg"]), vector(r["com_in_parent_m"]),
                    rotate_tensor(matrix(r["inertia_at_com_component_kg_m2"]),
                                  rotation(r["component_rpy_in_parent_rad"])), "ASSUMED_OFFLINE_INPUT")
        if r["disposition"] == "replace_link":
            if parent in replacements:
                raise ValueError("Multiple full replacements of the same link")
            replacements[parent] = replace(body, name=parent)
        else:
            additions.setdefault(parent, []).append(body)
    for r in reviews:
        replacement_expected = r["decision"] == "replace_complete_link"
        if replacement_expected != (r["link_name"] in replacements):
            raise ValueError("Review/replacement mismatch")
    links.update(replacements)
    for parent, attached in additions.items():
        links[parent] = replace(aggregate([links[parent], *attached]), name=parent,
                                evidence="ASSUMED_OFFLINE_INPUT")
    return {"readiness": "REV01_PENDING", "complete": True, "pending_fields": [],
            "links": links, "result": None, "hardware_verified": False,
            "command_eligible": False,
            "assumptions": ["Historical retained links are not real-assembly verified",
                            "Coverage notes assert additions are disjoint from retained/replaced links",
                            "Each addition is rigidly attached to its stated parent link"]}


def sensitivity(total_mass, total_com, accessory_mass, accessory_com):
    """Exact additive shift + analytic Jacobians in a shared frame."""
    m0, m = number(total_mass), number(accessory_mass)
    if m0 <= 0 or m < 0:
        raise ValueError("Positive baseline and nonnegative perturbation required")
    c, r = vector(total_com), vector(accessory_com)
    lever = subtract(r, c)
    return {"delta_com_m": tuple(m/(m0+m)*v for v in lever),
            "d_com_d_mass_m_per_kg": tuple(m0/(m0+m)**2*v for v in lever),
            "d_com_d_position": m/(m0+m)}


def project_support(com_base, translation_world, rpy_world, contacts_world,
                    plane_origin_world, plane_normal_world, gravity_world, plane_tolerance_m,
                    required_margin_m=0.):
    """Geometric offline projection along gravity, reusing unmodified C4-E.

    Inputs are geometry, not evidence of loaded/confirmed contacts or freshness.
    Signed margin uses C4-E Euclidean distance to boundary segments.
    """
    c = add(matvec(rotation(rpy_world), vector(com_base)), vector(translation_world))
    origin = vector(plane_origin_world)
    n = vector(plane_normal_world)
    g = vector(gravity_world)
    dot = lambda a, b: math.fsum(x*y for x, y in zip(a, b))
    norm = math.sqrt(dot(n, n))
    gnorm = math.sqrt(dot(g, g))
    tolerance = number(plane_tolerance_m)
    margin_required = number(required_margin_m)
    if norm <= 0 or gnorm <= 0 or tolerance < 0 or margin_required < 0:
        raise ValueError("Invalid plane, gravity or policy")
    n = tuple(v/norm for v in n)
    g = tuple(v/gnorm for v in g)
    denominator = dot(n, g)
    if abs(denominator) <= 1e-12:
        raise ValueError("Gravity parallel to support plane")
    contacts = [vector(p) for p in contacts_world]
    if len(contacts) < 3 or any(abs(dot(n, subtract(p, origin))) > tolerance for p in contacts):
        raise ValueError("Insufficient/coplanarity-invalid contact geometry")
    projected = add(c, tuple(-dot(n, subtract(c, origin))/denominator*v for v in g))
    reference = min(IDENTITY, key=lambda axis: abs(dot(axis, n)))
    cross = lambda a, b: (a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0])
    u0 = cross(reference, n)
    unorm = math.sqrt(dot(u0, u0))
    u = tuple(v/unorm for v in u0)
    v = cross(n, u)
    uv = lambda p: (dot(subtract(p, origin), u), dot(subtract(p, origin), v))
    hull = c4e._convex_hull([uv(p) for p in contacts])
    if len(hull) < 3:
        raise ValueError("Degenerate support polygon")
    margin = c4e._polygon_signed_margin(uv(projected), hull)
    return {"com_world_m": c, "projected_com_world_m": projected,
            "support_hull_plane_m": hull, "signed_margin_m": margin,
            "geometrically_inside": margin >= margin_required,
            "hardware_verified": False, "command_eligible": False}
