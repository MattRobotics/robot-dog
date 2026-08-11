#!/usr/bin/env python3
"""Model-first URDF contract for the MATDOG Geometry Compiler V5.

This module owns robot-definition facts only.  It parses topology, bounded
revolute joints, motor metadata, collision mesh references, per-mesh scales
and collision origins from the supplied URDF.  It contains no hardware
measurements, safety thresholds, parking poses, leg-name conventions or
canonical-URDF hash requirement.

Offline only: no Station, serial, motor command or EEPROM access.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable
import xml.etree.ElementTree as ET


class GeometryModelError(RuntimeError):
    """The URDF does not provide an unambiguous V5 geometry model."""


def _vector3(
    raw: str | None,
    *,
    default: tuple[float, float, float],
    field: str,
) -> tuple[float, float, float]:
    if raw is None:
        return default

    tokens = raw.split()
    if len(tokens) != 3:
        raise GeometryModelError(f"{field}: expected three values, got {raw!r}")

    values = tuple(float(token) for token in tokens)
    if not all(math.isfinite(value) for value in values):
        raise GeometryModelError(f"{field}: non-finite value in {raw!r}")
    return values  # type: ignore[return-value]


def _required_child_link(joint_node: ET.Element, tag: str, joint_name: str) -> str:
    node = joint_node.find(tag)
    value = node.get("link") if node is not None else None
    if not value:
        raise GeometryModelError(f"{joint_name}: missing <{tag} link=...>")
    return value


@dataclass(frozen=True)
class CollisionGeometrySpec:
    link_name: str
    filename: str
    scale: tuple[float, float, float]
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]


@dataclass(frozen=True)
class LinkSpec:
    name: str
    collision: CollisionGeometrySpec | None


@dataclass(frozen=True)
class JointSpec:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis_xyz: tuple[float, float, float]
    lower_limit_rad: float | None
    upper_limit_rad: float | None
    motor_id: int | None
    motor_direction: int | None
    document_index: int

    @property
    def is_actuated_revolute(self) -> bool:
        return (
            self.joint_type == "revolute"
            and self.motor_id is not None
            and self.motor_direction is not None
        )


@dataclass(frozen=True)
class ArticulatedBranch:
    """Independent top-level actuated branch derived from URDF topology."""

    root_joint_name: str
    joint_names: tuple[str, ...]
    link_names: tuple[str, ...]
    sort_motor_id: int


PAIR_CLASS_REVOLUTE_ADJACENT = "REVOLUTE_ADJACENT"
PAIR_CLASS_FIXED_ADJACENT = "FIXED_ADJACENT"
PAIR_CLASS_NON_ADJACENT = "NON_ADJACENT"

RELATION_SAME_BRANCH = "same_branch"
RELATION_BODY_VS_BRANCH = "body_vs_branch"
RELATION_CROSS_BRANCH = "cross_branch"
RELATION_BODY_INTERNAL = "body_internal"


@dataclass(frozen=True)
class RobotGeometryModel:
    urdf_path: Path
    root_link: str
    links: dict[str, LinkSpec]
    joints: dict[str, JointSpec]
    link_names_topological: tuple[str, ...]
    joint_names_topological: tuple[str, ...]
    actuated_joint_names: tuple[str, ...]
    collision_link_names: tuple[str, ...]
    parent_joint_by_link: dict[str, str]
    child_joint_names_by_link: dict[str, tuple[str, ...]]
    joint_chain_by_link: dict[str, tuple[str, ...]]
    actuated_chain_by_link: dict[str, tuple[str, ...]]
    branch_root_by_link: dict[str, str | None]
    branch_root_by_joint: dict[str, str]
    articulated_branches: tuple[ArticulatedBranch, ...]

    def collision_geometry(self, link_name: str) -> CollisionGeometrySpec:
        try:
            link = self.links[link_name]
        except KeyError as exc:
            raise GeometryModelError(f"unknown link: {link_name!r}") from exc
        if link.collision is None:
            raise GeometryModelError(f"{link_name}: no collision mesh in URDF")
        return link.collision

    def collision_mesh_path(self, link_name: str) -> Path:
        filename = self.collision_geometry(link_name).filename
        if filename.startswith("file://"):
            return Path(filename.removeprefix("file://")).resolve()
        if "://" in filename:
            raise GeometryModelError(
                f"{link_name}: unsupported collision mesh URI {filename!r}; "
                "V5 currently requires a filesystem path relative to the URDF or file://"
            )
        path = Path(filename)
        if not path.is_absolute():
            path = self.urdf_path.parent / path
        return path.resolve()

    def joint(self, joint_name: str) -> JointSpec:
        try:
            return self.joints[joint_name]
        except KeyError as exc:
            raise GeometryModelError(f"unknown joint: {joint_name!r}") from exc

    def active_revolute_pair(self, joint_name: str) -> tuple[str, str]:
        joint = self.joint(joint_name)
        if not joint.is_actuated_revolute:
            raise GeometryModelError(f"{joint_name}: not an actuated bounded revolute joint")
        return joint.parent_link, joint.child_link

    def classify_pair(self, link_a: str, link_b: str) -> str:
        if link_a not in self.links or link_b not in self.links:
            raise GeometryModelError(f"unknown link pair: {link_a!r}, {link_b!r}")

        for parent, child in ((link_a, link_b), (link_b, link_a)):
            joint_name = self.parent_joint_by_link.get(child)
            if joint_name is None:
                continue
            joint = self.joints[joint_name]
            if joint.parent_link != parent:
                continue
            if joint.joint_type == "fixed":
                return PAIR_CLASS_FIXED_ADJACENT
            return PAIR_CLASS_REVOLUTE_ADJACENT
        return PAIR_CLASS_NON_ADJACENT

    def pair_relation(self, link_a: str, link_b: str) -> str:
        branch_a = self.branch_root_by_link[link_a]
        branch_b = self.branch_root_by_link[link_b]
        if branch_a is None and branch_b is None:
            return RELATION_BODY_INTERNAL
        if branch_a is None or branch_b is None:
            return RELATION_BODY_VS_BRANCH
        if branch_a == branch_b:
            return RELATION_SAME_BRANCH
        return RELATION_CROSS_BRANCH

    def collision_pairs(self) -> tuple[tuple[str, str], ...]:
        return tuple(
            (link_a, link_b)
            for index, link_a in enumerate(self.collision_link_names)
            for link_b in self.collision_link_names[index + 1 :]
        )

    def path_obstruction_pairs(
        self,
        *,
        exclude_pair: tuple[str, str] | None = None,
    ) -> tuple[tuple[str, str], ...]:
        excluded = frozenset(exclude_pair) if exclude_pair is not None else None
        pairs: list[tuple[str, str]] = []
        for link_a, link_b in self.collision_pairs():
            if excluded is not None and frozenset((link_a, link_b)) == excluded:
                continue
            if self.classify_pair(link_a, link_b) == PAIR_CLASS_FIXED_ADJACENT:
                continue
            pairs.append((link_a, link_b))
        return tuple(pairs)

    def clearance_pairs(self) -> tuple[tuple[str, str], ...]:
        return tuple(
            pair
            for pair in self.collision_pairs()
            if self.classify_pair(*pair) == PAIR_CLASS_NON_ADJACENT
        )

    def relevant_actuated_joints_for_link(self, link_name: str) -> tuple[str, ...]:
        try:
            return self.actuated_chain_by_link[link_name]
        except KeyError as exc:
            raise GeometryModelError(f"unknown link: {link_name!r}") from exc

    def relevant_actuated_joints_for_pair(
        self,
        link_a: str,
        link_b: str,
    ) -> tuple[str, ...]:
        chain_a = self.relevant_actuated_joints_for_link(link_a)
        chain_b = self.relevant_actuated_joints_for_link(link_b)
        common = 0
        for joint_a, joint_b in zip(chain_a, chain_b):
            if joint_a != joint_b:
                break
            common += 1
        return chain_a[common:] + chain_b[common:]

    def home_pose(self) -> dict[str, float]:
        return {joint_name: 0.0 for joint_name in self.actuated_joint_names}

    def full_pose(self, overrides: dict[str, float]) -> dict[str, float]:
        unknown = sorted(set(overrides) - set(self.actuated_joint_names))
        if unknown:
            raise GeometryModelError(f"unknown actuated joints in pose: {unknown}")
        pose = self.home_pose()
        for joint_name, value in overrides.items():
            value = float(value)
            if not math.isfinite(value):
                raise GeometryModelError(f"{joint_name}: non-finite pose value {value!r}")
            pose[joint_name] = value
        return pose


def _parse_collision(link_node: ET.Element, link_name: str) -> CollisionGeometrySpec | None:
    collisions = link_node.findall("collision")
    if not collisions:
        return None
    if len(collisions) != 1:
        raise GeometryModelError(
            f"{link_name}: V5 currently requires exactly one collision element, found {len(collisions)}"
        )

    collision = collisions[0]
    mesh = collision.find("./geometry/mesh")
    if mesh is None:
        raise GeometryModelError(f"{link_name}: collision geometry is not an STL mesh")
    filename = mesh.get("filename")
    if not filename:
        raise GeometryModelError(f"{link_name}: collision mesh filename missing")

    scale = _vector3(
        mesh.get("scale"),
        default=(1.0, 1.0, 1.0),
        field=f"{link_name}.collision.mesh.scale",
    )
    if any(value == 0.0 for value in scale):
        raise GeometryModelError(f"{link_name}: collision mesh scale contains zero: {scale}")

    origin = collision.find("origin")
    origin_xyz = _vector3(
        origin.get("xyz") if origin is not None else None,
        default=(0.0, 0.0, 0.0),
        field=f"{link_name}.collision.origin.xyz",
    )
    origin_rpy = _vector3(
        origin.get("rpy") if origin is not None else None,
        default=(0.0, 0.0, 0.0),
        field=f"{link_name}.collision.origin.rpy",
    )
    return CollisionGeometrySpec(
        link_name=link_name,
        filename=filename,
        scale=scale,
        origin_xyz=origin_xyz,
        origin_rpy=origin_rpy,
    )


def _optional_hardware(joint_node: ET.Element, joint_name: str) -> tuple[int | None, int | None]:
    hardware_nodes = joint_node.findall("hardware")
    if len(hardware_nodes) > 1:
        raise GeometryModelError(f"{joint_name}: multiple <hardware> elements")
    if not hardware_nodes:
        return None, None

    hardware = hardware_nodes[0]
    motor_id_raw = hardware.findtext("motorId")
    direction_raw = hardware.findtext("motorDirection")
    if (motor_id_raw is None) != (direction_raw is None):
        raise GeometryModelError(
            f"{joint_name}: motorId and motorDirection must either both be present or both be absent"
        )
    if motor_id_raw is None:
        return None, None

    try:
        motor_id = int(motor_id_raw.strip())
        motor_direction = int(direction_raw.strip())  # type: ignore[union-attr]
    except ValueError as exc:
        raise GeometryModelError(f"{joint_name}: invalid integer motor metadata") from exc

    if motor_id <= 0:
        raise GeometryModelError(f"{joint_name}: motorId must be positive, got {motor_id}")
    if motor_direction not in (-1, 1):
        raise GeometryModelError(
            f"{joint_name}: motorDirection must be -1 or +1, got {motor_direction}"
        )
    return motor_id, motor_direction


def _parse_joint(joint_node: ET.Element, document_index: int) -> JointSpec:
    name = joint_node.get("name")
    joint_type = joint_node.get("type")
    if not name or not joint_type:
        raise GeometryModelError("joint missing name or type")
    if joint_type not in {"fixed", "revolute"}:
        raise GeometryModelError(
            f"{name}: unsupported joint type {joint_type!r}; V5 endpoint metrology supports fixed/revolute trees"
        )

    parent_link = _required_child_link(joint_node, "parent", name)
    child_link = _required_child_link(joint_node, "child", name)
    origin = joint_node.find("origin")
    origin_xyz = _vector3(
        origin.get("xyz") if origin is not None else None,
        default=(0.0, 0.0, 0.0),
        field=f"{name}.origin.xyz",
    )
    origin_rpy = _vector3(
        origin.get("rpy") if origin is not None else None,
        default=(0.0, 0.0, 0.0),
        field=f"{name}.origin.rpy",
    )
    axis = joint_node.find("axis")
    axis_xyz = _vector3(
        axis.get("xyz") if axis is not None else None,
        default=(1.0, 0.0, 0.0),
        field=f"{name}.axis.xyz",
    )

    lower: float | None = None
    upper: float | None = None
    if joint_type == "revolute":
        if math.sqrt(sum(value * value for value in axis_xyz)) == 0.0:
            raise GeometryModelError(f"{name}: revolute joint axis is zero")
        limit = joint_node.find("limit")
        lower_raw = limit.get("lower") if limit is not None else None
        upper_raw = limit.get("upper") if limit is not None else None
        if lower_raw is None or upper_raw is None:
            raise GeometryModelError(f"{name}: bounded revolute requires lower/upper limits")
        lower = float(lower_raw)
        upper = float(upper_raw)
        if not (math.isfinite(lower) and math.isfinite(upper) and lower < upper):
            raise GeometryModelError(f"{name}: invalid revolute limits [{lower}, {upper}]")

    motor_id, motor_direction = _optional_hardware(joint_node, name)
    if joint_type == "fixed" and (motor_id is not None or motor_direction is not None):
        raise GeometryModelError(f"{name}: fixed joint unexpectedly carries actuator metadata")
    if joint_type == "revolute" and (motor_id is None or motor_direction is None):
        raise GeometryModelError(
            f"{name}: bounded revolute lacks complete motor metadata; intended calibratable joints "
            "cannot be selected unambiguously without an explicit URDF metadata contract"
        )

    return JointSpec(
        name=name,
        joint_type=joint_type,
        parent_link=parent_link,
        child_link=child_link,
        origin_xyz=origin_xyz,
        origin_rpy=origin_rpy,
        axis_xyz=axis_xyz,
        lower_limit_rad=lower,
        upper_limit_rad=upper,
        motor_id=motor_id,
        motor_direction=motor_direction,
        document_index=document_index,
    )


def _ensure_unique(values: Iterable[str], label: str) -> None:
    values = tuple(values)
    if len(values) != len(set(values)):
        duplicates = sorted({value for value in values if values.count(value) > 1})
        raise GeometryModelError(f"duplicate {label}: {duplicates}")


def load_robot_geometry_model(
    urdf_path: Path,
    *,
    expected_actuated_joint_count: int | None = 12,
) -> RobotGeometryModel:
    """Load and validate one deterministic, tree-structured geometry model.

    For the accepted MATDOG URDF, `bounded revolute + complete motorId and
    motorDirection` is the explicit model-first selection rule and yields the
    intended 12 joints exactly.  A future passive bounded revolute must add an
    explicit calibratable marker rather than relying on a Python name rule.
    """
    path = Path(urdf_path).resolve()
    if not path.is_file():
        raise GeometryModelError(f"URDF not found: {path}")

    root = ET.parse(path).getroot()
    link_nodes = root.findall("link")
    joint_nodes = root.findall("joint")
    if not link_nodes:
        raise GeometryModelError("URDF contains no links")

    link_names = tuple(node.get("name") or "" for node in link_nodes)
    if any(not name for name in link_names):
        raise GeometryModelError("URDF link missing name")
    _ensure_unique(link_names, "link names")

    links = {
        name: LinkSpec(name=name, collision=_parse_collision(node, name))
        for name, node in zip(link_names, link_nodes)
    }
    parsed_joints = tuple(_parse_joint(node, index) for index, node in enumerate(joint_nodes))
    _ensure_unique((joint.name for joint in parsed_joints), "joint names")
    joints = {joint.name: joint for joint in parsed_joints}

    parent_joint_by_link: dict[str, str] = {}
    children_work: dict[str, list[str]] = {name: [] for name in link_names}
    for joint in parsed_joints:
        if joint.parent_link not in links or joint.child_link not in links:
            raise GeometryModelError(
                f"{joint.name}: parent/child references unknown link "
                f"{joint.parent_link!r}/{joint.child_link!r}"
            )
        if joint.child_link in parent_joint_by_link:
            raise GeometryModelError(f"{joint.child_link}: multiple parent joints")
        parent_joint_by_link[joint.child_link] = joint.name
        children_work[joint.parent_link].append(joint.name)

    roots = sorted(set(link_names) - set(parent_joint_by_link))
    if len(roots) != 1:
        raise GeometryModelError(f"URDF must have one root link, found {roots}")
    root_link = roots[0]

    link_order: list[str] = []
    joint_order: list[str] = []
    chains: dict[str, tuple[str, ...]] = {}
    visiting: set[str] = set()

    def walk(link_name: str, chain: tuple[str, ...]) -> None:
        if link_name in visiting:
            raise GeometryModelError(f"cycle detected at link {link_name}")
        if link_name in chains:
            raise GeometryModelError(f"link visited twice: {link_name}")
        visiting.add(link_name)
        chains[link_name] = chain
        link_order.append(link_name)
        for joint_name in children_work[link_name]:
            joint_order.append(joint_name)
            joint = joints[joint_name]
            walk(joint.child_link, chain + (joint_name,))
        visiting.remove(link_name)

    walk(root_link, ())
    if set(link_order) != set(link_names):
        raise GeometryModelError(
            f"URDF has disconnected links: {sorted(set(link_names) - set(link_order))}"
        )

    actuated_names_document = tuple(
        joint.name for joint in parsed_joints if joint.is_actuated_revolute
    )
    if expected_actuated_joint_count is not None and len(actuated_names_document) != expected_actuated_joint_count:
        raise GeometryModelError(
            f"expected exactly {expected_actuated_joint_count} model-selected actuated revolute joints, "
            f"found {len(actuated_names_document)}"
        )
    motor_ids = [joints[name].motor_id for name in actuated_names_document]
    if len(motor_ids) != len(set(motor_ids)):
        raise GeometryModelError(f"actuated motorId values are not unique: {motor_ids}")

    actuated_set = set(actuated_names_document)
    actuated_chain_by_link = {
        link_name: tuple(name for name in chain if name in actuated_set)
        for link_name, chain in chains.items()
    }
    branch_root_by_link = {
        link_name: (chain[0] if chain else None)
        for link_name, chain in actuated_chain_by_link.items()
    }
    branch_roots = {
        chain[0]
        for chain in actuated_chain_by_link.values()
        if chain
    }
    branch_root_by_joint: dict[str, str] = {}
    branches: list[ArticulatedBranch] = []
    for branch_root in branch_roots:
        branch_joint_names = tuple(
            name
            for name in joint_order
            if name in actuated_set
            and branch_root_by_link[joints[name].child_link] == branch_root
        )
        branch_link_names = tuple(
            name for name in link_order if branch_root_by_link[name] == branch_root
        )
        for name in branch_joint_names:
            branch_root_by_joint[name] = branch_root
        root_motor_id = joints[branch_root].motor_id
        assert root_motor_id is not None
        branches.append(
            ArticulatedBranch(
                root_joint_name=branch_root,
                joint_names=branch_joint_names,
                link_names=branch_link_names,
                sort_motor_id=root_motor_id,
            )
        )
    branches.sort(key=lambda branch: (branch.sort_motor_id, branch.root_joint_name))
    actuated_joint_names = tuple(
        joint_name for branch in branches for joint_name in branch.joint_names
    )

    collision_link_names = tuple(
        name for name in link_order if links[name].collision is not None
    )
    for joint_name in actuated_joint_names:
        joint = joints[joint_name]
        missing = [
            link for link in (joint.parent_link, joint.child_link)
            if links[link].collision is None
        ]
        if missing:
            raise GeometryModelError(
                f"{joint_name}: active revolute contact pair lacks collision mesh: {missing}"
            )

    return RobotGeometryModel(
        urdf_path=path,
        root_link=root_link,
        links=links,
        joints=joints,
        link_names_topological=tuple(link_order),
        joint_names_topological=tuple(joint_order),
        actuated_joint_names=actuated_joint_names,
        collision_link_names=collision_link_names,
        parent_joint_by_link=parent_joint_by_link,
        child_joint_names_by_link={
            link: tuple(names) for link, names in children_work.items()
        },
        joint_chain_by_link=chains,
        actuated_chain_by_link=actuated_chain_by_link,
        branch_root_by_link=branch_root_by_link,
        branch_root_by_joint=branch_root_by_joint,
        articulated_branches=tuple(branches),
    )
