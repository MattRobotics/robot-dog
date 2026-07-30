#!/usr/bin/env python3
"""Deterministic offline validator for the MATDOG Milestone I foundation."""

from __future__ import annotations

import argparse
import csv
import hashlib
import sys
import xml.etree.ElementTree as ET
from collections import Counter, defaultdict
from pathlib import Path


ROBOT_REPOSITORY = "MattRobotics/robot-dog"
ROBOT_BASE = "a6dc1184f56956dad696b3bcc24d74f375edb5b7"
NORMA_REPOSITORY = "MattRobotics/norma-core"
NORMA_MAIN = "32e3222c87016b7f5d7c1c1da497a4cea3e7b80a"
NORMA_PR_HEAD = "b06cc2bf2e36fb5bbaae12e48c5998c7668862ef"
XGO_REPOSITORY = "MattRobotics/xgolite-low-level-reconstruction"
XGO_TAG = "xgolite-static-closure-h2-2026-07-30"
XGO_COMMIT = "a1b34a8594e5bc76c76b1e3ddf89a3aef2b98298"

CLASSIFICATIONS = {
    "MATDOG_VERIFIED",
    "MATDOG_DERIVED",
    "HARDWARE_OBSERVATION",
    "NORMACORE_MATDOG_FORK_MAIN_FACT",
    "NORMACORE_EXPERIMENTAL_PR",
    "NORMACORE_UPSTREAM_REFERENCE",
    "NORMACORE_GENERIC_REFERENCE",
    "XGOLITE_ARCHITECTURAL_REFERENCE",
    "CORROBORATED",
    "UNKNOWN",
    "DECISION_REQUIRED",
    "SUPERSEDED",
}
CONFIDENCE = {"HIGH", "MEDIUM", "LOW", "NOT_APPLICABLE"}
PROFILE_STATUSES = {
    "validated",
    "partially-validated",
    "software-ready",
    "hardware-pending",
    "blocked",
    "unknown",
}
LEGS = {"LF", "RF", "RH", "LH"}
JOINTS = {
    "lf_hip_joint",
    "lf_upper_leg_joint",
    "lf_lower_leg_joint",
    "rf_hip_joint",
    "rf_upper_leg_joint",
    "rf_lower_leg_joint",
    "rh_hip_joint",
    "rh_upper_leg_joint",
    "rh_lower_leg_joint",
    "lh_hip_joint",
    "lh_upper_leg_joint",
    "lh_lower_leg_joint",
}
SERVO_IDS = {11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43}
FROZEN_UNKNOWN = {
    "C-UNKNOWN-WORLD",
    "C-UNKNOWN-CONTACTS-22",
    "C-UNKNOWN-SAFE",
    "C-UNKNOWN-FIRST-STAND",
    "C-UNKNOWN-RATE",
    "C-UNKNOWN-IMU",
}

REGISTRY_HEADERS = {
    "source_manifest.csv": [
        "source_id", "repository", "ref", "path", "sha256", "source_class",
        "authority", "scope", "temporal_status", "notes",
    ],
    "source_claim_registry.csv": [
        "claim_id", "domain", "statement", "classification", "confidence",
        "source_repository", "source_ref", "source_path", "source_locator",
        "units", "applies_to", "supersedes", "conflicts_with", "notes",
    ],
    "source_conflict_registry.csv": [
        "conflict_id", "domain", "left_claim_id", "right_claim_id",
        "resolution", "status", "classification", "source_id", "notes",
    ],
    "joint_registry.csv": [
        "joint_id", "leg", "leg_order", "joint_order", "joint_kind",
        "urdf_joint_name", "parent_link", "child_link", "joint_type",
        "origin_xyz_m", "origin_rpy_rad", "axis", "limit_lower_rad",
        "limit_upper_rad", "effort_nm", "velocity_rad_s", "units", "servo_id",
        "encoder_to_q_direction", "urdf_motor_direction", "zero_encoder_tick",
        "calibration_status", "min_profile_id", "max_profile_id", "source_id",
        "evidence_path",
    ],
    "frame_registry.csv": [
        "frame_id", "parent_frame", "frame_type", "origin_xyz_m",
        "origin_rpy_rad", "axes", "units", "status", "classification",
        "source_id", "notes",
    ],
    "servo_mapping_registry.csv": [
        "mapping_id", "leg", "joint_name", "servo_id", "direction",
        "direction_semantics", "zero_tick", "raw_q0_tick",
        "digital_zero_offset_i16", "final_readback_tick", "status",
        "source_id", "evidence_path",
    ],
    "calibration_registry.csv": [
        "profile_id", "leg", "joint_name", "servo_id", "contact_side",
        "software_status", "hardware_status", "probe_sign", "urdf_limit_tick",
        "guard_tick", "baseline_target_tick", "prerequisite_targets",
        "coarse_contact_tick", "fine_contact_tick", "repeatability_spread_tick",
        "measured_contact_tick", "operational_safe_limit_tick", "units",
        "source_id", "evidence_path", "notes",
    ],
    "limit_registry.csv": [
        "limit_id", "joint_name", "servo_id", "side", "limit_type", "value",
        "units", "status", "classification", "is_operational_safe_limit",
        "source_id", "evidence_path", "notes",
    ],
    "decision_registry.csv": [
        "decision_id", "question", "why_it_matters", "required_evidence",
        "blocked_work", "owner", "target_milestone", "status",
    ],
    "unresolved_registry.csv": [
        "unresolved_id", "domain", "question", "current_state",
        "required_evidence", "impact", "decision_id", "status",
        "classification", "source_id",
    ],
}

ID_COLUMNS = {
    "source_manifest.csv": "source_id",
    "source_claim_registry.csv": "claim_id",
    "source_conflict_registry.csv": "conflict_id",
    "joint_registry.csv": "joint_id",
    "frame_registry.csv": "frame_id",
    "servo_mapping_registry.csv": "mapping_id",
    "calibration_registry.csv": "profile_id",
    "limit_registry.csv": "limit_id",
    "decision_registry.csv": "decision_id",
    "unresolved_registry.csv": "unresolved_id",
}


def split_ids(value: str) -> list[str]:
    return [item for item in value.split(";") if item]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def is_relative_repo_path(value: str) -> bool:
    path = Path(value)
    return bool(value) and not path.is_absolute() and ".." not in path.parts


def parse_vector(value: str, size: int) -> tuple[float, ...]:
    parts = value.split()
    if len(parts) != size:
        raise ValueError(f"expected {size} scalars")
    return tuple(float(part) for part in parts)


def load_registries(root: Path, errors: list[str]) -> dict[str, list[dict[str, str]]]:
    registry_root = root / "06_Software/Matdog_Core/milestone_i/registries"
    loaded: dict[str, list[dict[str, str]]] = {}
    for name, expected in REGISTRY_HEADERS.items():
        path = registry_root / name
        if not path.is_file():
            errors.append(f"missing registry: {path.relative_to(root)}")
            loaded[name] = []
            continue
        try:
            with path.open("r", encoding="utf-8", newline="") as handle:
                reader = csv.DictReader(handle)
                if reader.fieldnames != expected:
                    errors.append(f"{name}: schema mismatch: {reader.fieldnames!r}")
                rows = list(reader)
        except (OSError, UnicodeError, csv.Error) as exc:
            errors.append(f"{name}: unreadable CSV: {exc}")
            loaded[name] = []
            continue
        for number, row in enumerate(rows, start=2):
            if None in row:
                errors.append(f"{name}:{number}: extra CSV fields")
            if any(value is None for value in row.values()):
                errors.append(f"{name}:{number}: missing CSV field")
        id_column = ID_COLUMNS[name]
        ids = [row.get(id_column, "") for row in rows]
        duplicates = sorted(key for key, count in Counter(ids).items() if count > 1)
        if "" in ids:
            errors.append(f"{name}: empty {id_column}")
        if duplicates:
            errors.append(f"{name}: duplicate IDs: {duplicates}")
        loaded[name] = rows
    return loaded


def check_sources(root: Path, rows: list[dict[str, str]], errors: list[str]) -> set[str]:
    ids = {row["source_id"] for row in rows}
    for row in rows:
        label = row["source_id"]
        if not row["repository"] or not row["ref"] or not row["path"]:
            errors.append(f"source {label}: repository/ref/path must be non-empty")
        if not is_relative_repo_path(row["path"]):
            errors.append(f"source {label}: path must be repository-relative")
        if row["source_class"] not in CLASSIFICATIONS:
            errors.append(f"source {label}: invalid source_class {row['source_class']!r}")
        digest = row["sha256"]
        if digest and (len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest)):
            errors.append(f"source {label}: malformed sha256")
        if row["repository"] == ROBOT_REPOSITORY:
            if row["ref"] != ROBOT_BASE:
                errors.append(f"source {label}: wrong robot-dog base ref")
            local = root / row["path"]
            if not local.is_file():
                errors.append(f"source {label}: local path does not exist: {row['path']}")
            elif not digest:
                errors.append(f"source {label}: local source hash is empty")
            elif sha256(local) != digest:
                errors.append(f"source {label}: local checksum mismatch")
        elif row["repository"] == NORMA_REPOSITORY:
            if row["source_class"] == "NORMACORE_EXPERIMENTAL_PR":
                if row["ref"] != NORMA_PR_HEAD:
                    errors.append(f"source {label}: wrong NormaCore PR head")
            elif row["ref"] != NORMA_MAIN:
                errors.append(f"source {label}: wrong NormaCore main ref")
        elif row["repository"] == XGO_REPOSITORY:
            if row["ref"] != XGO_TAG:
                errors.append(f"source {label}: wrong XGoLite tag")
        elif row["repository"] == "norma-core/norma-core":
            if row["source_class"] != "NORMACORE_UPSTREAM_REFERENCE":
                errors.append(f"source {label}: upstream must be reference-only")
        else:
            errors.append(f"source {label}: unrecognized repository {row['repository']!r}")

    xgo_anchor = next((row for row in rows if row["source_id"] == "X-README"), None)
    if not xgo_anchor or XGO_COMMIT not in xgo_anchor["notes"]:
        errors.append("source manifest: XGoLite tag commit pin is missing")
    if not any(row["source_id"] == "N-UPSTREAM" for row in rows):
        errors.append("source manifest: external NormaCore upstream row missing")
    return ids


def check_claims(
    root: Path,
    rows: list[dict[str, str]],
    source_rows: list[dict[str, str]],
    errors: list[str],
) -> set[str]:
    claim_ids = {row["claim_id"] for row in rows}
    statements = [row["statement"].strip() for row in rows]
    duplicates = sorted(item for item, count in Counter(statements).items() if item and count > 1)
    if duplicates:
        errors.append(f"claims: duplicate statements: {duplicates}")
    source_triples = {(r["repository"], r["ref"], r["path"]) for r in source_rows}
    for row in rows:
        label = row["claim_id"]
        if row["classification"] not in CLASSIFICATIONS:
            errors.append(f"claim {label}: invalid classification {row['classification']!r}")
        if row["confidence"] not in CONFIDENCE:
            errors.append(f"claim {label}: invalid confidence {row['confidence']!r}")
        for field in ("domain", "statement", "source_repository", "source_ref", "source_path", "source_locator", "applies_to"):
            if not row[field]:
                errors.append(f"claim {label}: required field {field} is empty")
        if not is_relative_repo_path(row["source_path"]):
            errors.append(f"claim {label}: source_path must be repository-relative")
        triple = (row["source_repository"], row["source_ref"], row["source_path"])
        if triple not in source_triples:
            errors.append(f"claim {label}: source ref/path is not registered in source manifest")
        if row["source_repository"] == ROBOT_REPOSITORY and not (root / row["source_path"]).is_file():
            errors.append(f"claim {label}: local source path does not exist")
        if label in FROZEN_UNKNOWN and row["classification"] != "UNKNOWN":
            errors.append(f"claim {label}: frozen UNKNOWN was improperly promoted")
        if row["source_repository"] == XGO_REPOSITORY:
            if row["classification"] != "XGOLITE_ARCHITECTURAL_REFERENCE":
                errors.append(f"claim {label}: XGo source promoted as MATDOG fact")
            if row["domain"] not in {"architecture", "interface"}:
                errors.append(f"claim {label}: XGo source used outside architecture/interface")
        for linked in split_ids(row["supersedes"]) + split_ids(row["conflicts_with"]):
            if linked not in claim_ids:
                errors.append(f"claim {label}: unknown linked claim {linked}")
    return claim_ids


def check_source_links(
    root: Path,
    registries: dict[str, list[dict[str, str]]],
    source_ids: set[str],
    errors: list[str],
) -> None:
    for name, rows in registries.items():
        if name in {"source_manifest.csv", "source_claim_registry.csv", "decision_registry.csv"}:
            continue
        for number, row in enumerate(rows, start=2):
            for source_id in split_ids(row.get("source_id", "")):
                if source_id not in source_ids:
                    errors.append(f"{name}:{number}: unknown source ID {source_id}")
            evidence = row.get("evidence_path", "")
            if evidence:
                if not is_relative_repo_path(evidence):
                    errors.append(f"{name}:{number}: evidence path must be repository-relative")
                elif not (root / evidence).is_file():
                    errors.append(f"{name}:{number}: evidence path does not exist: {evidence}")


def check_joints_and_urdf(
    root: Path,
    joints: list[dict[str, str]],
    servos: list[dict[str, str]],
    errors: list[str],
) -> None:
    names = {row["urdf_joint_name"] for row in joints}
    if names != JOINTS or len(joints) != 12:
        errors.append(f"joints: expected exact canonical 12; got {sorted(names)}")
    legs = {row["leg"] for row in joints}
    if legs != LEGS:
        errors.append(f"joints: expected legs {sorted(LEGS)}; got {sorted(legs)}")
    per_leg = Counter(row["leg"] for row in joints)
    if any(per_leg[leg] != 3 for leg in LEGS):
        errors.append(f"joints: expected three joints per leg; got {dict(per_leg)}")
    try:
        joint_servo_ids = [int(row["servo_id"]) for row in joints]
    except ValueError:
        errors.append("joints: non-integer servo ID")
        joint_servo_ids = []
    if set(joint_servo_ids) != SERVO_IDS or len(set(joint_servo_ids)) != 12:
        errors.append("joints: servo IDs are not the exact unique canonical set")
    for row in joints:
        label = row["joint_id"]
        if row["encoder_to_q_direction"] not in {"-1", "1"}:
            errors.append(f"joint {label}: direction must be -1 or 1")
        if not row["units"]:
            errors.append(f"joint {label}: units are empty")
        for field in ("origin_xyz_m", "origin_rpy_rad", "axis"):
            try:
                parse_vector(row[field], 3)
            except ValueError as exc:
                errors.append(f"joint {label}: invalid {field}: {exc}")
        for field in ("limit_lower_rad", "limit_upper_rad", "effort_nm", "velocity_rad_s"):
            try:
                float(row[field])
            except ValueError:
                errors.append(f"joint {label}: invalid numeric {field}")

    urdf_path = root / "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf"
    try:
        urdf_root = ET.parse(urdf_path).getroot()
    except (OSError, ET.ParseError) as exc:
        errors.append(f"URDF parse failed: {exc}")
        return
    urdf_links = {node.get("name", "") for node in urdf_root.findall("link")}
    urdf_joints = {node.get("name", ""): node for node in urdf_root.findall("joint")}
    for row in joints:
        name = row["urdf_joint_name"]
        node = urdf_joints.get(name)
        if node is None:
            errors.append(f"joint {name}: does not exist in URDF")
            continue
        parent = node.find("parent")
        child = node.find("child")
        axis = node.find("axis")
        limit = node.find("limit")
        if row["parent_link"] not in urdf_links or row["child_link"] not in urdf_links:
            errors.append(f"joint {name}: parent/child link does not exist")
        if parent is None or parent.get("link") != row["parent_link"]:
            errors.append(f"joint {name}: parent mismatch")
        if child is None or child.get("link") != row["child_link"]:
            errors.append(f"joint {name}: child mismatch")
        if node.get("type") != row["joint_type"]:
            errors.append(f"joint {name}: type mismatch")
        if axis is None or parse_vector(axis.get("xyz", ""), 3) != parse_vector(row["axis"], 3):
            errors.append(f"joint {name}: axis mismatch")
        if limit is None:
            errors.append(f"joint {name}: URDF limit missing")
        else:
            for attr, field in (("lower", "limit_lower_rad"), ("upper", "limit_upper_rad")):
                try:
                    if abs(float(limit.get(attr, "nan")) - float(row[field])) > 1e-12:
                        errors.append(f"joint {name}: {attr} limit mismatch")
                except ValueError:
                    errors.append(f"joint {name}: unparseable {attr} limit")

    if len(servos) != 12:
        errors.append(f"servos: expected 12 rows; got {len(servos)}")
    try:
        servo_ids = [int(row["servo_id"]) for row in servos]
    except ValueError:
        errors.append("servos: non-integer servo ID")
        servo_ids = []
    if set(servo_ids) != SERVO_IDS or len(set(servo_ids)) != 12:
        errors.append("servos: IDs are not the exact unique canonical set")
    joint_map = {row["urdf_joint_name"]: (row["servo_id"], row["encoder_to_q_direction"]) for row in joints}
    for row in servos:
        if row["direction"] not in {"-1", "1"}:
            errors.append(f"servo {row['mapping_id']}: direction must be -1 or 1")
        if row["joint_name"] not in joint_map:
            errors.append(f"servo {row['mapping_id']}: unknown joint")
        elif joint_map[row["joint_name"]] != (row["servo_id"], row["direction"]):
            errors.append(f"servo {row['mapping_id']}: joint/servo/direction mismatch")


def check_profiles_and_limits(
    joints: list[dict[str, str]],
    profiles: list[dict[str, str]],
    limits: list[dict[str, str]],
    errors: list[str],
) -> None:
    expected_profiles = {row["min_profile_id"] for row in joints} | {row["max_profile_id"] for row in joints}
    profile_ids = {row["profile_id"] for row in profiles}
    if len(profiles) != 24 or profile_ids != expected_profiles:
        errors.append("profiles: expected exact 24 MIN/MAX profile IDs")
    validated = set()
    for row in profiles:
        label = row["profile_id"]
        if row["software_status"] not in PROFILE_STATUSES or row["hardware_status"] not in PROFILE_STATUSES:
            errors.append(f"profile {label}: invalid status")
        if row["software_status"] != "software-ready":
            errors.append(f"profile {label}: pinned main profile is not marked software-ready")
        if row["hardware_status"] == "validated":
            validated.add(label)
        if row["probe_sign"] not in {"-1", "1"}:
            errors.append(f"profile {label}: invalid probe sign")
        if not row["units"]:
            errors.append(f"profile {label}: units are empty")
        for field in ("servo_id", "urdf_limit_tick", "guard_tick", "baseline_target_tick"):
            try:
                value = int(row[field])
                if value < 0 or value > 4095:
                    raise ValueError
            except ValueError:
                errors.append(f"profile {label}: invalid unsigned tick field {field}")
        if row["operational_safe_limit_tick"]:
            errors.append(f"profile {label}: safe limit is populated without evidence")
    if validated != {"LF_UPPER_M12_MIN", "LF_UPPER_M12_MAX"}:
        errors.append(f"profiles: hardware-validated set is wrong: {sorted(validated)}")

    safe_rows = [row for row in limits if row["limit_type"] == "operational_safe"]
    if len(safe_rows) != 24:
        errors.append(f"limits: expected 24 operational-safe unknown rows; got {len(safe_rows)}")
    for row in limits:
        label = row["limit_id"]
        if row["classification"] not in CLASSIFICATIONS:
            errors.append(f"limit {label}: invalid classification")
        if not row["units"]:
            errors.append(f"limit {label}: units are empty")
        flag = row["is_operational_safe_limit"]
        if flag not in {"true", "false"}:
            errors.append(f"limit {label}: invalid safe-limit flag")
        if row["limit_type"] == "mechanical_contact" and flag == "true":
            errors.append(f"limit {label}: mechanical contact used as operational safe limit")
        if flag == "true":
            if row["limit_type"] != "operational_safe" or not row["value"] or row["status"] == "unknown":
                errors.append(f"limit {label}: unsupported operational safe limit")
        if row["limit_type"] == "operational_safe":
            if row["value"] or row["status"] != "unknown" or row["classification"] != "UNKNOWN" or flag != "false":
                errors.append(f"limit {label}: unknown safe-limit boundary was promoted")


def check_conflicts_and_decisions(
    claims: list[dict[str, str]],
    conflicts: list[dict[str, str]],
    decisions: list[dict[str, str]],
    unresolved: list[dict[str, str]],
    claim_ids: set[str],
    errors: list[str],
) -> None:
    registered_pairs: set[frozenset[str]] = set()
    for row in conflicts:
        left, right = row["left_claim_id"], row["right_claim_id"]
        if left not in claim_ids or right not in claim_ids:
            errors.append(f"conflict {row['conflict_id']}: unknown claim")
        if not row["resolution"]:
            errors.append(f"conflict {row['conflict_id']}: resolution is empty")
        if row["classification"] not in CLASSIFICATIONS:
            errors.append(f"conflict {row['conflict_id']}: invalid classification")
        registered_pairs.add(frozenset((left, right)))
    for row in claims:
        for linked in split_ids(row["conflicts_with"]):
            if frozenset((row["claim_id"], linked)) not in registered_pairs:
                errors.append(f"claim {row['claim_id']}: conflict with {linked} is not registered")

    decision_ids = {row["decision_id"] for row in decisions}
    if not decisions:
        errors.append("decisions: registry must not be empty")
    for row in decisions:
        label = row["decision_id"]
        for field in ("question", "why_it_matters", "required_evidence", "blocked_work", "owner", "target_milestone"):
            if not row[field]:
                errors.append(f"decision {label}: required field {field} is empty")
        if row["status"] not in {"OPEN", "CLOSED"}:
            errors.append(f"decision {label}: invalid status")
    if not unresolved:
        errors.append("unresolved: registry must not be empty")
    for row in unresolved:
        label = row["unresolved_id"]
        if row["decision_id"] not in decision_ids:
            errors.append(f"unresolved {label}: unknown decision")
        if row["status"] != "OPEN":
            errors.append(f"unresolved {label}: expected OPEN")
        if row["classification"] not in {"UNKNOWN", "DECISION_REQUIRED"}:
            errors.append(f"unresolved {label}: invalid classification")
        if not row["required_evidence"]:
            errors.append(f"unresolved {label}: required evidence is empty")


def validate(root: Path) -> list[str]:
    root = root.resolve()
    errors: list[str] = []
    registries = load_registries(root, errors)
    sources = registries["source_manifest.csv"]
    source_ids = check_sources(root, sources, errors)
    claims = registries["source_claim_registry.csv"]
    claim_ids = check_claims(root, claims, sources, errors)
    check_source_links(root, registries, source_ids, errors)
    check_joints_and_urdf(
        root,
        registries["joint_registry.csv"],
        registries["servo_mapping_registry.csv"],
        errors,
    )
    check_profiles_and_limits(
        registries["joint_registry.csv"],
        registries["calibration_registry.csv"],
        registries["limit_registry.csv"],
        errors,
    )
    check_conflicts_and_decisions(
        claims,
        registries["source_conflict_registry.csv"],
        registries["decision_registry.csv"],
        registries["unresolved_registry.csv"],
        claim_ids,
        errors,
    )
    return errors


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="validate and return non-zero on error")
    parser.add_argument("--root", type=Path, help=argparse.SUPPRESS)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not args.check:
        print("error: --check is required", file=sys.stderr)
        return 2
    root = args.root or Path(__file__).resolve().parents[3]
    errors = validate(root)
    if errors:
        for error in errors:
            print(f"ERROR: {error}", file=sys.stderr)
        print(f"MATDOG_MILESTONE_I_FOUNDATION=FAIL errors={len(errors)}", file=sys.stderr)
        return 1
    registry_root = root / "06_Software/Matdog_Core/milestone_i/registries"
    counts = {}
    for name in REGISTRY_HEADERS:
        with (registry_root / name).open("r", encoding="utf-8", newline="") as handle:
            counts[name] = sum(1 for _ in csv.DictReader(handle))
    print(
        "MATDOG_MILESTONE_I_FOUNDATION=PASS "
        f"sources={counts['source_manifest.csv']} "
        f"claims={counts['source_claim_registry.csv']} "
        f"joints={counts['joint_registry.csv']} "
        f"profiles={counts['calibration_registry.csv']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
