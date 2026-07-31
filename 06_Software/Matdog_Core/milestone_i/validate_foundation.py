#!/usr/bin/env python3
"""Deterministic offline validator for the MATDOG Milestone I foundation."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import sys
import xml.etree.ElementTree as ET
from collections import Counter
from pathlib import Path
from typing import Any


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
AUTHORITIES = {"PRIMARY", "SECONDARY", "SUPPORTING", "REFERENCE"}
TEMPORAL_STATUSES = {
    "CURRENT_CANONICAL",
    "CURRENT_SUPPORTING",
    "HARDWARE_EVIDENCE",
    "SUPERSEDED",
    "EXPERIMENTAL",
    "UNKNOWN_STATUS",
}
PARSE_STATUSES = {
    "STRUCTURED_PARSEABLE",
    "TEXT_PARSEABLE",
    "TEXT_ONLY_NONPARSEABLE",
    "EXTERNAL_REFERENCE",
}
INTERPRETATION_STATUSES = {
    "MACHINE_READABLE",
    "PINNED_STATIC_TEXT",
    "PINNED_HUMAN_TEXT",
    "REFERENCE_ONLY",
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
}
EXPECTED_M11_SHA256 = "272d5e8e4e9158cd6ac058aaee1282aa132172c24e0faf3775f5b5e472a3afe3"
M11_SOURCE_ID = "R-DIR-M11"
M11_CLAIM_ID = "C-DIR-M11"
METRICS_PATTERN = re.compile(r"^FOUNDATION_METRICS_JSON: (\{.*\})$", re.MULTILINE)

REGISTRY_HEADERS = {
    "source_manifest.csv": [
        "source_id",
        "repository",
        "ref",
        "path",
        "sha256",
        "source_class",
        "authority",
        "scope",
        "temporal_status",
        "parse_status",
        "interpretation_status",
        "notes",
    ],
    "source_claim_registry.csv": [
        "claim_id",
        "domain",
        "statement",
        "classification",
        "confidence",
        "source_repository",
        "source_ref",
        "source_path",
        "source_locator",
        "units",
        "applies_to",
        "supersedes",
        "conflicts_with",
        "notes",
    ],
    "source_conflict_registry.csv": [
        "conflict_id",
        "domain",
        "left_claim_id",
        "right_claim_id",
        "resolution",
        "status",
        "classification",
        "source_id",
        "notes",
    ],
    "joint_registry.csv": [
        "joint_id",
        "leg",
        "leg_order",
        "joint_order",
        "joint_kind",
        "urdf_joint_name",
        "parent_link",
        "child_link",
        "joint_type",
        "origin_xyz_m",
        "origin_rpy_rad",
        "axis",
        "limit_lower_rad",
        "limit_upper_rad",
        "effort_nm",
        "velocity_rad_s",
        "units",
        "servo_id",
        "encoder_to_q_direction",
        "urdf_motor_direction",
        "urdf_motor_type",
        "urdf_armature",
        "zero_encoder_tick",
        "calibration_status",
        "min_profile_id",
        "max_profile_id",
        "source_id",
        "evidence_path",
    ],
    "frame_registry.csv": [
        "frame_id",
        "parent_frame",
        "frame_type",
        "origin_xyz_m",
        "origin_rpy_rad",
        "axes",
        "units",
        "status",
        "classification",
        "source_joint",
        "source_link",
        "source_id",
        "notes",
    ],
    "servo_mapping_registry.csv": [
        "mapping_id",
        "leg",
        "joint_name",
        "servo_id",
        "direction",
        "direction_semantics",
        "zero_tick",
        "raw_q0_tick",
        "digital_zero_offset_i16",
        "final_readback_tick",
        "status",
        "source_id",
        "evidence_path",
    ],
    "calibration_registry.csv": [
        "profile_id",
        "profile_order",
        "leg",
        "joint_role",
        "joint_name",
        "servo_id",
        "contact_side",
        "software_status",
        "hardware_status",
        "probe_sign",
        "home_visual_zero_tick",
        "urdf_limit_tick",
        "guard_tick",
        "baseline_target_tick",
        "allowed_motor_ids",
        "prerequisite_targets",
        "restore_order",
        "coarse_contact_tick",
        "fine_contact_tick",
        "repeatability_spread_tick",
        "measured_contact_tick",
        "operational_safe_limit_tick",
        "is_operational_safe_limit",
        "units",
        "source_id",
        "evidence_path",
        "notes",
    ],
    "limit_registry.csv": [
        "limit_id",
        "joint_name",
        "servo_id",
        "side",
        "limit_type",
        "value",
        "units",
        "status",
        "classification",
        "is_operational_safe_limit",
        "source_id",
        "evidence_path",
        "notes",
    ],
    "decision_registry.csv": [
        "decision_id",
        "question",
        "why_it_matters",
        "required_evidence",
        "blocked_work",
        "owner",
        "target_milestone",
        "status",
    ],
    "unresolved_registry.csv": [
        "unresolved_id",
        "domain",
        "question",
        "current_state",
        "required_evidence",
        "impact",
        "decision_id",
        "status",
        "classification",
        "source_id",
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

INVENTORY_KEYS = {
    "source_manifest.csv": ("sources", "sources"),
    "source_claim_registry.csv": ("claims", "claims"),
    "source_conflict_registry.csv": ("conflicts", "conflicts"),
    "joint_registry.csv": ("joints", "joints"),
    "frame_registry.csv": ("frames", "frames"),
    "servo_mapping_registry.csv": ("servos", "servos"),
    "calibration_registry.csv": ("profiles", "profiles"),
    "limit_registry.csv": ("limits", "limits"),
    "decision_registry.csv": ("decisions", "decisions"),
    "unresolved_registry.csv": ("unresolved", "unresolved"),
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


def numeric_equal(left: float, right: float, expectations: dict[str, Any]) -> bool:
    tolerance = expectations["numeric_tolerance"]
    return math.isclose(
        left,
        right,
        rel_tol=float(tolerance["relative"]),
        abs_tol=float(tolerance["absolute"]),
    )


def compare_vectors(
    label: str,
    field: str,
    registry_value: str,
    canonical_value: str,
    expectations: dict[str, Any],
    errors: list[str],
) -> None:
    try:
        registry = parse_vector(registry_value, 3)
        canonical = parse_vector(canonical_value, 3)
    except ValueError as exc:
        errors.append(f"{label}: invalid {field}: {exc}")
        return
    if any(
        not numeric_equal(left, right, expectations)
        for left, right in zip(registry, canonical)
    ):
        errors.append(
            f"{label}: {field} mismatch: registry={registry_value!r} "
            f"canonical={canonical_value!r}"
        )


def compare_counter(
    label: str,
    actual: Counter[str],
    expected: dict[str, int],
    errors: list[str],
) -> None:
    normalized = {key: value for key, value in actual.items() if value}
    if normalized != expected:
        errors.append(f"{label}: distribution mismatch: actual={normalized} expected={expected}")


def load_expectations(root: Path, errors: list[str]) -> dict[str, Any]:
    path = root / "06_Software/Matdog_Core/milestone_i/foundation_expectations.json"
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        errors.append(f"foundation expectations unreadable: {exc}")
        return {}
    if not isinstance(data, dict) or data.get("schema_version") != 1:
        errors.append("foundation expectations: unsupported schema_version")
        return {}
    pins = data.get("repository_pins", {})
    expected_pins = {
        "robot_dog_base": ROBOT_BASE,
        "normacore_main": NORMA_MAIN,
        "normacore_pr4": NORMA_PR_HEAD,
        "xgolite_tag": XGO_TAG,
        "xgolite_commit": XGO_COMMIT,
    }
    if pins != expected_pins:
        errors.append(f"foundation expectations: repository pins mismatch: {pins!r}")
    return data


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
                raw_rows = list(reader)
        except (OSError, UnicodeError, csv.Error) as exc:
            errors.append(f"{name}: unreadable CSV: {exc}")
            loaded[name] = []
            continue
        rows: list[dict[str, str]] = []
        for number, raw in enumerate(raw_rows, start=2):
            if None in raw:
                errors.append(f"{name}:{number}: extra CSV fields")
            if any(value is None for value in raw.values()):
                errors.append(f"{name}:{number}: missing CSV field")
            rows.append({field: raw.get(field) or "" for field in expected})
        id_column = ID_COLUMNS[name]
        ids = [row[id_column] for row in rows]
        duplicates = sorted(key for key, count in Counter(ids).items() if count > 1)
        if "" in ids:
            errors.append(f"{name}: empty {id_column}")
        if duplicates:
            errors.append(f"{name}: duplicate IDs: {duplicates}")
        loaded[name] = rows
    return loaded


def check_inventory(
    registries: dict[str, list[dict[str, str]]],
    expectations: dict[str, Any],
    errors: list[str],
) -> None:
    expected_ids = expectations.get("expected_ids", {})
    canonical_counts = expectations.get("canonical_counts", {})
    for filename, (inventory_key, count_key) in INVENTORY_KEYS.items():
        rows = registries[filename]
        id_column = ID_COLUMNS[filename]
        actual_ids = {row[id_column] for row in rows}
        wanted_ids = set(expected_ids.get(inventory_key, []))
        if actual_ids != wanted_ids:
            errors.append(
                f"{inventory_key}: expected ID set mismatch: "
                f"missing={sorted(wanted_ids - actual_ids)} "
                f"unexpected={sorted(actual_ids - wanted_ids)}"
            )
        expected_count = canonical_counts.get(count_key)
        if len(rows) != expected_count:
            errors.append(
                f"{inventory_key}: expected count {expected_count}; got {len(rows)}"
            )


def check_sources(
    root: Path,
    rows: list[dict[str, str]],
    expectations: dict[str, Any],
    errors: list[str],
) -> set[str]:
    ids = {row["source_id"] for row in rows}
    for row in rows:
        label = row["source_id"]
        for field in (
            "repository",
            "ref",
            "path",
            "source_class",
            "authority",
            "scope",
            "temporal_status",
            "parse_status",
            "interpretation_status",
        ):
            if not row[field]:
                errors.append(f"source {label}: required field {field} is empty")
        if not is_relative_repo_path(row["path"]):
            errors.append(f"source {label}: path must be repository-relative")
        if row["source_class"] not in CLASSIFICATIONS:
            errors.append(f"source {label}: invalid source_class {row['source_class']!r}")
        if row["authority"] not in AUTHORITIES:
            errors.append(f"source {label}: invalid authority {row['authority']!r}")
        if row["temporal_status"] not in TEMPORAL_STATUSES:
            errors.append(
                f"source {label}: invalid temporal_status {row['temporal_status']!r}"
            )
        if row["parse_status"] not in PARSE_STATUSES:
            errors.append(f"source {label}: invalid parse_status {row['parse_status']!r}")
        if row["interpretation_status"] not in INTERPRETATION_STATUSES:
            errors.append(
                f"source {label}: invalid interpretation_status "
                f"{row['interpretation_status']!r}"
            )

        digest = row["sha256"]
        if label == "N-UPSTREAM":
            if digest:
                errors.append("source N-UPSTREAM: external reference hash must remain empty")
        elif not digest:
            errors.append(f"source {label}: sha256 is required")
        elif len(digest) != 64 or any(c not in "0123456789abcdef" for c in digest):
            errors.append(f"source {label}: malformed sha256")

        if row["repository"] == ROBOT_REPOSITORY:
            if row["ref"] != ROBOT_BASE:
                errors.append(f"source {label}: wrong robot-dog base ref")
            if row["temporal_status"] == "UNKNOWN_STATUS":
                errors.append(f"source {label}: robot-dog source cannot have UNKNOWN_STATUS")
            local = root / row["path"]
            if not local.is_file():
                errors.append(f"source {label}: local path does not exist: {row['path']}")
            elif digest and sha256(local) != digest:
                errors.append(f"source {label}: local checksum mismatch")
        elif row["repository"] == NORMA_REPOSITORY:
            if row["source_class"] == "NORMACORE_EXPERIMENTAL_PR":
                if row["ref"] != NORMA_PR_HEAD:
                    errors.append(f"source {label}: wrong NormaCore PR head")
                if row["temporal_status"] != "EXPERIMENTAL":
                    errors.append(f"source {label}: PR source must be EXPERIMENTAL")
            elif row["ref"] != NORMA_MAIN:
                errors.append(f"source {label}: wrong NormaCore main ref")
        elif row["repository"] == XGO_REPOSITORY:
            if row["ref"] != XGO_TAG:
                errors.append(f"source {label}: wrong XGoLite tag")
            if row["source_class"] != "XGOLITE_ARCHITECTURAL_REFERENCE":
                errors.append(f"source {label}: XGoLite source must remain reference-only")
        elif row["repository"] == "norma-core/norma-core":
            if row["source_class"] != "NORMACORE_UPSTREAM_REFERENCE":
                errors.append(f"source {label}: upstream must be reference-only")
            if (
                row["parse_status"] != "EXTERNAL_REFERENCE"
                or row["interpretation_status"] != "REFERENCE_ONLY"
            ):
                errors.append(f"source {label}: external reference status mismatch")
        else:
            errors.append(f"source {label}: unrecognized repository {row['repository']!r}")

    xgo_anchor = next((row for row in rows if row["source_id"] == "X-README"), None)
    if not xgo_anchor or XGO_COMMIT not in xgo_anchor["notes"]:
        errors.append("source manifest: XGoLite tag commit pin is missing")

    m11 = next((row for row in rows if row["source_id"] == M11_SOURCE_ID), None)
    if not m11:
        errors.append("source manifest: M11 historical source missing")
    else:
        if m11["sha256"] != EXPECTED_M11_SHA256:
            errors.append("source R-DIR-M11: historical blob hash changed")
        if m11["parse_status"] != "TEXT_ONLY_NONPARSEABLE":
            errors.append("source R-DIR-M11: expected TEXT_ONLY_NONPARSEABLE")
        if m11["interpretation_status"] != "PINNED_HUMAN_TEXT":
            errors.append("source R-DIR-M11: expected PINNED_HUMAN_TEXT")
        m11_path = root / m11["path"]
        if m11_path.is_file():
            try:
                lines = m11_path.read_text(encoding="utf-8").splitlines()
            except (OSError, UnicodeError) as exc:
                errors.append(f"source R-DIR-M11: text read failed: {exc}")
            else:
                expected_lines = {
                    17: "calibration_result:",
                    18: "  encoder_increase_maps_to_urdf_q: negative",
                    19: "  direction: -1",
                    20: "  status: PASS_DIRECTION_TEST",
                }
                for line_number, expected_line in expected_lines.items():
                    if len(lines) < line_number or lines[line_number - 1] != expected_line:
                        errors.append(
                            f"source R-DIR-M11: text locator line {line_number} mismatch"
                        )

    profile_source = next((row for row in rows if row["source_id"] == "N-MATDOG"), None)
    expected_profile_sha = expectations["normacore_profile_derivation"]["source_sha256"]
    if not profile_source or profile_source["sha256"] != expected_profile_sha:
        errors.append("source N-MATDOG: profile derivation hash pin mismatch")
    return ids


def source_class_compatible(source_class: str, classification: str) -> bool:
    exact_classes = {
        "NORMACORE_MATDOG_FORK_MAIN_FACT",
        "NORMACORE_EXPERIMENTAL_PR",
        "NORMACORE_GENERIC_REFERENCE",
        "NORMACORE_UPSTREAM_REFERENCE",
        "XGOLITE_ARCHITECTURAL_REFERENCE",
    }
    if source_class in exact_classes:
        return source_class == classification
    return classification not in exact_classes


def check_claims(
    root: Path,
    rows: list[dict[str, str]],
    source_rows: list[dict[str, str]],
    expectations: dict[str, Any],
    errors: list[str],
) -> set[str]:
    claim_ids = {row["claim_id"] for row in rows}
    statements = [row["statement"].strip() for row in rows]
    duplicates = sorted(item for item, count in Counter(statements).items() if item and count > 1)
    if duplicates:
        errors.append(f"claims: duplicate statements: {duplicates}")
    source_triples = {
        (row["repository"], row["ref"], row["path"]): row for row in source_rows
    }
    for row in rows:
        label = row["claim_id"]
        if row["classification"] not in CLASSIFICATIONS:
            errors.append(f"claim {label}: invalid classification {row['classification']!r}")
        if row["confidence"] not in CONFIDENCE:
            errors.append(f"claim {label}: invalid confidence {row['confidence']!r}")
        for field in (
            "domain",
            "statement",
            "source_repository",
            "source_ref",
            "source_path",
            "source_locator",
            "applies_to",
        ):
            if not row[field]:
                errors.append(f"claim {label}: required field {field} is empty")
        if not is_relative_repo_path(row["source_path"]):
            errors.append(f"claim {label}: source_path must be repository-relative")
        triple = (row["source_repository"], row["source_ref"], row["source_path"])
        source = source_triples.get(triple)
        if source is None:
            errors.append(f"claim {label}: source ref/path is not registered in source manifest")
        elif not source_class_compatible(source["source_class"], row["classification"]):
            errors.append(
                f"claim {label}: classification {row['classification']} is incompatible "
                f"with source class {source['source_class']}"
            )
        if row["source_repository"] == ROBOT_REPOSITORY and not (
            root / row["source_path"]
        ).is_file():
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

    compare_counter(
        "claims classifications",
        Counter(row["classification"] for row in rows),
        expectations["claim_classification_counts"],
        errors,
    )

    row_by_id = {row["claim_id"]: row for row in rows}
    for claim_id, expected in expectations["critical_claim_provenance"].items():
        row = row_by_id.get(claim_id)
        if row is None:
            continue
        for field, wanted in expected.items():
            if row[field] != wanted:
                errors.append(
                    f"claim {claim_id}: critical provenance {field} mismatch: "
                    f"{row[field]!r} != {wanted!r}"
                )

    frame_claim = row_by_id.get("C-FRAME-BASE")
    if frame_claim:
        frame_source = root / frame_claim["source_path"]
        if frame_source.is_file():
            text = frame_source.read_text(encoding="utf-8")
            if "ROS convention: X forward, Y left, Z up." not in text:
                errors.append("claim C-FRAME-BASE: locator does not resolve in pinned source")

    m11_claim = row_by_id.get(M11_CLAIM_ID)
    if m11_claim and m11_claim["source_locator"] != (
        "lines 17-20: direction -1 and status PASS_DIRECTION_TEST"
    ):
        errors.append("claim C-DIR-M11: precise text-only locator is required")
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
                    errors.append(
                        f"{name}:{number}: evidence path does not exist: {evidence}"
                    )


def parse_urdf(
    root: Path, errors: list[str]
) -> tuple[ET.Element | None, set[str], dict[str, ET.Element]]:
    path = root / "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf"
    try:
        urdf_root = ET.parse(path).getroot()
    except (OSError, ET.ParseError) as exc:
        errors.append(f"URDF parse failed: {exc}")
        return None, set(), {}
    links = {node.get("name", "") for node in urdf_root.findall("link")}
    joints = {node.get("name", ""): node for node in urdf_root.findall("joint")}
    return urdf_root, links, joints


def parse_int(label: str, field: str, value: str, errors: list[str]) -> int | None:
    try:
        return int(value)
    except ValueError:
        errors.append(f"{label}: invalid integer {field}: {value!r}")
        return None


def parse_float(label: str, field: str, value: str, errors: list[str]) -> float | None:
    try:
        return float(value)
    except ValueError:
        errors.append(f"{label}: invalid numeric {field}: {value!r}")
        return None


def check_joints_and_servos(
    joints: list[dict[str, str]],
    servos: list[dict[str, str]],
    urdf_links: set[str],
    urdf_joints: dict[str, ET.Element],
    conflicts: list[dict[str, str]],
    expectations: dict[str, Any],
    errors: list[str],
) -> None:
    names = {row["urdf_joint_name"] for row in joints}
    if names != JOINTS:
        errors.append(f"joints: expected exact canonical URDF names; got {sorted(names)}")
    if {row["leg"] for row in joints} != LEGS:
        errors.append("joints: exact leg set mismatch")

    expected_directions = expectations["joint_directions"]
    joint_by_name: dict[str, dict[str, str]] = {}
    for row in joints:
        label = f"joint {row['joint_id']}"
        name = row["urdf_joint_name"]
        joint_by_name[name] = row
        direction_expectation = expected_directions.get(row["joint_id"])
        if direction_expectation:
            for field, wanted in (
                ("servo_id", direction_expectation["servo_id"]),
                ("encoder_to_q_direction", direction_expectation["encoder_to_q"]),
                ("urdf_motor_direction", direction_expectation["urdf_motor_direction"]),
            ):
                actual = parse_int(label, field, row[field], errors)
                if actual is not None and actual != wanted:
                    errors.append(
                        f"{label}: canonical {field} mismatch: {actual} != {wanted}"
                    )

        if row["units"] != "m rad N.m rad/s tick":
            errors.append(f"{label}: units mismatch")
        if row["calibration_status"] != "validated-digital-zero":
            errors.append(f"{label}: calibration status mismatch")
        if row["encoder_to_q_direction"] not in {"-1", "1"}:
            errors.append(f"{label}: direction must be -1 or 1")
        if parse_int(label, "zero_encoder_tick", row["zero_encoder_tick"], errors) != 2048:
            errors.append(f"{label}: home/visual zero must be 2048")

        node = urdf_joints.get(name)
        if node is None:
            errors.append(f"joint {name}: does not exist in URDF")
            continue
        parent = node.find("parent")
        child = node.find("child")
        origin = node.find("origin")
        axis = node.find("axis")
        limit = node.find("limit")
        hardware = node.find("hardware")
        if row["parent_link"] not in urdf_links or row["child_link"] not in urdf_links:
            errors.append(f"joint {name}: parent/child link does not exist")
        if parent is None or parent.get("link") != row["parent_link"]:
            errors.append(f"joint {name}: parent mismatch")
        if child is None or child.get("link") != row["child_link"]:
            errors.append(f"joint {name}: child mismatch")
        if node.get("type") != row["joint_type"]:
            errors.append(f"joint {name}: type mismatch")
        if origin is None:
            errors.append(f"joint {name}: URDF origin missing")
        else:
            compare_vectors(
                f"joint {name}",
                "origin xyz",
                row["origin_xyz_m"],
                origin.get("xyz", ""),
                expectations,
                errors,
            )
            compare_vectors(
                f"joint {name}",
                "origin rpy",
                row["origin_rpy_rad"],
                origin.get("rpy", ""),
                expectations,
                errors,
            )
        if axis is None:
            errors.append(f"joint {name}: URDF axis missing")
        else:
            compare_vectors(
                f"joint {name}",
                "axis",
                row["axis"],
                axis.get("xyz", ""),
                expectations,
                errors,
            )
        if limit is None:
            errors.append(f"joint {name}: URDF limit missing")
        else:
            for attr, field in (
                ("lower", "limit_lower_rad"),
                ("upper", "limit_upper_rad"),
                ("effort", "effort_nm"),
                ("velocity", "velocity_rad_s"),
            ):
                registry_value = parse_float(f"joint {name}", field, row[field], errors)
                canonical_value = parse_float(
                    f"joint {name}", f"URDF {attr}", limit.get(attr, ""), errors
                )
                if (
                    registry_value is not None
                    and canonical_value is not None
                    and not numeric_equal(registry_value, canonical_value, expectations)
                ):
                    errors.append(
                        f"joint {name}: {attr} mismatch: "
                        f"{registry_value} != {canonical_value}"
                    )

        if hardware is None:
            errors.append(f"joint {name}: custom hardware tags missing")
        else:
            tag_names = [child_node.tag for child_node in hardware]
            expected_tags = ["motorType", "motorId", "motorDirection", "armature"]
            if tag_names != expected_tags:
                errors.append(
                    f"joint {name}: custom hardware tag set/order mismatch: {tag_names}"
                )
            custom_fields = {
                "motorType": row["urdf_motor_type"],
                "motorId": row["servo_id"],
                "motorDirection": row["urdf_motor_direction"],
                "armature": row["urdf_armature"],
            }
            for tag, registry_value in custom_fields.items():
                element = hardware.find(tag)
                urdf_value = "" if element is None else (element.text or "").strip()
                if tag == "armature":
                    left = parse_float(f"joint {name}", "urdf_armature", registry_value, errors)
                    right = parse_float(f"joint {name}", "URDF armature", urdf_value, errors)
                    if (
                        left is not None
                        and right is not None
                        and not numeric_equal(left, right, expectations)
                    ):
                        errors.append(f"joint {name}: custom armature mismatch")
                elif registry_value != urdf_value:
                    errors.append(
                        f"joint {name}: custom {tag} mismatch: "
                        f"{registry_value!r} != {urdf_value!r}"
                    )

    try:
        joint_servo_ids = [int(row["servo_id"]) for row in joints]
    except ValueError:
        errors.append("joints: non-integer servo ID")
        joint_servo_ids = []
    if set(joint_servo_ids) != SERVO_IDS or len(set(joint_servo_ids)) != 12:
        errors.append("joints: servo IDs are not the exact unique canonical set")

    conflict_pairs = {
        frozenset((row["left_claim_id"], row["right_claim_id"])) for row in conflicts
    }
    for row in joints:
        encoder = parse_int(
            f"joint {row['joint_id']}",
            "encoder_to_q_direction",
            row["encoder_to_q_direction"],
            errors,
        )
        urdf_direction = parse_int(
            f"joint {row['joint_id']}",
            "urdf_motor_direction",
            row["urdf_motor_direction"],
            errors,
        )
        servo_id = parse_int(f"joint {row['joint_id']}", "servo_id", row["servo_id"], errors)
        if encoder is None or urdf_direction is None or servo_id is None:
            continue
        direction_claim = f"C-DIR-M{servo_id}"
        pair = frozenset(("C-URDF-MOTOR-DIR", direction_claim))
        if encoder != urdf_direction and pair not in conflict_pairs:
            errors.append(
                f"joint {row['joint_id']}: URDF motorDirection conflict is not registered"
            )

    servo_by_joint = {row["joint_name"]: row for row in servos}
    if len(servo_by_joint) != len(servos):
        errors.append("servos: joint names are not unique")
    for joint_name, joint in joint_by_name.items():
        servo = servo_by_joint.get(joint_name)
        if servo is None:
            errors.append(f"servo mapping missing for {joint_name}")
            continue
        label = f"servo {servo['mapping_id']}"
        for field, joint_field in (
            ("leg", "leg"),
            ("servo_id", "servo_id"),
            ("direction", "encoder_to_q_direction"),
        ):
            if servo[field] != joint[joint_field]:
                errors.append(f"{label}: {field} disagrees with joint registry")
        if servo["mapping_id"] != f"S-M{servo['servo_id']}":
            errors.append(f"{label}: mapping ID does not match servo ID")
        if servo["direction_semantics"] != "encoder_delta_to_positive_urdf_q":
            errors.append(f"{label}: direction semantics mismatch")
        if servo["status"] != "validated":
            errors.append(f"{label}: status must remain validated")
        zero = parse_int(label, "zero_tick", servo["zero_tick"], errors)
        raw = parse_int(label, "raw_q0_tick", servo["raw_q0_tick"], errors)
        offset = parse_int(
            label, "digital_zero_offset_i16", servo["digital_zero_offset_i16"], errors
        )
        readback = parse_int(
            label, "final_readback_tick", servo["final_readback_tick"], errors
        )
        if zero != 2048:
            errors.append(f"{label}: zero tick must be 2048")
        if raw is not None and offset is not None and offset != raw - 2048:
            errors.append(f"{label}: digital-zero offset does not match raw q0")
        if readback is not None and not 2048 <= readback <= 2051:
            errors.append(f"{label}: final readback is outside the validated range")


def check_frames(
    frames: list[dict[str, str]],
    urdf_links: set[str],
    urdf_joints: dict[str, ET.Element],
    expectations: dict[str, Any],
    errors: list[str],
) -> None:
    expected_statuses = expectations["frame_statuses"]
    materialized = 0
    for row in frames:
        frame_id = row["frame_id"]
        label = f"frame {frame_id}"
        expected_status = expected_statuses.get(frame_id)
        if row["status"] != expected_status:
            errors.append(
                f"{label}: canonical status mismatch: {row['status']!r} != {expected_status!r}"
            )
        if row["units"] != "m rad":
            errors.append(f"{label}: units must be 'm rad'")

        if row["status"] == "materialized":
            materialized += 1
            if row["classification"] != "MATDOG_VERIFIED":
                errors.append(f"{label}: materialized frame must be MATDOG_VERIFIED")
            if frame_id not in urdf_links:
                errors.append(f"{label}: materialized source link is absent from URDF")
            if frame_id == "base_link":
                if row["parent_frame"] or row["source_joint"]:
                    errors.append("frame base_link: root parent/source_joint must be empty")
                if row["source_link"] != "base_link" or row["frame_type"] != "root":
                    errors.append("frame base_link: root source metadata mismatch")
                compare_vectors(
                    label,
                    "origin xyz",
                    row["origin_xyz_m"],
                    "0 0 0",
                    expectations,
                    errors,
                )
                compare_vectors(
                    label,
                    "origin rpy",
                    row["origin_rpy_rad"],
                    "0 0 0",
                    expectations,
                    errors,
                )
                if row["axes"] != "+X forward;+Y left;+Z up":
                    errors.append("frame base_link: canonical axes mismatch")
                continue

            source_joint = urdf_joints.get(row["source_joint"])
            if source_joint is None:
                errors.append(f"{label}: source_joint does not exist in URDF")
                continue
            parent = source_joint.find("parent")
            child = source_joint.find("child")
            origin = source_joint.find("origin")
            if (
                parent is None
                or parent.get("link") != row["parent_frame"]
                or child is None
                or child.get("link") != frame_id
                or row["source_link"] != frame_id
            ):
                errors.append(f"{label}: parent/source joint/source link mismatch")
            if origin is None:
                errors.append(f"{label}: source joint origin is missing")
            else:
                compare_vectors(
                    label,
                    "origin xyz",
                    row["origin_xyz_m"],
                    origin.get("xyz", ""),
                    expectations,
                    errors,
                )
                compare_vectors(
                    label,
                    "origin rpy",
                    row["origin_rpy_rad"],
                    origin.get("rpy", ""),
                    expectations,
                    errors,
                )
            axis = source_joint.find("axis")
            if source_joint.get("type") == "fixed":
                if row["frame_type"] != "nominal_foot" or row["axes"] != "right-handed":
                    errors.append(f"{label}: fixed foot frame metadata mismatch")
            elif axis is not None:
                try:
                    vector = parse_vector(axis.get("xyz", ""), 3)
                except ValueError:
                    errors.append(f"{label}: source joint axis is invalid")
                else:
                    expected_axes = {
                        (1.0, 0.0, 0.0): "+X joint axis",
                        (0.0, 1.0, 0.0): "+Y joint axis",
                    }.get(vector)
                    if row["frame_type"] != "joint_child" or row["axes"] != expected_axes:
                        errors.append(f"{label}: joint-child axes metadata mismatch")
        else:
            if frame_id in urdf_links or frame_id in urdf_joints:
                errors.append(f"{label}: planned frame must not be materialized in REV00")
            if not row["frame_type"].startswith("planned_"):
                errors.append(f"{label}: non-materialized frame must be explicitly planned")
            if row["origin_xyz_m"] or row["origin_rpy_rad"]:
                errors.append(f"{label}: planned origin must remain unknown")
            if row["source_joint"] or row["source_link"]:
                errors.append(f"{label}: planned frame cannot cite materialized URDF nodes")
            if row["classification"] != "DECISION_REQUIRED":
                errors.append(f"{label}: planned frame must remain DECISION_REQUIRED")

    expected_materialized = expectations["canonical_counts"]["materialized_frames"]
    if materialized != expected_materialized:
        errors.append(
            f"frames: expected {expected_materialized} materialized; got {materialized}"
        )


def tick_for_delta(home: int, direction: int, delta: int) -> int:
    return home + direction * delta


def expected_profiles_from_derivation(
    joints: list[dict[str, str]],
    expectations: dict[str, Any],
    errors: list[str],
) -> list[dict[str, str]]:
    derivation = expectations["normacore_profile_derivation"]
    joint_by_leg_role = {(row["leg"], row["joint_kind"]): row for row in joints}
    expected_joint_keys = {
        (leg, role)
        for leg in derivation["leg_order"]
        for role in derivation["joint_role_order"]
    }
    missing_joint_keys = sorted(expected_joint_keys - set(joint_by_leg_role))
    if missing_joint_keys:
        errors.append(f"profiles: missing canonical joints: {missing_joint_keys}")
        return []
    for key, row in joint_by_leg_role.items():
        for field in ("encoder_to_q_direction", "servo_id"):
            try:
                int(row[field])
            except ValueError:
                errors.append(
                    f"profiles: joint {key} has invalid integer {field}: {row[field]!r}"
                )
                return []
    expected: list[dict[str, str]] = []
    home = int(derivation["home_visual_zero_tick"])
    guard_overshoot = int(derivation["guard_overshoot_ticks"])
    baseline_travel = int(derivation["baseline_travel_ticks"])
    order = 0
    for leg in derivation["leg_order"]:
        for role in derivation["joint_role_order"]:
            joint = joint_by_leg_role.get((leg, role))
            if joint is None:
                continue
            direction = int(joint["encoder_to_q_direction"])
            servo_id = int(joint["servo_id"])
            prerequisites: list[tuple[int, int]] = []
            parking_leg = {"LF": "LH", "RF": "RH"}.get(leg)
            if parking_leg:
                parking_joint = joint_by_leg_role[(parking_leg, "upper")]
                parking_tick = tick_for_delta(
                    home,
                    int(parking_joint["encoder_to_q_direction"]),
                    int(derivation["parking_upper_delta_ticks"]),
                )
                prerequisites.append((int(parking_joint["servo_id"]), parking_tick))
            for target_role, delta in derivation["prerequisite_joint_deltas"][role]:
                target_joint = joint_by_leg_role[(leg, target_role)]
                target_tick = tick_for_delta(
                    home, int(target_joint["encoder_to_q_direction"]), int(delta)
                )
                prerequisites.append((int(target_joint["servo_id"]), target_tick))

            for side in derivation["side_order"]:
                order += 1
                q_sign = -1 if side == "MIN" else 1
                probe_sign = direction * q_sign
                delta = int(derivation["joint_limit_delta_ticks"][role][side])
                urdf_tick = tick_for_delta(home, direction, delta)
                expected.append(
                    {
                        "profile_id": f"{leg}_{role.upper()}_M{servo_id}_{side}",
                        "profile_order": str(order),
                        "leg": leg,
                        "joint_role": role,
                        "joint_name": joint["urdf_joint_name"],
                        "servo_id": str(servo_id),
                        "contact_side": side,
                        "probe_sign": str(probe_sign),
                        "home_visual_zero_tick": str(home),
                        "urdf_limit_tick": str(urdf_tick),
                        "guard_tick": str(urdf_tick + probe_sign * guard_overshoot),
                        "baseline_target_tick": str(home + probe_sign * baseline_travel),
                        "allowed_motor_ids": ";".join(
                            f"M{motor_id}"
                            for motor_id in derivation["allowed_motor_ids"][leg]
                        ),
                        "prerequisite_targets": ";".join(
                            f"M{motor_id}={target}" for motor_id, target in prerequisites
                        ),
                        "restore_order": ";".join(
                            f"M{motor_id}" for motor_id, _target in reversed(prerequisites)
                        ),
                    }
                )
    return expected


def check_profiles(
    joints: list[dict[str, str]],
    profiles: list[dict[str, str]],
    expectations: dict[str, Any],
    errors: list[str],
) -> None:
    expected_profiles = expected_profiles_from_derivation(joints, expectations, errors)
    expected_by_id = {row["profile_id"]: row for row in expected_profiles}
    actual_by_id = {row["profile_id"]: row for row in profiles}
    if [row["profile_id"] for row in profiles] != [
        row["profile_id"] for row in expected_profiles
    ]:
        errors.append("profiles: canonical NormaCore all_profiles order mismatch")

    hardware_expected = expectations["hardware_validated_profiles"]
    formula_fields = (
        "profile_order",
        "leg",
        "joint_role",
        "joint_name",
        "servo_id",
        "contact_side",
        "probe_sign",
        "home_visual_zero_tick",
        "urdf_limit_tick",
        "guard_tick",
        "baseline_target_tick",
        "allowed_motor_ids",
        "prerequisite_targets",
        "restore_order",
    )
    for profile_id, expected in expected_by_id.items():
        row = actual_by_id.get(profile_id)
        if row is None:
            continue
        for field in formula_fields:
            if row[field] != expected[field]:
                errors.append(
                    f"profile {profile_id}: {field} mismatch: "
                    f"{row[field]!r} != {expected[field]!r}"
                )
        if row["software_status"] not in PROFILE_STATUSES:
            errors.append(f"profile {profile_id}: invalid software status")
        if row["hardware_status"] not in PROFILE_STATUSES:
            errors.append(f"profile {profile_id}: invalid hardware status")
        if row["software_status"] != "software-ready":
            errors.append(f"profile {profile_id}: expected software-ready")
        if row["units"] != "tick":
            errors.append(f"profile {profile_id}: units must be tick")
        if "N-MATDOG" not in split_ids(row["source_id"]):
            errors.append(f"profile {profile_id}: N-MATDOG source pin missing")
        if row["operational_safe_limit_tick"]:
            errors.append(f"profile {profile_id}: unsupported operational safe limit")
        if row["is_operational_safe_limit"] != "false":
            errors.append(f"profile {profile_id}: unsafe safe-limit promotion")

        measured_fields = (
            "coarse_contact_tick",
            "fine_contact_tick",
            "repeatability_spread_tick",
            "measured_contact_tick",
        )
        if profile_id in hardware_expected:
            if row["hardware_status"] != "validated":
                errors.append(f"profile {profile_id}: hardware status must be validated")
            for field, wanted in hardware_expected[profile_id].items():
                if row[field] != str(wanted):
                    errors.append(
                        f"profile {profile_id}: {field} hardware evidence mismatch"
                    )
            if not row["evidence_path"]:
                errors.append(f"profile {profile_id}: hardware evidence path is missing")
        else:
            if row["hardware_status"] != "hardware-pending":
                errors.append(f"profile {profile_id}: expected hardware-pending")
            if any(row[field] for field in measured_fields):
                errors.append(
                    f"profile {profile_id}: mechanical evidence exists without validation"
                )
            if row["evidence_path"]:
                errors.append(
                    f"profile {profile_id}: evidence path exists while hardware-pending"
                )

    compare_counter(
        "profiles software status",
        Counter(row["software_status"] for row in profiles),
        expectations["profile_status_counts"]["software"],
        errors,
    )
    compare_counter(
        "profiles hardware status",
        Counter(row["hardware_status"] for row in profiles),
        expectations["profile_status_counts"]["hardware"],
        errors,
    )


def check_limits(
    joints: list[dict[str, str]],
    profiles: list[dict[str, str]],
    limits: list[dict[str, str]],
    expectations: dict[str, Any],
    errors: list[str],
) -> None:
    limit_by_id = {row["limit_id"]: row for row in limits}
    for joint in joints:
        for side, joint_field in (("MIN", "limit_lower_rad"), ("MAX", "limit_upper_rad")):
            limit_id = f"L-URDF-M{joint['servo_id']}-{side}"
            row = limit_by_id.get(limit_id)
            if row is None:
                continue
            if (
                row["joint_name"] != joint["urdf_joint_name"]
                or row["servo_id"] != joint["servo_id"]
                or row["side"] != side
                or row["limit_type"] != "urdf_joint"
                or row["units"] != "rad"
                or row["status"] != "validated"
                or row["classification"] != "MATDOG_VERIFIED"
                or row["is_operational_safe_limit"] != "false"
            ):
                errors.append(f"limit {limit_id}: URDF metadata mismatch")
            actual = parse_float(f"limit {limit_id}", "value", row["value"], errors)
            wanted = parse_float(
                f"limit {limit_id}", joint_field, joint[joint_field], errors
            )
            if (
                actual is not None
                and wanted is not None
                and not numeric_equal(actual, wanted, expectations)
            ):
                errors.append(f"limit {limit_id}: URDF value mismatch")

            safe_id = f"L-SAFE-M{joint['servo_id']}-{side}"
            safe = limit_by_id.get(safe_id)
            if safe is None:
                continue
            if (
                safe["joint_name"] != joint["urdf_joint_name"]
                or safe["servo_id"] != joint["servo_id"]
                or safe["side"] != side
                or safe["limit_type"] != "operational_safe"
                or safe["value"]
                or safe["units"] != "tick"
                or safe["status"] != "unknown"
                or safe["classification"] != "UNKNOWN"
                or safe["is_operational_safe_limit"] != "false"
            ):
                errors.append(f"limit {safe_id}: unknown safe-limit boundary was promoted")

    profile_by_id = {row["profile_id"]: row for row in profiles}
    for profile_id, contact_id in (
        ("LF_UPPER_M12_MIN", "L-CONTACT-M12-MIN"),
        ("LF_UPPER_M12_MAX", "L-CONTACT-M12-MAX"),
    ):
        profile = profile_by_id.get(profile_id)
        contact = limit_by_id.get(contact_id)
        if profile is None or contact is None:
            continue
        if (
            contact["joint_name"] != profile["joint_name"]
            or contact["servo_id"] != profile["servo_id"]
            or contact["side"] != profile["contact_side"]
            or contact["limit_type"] != "mechanical_contact"
            or contact["value"] != profile["measured_contact_tick"]
            or contact["units"] != "tick"
            or contact["status"] != "validated"
            or contact["classification"] != "HARDWARE_OBSERVATION"
            or contact["is_operational_safe_limit"] != "false"
        ):
            errors.append(f"limit {contact_id}: mechanical-contact evidence mismatch")

    for row in limits:
        if row["classification"] not in CLASSIFICATIONS:
            errors.append(f"limit {row['limit_id']}: invalid classification")
        if row["is_operational_safe_limit"] not in {"true", "false"}:
            errors.append(f"limit {row['limit_id']}: invalid safe-limit flag")
        if (
            row["limit_type"] == "mechanical_contact"
            and row["is_operational_safe_limit"] == "true"
        ):
            errors.append(
                f"limit {row['limit_id']}: mechanical contact used as operational safe limit"
            )

    compare_counter(
        "limits types",
        Counter(row["limit_type"] for row in limits),
        expectations["limit_type_counts"],
        errors,
    )
    safe_count = sum(row["is_operational_safe_limit"] == "true" for row in limits)
    expected_safe = expectations["canonical_counts"]["operational_safe_limits"]
    if safe_count != expected_safe:
        errors.append(
            f"limits: expected {expected_safe} operational-safe flags; got {safe_count}"
        )


def check_conflicts_decisions_unresolved(
    claims: list[dict[str, str]],
    conflicts: list[dict[str, str]],
    decisions: list[dict[str, str]],
    unresolved: list[dict[str, str]],
    claim_ids: set[str],
    expectations: dict[str, Any],
    errors: list[str],
) -> None:
    registered_pairs: set[frozenset[str]] = set()
    expected_conflict_statuses = expectations["conflict_statuses"]
    for row in conflicts:
        label = row["conflict_id"]
        left, right = row["left_claim_id"], row["right_claim_id"]
        if left not in claim_ids or right not in claim_ids:
            errors.append(f"conflict {label}: unknown claim")
        if not row["resolution"]:
            errors.append(f"conflict {label}: resolution is empty")
        if row["classification"] not in CLASSIFICATIONS:
            errors.append(f"conflict {label}: invalid classification")
        expected_status = expected_conflict_statuses.get(label)
        if row["status"] != expected_status:
            errors.append(
                f"conflict {label}: canonical status mismatch: "
                f"{row['status']!r} != {expected_status!r}"
            )
        registered_pairs.add(frozenset((left, right)))
    for row in claims:
        for linked in split_ids(row["conflicts_with"]):
            if frozenset((row["claim_id"], linked)) not in registered_pairs:
                errors.append(
                    f"claim {row['claim_id']}: conflict with {linked} is not registered"
                )
    compare_counter(
        "conflicts statuses",
        Counter(row["status"] for row in conflicts),
        expectations["conflict_status_counts"],
        errors,
    )

    expected_decisions = expectations["decision_statuses"]
    decision_ids = {row["decision_id"] for row in decisions}
    for row in decisions:
        label = row["decision_id"]
        for field in (
            "question",
            "why_it_matters",
            "required_evidence",
            "blocked_work",
            "owner",
            "target_milestone",
        ):
            if not row[field]:
                errors.append(f"decision {label}: required field {field} is empty")
        if row["status"] != expected_decisions.get(label):
            errors.append(f"decision {label}: canonical status mismatch")
    compare_counter(
        "decisions statuses",
        Counter(row["status"] for row in decisions),
        expectations["decision_status_counts"],
        errors,
    )

    expected_unresolved = expectations["unresolved_expectations"]
    for row in unresolved:
        label = row["unresolved_id"]
        if row["decision_id"] not in decision_ids:
            errors.append(f"unresolved {label}: unknown decision")
        if not row["required_evidence"]:
            errors.append(f"unresolved {label}: required evidence is empty")
        expected = expected_unresolved.get(label)
        if expected and (
            row["domain"] != expected["category"]
            or row["status"] != expected["status"]
            or row["classification"] != expected["classification"]
        ):
            errors.append(
                f"unresolved {label}: canonical category/status/classification mismatch"
            )
    compare_counter(
        "unresolved statuses",
        Counter(row["status"] for row in unresolved),
        expectations["unresolved_status_counts"],
        errors,
    )
    compare_counter(
        "unresolved classifications",
        Counter(row["classification"] for row in unresolved),
        expectations["unresolved_classification_counts"],
        errors,
    )


def check_document_metrics(
    root: Path, expectations: dict[str, Any], errors: list[str]
) -> None:
    expected = expectations["document_metrics"]
    documents = {
        "acceptance": root
        / "01_Docs/02_Architecture/Milestone_I/MATDOG_MILESTONE_I_FOUNDATION_ACCEPTANCE.md",
        "handoff": root
        / "09_Logs/Development_Log/2026-07-30_MILESTONE_I_FOUNDATION_HANDOFF.md",
    }
    for label, path in documents.items():
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeError) as exc:
            errors.append(f"{label}: metrics document unreadable: {exc}")
            continue
        match = METRICS_PATTERN.search(text)
        if not match:
            errors.append(f"{label}: FOUNDATION_METRICS_JSON marker missing")
            continue
        try:
            actual = json.loads(match.group(1))
        except json.JSONDecodeError as exc:
            errors.append(f"{label}: metrics JSON invalid: {exc}")
            continue
        if actual != expected:
            errors.append(f"{label}: metrics diverge from foundation expectations")


def validate(root: Path) -> list[str]:
    root = root.resolve()
    errors: list[str] = []
    expectations = load_expectations(root, errors)
    registries = load_registries(root, errors)
    if not expectations:
        return errors
    check_inventory(registries, expectations, errors)
    sources = registries["source_manifest.csv"]
    source_ids = check_sources(root, sources, expectations, errors)
    claims = registries["source_claim_registry.csv"]
    claim_ids = check_claims(root, claims, sources, expectations, errors)
    check_source_links(root, registries, source_ids, errors)

    _urdf_root, urdf_links, urdf_joints = parse_urdf(root, errors)
    check_joints_and_servos(
        registries["joint_registry.csv"],
        registries["servo_mapping_registry.csv"],
        urdf_links,
        urdf_joints,
        registries["source_conflict_registry.csv"],
        expectations,
        errors,
    )
    check_frames(
        registries["frame_registry.csv"],
        urdf_links,
        urdf_joints,
        expectations,
        errors,
    )
    check_profiles(
        registries["joint_registry.csv"],
        registries["calibration_registry.csv"],
        expectations,
        errors,
    )
    check_limits(
        registries["joint_registry.csv"],
        registries["calibration_registry.csv"],
        registries["limit_registry.csv"],
        expectations,
        errors,
    )
    check_conflicts_decisions_unresolved(
        claims,
        registries["source_conflict_registry.csv"],
        registries["decision_registry.csv"],
        registries["unresolved_registry.csv"],
        claim_ids,
        expectations,
        errors,
    )
    check_document_metrics(root, expectations, errors)
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
        f"conflicts={counts['source_conflict_registry.csv']} "
        f"unresolved={counts['unresolved_registry.csv']} "
        f"joints={counts['joint_registry.csv']} "
        f"frames={counts['frame_registry.csv']} "
        f"profiles={counts['calibration_registry.csv']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
