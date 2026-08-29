#!/usr/bin/env python3
"""Generate and verify the Full Leg Calibrator geometry-plan header.

The checked-in C++ table is a compact, deterministic projection of two
immutable Geometry Compiler V5 consumers:

* the corrected four-worker path/parking deployment artifact; and
* the final external 3 mm clearance-policy artifact.

The raw parking artifact proves sampled geometric feasibility.  The external
policy classifies clearance evidence.  Neither artifact grants permission to
move hardware; live build/session/operator authorization remains a separate
firmware concern.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
from decimal import Decimal
import hashlib
import json
from pathlib import Path
from typing import Any, Iterable, Sequence

from matdog_geometry_path_planner_v5 import (
    PARKING_FEASIBLE_1DOF,
    PARKING_NOT_NEEDED,
    derive_geometry_profile_with_path_plans,
    validate_parking_artifact,
)
from matdog_geometry_profile_v5 import (
    semantic_content_sha256 as profile_content_sha256,
    validate_pure_geometry_profile,
)
from matdog_geometry_safety_policy import (
    MOTION_AUTHORIZATION_NOT_GRANTED,
    POLICY_FAIL_EXACT,
    POLICY_PASS,
    POLICY_REJECT_GEOMETRY,
    POLICY_UNRESOLVED_BOUND,
    POLICY_UNRESOLVED_MISSING,
    validate_safety_policy_artifact,
)


class GeometryPlanGenerationError(RuntimeError):
    """The canonical artifact chain cannot safely generate a firmware table."""


SCRIPT_PATH = Path(__file__).resolve()
DEFAULT_REPO_ROOT = SCRIPT_PATH.parents[3]

PARKING_RELATIVE_PATH = Path(
    "09_Logs/Validation_Reports/Geometry_Compiler/"
    "2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_"
    "BENCHMARK_D_W4_PATH_PARKING.json"
)
ENDPOINT_PROFILE_RELATIVE_PATH = Path(
    "09_Logs/Validation_Reports/Geometry_Compiler/"
    "2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_"
    "BENCHMARK_D_W4_ENDPOINT_PROFILE.json"
)
COMBINED_PROFILE_RELATIVE_PATH = Path(
    "09_Logs/Validation_Reports/Geometry_Compiler/"
    "2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_"
    "BENCHMARK_D_W4_COMBINED_PROFILE.json"
)
RUN_MANIFEST_RELATIVE_PATH = Path(
    "09_Logs/Validation_Reports/Geometry_Compiler/"
    "2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_"
    "BENCHMARK_D_W4_RUN_MANIFEST.json"
)
SAFETY_POLICY_RELATIVE_PATH = Path(
    "09_Logs/Validation_Reports/Geometry_Compiler/"
    "2026-08-11_132758_MATDOG_GEOMETRY_V5_G12_FINAL_"
    "EXTERNAL_SAFETY_POLICY.json"
)
OUTPUT_RELATIVE_PATH = Path(
    "05_Firmware/Full_Leg_Calibrator_V1/"
    "matdog_full_leg_calibrator_v1/flc_leg_plan.h"
)

EXPECTED_PARKING_FILE_SHA256 = (
    "e561e7fb98880843e590f4676e8559374c51d0e641102f89a803b3619722c4d7"
)
EXPECTED_PARKING_SEMANTIC_SHA256 = (
    "67c58430e78241af1a636cdcc22092ff855371713fc7f26bc56412f7c7181139"
)
EXPECTED_ENDPOINT_FILE_SHA256 = (
    "dd8cb42c3b916d067f97a321c5ffcdfb013dde1f2f2a6ba71f73becff360dc0f"
)
EXPECTED_ENDPOINT_SEMANTIC_SHA256 = (
    "de205209f6015734f43af7f49146ecf60f89a74d6ce1276ce134c189a89c9f7e"
)
EXPECTED_COMBINED_FILE_SHA256 = (
    "448ebcb3ed56d7f5225e6f9906e2efeb622010d8ecc457fe9ad00bda6407b00c"
)
EXPECTED_COMBINED_SEMANTIC_SHA256 = (
    "0a772234a46afad14eb4af0999294020bb0fb8974ca0b68f3ccd780fa057db51"
)
EXPECTED_RUN_MANIFEST_FILE_SHA256 = (
    "0db86e633599f63a769dbba75db3a54c1e6470e128c007eb24c465f92a428b17"
)
EXPECTED_RUN_MANIFEST_CONTENT_SHA256 = (
    "4e7172d473b5d19c79112252e50212565ef73505551bff993b6a0c927f2aaee7"
)
EXPECTED_SAFETY_POLICY_FILE_SHA256 = (
    "82f00a9414df06676babed1101a73e1a9d7bfa126592cdd7a425f66d5f01f1cc"
)
EXPECTED_SAFETY_POLICY_SEMANTIC_SHA256 = (
    "e5cb2a4c33082c59c6f5d381f90fcc19680cc899e326d13c9c9129932bde5d08"
)
EXPECTED_GEOMETRY_SOURCE_COMBINED_SHA256 = (
    "e4eea175e90131b1d8e55ae381fe8e88d6f966c0f87d4eee5c064d0a6967737f"
)
EXPECTED_URDF_SHA256 = (
    "3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59"
)

EXPECTED_BUNDLE_KEYS = frozenset(
    {
        "combined_profile",
        "combined_report",
        "endpoint_profile",
        "endpoint_report",
        "oracle_json",
        "oracle_report",
        "parking_json",
        "parking_report",
    }
)

PICORADIANS_PER_RADIAN = Decimal("1000000000000")
SUPPORTED_POLICY_RESULTS = frozenset(
    {
        POLICY_PASS,
        POLICY_FAIL_EXACT,
        POLICY_REJECT_GEOMETRY,
        POLICY_UNRESOLVED_BOUND,
        POLICY_UNRESOLVED_MISSING,
    }
)


@dataclass(frozen=True)
class CanonicalInputs:
    parking: dict[str, Any]
    parking_decimal: dict[str, Any]
    endpoint_profile: dict[str, Any]
    combined_profile: dict[str, Any]
    run_manifest: dict[str, Any]
    safety_policy: dict[str, Any]


@dataclass(frozen=True)
class EndpointPlanRow:
    canonical_index: int
    endpoint_id: str
    joint_name: str
    limit_side: str
    target_joint_index: int
    target_angle_picorad: int
    declared_limit_picorad: int
    parking_outcome: str
    target_domain: str
    baseline_status: str
    auxiliary_joint_name: str | None
    auxiliary_joint_index: int | None
    auxiliary_angle_picorad: int
    blocking_link_a: str | None
    blocking_link_b: str | None
    blocking_relation: str | None
    clearance_policy_result: str
    motion_authorization: str
    q0_start_mask: int
    q0_held_during_task_mask: int


@dataclass(frozen=True, order=True)
class GeometryDependency:
    prerequisite_joint_index: int
    target_joint_index: int
    endpoint_index: int


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise GeometryPlanGenerationError(f"cannot load canonical JSON: {path}") from exc
    if not isinstance(value, dict):
        raise GeometryPlanGenerationError(f"canonical JSON is not an object: {path}")
    return value


def _load_decimal_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"), parse_float=Decimal)
    except (OSError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise GeometryPlanGenerationError(
            f"cannot load lossless canonical JSON: {path}"
        ) from exc
    if not isinstance(value, dict):
        raise GeometryPlanGenerationError(f"canonical JSON is not an object: {path}")
    return value


def _canonical_content_sha256(value: dict[str, Any], excluded: set[str]) -> str:
    content = {key: child for key, child in value.items() if key not in excluded}
    encoded = json.dumps(
        content,
        sort_keys=True,
        separators=(",", ":"),
        ensure_ascii=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _require_equal(actual: Any, expected: Any, label: str) -> None:
    if actual != expected:
        raise GeometryPlanGenerationError(
            f"{label} mismatch: expected={expected!r}, actual={actual!r}"
        )


def _validate_pinned_file(path: Path, expected_sha256: str, label: str) -> None:
    if not path.is_file():
        raise GeometryPlanGenerationError(f"missing {label}: {path}")
    _require_equal(_sha256_file(path), expected_sha256, f"{label} file SHA256")


def _validate_bundle(repo_root: Path, manifest: dict[str, Any]) -> None:
    _require_equal(
        manifest.get("schema_version"),
        "matdog.geometry_compiler_v5.integrated_run.v2",
        "run-manifest schema",
    )
    _require_equal(manifest.get("benchmark_id"), "D", "run-manifest benchmark")
    _require_equal(
        manifest.get("manifest_content_sha256"),
        EXPECTED_RUN_MANIFEST_CONTENT_SHA256,
        "run-manifest content SHA256",
    )
    _require_equal(
        _canonical_content_sha256(manifest, {"manifest_content_sha256"}),
        EXPECTED_RUN_MANIFEST_CONTENT_SHA256,
        "recomputed run-manifest content SHA256",
    )
    _require_equal(
        manifest.get("bundle_validity"),
        {
            "marker": "RUN_MANIFEST_PUBLISHED_LAST",
            "manifest_required": True,
            "artifact_count": 8,
            "artifact_hash_algorithm": "SHA256",
        },
        "run-manifest validity marker",
    )
    records = manifest.get("artifacts")
    if not isinstance(records, dict) or set(records) != EXPECTED_BUNDLE_KEYS:
        raise GeometryPlanGenerationError("run-manifest artifact set is not exact")
    observed_paths: set[Path] = set()
    for key in sorted(EXPECTED_BUNDLE_KEYS):
        record = records[key]
        if not isinstance(record, dict) or set(record) != {
            "relative_path",
            "file_sha256",
        }:
            raise GeometryPlanGenerationError(f"invalid run-manifest record: {key}")
        relative = Path(str(record["relative_path"]))
        if relative.is_absolute() or ".." in relative.parts:
            raise GeometryPlanGenerationError(f"unsafe run-manifest path: {relative}")
        path = (repo_root / relative).resolve()
        try:
            path.relative_to(repo_root)
        except ValueError as exc:
            raise GeometryPlanGenerationError(
                f"run-manifest artifact escapes repository: {relative}"
            ) from exc
        if path in observed_paths:
            raise GeometryPlanGenerationError("run-manifest artifact paths alias")
        observed_paths.add(path)
        _validate_pinned_file(path, str(record["file_sha256"]), f"bundle member {key}")


def _validate_current_provenance(
    repo_root: Path,
    parking: dict[str, Any],
    run_manifest: dict[str, Any],
    safety_policy: dict[str, Any],
) -> None:
    calibration_dir = repo_root / "06_Software/Matdog_Core/calibration"
    source_manifest = parking.get("provenance", {}).get("source_file_sha256", {})
    if not isinstance(source_manifest, dict) or len(source_manifest) != 9:
        raise GeometryPlanGenerationError("parking source manifest is not exact 9-file V5 set")
    for relative, expected in source_manifest.items():
        _validate_pinned_file(
            calibration_dir / relative,
            str(expected),
            f"canonical geometry source {relative}",
        )
    _require_equal(
        parking.get("provenance", {}).get("source_combined_sha256"),
        EXPECTED_GEOMETRY_SOURCE_COMBINED_SHA256,
        "geometry source combined SHA256",
    )

    input_manifest = run_manifest.get("provenance", {}).get("input_file_sha256", {})
    if not isinstance(input_manifest, dict) or len(input_manifest) != 18:
        raise GeometryPlanGenerationError("run-manifest input set is not URDF plus 17 meshes")
    for relative, expected in input_manifest.items():
        _validate_pinned_file(
            repo_root / relative,
            str(expected),
            f"geometry input {relative}",
        )
    _require_equal(
        run_manifest.get("provenance", {}).get("urdf_sha256"),
        EXPECTED_URDF_SHA256,
        "run-manifest URDF SHA256",
    )

    policy_sources = safety_policy.get("provenance", {}).get(
        "source_file_sha256", {}
    )
    if not isinstance(policy_sources, dict) or len(policy_sources) != 1:
        raise GeometryPlanGenerationError("safety-policy source manifest is not exact")
    for relative, expected in policy_sources.items():
        _validate_pinned_file(
            repo_root / relative,
            str(expected),
            f"safety-policy source {relative}",
        )


def load_canonical_inputs(repo_root: Path = DEFAULT_REPO_ROOT) -> CanonicalInputs:
    repo_root = repo_root.resolve()
    parking_path = repo_root / PARKING_RELATIVE_PATH
    endpoint_path = repo_root / ENDPOINT_PROFILE_RELATIVE_PATH
    combined_path = repo_root / COMBINED_PROFILE_RELATIVE_PATH
    manifest_path = repo_root / RUN_MANIFEST_RELATIVE_PATH
    policy_path = repo_root / SAFETY_POLICY_RELATIVE_PATH

    for path, expected, label in (
        (parking_path, EXPECTED_PARKING_FILE_SHA256, "canonical parking artifact"),
        (endpoint_path, EXPECTED_ENDPOINT_FILE_SHA256, "canonical endpoint profile"),
        (combined_path, EXPECTED_COMBINED_FILE_SHA256, "canonical combined profile"),
        (manifest_path, EXPECTED_RUN_MANIFEST_FILE_SHA256, "canonical run manifest"),
        (policy_path, EXPECTED_SAFETY_POLICY_FILE_SHA256, "canonical safety policy"),
    ):
        _validate_pinned_file(path, expected, label)

    parking = _load_json(parking_path)
    endpoint = _load_json(endpoint_path)
    combined = _load_json(combined_path)
    manifest = _load_json(manifest_path)
    policy = _load_json(policy_path)

    validate_parking_artifact(parking)
    validate_pure_geometry_profile(endpoint)
    validate_pure_geometry_profile(combined)
    validate_safety_policy_artifact(policy)
    _validate_bundle(repo_root, manifest)

    _require_equal(
        parking.get("semantic_content_sha256"),
        EXPECTED_PARKING_SEMANTIC_SHA256,
        "parking semantic SHA256",
    )
    _require_equal(
        endpoint.get("semantic_content_sha256"),
        EXPECTED_ENDPOINT_SEMANTIC_SHA256,
        "endpoint semantic SHA256",
    )
    _require_equal(
        combined.get("semantic_content_sha256"),
        EXPECTED_COMBINED_SEMANTIC_SHA256,
        "combined semantic SHA256",
    )
    _require_equal(
        policy.get("semantic_content_sha256"),
        EXPECTED_SAFETY_POLICY_SEMANTIC_SHA256,
        "safety-policy semantic SHA256",
    )
    _require_equal(
        parking.get("input_geometry_profile", {}).get("semantic_content_sha256"),
        EXPECTED_ENDPOINT_SEMANTIC_SHA256,
        "parking input endpoint semantic SHA256",
    )
    _require_equal(
        parking.get("generation_metadata", {})
        .get("audit_input_geometry_profile", {})
        .get("file_sha256"),
        EXPECTED_ENDPOINT_FILE_SHA256,
        "parking audit endpoint file SHA256",
    )
    _require_equal(
        policy.get("input_parking_artifact", {}).get("file_sha256"),
        EXPECTED_PARKING_FILE_SHA256,
        "safety-policy parking file SHA256",
    )
    _require_equal(
        policy.get("input_parking_artifact", {}).get("semantic_content_sha256"),
        EXPECTED_PARKING_SEMANTIC_SHA256,
        "safety-policy parking semantic SHA256",
    )
    _require_equal(
        policy.get("summary", {}).get("motion_authorization_granted_count"),
        0,
        "safety-policy motion authorization count",
    )

    reconstructed = derive_geometry_profile_with_path_plans(endpoint, parking)
    _require_equal(
        profile_content_sha256(reconstructed),
        EXPECTED_COMBINED_SEMANTIC_SHA256,
        "reconstructed combined-profile semantic SHA256",
    )
    _validate_current_provenance(repo_root, parking, manifest, policy)

    return CanonicalInputs(
        parking=parking,
        parking_decimal=_load_decimal_json(parking_path),
        endpoint_profile=endpoint,
        combined_profile=combined,
        run_manifest=manifest,
        safety_policy=policy,
    )


def canonical_joint_names(parking: dict[str, Any]) -> tuple[str, ...]:
    names: list[str] = []
    for plan in parking.get("plans", []):
        name = plan.get("joint_name")
        if not isinstance(name, str) or not name:
            raise GeometryPlanGenerationError("parking plan lacks a joint name")
        if name not in names:
            names.append(name)
    if len(names) != 12:
        raise GeometryPlanGenerationError(
            f"canonical endpoint set does not contain 12 joints: {names}"
        )
    return tuple(names)


def _picoradians(value: Decimal, label: str) -> int:
    if not isinstance(value, Decimal) or not value.is_finite():
        raise GeometryPlanGenerationError(f"{label} is not a finite JSON decimal")
    scaled = value * PICORADIANS_PER_RADIAN
    integral = scaled.to_integral_value()
    if scaled != integral:
        raise GeometryPlanGenerationError(
            f"{label} cannot be represented losslessly in integer picoradians: {value}"
        )
    return int(integral)


def _position_map_matches(
    actual: Any,
    joint_names: Sequence[str],
    nonzero: dict[str, float],
) -> bool:
    if not isinstance(actual, dict) or set(actual) != set(joint_names):
        return False
    for name in joint_names:
        expected = nonzero.get(name, 0.0)
        if float(actual[name]) != float(expected):
            return False
    return True


def _validate_q0_sequence(
    plan: dict[str, Any],
    joint_names: Sequence[str],
    auxiliary_name: str | None,
    auxiliary_angle: float,
) -> tuple[int, int]:
    all_mask = (1 << len(joint_names)) - 1
    target = str(plan["joint_name"])
    target_angle = float(plan["target_angle_rad"])
    task = plan.get("task_path")
    task_return = plan.get("task_return")
    if not isinstance(task, dict) or not isinstance(task_return, dict):
        raise GeometryPlanGenerationError(f"{plan['endpoint_id']}: task/return missing")

    parked = {auxiliary_name: auxiliary_angle} if auxiliary_name is not None else {}
    if not _position_map_matches(
        task.get("start_joint_positions_rad"), joint_names, parked
    ):
        raise GeometryPlanGenerationError(
            f"{plan['endpoint_id']}: task does not start in validated q0/parking context"
        )
    if not _position_map_matches(
        task.get("end_joint_positions_rad"),
        joint_names,
        {**parked, target: target_angle},
    ):
        raise GeometryPlanGenerationError(
            f"{plan['endpoint_id']}: task does not end in validated target context"
        )
    if not _position_map_matches(
        task_return.get("start_joint_positions_rad"),
        joint_names,
        {**parked, target: target_angle},
    ) or not _position_map_matches(
        task_return.get("end_joint_positions_rad"), joint_names, parked
    ):
        raise GeometryPlanGenerationError(
            f"{plan['endpoint_id']}: return is not the validated target-to-q0 reverse"
        )

    if auxiliary_name is None:
        if plan.get("path_in") is not None or plan.get("path_out") is not None:
            raise GeometryPlanGenerationError(
                f"{plan['endpoint_id']}: no-parking row unexpectedly moves an auxiliary"
            )
    else:
        path_in = plan.get("path_in")
        path_out = plan.get("path_out")
        if not isinstance(path_in, dict) or not isinstance(path_out, dict):
            raise GeometryPlanGenerationError(
                f"{plan['endpoint_id']}: parking row lacks enter/restore paths"
            )
        if not _position_map_matches(
            path_in.get("start_joint_positions_rad"), joint_names, {}
        ) or not _position_map_matches(
            path_in.get("end_joint_positions_rad"), joint_names, parked
        ):
            raise GeometryPlanGenerationError(
                f"{plan['endpoint_id']}: parking entry is not q0-to-park"
            )
        if not _position_map_matches(
            path_out.get("start_joint_positions_rad"), joint_names, parked
        ) or not _position_map_matches(
            path_out.get("end_joint_positions_rad"), joint_names, {}
        ):
            raise GeometryPlanGenerationError(
                f"{plan['endpoint_id']}: parking restore is not park-to-q0"
            )

    target_index = joint_names.index(target)
    held_mask = all_mask & ~(1 << target_index)
    if auxiliary_name is not None:
        held_mask &= ~(1 << joint_names.index(auxiliary_name))
    return all_mask, held_mask


def build_endpoint_rows(inputs: CanonicalInputs) -> tuple[EndpointPlanRow, ...]:
    plans = inputs.parking.get("plans", [])
    decimal_plans = inputs.parking_decimal.get("plans", [])
    policy_results = inputs.safety_policy.get("endpoint_policy_results", [])
    if len(plans) != 24 or len(decimal_plans) != 24 or len(policy_results) != 24:
        raise GeometryPlanGenerationError("canonical projection requires exact 24/24 inputs")
    joint_names = canonical_joint_names(inputs.parking)
    rows: list[EndpointPlanRow] = []

    for index, (plan, decimal_plan, policy) in enumerate(
        zip(plans, decimal_plans, policy_results)
    ):
        endpoint_id = plan.get("endpoint_id")
        _require_equal(plan.get("canonical_endpoint_index"), index, "endpoint index")
        _require_equal(decimal_plan.get("endpoint_id"), endpoint_id, "decimal endpoint")
        _require_equal(policy.get("canonical_endpoint_index"), index, "policy index")
        _require_equal(policy.get("endpoint_id"), endpoint_id, "policy endpoint")
        _require_equal(
            policy.get("raw_geometry_outcome"),
            plan.get("outcome"),
            f"{endpoint_id} policy geometry outcome",
        )
        _require_equal(
            policy.get("raw_target_domain"),
            plan.get("target_domain"),
            f"{endpoint_id} policy target domain",
        )
        _require_equal(
            policy.get("motion_authorization"),
            MOTION_AUTHORIZATION_NOT_GRANTED,
            f"{endpoint_id} motion authorization provenance",
        )
        if policy.get("policy_result") not in SUPPORTED_POLICY_RESULTS:
            raise GeometryPlanGenerationError(
                f"{endpoint_id}: unsupported clearance-policy result"
            )

        outcome = plan.get("outcome")
        parking = plan.get("parking_configuration_rad")
        parking_decimal = decimal_plan.get("parking_configuration_rad")
        auxiliary_name: str | None = None
        auxiliary_index: int | None = None
        auxiliary_angle = 0.0
        auxiliary_picorad = 0
        if outcome == PARKING_NOT_NEEDED:
            if parking != {} or parking_decimal != {}:
                raise GeometryPlanGenerationError(
                    f"{endpoint_id}: NOT_NEEDED row carries a parking pose"
                )
            if plan.get("baseline_task_path", {}).get("status") != "COLLISION_FREE":
                raise GeometryPlanGenerationError(
                    f"{endpoint_id}: NOT_NEEDED baseline is not collision-free"
                )
        elif outcome == PARKING_FEASIBLE_1DOF:
            if not isinstance(parking, dict) or len(parking) != 1:
                raise GeometryPlanGenerationError(
                    f"{endpoint_id}: 1-DOF outcome lacks exactly one auxiliary"
                )
            if not isinstance(parking_decimal, dict) or set(parking_decimal) != set(parking):
                raise GeometryPlanGenerationError(
                    f"{endpoint_id}: lossless parking pose does not match"
                )
            auxiliary_name, auxiliary_angle = next(iter(parking.items()))
            auxiliary_index = joint_names.index(auxiliary_name)
            auxiliary_picorad = _picoradians(
                parking_decimal[auxiliary_name],
                f"{endpoint_id} auxiliary angle",
            )
            if plan.get("baseline_task_path", {}).get("status") != "PATH_OBSTRUCTION":
                raise GeometryPlanGenerationError(
                    f"{endpoint_id}: parking baseline is not obstructed"
                )
        else:
            raise GeometryPlanGenerationError(
                f"{endpoint_id}: unsupported canonical parking outcome {outcome!r}"
            )

        for segment_name in ("task_path", "task_return"):
            if plan.get(segment_name, {}).get("status") != "COLLISION_FREE":
                raise GeometryPlanGenerationError(
                    f"{endpoint_id}: {segment_name} is not collision-free"
                )
        if auxiliary_name is not None:
            for segment_name in ("path_in", "path_out"):
                if plan.get(segment_name, {}).get("status") != "COLLISION_FREE":
                    raise GeometryPlanGenerationError(
                        f"{endpoint_id}: {segment_name} is not collision-free"
                    )

        blocker = plan.get("first_refined_blocking_pair")
        relation = plan.get("first_refined_blocking_relation")
        if auxiliary_name is None:
            if blocker is not None or relation is not None:
                raise GeometryPlanGenerationError(
                    f"{endpoint_id}: collision-free baseline retains a blocker"
                )
            blocking_a = blocking_b = None
        else:
            if not isinstance(blocker, list) or len(blocker) != 2:
                raise GeometryPlanGenerationError(
                    f"{endpoint_id}: parking row lacks refined blocking pair"
                )
            if relation not in {"same_branch", "cross_branch"}:
                raise GeometryPlanGenerationError(
                    f"{endpoint_id}: parking row lacks topology relation"
                )
            blocking_a, blocking_b = map(str, blocker)

        q0_start_mask, q0_held_mask = _validate_q0_sequence(
            plan,
            joint_names,
            auxiliary_name,
            float(auxiliary_angle),
        )
        rows.append(
            EndpointPlanRow(
                canonical_index=index,
                endpoint_id=str(endpoint_id),
                joint_name=str(plan["joint_name"]),
                limit_side=str(plan["limit_side"]),
                target_joint_index=joint_names.index(str(plan["joint_name"])),
                target_angle_picorad=_picoradians(
                    decimal_plan["target_angle_rad"],
                    f"{endpoint_id} target angle",
                ),
                declared_limit_picorad=_picoradians(
                    decimal_plan["declared_limit_rad"],
                    f"{endpoint_id} declared limit",
                ),
                parking_outcome=str(outcome),
                target_domain=str(plan["target_domain"]),
                baseline_status=str(plan["baseline_task_path"]["status"]),
                auxiliary_joint_name=auxiliary_name,
                auxiliary_joint_index=auxiliary_index,
                auxiliary_angle_picorad=auxiliary_picorad,
                blocking_link_a=blocking_a,
                blocking_link_b=blocking_b,
                blocking_relation=str(relation) if relation is not None else None,
                clearance_policy_result=str(policy["policy_result"]),
                motion_authorization=str(policy["motion_authorization"]),
                q0_start_mask=q0_start_mask,
                q0_held_during_task_mask=q0_held_mask,
            )
        )

    if len({row.endpoint_id for row in rows}) != 24:
        raise GeometryPlanGenerationError("generated endpoint identifiers are not unique")
    if sum(row.parking_outcome == PARKING_NOT_NEEDED for row in rows) != 18:
        raise GeometryPlanGenerationError("canonical no-parking count is not 18")
    if sum(row.parking_outcome == PARKING_FEASIBLE_1DOF for row in rows) != 6:
        raise GeometryPlanGenerationError("canonical 1-DOF parking count is not 6")
    return tuple(rows)


def derive_dependencies(rows: Iterable[EndpointPlanRow]) -> tuple[GeometryDependency, ...]:
    dependencies = {
        GeometryDependency(
            prerequisite_joint_index=row.auxiliary_joint_index,
            target_joint_index=row.target_joint_index,
            endpoint_index=row.canonical_index,
        )
        for row in rows
        if row.auxiliary_joint_index is not None
    }
    if any(
        dependency.prerequisite_joint_index == dependency.target_joint_index
        for dependency in dependencies
    ):
        raise GeometryPlanGenerationError("geometry dependency contains a self-cycle")
    return tuple(sorted(dependencies, key=lambda edge: edge.endpoint_index))


def topological_order(
    joint_names: Sequence[str],
    dependencies: Iterable[GeometryDependency],
) -> tuple[int, ...]:
    dependencies = tuple(dependencies)
    count = len(joint_names)
    indegree = [0] * count
    successors: list[set[int]] = [set() for _ in range(count)]
    for dependency in dependencies:
        prerequisite = dependency.prerequisite_joint_index
        target = dependency.target_joint_index
        if not 0 <= prerequisite < count or not 0 <= target < count:
            raise GeometryPlanGenerationError("geometry dependency index is out of range")
        if target not in successors[prerequisite]:
            successors[prerequisite].add(target)
            indegree[target] += 1

    remaining = set(range(count))
    result: list[int] = []
    while remaining:
        ready = min((index for index in remaining if indegree[index] == 0), default=None)
        if ready is None:
            cycle_names = ", ".join(joint_names[index] for index in sorted(remaining))
            raise GeometryPlanGenerationError(
                f"geometry parking dependency cycle: {cycle_names}"
            )
        result.append(ready)
        remaining.remove(ready)
        for successor in successors[ready]:
            indegree[successor] -= 1
    return tuple(result)


def _cpp_string(value: str | None) -> str:
    if value is None:
        return "0"
    return json.dumps(value, ensure_ascii=True)


def _joint_symbol(joint_name: str) -> str:
    suffix = joint_name.removesuffix("_joint").upper()
    return f"FLC_GEOMETRY_JOINT_{suffix}"


OUTCOME_SYMBOLS = {
    PARKING_NOT_NEEDED: "FLC_NO_PARKING_REQUIRED",
    PARKING_FEASIBLE_1DOF: "FLC_PARKING_REQUIRED_1DOF",
}
TARGET_DOMAIN_SYMBOLS = {
    "EXECUTABLE_URDF_DOMAIN": "FLC_TARGET_EXECUTABLE_URDF_DOMAIN",
    "DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS": (
        "FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS"
    ),
}
PATH_STATUS_SYMBOLS = {
    "COLLISION_FREE": "FLC_GEOMETRY_PATH_COLLISION_FREE",
    "PATH_OBSTRUCTION": "FLC_GEOMETRY_PATH_OBSTRUCTION",
}
RELATION_SYMBOLS = {
    None: "FLC_GEOMETRY_RELATION_NONE",
    "same_branch": "FLC_GEOMETRY_RELATION_SAME_BRANCH",
    "cross_branch": "FLC_GEOMETRY_RELATION_CROSS_BRANCH",
}
POLICY_SYMBOLS = {
    POLICY_PASS: "FLC_CLEARANCE_POLICY_PASS",
    POLICY_FAIL_EXACT: "FLC_CLEARANCE_POLICY_FAIL_EXACT",
    POLICY_REJECT_GEOMETRY: "FLC_CLEARANCE_POLICY_REJECT_GEOMETRY",
    POLICY_UNRESOLVED_BOUND: "FLC_CLEARANCE_POLICY_UNRESOLVED_LOWER_BOUND",
    POLICY_UNRESOLVED_MISSING: "FLC_CLEARANCE_POLICY_UNRESOLVED_MISSING",
}


def render_header(
    inputs: CanonicalInputs,
    rows: Sequence[EndpointPlanRow],
    dependencies: Sequence[GeometryDependency],
    order: Sequence[int],
) -> str:
    joint_names = canonical_joint_names(inputs.parking)
    generator_sha256 = _sha256_file(SCRIPT_PATH)
    lines = [
        "/*",
        " * MATDOG FULL LEG CALIBRATOR V1 — generated geometry plan.",
        " *",
        " * DO NOT HAND EDIT. Regenerate/check with:",
        " *   python3 -B 06_Software/Matdog_Core/calibration/",
        " *     generate_flc_leg_plan.py [--check]",
        " *",
        " * Raw Geometry Compiler feasibility is sampled geometric evidence, not a",
        " * continuous swept-volume proof. The joined G12 result is an external",
        " * clearance classification only. Both artifacts explicitly grant ZERO",
        " * motion authorizations; live build/session/operator gates stay separate.",
        " */",
        "",
        "#ifndef FLC_LEG_PLAN_H",
        "#define FLC_LEG_PLAN_H",
        "",
        "#include <stdint.h>",
        "",
        f"static const char FLC_GEOMETRY_PLAN_GENERATOR_SHA256[] = {json.dumps(generator_sha256)};",
        f"static const char FLC_GEOMETRY_PARKING_ARTIFACT_PATH[] = {json.dumps(str(PARKING_RELATIVE_PATH))};",
        f"static const char FLC_GEOMETRY_PARKING_FILE_SHA256[] = {json.dumps(EXPECTED_PARKING_FILE_SHA256)};",
        f"static const char FLC_GEOMETRY_PARKING_SEMANTIC_SHA256[] = {json.dumps(EXPECTED_PARKING_SEMANTIC_SHA256)};",
        f"static const char FLC_GEOMETRY_ENDPOINT_PROFILE_PATH[] = {json.dumps(str(ENDPOINT_PROFILE_RELATIVE_PATH))};",
        f"static const char FLC_GEOMETRY_ENDPOINT_FILE_SHA256[] = {json.dumps(EXPECTED_ENDPOINT_FILE_SHA256)};",
        f"static const char FLC_GEOMETRY_ENDPOINT_SEMANTIC_SHA256[] = {json.dumps(EXPECTED_ENDPOINT_SEMANTIC_SHA256)};",
        f"static const char FLC_GEOMETRY_COMBINED_SEMANTIC_SHA256[] = {json.dumps(EXPECTED_COMBINED_SEMANTIC_SHA256)};",
        f"static const char FLC_GEOMETRY_RUN_MANIFEST_PATH[] = {json.dumps(str(RUN_MANIFEST_RELATIVE_PATH))};",
        f"static const char FLC_GEOMETRY_RUN_MANIFEST_FILE_SHA256[] = {json.dumps(EXPECTED_RUN_MANIFEST_FILE_SHA256)};",
        f"static const char FLC_GEOMETRY_RUN_MANIFEST_CONTENT_SHA256[] = {json.dumps(EXPECTED_RUN_MANIFEST_CONTENT_SHA256)};",
        f"static const char FLC_GEOMETRY_SAFETY_POLICY_PATH[] = {json.dumps(str(SAFETY_POLICY_RELATIVE_PATH))};",
        f"static const char FLC_GEOMETRY_SAFETY_POLICY_FILE_SHA256[] = {json.dumps(EXPECTED_SAFETY_POLICY_FILE_SHA256)};",
        f"static const char FLC_GEOMETRY_SAFETY_POLICY_SEMANTIC_SHA256[] = {json.dumps(EXPECTED_SAFETY_POLICY_SEMANTIC_SHA256)};",
        f"static const char FLC_GEOMETRY_SOURCE_COMBINED_SHA256[] = {json.dumps(EXPECTED_GEOMETRY_SOURCE_COMBINED_SHA256)};",
        f"static const char FLC_GEOMETRY_URDF_SHA256[] = {json.dumps(EXPECTED_URDF_SHA256)};",
        "static const int64_t FLC_PICORADIANS_PER_RADIAN = 1000000000000LL;",
        "static const bool FLC_GEOMETRY_ARTIFACT_GRANTS_MOTION_AUTHORIZATION = false;",
        "",
        "enum FlcGeometryJoint : uint8_t {",
    ]
    for index, name in enumerate(joint_names):
        lines.append(f"  {_joint_symbol(name)} = {index},")
    lines.extend(
        [
            f"  FLC_GEOMETRY_JOINT_COUNT = {len(joint_names)},",
            "  FLC_GEOMETRY_JOINT_NONE = 255",
            "};",
            "",
            "enum FlcParkingOutcome : uint8_t {",
            "  FLC_NO_PARKING_REQUIRED = 0,",
            "  FLC_PARKING_REQUIRED_1DOF = 1",
            "};",
            "",
            "enum FlcGeometryTargetDomain : uint8_t {",
            "  FLC_TARGET_EXECUTABLE_URDF_DOMAIN = 0,",
            "  FLC_TARGET_DIAGNOSTIC_OUTSIDE_URDF_LIMITS = 1",
            "};",
            "",
            "enum FlcGeometryPathStatus : uint8_t {",
            "  FLC_GEOMETRY_PATH_COLLISION_FREE = 0,",
            "  FLC_GEOMETRY_PATH_OBSTRUCTION = 1",
            "};",
            "",
            "enum FlcGeometryRelation : uint8_t {",
            "  FLC_GEOMETRY_RELATION_NONE = 0,",
            "  FLC_GEOMETRY_RELATION_SAME_BRANCH = 1,",
            "  FLC_GEOMETRY_RELATION_CROSS_BRANCH = 2",
            "};",
            "",
            "enum FlcGeometryClearancePolicyResult : uint8_t {",
            "  FLC_CLEARANCE_POLICY_PASS = 0,",
            "  FLC_CLEARANCE_POLICY_FAIL_EXACT = 1,",
            "  FLC_CLEARANCE_POLICY_REJECT_GEOMETRY = 2,",
            "  FLC_CLEARANCE_POLICY_UNRESOLVED_LOWER_BOUND = 3,",
            "  FLC_CLEARANCE_POLICY_UNRESOLVED_MISSING = 4",
            "};",
            "",
            "enum FlcGeometryMotionAuthorizationProvenance : uint8_t {",
            "  FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY = 0",
            "};",
            "",
            "struct FlcEndpointGeometryPlan {",
            "  uint8_t canonicalEndpointIndex;",
            "  const char *endpointId;",
            "  FlcGeometryJoint targetJoint;",
            "  const char *jointName;",
            "  const char *limitSide;",
            "  int64_t targetAnglePicoRad;",
            "  int64_t declaredLimitPicoRad;",
            "  FlcParkingOutcome parkingOutcome;",
            "  FlcGeometryTargetDomain targetDomain;",
            "  FlcGeometryPathStatus baselinePathStatus;",
            "  FlcGeometryJoint auxiliaryJoint;",
            "  int64_t auxiliaryAnglePicoRad;",
            "  const char *blockingLinkA;",
            "  const char *blockingLinkB;",
            "  FlcGeometryRelation blockingRelation;",
            "  FlcGeometryClearancePolicyResult clearancePolicyResult;",
            "  FlcGeometryMotionAuthorizationProvenance motionAuthorizationProvenance;",
            "  // Exact source sequence starts all 12 joints at q=0. During the",
            "  // task, every masked joint must remain verified at q=0.",
            "  uint16_t q0StartMask;",
            "  uint16_t q0HeldDuringTaskMask;",
            "};",
            "",
            "static const FlcEndpointGeometryPlan FLC_ENDPOINT_GEOMETRY_PLANS[] = {",
        ]
    )

    for row in rows:
        aux_symbol = (
            _joint_symbol(row.auxiliary_joint_name)
            if row.auxiliary_joint_name is not None
            else "FLC_GEOMETRY_JOINT_NONE"
        )
        lines.extend(
            [
                f"  // {row.canonical_index}: {row.endpoint_id}",
                "  {"
                f"{row.canonical_index}, {_cpp_string(row.endpoint_id)}, "
                f"{_joint_symbol(row.joint_name)}, {_cpp_string(row.joint_name)}, "
                f"{_cpp_string(row.limit_side)}, {row.target_angle_picorad}LL, "
                f"{row.declared_limit_picorad}LL, "
                f"{OUTCOME_SYMBOLS[row.parking_outcome]}, "
                f"{TARGET_DOMAIN_SYMBOLS[row.target_domain]}, "
                f"{PATH_STATUS_SYMBOLS[row.baseline_status]}, {aux_symbol}, "
                f"{row.auxiliary_angle_picorad}LL, "
                f"{_cpp_string(row.blocking_link_a)}, {_cpp_string(row.blocking_link_b)}, "
                f"{RELATION_SYMBOLS[row.blocking_relation]}, "
                f"{POLICY_SYMBOLS[row.clearance_policy_result]}, "
                "FLC_MOTION_NOT_GRANTED_OFFLINE_EVIDENCE_ONLY, "
                f"0x{row.q0_start_mask:03X}, 0x{row.q0_held_during_task_mask:03X}"
                "},",
            ]
        )
    lines.extend(
        [
            "};",
            "static const int FLC_ENDPOINT_GEOMETRY_PLAN_COUNT =",
            "    (int)(sizeof(FLC_ENDPOINT_GEOMETRY_PLANS) /",
            "          sizeof(FLC_ENDPOINT_GEOMETRY_PLANS[0]));",
            "",
            "struct FlcGeometryDependency {",
            "  FlcGeometryJoint prerequisiteJoint;",
            "  FlcGeometryJoint targetJoint;",
            "  uint8_t endpointIndex;",
            "};",
            "",
            "static const FlcGeometryDependency FLC_GEOMETRY_DEPENDENCIES[] = {",
        ]
    )
    for dependency in dependencies:
        lines.append(
            "  {"
            f"{_joint_symbol(joint_names[dependency.prerequisite_joint_index])}, "
            f"{_joint_symbol(joint_names[dependency.target_joint_index])}, "
            f"{dependency.endpoint_index}"
            "},"
        )
    lines.extend(
        [
            "};",
            "static const int FLC_GEOMETRY_DEPENDENCY_COUNT =",
            "    (int)(sizeof(FLC_GEOMETRY_DEPENDENCIES) /",
            "          sizeof(FLC_GEOMETRY_DEPENDENCIES[0]));",
            "",
            "// Kahn order with canonical joint index as the deterministic tie-break.",
            "static const FlcGeometryJoint FLC_GEOMETRY_TOPOLOGICAL_ORDER[] = {",
            "  " + ", ".join(_joint_symbol(joint_names[index]) for index in order),
            "};",
            "",
            "static const char *const FLC_GEOMETRY_JOINT_NAMES[] = {",
            "  " + ", ".join(_cpp_string(name) for name in joint_names),
            "};",
            "",
            "inline bool flcGeometryStrEq(const char *a, const char *b) {",
            "  if (a == 0 || b == 0) return false;",
            "  while (*a && *b) {",
            "    if (*a != *b) return false;",
            "    ++a;",
            "    ++b;",
            "  }",
            "  return *a == *b;",
            "}",
            "",
            "inline const char *flcGeometryJointName(FlcGeometryJoint joint) {",
            "  const int index = (int)joint;",
            "  if (index < 0 || index >= FLC_GEOMETRY_JOINT_COUNT) return 0;",
            "  return FLC_GEOMETRY_JOINT_NAMES[index];",
            "}",
            "",
            "inline uint16_t flcGeometryJointMask(FlcGeometryJoint joint) {",
            "  const int index = (int)joint;",
            "  if (index < 0 || index >= FLC_GEOMETRY_JOINT_COUNT) return 0;",
            "  return (uint16_t)(1U << index);",
            "}",
            "",
            "// NULL means INVALID/UNKNOWN. A known no-parking endpoint has an",
            "// explicit row whose parkingOutcome is FLC_NO_PARKING_REQUIRED.",
            "inline const FlcEndpointGeometryPlan *flcGeometryPlanFor(",
            "    const char *jointName, const char *limitSide) {",
            "  for (int i = 0; i < FLC_ENDPOINT_GEOMETRY_PLAN_COUNT; ++i) {",
            "    const FlcEndpointGeometryPlan &row = FLC_ENDPOINT_GEOMETRY_PLANS[i];",
            "    if (flcGeometryStrEq(row.jointName, jointName) &&",
            "        flcGeometryStrEq(row.limitSide, limitSide)) return &row;",
            "  }",
            "  return 0;",
            "}",
            "",
            "inline const FlcEndpointGeometryPlan *flcGeometryPlanForEndpointId(",
            "    const char *endpointId) {",
            "  for (int i = 0; i < FLC_ENDPOINT_GEOMETRY_PLAN_COUNT; ++i) {",
            "    if (flcGeometryStrEq(FLC_ENDPOINT_GEOMETRY_PLANS[i].endpointId,",
            "                         endpointId)) return &FLC_ENDPOINT_GEOMETRY_PLANS[i];",
            "  }",
            "  return 0;",
            "}",
            "",
            "inline int flcGeometryDependencyCountFor(FlcGeometryJoint target) {",
            "  int count = 0;",
            "  for (int i = 0; i < FLC_GEOMETRY_DEPENDENCY_COUNT; ++i) {",
            "    if (FLC_GEOMETRY_DEPENDENCIES[i].targetJoint == target) ++count;",
            "  }",
            "  return count;",
            "}",
            "",
            "inline FlcGeometryJoint flcGeometryDependencyAt(",
            "    FlcGeometryJoint target, int ordinal) {",
            "  if (ordinal < 0) return FLC_GEOMETRY_JOINT_NONE;",
            "  for (int i = 0; i < FLC_GEOMETRY_DEPENDENCY_COUNT; ++i) {",
            "    if (FLC_GEOMETRY_DEPENDENCIES[i].targetJoint != target) continue;",
            "    if (ordinal-- == 0) return FLC_GEOMETRY_DEPENDENCIES[i].prerequisiteJoint;",
            "  }",
            "  return FLC_GEOMETRY_JOINT_NONE;",
            "}",
            "",
            "inline bool flcGeometryBuildTopologicalOrder(",
            "    FlcGeometryJoint *out, int capacity) {",
            "  if (out == 0 || capacity < FLC_GEOMETRY_JOINT_COUNT) return false;",
            "  uint8_t indegree[FLC_GEOMETRY_JOINT_COUNT] = {0};",
            "  bool emitted[FLC_GEOMETRY_JOINT_COUNT] = {false};",
            "  for (int i = 0; i < FLC_GEOMETRY_DEPENDENCY_COUNT; ++i) {",
            "    const int prerequisite = (int)FLC_GEOMETRY_DEPENDENCIES[i].prerequisiteJoint;",
            "    const int target = (int)FLC_GEOMETRY_DEPENDENCIES[i].targetJoint;",
            "    if (prerequisite < 0 || prerequisite >= FLC_GEOMETRY_JOINT_COUNT ||",
            "        target < 0 || target >= FLC_GEOMETRY_JOINT_COUNT ||",
            "        prerequisite == target) return false;",
            "    ++indegree[target];",
            "  }",
            "  for (int outputIndex = 0; outputIndex < FLC_GEOMETRY_JOINT_COUNT;",
            "       ++outputIndex) {",
            "    int ready = -1;",
            "    for (int joint = 0; joint < FLC_GEOMETRY_JOINT_COUNT; ++joint) {",
            "      if (!emitted[joint] && indegree[joint] == 0) { ready = joint; break; }",
            "    }",
            "    if (ready < 0) return false;",
            "    emitted[ready] = true;",
            "    out[outputIndex] = (FlcGeometryJoint)ready;",
            "    for (int i = 0; i < FLC_GEOMETRY_DEPENDENCY_COUNT; ++i) {",
            "      if ((int)FLC_GEOMETRY_DEPENDENCIES[i].prerequisiteJoint == ready) {",
            "        const int target = (int)FLC_GEOMETRY_DEPENDENCIES[i].targetJoint;",
            "        if (indegree[target] == 0) return false;",
            "        --indegree[target];",
            "      }",
            "    }",
            "  }",
            "  return true;",
            "}",
            "",
            "inline bool flcGeometryDependencyGraphAcyclic() {",
            "  FlcGeometryJoint order[FLC_GEOMETRY_JOINT_COUNT];",
            "  return flcGeometryBuildTopologicalOrder(order, FLC_GEOMETRY_JOINT_COUNT);",
            "}",
            "",
            "inline bool flcGeometryGeneratedTopologicalOrderValid() {",
            "  FlcGeometryJoint derived[FLC_GEOMETRY_JOINT_COUNT];",
            "  if (!flcGeometryBuildTopologicalOrder(derived, FLC_GEOMETRY_JOINT_COUNT))",
            "    return false;",
            "  for (int i = 0; i < FLC_GEOMETRY_JOINT_COUNT; ++i) {",
            "    if (derived[i] != FLC_GEOMETRY_TOPOLOGICAL_ORDER[i]) return false;",
            "  }",
            "  return true;",
            "}",
            "",
            "#endif  // FLC_LEG_PLAN_H",
            "",
        ]
    )
    return "\n".join(lines)


def generate(repo_root: Path = DEFAULT_REPO_ROOT) -> str:
    inputs = load_canonical_inputs(repo_root)
    rows = build_endpoint_rows(inputs)
    joint_names = canonical_joint_names(inputs.parking)
    dependencies = derive_dependencies(rows)
    if len(dependencies) != 6:
        raise GeometryPlanGenerationError("canonical dependency count is not six")
    order = topological_order(joint_names, dependencies)
    return render_header(inputs, rows, dependencies, order)


def check_generated_header(repo_root: Path = DEFAULT_REPO_ROOT) -> None:
    expected = generate(repo_root)
    output = repo_root.resolve() / OUTPUT_RELATIVE_PATH
    try:
        actual = output.read_text(encoding="utf-8")
    except OSError as exc:
        raise GeometryPlanGenerationError(f"cannot read generated header: {output}") from exc
    if actual != expected:
        raise GeometryPlanGenerationError(
            f"generated header is stale: run {SCRIPT_PATH.relative_to(repo_root)}"
        )


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=DEFAULT_REPO_ROOT)
    parser.add_argument(
        "--check",
        action="store_true",
        help="verify the checked-in header byte-for-byte without writing",
    )
    args = parser.parse_args(argv)
    repo_root = args.repo_root.resolve()
    try:
        if args.check:
            check_generated_header(repo_root)
            print("FLC_GEOMETRY_PLAN_CHECK=PASS")
        else:
            output = repo_root / OUTPUT_RELATIVE_PATH
            output.write_text(generate(repo_root), encoding="utf-8")
            print(f"FLC_GEOMETRY_PLAN_GENERATED={output}")
    except GeometryPlanGenerationError as exc:
        parser.exit(1, f"STOP: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
