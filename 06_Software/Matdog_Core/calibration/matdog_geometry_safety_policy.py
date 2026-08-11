#!/usr/bin/env python3
"""External safety-policy consumer for MATDOG V5 geometry artifacts.

Geometric feasibility and raw clearance come from the immutable path/parking
artifact.  This module applies an explicit policy threshold without mutating
or redefining geometry.  It can also reproduce the frozen G4 3 mm policy
verdicts from their serialized raw segment evidence.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any

from matdog_geometry_path_planner_v5 import (
    PARKING_FEASIBLE_1DOF,
    PARKING_FEASIBLE_2DOF,
    PARKING_NOT_NEEDED,
    validate_parking_artifact,
)


REFERENCE_CLEARANCE_THRESHOLD_M = 0.003
EXPECTED_G4_CONTENT_SHA256 = (
    "4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61"
)

POLICY_PASS = "PASS"
POLICY_FAIL_EXACT = "FAIL_EXACT_CLEARANCE_BELOW_THRESHOLD"
POLICY_REJECT_GEOMETRY = "REJECT_GEOMETRICALLY_INFEASIBLE"
POLICY_UNRESOLVED_BOUND = "UNRESOLVED_LOWER_BOUND_BELOW_THRESHOLD"
POLICY_UNRESOLVED_MISSING = "UNRESOLVED_NO_CLEARANCE_MEASUREMENT"
MOTION_AUTHORIZATION_NOT_GRANTED = "NOT_GRANTED_OFFLINE_EVIDENCE_ONLY"
POLICY_ID = "MATDOG_REFERENCE_MINIMUM_CLEARANCE"
POLICY_ACCEPTANCE_SCOPE = "minimum sampled geometric clearance evidence only"
POLICY_MOTION_AUTHORIZATION_RULE = (
    "not evaluated or granted; target URDF-domain eligibility is preserved "
    "as separate raw evidence"
)
TARGET_DOMAIN_EXECUTABLE = "EXECUTABLE_URDF_DOMAIN"
TARGET_DOMAIN_DIAGNOSTIC = "DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS"


class GeometrySafetyPolicyError(RuntimeError):
    """A saved geometry input or external policy declaration is invalid."""


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _canonical_hash(value: dict[str, Any], excluded: set[str]) -> str:
    content = {key: child for key, child in value.items() if key not in excluded}
    canonical = json.dumps(content, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _source_provenance() -> dict[str, Any]:
    source_path = Path(__file__).resolve()
    repo_root = source_path.parents[3]
    relative_path = str(source_path.relative_to(repo_root))
    source_manifest = {relative_path: _sha256_file(source_path)}
    return {
        "source_file_sha256": source_manifest,
        "source_combined_sha256": _canonical_hash(source_manifest, set()),
    }


def safety_policy_content_sha256(artifact: dict[str, Any]) -> str:
    return _canonical_hash(
        artifact,
        {"generation_metadata", "semantic_content_sha256"},
    )


def _evaluate_policy(
    *,
    geometry_feasible: bool,
    clearance_m: float | None,
    clearance_kind: str | None,
    threshold_m: float,
) -> tuple[str, bool | None]:
    if not isinstance(geometry_feasible, bool):
        raise GeometrySafetyPolicyError("geometry feasibility must be boolean")
    if not (isinstance(threshold_m, (int, float)) and math.isfinite(threshold_m)):
        raise GeometrySafetyPolicyError(f"invalid external threshold: {threshold_m!r}")
    if threshold_m < 0.0:
        raise GeometrySafetyPolicyError(f"negative external threshold: {threshold_m!r}")
    if clearance_m is None:
        if clearance_kind is not None:
            raise GeometrySafetyPolicyError(
                "clearance kind must be null when clearance measurement is absent"
            )
    else:
        if not (
            isinstance(clearance_m, (int, float))
            and math.isfinite(clearance_m)
            and clearance_m >= 0.0
        ):
            raise GeometrySafetyPolicyError(f"invalid clearance measurement: {clearance_m!r}")
        if clearance_kind not in {"EXACT", "LOWER_BOUND"}:
            raise GeometrySafetyPolicyError(f"unknown clearance kind: {clearance_kind!r}")
    if not geometry_feasible:
        return POLICY_REJECT_GEOMETRY, False
    if clearance_m is None:
        return POLICY_UNRESOLVED_MISSING, None
    if clearance_m >= threshold_m:
        return POLICY_PASS, True
    if clearance_kind == "EXACT":
        return POLICY_FAIL_EXACT, False
    return POLICY_UNRESOLVED_BOUND, None


def _legacy_content_sha256(profile: dict[str, Any]) -> str:
    return _canonical_hash(
        profile,
        {"generation_timestamp_utc", "content_sha256"},
    )


def _legacy_segment_gate(segment: dict[str, Any], threshold_m: float) -> str:
    if segment.get("has_true_collision"):
        return "FAIL"
    clearance = segment.get("min_clearance_m")
    kind = segment.get("min_clearance_kind")
    if clearance is None:
        return "FAIL"
    if float(clearance) >= threshold_m:
        return "PASS"
    if kind == "EXACT":
        return "FAIL"
    return "UNRESOLVED_FOR_THRESHOLD"


def reproduce_frozen_g4_policy(
    g4_profile: dict[str, Any],
    *,
    reference: dict[str, str],
) -> dict[str, Any]:
    stored = g4_profile.get("content_sha256")
    recomputed = _legacy_content_sha256(g4_profile)
    if stored != EXPECTED_G4_CONTENT_SHA256 or recomputed != EXPECTED_G4_CONTENT_SHA256:
        raise GeometrySafetyPolicyError(
            f"frozen G4 content SHA mismatch: stored={stored!r}, recomputed={recomputed!r}"
        )
    threshold = g4_profile.get("numerical_parameters", {}).get("min_clearance_pass_m")
    if not isinstance(threshold, (int, float)) or not math.isclose(
        float(threshold),
        REFERENCE_CLEARANCE_THRESHOLD_M,
        rel_tol=0.0,
        abs_tol=1e-15,
    ):
        raise GeometrySafetyPolicyError(
            f"frozen G4 reference threshold is not 3 mm: {threshold!r}"
        )

    plan_records = []
    exact_replay = True
    for presentation_key, plan in sorted(g4_profile.get("parking_plans", {}).items()):
        segments = []
        for segment in plan.get("segments", []):
            replayed_gate = _legacy_segment_gate(segment, float(threshold))
            replayed_passed = replayed_gate == "PASS"
            agrees = (
                replayed_gate == segment.get("clearance_gate_result")
                and replayed_passed == segment.get("passed")
            )
            exact_replay = exact_replay and agrees
            segments.append(
                {
                    "description": segment.get("description"),
                    "raw_has_true_collision": segment.get("has_true_collision"),
                    "raw_min_clearance_m": segment.get("min_clearance_m"),
                    "raw_min_clearance_kind": segment.get("min_clearance_kind"),
                    "replayed_gate_result": replayed_gate,
                    "stored_gate_result": segment.get("clearance_gate_result"),
                    "replayed_passed": replayed_passed,
                    "stored_passed": segment.get("passed"),
                    "exact_match": agrees,
                }
            )
        replayed_sequence_passed = all(segment["replayed_passed"] for segment in segments)
        sequence_agrees = replayed_sequence_passed == plan.get("active_leg_sequence_passed")
        exact_replay = exact_replay and sequence_agrees
        plan_records.append(
            {
                "presentation_key": presentation_key,
                "active_leg_sequence_replayed_passed": replayed_sequence_passed,
                "active_leg_sequence_stored_passed": plan.get("active_leg_sequence_passed"),
                "sequence_exact_match": sequence_agrees,
                "raw_active_leg_sequence_min_clearance_m": plan.get(
                    "active_leg_sequence_min_clearance_m"
                ),
                "raw_auxiliary_parking_required": plan.get("auxiliary_parking_required"),
                "raw_park_path_min_clearance_m": plan.get("park_path_min_clearance_m"),
                "raw_park_path_passed": plan.get("park_path_passed"),
                "segments": segments,
            }
        )
    if len(plan_records) != 4 or not exact_replay:
        raise GeometrySafetyPolicyError(
            f"frozen G4 3 mm policy replay failed: plans={len(plan_records)}, exact={exact_replay}"
        )
    return {
        "input": dict(sorted(reference.items())),
        "content_sha256": EXPECTED_G4_CONTENT_SHA256,
        "reference_threshold_m": float(threshold),
        "exact_replay": exact_replay,
        "active_sequence_passed_count": sum(
            plan["active_leg_sequence_replayed_passed"] for plan in plan_records
        ),
        "active_sequence_failed_or_unresolved_count": sum(
            not plan["active_leg_sequence_replayed_passed"] for plan in plan_records
        ),
        "plans": plan_records,
    }


def build_safety_policy_artifact(
    parking_artifact: dict[str, Any],
    *,
    parking_reference: dict[str, str],
    threshold_m: float = REFERENCE_CLEARANCE_THRESHOLD_M,
    frozen_g4_profile: dict[str, Any] | None = None,
    frozen_g4_reference: dict[str, str] | None = None,
) -> dict[str, Any]:
    validate_parking_artifact(parking_artifact)
    if not (math.isfinite(threshold_m) and threshold_m >= 0.0):
        raise GeometrySafetyPolicyError(f"invalid external threshold: {threshold_m}")
    if parking_reference.get("semantic_content_sha256") != parking_artifact.get(
        "semantic_content_sha256"
    ):
        raise GeometrySafetyPolicyError("parking artifact semantic reference mismatch")

    feasible_outcomes = {
        PARKING_NOT_NEEDED,
        PARKING_FEASIBLE_1DOF,
        PARKING_FEASIBLE_2DOF,
    }
    results = []
    for plan in parking_artifact["plans"]:
        objective = plan.get("objective")
        clearance = objective.get("min_clearance_m") if objective is not None else None
        kind = objective.get("min_clearance_kind") if objective is not None else None
        geometry_feasible = plan.get("outcome") in feasible_outcomes
        result, accepted = _evaluate_policy(
            geometry_feasible=geometry_feasible,
            clearance_m=clearance,
            clearance_kind=kind,
            threshold_m=threshold_m,
        )
        results.append(
            {
                "canonical_endpoint_index": plan.get("canonical_endpoint_index"),
                "endpoint_id": plan.get("endpoint_id"),
                "raw_geometry_outcome": plan.get("outcome"),
                "raw_geometry_feasible": geometry_feasible,
                "raw_target_domain": plan.get("target_domain"),
                "raw_target_within_urdf_limits": plan.get(
                    "target_within_urdf_limits"
                ),
                "raw_minimum_sampled_clearance_m": clearance,
                "raw_clearance_kind": kind,
                "policy_result": result,
                "clearance_policy_accepted": accepted,
                "motion_authorization": MOTION_AUTHORIZATION_NOT_GRANTED,
            }
        )

    legacy_replay = None
    if frozen_g4_profile is not None:
        if frozen_g4_reference is None:
            raise GeometrySafetyPolicyError("G4 profile supplied without provenance reference")
        legacy_replay = reproduce_frozen_g4_policy(
            frozen_g4_profile,
            reference=frozen_g4_reference,
        )
    artifact: dict[str, Any] = {
        "schema_version": "matdog.geometry_safety_policy.v1",
        "generation_metadata": {
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        },
        "provenance": _source_provenance(),
        "input_parking_artifact": dict(sorted(parking_reference.items())),
        "policy": {
            "policy_id": POLICY_ID,
            "threshold_m": threshold_m,
            "geometry_mutated": False,
            "acceptance_scope": POLICY_ACCEPTANCE_SCOPE,
            "motion_authorization_rule": POLICY_MOTION_AUTHORIZATION_RULE,
            "geometry_feasibility_rule": "consumed unchanged from raw parking artifact",
            "clearance_rule": (
                "PASS when exact distance or conservative lower bound proves clearance >= threshold; "
                "exact below threshold FAIL; lower bound below threshold UNRESOLVED"
            ),
        },
        "summary": {
            "endpoint_count": len(results),
            "pass_count": sum(result["policy_result"] == POLICY_PASS for result in results),
            "fail_count": sum(
                result["policy_result"] in {POLICY_FAIL_EXACT, POLICY_REJECT_GEOMETRY}
                for result in results
            ),
            "unresolved_count": sum(
                result["policy_result"] in {POLICY_UNRESOLVED_BOUND, POLICY_UNRESOLVED_MISSING}
                for result in results
            ),
            "urdf_executable_target_count": sum(
                result["raw_target_within_urdf_limits"] is True for result in results
            ),
            "diagnostic_target_outside_urdf_limits_count": sum(
                result["raw_target_within_urdf_limits"] is False for result in results
            ),
            "clearance_pass_but_target_outside_urdf_limits_count": sum(
                result["policy_result"] == POLICY_PASS
                and result["raw_target_within_urdf_limits"] is False
                for result in results
            ),
            "motion_authorization_granted_count": 0,
        },
        "endpoint_policy_results": results,
        "frozen_g4_legacy_3mm_replay": legacy_replay,
    }
    artifact["semantic_content_sha256"] = safety_policy_content_sha256(artifact)
    return artifact


def validate_safety_policy_artifact(artifact: dict[str, Any]) -> None:
    if artifact.get("schema_version") != "matdog.geometry_safety_policy.v1":
        raise GeometrySafetyPolicyError(
            f"unexpected safety schema: {artifact.get('schema_version')!r}"
        )
    stored = artifact.get("semantic_content_sha256")
    recomputed = safety_policy_content_sha256(artifact)
    if stored != recomputed:
        raise GeometrySafetyPolicyError(
            f"safety policy semantic SHA mismatch: stored={stored!r}, recomputed={recomputed!r}"
        )
    policy = artifact.get("policy", {})
    if policy.get("geometry_mutated") is not False:
        raise GeometrySafetyPolicyError("safety artifact does not prove geometry remained immutable")
    threshold = policy.get("threshold_m")
    if not (
        isinstance(threshold, (int, float))
        and math.isfinite(threshold)
        and threshold >= 0.0
    ):
        raise GeometrySafetyPolicyError(f"invalid serialized safety threshold: {threshold!r}")
    if policy.get("policy_id") != POLICY_ID:
        raise GeometrySafetyPolicyError("unexpected safety policy identifier")
    if policy.get("acceptance_scope") != POLICY_ACCEPTANCE_SCOPE:
        raise GeometrySafetyPolicyError("safety policy acceptance scope is not clearance-only")
    if policy.get("motion_authorization_rule") != POLICY_MOTION_AUTHORIZATION_RULE:
        raise GeometrySafetyPolicyError("safety policy motion-authorization boundary changed")
    results = artifact.get("endpoint_policy_results", [])
    if len(results) != 24:
        raise GeometrySafetyPolicyError("safety artifact requires 24 endpoint results")
    indices = [result.get("canonical_endpoint_index") for result in results]
    if indices != list(range(24)):
        raise GeometrySafetyPolicyError("safety endpoint results are not in canonical 0..23 order")
    endpoint_ids = [result.get("endpoint_id") for result in results]
    if any(not isinstance(endpoint_id, str) or not endpoint_id for endpoint_id in endpoint_ids):
        raise GeometrySafetyPolicyError("safety endpoint identifier is missing")
    if len(set(endpoint_ids)) != 24:
        raise GeometrySafetyPolicyError("safety endpoint identifiers are not unique")
    provenance = artifact.get("provenance", {})
    source_manifest = provenance.get("source_file_sha256")
    if not isinstance(source_manifest, dict) or not source_manifest:
        raise GeometrySafetyPolicyError("safety policy source provenance is missing")
    if provenance.get("source_combined_sha256") != _canonical_hash(source_manifest, set()):
        raise GeometrySafetyPolicyError("safety policy source provenance digest mismatch")
    for relative_path, digest in source_manifest.items():
        if (
            not isinstance(relative_path, str)
            or Path(relative_path).is_absolute()
            or ".." in Path(relative_path).parts
            or not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise GeometrySafetyPolicyError("invalid safety policy source provenance entry")
    input_reference = artifact.get("input_parking_artifact", {})
    input_path = input_reference.get("path")
    if (
        not isinstance(input_path, str)
        or Path(input_path).is_absolute()
        or ".." in Path(input_path).parts
    ):
        raise GeometrySafetyPolicyError("parking input path is not repository-relative")
    for digest_key in ("file_sha256", "semantic_content_sha256"):
        digest = input_reference.get(digest_key)
        if (
            not isinstance(digest, str)
            or len(digest) != 64
            or any(character not in "0123456789abcdef" for character in digest)
        ):
            raise GeometrySafetyPolicyError(f"invalid parking input {digest_key}")
    feasible_outcomes = {
        PARKING_NOT_NEEDED,
        PARKING_FEASIBLE_1DOF,
        PARKING_FEASIBLE_2DOF,
    }
    for result in results:
        if result.get("raw_target_within_urdf_limits") not in {True, False}:
            raise GeometrySafetyPolicyError("safety result lacks target URDF-domain evidence")
        expected_target_domain = (
            TARGET_DOMAIN_EXECUTABLE
            if result["raw_target_within_urdf_limits"]
            else TARGET_DOMAIN_DIAGNOSTIC
        )
        if result.get("raw_target_domain") != expected_target_domain:
            raise GeometrySafetyPolicyError("target domain contradicts URDF-limit evidence")
        if result.get("motion_authorization") != MOTION_AUTHORIZATION_NOT_GRANTED:
            raise GeometrySafetyPolicyError("safety result improperly grants motion authorization")
        expected_feasible = result.get("raw_geometry_outcome") in feasible_outcomes
        if result.get("raw_geometry_feasible") is not expected_feasible:
            raise GeometrySafetyPolicyError("geometry feasibility does not match raw outcome")
        expected_result, expected_accepted = _evaluate_policy(
            geometry_feasible=expected_feasible,
            clearance_m=result.get("raw_minimum_sampled_clearance_m"),
            clearance_kind=result.get("raw_clearance_kind"),
            threshold_m=float(threshold),
        )
        if (
            result.get("policy_result") != expected_result
            or result.get("clearance_policy_accepted") is not expected_accepted
        ):
            raise GeometrySafetyPolicyError("serialized clearance-policy result is inconsistent")
    expected_summary = {
        "endpoint_count": len(results),
        "pass_count": sum(result["policy_result"] == POLICY_PASS for result in results),
        "fail_count": sum(
            result["policy_result"] in {POLICY_FAIL_EXACT, POLICY_REJECT_GEOMETRY}
            for result in results
        ),
        "unresolved_count": sum(
            result["policy_result"] in {POLICY_UNRESOLVED_BOUND, POLICY_UNRESOLVED_MISSING}
            for result in results
        ),
        "urdf_executable_target_count": sum(
            result["raw_target_within_urdf_limits"] is True for result in results
        ),
        "diagnostic_target_outside_urdf_limits_count": sum(
            result["raw_target_within_urdf_limits"] is False for result in results
        ),
        "clearance_pass_but_target_outside_urdf_limits_count": sum(
            result["policy_result"] == POLICY_PASS
            and result["raw_target_within_urdf_limits"] is False
            for result in results
        ),
        "motion_authorization_granted_count": 0,
    }
    if artifact.get("summary") != expected_summary:
        raise GeometrySafetyPolicyError("safety policy summary is inconsistent")
    legacy = artifact.get("frozen_g4_legacy_3mm_replay")
    if legacy is not None:
        if (
            legacy.get("content_sha256") != EXPECTED_G4_CONTENT_SHA256
            or legacy.get("exact_replay") is not True
            or legacy.get("reference_threshold_m") != REFERENCE_CLEARANCE_THRESHOLD_M
            or legacy.get("active_sequence_passed_count") != 0
            or legacy.get("active_sequence_failed_or_unresolved_count") != 4
            or len(legacy.get("plans", [])) != 4
        ):
            raise GeometrySafetyPolicyError("frozen G4 replay evidence is inconsistent")
        replayed_pass_count = 0
        for plan in legacy["plans"]:
            segments = plan.get("segments", [])
            if not segments or plan.get("sequence_exact_match") is not True:
                raise GeometrySafetyPolicyError("frozen G4 plan replay is not exact")
            for segment in segments:
                expected_passed = segment.get("replayed_gate_result") == "PASS"
                if (
                    segment.get("exact_match") is not True
                    or segment.get("replayed_gate_result")
                    != segment.get("stored_gate_result")
                    or segment.get("replayed_passed") is not expected_passed
                    or segment.get("stored_passed") is not expected_passed
                ):
                    raise GeometrySafetyPolicyError("frozen G4 segment replay is not exact")
            sequence_passed = all(segment["replayed_passed"] for segment in segments)
            if (
                plan.get("active_leg_sequence_replayed_passed") is not sequence_passed
                or plan.get("active_leg_sequence_stored_passed") is not sequence_passed
            ):
                raise GeometrySafetyPolicyError("frozen G4 sequence replay is inconsistent")
            replayed_pass_count += int(sequence_passed)
        if (
            replayed_pass_count != legacy["active_sequence_passed_count"]
            or len(legacy["plans"]) - replayed_pass_count
            != legacy["active_sequence_failed_or_unresolved_count"]
        ):
            raise GeometrySafetyPolicyError("frozen G4 replay counts are inconsistent")


def render_safety_policy_report(artifact: dict[str, Any]) -> str:
    validate_safety_policy_artifact(artifact)
    summary = artifact["summary"]
    lines = [
        "# MATDOG Geometry Compiler V5 — external safety policy",
        "",
        f"parking semantic SHA256: `{artifact['input_parking_artifact']['semantic_content_sha256']}`",
        f"safety semantic SHA256: `{artifact['semantic_content_sha256']}`",
        f"safety-policy source SHA256: `{artifact['provenance']['source_combined_sha256']}`",
        f"external threshold: `{artifact['policy']['threshold_m']}` m",
        "geometry mutated: `false`",
        "motion authorization granted: `false` (offline clearance evidence only)",
        "",
        f"Policy results: PASS {summary['pass_count']}, FAIL {summary['fail_count']}, "
        f"UNRESOLVED {summary['unresolved_count']} (24 total).",
        "",
        "| Endpoint | Raw geometry feasible | URDF target | Raw outcome | Clearance | Kind | External clearance policy | Clearance accepted | Motion authorization |",
        "|---|---:|---|---|---:|---|---|---:|---|",
    ]
    for result in artifact["endpoint_policy_results"]:
        lines.append(
            f"| {result['endpoint_id']} | {result['raw_geometry_feasible']} | "
            f"{result['raw_target_domain']} | {result['raw_geometry_outcome']} | "
            f"{result['raw_minimum_sampled_clearance_m']} | "
            f"{result['raw_clearance_kind']} | {result['policy_result']} | "
            f"{result['clearance_policy_accepted']} | "
            f"{result['motion_authorization']} |"
        )
    legacy = artifact.get("frozen_g4_legacy_3mm_replay")
    if legacy is not None:
        lines.extend(
            [
                "",
                "## Frozen G4 legacy 3 mm replay",
                "",
                f"G4 content SHA256: `{legacy['content_sha256']}`",
                f"exact serialized-policy replay: `{legacy['exact_replay']}`",
                f"active sequences passed: {legacy['active_sequence_passed_count']}/4",
                f"active sequences failed/unresolved: {legacy['active_sequence_failed_or_unresolved_count']}/4",
            ]
        )
    lines.extend(
        [
            "",
            "Geometric feasibility is unchanged input. Clearance is unchanged measured geometry.",
            "This external policy does not define endpoints, parking, meshes, or search behavior.",
            "",
        ]
    )
    return "\n".join(lines)


def write_safety_policy_artifact(
    artifact: dict[str, Any],
    json_path: Path,
) -> tuple[Path, Path]:
    validate_safety_policy_artifact(artifact)
    target = Path(json_path)
    if target.suffix != ".json":
        raise GeometrySafetyPolicyError("safety output must use the .json extension")
    report_path = target.with_suffix(".md")
    existing = [path for path in (target, report_path) if path.exists()]
    if existing:
        raise GeometrySafetyPolicyError(
            "refusing to overwrite safety artifact: "
            + ", ".join(str(path) for path in existing)
        )
    target.parent.mkdir(parents=True, exist_ok=True)

    def atomic_no_clobber(text: str, path: Path) -> None:
        descriptor, temporary_name = tempfile.mkstemp(
            dir=path.parent,
            prefix=f".{path.name}.",
            suffix=".tmp",
            text=True,
        )
        try:
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                handle.write(text)
                handle.flush()
                os.fsync(handle.fileno())
            os.link(temporary_name, path)
            os.unlink(temporary_name)
        except BaseException:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass
            raise

    atomic_no_clobber(json.dumps(artifact, indent=2, sort_keys=True) + "\n", target)
    try:
        atomic_no_clobber(render_safety_policy_report(artifact), report_path)
    except BaseException:
        target.unlink()
        raise
    return target, report_path


def _resolve_within_repo(
    repo_root: Path,
    path: Path,
    *,
    label: str,
    must_exist: bool,
) -> Path:
    resolved = Path(path).resolve(strict=must_exist)
    try:
        resolved.relative_to(repo_root)
    except ValueError as exc:
        raise GeometrySafetyPolicyError(
            f"{label} must remain inside repository root {repo_root}: {resolved}"
        ) from exc
    return resolved


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Apply an external clearance policy to a saved V5 parking artifact."
    )
    parser.add_argument("--repo-root", type=Path, required=True)
    parser.add_argument("--parking-artifact", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument(
        "--threshold-m",
        type=float,
        default=REFERENCE_CLEARANCE_THRESHOLD_M,
    )
    parser.add_argument("--frozen-g4-profile", type=Path)
    args = parser.parse_args()

    repo_root = Path(args.repo_root).resolve(strict=True)
    parking_path = _resolve_within_repo(
        repo_root,
        args.parking_artifact,
        label="parking artifact",
        must_exist=True,
    )
    output_path = _resolve_within_repo(
        repo_root,
        args.output_json,
        label="safety output",
        must_exist=False,
    )
    report_path = output_path.with_suffix(".md")
    input_paths = {parking_path}
    if args.frozen_g4_profile is not None:
        input_paths.add(
            _resolve_within_repo(
                repo_root,
                args.frozen_g4_profile,
                label="frozen G4 profile",
                must_exist=True,
            )
        )
    if output_path in input_paths or report_path in input_paths:
        raise GeometrySafetyPolicyError("safety output aliases an input artifact")
    if output_path.exists() or report_path.exists():
        raise GeometrySafetyPolicyError("refusing to overwrite an existing safety artifact")

    source_before = _source_provenance()
    parking_sha_before = _sha256_file(parking_path)
    parking = json.loads(parking_path.read_text(encoding="utf-8"))
    parking_reference = {
        "path": str(parking_path.relative_to(repo_root)),
        "file_sha256": parking_sha_before,
        "semantic_content_sha256": parking.get("semantic_content_sha256"),
    }
    g4_profile = None
    g4_reference = None
    if args.frozen_g4_profile is not None:
        g4_path = _resolve_within_repo(
            repo_root,
            args.frozen_g4_profile,
            label="frozen G4 profile",
            must_exist=True,
        )
        g4_sha_before = _sha256_file(g4_path)
        g4_profile = json.loads(g4_path.read_text(encoding="utf-8"))
        g4_reference = {
            "path": str(g4_path.relative_to(repo_root)),
            "file_sha256": g4_sha_before,
        }
    artifact = build_safety_policy_artifact(
        parking,
        parking_reference=parking_reference,
        threshold_m=args.threshold_m,
        frozen_g4_profile=g4_profile,
        frozen_g4_reference=g4_reference,
    )
    if _sha256_file(parking_path) != parking_sha_before:
        raise GeometrySafetyPolicyError("parking artifact changed during policy evaluation")
    if args.frozen_g4_profile is not None and _sha256_file(g4_path) != g4_sha_before:
        raise GeometrySafetyPolicyError("frozen G4 profile changed during policy evaluation")
    if _source_provenance() != source_before or artifact.get("provenance") != source_before:
        raise GeometrySafetyPolicyError("safety policy source changed during evaluation")
    json_path, report_path = write_safety_policy_artifact(artifact, output_path)
    written = json.loads(json_path.read_text(encoding="utf-8"))
    validate_safety_policy_artifact(written)
    if written != artifact:
        raise GeometrySafetyPolicyError("written safety artifact does not match in-memory artifact")
    if (
        _sha256_file(parking_path) != parking_sha_before
        or _source_provenance() != source_before
        or (
            args.frozen_g4_profile is not None
            and _sha256_file(g4_path) != g4_sha_before
        )
    ):
        raise GeometrySafetyPolicyError("policy provenance changed during output serialization")
    print(f"safety semantic SHA256: {artifact['semantic_content_sha256']}")
    print(f"safety JSON: {json_path}")
    print(f"safety report: {report_path}")
    print("NO HARDWARE USED. NO NORMA-CORE MODIFIED. NO MERGE PERFORMED.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
