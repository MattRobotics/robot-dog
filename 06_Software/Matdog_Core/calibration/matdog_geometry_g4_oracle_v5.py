#!/usr/bin/env python3
"""Read-only G4 → G7 same-new-geometry replay-oracle adapter.

The adapter extracts only geometric context and numerical search inputs from
the frozen schema-v4 profile. Hardware and safety verdicts are never imported
into V5 geometry decisions.  This module is refactor evidence only: a G4
same-context replay must never be published as the canonical, context-free V5
geometry profile.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
import time
from typing import Any, Iterable

from matdog_geometry_contact_search_v5 import (
    GEOMETRIC_CONTACT_FOUND,
    NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN,
    NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN,
    PATH_DOMAIN_FULL_ENDPOINT_ENVELOPE,
    PATH_OBSTRUCTION,
    EndpointAnalysisV5,
    EndpointSearchTaskV5,
    EndpointSpecV5,
    load_endpoint_specs,
)
from matdog_geometry_mesh_kernel import clear_mesh_cache
from matdog_geometry_process_workers_v5 import (
    execute_contact_tasks,
    scene_input_fingerprint,
)
from matdog_geometry_profile_v5 import geometry_source_manifest
from matdog_geometry_scene_v5 import RobotSceneV5


EXPECTED_G4_CONTENT_SHA256 = (
    "4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61"
)
EXPECTED_G4_FILE_SHA256 = (
    "f1b059a58c51508345ec583cc421cf9ca66ec8e9ac5547649c05bbf755e5c5fa"
)

# Equality for serialized numerical inputs is intentionally distinct from the
# search-result acceptance tolerance.  G4 rounds many inputs to 12 decimal
# places, so 1e-12 only absorbs that representation boundary; outcome angles,
# brackets, and declared-limit deltas continue to use the declared bisection
# resolution below.
SERIALIZED_INPUT_ABS_TOLERANCE = 1.0e-12


class G4OracleV5Error(RuntimeError):
    """Frozen G4 provenance or G4/G7 geometric comparison failed."""


@dataclass(frozen=True)
class G4ReplayTask(EndpointSearchTaskV5):
    """Explicitly non-canonical endpoint task using the frozen G4 context."""

    g4_record: dict[str, Any]

    def __post_init__(self) -> None:
        if self.path_domain_mode != PATH_DOMAIN_FULL_ENDPOINT_ENVELOPE:
            raise G4OracleV5Error(
                "G4 replay requires the historical FULL_ENDPOINT_ENVELOPE path domain"
            )


@dataclass(frozen=True)
class G4ReplayRunV5:
    """In-memory replay evidence; deliberately incapable of carrying a profile."""

    analyses: tuple[EndpointAnalysisV5, ...]
    comparison: dict[str, Any]
    runtime_seconds: float


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _require_repo_file(repo_root: Path, path: Path, label: str) -> Path:
    resolved = Path(path).resolve()
    try:
        resolved.relative_to(repo_root)
    except ValueError as exc:
        raise G4OracleV5Error(
            f"{label} must be inside the active repository: {resolved}"
        ) from exc
    if not resolved.is_file():
        raise G4OracleV5Error(f"{label} is not a file: {resolved}")
    return resolved


def _validate_replay_fingerprint(
    fingerprint: Any,
    g4_profile: dict[str, Any],
) -> None:
    q0_status = dict(fingerprint.q0_status_by_joint)
    separated_statuses = {
        "SEPARATED_AABB",
        "SEPARATED_HULL",
        "SEPARATED_NARROW",
    }
    if len(q0_status) != 12 or any(
        status not in separated_statuses for status in q0_status.values()
    ):
        raise G4OracleV5Error(
            "STOP: G4 replay q=0 gate must be exactly 12/12 separated under "
            "the closed collision-kernel status taxonomy; "
            f"observed={q0_status}"
        )

    expected_urdf_sha = g4_profile.get("urdf", {}).get("sha256")
    if fingerprint.urdf_sha256 != expected_urdf_sha:
        raise G4OracleV5Error(
            "STOP: G4 replay URDF SHA differs from frozen G4 provenance: "
            f"expected={expected_urdf_sha}, observed={fingerprint.urdf_sha256}"
        )

    expected_mesh_sha = {
        link_name: record.get("sha256")
        for link_name, record in g4_profile.get("collision_mesh_manifest", {}).items()
    }
    observed_mesh_sha = dict(fingerprint.collision_mesh_sha256)
    if (
        len(expected_mesh_sha) != 17
        or len(observed_mesh_sha) != 17
        or observed_mesh_sha != expected_mesh_sha
    ):
        raise G4OracleV5Error(
            "STOP: G4 replay collision geometry differs from the frozen 17-mesh manifest"
        )


def load_frozen_g4_profile(path: Path) -> dict[str, Any]:
    profile = json.loads(Path(path).read_text(encoding="utf-8"))
    observed = profile.get("content_sha256")
    canonical_content = {
        key: value
        for key, value in profile.items()
        if key not in {"generation_timestamp_utc", "content_sha256"}
    }
    recomputed = hashlib.sha256(
        json.dumps(
            canonical_content,
            sort_keys=True,
            separators=(",", ":"),
            ensure_ascii=False,
        ).encode("utf-8")
    ).hexdigest()
    if observed != EXPECTED_G4_CONTENT_SHA256 or recomputed != EXPECTED_G4_CONTENT_SHA256:
        raise G4OracleV5Error(
            "frozen G4 content SHA mismatch: "
            f"expected={EXPECTED_G4_CONTENT_SHA256}, stored={observed}, recomputed={recomputed}"
        )
    if profile.get("schema_version") != "matdog.calibration_geometry_profile.v4":
        raise G4OracleV5Error(
            f"unexpected G4 schema: {profile.get('schema_version')!r}"
        )
    return profile


def build_g4_replay_tasks(
    scene: RobotSceneV5,
    endpoint_specs: Iterable[EndpointSpecV5],
    g4_profile: dict[str, Any],
) -> tuple[G4ReplayTask, ...]:
    by_key = {
        (record["joint_name"], record["side"]): record
        for record in g4_profile["endpoints"]
    }
    numerical = g4_profile["numerical_parameters"]
    tasks: list[G4ReplayTask] = []
    for endpoint in endpoint_specs:
        key = (endpoint.joint_name, endpoint.side)
        record = by_key.get(key)
        if record is None:
            raise G4OracleV5Error(f"G4 endpoint missing: {key}")
        context = dict(record.get("prerequisite_pose_rad", {}))
        context.update(record.get("other_legs_pose_rad", {}))
        unknown = sorted(set(context) - set(scene.model.actuated_joint_names))
        if unknown:
            raise G4OracleV5Error(f"{key}: G4 context contains unknown joints {unknown}")
        context.pop(endpoint.joint_name, None)
        search = record["numerical_search"]
        tasks.append(
            G4ReplayTask(
                endpoint=endpoint,
                context_pose_rad=context,
                coarse_step_rad=float(search["coarse_step_rad"]),
                envelope_margin_rad=float(numerical["envelope_margin_rad"]),
                bisection_resolution_rad=float(search["bisection_resolution_rad"]),
                max_bisection_iterations=int(numerical["max_bisection_iterations"]),
                path_domain_mode=PATH_DOMAIN_FULL_ENDPOINT_ENVELOPE,
                g4_record=record,
            )
        )
    if len(tasks) != 24 or len(by_key) != 24:
        raise G4OracleV5Error(
            f"G4/G7 endpoint cardinality must be 24/24, got tasks={len(tasks)}, G4={len(by_key)}"
        )
    return tuple(tasks)


def _angle_difference(
    expected: float | None,
    observed: float | None,
) -> float | None:
    if expected is None or observed is None:
        return None
    return abs(float(expected) - float(observed))


def _pair_equal(
    expected: list[str] | tuple[str, str] | None,
    observed: list[str] | tuple[str, str] | None,
) -> bool:
    if expected is None or observed is None:
        return expected is None and observed is None
    return frozenset(expected) == frozenset(observed)


def _ordered_pair_equal(
    expected: list[str] | tuple[str, str] | None,
    observed: list[str] | tuple[str, str] | None,
) -> bool:
    if expected is None or observed is None:
        return expected is None and observed is None
    return tuple(expected) == tuple(observed)


def _number_equal(
    expected: float | int | None,
    observed: float | int | None,
    *,
    tolerance: float = SERIALIZED_INPUT_ABS_TOLERANCE,
) -> bool:
    if expected is None or observed is None:
        return expected is None and observed is None
    return math.isclose(
        float(expected),
        float(observed),
        rel_tol=0.0,
        abs_tol=tolerance,
    )


def _numbers_equal(
    expected: Iterable[float],
    observed: Iterable[float],
    *,
    tolerance: float = SERIALIZED_INPUT_ABS_TOLERANCE,
) -> bool:
    expected_values = tuple(expected)
    observed_values = tuple(observed)
    return len(expected_values) == len(observed_values) and all(
        _number_equal(left, right, tolerance=tolerance)
        for left, right in zip(expected_values, observed_values)
    )


def _poses_equal(expected: dict[str, float], observed: dict[str, float]) -> bool:
    return set(expected) == set(observed) and all(
        _number_equal(expected[name], observed[name]) for name in expected
    )


def _legacy_endpoint_id(endpoint: EndpointSpecV5) -> str:
    joint_stem = (
        endpoint.joint_name[: -len("_joint")]
        if endpoint.joint_name.endswith("_joint")
        else endpoint.joint_name
    )
    return f"{joint_stem}_{endpoint.side}"


def _g4_context(g4: dict[str, Any], endpoint: EndpointSpecV5) -> dict[str, float]:
    context = dict(g4.get("prerequisite_pose_rad", {}))
    context.update(g4.get("other_legs_pose_rad", {}))
    context.pop(endpoint.joint_name, None)
    return context


def _g4_contact_status(result_kind: str) -> str | None:
    return {
        "MESH_CONTACT_FOUND": GEOMETRIC_CONTACT_FOUND,
        "NO_MESH_CONTACT_IN_ENVELOPE": NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN,
    }.get(result_kind)


def _iterations_for_full_coarse_bracket(
    coarse_step_rad: float,
    bisection_resolution_rad: float,
    max_bisection_iterations: int,
) -> int:
    width = float(coarse_step_rad)
    iterations = 0
    while (
        width > float(bisection_resolution_rad)
        and iterations < int(max_bisection_iterations)
    ):
        width /= 2.0
        iterations += 1
    return iterations


def _optional_angle_match(
    expected: float | None,
    observed: float | None,
    tolerance: float,
) -> bool:
    delta = _angle_difference(expected, observed)
    if expected is None or observed is None:
        return expected is None and observed is None
    assert delta is not None
    return delta <= tolerance


def compare_g4_g7(
    tasks: Iterable[G4ReplayTask],
    analyses: Iterable[EndpointAnalysisV5],
) -> dict[str, Any]:
    task_values = tuple(tasks)
    analysis_values = tuple(analyses)
    task_by_id = {task.endpoint.endpoint_id: task for task in task_values}
    analysis_by_id = {
        analysis.geometry.endpoint.endpoint_id: analysis for analysis in analysis_values
    }
    if len(task_by_id) != len(task_values):
        raise G4OracleV5Error("G4 replay tasks contain duplicate endpoint identities")
    if len(analysis_by_id) != len(analysis_values):
        raise G4OracleV5Error("G7 replay analyses contain duplicate endpoint identities")
    if set(task_by_id) != set(analysis_by_id):
        raise G4OracleV5Error("G4 tasks and G7 analyses cover different endpoints")

    comparisons = []
    unexpected: list[str] = []
    unexpected_checks: dict[str, list[str]] = {}
    for endpoint_id in task_by_id:
        task = task_by_id[endpoint_id]
        analysis = analysis_by_id[endpoint_id]
        g4 = task.g4_record
        g7 = analysis.geometry
        endpoint = task.endpoint
        g7_endpoint = g7.endpoint
        search = g4["numerical_search"]
        tolerance = max(
            float(search["bisection_resolution_rad"]),
            float(g7.bisection_resolution_rad),
        )
        expected_contact_status = _g4_contact_status(str(g4.get("result_kind")))
        expected_found = expected_contact_status == GEOMETRIC_CONTACT_FOUND
        contact_delta = _angle_difference(
            g4.get("mesh_predicted_contact_rad"),
            g7.contact_angle_rad,
        )
        contact_angle_match = _optional_angle_match(
            g4.get("mesh_predicted_contact_rad"),
            g7.contact_angle_rad,
            tolerance,
        )

        g4_declared = g4.get("urdf_declared_limit_rad")
        g4_envelope = tuple(float(value) for value in search["analysis_envelope_rad"])
        expected_g4_envelope = tuple(
            sorted(
                (
                    endpoint.urdf_declared_limit_rad - task.envelope_margin_rad,
                    endpoint.urdf_declared_limit_rad + task.envelope_margin_rad,
                )
            )
        )
        far_bound = g4_envelope[0] if endpoint.side == "min" else g4_envelope[1]
        expected_g7_domain = (min(0.0, far_bound), max(0.0, far_bound))
        expected_context = _g4_context(g4, endpoint)

        g4_path_angle = g4.get("path_collision_angle_rad")
        g4_path_pair = g4.get("path_collision_link_pair")
        expected_path = g4_path_angle is not None
        expected_path_status = (
            PATH_OBSTRUCTION
            if expected_path
            else NO_PATH_OBSTRUCTION_IN_SEARCH_DOMAIN
        )
        g7_path = analysis.path
        g7_path_angle = g7_path.obstruction_angle_rad if g7_path is not None else None
        g7_path_pair = g7_path.obstruction_link_pair if g7_path is not None else None
        path_delta = _angle_difference(g4_path_angle, g7_path_angle)
        expected_precedes = bool(
            expected_path
            and (
                not expected_found
                or abs(float(g4_path_angle))
                < abs(float(g4.get("mesh_predicted_contact_rad")))
            )
        )

        path_bracket_clear_match = False
        path_bracket_contact_match = False
        path_bisection_iterations_match = False
        path_record_shape_valid = False
        if g7_path is not None:
            if expected_path:
                # G4 does not serialize the path clear-side bracket or its
                # iteration count.  For every frozen G4 path event the event
                # lies strictly before the final (possibly truncated) coarse
                # interval, so both values are deterministically reconstructible
                # from the recorded full coarse step, resolution, side, and
                # contact-side bracket bound (`path_collision_angle_rad`).
                full_step_event = (
                    abs(far_bound - float(g4_path_angle))
                    > task.coarse_step_rad + SERIALIZED_INPUT_ABS_TOLERANCE
                )
                expected_path_iterations = _iterations_for_full_coarse_bracket(
                    task.coarse_step_rad,
                    task.bisection_resolution_rad,
                    task.max_bisection_iterations,
                )
                final_width = task.coarse_step_rad / (2.0 ** expected_path_iterations)
                sign = -1.0 if endpoint.side == "min" else 1.0
                expected_path_clear = float(g4_path_angle) - sign * final_width
                path_bracket_clear_match = bool(
                    full_step_event
                    and _optional_angle_match(
                        expected_path_clear,
                        g7_path.bracket_clear_rad,
                        tolerance,
                    )
                )
                path_bracket_contact_match = _optional_angle_match(
                    g4_path_angle,
                    g7_path.bracket_contact_rad,
                    tolerance,
                )
                path_bisection_iterations_match = bool(
                    full_step_event
                    and g7_path.bisection_iterations == expected_path_iterations
                )
                path_record_shape_valid = all(
                    (
                        g7_path.obstruction_angle_rad is not None,
                        g7_path.obstruction_link_pair is not None,
                        g7_path.relation is not None,
                        g7_path.bracket_clear_rad is not None,
                        g7_path.bracket_contact_rad is not None,
                    )
                )
            else:
                path_bracket_clear_match = g7_path.bracket_clear_rad is None
                path_bracket_contact_match = g7_path.bracket_contact_rad is None
                path_bisection_iterations_match = g7_path.bisection_iterations == 0
                path_record_shape_valid = all(
                    (
                        g7_path.obstruction_angle_rad is None,
                        g7_path.obstruction_link_pair is None,
                        g7_path.relation is None,
                    )
                )

        # Active pairs retain parent -> child order.  Collision-result pairs
        # are physically unordered and therefore compare as sets.
        active_pair_match = all(
            (
                _ordered_pair_equal(g4.get("active_revolute_pair"), endpoint.active_link_pair),
                _ordered_pair_equal(endpoint.active_link_pair, g7_endpoint.active_link_pair),
            )
        )
        if expected_found:
            contact_pair_match = all(
                (
                    _pair_equal(
                        g4.get("contact_link_pair"),
                        g7.contact_link_pair,
                    ),
                    _pair_equal(g7.contact_link_pair, g7_endpoint.active_link_pair),
                )
            )
        else:
            # G4 has no result pair for a no-contact record; V5 deliberately
            # retains the active pair that was searched.
            contact_pair_match = bool(
                g4.get("contact_link_pair") is None
                and _ordered_pair_equal(g7.contact_link_pair, g7_endpoint.active_link_pair)
            )

        checks = {
            "endpoint_identity": all(
                (
                    g4.get("endpoint_id") == _legacy_endpoint_id(endpoint),
                    g4.get("joint_name") == endpoint.joint_name,
                    g4.get("side") == endpoint.side,
                    g7_endpoint.endpoint_id == endpoint.endpoint_id,
                    g7_endpoint.joint_name == endpoint.joint_name,
                    g7_endpoint.side == endpoint.side,
                )
            ),
            "active_pair": active_pair_match,
            "contact_status": bool(
                expected_contact_status is not None
                and g7.status == expected_contact_status
            ),
            "contact_angle": contact_angle_match,
            "contact_pair": contact_pair_match,
            "contact_bracket_clear": _optional_angle_match(
                g4.get("bracket_clear_rad"),
                g7.bracket_clear_rad,
                tolerance,
            ),
            "contact_bracket_contact": _optional_angle_match(
                g4.get("bracket_contact_rad"),
                g7.bracket_contact_rad,
                tolerance,
            ),
            "declared_limit": all(
                (
                    _number_equal(
                        g4_declared,
                        endpoint.urdf_declared_limit_rad,
                    ),
                    _number_equal(
                        endpoint.urdf_declared_limit_rad,
                        g7_endpoint.urdf_declared_limit_rad,
                    ),
                )
            ),
            "declared_limit_delta": _optional_angle_match(
                g4.get("delta_from_declared_rad"),
                g7.declared_limit_delta_rad,
                tolerance,
            ),
            "g4_analysis_envelope": _numbers_equal(
                g4_envelope,
                expected_g4_envelope,
            ),
            "contact_search_start": _number_equal(g7.search_start_rad, 0.0),
            "contact_search_domain": _numbers_equal(
                expected_g7_domain,
                g7.search_domain_rad,
            ),
            "contact_coarse_step": all(
                (
                    _number_equal(search.get("coarse_step_rad"), task.coarse_step_rad),
                    _number_equal(task.coarse_step_rad, g7.coarse_step_rad),
                )
            ),
            "contact_bisection_resolution": all(
                (
                    _number_equal(
                        search.get("bisection_resolution_rad"),
                        task.bisection_resolution_rad,
                    ),
                    _number_equal(
                        task.bisection_resolution_rad,
                        g7.bisection_resolution_rad,
                    ),
                )
            ),
            "contact_max_bisection_iterations": (
                g7.max_bisection_iterations == task.max_bisection_iterations
            ),
            "contact_bisection_iterations": (
                g7.bisection_iterations == int(search.get("bisection_iterations", -1))
            ),
            "replay_context": all(
                (
                    _poses_equal(expected_context, task.context_pose_rad),
                    _poses_equal(task.context_pose_rad, g7.context_pose_rad),
                )
            ),
            "g4_path_record_shape": (
                (g4_path_angle is None) == (g4_path_pair is None)
            ),
            "path_analysis_present": g7_path is not None,
            "path_endpoint_identity": bool(
                g7_path is not None and g7_path.endpoint_id == endpoint.endpoint_id
            ),
            "path_status": bool(
                g7_path is not None and g7_path.status == expected_path_status
            ),
            "path_angle": _optional_angle_match(
                g4_path_angle,
                g7_path_angle,
                tolerance,
            ),
            "path_pair": _pair_equal(g4_path_pair, g7_path_pair),
            "path_bracket_clear": path_bracket_clear_match,
            "path_bracket_contact": path_bracket_contact_match,
            "path_search_domain": bool(
                g7_path is not None
                and _numbers_equal(expected_g7_domain, g7_path.search_domain_rad)
            ),
            "path_coarse_step": bool(
                g7_path is not None
                and _number_equal(task.coarse_step_rad, g7_path.coarse_step_rad)
            ),
            "path_bisection_resolution": bool(
                g7_path is not None
                and _number_equal(
                    task.bisection_resolution_rad,
                    g7_path.bisection_resolution_rad,
                )
            ),
            "path_max_bisection_iterations": bool(
                g7_path is not None
                and g7_path.max_bisection_iterations
                == task.max_bisection_iterations
            ),
            "path_bisection_iterations": path_bisection_iterations_match,
            "path_record_shape": path_record_shape_valid,
            "path_context": bool(
                g7_path is not None
                and _poses_equal(task.context_pose_rad, g7_path.context_pose_rad)
            ),
            "path_precedes_contact": (
                analysis.path_obstruction_precedes_contact == expected_precedes
            ),
            "g4_path_classification": (
                (g4.get("contact_model_status") == "PATH_COLLISION_BEFORE_ENDPOINT")
                == expected_precedes
            ),
        }

        endpoint_pass = all(checks.values())
        failed_checks = sorted(name for name, passed in checks.items() if not passed)
        if not endpoint_pass:
            unexpected.append(endpoint_id)
            unexpected_checks[endpoint_id] = failed_checks

        comparisons.append(
            {
                "endpoint_id": endpoint_id,
                "joint_name": task.endpoint.joint_name,
                "limit_side": task.endpoint.side,
                "numerical_tolerance_rad": tolerance,
                "g4_legacy_contact_model_status": g4.get("contact_model_status"),
                "g4_contact_found": expected_found,
                "g7_geometric_contact_status": g7.status,
                "g4_contact_angle_rad": g4.get("mesh_predicted_contact_rad"),
                "g7_contact_angle_rad": g7.contact_angle_rad,
                "contact_abs_delta_rad": contact_delta,
                "contact_pair_match": contact_pair_match,
                "contact_bracket_clear_abs_delta_rad": _angle_difference(
                    g4.get("bracket_clear_rad"),
                    g7.bracket_clear_rad,
                ),
                "contact_bracket_contact_abs_delta_rad": _angle_difference(
                    g4.get("bracket_contact_rad"),
                    g7.bracket_contact_rad,
                ),
                "declared_limit_delta_abs_delta_rad": _angle_difference(
                    g4.get("delta_from_declared_rad"),
                    g7.declared_limit_delta_rad,
                ),
                "g4_path_obstruction_angle_rad": g4_path_angle,
                "g7_path_obstruction_angle_rad": g7_path_angle,
                "path_abs_delta_rad": path_delta,
                "path_pair_match": checks["path_pair"],
                "g7_path_relation": g7_path.relation if g7_path is not None else None,
                "g7_path_precedes_contact": analysis.path_obstruction_precedes_contact,
                "checks": checks,
                "failed_checks": failed_checks,
                "comparison_status": "PASS" if endpoint_pass else "UNEXPECTED_REGRESSION",
            }
        )

    found_count = sum(
        analysis.geometry.status == GEOMETRIC_CONTACT_FOUND
        for analysis in analysis_by_id.values()
    )
    no_contact_count = sum(
        analysis.geometry.status == NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN
        for analysis in analysis_by_id.values()
    )
    path_count = sum(
        analysis.path is not None and analysis.path.status == PATH_OBSTRUCTION
        for analysis in analysis_by_id.values()
    )
    preceding_count = sum(
        analysis.path_obstruction_precedes_contact is True
        for analysis in analysis_by_id.values()
    )
    contact_deltas = [
        row["contact_abs_delta_rad"]
        for row in comparisons
        if row["contact_abs_delta_rad"] is not None
    ]
    path_deltas = [
        row["path_abs_delta_rad"]
        for row in comparisons
        if row["path_abs_delta_rad"] is not None
    ]
    result = {
        "oracle": "G4 frozen-context replay -> V5 same-context refactor oracle",
        "artifact_role": "NONCANONICAL_G4_REPLAY_ORACLE",
        "canonical_profile_eligible": False,
        "canonical_use": "NON_CANONICAL_REPLAY_EVIDENCE_ONLY",
        "g4_content_sha256": EXPECTED_G4_CONTENT_SHA256,
        "comparison_rule": (
            "outcome angle/bracket/delta absolute difference <= max(G4,G7 "
            "bisection resolution); serialized numerical inputs compare at their "
            "12-decimal representation boundary"
        ),
        "comparison_coverage": [
            "endpoint identity (legacy presentation ID, joint, side, replay ID)",
            "ordered active revolute parent-child pair",
            "contact status, angle, unordered contact pair, clear/contact bracket",
            "declared limit and declared-limit delta",
            "G4 reported analysis envelope and G7 q=0-to-far-bound search domain",
            "contact coarse step, bisection resolution, maximum and actual iterations",
            "exact G4 replay context at task, geometric search, and path search boundaries",
            "path status, angle, unordered pair, reconstructed clear/contact bracket",
            "path search domain, coarse step, bisection resolution, maximum and reconstructed iterations",
            "path-before-contact classification",
        ],
        "representation_exceptions": [
            (
                "G4 numerical_search.analysis_envelope_rad is a symmetric reporting "
                "envelope around the declared limit, while both algorithms actually "
                "sweep from q=0 to the directional far bound; the oracle validates the "
                "G4 envelope and compares that far bound to the complete G7 search domain."
            ),
            (
                "G4 does not serialize a path relation; G7 relation presence/absence is "
                "validated but equality is not claimed."
            ),
            (
                "G4 does not directly serialize the path clear-side bracket or path "
                "bisection count; for all six frozen interior full-coarse-step events "
                "they are deterministically reconstructed from side, path contact bound, "
                "coarse step, resolution, and maximum iterations before comparison."
            ),
            (
                "For a hypothetical G4 no-contact result, G4 has no contact result pair "
                "whereas V5 retains the active pair that was searched; the ordered active "
                "pair is compared instead.  Frozen G4 contains 24 contacts."
            ),
        ],
        "status": "PASS" if not unexpected else "STOP_UNEXPECTED_GEOMETRIC_REGRESSION",
        "endpoint_count": len(comparisons),
        "geometric_contact_found_count": found_count,
        "no_geometric_contact_count": no_contact_count,
        "path_obstruction_count": path_count,
        "path_obstruction_precedes_contact_count": preceding_count,
        "tight_replay_diagnostic": {
            "policy_effect": "DIAGNOSTIC_ONLY_DOES_NOT_TIGHTEN_ACCEPTANCE",
            "max_contact_abs_delta_rad": max(contact_deltas, default=None),
            "max_path_abs_delta_rad": max(path_deltas, default=None),
        },
        "unexpected_regression_endpoint_ids": unexpected,
        "unexpected_regression_checks_by_endpoint": unexpected_checks,
        "expected_semantic_differences": [
            "hardware evidence and hardware verdicts are absent from V5 pure geometry",
            "legacy MODEL_INCOMPLETE records remain ordinary geometric contacts",
            "legacy PATH_COLLISION_BEFORE_ENDPOINT is represented as contact plus independent obstruction",
            "PATH_OBSTRUCTION relation is topology-derived and is not automatically cross-leg",
        ],
        "comparisons": comparisons,
    }
    if unexpected:
        details = "; ".join(
            f"{endpoint_id}({','.join(unexpected_checks[endpoint_id])})"
            for endpoint_id in unexpected
        )
        raise G4OracleV5Error(
            "unexpected G4/G7 geometric regression: " + details
        )
    return result


def run_g4_replay_v5(
    *,
    repo_root: Path,
    urdf_path: Path,
    g4_profile_path: Path,
    workers: int,
    source_files: tuple[str, ...],
) -> G4ReplayRunV5:
    """Execute the frozen-context refactor oracle without building a profile.

    The API intentionally returns only analyses, comparison evidence, and the
    contact-compute runtime.  It has no profile or artifact writer, so replay
    context cannot accidentally flow into canonical V5 serialization.
    """

    if workers not in (1, 4):
        raise G4OracleV5Error(f"workers must be exactly 1 or 4, got {workers}")
    if not source_files or len(source_files) != len(set(source_files)):
        raise G4OracleV5Error("source_files must be a non-empty unique tuple")
    if "matdog_geometry_g4_oracle_v5.py" not in source_files:
        raise G4OracleV5Error(
            "G4 replay source provenance must include matdog_geometry_g4_oracle_v5.py"
        )

    repo_root = Path(repo_root).resolve()
    module_repo_root = Path(__file__).resolve().parents[3]
    if repo_root != module_repo_root:
        raise G4OracleV5Error(
            "repo_root does not match the worktree that loaded the replay code: "
            f"expected={module_repo_root}, observed={repo_root}"
        )
    urdf_path = _require_repo_file(repo_root, urdf_path, "URDF")
    g4_profile_path = _require_repo_file(
        repo_root,
        g4_profile_path,
        "frozen G4 profile",
    )

    initial_g4_file_sha = _sha256_file(g4_profile_path)
    if initial_g4_file_sha != EXPECTED_G4_FILE_SHA256:
        raise G4OracleV5Error(
            "STOP: frozen G4 file SHA mismatch: "
            f"expected={EXPECTED_G4_FILE_SHA256}, observed={initial_g4_file_sha}"
        )
    g4_profile = load_frozen_g4_profile(g4_profile_path)
    initial_source_manifest = geometry_source_manifest(source_files)

    scene = RobotSceneV5.from_urdf(urdf_path)
    try:
        input_fingerprint = scene_input_fingerprint(scene)
        _validate_replay_fingerprint(input_fingerprint, g4_profile)
        endpoint_specs = load_endpoint_specs(scene.model)
        tasks = build_g4_replay_tasks(scene, endpoint_specs, g4_profile)
        if len(tasks) != 24 or any(
            task.path_domain_mode != PATH_DOMAIN_FULL_ENDPOINT_ENVELOPE
            for task in tasks
        ):
            raise G4OracleV5Error(
                "STOP: G4 replay must contain 24 FULL_ENDPOINT_ENVELOPE tasks"
            )
        actuated_joint_names = scene.model.actuated_joint_names
    finally:
        del scene
        clear_mesh_cache()

    started = time.perf_counter()
    analyses = execute_contact_tasks(
        urdf_path,
        actuated_joint_names,
        input_fingerprint,
        tasks,
        workers=workers,
        include_path_obstruction=True,
    )
    runtime_seconds = time.perf_counter() - started
    comparison = compare_g4_g7(tasks, analyses)

    postflight_scene = RobotSceneV5.from_urdf(urdf_path)
    try:
        postflight_fingerprint = scene_input_fingerprint(postflight_scene)
        _validate_replay_fingerprint(postflight_fingerprint, g4_profile)
        if postflight_fingerprint != input_fingerprint:
            raise G4OracleV5Error(
                "STOP: URDF/mesh/q0 fingerprint changed during G4 replay"
            )
    finally:
        del postflight_scene
        clear_mesh_cache()

    if _sha256_file(g4_profile_path) != initial_g4_file_sha:
        raise G4OracleV5Error("STOP: frozen G4 file changed during replay")
    # Re-read the frozen artifact so both byte-level and canonical-content
    # provenance are revalidated after worker completion.
    load_frozen_g4_profile(g4_profile_path)
    if geometry_source_manifest(source_files) != initial_source_manifest:
        raise G4OracleV5Error(
            "STOP: replay source provenance changed during G4 execution"
        )

    comparison["replay_execution_provenance"] = {
        "workers": workers,
        "path_domain_mode": PATH_DOMAIN_FULL_ENDPOINT_ENVELOPE,
        "g4_file_sha256": initial_g4_file_sha,
        "g4_content_sha256": EXPECTED_G4_CONTENT_SHA256,
        "urdf_sha256": input_fingerprint.urdf_sha256,
        "collision_mesh_sha256": dict(input_fingerprint.collision_mesh_sha256),
        "q0_status_by_joint": dict(input_fingerprint.q0_status_by_joint),
        "source_file_sha256": initial_source_manifest,
    }
    return G4ReplayRunV5(
        analyses=analyses,
        comparison=comparison,
        runtime_seconds=runtime_seconds,
    )


def render_g4_g7_report(comparison: dict[str, Any]) -> str:
    def degrees_or_dash(value: float | None) -> str:
        return "-" if value is None else f"{math.degrees(value):+.6f}"

    def scientific_or_dash(value: float | None) -> str:
        return "-" if value is None else f"{value:.3e}"

    diagnostic = comparison.get("tight_replay_diagnostic", {})
    lines = [
        "# MATDOG G4 → V5 frozen-context replay oracle (non-canonical)",
        "",
        f"status: **{comparison['status']}**",
        f"artifact role: **{comparison.get('artifact_role', 'NONCANONICAL_G4_REPLAY_ORACLE')}**",
        f"canonical profile eligible: **{str(comparison.get('canonical_profile_eligible', False)).lower()}**",
        f"G4 content SHA256: `{comparison['g4_content_sha256']}`",
        f"endpoints: {comparison['endpoint_count']}",
        f"geometric contacts: {comparison['geometric_contact_found_count']}",
        f"no geometric contact: {comparison.get('no_geometric_contact_count', 0)}",
        f"path obstructions: {comparison['path_obstruction_count']}",
        f"obstructions preceding contact: {comparison['path_obstruction_precedes_contact_count']}",
        (
            "tight replay max contact delta (diagnostic only): "
            f"{scientific_or_dash(diagnostic.get('max_contact_abs_delta_rad'))} rad"
        ),
        (
            "tight replay max path delta (diagnostic only): "
            f"{scientific_or_dash(diagnostic.get('max_path_abs_delta_rad'))} rad"
        ),
        f"canonical use: **{comparison.get('canonical_use', 'NON_CANONICAL_REPLAY_EVIDENCE_ONLY')}**",
        "",
        "| Endpoint | G4 contact (deg) | G7 contact (deg) | abs delta (rad) | Path relation | Result |",
        "|---|---:|---:|---:|---|---|",
    ]

    for row in comparison["comparisons"]:
        g4_angle = row["g4_contact_angle_rad"]
        g7_angle = row["g7_contact_angle_rad"]
        lines.append(
            f"| {row['endpoint_id']} | "
            f"{degrees_or_dash(g4_angle)} | {degrees_or_dash(g7_angle)} | "
            f"{scientific_or_dash(row['contact_abs_delta_rad'])} | "
            f"{row['g7_path_relation'] or '-'} | "
            f"{row['comparison_status']} |"
        )
    lines.extend(
        [
            "",
            "Comparison coverage:",
            "",
            *[f"- {item}" for item in comparison.get("comparison_coverage", [])],
            "",
            "Exact representation exceptions:",
            "",
            *[f"- {item}" for item in comparison.get("representation_exceptions", [])],
            "",
            "Expected semantic separation:",
            "",
            *[f"- {item}" for item in comparison["expected_semantic_differences"]],
            "",
            "No hardware evidence or safety policy was used to define V5 geometry.",
        ]
    )
    return "\n".join(lines) + "\n"
