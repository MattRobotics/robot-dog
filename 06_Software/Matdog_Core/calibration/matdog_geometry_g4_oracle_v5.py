#!/usr/bin/env python3
"""Read-only G4 → G7 same-new-geometry oracle adapter.

The adapter extracts only geometric context and numerical search inputs from
the frozen schema-v4 profile. Hardware and safety verdicts are never imported
into V5 geometry decisions.
"""

from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable

from matdog_geometry_contact_search_v5 import (
    GEOMETRIC_CONTACT_FOUND,
    NO_GEOMETRIC_CONTACT_IN_SEARCH_DOMAIN,
    PATH_OBSTRUCTION,
    EndpointAnalysisV5,
    EndpointSpecV5,
)
from matdog_geometry_scene_v5 import RobotSceneV5


EXPECTED_G4_CONTENT_SHA256 = (
    "4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61"
)


class G4OracleV5Error(RuntimeError):
    """Frozen G4 provenance or G4/G7 geometric comparison failed."""


@dataclass(frozen=True)
class G4ReplayTask:
    endpoint: EndpointSpecV5
    context_pose_rad: dict[str, float]
    coarse_step_rad: float
    envelope_margin_rad: float
    bisection_resolution_rad: float
    max_bisection_iterations: int
    g4_record: dict[str, Any]


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


def _pair_equal(expected: list[str] | None, observed: tuple[str, str] | None) -> bool:
    if expected is None or observed is None:
        return expected is None and observed is None
    return frozenset(expected) == frozenset(observed)


def compare_g4_g7(
    tasks: Iterable[G4ReplayTask],
    analyses: Iterable[EndpointAnalysisV5],
) -> dict[str, Any]:
    task_by_id = {task.endpoint.endpoint_id: task for task in tasks}
    analysis_by_id = {
        analysis.geometry.endpoint.endpoint_id: analysis for analysis in analyses
    }
    if set(task_by_id) != set(analysis_by_id):
        raise G4OracleV5Error("G4 tasks and G7 analyses cover different endpoints")

    comparisons = []
    unexpected: list[str] = []
    for endpoint_id in task_by_id:
        task = task_by_id[endpoint_id]
        analysis = analysis_by_id[endpoint_id]
        g4 = task.g4_record
        g7 = analysis.geometry
        tolerance = max(
            float(g4["numerical_search"]["bisection_resolution_rad"]),
            float(g7.bisection_resolution_rad),
        )
        expected_found = g4["result_kind"] == "MESH_CONTACT_FOUND"
        observed_found = g7.status == GEOMETRIC_CONTACT_FOUND
        contact_delta = _angle_difference(
            g4.get("mesh_predicted_contact_rad"),
            g7.contact_angle_rad,
        )
        contact_pair_match = _pair_equal(
            g4.get("contact_link_pair"),
            g7.contact_link_pair,
        )
        contact_angle_match = (
            contact_delta is not None and contact_delta <= tolerance
            if expected_found and observed_found
            else expected_found == observed_found
        )

        g4_path_angle = g4.get("path_collision_angle_rad")
        g4_path_pair = g4.get("path_collision_link_pair")
        g7_path = analysis.path
        g7_path_angle = g7_path.obstruction_angle_rad if g7_path is not None else None
        g7_path_pair = g7_path.obstruction_link_pair if g7_path is not None else None
        path_delta = _angle_difference(g4_path_angle, g7_path_angle)
        expected_path = g4_path_angle is not None
        observed_path = (
            g7_path is not None
            and g7_path.status == PATH_OBSTRUCTION
            and g7_path_angle is not None
        )
        path_angle_match = (
            path_delta is not None and path_delta <= tolerance
            if expected_path and observed_path
            else expected_path == observed_path
        )
        path_pair_match = _pair_equal(g4_path_pair, g7_path_pair)

        endpoint_pass = all(
            (
                expected_found == observed_found,
                contact_angle_match,
                contact_pair_match,
                path_angle_match,
                path_pair_match,
            )
        )
        if not endpoint_pass:
            unexpected.append(endpoint_id)

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
                "g4_path_obstruction_angle_rad": g4_path_angle,
                "g7_path_obstruction_angle_rad": g7_path_angle,
                "path_abs_delta_rad": path_delta,
                "path_pair_match": path_pair_match,
                "g7_path_relation": g7_path.relation if g7_path is not None else None,
                "g7_path_precedes_contact": analysis.path_obstruction_precedes_contact,
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
    result = {
        "oracle": "G4 new geometry Phase1B -> G7 same new geometry V5 pure geometry",
        "g4_content_sha256": EXPECTED_G4_CONTENT_SHA256,
        "comparison_rule": "absolute angle delta <= max(G4,G7 bisection resolution)",
        "status": "PASS" if not unexpected else "STOP_UNEXPECTED_GEOMETRIC_REGRESSION",
        "endpoint_count": len(comparisons),
        "geometric_contact_found_count": found_count,
        "no_geometric_contact_count": no_contact_count,
        "path_obstruction_count": path_count,
        "path_obstruction_precedes_contact_count": preceding_count,
        "unexpected_regression_endpoint_ids": unexpected,
        "expected_semantic_differences": [
            "hardware evidence and hardware verdicts are absent from V5 pure geometry",
            "legacy MODEL_INCOMPLETE records remain ordinary geometric contacts",
            "legacy PATH_COLLISION_BEFORE_ENDPOINT is represented as contact plus independent obstruction",
            "PATH_OBSTRUCTION relation is topology-derived and is not automatically cross-leg",
        ],
        "comparisons": comparisons,
    }
    if unexpected:
        raise G4OracleV5Error(
            "unexpected G4/G7 geometric regression: " + ", ".join(unexpected)
        )
    return result


def render_g4_g7_report(comparison: dict[str, Any]) -> str:
    lines = [
        "# MATDOG G4 → G7 same-new-geometry oracle",
        "",
        f"status: **{comparison['status']}**",
        f"G4 content SHA256: `{comparison['g4_content_sha256']}`",
        f"endpoints: {comparison['endpoint_count']}",
        f"geometric contacts: {comparison['geometric_contact_found_count']}",
        f"path obstructions: {comparison['path_obstruction_count']}",
        f"obstructions preceding contact: {comparison['path_obstruction_precedes_contact_count']}",
        "",
        "| Endpoint | G4 contact (deg) | G7 contact (deg) | abs delta (rad) | Path relation | Result |",
        "|---|---:|---:|---:|---|---|",
    ]
    for row in comparison["comparisons"]:
        g4_angle = row["g4_contact_angle_rad"]
        g7_angle = row["g7_contact_angle_rad"]
        lines.append(
            f"| {row['endpoint_id']} | "
            f"{math.degrees(g4_angle):+.6f} | {math.degrees(g7_angle):+.6f} | "
            f"{row['contact_abs_delta_rad']:.3e} | {row['g7_path_relation'] or '-'} | "
            f"{row['comparison_status']} |"
        )
    lines.extend(
        [
            "",
            "Expected semantic separation:",
            "",
            *[f"- {item}" for item in comparison["expected_semantic_differences"]],
            "",
            "No hardware evidence or safety policy was used to define V5 geometry.",
        ]
    )
    return "\n".join(lines) + "\n"
