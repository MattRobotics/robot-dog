#!/usr/bin/env python3
"""Human-readable report for a saved pure Geometry Compiler V5 profile."""

from __future__ import annotations

import math
import os
from pathlib import Path
import tempfile
from typing import Any

from matdog_geometry_profile_v5 import validate_pure_geometry_profile


def _degrees(value: float | None) -> str:
    return "-" if value is None else f"{math.degrees(value):+.6f}"


def render_geometry_report_v5(profile: dict[str, Any]) -> str:
    validate_pure_geometry_profile(profile)
    endpoints = profile["endpoint_searches"]
    contact_counts: dict[str, int] = {}
    path_counts: dict[str, int] = {}
    relation_counts: dict[str, int] = {}
    for endpoint in endpoints:
        contact = endpoint["geometric_contact"]
        contact_counts[contact["status"]] = contact_counts.get(contact["status"], 0) + 1
        path = endpoint.get("path_obstruction")
        if path is not None:
            path_counts[path["status"]] = path_counts.get(path["status"], 0) + 1
            if path.get("relation") is not None:
                relation = path["relation"]
                relation_counts[relation] = relation_counts.get(relation, 0) + 1

    lines = [
        "# MATDOG Geometry Compiler V5 — pure geometry report",
        "",
        f"schema: `{profile['schema_version']}`",
        f"semantic content SHA256: `{profile['semantic_content_sha256']}`",
        f"URDF: `{profile['provenance']['urdf']['relative_path']}`",
        f"URDF SHA256: `{profile['provenance']['urdf']['sha256']}`",
        f"selected actuated revolute joints: {profile['model']['selected_actuated_joint_count']}",
        f"topology-derived articulated branches: {len(profile['model']['articulated_branches'])}",
        "",
        "This artifact reports geometry only. Geometric contact, path obstruction, "
        "external empirical evidence and safety-policy acceptance are distinct facts.",
        "",
        "## Outcome summary",
        "",
    ]
    for status, count in sorted(contact_counts.items()):
        lines.append(f"- contact `{status}`: {count}")
    for status, count in sorted(path_counts.items()):
        lines.append(f"- path `{status}`: {count}")
    for relation, count in sorted(relation_counts.items()):
        lines.append(f"- obstruction relation `{relation}`: {count}")

    lines.extend(
        [
            "",
            "## Endpoint searches",
            "",
            "| Joint | Side | Declared deg | Contact status | Contact deg | Delta deg | Active pair | Path status | Path deg | Relation | Precedes contact |",
            "|---|---|---:|---|---:|---:|---|---|---:|---|---|",
        ]
    )
    for endpoint in endpoints:
        identity = endpoint["identity"]
        contact = endpoint["geometric_contact"]
        path = endpoint.get("path_obstruction")
        active_pair = " ↔ ".join(endpoint["active_revolute_pair"])
        lines.append(
            f"| {identity['joint_name']} | {identity['limit_side']} | "
            f"{_degrees(endpoint['declared_limit_rad'])} | {contact['status']} | "
            f"{_degrees(contact['angle_rad'])} | "
            f"{_degrees(contact['minus_declared_limit_rad'])} | {active_pair} | "
            f"{path['status'] if path is not None else '-'} | "
            f"{_degrees(path['angle_rad']) if path is not None else '-'} | "
            f"{path.get('relation') or '-' if path is not None else '-'} | "
            f"{path.get('precedes_geometric_contact') if path is not None else '-'} |"
        )

    lines.extend(
        [
            "",
            "## Topology-derived branches",
            "",
            "| Branch ID | Root motorId | Joint chain | Link set |",
            "|---|---:|---|---|",
        ]
    )
    for branch in profile["model"]["articulated_branches"]:
        lines.append(
            f"| {branch['branch_id']} | {branch['deterministic_sort_motor_id']} | "
            f"{' → '.join(branch['joint_names'])} | {', '.join(branch['link_names'])} |"
        )

    lines.extend(
        [
            "",
            "## Collision mesh provenance",
            "",
            "| Link | URDF collision filename | SHA256 | Triangles | Scale | Origin xyz | Origin rpy |",
            "|---|---|---|---:|---|---|---|",
        ]
    )
    for link, mesh in profile["provenance"]["collision_meshes"].items():
        lines.append(
            f"| {link} | `{mesh['stl_relative_path']}` | `{mesh['sha256']}` | "
            f"{mesh['triangle_count']} | `{mesh['scale_xyz']}` | "
            f"`{mesh['origin_xyz']}` | `{mesh['origin_rpy']}` |"
        )

    lines.extend(["", "## Geometry UNKNOWNs", ""])
    unknowns = profile.get("geometry_unknowns", [])
    if unknowns:
        for unknown in unknowns:
            lines.append(f"- {unknown}")
    else:
        lines.append("(none recorded by this run)")

    lines.extend(
        [
            "",
            "No Station, serial, servo, EEPROM or other hardware access occurred.",
            "No norma-core file was modified. No merge was performed.",
            "",
        ]
    )
    return "\n".join(lines)


def write_geometry_report_v5(text: str, path: Path) -> None:
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=target.parent,
        prefix=f".{target.name}.",
        suffix=".tmp",
        text=True,
    )
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(text)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary_name, target)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise
