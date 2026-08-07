#!/usr/bin/env python3
"""
MATDOG — Geometry Compiler human-readable report generator.

Renders the machine-readable geometry profile (matdog_geometry_profile)
into the Markdown checkpoint format required by Phase 1 section 15:
24-endpoint table, LF/RF and RH/LH and FRONT/HIND comparisons,
prerequisite/parking tables, minimum path clearances, hashes,
MODEL_LIMIT_MISMATCH / unintended-collision lists, and the residual
UNKNOWN list.

Deliberately consumes only the profile dict (not the live dataclasses
used to build it), so a report can always be regenerated later from a
saved *_PROFILE.json alone.

Offline only; this module only writes a report file, it never touches
hardware, Station, norma-core or the canonical URDF.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any


MIRROR_PAIRS: tuple[tuple[str, str], ...] = (("lf", "rf"), ("rh", "lh"))


def _deg(rad: float | None) -> str:
    if rad is None:
        return "-"
    return f"{math.degrees(rad):+.3f} deg"

def _mm(value: float | None) -> str:
    if value is None:
        return "-"
    return f"{value * 1000.0:.4f} mm"


def _endpoint_row(record: dict[str, Any]) -> str:
    pair = record["contact_link_pair"]
    pair_text = "-" if pair is None else f"{pair[0]} <-> {pair[1]}"
    delta = _deg(record["delta_from_declared_rad"])
    path_pair = record.get("path_collision_link_pair")
    path_text = "-" if not path_pair else f"{path_pair[0]} <-> {path_pair[1]} @ {_deg(record.get('path_collision_angle_rad'))}"

    return (
        f"| {record['leg'].upper()} | {record['joint_group']} | {record['side']} "
        f"| {_deg(record['urdf_declared_limit_rad'])} | {_deg(record['mesh_predicted_contact_rad'])} "
        f"| {delta} | {pair_text} | {_mm(record['clearance_before_contact_m'])} "
        f"| **{record.get('contact_model_status', record['result_kind'])}** | {path_text} |"
    )


def _endpoint_table(endpoints: list[dict[str, Any]]) -> str:
    header = (
        "| Leg | Joint | Side | Declared URDF | Mesh predicted contact | Delta | Contact pair "
        "| Clearance before contact | Contact model status | Path collision (if any) |\n"
        "|---|---|---|---:|---:|---:|---|---:|---|---|\n"
    )
    rows = "\n".join(_endpoint_row(r) for r in sorted(endpoints, key=lambda r: r["endpoint_id"]))
    return header + rows


def _mirror_comparison(endpoints_by_id: dict[str, dict[str, Any]], leg_a: str, leg_b: str) -> str:
    lines = [
        f"### {leg_a.upper()} vs {leg_b.upper()}",
        "",
        f"| Joint | Side | {leg_a.upper()} contact | {leg_b.upper()} contact | Delta |",
        "|---|---|---:|---:|---:|",
    ]

    for group in ("hip", "upper_leg", "lower_leg"):
        for side in ("min", "max"):
            ra = endpoints_by_id.get(f"{leg_a}_{group}_{side}")
            rb = endpoints_by_id.get(f"{leg_b}_{group}_{side}")

            if ra is None or rb is None:
                continue

            ca = ra["mesh_predicted_contact_rad"]
            cb = rb["mesh_predicted_contact_rad"]
            delta = "-" if (ca is None or cb is None) else _deg(ca - cb)
            lines.append(
                f"| {group} | {side} | {_deg(ca)} ({ra.get('contact_model_status', ra['result_kind'])}) "
                f"| {_deg(cb)} ({rb.get('contact_model_status', rb['result_kind'])}) | {delta} |"
            )

    return "\n".join(lines)


def _front_hind_comparison(endpoints_by_id: dict[str, dict[str, Any]]) -> str:
    lines = ["### FRONT vs HIND", "", "| Joint | Side | FRONT (LF) | HIND (RH) | Note |", "|---|---|---:|---:|---|"]

    for group in ("hip", "upper_leg", "lower_leg"):
        for side in ("min", "max"):
            front = endpoints_by_id.get(f"lf_{group}_{side}")
            hind = endpoints_by_id.get(f"rh_{group}_{side}")

            if front is None or hind is None:
                continue

            fs = front.get("contact_model_status", front["result_kind"])
            hs = hind.get("contact_model_status", hind["result_kind"])
            note = "same status" if fs == hs else "DIFFERENT status"
            lines.append(
                f"| {group} | {side} | {_deg(front['mesh_predicted_contact_rad'])} ({fs}) "
                f"| {_deg(hind['mesh_predicted_contact_rad'])} ({hs}) | {note} |"
            )

    lines.append("")
    lines.append(
        "Front hip Z ~= 0.0465 m, hind hip Z ~= 0.0265 m (20 mm difference, verified "
        "numerically in tests/test_matdog_geometry_scene.py); this does not change detector "
        "physics but can change prerequisite/parking/path clearance, per canonical handoff "
        "section 4."
    )
    return "\n".join(lines)


def _parking_table(parking_plans: dict[str, dict[str, Any]]) -> str:
    lines = [
        "| Leg | Auxiliary parking | Parked leg | Angle | Reason |",
        "|---|---|---|---:|---|",
    ]

    for leg, plan in sorted(parking_plans.items()):
        status = "REQUIRED" if plan["auxiliary_parking_required"] else "NOT REQUIRED"
        angle = _deg(plan["parking_angle_rad"])
        lines.append(f"| {leg.upper()} | {status} | {plan['parked_leg'] or '-'} | {angle} | {plan['reason']} |")

    return "\n".join(lines)


def _lf_v25_reconciliation_table(rows: list[dict[str, Any]]) -> str:
    lines = [
        "| Endpoint | Declared URDF | Hardware contact | Compiler mesh contact | Compiler contact pair "
        "| hw vs URDF | mesh vs hw | Contact model status | Corrective action |",
        "|---|---:|---:|---:|---|---|---|---|---|",
    ]

    for r in rows:
        pair = r["compiler_contact_link_pair"]
        pair_text = "-" if pair is None else f"{pair[0]} <-> {pair[1]}"
        lines.append(
            f"| {r['endpoint_id']} | {_deg(r['urdf_declared_limit_rad'])} "
            f"| {_deg(r['hardware_contact_rad'])} "
            f"| {_deg(r['compiler_mesh_predicted_contact_rad'])} | {pair_text} "
            f"| {r['hardware_vs_urdf_status']} | {r['mesh_vs_hardware_status']} "
            f"| **{r['compiler_contact_model_status']}** "
            f"| {r['required_corrective_action']} |"
        )

    return "\n".join(lines)


def _clearance_table(parking_plans: dict[str, dict[str, Any]]) -> str:
    lines = ["| Leg | Sequence PASS | Minimum modelled clearance |", "|---|---|---:|"]

    for leg, plan in sorted(parking_plans.items()):
        lines.append(
            f"| {leg.upper()} | {plan['active_leg_sequence_passed']} "
            f"| {_mm(plan['active_leg_sequence_min_clearance_m'])} |"
        )

    return "\n".join(lines)


def render_report(profile: dict[str, Any]) -> str:
    endpoints: list[dict[str, Any]] = profile["endpoints"]
    parking_plans: dict[str, dict[str, Any]] = profile["parking_plans"]
    endpoints_by_id = {r["endpoint_id"]: r for r in endpoints}

    mismatches = [r for r in endpoints if r.get("contact_model_status") == "MODEL_LIMIT_MISMATCH"]
    path_collisions = [r for r in endpoints if r.get("contact_model_status") == "PATH_COLLISION_BEFORE_ENDPOINT"]
    model_incomplete = [r for r in endpoints if r.get("contact_model_status") == "MODEL_INCOMPLETE"]
    unintended_self = [r for r in endpoints if r.get("contact_model_status") == "UNINTENDED_SELF_COLLISION"]
    no_contact = [r for r in endpoints if r.get("contact_model_status") == "NO_MODELED_ENDSTOP"]
    lf_v25_rows: list[dict[str, Any]] = profile.get("lf_v25_hardware_reconciliation", [])

    lines: list[str] = []
    lines.append("# MATDOG Calibration Geometry Profile — Phase 1 report")
    lines.append("")
    lines.append(f"schema_version: `{profile['schema_version']}`")
    lines.append(f"generation_timestamp_utc: `{profile['generation_timestamp_utc']}`")
    lines.append(f"robot_dog_commit_sha: `{profile['robot_dog_commit_sha']}`")
    lines.append(f"robot_dog_working_tree_dirty: `{profile['robot_dog_working_tree_dirty']}`")
    lines.append(f"content_sha256: `{profile['content_sha256']}`")
    lines.append(f"urdf_sha256: `{profile['urdf']['sha256']}`")
    lines.append("")
    lines.append("HARDWARE NOT USED. NORMA-CORE NOT MODIFIED. NO COMMIT/PUSH/PR/MERGE.")
    lines.append("")

    lines.append("## LF V25 hardware reconciliation (read before the 24-endpoint table)")
    lines.append("")
    if lf_v25_rows:
        lines.append(_lf_v25_reconciliation_table(lf_v25_rows))
    else:
        lines.append("(no LF endpoints in this run -- table not applicable)")
    lines.append("")

    lines.append("## 24 endpoints")
    lines.append("")
    lines.append(_endpoint_table(endpoints))
    lines.append("")

    lines.append("## Mirror comparisons")
    lines.append("")
    for leg_a, leg_b in MIRROR_PAIRS:
        lines.append(_mirror_comparison(endpoints_by_id, leg_a, leg_b))
        lines.append("")

    lines.append(_front_hind_comparison(endpoints_by_id))
    lines.append("")

    lines.append("## Parking")
    lines.append("")
    lines.append(_parking_table(parking_plans))
    lines.append("")

    lines.append("## Minimum modelled clearance per leg sequence")
    lines.append("")
    lines.append(_clearance_table(parking_plans))
    lines.append("")

    lines.append("## MODEL_LIMIT_MISMATCH")
    lines.append("")
    if mismatches:
        for r in mismatches:
            lines.append(
                f"- {r['endpoint_id']}: declared {_deg(r['urdf_declared_limit_rad'])}, "
                f"mesh {_deg(r['mesh_predicted_contact_rad'])}, delta {_deg(r['delta_from_declared_rad'])}"
            )
    else:
        lines.append("(none)")
    lines.append("")

    lines.append("## PATH_COLLISION_BEFORE_ENDPOINT (cross-leg path obstructions)")
    lines.append("")
    if path_collisions:
        for r in path_collisions:
            pair = r.get("path_collision_link_pair") or ("?", "?")
            lines.append(
                f"- {r['endpoint_id']}: {pair[0]} <-> {pair[1]} at {_deg(r.get('path_collision_angle_rad'))} "
                f"-- {r.get('contact_model_status_reason', '')}"
            )
    else:
        lines.append("(none)")
    lines.append("")

    lines.append("## MODEL_INCOMPLETE_FOR_ENDPOINT_METROLOGY (hardware disagrees, real stop not in collision STL)")
    lines.append("")
    if model_incomplete:
        for r in model_incomplete:
            lines.append(f"- {r['endpoint_id']}: {r.get('contact_model_status_reason', '')}")
    else:
        lines.append("(none)")
    lines.append("")

    lines.append("## UNINTENDED_SELF_COLLISION")
    lines.append("")
    if unintended_self:
        for r in unintended_self:
            lines.append(f"- {r['endpoint_id']}: {r.get('contact_model_status_reason', '')}")
    else:
        lines.append("(none)")
    lines.append("")

    lines.append("## NO_MODELED_ENDSTOP (no mesh contact found in envelope)")
    lines.append("")
    if no_contact:
        for r in no_contact:
            envelope = r["numerical_search"]["analysis_envelope_rad"]
            lines.append(
                f"- {r['endpoint_id']}: declared {_deg(r['urdf_declared_limit_rad'])}, "
                f"envelope {_deg(envelope[0])}..{_deg(envelope[1])}"
            )
    else:
        lines.append("(none)")
    lines.append("")

    lines.append("## Unresolved assumptions / UNKNOWN")
    lines.append("")
    for note in profile["unresolved_assumptions"]:
        lines.append(f"- {note}")
    lines.append("")

    lines.append("## Collision mesh manifest")
    lines.append("")
    lines.append("| Link | SHA256 | Triangles | Degenerate dropped |")
    lines.append("|---|---|---:|---:|")
    for link, entry in sorted(profile["collision_mesh_manifest"].items()):
        lines.append(
            f"| {link} | `{entry['sha256']}` | {entry['triangle_count']} "
            f"| {entry['degenerate_triangles_dropped']} |"
        )
    lines.append("")

    return "\n".join(lines)


def write_report(text: str, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
