#!/usr/bin/env python3
"""Separate, read-only MATDOG geometry ↔ hardware evidence reconciler.

This layer consumes a saved pure-geometry V5 profile and an immutable external
evidence dataset. It does not import or call the geometry scene, contact search,
path planner or safety policy and never adjusts their outputs.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from typing import Any


GEOMETRY_SCHEMA = "matdog.calibration_geometry_profile.v5"
EVIDENCE_SCHEMA = "matdog.hardware_endpoint_evidence.v1"
RECONCILIATION_SCHEMA = "matdog.geometry_hardware_reconciliation.v1"
DEFAULT_AGREEMENT_THRESHOLD_DEG = 2.0


class HardwareReconciliationError(RuntimeError):
    """Invalid immutable evidence or incompatible saved geometry profile."""


def _sha256_file(path: Path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _canonical_content_hash(value: dict[str, Any], excluded: set[str]) -> str:
    content = {key: item for key, item in value.items() if key not in excluded}
    canonical = json.dumps(content, sort_keys=True, separators=(",", ":"), ensure_ascii=False)
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def _validate_geometry_profile(profile: dict[str, Any]) -> None:
    if profile.get("schema_version") != GEOMETRY_SCHEMA:
        raise HardwareReconciliationError(
            f"expected {GEOMETRY_SCHEMA}, got {profile.get('schema_version')!r}"
        )
    stored = profile.get("semantic_content_sha256")
    recomputed = _canonical_content_hash(
        profile,
        {"generation_metadata", "semantic_content_sha256"},
    )
    if stored != recomputed:
        raise HardwareReconciliationError(
            f"geometry semantic hash mismatch: stored={stored!r}, recomputed={recomputed!r}"
        )


def load_hardware_evidence(path: Path) -> tuple[dict[str, Any], str]:
    evidence_path = Path(path)
    evidence = json.loads(evidence_path.read_text(encoding="utf-8"))
    if evidence.get("schema_version") != EVIDENCE_SCHEMA:
        raise HardwareReconciliationError(
            f"expected {EVIDENCE_SCHEMA}, got {evidence.get('schema_version')!r}"
        )
    if evidence.get("status") != "IMMUTABLE_EXTERNAL_EVIDENCE":
        raise HardwareReconciliationError("hardware dataset is not marked immutable")
    if evidence.get("scope", {}).get("mirroring_to_other_assemblies") is not False:
        raise HardwareReconciliationError("hardware dataset must explicitly forbid mirroring")

    rows = evidence.get("endpoints")
    if not isinstance(rows, list) or len(rows) != 6:
        raise HardwareReconciliationError(f"LF V25 evidence must contain six endpoints, got {rows!r}")
    identities = [(row.get("joint_name"), row.get("limit_side")) for row in rows]
    if len(set(identities)) != 6:
        raise HardwareReconciliationError("hardware evidence identities are not unique")
    for row in rows:
        angle = float(row["hardware_contact_angle_rad"])
        approved_deg = float(row["approved_contact_angle_deg"])
        if not math.isfinite(angle):
            raise HardwareReconciliationError(f"non-finite hardware angle in {row}")
        if abs(math.degrees(angle) - approved_deg) > 0.0006:
            raise HardwareReconciliationError(
                f"{row['evidence_id']}: radian value does not round to approved degree value"
            )
    return evidence, _sha256_file(evidence_path)


def _endpoint_index(profile: dict[str, Any]) -> dict[tuple[str, str], dict[str, Any]]:
    result: dict[tuple[str, str], dict[str, Any]] = {}
    for endpoint in profile.get("endpoint_searches", []):
        identity = endpoint["identity"]
        key = (identity["joint_name"], identity["limit_side"])
        if key in result:
            raise HardwareReconciliationError(f"duplicate geometry endpoint {key}")
        result[key] = endpoint
    if len(result) != 24:
        raise HardwareReconciliationError(
            f"geometry profile must contain 24 endpoints, found {len(result)}"
        )
    return result


def reconciliation_content_sha256(report: dict[str, Any]) -> str:
    return _canonical_content_hash(
        report,
        {"generation_metadata", "reconciliation_content_sha256"},
    )


def reconcile_geometry_with_hardware(
    geometry_profile: dict[str, Any],
    evidence: dict[str, Any],
    *,
    evidence_file_sha256: str,
    geometry_profile_file_sha256: str | None = None,
    agreement_threshold_deg: float = DEFAULT_AGREEMENT_THRESHOLD_DEG,
) -> dict[str, Any]:
    if not (math.isfinite(agreement_threshold_deg) and agreement_threshold_deg >= 0.0):
        raise HardwareReconciliationError(
            f"invalid agreement threshold: {agreement_threshold_deg}"
        )
    geometry_snapshot = deepcopy(geometry_profile)
    _validate_geometry_profile(geometry_profile)
    if evidence.get("schema_version") != EVIDENCE_SCHEMA:
        raise HardwareReconciliationError("invalid evidence schema")

    endpoints = _endpoint_index(geometry_profile)
    rows = []
    for hardware in evidence["endpoints"]:
        key = (hardware["joint_name"], hardware["limit_side"])
        geometry = endpoints.get(key)
        if geometry is None:
            raise HardwareReconciliationError(f"hardware endpoint absent from geometry: {key}")
        contact = geometry["geometric_contact"]
        geometry_angle = contact.get("angle_rad")
        hardware_angle = float(hardware["hardware_contact_angle_rad"])
        declared_angle = float(geometry["declared_limit_rad"])

        if geometry_angle is None or contact.get("status") != "GEOMETRIC_CONTACT_FOUND":
            geometry_vs_hardware_status = "NO_GEOMETRIC_CONTACT"
            delta_rad = None
            delta_deg = None
        else:
            delta_rad = float(geometry_angle) - hardware_angle
            delta_deg = math.degrees(delta_rad)
            geometry_vs_hardware_status = (
                "AGREES"
                if abs(delta_deg) <= agreement_threshold_deg
                else "DISAGREES"
            )

        hardware_minus_declared_rad = hardware_angle - declared_angle
        hardware_minus_declared_deg = math.degrees(hardware_minus_declared_rad)
        hardware_vs_declared_status = (
            "AGREES"
            if abs(hardware_minus_declared_deg) <= agreement_threshold_deg
            else "DISAGREES"
        )
        path = geometry.get("path_obstruction")
        rows.append(
            {
                "evidence_id": hardware["evidence_id"],
                "identity": {
                    "joint_name": key[0],
                    "limit_side": key[1],
                },
                "declared_limit_rad": declared_angle,
                "geometric_contact_status": contact.get("status"),
                "geometric_contact_angle_rad": geometry_angle,
                "geometric_contact_link_pair": contact.get("link_pair"),
                "hardware_contact_angle_rad": hardware_angle,
                "geometry_minus_hardware_rad": delta_rad,
                "geometry_minus_hardware_deg": delta_deg,
                "geometry_vs_hardware_status": geometry_vs_hardware_status,
                "hardware_minus_declared_rad": hardware_minus_declared_rad,
                "hardware_minus_declared_deg": hardware_minus_declared_deg,
                "hardware_vs_declared_status": hardware_vs_declared_status,
                "path_obstruction_context": (
                    {
                        "status": path.get("status"),
                        "angle_rad": path.get("angle_rad"),
                        "link_pair": path.get("link_pair"),
                        "relation": path.get("relation"),
                        "precedes_geometric_contact": path.get("precedes_geometric_contact"),
                    }
                    if path is not None
                    else None
                ),
                "corrective_action": (
                    "NONE; preserve geometry and record agreement"
                    if geometry_vs_hardware_status == "AGREES"
                    else "NONE AUTOMATIC; preserve geometry, disagreement cause remains an engineering UNKNOWN"
                ),
            }
        )

    counts = {
        status: sum(row["geometry_vs_hardware_status"] == status for row in rows)
        for status in ("AGREES", "DISAGREES", "NO_GEOMETRIC_CONTACT")
    }
    source_sha = _sha256_file(Path(__file__))
    report: dict[str, Any] = {
        "schema_version": RECONCILIATION_SCHEMA,
        "generation_metadata": {
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        },
        "geometry_profile": {
            "schema_version": geometry_profile["schema_version"],
            "semantic_content_sha256": geometry_profile["semantic_content_sha256"],
            "file_sha256": geometry_profile_file_sha256,
        },
        "hardware_evidence": {
            "schema_version": evidence["schema_version"],
            "dataset_id": evidence["dataset_id"],
            "file_sha256": evidence_file_sha256,
            "scope": evidence["scope"],
        },
        "reconciler": {
            "source_file": Path(__file__).name,
            "source_sha256": source_sha,
            "agreement_threshold_deg": agreement_threshold_deg,
        },
        "summary": {
            "hardware_evidence_endpoint_count": len(rows),
            "geometry_only_endpoint_count": len(endpoints) - len(rows),
            "geometry_vs_hardware_counts": counts,
            "geometry_modified": False,
        },
        "reconciliations": rows,
        "constraints": [
            "LF evidence is not mirrored to any other assembly.",
            "Hardware comparison does not alter geometry, URDF, meshes, search or thresholds.",
            "Path obstruction context does not erase geometric endpoint contact.",
        ],
    }
    report["reconciliation_content_sha256"] = reconciliation_content_sha256(report)
    if geometry_profile != geometry_snapshot:
        raise HardwareReconciliationError("internal error: geometry input was mutated")
    return report


def render_hardware_reconciliation_report(report: dict[str, Any]) -> str:
    summary = report["summary"]
    counts = summary["geometry_vs_hardware_counts"]
    lines = [
        "# MATDOG LF V25 — separate Hardware Reconciliation",
        "",
        f"schema: `{report['schema_version']}`",
        f"geometry semantic SHA256: `{report['geometry_profile']['semantic_content_sha256']}`",
        f"hardware evidence SHA256: `{report['hardware_evidence']['file_sha256']}`",
        f"reconciler source SHA256: `{report['reconciler']['source_sha256']}`",
        f"agreement threshold: {report['reconciler']['agreement_threshold_deg']:.3f} deg",
        "",
        f"evidence endpoints: {summary['hardware_evidence_endpoint_count']}",
        f"geometry-only endpoints: {summary['geometry_only_endpoint_count']}",
        f"AGREES: {counts['AGREES']}",
        f"DISAGREES: {counts['DISAGREES']}",
        f"NO_GEOMETRIC_CONTACT: {counts['NO_GEOMETRIC_CONTACT']}",
        "",
        "| Evidence | Joint | Side | Geometry deg | Hardware deg | Delta deg | Result | Path context |",
        "|---|---|---|---:|---:|---:|---|---|",
    ]
    for row in report["reconciliations"]:
        geometry_angle = row["geometric_contact_angle_rad"]
        hardware_angle = row["hardware_contact_angle_rad"]
        path = row["path_obstruction_context"]
        path_text = "-"
        if path is not None and path["status"] == "PATH_OBSTRUCTION":
            path_text = (
                f"{path['relation']} @ {math.degrees(path['angle_rad']):+.3f} deg; "
                f"precedes={path['precedes_geometric_contact']}"
            )
        lines.append(
            f"| {row['evidence_id']} | {row['identity']['joint_name']} | "
            f"{row['identity']['limit_side']} | "
            f"{math.degrees(geometry_angle):+.3f} | {math.degrees(hardware_angle):+.3f} | "
            f"{row['geometry_minus_hardware_deg']:+.3f} | "
            f"{row['geometry_vs_hardware_status']} | {path_text} |"
        )
    lines.extend(
        [
            "",
            "No geometry was changed. RF/RH/LH remain geometry-only predictions.",
            "No hardware was accessed during this offline reconciliation.",
            "",
        ]
    )
    return "\n".join(lines)


def _atomic_text(text: str, path: Path) -> None:
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


def write_hardware_reconciliation(report: dict[str, Any], json_path: Path) -> tuple[Path, Path]:
    json_path = Path(json_path)
    _atomic_text(
        json.dumps(report, indent=2, sort_keys=True, ensure_ascii=False) + "\n",
        json_path,
    )
    markdown_path = json_path.with_suffix(".md")
    _atomic_text(render_hardware_reconciliation_report(report), markdown_path)
    return json_path, markdown_path


def main() -> int:
    parser = argparse.ArgumentParser(description="Offline MATDOG Geometry V5 ↔ LF V25 reconciliation")
    parser.add_argument("--geometry-profile", type=Path, required=True)
    parser.add_argument("--hardware-evidence", type=Path, required=True)
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--agreement-threshold-deg", type=float, default=DEFAULT_AGREEMENT_THRESHOLD_DEG)
    args = parser.parse_args()

    geometry_profile = json.loads(args.geometry_profile.read_text(encoding="utf-8"))
    evidence, evidence_sha = load_hardware_evidence(args.hardware_evidence)
    report = reconcile_geometry_with_hardware(
        geometry_profile,
        evidence,
        evidence_file_sha256=evidence_sha,
        geometry_profile_file_sha256=_sha256_file(args.geometry_profile),
        agreement_threshold_deg=args.agreement_threshold_deg,
    )
    json_path, markdown_path = write_hardware_reconciliation(report, args.output_json)
    print(f"geometry unchanged: {report['summary']['geometry_modified'] is False}")
    print(f"AGREES: {report['summary']['geometry_vs_hardware_counts']['AGREES']}")
    print(f"DISAGREES: {report['summary']['geometry_vs_hardware_counts']['DISAGREES']}")
    print(f"JSON: {json_path}")
    print(f"report: {markdown_path}")
    print("NO HARDWARE ACCESSED. NO GEOMETRY MODIFIED. NO MERGE PERFORMED.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
