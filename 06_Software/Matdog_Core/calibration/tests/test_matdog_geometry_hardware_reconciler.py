#!/usr/bin/env python3
"""Offline G8 tests for the separate immutable Hardware Reconciler."""

from __future__ import annotations

import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
import unittest


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = CALIBRATION_DIR.parents[2]
EVIDENCE_PATH = CALIBRATION_DIR / "MATDOG_LF_V25_HARDWARE_EVIDENCE_2026-08-04.json"
PROFILE_PATH = (
    REPO_ROOT
    / "09_Logs/Validation_Reports/Geometry_Compiler"
    / "2026-08-10_185433_MATDOG_GEOMETRY_V5_G7_PURE_PROFILE.json"
)

if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

from matdog_geometry_hardware_reconciler import (  # noqa: E402
    load_hardware_evidence,
    reconcile_geometry_with_hardware,
    reconciliation_content_sha256,
    render_hardware_reconciliation_report,
)


class TestImmutableLfV25Evidence(unittest.TestCase):
    def test_dataset_file_hash_and_six_approved_values(self):
        evidence, digest = load_hardware_evidence(EVIDENCE_PATH)
        self.assertEqual(
            digest,
            "6eae3201a00b5299550028d5b4e1e73d67520deccf5a85e548f3b07b1777cab4",
        )
        self.assertEqual(
            [row["approved_contact_angle_deg"] for row in evidence["endpoints"]],
            [-42.803, 39.375, -53.525, 122.607, -91.846, 34.277],
        )
        self.assertFalse(evidence["scope"]["mirroring_to_other_assemblies"])
        self.assertFalse(evidence["scope"]["geometry_authority"])


class TestSeparateReconciliation(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.profile = json.loads(PROFILE_PATH.read_text(encoding="utf-8"))
        cls.evidence, cls.evidence_sha = load_hardware_evidence(EVIDENCE_PATH)

    def reconcile(self):
        return reconcile_geometry_with_hardware(
            self.profile,
            self.evidence,
            evidence_file_sha256=self.evidence_sha,
            geometry_profile_file_sha256=hashlib.sha256(PROFILE_PATH.read_bytes()).hexdigest(),
        )

    def test_three_agree_three_disagree_and_eighteen_geometry_only(self):
        report = self.reconcile()
        self.assertEqual(
            report["summary"]["geometry_vs_hardware_counts"],
            {"AGREES": 3, "DISAGREES": 3, "NO_GEOMETRIC_CONTACT": 0},
        )
        self.assertEqual(report["summary"]["hardware_evidence_endpoint_count"], 6)
        self.assertEqual(report["summary"]["geometry_only_endpoint_count"], 18)
        self.assertFalse(report["summary"]["geometry_modified"])

    def test_reconciler_does_not_mutate_geometry_profile(self):
        snapshot = deepcopy(self.profile)
        report = self.reconcile()
        self.assertEqual(self.profile, snapshot)
        self.assertEqual(
            report["geometry_profile"]["semantic_content_sha256"],
            self.profile["semantic_content_sha256"],
        )
        self.assertEqual(
            report["reconciliation_content_sha256"],
            reconciliation_content_sha256(report),
        )

    def test_path_obstruction_is_context_not_hardware_override(self):
        report = self.reconcile()
        row = next(item for item in report["reconciliations"] if item["evidence_id"] == "lf_hip_min")
        self.assertEqual(row["geometry_vs_hardware_status"], "DISAGREES")
        self.assertEqual(row["path_obstruction_context"]["status"], "PATH_OBSTRUCTION")
        self.assertEqual(row["path_obstruction_context"]["relation"], "body_vs_branch")
        self.assertTrue(row["path_obstruction_context"]["precedes_geometric_contact"])
        self.assertEqual(row["geometric_contact_status"], "GEOMETRIC_CONTACT_FOUND")

    def test_report_renders_without_mirroring_other_assemblies(self):
        text = render_hardware_reconciliation_report(self.reconcile())
        self.assertIn("AGREES: 3", text)
        self.assertIn("DISAGREES: 3", text)
        self.assertIn("RF/RH/LH remain geometry-only predictions", text)
        self.assertIn("No geometry was changed", text)

    def test_reconciler_has_no_geometry_engine_imports(self):
        path = CALIBRATION_DIR / "matdog_geometry_hardware_reconciler.py"
        tree = ast.parse(path.read_text(encoding="utf-8"))
        imported = {
            node.module
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module is not None
        }
        self.assertTrue(
            {
                "matdog_geometry_scene_v5",
                "matdog_geometry_contact_search_v5",
                "matdog_geometry_path_planner_v5",
                "matdog_geometry_safety_policy",
            }.isdisjoint(imported)
        )


if __name__ == "__main__":
    unittest.main()
