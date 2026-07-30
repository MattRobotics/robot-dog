#!/usr/bin/env python3
"""Mutation tests for the offline Milestone I foundation validator."""

from __future__ import annotations

import csv
import importlib.util
import shutil
import sys
import tempfile
import unittest
from pathlib import Path


sys.dont_write_bytecode = True
REPO_ROOT = Path(__file__).resolve().parents[2]
VALIDATOR_PATH = REPO_ROOT / "06_Software/Matdog_Core/milestone_i/validate_foundation.py"
SPEC = importlib.util.spec_from_file_location("matdog_foundation_validator", VALIDATOR_PATH)
assert SPEC and SPEC.loader
validator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validator)


class FoundationValidatorTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory(prefix="matdog-milestone-i-")
        self.root = Path(self.temporary.name)
        source_registries = REPO_ROOT / "06_Software/Matdog_Core/milestone_i/registries"
        target_registries = self.root / "06_Software/Matdog_Core/milestone_i/registries"
        shutil.copytree(source_registries, target_registries)

        with (source_registries / "source_manifest.csv").open(
            "r", encoding="utf-8", newline=""
        ) as handle:
            sources = list(csv.DictReader(handle))
        for row in sources:
            if row["repository"] != validator.ROBOT_REPOSITORY:
                continue
            source = REPO_ROOT / row["path"]
            target = self.root / row["path"]
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)

    def tearDown(self) -> None:
        self.temporary.cleanup()

    def registry(self, name: str) -> Path:
        return self.root / "06_Software/Matdog_Core/milestone_i/registries" / name

    def mutate(self, name: str, mutation) -> None:
        path = self.registry(name)
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            headers = list(reader.fieldnames or [])
            rows = list(reader)
        mutation(rows)
        with path.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=headers, lineterminator="\n")
            writer.writeheader()
            writer.writerows(rows)

    def errors(self) -> list[str]:
        return validator.validate(self.root)

    def assert_error_contains(self, text: str) -> None:
        errors = self.errors()
        self.assertTrue(any(text in error for error in errors), errors)

    def test_baseline_valid(self) -> None:
        self.assertEqual(self.errors(), [])

    def test_joint_missing(self) -> None:
        self.mutate("joint_registry.csv", lambda rows: rows.pop())
        self.assert_error_contains("expected exact canonical 12")

    def test_servo_duplicate(self) -> None:
        def change(rows):
            rows[1]["servo_id"] = rows[0]["servo_id"]

        self.mutate("servo_mapping_registry.csv", change)
        self.assert_error_contains("exact unique canonical set")

    def test_direction_invalid(self) -> None:
        self.mutate("servo_mapping_registry.csv", lambda rows: rows[0].update(direction="0"))
        self.assert_error_contains("direction must be -1 or 1")

    def test_classification_invalid(self) -> None:
        self.mutate(
            "source_claim_registry.csv",
            lambda rows: rows[0].update(classification="CERTAIN"),
        )
        self.assert_error_contains("invalid classification")

    def test_confidence_invalid(self) -> None:
        self.mutate(
            "source_claim_registry.csv",
            lambda rows: rows[0].update(confidence="ABSOLUTE"),
        )
        self.assert_error_contains("invalid confidence")

    def test_source_ref_missing(self) -> None:
        self.mutate("source_manifest.csv", lambda rows: rows[0].update(ref=""))
        self.assert_error_contains("repository/ref/path must be non-empty")

    def test_unknown_improperly_promoted(self) -> None:
        def change(rows):
            next(row for row in rows if row["claim_id"] == "C-UNKNOWN-SAFE")[
                "classification"
            ] = "MATDOG_VERIFIED"

        self.mutate("source_claim_registry.csv", change)
        self.assert_error_contains("frozen UNKNOWN was improperly promoted")

    def test_mechanical_contact_used_as_safe_limit(self) -> None:
        def change(rows):
            next(row for row in rows if row["limit_type"] == "mechanical_contact")[
                "is_operational_safe_limit"
            ] = "true"

        self.mutate("limit_registry.csv", change)
        self.assert_error_contains("mechanical contact used as operational safe limit")

    def test_xgo_physical_constant_attributed_to_matdog(self) -> None:
        def change(rows):
            next(row for row in rows if row["claim_id"] == "C-XGO-ARCH")[
                "classification"
            ] = "MATDOG_VERIFIED"

        self.mutate("source_claim_registry.csv", change)
        self.assert_error_contains("XGo source promoted as MATDOG fact")

    def test_joint_urdf_nonexistent(self) -> None:
        self.mutate(
            "joint_registry.csv",
            lambda rows: rows[0].update(urdf_joint_name="lf_missing_joint"),
        )
        self.assert_error_contains("does not exist in URDF")

    def test_conflict_not_registered(self) -> None:
        def change(rows):
            rows[:] = [row for row in rows if row["conflict_id"] != "CF-CONTACT-SAFE"]

        self.mutate("source_conflict_registry.csv", change)
        self.assert_error_contains("conflict with C-UNKNOWN-SAFE is not registered")

    def test_decision_missing_required_evidence(self) -> None:
        self.mutate("decision_registry.csv", lambda rows: rows[0].update(required_evidence=""))
        self.assert_error_contains("required field required_evidence is empty")

    def test_local_checksum_wrong(self) -> None:
        def change(rows):
            next(row for row in rows if row["source_id"] == "R-URDF")["sha256"] = "0" * 64

        self.mutate("source_manifest.csv", change)
        self.assert_error_contains("local checksum mismatch")


if __name__ == "__main__":
    unittest.main()
