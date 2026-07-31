#!/usr/bin/env python3
"""Mutation tests for the offline Milestone I foundation validator."""

from __future__ import annotations

import contextlib
import csv
import importlib.util
import io
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
        source_milestone = REPO_ROOT / "06_Software/Matdog_Core/milestone_i"
        target_milestone = self.root / "06_Software/Matdog_Core/milestone_i"
        shutil.copytree(source_milestone / "registries", target_milestone / "registries")
        shutil.copy2(
            source_milestone / "foundation_expectations.json",
            target_milestone / "foundation_expectations.json",
        )

        with (source_milestone / "registries/source_manifest.csv").open(
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

        for relative in (
            "01_Docs/02_Architecture/Milestone_I/"
            "MATDOG_MILESTONE_I_FOUNDATION_ACCEPTANCE.md",
            "09_Logs/Development_Log/2026-07-30_MILESTONE_I_FOUNDATION_HANDOFF.md",
        ):
            source = REPO_ROOT / relative
            target = self.root / relative
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

    def mutate_document(self, relative: str, old: str, new: str) -> None:
        path = self.root / relative
        text = path.read_text(encoding="utf-8")
        self.assertIn(old, text)
        path.write_text(text.replace(old, new, 1), encoding="utf-8")

    def errors(self) -> list[str]:
        return validator.validate(self.root)

    def exit_code(self) -> int:
        output = io.StringIO()
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
            return validator.main(["--check", "--root", str(self.root)])

    def assert_error_contains(self, text: str) -> None:
        errors = self.errors()
        self.assertTrue(any(text in error for error in errors), errors)
        self.assertEqual(self.exit_code(), 1)

    # Original 14 tests retained.

    def test_baseline_valid(self) -> None:
        self.assertEqual(self.errors(), [])
        self.assertEqual(self.exit_code(), 0)

    def test_joint_missing(self) -> None:
        self.mutate("joint_registry.csv", lambda rows: rows.pop())
        self.assert_error_contains("joints: expected ID set mismatch")

    def test_servo_duplicate(self) -> None:
        def change(rows):
            rows[1]["servo_id"] = rows[0]["servo_id"]

        self.mutate("servo_mapping_registry.csv", change)
        self.assert_error_contains("disagrees with joint registry")

    def test_direction_invalid(self) -> None:
        self.mutate("servo_mapping_registry.csv", lambda rows: rows[0].update(direction="0"))
        self.assert_error_contains("direction disagrees with joint registry")

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
        self.assert_error_contains("required field ref is empty")

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
        self.assert_error_contains("conflicts: expected ID set mismatch")

    def test_decision_missing_required_evidence(self) -> None:
        self.mutate("decision_registry.csv", lambda rows: rows[0].update(required_evidence=""))
        self.assert_error_contains("required field required_evidence is empty")

    def test_local_checksum_wrong(self) -> None:
        def change(rows):
            next(row for row in rows if row["source_id"] == "R-URDF")["sha256"] = "0" * 64

        self.mutate("source_manifest.csv", change)
        self.assert_error_contains("local checksum mismatch")

    # Eight independently reproduced false PASS cases.

    def test_joint_origin_canonical_mismatch(self) -> None:
        self.mutate("joint_registry.csv", lambda rows: rows[0].update(origin_xyz_m="9 9 9"))
        self.assert_error_contains("origin xyz mismatch")

    def test_joint_effort_canonical_mismatch(self) -> None:
        self.mutate("joint_registry.csv", lambda rows: rows[0].update(effort_nm="999"))
        self.assert_error_contains("effort mismatch")

    def test_frame_origin_canonical_mismatch(self) -> None:
        self.mutate("frame_registry.csv", lambda rows: rows[1].update(origin_xyz_m="9 9 9"))
        self.assert_error_contains("origin xyz mismatch")

    def test_profile_urdf_limit_tick_canonical_mismatch(self) -> None:
        self.mutate(
            "calibration_registry.csv",
            lambda rows: rows[0].update(urdf_limit_tick="1452"),
        )
        self.assert_error_contains("urdf_limit_tick mismatch")

    def test_claim_removed(self) -> None:
        def change(rows):
            rows[:] = [row for row in rows if row["claim_id"] != "C-MODEL-ORDER"]

        self.mutate("source_claim_registry.csv", change)
        self.assert_error_contains("claims: expected ID set mismatch")

    def test_unresolved_removed(self) -> None:
        def change(rows):
            rows[:] = [row for row in rows if row["unresolved_id"] != "U-COLLISION"]

        self.mutate("unresolved_registry.csv", change)
        self.assert_error_contains("unresolved: expected ID set mismatch")

    def test_conflict_status_changed(self) -> None:
        def change(rows):
            next(row for row in rows if row["conflict_id"] == "CF-VISUAL-DIGITAL")[
                "status"
            ] = "OPEN"

        self.mutate("source_conflict_registry.csv", change)
        self.assert_error_contains("canonical status mismatch")

    def test_source_authority_empty(self) -> None:
        self.mutate("source_manifest.csv", lambda rows: rows[0].update(authority=""))
        self.assert_error_contains("required field authority is empty")

    # Additional provenance, status, acceptance and handoff mutations.

    def test_provenance_locator_nonexistent(self) -> None:
        def change(rows):
            next(row for row in rows if row["claim_id"] == "C-FRAME-BASE")[
                "source_locator"
            ] = "nonexistent locator"

        self.mutate("source_claim_registry.csv", change)
        self.assert_error_contains("critical provenance source_locator mismatch")

    def test_m11_parse_status_missing(self) -> None:
        def change(rows):
            next(row for row in rows if row["source_id"] == "R-DIR-M11")[
                "parse_status"
            ] = ""

        self.mutate("source_manifest.csv", change)
        self.assert_error_contains("expected TEXT_ONLY_NONPARSEABLE")

    def test_claim_source_incompatible_with_classification(self) -> None:
        def change(rows):
            row = next(row for row in rows if row["claim_id"] == "C-FRAME-BASE")
            row.update(
                source_repository=validator.NORMA_REPOSITORY,
                source_ref=validator.NORMA_MAIN,
                source_path="software/drivers/st3215/src/auto_calibrate/so101.rs",
                source_locator="send_eeprom_write_verified",
            )

        self.mutate("source_claim_registry.csv", change)
        self.assert_error_contains("is incompatible with source class")

    def test_planned_frame_treated_as_materialized(self) -> None:
        def change(rows):
            next(row for row in rows if row["frame_id"] == "world").update(
                status="materialized",
                frame_type="root",
            )

        self.mutate("frame_registry.csv", change)
        self.assert_error_contains("canonical status mismatch")

    def test_urdf_motor_direction_used_as_hardware_without_conflict(self) -> None:
        def change_joint(rows):
            next(row for row in rows if row["joint_id"] == "J-LF-HIP")[
                "encoder_to_q_direction"
            ] = "1"

        def change_servo(rows):
            next(row for row in rows if row["mapping_id"] == "S-M13")["direction"] = "1"

        def change_conflict(rows):
            rows[:] = [
                row for row in rows if row["conflict_id"] != "CF-URDF-DIRECTION"
            ]

        self.mutate("joint_registry.csv", change_joint)
        self.mutate("servo_mapping_registry.csv", change_servo)
        self.mutate("source_conflict_registry.csv", change_conflict)
        self.assert_error_contains("canonical encoder_to_q_direction mismatch")

    def test_acceptance_count_divergent(self) -> None:
        self.mutate_document(
            "01_Docs/02_Architecture/Milestone_I/"
            "MATDOG_MILESTONE_I_FOUNDATION_ACCEPTANCE.md",
            '"claims": 51',
            '"claims": 50',
        )
        self.assert_error_contains("acceptance: metrics diverge")

    def test_handoff_count_divergent(self) -> None:
        self.mutate_document(
            "09_Logs/Development_Log/2026-07-30_MILESTONE_I_FOUNDATION_HANDOFF.md",
            '"claims": 51',
            '"claims": 50',
        )
        self.assert_error_contains("handoff: metrics diverge")


if __name__ == "__main__":
    unittest.main()
