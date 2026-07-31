#!/usr/bin/env python3
"""Mutation tests for the offline Milestone I foundation validator."""

from __future__ import annotations

import contextlib
import csv
import importlib.util
import io
import json
import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path


sys.dont_write_bytecode = True
REPO_ROOT = Path(__file__).resolve().parents[2]
ROBOT_DOG_REPO = Path(os.environ.get("MATDOG_ROBOT_DOG_REPO", REPO_ROOT))
NORMACORE_REPO = Path(
    os.environ.get("MATDOG_NORMACORE_REPO", REPO_ROOT.parents[2] / "norma-core")
)
XGOLITE_REPO = Path(
    os.environ.get(
        "MATDOG_XGOLITE_REPO",
        REPO_ROOT.parents[2]
        / "robotics-reverse/worktrees/xgolite-main-checkpoint",
    )
)
VALIDATOR_PATH = REPO_ROOT / "06_Software/Matdog_Core/milestone_i/validate_foundation.py"
SPEC = importlib.util.spec_from_file_location("matdog_foundation_validator", VALIDATOR_PATH)
assert SPEC and SPEC.loader
validator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = validator
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

    def mutate_expectations(self, mutation) -> None:
        path = self.root / "06_Software/Matdog_Core/milestone_i/foundation_expectations.json"
        data = json.loads(path.read_text(encoding="utf-8"))
        mutation(data)
        path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")

    def repository_paths(self, **overrides) -> dict[str, Path]:
        paths = {
            "robot_dog_repo": ROBOT_DOG_REPO,
            "normacore_repo": NORMACORE_REPO,
            "xgolite_repo": XGOLITE_REPO,
        }
        paths.update(overrides)
        return paths

    def errors(self, **overrides) -> list[str]:
        return validator.validate(self.root, **self.repository_paths(**overrides))

    def exit_code(self, **overrides) -> int:
        output = io.StringIO()
        paths = self.repository_paths(**overrides)
        with contextlib.redirect_stdout(output), contextlib.redirect_stderr(output):
            return validator.main(
                [
                    "--check",
                    "--root",
                    str(self.root),
                    "--robot-dog-repo",
                    str(paths["robot_dog_repo"]),
                    "--normacore-repo",
                    str(paths["normacore_repo"]),
                    "--xgolite-repo",
                    str(paths["xgolite_repo"]),
                ]
            )

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
        self.assert_error_contains("pinned blob checksum mismatch")

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
        self.assert_error_contains("pinned-URDF/code status mismatch")

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
        self.assert_error_contains("pinned robot calibration encoder_to_q_direction mismatch")

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

    # Fourteen final-remediation cases: the nine reproduced false PASS cases
    # plus five additional fail-closed Git/enum/locator mutations.

    def test_coordinated_joint_expectation_and_registries(self) -> None:
        self.mutate_expectations(
            lambda data: data["joint_directions"]["J-LF-HIP"].update(
                encoder_to_q=1
            )
        )

        def change_joint(rows):
            next(row for row in rows if row["joint_id"] == "J-LF-HIP")[
                "encoder_to_q_direction"
            ] = "1"

        def change_servo(rows):
            next(row for row in rows if row["mapping_id"] == "S-M13")[
                "direction"
            ] = "1"

        def change_profiles(rows):
            minimum = next(
                row for row in rows if row["profile_id"] == "LF_HIP_M13_MIN"
            )
            maximum = next(
                row for row in rows if row["profile_id"] == "LF_HIP_M13_MAX"
            )
            minimum.update(
                probe_sign="-1",
                urdf_limit_tick="1536",
                guard_tick="1472",
                baseline_target_tick="1984",
            )
            maximum.update(
                probe_sign="1",
                urdf_limit_tick="2560",
                guard_tick="2624",
                baseline_target_tick="2112",
            )

        self.mutate("joint_registry.csv", change_joint)
        self.mutate("servo_mapping_registry.csv", change_servo)
        self.mutate("calibration_registry.csv", change_profiles)
        self.assert_error_contains(
            "pinned robot calibration encoder_to_q_direction mismatch"
        )

    def test_coordinated_frame_expectation_and_registry(self) -> None:
        self.mutate_expectations(
            lambda data: data["frame_statuses"].update(world="decision-required")
        )

        def change(rows):
            next(row for row in rows if row["frame_id"] == "world")[
                "status"
            ] = "decision-required"

        self.mutate("frame_registry.csv", change)
        self.assert_error_contains("pinned-URDF/code status mismatch")

    def test_coordinated_profile_expectation_calibration_and_limit(self) -> None:
        def change_expectation(data):
            values = data["hardware_validated_profiles"]["LF_UPPER_M12_MIN"]
            values.update(
                coarse_contact_tick=1444,
                fine_contact_tick=1444,
                measured_contact_tick=1444,
            )

        def change_profile(rows):
            next(
                row for row in rows if row["profile_id"] == "LF_UPPER_M12_MIN"
            ).update(
                coarse_contact_tick="1444",
                fine_contact_tick="1444",
                measured_contact_tick="1444",
            )

        def change_limit(rows):
            next(row for row in rows if row["limit_id"] == "L-CONTACT-M12-MIN")[
                "value"
            ] = "1444"

        self.mutate_expectations(change_expectation)
        self.mutate("calibration_registry.csv", change_profile)
        self.mutate("limit_registry.csv", change_limit)
        self.assert_error_contains("differ from pinned checkpoint")

    def test_locator_existing_but_wrong_segment(self) -> None:
        wrong_hash = "fc492195c94522e03b5c43e74da9c5404d369cf67213d78d0399a9d28ef80a4e"

        def change_claim(rows):
            next(row for row in rows if row["claim_id"] == "C-FRAME-BASE").update(
                source_locator="canonical source of truth",
                line_start="21",
                line_end="21",
                expected_excerpt_sha256=wrong_hash,
            )

        def change_expectation(data):
            data["critical_claim_provenance"]["C-FRAME-BASE"].update(
                source_locator="canonical source of truth",
                line_start=21,
                line_end=21,
                expected_excerpt_sha256=wrong_hash,
            )

        self.mutate("source_claim_registry.csv", change_claim)
        self.mutate_expectations(change_expectation)
        self.assert_error_contains("critical locator C-FRAME-BASE line_start mismatch")

    def test_classification_incompatible_with_authority(self) -> None:
        def change_claim(rows):
            next(row for row in rows if row["claim_id"] == "C-MODEL-ORDER")[
                "classification"
            ] = "HARDWARE_OBSERVATION"

        def change_expectation(data):
            counts = data["claim_classification_counts"]
            counts["MATDOG_VERIFIED"] -= 1
            counts["HARDWARE_OBSERVATION"] += 1

        self.mutate("source_claim_registry.csv", change_claim)
        self.mutate_expectations(change_expectation)
        self.assert_error_contains("incompatible with source class/authority/scope")

    def test_normacore_hash_wrong(self) -> None:
        def change(rows):
            next(row for row in rows if row["source_id"] == "N-SO101")[
                "sha256"
            ] = "0" * 64

        self.mutate("source_manifest.csv", change)
        self.assert_error_contains("source N-SO101: pinned blob checksum mismatch")

    def test_normacore_path_nonexistent(self) -> None:
        def change(rows):
            next(row for row in rows if row["source_id"] == "N-MATDOG-TEST")[
                "path"
            ] = "software/drivers/st3215/src/auto_calibrate/does_not_exist.rs"

        self.mutate("source_manifest.csv", change)
        self.assert_error_contains("source N-MATDOG-TEST: git object does not exist")

    def test_xgolite_hash_wrong(self) -> None:
        def change(rows):
            next(row for row in rows if row["source_id"] == "X-H1-SPEC")[
                "sha256"
            ] = "0" * 64

        self.mutate("source_manifest.csv", change)
        self.assert_error_contains("source X-H1-SPEC: pinned blob checksum mismatch")

    def test_xgolite_path_nonexistent(self) -> None:
        def change(rows):
            next(row for row in rows if row["source_id"] == "X-H1-SPEC")[
                "path"
            ] = "docs/reverse/does_not_exist.md"

        self.mutate("source_manifest.csv", change)
        self.assert_error_contains("source X-H1-SPEC: git object does not exist")

    def test_external_git_ref_nonexistent(self) -> None:
        def change(rows):
            next(row for row in rows if row["source_id"] == "N-SO101")[
                "ref"
            ] = "0000000000000000000000000000000000000000"

        self.mutate("source_manifest.csv", change)
        self.assert_error_contains("source N-SO101: git object does not exist")

    def test_repository_root_missing(self) -> None:
        missing = self.root / "missing-normacore-repository"
        errors = self.errors(normacore_repo=missing)
        self.assertTrue(any("repository root missing" in error for error in errors), errors)
        self.assertEqual(self.exit_code(normacore_repo=missing), 1)

    def test_frame_status_bogus(self) -> None:
        def change(rows):
            next(row for row in rows if row["frame_id"] == "world")["status"] = "BOGUS"

        self.mutate("frame_registry.csv", change)
        self.assert_error_contains("invalid frame status 'BOGUS'")

    def test_robot_historical_value_altered_in_registry(self) -> None:
        def change(rows):
            next(row for row in rows if row["mapping_id"] == "S-M13")[
                "raw_q0_tick"
            ] = "1544"

        self.mutate("servo_mapping_registry.csv", change)
        self.assert_error_contains("pinned robot calibration raw_q0_tick mismatch")

    def test_expected_excerpt_hash_wrong(self) -> None:
        def change(rows):
            next(row for row in rows if row["claim_id"] == "C-DIR-M11")[
                "expected_excerpt_sha256"
            ] = "0" * 64

        self.mutate("source_claim_registry.csv", change)
        self.assert_error_contains(
            "code-owned critical locator expected_excerpt_sha256 mismatch"
        )


if __name__ == "__main__":
    unittest.main()
