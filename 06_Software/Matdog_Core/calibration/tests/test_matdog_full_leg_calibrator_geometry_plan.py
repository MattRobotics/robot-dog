"""Artifact-locked tests for the Full Leg Calibrator geometry plan."""

from __future__ import annotations

from dataclasses import replace
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = CALIBRATION_DIR.parents[2]
if str(CALIBRATION_DIR) not in sys.path:
    sys.path.insert(0, str(CALIBRATION_DIR))

import generate_flc_leg_plan as generator  # noqa: E402
from matdog_geometry_path_planner_v5 import (  # noqa: E402
    PARKING_FEASIBLE_1DOF,
    PARKING_NOT_NEEDED,
)
from matdog_geometry_safety_policy import (  # noqa: E402
    MOTION_AUTHORIZATION_NOT_GRANTED,
    POLICY_PASS,
    POLICY_UNRESOLVED_BOUND,
)


EXPECTED_ENDPOINTS = (
    "lf_hip_joint:min",
    "lf_hip_joint:max",
    "lf_upper_leg_joint:min",
    "lf_upper_leg_joint:max",
    "lf_lower_leg_joint:min",
    "lf_lower_leg_joint:max",
    "rf_hip_joint:min",
    "rf_hip_joint:max",
    "rf_upper_leg_joint:min",
    "rf_upper_leg_joint:max",
    "rf_lower_leg_joint:min",
    "rf_lower_leg_joint:max",
    "rh_hip_joint:min",
    "rh_hip_joint:max",
    "rh_upper_leg_joint:min",
    "rh_upper_leg_joint:max",
    "rh_lower_leg_joint:min",
    "rh_lower_leg_joint:max",
    "lh_hip_joint:min",
    "lh_hip_joint:max",
    "lh_upper_leg_joint:min",
    "lh_upper_leg_joint:max",
    "lh_lower_leg_joint:min",
    "lh_lower_leg_joint:max",
)

EXPECTED_PARKING = {
    "lf_upper_leg_joint:max": ("lh_upper_leg_joint", 610865238198),
    "lf_lower_leg_joint:min": ("lf_upper_leg_joint", 1119919603363),
    "rf_upper_leg_joint:max": ("rh_upper_leg_joint", 610865238198),
    "rf_lower_leg_joint:min": ("rf_upper_leg_joint", 1119919603363),
    "rh_lower_leg_joint:min": ("rh_upper_leg_joint", 1628973968528),
    "lh_lower_leg_joint:min": ("lh_upper_leg_joint", 1628973968528),
}

EXPECTED_UNRESOLVED = {
    "lf_hip_joint:min",
    "lf_lower_leg_joint:min",
    "rf_hip_joint:max",
    "rf_lower_leg_joint:min",
    "rh_hip_joint:max",
    "rh_lower_leg_joint:min",
    "lh_hip_joint:min",
    "lh_lower_leg_joint:min",
}


class TestCanonicalGeometryProjection(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.inputs = generator.load_canonical_inputs(REPO_ROOT)
        cls.rows = generator.build_endpoint_rows(cls.inputs)
        cls.joint_names = generator.canonical_joint_names(cls.inputs.parking)
        cls.dependencies = generator.derive_dependencies(cls.rows)
        cls.order = generator.topological_order(cls.joint_names, cls.dependencies)

    def test_all_24_endpoint_rows_are_explicit_and_canonical(self):
        self.assertEqual(tuple(row.endpoint_id for row in self.rows), EXPECTED_ENDPOINTS)
        self.assertEqual([row.canonical_index for row in self.rows], list(range(24)))
        self.assertEqual(len({(row.joint_name, row.limit_side) for row in self.rows}), 24)

    def test_exact_18_no_parking_and_6_one_dof_outcomes(self):
        parking = {
            row.endpoint_id: (
                row.auxiliary_joint_name,
                row.auxiliary_angle_picorad,
            )
            for row in self.rows
            if row.parking_outcome == PARKING_FEASIBLE_1DOF
        }
        self.assertEqual(parking, EXPECTED_PARKING)
        self.assertEqual(
            sum(row.parking_outcome == PARKING_NOT_NEEDED for row in self.rows),
            18,
        )
        for row in self.rows:
            if row.endpoint_id in EXPECTED_PARKING:
                self.assertEqual(row.baseline_status, "PATH_OBSTRUCTION")
            else:
                self.assertEqual(row.parking_outcome, PARKING_NOT_NEEDED)
                self.assertEqual(row.baseline_status, "COLLISION_FREE")
                self.assertIsNone(row.auxiliary_joint_name)
                self.assertEqual(row.auxiliary_angle_picorad, 0)

    def test_raw_domain_and_clearance_policy_do_not_authorize_motion(self):
        self.assertEqual(
            sum(row.target_domain == "EXECUTABLE_URDF_DOMAIN" for row in self.rows),
            8,
        )
        self.assertEqual(
            sum(
                row.target_domain == "DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS"
                for row in self.rows
            ),
            16,
        )
        unresolved = {
            row.endpoint_id
            for row in self.rows
            if row.clearance_policy_result == POLICY_UNRESOLVED_BOUND
        }
        self.assertEqual(unresolved, EXPECTED_UNRESOLVED)
        self.assertEqual(
            sum(row.clearance_policy_result == POLICY_PASS for row in self.rows),
            16,
        )
        self.assertTrue(
            all(
                row.motion_authorization == MOTION_AUTHORIZATION_NOT_GRANTED
                for row in self.rows
            )
        )
        # Only the two cross-branch upper maxima are both URDF-executable and
        # clearance-PASS. The four lower-min parking rows remain diagnostic and
        # policy-unresolved; no row is a live authorization either way.
        executable_parking = {
            row.endpoint_id
            for row in self.rows
            if row.parking_outcome == PARKING_FEASIBLE_1DOF
            and row.target_domain == "EXECUTABLE_URDF_DOMAIN"
            and row.clearance_policy_result == POLICY_PASS
        }
        self.assertEqual(
            executable_parking,
            {"lf_upper_leg_joint:max", "rf_upper_leg_joint:max"},
        )

    def test_every_source_sequence_starts_at_q0_and_preserves_q0_context(self):
        all_joint_mask = (1 << 12) - 1
        for row in self.rows:
            with self.subTest(endpoint=row.endpoint_id):
                self.assertEqual(row.q0_start_mask, all_joint_mask)
                expected_held = all_joint_mask & ~(1 << row.target_joint_index)
                if row.auxiliary_joint_index is not None:
                    expected_held &= ~(1 << row.auxiliary_joint_index)
                self.assertEqual(row.q0_held_during_task_mask, expected_held)

    def test_dependency_graph_is_exact_and_acyclic(self):
        named_edges = {
            (
                self.joint_names[edge.prerequisite_joint_index],
                self.joint_names[edge.target_joint_index],
            )
            for edge in self.dependencies
        }
        self.assertEqual(
            named_edges,
            {
                ("lh_upper_leg_joint", "lf_upper_leg_joint"),
                ("lf_upper_leg_joint", "lf_lower_leg_joint"),
                ("rh_upper_leg_joint", "rf_upper_leg_joint"),
                ("rf_upper_leg_joint", "rf_lower_leg_joint"),
                ("rh_upper_leg_joint", "rh_lower_leg_joint"),
                ("lh_upper_leg_joint", "lh_lower_leg_joint"),
            },
        )
        self.assertEqual(len(self.order), 12)
        positions = {joint: position for position, joint in enumerate(self.order)}
        for edge in self.dependencies:
            self.assertLess(
                positions[edge.prerequisite_joint_index],
                positions[edge.target_joint_index],
            )

    def test_synthetic_dependency_cycle_is_rejected(self):
        lf_upper = self.joint_names.index("lf_upper_leg_joint")
        lf_lower = self.joint_names.index("lf_lower_leg_joint")
        cycle = self.dependencies + (
            generator.GeometryDependency(
                prerequisite_joint_index=lf_lower,
                target_joint_index=lf_upper,
                endpoint_index=255,
            ),
        )
        with self.assertRaisesRegex(
            generator.GeometryPlanGenerationError,
            "dependency cycle",
        ):
            generator.topological_order(self.joint_names, cycle)

    def test_lossless_pico_radian_values_are_not_old_millidegree_rounding(self):
        front = next(
            row for row in self.rows if row.endpoint_id == "lf_lower_leg_joint:min"
        )
        hind = next(
            row for row in self.rows if row.endpoint_id == "rh_lower_leg_joint:min"
        )
        self.assertEqual(front.auxiliary_angle_picorad, 1119919603363)
        self.assertEqual(hind.auxiliary_angle_picorad, 1628973968528)
        # These exact source decimals differ from +64.167 and +93.333 degrees.
        self.assertNotEqual(front.auxiliary_angle_picorad, 1119925421126)
        self.assertNotEqual(hind.auxiliary_angle_picorad, 1628978150455)


class TestGeneratedGeometryHeader(unittest.TestCase):
    def test_checked_in_header_is_byte_exact_generator_output(self):
        generator.check_generated_header(REPO_ROOT)

    def test_cli_check_is_read_only_and_passes(self):
        result = subprocess.run(
            [
                sys.executable,
                "-B",
                str(CALIBRATION_DIR / "generate_flc_leg_plan.py"),
                "--repo-root",
                str(REPO_ROOT),
                "--check",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("FLC_GEOMETRY_PLAN_CHECK=PASS", result.stdout)

    @unittest.skipIf(shutil.which("g++") is None, "g++ is required")
    def test_header_is_standalone_werror_and_lookup_is_fail_closed(self):
        header_dir = (
            REPO_ROOT
            / "05_Firmware/Full_Leg_Calibrator_V1/matdog_full_leg_calibrator_v1"
        )
        source = r'''
#include "flc_leg_plan.h"

int main() {
  if (FLC_ENDPOINT_GEOMETRY_PLAN_COUNT != 24) return 1;
  if (FLC_GEOMETRY_DEPENDENCY_COUNT != 6) return 2;
  if (!flcGeometryDependencyGraphAcyclic()) return 3;
  if (!flcGeometryGeneratedTopologicalOrderValid()) return 4;
  const FlcEndpointGeometryPlan *known =
      flcGeometryPlanFor("lf_hip_joint", "min");
  if (known == 0 || known->parkingOutcome != FLC_NO_PARKING_REQUIRED) return 5;
  const FlcEndpointGeometryPlan *park =
      flcGeometryPlanForEndpointId("lf_lower_leg_joint:min");
  if (park == 0 || park->auxiliaryAnglePicoRad != 1119919603363LL) return 6;
  if (flcGeometryPlanFor("unknown_joint", "min") != 0) return 7;
  if (flcGeometryPlanFor("lf_hip_joint", "typo") != 0) return 8;
  if (FLC_GEOMETRY_ARTIFACT_GRANTS_MOTION_AUTHORIZATION) return 9;
  return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            directory_path = Path(directory)
            source_path = directory_path / "geometry_plan.cpp"
            binary_path = directory_path / "geometry_plan"
            source_path.write_text(source, encoding="utf-8")
            build = subprocess.run(
                [
                    "g++",
                    "-std=c++11",
                    "-Wall",
                    "-Wextra",
                    "-Werror",
                    "-I",
                    str(header_dir),
                    str(source_path),
                    "-o",
                    str(binary_path),
                ],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(build.returncode, 0, build.stderr)
            run = subprocess.run(
                [str(binary_path)],
                capture_output=True,
                text=True,
                check=False,
            )
            self.assertEqual(run.returncode, 0, run.stderr)


if __name__ == "__main__":
    unittest.main()
