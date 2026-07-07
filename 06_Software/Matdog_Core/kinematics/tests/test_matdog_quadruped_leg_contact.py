from __future__ import annotations

import math
from pathlib import Path
import unittest

from matdog_quadruped_leg_contact import (
    LEG_IDS,
    QuadrupedLegContactError,
    leg_foot_contact_from_joint_angles,
    leg_joint_names,
    load_leg_kinematic_contract,
)
from matdog_urdf_fk import (
    canonical_urdf_path,
    forward_kinematics,
)


def repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        candidate = (
            parent
            / "03_CAD/URDF/matt_robodog_rev00/"
            "matt_robodog_rev00.urdf"
        )

        if candidate.is_file():
            return parent

    raise RuntimeError("Repository root MATDOG non trovato")


class TestMatdogQuadrupedLegContact(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = repo_root()
        cls.urdf = canonical_urdf_path(cls.repo)

    def test_contracts_match_canonical_leg_chains(self):
        for leg in LEG_IDS:
            with self.subTest(leg=leg):
                contract = load_leg_kinematic_contract(
                    leg,
                    self.repo,
                )

                self.assertEqual(
                    contract.expected_chain_joint_names,
                    (
                        f"{leg}_hip_joint",
                        f"{leg}_upper_leg_joint",
                        f"{leg}_lower_leg_joint",
                        f"{leg}_foot_joint",
                    ),
                )

                self.assertEqual(
                    contract.foot_link_name,
                    f"{leg}_foot_link",
                )

                self.assertEqual(
                    len(contract.joint_limits_rad),
                    3,
                )

                for lower, upper in contract.joint_limits_rad:
                    self.assertLess(lower, upper)

    def test_visual_zero_contact_matches_fk_for_every_leg(self):
        for leg in LEG_IDS:
            with self.subTest(leg=leg):
                names = leg_joint_names(leg)
                joints = dict.fromkeys(names, 0.0)

                result = leg_foot_contact_from_joint_angles(
                    leg_id=leg,
                    joint_positions_rad=joints,
                    repo_root=self.repo,
                )

                fk = forward_kinematics(
                    urdf_path=self.urdf,
                    root_link="base_link",
                    tip_link=f"{leg}_foot_link",
                    joint_positions_rad=joints,
                    enforce_limits=True,
                )

                self.assertEqual(
                    result.contact.support_mode,
                    "NOMINAL_STRIP_CONTACT",
                )

                for actual, expected in zip(
                    result.world_from_foot_translation_m,
                    fk.tip_position_m,
                ):
                    self.assertAlmostEqual(
                        actual,
                        expected,
                        places=12,
                    )

                for actual, expected in zip(
                    result.contact.cross_section_contact_center_world_m,
                    fk.tip_position_m,
                ):
                    self.assertAlmostEqual(
                        actual,
                        expected,
                        places=12,
                    )

    def test_positive_hip_probe_is_edge_biased_for_every_leg(self):
        for leg in LEG_IDS:
            with self.subTest(leg=leg):
                names = leg_joint_names(leg)

                result = leg_foot_contact_from_joint_angles(
                    leg_id=leg,
                    joint_positions_rad={
                        names[0]: math.radians(10.0),
                        names[1]: 0.0,
                        names[2]: 0.0,
                    },
                    repo_root=self.repo,
                )

                self.assertEqual(
                    result.contact.support_mode,
                    "EDGE_BIASED_CONTACT",
                )

                self.assertAlmostEqual(
                    result.contact.axis_tilt_from_ground_rad,
                    math.radians(10.0),
                    places=12,
                )

    def test_world_translation_applies_to_every_leg(self):
        shift = (0.100, -0.050, 0.300)

        for leg in LEG_IDS:
            with self.subTest(leg=leg):
                names = leg_joint_names(leg)
                joints = dict.fromkeys(names, 0.0)

                base_result = leg_foot_contact_from_joint_angles(
                    leg_id=leg,
                    joint_positions_rad=joints,
                    repo_root=self.repo,
                )

                shifted_result = leg_foot_contact_from_joint_angles(
                    leg_id=leg,
                    joint_positions_rad=joints,
                    repo_root=self.repo,
                    world_from_base_translation_m=shift,
                )

                for base, shifted, delta in zip(
                    base_result.contact
                    .cross_section_contact_center_world_m,
                    shifted_result.contact
                    .cross_section_contact_center_world_m,
                    shift,
                ):
                    self.assertAlmostEqual(
                        shifted - base,
                        delta,
                        places=12,
                    )

    def test_incomplete_or_mixed_leg_joint_map_is_rejected(self):
        with self.assertRaises(QuadrupedLegContactError):
            leg_foot_contact_from_joint_angles(
                leg_id="rf",
                joint_positions_rad={
                    "rf_hip_joint": 0.0,
                    "rf_upper_leg_joint": 0.0,
                },
                repo_root=self.repo,
            )

        with self.assertRaises(QuadrupedLegContactError):
            leg_foot_contact_from_joint_angles(
                leg_id="rh",
                joint_positions_rad={
                    "rh_hip_joint": 0.0,
                    "rh_upper_leg_joint": 0.0,
                    "rh_lower_leg_joint": 0.0,
                    "lf_hip_joint": 0.0,
                },
                repo_root=self.repo,
            )

    def test_unknown_leg_is_rejected(self):
        with self.assertRaises(QuadrupedLegContactError):
            load_leg_kinematic_contract(
                "invalid",
                self.repo,
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
