from __future__ import annotations

import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from matdog_foot_contact import (
    FootContactDegeneracyError,
    FootContactError,
    contact_from_foot_pose,
    load_foot_contact_model,
)


def repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        candidate = (
            parent
            / "06_Software/Matdog_Core/kinematics/"
            "MATDOG_FOOT_CONTACT_GEOMETRY.yaml"
        )

        if candidate.is_file():
            return parent

    raise RuntimeError("Repository root MATDOG non trovato")


IDENTITY = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


def rotation_x(angle_rad: float):
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)

    return (
        (1.0, 0.0, 0.0),
        (0.0, cosine, -sine),
        (0.0, sine, cosine),
    )


class TestMatdogFootContact(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = repo_root()
        cls.model = load_foot_contact_model(cls.repo)

    def test_local_contact_model_does_not_duplicate_urdf_offset(self):
        self.assertEqual(self.model.frame_name, "foot_link")
        self.assertEqual(
            self.model.nominal_contact_point_m,
            (0.0, 0.0, 0.0),
        )
        self.assertEqual(
            self.model.cylinder_center_in_foot_link_m,
            (0.0, 0.0, 0.0149),
        )
        self.assertEqual(
            self.model.cylinder_axis_in_foot_link_unit,
            (0.0, 1.0, 0.0),
        )

    def test_model_matches_verified_cad_primitive(self):
        self.assertAlmostEqual(self.model.cylinder_radius_m, 0.0149)
        self.assertAlmostEqual(self.model.total_tread_width_m, 0.0139)
        self.assertAlmostEqual(self.model.end_fillet_radius_m, 0.0020)
        self.assertAlmostEqual(
            self.model.central_rigid_support_width_m,
            0.0099,
        )

    def test_all_foot_joint_transforms_match_canonical_contract(self):
        urdf = self.repo / (
            "03_CAD/URDF/matt_robodog_rev00/"
            "matt_robodog_rev00.urdf"
        )

        root = ET.parse(urdf).getroot()

        expected = {
            "lf": ("0.107 -0.0015 -0.0499", "0 0 0"),
            "rf": ("0.107 0.0015 -0.0499", "0 0 0"),
            "rh": ("0.107 0.0015 -0.0499", "0 0 0"),
            "lh": ("0.107 -0.0015 -0.0499", "0 0 0"),
        }

        for leg, (expected_xyz, expected_rpy) in expected.items():
            with self.subTest(leg=leg):
                joint = root.find(
                    f"./joint[@name='{leg}_foot_joint']"
                )

                self.assertIsNotNone(joint)

                parent = joint.find("parent")
                child = joint.find("child")
                origin = joint.find("origin")

                self.assertIsNotNone(parent)
                self.assertIsNotNone(child)
                self.assertIsNotNone(origin)

                self.assertEqual(
                    parent.attrib["link"],
                    f"{leg}_lower_leg_link",
                )
                self.assertEqual(
                    child.attrib["link"],
                    f"{leg}_foot_link",
                )
                self.assertEqual(
                    origin.attrib.get("xyz"),
                    expected_xyz,
                )
                self.assertEqual(
                    origin.attrib.get("rpy"),
                    expected_rpy,
                )

    def test_nominal_pose_has_central_support_strip(self):
        contact = contact_from_foot_pose(
            model=self.model,
            rotation_world_from_foot=IDENTITY,
            translation_world_from_foot_m=(0.0, 0.0, 0.0),
        )

        self.assertEqual(
            contact.support_mode,
            "NOMINAL_STRIP_CONTACT",
        )
        self.assertAlmostEqual(
            contact.axis_tilt_from_ground_rad,
            0.0,
            places=12,
        )

        for actual, expected in zip(
            contact.cross_section_contact_center_world_m,
            (0.0, 0.0, 0.0),
        ):
            self.assertAlmostEqual(actual, expected, places=12)

        self.assertAlmostEqual(
            contact.support_strip_end_a_world_m[1],
            -0.00495,
            places=12,
        )
        self.assertAlmostEqual(
            contact.support_strip_end_b_world_m[1],
            +0.00495,
            places=12,
        )
        self.assertAlmostEqual(
            contact.support_strip_end_a_world_m[2],
            0.0,
            places=12,
        )
        self.assertAlmostEqual(
            contact.support_strip_end_b_world_m[2],
            0.0,
            places=12,
        )

    def test_tilted_axis_becomes_edge_biased(self):
        contact = contact_from_foot_pose(
            model=self.model,
            rotation_world_from_foot=rotation_x(math.radians(10.0)),
            translation_world_from_foot_m=(0.0, 0.0, 0.0),
        )

        self.assertEqual(
            contact.support_mode,
            "EDGE_BIASED_CONTACT",
        )
        self.assertAlmostEqual(
            contact.axis_tilt_from_ground_rad,
            math.radians(10.0),
            places=12,
        )
        self.assertLess(
            contact.lowest_core_point_world_m[2],
            contact.cross_section_contact_center_world_m[2],
        )

    def test_axis_vertical_is_rejected_as_degenerate(self):
        with self.assertRaises(FootContactDegeneracyError):
            contact_from_foot_pose(
                model=self.model,
                rotation_world_from_foot=rotation_x(math.pi / 2.0),
                translation_world_from_foot_m=(0.0, 0.0, 0.0),
            )

    def test_non_rotation_matrix_is_rejected(self):
        invalid = (
            (2.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        )

        with self.assertRaises(FootContactError):
            contact_from_foot_pose(
                model=self.model,
                rotation_world_from_foot=invalid,
                translation_world_from_foot_m=(0.0, 0.0, 0.0),
            )


if __name__ == "__main__":
    unittest.main(verbosity=2)
