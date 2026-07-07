from __future__ import annotations

from pathlib import Path
import unittest

from matdog_body_stance import (
    audit_body_stance_geometry,
    load_body_stance_geometry,
)


def repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        candidate = (
            parent
            / "06_Software/Matdog_Core/kinematics/"
            "MATDOG_BODY_STANCE_GEOMETRY.yaml"
        )

        if candidate.is_file():
            return parent

    raise RuntimeError("Repository root MATDOG non trovato")


class TestMatdogBodyStance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.repo = repo_root()

    def test_design_contract_values(self):
        geometry = load_body_stance_geometry(self.repo)

        self.assertEqual(set(geometry.front_legs), {"lf", "rf"})
        self.assertEqual(set(geometry.rear_legs), {"rh", "lh"})

        self.assertAlmostEqual(
            geometry.front_hip_axis_elevation_relative_rear_m,
            0.0200,
            places=12,
        )
        self.assertAlmostEqual(
            geometry.visual_zero_front_contact_z_minus_rear_contact_z_m,
            0.0200,
            places=12,
        )
        self.assertAlmostEqual(
            geometry.expected_visual_zero_contact_spread_m,
            0.0200,
            places=12,
        )

    def test_urdf_and_fk_preserve_intentional_front_rear_offset(self):
        audit = audit_body_stance_geometry(self.repo)

        self.assertAlmostEqual(
            audit.front_minus_rear_hip_elevation_m,
            0.0200,
            places=12,
        )
        self.assertAlmostEqual(
            audit.front_minus_rear_contact_z_m,
            0.0200,
            places=12,
        )
        self.assertAlmostEqual(
            audit.contact_z_spread_m,
            0.0200,
            places=12,
        )

    def test_visual_zero_is_intentionally_not_coplanar(self):
        audit = audit_body_stance_geometry(self.repo)

        self.assertGreater(
            audit.contact_z_spread_m,
            0.0,
        )
        self.assertEqual(
            tuple(
                item.contact.support_mode
                for item in audit.visual_zero_contacts
            ),
            (
                "NOMINAL_STRIP_CONTACT",
                "NOMINAL_STRIP_CONTACT",
                "NOMINAL_STRIP_CONTACT",
                "NOMINAL_STRIP_CONTACT",
            ),
        )


if __name__ == "__main__":
    unittest.main(verbosity=2)
