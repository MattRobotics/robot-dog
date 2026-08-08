"""Test automatici MATDOG per l'analisi di sensibilità/incertezza per-endpoint."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

from matdog_geometry_contact_search import (  # noqa: E402
    _pose_for_probe_angle,
    load_all_endpoints,
    search_endpoint_contact,
)
from matdog_geometry_scene import RobotScene  # noqa: E402
from matdog_geometry_uncertainty import (  # noqa: E402
    TOLERANCE_BUDGET_NOTE,
    compute_contact_sensitivity,
)


def _repo_root() -> Path:
    for parent in Path(__file__).resolve().parents:
        if (parent / "03_CAD/URDF/matt_robodog_rev00/matt_robodog_rev00.urdf").is_file():
            return parent
    raise RuntimeError("Repository root non trovato")


REPO_ROOT = _repo_root()


class TestSensitivityPairPinning(unittest.TestCase):
    """Item H of the reconciliation test contract: the sensitivity
    gradient must be evaluated on the exact same contact pair that
    defines the endpoint's own converged result, never on whatever pair
    `worst_pair_at_pose` happens to consider worst overall near that
    pose (canonical handoff section 8)."""

    @classmethod
    def setUpClass(cls):
        cls.scene = RobotScene(REPO_ROOT)
        cls.endpoints = {e.endpoint_id: e for e in load_all_endpoints(REPO_ROOT)}
        # Each search_endpoint_contact call on real geometry costs minutes
        # (coarse scan + bisection, run twice for the endstop/path dual
        # policy); compute it once here and share across test methods
        # instead of once per method.
        cls.endpoint = cls.endpoints["lf_hip_min"]
        cls.result = search_endpoint_contact(cls.scene, cls.endpoint, other_legs_pose={})
        cls.sensitivity = compute_contact_sensitivity(
            cls.scene, cls.endpoint, cls.result.bracket_clear_rad, {},
            cls.result.contact_link_a, cls.result.contact_link_b,
        )

    def test_sensitivity_result_pair_matches_requested_pair(self):
        self.assertEqual(self.result.result_kind, "MESH_CONTACT_FOUND")
        self.assertIsNotNone(self.result.contact_link_a)
        self.assertIsNotNone(self.result.contact_link_b)
        self.assertEqual(self.sensitivity.contact_link_a, self.result.contact_link_a)
        self.assertEqual(self.sensitivity.contact_link_b, self.result.contact_link_b)

    def test_sensitivity_uses_check_link_pair_directly_on_the_endpoint_pair(self):
        """Proves the fix at the numeric level: `compute_contact_sensitivity`'s
        near-probe clearance must equal `scene.check_link_pair` evaluated
        directly on the endpoint's own pair at the same pose -- the
        pre-fix implementation called `worst_pair_at_pose` instead, which
        is not guaranteed to agree (it can return a different, worse
        pair's clearance)."""
        pose_near = _pose_for_probe_angle(self.endpoint, self.result.bracket_clear_rad, {})
        direct = self.scene.check_link_pair(
            self.result.contact_link_a, self.result.contact_link_b, pose_near, require_distance=True
        )

        self.assertAlmostEqual(self.sensitivity.clearance_near_m, direct.clearance_m, places=9)
        self.assertEqual(self.sensitivity.clearance_near_kind, direct.clearance_kind)

    def test_tolerance_budget_note_present_and_explicit_about_two_parts(self):
        self.assertEqual(self.sensitivity.tolerance_budget_note, TOLERANCE_BUDGET_NOTE)
        self.assertIn("TWO independently-printed parts", self.sensitivity.tolerance_budget_note)
        self.assertIn("NOT summed or RSS-combined", self.sensitivity.tolerance_budget_note)


if __name__ == "__main__":
    unittest.main()
