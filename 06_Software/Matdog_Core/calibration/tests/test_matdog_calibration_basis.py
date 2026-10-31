"""The REV00 calibration basis: preserved inputs stay verifiable and the operational model may
differ from them only by the declared delta. Mutations run on temporary copies."""

from __future__ import annotations

import shutil
import sys
import tempfile
import unittest
from pathlib import Path

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

import matdog_calibration_basis as basis  # noqa: E402

REPO = CALDIR.parents[2]
MODEL = "03_CAD/URDF/matt_robodog_rev00"
REV00_URDF_SHA = "3890a3f0732dbed8abdc559106d7f32ee8d6e2111c8e1a06d2485bf2ffc81e59"
REV00_BASE_SHA = "0a485e7a1101d457f317b664e52a7a4ef061382e6ecc2c7c87a333c81d5b466f"
REV00_MANIFEST_SHA = "60fff604eae0857c7c61f115bbe3922949ababdd827fab61c74e1673336c39e1"


def make_tree(tmp: Path) -> Path:
    shutil.copytree(REPO / basis.BASIS_DIR, tmp / basis.BASIS_DIR)
    (tmp / MODEL).mkdir(parents=True)
    shutil.copy(REPO / MODEL / basis.URDF_NAME, tmp / MODEL / basis.URDF_NAME)
    shutil.copytree(REPO / MODEL / "meshes/collision", tmp / MODEL / "meshes/collision")
    return tmp


class CalibrationBasisTests(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self._tmp.cleanup)
        self.tree = make_tree(Path(self._tmp.name))

    def test_repository_is_rev00_plus_declared_delta(self):
        self.assertEqual(basis.verify_delta(REPO), [])

    def test_preserved_files_are_the_recorded_rev00_inputs(self):
        self.assertEqual(basis.sha256_file(basis.basis_file(REPO, basis.URDF_NAME)), REV00_URDF_SHA)
        self.assertEqual(basis.sha256_file(basis.basis_file(REPO, basis.MANIFEST_NAME)), REV00_MANIFEST_SHA)
        self.assertEqual(basis.sha256_file(basis.basis_file(REPO, "meshes/collision/base_link.stl")),
                         REV00_BASE_SHA)
        # operational model is genuinely different for the superseded files
        self.assertNotEqual(basis.sha256_file(REPO / MODEL / basis.URDF_NAME), REV00_URDF_SHA)
        self.assertNotEqual(basis.sha256_file(REPO / MODEL / "meshes/collision/base_link.stl"), REV00_BASE_SHA)

    def test_recorded_input_resolves_to_preserved_copy_only_for_superseded_files(self):
        urdf = basis.resolve_recorded_input(REPO, f"{MODEL}/{basis.URDF_NAME}", REV00_URDF_SHA)
        self.assertTrue(str(urdf).startswith(str(REPO / basis.BASIS_DIR)))
        live = basis.resolve_recorded_input(
            REPO, f"{MODEL}/meshes/collision/lf_hip_link.stl",
            basis.sha256_file(REPO / MODEL / "meshes/collision/lf_hip_link.stl"))
        self.assertEqual(live, REPO / MODEL / "meshes/collision/lf_hip_link.stl")

    def test_wrong_recorded_hash_is_still_a_drift(self):
        with self.assertRaises(basis.BasisError):
            basis.resolve_recorded_input(REPO, f"{MODEL}/{basis.URDF_NAME}", "0" * 64)
        # a non-superseded file never falls back to the preserved copies
        with self.assertRaises(basis.BasisError):
            basis.resolve_recorded_input(REPO, f"{MODEL}/meshes/collision/lf_hip_link.stl", REV00_BASE_SHA)

    def test_joint_limit_change_is_rejected(self):
        self.assertEqual(basis.verify_delta(self.tree), [])
        self._edit_first('lower="-0.785398163397"', 'lower="-0.795398163397"')
        self.assertTrue(any("beyond the declared delta" in p for p in basis.verify_delta(self.tree)))

    def _edit_first(self, old: str, new: str):
        path = self.tree / MODEL / basis.URDF_NAME
        path.write_text(path.read_text(encoding="utf-8").replace(old, new, 1), encoding="utf-8")

    def test_motor_metadata_change_is_rejected(self):
        path = self.tree / MODEL / basis.URDF_NAME
        text = path.read_text(encoding="utf-8")
        self.assertIn("<motorId>", text)
        start = text.index("<motorId>") + len("<motorId>")
        end = text.index("</motorId>")
        path.write_text(text[:start] + str(int(text[start:end]) + 1) + text[end:], encoding="utf-8")
        self.assertTrue(any("beyond the declared delta" in p for p in basis.verify_delta(self.tree)))

    def test_hip_z_not_equal_to_declared_is_rejected(self):
        self._edit_first('xyz="0.1125 0.0475 0.0487"', 'xyz="0.1125 0.0475 0.0488"')
        self.assertTrue(any("operational z" in p for p in basis.verify_delta(self.tree)))

    def test_hip_xy_change_is_rejected(self):
        self._edit_first('xyz="0.1125 0.0475 0.0487"', 'xyz="0.1126 0.0475 0.0487"')
        self.assertTrue(any("x/y changed" in p for p in basis.verify_delta(self.tree)))

    def test_other_collision_mesh_change_is_rejected(self):
        target = self.tree / MODEL / "meshes/collision/rf_foot_link.stl"
        target.write_bytes(target.read_bytes() + b"\0")
        self.assertTrue(any("rf_foot_link.stl" in p and "not identical" in p
                            for p in basis.verify_delta(self.tree)))

    def test_altered_preserved_file_is_rejected(self):
        target = self.tree / basis.BASIS_DIR / basis.MANIFEST_NAME
        target.write_text(target.read_text(encoding="utf-8") + "\n", encoding="utf-8")
        self.assertTrue(any("altered" in p for p in basis.verify_delta(self.tree)))

    def test_visual_mesh_outside_collision_set_is_rejected(self):
        path = self.tree / MODEL / basis.URDF_NAME
        text = path.read_text(encoding="utf-8")
        marker = '<mesh filename="meshes/collision/base_link.stl"'
        self.assertGreaterEqual(text.count(marker), 1)
        path.write_text(text.replace(marker, '<mesh filename="meshes/base_link.stl"', 1), encoding="utf-8")
        self.assertTrue(basis.verify_delta(self.tree))


if __name__ == "__main__":
    unittest.main()
