#!/usr/bin/env python3
"""Offline G10 immutability pins for approved geometry inputs and baselines."""

from __future__ import annotations

import hashlib
from pathlib import Path
import unittest


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = CALIBRATION_DIR.parents[2]
MODEL_DIR = REPO_ROOT / "03_CAD/URDF/matt_robodog_rev00"
REPORT_DIR = REPO_ROOT / "09_Logs/Validation_Reports/Geometry_Compiler"


PROTECTED_SHA256 = {
    # Historical schema v1/v2/v3/v4 artifacts.
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_155742_MATDOG_CALIBRATION_GEOMETRY_PROFILE.json": "b327aa21fa3ec29b5e1762831883e85d448662134b4ff0892952920d43fbb322",
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_155742_MATDOG_CALIBRATION_GEOMETRY_REPORT.md": "a6369f67b3d5b5111c045ec9ad931e1aaf72f00a2105919e92e12c8a01c5207c",
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_193932_MATDOG_CALIBRATION_GEOMETRY_PROFILE.json": "d3db673832bba3803082231717d578c5e15a70d47ce11ba59beded76777fc7d3",
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_193932_MATDOG_CALIBRATION_GEOMETRY_REPORT.md": "757ceec3907d669addfd895eb7622cd2a002d1c8db99877034a7b7ba7ec35504",
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_PROFILE.json": "6b438285f7145b2cb4f9fc11f1e9b2342dfbb7360d83976ad9a25cd590a0b7c5",
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-07_204107_MATDOG_CALIBRATION_GEOMETRY_REPORT.md": "de483529941ceee04e921086dd42cd66956a0749bfc39d161115650d619a4b50",
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-08_231600_MATDOG_CALIBRATION_GEOMETRY_PROFILE.json": "c2e980bfc49e80b957fc0a326b1ac98724fc1d4ac16121da19fcf6ab2be3eff6",
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-08_231600_MATDOG_CALIBRATION_GEOMETRY_REPORT.md": "7774af1c41df2859fed89bfb468d4e04be95352158fa7a199493bf823775a858",
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-08_231600_MATDOG_COLLISION_MESH_HASH_MANIFEST.json": "17904a647d6dd357586765829d24dbd0b09c3b1fb976a2084eb9b1f1537aac03",
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-08_231600_MATDOG_COLLISION_MESH_HASH_MANIFEST.md": "cb681649b6d4d0c54457eaa3f386ec2de04c083f4279c377c276ad96475f0f06",
    # Frozen G4 reference and audit evidence.
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_PROFILE.json": "f1b059a58c51508345ec583cc421cf9ca66ec8e9ac5547649c05bbf755e5c5fa",
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_REPORT.md": "140c0aa5a06e3d2855222a28cf06d8de417a391938e954607e1b91b49e4e580c",
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_BENCHMARK_B_TIME.txt": "02e89f8431ebd94735964654964b1b726e9efb4f6175b917ffff8ebef689ea78",
    "09_Logs/Validation_Reports/Geometry_Compiler/2026-08-10_164419_MATDOG_NEW_GEOMETRY_G0_G4_BASELINE.md": "53ba3165b5398ef179afb1abfdfe6036bc4f8b2f749d316b73a4a4e11d017920",
    # Frozen validated Phase1B sources.
    "06_Software/Matdog_Core/calibration/matdog_geometry_compiler.py": "fa23b952ddb8dc6249b0a66319cfcbf6d5a2c69c0b4d66d9ca754405c9e86ef6",
    "06_Software/Matdog_Core/calibration/matdog_geometry_contact_search.py": "3c330e0565f8e296eb55981fc221a4b043e0cee6eb2caafb868e5e04fbe997bf",
    "06_Software/Matdog_Core/calibration/matdog_geometry_mesh_kernel.py": "f7119fd93b913968ea342269b4369546ecbcad2e8731e7d4d8fabd4ba08c017c",
    "06_Software/Matdog_Core/calibration/matdog_geometry_path_planner.py": "a0d12f98dc2a89fedc13a5a4e4f8d5d062bfa523e1b39967b036fa851b0ccc5b",
    "06_Software/Matdog_Core/calibration/matdog_geometry_profile.py": "d1175234cb0362d581b392df9c9eb12d2fd7d59d7dd2c49977c0a6f3a949c62a",
    "06_Software/Matdog_Core/calibration/matdog_geometry_scene.py": "6c5043e053453f5215f9ebab5ade9ea781703119b884f2719d78910e77bbb2c9",
    "06_Software/Matdog_Core/calibration/matdog_geometry_uncertainty.py": "6f11b9577e71d695e6723f4e3aa23453dad707e0200dfe0c0f00f8d090de4152",
    # Immutable LF evidence consumed only by the separate reconciler.
    "06_Software/Matdog_Core/calibration/MATDOG_LF_CALIBRATION_V25_FINAL.md": "8646a8c2a59998396f0766b4b8d2fee27343e4750a46880b15727a200f468332",
    "06_Software/Matdog_Core/calibration/MATDOG_LF_V25_HARDWARE_EVIDENCE_2026-08-04.json": "6eae3201a00b5299550028d5b4e1e73d67520deccf5a85e548f3b07b1777cab4",
}

MODEL_SHA256SUMS_SHA256 = (
    "60fff604eae0857c7c61f115bbe3922949ababdd827fab61c74e1673336c39e1"
)


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class TestGeometryV5Immutability(unittest.TestCase):
    def test_historical_artifacts_phase1b_sources_and_lf_evidence_are_untouched(self):
        for relative_path, expected_sha in PROTECTED_SHA256.items():
            with self.subTest(path=relative_path):
                path = REPO_ROOT / relative_path
                self.assertTrue(path.is_file(), f"protected file missing: {relative_path}")
                self.assertEqual(_sha256(path), expected_sha, relative_path)

    def test_approved_model_manifest_and_all_36_entries_match(self):
        manifest_path = MODEL_DIR / "SHA256SUMS.txt"
        self.assertEqual(_sha256(manifest_path), MODEL_SHA256SUMS_SHA256)
        entries = []
        for line in manifest_path.read_text(encoding="utf-8").splitlines():
            expected_sha, relative_path = line.split(maxsplit=1)
            entries.append((expected_sha, relative_path))
        self.assertEqual(len(entries), 36)
        for expected_sha, relative_path in entries:
            with self.subTest(path=relative_path):
                self.assertEqual(_sha256(MODEL_DIR / relative_path), expected_sha)

    def test_frozen_g4_content_hash_is_exact(self):
        import json

        profile_path = REPORT_DIR / (
            "2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_PROFILE.json"
        )
        profile = json.loads(profile_path.read_text(encoding="utf-8"))
        self.assertEqual(
            profile["content_sha256"],
            "4a2a2324f2838b9da0240f838e8172701ff35f83d20d29edddeec2fe15d83a61",
        )


if __name__ == "__main__":
    unittest.main()
