#!/usr/bin/env python3
"""Frozen G4 provenance hard-gate tests for the V5 oracle adapter."""

from __future__ import annotations

import json
from pathlib import Path
import sys
import tempfile
import unittest


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(CALIBRATION_DIR))

from matdog_geometry_g4_oracle_v5 import (  # noqa: E402
    EXPECTED_G4_CONTENT_SHA256,
    G4OracleV5Error,
    load_frozen_g4_profile,
)


FROZEN_G4 = REPO_ROOT / (
    "09_Logs/Validation_Reports/Geometry_Compiler/"
    "2026-08-10_164419_MATDOG_NEW_GEOMETRY_PHASE1B_REFERENCE_PROFILE.json"
)


class TestFrozenG4ContentGate(unittest.TestCase):
    def test_stored_and_recomputed_content_hash_both_match(self) -> None:
        profile = load_frozen_g4_profile(FROZEN_G4)
        self.assertEqual(profile["content_sha256"], EXPECTED_G4_CONTENT_SHA256)

    def test_content_tamper_is_rejected_even_when_stored_hash_is_unchanged(self) -> None:
        profile = json.loads(FROZEN_G4.read_text(encoding="utf-8"))
        profile["endpoints"][0]["mesh_predicted_contact_rad"] += 0.001
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "tampered.json"
            path.write_text(json.dumps(profile), encoding="utf-8")
            with self.assertRaises(G4OracleV5Error):
                load_frozen_g4_profile(path)


if __name__ == "__main__":
    unittest.main()
