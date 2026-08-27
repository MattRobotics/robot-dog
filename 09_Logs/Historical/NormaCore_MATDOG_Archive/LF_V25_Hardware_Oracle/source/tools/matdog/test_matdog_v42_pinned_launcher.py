from __future__ import annotations

import hashlib
import unittest

from tools.matdog import matdog_v42_pinned_launcher as launcher


class MatdogV42PinnedLauncherTests(unittest.TestCase):
    def test_reviewed_python_inputs_match_repository_files(self) -> None:
        runner_hash = hashlib.sha256(launcher.RUNNER_PATH.read_bytes()).hexdigest()
        observer_hash = hashlib.sha256(
            launcher.OBSERVER_PATH.read_bytes()
        ).hexdigest()
        self.assertEqual(runner_hash, launcher.EXPECTED_RUNNER_SHA256)
        self.assertEqual(observer_hash, launcher.EXPECTED_OBSERVER_SHA256)

    def test_station_pin_and_provenance_are_complete(self) -> None:
        self.assertRegex(launcher.PINNED_STATION_SHA256, r"^[0-9a-f]{64}$")
        self.assertRegex(launcher.PINNED_STATION_SOURCE_COMMIT, r"^[0-9a-f]{40}$")
        self.assertRegex(
            launcher.PINNED_STATION_ARTIFACT_ZIP_SHA256,
            r"^[0-9a-f]{64}$",
        )
        self.assertGreater(launcher.PINNED_STATION_ARTIFACT_ID, 0)

    def test_launcher_installs_native_authority_observer_and_station_pin(self) -> None:
        runner = launcher.load_reviewed_runner()
        self.assertEqual(
            runner.EXPECTED_STATION_SHA256,
            launcher.PINNED_STATION_SHA256,
        )
        self.assertTrue(
            getattr(
                runner.FrameContract,
                "_matdog_native_authority_observer",
                False,
            )
        )
        self.assertFalse(
            getattr(runner.FrameContract, "_matdog_q0_phase_aware", False)
        )
        self.assertEqual(runner.EXPECTED_BUS_SERIAL, "5B14114953")
        self.assertEqual(runner.EXPECTED_FULL_TOTAL_STEPS, 58)
        self.assertEqual(
            runner.EXPECTED_START_BODY_HEX,
            "0a0a354231343131343935338a01020801",
        )
        self.assertEqual(
            runner.EXPECTED_STOP_BODY_HEX,
            "0a0a354231343131343935339201020801",
        )


if __name__ == "__main__":
    unittest.main()
