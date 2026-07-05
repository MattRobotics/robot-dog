"""Test dinamici delle guardie di scrittura del capturer MATDOG.

I test non avviano Station e non inviano comandi ai servo.
"""

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock


CALDIR = Path(__file__).resolve().parents[1]
CAPTURE_PATH = CALDIR / "matdog_capture_visual_zero.py"

if str(CALDIR) not in sys.path:
    sys.path.insert(0, str(CALDIR))

spec = importlib.util.spec_from_file_location(
    "matdog_capture_visual_zero_under_test",
    CAPTURE_PATH,
)
capture = importlib.util.module_from_spec(spec)
assert spec.loader is not None
spec.loader.exec_module(capture)


class TestMatdogCaptureVisualZeroPaths(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.root = Path(self.tempdir.name)
        self.log_root = self.root / "09_Logs/Calibration_Sessions"
        self.log_root.mkdir(parents=True)

        self.config_path = self.root / "MATDOG_JOINT_CALIBRATION.yaml"
        self.config_path.write_text("schema_version: 2\n")

        self.log_root_patch = mock.patch.object(
            capture,
            "DEFAULT_LOG_DIR",
            self.log_root,
        )
        self.log_root_patch.start()

    def tearDown(self):
        self.log_root_patch.stop()
        self.tempdir.cleanup()

    def test_accepts_new_yaml_inside_log_root(self):
        output = self.log_root / "capture.yaml"

        resolved = capture.ensure_safe_output_path(
            output,
            self.config_path,
        )

        self.assertEqual(resolved, output.resolve())

    def test_rejects_config_as_output(self):
        with self.assertRaisesRegex(RuntimeError, "non può coincidere"):
            capture.ensure_safe_output_path(
                self.config_path,
                self.config_path,
            )

    def test_rejects_output_outside_log_root(self):
        outside = self.root / "outside.yaml"

        with self.assertRaisesRegex(RuntimeError, "deve stare sotto"):
            capture.ensure_safe_output_path(
                outside,
                self.config_path,
            )

    def test_rejects_non_yaml_output(self):
        output = self.log_root / "capture.txt"

        with self.assertRaisesRegex(RuntimeError, "estensione"):
            capture.ensure_safe_output_path(
                output,
                self.config_path,
            )

    def test_rejects_existing_output(self):
        output = self.log_root / "existing.yaml"
        output.write_text("already here\n")

        with self.assertRaisesRegex(RuntimeError, "esiste già"):
            capture.ensure_safe_output_path(
                output,
                self.config_path,
            )

    def test_rejects_stale_temporary_file(self):
        output = self.log_root / "capture.yaml"
        temporary = output.with_name(f".{output.name}.tmp")
        temporary.write_text("stale\n")

        with self.assertRaisesRegex(RuntimeError, "temporaneo"):
            capture.ensure_safe_output_path(
                output,
                self.config_path,
            )

    def test_atomic_write_preserves_preexisting_temporary_file(self):
        output = self.log_root / "capture.yaml"
        temporary = output.with_name(f".{output.name}.tmp")
        temporary.write_text("stale\n")

        with self.assertRaisesRegex(RuntimeError, "temporaneo"):
            capture.write_yaml_atomically(output, "new: value\n")

        self.assertFalse(output.exists())
        self.assertTrue(temporary.exists())
        self.assertEqual(temporary.read_text(), "stale\n")

    def test_atomic_write_publishes_once_without_overwrite(self):
        output = self.log_root / "capture.yaml"
        temporary = output.with_name(f".{output.name}.tmp")

        capture.write_yaml_atomically(output, "first: value\n")

        self.assertTrue(output.is_file())
        self.assertEqual(output.read_text(), "first: value\n")
        self.assertFalse(temporary.exists())

        with self.assertRaisesRegex(RuntimeError, "non verrà sovrascritto"):
            capture.write_yaml_atomically(output, "second: value\n")

        self.assertEqual(output.read_text(), "first: value\n")
        self.assertFalse(temporary.exists())


if __name__ == "__main__":
    unittest.main(verbosity=2)
