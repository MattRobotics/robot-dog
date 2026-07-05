"""Test statici di sicurezza del capturer zero visuale MATDOG.

Non importano il capturer: restano offline e non dipendono da Station.
"""

import ast
import unittest
from pathlib import Path


CAPTURE_PATH = (
    Path(__file__).resolve().parents[1]
    / "matdog_capture_visual_zero.py"
)


class TestMatdogCaptureVisualZeroStatic(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = CAPTURE_PATH.read_text()
        cls.tree = ast.parse(cls.source, filename=str(CAPTURE_PATH))

    def test_no_motor_command_imports_or_symbols(self):
        forbidden_names = {
            "send_motor_commands",
            "set_torque",
            "send_goal",
            "MotorCommand",
        }

        seen_names = {
            node.id
            for node in ast.walk(self.tree)
            if isinstance(node, ast.Name)
        }

        self.assertFalse(forbidden_names & seen_names)

        imported_modules = set()

        for node in ast.walk(self.tree):
            if isinstance(node, ast.Import):
                imported_modules.update(
                    alias.name.split(".")[0]
                    for alias in node.names
                )

            if isinstance(node, ast.ImportFrom) and node.module:
                imported_modules.add(node.module.split(".")[0])

        self.assertNotIn("commands", imported_modules)
        self.assertNotIn("mirror", imported_modules)

    def test_only_temporary_path_is_opened_for_writing(self):
        direct_write_text_targets = []
        open_calls = []

        for node in ast.walk(self.tree):
            if not isinstance(node, ast.Call):
                continue

            if (
                isinstance(node.func, ast.Attribute)
                and node.func.attr == "write_text"
            ):
                direct_write_text_targets.append(
                    ast.unparse(node.func.value)
                )

            if (
                isinstance(node.func, ast.Attribute)
                and node.func.attr == "open"
            ):
                mode = None

                if node.args:
                    mode = ast.literal_eval(node.args[0])

                for keyword in node.keywords:
                    if keyword.arg == "mode":
                        mode = ast.literal_eval(keyword.value)

                if isinstance(mode, str) and (
                    "w" in mode or "a" in mode or "x" in mode or "+" in mode
                ):
                    open_calls.append(
                        (ast.unparse(node.func.value), mode)
                    )

        self.assertEqual(direct_write_text_targets, [])
        self.assertEqual(open_calls, [("temporary_path", "x")])

    def test_output_guard_contract_exists(self):
        self.assertIn("output_path == config_path", self.source)
        self.assertIn("output_path.exists()", self.source)
        self.assertIn("output_path.relative_to(log_root)", self.source)
        self.assertIn("os.link(temporary_path, output_path)", self.source)
        self.assertIn("temporary_created", self.source)
        self.assertNotIn("os.replace(", self.source)


if __name__ == "__main__":
    unittest.main(verbosity=2)
