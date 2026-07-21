import ast
import importlib.util
from pathlib import Path
import sys
import unittest


CALIBRATION_DIR = Path(__file__).resolve().parents[1]
MODULE_PATH = (
    CALIBRATION_DIR / "matdog_mechanical_endstop_calibration.py"
)

spec = importlib.util.spec_from_file_location(
    "matdog_mechanical_endstop_calibration",
    MODULE_PATH,
)
module = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = module
spec.loader.exec_module(module)


class MechanicalEndstopPlanTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.data = module.load_contract(module.DEFAULT_CONFIG)
        cls.steps = module.build_plan(cls.data)

    def test_contact_state_contract(self):
        self.assertEqual(
            [state.value for state in module.ContactState],
            [
                "FREE_MOTION",
                "CONTACT_SUSPECTED",
                "CONTACT_CONFIRMED",
                "CONTACT_REPEATABLE",
                "AMBIGUOUS_CONTACT",
                "HARD_ABORT",
            ],
        )

    def test_complete_plan_step_count(self):
        self.assertEqual(len(self.steps), 96)

    def test_contact_approaches_and_backoffs(self):
        approaches = [
            step
            for step in self.steps
            if step.action == "APPROACH_UNTIL_CONTACT"
        ]
        backoffs = [
            step
            for step in self.steps
            if step.action == "BACKOFF_AND_VERIFY_RECOVERY"
        ]

        self.assertEqual(len(approaches), 48)
        self.assertEqual(len(backoffs), 24)

        self.assertEqual(
            sum(step.attempt == 1 for step in approaches),
            24,
        )
        self.assertEqual(
            sum(step.attempt == 2 for step in approaches),
            24,
        )

    def test_front_leg_parking_dependencies(self):
        lf_first = next(step for step in self.steps if step.leg == "LF")
        rf_first = next(step for step in self.steps if step.leg == "RF")
        rh_first = next(step for step in self.steps if step.leg == "RH")
        lh_first = next(step for step in self.steps if step.leg == "LH")

        self.assertEqual(lf_first.dependent_leg, "LH")
        self.assertEqual(rf_first.dependent_leg, "RH")
        self.assertNotEqual(rh_first.phase, "DEPENDENCY_PARK")
        self.assertNotEqual(lh_first.phase, "DEPENDENCY_PARK")

    def test_all_hardware_limits_remain_null(self):
        for joint in self.data["joints"].values():
            self.assertEqual(
                joint["measured_contact_rad"],
                {"min": None, "max": None},
            )
            self.assertEqual(
                joint["safe_limit_rad"],
                {"min": None, "max": None},
            )

    def test_planner_has_no_command_capability(self):
        tree = ast.parse(MODULE_PATH.read_text(encoding="utf-8"))

        forbidden_import_roots = {
            "serial",
            "pyserial",
            "commands",
            "mirror",
            "state",
        }
        forbidden_call_names = {
            "send_commands",
            "send_motor_commands",
            "send_goal",
            "set_torque",
            "new_station_client",
        }

        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                for alias in node.names:
                    self.assertNotIn(
                        alias.name.split(".")[0],
                        forbidden_import_roots,
                    )

            if isinstance(node, ast.ImportFrom):
                root = (node.module or "").split(".")[0]
                self.assertNotIn(root, forbidden_import_roots)
                self.assertFalse(
                    (node.module or "").startswith("software.station")
                )

            if isinstance(node, ast.Call) and isinstance(node.func, ast.Name):
                self.assertNotIn(node.func.id, forbidden_call_names)

        source = MODULE_PATH.read_text(encoding="utf-8")
        self.assertNotIn('"--execute"', source)
        self.assertNotIn("'--execute'", source)


if __name__ == "__main__":
    unittest.main()
