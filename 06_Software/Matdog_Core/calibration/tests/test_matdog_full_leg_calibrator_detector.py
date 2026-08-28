"""Fault-injection tests for the bounded contact/endpoint state machine.

These drive the ACTUAL firmware engine: ``flc_contact_detector.h`` is compiled
into a host harness and fed observation streams over a pipe. There is no Python
re-implementation of the detector, so a passing test here is evidence about the
code that would run on the ESP32-S3, not about a model of it.

Covers, from section 13 of the V1 specification: command outside 0..4095,
approach toward the wrap boundary, contact never occurs, contact occurs too
early, one-sample false contact spike, repeated contacts inconsistent, current
spike without persistent mechanical contact, temperature guard, voltage guard,
telemetry timeout, bus read failure, target joint does not move, and wrong
direction.
"""

from __future__ import annotations

import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

CALDIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(CALDIR))

REPO_ROOT = CALDIR.parents[2]  # calibration -> Matdog_Core -> 06_Software -> repo
FIRMWARE_DIR = REPO_ROOT / "05_Firmware" / "Full_Leg_Calibrator_V1"
SKETCH_DIR = FIRMWARE_DIR / "matdog_full_leg_calibrator_v1"
HARNESS_SRC = FIRMWARE_DIR / "tests" / "flc_detector_harness.cpp"

from matdog_joint_math import signed_tick_delta  # noqa: E402

# A configuration whose numbers exist only to exercise the state machine.
# They are NOT proposed calibration constants: every contact parameter for the
# real robot is CHARACTERIZATION_REQUIRED. See matdog_full_leg_calibrator_policy.
TEST_CONFIG = dict(
    max_progress=2,
    max_velocity=10,
    target_tolerance=10,
    min_travel=24,
    persistence=3,
    grace=4,
    hard_current=200,
    expected_torque_limit=400,
    thermal_c=70,
    voltage_min=40,
    voltage_max=140,
    travel_budget=600,
    time_budget_ms=20000,
    probe_sign=1,
)


class DetectorHarness:
    """Thin driver around the compiled C++ harness process."""

    def __init__(self, binary: Path) -> None:
        self.process = subprocess.Popen(
            [str(binary)],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
        )

    def send(self, line: str) -> str:
        assert self.process.stdin and self.process.stdout
        self.process.stdin.write(line + "\n")
        self.process.stdin.flush()
        return self.process.stdout.readline().strip()

    def configure(self, **overrides) -> str:
        cfg = {**TEST_CONFIG, **overrides}
        return self.send(
            "CONFIG {max_progress} {max_velocity} {target_tolerance} {min_travel} "
            "{persistence} {grace} {hard_current} {expected_torque_limit} "
            "{thermal_c} {voltage_min} {voltage_max} {travel_budget} "
            "{time_budget_ms} {probe_sign}".format(**cfg)
        )

    def baseline(self, median: int = 20, mad: int = 3, valid: int = 1) -> str:
        return self.send(f"BASELINE {median} {mad} {valid}")

    def init(self, start: int) -> str:
        return self.send(f"INIT {start}")

    def observe(
        self,
        position: int,
        target: int,
        *,
        velocity: int = 0,
        current: int = 20,
        telemetry_valid: int = 1,
        driver_error: int = 0,
        status: int = 0,
        torque_enabled: int = 1,
        torque_limit: int | None = None,
        goal: int | None = None,
        temperature: int = 30,
        voltage: int = 120,
        elapsed_ms: int = 100,
    ) -> dict[str, str]:
        if torque_limit is None:
            torque_limit = TEST_CONFIG["expected_torque_limit"]
        if goal is None:
            goal = target
        raw = self.send(
            f"OBS {telemetry_valid} {driver_error} {status} {torque_enabled} "
            f"{torque_limit} {goal} {position} {velocity} {current} {temperature} "
            f"{voltage} {elapsed_ms} {target}"
        )
        return dict(part.split("=", 1) for part in raw.split() if "=" in part)

    def close(self) -> None:
        if self.process.stdin:
            self.process.stdin.close()
        self.process.wait(timeout=10)


@unittest.skipIf(shutil.which("g++") is None, "g++ is required to build the harness")
class TestContactDetector(unittest.TestCase):
    tempdir: tempfile.TemporaryDirectory
    binary: Path

    @classmethod
    def setUpClass(cls) -> None:
        cls.tempdir = tempfile.TemporaryDirectory()
        cls.binary = Path(cls.tempdir.name) / "flc_detector_harness"
        result = subprocess.run(
            [
                "g++", "-std=c++17", "-O0", "-Wall", "-Wextra", "-Werror",
                "-I", str(SKETCH_DIR), str(HARNESS_SRC), "-o", str(cls.binary),
            ],
            capture_output=True,
            text=True,
        )
        if result.returncode != 0:
            raise AssertionError(f"harness build failed:\n{result.stderr}")

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tempdir.cleanup()

    def setUp(self) -> None:
        self.h = DetectorHarness(self.binary)
        self.addCleanup(self.h.close)
        self.h.configure()
        self.h.baseline()

    # -- encoder domain ----------------------------------------------------
    def test_tick_math_matches_canonical_matdog_joint_math(self):
        """The firmware's tick math must agree with matdog_joint_math exactly.

        This is the regression guard for the C++ truncating-modulo bug: a bare
        '%' on a negative numerator yields -4095 where Python yields +1.
        """
        vectors = [
            (0, 4095), (1, 4095), (4095, 0), (4094, 1), (1200, 1000),
            (1000, 1200), (2048, 2048), (0, 0), (2047, 4095), (100, 4000),
        ]
        for present, reference in vectors:
            with self.subTest(present=present, reference=reference):
                out = self.h.send(f"DELTA {present} {reference}")
                self.assertEqual(
                    int(out.split("=")[1]), signed_tick_delta(present, reference)
                )

    def test_plan_crossing_wrap_boundary_is_refused(self):
        """Approach toward the 0/4095 boundary must be refused, not wrapped."""
        self.assertEqual(self.h.send("DOMAIN 2048 600 1"), "IN_DOMAIN=1")
        self.assertEqual(self.h.send("DOMAIN 4000 200 1"), "IN_DOMAIN=0")
        self.assertEqual(self.h.send("DOMAIN 100 200 -1"), "IN_DOMAIN=0")
        self.assertEqual(self.h.send("DOMAIN 0 1 -1"), "IN_DOMAIN=0")
        self.assertEqual(self.h.send("DOMAIN 4095 1 1"), "IN_DOMAIN=0")

    def test_position_outside_domain_hard_aborts(self):
        self.h.init(2048)
        result = self.h.observe(position=5000, target=2300)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "POSITION_OUT_OF_DOMAIN")

    def test_commanded_target_outside_domain_hard_aborts(self):
        self.h.init(2048)
        result = self.h.observe(position=2048, target=4096)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "POSITION_OUT_OF_DOMAIN")

    # -- happy path --------------------------------------------------------
    def _drive_to_contact(self, contact_at: int = 2120, target: int = 2300):
        """Free motion out to `contact_at`, then stall there."""
        self.h.init(2048)
        states = []
        position = 2048
        while position < contact_at:
            position = min(position + 20, contact_at)
            states.append(self.h.observe(position=position, target=target, velocity=200))
        for _ in range(6):
            states.append(self.h.observe(position=contact_at, target=target, velocity=0))
        return states

    def test_persistent_stall_short_of_target_is_contact(self):
        states = self._drive_to_contact()
        self.assertEqual(states[-1]["STATE"], "CONTACT_CONFIRMED")
        self.assertEqual(states[-1]["CONTACT_TICK"], "2120")

    def test_contact_requires_persistence_not_one_sample(self):
        """A single stalled sample must never be reported as contact."""
        self.h.init(2048)
        position = 2048
        for _ in range(6):
            position += 20
            self.h.observe(position=position, target=2300, velocity=200)
        first = self.h.observe(position=position, target=2300, velocity=0)
        self.assertEqual(first["STATE"], "CONFIRMING")
        self.assertEqual(first["CONFIRMING"], "1")

    def test_reaching_the_commanded_target_is_not_contact(self):
        """Arriving where told is free motion, never an endstop."""
        self.h.init(2048)
        for _ in range(8):
            result = self.h.observe(position=2300, target=2300, velocity=0)
        self.assertEqual(result["STATE"], "FREE_MOTION")

    # -- section 13 fault modes -------------------------------------------
    def test_contact_never_occurs_exceeds_travel_budget(self):
        self.h.init(2048)
        result = self.h.observe(position=2700, target=2800, velocity=200)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "TRAVEL_BUDGET_EXCEEDED")

    def test_contact_never_occurs_exceeds_time_budget(self):
        self.h.init(2048)
        result = self.h.observe(position=2100, target=2300, velocity=200, elapsed_ms=25000)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "TIME_BUDGET_EXCEEDED")

    def test_contact_too_early_is_rejected(self):
        """Stalling before minimum credible travel is an obstruction, not an endstop."""
        self.h.init(2048)
        for _ in range(4):
            self.h.observe(position=2058, target=2300, velocity=200)
        result = None
        for _ in range(6):
            result = self.h.observe(position=2058, target=2300, velocity=0)
            if result["STATE"] == "HARD_ABORT":
                break
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "CONTACT_TOO_EARLY")

    def test_target_joint_does_not_move_at_all(self):
        self.h.init(2048)
        result = None
        for _ in range(12):
            result = self.h.observe(position=2048, target=2300, velocity=0)
            if result["STATE"] == "HARD_ABORT":
                break
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "TARGET_DID_NOT_MOVE")

    def test_motion_in_the_wrong_direction_hard_aborts(self):
        self.h.init(2048)
        result = self.h.observe(position=2000, target=2300, velocity=-200)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "WRONG_DIRECTION")

    def test_invalid_probe_sign_is_refused(self):
        self.h.configure(probe_sign=0)
        self.h.init(2048)
        result = self.h.observe(position=2100, target=2300)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "WRONG_DIRECTION")

    def test_current_spike_without_stall_is_not_contact(self):
        """A current transient while still moving must not become contact."""
        self.h.init(2048)
        position = 2048
        for _ in range(6):
            position += 20
            self.h.observe(position=position, target=2300, velocity=200)
        result = self.h.observe(position=position + 20, target=2300, velocity=200, current=150)
        self.assertEqual(result["STATE"], "FREE_MOTION")

    def test_hard_overcurrent_aborts(self):
        self.h.init(2048)
        result = self.h.observe(position=2100, target=2300, velocity=200, current=250)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "OVERCURRENT")

    def test_thermal_guard(self):
        self.h.init(2048)
        result = self.h.observe(position=2100, target=2300, temperature=70)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "THERMAL")

    def test_voltage_guard_low_and_high(self):
        for voltage in (39, 141):
            with self.subTest(voltage=voltage):
                self.h.init(2048)
                result = self.h.observe(position=2100, target=2300, voltage=voltage)
                self.assertEqual(result["STATE"], "HARD_ABORT")
                self.assertEqual(result["REASON"], "VOLTAGE")

    def test_telemetry_timeout(self):
        self.h.init(2048)
        result = self.h.observe(position=2100, target=2300, telemetry_valid=0)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "TELEMETRY_TIMEOUT")

    def test_bus_read_failure_surfaces_as_driver_error(self):
        self.h.init(2048)
        result = self.h.observe(position=2100, target=2300, driver_error=1)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "DRIVER_ERROR")

    def test_servo_status_error_aborts(self):
        self.h.init(2048)
        result = self.h.observe(position=2100, target=2300, status=0x20)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "STATUS_ERROR")

    def test_torque_dropping_off_mid_approach_aborts(self):
        self.h.init(2048)
        result = self.h.observe(position=2100, target=2300, torque_enabled=0)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "TORQUE_UNEXPECTEDLY_OFF")

    def test_torque_limit_drift_aborts(self):
        """A TorqueLimit that is not the commanded one invalidates the approach."""
        self.h.init(2048)
        result = self.h.observe(position=2100, target=2300, torque_limit=1000)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "TORQUE_LIMIT_MISMATCH")

    def test_goal_mismatch_aborts(self):
        """The servo's GoalPosition must equal what the firmware commanded."""
        self.h.init(2048)
        result = self.h.observe(position=2100, target=2300, goal=2500)
        self.assertEqual(result["STATE"], "HARD_ABORT")
        self.assertEqual(result["REASON"], "GOAL_MISMATCH")

    # -- circular tick statistics used by the manual-q0 capture ------------
    def _summary(self, ticks: list[int]) -> dict[str, int]:
        raw = self.h.send("SUMMARY " + " ".join(str(t) for t in ticks))
        return {k: int(v) for k, v in (p.split("=", 1) for p in raw.split())}

    def test_summary_of_a_normal_cloud(self):
        out = self._summary([2050, 2051, 2052, 2051, 2050])
        self.assertEqual(out["CENTRE"], 2050)
        self.assertEqual(out["MIN"], 2050)
        self.assertEqual(out["MAX"], 2052)
        self.assertEqual(out["SPREAD"], 2)

    def test_summary_across_the_wrap_boundary(self):
        """Regression guard: a linear mean here would give ~2047 and spread 4095."""
        out = self._summary([4094, 4095, 0, 1, 2])
        self.assertEqual(out["SPREAD"], 4)
        self.assertEqual(out["MIN"], 4094)
        self.assertEqual(out["MAX"], 2)
        self.assertEqual(out["CENTRE"], 0)

    def test_summary_matches_canonical_python_spread(self):
        from matdog_joint_math import circular_tick_summary

        for samples in (
            [2050, 2051, 2052],
            [4094, 4095, 0, 1, 2],
            [4095, 0, 1, 2],
            [100, 101, 102, 103],
        ):
            with self.subTest(samples=samples):
                _, expected_spread = circular_tick_summary(samples)
                self.assertEqual(self._summary(samples)["SPREAD"], expected_spread)

    def test_summary_of_a_single_sample(self):
        out = self._summary([4079])
        self.assertEqual(out["CENTRE"], 4079)
        self.assertEqual(out["SPREAD"], 0)

    def test_summary_rejects_empty_input(self):
        self.assertEqual(self._summary([])["VALID"], 0)

    # -- repeatability -----------------------------------------------------
    def test_repeatable_contacts_accepted_and_midpointed(self):
        out = dict(p.split("=", 1) for p in self.h.send("REPEAT 2400 2406 16").split())
        self.assertEqual(out["ACCEPTED"], "1")
        self.assertEqual(out["SPREAD"], "6")
        self.assertEqual(out["CONTACT_TICK"], "2403")

    def test_inconsistent_repeated_contacts_rejected(self):
        out = dict(p.split("=", 1) for p in self.h.send("REPEAT 2400 2460 16").split())
        self.assertEqual(out["ACCEPTED"], "0")
        self.assertEqual(out["SPREAD"], "60")

    def test_repeatability_midpoint_is_circular(self):
        out = dict(p.split("=", 1) for p in self.h.send("REPEAT 4094 2 16").split())
        self.assertEqual(out["SPREAD"], "4")
        self.assertEqual(out["CONTACT_TICK"], "0")

    # -- negative probe direction -----------------------------------------
    def test_engine_is_generic_over_probe_direction(self):
        """The same engine must handle a decreasing-raw joint with no special case."""
        self.h.configure(probe_sign=-1)
        self.h.baseline()
        self.h.init(2048)
        position = 2048
        while position > 1928:
            position = max(position - 20, 1928)
            self.h.observe(position=position, target=1800, velocity=-200)
        result = None
        for _ in range(6):
            result = self.h.observe(position=1928, target=1800, velocity=0)
        self.assertEqual(result["STATE"], "CONTACT_CONFIRMED")
        self.assertEqual(result["CONTACT_TICK"], "1928")


if __name__ == "__main__":
    unittest.main(verbosity=2)
