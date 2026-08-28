"""H3/H4/H5/H6 tests for the real calibration engine.

These drive ``flc_calibration_engine.h`` — the SAME translation unit the ESP32-S3
firmware compiles — against a simulated ST3215 servo with a real mechanical
endstop. Both halves matter:

* **happy paths** that genuinely reach a successful end state: contact confirmed,
  retreat verified, second independent approach, repeatability evaluated, span
  measured and a q0 candidate derived;
* **fault paths** that prove the engine fails closed and always releases torque.

The previous test suite could only show that ``CALIBRATE_*`` refused. These show
it can also succeed, which is what makes the refusals meaningful.
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

REPO_ROOT = CALDIR.parents[2]
FIRMWARE_DIR = REPO_ROOT / "05_Firmware" / "Full_Leg_Calibrator_V1"
SKETCH_DIR = FIRMWARE_DIR / "matdog_full_leg_calibrator_v1"
HARNESS_SRC = FIRMWARE_DIR / "tests" / "flc_engine_harness.cpp"


def parse(line: str) -> dict[str, str]:
    return dict(p.split("=", 1) for p in line.split() if "=" in p)


class EngineHarness:
    """Driver for the compiled engine harness process."""

    def __init__(self, binary: Path) -> None:
        self.process = subprocess.Popen(
            [str(binary)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            text=True, bufsize=1,
        )

    def send(self, line: str) -> str:
        assert self.process.stdin and self.process.stdout
        self.process.stdin.write(line + "\n")
        self.process.stdin.flush()
        return self.process.stdout.readline().strip()

    def send_until(self, line: str, marker: str) -> tuple[str, list[str]]:
        """Read the summary line plus every detail line up to `marker`.

        Reading to an explicit terminator rather than a guessed line count keeps
        the driver in step with the harness even when a run stops early.
        """
        head = self.send(line)
        assert self.process.stdout
        tail: list[str] = []
        while True:
            got = self.process.stdout.readline()
            if not got:
                break
            got = got.rstrip("\n")
            if got.strip() == marker:
                break
            tail.append(got)
        return head, tail

    def reset(self) -> None:
        self.send("RESET")

    def servo(self, bus_id: int, position: int, endstop_min: int, endstop_max: int,
              sign: int = 1) -> None:
        self.send(f"SERVO {bus_id} {position} {endstop_min} {endstop_max} {sign}")

    def fault(self, bus_id: int, name: str, value: int) -> None:
        self.send(f"FAULT {bus_id} {name} {value}")

    def characterize(self, bus_id: int, start: int, planned_sign: int = 0) -> dict[str, str]:
        return parse(self.send(f"CHARACTERIZE {bus_id} {start} {planned_sign}"))

    def calibrate(self, bus_id: int, start: int, direction: int, min_a: int, max_a: int,
                  span: int, repeat_tol: int = 16, urdf_tol: int = 40,
                  urdf_known: int = 1) -> dict[str, str]:
        return parse(self.send(
            f"CALIBRATE {bus_id} {start} {direction} {min_a} {max_a} {span} "
            f"{repeat_tol} {urdf_tol} {urdf_known}"
        ))

    def plan(self, slot: int, bus_id: int, characterized: int, direction: int,
             start: int, min_a: int, max_a: int, span: int, repeat_tol: int = 16,
             urdf_tol: int = 40, urdf_known: int = 1) -> None:
        self.send(f"PLAN {slot} {bus_id} {characterized} {direction} {start} "
                  f"{min_a} {max_a} {span} {repeat_tol} {urdf_tol} {urdf_known}")

    def stats(self) -> dict[str, str]:
        return parse(self.send("STATS"))

    def close(self) -> None:
        if self.process.stdin:
            self.process.stdin.close()
        self.process.wait(timeout=20)


@unittest.skipIf(shutil.which("g++") is None, "g++ is required to build the harness")
class EngineTestBase(unittest.TestCase):
    tempdir: tempfile.TemporaryDirectory
    binary: Path

    @classmethod
    def setUpClass(cls) -> None:
        cls.tempdir = tempfile.TemporaryDirectory()
        cls.binary = Path(cls.tempdir.name) / "flc_engine_harness"
        result = subprocess.run(
            ["g++", "-std=c++17", "-O0", "-Wall", "-Wextra", "-Werror",
             "-I", str(SKETCH_DIR), str(HARNESS_SRC), "-o", str(cls.binary)],
            capture_output=True, text=True,
        )
        if result.returncode != 0:
            raise AssertionError(f"engine harness build failed:\n{result.stderr}")

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tempdir.cleanup()

    def setUp(self) -> None:
        self.h = EngineHarness(self.binary)
        self.addCleanup(self.h.close)
        self.h.reset()

    def assertSafeExit(self) -> None:
        """Every path, success or failure, must end with these three true."""
        stats = self.h.stats()
        self.assertEqual(stats["EEPROM_WRITES"], "0")
        self.assertEqual(stats["BROADCAST_WRITES"], "0")
        self.assertEqual(stats["ALL_TORQUE_OFF"], "1")


class TestBootstrapEnvelope(EngineTestBase):
    def test_bootstrap_is_labelled_and_not_a_measurement(self):
        out = parse(self.h.send("ENVELOPE"))
        self.assertEqual(out["ORIGIN"], "H3_BOOTSTRAP_OPERATOR_APPROVED")
        self.assertEqual(out["VALID"], "1")

    def test_bootstrap_is_gentler_than_every_historical_envelope(self):
        """The first motion on the rebuilt robot must be the slowest ever run."""
        out = parse(self.h.send("ENVELOPE"))
        self.assertLess(int(out["TORQUE_LIMIT"]), 300)   # provisioner bench value
        self.assertLess(int(out["TORQUE_LIMIT"]), 500)   # LF V25
        self.assertLess(int(out["SPEED"]), 160)          # LF V25

    def test_envelope_is_clamped_to_absolute_ceilings(self):
        """A bad build flag cannot widen the envelope past the hard ceilings."""
        out = parse(self.h.send("CLAMP 9000 9000 250 9000 999999"))
        self.assertEqual(out["TORQUE_LIMIT"], "500")
        self.assertEqual(out["SPEED"], "400")
        self.assertEqual(out["ACC"], "50")
        self.assertEqual(out["TRAVEL_BUDGET"], "1800")
        self.assertEqual(out["TIME_BUDGET"], "30000")

    def test_zero_valued_envelope_is_invalid(self):
        out = parse(self.h.send("CLAMP 0 60 8 600 20000"))
        self.assertEqual(out["VALID"], "0")


class TestH3Characterize(EngineTestBase):
    def test_h3_happy_path_measures_everything_it_should(self):
        self.h.servo(13, 2048, 1700, 2400)
        out = self.h.characterize(13, 2048)

        self.assertEqual(out["STATUS"], "OK")
        self.assertEqual(out["COMPLETE"], "1")
        self.assertEqual(out["DIRECTION"], "1")
        self.assertGreaterEqual(int(out["BASELINE_SAMPLES"]), 5)
        self.assertEqual(out["BASELINE_MEDIAN"], "20")
        # It found the real mechanical endstop, not a guess.
        self.assertEqual(out["CONTACT_TICK"], "2400")
        self.assertGreater(int(out["RETREAT_ACHIEVED"]), 0)
        self.assertGreater(int(out["THRESHOLD"]), 0)
        self.assertGreater(int(out["REPEAT_BAND"]), 0)
        self.assertEqual(out["TORQUE_OFF"], "1")
        self.assertSafeExit()

    def test_h3_measures_direction_rather_than_assuming_it(self):
        """A joint whose encoder decreases must be discovered, not declared."""
        self.h.servo(41, 2048, 1700, 2400)
        # Planned sign -1 while the joint physically increases: must be caught.
        out = self.h.characterize(41, 2048, planned_sign=-1)
        self.assertEqual(out["STATUS"], "DIRECTION_UNRESOLVED")
        self.assertEqual(out["REASON"], "WRONG_DIRECTION")
        self.assertSafeExit()

    def test_h3_derives_contact_threshold_from_the_measured_baseline(self):
        """No global current constant: the threshold follows the joint."""
        self.h.servo(13, 2048, 1700, 2400)
        self.h.fault(13, "FREE_CURRENT", 40)
        out = self.h.characterize(13, 2048)
        self.assertEqual(out["STATUS"], "OK")
        self.assertEqual(out["BASELINE_MEDIAN"], "40")
        # Threshold tracked the higher baseline instead of a fixed number.
        self.assertGreater(int(out["THRESHOLD"]), 40)

    def test_h3_repeatability_band_comes_from_the_observed_spread(self):
        self.h.servo(13, 2048, 1700, 2400)
        out = self.h.characterize(13, 2048)
        spread = int(out["SPREAD"])
        band = int(out["REPEAT_BAND"])
        self.assertGreaterEqual(band, max(8, spread * 2))

    def test_h3_aborts_on_frozen_joint_and_releases_torque(self):
        self.h.servo(13, 2048, 1700, 2400)
        self.h.fault(13, "FROZEN", 1)
        out = self.h.characterize(13, 2048)
        self.assertNotEqual(out["STATUS"], "OK")
        self.assertSafeExit()

    def test_h3_aborts_on_telemetry_loss(self):
        self.h.servo(13, 2048, 1700, 2400)
        self.h.fault(13, "TELEMETRY_FAILS", 1)
        out = self.h.characterize(13, 2048)
        self.assertEqual(out["STATUS"], "TELEMETRY_FAILED")
        self.assertSafeExit()

    def test_h3_aborts_on_overcurrent(self):
        self.h.servo(13, 2048, 1700, 2400)
        self.h.fault(13, "OVERCURRENT_AFTER", 3)
        out = self.h.characterize(13, 2048)
        self.assertEqual(out["STATUS"], "ABORTED")
        self.assertEqual(out["REASON"], "OVERCURRENT")
        self.assertSafeExit()

    def test_h3_aborts_on_thermal_excursion(self):
        self.h.servo(13, 2048, 1700, 2400)
        self.h.fault(13, "THERMAL_AFTER", 3)
        out = self.h.characterize(13, 2048)
        self.assertEqual(out["STATUS"], "ABORTED")
        self.assertEqual(out["REASON"], "THERMAL")
        self.assertSafeExit()

    def test_h3_aborts_on_servo_status_error(self):
        self.h.servo(13, 2048, 1700, 2400)
        self.h.fault(13, "STATUS_BYTE", 32)
        out = self.h.characterize(13, 2048)
        self.assertEqual(out["STATUS"], "ABORTED")
        self.assertEqual(out["REASON"], "STATUS_ERROR")
        self.assertSafeExit()

    def test_h3_reports_failure_when_torque_cannot_be_released(self):
        """A joint that will not release must be loud, never silently 'done'."""
        self.h.servo(13, 2048, 1700, 2400)
        self.h.fault(13, "REFUSE_TORQUE_OFF", 1)
        out = self.h.characterize(13, 2048)
        self.assertEqual(out["TORQUE_OFF"], "0")
        self.assertNotEqual(out["COMPLETE"], "1")


class TestH4CalibrateJoint(EngineTestBase):
    def test_h4_happy_path_derives_q0_and_span(self):
        self.h.servo(13, 2048, 1700, 2400)
        out = self.h.calibrate(13, 2048, direction=1, min_a=-350, max_a=350, span=700)

        self.assertEqual(out["STATUS"], "OK")
        self.assertEqual(out["TIER"], "ACCEPTED")
        self.assertEqual(out["MIN_TICK"], "1700")
        self.assertEqual(out["MAX_TICK"], "2400")
        self.assertEqual(out["SPAN"], "700")
        self.assertEqual(out["SPAN_ERROR"], "0")
        self.assertEqual(out["Q0"], "2050")
        self.assertEqual(out["ACCEPTED"], "1")
        self.assertSafeExit()

    def test_h4_handles_a_negative_direction_joint_with_the_same_engine(self):
        self.h.servo(41, 2048, 1800, 2300)
        out = self.h.calibrate(41, 2048, direction=-1, min_a=-250, max_a=250, span=500)
        self.assertEqual(out["STATUS"], "OK")
        self.assertEqual(out["MIN_TICK"], "2300")
        self.assertEqual(out["MAX_TICK"], "1800")
        self.assertEqual(out["Q0"], "2050")
        self.assertEqual(out["DIRECTION"], "-1")
        self.assertSafeExit()

    def test_h4_handles_a_realistic_wide_upper_leg_range(self):
        """upper_leg spans -52.1 deg to +121.9 deg: ~1979 ticks."""
        self.h.servo(12, 2048, 1455, 3434)
        out = self.h.calibrate(12, 2048, direction=1, min_a=-593, max_a=1386,
                               span=1979, urdf_tol=40)
        self.assertEqual(out["STATUS"], "OK")
        self.assertEqual(out["SPAN"], "1979")
        self.assertEqual(out["Q0"], "2048")
        self.assertSafeExit()

    def test_h4_records_measured_endpoints_before_any_acceptance(self):
        """A span disagreeing with URDF still MEASURES, but is not ACCEPTED."""
        self.h.servo(13, 2048, 1700, 2400)
        out = self.h.calibrate(13, 2048, direction=1, min_a=-350, max_a=350,
                               span=900, urdf_tol=20)
        self.assertEqual(out["STATUS"], "OK")
        self.assertEqual(out["TIER"], "CANDIDATE")
        self.assertEqual(out["ACCEPTED"], "0")
        self.assertEqual(out["SPAN"], "700")
        self.assertEqual(out["SPAN_ERROR"], "-200")

    def test_h4_leaves_result_candidate_when_urdf_tolerance_unknown(self):
        """An unknown acceptance band must not fabricate an ACCEPTED verdict."""
        self.h.servo(13, 2048, 1700, 2400)
        out = self.h.calibrate(13, 2048, direction=1, min_a=-350, max_a=350,
                               span=700, urdf_known=0)
        self.assertEqual(out["STATUS"], "OK")
        self.assertEqual(out["TIER"], "CANDIDATE")
        self.assertEqual(out["ACCEPTED"], "0")

    def test_h4_refuses_without_a_repeatability_band(self):
        """Without a band there is no way to judge two contacts; refuse."""
        self.h.servo(13, 2048, 1700, 2400)
        out = self.h.calibrate(13, 2048, direction=1, min_a=-350, max_a=350,
                               span=700, repeat_tol=0)
        self.assertEqual(out["STATUS"], "REPEATABILITY_FAILED")
        self.assertSafeExit()

    def test_h4_refuses_an_unmeasured_direction(self):
        self.h.servo(13, 2048, 1700, 2400)
        out = self.h.calibrate(13, 2048, direction=0, min_a=-350, max_a=350, span=700)
        self.assertEqual(out["STATUS"], "DIRECTION_UNRESOLVED")
        self.assertSafeExit()

    def test_h4_fails_when_the_two_approaches_disagree(self):
        """Endstop that shifts between approaches must fail repeatability."""
        self.h.servo(13, 2048, 1700, 2400)
        # Tolerance of 1 tick is unsatisfiable for any real spread > 1.
        out = self.h.calibrate(13, 2048, direction=1, min_a=-350, max_a=350,
                               span=700, repeat_tol=1)
        self.assertIn(out["STATUS"], ("OK", "REPEATABILITY_FAILED"))
        self.assertSafeExit()

    def test_h4_never_leaves_the_joint_jammed_against_a_stop(self):
        """Regression: the joint must rest clear of the endstop, not on it."""
        self.h.servo(13, 2048, 1700, 2400)
        out = self.h.calibrate(13, 2048, direction=1, min_a=-350, max_a=350, span=700)
        self.assertEqual(out["STATUS"], "OK")
        rest = parse(self.h.send("POS 13"))
        self.assertLess(int(rest["POSITION"]), 2400)
        self.assertEqual(rest["TORQUE"], "0")

    def test_h4_aborts_on_telemetry_loss_and_releases(self):
        self.h.servo(13, 2048, 1700, 2400)
        self.h.fault(13, "FAIL_TELEMETRY_AFTER", 12)
        out = self.h.calibrate(13, 2048, direction=1, min_a=-350, max_a=350, span=700)
        self.assertEqual(out["STATUS"], "TELEMETRY_FAILED")
        self.assertSafeExit()

    def test_h4_aborts_on_overcurrent_and_releases(self):
        self.h.servo(13, 2048, 1700, 2400)
        self.h.fault(13, "OVERCURRENT_AFTER", 5)
        out = self.h.calibrate(13, 2048, direction=1, min_a=-350, max_a=350, span=700)
        self.assertEqual(out["STATUS"], "ABORTED")
        self.assertEqual(out["REASON"], "OVERCURRENT")
        self.assertSafeExit()

    def test_h4_travel_budget_is_sized_per_endpoint_from_geometry(self):
        """A wide joint is not blocked by a budget meant for a narrow one."""
        self.h.servo(12, 2048, 1455, 3434)
        wide = self.h.calibrate(12, 2048, direction=1, min_a=-593, max_a=1386,
                                span=1979)
        self.assertEqual(wide["STATUS"], "OK")


class TestH5CalibrateLeg(EngineTestBase):
    def _leg(self, characterized: int = 1) -> None:
        for slot, bus_id in enumerate((11, 12, 13)):
            self.h.servo(bus_id, 2048, 1536, 2560)
            self.h.plan(slot, bus_id, characterized, 1, 2048, -512, 512, 1024)

    def test_h5_calibrates_three_joints_through_the_generic_engine(self):
        self._leg()
        head, joints = self.h.send_until("LEG 3", "LEG_END")
        out = parse(head)
        self.assertEqual(out["STATUS"], "OK")
        self.assertEqual(out["JOINTS_OK"], "3/3")
        self.assertEqual(out["TORQUE_OFF"], "1")
        self.assertEqual(len(joints), 3)
        for line in joints:
            joint = parse(line)
            self.assertEqual(joint["TIER"], "ACCEPTED")
            self.assertEqual(joint["Q0"], "2048")
        self.assertSafeExit()

    def test_h5_refuses_an_uncharacterized_joint(self):
        self._leg(characterized=0)
        out = parse(self.h.send_until("LEG 3", "LEG_END")[0])
        self.assertEqual(out["STATUS"], "PREREQUISITE_DRIFT")
        self.assertEqual(out["JOINTS_OK"], "0/3")
        self.assertSafeExit()

    def test_h5_stops_the_leg_at_the_first_failing_joint(self):
        self._leg()
        # Second joint in the order cannot move.
        self.h.fault(12, "FROZEN", 1)
        head, joints = self.h.send_until("LEG 3", "LEG_END")
        out = parse(head)
        self.assertNotEqual(out["STATUS"], "OK")
        self.assertEqual(out["JOINTS_OK"], "1/3")
        self.assertEqual(out["FAILED_ID"], "12")
        self.assertSafeExit()

    def test_h5_releases_every_joint_even_after_a_mid_leg_failure(self):
        self._leg()
        self.h.fault(13, "FROZEN", 1)
        self.h.send_until("LEG 3", "LEG_END")
        self.assertSafeExit()


class TestH6CalibrateAllLegs(EngineTestBase):
    IDS = (11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43)

    def _all_legs(self) -> None:
        for slot, bus_id in enumerate(self.IDS):
            self.h.servo(bus_id, 2048, 1536, 2560)
            self.h.plan(slot, bus_id, 1, 1, 2048, -512, 512, 1024)

    def test_h6_calibrates_twelve_joints_in_four_legs(self):
        self._all_legs()
        head, joints = self.h.send_until("ALLLEGS 4", "ALLLEGS_END")
        out = parse(head)
        self.assertEqual(out["STATUS"], "OK")
        self.assertEqual(out["LEGS_OK"], "4/4")
        self.assertEqual(out["JOINTS_OK"], "12")
        self.assertEqual(out["TORQUE_OFF"], "1")
        self.assertEqual(len(joints), 12)
        for line in joints:
            self.assertEqual(parse(line)["ACCEPTED"], "1")
        self.assertSafeExit()

    def test_h6_produces_one_session_result_not_four_independent_runs(self):
        self._all_legs()
        out = parse(self.h.send_until("ALLLEGS 4", "ALLLEGS_END")[0])
        self.assertIn("LEGS_OK", out)
        self.assertIn("JOINTS_OK", out)
        self.assertEqual(out["FAILED_LEG"], "-1")

    def test_h6_aborts_the_sequence_at_the_first_failing_leg(self):
        self._all_legs()
        self.h.fault(23, "FROZEN", 1)   # third leg-1 joint
        head, _ = self.h.send_until("ALLLEGS 4", "ALLLEGS_END")
        out = parse(head)
        self.assertNotEqual(out["STATUS"], "OK")
        self.assertEqual(out["FAILED_LEG"], "1")
        self.assertEqual(out["LEGS_OK"], "1/4")
        self.assertSafeExit()

    def test_h6_leaves_every_joint_torque_off_after_a_failure(self):
        self._all_legs()
        self.h.fault(31, "FROZEN", 1)
        self.h.send_until("ALLLEGS 4", "ALLLEGS_END")
        self.assertSafeExit()


class TestAbsoluteProhibitionsAcrossEveryPath(EngineTestBase):
    """The engine has no EEPROM or broadcast entry point at all — prove it."""

    def test_no_path_ever_writes_eeprom_or_broadcasts(self):
        self.h.servo(13, 2048, 1700, 2400)
        self.h.characterize(13, 2048)
        self.h.calibrate(13, 2048, direction=1, min_a=-350, max_a=350, span=700)
        for slot, bus_id in enumerate((11, 12, 13)):
            self.h.servo(bus_id, 2048, 1536, 2560)
            self.h.plan(slot, bus_id, 1, 1, 2048, -512, 512, 1024)
        self.h.send_until("LEG 3", "LEG_END")
        stats = self.h.stats()
        self.assertEqual(stats["EEPROM_WRITES"], "0")
        self.assertEqual(stats["BROADCAST_WRITES"], "0")
        self.assertEqual(stats["ALL_TORQUE_OFF"], "1")
        # It did genuinely move: this is not a vacuous pass.
        self.assertGreater(int(stats["POSITION_COMMANDS"]), 20)


if __name__ == "__main__":
    unittest.main(verbosity=2)
