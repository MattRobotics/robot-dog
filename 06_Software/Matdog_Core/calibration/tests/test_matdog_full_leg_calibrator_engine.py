"""H3/H4/H5/H6 tests for the real calibration engine.

These drive ``flc_calibration_engine.h`` — the SAME translation unit the ESP32-S3
firmware compiles — against a simulated ST3215 servo with a real mechanical
endstop. Both halves matter:

* **happy paths** that genuinely reach a successful end state: contact confirmed,
  retreat verified, second independent approach, repeatability evaluated, span
  measured, direction resolved and a q0 candidate derived;
* **fault paths** that prove the engine fails closed and always releases torque.

H4, H5 and H6 are deliberately NOT separate entry points. All three build the
same 12-row plan catalog and call the same ``flcRunCalibrationPlan``; they differ
only in their selection mask. That is exactly what the firmware does, so a test
passing here is a statement about the code that would move the robot.
"""

from __future__ import annotations

import math
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

TICKS_PER_RADIAN = 4096.0 / (2.0 * math.pi)


def angle_ticks(rad: float) -> int:
    """Same rounding the firmware's angleToTicks() uses."""
    ticks = rad * TICKS_PER_RADIAN
    return int(ticks + 0.5) if ticks >= 0 else int(ticks - 0.5)


# Slot order is the generated FlcGeometryJoint order, which the firmware's
# JOINTS[] table matches row for row: LF, RF, RH, LH x (hip, upper, lower).
JOINT_LAYOUT = [
    (0, 13, "lf_hip_joint", -0.803055986689, 0.789284248),
    (1, 12, "lf_upper_leg_joint", -0.909889226, 2.127120026),
    (2, 11, "lf_lower_leg_joint", -1.606998273, 0.666361254),
    (3, 23, "rf_hip_joint", -0.789284248, 0.803055986689),
    (4, 22, "rf_upper_leg_joint", -0.909889226, 2.127120026),
    (5, 21, "rf_lower_leg_joint", -1.606998273, 0.666361254),
    (6, 33, "rh_hip_joint", -0.788125240, 0.803055986689),
    (7, 32, "rh_upper_leg_joint", -0.909889226, 2.127120026),
    (8, 31, "rh_lower_leg_joint", -1.606998273, 0.666361254),
    (9, 43, "lh_hip_joint", -0.803055986689, 0.788125240),
    (10, 42, "lh_upper_leg_joint", -0.909889226, 2.127120026),
    (11, 41, "lh_lower_leg_joint", -1.606998273, 0.666361254),
]

LEG_MASKS = {"LF": 0x007, "RF": 0x038, "RH": 0x1C0, "LH": 0xE00}
ALL_JOINTS_MASK = 0xFFF

# The six endpoints the Geometry Compiler V5 plan marks PARKING_REQUIRED_1DOF,
# with the auxiliary joint each one depends on. Asymmetric on purpose: LF and RF
# upper:max need parking while RH and LH upper:max do not.
PARKING_ENDPOINTS = {
    3: ("lf_upper_leg_joint:max", 1, 10),
    4: ("lf_lower_leg_joint:min", 2, 1),
    9: ("rf_upper_leg_joint:max", 4, 7),
    10: ("rf_lower_leg_joint:min", 5, 4),
    16: ("rh_lower_leg_joint:min", 8, 7),
    22: ("lh_lower_leg_joint:min", 11, 10),
}


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

    def context(self, valid: bool) -> None:
        self.send(f"CONTEXT {1 if valid else 0}")

    def characterize(self, bus_id: int, start: int, planned_sign: int = 1) -> dict[str, str]:
        return parse(self.send(f"CHARACTERIZE {bus_id} {start} {planned_sign}"))

    def plan(self, slot: int, bus_id: int, *, characterized: int = 1,
             start: int = 2048, q0_watch: int = 2048, q0_tol: int = 12,
             witness: int = 1, q_min: int = -350, q_max: int = 350,
             span: int = 700, repeat_tol: int = 16, urdf_tol: int = 40,
             urdf_known: int = 1, manual_known: int = 1, manual_tick: int = 2048,
             xcheck_tol: int = 24, xcheck_known: int = 1, cal_known: int = 0,
             known_direction: int = 0, known_q0: int = -1) -> None:
        self.send(
            f"PLAN {slot} {bus_id} {characterized} {start} {q0_watch} {q0_tol} "
            f"{witness} {q_min} {q_max} {span} {repeat_tol} {urdf_tol} "
            f"{urdf_known} {manual_known} {manual_tick} {xcheck_tol} "
            f"{xcheck_known} {cal_known} {known_direction} {known_q0}"
        )

    def run(self, mask: int) -> tuple[dict[str, str], dict[int, dict[str, str]],
                                     list[int], dict[int, dict[str, str]]]:
        head, tail = self.send_until(f"RUN {mask:03X}", "RUN_END")
        summary = parse(head)
        joints: dict[int, dict[str, str]] = {}
        finals: dict[int, dict[str, str]] = {}
        order: list[int] = []
        for line in tail:
            stripped = line.strip()
            if stripped.startswith("JOINT "):
                row = parse(stripped)
                joints[int(row["SLOT"])] = row
            elif stripped.startswith("FINAL "):
                row = parse(stripped)
                finals[int(row["SLOT"])] = row
            elif stripped.startswith("ORDER "):
                order.append(int(stripped.split("=", 1)[1]))
        return summary, joints, order, finals

    def geometry(self) -> tuple[dict[str, str], list[dict[str, str]], list[int]]:
        head, tail = self.send_until("GEOMETRY", "GEOMETRY_END")
        rows: list[dict[str, str]] = []
        order: list[int] = []
        for line in tail:
            stripped = line.strip()
            if stripped.startswith("ROW "):
                rows.append(parse(stripped))
            elif stripped.startswith("ORDER "):
                order.append(int(stripped.split("=", 1)[1]))
        return parse(head), rows, order

    def stats(self) -> dict[str, str]:
        return parse(self.send("STATS"))

    def close(self) -> None:
        if self.process.stdin:
            self.process.stdin.close()
        self.process.wait(timeout=60)


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

    # ---------------------------------------------------------------- fixture

    def build_fixture(self, *, q0: int = 2048, directions: dict[int, int] | None = None,
                      seeded: set[int] | None = None,
                      characterized: set[int] | None = None,
                      urdf_known: int = 1, xcheck_known: int = 1,
                      manual_offsets: dict[int, int] | None = None) -> dict[int, dict]:
        """Seed all 12 servos and all 12 plan rows.

        The whole catalog is always built: ``flcRunCalibrationPlan`` needs the
        un-selected joints too, both to hold them at q0 and to express a
        cross-leg parking angle in raw ticks.
        """
        directions = directions or {}
        seeded = seeded if seeded is not None else set()
        characterized = characterized if characterized is not None else set(range(12))
        manual_offsets = manual_offsets or {}
        info: dict[int, dict] = {}

        for slot, bus_id, name, lo_rad, hi_rad in JOINT_LAYOUT:
            direction = directions.get(slot, 1)
            q_min = angle_ticks(lo_rad)
            q_max = angle_ticks(hi_rad)
            raw_a = q0 + direction * q_min
            raw_b = q0 + direction * q_max
            endstop_min, endstop_max = min(raw_a, raw_b), max(raw_a, raw_b)
            info[slot] = {
                "bus_id": bus_id, "name": name, "direction": direction,
                "q_min": q_min, "q_max": q_max, "span": q_max - q_min,
                "endstop_min": endstop_min, "endstop_max": endstop_max,
                "q0": q0,
                # A +1 joint reaches q_max by driving raw UP; a -1 joint reaches
                # the same kinematic limit by driving raw DOWN. Both are real
                # MATDOG mappings and both must go through this same engine.
                "raw_at_q_min": raw_a, "raw_at_q_max": raw_b,
            }
            self.h.servo(bus_id, q0, endstop_min, endstop_max, direction)

        for slot, bus_id, _, _, _ in JOINT_LAYOUT:
            meta = info[slot]
            self.h.plan(
                slot, bus_id,
                characterized=1 if slot in characterized else 0,
                start=q0, q0_watch=q0, q0_tol=12,
                witness=meta["direction"],
                q_min=meta["q_min"], q_max=meta["q_max"], span=meta["span"],
                repeat_tol=16, urdf_tol=40, urdf_known=urdf_known,
                manual_known=1, manual_tick=q0 + manual_offsets.get(slot, 0),
                xcheck_tol=24, xcheck_known=xcheck_known,
                cal_known=1 if slot in seeded else 0,
                known_direction=meta["direction"] if slot in seeded else 0,
                known_q0=q0 if slot in seeded else -1,
            )
        return info

    def assertSafeExit(self) -> None:
        """Every path, success or failure, must end with these three true."""
        stats = self.h.stats()
        self.assertEqual(stats["EEPROM_WRITES"], "0")
        self.assertEqual(stats["BROADCAST_WRITES"], "0")
        self.assertEqual(stats["ALL_TORQUE_OFF"], "1")


# ==========================================================================
# Envelope
# ==========================================================================

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


# ==========================================================================
# Generated geometry plan, as the production engine itself validates it
# ==========================================================================

class TestGeneratedGeometryPlan(EngineTestBase):
    def test_plan_is_24_rows_split_18_no_parking_and_6_parking(self):
        summary, rows, _ = self.h.geometry()
        self.assertEqual(summary["SUMMARY_OK"], "1")
        self.assertEqual(summary["ROWS"], "24")
        self.assertEqual(summary["NO_PARKING"], "18")
        self.assertEqual(summary["PARKING"], "6")
        self.assertEqual(len(rows), 24)

    def test_every_joint_contributes_exactly_two_endpoints(self):
        _, rows, _ = self.h.geometry()
        self.assertEqual(len({r["ID"] for r in rows}), 24)
        targets = [int(r["TARGET"]) for r in rows]
        for joint in range(12):
            self.assertEqual(targets.count(joint), 2, f"joint {joint}")

    def test_parking_rows_are_exactly_the_six_expected_and_asymmetric(self):
        _, rows, _ = self.h.geometry()
        parking = {r["ID"]: r for r in rows if r["PARKING"] == "1"}
        self.assertEqual(set(parking), {v[0] for v in PARKING_ENDPOINTS.values()})
        # Asymmetry is the point: the front upper legs need parking, the hind
        # ones do not. A symmetry assumption would silently break this.
        self.assertIn("lf_upper_leg_joint:max", parking)
        self.assertIn("rf_upper_leg_joint:max", parking)
        no_parking = {r["ID"] for r in rows if r["PARKING"] == "0"}
        self.assertIn("rh_upper_leg_joint:max", no_parking)
        self.assertIn("lh_upper_leg_joint:max", no_parking)

    def test_parking_dependencies_include_cross_leg_pairs(self):
        """LF's upper leg parks against LH's, not against its own leg."""
        _, rows, _ = self.h.geometry()
        by_id = {r["ID"]: r for r in rows}
        self.assertEqual(int(by_id["lf_upper_leg_joint:max"]["AUX"]), 10)  # LH upper
        self.assertEqual(int(by_id["rf_upper_leg_joint:max"]["AUX"]), 7)   # RH upper

    def test_no_parking_rows_carry_explicit_none_auxiliary_provenance(self):
        _, rows, _ = self.h.geometry()
        for row in rows:
            if row["PARKING"] == "0":
                self.assertEqual(row["AUX"], "255", row["ID"])

    def test_dependency_order_is_derived_not_assumed(self):
        summary, _, order = self.h.geometry()
        self.assertEqual(summary["ORDER_OK"], "1")
        self.assertEqual(len(order), 12)
        self.assertEqual(sorted(order), list(range(12)))

    def test_naive_distal_first_ordering_is_rejected_by_the_graph(self):
        """LOWER before UPPER would violate the real dependency edges."""
        _, _, order = self.h.geometry()
        position = {joint: i for i, joint in enumerate(order)}
        for lower, upper in ((2, 1), (5, 4), (8, 7), (11, 10)):
            self.assertLess(position[upper], position[lower],
                            f"upper {upper} must precede lower {lower}")

    def test_cross_leg_prerequisites_are_respected(self):
        _, _, order = self.h.geometry()
        position = {joint: i for i, joint in enumerate(order)}
        self.assertLess(position[10], position[1])  # LH upper before LF upper
        self.assertLess(position[7], position[4])   # RH upper before RF upper

    def test_an_injected_cycle_is_detected_offline(self):
        """The production Kahn walk must reject a cycle, not trust the table."""
        out = parse(self.h.send("CYCLE"))
        self.assertEqual(out["CYCLE_ACCEPTED"], "0")


# ==========================================================================
# H3 — characterize one joint
# ==========================================================================

class TestH3Characterize(EngineTestBase):
    def test_h3_happy_path_measures_everything_it_should(self):
        self.h.servo(13, 2048, 1700, 2400)
        out = self.h.characterize(13, 2048, planned_sign=1)

        self.assertEqual(out["STATUS"], "OK")
        self.assertEqual(out["COMPLETE"], "1")
        self.assertEqual(out["RESPONDS"], "1")
        self.assertGreaterEqual(int(out["BASELINE_SAMPLES"]), 5)
        self.assertEqual(out["BASELINE_MEDIAN"], "20")
        # It found the real mechanical endstop, not a guess.
        self.assertEqual(out["CONTACT_TICK"], "2400")
        self.assertGreater(int(out["RETREAT_ACHIEVED"]), 0)
        self.assertGreater(int(out["THRESHOLD"]), 0)
        self.assertGreater(int(out["REPEAT_BAND"]), 0)
        self.assertEqual(out["TORQUE_OFF"], "1")
        self.assertSafeExit()

    def test_h3_reports_raw_probe_sign_never_a_kinematic_direction(self):
        """H3 sees only one endpoint, so it cannot resolve q = d * delta."""
        self.h.servo(13, 2048, 1700, 2400)
        out = self.h.characterize(13, 2048, planned_sign=1)
        self.assertEqual(out["RAW_PROBE_SIGN"], "1")
        self.assertNotIn("DIRECTION", out)

    def test_h3_probes_the_side_it_was_told_to_probe(self):
        """An explicit sign is a geometry decision and is never reversed."""
        self.h.servo(41, 2048, 1700, 2400)
        out = self.h.characterize(41, 2048, planned_sign=-1)
        self.assertEqual(out["STATUS"], "OK")
        self.assertEqual(out["RAW_PROBE_SIGN"], "-1")
        self.assertEqual(out["CONTACT_TICK"], "1700")
        self.assertSafeExit()

    def test_h3_rejects_an_invalid_probe_sign(self):
        self.h.servo(13, 2048, 1700, 2400)
        out = self.h.characterize(13, 2048, planned_sign=7)
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

    def test_h3_aborts_on_lost_session_context(self):
        """A reset or reconnect mid-motion must stop the run, not finish it."""
        self.h.servo(13, 2048, 1700, 2400)
        self.h.fault(13, "CONTEXT_INVALID", 1)
        out = self.h.characterize(13, 2048)
        self.assertEqual(out["STATUS"], "IDENTITY_OR_SESSION_DRIFT")
        self.assertEqual(out["REASON"], "IDENTITY_OR_SESSION_DRIFT")
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


# ==========================================================================
# H4 — one joint, through the shared run
# ==========================================================================

class TestH4CalibrateJoint(EngineTestBase):
    def test_h4_happy_path_derives_q0_span_and_direction(self):
        info = self.build_fixture()
        summary, joints, order, finals = self.h.run(1 << 0)

        self.assertEqual(summary["STATUS"], "OK")
        self.assertEqual(summary["JOINTS_OK"], "1")
        self.assertEqual(summary["REQUESTED"], "1")
        self.assertEqual(order, [0])

        row = joints[0]
        self.assertEqual(row["TIER"], "ACCEPTED")
        self.assertEqual(int(row["MIN_TICK"]), info[0]["raw_at_q_min"])
        self.assertEqual(int(row["MAX_TICK"]), info[0]["raw_at_q_max"])
        self.assertEqual(int(row["SPAN"]), info[0]["span"])
        self.assertEqual(row["SPAN_ERROR"], "0")
        self.assertEqual(int(row["DERIVED_Q0"]), 2048)
        self.assertEqual(row["DIRECTION"], "1")
        self.assertEqual(row["ACCEPTED"], "1")
        self.assertEqual(finals[0]["USABLE"], "1")
        self.assertSafeExit()

    def test_h4_handles_the_opposite_raw_to_q_mapping_in_the_same_engine(self):
        """A -1 joint reaches q_max by driving raw DOWN. Same code path."""
        info = self.build_fixture(directions={0: -1})
        summary, joints, _, _ = self.h.run(1 << 0)

        self.assertEqual(summary["STATUS"], "OK")
        row = joints[0]
        self.assertEqual(row["DIRECTION"], "-1")
        # Kinematic max now sits BELOW q0 in raw ticks.
        self.assertLess(int(row["MAX_TICK"]), 2048)
        self.assertGreater(int(row["MIN_TICK"]), 2048)
        self.assertEqual(int(row["DERIVED_Q0"]), 2048)
        self.assertEqual(int(row["SPAN"]), info[0]["span"])
        self.assertSafeExit()

    def test_h4_handles_a_realistic_wide_upper_leg_range(self):
        """upper_leg spans -52.1 deg to +121.9 deg: ~1980 ticks."""
        info = self.build_fixture(seeded={10})
        summary, joints, _, _ = self.h.run(1 << 1)
        self.assertEqual(summary["STATUS"], "OK")
        self.assertEqual(int(joints[1]["SPAN"]), info[1]["span"])
        self.assertEqual(int(joints[1]["DERIVED_Q0"]), 2048)
        self.assertSafeExit()

    def test_h4_records_measured_endpoints_before_any_acceptance(self):
        """A span disagreeing with URDF still MEASURES, but is not ACCEPTED."""
        info = self.build_fixture()
        # Shrink the mechanical range so the measured span misses URDF badly.
        self.h.servo(13, 2048, 2048 - 200, 2048 + 200, 1)
        summary, joints, _, _ = self.h.run(1 << 0)
        row = joints[0]
        self.assertEqual(row["STATUS"], "GEOMETRY_INCONSISTENT")
        self.assertEqual(row["REASON"], "ENDPOINT_GEOMETRY_MISMATCH")
        # The measurement is still reported: refusing is not forgetting.
        self.assertEqual(int(row["MIN_TICK"]), 1848)
        self.assertEqual(int(row["MAX_TICK"]), 2248)
        self.assertEqual(row["ACCEPTED"], "0")
        self.assertNotEqual(row["TIER"], "ACCEPTED")
        del info
        self.assertSafeExit()

    def test_h4_leaves_result_candidate_when_urdf_tolerance_unknown(self):
        """An unvalidated tolerance may never be resolved into ACCEPTED."""
        self.build_fixture(urdf_known=0)
        summary, joints, _, _ = self.h.run(1 << 0)
        self.assertEqual(summary["STATUS"], "OK")
        self.assertEqual(joints[0]["TIER"], "CANDIDATE")
        self.assertEqual(joints[0]["ACCEPTED"], "0")
        self.assertSafeExit()

    def test_h4_blocks_when_the_q0_crosscheck_tolerance_is_unvalidated(self):
        self.build_fixture(xcheck_known=0)
        _, joints, _, _ = self.h.run(1 << 0)
        self.assertEqual(joints[0]["Q0_XCHECK"], "BLOCKED_TOLERANCE_UNVALIDATED")
        self.assertEqual(joints[0]["TIER"], "CANDIDATE")
        self.assertEqual(joints[0]["ACCEPTED"], "0")
        self.assertSafeExit()

    def test_h4_refuses_without_a_repeatability_band(self):
        self.build_fixture()
        self.h.plan(0, 13, q_min=-524, q_max=515, span=1039, repeat_tol=0)
        _, joints, _, _ = self.h.run(1 << 0)
        self.assertEqual(joints[0]["STATUS"], "REPEATABILITY_FAILED")
        self.assertSafeExit()

    def test_h4_refuses_without_a_semantic_direction_witness(self):
        """Direction must be declared from physical evidence, never inferred."""
        self.build_fixture()
        self.h.plan(0, 13, witness=0, q_min=-524, q_max=515, span=1039)
        _, joints, _, _ = self.h.run(1 << 0)
        self.assertEqual(joints[0]["STATUS"], "DIRECTION_UNRESOLVED")
        self.assertEqual(joints[0]["REASON"], "WRONG_DIRECTION")
        self.assertSafeExit()

    def test_h4_refuses_an_uncharacterized_joint(self):
        self.build_fixture(characterized=set(range(12)) - {0})
        summary, joints, _, _ = self.h.run(1 << 0)
        self.assertEqual(joints[0]["STATUS"], "PREREQUISITE_DRIFT")
        self.assertNotEqual(summary["STATUS"], "OK")
        self.assertSafeExit()

    def test_h4_detects_a_witness_that_contradicts_the_measurement(self):
        """Witness says +1 but the hardware maps the other way: fail closed.

        The upper leg is used deliberately: its limits are strongly asymmetric
        (-593 / +1387 ticks). Because the travel budget for each endpoint is
        sized from the geometry the witness implies, a contradictory witness is
        caught by the budget after ~593 ticks rather than after the 1387 the
        joint would really have to travel. The wrong direction is therefore
        stopped early by a motion guard, not merely rejected afterwards.

        A near-symmetric joint such as the hip cannot be discriminated this way,
        which is precisely why the semantic operator witness is required rather
        than optional.
        """
        info = self.build_fixture(directions={1: -1}, seeded={10})
        # Keep the -1 servo but declare the opposite witness.
        self.h.plan(1, 12, witness=1, q_min=info[1]["q_min"],
                    q_max=info[1]["q_max"], span=info[1]["span"])
        summary, joints, _, _ = self.h.run(1 << 1)
        self.assertEqual(joints[1]["STATUS"], "ABORTED")
        self.assertEqual(joints[1]["REASON"], "TRAVEL_BUDGET_EXCEEDED")
        self.assertEqual(joints[1]["ACCEPTED"], "0")
        self.assertEqual(joints[1]["TIER"], "MEASURED")
        # No q0 was derived from a contradicted direction.
        self.assertEqual(joints[1]["DERIVED_Q0"], "-1")
        self.assertEqual(joints[1]["Q0_XCHECK"], "NOT_RUN")
        self.assertEqual(summary["JOINTS_OK"], "0")
        self.assertSafeExit()

    def test_h4_escalates_when_telemetry_dies_and_torque_off_cannot_be_proven(self):
        """Transport acknowledgement is not proof: unverifiable means latched."""
        self.build_fixture()
        self.h.fault(13, "FAIL_TELEMETRY_AFTER", 12)
        summary, _, _, _ = self.h.run(1 << 0)
        self.assertEqual(summary["STATUS"], "TORQUE_OFF_FAILED")
        self.assertEqual(summary["CUT_POWER"], "1")
        self.assertEqual(summary["SAFE_OFF"], "0")

    def test_h4_reports_telemetry_loss_when_the_link_recovers_enough_to_release(self):
        self.build_fixture()
        # Fails during the run, then the servo answers again so flcEndMotion can
        # actually verify the release.
        self.h.fault(13, "TELEMETRY_FAILS", 1)
        summary, _, _, _ = self.h.run(1 << 0)
        self.assertNotEqual(summary["STATUS"], "OK")
        self.h.fault(13, "TELEMETRY_FAILS", 0)
        self.assertEqual(parse(self.h.send("POS 13"))["TORQUE"], "0")

    def test_h4_aborts_on_overcurrent_and_releases(self):
        self.build_fixture()
        self.h.fault(13, "OVERCURRENT_AFTER", 6)
        summary, _, _, _ = self.h.run(1 << 0)
        self.assertNotEqual(summary["STATUS"], "OK")
        self.assertSafeExit()

    def test_h4_never_leaves_the_joint_jammed_against_a_stop(self):
        info = self.build_fixture()
        self.h.run(1 << 0)
        out = parse(self.h.send("POS 13"))
        self.assertEqual(out["TORQUE"], "0")
        # Back at q0, not resting on either mechanical endstop.
        self.assertNotEqual(int(out["POSITION"]), info[0]["endstop_min"])
        self.assertNotEqual(int(out["POSITION"]), info[0]["endstop_max"])
        self.assertLessEqual(abs(int(out["POSITION"]) - 2048), 12)


# ==========================================================================
# Manual q0 and derived q0 stay two distinct concepts
# ==========================================================================

class TestQ0Integration(EngineTestBase):
    def test_manual_and_derived_q0_are_reported_separately(self):
        self.build_fixture()
        _, joints, _, _ = self.h.run(1 << 0)
        row = joints[0]
        self.assertIn("MANUAL_Q0", row)
        self.assertIn("DERIVED_Q0", row)
        self.assertEqual(row["Q0_XCHECK"], "MATCH")
        self.assertEqual(row["Q0_ERROR"], "0")

    def test_a_disagreeing_manual_q0_fails_closed_and_is_never_averaged(self):
        """The two values must not be blended into a plausible middle."""
        self.build_fixture(manual_offsets={0: 300})
        _, joints, _, _ = self.h.run(1 << 0)
        row = joints[0]
        self.assertEqual(row["STATUS"], "Q0_CROSSCHECK_FAILED")
        self.assertEqual(row["Q0_XCHECK"], "MISMATCH")
        self.assertEqual(int(row["MANUAL_Q0"]), 2348)
        self.assertEqual(int(row["DERIVED_Q0"]), 2048)
        # Neither value moved toward the other.
        self.assertNotEqual(int(row["DERIVED_Q0"]), 2198)
        self.assertEqual(row["ACCEPTED"], "0")
        self.assertSafeExit()

    def test_a_small_disagreement_within_tolerance_still_matches(self):
        self.build_fixture(manual_offsets={0: 10})
        _, joints, _, _ = self.h.run(1 << 0)
        self.assertEqual(joints[0]["Q0_XCHECK"], "MATCH")
        self.assertEqual(joints[0]["Q0_ERROR"], "10")
        self.assertEqual(joints[0]["TIER"], "ACCEPTED")

    def test_derived_q0_survives_the_opposite_direction_mapping(self):
        self.build_fixture(directions={0: -1})
        _, joints, _, _ = self.h.run(1 << 0)
        self.assertEqual(int(joints[0]["DERIVED_Q0"]), 2048)
        self.assertEqual(joints[0]["Q0_XCHECK"], "MATCH")


# ==========================================================================
# Parking and dependency orchestration
# ==========================================================================

class TestParkingOrchestration(EngineTestBase):
    def test_no_parking_endpoints_carry_explicit_provenance(self):
        self.build_fixture()
        _, joints, _, _ = self.h.run(1 << 0)
        row = joints[0]
        self.assertEqual(row["MIN_PARK"], "0")
        self.assertEqual(row["MAX_PARK"], "0")
        self.assertEqual(row["MIN_NOPARK_PROV"], "1")
        self.assertEqual(row["MAX_NOPARK_PROV"], "1")

    def test_a_parking_endpoint_executes_and_restores_transactionally(self):
        self.build_fixture(seeded={10})
        summary, joints, _, _ = self.h.run(1 << 1)   # LF upper: max needs LH upper
        self.assertEqual(summary["STATUS"], "OK")
        self.assertEqual(joints[1]["MAX_PARK"], "1")
        self.assertEqual(joints[1]["MAX_RESTORE"], "1")
        self.assertEqual(summary["PARKING_REQUIRED"], "1")
        self.assertEqual(summary["PARKING_RESTORED"], "1")
        self.assertSafeExit()

    def test_the_auxiliary_joint_is_returned_to_its_saved_position(self):
        self.build_fixture(seeded={10})
        self.h.run(1 << 1)
        out = parse(self.h.send("POS 42"))     # LH upper leg
        self.assertEqual(out["TORQUE"], "0")
        self.assertLessEqual(abs(int(out["POSITION"]) - 2048), 12)

    def test_parking_is_refused_without_a_calibrated_auxiliary(self):
        """A cross-leg prerequisite cannot be guessed. No calibration, no move.

        The gate fires before the joint is attempted, so the absence of a
        per-joint result row is the evidence that nothing moved.
        """
        self.build_fixture()                  # nothing seeded
        summary, joints, _, _ = self.h.run(1 << 1)
        self.assertEqual(summary["STATUS"], "PREREQUISITE_DRIFT")
        self.assertEqual(summary["FAILED_ID"], "12")
        self.assertEqual(summary["JOINTS_OK"], "0")
        self.assertEqual(joints, {})
        self.assertSafeExit()

    def test_selection_order_follows_the_dependency_graph_not_the_mask(self):
        self.build_fixture()
        _, _, order, _ = self.h.run(ALL_JOINTS_MASK)
        position = {joint: i for i, joint in enumerate(order)}
        self.assertLess(position[10], position[1])
        self.assertLess(position[7], position[4])
        self.assertLess(position[1], position[2])

    def test_every_parking_row_in_the_full_run_is_entered_and_restored(self):
        self.build_fixture()
        summary, _, _, _ = self.h.run(ALL_JOINTS_MASK)
        self.assertEqual(summary["STATUS"], "OK")
        self.assertEqual(summary["PARKING_REQUIRED"], "6")
        self.assertEqual(summary["NO_PARKING"], "18")
        self.assertEqual(summary["PARKING_RESTORED"], "6")


# ==========================================================================
# H5 — one leg
# ==========================================================================

class TestH5CalibrateLeg(EngineTestBase):
    def test_h5_calibrates_three_joints_through_the_shared_run(self):
        self.build_fixture(seeded={10})
        summary, joints, order, _ = self.h.run(LEG_MASKS["LF"])
        self.assertEqual(summary["STATUS"], "OK")
        self.assertEqual(summary["JOINTS_OK"], "3")
        self.assertEqual(sorted(joints), [0, 1, 2])
        self.assertEqual(order, [0, 1, 2])
        for slot in (0, 1, 2):
            self.assertEqual(joints[slot]["STATUS"], "OK")
        self.assertSafeExit()

    def test_h5_on_a_hind_leg_needs_no_cross_leg_seed(self):
        """RH's parking auxiliary is its own upper leg, so RH stands alone."""
        self.build_fixture()
        summary, joints, _, _ = self.h.run(LEG_MASKS["RH"])
        self.assertEqual(summary["STATUS"], "OK")
        self.assertEqual(summary["JOINTS_OK"], "3")
        self.assertEqual(joints[8]["MIN_PARK"], "1")
        self.assertSafeExit()

    def test_h5_front_leg_without_the_cross_leg_prerequisite_fails_closed(self):
        self.build_fixture()
        summary, _, _, _ = self.h.run(LEG_MASKS["LF"])
        self.assertEqual(summary["STATUS"], "PREREQUISITE_DRIFT")
        self.assertSafeExit()

    def test_h5_refuses_an_uncharacterized_joint(self):
        self.build_fixture(seeded={10}, characterized=set(range(12)) - {2})
        summary, joints, _, _ = self.h.run(LEG_MASKS["LF"])
        self.assertNotEqual(summary["STATUS"], "OK")
        self.assertEqual(joints[2]["STATUS"], "PREREQUISITE_DRIFT")
        self.assertSafeExit()

    def test_h5_stops_the_leg_at_the_first_failing_joint(self):
        self.build_fixture(seeded={10})
        self.h.fault(12, "FROZEN", 1)          # LF upper leg
        summary, joints, _, _ = self.h.run(LEG_MASKS["LF"])
        self.assertNotEqual(summary["STATUS"], "OK")
        self.assertEqual(summary["FAILED_ID"], "12")
        self.assertIn(0, joints)               # hip already done
        self.assertNotIn(2, joints)            # lower leg never started
        self.assertSafeExit()

    def test_h5_releases_every_joint_even_after_a_mid_leg_failure(self):
        self.build_fixture(seeded={10})
        self.h.fault(12, "FROZEN", 1)
        self.h.run(LEG_MASKS["LF"])
        for bus_id in (13, 12, 11, 42):
            out = parse(self.h.send(f"POS {bus_id}"))
            self.assertEqual(out["TORQUE"], "0", f"servo {bus_id} still energised")


# ==========================================================================
# H6 — all four legs, one session result
# ==========================================================================

class TestH6CalibrateAllLegs(EngineTestBase):
    def test_h6_calibrates_twelve_joints_in_one_run(self):
        self.build_fixture()
        summary, joints, order, finals = self.h.run(ALL_JOINTS_MASK)
        self.assertEqual(summary["STATUS"], "OK")
        self.assertEqual(summary["JOINTS_OK"], "12")
        self.assertEqual(summary["REQUESTED"], "12")
        self.assertEqual(len(joints), 12)
        self.assertEqual(len(order), 12)
        for slot in range(12):
            self.assertEqual(joints[slot]["STATUS"], "OK", f"slot {slot}")
            self.assertEqual(finals[slot]["USABLE"], "1")
        self.assertSafeExit()

    def test_h6_is_one_session_result_not_four_independent_runs(self):
        """A single selection mask, a single order, a single safe-off."""
        self.build_fixture()
        summary, _, order, _ = self.h.run(ALL_JOINTS_MASK)
        self.assertEqual(summary["SELECTION"], "0FFF")
        self.assertEqual(summary["EXEC_COUNT"], "12")
        # The order interleaves legs; four independent leg runs could not.
        legs = [joint // 3 for joint in order]
        self.assertNotEqual(legs, sorted(legs))
        self.assertEqual(summary["SAFE_OFF"], "1")

    def test_h6_aborts_the_sequence_at_the_first_failing_joint(self):
        self.build_fixture()
        self.h.fault(33, "FROZEN", 1)          # RH hip, third in the order
        summary, joints, _, _ = self.h.run(ALL_JOINTS_MASK)
        self.assertNotEqual(summary["STATUS"], "OK")
        self.assertEqual(summary["FAILED_ID"], "33")
        self.assertLess(len(joints), 12)
        self.assertSafeExit()

    def test_h6_leaves_every_joint_torque_off_after_a_failure(self):
        self.build_fixture()
        self.h.fault(33, "FROZEN", 1)
        self.h.run(ALL_JOINTS_MASK)
        for _, bus_id, _, _, _ in JOINT_LAYOUT:
            out = parse(self.h.send(f"POS {bus_id}"))
            self.assertEqual(out["TORQUE"], "0", f"servo {bus_id} still energised")

    def test_h6_handles_a_mixed_direction_robot(self):
        """Half the joints mapped one way, half the other. One engine."""
        directions = {slot: (1 if slot % 2 == 0 else -1) for slot in range(12)}
        info = self.build_fixture(directions=directions)
        summary, joints, _, _ = self.h.run(ALL_JOINTS_MASK)
        self.assertEqual(summary["STATUS"], "OK")
        self.assertEqual(summary["JOINTS_OK"], "12")
        for slot in range(12):
            self.assertEqual(int(joints[slot]["DIRECTION"]), info[slot]["direction"])
            self.assertEqual(int(joints[slot]["DERIVED_Q0"]), 2048)
        self.assertSafeExit()


# ==========================================================================
# Fail-closed behaviour of the run itself
# ==========================================================================

class TestRunFailsClosed(EngineTestBase):
    def test_an_empty_selection_is_refused(self):
        self.build_fixture()
        summary, _, _, _ = self.h.run(0)
        self.assertEqual(summary["STATUS"], "INVALID_PLAN")
        self.assertSafeExit()

    def test_a_missing_servo_stops_the_run_loudly(self):
        """A servo identified at census that later disappears is a hard stop."""
        self.build_fixture()
        self.h.fault(13, "CONTEXT_INVALID", 1)
        summary, _, _, _ = self.h.run(1 << 0)
        self.assertEqual(summary["STATUS"], "IDENTITY_OR_SESSION_DRIFT")
        self.assertSafeExit()

    def test_losing_the_session_mid_run_stops_it(self):
        self.build_fixture()
        self.h.context(False)
        summary, _, _, _ = self.h.run(ALL_JOINTS_MASK)
        self.assertNotEqual(summary["STATUS"], "OK")
        self.assertEqual(summary["JOINTS_OK"], "0")

    def test_a_joint_starting_away_from_q0_is_refused_before_any_motion(self):
        self.build_fixture()
        self.h.servo(13, 2400, 1524, 2563, 1)   # parked far from its q0 watch
        summary, joints, _, _ = self.h.run(1 << 0)
        self.assertEqual(summary["STATUS"], "PREREQUISITE_DRIFT")
        self.assertEqual(summary["FAILED_ID"], "13")
        self.assertEqual(joints, {})            # nothing was attempted
        self.assertSafeExit()

    def test_a_servo_that_will_not_release_latches_cut_power(self):
        self.build_fixture()
        self.h.fault(13, "REFUSE_TORQUE_OFF", 1)
        summary, _, _, _ = self.h.run(1 << 0)
        self.assertEqual(summary["STATUS"], "TORQUE_OFF_FAILED")
        self.assertEqual(summary["CUT_POWER"], "1")
        self.assertEqual(summary["SAFE_OFF"], "0")

    def test_a_healthy_run_never_asks_for_power_to_be_cut(self):
        self.build_fixture()
        summary, _, _, _ = self.h.run(ALL_JOINTS_MASK)
        self.assertEqual(summary["CUT_POWER"], "0")
        self.assertEqual(summary["SAFE_OFF"], "1")


# ==========================================================================
# Absolute prohibitions
# ==========================================================================

class TestAbsoluteProhibitionsAcrossEveryPath(EngineTestBase):
    """No engine path may write EEPROM or address the broadcast id."""

    def test_no_path_ever_writes_eeprom_or_broadcasts(self):
        self.build_fixture()
        self.h.characterize(13, 2048)
        self.h.run(ALL_JOINTS_MASK)
        self.h.fault(22, "FROZEN", 1)
        self.h.run(LEG_MASKS["RF"])
        stats = self.h.stats()
        self.assertEqual(stats["EEPROM_WRITES"], "0")
        self.assertEqual(stats["BROADCAST_WRITES"], "0")
        self.assertGreater(int(stats["POSITION_COMMANDS"]), 0)

    def test_torque_is_always_released_after_the_whole_sequence(self):
        self.build_fixture()
        self.h.run(ALL_JOINTS_MASK)
        self.assertEqual(self.h.stats()["ALL_TORQUE_OFF"], "1")


if __name__ == "__main__":
    unittest.main()
