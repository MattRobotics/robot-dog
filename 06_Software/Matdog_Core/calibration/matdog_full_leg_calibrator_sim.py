"""MATDOG Full Leg Calibrator V1 — offline ST3215 bus and firmware simulator.

Lets the complete calibrator state machine be exercised with no hardware, no
serial port and no servo power, so every fault path in section 13 of the V1
specification has a test.

Two layers:

* :class:`MockST3215Bus` — register-level servo model. Every access goes through
  it, so the test suite can assert not just *what the firmware concluded* but
  *what it put on the wire*: which addresses were written, to which unicast id,
  and whether any EEPROM address was ever touched.

* :class:`MockCalibratorFirmware` — mirrors the decision logic of
  ``matdog_full_leg_calibrator_v1.ino``: the write allowlist, the census
  invariants, the manual-q0 capture and the H0..H7 stage / characterization
  gates. ``test_matdog_full_leg_calibrator_firmware_sync.py`` pins the shared
  constants against the firmware source so the two cannot drift silently.

The bounded contact search itself is NOT re-implemented here. That state machine
lives in ``flc_contact_detector.h`` and is tested by compiling the identical
header into a host harness, so there is exactly one implementation of it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Iterable

from matdog_full_leg_calibrator_policy import (
    EXPECTED_LEG_IDS,
    AUTHORIZED_STAGE,
    HardwareStage,
    pre_motion_blockers,
)

# Register map — mirrors the firmware and the frozen bench tooling.
REG_MODEL = 0x03
REG_ID = 0x05
REG_BAUD = 0x06
REG_RESPONSE_STATUS = 0x08
REG_POSITION_OFFSET = 0x1F
REG_TORQUE_ENABLE = 0x28
REG_ACC = 0x29
REG_GOAL_POSITION = 0x2A
REG_GOAL_SPEED = 0x2E
REG_TORQUE_LIMIT = 0x30
REG_LOCK = 0x37
REG_PRESENT_POSITION = 0x38
REG_PRESENT_VOLTAGE = 0x3E
REG_PRESENT_TEMPERATURE = 0x3F
REG_STATUS = 0x40

BROADCAST_ID = 254

#: Addresses whose modification would violate the EEPROM-write-free contract.
EEPROM_ADDRESSES = frozenset(
    {
        REG_ID, REG_BAUD, 0x07, REG_RESPONSE_STATUS, 0x09, 0x0A, 0x0B, 0x0C,
        0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
        0x18, 0x19, 0x1A, 0x1B, 0x1C, 0x1D, 0x1E, REG_POSITION_OFFSET,
        0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, REG_LOCK,
    }
)

#: The 20-register persistent profile, MATDOG_C018_V1.
CANONICAL_PROFILE: dict[int, int] = {
    0x09: 0, 0x0B: 4095, 0x0D: 70, 0x0E: 140, 0x0F: 40, 0x10: 1000,
    0x15: 32, 0x16: 32, 0x17: 0, 0x18: 16, 0x1A: 1, 0x1B: 1, 0x1C: 310,
    0x21: 0, 0x22: 20, 0x23: 200, 0x24: 80, 0x25: 10, 0x26: 200, 0x27: 200,
}

EXPECTED_MODEL = 777
EXPECTED_RESPONSE_STATUS = 1
EXPECTED_BAUD = 0
EXPECTED_POSITION_OFFSET = 0
ENCODER_MAX = 4095
THERMAL_LIMIT_C = 70
VOLTAGE_MIN = 40
VOLTAGE_MAX = 140
Q0_MAX_SPREAD_TICKS = 12
Q0_PLAUSIBLE_RESIDUAL_TICKS = 120
Q0_SAMPLES_MIN = 16


class BusFault(Enum):
    """Injectable per-servo transport faults."""

    NONE = "NONE"
    NO_RESPONDER = "NO_RESPONDER"
    READ_TIMEOUT = "READ_TIMEOUT"
    DRIVER_ERROR = "DRIVER_ERROR"
    WRITE_REJECTED = "WRITE_REJECTED"


@dataclass
class SimServo:
    """One simulated ST-3215-C018."""

    servo_id: int
    model: int = EXPECTED_MODEL
    stored_id: int | None = None
    baud: int = EXPECTED_BAUD
    response_status: int = EXPECTED_RESPONSE_STATUS
    position_offset: int = EXPECTED_POSITION_OFFSET
    torque_enable: int = 0
    torque_limit: int = 1000
    present_position: int = 2048
    present_speed: int = 0
    present_current: int = 0
    voltage: int = 120
    temperature: int = 28
    status_byte: int = 0
    profile: dict[int, int] = field(default_factory=lambda: dict(CANONICAL_PROFILE))
    fault: BusFault = BusFault.NONE
    #: Successive PresentPosition values returned by ReadPos, for q0 dispersion.
    position_sequence: list[int] | None = None
    _sequence_index: int = 0

    def __post_init__(self) -> None:
        if self.stored_id is None:
            self.stored_id = self.servo_id

    def next_position(self) -> int:
        if not self.position_sequence:
            return self.present_position
        value = self.position_sequence[self._sequence_index % len(self.position_sequence)]
        self._sequence_index += 1
        return value


@dataclass
class WriteRecord:
    servo_id: int
    address: int
    width: int
    value: int
    allowed: bool


class BroadcastWriteError(AssertionError):
    """Raised the instant a broadcast write is attempted. Never permitted."""


class EepromWriteError(AssertionError):
    """Raised the instant an EEPROM address is written. Never permitted."""


class MockST3215Bus:
    """Register-level ST3215 bus with fault injection and a full write log."""

    def __init__(self, servos: Iterable[SimServo] | None = None) -> None:
        self.servos: dict[int, SimServo] = {}
        for servo in servos or ():
            self.servos[servo.servo_id] = servo
        self.write_log: list[WriteRecord] = []
        self.read_count = 0
        self.error = 0

    # -- transport ---------------------------------------------------------
    def _reachable(self, servo_id: int) -> SimServo | None:
        servo = self.servos.get(servo_id)
        if servo is None:
            self.error = 1
            return None
        if servo.fault in (BusFault.NO_RESPONDER, BusFault.READ_TIMEOUT):
            self.error = 1
            return None
        if servo.fault is BusFault.DRIVER_ERROR:
            self.error = 0x40
            return None
        self.error = 0
        return servo

    def ping(self, servo_id: int) -> int:
        servo = self._reachable(servo_id)
        return servo_id if servo is not None else -1

    def read_snapshot(self, servo_id: int) -> dict[str, int] | None:
        servo = self._reachable(servo_id)
        if servo is None:
            return None
        self.read_count += 1
        snapshot = {
            "model": servo.model,
            "stored_id": servo.stored_id,
            "baud": servo.baud,
            "response_status": servo.response_status,
            "position_offset": servo.position_offset,
            "torque_enable": servo.torque_enable,
            "torque_limit": servo.torque_limit,
            "present_position": servo.present_position,
            "voltage": servo.voltage,
            "temperature": servo.temperature,
            "status_byte": servo.status_byte,
        }
        snapshot.update({f"profile_{addr:#04x}": val for addr, val in servo.profile.items()})
        return snapshot

    def read_position(self, servo_id: int) -> int:
        servo = self._reachable(servo_id)
        if servo is None:
            return -1
        self.read_count += 1
        return servo.next_position()

    def read_byte(self, servo_id: int, address: int) -> int:
        servo = self._reachable(servo_id)
        if servo is None:
            return -1
        self.read_count += 1
        if address == REG_TORQUE_ENABLE:
            return servo.torque_enable
        if address == REG_PRESENT_VOLTAGE:
            return servo.voltage
        if address == REG_PRESENT_TEMPERATURE:
            return servo.temperature
        if address == REG_STATUS:
            return servo.status_byte
        return servo.profile.get(address, 0)

    def write(self, servo_id: int, address: int, width: int, value: int) -> int:
        """Any write reaching here has already passed the firmware allowlist.

        The two hard prohibitions are enforced here as an independent barrier, so
        a bug in the allowlist surfaces as a loud test failure rather than a
        silently-accepted EEPROM write.
        """
        if servo_id == BROADCAST_ID:
            raise BroadcastWriteError(
                f"broadcast write attempted: addr {address:#04x} value {value}"
            )
        if address in EEPROM_ADDRESSES:
            raise EepromWriteError(
                f"EEPROM write attempted on id {servo_id}: "
                f"addr {address:#04x} value {value}"
            )

        servo = self._reachable(servo_id)
        if servo is None:
            self.write_log.append(WriteRecord(servo_id, address, width, value, False))
            return 0
        if servo.fault is BusFault.WRITE_REJECTED:
            self.write_log.append(WriteRecord(servo_id, address, width, value, False))
            self.error = 1
            return 0

        self.write_log.append(WriteRecord(servo_id, address, width, value, True))
        if address == REG_TORQUE_ENABLE:
            servo.torque_enable = value
        elif address == REG_TORQUE_LIMIT:
            servo.torque_limit = value
        elif address == REG_GOAL_POSITION:
            servo.present_position = value
        return 1

    # -- assertions used by the tests --------------------------------------
    @property
    def eeprom_writes(self) -> list[WriteRecord]:
        return [w for w in self.write_log if w.address in EEPROM_ADDRESSES]

    def all_torque_off(self) -> bool:
        return all(s.torque_enable == 0 for s in self.servos.values())


def make_healthy_leg_bus() -> MockST3215Bus:
    """Twelve correctly provisioned leg servos, torque off, head absent."""
    return MockST3215Bus(SimServo(servo_id=i) for i in EXPECTED_LEG_IDS)


@dataclass
class CensusServoResult:
    bus_id: int
    present: bool
    identity_ok: bool
    reasons: tuple[str, ...] = ()


@dataclass
class CensusResult:
    passed: bool
    present_count: int
    identity_ok_count: int
    unexpected_responders: tuple[int, ...]
    servos: tuple[CensusServoResult, ...]
    fail_reason: str = ""


@dataclass
class ModeResult:
    """Outcome of any calibrator operating mode."""

    mode: str
    accepted: bool
    reasons: tuple[str, ...] = ()
    payload: dict | None = None


class MockCalibratorFirmware:
    """Mirrors the decision logic of matdog_full_leg_calibrator_v1.ino."""

    def __init__(
        self,
        bus: MockST3215Bus,
        stage: HardwareStage = AUTHORIZED_STAGE,
    ) -> None:
        self.bus = bus
        self.stage = stage
        self.census_fresh = False
        self.census_epoch = 0
        self.last_fault = "NONE"
        #: Operator confirmation of the bootstrap envelope, this session only.
        self.bootstrap_approved = False
        #: bus_id -> characterization evidence, tied to a census epoch.
        self.characterized: dict[int, dict] = {}

    # -- gates -------------------------------------------------------------
    def _require_stage(self, needed: HardwareStage, mode: str) -> str | None:
        if self.stage.value >= needed.value:
            return None
        return f"{mode} needs {needed.name}, authorized stage is {self.stage.name}"

    def _require_fresh_census(self, mode: str) -> str | None:
        if self.census_fresh:
            return None
        return f"{mode} requires a fresh successful census in this physical session"

    def _require_pre_motion_parameters(self, mode: str) -> str | None:
        """Only CLASS_A parameters gate motion, and only without a bootstrap.

        CLASS_D acceptance tolerances are deliberately absent: a tolerance that
        judges a measurement cannot be a precondition of taking it.
        """
        blockers = pre_motion_blockers()
        if not blockers or self.bootstrap_approved:
            return None
        return (
            f"{mode} blocked by unresolved pre-motion parameters: "
            + ", ".join(b.name for b in blockers)
            + " (no approved bootstrap envelope)"
        )

    def _require_characterized(self, mode: str, bus_id: int) -> str | None:
        entry = self.characterized.get(bus_id)
        if entry is not None and entry["census_epoch"] == self.census_epoch:
            return None
        return f"{mode} requires {bus_id} characterized in this physical session"

    # -- LEGS_12_CENSUS ----------------------------------------------------
    def census(self) -> CensusResult:
        self.census_fresh = False
        self.census_epoch += 1
        # A new census is a new physical session epoch: characterization evidence
        # and bootstrap approval belong to the setup verified when granted.
        self.characterized.clear()
        self.bootstrap_approved = False

        results: list[CensusServoResult] = []
        present = 0
        identity_ok = 0

        for bus_id in EXPECTED_LEG_IDS:
            snap = self.bus.read_snapshot(bus_id)
            if snap is None:
                results.append(CensusServoResult(bus_id, False, False, ("MISSING",)))
                continue
            present += 1

            reasons: list[str] = []
            if snap["model"] != EXPECTED_MODEL:
                reasons.append("WRONG_MODEL")
            if snap["stored_id"] != bus_id:
                reasons.append("ID_REGISTER_MISMATCH")
            if snap["baud"] != EXPECTED_BAUD:
                reasons.append("BAUD_MISMATCH")
            if snap["response_status"] != EXPECTED_RESPONSE_STATUS:
                reasons.append("RESPONSE_STATUS")
            if snap["position_offset"] != EXPECTED_POSITION_OFFSET:
                reasons.append("NONZERO_POSITION_OFFSET")
            if not 0 <= snap["present_position"] <= ENCODER_MAX:
                reasons.append("IMPOSSIBLE_POSITION")
            if not VOLTAGE_MIN <= snap["voltage"] <= VOLTAGE_MAX:
                reasons.append("VOLTAGE_OUT_OF_RANGE")
            if snap["temperature"] >= THERMAL_LIMIT_C:
                reasons.append("TEMPERATURE")
            if snap["status_byte"] != 0:
                reasons.append("STATUS_ERROR")
            if snap["torque_enable"] != 0:
                reasons.append("TORQUE_UNEXPECTEDLY_ON")
            for addr, expected in CANONICAL_PROFILE.items():
                if snap.get(f"profile_{addr:#04x}") != expected:
                    reasons.append(f"PROFILE_MISMATCH_{addr:#04x}")

            ok = not reasons
            if ok:
                identity_ok += 1
            results.append(CensusServoResult(bus_id, True, ok, tuple(reasons)))

        unexpected = tuple(
            sid
            for sid in sorted(self.bus.servos)
            if sid not in EXPECTED_LEG_IDS and self.bus.ping(sid) >= 0
        )

        passed = (
            identity_ok == len(EXPECTED_LEG_IDS)
            and not unexpected
            and present == len(EXPECTED_LEG_IDS)
        )
        fail_reason = ""
        if not passed:
            if present == 0:
                fail_reason = "NO_RESPONDERS"
            elif unexpected:
                fail_reason = "UNEXPECTED_RESPONDER"
            else:
                fail_reason = "EXPECTED_LEG_INVARIANTS_NOT_SATISFIED"

        self.census_fresh = passed
        return CensusResult(
            passed=passed,
            present_count=present,
            identity_ok_count=identity_ok,
            unexpected_responders=unexpected,
            servos=tuple(results),
            fail_reason=fail_reason,
        )

    # -- CAPTURE_MANUAL_Q0 -------------------------------------------------
    def capture_manual_q0(self, samples: int = 64) -> ModeResult:
        reasons = [
            r
            for r in (
                self._require_stage(HardwareStage.H2_MANUAL_Q0, "CAPTURE_Q0"),
                self._require_fresh_census("CAPTURE_Q0"),
            )
            if r
        ]
        if reasons:
            return ModeResult("CAPTURE_Q0", False, tuple(reasons))

        from matdog_full_leg_calibrator_derive import capture_manual_q0

        payload: dict[str, dict] = {}
        accepted = True
        for bus_id in EXPECTED_LEG_IDS:
            torque = self.bus.read_byte(bus_id, REG_TORQUE_ENABLE)
            if torque < 0:
                accepted = False
                payload[str(bus_id)] = {"status": "NO_RESPONDER"}
                continue
            if torque != 0:
                accepted = False
                payload[str(bus_id)] = {"status": "REFUSED_TORQUE_ON"}
                continue

            readings = []
            for _ in range(samples):
                pos = self.bus.read_position(bus_id)
                if pos >= 0:
                    readings.append(pos)

            candidate = capture_manual_q0(
                joint_name=f"bus_{bus_id}",
                bus_id=bus_id,
                unit_label="",
                samples=readings,
                max_spread_ticks=Q0_MAX_SPREAD_TICKS,
                min_samples=Q0_SAMPLES_MIN,
                plausible_residual_ticks=Q0_PLAUSIBLE_RESIDUAL_TICKS,
            )
            payload[str(bus_id)] = candidate.as_dict()
            if not candidate.ok:
                accepted = False

        return ModeResult("CAPTURE_Q0", accepted, (), payload)

    # -- H3 bootstrap approval --------------------------------------------
    def approve_bootstrap(self) -> ModeResult:
        """Arm the conservative first-motion envelope for THIS session."""
        reason = self._require_stage(HardwareStage.H3_JOINT_CHARACTERIZE,
                                     "APPROVE_BOOTSTRAP")
        if reason:
            return ModeResult("APPROVE_BOOTSTRAP", False, (reason,))
        self.bootstrap_approved = True
        return ModeResult("APPROVE_BOOTSTRAP", True, (),
                          {"origin": "H3_BOOTSTRAP_OPERATOR_APPROVED",
                           "is_measurement": False})

    # -- motion modes ------------------------------------------------------
    def characterize_joint(self, bus_id: int, succeed: bool = True) -> ModeResult:
        """H3. Gated by stage, census and the pre-motion parameter gate — but
        NOT by the acceptance tolerances it exists to inform."""
        reasons = [
            r
            for r in (
                None if bus_id in EXPECTED_LEG_IDS else f"{bus_id} is not a leg bus id",
                self._require_stage(HardwareStage.H3_JOINT_CHARACTERIZE,
                                    "CHARACTERIZE_JOINT"),
                self._require_fresh_census("CHARACTERIZE_JOINT"),
                self._require_pre_motion_parameters("CHARACTERIZE_JOINT"),
            )
            if r
        ]
        if reasons:
            return ModeResult("CHARACTERIZE_JOINT", False, tuple(reasons))

        if not succeed:
            return ModeResult("CHARACTERIZE_JOINT", False,
                              ("simulated characterization failure",))

        # The real measurement happens in the C++ engine; the simulator records
        # the session bookkeeping the firmware performs around it.
        self.characterized[bus_id] = {
            "census_epoch": self.census_epoch,
            "origin": "CHARACTERIZED_CURRENT_HARDWARE",
            "direction": 1,
        }
        return ModeResult("CHARACTERIZE_JOINT", True, (),
                          {"bus_id": bus_id, "origin": "CHARACTERIZED_CURRENT_HARDWARE"})

    def calibrate_joint(self, bus_id: int) -> ModeResult:
        reasons = [
            r
            for r in (
                None if bus_id in EXPECTED_LEG_IDS else f"{bus_id} is not a leg bus id",
                self._require_stage(HardwareStage.H4_JOINT_CALIBRATE, "CALIBRATE_JOINT"),
                self._require_fresh_census("CALIBRATE_JOINT"),
                self._require_pre_motion_parameters("CALIBRATE_JOINT"),
                self._require_characterized("CALIBRATE_JOINT", bus_id),
            )
            if r
        ]
        if reasons:
            return ModeResult("CALIBRATE_JOINT", False, tuple(reasons))
        return ModeResult("CALIBRATE_JOINT", True, (),
                          {"bus_id": bus_id, "promotion": "REQUIRES_EXPLICIT_GATE"})

    def calibrate_leg(self, leg: str) -> ModeResult:
        ids = [i for i in EXPECTED_LEG_IDS if str(i)[0] == {"LF": "1", "RF": "2",
                                                            "RH": "3", "LH": "4"}[leg]]
        reasons = [
            r
            for r in (
                self._require_stage(HardwareStage.H5_LEG, "CALIBRATE_LEG"),
                self._require_fresh_census("CALIBRATE_LEG"),
                self._require_pre_motion_parameters("CALIBRATE_LEG"),
            )
            if r
        ]
        reasons += [r for r in (self._require_characterized("CALIBRATE_LEG", i)
                                for i in ids) if r]
        if reasons:
            return ModeResult("CALIBRATE_LEG", False, tuple(reasons))
        return ModeResult("CALIBRATE_LEG", True, (), {"leg": leg, "joints": ids})

    def calibrate_all_legs(self) -> ModeResult:
        reasons = [
            r
            for r in (
                self._require_stage(HardwareStage.H6_FOUR_LEGS, "CALIBRATE_ALL"),
                self._require_fresh_census("CALIBRATE_ALL"),
                self._require_pre_motion_parameters("CALIBRATE_ALL"),
            )
            if r
        ]
        reasons += [r for r in (self._require_characterized("CALIBRATE_ALL", i)
                                for i in EXPECTED_LEG_IDS) if r]
        if reasons:
            return ModeResult("CALIBRATE_ALL", False, tuple(reasons))
        return ModeResult("CALIBRATE_ALL", True, (),
                          {"legs": ["LF", "RF", "RH", "LH"], "joints": 12,
                           "promotion": "REQUIRES_EXPLICIT_GATE"})

    # -- SAFE_OFF ----------------------------------------------------------
    def safe_off(self) -> ModeResult:
        """Unicast torque OFF to every known leg servo, with readback. Idempotent."""
        failures: list[str] = []
        responders_seen = False

        for bus_id in EXPECTED_LEG_IDS:
            probe = self.bus.read_byte(bus_id, REG_TORQUE_ENABLE)
            if probe < 0:
                continue  # an absent servo cannot hold torque
            responders_seen = True
            if probe == 0:
                continue
            self.bus.write(bus_id, REG_TORQUE_ENABLE, 1, 0)
            if self.bus.read_byte(bus_id, REG_TORQUE_ENABLE) != 0:
                failures.append(f"id {bus_id} did not confirm torque off")

        if failures:
            self.last_fault = "SAFE_OFF_FAILED"
        return ModeResult(
            "SAFE_OFF",
            not failures,
            tuple(failures),
            {"responders_seen": responders_seen},
        )
