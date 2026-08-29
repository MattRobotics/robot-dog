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

import copy
import itertools
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


class SessionState(Enum):
    """Volatile firmware session state; a fault stays latched until reset."""

    SESSION_IDLE = 0
    SESSION_CENSUS_OK = 1
    SESSION_FAULT = 2


@dataclass(frozen=True)
class EvidenceBinding:
    """Identity shared by every RAM-only authorization/evidence object."""

    boot_session_id: str
    host_session_id: str
    session_generation: int
    census_epoch: int


@dataclass(frozen=True)
class ManualQ0Evidence:
    binding: EvidenceBinding
    centre_tick: int
    min_tick: int
    max_tick: int
    spread_ticks: int
    samples: int


@dataclass(frozen=True)
class DirectionWitness:
    """Current-build semantic meaning of positive MATDOG joint motion."""

    binding: EvidenceBinding
    direction: int
    semantic: str


@dataclass(frozen=True)
class BootstrapApproval:
    binding: EvidenceBinding
    origin: str = "H3_BOOTSTRAP_OPERATOR_APPROVED"


@dataclass(frozen=True)
class CharacterizationEvidence:
    """H3 physical observations, deliberately excluding kinematic direction."""

    binding: EvidenceBinding
    raw_probe_sign: int
    origin: str = "CHARACTERIZED_CURRENT_HARDWARE"


@dataclass(frozen=True)
class CalibrationEvidence:
    binding: EvidenceBinding
    direction: int
    manual_q0_tick: int
    derived_q0_tick: int
    tier: str = "CANDIDATE_BLOCKED_TOLERANCE_UNVALIDATED"


class MockCalibratorFirmware:
    """Mirrors the volatile policy state of the ESP32 calibrator firmware.

    The optional ``host_evidence`` object represents a report loaded by a host
    process after reconnect.  It is retained only so a test can prove that such
    a file has no authority: no gate below consults it.
    """

    _boot_ids = itertools.count(1)

    def __init__(
        self,
        bus: MockST3215Bus,
        stage: HardwareStage = AUTHORIZED_STAGE,
        *,
        boot_session_id: str | int | None = None,
        bootstrap_build_approved: bool | None = None,
        host_evidence: dict | None = None,
    ) -> None:
        self.bus = bus
        self.stage = stage
        self.bootstrap_build_approved = (
            stage.value >= HardwareStage.H3_JOINT_CHARACTERIZE.value
            if bootstrap_build_approved is None
            else bool(bootstrap_build_approved)
        )
        self.host_evidence = copy.deepcopy(host_evidence)
        self.boot_session_id = self._normalize_session_id(
            self._next_boot_id() if boot_session_id is None else boot_session_id,
            allow_zero=False,
        )
        self.active_host_session_id: str | None = None
        self.session_generation = 0
        self.session_state = SessionState.SESSION_IDLE
        self.census_fresh = False
        self.census_epoch = 0
        self.last_fault = "NONE"
        self.manual_q0: dict[int, ManualQ0Evidence] = {}
        self.direction_witnesses: dict[int, DirectionWitness] = {}
        self.bootstrap: BootstrapApproval | None = None
        self.characterized: dict[int, CharacterizationEvidence] = {}
        self.calibrated: dict[int, CalibrationEvidence] = {}

    # -- lifecycle / identity ---------------------------------------------
    @classmethod
    def _next_boot_id(cls) -> int:
        value = next(cls._boot_ids) & 0xFFFFFFFF
        return value or next(cls._boot_ids) & 0xFFFFFFFF

    @staticmethod
    def _normalize_session_id(value: str | int, *, allow_zero: bool) -> str:
        if isinstance(value, int):
            number = value
        elif isinstance(value, str):
            raw = value.strip()
            if raw.lower().startswith("0x"):
                raw = raw[2:]
            if (
                not raw
                or len(raw) > 8
                or any(c not in "0123456789abcdefABCDEF" for c in raw)
            ):
                raise ValueError("session id must contain one to eight hexadecimal digits")
            number = int(raw, 16)
        else:
            raise TypeError("session id must be an integer or hexadecimal string")
        if not 0 <= number <= 0xFFFFFFFF or (number == 0 and not allow_zero):
            raise ValueError("session id must be a non-zero uint32")
        return f"{number:08X}"

    @property
    def fault_latched(self) -> bool:
        return self.session_state is SessionState.SESSION_FAULT

    @property
    def bootstrap_approved(self) -> bool:
        return self.bootstrap is not None and self._binding_is_current(
            self.bootstrap.binding
        )

    def _current_binding(self) -> EvidenceBinding:
        if self.active_host_session_id is None:
            raise RuntimeError("cannot bind evidence without an active host session")
        return EvidenceBinding(
            boot_session_id=self.boot_session_id,
            host_session_id=self.active_host_session_id,
            session_generation=self.session_generation,
            census_epoch=self.census_epoch,
        )

    def _binding_is_current(self, binding: EvidenceBinding) -> bool:
        return (
            self.active_host_session_id is not None
            and binding.boot_session_id == self.boot_session_id
            and binding.host_session_id == self.active_host_session_id
            and binding.session_generation == self.session_generation
            and binding.census_epoch == self.census_epoch
        )

    def _clear_session_evidence(self) -> None:
        self.manual_q0.clear()
        self.direction_witnesses.clear()
        self.bootstrap = None
        self.characterized.clear()
        self.calibrated.clear()

    def _invalidate_census(self) -> None:
        self.census_fresh = False

    def _latch_fault(self, reason: str) -> None:
        self.last_fault = reason
        self.session_state = SessionState.SESSION_FAULT
        self._invalidate_census()
        self.bootstrap = None

    def reset(self, boot_session_id: str | int | None = None) -> ModeResult:
        """Model an ESP32 reset; no host evidence or authorization survives."""
        old_boot = self.boot_session_id
        self.boot_session_id = self._normalize_session_id(
            self._next_boot_id() if boot_session_id is None else boot_session_id,
            allow_zero=False,
        )
        self.active_host_session_id = None
        self.session_generation = 0
        self.session_state = SessionState.SESSION_IDLE
        self.census_epoch = 0
        self._invalidate_census()
        self._clear_session_evidence()
        self.last_fault = "NONE"
        return ModeResult(
            "RESET",
            True,
            (),
            {"old_boot_session_id": old_boot, "boot_session_id": self.boot_session_id},
        )

    def reconnect(
        self,
        host_session_id: str | int,
        *,
        boot_session_id: str | int | None = None,
    ) -> ModeResult:
        """Opening a new simulated USB link resets the ESP32, then leases it."""
        old_boot = self.boot_session_id
        self.reset(boot_session_id)
        result = self.begin_session(host_session_id)
        if result.payload is not None:
            result.payload["reconnected_from_boot_session_id"] = old_boot
        return result

    def begin_session(self, host_session_id: str | int) -> ModeResult:
        mode = "SESSION_BEGIN"
        fault = self._require_not_faulted(mode)
        if fault:
            return ModeResult(mode, False, (fault,))
        try:
            normalized = self._normalize_session_id(host_session_id, allow_zero=False)
        except (TypeError, ValueError) as exc:
            return ModeResult(mode, False, (str(exc),))

        if self.active_host_session_id == normalized:
            return ModeResult(
                mode,
                True,
                (),
                {
                    "boot_session_id": self.boot_session_id,
                    "active_host_session_id": normalized,
                    "session_generation": self.session_generation,
                    "idempotent": True,
                },
            )

        safe = self.safe_off()
        if not safe.accepted:
            return ModeResult(mode, False, ("SESSION_BEGIN_SAFE_OFF_FAILED",))

        self._invalidate_census()
        self._clear_session_evidence()
        self.active_host_session_id = normalized
        self.session_generation += 1
        self.session_state = SessionState.SESSION_IDLE
        self.last_fault = "NONE"
        return ModeResult(
            mode,
            True,
            (),
            {
                "boot_session_id": self.boot_session_id,
                "active_host_session_id": normalized,
                "session_generation": self.session_generation,
                "idempotent": False,
            },
        )

    def end_session(self) -> ModeResult:
        mode = "SESSION_END"
        fault = self._require_not_faulted(mode)
        if fault:
            return ModeResult(mode, False, (fault,))
        active = self._require_active_session(mode)
        if active:
            return ModeResult(mode, False, (active,))
        safe = self.safe_off()
        if not safe.accepted:
            return ModeResult(mode, False, ("SESSION_END_SAFE_OFF_FAILED",))
        self._invalidate_census()
        self._clear_session_evidence()
        self.active_host_session_id = None
        self.session_state = SessionState.SESSION_IDLE
        return ModeResult(mode, True, (), {"safe_off_verified": True})

    def status(self) -> ModeResult:
        """Read volatile identity even when SESSION_FAULT is latched."""
        payload = {
            "boot_session_id": self.boot_session_id,
            "active_host_session_id": self.active_host_session_id,
            "session_generation": self.session_generation,
            "session_state": self.session_state.name,
            "census_fresh": self.census_fresh,
            "census_epoch": self.census_epoch,
            "last_fault": self.last_fault,
            "manual_q0_candidates": sum(
                self._binding_is_current(e.binding) for e in self.manual_q0.values()
            ),
            "direction_witnesses": sum(
                self._binding_is_current(e.binding)
                for e in self.direction_witnesses.values()
            ),
            "bootstrap_active": self.bootstrap_approved,
            "joints_characterized": sum(
                self._binding_is_current(e.binding) for e in self.characterized.values()
            ),
            "calibration_candidates": sum(
                self._binding_is_current(e.binding) for e in self.calibrated.values()
            ),
            "host_evidence_authoritative": False,
        }
        return ModeResult("STATUS", True, (), payload)

    # -- gates -------------------------------------------------------------
    def _require_not_faulted(self, mode: str) -> str | None:
        if not self.fault_latched:
            return None
        return f"{mode} refused: SESSION_FAULT_LATCHED ({self.last_fault})"

    def _require_active_session(self, mode: str) -> str | None:
        if self.active_host_session_id is not None:
            return None
        return f"{mode} requires an active persistent host session"

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
        if entry is not None and self._binding_is_current(entry.binding):
            return None
        return f"{mode} requires {bus_id} characterized in this physical session"

    def _require_manual_q0(self, mode: str, bus_id: int) -> str | None:
        entry = self.manual_q0.get(bus_id)
        if entry is not None and self._binding_is_current(entry.binding):
            return None
        return f"{mode} requires a current manual q0 candidate for {bus_id}"

    def _require_all_manual_q0(self, mode: str) -> str | None:
        if all(
            self._require_manual_q0(mode, bus_id) is None
            for bus_id in EXPECTED_LEG_IDS
        ):
            return None
        return f"{mode} requires stored manual q0 evidence for all 12 joints"

    def _require_direction_witness(self, mode: str, bus_id: int) -> str | None:
        entry = self.direction_witnesses.get(bus_id)
        if (
            entry is not None
            and self._binding_is_current(entry.binding)
            and entry.direction in (-1, 1)
        ):
            return None
        return f"{mode} requires a semantic direction witness for {bus_id}"

    def _stateful_reasons(
        self,
        mode: str,
        *,
        stage: HardwareStage | None = None,
        fresh_census: bool = False,
    ) -> list[str]:
        fault = self._require_not_faulted(mode)
        if fault:
            return [fault]
        reasons = [self._require_active_session(mode)]
        if stage is not None:
            reasons.append(self._require_stage(stage, mode))
        if fresh_census:
            reasons.append(self._require_fresh_census(mode))
        return [reason for reason in reasons if reason]

    # -- LEGS_12_CENSUS ----------------------------------------------------
    def census(self) -> CensusResult:
        reasons = self._stateful_reasons("CENSUS")
        if reasons:
            return CensusResult(False, 0, 0, (), (), reasons[0])

        self._invalidate_census()
        self.census_epoch += 1
        # A new census is a new physical setup epoch. No dependent evidence,
        # including H2 q0 or H4 candidates, survives even when this census fails.
        self._clear_session_evidence()

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
        self.session_state = (
            SessionState.SESSION_CENSUS_OK if passed else SessionState.SESSION_IDLE
        )
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
        mode = "CAPTURE_Q0"
        reasons = self._stateful_reasons(
            mode, stage=HardwareStage.H2_MANUAL_Q0, fresh_census=True
        )
        if reasons:
            return ModeResult(mode, False, tuple(reasons))

        from matdog_full_leg_calibrator_derive import capture_manual_q0

        # The capture is atomic. Starting a replacement capture invalidates the
        # old q0-dependent authorization even if the replacement later fails.
        self._clear_session_evidence()
        binding = self._current_binding()
        payload: dict[str, dict] = {}
        staged: dict[int, ManualQ0Evidence] = {}
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
                continue
            staged[bus_id] = ManualQ0Evidence(
                binding=binding,
                centre_tick=candidate.median_tick,
                min_tick=candidate.min_tick,
                max_tick=candidate.max_tick,
                spread_ticks=candidate.spread_ticks,
                samples=candidate.sample_count,
            )

        if accepted and len(staged) == len(EXPECTED_LEG_IDS):
            self.manual_q0.update(staged)
        else:
            self.manual_q0.clear()
        return ModeResult(mode, accepted, (), payload)

    # -- semantic direction witness ---------------------------------------
    def witness_direction(self, bus_id: int, semantic: str) -> ModeResult:
        mode = "WITNESS_DIRECTION"
        reasons = self._stateful_reasons(
            mode, stage=HardwareStage.H2_MANUAL_Q0, fresh_census=True
        )
        if bus_id not in EXPECTED_LEG_IDS:
            reasons.append(f"{bus_id} is not a leg bus id")
        elif not reasons:
            manual_reason = self._require_manual_q0(mode, bus_id)
            if manual_reason:
                reasons.append(manual_reason)

        semantic_map = {
            "Q_PLUS_RAW_INCREASES": 1,
            "Q_PLUS_RAW_DECREASES": -1,
            "increases": 1,
            "decreases": -1,
        }
        direction = semantic_map.get(semantic)
        if direction is None:
            reasons.append(
                "semantic witness must be Q_PLUS_RAW_INCREASES or "
                "Q_PLUS_RAW_DECREASES"
            )
        if reasons:
            return ModeResult(mode, False, tuple(reasons))

        canonical = (
            "Q_PLUS_RAW_INCREASES" if direction == 1 else "Q_PLUS_RAW_DECREASES"
        )
        self.direction_witnesses[bus_id] = DirectionWitness(
            binding=self._current_binding(),
            direction=direction,
            semantic=canonical,
        )
        # Evidence interpreted under an older semantic mapping is unusable.
        self.characterized.pop(bus_id, None)
        self.calibrated.pop(bus_id, None)
        return ModeResult(
            mode,
            True,
            (),
            {
                "bus_id": bus_id,
                "semantic": canonical,
                "direction": direction,
                "meaning": "q=direction*signed_tick_delta(raw,q0)",
            },
        )

    # -- H3 bootstrap approval --------------------------------------------
    def approve_bootstrap(self) -> ModeResult:
        """Arm the conservative first-motion envelope for THIS session."""
        mode = "APPROVE_BOOTSTRAP"
        reasons = self._stateful_reasons(
            mode, stage=HardwareStage.H3_JOINT_CHARACTERIZE, fresh_census=True
        )
        if not reasons:
            manual_reason = self._require_all_manual_q0(mode)
            if manual_reason:
                reasons.append(manual_reason)
        if not self.bootstrap_build_approved:
            reasons.append("APPROVE_BOOTSTRAP requires a bootstrap-enabled H3+ build")
        if reasons:
            return ModeResult(mode, False, tuple(reasons))
        self.bootstrap = BootstrapApproval(self._current_binding())
        return ModeResult(mode, True, (),
                          {"origin": "H3_BOOTSTRAP_OPERATOR_APPROVED",
                           "is_measurement": False})

    # -- motion modes ------------------------------------------------------
    def characterize_joint(
        self,
        bus_id: int,
        succeed: bool = True,
        *,
        raw_probe_sign: int = 1,
    ) -> ModeResult:
        """H3. Gated by stage, census and the pre-motion parameter gate — but
        NOT by acceptance tolerances or a semantic direction value.  The raw
        probe sign records which encoder search direction found a useful path;
        it is not the MATDOG ``q`` direction."""
        mode = "CHARACTERIZE_JOINT"
        reasons = self._stateful_reasons(
            mode, stage=HardwareStage.H3_JOINT_CHARACTERIZE, fresh_census=True
        )
        if bus_id not in EXPECTED_LEG_IDS:
            reasons.append(f"{bus_id} is not a leg bus id")
        elif not reasons:
            for reason in (
                self._require_manual_q0(mode, bus_id),
                self._require_pre_motion_parameters(mode),
            ):
                if reason:
                    reasons.append(reason)
        if raw_probe_sign not in (-1, 1):
            reasons.append("raw_probe_sign must be -1 or +1")
        if reasons:
            return ModeResult(mode, False, tuple(reasons))

        if not succeed:
            self.characterized.pop(bus_id, None)
            self.calibrated.pop(bus_id, None)
            return ModeResult(mode, False,
                              ("simulated characterization failure",))

        # The real measurement happens in the C++ engine; the simulator records
        # the session bookkeeping the firmware performs around it.
        self.characterized[bus_id] = CharacterizationEvidence(
            binding=self._current_binding(), raw_probe_sign=raw_probe_sign
        )
        self.calibrated.pop(bus_id, None)
        return ModeResult(
            mode,
            True,
            (),
            {
                "bus_id": bus_id,
                "origin": "CHARACTERIZED_CURRENT_HARDWARE",
                "raw_probe_sign": raw_probe_sign,
                "semantic_direction": None,
            },
        )

    def _calibration_reasons(self, mode: str, bus_id: int) -> list[str]:
        reasons = self._stateful_reasons(mode, fresh_census=True)
        if bus_id not in EXPECTED_LEG_IDS:
            reasons.append(f"{bus_id} is not a leg bus id")
            return reasons
        if reasons:
            return reasons
        for reason in (
            self._require_pre_motion_parameters(mode),
            self._require_manual_q0(mode, bus_id),
            self._require_direction_witness(mode, bus_id),
            self._require_characterized(mode, bus_id),
        ):
            if reason:
                reasons.append(reason)
        return reasons

    def _store_calibration(
        self, bus_id: int, derived_q0_tick: int | None = None
    ) -> CalibrationEvidence:
        manual = self.manual_q0[bus_id]
        witness = self.direction_witnesses[bus_id]
        derived = manual.centre_tick if derived_q0_tick is None else derived_q0_tick
        if not 0 <= derived <= ENCODER_MAX:
            raise ValueError("derived q0 must be in unsigned raw domain 0..4095")
        evidence = CalibrationEvidence(
            binding=self._current_binding(),
            direction=witness.direction,
            manual_q0_tick=manual.centre_tick,
            derived_q0_tick=derived,
        )
        self.calibrated[bus_id] = evidence
        return evidence

    @staticmethod
    def _calibration_payload(bus_id: int, evidence: CalibrationEvidence) -> dict:
        return {
            "bus_id": bus_id,
            "manual_q0_candidate_tick": evidence.manual_q0_tick,
            "derived_q0_tick": evidence.derived_q0_tick,
            "direction": evidence.direction,
            "direction_origin": "EXPLICIT_CURRENT_BUILD_SEMANTIC_WITNESS",
            "tier": evidence.tier,
            "acceptance": "BLOCKED_TOLERANCE_UNVALIDATED",
            "promotion": "REQUIRES_EXPLICIT_GATE",
        }

    def calibrate_joint(
        self, bus_id: int, *, derived_q0_tick: int | None = None
    ) -> ModeResult:
        mode = "CALIBRATE_JOINT"
        reasons = self._stateful_reasons(
            mode, stage=HardwareStage.H4_JOINT_CALIBRATE, fresh_census=True
        )
        if not reasons:
            reasons.extend(self._calibration_reasons(mode, bus_id))
        if reasons:
            return ModeResult(mode, False, tuple(dict.fromkeys(reasons)))
        try:
            evidence = self._store_calibration(bus_id, derived_q0_tick)
        except ValueError as exc:
            return ModeResult(mode, False, (str(exc),))
        return ModeResult(mode, True, (), self._calibration_payload(bus_id, evidence))

    def calibrate_leg(self, leg: str) -> ModeResult:
        mode = "CALIBRATE_LEG"
        reasons = self._stateful_reasons(
            mode, stage=HardwareStage.H5_LEG, fresh_census=True
        )
        if reasons:
            return ModeResult(mode, False, tuple(reasons))
        prefix = {"LF": 1, "RF": 2, "RH": 3, "LH": 4}.get(leg)
        if prefix is None:
            return ModeResult(mode, False, (f"unknown leg {leg!r}",))
        ids = [i for i in EXPECTED_LEG_IDS if i // 10 == prefix]
        for bus_id in ids:
            reasons.extend(self._calibration_reasons(mode, bus_id))
        if reasons:
            return ModeResult(mode, False, tuple(dict.fromkeys(reasons)))
        candidates = {
            str(bus_id): self._calibration_payload(
                bus_id, self._store_calibration(bus_id)
            )
            for bus_id in ids
        }
        return ModeResult(mode, True, (), {"leg": leg, "joints": ids,
                                           "candidates": candidates})

    def calibrate_all_legs(self) -> ModeResult:
        mode = "CALIBRATE_ALL"
        reasons = self._stateful_reasons(
            mode, stage=HardwareStage.H6_FOUR_LEGS, fresh_census=True
        )
        if not reasons:
            for bus_id in EXPECTED_LEG_IDS:
                reasons.extend(self._calibration_reasons(mode, bus_id))
        if reasons:
            return ModeResult(mode, False, tuple(dict.fromkeys(reasons)))
        for bus_id in EXPECTED_LEG_IDS:
            self._store_calibration(bus_id)
        return ModeResult(mode, True, (),
                          {"legs": ["LF", "RF", "RH", "LH"], "joints": 12,
                           "tier": "CANDIDATE_BLOCKED_TOLERANCE_UNVALIDATED",
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
            self._latch_fault("SAFE_OFF_FAILED")
        return ModeResult(
            "SAFE_OFF",
            not failures,
            tuple(failures),
            {"responders_seen": responders_seen},
        )
