"""MATDOG Full Leg Calibrator V1 — centralized calibration policy and joint specs.

Single source of truth for the host side of the calibrator. Nothing here opens a
serial port or commands a servo; it decides *what is allowed* and *what is still
unknown*, so the runner and the tests agree on both.

Three ideas structure this module.

1. **Provenance is a first-class property of every constant.** A number that
   passed on LF V25 is not a number that may authorize the current hardware. Each
   value carries a :class:`Provenance` tag, and :func:`motion_blockers` refuses
   motion while any safety-critical parameter is still unvalidated.

2. **The four calibration data layers stay separate**, per
   ``09_Logs/Calibration/MATDOG_CALIBRATION_RESET_2026-08-27.md``: the servo's
   physical RAW centre, the mechanical mounting, the software digital-zero
   mapping and the joint calibration. In particular ``manual_pose_q0_candidate``
   and ``derived_q0_final`` are different objects and are never conflated.

3. **Geometry and hardware are independent witnesses.** The URDF/Geometry
   Compiler V5 endpoints describe the design and survived the reassembly. The
   measured contacts describe the build. Neither is edited to make the other
   agree; a disagreement is a structured failure.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Mapping

import yaml

CALIBRATION_DIR = Path(__file__).resolve().parent
MATDOG_CORE = CALIBRATION_DIR.parent
REPO_ROOT = MATDOG_CORE.parent.parent

SERVO_ALLOCATION_PATH = MATDOG_CORE / "config" / "MATDOG_SERVO_ALLOCATION.yaml"
SERVO_PROFILE_PATH = MATDOG_CORE / "config" / "MATDOG_ST3215_C018_V1.yaml"
GEOMETRY_ENDPOINT_PROFILE_PATH = (
    REPO_ROOT
    / "09_Logs"
    / "Validation_Reports"
    / "Geometry_Compiler"
    / "2026-08-11_131818_MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4_ENDPOINT_PROFILE.json"
)

#: Calibrator-local host<->ESP32 protocol. Deliberately NOT the final MATDOG
#: runtime protocol, which remains TBD in 01_Docs/02_Architecture/ARCHITECTURE.md.
PROTOCOL_ID = "FLC1"
PROTOCOL_SCOPE = "CALIBRATOR_LOCAL_NOT_FINAL_RUNTIME_PROTOCOL"

FIRMWARE_SKETCH = (
    REPO_ROOT
    / "05_Firmware"
    / "Full_Leg_Calibrator_V1"
    / "matdog_full_leg_calibrator_v1"
    / "matdog_full_leg_calibrator_v1.ino"
)

ENCODER_MODULUS = 4096
ENCODER_MAX = 4095
#: Servo electrical centre. This is a hardware fact, NOT the joint zero.
RAW_ELECTRICAL_CENTER = 2048

#: Expected leg bus ids. Head ids 51..55 are allocated but not installed.
EXPECTED_LEG_IDS = (11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43)
HEAD_IDS = (51, 52, 53, 54, 55)


class Provenance(Enum):
    """Why a constant is believed, which decides what it may authorize."""

    #: Verified on the current units/build, evidence cited.
    VALIDATED_CURRENT_HARDWARE = "VALIDATED_CURRENT_HARDWARE"
    #: Mechanism reused from frozen MATDOG ESP32 tooling; load-independent.
    REUSABLE_GENERIC_ALGORITHM = "REUSABLE_GENERIC_ALGORITHM"
    #: Measured on an installation that no longer exists. Never authorizes motion.
    HISTORICAL_CANDIDATE_ONLY = "HISTORICAL_CANDIDATE_ONLY"
    #: No current evidence. Hard-blocks motion until measured on this build.
    CHARACTERIZATION_REQUIRED = "CHARACTERIZATION_REQUIRED"


class ParameterClass(Enum):
    """WHEN a parameter can be known — which decides what it may gate.

    The first version of this policy treated all eight contact parameters as
    pre-motion blockers. That was a deadlock: H4 needed them, H3 was supposed to
    measure them, and H3 was blocked by them. Classifying by *when* a value can
    exist breaks the circle without inventing numbers.
    """

    #: Must be known BEFORE any H3 motion. Cannot be measured first, so it comes
    #: from the explicitly approved, deliberately conservative bootstrap envelope.
    A_PRE_MOTION = "A_PRE_MOTION"
    #: Measured DURING H3 on this build.
    B_MEASURED_H3 = "B_MEASURED_H3"
    #: Computed from H3/H4 data; never a standalone constant.
    C_DERIVED = "C_DERIVED"
    #: Post-measure acceptance gate. Decides whether a result is ACCEPTED, and
    #: must NEVER block acquiring the measurement it exists to judge.
    D_ACCEPTANCE = "D_ACCEPTANCE"


class ParameterOrigin(Enum):
    """Where a runtime number came from. A bootstrap value and a measured value
    are different things and must never be readable as one another."""

    NONE = "NONE"
    H3_BOOTSTRAP_OPERATOR_APPROVED = "H3_BOOTSTRAP_OPERATOR_APPROVED"
    CHARACTERIZED_CURRENT_HARDWARE = "CHARACTERIZED_CURRENT_HARDWARE"


class HardwareStage(Enum):
    """Progressive hardware validation gates. See section 9 of the V1 spec."""

    H0_ESP32_ONLY = 0
    H1_CENSUS_READONLY = 1
    H2_MANUAL_Q0 = 2
    H3_JOINT_CHARACTERIZE = 3
    H4_JOINT_CALIBRATE = 4
    H5_LEG = 5
    H6_FOUR_LEGS = 6
    H7_FREEZE = 7


#: The stage this build is authorized to execute. Raising it is a deliberate
#: source change made only once the evidence for the next stage exists.
AUTHORIZED_STAGE = HardwareStage.H0_ESP32_ONLY


class CalibrationPolicyError(RuntimeError):
    """Raised when the policy refuses an operation. Always fail closed."""


@dataclass(frozen=True)
class PolicyValue:
    """One calibration constant plus the reason it is or is not trustworthy."""

    name: str
    value: int | float | None
    provenance: Provenance
    evidence: str
    unit: str = ""
    parameter_class: ParameterClass | None = None

    @property
    def resolved(self) -> bool:
        return self.value is not None

    @property
    def may_authorize_motion(self) -> bool:
        """Only current-hardware evidence and load-independent mechanism may."""
        if not self.resolved:
            return False
        return self.provenance in (
            Provenance.VALIDATED_CURRENT_HARDWARE,
            Provenance.REUSABLE_GENERIC_ALGORITHM,
        )


# ---------------------------------------------------------------------------
# Servo identity invariants — current hardware, provisioning campaign 2026-08-27
# ---------------------------------------------------------------------------

SERVO_IDENTITY = (
    PolicyValue(
        "EXPECTED_MODEL", 777, Provenance.VALIDATED_CURRENT_HARDWARE,
        "MATDOG_ST3215_C018_V1.yaml model_register 0x03, observed on 17/17 units",
    ),
    PolicyValue(
        "EXPECTED_POSITION_OFFSET", 0, Provenance.VALIDATED_CURRENT_HARDWARE,
        "provisioning campaign wrote PositionOffset=0 on all 17 units; "
        "rewriting it to compensate mounting is permanently forbidden",
    ),
    PolicyValue(
        "EXPECTED_RESPONSE_STATUS", 1, Provenance.VALIDATED_CURRENT_HARDWARE,
        "preserve_only ResponseStatus observed_on_all_17: 1",
    ),
    PolicyValue(
        "EXPECTED_BAUD_REGISTER", 0, Provenance.VALIDATED_CURRENT_HARDWARE,
        "baud policy VERIFICATION_ONLY_IN_V1, expected 0 == 1 Mbps",
    ),
    PolicyValue(
        "GOAL_POSITION_MIN", 0, Provenance.VALIDATED_CURRENT_HARDWARE,
        "goal_position_domain [0, 4095]; signed wrap FORBIDDEN",
    ),
    PolicyValue(
        "GOAL_POSITION_MAX", 4095, Provenance.VALIDATED_CURRENT_HARDWARE,
        "goal_position_domain [0, 4095]; signed wrap FORBIDDEN",
    ),
)

# ---------------------------------------------------------------------------
# Generic servo-protection guards — mechanism level, load independent
# ---------------------------------------------------------------------------

GENERIC_GUARDS = (
    PolicyValue(
        "THERMAL_LIMIT_C", 70, Provenance.VALIDATED_CURRENT_HARDWARE,
        "MaxTemperature 0x0D = 70 in the frozen MATDOG_C018_V1 profile", "degC",
    ),
    PolicyValue(
        "VOLTAGE_MIN", 40, Provenance.VALIDATED_CURRENT_HARDWARE,
        "MinVoltage 0x0F = 40 in the frozen MATDOG_C018_V1 profile", "0.1V",
    ),
    PolicyValue(
        "VOLTAGE_MAX", 140, Provenance.VALIDATED_CURRENT_HARDWARE,
        "MaxVoltage 0x0E = 140 in the frozen MATDOG_C018_V1 profile", "0.1V",
    ),
    PolicyValue(
        "PROTECTION_CURRENT_RAW", 310, Provenance.VALIDATED_CURRENT_HARDWARE,
        "ProtectionCurrent 0x1C = 310 in the frozen MATDOG_C018_V1 profile", "raw",
    ),
    PolicyValue(
        "MON_PERIOD_US", 2000, Provenance.REUSABLE_GENERIC_ALGORITHM,
        "QC V6.1 QC_FAST_PERIOD_US, 26-run bench campaign", "us",
    ),
    PolicyValue(
        "WRITE_SETTLE_MS", 20, Provenance.REUSABLE_GENERIC_ALGORITHM,
        "Provisioner V6 WRITE_SETTLE_MS, write/readback settling", "ms",
    ),
    PolicyValue(
        "THERMAL_CONFIRMATIONS", 3, Provenance.REUSABLE_GENERIC_ALGORITHM,
        "QC V6.1 MON_THERMAL_CONFIRMATIONS; rejects single-sample thermal spikes",
    ),
    PolicyValue(
        "STATIONARY_SPEED_RAW", 10, Provenance.REUSABLE_GENERIC_ALGORITHM,
        "QC V6.1 MON_STATIONARY_SPEED; PresentSpeed magnitude counting as stopped",
    ),
    PolicyValue(
        "TELEMETRY_LOSS_SAMPLES", 25, Provenance.REUSABLE_GENERIC_ALGORITHM,
        "QC V6.1 MON_TELEMETRY_LOSS_SAMPLES, 50 ms at 2 ms period",
    ),
)

# ---------------------------------------------------------------------------
# LF V25 historical candidates — recorded, never authorizing
# ---------------------------------------------------------------------------

HISTORICAL_CANDIDATES = (
    PolicyValue(
        "LF_V25_TORQUE_LIMIT", 500, Provenance.HISTORICAL_CANDIDATE_ONLY,
        "matdog.rs TORQUE_LIMIT; previous installation. The 2026-07-10 "
        "pre-recenter backup shows only old bus ids 13 and 23 carried runtime "
        "TorqueLimit 500 while the other ten carried 1000 — the fleet was NOT "
        "in a homogeneous runtime torque state when those numbers were taken",
    ),
    PolicyValue(
        "LF_V25_GOAL_SPEED", 160, Provenance.HISTORICAL_CANDIDATE_ONLY,
        "matdog.rs GOAL_SPEED; previous installation",
    ),
    PolicyValue(
        "LF_V25_ACCELERATION", 8, Provenance.HISTORICAL_CANDIDATE_ONLY,
        "matdog.rs ACCELERATION; previous installation",
    ),
    PolicyValue(
        "LF_V25_COARSE_STEP_TICKS", 64, Provenance.HISTORICAL_CANDIDATE_ONLY,
        "matdog.rs COARSE_STEP_TICKS; previous installation",
    ),
    PolicyValue(
        "LF_V25_FINE_STEP_TICKS", 8, Provenance.HISTORICAL_CANDIDATE_ONLY,
        "matdog.rs FINE_STEP_TICKS; previous installation",
    ),
    PolicyValue(
        "LF_V25_BACKOFF_TICKS", 96, Provenance.HISTORICAL_CANDIDATE_ONLY,
        "matdog.rs BACKOFF_TICKS; previous installation",
    ),
    PolicyValue(
        "LF_V25_HARD_CURRENT_ABORT_RAW", 200, Provenance.HISTORICAL_CANDIDATE_ONLY,
        "matdog.rs HARD_CURRENT_ABORT_RAW; previous installation",
    ),
    PolicyValue(
        "LF_V25_MIN_CONTACT_TRAVEL_TICKS", 24, Provenance.HISTORICAL_CANDIDATE_ONLY,
        "matdog.rs MINIMUM_CONTACT_TRAVEL_TICKS; previous installation",
    ),
    PolicyValue(
        "LF_V25_REPEATABILITY_TOLERANCE_TICKS", 16, Provenance.HISTORICAL_CANDIDATE_ONLY,
        "matdog.rs REPEATABILITY_TOLERANCE_TICKS; previous installation",
    ),
    PolicyValue(
        "PROVISIONER_CENTER_TORQUE_LIMIT", 300, Provenance.HISTORICAL_CANDIDATE_ONLY,
        "Provisioner V6 CENTER_TORQUE_LIMIT. Validated for BENCH FREE-SHAFT "
        "centering with no mechanical load; it says nothing about driving an "
        "assembled leg into a mechanical endstop",
    ),
)

# ---------------------------------------------------------------------------
# Parameters that require new hardware characterization. These block motion.
# Mirrors CHARACTERIZATION[] in the firmware; kept in sync by a test.
# ---------------------------------------------------------------------------

CHARACTERIZATION_REQUIRED = (
    PolicyValue(
        "CONTACT_TORQUE_LIMIT", None, Provenance.CHARACTERIZATION_REQUIRED,
        "needed to move at all, so it cannot be measured first. Supplied by the "
        "approved H3 bootstrap envelope, which is deliberately below both the "
        "provisioner bench value (300) and LF V25 (500), and refined by H3",
        parameter_class=ParameterClass.A_PRE_MOTION,
    ),
    PolicyValue(
        "CONTACT_GOAL_SPEED", None, Provenance.CHARACTERIZATION_REQUIRED,
        "needed to move at all; bootstrap 60 is far below LF V25 160",
        parameter_class=ParameterClass.A_PRE_MOTION,
    ),
    PolicyValue(
        "CONTACT_ACCELERATION", None, Provenance.CHARACTERIZATION_REQUIRED,
        "needed to move at all; bootstrap matches the slowest historical value",
        parameter_class=ParameterClass.A_PRE_MOTION,
    ),
    PolicyValue(
        "CONTACT_RETREAT_TICKS", None, Provenance.CHARACTERIZATION_REQUIRED,
        "H3 retreats with the bootstrap distance and reports what was actually "
        "achieved; H4 then uses the measured value",
        parameter_class=ParameterClass.B_MEASURED_H3,
    ),
    PolicyValue(
        "CONTACT_CURRENT_THRESHOLD_RAW", None, Provenance.CHARACTERIZATION_REQUIRED,
        "NOT a global constant. The detector derives its contact threshold from "
        "the per-joint free-motion median/MAD baseline H3 measures, so no "
        "fleet-wide current threshold is needed or wanted",
        parameter_class=ParameterClass.C_DERIVED,
    ),
    PolicyValue(
        "CONTACT_REPEATABILITY_TOLERANCE_TICKS", None, Provenance.CHARACTERIZATION_REQUIRED,
        "judges whether two contacts agree. Post-measure: it cannot gate the "
        "approaches whose spread it evaluates. H3 measures the spread and derives "
        "the band from it",
        parameter_class=ParameterClass.D_ACCEPTANCE,
    ),
    PolicyValue(
        "ENDPOINT_VS_URDF_TOLERANCE_TICKS", None, Provenance.CHARACTERIZATION_REQUIRED,
        "judges measured span against URDF geometry. Post-measure only: an "
        "unknown band leaves the result CANDIDATE rather than blocking the "
        "measurement",
        parameter_class=ParameterClass.D_ACCEPTANCE,
    ),
    PolicyValue(
        "MANUAL_Q0_VS_DERIVED_Q0_TOLERANCE_TICKS", None, Provenance.CHARACTERIZATION_REQUIRED,
        "judges manual pose against derived q0. It must never prevent acquiring "
        "the data needed to derive q0 in the first place",
        parameter_class=ParameterClass.D_ACCEPTANCE,
    ),
)

# ---------------------------------------------------------------------------
# H3 bootstrap envelope
#
# The honest answer to "how do you move a joint you have not characterized yet".
# Deliberately gentler than every historical envelope, separate from any
# characterized result, inert unless explicitly approved, and never canonical.
# Mirrors flc_stage_config.h; a test pins the two together.
# ---------------------------------------------------------------------------

BOOTSTRAP_ENVELOPE = {
    "origin": ParameterOrigin.H3_BOOTSTRAP_OPERATOR_APPROVED,
    "torque_limit": 200,
    "goal_speed": 60,
    "acceleration": 8,
    "retreat_ticks": 96,
    "is_measurement": False,
    "may_become_canonical": False,
    "requires_build_flag": "FLC_H3_BOOTSTRAP_APPROVED=1",
    "requires_session_confirmation": "@APPROVE_BOOTSTRAP CONFIRM",
    "evidence": (
        "NOT a measurement of this build and NOT inherited from LF V25. Chosen "
        "strictly below every historical envelope so the first motion on the "
        "reassembled robot is the gentlest anyone has run: torque 200 < "
        "provisioner 300 < LF V25 500; speed 60 << LF V25 160."
    ),
}

#: Hard ceilings no envelope may exceed. Mirrors flc_stage_config.h.
ABSOLUTE_CEILINGS = {
    "torque_limit": 500,
    "goal_speed": 400,
    "acceleration": 50,
    "travel_budget_ticks": 1800,
    "time_budget_ms": 30000,
}

ALL_POLICY_VALUES = (
    SERVO_IDENTITY + GENERIC_GUARDS + HISTORICAL_CANDIDATES + CHARACTERIZATION_REQUIRED
)


def policy_value(name: str) -> PolicyValue:
    for value in ALL_POLICY_VALUES:
        if value.name == name:
            return value
    raise KeyError(f"unknown policy value: {name}")


def parameters_in_class(cls: ParameterClass) -> list[PolicyValue]:
    return [v for v in CHARACTERIZATION_REQUIRED if v.parameter_class is cls]


def pre_motion_blockers() -> list[PolicyValue]:
    """Unresolved CLASS_A parameters — the only ones that may block first motion.

    CLASS_D acceptance tolerances deliberately do NOT appear here: a tolerance
    that judges a measurement cannot be a precondition of taking it.
    """
    return [
        v for v in parameters_in_class(ParameterClass.A_PRE_MOTION)
        if not v.may_authorize_motion
    ]


def motion_blockers() -> list[PolicyValue]:
    """Backwards-compatible name for the pre-motion gate."""
    return pre_motion_blockers()


def acceptance_gates_unresolved() -> list[PolicyValue]:
    """CLASS_D tolerances still unknown. These cap a result at CANDIDATE; they
    never prevent measuring."""
    return [
        v for v in parameters_in_class(ParameterClass.D_ACCEPTANCE)
        if not v.resolved
    ]


def motion_authorized(
    stage: HardwareStage = AUTHORIZED_STAGE,
    bootstrap_approved: bool = False,
) -> bool:
    """True only when the stage gate AND the pre-motion parameter gate agree.

    H3 is the lowest stage at which anything may move. The pre-motion parameters
    are satisfied either by being resolved outright or by an explicitly approved
    bootstrap envelope — never by promoting a historical value.
    """
    if stage.value < HardwareStage.H3_JOINT_CHARACTERIZE.value:
        return False
    return not pre_motion_blockers() or bootstrap_approved


def require_motion_authorized(
    mode: str,
    stage: HardwareStage = AUTHORIZED_STAGE,
    bootstrap_approved: bool = False,
) -> None:
    """Fail closed before any hardware-motion operation."""
    reasons: list[str] = []
    if stage.value < HardwareStage.H3_JOINT_CHARACTERIZE.value:
        reasons.append(
            f"hardware stage {stage.name} is below H3_JOINT_CHARACTERIZE"
        )
    blockers = pre_motion_blockers()
    if blockers and not bootstrap_approved:
        reasons.append(
            "unresolved pre-motion parameters and no approved bootstrap envelope: "
            + ", ".join(b.name for b in blockers)
        )
    if reasons:
        raise CalibrationPolicyError(
            f"MOTION BLOCKED — {mode}\n  " + "\n  ".join(reasons) +
            "\n  No LF V25 numeric result may authorize current hardware."
        )


def require_calibration_authorized(
    mode: str,
    stage: HardwareStage = AUTHORIZED_STAGE,
    characterized_joints: int = 0,
) -> None:
    """H4+ additionally requires at least one joint characterized this session."""
    if stage.value < HardwareStage.H4_JOINT_CALIBRATE.value:
        raise CalibrationPolicyError(
            f"CALIBRATION BLOCKED — {mode}\n"
            f"  hardware stage {stage.name} is below H4_JOINT_CALIBRATE"
        )
    if characterized_joints <= 0:
        raise CalibrationPolicyError(
            f"CALIBRATION BLOCKED — {mode}\n"
            "  no joint has been characterized in this physical session; "
            "run CHARACTERIZE_JOINT first"
        )


# ---------------------------------------------------------------------------
# Joint specification — one generic engine, twelve data rows
# ---------------------------------------------------------------------------

LEG_ORDER = ("LF", "RF", "RH", "LH")
JOINT_KINDS = ("HIP", "UPPER", "LOWER")


@dataclass(frozen=True)
class JointSpec:
    """Everything the generic engine needs about one leg joint.

    ``direction`` is deliberately ``None``. The pre-2026-08-27 directions in
    ``MATDOG_JOINT_CALIBRATION.yaml`` describe an installation that no longer
    exists; encoder sign must be measured on this build before it may be used.
    """

    bus_id: int
    joint_name: str
    unit_label: str
    leg: str
    kind: str
    declared_min_rad: float
    declared_max_rad: float
    geometric_contact_min_rad: float
    geometric_contact_max_rad: float
    direction: int | None = None

    @property
    def direction_known(self) -> bool:
        return self.direction in (-1, 1)


def _load_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise CalibrationPolicyError(f"required config not found: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, Mapping):
        raise CalibrationPolicyError(f"config is not a mapping: {path}")
    return dict(data)


def load_leg_allocation(path: Path | None = None) -> dict[int, dict[str, Any]]:
    """Read the authoritative bus-id -> physical-unit map for the 12 leg servos.

    Verified against :data:`EXPECTED_LEG_IDS` rather than trusted blindly, and
    the servo-level invariants (PositionOffset 0, profile) are asserted here so a
    corrupted allocation cannot silently authorize a census.
    """
    data = _load_yaml(path or SERVO_ALLOCATION_PATH)
    units = data.get("units")
    if not isinstance(units, list):
        raise CalibrationPolicyError("allocation has no 'units' list")

    by_id: dict[int, dict[str, Any]] = {}
    for entry in units:
        bus_id = entry.get("bus_id")
        if bus_id not in EXPECTED_LEG_IDS:
            continue  # head ids 51..55 are allocated but not installed
        if bus_id in by_id:
            raise CalibrationPolicyError(f"duplicate bus_id in allocation: {bus_id}")
        if entry.get("position_offset") != 0:
            raise CalibrationPolicyError(
                f"bus_id {bus_id} has non-zero position_offset in the allocation"
            )
        by_id[bus_id] = dict(entry)

    missing = sorted(set(EXPECTED_LEG_IDS) - set(by_id))
    if missing:
        raise CalibrationPolicyError(f"allocation is missing leg bus ids: {missing}")
    if data.get("profile") != "MATDOG_C018_V1":
        raise CalibrationPolicyError("allocation profile is not MATDOG_C018_V1")
    return by_id


def load_geometry_endpoints(path: Path | None = None) -> dict[str, dict[str, float]]:
    """Read URDF/Geometry Compiler V5 expected endpoint angles per joint.

    Geometry describes the design and survived the 2026-08-27 reassembly; the
    calibration of the physical build did not. These angles are therefore a valid
    cross-check witness while every hardware number is being re-measured.

    Deliberately makes no symmetry assumption: the hip endpoints are not equal
    across legs and not sign-symmetric within a leg.
    """
    target = path or GEOMETRY_ENDPOINT_PROFILE_PATH
    if not target.is_file():
        raise CalibrationPolicyError(f"geometry endpoint profile not found: {target}")
    data = json.loads(target.read_text(encoding="utf-8"))

    endpoints: dict[str, dict[str, float]] = {}
    for search in data.get("endpoint_searches", []):
        identity = search.get("identity", {})
        joint = identity.get("joint_name")
        side = identity.get("limit_side")
        contact = search.get("geometric_contact", {})
        if not joint or side not in ("min", "max"):
            continue
        if contact.get("status") != "GEOMETRIC_CONTACT_FOUND":
            continue
        angle = contact.get("angle_rad")
        if angle is None:
            continue
        endpoints.setdefault(joint, {})
        endpoints[joint][side] = float(angle)
        endpoints[joint][f"declared_{side}"] = float(search["declared_limit_rad"])
    return endpoints


def build_joint_specs(
    allocation: Mapping[int, Mapping[str, Any]] | None = None,
    endpoints: Mapping[str, Mapping[str, float]] | None = None,
) -> tuple[JointSpec, ...]:
    """Compose the 12 joint specs from allocation + geometry, verifying both."""
    allocation = allocation if allocation is not None else load_leg_allocation()
    endpoints = endpoints if endpoints is not None else load_geometry_endpoints()

    # allocation joint labels are LF_HIP style; geometry uses lf_hip_joint style.
    def geometry_key(alloc_joint: str) -> str:
        leg, _, kind = alloc_joint.partition("_")
        suffix = {"HIP": "hip_joint", "UPPER": "upper_leg_joint", "LOWER": "lower_leg_joint"}
        return f"{leg.lower()}_{suffix[kind]}"

    specs: list[JointSpec] = []
    for bus_id in EXPECTED_LEG_IDS:
        entry = allocation[bus_id]
        alloc_joint = str(entry["joint"])
        leg, _, kind = alloc_joint.partition("_")
        joint_name = geometry_key(alloc_joint)
        geometry = endpoints.get(joint_name)
        if geometry is None:
            raise CalibrationPolicyError(
                f"no geometric endpoint for {joint_name}; refusing to guess"
            )
        for required in ("min", "max", "declared_min", "declared_max"):
            if required not in geometry:
                raise CalibrationPolicyError(
                    f"{joint_name} geometry is missing '{required}'"
                )
        specs.append(
            JointSpec(
                bus_id=bus_id,
                joint_name=joint_name,
                unit_label=str(entry["unit"]),
                leg=leg,
                kind=kind,
                declared_min_rad=geometry["declared_min"],
                declared_max_rad=geometry["declared_max"],
                geometric_contact_min_rad=geometry["min"],
                geometric_contact_max_rad=geometry["max"],
                direction=None,
            )
        )
    return tuple(specs)


@dataclass
class CensusExpectation:
    """What a LEGS_12 census must find, and what it must tolerate missing."""

    expected_leg_ids: tuple[int, ...] = EXPECTED_LEG_IDS
    head_ids: tuple[int, ...] = HEAD_IDS
    #: The head is not built yet; its absence is expected, never a failure.
    head_absence_expected: bool = True
    forbid_unexpected_responders: bool = True
    require_torque_off: bool = True
    require_profile_match: bool = True
    expected_model: int = 777
    expected_position_offset: int = 0
    forbidden_operations: tuple[str, ...] = field(
        default_factory=lambda: (
            "CalibrationOfs",
            "one_key_middle",
            "factory_reset",
            "broadcast_write",
            "PositionOffset_rewrite",
            "servo_id_rewrite",
            "persistent_profile_rewrite",
            "any_EEPROM_write",
        )
    )
