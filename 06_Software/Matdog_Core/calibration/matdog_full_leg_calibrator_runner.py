#!/usr/bin/env python3
"""MATDOG Full Leg Calibrator V1 — host runner and session evidence manager.

Talks to ``matdog_full_leg_calibrator_v1.ino`` over USB CDC. NormaCore Station is
not involved: this process opens the serial port directly and the ESP32-S3 owns
every servo-bus write.

The host is deliberately weak by design. It cannot compose a servo write, cannot
stream GoalPosition and cannot lift a gate — there is no ``--register``,
``--address``, ``--raw-goal-position``, ``--write-byte`` or ``--write-word``
option, and there never should be. It asks the firmware for a *named* operation
and records what came back. The policy checks it performs are a second barrier
in front of the firmware's own, not the primary one.

Every run produces a self-contained session directory whose metadata describes
what *actually happened* in that run rather than a hard-coded assumption.

Usage
-----
    matdog_full_leg_calibrator_runner.py h0-smoke              # ESP32 only
    matdog_full_leg_calibrator_runner.py status
    matdog_full_leg_calibrator_runner.py census                # H1
    matdog_full_leg_calibrator_runner.py capture-q0            # H2
    matdog_full_leg_calibrator_runner.py approve-bootstrap     # arm H3
    matdog_full_leg_calibrator_runner.py characterize-joint --joint 13   # H3
    matdog_full_leg_calibrator_runner.py calibrate-joint --joint 13      # H4
    matdog_full_leg_calibrator_runner.py calibrate-leg --leg LF          # H5
    matdog_full_leg_calibrator_runner.py calibrate-all                   # H6
    matdog_full_leg_calibrator_runner.py safe-off
    matdog_full_leg_calibrator_runner.py session                        # one link
    matdog_full_leg_calibrator_runner.py session --script workflow.txt
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import re
import shlex
import subprocess
import sys
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import TYPE_CHECKING, Any, Callable, Iterable

if TYPE_CHECKING:  # pragma: no cover - typing only, never imported at runtime
    import serial

CALIBRATION_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(CALIBRATION_DIR))

from matdog_full_leg_calibrator_policy import (  # noqa: E402
    AUTHORIZED_STAGE,
    BOOTSTRAP_ENVELOPE,
    EXPECTED_LEG_IDS,
    FIRMWARE_GEOMETRY_PLAN,
    FIRMWARE_SKETCH,
    GEOMETRY_ENDPOINT_PROFILE_PATH,
    GEOMETRY_PARKING_PATH,
    GEOMETRY_PLAN_GENERATOR,
    GEOMETRY_SAFETY_POLICY_PATH,
    PROTOCOL_ID,
    PROTOCOL_SCOPE,
    SERVO_ALLOCATION_PATH,
    CalibrationPolicyError,
    HardwareStage,
    acceptance_gates_unresolved,
    build_joint_specs,
    pre_motion_blockers,
    require_motion_authorized,
)

REPO_ROOT = CALIBRATION_DIR.parents[2]
SESSION_ROOT = REPO_ROOT / "09_Logs" / "Validation_Reports" / "Full_Leg_Calibrator_V1"

DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00"
)
DEFAULT_BAUD = 115200
FQBN = (
    "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,"
    "CPUFreq=240,FlashMode=qio,FlashSize=16M,"
    "PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi"
)

#: Terminal markers. Reading stops as soon as one arrives, so a command never
#: relies on a fixed sleep or a guessed line count.
END_MARKERS = (
    "STATUS_END", "CENSUS_END", "CAPTURE_Q0_END", "SAFE_OFF_END",
    "CHARACTERIZE_JOINT_END", "APPROVE_BOOTSTRAP_END",
    "CALIBRATE_JOINT_END", "CALIBRATE_LEG_END", "CALIBRATE_ALL_END",
    "WITNESS_DIRECTION_END", "SESSION_BEGIN_END", "SESSION_END_END",
)

BEGIN_MARKERS = tuple(marker.removesuffix("_END") + "_BEGIN" for marker in END_MARKERS)

_SEMANTIC_COMMAND_PATTERNS = tuple(
    re.compile(pattern)
    for pattern in (
        r"@STATUS",
        r"@SAFE_OFF",
        r"@CENSUS",
        r"@CAPTURE_Q0 (?:[1-9][0-9]{0,2})",
        r"@APPROVE_BOOTSTRAP CONFIRM",
        r"@WITNESS_DIRECTION (?:11|12|13|21|22|23|31|32|33|41|42|43) "
        r"(?:Q_PLUS_RAW_INCREASES|Q_PLUS_RAW_DECREASES) CONFIRM",
        r"@CHARACTERIZE_JOINT (?:11|12|13|21|22|23|31|32|33|41|42|43)",
        r"@CALIBRATE_JOINT (?:11|12|13|21|22|23|31|32|33|41|42|43)",
        r"@CALIBRATE_LEG (?:LF|RF|RH|LH)",
        r"@CALIBRATE_ALL",
        r"@SESSION_BEGIN [0-9a-f]{8}",
        r"@SESSION_END",
    )
)


def semantic_command_allowed(command: str) -> bool:
    """True only for one complete command in the fixed calibrator vocabulary."""
    return "\n" not in command and "\r" not in command and any(
        pattern.fullmatch(command) for pattern in _SEMANTIC_COMMAND_PATTERNS
    )

#: Which hardware stage each runner mode needs, and whether it can move a servo.
MODE_STAGE: dict[str, tuple[HardwareStage, bool]] = {
    "session": (HardwareStage.H0_ESP32_ONLY, False),
    "status": (HardwareStage.H0_ESP32_ONLY, False),
    "h0-smoke": (HardwareStage.H0_ESP32_ONLY, False),
    "safe-off": (HardwareStage.H0_ESP32_ONLY, False),
    "census": (HardwareStage.H1_CENSUS_READONLY, False),
    "capture-q0": (HardwareStage.H2_MANUAL_Q0, False),
    "approve-bootstrap": (HardwareStage.H3_JOINT_CHARACTERIZE, False),
    "witness-direction": (HardwareStage.H2_MANUAL_Q0, False),
    "characterize-joint": (HardwareStage.H3_JOINT_CHARACTERIZE, True),
    "calibrate-joint": (HardwareStage.H4_JOINT_CALIBRATE, True),
    "calibrate-leg": (HardwareStage.H5_LEG, True),
    "calibrate-all": (HardwareStage.H6_FOUR_LEGS, True),
}

# These operations depend on volatile evidence produced earlier in the same
# firmware boot. A one-shot invocation would reset the ESP32 before it could use
# that evidence, so the public runner keeps them on the persistent session path.
PERSISTENT_ONLY_MODES = {
    "capture-q0",
    "approve-bootstrap",
    "witness-direction",
    "characterize-joint",
    "calibrate-joint",
    "calibrate-leg",
    "calibrate-all",
}


class CalibratorProtocolError(RuntimeError):
    """The USB stream did not satisfy the framed calibrator protocol."""


class CalibratorResetDetected(CalibratorProtocolError):
    """The ESP32 rebooted or its volatile physical-session identity changed."""


class CalibratorLinkLost(CalibratorProtocolError):
    """The one permitted USB connection became unusable."""


@dataclass(frozen=True)
class OperationSpec:
    mode: str
    command: str
    result_marker: str
    begin_marker: str
    end_marker: str
    timeout: float
    servo_ids: tuple[int, ...] = ()


@dataclass(frozen=True)
class OperationOutcome:
    mode: str
    status: str
    lines: tuple[str, ...] = ()
    reason: str = ""

    @property
    def passed(self) -> bool:
        return self.status == "PASS"


@dataclass(frozen=True)
class WorkflowAction:
    """One parsed operator-level action; never a raw firmware command."""

    action: str
    joint: int | None = None
    leg: str | None = None
    samples: int = 64
    q_plus_raw: str | None = None
    label: str = ""


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else ""


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def operation_spec(
    mode: str,
    *,
    joint: int | None = None,
    leg: str | None = None,
    samples: int = 64,
    q_plus_raw: str | None = None,
) -> OperationSpec:
    """Translate one semantic host operation to the fixed firmware vocabulary."""
    if mode not in MODE_STAGE:
        raise ValueError(f"unknown calibrator operation {mode!r}")
    if joint is not None and joint not in EXPECTED_LEG_IDS:
        raise ValueError(f"joint must be one of {list(EXPECTED_LEG_IDS)}")

    if mode == "status":
        return OperationSpec(mode, "@STATUS", "STATUS_RESULT", "STATUS_BEGIN",
                             "STATUS_END", 30.0)
    if mode == "safe-off":
        return OperationSpec(mode, "@SAFE_OFF", "SAFE_OFF_RESULT", "SAFE_OFF_BEGIN",
                             "SAFE_OFF_END", 90.0, tuple(EXPECTED_LEG_IDS))
    if mode == "census":
        return OperationSpec(mode, "@CENSUS", "CENSUS_RESULT", "CENSUS_BEGIN",
                             "CENSUS_END", 120.0, tuple(EXPECTED_LEG_IDS))
    if mode == "capture-q0":
        bounded = max(1, min(int(samples), 256))
        return OperationSpec(
            mode, f"@CAPTURE_Q0 {bounded}", "CAPTURE_Q0_RESULT",
            "CAPTURE_Q0_BEGIN", "CAPTURE_Q0_END", 180.0,
            tuple(EXPECTED_LEG_IDS),
        )
    if mode == "approve-bootstrap":
        return OperationSpec(
            mode, "@APPROVE_BOOTSTRAP CONFIRM", "APPROVE_BOOTSTRAP_RESULT",
            "APPROVE_BOOTSTRAP_BEGIN", "APPROVE_BOOTSTRAP_END", 30.0,
        )
    if mode == "witness-direction":
        if joint is None:
            raise ValueError("witness-direction requires a joint")
        semantic = {
            "increases": "Q_PLUS_RAW_INCREASES",
            "decreases": "Q_PLUS_RAW_DECREASES",
        }.get(q_plus_raw or "")
        if semantic is None:
            raise ValueError("witness-direction requires q_plus_raw increases|decreases")
        return OperationSpec(
            mode, f"@WITNESS_DIRECTION {joint} {semantic} CONFIRM",
            "WITNESS_DIRECTION_RESULT", "WITNESS_DIRECTION_BEGIN",
            "WITNESS_DIRECTION_END", 30.0, (joint,),
        )
    if mode in ("characterize-joint", "calibrate-joint"):
        if joint is None:
            raise ValueError(f"{mode} requires a joint")
        verb = "CHARACTERIZE_JOINT" if mode == "characterize-joint" else "CALIBRATE_JOINT"
        timeout = 300.0 if mode == "characterize-joint" else 600.0
        return OperationSpec(
            mode, f"@{verb} {joint}", f"{verb}_RESULT", f"{verb}_BEGIN",
            f"{verb}_END", timeout, (joint,),
        )
    if mode == "calibrate-leg":
        if leg not in ("LF", "RF", "RH", "LH"):
            raise ValueError("calibrate-leg requires leg LF|RF|RH|LH")
        prefix = {"LF": 1, "RF": 2, "RH": 3, "LH": 4}[leg]
        ids = tuple(bus_id for bus_id in EXPECTED_LEG_IDS if bus_id // 10 == prefix)
        return OperationSpec(
            mode, f"@CALIBRATE_LEG {leg}", "CALIBRATE_LEG_RESULT",
            "CALIBRATE_LEG_BEGIN", "CALIBRATE_LEG_END", 1200.0, ids,
        )
    if mode == "calibrate-all":
        return OperationSpec(
            mode, "@CALIBRATE_ALL", "CALIBRATE_ALL_RESULT",
            "CALIBRATE_ALL_BEGIN", "CALIBRATE_ALL_END", 3600.0,
            tuple(EXPECTED_LEG_IDS),
        )
    raise ValueError(f"{mode!r} is a workflow, not a single firmware operation")


def _git(*args: str) -> str:
    result = subprocess.run(
        ["git", *args], cwd=REPO_ROOT, capture_output=True, text=True
    )
    return result.stdout.strip() if result.returncode == 0 else ""


def git_commit() -> str:
    return _git("rev-parse", "HEAD") or "UNKNOWN"


def git_dirty() -> bool:
    """True when the working tree differs from HEAD.

    Recorded in every session: a run made from a dirty tree is NOT reproducible
    from its commit, and the evidence must say so rather than imply otherwise.
    """
    return bool(_git("status", "--porcelain"))


def firmware_binary_path() -> Path | None:
    """Path arduino-cli --export-binaries writes the application image to."""
    build = (
        FIRMWARE_SKETCH.parent / "build" / "esp32.esp32.esp32s3"
        / "matdog_full_leg_calibrator_v1.ino.bin"
    )
    return build if build.is_file() else None


@dataclass
class GateResult:
    name: str
    passed: bool
    detail: str = ""

    def as_dict(self) -> dict:
        return {"gate": self.name, "result": "PASS" if self.passed else "FAIL",
                "detail": self.detail}


@dataclass
class Session:
    """One self-contained, reproducible calibrator session."""

    mode: str
    port: str
    requested_stage: HardwareStage
    host_authorized_stage: HardwareStage = AUTHORIZED_STAGE
    started_utc: str = field(default_factory=utc_now)
    transcript: list[str] = field(default_factory=list)
    commands: list[dict] = field(default_factory=list)
    gates: list[GateResult] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    #: Populated from the firmware banner, so evidence names the running image.
    firmware_identity: dict[str, str] = field(default_factory=dict)
    servo_ids_involved: list[int] = field(default_factory=list)
    host_session_id: str = ""
    firmware_boot_session_id: str = ""
    firmware_session_generation: str = ""
    connection_open_count: int = 0
    continuity_state: str = "NOT_STARTED"
    refused: bool = False

    def raise_requested_stage(self, stage: HardwareStage) -> None:
        """Record the highest stage requested without claiming it was exercised."""
        if stage.value > self.requested_stage.value:
            self.requested_stage = stage

    def record(self, command: str, lines: list[str]) -> None:
        self.commands.append({"command": command, "response": lines})
        self.transcript.append(f">>> {command}")
        self.transcript.extend(lines)

    def record_host_event(self, event: str, detail: str = "") -> None:
        line = f"HOST_EVENT={event}" + (f" DETAIL={detail}" if detail else "")
        self.transcript.append(line)
        self.commands.append({"command": f"HOST:{event}", "response": [detail] if detail else []})

    def gate(self, name: str, passed: bool, detail: str = "") -> GateResult:
        result = GateResult(name, passed, detail)
        self.gates.append(result)
        return result

    @property
    def passed(self) -> bool:
        return all(g.passed for g in self.gates)

    # -- derived observations, never hard-coded ----------------------------
    @property
    def all_lines(self) -> list[str]:
        return [line for entry in self.commands for line in entry["response"]]

    @property
    def motion_attempted(self) -> bool:
        """True only if the firmware actually emitted a MOTION line."""
        return any(line.startswith("MOTION ") for line in self.all_lines)

    @property
    def servos_responded(self) -> bool:
        if any("CENSUS_PRESENT=0/12" in line for line in self.all_lines):
            return False
        return any(
            line.startswith("CENSUS_SERVO ") and "RESULT=MISSING" not in line
            for line in self.all_lines
        ) or any("SAFE_OFF_RESPONDERS_SEEN=YES" in line for line in self.all_lines)

    def writes_by_class(self) -> dict[str, int]:
        """Count observed servo writes by register class, from the transcript."""
        counts = {"torque_enable": 0, "torque_limit": 0, "goal_position": 0,
                  "eeprom": 0, "broadcast": 0, "refused": 0}
        for line in self.all_lines:
            if line.startswith("WRITE "):
                fields = dict(
                    p.split("=", 1) for p in line.split() if "=" in p
                )
                if fields.get("ID") in ("254", "0xFE", "0xfe"):
                    counts["broadcast"] += 1
                addr = fields.get("ADDR", "").lower()
                if addr == "0x28":
                    counts["torque_enable"] += 1
                elif addr == "0x30":
                    counts["torque_limit"] += 1
                else:
                    # Any other address reaching the wire would be an EEPROM
                    # write, which the firmware allowlist makes unreachable.
                    counts["eeprom"] += 1
            elif line.startswith("MOTION "):
                counts["goal_position"] += 1
            elif line.startswith("WRITE_REFUSED") or "MOTION_REFUSED" in line:
                counts["refused"] += 1
        return counts

    def final_torque_state(self) -> str:
        lines = self.all_lines
        if any("CUT_SERVO_POWER_NOW" in line for line in lines):
            return "UNVERIFIED_HARD_FAULT"
        if any("SAFE_OFF_RESULT PASS" in line or "SAFE_OFF_SWEEP RESULT=PASS" in line
               for line in lines):
            return "OFF_VERIFIED"
        # The shared H4/H5/H6 orchestrator reports one run-level release for
        # every joint it touched; H3 reports its single joint. Both are proof
        # that torque OFF was READ BACK, not merely commanded.
        if any("TORQUE_OFF_VERIFIED=YES" in line or "SAFE_OFF_VERIFIED=YES" in line
               for line in lines):
            return "OFF_VERIFIED"
        if not self.motion_attempted:
            return "NEVER_ENABLED"
        return "UNKNOWN"

    def fault_reason(self) -> str:
        for line in self.all_lines:
            if "_REFUSED REASON=" in line or "_FAIL_REASON=" in line:
                return line.strip()
            if "STATUS=" in line and "REASON=" in line and "REASON=NONE" not in line:
                return line.strip()
        return ""

    def candidate_outputs(self) -> list[str]:
        keys = ("_DERIVED_Q0", "CAPTURE_Q0_JOINT", "_JOINT_SPAN",
                "CHARACTERIZE_JOINT_OUTPUT", "_JOINT_MIN_ENDPOINT",
                "_JOINT_MAX_ENDPOINT", "WITNESS_DIRECTION ID=",
                "_JOINT_DIRECTION", "_JOINT_STATUS=")
        return [ln for ln in self.all_lines if any(k in ln for k in keys)]

    def provenance(self) -> dict:
        binary = firmware_binary_path()
        specs = build_joint_specs()
        return {
            "git_commit": git_commit(),
            "git_working_tree_dirty": git_dirty(),
            "firmware_source_sha256": sha256_file(FIRMWARE_SKETCH),
            "firmware_detector_sha256": sha256_file(
                FIRMWARE_SKETCH.parent / "flc_contact_detector.h"
            ),
            "firmware_engine_sha256": sha256_file(
                FIRMWARE_SKETCH.parent / "flc_calibration_engine.h"
            ),
            "firmware_plan_path": str(FIRMWARE_GEOMETRY_PLAN.relative_to(REPO_ROOT)),
            "firmware_plan_sha256": sha256_file(FIRMWARE_GEOMETRY_PLAN),
            "geometry_plan_generator_path": str(
                GEOMETRY_PLAN_GENERATOR.relative_to(REPO_ROOT)
            ),
            "geometry_plan_generator_sha256": sha256_file(GEOMETRY_PLAN_GENERATOR),
            "geometry_parking_source_path": str(
                GEOMETRY_PARKING_PATH.relative_to(REPO_ROOT)
            ),
            "geometry_parking_source_sha256": sha256_file(GEOMETRY_PARKING_PATH),
            "geometry_safety_policy_path": str(
                GEOMETRY_SAFETY_POLICY_PATH.relative_to(REPO_ROOT)
            ),
            "geometry_safety_policy_sha256": sha256_file(
                GEOMETRY_SAFETY_POLICY_PATH
            ),
            "firmware_stage_config_sha256": sha256_file(
                FIRMWARE_SKETCH.parent / "flc_stage_config.h"
            ),
            "firmware_binary_sha256": sha256_file(binary) if binary else "",
            "firmware_binary_path": str(binary) if binary else "",
            "runner_sha256": sha256_file(Path(__file__).resolve()),
            "policy_sha256": sha256_file(
                CALIBRATION_DIR / "matdog_full_leg_calibrator_policy.py"
            ),
            "fqbn": FQBN,
            "servo_allocation_path": str(SERVO_ALLOCATION_PATH.relative_to(REPO_ROOT)),
            "servo_allocation_sha256": sha256_file(SERVO_ALLOCATION_PATH),
            "geometry_endpoint_profile_path": str(
                GEOMETRY_ENDPOINT_PROFILE_PATH.relative_to(REPO_ROOT)
            ),
            "geometry_endpoint_profile_sha256": sha256_file(
                GEOMETRY_ENDPOINT_PROFILE_PATH
            ),
            "protocol_id": PROTOCOL_ID,
            "protocol_scope": PROTOCOL_SCOPE,
            "host_policy_authorized_stage": self.host_authorized_stage.name,
            "firmware_reported_identity": self.firmware_identity,
            "firmware_build_matches_local_git": (
                self.firmware_identity.get("BUILD_GIT_SHA") == git_commit()
            ),
            "expected_leg_ids": list(EXPECTED_LEG_IDS),
            "joint_specs": [
                {"bus_id": s.bus_id, "joint": s.joint_name, "unit": s.unit_label,
                 "direction": s.direction}
                for s in specs
            ],
            "pre_motion_blockers": [
                {"name": b.name, "class": b.parameter_class.value if b.parameter_class
                 else None, "reason": b.evidence}
                for b in pre_motion_blockers()
            ],
            "acceptance_gates_unresolved": [
                {"name": b.name, "class": b.parameter_class.value if b.parameter_class
                 else None}
                for b in acceptance_gates_unresolved()
            ],
            "bootstrap_envelope": {
                **{k: (v.value if hasattr(v, "value") else v)
                   for k, v in BOOTSTRAP_ENVELOPE.items()},
            },
        }

    def write(self, root: Path = SESSION_ROOT) -> Path:
        stamp = self.started_utc.replace(":", "").replace("-", "")
        safe_mode = self.mode.replace("-", "_")
        suffix = f"_{self.host_session_id}" if self.host_session_id else ""
        directory = root / "sessions" / f"{stamp}_{safe_mode}{suffix}"
        directory.mkdir(parents=True, exist_ok=True)

        transcript_path = directory / "transcript.txt"
        transcript_tmp = directory / "transcript.txt.tmp"
        transcript_tmp.write_text("\n".join(self.transcript) + "\n", encoding="utf-8")
        transcript_tmp.replace(transcript_path)

        writes = self.writes_by_class()
        firmware_stage = self.firmware_identity.get("AUTHORIZED_HARDWARE_STAGE", "")
        report = {
            "session": {
                "mode": self.mode,
                "port": self.port,
                "host_session_id": self.host_session_id,
                "firmware_boot_session_id": self.firmware_boot_session_id,
                "firmware_session_generation": self.firmware_session_generation,
                "connection_open_count": self.connection_open_count,
                "continuity_state": self.continuity_state,
                "started_utc": self.started_utc,
                "finished_utc": utc_now(),
                "result": ("FAIL" if self.continuity_state == "FAULT"
                           else ("REFUSED" if self.refused
                           else ("PASS" if self.passed else "FAIL"))),
            },
            "stage": {
                "requested_stage": self.requested_stage.name,
                "firmware_authorized_stage": firmware_stage,
                "host_policy_authorized_stage": self.host_authorized_stage.name,
                "mode_can_command_motion": MODE_STAGE.get(self.mode, (None, False))[1],
            },
            # Everything below is OBSERVED from the transcript, not assumed.
            "observed": {
                "servos_responded": self.servos_responded,
                "motion_actually_attempted": self.motion_attempted,
                "servo_ids_involved": sorted(set(self.servo_ids_involved)),
                "writes_by_register_class": writes,
                "eeprom_writes": writes["eeprom"],
                "broadcast_writes": writes["broadcast"],
                "final_torque_state": self.final_torque_state(),
                "fault_reason": self.fault_reason(),
                "candidate_outputs": self.candidate_outputs(),
            },
            "provenance": self.provenance(),
            "gates": [g.as_dict() for g in self.gates],
            "commands": self.commands,
            "notes": self.notes,
            "hardware_validation_scope": {
                "highest_stage_exercised_in_this_session": (
                    self.requested_stage.name if self.motion_attempted
                    else ("H1_CENSUS_READONLY" if self.servos_responded
                          else "H0_ESP32_ONLY")
                ),
                "statement": (
                    "This block reports what this session observed. It makes no "
                    "claim about stages not exercised here."
                ),
            },
        }
        report_path = directory / "session_report.json"
        report_tmp = directory / "session_report.json.tmp"
        report_tmp.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        report_tmp.replace(report_path)
        return directory


def _default_serial_factory() -> Callable[..., Any]:
    """Import pyserial only when a real port is about to be opened.

    Importing this module must never pull a hardware transport into the
    process: the offline suites, the policy checks and the geometry tooling all
    import it, and none of them may be one attribute access away from a device.
    A missing pyserial is therefore an error at connect time, not import time.
    """
    try:
        import serial as _serial
    except ImportError as exc:  # pragma: no cover - exercised only without pyserial
        raise CalibratorLinkLost(
            "pyserial is not installed; the calibration link cannot be opened"
        ) from exc
    return _serial.Serial


class CalibratorLink:
    """USB CDC link to the ESP32-S3. Sends named commands, never raw registers."""

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
                 timeout: float = 2.0,
                 serial_factory: Callable[..., object] | None = None) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.serial: Any | None = None
        # Resolved lazily in __enter__ so constructing a link (which the tests
        # and the dry-run paths do) still touches no hardware module.
        self.serial_factory = serial_factory
        self.open_count = 0
        self.banner: list[str] = []

    def __enter__(self) -> "CalibratorLink":
        if self.serial is not None:
            raise CalibratorProtocolError("the calibration link is already open")
        factory = self.serial_factory or _default_serial_factory()
        try:
            self.serial = factory(self.port, self.baud, timeout=self.timeout)
        except Exception as exc:
            raise CalibratorLinkLost(f"could not open {self.port}: {exc}") from exc
        self.open_count += 1
        # The ESP32-S3 resets on port open; wait for the banner rather than sleep.
        try:
            self.banner = self.read_until("FULL_LEG_CALIBRATOR_READY", timeout=8.0)
        except Exception:
            self.serial.close()
            self.serial = None
            raise
        if not any(line == "FULL_LEG_CALIBRATOR_READY" for line in self.banner):
            self.serial.close()
            self.serial = None
            raise CalibratorProtocolError("firmware READY marker was not received")
        # Current firmware prints its semantic help immediately after READY.
        # Consume that known startup tail so the first command can require its
        # BEGIN marker as the first non-empty response line. No RX reset is used:
        # an unexpected later READY remains observable reset evidence.
        startup_tail = self.read_until("  @HELP", timeout=2.0)
        if startup_tail and not any(line == "  @HELP" for line in startup_tail):
            self.serial.close()
            self.serial = None
            raise CalibratorProtocolError("truncated firmware startup help")
        self.banner.extend(startup_tail)
        return self

    def __exit__(self, *exc) -> None:
        if self.serial is not None:
            self.serial.close()
            self.serial = None

    def read_until(self, marker: str, timeout: float = 15.0) -> list[str]:
        assert self.serial is not None
        lines: list[str] = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                raw = self.serial.readline()
            except Exception as exc:
                raise CalibratorLinkLost(f"serial read failed: {exc}") from exc
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="strict").rstrip("\r\n")
            except UnicodeDecodeError as exc:
                raise CalibratorProtocolError("non-UTF-8 data on calibrator link") from exc
            lines.append(line)
            if line == marker:
                break
        return lines

    def send(self, command: str, timeout: float = 30.0,
             *, expected_begin: str, expected_end: str) -> list[str]:
        assert self.serial is not None
        if not semantic_command_allowed(command):
            raise CalibratorProtocolError(
                f"command is outside the fixed semantic calibrator surface: {command!r}"
            )
        if expected_begin not in BEGIN_MARKERS or expected_end not in END_MARKERS:
            raise CalibratorProtocolError("unknown response framing markers")
        try:
            # Do not reset/discard RX here. An unexpected READY banner is reset
            # evidence and must be observed, never silently erased.
            self.serial.write((command + "\n").encode("ascii"))
            self.serial.flush()
        except Exception as exc:
            raise CalibratorLinkLost(f"serial write failed: {exc}") from exc

        lines: list[str] = []
        saw_begin = False
        saw_end = False
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                raw = self.serial.readline()
            except Exception as exc:
                raise CalibratorLinkLost(f"serial read failed: {exc}") from exc
            if not raw:
                continue
            try:
                line = raw.decode("utf-8", errors="strict").rstrip("\r\n")
            except UnicodeDecodeError as exc:
                raise CalibratorProtocolError("non-UTF-8 data on calibrator link") from exc
            if not line:
                continue
            lines.append(line)
            if line == "FULL_LEG_CALIBRATOR_READY":
                raise CalibratorResetDetected(
                    f"firmware reset while awaiting response to {command}"
                )
            if not saw_begin:
                if line != expected_begin:
                    raise CalibratorProtocolError(
                        f"expected {expected_begin} first for {command}, got {line!r}"
                    )
                saw_begin = True
                continue
            if line == expected_begin:
                raise CalibratorProtocolError(
                    f"duplicate {expected_begin} while awaiting {command}"
                )
            if line in BEGIN_MARKERS:
                raise CalibratorProtocolError(
                    f"unexpected frame begin {line} while awaiting {command}"
                )
            if line == expected_end:
                saw_end = True
                break
            if line in END_MARKERS:
                raise CalibratorProtocolError(
                    f"unexpected frame end {line} while awaiting {command}"
                )
        if not saw_end:
            raise CalibratorProtocolError(
                f"timeout/missing {expected_end} while awaiting {command}"
            )
        if not saw_begin:
            raise CalibratorProtocolError(
                f"missing {expected_begin} while awaiting {command}"
            )
        return lines


def parse_fields(lines: list[str]) -> dict[str, str]:
    fields: dict[str, str] = {}
    for line in lines:
        if "=" in line and " " not in line.split("=", 1)[0]:
            key, _, value = line.partition("=")
            fields[key.strip()] = value.strip()
    return fields


IDENTITY_KEYS = (
    "FIRMWARE_NAME", "FIRMWARE_VERSION", "FIRMWARE_BUILD_ID",
    "BUILD_GIT_SHA", "BUILD_WORKTREE_DIRTY", "BUILD_DATE", "BUILD_TIME",
    "PROTOCOL_ID", "PROTOCOL_SCOPE", "AUTHORIZED_HARDWARE_STAGE",
    "FIRMWARE_BUILD_ID", "BOOT_SESSION_ID", "ACTIVE_HOST_SESSION_ID",
    "SESSION_GENERATION", "SESSION_STATE", "CENSUS_FRESH", "CENSUS_EPOCH",
    "LAST_FAULT",
    "H3_BOOTSTRAP_BUILD_APPROVED", "H3_BOOTSTRAP_SESSION_APPROVED",
    "H3_BOOTSTRAP_ACTIVE", "MOTION_UNLOCKED", "PRE_MOTION_OUTSTANDING",
    "JOINTS_CHARACTERIZED", "MANUAL_Q0_CANDIDATES", "DIRECTION_WITNESSES",
    "CALIBRATION_CANDIDATES", "STATION_IN_CONTROL_PATH", "EEPROM_WRITE_SURFACE",
    "BROADCAST_WRITE", "GOAL_POSITION_AUTHORITY", "GOAL_POSITION_DOMAIN",
    "HEAD_SERVOS_EXPECTED_PRESENT",
)


def capture_identity(session: Session, link: CalibratorLink) -> dict[str, str]:
    """Always record which firmware image produced the evidence."""
    lines = link.send(
        "@STATUS", expected_begin="STATUS_BEGIN", expected_end="STATUS_END"
    )
    session.record("@STATUS", lines)
    if not any(line == "STATUS_RESULT PASS" for line in lines):
        raise CalibratorProtocolError("STATUS returned no exact PASS result")
    fields = parse_fields(lines)
    session.firmware_identity = {k: fields[k] for k in IDENTITY_KEYS if k in fields}
    return fields


def check_host_authorization(session: Session, mode: str,
                             host_stage: HardwareStage = AUTHORIZED_STAGE) -> bool:
    """Enforce the host source gate before transmitting an operation command."""
    needed, _ = MODE_STAGE[mode]
    ok = host_stage.value >= needed.value
    session.gate(
        f"host_stage_authorizes_{mode}", ok,
        f"host={host_stage.name} required={needed.name}",
    )
    if not ok:
        session.refused = True
        session.notes.append(
            f"Host policy refused {mode}: {host_stage.name} is below {needed.name}."
        )
    return ok


def check_stage_compatibility(session: Session, status: dict[str, str],
                              mode: str | None = None) -> bool:
    """Refuse when the flashed image cannot perform the requested mode."""
    operation = mode or session.mode
    reported = status.get("AUTHORIZED_HARDWARE_STAGE", "")
    needed_stage = MODE_STAGE.get(operation, (session.requested_stage, False))[0]
    needed = needed_stage.value
    if not reported.startswith("H"):
        session.gate("firmware_stage_reported", False, f"got {reported!r}")
        return False
    try:
        firmware_stage = int(reported[1:])
    except ValueError:
        session.gate("firmware_stage_reported", False, f"got {reported!r}")
        return False

    ok = firmware_stage >= needed
    session.gate(
        "firmware_stage_supports_mode", ok,
        f"firmware=H{firmware_stage} required=H{needed} for {operation}",
    )
    if not ok:
        session.notes.append(
            f"The flashed image is H{firmware_stage}; {operation} needs "
            f"H{needed}. Reflash with tools/build_stage.sh {needed}."
        )
    return ok


class CalibrationSessionController:
    """One physical calibration transaction over exactly one USB connection.

    Host-side state is only a second barrier and an evidence aid. It is never
    loaded from an earlier report and never recreates firmware authorization.
    """

    def __init__(
        self,
        session: Session,
        link: CalibratorLink,
        *,
        host_stage: HardwareStage = AUTHORIZED_STAGE,
        host_session_id: str | None = None,
        evidence_root: Path | None = SESSION_ROOT,
    ) -> None:
        self.session = session
        self.link = link
        self.host_stage = host_stage
        self.session.host_authorized_stage = host_stage
        self.host_session_id = (host_session_id or uuid.uuid4().hex[:8]).lower()
        if self.host_session_id == "00000000" or len(self.host_session_id) != 8 or any(
            c not in "0123456789abcdef" for c in self.host_session_id
        ):
            raise ValueError(
                "host_session_id must be exactly 8 hexadecimal characters and non-zero"
            )
        self.evidence_root = evidence_root
        self.started = False
        self.active = False
        self.faulted = False
        self.operation_failed = False
        self.boot_session_id = ""
        self.session_generation = ""
        self.last_census_epoch: int | None = None
        self.last_census_fresh: bool | None = None
        self.last_session_state: int | None = None
        self.census_valid = False
        self.manual_q0_epoch: int | None = None
        self.bootstrap_epoch: int | None = None
        self.direction_witnesses: dict[int, tuple[str, int]] = {}
        self.characterized: dict[int, int] = {}
        self.calibrated: dict[int, int] = {}
        self.session.host_session_id = self.host_session_id

    def __enter__(self) -> "CalibrationSessionController":
        self.start()
        return self

    def __exit__(self, exc_type, exc, traceback) -> None:
        self.finish(aborted=exc is not None)

    def checkpoint(self) -> Path | None:
        if self.evidence_root is None:
            return None
        return self.session.write(self.evidence_root)

    def _record_protocol_fault(self, exc: Exception) -> None:
        if self.faulted:
            return
        self.faulted = True
        self.active = False
        self.session.continuity_state = "FAULT"
        self.session.refused = True
        self.session.gate("physical_session_continuity", False, str(exc))
        self.session.notes.append(
            "Physical session continuity was lost. No reconnect or evidence "
            "restoration was attempted; all prerequisites must be re-established."
        )
        self.checkpoint()

    @staticmethod
    def _parse_u32(status: dict[str, str], key: str) -> int:
        raw = status.get(key, "")
        try:
            value = int(raw)
        except ValueError as exc:
            raise CalibratorProtocolError(f"invalid {key} {raw!r}") from exc
        if not 0 <= value <= 0xFFFFFFFF:
            raise CalibratorProtocolError(f"{key} out of range: {value}")
        if raw != str(value):
            raise CalibratorProtocolError(f"non-canonical {key} {raw!r}")
        return value

    @classmethod
    def _parse_epoch(cls, status: dict[str, str]) -> int:
        return cls._parse_u32(status, "CENSUS_EPOCH")

    @staticmethod
    def _parse_hex_id(status: dict[str, str], key: str, *, allow_zero: bool) -> str:
        raw = status.get(key, "").strip()
        if not re.fullmatch(r"[0-9A-Fa-f]{8}", raw):
            raise CalibratorProtocolError(f"invalid {key} {raw!r}")
        normalized = raw.lower()
        if not allow_zero and normalized == "00000000":
            raise CalibratorProtocolError(f"{key} must be non-zero")
        return normalized

    def _verify_context(self, status: dict[str, str], *, require_active: bool) -> None:
        boot_id = self._parse_hex_id(
            status, "BOOT_SESSION_ID", allow_zero=False
        )
        if self.boot_session_id and boot_id != self.boot_session_id:
            raise CalibratorResetDetected(
                f"BOOT_SESSION_ID changed {self.boot_session_id} -> {boot_id}"
            )
        if not self.boot_session_id:
            self.boot_session_id = boot_id
            self.session.firmware_boot_session_id = boot_id

        generation_value = self._parse_u32(status, "SESSION_GENERATION")
        generation = str(generation_value)
        if require_active and generation_value == 0:
            raise CalibratorProtocolError("active session has zero SESSION_GENERATION")
        if self.session_generation and generation != self.session_generation:
            raise CalibratorResetDetected(
                f"SESSION_GENERATION changed {self.session_generation} -> {generation}"
            )
        if require_active and not self.session_generation:
            self.session_generation = generation
            self.session.firmware_session_generation = generation

        active = self._parse_hex_id(
            status, "ACTIVE_HOST_SESSION_ID", allow_zero=True
        )
        if require_active:
            if active != self.host_session_id:
                raise CalibratorResetDetected(
                    f"active host session is {active!r}, expected {self.host_session_id!r}"
                )
        elif active != "00000000":
            raise CalibratorProtocolError(
                f"firmware already has active host session {active!r} before SESSION_BEGIN"
            )

        state_raw = status.get("SESSION_STATE", "")
        try:
            state = int(state_raw)
        except ValueError as exc:
            raise CalibratorProtocolError(f"invalid SESSION_STATE {state_raw!r}") from exc
        if state_raw != str(state) or state not in (0, 1, 2):
            raise CalibratorProtocolError(f"invalid SESSION_STATE {state_raw!r}")

        fresh_raw = status.get("CENSUS_FRESH", "")
        if fresh_raw not in ("YES", "NO"):
            raise CalibratorProtocolError(f"invalid CENSUS_FRESH {fresh_raw!r}")
        fresh = fresh_raw == "YES"
        if fresh != (state == 1):
            raise CalibratorProtocolError(
                f"incoherent census state: SESSION_STATE={state} CENSUS_FRESH={fresh_raw}"
            )
        if require_active and state == 2:
            raise CalibratorProtocolError(
                f"firmware session fault latched: {status.get('LAST_FAULT', 'UNKNOWN')}"
            )
        if not require_active and fresh:
            raise CalibratorProtocolError("fresh census exists before host session lease")

        self.last_census_epoch = self._parse_epoch(status)
        self.last_census_fresh = fresh
        self.last_session_state = state
        self.census_valid = fresh

    def _status(self, *, require_active: bool) -> dict[str, str]:
        status = capture_identity(self.session, self.link)
        self._verify_context(status, require_active=require_active)
        return status

    def start(self) -> None:
        if self.started:
            raise CalibratorProtocolError("physical session already started")
        if self.link.open_count != 0:
            raise CalibratorProtocolError(
                "a CalibrationSessionController never reconnects an already-used link"
            )
        try:
            self.link.__enter__()
            self.started = True
            self.session.connection_open_count = self.link.open_count
            self.session.continuity_state = "CONNECTED"
            self.session.record("<FIRMWARE_BOOT>", list(self.link.banner))

            status = self._status(require_active=False)
            identity_ok = (
                status.get("FIRMWARE_NAME") == "matdog_full_leg_calibrator_v1"
                and status.get("PROTOCOL_ID") == PROTOCOL_ID
                and status.get("PROTOCOL_SCOPE") == PROTOCOL_SCOPE
            )
            self.session.gate(
                "firmware_identity",
                identity_ok,
                f"name={status.get('FIRMWARE_NAME')} protocol={status.get('PROTOCOL_ID')}",
            )
            if not identity_ok:
                raise CalibratorProtocolError("unexpected calibrator firmware identity")
            generation_before = self._parse_u32(status, "SESSION_GENERATION")

            command = f"@SESSION_BEGIN {self.host_session_id}"
            lines = self.link.send(
                command, timeout=30.0,
                expected_begin="SESSION_BEGIN_BEGIN",
                expected_end="SESSION_BEGIN_END",
            )
            self.session.record(command, lines)
            if not any(line == "SESSION_BEGIN_RESULT PASS" for line in lines):
                raise CalibratorProtocolError("firmware refused SESSION_BEGIN")
            begin = parse_fields(lines)
            begin_boot = self._parse_hex_id(
                {"BOOT_SESSION_ID": begin.get("SESSION_BEGIN_BOOT_SESSION_ID", "")},
                "BOOT_SESSION_ID",
                allow_zero=False,
            )
            begin_active = self._parse_hex_id(
                {
                    "ACTIVE_HOST_SESSION_ID": begin.get(
                        "SESSION_BEGIN_ACTIVE_HOST_SESSION_ID", ""
                    )
                },
                "ACTIVE_HOST_SESSION_ID",
                allow_zero=False,
            )
            begin_generation = self._parse_u32(
                {
                    "SESSION_GENERATION": begin.get(
                        "SESSION_BEGIN_GENERATION", ""
                    )
                },
                "SESSION_GENERATION",
            )
            expected_generation = (generation_before + 1) & 0xFFFFFFFF
            if (
                begin_boot != self.boot_session_id
                or begin_active != self.host_session_id
                or begin_generation != expected_generation
            ):
                raise CalibratorResetDetected(
                    "SESSION_BEGIN identity/generation did not match the pre-lease STATUS"
                )

            status = self._status(require_active=True)
            if self._parse_u32(status, "SESSION_GENERATION") != expected_generation:
                raise CalibratorResetDetected(
                    "post-lease SESSION_GENERATION did not match SESSION_BEGIN"
                )
            self.census_valid = status.get("CENSUS_FRESH") == "YES"
            self.active = True
            self.session.continuity_state = "ACTIVE"
            self.session.gate("physical_session_started", True, self.host_session_id)
            self.checkpoint()
        except Exception as exc:
            self._record_protocol_fault(exc)
            if self.started:
                self.link.__exit__(type(exc), exc, exc.__traceback__)
                self.started = False
            raise

    def _safe_off(self, label: str) -> bool:
        lines = self.link.send(
            "@SAFE_OFF", timeout=90.0,
            expected_begin="SAFE_OFF_BEGIN", expected_end="SAFE_OFF_END",
        )
        self.session.record(f"@SAFE_OFF ({label})", lines)
        ok = any(line == "SAFE_OFF_RESULT PASS" for line in lines)
        self.session.gate(f"safe_off_{label}", ok)
        if not ok:
            raise CalibratorProtocolError(f"SAFE_OFF failed during {label}")
        return True

    def _host_motion_gate(self, mode: str, status: dict[str, str]) -> bool:
        _, can_move = MODE_STAGE[mode]
        if not can_move:
            return True
        try:
            require_motion_authorized(
                mode,
                stage=self.host_stage,
                bootstrap_approved=(
                    status.get("H3_BOOTSTRAP_SESSION_APPROVED") == "YES"
                ),
            )
        except CalibrationPolicyError as exc:
            self.session.gate(f"host_motion_policy_{mode}", False, str(exc).splitlines()[0])
            self.session.refused = True
            return False
        self.session.gate(f"host_motion_policy_{mode}", True)
        return True

    def _host_evidence_gate(self, mode: str, joint: int | None, leg: str | None) -> bool:
        """Second-barrier checks for volatile prerequisites observed by this host.

        These observations are never loaded from evidence on disk. A fresh
        controller therefore cannot recreate authorization after reconnect.
        Firmware remains authoritative and repeats all of these checks.
        """
        epoch = self.last_census_epoch
        reasons: list[str] = []
        if mode not in ("status", "safe-off", "census") and not self.census_valid:
            reasons.append("no fresh census observed in this open connection")
        if mode in (
            "witness-direction",
            "approve-bootstrap",
            "characterize-joint",
            "calibrate-joint",
            "calibrate-leg",
            "calibrate-all",
        ) and self.manual_q0_epoch != epoch:
            reasons.append("manual q0 was not captured in the current census epoch")
        if mode == "characterize-joint" and joint is not None:
            witness = self.direction_witnesses.get(joint)
            if witness is None or witness[1] != epoch:
                reasons.append("no semantic direction witness for this joint/epoch")
            if self.bootstrap_epoch != epoch:
                reasons.append("bootstrap was not approved in the current census epoch")
        if mode == "calibrate-joint" and joint is not None:
            if self.characterized.get(joint) != epoch:
                reasons.append("joint was not characterized in the current census epoch")
        if mode == "calibrate-leg" and leg is not None:
            prefix = {"LF": 1, "RF": 2, "RH": 3, "LH": 4}[leg]
            needed = [bus_id for bus_id in EXPECTED_LEG_IDS if bus_id // 10 == prefix]
            missing = [bus_id for bus_id in needed if self.characterized.get(bus_id) != epoch]
            if missing:
                reasons.append(f"leg joints not characterized in this epoch: {missing}")
        if mode == "calibrate-all":
            missing = [
                bus_id for bus_id in EXPECTED_LEG_IDS
                if self.characterized.get(bus_id) != epoch
            ]
            if missing:
                reasons.append(f"joints not characterized in this epoch: {missing}")

        ok = not reasons
        self.session.gate(
            f"host_volatile_evidence_{mode}", ok,
            "; ".join(reasons) if reasons else f"census_epoch={epoch}",
        )
        if not ok:
            self.session.refused = True
            self.session.notes.append(
                f"Host volatile-evidence gate refused {mode}: " + "; ".join(reasons)
            )
        return ok

    def _update_host_observations(
        self,
        spec: OperationSpec,
        outcome: OperationOutcome,
        status: dict[str, str],
        q_plus_raw: str | None,
    ) -> None:
        epoch = self._parse_epoch(status)
        if spec.mode == "census":
            self.manual_q0_epoch = None
            self.bootstrap_epoch = None
            self.direction_witnesses.clear()
            self.characterized.clear()
            self.calibrated.clear()
            self.census_valid = outcome.passed and status.get("CENSUS_FRESH") == "YES"
        elif not outcome.passed:
            return
        elif spec.mode == "capture-q0":
            self.manual_q0_epoch = epoch
        elif spec.mode == "approve-bootstrap":
            self.bootstrap_epoch = epoch
        elif spec.mode == "witness-direction" and spec.servo_ids:
            self.direction_witnesses[spec.servo_ids[0]] = (q_plus_raw or "", epoch)
        elif spec.mode == "characterize-joint" and spec.servo_ids:
            self.characterized[spec.servo_ids[0]] = epoch
        elif spec.mode == "calibrate-joint" and spec.servo_ids:
            self.calibrated[spec.servo_ids[0]] = epoch

    def execute(
        self,
        mode: str,
        *,
        joint: int | None = None,
        leg: str | None = None,
        samples: int = 64,
        q_plus_raw: str | None = None,
    ) -> OperationOutcome:
        if not self.started or not self.active or self.faulted or self.operation_failed:
            raise CalibratorProtocolError("no active persistent physical session")
        spec = operation_spec(
            mode, joint=joint, leg=leg, samples=samples, q_plus_raw=q_plus_raw
        )
        self.session.raise_requested_stage(MODE_STAGE[mode][0])

        # This gate is deliberately before STATUS: an unauthorized operation is
        # not transmitted at all, even when a higher-stage image is flashed.
        if not check_host_authorization(self.session, mode, self.host_stage):
            self.checkpoint()
            return OperationOutcome(mode, "REFUSED", reason="HOST_STAGE_LOCKED")

        try:
            before = self._status(require_active=True)
            before_epoch = self._parse_epoch(before)
            before_fresh = before["CENSUS_FRESH"]
            if not check_stage_compatibility(self.session, before, mode):
                self.session.refused = True
                self.checkpoint()
                return OperationOutcome(mode, "REFUSED", reason="FIRMWARE_STAGE_LOCKED")
            if not self._host_evidence_gate(mode, joint, leg):
                self.checkpoint()
                return OperationOutcome(mode, "REFUSED", reason="HOST_EVIDENCE_MISSING")
            if not self._host_motion_gate(mode, before):
                self.checkpoint()
                return OperationOutcome(mode, "REFUSED", reason="HOST_MOTION_GATE")

            lines = self.link.send(
                spec.command, timeout=spec.timeout,
                expected_begin=spec.begin_marker, expected_end=spec.end_marker,
            )
            self.session.record(spec.command, lines)
            self.session.servo_ids_involved.extend(spec.servo_ids)

            passed = any(line == f"{spec.result_marker} PASS" for line in lines)
            refused = any(line == f"{spec.result_marker} REFUSED" for line in lines)
            failed = any(line == f"{spec.result_marker} FAIL" for line in lines)
            if passed:
                outcome = OperationOutcome(mode, "PASS", tuple(lines))
            elif refused:
                outcome = OperationOutcome(mode, "REFUSED", tuple(lines), "FIRMWARE_REFUSED")
                self.session.refused = True
            elif failed:
                outcome = OperationOutcome(mode, "FAIL", tuple(lines), "FIRMWARE_FAILED")
            else:
                raise CalibratorProtocolError(
                    f"{spec.command} returned no terminal result marker"
                )
            self.session.gate(
                f"operation_{len(self.session.commands)}_{mode}",
                outcome.passed or outcome.status == "REFUSED", outcome.status,
            )

            if mode != "status":
                if mode != "safe-off":
                    self._safe_off(f"after_{mode}")
                elif not outcome.passed:
                    raise CalibratorProtocolError("explicit SAFE_OFF did not pass")

            after = self._status(require_active=True)
            after_epoch = self._parse_epoch(after)
            if mode == "census":
                expected_epoch = (before_epoch + 1) & 0xFFFFFFFF
                if after_epoch != expected_epoch:
                    raise CalibratorResetDetected(
                        f"census epoch changed {before_epoch} -> {after_epoch}, "
                        f"expected {expected_epoch}"
                    )
            elif after_epoch != before_epoch:
                raise CalibratorResetDetected(
                    f"census epoch changed during {mode}: {before_epoch} -> {after_epoch}"
                )
            if mode != "census" and after["CENSUS_FRESH"] != before_fresh:
                raise CalibratorResetDetected(
                    f"census freshness changed during {mode}: "
                    f"{before_fresh} -> {after['CENSUS_FRESH']}"
                )
            self._update_host_observations(spec, outcome, after, q_plus_raw)
            if outcome.status == "FAIL":
                self.operation_failed = True
                self.session.continuity_state = "OPERATION_FAILED_SAFE_OFF"
                self.session.notes.append(
                    f"{mode} failed; SAFE_OFF passed and the host ended the lease "
                    "without attempting another operation."
                )
            self.checkpoint()
            return outcome
        except Exception as exc:
            self._record_protocol_fault(exc)
            raise

    def pause(self, label: str = "operator_review") -> None:
        if not self.active or self.faulted:
            raise CalibratorProtocolError("cannot pause an inactive physical session")
        self.session.record_host_event("PAUSE", label)
        self.session.continuity_state = "PAUSED_LINK_HELD_OPEN"
        self.checkpoint()
        self.session.continuity_state = "ACTIVE"

    def state_summary(self) -> dict:
        return {
            "host_session_id": self.host_session_id,
            "boot_session_id": self.boot_session_id,
            "session_generation": self.session_generation,
            "census_epoch": self.last_census_epoch,
            "census_valid": self.census_valid,
            "manual_q0_epoch": self.manual_q0_epoch,
            "bootstrap_epoch": self.bootstrap_epoch,
            "direction_witnesses": self.direction_witnesses,
            "characterized": self.characterized,
            "calibrated": self.calibrated,
            "connection_open_count": self.link.open_count,
            "faulted": self.faulted,
            "operation_failed": self.operation_failed,
        }

    def finish(self, *, aborted: bool = False) -> None:
        if not self.started:
            return
        try:
            if self.link.serial is not None:
                try:
                    self._safe_off("session_final")
                except Exception as exc:
                    self._record_protocol_fault(exc)

                if not self.faulted and self.active:
                    try:
                        lines = self.link.send(
                            "@SESSION_END", timeout=90.0,
                            expected_begin="SESSION_END_BEGIN",
                            expected_end="SESSION_END_END",
                        )
                        self.session.record("@SESSION_END", lines)
                        end = parse_fields(lines)
                        if (
                            not any(line == "SESSION_END_RESULT PASS" for line in lines)
                            or end.get("SESSION_END_SAFE_OFF_VERIFIED") != "YES"
                        ):
                            raise CalibratorProtocolError("firmware refused SESSION_END")
                        self.active = False
                        if self.operation_failed:
                            self.session.continuity_state = "FAILED_CLOSED_SAFE_OFF"
                        else:
                            self.session.continuity_state = (
                                "ABORTED_SAFE_OFF" if aborted else "CLOSED_SAFE_OFF"
                            )
                    except Exception as exc:
                        self._record_protocol_fault(exc)
        finally:
            self.session.connection_open_count = self.link.open_count
            self.link.__exit__(None, None, None)
            self.started = False
            self.checkpoint()


def parse_workflow_line(line: str) -> WorkflowAction | None:
    """Parse one session script/prompt line into the semantic operation surface.

    The grammar intentionally has no firmware-command escape hatch. In
    particular a line beginning with ``@`` is rejected rather than forwarded.
    """
    try:
        tokens = shlex.split(line, comments=True, posix=True)
    except ValueError as exc:
        raise ValueError(f"invalid session line: {exc}") from exc
    if not tokens:
        return None

    action, *args = tokens
    if action in ("end", "quit", "exit"):
        if args:
            raise ValueError(f"{action} takes no arguments")
        return WorkflowAction("end")
    if action == "pause":
        return WorkflowAction("pause", label=" ".join(args) or "operator_review")
    if action in ("status", "census", "approve-bootstrap", "calibrate-all", "safe-off"):
        if args:
            raise ValueError(f"{action} takes no arguments")
        return WorkflowAction(action)
    if action == "capture-q0":
        if len(args) > 1:
            raise ValueError("capture-q0 takes at most one sample count")
        try:
            samples = int(args[0]) if args else 64
        except ValueError as exc:
            raise ValueError("capture-q0 sample count must be an integer") from exc
        if not 1 <= samples <= 256:
            raise ValueError("capture-q0 sample count must be in 1..256")
        return WorkflowAction(action, samples=samples)
    if action in ("characterize-joint", "calibrate-joint"):
        if len(args) != 1:
            raise ValueError(f"{action} requires one leg bus id")
        try:
            joint = int(args[0])
        except ValueError as exc:
            raise ValueError(f"{action} bus id must be an integer") from exc
        operation_spec(action, joint=joint)
        return WorkflowAction(action, joint=joint)
    if action == "witness-direction":
        if len(args) != 2:
            raise ValueError(
                "witness-direction requires <bus-id> <increases|decreases>"
            )
        try:
            joint = int(args[0])
        except ValueError as exc:
            raise ValueError("witness-direction bus id must be an integer") from exc
        operation_spec(action, joint=joint, q_plus_raw=args[1])
        return WorkflowAction(action, joint=joint, q_plus_raw=args[1])
    if action == "calibrate-leg":
        if len(args) != 1:
            raise ValueError("calibrate-leg requires LF|RF|RH|LH")
        operation_spec(action, leg=args[0])
        return WorkflowAction(action, leg=args[0])
    raise ValueError(f"unknown semantic session action {action!r}")


def run_session_workflow(
    controller: CalibrationSessionController,
    lines: Iterable[str],
    *,
    pause_callback: Callable[[str], None] | None = None,
    emit: Callable[[str], None] | None = None,
) -> list[OperationOutcome]:
    """Execute parsed actions while the controller retains its one USB link."""
    outcomes: list[OperationOutcome] = []
    for line_number, line in enumerate(lines, start=1):
        try:
            action = parse_workflow_line(line)
        except ValueError as exc:
            raise CalibratorProtocolError(
                f"session workflow line {line_number}: {exc}"
            ) from exc
        if action is None:
            continue
        if action.action == "end":
            break
        if action.action == "pause":
            controller.pause(action.label)
            if emit is not None:
                emit(f"[PAUSE] {action.label} — USB link remains open")
            if pause_callback is not None:
                pause_callback(action.label)
            continue

        outcome = controller.execute(
            action.action,
            joint=action.joint,
            leg=action.leg,
            samples=action.samples,
            q_plus_raw=action.q_plus_raw,
        )
        outcomes.append(outcome)
        if emit is not None:
            detail = f" ({outcome.reason})" if outcome.reason else ""
            emit(f"[{outcome.status}] {action.action}{detail}")
        if not outcome.passed:
            # A refusal/failure is a review boundary. Do not let a script cascade
            # into later stages on prerequisites it did not establish.
            break
    return outcomes


def interactive_workflow_lines() -> Iterable[str]:
    """Yield semantic actions while leaving the serial owner process alive."""
    while True:
        try:
            line = input("flc-session> ")
        except EOFError:
            yield "end"
            return
        yield line


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------

def run_h0_smoke(session: Session, link: CalibratorLink) -> None:
    """H0: ESP32-S3 only. The expected census failure is the point of the test."""
    status = capture_identity(session, link)

    session.gate(
        "firmware_identity",
        status.get("FIRMWARE_NAME") == "matdog_full_leg_calibrator_v1",
        f"name={status.get('FIRMWARE_NAME')} version={status.get('FIRMWARE_VERSION')} "
        f"built={status.get('BUILD_DATE')} {status.get('BUILD_TIME')}",
    )
    session.gate(
        "protocol_labelled_calibrator_local",
        status.get("PROTOCOL_ID") == PROTOCOL_ID
        and "NOT_FINAL_RUNTIME_PROTOCOL" in status.get("PROTOCOL_SCOPE", ""),
    )
    session.gate("station_absent_from_control_path",
                 status.get("STATION_IN_CONTROL_PATH") == "NO")
    session.gate("no_eeprom_write_surface",
                 status.get("EEPROM_WRITE_SURFACE") == "NONE")
    session.gate("no_broadcast_write", status.get("BROADCAST_WRITE") == "NEVER")
    session.gate("single_goal_position_authority",
                 status.get("GOAL_POSITION_AUTHORITY") == "flcWritePosEx")
    session.gate("unsigned_domain_enforced",
                 status.get("GOAL_POSITION_DOMAIN") == "0..4095")
    session.gate("authorized_stage_is_h0",
                 status.get("AUTHORIZED_HARDWARE_STAGE") == "H0",
                 f"stage={status.get('AUTHORIZED_HARDWARE_STAGE')}")
    session.gate("motion_locked", status.get("MOTION_UNLOCKED") == "NO")
    session.gate("bootstrap_inactive", status.get("H3_BOOTSTRAP_ACTIVE") == "NO")
    session.gate("head_absence_expected",
                 status.get("HEAD_SERVOS_EXPECTED_PRESENT") == "NO")

    census_lines = link.send(
        "@CENSUS", timeout=90.0,
        expected_begin="CENSUS_BEGIN", expected_end="CENSUS_END",
    )
    session.record("@CENSUS", census_lines)
    census = parse_fields(census_lines)

    responders = [ln for ln in census_lines if "CENSUS_UNEXPECTED_RESPONDER" in ln]
    found_any = census.get("CENSUS_PRESENT", "0/12").split("/")[0] != "0"
    if responders or found_any:
        session.notes.append(
            "STOPPED: a servo responded while none should be connected. "
            "No writes were issued; torque was left untouched."
        )
        session.gate("no_unexpected_responder", False, "; ".join(responders))
        return

    session.gate("no_unexpected_responder", True, "bus silent, as expected")
    session.gate(
        "census_correctly_reports_no_responders",
        census.get("CENSUS_PRESENT") == "0/12"
        and census.get("CENSUS_FAIL_REASON") == "NO_RESPONDERS",
        f"present={census.get('CENSUS_PRESENT')} reason={census.get('CENSUS_FAIL_REASON')}",
    )
    session.gate("census_refuses_to_pass",
                 any("CENSUS_RESULT FAIL" in ln for ln in census_lines),
                 "refusing to proceed with no servos is the correct behaviour")

    for command, marker, begin, end in (
        ("@CHARACTERIZE_JOINT 13", "CHARACTERIZE_JOINT_RESULT REFUSED",
         "CHARACTERIZE_JOINT_BEGIN", "CHARACTERIZE_JOINT_END"),
        ("@CALIBRATE_JOINT 13", "CALIBRATE_JOINT_RESULT REFUSED",
         "CALIBRATE_JOINT_BEGIN", "CALIBRATE_JOINT_END"),
        ("@CALIBRATE_LEG LF", "CALIBRATE_LEG_RESULT REFUSED",
         "CALIBRATE_LEG_BEGIN", "CALIBRATE_LEG_END"),
        ("@CALIBRATE_ALL", "CALIBRATE_ALL_RESULT REFUSED",
         "CALIBRATE_ALL_BEGIN", "CALIBRATE_ALL_END"),
    ):
        lines = link.send(command, expected_begin=begin, expected_end=end)
        session.record(command, lines)
        refused = any(marker in ln for ln in lines)
        no_motion = not any(ln.startswith("MOTION ") for ln in lines)
        session.gate(
            "motion_refused" + command.replace("@", "_").replace(" ", "_").lower(),
            refused and no_motion,
            f"refused={refused} no_motion_line={no_motion}",
        )

    q0_lines = link.send(
        "@CAPTURE_Q0 32", expected_begin="CAPTURE_Q0_BEGIN",
        expected_end="CAPTURE_Q0_END",
    )
    session.record("@CAPTURE_Q0 32", q0_lines)
    session.gate("manual_q0_refused_without_census",
                 any("CAPTURE_Q0_RESULT REFUSED" in ln for ln in q0_lines))

    boot_lines = link.send(
        "@APPROVE_BOOTSTRAP CONFIRM", expected_begin="APPROVE_BOOTSTRAP_BEGIN",
        expected_end="APPROVE_BOOTSTRAP_END",
    )
    session.record("@APPROVE_BOOTSTRAP CONFIRM", boot_lines)
    session.gate("bootstrap_refused_on_h0_build",
                 any("APPROVE_BOOTSTRAP_RESULT REFUSED" in ln for ln in boot_lines))

    first = link.send(
        "@SAFE_OFF", timeout=60.0,
        expected_begin="SAFE_OFF_BEGIN", expected_end="SAFE_OFF_END",
    )
    session.record("@SAFE_OFF", first)
    second = link.send(
        "@SAFE_OFF", timeout=60.0,
        expected_begin="SAFE_OFF_BEGIN", expected_end="SAFE_OFF_END",
    )
    session.record("@SAFE_OFF (idempotency)", second)

    session.gate("safe_off_passes_with_no_responders",
                 any("SAFE_OFF_RESULT PASS" in ln for ln in first))
    session.gate("safe_off_is_idempotent",
                 any("SAFE_OFF_RESULT PASS" in ln for ln in second))
    session.gate("safe_off_saw_no_responders",
                 any("SAFE_OFF_RESPONDERS_SEEN=NO" in ln for ln in first))
    session.gate("safe_off_issued_no_writes",
                 not any(ln.startswith("WRITE ") for ln in first + second))

    try:
        require_motion_authorized("CALIBRATE_ALL")
        session.gate("host_policy_blocks_motion", False, "host policy did NOT block")
    except CalibrationPolicyError as exc:
        session.gate("host_policy_blocks_motion", True, str(exc).splitlines()[0])


def run_named(session: Session, link: CalibratorLink, spec: OperationSpec,
              host_stage: HardwareStage = AUTHORIZED_STAGE) -> None:
    """Generic path for a single named firmware operation."""
    session.host_authorized_stage = host_stage
    session.raise_requested_stage(MODE_STAGE[spec.mode][0])
    if not check_host_authorization(session, spec.mode, host_stage):
        return
    if spec.mode in PERSISTENT_ONLY_MODES:
        session.gate(
            f"persistent_session_required_{spec.mode}", False,
            "dependent modes are available only inside `session` on one open link",
        )
        session.refused = True
        session.notes.append(
            f"One-shot {spec.mode} was refused because reopening USB would erase "
            "the volatile calibration prerequisites."
        )
        return
    status = capture_identity(session, link)
    if not check_stage_compatibility(session, status, spec.mode):
        session.refused = True
        return

    _, can_move = MODE_STAGE[spec.mode]
    if can_move:
        try:
            require_motion_authorized(
                spec.mode,
                stage=host_stage,
                bootstrap_approved=(
                    status.get("H3_BOOTSTRAP_SESSION_APPROVED") == "YES"
                ),
            )
        except CalibrationPolicyError as exc:
            session.gate("host_motion_policy", False, str(exc).splitlines()[0])
            session.refused = True
            return

    lines = link.send(
        spec.command, timeout=spec.timeout,
        expected_begin=spec.begin_marker, expected_end=spec.end_marker,
    )
    session.record(spec.command, lines)
    session.servo_ids_involved.extend(spec.servo_ids)

    passed = any(ln == f"{spec.result_marker} PASS" for ln in lines)
    refused = any(ln == f"{spec.result_marker} REFUSED" for ln in lines)
    session.refused = refused and not passed
    session.gate(
        f"{session.mode}_completed", passed or refused,
        "REFUSED by firmware gate" if refused else "completed",
    )
    if passed:
        session.gate(f"{session.mode}_passed", True)
    elif not refused:
        session.gate(f"{session.mode}_passed", False, "no PASS and no REFUSED marker")

    # Whatever happened, leave the bus safe and record that we did.
    safe = link.send(
        "@SAFE_OFF", timeout=90.0,
        expected_begin="SAFE_OFF_BEGIN", expected_end="SAFE_OFF_END",
    )
    session.record("@SAFE_OFF (final)", safe)
    session.gate("final_safe_off",
                 any("SAFE_OFF_RESULT PASS" in ln for ln in safe))


def _session_outcome(session: Session) -> str:
    if session.continuity_state == "FAULT":
        return "FAIL"
    if session.refused:
        return "REFUSED"
    return "PASS" if session.passed else "FAIL"


def main(
    argv: list[str] | None = None,
    *,
    serial_factory: Callable[..., object] | None = None,
) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=sorted(MODE_STAGE))
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--joint", type=int, help="leg bus id, e.g. 13")
    parser.add_argument("--leg", choices=("LF", "RF", "RH", "LH"))
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument(
        "--q-plus-raw", choices=("increases", "decreases"),
        help="semantic MATDOG +q witness for witness-direction",
    )
    parser.add_argument(
        "--script", type=Path,
        help="session workflow file; `pause` keeps the one USB link open",
    )
    parser.add_argument(
        "--session-id",
        help="optional non-zero 8-hex host nonce (normally generated)",
    )
    parser.add_argument("--no-write", action="store_true",
                        help="do not persist a session directory")
    args = parser.parse_args(argv)

    requested_stage, _ = MODE_STAGE[args.mode]
    session = Session(
        mode=args.mode,
        port=args.port,
        requested_stage=requested_stage,
        host_authorized_stage=AUTHORIZED_STAGE,
    )

    if args.mode in ("characterize-joint", "calibrate-joint"):
        if args.joint is None:
            parser.error(f"{args.mode} requires --joint <bus_id>")
        if args.joint not in EXPECTED_LEG_IDS:
            parser.error(f"--joint must be one of {list(EXPECTED_LEG_IDS)}")
        session.servo_ids_involved.append(args.joint)
    if args.mode == "calibrate-leg" and args.leg is None:
        parser.error("calibrate-leg requires --leg <LF|RF|RH|LH>")
    if args.mode == "witness-direction":
        if args.joint is None or args.q_plus_raw is None:
            parser.error(
                "witness-direction requires --joint <bus_id> "
                "--q-plus-raw <increases|decreases>"
            )
        if args.joint not in EXPECTED_LEG_IDS:
            parser.error(f"--joint must be one of {list(EXPECTED_LEG_IDS)}")
    if args.mode != "session" and args.script is not None:
        parser.error("--script is valid only with session mode")
    if args.mode != "session" and args.session_id is not None:
        parser.error("--session-id is valid only with session mode")
    if not 1 <= args.samples <= 256:
        parser.error("--samples must be in 1..256")
    if args.mode in ("calibrate-leg", "calibrate-all"):
        session.servo_ids_involved.extend(EXPECTED_LEG_IDS)

    evidence_root = None if args.no_write else SESSION_ROOT
    protocol_error: Exception | None = None
    try:
        # Higher-stage one-shot modes are refused before opening USB. Reopening
        # would reset the ESP32 and erase exactly the state they require.
        if args.mode != "session" and not check_host_authorization(
            session, args.mode, AUTHORIZED_STAGE
        ):
            pass
        elif args.mode in PERSISTENT_ONLY_MODES:
            session.gate(
                f"persistent_session_required_{args.mode}", False,
                "run this action inside `session` without closing USB",
            )
            session.refused = True
            session.notes.append(
                f"{args.mode} was not sent: use persistent session mode so a "
                "serial reset cannot erase prerequisites."
            )
        elif args.mode == "session":
            if args.script is not None:
                try:
                    workflow_lines: Iterable[str] = args.script.read_text(
                        encoding="utf-8"
                    ).splitlines()
                except OSError as exc:
                    raise CalibratorProtocolError(
                        f"could not read workflow {args.script}: {exc}"
                    ) from exc
            else:
                workflow_lines = interactive_workflow_lines()

            link = CalibratorLink(args.port, serial_factory=serial_factory)
            controller = CalibrationSessionController(
                session,
                link,
                host_stage=AUTHORIZED_STAGE,
                host_session_id=args.session_id,
                evidence_root=evidence_root,
            )
            with controller:
                print(
                    "Persistent calibration session active. The USB link stays "
                    "open until `end`/EOF."
                )
                run_session_workflow(
                    controller,
                    workflow_lines,
                    pause_callback=(
                        lambda label: input(
                            f"Review {label}; press Enter to continue on the same link: "
                        )
                    ),
                    emit=print,
                )
        elif args.mode == "h0-smoke":
            link = CalibratorLink(args.port, serial_factory=serial_factory)
            controller = CalibrationSessionController(
                session,
                link,
                host_stage=AUTHORIZED_STAGE,
                evidence_root=evidence_root,
            )
            with controller:
                run_h0_smoke(session, link)
        elif args.mode == "census":
            link = CalibratorLink(args.port, serial_factory=serial_factory)
            controller = CalibrationSessionController(
                session,
                link,
                host_stage=AUTHORIZED_STAGE,
                evidence_root=evidence_root,
            )
            with controller:
                controller.execute("census")
        else:
            link = CalibratorLink(args.port, serial_factory=serial_factory)
            with link:
                if args.mode == "status":
                    capture_identity(session, link)
                    session.gate("status_reported", bool(session.firmware_identity))
                else:
                    run_named(
                        session,
                        link,
                        operation_spec(
                            args.mode,
                            joint=args.joint,
                            leg=args.leg,
                            samples=args.samples,
                            q_plus_raw=args.q_plus_raw,
                        ),
                        AUTHORIZED_STAGE,
                    )
    except (CalibratorProtocolError, ValueError) as exc:
        protocol_error = exc
        if session.continuity_state != "FAULT":
            session.gate("runner_protocol", False, str(exc))
        print(f"ERROR: {exc}", file=sys.stderr)

    for gate in session.gates:
        print(f"[{'PASS' if gate.passed else 'FAIL'}] {gate.name}"
              + (f" — {gate.detail}" if gate.detail else ""))
    for note in session.notes:
        print(f"NOTE: {note}")

    if not args.no_write:
        directory = session.write()
        print(f"\nsession evidence: {directory}")

    outcome = _session_outcome(session)
    print(f"\nSESSION RESULT: {outcome}")
    return 1 if protocol_error is not None or outcome == "FAIL" else 0


if __name__ == "__main__":
    raise SystemExit(main())
