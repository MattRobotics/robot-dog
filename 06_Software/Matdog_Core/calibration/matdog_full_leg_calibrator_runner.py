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
"""

from __future__ import annotations

import argparse
import datetime as dt
import hashlib
import json
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

import serial

CALIBRATION_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(CALIBRATION_DIR))

from matdog_full_leg_calibrator_policy import (  # noqa: E402
    AUTHORIZED_STAGE,
    BOOTSTRAP_ENVELOPE,
    EXPECTED_LEG_IDS,
    FIRMWARE_SKETCH,
    GEOMETRY_ENDPOINT_PROFILE_PATH,
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
)

#: Which hardware stage each runner mode needs, and whether it can move a servo.
MODE_STAGE: dict[str, tuple[HardwareStage, bool]] = {
    "status": (HardwareStage.H0_ESP32_ONLY, False),
    "h0-smoke": (HardwareStage.H0_ESP32_ONLY, False),
    "safe-off": (HardwareStage.H0_ESP32_ONLY, False),
    "census": (HardwareStage.H1_CENSUS_READONLY, False),
    "capture-q0": (HardwareStage.H2_MANUAL_Q0, False),
    "approve-bootstrap": (HardwareStage.H3_JOINT_CHARACTERIZE, False),
    "characterize-joint": (HardwareStage.H3_JOINT_CHARACTERIZE, True),
    "calibrate-joint": (HardwareStage.H4_JOINT_CALIBRATE, True),
    "calibrate-leg": (HardwareStage.H5_LEG, True),
    "calibrate-all": (HardwareStage.H6_FOUR_LEGS, True),
}


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else ""


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


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
    started_utc: str = field(default_factory=utc_now)
    transcript: list[str] = field(default_factory=list)
    commands: list[dict] = field(default_factory=list)
    gates: list[GateResult] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)
    #: Populated from the firmware banner, so evidence names the running image.
    firmware_identity: dict[str, str] = field(default_factory=dict)
    servo_ids_involved: list[int] = field(default_factory=list)
    refused: bool = False

    def record(self, command: str, lines: list[str]) -> None:
        self.commands.append({"command": command, "response": lines})
        self.transcript.append(f">>> {command}")
        self.transcript.extend(lines)

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
        if any("TORQUE_OFF_VERIFIED=YES" in line for line in lines):
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
                "_JOINT_MAX_ENDPOINT")
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
            "host_policy_authorized_stage": AUTHORIZED_STAGE.name,
            "firmware_reported_identity": self.firmware_identity,
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
        directory = root / "sessions" / f"{stamp}_{safe_mode}"
        directory.mkdir(parents=True, exist_ok=True)

        (directory / "transcript.txt").write_text(
            "\n".join(self.transcript) + "\n", encoding="utf-8"
        )

        writes = self.writes_by_class()
        firmware_stage = self.firmware_identity.get("AUTHORIZED_HARDWARE_STAGE", "")
        report = {
            "session": {
                "mode": self.mode,
                "port": self.port,
                "started_utc": self.started_utc,
                "finished_utc": utc_now(),
                "result": ("REFUSED" if self.refused
                           else ("PASS" if self.passed else "FAIL")),
            },
            "stage": {
                "requested_stage": self.requested_stage.name,
                "firmware_authorized_stage": firmware_stage,
                "host_policy_authorized_stage": AUTHORIZED_STAGE.name,
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
        (directory / "session_report.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
        return directory


class CalibratorLink:
    """USB CDC link to the ESP32-S3. Sends named commands, never raw registers."""

    def __init__(self, port: str = DEFAULT_PORT, baud: int = DEFAULT_BAUD,
                 timeout: float = 2.0) -> None:
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.serial: serial.Serial | None = None

    def __enter__(self) -> "CalibratorLink":
        self.serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
        # The ESP32-S3 resets on port open; wait for the banner rather than sleep.
        self.read_until("FULL_LEG_CALIBRATOR_READY", timeout=8.0)
        return self

    def __exit__(self, *exc) -> None:
        if self.serial is not None:
            self.serial.close()

    def read_until(self, marker: str, timeout: float = 15.0) -> list[str]:
        assert self.serial is not None
        lines: list[str] = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw = self.serial.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            lines.append(line)
            if marker in line:
                break
        return lines

    def send(self, command: str, timeout: float = 30.0) -> list[str]:
        assert self.serial is not None
        self.serial.reset_input_buffer()
        self.serial.write((command + "\n").encode("ascii"))
        self.serial.flush()

        lines: list[str] = []
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            raw = self.serial.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            lines.append(line)
            if any(marker in line for marker in END_MARKERS):
                break
        return lines


def parse_fields(lines: list[str]) -> dict[str, str]:
    fields: dict[str, str] = {}
    for line in lines:
        if "=" in line and " " not in line.split("=", 1)[0]:
            key, _, value = line.partition("=")
            fields[key.strip()] = value.strip()
    return fields


IDENTITY_KEYS = (
    "FIRMWARE_NAME", "FIRMWARE_VERSION", "BUILD_DATE", "BUILD_TIME",
    "PROTOCOL_ID", "PROTOCOL_SCOPE", "AUTHORIZED_HARDWARE_STAGE",
    "H3_BOOTSTRAP_BUILD_APPROVED", "H3_BOOTSTRAP_SESSION_APPROVED",
    "H3_BOOTSTRAP_ACTIVE", "MOTION_UNLOCKED", "PRE_MOTION_OUTSTANDING",
    "JOINTS_CHARACTERIZED", "STATION_IN_CONTROL_PATH", "EEPROM_WRITE_SURFACE",
)


def capture_identity(session: Session, link: CalibratorLink) -> dict[str, str]:
    """Always record which firmware image produced the evidence."""
    lines = link.send("@STATUS")
    session.record("@STATUS", lines)
    fields = parse_fields(lines)
    session.firmware_identity = {k: fields[k] for k in IDENTITY_KEYS if k in fields}
    return fields


def check_stage_compatibility(session: Session, status: dict[str, str]) -> bool:
    """Refuse when the flashed image cannot perform the requested mode."""
    reported = status.get("AUTHORIZED_HARDWARE_STAGE", "")
    needed = session.requested_stage.value
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
        f"firmware=H{firmware_stage} required=H{needed} for {session.mode}",
    )
    if not ok:
        session.notes.append(
            f"The flashed image is H{firmware_stage}; {session.mode} needs "
            f"H{needed}. Reflash with tools/build_stage.sh {needed}."
        )
    return ok


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

    census_lines = link.send("@CENSUS", timeout=90.0)
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

    for command, marker in (
        ("@CHARACTERIZE_JOINT 13", "CHARACTERIZE_JOINT_RESULT REFUSED"),
        ("@CALIBRATE_JOINT 13", "CALIBRATE_JOINT_RESULT REFUSED"),
        ("@CALIBRATE_LEG LF", "CALIBRATE_LEG_RESULT REFUSED"),
        ("@CALIBRATE_ALL", "CALIBRATE_ALL_RESULT REFUSED"),
    ):
        lines = link.send(command)
        session.record(command, lines)
        refused = any(marker in ln for ln in lines)
        no_motion = not any(ln.startswith("MOTION ") for ln in lines)
        session.gate(
            "motion_refused" + command.replace("@", "_").replace(" ", "_").lower(),
            refused and no_motion,
            f"refused={refused} no_motion_line={no_motion}",
        )

    q0_lines = link.send("@CAPTURE_Q0 32")
    session.record("@CAPTURE_Q0 32", q0_lines)
    session.gate("manual_q0_refused_without_census",
                 any("CAPTURE_Q0_RESULT REFUSED" in ln for ln in q0_lines))

    boot_lines = link.send("@APPROVE_BOOTSTRAP CONFIRM")
    session.record("@APPROVE_BOOTSTRAP CONFIRM", boot_lines)
    session.gate("bootstrap_refused_on_h0_build",
                 any("APPROVE_BOOTSTRAP_RESULT REFUSED" in ln for ln in boot_lines))

    first = link.send("@SAFE_OFF", timeout=60.0)
    session.record("@SAFE_OFF", first)
    second = link.send("@SAFE_OFF", timeout=60.0)
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


def run_named(session: Session, link: CalibratorLink, command: str,
              expect_marker: str, timeout: float = 240.0) -> None:
    """Generic path for a single named firmware operation."""
    status = capture_identity(session, link)
    if not check_stage_compatibility(session, status):
        session.refused = True
        return

    lines = link.send(command, timeout=timeout)
    session.record(command, lines)

    passed = any(f"{expect_marker} PASS" in ln for ln in lines)
    refused = any(f"{expect_marker} REFUSED" in ln for ln in lines)
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
    safe = link.send("@SAFE_OFF", timeout=90.0)
    session.record("@SAFE_OFF (final)", safe)
    session.gate("final_safe_off",
                 any("SAFE_OFF_RESULT PASS" in ln for ln in safe))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("mode", choices=sorted(MODE_STAGE))
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--joint", type=int, help="leg bus id, e.g. 13")
    parser.add_argument("--leg", choices=("LF", "RF", "RH", "LH"))
    parser.add_argument("--samples", type=int, default=64)
    parser.add_argument("--no-write", action="store_true",
                        help="do not persist a session directory")
    args = parser.parse_args()

    requested_stage, _ = MODE_STAGE[args.mode]
    session = Session(mode=args.mode, port=args.port, requested_stage=requested_stage)

    if args.mode in ("characterize-joint", "calibrate-joint"):
        if args.joint is None:
            parser.error(f"{args.mode} requires --joint <bus_id>")
        if args.joint not in EXPECTED_LEG_IDS:
            parser.error(f"--joint must be one of {list(EXPECTED_LEG_IDS)}")
        session.servo_ids_involved.append(args.joint)
    if args.mode == "calibrate-leg" and args.leg is None:
        parser.error("calibrate-leg requires --leg <LF|RF|RH|LH>")
    if args.mode in ("calibrate-leg", "calibrate-all"):
        session.servo_ids_involved.extend(EXPECTED_LEG_IDS)

    with CalibratorLink(args.port) as link:
        if args.mode == "h0-smoke":
            run_h0_smoke(session, link)
        elif args.mode == "status":
            capture_identity(session, link)
            session.gate("status_reported", bool(session.firmware_identity))
        elif args.mode == "census":
            run_named(session, link, "@CENSUS", "CENSUS_RESULT", timeout=120.0)
        elif args.mode == "capture-q0":
            run_named(session, link, f"@CAPTURE_Q0 {args.samples}",
                      "CAPTURE_Q0_RESULT", timeout=180.0)
        elif args.mode == "approve-bootstrap":
            run_named(session, link, "@APPROVE_BOOTSTRAP CONFIRM",
                      "APPROVE_BOOTSTRAP_RESULT", timeout=30.0)
        elif args.mode == "characterize-joint":
            run_named(session, link, f"@CHARACTERIZE_JOINT {args.joint}",
                      "CHARACTERIZE_JOINT_RESULT", timeout=300.0)
        elif args.mode == "calibrate-joint":
            run_named(session, link, f"@CALIBRATE_JOINT {args.joint}",
                      "CALIBRATE_JOINT_RESULT", timeout=600.0)
        elif args.mode == "calibrate-leg":
            run_named(session, link, f"@CALIBRATE_LEG {args.leg}",
                      "CALIBRATE_LEG_RESULT", timeout=1200.0)
        elif args.mode == "calibrate-all":
            run_named(session, link, "@CALIBRATE_ALL", "CALIBRATE_ALL_RESULT",
                      timeout=3600.0)
        else:
            run_named(session, link, "@SAFE_OFF", "SAFE_OFF_RESULT", timeout=90.0)

    for gate in session.gates:
        print(f"[{'PASS' if gate.passed else 'FAIL'}] {gate.name}"
              + (f" — {gate.detail}" if gate.detail else ""))
    for note in session.notes:
        print(f"NOTE: {note}")

    if not args.no_write:
        directory = session.write()
        print(f"\nsession evidence: {directory}")

    outcome = "REFUSED" if session.refused else ("PASS" if session.passed else "FAIL")
    print(f"\nSESSION RESULT: {outcome}")
    return 0 if session.passed or session.refused else 1


if __name__ == "__main__":
    raise SystemExit(main())
