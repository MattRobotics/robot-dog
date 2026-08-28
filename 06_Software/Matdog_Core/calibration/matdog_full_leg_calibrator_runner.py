#!/usr/bin/env python3
"""MATDOG Full Leg Calibrator V1 — host runner and session evidence manager.

Talks to ``matdog_full_leg_calibrator_v1.ino`` over USB CDC. NormaCore Station is
not involved: this process opens the serial port directly and the ESP32-S3 owns
every servo-bus write.

The host is deliberately weak by design. It cannot stream GoalPosition, cannot
compose a servo write and cannot lift a gate; it asks the firmware for a named
operation and records what came back. The gates below are a *second* barrier
in front of the firmware's own, not the primary one.

Every run produces a self-contained session directory containing the full
transcript, the firmware source and binary hashes, the toolchain identity, the
allocation and geometry provenance, and a PASS/FAIL per gate.

Usage
-----
    matdog_full_leg_calibrator_runner.py h0-smoke   # ESP32 only, no servos
    matdog_full_leg_calibrator_runner.py status
    matdog_full_leg_calibrator_runner.py census
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
    EXPECTED_LEG_IDS,
    FIRMWARE_SKETCH,
    GEOMETRY_ENDPOINT_PROFILE_PATH,
    PROTOCOL_ID,
    PROTOCOL_SCOPE,
    SERVO_ALLOCATION_PATH,
    CalibrationPolicyError,
    build_joint_specs,
    motion_blockers,
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
#: relies on a fixed sleep.
END_MARKERS = (
    "STATUS_END", "CENSUS_END", "CAPTURE_Q0_END", "SAFE_OFF_END",
    "CALIBRATE_JOINT_END", "CALIBRATE_LEG_END", "CALIBRATE_ALL_END",
)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else ""


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def git_commit(repo: Path = REPO_ROOT) -> str:
    result = subprocess.run(
        ["git", "rev-parse", "HEAD"], cwd=repo, capture_output=True, text=True
    )
    return result.stdout.strip() if result.returncode == 0 else "UNKNOWN"


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
    started_utc: str = field(default_factory=utc_now)
    transcript: list[str] = field(default_factory=list)
    commands: list[dict] = field(default_factory=list)
    gates: list[GateResult] = field(default_factory=list)
    notes: list[str] = field(default_factory=list)

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

    def provenance(self) -> dict:
        binary = firmware_binary_path()
        specs = build_joint_specs()
        return {
            "git_commit": git_commit(),
            "firmware_source_sha256": sha256_file(FIRMWARE_SKETCH),
            "firmware_detector_sha256": sha256_file(
                FIRMWARE_SKETCH.parent / "flc_contact_detector.h"
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
            "authorized_hardware_stage": AUTHORIZED_STAGE.name,
            "expected_leg_ids": list(EXPECTED_LEG_IDS),
            "joint_specs": [
                {
                    "bus_id": s.bus_id, "joint": s.joint_name, "unit": s.unit_label,
                    "direction": s.direction,
                }
                for s in specs
            ],
            "motion_blockers": [
                {"name": b.name, "reason": b.evidence} for b in motion_blockers()
            ],
        }

    def write(self, root: Path = SESSION_ROOT) -> Path:
        stamp = self.started_utc.replace(":", "").replace("-", "")
        directory = root / "sessions" / f"{stamp}_{self.mode}"
        directory.mkdir(parents=True, exist_ok=True)

        (directory / "transcript.txt").write_text(
            "\n".join(self.transcript) + "\n", encoding="utf-8"
        )
        report = {
            "session": {
                "mode": self.mode,
                "port": self.port,
                "started_utc": self.started_utc,
                "finished_utc": utc_now(),
                "result": "PASS" if self.passed else "FAIL",
            },
            "provenance": self.provenance(),
            "gates": [g.as_dict() for g in self.gates],
            "commands": self.commands,
            "notes": self.notes,
            "hardware_validation_scope": {
                "stage_exercised": "H0_ESP32_ONLY",
                "servos_connected": False,
                "servo_rail_energized": False,
                "motion_commanded": False,
                "eeprom_written": False,
                "h1_plus_validated": False,
                "statement": (
                    "No servo was connected and no servo power was present. "
                    "Nothing about H1 or later is validated by this session."
                ),
            },
        }
        (directory / "session_report.json").write_text(
            json.dumps(report, indent=2) + "\n", encoding="utf-8"
        )
        return directory


def firmware_binary_path() -> Path | None:
    """Path arduino-cli --export-binaries writes the application image to."""
    build = (
        FIRMWARE_SKETCH.parent
        / "build"
        / "esp32.esp32.esp32s3"
        / "matdog_full_leg_calibrator_v1.ino.bin"
    )
    return build if build.is_file() else None


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


def run_h0_smoke(port: str) -> Session:
    """H0: ESP32-S3 only. No servos, no servo power.

    The expected census failure is the point of the test: a system that refuses
    to proceed with no responders is behaving correctly. An unexpected responder
    is a hard stop — this session issues no writes in that case.
    """
    session = Session(mode="h0_smoke", port=port)

    with CalibratorLink(port) as link:
        # --- STATUS -------------------------------------------------------
        status_lines = link.send("@STATUS")
        session.record("@STATUS", status_lines)
        status = parse_fields(status_lines)

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
            f"{status.get('PROTOCOL_ID')} / {status.get('PROTOCOL_SCOPE')}",
        )
        session.gate(
            "station_absent_from_control_path",
            status.get("STATION_IN_CONTROL_PATH") == "NO",
            "firmware reports Station is not in the control path",
        )
        session.gate(
            "no_eeprom_write_surface",
            status.get("EEPROM_WRITE_SURFACE") == "NONE",
            "firmware reports no EEPROM write surface",
        )
        session.gate(
            "no_broadcast_write",
            status.get("BROADCAST_WRITE") == "NEVER",
        )
        session.gate(
            "single_goal_position_authority",
            status.get("GOAL_POSITION_AUTHORITY") == "flcWritePosEx",
        )
        session.gate(
            "unsigned_domain_enforced",
            status.get("GOAL_POSITION_DOMAIN") == "0..4095",
        )
        session.gate(
            "authorized_stage_is_h0",
            status.get("AUTHORIZED_HARDWARE_STAGE") == "H0",
            f"stage={status.get('AUTHORIZED_HARDWARE_STAGE')}",
        )
        session.gate(
            "motion_locked",
            status.get("MOTION_UNLOCKED") == "NO",
            f"outstanding characterization parameters="
            f"{status.get('CHARACTERIZATION_OUTSTANDING')}",
        )
        session.gate(
            "head_absence_expected",
            status.get("HEAD_SERVOS_EXPECTED_PRESENT") == "NO",
        )

        # --- CENSUS with no servos attached -------------------------------
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
            return session

        session.gate("no_unexpected_responder", True, "bus silent, as expected")
        session.gate(
            "census_correctly_reports_no_responders",
            census.get("CENSUS_PRESENT") == "0/12"
            and census.get("CENSUS_FAIL_REASON") == "NO_RESPONDERS",
            f"present={census.get('CENSUS_PRESENT')} reason={census.get('CENSUS_FAIL_REASON')}",
        )
        session.gate(
            "census_refuses_to_pass",
            any("CENSUS_RESULT FAIL" in ln for ln in census_lines),
            "expected FAIL with no servos; refusing to proceed is the correct behaviour",
        )

        # --- Motion modes must all refuse ---------------------------------
        for command, marker in (
            ("@CALIBRATE_JOINT 13", "CALIBRATE_JOINT_RESULT REFUSED"),
            ("@CALIBRATE_LEG LF", "CALIBRATE_LEG_RESULT REFUSED"),
            ("@CALIBRATE_ALL", "CALIBRATE_ALL_RESULT REFUSED"),
        ):
            lines = link.send(command)
            session.record(command, lines)
            refused = any(marker in ln for ln in lines)
            no_motion = not any(ln.startswith("MOTION ") for ln in lines)
            session.gate(
                f"motion_refused{command.replace('@', '_').replace(' ', '_').lower()}",
                refused and no_motion,
                f"refused={refused} no_motion_line={no_motion}",
            )

        # --- Manual q0 must refuse without a census -----------------------
        q0_lines = link.send("@CAPTURE_Q0 32")
        session.record("@CAPTURE_Q0 32", q0_lines)
        session.gate(
            "manual_q0_refused_without_census",
            any("CAPTURE_Q0_RESULT REFUSED" in ln for ln in q0_lines),
        )

        # --- SAFE_OFF must be safe and idempotent with no responders ------
        first = link.send("@SAFE_OFF", timeout=60.0)
        session.record("@SAFE_OFF", first)
        second = link.send("@SAFE_OFF", timeout=60.0)
        session.record("@SAFE_OFF (idempotency)", second)

        session.gate(
            "safe_off_passes_with_no_responders",
            any("SAFE_OFF_RESULT PASS" in ln for ln in first),
        )
        session.gate(
            "safe_off_is_idempotent",
            any("SAFE_OFF_RESULT PASS" in ln for ln in second),
        )
        session.gate(
            "safe_off_saw_no_responders",
            any("SAFE_OFF_RESPONDERS_SEEN=NO" in ln for ln in first),
        )
        session.gate(
            "safe_off_issued_no_writes",
            not any(ln.startswith("WRITE ") for ln in first + second),
            "no WRITE line emitted, because no servo answered",
        )

    # --- Host-side gate, independent of the firmware ----------------------
    try:
        require_motion_authorized("CALIBRATE_ALL")
        session.gate("host_policy_blocks_motion", False, "host policy did NOT block")
    except CalibrationPolicyError as exc:
        session.gate("host_policy_blocks_motion", True, str(exc).splitlines()[0])

    return session


def run_simple(port: str, command: str, mode: str) -> Session:
    session = Session(mode=mode, port=port)
    with CalibratorLink(port) as link:
        lines = link.send(command, timeout=90.0)
        session.record(command, lines)
        session.gate("command_completed", bool(lines), f"{len(lines)} lines")
    return session


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "mode", choices=("h0-smoke", "status", "census", "safe-off"),
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--no-write", action="store_true",
                        help="do not persist a session directory")
    args = parser.parse_args()

    if args.mode == "h0-smoke":
        session = run_h0_smoke(args.port)
    elif args.mode == "status":
        session = run_simple(args.port, "@STATUS", "status")
    elif args.mode == "census":
        session = run_simple(args.port, "@CENSUS", "census")
    else:
        session = run_simple(args.port, "@SAFE_OFF", "safe_off")

    for gate in session.gates:
        print(f"[{'PASS' if gate.passed else 'FAIL'}] {gate.name}"
              + (f" — {gate.detail}" if gate.detail else ""))
    for note in session.notes:
        print(f"NOTE: {note}")

    if not args.no_write:
        directory = session.write()
        print(f"\nsession evidence: {directory}")

    print(f"\nSESSION RESULT: {'PASS' if session.passed else 'FAIL'}")
    return 0 if session.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
