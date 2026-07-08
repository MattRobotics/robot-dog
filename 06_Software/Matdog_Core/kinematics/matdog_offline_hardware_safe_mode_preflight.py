#!/usr/bin/env python3
"""
MATDOG — C4-F offline hardware safe-mode preflight source audit.

This tool does not command the robot.

It scans the repository before the first physical stand preparation and records:
- offline tools;
- read-only live tools;
- existing command-capable calibration/probe tools;
- direct-serial risks;
- files that must be blacklisted for first-stand execution.

Offline only:
- no Station connection;
- no serial port;
- no motor command;
- no torque, target, speed, accel, stand or gait command.
"""

from __future__ import annotations

from datetime import datetime, timezone
import json
from pathlib import Path
import re
from typing import Any


REPO_ROOT = Path(__file__).resolve().parents[3]
CORE_ROOT = REPO_ROOT / "06_Software/Matdog_Core"
REPORTS_RELATIVE_DIR = Path(
    "09_Logs/Validation_Reports/C4_hardware_safe_mode_preflight"
)

COMMAND_PATTERNS = [
    r"\bawait\s+set_torque\b",
    r"\bset_torque\s*\(",
    r"\bawait\s+send_goal\b",
    r"\bsend_goal\s*\(",
    r"\bawait\s+send_motor_commands\b",
    r"\bsend_motor_commands\s*\(",
    r"\bsend_commands\s*\(",
    r"ST3215SyncWriteCommand",
    r"RAM_GOAL_POSITION",
    r"RAM_GOAL_SPEED",
    r"RAM_ACC",
]

DIRECT_SERIAL_PATTERNS = [
    r"\bimport\s+serial\b",
    r"\bfrom\s+serial\s+import\b",
    r"serial\.Serial\s*\(",
    r"/dev/tty",
]

STATION_PATTERNS = [
    r"Station",
    r"new_station_client",
    r"software\.station",
    r"st3215/inference",
]

READ_ONLY_PATTERNS = [
    r"read-only",
    r"read_only",
    r"Nessun torque",
    r"Non invia comandi",
    r"legge soltanto",
    r"telemetria Station",
]

OFFLINE_PATTERNS = [
    r"Offline only",
    r"offline-only",
    r"no Station",
    r"no serial",
    r"no motor command",
]

CONFIRMATION_PATTERNS = [
    r"--execute",
    r"--confirm",
]

KNOWN_FIRST_STAND_FORBIDDEN_KEYWORDS = [
    "micro_probe",
    "visual_zero_pose_probe",
    "leg_hold_probe",
]


class HardwareSafeModePreflightError(RuntimeError):
    """C4-F source audit failure."""


def _latest_c4e_report() -> Path:
    reports = sorted(
        (
            REPO_ROOT
            / "09_Logs/Validation_Reports/C4_static_stability_support_polygon"
        ).glob("*_C4E_static_stability_support_polygon.json")
    )

    if not reports:
        raise HardwareSafeModePreflightError("Nessun report C4-E trovato")

    return reports[-1]


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _default_report_path() -> Path:
    stamp = datetime.now(timezone.utc).strftime("%Y-%m-%d_%H%M%S")
    return (
        REPO_ROOT
        / REPORTS_RELATIVE_DIR
        / f"{stamp}_C4F_hardware_safe_mode_source_audit.json"
    )


def _match_any(patterns: list[str], text: str) -> list[str]:
    matches = []

    for pattern in patterns:
        if re.search(pattern, text, flags=re.IGNORECASE):
            matches.append(pattern)

    return matches


def _python_files() -> list[Path]:
    files = []

    for path in CORE_ROOT.rglob("*.py"):
        if "__pycache__" in path.parts:
            continue
        if "/tests/" in str(path):
            continue
        files.append(path)

    return sorted(files)


def _classify_file(path: Path) -> dict[str, Any]:
    text = path.read_text(encoding="utf-8", errors="replace")
    rel = path.relative_to(REPO_ROOT)

    command_matches = _match_any(COMMAND_PATTERNS, text)
    serial_matches = _match_any(DIRECT_SERIAL_PATTERNS, text)
    station_matches = _match_any(STATION_PATTERNS, text)
    readonly_matches = _match_any(READ_ONLY_PATTERNS, text)
    offline_matches = _match_any(OFFLINE_PATTERNS, text)
    confirmation_matches = _match_any(CONFIRMATION_PATTERNS, text)

    command_capable = bool(command_matches)
    direct_serial_risk = bool(serial_matches)
    station_related = bool(station_matches)

    path_text = str(rel)
    first_stand_forbidden = command_capable and any(
        keyword in path_text
        for keyword in KNOWN_FIRST_STAND_FORBIDDEN_KEYWORDS
    )

    if command_capable:
        classification = "COMMAND_CAPABLE_EXISTING_TOOL"
    elif readonly_matches and station_related:
        classification = "LIVE_READ_ONLY_TOOL"
    elif offline_matches and not station_related:
        classification = "OFFLINE_TOOL"
    else:
        classification = "UTILITY_OR_LIBRARY"

    return {
        "path": str(rel),
        "classification": classification,
        "command_capable": command_capable,
        "direct_serial_risk": direct_serial_risk,
        "station_related": station_related,
        "claims_read_only": bool(readonly_matches),
        "claims_offline_only": bool(offline_matches),
        "has_execute_confirm_gate": bool(confirmation_matches),
        "first_stand_forbidden": first_stand_forbidden,
        "matches": {
            "command": command_matches,
            "direct_serial": serial_matches,
            "station": station_matches,
            "read_only": readonly_matches,
            "offline": offline_matches,
            "confirmation": confirmation_matches,
        },
    }


def _build_report() -> dict[str, Any]:
    c4e_report_path = _latest_c4e_report()
    c4e = _load_json(c4e_report_path)

    if c4e.get("status") != "OFFLINE_STATIC_STABILITY_VALID_WITH_COM_PROXY_UNCERTAINTY":
        raise HardwareSafeModePreflightError(
            f"C4-E status inatteso: {c4e.get('status')!r}"
        )

    files = [_classify_file(path) for path in _python_files()]

    # Do not classify this audit script as a repository risk just because it
    # contains the scan patterns used to detect serial/command-capable code.
    this_audit_script = str(Path(__file__).resolve().relative_to(REPO_ROOT))
    files = [
        item
        for item in files
        if item["path"] != this_audit_script
    ]

    command_capable = [
        item for item in files if item["command_capable"]
    ]
    direct_serial_risks = [
        item for item in files if item["direct_serial_risk"]
    ]
    first_stand_blacklist = [
        item for item in files if item["first_stand_forbidden"]
    ]
    read_only_tools = [
        item for item in files if item["classification"] == "LIVE_READ_ONLY_TOOL"
    ]
    offline_tools = [
        item for item in files if item["classification"] == "OFFLINE_TOOL"
    ]

    stand_command_candidates = [
        item
        for item in command_capable
        if "stand" in item["path"].lower()
    ]

    status = (
        "OFFLINE_HARDWARE_SAFE_MODE_SOURCE_AUDIT_VALID"
        if not direct_serial_risks and not stand_command_candidates
        else "FAIL"
    )

    return {
        "schema": 1,
        "kind": "MATDOG_C4F_OFFLINE_HARDWARE_SAFE_MODE_SOURCE_AUDIT",
        "status": status,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "repo_root": str(REPO_ROOT),
        "offline_only": True,
        "station_used": False,
        "serial_used": False,
        "motor_command_used": False,
        "source_reports": {
            "c4e_static_stability_support_polygon": str(c4e_report_path),
        },
        "metrics": {
            "python_files_scanned": len(files),
            "offline_tools": len(offline_tools),
            "live_read_only_tools": len(read_only_tools),
            "command_capable_existing_tools": len(command_capable),
            "direct_serial_risks": len(direct_serial_risks),
            "stand_command_candidates": len(stand_command_candidates),
            "first_stand_blacklisted_tools": len(first_stand_blacklist),
        },
        "policy": {
            "station_single_serial_owner": True,
            "direct_serial_allowed": False,
            "first_stand_command_allowed": False,
            "command_capable_existing_tools_allowed_for_first_stand": False,
            "required_next_gate": (
                "manual hardware checklist, explicit operator approval, "
                "abort procedure, then read-only live preflight"
            ),
        },
        "read_only_tools": read_only_tools,
        "offline_tools": offline_tools,
        "command_capable_existing_tools": command_capable,
        "direct_serial_risks": direct_serial_risks,
        "stand_command_candidates": stand_command_candidates,
        "first_stand_blacklist": first_stand_blacklist,
        "files": files,
        "command_eligibility": {
            "command_eligible": False,
            "reason": (
                "C4-F source audit only classifies existing tools and blocks "
                "first-stand execution until hardware checklist, operator "
                "approval and abort procedure are complete."
            ),
        },
    }


def _print_summary(report: dict[str, Any], report_path: Path) -> None:
    metrics = report["metrics"]

    print("=== MATDOG C4-F OFFLINE HARDWARE SAFE-MODE SOURCE AUDIT ===")
    print(f"status: {report['status']}")
    print("")
    print("METRICS:")
    for key, value in metrics.items():
        print(f"  {key}: {value}")

    print("")
    print("FIRST-STAND BLACKLIST:")
    if report["first_stand_blacklist"]:
        for item in report["first_stand_blacklist"]:
            print(f"  - {item['path']}")
    else:
        print("  none")

    print("")
    print("DIRECT SERIAL RISKS:")
    if report["direct_serial_risks"]:
        for item in report["direct_serial_risks"]:
            print(f"  - {item['path']}")
    else:
        print("  none")

    print("")
    print("STAND COMMAND CANDIDATES:")
    if report["stand_command_candidates"]:
        for item in report["stand_command_candidates"]:
            print(f"  - {item['path']}")
    else:
        print("  none")

    print("")
    print(f"report: {report_path}")
    print("COMMAND_ELIGIBLE: false")
    print("Offline only: no Station, serial or motor command was used.")


def main() -> None:
    report = _build_report()
    report_path = _default_report_path()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    _print_summary(report, report_path)

    if report["status"] == "FAIL":
        raise SystemExit(1)


if __name__ == "__main__":
    main()
