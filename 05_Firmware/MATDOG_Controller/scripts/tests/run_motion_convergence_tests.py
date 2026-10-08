#!/usr/bin/env python3
"""Source, purity and provenance gate for the SELECTIVELY integrated pure motion library (PR-2).

This gate checks the integrated source set against motion_integration_manifest_pr2.json, the
purity/dependency/wiring boundaries of src/motion, and that every DEFERRED artifact is absent
from the tree. The motion execution suites (motion host tests, fresh oracles, audit-tool tests)
are DEFERRED together with the files they need; they are reported as such, never as PASS.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import re

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
FW = ROOT / "05_Firmware/MATDOG_Controller"
PINS = HERE / "motion_integration_manifest_pr2.json"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_sources():
    pins = json.loads(PINS.read_text())
    checks = []
    failures = []
    for group in ("copied_sources", "canonical_inputs", "retained_dependencies", "protected_installation"):
        for row in pins[group]:
            path = ROOT / row["path"]
            actual = digest(path) if path.is_file() else None
            passed = actual == row["sha256"]
            checks.append(dict(group=group, path=row["path"], expected=row["sha256"], actual=actual, passed=passed))
            if not passed:
                failures.append("source mismatch: " + row["path"])
    # Deferred artifacts are recorded by path and SHA-256 only. They must NOT be present: this
    # keeps excluded third-party material out of the tree and makes deferral explicit.
    for row in pins["deferred_sources"]:
        present = (ROOT / row["path"]).exists()
        checks.append(dict(group="deferred_sources", path=row["path"], expected="ABSENT",
                           actual="PRESENT" if present else "ABSENT", passed=not present))
        if present:
            failures.append("deferred artifact present in tree: " + row["path"])
    if (ROOT / "09_Logs/Validation_Reports/G35_Pose_Audit/xgo_static_extracts").exists():
        failures.append("excluded third-party extracts directory is present")
    motion = FW / "src/motion"
    local_headers = {p.name for p in motion.glob("*.h")}
    allowed_standard = {"stdint.h", "cmath", "limits"}
    forbidden = re.compile(r"\b(?:Arduino|Adafruit|ServoBus|Preferences|HardwareSerial|WiFi|GoalPosition|raw_ticks|EEPROM|EnableTorque|WritePosEx|sh2_saveDcdNow)\b|\b(?:malloc|calloc|realloc|free|millis|micros|delay)\s*\(")
    for path in sorted(motion.iterdir()):
        if path.suffix not in (".h", ".cpp"):
            continue
        code = re.sub(r"/\*.*?\*/|//[^\n]*", "", path.read_text(), flags=re.S)
        for bracket, name in re.findall(r'^\s*#\s*include\s*([<"])([^>"]+)[>"]', code, re.M):
            passed = name in (local_headers if bracket == '"' else allowed_standard)
            if not passed:
                failures.append("motion dependency escape: " + str(path.relative_to(ROOT)) + ": " + name)
        if forbidden.search(code):
            failures.append("hardware/allocation primitive in pure motion: " + str(path.relative_to(ROOT)))
    # The port remains a library: no existing runtime owner may acquire a motion dependency.
    for path in sorted((FW / "src").rglob("*")):
        if not path.is_file() or path.suffix not in (".h", ".cpp") or path.is_relative_to(motion):
            continue
        code = re.sub(r"/\*.*?\*/|//[^\n]*", "", path.read_text(), flags=re.S)
        if re.search(r'#\s*include\s*[<"][^>"]*motion/|\b(?:matdog\s*::\s*)?motion\s*::', code):
            failures.append("live motion wiring outside pure library: " + str(path.relative_to(ROOT)))
        if "esp_phy_erase_cal_data_in_nvs" in code:
            failures.append("diagnostic PHY erase imported: " + str(path.relative_to(ROOT)))
    return dict(passed=not failures, failures=failures, checks=checks,
                source_commit=pins["source_commit"], baseline_commit=pins["baseline_commit"],
                copied_file_count=len(pins["copied_sources"]),
                deferred_file_count=len(pins["deferred_sources"]),
                manifest_version=pins["manifest_version"],
                hardware_or_network_access=False, historical_evidence_regenerated=False,
                pure_library=True, runtime_actuator_or_imu_wiring_added=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-only", action="store_true",
                        help="Accepted for compatibility with the static audit; this gate is source-only.")
    parser.parse_args()
    provenance = check_sources()
    print(json.dumps(provenance, sort_keys=True))
    print("MOTION_CONVERGENCE_SOURCE = " + ("PASS" if provenance["passed"] else "FAIL"))
    print("MOTION_EXECUTION_SUITES = DEFERRED (%d artifacts deferred: motion host tests, fresh oracles and "
          "audit-tool tests are not integrated in this selection)" % provenance["deferred_file_count"])
    return 0 if provenance["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
