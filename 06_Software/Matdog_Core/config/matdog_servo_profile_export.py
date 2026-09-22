#!/usr/bin/env python3
"""Export MATDOG_C018_V1 as a compact constexpr table for the Controller.

Source of truth: MATDOG_ST3215_C018_V1.yaml, right next to this file. The 20
persistent-profile register values are NOT retyped anywhere in the firmware;
they are copied from the YAML by this exporter, together with the SHA-256 of
the YAML itself so the firmware can state which revision it was built from.

  MATDOG_ST3215_C018_V1.yaml        <- the reviewed authority
        -> THIS EXPORTER            <- pure reduction, no policy
        -> src/servo/ServoProfileData.h
        -> ServoProfile verifier    <- compares register by register

THREE THINGS THAT MUST NOT BE CONFLATED, and are not:

  persistent profile   the 20 EEPROM registers that define "provisioned"
  servo invariants     model word, GoalPosition domain, raw centre,
                       PositionOffset, baud - properties of the unit, checked
                       but not part of the 20-register delta set
  runtime RAM state    TorqueLimit, GoalSpeed, Acc - written per motion, never
                       persistent, and explicitly excluded by the YAML's own
                       `runtime_policy.torque_limit_is_persistent_profile: false`

The exporter refuses to emit anything that would blur those boundaries: a
runtime register appearing in `persistent_profile` is a hard failure.

Usage:
    python3 matdog_servo_profile_export.py [--check]

--check verifies the committed header without writing; that is what the
firmware static audit runs.
"""
from __future__ import annotations

import argparse
import hashlib
import re
import sys
from pathlib import Path

CONFIG_DIR = Path(__file__).resolve().parent
REPO_ROOT = CONFIG_DIR.parents[2]

SOURCE = CONFIG_DIR / "MATDOG_ST3215_C018_V1.yaml"
OUTPUT = REPO_ROOT / "05_Firmware/MATDOG_Controller/src/servo/ServoProfileData.h"

EXPECTED_REGISTER_COUNT = 20

# Runtime RAM concerns. If any of these ever appears inside persistent_profile
# the export fails: they are written per motion and are not a provisioning
# contract.
RUNTIME_ONLY = ("TorqueLimit", "GoalSpeed", "Acc", "GoalPosition", "Acceleration")


class ExportError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_source() -> dict:
    text = SOURCE.read_text(encoding="utf-8")

    def scalar(key: str, section: str | None = None) -> str:
        pattern = rf"^\s*{key}:\s*\"?([^\"\n#]+)\"?\s*$"
        scope = text
        if section is not None:
            m = re.search(rf"^{section}:\n(.*?)(?=^\S)", text, re.M | re.S)
            if not m:
                raise ExportError(f"section {section!r} not found in {SOURCE.name}")
            scope = m.group(1)
        m = re.search(pattern, scope, re.M)
        if not m:
            raise ExportError(f"key {key!r} not found in {SOURCE.name}"
                              f"{'' if section is None else ' section ' + section}")
        return m.group(1).strip()

    block = re.search(r"^persistent_profile:\n(.*?)(?=^\S)", text, re.M | re.S)
    if not block:
        raise ExportError("persistent_profile block not found")

    registers = []
    for name, addr, width, value in re.findall(
            r"^\s+(\w+):\s*\{\s*address:\s*\"(0x[0-9A-Fa-f]+)\",\s*"
            r"width:\s*(\d+),\s*value:\s*(\d+)\s*\}\s*$", block.group(1), re.M):
        if name in RUNTIME_ONLY:
            raise ExportError(f"{name!r} is runtime RAM state and must never appear in "
                              f"persistent_profile")
        w = int(width)
        if w not in (1, 2):
            raise ExportError(f"{name}: unsupported register width {w}")
        v = int(value)
        if v < 0 or v > (0xFF if w == 1 else 0xFFFF):
            raise ExportError(f"{name}: value {v} does not fit in {w} byte(s)")
        registers.append({"name": name, "addr": int(addr, 16), "width": w, "value": v})

    if len(registers) != EXPECTED_REGISTER_COUNT:
        raise ExportError(f"parsed {len(registers)} persistent registers, expected "
                          f"{EXPECTED_REGISTER_COUNT}")
    seen = set()
    for reg in registers:
        if reg["addr"] in seen:
            raise ExportError(f"duplicate register address 0x{reg['addr']:02X}")
        seen.add(reg["addr"])
    registers.sort(key=lambda r: r["addr"])

    # --- invariants, kept as a SEPARATE concept from the 20 registers -------
    model_block = re.search(r"model_register:\n(.*?)(?=^\s{2}\w|^\S)", text, re.M | re.S)
    if not model_block:
        raise ExportError("servo.model_register block not found")
    model_addr = int(re.search(r"address:\s*\"(0x[0-9A-Fa-f]+)\"",
                               model_block.group(1)).group(1), 16)
    model_value = int(re.search(r"expected_value:\s*(\d+)", model_block.group(1)).group(1))

    offset_block = re.search(r"^position_offset:\n(.*?)(?=^\S)", text, re.M | re.S)
    if not offset_block:
        raise ExportError("position_offset block not found")
    offset_addr = int(re.search(r"address:\s*\"(0x[0-9A-Fa-f]+)\"",
                                offset_block.group(1)).group(1), 16)
    offset_value = int(re.search(r"^\s*value:\s*(-?\d+)\s*$", offset_block.group(1),
                                 re.M).group(1))
    offset_encoding = re.search(r"encoding:\s*(\S+)", offset_block.group(1)).group(1)
    if offset_encoding != "int16_le_twos_complement":
        raise ExportError(f"unexpected PositionOffset encoding {offset_encoding!r}; the "
                          f"firmware decoder implements two's complement only")

    baud_block = re.search(r"^baud:\n(.*?)(?=^\S)", text, re.M | re.S)
    baud_addr = int(re.search(r"address:\s*\"(0x[0-9A-Fa-f]+)\"", baud_block.group(1)).group(1), 16)
    baud_value = int(re.search(r"expected_value:\s*(\d+)", baud_block.group(1)).group(1))
    if re.search(r"written_by_v1_tooling:\s*(\w+)", baud_block.group(1)).group(1) != "false":
        raise ExportError("baud is declared writable; V1 policy is verification only")

    # The runtime boundary, asserted rather than assumed.
    if scalar("torque_limit_is_persistent_profile", "runtime_policy") != "false":
        raise ExportError("the YAML now claims TorqueLimit is part of the persistent "
                          "profile; that would merge runtime RAM state into a "
                          "provisioning contract")

    return {
        "profile_id": scalar("profile_id"),
        "frozen_at": scalar("frozen_at"),
        "registers": registers,
        "model": {"addr": model_addr, "value": model_value},
        "offset": {"addr": offset_addr, "value": offset_value},
        "baud": {"addr": baud_addr, "value": baud_value},
        "raw_center": int(scalar("physical_raw_center", "invariants")),
        "center_tolerance": int(scalar("center_acceptance_ticks", "invariants")),
        "source_sha256": sha256_file(SOURCE),
    }


def render(data: dict) -> str:
    lines = []
    add = lines.append
    add("// GENERATED FILE - DO NOT EDIT BY HAND.")
    add("//")
    add("// Produced by 06_Software/Matdog_Core/config/matdog_servo_profile_export.py")
    add(f"// from 06_Software/Matdog_Core/config/{SOURCE.name}")
    add(f"//   sha256 {data['source_sha256']}")
    add("//")
    add("// The 20 persistent-profile register values are NOT retyped here; they are")
    add("// copied from the reviewed YAML. Regenerate with the exporter; a hand-patched")
    add("// value is caught by the static audit.")
    add("")
    add("#ifndef MATDOG_SERVO_SERVO_PROFILE_DATA_H")
    add("#define MATDOG_SERVO_SERVO_PROFILE_DATA_H")
    add("")
    add('#include "ServoProfile.h"')
    add("")
    add("namespace matdog {")
    add("namespace servo {")
    add("namespace profile_data {")
    add("")
    add(f'constexpr char kProfileId[] = "{data["profile_id"]}";')
    add(f'constexpr char kFrozenAt[] = "{data["frozen_at"]}";')
    add(f'constexpr char kSourceSha256[] = "{data["source_sha256"]}";')
    add("")
    add("// The persistent EEPROM contract: a unit either matches all twenty or it is")
    add("// not provisioned. Sorted by address so the verifier reads the bus in order.")
    add(f"constexpr uint8_t kPersistentRegisterCount = {len(data['registers'])};")
    add("constexpr ProfileRegister kPersistentRegisters[kPersistentRegisterCount] = {")
    for reg in data["registers"]:
        add(f"    {{0x{reg['addr']:02X}, {reg['width']}, {reg['value']}, "
            f'"{reg["name"]}"}},')
    add("};")
    add("")
    add("// Servo invariants. Checked, but NOT part of the twenty-register delta set -")
    add("// they are properties of the unit rather than of the provisioning contract.")
    add("constexpr ServoInvariants kInvariants = {")
    add(f"    0x{data['model']['addr']:02X}, {data['model']['value']},   // model word")
    add(f"    0x{data['offset']['addr']:02X}, {data['offset']['value']},"
        f"     // PositionOffset, int16 LE two's complement")
    add(f"    0x{data['baud']['addr']:02X}, {data['baud']['value']},"
        f"      // BaudRate - verification only, never written")
    add(f"    {data['raw_center']}, {data['center_tolerance']},"
        f"  // physical raw centre and its acceptance band")
    add("};")
    add("")
    add("}  // namespace profile_data")
    add("}  // namespace servo")
    add("}  // namespace matdog")
    add("")
    add("#endif  // MATDOG_SERVO_SERVO_PROFILE_DATA_H")
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true",
                        help="verify the committed header without writing")
    args = parser.parse_args()

    try:
        rendered = render(parse_source())
    except ExportError as exc:
        print(f"SERVO_PROFILE_EXPORT = FAIL\n  {exc}", file=sys.stderr)
        return 1

    if args.check:
        if not OUTPUT.exists() or OUTPUT.read_text(encoding="utf-8") != rendered:
            print("SERVO_PROFILE_EXPORT = FAIL\n"
                  f"  {OUTPUT.relative_to(REPO_ROOT)} does not match {SOURCE.name}.\n"
                  "  Regenerate with matdog_servo_profile_export.py; do not edit it.",
                  file=sys.stderr)
            return 1
        print("SERVO_PROFILE_EXPORT = PASS (header matches the canonical YAML)")
        return 0

    OUTPUT.write_text(rendered, encoding="utf-8")
    print(f"SERVO_PROFILE_EXPORT = PASS\n"
          f"  wrote {OUTPUT.relative_to(REPO_ROOT)}\n"
          f"  {EXPECTED_REGISTER_COUNT} persistent registers + invariants")
    return 0


if __name__ == "__main__":
    sys.exit(main())
