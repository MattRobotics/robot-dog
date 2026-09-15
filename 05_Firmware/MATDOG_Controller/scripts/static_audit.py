#!/usr/bin/env python3
"""MATDOG Controller V0.1 static safety audit.

This is a regression tripwire, not a formal verifier (handoff section 31).
It fails the build if forbidden functionality leaks into the operational
firmware: EEPROM/ID/CalibrationOfs writes, automatic torque-on, signed
GoalPosition, automatic BNO085 DCD save, DALY configuration writes, UART
peripheral collisions, and duplicate GPIO ownership.

Session 2 (hardening) additions: a synchronous multi-ID servo scan loop
(must stay a one-Ping()-per-tick state machine), LED transport driven while
the rail is unpowered (USB_ONLY profile), the three power-availability
profile flags drifting from their current USB_ONLY values, and the
application-only flash script regressing to reference the
bootloader/partition-table/boot_app0 artifacts it must never write.

Usage: python3 static_audit.py [sketch_dir]
Exit code 0 = PASS, 1 = FAIL.
"""
import re
import sys
from pathlib import Path

SKETCH_DIR = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent.parent

SOURCE_EXTS = {".h", ".hpp", ".c", ".cpp", ".ino"}


def iter_source_files():
    for path in sorted(SKETCH_DIR.rglob("*")):
        if path.suffix in SOURCE_EXTS and path.is_file():
            yield path


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//.*", "", text)
    return text


failures = []
warnings = []


def fail(msg):
    failures.append(msg)


def check_forbidden_literals(files):
    forbidden = [
        "CalibrationOfs",
        "runNormalizeMatdog",
        "NORMALIZE_MATDOG",
        "unLockEprom",
        "LockEprom",
        "sh2_saveDcdNow",
        "WritePosEx",
        "RegWritePosEx",
        "SyncWritePosEx",
        "WheelMode",
        "SMS_STS_GOAL_POSITION",
        "SMS_STS_OFS_L",
        "SMS_STS_OFS_H",
        "factory reset",
        "FactoryReset",
        "broadcast write",
    ]
    for path, code in files:
        for token in forbidden:
            if token.lower() in code.lower():
                fail(f"{path}: forbidden token found: {token!r}")


def check_torque_enable(files):
    pattern = re.compile(r"EnableTorque\([^,]+,\s*([^)]+)\)")
    for path, code in files:
        for match in pattern.finditer(code):
            arg = match.group(1).strip()
            if arg not in {"0"}:
                fail(f"{path}: EnableTorque called with non-zero argument {arg!r} "
                     f"(automatic/host torque-on is forbidden)")


def check_servo_id_write(files):
    # Any writeByte/writeWord call at all is suspicious in V0.1 — the module
    # is read-only plus the single EnableTorque(id, 0) safety write, which
    # goes through the library's own EnableTorque(), not a raw register write.
    pattern = re.compile(r"\bwriteByte\(|\bwriteWord\(")
    for path, code in files:
        if "ServoBus.cpp" not in str(path):
            continue
        if pattern.search(code):
            fail(f"{path}: raw writeByte/writeWord call found — V0.1 ServoBus must stay read-only "
                 f"plus EnableTorque(id, 0) only")


def check_daly_write(files):
    for path, code in files:
        if "DalyBms.cpp" not in str(path):
            continue
        write_calls = re.findall(r"bms_uart_\.write\(", code)
        if len(write_calls) > 1:
            fail(f"{path}: more than one bms_uart_.write() call found — "
                 f"V0.1 DALY module must only ever transmit the fixed read query")

        query_match = re.search(r"kQuery\[\]\s*=\s*\{([^}]+)\}", code, re.DOTALL)
        if not query_match:
            fail(f"{path}: could not locate kQuery[] to verify Modbus function code")
        else:
            bytes_hex = re.findall(r"0x[0-9A-Fa-f]{2}", query_match.group(1))
            if len(bytes_hex) < 2:
                fail(f"{path}: kQuery[] too short to contain a function code")
            elif int(bytes_hex[1], 16) != 0x03:
                fail(f"{path}: kQuery[] function code is {bytes_hex[1]}, expected 0x03 (READ)")

    for path, code in files:
        if path.name == "DalyBms.h":
            if "bool requestDischargeOff() { return false; }" not in code:
                fail(f"{path}: requestDischargeOff() must be a no-op returning false "
                     f"until the K-Series write protocol is verified (handoff 8A.8)")


def check_pin_collisions(files):
    for path, code in files:
        if path.name != "Pins.h":
            continue
        matches = re.findall(r"constexpr int (k\w+)\s*=\s*(\d+);", code)
        seen = {}
        for name, value in matches:
            if value in seen:
                fail(f"{path}: GPIO{value} assigned to both {seen[value]} and {name}")
            else:
                seen[value] = name
        if len(matches) < 10:
            fail(f"{path}: expected at least 10 pin constants, found {len(matches)} "
                 f"(audit may be out of sync with Pins.h)")


def check_uart_peripheral_separation(files):
    servo_idx = None
    daly_idx = None
    for path, code in files:
        if path.name == "ServoBus.h":
            m = re.search(r"HardwareSerial\s+servo_uart_\{(\d+)\}", code)
            if m:
                servo_idx = m.group(1)
        if path.name == "DalyBms.h":
            m = re.search(r"HardwareSerial\s+bms_uart_\{(\d+)\}", code)
            if m:
                daly_idx = m.group(1)
    if servo_idx is None or daly_idx is None:
        fail("could not locate HardwareSerial peripheral index for ServoBus and/or DalyBms")
    elif servo_idx == daly_idx:
        fail(f"ServoBus and DalyBms both bound to HardwareSerial({servo_idx}) — UART collision")


def check_no_auto_scan_on_boot(files):
    for path, code in files:
        if path.name == "ServoBus.cpp":
            begin_match = re.search(r"bool ServoBus::begin\(\)\s*\{(.*?)\n\}", code, re.DOTALL)
            if begin_match and ("startScan(" in begin_match.group(1) or ".Ping(" in begin_match.group(1)):
                fail(f"{path}: ServoBus::begin() must not automatically ping/scan the bus")


def check_servo_scan_non_blocking(files):
    # Session 2 regression tripwire: Session 1's scan() pinged an entire ID
    # range synchronously in one call (each Ping() carries ~100ms IOTimeOut),
    # which could monopolize loop() for seconds. Fail if a `for` loop and a
    # `.Ping(` call ever end up back inside the same brace-free block again -
    # the non-blocking design calls at most one Ping() per update() tick,
    # never inside a loop over an ID range.
    pattern = re.compile(r"for\s*\([^)]*\)\s*\{[^{}]*\.Ping\(", re.DOTALL)
    for path, code in files:
        if path.name != "ServoBus.cpp":
            continue
        if pattern.search(code):
            fail(f"{path}: found a `for` loop calling .Ping() — servo scanning must probe "
                 f"exactly one ID per update() tick (non-blocking), not loop over a range "
                 f"synchronously")
        if "startScan(" not in code or "ScanState::RUNNING" not in code:
            fail(f"{path}: expected non-blocking startScan()/ScanState state machine not found")


def check_led_anti_back_power(files):
    # Session 2 hardening: under USB_ONLY (build::kLedRailPowered == false)
    # the WS2812 transport must never be initialized or driven — no
    # pixels_.begin()/show() may execute before the kLedRailPowered guard.
    for path, code in files:
        if path.name != "LedRing.cpp":
            continue
        begin_match = re.search(r"bool LedRing::begin\(\)\s*\{(.*?)\n\}", code, re.DOTALL)
        if not begin_match:
            fail(f"{path}: could not locate LedRing::begin() to audit anti-back-power guard")
            continue
        body = begin_match.group(1)
        guard_pos = body.find("if (!build::kLedRailPowered)")
        pixels_begin_pos = body.find("pixels_.begin()")
        if guard_pos == -1:
            fail(f"{path}: LedRing::begin() is missing the kLedRailPowered guard")
        elif pixels_begin_pos != -1 and pixels_begin_pos < guard_pos:
            fail(f"{path}: pixels_.begin() appears before the kLedRailPowered guard in begin()")

    for path, code in files:
        if path.name != "BuildConfig.h":
            continue
        # This session is exclusively USB_ONLY hardening (see handoff
        # Session 2). These three flags must read false right now; flipping
        # any of them is a deliberate future ROBOT_POWERED change, not
        # something that should happen silently in this codebase's history.
        for flag in ("kServoPowerAvailable", "kBatteryAvailable", "kLedRailPowered"):
            m = re.search(rf"constexpr bool {flag}\s*=\s*(\w+);", code)
            if not m:
                fail(f"{path}: could not locate {flag} to audit its value")
            elif m.group(1) != "false":
                fail(f"{path}: {flag} = {m.group(1)}, expected false for the current "
                     f"USB_ONLY-only session (flip deliberately for ROBOT_POWERED work)")


def check_app_only_script_never_targets_other_partitions(sketch_dir):
    # Not a C++ source check: audits the application-only flashing script
    # itself so it can never be edited into silently writing the
    # bootloader/partition-table/boot_app0/otadata regions again.
    forbidden_names = ("bootloader.bin", "partitions.bin", "boot_app0.bin")
    scripts_dir = sketch_dir / "scripts"
    target = scripts_dir / "flash_app_only.sh"
    if not target.exists():
        fail(f"{target}: application-only flash script not found")
        return
    text = target.read_text(encoding="utf-8")
    for name in forbidden_names:
        if name in text:
            fail(f"{target}: references {name!r} — the application-only flasher must never "
                 f"name the bootloader/partition-table/boot_app0 artifacts")
    for required in ("APPLICATION_OFFSET", "APPLICATION_SHA256", "MAX_PARTITION_SIZE"):
        if required not in text:
            fail(f"{target}: missing required safety-gate output {required!r}")


def main():
    files = [(p, strip_comments(p.read_text(encoding="utf-8"))) for p in iter_source_files()]

    if not files:
        fail(f"no source files found under {SKETCH_DIR}")

    check_forbidden_literals(files)
    check_torque_enable(files)
    check_servo_id_write(files)
    check_daly_write(files)
    check_pin_collisions(files)
    check_uart_peripheral_separation(files)
    check_no_auto_scan_on_boot(files)
    check_servo_scan_non_blocking(files)
    check_led_anti_back_power(files)
    check_app_only_script_never_targets_other_partitions(SKETCH_DIR)

    print(f"Scanned {len(files)} source files under {SKETCH_DIR}")

    if failures:
        print(f"\nSTATIC_AUDIT = FAIL ({len(failures)} finding(s))")
        for f in failures:
            print(f"  - {f}")
        return 1

    print("STATIC_AUDIT = PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
