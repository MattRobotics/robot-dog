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

Session 2.1 (final pre-merge hardening) additions: @SERVO SCAN/@SERVO READ
reachable outside core::OperatingMode::MAINTENANCE (both can block for a
bounded but real per-ID timeout - see ServoBus.h), @SERVO SAFE_OFF
regressing to require MAINTENANCE (it must stay reachable in every mode),
and the OTA partition verifier's fail-closed validity checks (CRC/state,
not just raw sequence-number comparison) regressing or its offline test
suite failing.

Session 2.2 (final merge gate) additions: the OTA subtype filter
regressing to a numeric threshold instead of the real
(subtype & 0xF0) == PART_SUBTYPE_OTA_FLAG bitmask (which would again match
PART_SUBTYPE_TEST/TEE_0/TEE_1), the rollback/anti-rollback parameters
gaining an unsafe default or the flasher no longer reading them from the
real build's sdkconfig, ServoBus::begin() turning the diagnostic servo
timeout back into a standing global override instead of a scoped
per-transaction one, and @SERVO SAFE_OFF classifying success from
EnableTorque()'s own return value again instead of an independent
TorqueEnable readback (SCS::Ack() returns 0 on failure, not -1, so a
naive `>= 0` check can never observe failure).

Session 2.3 (final consistency fix) additions: safeOff()/readRuntimeState()
regressing to use kDiagnosticTimeoutMs (the 20ms MAINTENANCE-only
absence-detection budget) instead of kOperationalTimeoutMs (Finding 1 - both
are operational/safety primitives reachable outside MAINTENANCE, or reused
by a future motion controller); the sdkconfig rollback/anti-rollback parser
losing its three-way SdkconfigFlag.UNKNOWN case and going back to treating
"symbol absent" the same as "symbol explicitly disabled" (Finding 2); and
ota_app_partitions() losing its contiguous-slot-index requirement, letting a
sparse OTA layout (e.g. {0, 2}) resolve instead of refusing (Finding 3).

G2 (ROBOT_POWERED configuration support) additions: the three rail
availability flags regressing from DERIVED values back to independently
editable literals (they must come from config/HardwareProfile.h's single
expectationsFor() table, so a profile name can never contradict its own
rail facts); the profile table itself mismapping USB_ONLY/ROBOT_POWERED;
the compiled-in DEFAULT profile being anything other than USB_ONLY (G3 —
powered hardware validation — is not authorized, so a ROBOT_POWERED image
must never be producible by an unreviewed edit); the servo population model
losing the canonical-17 / expected-now-13 / absent-by-design-4 distinction
or drifting from MATDOG_SERVO_ALLOCATION.yaml; a "17 responders = PASS"
rule reappearing (false for the current robot); the population/census
translation units gaining a Serial or Arduino dependency (they must stay
pure so a future telemetry snapshot and Web UI can reuse the SAME
classification without re-scanning the bus); Controller::begin() starting a
servo scan/census at boot; and a direct network-handler -> servo-primitive
path (no network subsystem exists yet — this is a tripwire armed in
advance, not a test of invented code).

G2 pre-G3 hardening additions (independent review findings 1 and 2): the
build-manifest profile-provenance gate being removed, neutralized, given a
permissive default, or moved after the device write (build.sh must record
the hardware profile + binary digest it produced; flash_app_only.sh must
verify them and require MATDOG_FLASH_PROFILE before writing, without
weakening any pre-existing backup/MAC/partition/rollback/verify gate); and
classify() collapsing DetectedState::UNKNOWN back into an observed absence,
or a module faking a physical WS2812 detection to make a status green.

Pre-G3 closure addition (review Finding A): the recorded build FQBN going
unverified again. The manifest carried FQBN from the start but nothing
compared it, leaving the partition scheme, flash size/mode, PSRAM mode,
USB/CDC mode and CPU frequency unchecked at flash time. verify_manifest()
must take an explicit expected_fqbn with no default, compare it for exact
equality, and flash_app_only.sh must pass its own pinned "$FQBN".

G3.1 (live regression found after G3, 2026-09-18) addition: USB CDC
transmit regaining the ability to block the Controller loop. The installed
HWCDC (esp32:esp32 3.3.11) keeps a host "connected" after it closes the
port and, with its default 100 ms TX timeout, waits up to ~2 s per print on
a full ring (BNO085 RV fell from 50.07 Hz to 0.68 Hz). The TX timeout must
be explicitly 0 and set, with the ring size, before Serial.begin() and any
output; nothing may set it again; Serial.flush() (discards or waits) and
debug-output routing (a second per-character transmit path) are forbidden.

DALY KEY read-only probe (2026-09-19) additions: a second DALY transaction
(the one-shot 0x81 KEY/parameter read) replaced the old "exactly one fixed
kQuery[]" rule with a stricter whitelist, not a looser one. DALY firmware may
put on the bus ONLY the two known FC03 read frames (D2 03 00 00 00 3E D7 B9 and
81 03 01 00 00 78 5B D4), each re-verified here byte-for-byte, function 0x03
and CRC-16/MODBUS; through exactly ONE bms_uart_.write() whose bytes come only
from dalyRequestFrame(<enum>); with no other use of bms_uart_, no other UART2
route, no other initialized byte array, and no Modbus write function literal
(0x06/0x10) in the DALY sources. requestDischargeOff() stays a no-op returning
false. The command surface gains only two argument-free commands (@BMS KEY
READ, MAINTENANCE-gated; @BMS KEY STATUS, cache-only) and no @BMS command may
parse arguments or name a write. The KEY probe APIs may not leak into the
power-state or health logic. The protocol unit stays Arduino-free for the host
suite, and scripts/tests/test_static_audit_daly.py proves these rules fail on
mutation (e.g. the KEY request function 0x03 -> 0x06).

DALY KEY discharge configuration (2026-09-19) addition - the ONE permitted
DALY write, made obvious here (DALY_THE_ONE_WRITE): FC06, address 0x81,
register 0x0120 (KEY logic) := 0x005A (DISCHARGE), frame 81 06 01 20 00 5A
16 07, built by the parameterless dalyKeyLogicDischargeWriteFrame() whose
whole body must match the reviewed recipe (literal bytes 0-5, CRC appended by
crc16Modbus). Only @BMS KEY SET DISCHARGE CONFIRM (exact text,
MAINTENANCE-gated, passing only the live operating mode) can request it, via
exactly one call chain. Still forbidden: FC10 anywhere; FC06 to any other
register (0x0121/0x0122 MOS control included) or with any other value; a
second write frame; any caller-supplied address/register/value; raw write
commands; persistence in the DALY module; any requestDischargeOff() body.
The write may leave the UART only if the operating mode is STILL MAINTENANCE
at the final pre-transmit check: DalyBms::update() takes the live mode from
the Controller on every loop and the check must use it (never `true`, a
constant, or the mode seen when the command arrived).

Usage: python3 static_audit.py [sketch_dir]
Exit code 0 = PASS, 1 = FAIL.
"""
import os
import pathlib
import re
import subprocess
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
        # SMS_STS_OFS_L is NOT banned outright any more. A total ban blocked
        # reading PositionOffset as well as writing it, and left the Controller
        # unable to verify the single most safety-relevant provisioning fact.
        # check_position_offset_boundary() replaces it with a narrower and
        # stronger rule: one approved read accessor, and no write, ever.
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


# The complete DALY transmit vocabulary. Each entry is re-verified below
# (function 0x03, CRC-16/MODBUS), so editing this table to admit a write frame
# still fails the audit.
DALY_WHITELISTED_READ_FRAMES = {
    "kDalyTelemetryRequest": bytes.fromhex("D2 03 00 00 00 3E D7 B9"),
    "kDalyKeyConfigRequest": bytes.fromhex("81 03 01 00 00 78 5B D4"),
}
DALY_SOURCE_NAMES = {"DalyBms.h", "DalyBms.cpp", "DalyProtocol.h", "DalyProtocol.cpp"}
DALY_TRANSMIT_CALL = "bms_uart_.write(dalyRequestFrame(request), kDalyRequestLen)"
DALY_UART_METHODS = {"begin", "available", "read", "write", "flush"}
BMS_COMMANDS_ALLOWED = {"@BMS STATUS", "@BMS STREAM ON", "@BMS STREAM OFF",
                        "@BMS KEY READ", "@BMS KEY STATUS",
                        "@BMS KEY SET DISCHARGE CONFIRM", "@BMS KEY WRITE STATUS"}
BMS_HELP_TOKENS_ALLOWED = BMS_COMMANDS_ALLOWED | {"@BMS STREAM ON|OFF"}

# THE ONE PERMITTED DALY WRITE. Everything below is checked against this.
DALY_THE_ONE_WRITE = {
    "address": 0x81,        # 0x80 + board 1 (the live-verified 0x81 personality)
    "function": 0x06,       # write single register
    "register": 0x0120,     # KEY logic
    "value": 0x005A,        # DISCHARGE: KEY OFF -> discharge MOS OFF, charge MOS kept
}
DALY_WRITE_PAYLOAD = bytes([DALY_THE_ONE_WRITE["address"], DALY_THE_ONE_WRITE["function"],
                            DALY_THE_ONE_WRITE["register"] >> 8,
                            DALY_THE_ONE_WRITE["register"] & 0xFF,
                            DALY_THE_ONE_WRITE["value"] >> 8, DALY_THE_ONE_WRITE["value"] & 0xFF])
# The reviewed recipe, whitespace-normalized: literal bytes 0-5, CRC appended.
DALY_WRITE_BUILDER_BODY = (
    "DalyFrame f = {{0x81, 0x06, 0x01, 0x20, 0x00, 0x5A, 0x00, 0x00}}; "
    "const uint16_t crc = crc16Modbus(f.bytes, kDalyRequestLen - 2); "
    "f.bytes[6] = static_cast<uint8_t>(crc & 0xFF); "
    "f.bytes[7] = static_cast<uint8_t>(crc >> 8); "
    "return f;")


def modbus_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def daly_the_one_write_frame():
    crc = modbus_crc16(DALY_WRITE_PAYLOAD)
    return DALY_WRITE_PAYLOAD + bytes([crc & 0xFF, crc >> 8])


def check_daly_write(files):
    """DALY runtime may transmit only the whitelisted FC03 READ frames and
    the ONE semantic KEY write (DALY_THE_ONE_WRITE).

    Origin: V0.1 allowed exactly one fixed read query and one write call.
    The DALY KEY probe added a second whitelisted read; the KEY discharge
    configuration adds exactly one write. Everything else about the
    invariant is kept or tightened (see the module docstring).
    """
    for name, frame in DALY_WHITELISTED_READ_FRAMES.items():
        if len(frame) != 8 or frame[1] != 0x03 or \
                modbus_crc16(frame[:6]) != frame[6] | (frame[7] << 8):
            fail(f"static_audit.py: whitelisted DALY frame {name} is not a well-formed FC03 "
                 f"read - the whitelist itself must never admit a write")
    if DALY_THE_ONE_WRITE != {"address": 0x81, "function": 0x06, "register": 0x0120,
                              "value": 0x005A} or \
            daly_the_one_write_frame() != bytes.fromhex("81 06 01 20 00 5A 16 07"):
        fail("static_audit.py: DALY_THE_ONE_WRITE drifted from the reviewed KEY logic "
             "DISCHARGE write (FC06 0x0120 := 0x005A at 0x81, frame 81 06 01 20 00 5A 16 07)")

    daly = [(p, c) for p, c in files if p.name in DALY_SOURCE_NAMES]
    by_name = {p.name: (p, c) for p, c in daly}
    for required in ("DalyBms.h", "DalyBms.cpp", "DalyProtocol.h"):
        if required not in by_name:
            fail(f"{required}: DALY source not found - the transmit whitelist cannot be verified")
            return

    # 1. Every initialized byte array in the DALY sources is either a zeroed
    #    receive buffer or one of the whitelisted frames, byte for byte.
    found = {}
    for path, code in daly:
        for m in re.finditer(r"uint8_t\s+(\w+)\s*\[[^\]]*\]\s*=\s*\{([^}]*)\}", code):
            name, body = m.group(1), m.group(2)
            if body.strip() in ("0", ""):
                continue
            values = re.findall(r"0x([0-9A-Fa-f]{1,2})\b|\b(\d+)\b", body)
            frame = bytes(int(h, 16) if h else int(d) for h, d in values)
            if name not in DALY_WHITELISTED_READ_FRAMES:
                fail(f"{path}: initialized byte array {name}[] = {frame.hex(' ')} is not a "
                     f"whitelisted DALY read frame - no other transmit bytes may exist")
                continue
            if name in found:
                fail(f"{path}: {name}[] defined more than once")
            found[name] = frame
            if frame != DALY_WHITELISTED_READ_FRAMES[name]:
                fail(f"{path}: {name}[] = {frame.hex(' ')} differs from the whitelisted "
                     f"{DALY_WHITELISTED_READ_FRAMES[name].hex(' ')}")
            if len(frame) < 2 or frame[1] != 0x03:
                fail(f"{path}: {name}[] function code is "
                     f"{frame[1] if len(frame) > 1 else None!r}, expected 0x03 (READ)")
            elif len(frame) == 8 and modbus_crc16(frame[:6]) != frame[6] | (frame[7] << 8):
                fail(f"{path}: {name}[] CRC-16/MODBUS does not match its bytes")
    for name in DALY_WHITELISTED_READ_FRAMES:
        if name not in found:
            fail(f"DalyProtocol.h: whitelisted read frame {name}[] not found")

    proto_path, proto = by_name["DalyProtocol.h"]
    if not re.search(r"constexpr\s+size_t\s+kDalyRequestLen\s*=\s*8\s*;", proto):
        fail(f"{proto_path}: kDalyRequestLen must be exactly 8 (one FC03 read request)")

    # 2. The only frame selector takes an enum and can return only the two
    #    whitelisted arrays - no buffer, pointer or register parameter.
    selectors = [(p, m) for p, c in daly for m in re.finditer(
        r"const\s+uint8_t\s*\*\s*dalyRequestFrame\s*\(([^)]*)\)\s*\{(.*?)\n\}", c, re.DOTALL)]
    if len(selectors) != 1:
        fail(f"dalyRequestFrame() must be defined exactly once, found {len(selectors)}")
    for path, m in selectors:
        if m.group(1).split() != ["DalyRequest", "request"]:
            fail(f"{path}: dalyRequestFrame() must take only (DalyRequest request), "
                 f"found ({m.group(1).strip()})")
        body = re.sub(r"\s+", " ", m.group(2)).strip()
        expected = ("return request == DalyRequest::KEY_CONFIG ? kDalyKeyConfigRequest "
                    ": request == DalyRequest::KEY_LOGIC_DISCHARGE_WRITE ? "
                    "kDalyKeyLogicDischargeWrite.bytes : kDalyTelemetryRequest;")
        if body != expected:
            fail(f"{path}: dalyRequestFrame() must map KEY_CONFIG -> the KEY read, "
                 f"KEY_LOGIC_DISCHARGE_WRITE -> the one write, anything else -> telemetry, "
                 f"exactly; found {body!r}")

    # 2b. THE ONE WRITE: a single parameterless builder whose body is the
    #     reviewed recipe, and no other DalyFrame anywhere.
    builders = [(p, m) for p, c in daly for m in re.finditer(
        r"constexpr\s+DalyFrame\s+(\w+)\s*\(([^)]*)\)\s*\{(.*?)\n\}", c, re.DOTALL)]
    if len(builders) != 1:
        fail(f"exactly one DalyFrame builder may exist (the KEY logic DISCHARGE write), "
             f"found {len(builders)}")
    for path, m in builders:
        if m.group(1) != "dalyKeyLogicDischargeWriteFrame" or m.group(2).strip():
            fail(f"{path}: the write builder must be the parameterless "
                 f"dalyKeyLogicDischargeWriteFrame() - no caller may supply address, register "
                 f"or value (found {m.group(1)}({m.group(2).strip()}))")
        if re.sub(r"\s+", " ", m.group(3)).strip() != DALY_WRITE_BUILDER_BODY:
            fail(f"{path}: dalyKeyLogicDischargeWriteFrame() differs from the reviewed recipe "
                 f"{DALY_WRITE_BUILDER_BODY!r}")
    for path, code in daly:
        for m in re.finditer(r"DalyFrame\s+\w+\s*=\s*\{\{([^}]*)\}\}", code):
            values = re.findall(r"0x([0-9A-Fa-f]{1,2})\b|\b(\d+)\b", m.group(1))
            payload = bytes(int(h, 16) if h else int(d) for h, d in values)
            if payload != DALY_WRITE_PAYLOAD + b"\x00\x00":
                fail(f"{path}: write payload {payload.hex(' ')} is not the one permitted write "
                     f"{DALY_WRITE_PAYLOAD.hex(' ')} (FC06 0x0120 := 0x005A)")
        constants = re.findall(r"constexpr\s+DalyFrame\s+(\w+)\s*=\s*([^;]*);", code)
        for name, init in constants:
            if name != "kDalyKeyLogicDischargeWrite" or \
                    init.strip() != "dalyKeyLogicDischargeWriteFrame()":
                fail(f"{path}: extra write frame constant {name} = {init.strip()} - only "
                     f"kDalyKeyLogicDischargeWrite may exist")
    for path, code in files:
        if "kDalyKeyLogicDischargeWrite" in code and path.name not in \
                ("DalyProtocol.h", "DalyProtocol.cpp") and "scripts" not in path.parts:
            fail(f"{path}: references the write frame directly - it may only leave through "
                 f"dalyRequestFrame(DalyRequest::KEY_LOGIC_DISCHARGE_WRITE)")

    # 2c. Exactly one call chain can request it: @BMS KEY SET DISCHARGE CONFIRM
    #     -> DalyBms::requestKeyLogicDischarge(mode) -> scheduler.
    sched_calls = [(p, c.count("requestKeyLogicDischargeWrite(")) for p, c in files
                   if "requestKeyLogicDischargeWrite(" in c and "scripts" not in p.parts]
    if sorted((p.name, n) for p, n in sched_calls) != [("DalyBms.cpp", 1), ("DalyProtocol.h", 1)]:
        fail(f"requestKeyLogicDischargeWrite() must be declared once (DalyProtocol.h) and "
             f"called once (DalyBms::requestKeyLogicDischarge), found "
             f"{[(str(p), n) for p, n in sched_calls]}")
    bms_h_path, bms_h = by_name["DalyBms.h"]
    if not re.search(r"DalyKeyWriteGate\s+requestKeyLogicDischarge\(\s*core::OperatingMode\s+"
                     r"mode\s*\)\s*;", bms_h):
        fail(f"{bms_h_path}: requestKeyLogicDischarge() must take only (core::OperatingMode "
             f"mode) - no register or value parameter")

    # 2d. The FC06 frame may leave only if the operating mode is STILL
    #     MAINTENANCE at the pre-transmit check: DalyBms::update() receives
    #     the live mode from the Controller every loop, and every gate input
    #     derives MAINTENANCE from a live mode - never `true`, a constant, or
    #     the mode seen when the command arrived.
    bms_cpp_path, bms_cpp = by_name["DalyBms.cpp"]
    if not re.search(r"void\s+DalyBms::update\(\s*uint32_t\s+now_ms\s*,\s*"
                     r"core::OperatingMode\s+mode\s*\)", bms_cpp):
        fail(f"{bms_cpp_path}: DalyBms::update() must take the live (core::OperatingMode mode)")
    precheck = ("dalyKeyWritePreTransmitCheck(&bus_,&key_write_,keyWriteInputs(mode=="
                "core::OperatingMode::MAINTENANCE,false,now_ms));")
    if re.sub(r"\s+", "", bms_cpp).count(precheck) != 1:
        fail(f"{bms_cpp_path}: the pre-transmit write check must be exactly "
             f"dalyKeyWritePreTransmitCheck(&bus_, &key_write_, keyWriteInputs(mode == "
             f"core::OperatingMode::MAINTENANCE, false, now_ms)) - the live mode, once")
    for m in re.finditer(r"keyWriteInputs\(([^,]*),", bms_cpp):
        if m.group(1).strip() not in ("mode == core::OperatingMode::MAINTENANCE",
                                      "bool maintenance_mode"):
            fail(f"{bms_cpp_path}: keyWriteInputs() called with maintenance="
                 f"{m.group(1).strip()!r} - MAINTENANCE must come from the live operating mode")
    helper_calls = [p for p, c in files if "dalyKeyWritePreTransmitCheck(" in c and
                    "scripts" not in p.parts]
    if sorted(p.name for p in helper_calls) != ["DalyBms.cpp", "DalyProtocol.cpp",
                                                "DalyProtocol.h"]:
        fail(f"dalyKeyWritePreTransmitCheck() must be defined in DalyProtocol and called only by "
             f"DalyBms::update(), found {[str(p) for p in helper_calls]}")
    proto_cpp = by_name.get("DalyProtocol.cpp", (None, ""))[1]
    body = re.search(r"bool dalyKeyWritePreTransmitCheck\(.*?\n\}", proto_cpp, re.DOTALL)
    if not body or not all(t in body.group(0) for t in (
            "evaluateDalyKeyWrite(live)", "cancelQueuedOperatorRequest()",
            "cancelBeforeTransmit(gate)")):
        fail("DalyProtocol.cpp: dalyKeyWritePreTransmitCheck() must evaluate the live inputs and "
             "cancel (zero TX) a write that no longer passes")
    ctl = [(p, c) for p, c in files if p.name == "Controller.cpp"]
    if not ctl or re.findall(r"daly_\.update\(([^;]*)\);", ctl[0][1]) != \
            ["now_ms, operating_mode_.mode()"]:
        fail("Controller.cpp: must call daly_.update(now_ms, operating_mode_.mode()) - the DALY "
             "module needs the operating mode as it is on every loop")

    # 3. Exactly one transmit call in the whole firmware, in DalyBms.cpp,
    #    sending only what dalyRequestFrame() selected.
    writes = [(p, m) for p, c in files for m in re.finditer(r"bms_uart_\.write\(", c)]
    if len(writes) != 1:
        fail(f"expected exactly one bms_uart_.write() call in the firmware, found "
             f"{len(writes)} {[str(p) for p, _ in writes]} - DALY transport must have one "
             f"transmit point")
    for path, code in daly:
        for m in re.finditer(r"bms_uart_\.write\([^;]*\)", code):
            call = re.sub(r"\s+", "", m.group(0))
            if path.name != "DalyBms.cpp" or call != DALY_TRANSMIT_CALL.replace(" ", ""):
                fail(f"{path}: DALY transmit must be exactly {DALY_TRANSMIT_CALL!r}, "
                     f"found {m.group(0)!r}")

    # 4. bms_uart_ is used only for begin/available/read/write/flush - never
    #    print*/other writers, never handed to another function as a stream.
    for path, code in files:
        for m in re.finditer(r"\bbms_uart_\b", code):
            tail = code[m.end():m.end() + 24]
            method = re.match(r"\.(\w+)\(", tail)
            declaration = re.match(r"\{2\}", tail) and \
                code[max(0, m.start() - 16):m.start()].rstrip().endswith("HardwareSerial")
            if declaration:
                continue
            if path.name not in ("DalyBms.h", "DalyBms.cpp") or not method or \
                    method.group(1) not in DALY_UART_METHODS:
                fail(f"{path}: bms_uart_ used as {('bms_uart_' + tail.split(chr(10))[0])!r} - "
                     f"only {sorted(DALY_UART_METHODS)} are allowed, inside DalyBms")

    # 5. No second route to the DALY UART.
    for path, code in files:
        for token in ("Serial2", "uart_write_bytes", "uart_tx_chars"):
            if re.search(rf"\b{token}\b", code):
                fail(f"{path}: {token} is forbidden - bytes for the DALY bus may only leave "
                     f"through DalyBms's single bms_uart_ transmit point")
        for m in re.finditer(r"HardwareSerial\s+(\w+)\s*[{(]", code):
            if m.group(1) not in ("servo_uart_", "bms_uart_"):
                fail(f"{path}: extra HardwareSerial {m.group(1)} - only servo_uart_ and "
                     f"bms_uart_ may own a UART")

    # 6. No Modbus write function code or write helper in the DALY sources.
    for path, code in daly:
        # The one reviewed write payload is the only place 0x06 may appear.
        scan = re.sub(r"DalyFrame\s+f\s*=\s*\{\{0x81, 0x06, 0x01, 0x20, 0x00, 0x5A, "
                      r"0x00, 0x00\}\}", "", code)
        for m in re.finditer(r"\b0[xX](06|10)\b", scan):
            fail(f"{path}: Modbus write function literal {m.group(0)} in DALY source - FC10 is "
                 f"forbidden and FC06 exists only in the one reviewed KEY logic write")
        for token in ("Preferences", "nvs_", "EEPROM"):
            if token in code:
                fail(f"{path}: {token} in the DALY module - no DALY/KEY state may persist; the "
                     f"BMS register is the only state (a power loss after the write must "
                     f"never need a second write to finish it)")
        m = re.search(r"(?i)\b(write_?(single|multiple)_?reg\w*|write_?register\w*|"
                      r"func_?0x(06|10)\w*|fc_?(06|10)\w*|modbus_?write\w*)\b", code)
        if m:
            fail(f"{path}: DALY write helper {m.group(0)!r} is forbidden")

    # 7. requestDischargeOff() stays the fail-closed no-op.
    path, code = by_name["DalyBms.h"]
    if "bool requestDischargeOff() { return false; }" not in code:
        fail(f"{path}: requestDischargeOff() must be a no-op returning false "
             f"until the K-Series write protocol is verified (handoff 8A.8)")
    for p2, c2 in files:
        if "DalyBms::requestDischargeOff" in c2 or c2.count("requestDischargeOff(") > \
                (1 if p2.name == "DalyBms.h" else 0):
            fail(f"{p2}: requestDischargeOff() redefined or called - it must stay the single "
                 f"inline no-op in DalyBms.h")


def check_bms_command_surface(files):
    """The DALY KEY probe is two fixed commands; nothing takes an argument."""
    router = [(p, c) for p, c in files if p.name == "CommandRouter.cpp"]
    if not router:
        fail("CommandRouter.cpp not found - the @BMS command surface cannot be audited")
        return
    path, code = router[0]

    compared = set(re.findall(r'upper\s*==\s*"(@BMS[^"]*)"', code))
    for command in sorted(compared - BMS_COMMANDS_ALLOWED):
        fail(f"{path}: unreviewed BMS command {command!r} - only "
             f"{sorted(BMS_COMMANDS_ALLOWED)} may exist")
    for command in sorted(BMS_COMMANDS_ALLOWED - {"@BMS STATUS", "@BMS STREAM ON",
                                                  "@BMS STREAM OFF"}):
        if command not in compared:
            fail(f"{path}: {command!r} handler not found")
    if re.search(r'startsWith\(\s*"@BMS', code) or re.search(r'sscanf\([^;]*"@BMS', code):
        fail(f"{path}: an @BMS command parses arguments - no register, address or value "
             f"input may reach the DALY module")
    # Every @BMS command text anywhere in the router (handlers, help) must be
    # one of the reviewed commands - no other SET/WRITE/REG/MOS spelling.
    for literal in re.findall(r'"([^"\n]*@BMS[^"\n]*)"', code):
        for token in re.findall(r"@BMS(?: [A-Z|]+)*", literal):
            if token not in BMS_HELP_TOKENS_ALLOWED:
                fail(f"{path}: unreviewed BMS command text {token!r}")

    branch = re.search(r'upper\s*==\s*"@BMS KEY READ"\)\s*\{(.*?)\}\s*else if', code, re.DOTALL)
    if not branch or "modules_.operating_mode->mode() != OperatingMode::MAINTENANCE" \
            not in branch.group(1):
        fail(f"{path}: @BMS KEY READ must refuse outside OperatingMode::MAINTENANCE")
    if code.count("requestKeyConfigRead(") != 1 or \
            (branch and "requestKeyConfigRead(" not in branch.group(1)):
        fail(f"{path}: requestKeyConfigRead() must be called once, from @BMS KEY READ only - "
             f"@BMS KEY STATUS performs zero bus transactions")

    # The one write: MAINTENANCE-gated, requested exactly once, from its own
    # branch, with the live operating mode (never a constant).
    set_branch = re.search(r'upper\s*==\s*"@BMS KEY SET DISCHARGE CONFIRM"\)\s*\{(.*?)'
                           r'\}\s*else if\s*\(upper\s*==\s*"@BMS KEY WRITE STATUS"\)', code,
                           re.DOTALL)
    if not set_branch or "modules_.operating_mode->mode() != OperatingMode::MAINTENANCE" \
            not in set_branch.group(1):
        fail(f"{path}: @BMS KEY SET DISCHARGE CONFIRM must refuse outside "
             f"OperatingMode::MAINTENANCE (and be followed by @BMS KEY WRITE STATUS)")
    calls = re.findall(r"requestKeyLogicDischarge\(([^)]*\)?)\)", code)
    if calls != ["modules_.operating_mode->mode()"] or \
            (set_branch and "requestKeyLogicDischarge(" not in set_branch.group(1)):
        fail(f"{path}: requestKeyLogicDischarge() must be called exactly once, from @BMS KEY SET "
             f"DISCHARGE CONFIRM, with modules_.operating_mode->mode() - found {calls}")

    # The KEY probe is diagnostics only: no power-state/health consumer yet.
    # Firmware sources only - the offline host suite exercises these APIs.
    for p2, c2 in files:
        if "scripts" in p2.parts or p2.name in DALY_SOURCE_NAMES or \
                p2.name in ("CommandRouter.cpp", "CommandRouter.h"):
            continue
        for token in ("requestKeyConfigRead", "keyConfigSnapshot", "keyConfigReadResult",
                      "DalyKeyLogic", "DalyKeyConfigSnapshot", "requestKeyLogicDischarge",
                      "keyWriteStatus", "DalyKeyWrite", "kDalyKeyLogicDischargeWrite"):
            if token in c2:
                fail(f"{p2}: uses the DALY KEY probe ({token}) - the KEY mapping must not "
                     f"become operational before it is live-validated")


def check_daly_protocol_is_host_linkable(files):
    """The protocol unit is linked by the offline host suite: it must stay
    free of Arduino, Serial and wall-clock time so the tests exercise the
    shipped frames/decoders/scheduling, not a copy."""
    names = {p.name for p, _ in files}
    for required in ("DalyProtocol.h", "DalyProtocol.cpp"):
        if required not in names:
            fail(f"{required}: DALY protocol unit not found")
    for path, code in files:
        if path.name not in ("DalyProtocol.h", "DalyProtocol.cpp"):
            continue
        for token in ("#include <Arduino.h>", "Serial.", "millis(", "HardwareSerial"):
            if token in code:
                fail(f"{path}: contains {token!r} - keep the DALY protocol unit host-linkable")


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


def check_servo_scan_bounded_incremental(files):
    # Session 2 regression tripwire, terminology corrected in Session 2.1:
    # Session 1's scan() pinged an entire ID range synchronously in one call
    # (each Ping() carries a per-call IOTimeOut), which could monopolize
    # loop() for the whole range. Fail if a `for` loop and a `.Ping(` call
    # ever end up back inside the same brace-free block again - the
    # incremental design calls at most one Ping() per update() tick, never
    # inside a loop over an ID range. This is "bounded per-ID blocking", not
    # non-blocking — see ServoBus.h. Session 2.1 additionally requires that
    # usage be confined to MAINTENANCE mode; see
    # check_servo_diagnostics_require_maintenance_mode below.
    pattern = re.compile(r"for\s*\([^)]*\)\s*\{[^{}]*\.Ping\(", re.DOTALL)
    for path, code in files:
        if path.name != "ServoBus.cpp":
            continue
        if pattern.search(code):
            fail(f"{path}: found a `for` loop calling .Ping() — servo scanning must probe "
                 f"exactly one ID per update() tick (bounded per-ID blocking), not loop over "
                 f"a range synchronously")
        if "startScan(" not in code or "ScanState::RUNNING" not in code:
            fail(f"{path}: expected incremental startScan()/ScanState state machine not found")


def check_servo_diagnostics_require_maintenance_mode(files):
    # Session 2.1: @SERVO SCAN and @SERVO READ can each block for up to one
    # SCServo IOTimeOut per ID (see ServoBus.h kPingTimeoutMs) - acceptable
    # for a diagnostic tool, not for a future deterministic motion loop.
    # Both must refuse outside core::OperatingMode::MAINTENANCE.
    for path, code in files:
        if path.name != "CommandRouter.cpp":
            continue
        guard = "modules_.operating_mode->mode() != OperatingMode::MAINTENANCE"
        count = code.count(guard)
        if count < 2:
            fail(f"{path}: expected the MAINTENANCE-mode guard on both @SERVO SCAN and "
                 f"@SERVO READ, found it protecting only {count} command(s)")

        # @SERVO SAFE_OFF is the one servo write that can only decrease risk
        # (torque off) and must stay reachable in every mode - it must never
        # gain this guard.
        safe_off_branch = re.search(
            r'upper\.startsWith\("@SERVO SAFE_OFF"\)\)\s*\{(.*?)\}\s*else if',
            code, re.DOTALL)
        if safe_off_branch and "OperatingMode::MAINTENANCE" in safe_off_branch.group(1):
            fail(f"{path}: @SERVO SAFE_OFF must remain reachable regardless of operating "
                 f"mode (handoff: 'SAFE_OFF resta l'unico torque write consentito') - found "
                 f"a MAINTENANCE-mode gate on it")


def check_ota_partition_verifier_fail_closed(sketch_dir):
    # Session 2.1: the OTA partition selection logic must validate the real
    # esp_ota_select_entry_t CRC/state fields (not just compare raw seq
    # numbers) and refuse on every ambiguous case, matching the actual
    # installed ESP-IDF bootloader algorithm - see
    # scripts/ota_partition_logic.py's module docstring for the exact
    # source citation.
    #
    # Session 2.2 additions: (Finding A) OTA subtype recognition must use
    # the real (subtype & 0xF0) == 0x10 bitmask, never a `>= 0x10`
    # threshold that would also match PART_SUBTYPE_TEST/TEE_0/TEE_1.
    # (Finding B) the selector must require the caller to supply
    # rollback_enabled/anti_rollback_enabled explicitly (no silent default)
    # and refuse when the real build's sdkconfig has either enabled in a
    # way this tool cannot safely reason about.
    logic_path = sketch_dir / "scripts" / "ota_partition_logic.py"
    if not logic_path.exists():
        fail(f"{logic_path}: OTA partition selection logic module not found")
        return
    text = logic_path.read_text(encoding="utf-8")
    required_tokens = [
        "OtaAmbiguous", "crc_expected", "is_invalid", "is_valid",
        "OTA_STATE_INVALID", "OTA_STATE_ABORTED", "BLANK_SEQ",
        "PART_SUBTYPE_OTA_FLAG", "PART_SUBTYPE_OTA_MASK",
        "is_rollback_unstable", "parse_sdkconfig_ota_flags",
        "SdkconfigFlag", "parse_sdkconfig_flag",
    ]
    for token in required_tokens:
        if token not in text:
            fail(f"{logic_path}: missing required fail-closed OTA validity primitive {token!r}")

    # Forbid the specific regressions found this session: a bare numeric
    # subtype threshold instead of the real bitmask, and a defaulted
    # rollback flag that would silently assume "safe".
    # `\.subtype` (attribute access) so this only matches real code like
    # `e.subtype >= OTA_SUBTYPE_BASE`, not this file's own docstring prose
    # quoting that exact bad pattern as an example of the bug it fixed.
    if re.search(r"\.subtype\s*>=\s*(?:OTA_SUBTYPE_BASE|0x10)", text):
        fail(f"{logic_path}: found a `subtype >= ...` threshold check - OTA slot "
             f"recognition must use the (subtype & 0xF0) == PART_SUBTYPE_OTA_FLAG bitmask, "
             f"which also excludes PART_SUBTYPE_TEST/TEE_0/TEE_1")
    if re.search(r"rollback\s*:\s*SdkconfigFlag\s*=", text) or \
       re.search(r"anti_rollback\s*:\s*SdkconfigFlag\s*=", text):
        fail(f"{logic_path}: rollback/anti_rollback must not have a default value in "
             f"resolve_application_partition() - callers must read the real sdkconfig "
             f"and pass them explicitly")

    # Session 2.3, Finding 2: a symbol absent from the sdkconfig text
    # entirely must resolve to SdkconfigFlag.UNKNOWN (not silently treated
    # as DISABLED), and resolve_application_partition() must REFUSE on
    # UNKNOWN for both symbols exactly like it does on ENABLED.
    if "SdkconfigFlag.UNKNOWN" not in text:
        fail(f"{logic_path}: missing SdkconfigFlag.UNKNOWN handling (Session 2.3 Finding 2) "
             f"- an absent sdkconfig symbol must not be conflated with an explicitly "
             f"disabled one")
    if not re.search(r"rollback\s+is\s+SdkconfigFlag\.UNKNOWN", text):
        fail(f"{logic_path}: resolve_application_partition() does not appear to REFUSE on "
             f"rollback is SdkconfigFlag.UNKNOWN (Session 2.3 Finding 2)")
    if not re.search(r"anti_rollback\s+is\s+SdkconfigFlag\.UNKNOWN", text):
        fail(f"{logic_path}: resolve_application_partition() does not appear to REFUSE on "
             f"anti_rollback is SdkconfigFlag.UNKNOWN (Session 2.3 Finding 2)")

    # Session 2.3, Finding 3: OTA slot index sets must be contiguous
    # starting at 0 - a sparse layout ({0,2}, {1}, {0,1,3}, ...) must
    # REFUSE, not silently resolve.
    if "not contiguous" not in text or "set(range(" not in text:
        fail(f"{logic_path}: missing OTA slot contiguity check (Session 2.3 Finding 3) - "
             f"ota_app_partitions() must refuse a non-contiguous OTA slot index set")

    flash_script_path = sketch_dir / "scripts" / "flash_app_only.sh"
    if flash_script_path.exists():
        flash_script_text = flash_script_path.read_text(encoding="utf-8")
        if "--sdkconfig" not in flash_script_text:
            fail(f"{flash_script_path}: must pass --sdkconfig to "
                 f"verify_application_partition.py so rollback/anti-rollback are read from "
                 f"the real build, not assumed")

    tests_path = sketch_dir / "scripts" / "tests" / "test_ota_partition_logic.py"
    if not tests_path.exists():
        fail(f"{tests_path}: OTA partition parser offline test suite not found")
        return
    result = subprocess.run([sys.executable, str(tests_path)], capture_output=True, text=True)
    if result.returncode != 0:
        fail(f"{tests_path}: OTA partition parser offline tests FAILED "
             f"(stdout={result.stdout!r} stderr={result.stderr!r})")


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

    # The BuildConfig rail-flag audit that used to live here moved to
    # check_hardware_profile_authority() in G2: the three flags are no
    # longer independently editable literals, so auditing their literal
    # value is no longer the right question. What replaced it is strictly
    # stronger - it checks that they are DERIVED from one profile authority
    # AND that the active profile is still USB_ONLY.


def check_actuator_runtime_boundaries(files):
    """I4: the Safe Actuator runtime adapter (src/actuator/ActuatorRuntime.*)
    must stay host-linkable exactly like ActuatorWritePolicy itself, and its
    mere existence must not make ordinary physical motion reachable. There is
    still no production ActuatorBackend anywhere in this firmware - ServoBus
    exposes exactly one write, safeOff() (torque OFF) - so nothing may
    construct or reference ActuatorRuntime outside src/actuator/ (the
    adapter itself) or the offline test suite."""
    names = {p.name for p, _ in files}
    for required in ("ActuatorRuntime.h", "ActuatorRuntime.cpp"):
        if required not in names:
            fail(f"{required}: Safe Actuator runtime adapter unit not found")

    for path, code in files:
        if path.name not in ("ActuatorRuntime.h", "ActuatorRuntime.cpp"):
            continue
        for token in ("#include <Arduino.h>", "Serial.", "millis(", "ServoBus"):
            if token in code:
                fail(f"{path}: contains {token!r} - keep the Safe Actuator runtime adapter "
                     f"host-linkable, the same contract as ActuatorWritePolicy itself")

    allowed_dirs = {"actuator", "tests"}
    for path, code in files:
        if path.name in ("ActuatorRuntime.h", "ActuatorRuntime.cpp"):
            continue
        if path.parent.name in allowed_dirs:
            continue
        if re.search(r"\bActuatorRuntime\b", code):
            fail(f"{path}: references ActuatorRuntime - I4's runtime adapter has no production "
                 f"backend (ServoBus exposes no torque-on/GoalPosition write) and must not be "
                 f"constructed or owned outside src/actuator/ or the offline test suite")


def check_led_status_boundaries(files):
    """LED presentation must have exactly one periodic owner (I2).
    LedStatusPolicy is the pure decision core - host-linkable, the same
    contract as network/WifiPolicy and update/OtaPolicy - and
    LedStatusManager.cpp is the only translation unit allowed to call
    LedRing::setSolid() on the periodic path. The manual @LED TEST/@LED OFF
    diagnostic stays in CommandRouter and does not call setSolid()."""
    names = {p.name for p, _ in files}
    for required in ("LedStatusPolicy.h", "LedStatusPolicy.cpp",
                      "LedStatusManager.h", "LedStatusManager.cpp"):
        if required not in names:
            fail(f"{required}: LED status manager unit not found")

    for path, code in files:
        if path.name not in ("LedStatusPolicy.h", "LedStatusPolicy.cpp"):
            continue
        for token in ("#include <Arduino.h>", "Serial.", "millis(", "LedRing"):
            if token in code:
                fail(f"{path}: contains {token!r} - keep the LED status decision core "
                     f"host-linkable and hardware-free, the same contract as "
                     f"network/WifiPolicy and update/OtaPolicy")

    allowed_setsolid_callers = {"LedStatusManager.cpp", "LedRing.cpp"}
    for path, code in files:
        if path.suffix != ".cpp" or path.name in allowed_setsolid_callers:
            continue
        if re.search(r"\bsetSolid\s*\(", code):
            fail(f"{path}: calls setSolid() directly - LED presentation has exactly one "
                 f"periodic owner (LedStatusManager); nothing else may drive the ring "
                 f"outside the manual @LED TEST/@LED OFF diagnostic path")


def check_servo_timeout_not_global(files):
    # Session 2.2, Finding C: no servo bus timeout may ever become a
    # standing global override of SCServo's own conservative default. Fails
    # if ServoBus::begin() assigns st_.IOTimeOut directly (Session 2.1's
    # pattern) instead of leaving it alone and relying on the scoped guard
    # per-transaction. Renamed ScopedPingTimeout -> ScopedIOTimeout in
    # Session 2.3 Finding 1, once it started guarding two named timeout
    # categories (kDiagnosticTimeoutMs/kOperationalTimeoutMs) instead of one.
    for path, code in files:
        if path.name != "ServoBus.cpp":
            continue
        begin_match = re.search(r"bool ServoBus::begin\(\)\s*\{(.*?)\n\}", code, re.DOTALL)
        if begin_match and re.search(r"st_\.IOTimeOut\s*=", begin_match.group(1)):
            fail(f"{path}: ServoBus::begin() assigns st_.IOTimeOut directly - this makes "
                 f"the servo bus timeout a standing global override instead of a scoped, "
                 f"per-transaction one (see ScopedIOTimeout)")
        if "ScopedIOTimeout" not in code:
            fail(f"{path}: expected ScopedIOTimeout guard usage not found")

    for path, code in files:
        if path.name != "ServoBus.h":
            continue
        if "class ScopedIOTimeout" not in code:
            fail(f"{path}: ScopedIOTimeout RAII guard not found")


def check_servo_timeout_categories_finding1(files):
    # Session 2.3, Finding 1: Session 2.2's fix still applied the single
    # diagnostic timeout to every transaction, including safeOff() and
    # readRuntimeState() - so the documented "operational timeout = 100ms,
    # diagnostic timeout = 20ms" split was not actually true in code. Both
    # are operational/safety primitives (SAFE_OFF is reachable from any
    # OperatingMode; readRuntimeState() is what a future motion controller
    # will naturally reuse) and must use kOperationalTimeoutMs, never
    # kDiagnosticTimeoutMs, absent a future explicit documented decision to
    # reunify them.
    for path, code in files:
        if path.name != "ServoBus.cpp":
            continue
        for fn_name, signature in (
            ("safeOff", r"SafeOffResult ServoBus::safeOff\(int id\)\s*\{(.*?)\n\}"),
            ("readRuntimeState", r"bool ServoBus::readRuntimeState\([^)]*\)\s*\{(.*?)\n\}"),
        ):
            m = re.search(signature, code, re.DOTALL)
            if not m:
                fail(f"{path}: {fn_name}() not found to audit its timeout category")
                continue
            body = m.group(1)
            if "kDiagnosticTimeoutMs" in body:
                fail(f"{path}: {fn_name}() uses kDiagnosticTimeoutMs - this is an "
                     f"operational/safety primitive and must use kOperationalTimeoutMs "
                     f"instead (Session 2.3 Finding 1)")
            if "kOperationalTimeoutMs" not in body:
                fail(f"{path}: {fn_name}() does not use kOperationalTimeoutMs - expected "
                     f"an explicit ScopedIOTimeout(st_, kOperationalTimeoutMs) guard "
                     f"(Session 2.3 Finding 1)")

    for path, code in files:
        if path.name != "ServoBus.h":
            continue
        if "kDiagnosticTimeoutMs" not in code or "kOperationalTimeoutMs" not in code:
            fail(f"{path}: expected both kDiagnosticTimeoutMs and kOperationalTimeoutMs "
                 f"as separately named constants (Session 2.3 Finding 1) - a single shared "
                 f"timeout constant is no longer sufficient")


def check_safe_off_verifies_readback(files):
    # Session 2.2, Finding D: SCS::Ack() (used by EnableTorque/writeByte)
    # returns 0 on failure, not -1 like Ping()/readByte()/readWord() - a
    # `result >= 0` check on it can never observe failure. safeOff() must
    # classify strictly from an independent TorqueEnable readback, not from
    # the write's own return value, and must never be a bare bool.
    for path, code in files:
        if path.name != "ServoBus.cpp":
            continue
        safe_off_match = re.search(
            r"SafeOffResult ServoBus::safeOff\(int id\)\s*\{(.*?)\n\}", code, re.DOTALL)
        if not safe_off_match:
            fail(f"{path}: ServoBus::safeOff() not found, or no longer returns SafeOffResult "
                 f"(a bare bool cannot distinguish VERIFIED_OFF from an unverifiable write)")
            continue
        body = safe_off_match.group(1)
        if re.search(r"EnableTorque\([^)]*\)\s*(?:>=|==)\s*0", body):
            fail(f"{path}: safeOff() still branches on EnableTorque()'s own return value - "
                 f"SCS::Ack() returns 0 on failure (not -1), so `>= 0` is always true and "
                 f"`== 0` would invert success/failure; classification must come from the "
                 f"TorqueEnable readback instead")
        if "SMS_STS_TORQUE_ENABLE" not in body or "readByte" not in body:
            fail(f"{path}: safeOff() must read back SMS_STS_TORQUE_ENABLE to verify the "
                 f"write, not just trust the write's own ACK")

    for path, code in files:
        if path.name != "ServoBus.h":
            continue
        for token in ("VERIFIED_OFF", "UNVERIFIED_NO_RESPONSE", "VERIFY_FAILED"):
            if token not in code:
                fail(f"{path}: SafeOffResult is missing required state {token!r}")

    for path, code in files:
        if path.name != "CommandRouter.cpp":
            continue
        if re.search(r'"OK"\s*:\s*"NO_RESPONSE"', code):
            fail(f"{path}: found the old bare OK/NO_RESPONSE SAFE_OFF reply - must print "
                 f"servo::toString(SafeOffResult) instead")


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


def check_hardware_profile_authority(files, sketch_dir):
    """G2: one profile authority -> consistent expected hardware semantics.

    V0.1 stated the bench configuration four times over (a kTestProfile
    string plus three independent `constexpr bool` literals) with nothing
    tying them together, so a partial edit could produce a firmware whose
    printed profile name contradicted its own rail expectations. This check
    enforces the replacement invariant and keeps the original protection's
    teeth: the DEFAULT profile compiled into source must remain USB_ONLY
    until G3 is authorized.
    """
    build_config = None
    profile_header = None
    for path, code in files:
        if path.name == "BuildConfig.h":
            build_config = (path, code)
        if path.name == "HardwareProfile.h":
            profile_header = (path, code)

    if profile_header is None:
        fail("config/HardwareProfile.h not found - the hardware profile authority "
             "must exist as a single mapping table (G2)")
    else:
        path, code = profile_header
        for token in ("USB_ONLY", "ROBOT_POWERED", "ProfileExpectations", "expectationsFor"):
            if token not in code:
                fail(f"{path}: missing required profile authority symbol {token!r}")
        # The mapping table itself: ROBOT_POWERED powers all three rails,
        # anything else powers none. A silent edit here would flip every
        # module's expectations at once, so its exact shape is audited.
        if not re.search(
                r"HardwareProfile::ROBOT_POWERED\)\s*\?\s*"
                r"ProfileExpectations\{true,\s*true,\s*true\}\s*:\s*"
                r"ProfileExpectations\{false,\s*false,\s*false\}", code):
            fail(f"{path}: expectationsFor() no longer maps ROBOT_POWERED -> "
                 f"(servo+battery+led all true) and USB_ONLY -> (all false); the profile "
                 f"table must not be reshaped without review")
        if "#include <Arduino.h>" in code:
            fail(f"{path}: must stay Arduino-free so the offline host tests can link the "
                 f"real profile table instead of a copy")

    if build_config is None:
        fail("config/BuildConfig.h not found")
        return

    path, code = build_config

    # (a) The three rail flags must be DERIVED, never literals again.
    for flag, field in (("kServoPowerAvailable", "servo_power_available"),
                        ("kBatteryAvailable", "battery_available"),
                        ("kLedRailPowered", "led_rail_powered")):
        m = re.search(rf"constexpr bool {flag}\s*=\s*([^;]+);", code)
        if not m:
            fail(f"{path}: could not locate {flag} to audit its derivation")
            continue
        rhs = m.group(1).strip()
        if rhs in ("true", "false"):
            fail(f"{path}: {flag} is a bare literal {rhs!r} again - it must be derived "
                 f"from kProfileExpectations so the three rail facts and the profile "
                 f"name cannot drift apart (G2)")
        elif f"kProfileExpectations.{field}" not in rhs:
            fail(f"{path}: {flag} is not derived from kProfileExpectations.{field} "
                 f"(found {rhs!r})")

    # (b) The profile name must be derived too, not typed independently.
    m = re.search(r"constexpr const char\* kTestProfile\s*=\s*([^;]+);", code)
    if not m:
        fail(f"{path}: could not locate kTestProfile")
    elif "toString(kHardwareProfile)" not in m.group(1):
        fail(f"{path}: kTestProfile must be derived via config::toString(kHardwareProfile), "
             f"not written as an independent string literal (found {m.group(1).strip()!r})")

    # (c) THE G3 GATE. This is the direct successor to Session 2's
    # "all three flags must be false" check: the source default must stay
    # USB_ONLY, so no ROBOT_POWERED image can be built by an unreviewed
    # edit. Powered hardware validation is a separately authorized gate.
    m = re.search(r"#define\s+MATDOG_ACTIVE_HARDWARE_PROFILE\s+(.+)", code)
    if not m:
        fail(f"{path}: could not locate the MATDOG_ACTIVE_HARDWARE_PROFILE default")
    elif not m.group(1).strip().endswith("HardwareProfile::USB_ONLY"):
        fail(f"{path}: the default hardware profile is {m.group(1).strip()!r}, expected "
             f"HardwareProfile::USB_ONLY - ROBOT_POWERED must not be the compiled-in "
             f"default until the G3 powered validation gate is explicitly authorized")


def check_servo_population_model(files, sketch_dir):
    """G2: canonical 17 / expected-now 13 / absent-by-design 4 stay distinct.

    The handoff is explicit that "17 servos must respond for PASS" is FALSE
    for the current robot, and that an absent-by-design servo must never be
    classified as a failure. Both are easy to regress with a one-line edit,
    so both are audited.
    """
    population = None
    for path, code in files:
        if path.name == "ServoPopulation.h":
            population = (path, code)
    if population is None:
        fail("servo/ServoPopulation.h not found - the G2 population model must exist")
        return

    path, code = population

    for token in ("kCanonicalServos", "canonical_allocated", "expected_now",
                  "PRESENT_EXPECTED", "MISSING_EXPECTED", "ABSENT_BY_DESIGN",
                  "ABSENT_BY_DESIGN_PRESENT", "UNEXPECTED_ID", "PROFILE_MISMATCH"):
        if token not in code:
            fail(f"{path}: missing required population semantic {token!r}")

    # kCanonicalServoCount must be COMPUTED from the table, never a literal
    # that could silently disagree with it.
    if not re.search(r"kCanonicalServoCount\s*=\s*[^;]*sizeof\(kCanonicalServos\)", code):
        fail(f"{path}: kCanonicalServoCount must be derived with sizeof(kCanonicalServos), "
             f"not written as a literal")

    rows = re.findall(
        r'\{\s*(\d+),\s*"(\w+)",\s*"(\w+)",\s*CurrentConfig::(\w+)\s*\}', code)
    if len(rows) != 17:
        fail(f"{path}: canonical servo table has {len(rows)} entries, expected 17 "
             f"(MATDOG canonical allocation)")
    installed = [r for r in rows if r[3] == "INSTALLED"]
    absent = [r for r in rows if r[3] == "ABSENT_BY_DESIGN"]
    if len(installed) != 13:
        fail(f"{path}: {len(installed)} servos marked INSTALLED, expected 13 for the "
             f"current physical configuration")
    absent_ids = sorted(int(r[0]) for r in absent)
    if absent_ids != [52, 53, 54, 55]:
        fail(f"{path}: ABSENT_BY_DESIGN ids are {absent_ids}, expected [52, 53, 54, 55] "
             f"(NECK_PITCH/HEAD_ROTATION/HEAD_PITCH/JAW are allocated but not installed)")

    # No "17 responders = PASS" rule anywhere in the classification or its
    # presentation.
    for p2, c2 in files:
        if p2.name not in ("ServoPopulation.cpp", "ServoCensus.cpp", "CommandRouter.cpp"):
            continue
        if re.search(r"(==|>=)\s*17\b", c2) or re.search(r"\b17\s*(==|<=)", c2):
            fail(f"{p2}: found a hardcoded comparison against 17 - a healthy census for "
                 f"the current robot is 13 present + 4 absent by design, so 17 must never "
                 f"be a PASS threshold")

    # Provenance: the embedded table must not drift from the canonical YAML.
    yaml_path = sketch_dir.parents[1] / "06_Software" / "Matdog_Core" / "config" / \
        "MATDOG_SERVO_ALLOCATION.yaml"
    if not yaml_path.exists():
        fail(f"{yaml_path}: canonical servo allocation not found - the firmware table's "
             f"provenance cannot be verified")
        return
    yaml_ids = sorted(int(m) for m in re.findall(r"^\s*bus_id:\s*(\d+)\s*$",
                                                 yaml_path.read_text(encoding="utf-8"),
                                                 re.MULTILINE))
    table_ids = sorted(int(r[0]) for r in rows)
    if yaml_ids != table_ids:
        fail(f"{path}: embedded canonical table {table_ids} disagrees with "
             f"{yaml_path.name} {yaml_ids} - the YAML is the canonical project "
             f"authority; fix the firmware table, not the YAML")

    # The full identity triple, not just the ids. The firmware carries the
    # EXPECTED physical unit so the preflight can report it; if that drifts
    # from the allocation, the report would name the wrong servo.
    yaml_text = yaml_path.read_text(encoding="utf-8")
    yaml_triples = {
        int(bus): (unit, joint)
        for unit, joint, bus in re.findall(
            r"- unit:\s*(\S+)\n\s+joint:\s*(\S+)\n\s+bus_id:\s*(\d+)", yaml_text)
    }
    for bus, joint, unit, _config in rows:
        expected = yaml_triples.get(int(bus))
        if expected is None:
            fail(f"{path}: bus id {bus} is not in {yaml_path.name}")
        elif (unit, joint) != expected:
            fail(f"{path}: bus id {bus} is ({unit}, {joint}) in the firmware table but "
                 f"{expected} in {yaml_path.name} - the YAML is the authority for the "
                 f"physical unit binding, and the preflight reports that binding as "
                 f"EXPECTED identity")


def check_g2_state_is_transport_independent(files):
    """G2 handoff sections 7/8/9: new domain logic must not live inside a
    transport. If the ONLY representation of the census were Serial.printf()
    output, the future Web UI / HostLink would have to either reimplement
    the classification or re-scan the bus to render a page - both are
    explicitly forbidden architectures.
    """
    for path, code in files:
        if path.name not in ("ServoPopulation.h", "ServoPopulation.cpp",
                             "ServoCensus.h", "ServoCensus.cpp"):
            continue
        if "Serial." in code:
            fail(f"{path}: contains Serial output - the G2 population/census layer must "
                 f"stay transport-independent so USB CDC and a future Web UI can both "
                 f"consume the same structured result")
        if "#include <Arduino.h>" in code:
            fail(f"{path}: includes <Arduino.h> directly - keep this layer host-linkable "
                 f"so the offline tests exercise the shipped logic, not a copy")

    # These carry the classification rules the host tests link against.
    for path, code in files:
        if path.name in ("Availability.h", "Availability.cpp", "SystemState.h",
                         "HardwareProfile.h"):
            if "#include <Arduino.h>" in code:
                fail(f"{path}: includes <Arduino.h> - this translation unit is linked by "
                     f"the offline host test suite and must stay Arduino-free (G2)")

    # The census result must be a plain copyable struct, not something a
    # snapshot would have to re-derive.
    for path, code in files:
        if path.name != "ServoPopulation.h":
            continue
        if "struct CensusResult" not in code:
            fail(f"{path}: CensusResult struct not found - the census must produce "
                 f"structured state, not formatted text")


def check_no_startup_servo_traffic(files):
    """No bus traffic of any kind at boot - extends the existing
    ServoBus::begin() rule to the Controller, which now owns a census
    service that must never be auto-started."""
    for path, code in files:
        if path.name != "Controller.cpp":
            continue
        begin_match = re.search(r"void Controller::begin\(\)\s*\{(.*?)\n\}", code, re.DOTALL)
        if not begin_match:
            fail(f"{path}: could not locate Controller::begin() to audit startup behaviour")
            continue
        body = begin_match.group(1)
        for forbidden in ("startScan(", "servo_census_.start(", ".ping(", "EnableTorque("):
            if forbidden in body:
                fail(f"{path}: Controller::begin() calls {forbidden!r} - boot must issue no "
                     f"servo bus traffic, torque or scan at all")


def check_no_network_to_servo_path(files):
    """V2 permanent invariant: network callback != servo command authority.

    No network subsystem exists yet and none is invented here (the G2
    handoff forbids that). This is a tripwire armed in advance: the day a
    Wi-Fi/HTTP/WebSocket handler is added, it must route through
    CommandRouter -> Controller services -> authority, never call a servo
    primitive directly.
    """
    network_markers = ("WiFi.h", "WebServer.h", "AsyncWebServer", "esp_http_server",
                       "WebSocketsServer", "ESPAsyncWebServer", "HTTPClient")
    servo_primitives = ("ServoBus", "EnableTorque(", "WritePos", "SMS_STS", "st_.")
    for path, code in files:
        if not any(marker in code for marker in network_markers):
            continue
        hits = [prim for prim in servo_primitives if prim in code]
        if hits:
            fail(f"{path}: a network transport translation unit also references servo "
                 f"primitives {hits} - the browser/network path must go through "
                 f"CommandRouter and the Controller service layer, never directly to "
                 f"ServoBus (V2 architecture, forbidden path)")


def check_calibration_boundaries(files, sketch_dir):
    """Calibration is the subsystem that will eventually move the robot, so the
    boundaries that keep it inert today are enforced rather than reviewed:

      1. the pure model stays pure - no Arduino, ServoBus, Wi-Fi or OTA;
      2. the manager owns no transport and no actuator primitive;
      3. no write path is introduced anywhere by this subsystem;
      4. SAFE_OFF stays outside it;
      5. the historical fixture cannot become runtime calibration;
      6. hardware motion stays compile-time blocked;
      7. exactly one Controller-owned CalibrationManager.
    """
    by_name = {path.name: (path, code) for path, code in files}
    cal_dir = sketch_dir / "src" / "calibration"

    # --- (1)(2) purity of the whole subsystem -----------------------------
    for name in ("CalibrationDomain.h", "CalibrationDomain.cpp",
                 "CalibrationManager.h", "CalibrationManager.cpp"):
        entry = by_name.get(name)
        if entry is None:
            fail(f"{cal_dir / name}: calibration unit not found")
            continue
        path, code = entry
        for forbidden in ("#include <Arduino.h>", "#include <WiFi.h>",
                          "#include <esp_ota_ops.h>", "ServoBus", "ServoCensus",
                          "WifiManager", "OtaManager", "HardwareSerial"):
            if forbidden in code:
                fail(f"{path}: references {forbidden!r} - the calibration layer must stay "
                     f"pure and host-linkable, and must never own a transport. Population "
                     f"evidence is an INPUT from servo/ServoCensus; a second census or a "
                     f"direct bus path is forbidden")
        if "Serial." in code:
            fail(f"{path}: contains Serial output - the calibration layer must stay "
                 f"transport-independent")

        # --- (3) no write path, anywhere in the subsystem -----------------
        for token in ("EnableTorque", "TorqueEnable", "WritePos", "SyncWrite", "RegWrite",
                      "SMS_STS", "CalibrationOfs", "unLockEprom", "LockEprom",
                      "PositionOffset", "GoalPosition"):
            if token in code:
                fail(f"{path}: references actuator/EEPROM primitive {token!r} - this phase "
                     f"implements the arbiter and the evidence model, not a write path")

    # --- (4) SAFE_OFF is not routed through calibration --------------------
    router = by_name.get("CommandRouter.cpp")
    if router is not None:
        path, code = router
        branch = re.search(
            r'upper\.startsWith\("@SERVO SAFE_OFF"\)\s*\)\s*\{(.*?)\}\s*else',
            code, re.DOTALL)
        if branch and ("calibration" in branch.group(1) or "Calibration" in branch.group(1)):
            fail(f"{path}: the @SERVO SAFE_OFF branch references calibration - a safety "
                 f"de-escalation must never be routed through a calibration session")

    for name in ("ServoBus.h", "ServoBus.cpp"):
        entry = by_name.get(name)
        if entry is None:
            continue
        path, code = entry
        if "Calibration" in code or "calibration" in code:
            fail(f"{path}: references calibration - the servo transport must not be able to "
                 f"consult a calibration session, so SAFE_OFF stays reachable in every state")

    # --- (5) the historical fixture is not runtime calibration -------------
    for path, code in files:
        if "/scripts/tests/" in str(path):
            continue
        if "lf_v25_oracle_fixture" in code or "kLfV25" in code:
            fail(f"{path}: references the LF V25 historical fixture outside the offline "
                 f"tests - it describes a physical installation that no longer exists and "
                 f"must never be compiled as runtime calibration")

    # --- (6) hardware motion stays compile-time blocked --------------------
    manager = by_name.get("CalibrationManager.h")
    if manager is None:
        fail(f"{cal_dir / 'CalibrationManager.h'}: CalibrationManager not found")
    else:
        path, code = manager
        m = re.search(r"#define\s+MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED\s+(\S+)",
                      code)
        if not m:
            fail(f"{path}: could not locate the MATDOG_CALIBRATION_HARDWARE_MOTION_AUTHORIZED "
                 f"default")
        elif m.group(1).strip() != "0":
            fail(f"{path}: hardware motion defaults to {m.group(1)!r}, expected 0 - "
                 f"MATDOG_JOINT_CALIBRATION.yaml declares "
                 f"CALIBRATION_RESET_PENDING_FULL_RECALIBRATION with "
                 f"hardware_motion_authorized: false, and unblocking it requires a real "
                 f"recalibration, not a flag flip")

    # --- q0 must not be able to default to the raw servo centre ------------
    domain = by_name.get("CalibrationDomain.cpp")
    if domain is not None:
        path, code = domain
        if re.search(r"tick\s*=\s*kServoRawCenter", code) or \
           re.search(r"tick\s*=\s*2048", code):
            fail(f"{path}: assigns the raw servo centre to a q0 tick - the raw centre is a "
                 f"servo-level fact that says nothing about joint zero. "
                 f"MATDOG_JOINT_CALIBRATION.yaml: the final value MUST BE MEASURED")

    # --- (7) exactly one manager, owned by the Controller ------------------
    owners = []
    for path, code in files:
        if "/scripts/" in str(path):
            continue
        for m in re.finditer(r"CalibrationManager\s+(\w+)\s*[;{]", code):
            owners.append((str(path), m.group(1)))
    if owners and (len(owners) != 1 or pathlib.Path(owners[0][0]).name != "Controller.h"):
        fail(f"CalibrationManager is instantiated at {owners} - exactly one instance must "
             f"exist, owned by core/Controller.h")


def check_actuator_authority(files, sketch_dir):
    """The arbiter is the single point that decides who may write actuators,
    so the properties that make it trustworthy are enforced, not reviewed:

      1. it stays host-linkable and knows nothing about hardware;
      2. SAFE_OFF is outside arbitration, in BOTH directions;
      3. there is exactly one arbiter instance, owned by the Controller;
      4. boot always lands on NONE;
      5. nothing caches a copy of the authority state.
    """
    by_name = {path.name: (path, code) for path, code in files}
    core_dir = sketch_dir / "src" / "core"

    # --- (1) host-linkable, hardware-blind -------------------------------
    for name in ("ActuatorAuthority.h", "ActuatorAuthority.cpp"):
        entry = by_name.get(name)
        if entry is None:
            fail(f"{core_dir / name}: the actuator authority arbiter was not found")
            continue
        path, code = entry
        for forbidden in ("#include <Arduino.h>", "#include <WiFi.h>",
                          "#include <esp_ota_ops.h>"):
            if forbidden in code:
                fail(f"{path}: contains {forbidden!r} - the arbiter must stay host-linkable "
                     f"so scripts/tests/test_actuator_authority.cpp drives the REAL "
                     f"decision logic")
        if "Serial." in code:
            fail(f"{path}: contains Serial output - the arbiter must stay "
                 f"transport-independent")
        # It arbitrates authority; it must not know how to use it.
        for forbidden in ("ServoBus", "EnableTorque", "WritePos", "SMS_STS", "ServoCensus"):
            if forbidden in code:
                fail(f"{path}: references {forbidden!r} - the arbiter decides WHO may "
                     f"write actuators and must never be able to write one itself")

    # --- (2) SAFE_OFF is outside arbitration, both directions -------------
    # (a) ServoBus must not be able to consult an authority even if a later
    #     change wanted it to.
    for name in ("ServoBus.h", "ServoBus.cpp"):
        entry = by_name.get(name)
        if entry is None:
            continue
        path, code = entry
        for token in ("ActuatorAuthority", "authority", "Authority"):
            if token in code:
                fail(f"{path}: references {token!r} - SAFE_OFF must stay reachable in every "
                     f"authority state, so the servo transport must not be able to consult "
                     f"the arbiter at all (permanent invariant)")

    # (b) The SAFE_OFF command branch must not gain an authority condition.
    router = by_name.get("CommandRouter.cpp")
    if router is not None:
        path, code = router
        branch = re.search(
            r'upper\.startsWith\("@SERVO SAFE_OFF"\)\s*\)\s*\{(.*?)\}\s*else',
            code, re.DOTALL)
        if not branch:
            fail(f"{path}: could not locate the @SERVO SAFE_OFF branch to audit it")
        else:
            body = branch.group(1)
            for token in ("authority", "Authority", "operating_mode"):
                if token in body:
                    fail(f"{path}: the @SERVO SAFE_OFF branch references {token!r} - a safety "
                         f"de-escalation must never be gated on authority or mode")
        printer = re.search(r"void CommandRouter::printServoSafeOff\(int id\)\s*\{(.*?)\n\}",
                            code, re.DOTALL)
        if printer and ("authority" in printer.group(1) or "Authority" in printer.group(1)):
            fail(f"{path}: printServoSafeOff() consults the authority - SAFE_OFF must not be "
                 f"arbitrated")

    # --- (3)(5) exactly one arbiter, owned by the Controller --------------
    owners = []
    for path, code in files:
        for m in re.finditer(r"ActuatorAuthorityArbiter\s+(\w+)\s*[;{]", code):
            owners.append((str(path), m.group(1)))
    # Controller.h holds the one instance; the test suite may create its own.
    real = [(p, n) for p, n in owners if "/scripts/" not in p]
    if len(real) != 1 or pathlib.Path(real[0][0]).name != "Controller.h":
        fail(f"ActuatorAuthorityArbiter is instantiated at {real} - exactly one instance "
             f"must exist, owned by core/Controller.h. A second arbiter is a second "
             f"authority model")

    # Nobody may cache the current owner: they hold a pointer and ask.
    for path, code in files:
        if path.name in ("ActuatorAuthority.h", "ActuatorAuthority.cpp", "Controller.h"):
            continue
        if "/scripts/" in str(path):
            continue
        if re.search(r"ActuatorAuthority\s+\w+_\s*(=|;)", code):
            fail(f"{path}: stores a copy of the ActuatorAuthority value - there is one "
                 f"authority state and it lives in the arbiter; consult it, do not "
                 f"remember it")

    # --- (4) boot lands on NONE ------------------------------------------
    ctl = by_name.get("Controller.cpp")
    if ctl is not None:
        path, code = ctl
        body = re.search(r"void Controller::begin\(\)\s*\{(.*?)\n\}", code, re.DOTALL)
        if not body:
            fail(f"{path}: could not locate Controller::begin() to audit authority init")
        elif not re.search(r"authority_\.reset\(", body.group(1)):
            fail(f"{path}: Controller::begin() does not reset the actuator authority - boot "
                 f"must always land on NONE and must never restore a previous authority")

    # --- OTA-B: the placeholder must be gone, not coexisting ---------------
    for path, code in files:
        for token in ("OtaStageAGate", "PERMITTED_OTA_A_NO_AUTHORITY_MODEL_YET"):
            if token in code:
                fail(f"{path}: still references the OTA-A placeholder {token!r} - OTA-B "
                     f"replaces it; both must not coexist")

    gate = by_name.get("OtaAuthorityGate.cpp")
    if gate is None:
        fail(f"{sketch_dir / 'src' / 'update' / 'OtaAuthorityGate.cpp'}: the OTA-B "
             f"authorization gate was not found")
    else:
        path, code = gate
        if "requestInhibit(" not in code:
            fail(f"{path}: the OTA gate does not take an exclusivity hold - a plain "
                 f"`authority == NONE` query cannot close the window between the check and "
                 f"a multi-second update (OTA-B TOCTOU)")
        if "request(" in code.replace("requestInhibit(", ""):
            fail(f"{path}: the OTA gate acquires an actuator OWNER - OTA does not drive "
                 f"actuators; it must inhibit, not own")

    # OTA must never appear as an actuator owner.
    auth_header = by_name.get("ActuatorAuthority.h")
    if auth_header is not None:
        path, code = auth_header
        m = re.search(r"enum class ActuatorAuthority\s*:\s*uint8_t\s*\{(.*?)\}", code,
                      re.DOTALL)
        if not m:
            fail(f"{path}: could not locate the ActuatorAuthority enum")
        else:
            body = m.group(1)
            if re.search(r"\bOTA\b|UPDATE|FIRMWARE", body):
                fail(f"{path}: the ActuatorAuthority enum contains an OTA/update owner - OTA "
                     f"is not an actuator user; it requires that nobody is one")
            for required in ("NONE", "DIAGNOSTICS", "CALIBRATION", "QC", "PROVISIONING",
                             "MOTION"):
                if required not in body:
                    fail(f"{path}: the ActuatorAuthority enum is missing {required!r}")


def check_safe_actuator_boundaries(files, sketch_dir):
    """The Safe Actuator Layer is the boundary every future actuator write must
    pass through, so the properties that make it a boundary are enforced rather
    than reviewed:

      1. the policy core stays pure - no Arduino, ServoBus, Wi-Fi, OTA, Serial;
      2. it is a DECISION, not a transport: no bus primitive lives in it;
      3. no write path is enabled anywhere under src/actuator/;
      4. SAFE_OFF stays outside it, in BOTH directions;
      5. a persistent/provisioning write is not expressible as an operation;
      6. limits are admitted on provenance, and nothing at runtime admits any;
      7. at most one policy instance, and it would be the Controller's.
    """
    by_name = {path.name: (path, code) for path, code in files}
    actuator_dir = sketch_dir / "src" / "actuator"

    policy_units = ("ActuatorWritePolicy.h", "ActuatorWritePolicy.cpp")
    for name in policy_units:
        if name not in by_name:
            fail(f"{actuator_dir / name}: the Safe Actuator Layer policy core was not found")
            return

    # --- (1)(2) pure decision core ----------------------------------------
    for name in policy_units:
        path, code = by_name[name]
        for forbidden in ("#include <Arduino.h>", "#include <WiFi.h>",
                          "#include <esp_ota_ops.h>", "#include <SCServo.h>",
                          "ServoBus", "ServoCensus", "WifiManager", "OtaManager",
                          "HardwareSerial", "SMS_STS"):
            if forbidden in code:
                fail(f"{path}: references {forbidden!r} - the Safe Actuator Layer policy "
                     f"decides whether a write may happen and must never be able to perform "
                     f"one. scripts/tests/test_actuator_write_policy.cpp links the REAL "
                     f"policy, which is only possible while it stays host-linkable")
        if "Serial." in code:
            fail(f"{path}: contains Serial output - the policy must stay "
                 f"transport-independent")
        for token in ("EnableTorque", "WritePos", "SyncWrite", "RegWrite", "readByte(",
                      "readWord(", "Ping("):
            if token in code:
                fail(f"{path}: references bus primitive {token!r} - an ACCEPT from this "
                     f"policy is a decision, never an action")

    # --- (3) no write path is enabled anywhere under src/actuator/ ---------
    # The runtime adapter is TO_IMPLEMENT (SAFE_ACTUATOR_LAYER.md §7): building
    # one would require weakening check_torque_enable and the goal-position
    # prohibitions in check_forbidden_literals BEFORE anything could validate
    # the replacement. Enabling a write path here is a reviewed decision that
    # belongs to a later gate, not a quiet addition.
    for path, code in files:
        if f"{os.sep}src{os.sep}actuator{os.sep}" not in str(path):
            continue
        for token in ("EnableTorque", "WritePos", "SyncWrite", "RegWrite", "writeByte",
                      "writeWord"):
            if token in code:
                fail(f"{path}: contains actuator write primitive {token!r} - the Safe "
                     f"Actuator Layer runtime adapter is TO_IMPLEMENT and no write path "
                     f"may be enabled in the default build")

    # --- (4) SAFE_OFF is outside this layer, both directions ---------------
    layer_tokens = ("SafeActuatorPolicy", "ActuatorWritePolicy", "WriteDecision",
                    "ActuatorOperation", "ActuatorTransaction", "actuator::")
    for name in ("ServoBus.h", "ServoBus.cpp"):
        entry = by_name.get(name)
        if entry is None:
            continue
        path, code = entry
        for token in layer_tokens:
            if token in code:
                fail(f"{path}: references {token!r} - SAFE_OFF must stay reachable with no "
                     f"authority, stale authority, a failed calibration manager or a "
                     f"rejecting policy, so the servo transport must not be able to consult "
                     f"the Safe Actuator Layer at all (permanent invariant)")

    router = by_name.get("CommandRouter.cpp")
    if router is not None:
        path, code = router
        branch = re.search(
            r'upper\.startsWith\("@SERVO SAFE_OFF"\)\s*\)\s*\{(.*?)\}\s*else',
            code, re.DOTALL)
        if not branch:
            fail(f"{path}: could not locate the @SERVO SAFE_OFF branch to audit it")
        else:
            for token in layer_tokens:
                if token in branch.group(1):
                    fail(f"{path}: the @SERVO SAFE_OFF branch references {token!r} - a "
                         f"safety de-escalation must never be routed through the write "
                         f"policy, which can reject")

    # --- (5) the operation classes ----------------------------------------
    path, code = by_name["ActuatorWritePolicy.h"]
    m = re.search(r"enum class ActuatorOperation\s*:\s*uint8_t\s*\{(.*?)\}", code, re.DOTALL)
    if not m:
        fail(f"{path}: could not locate the ActuatorOperation enum")
    else:
        body = m.group(1)
        # A persistent/configuration write must not be expressible here while
        # the repository still records its owner as TO_DESIGN
        # (CALIBRATION_SOURCE_PRECEDENCE.md §7).
        for banned in ("EEPROM", "ID_WRITE", "OFFSET", "PERSIST", "PROVISION", "LOCK",
                       "RESET"):
            if banned in body:
                fail(f"{path}: the ActuatorOperation enum contains {banned!r} - a "
                     f"persistent/provisioning write must stay inexpressible through this "
                     f"layer: runtime actuator command != persistent configuration write")
        # And no class may name a torque-removal: SAFE_OFF must not become
        # something this policy can be asked to gate.
        for banned in ("SAFE_OFF", "TORQUE_OFF", "DISABLE", "TORQUE_DISABLE"):
            if banned in body:
                fail(f"{path}: the ActuatorOperation enum contains {banned!r} - removing "
                     f"torque is SAFE_OFF's job and must stay outside this layer, so it "
                     f"must not be expressible as a policy operation")
        for required in ("NONE", "TORQUE_ENABLE", "POSITION_COMMAND",
                         "CALIBRATION_CONTACT_PROBE"):
            if required not in body:
                fail(f"{path}: the ActuatorOperation enum is missing {required!r}")
        members = len(re.findall(r"^\s*([A-Z_]+)\s*=", body, re.M))
        count = re.search(r"kActuatorOperationCount\s*=\s*(\d+)", code)
        if not count:
            fail(f"{path}: could not locate kActuatorOperationCount")
        elif int(count.group(1)) != members:
            fail(f"{path}: kActuatorOperationCount is {count.group(1)} but the enum has "
                 f"{members} members - isKnownOperation() would then accept a value with no "
                 f"meaning, or reject one with meaning. Fail closed means these agree")

    # --- (6) limits are admitted on provenance, and never at runtime -------
    path, code = by_name["ActuatorWritePolicy.cpp"]
    provenance = re.search(r"bool JointLimit::usableProvenance\(\) const\s*\{(.*?)\n\}",
                           code, re.DOTALL)
    if not provenance:
        fail(f"{path}: could not locate JointLimit::usableProvenance() to audit it")
    else:
        body = provenance.group(1)
        for required in ("mayPromote", "isOperationalEvidence"):
            if required not in body:
                fail(f"{path}: usableProvenance() does not consult calibration::{required} "
                     f"- a limit may only come from operational calibration measured on the "
                     f"current installation. Historical LF V25 values are fixtures, never "
                     f"live safety bounds")
    if "HISTORICAL_REPLAY" in code:
        fail(f"{path}: names CalibrationOrigin::HISTORICAL_REPLAY - the policy must ask "
             f"calibration::mayPromote() rather than special-case a replay, so a new "
             f"non-promotable origin cannot silently become admissible")

    # Nothing in the runtime may fill the accepted-limit store. Today nothing
    # in the repository qualifies to: MATDOG_JOINT_CALIBRATION.yaml records
    # {min: null, max: null} for all twelve leg joints.
    for path, code in files:
        if path.name in policy_units or "/scripts/" in str(path).replace(os.sep, "/"):
            continue
        if re.search(r"\blimits\(\)\.admit\(|\bActuatorLimitTable\b", code):
            fail(f"{path}: populates or holds an ActuatorLimitTable - accepted joint bounds "
                 f"do not exist yet, and a runtime translation unit must not be the thing "
                 f"that invents them")

    # --- (7) at most one policy instance, and it would be the Controller's --
    owners = []
    for path, code in files:
        if "/scripts/" in str(path).replace(os.sep, "/"):
            continue
        for m in re.finditer(r"SafeActuatorPolicy\s+(\w+)\s*[;{]", code):
            owners.append((str(path), m.group(1)))
    if len(owners) > 1 or (owners and pathlib.Path(owners[0][0]).name != "Controller.h"):
        fail(f"SafeActuatorPolicy is instantiated at {owners} - at most one instance may "
             f"exist and it belongs to core/Controller.h, next to the one arbiter. A second "
             f"policy is a second write boundary")


def check_calibration_geometry_boundaries(files, sketch_dir):
    """The calibration bootstrap consumes a geometry plan it did not compute, so
    the properties that keep the plan trustworthy are enforced rather than
    reviewed:

      1. the profile stays pure - no Arduino, no bus, no mesh, no Serial;
      2. the generated table is generated, and still matches its bundle;
      3. the executability door needs BOTH the URDF domain and a PASS clearance;
      4. the compiled counts are the canonical bundle's, and no UNRESOLVED
         verdict sits on an executable endpoint;
      5. the superseded hardcoded prerequisite poses are absent;
      6. POSITION_COMMAND keeps the accepted-limits route to itself.
    """
    by_name = {path.name: (path, code) for path, code in files}
    actuator_dir = sketch_dir / "src" / "actuator"

    units = ("CalibrationGeometryProfile.h", "CalibrationGeometryProfile.cpp",
             "CalibrationGeometryProfileData.h")
    for name in units:
        if name not in by_name:
            fail(f"{actuator_dir / name}: the calibration geometry profile was not found")
            return

    # --- (1) the profile is data plus lookups, never a geometry engine -----
    for name in units:
        path, code = by_name[name]
        for forbidden in ("#include <Arduino.h>", "#include <WiFi.h>", "#include <math.h>",
                          "#include <cmath>", "ServoBus", "ServoCensus", "SMS_STS",
                          "HardwareSerial", "WifiManager", "OtaManager"):
            if forbidden in code:
                fail(f"{path}: references {forbidden!r} - the Controller consumes a "
                     f"prevalidated geometry plan and must never recompute geometry or "
                     f"reach a bus to act on one")
        if "Serial." in code:
            fail(f"{path}: contains Serial output - the geometry profile must stay "
                 f"transport-independent")
        if "float" in code or "double" in code:
            fail(f"{path}: uses floating point - every geometric bound here is an integer "
                 f"micro-radian so that a safety comparison cannot depend on rounding")

    # --- (2) the generated table is generated ------------------------------
    data_path, data_code = by_name["CalibrationGeometryProfileData.h"]
    raw = data_path.read_text(encoding="utf-8")
    for marker in ("GENERATED FILE - DO NOT EDIT BY HAND",
                   "matdog_calibration_geometry_export.py",
                   "MATDOG_GEOMETRY_V5_REMEDIATION_BENCHMARK_D_W4"):
        if marker not in raw:
            fail(f"{data_path}: missing the generated-file marker {marker!r} - this table "
                 f"produced from the canonical Geometry Compiler V5 bundle and must never "
                 f"be hand-written")

    # --- (3) the one door --------------------------------------------------
    path, code = by_name["CalibrationGeometryProfile.cpp"]
    body = re.search(r"bool isExecutable\(const GeometryEndpointRecord& endpoint\)\s*\{(.*?)\n\}",
                     code, re.DOTALL)
    if not body:
        fail(f"{path}: could not locate isExecutable() to audit it")
    else:
        text = body.group(1)
        if "EXECUTABLE_URDF_DOMAIN" not in text:
            fail(f"{path}: isExecutable() does not check the target domain - sixteen of the "
                 f"twenty-four canonical endpoints contact BEYOND the declared URDF limit, "
                 f"and a diagnostic endpoint is evidence, never a motion target")
        if "ClearancePolicyResult::PASS" not in text:
            fail(f"{path}: isExecutable() does not require a PASS clearance verdict - "
                 f"UNRESOLVED is not PASS")

    # --- (4)(5) the compiled data itself -----------------------------------
    records = re.findall(r"\{calibration::Leg::(\w+), calibration::JointKind::(\w+), "
                         r"calibration::ContactSide::(\w+), TargetDomain::(\w+), "
                         r"ParkingOutcome::(\w+), ClearancePolicyResult::(\w+), "
                         r"(-?\d+), (-?\d+), (-?\d+), (\w+), calibration::Leg::(\w+), "
                         r"calibration::JointKind::(\w+), (-?\d+)\}", data_code)
    if len(records) != 24:
        fail(f"{data_path}: parsed {len(records)} endpoint records, expected the canonical 24")
    executable = [r for r in records if r[3] == "EXECUTABLE_URDF_DOMAIN"]
    parking = [r for r in records if r[4] == "FEASIBLE_1DOF_PLAN_FOUND"]
    if len(executable) != 8:
        fail(f"{data_path}: {len(executable)} executable endpoints, expected 8")
    if len(parking) != 6:
        fail(f"{data_path}: {len(parking)} endpoints needing parking, expected 6")
    for record in records:
        if record[5] != "PASS" and record[3] == "EXECUTABLE_URDF_DOMAIN":
            fail(f"{data_path}: an EXECUTABLE endpoint carries clearance verdict "
                 f"{record[5]!r} - "
                 f"no executable target may rest on unresolved clearance evidence")
        if record[4] == "NOT_NEEDED" and record[9] != "false":
            fail(f"{data_path}: a NOT_NEEDED plan carries an auxiliary joint")
        if record[4] == "FEASIBLE_1DOF_PLAN_FOUND" and record[9] != "true":
            fail(f"{data_path}: an obstructed plan carries no auxiliary joint - a missing "
                 f"parking plan must fail closed, not silently become a direct path")
    # 30, 50, 85 and 90 degrees: the superseded hardcoded prerequisites. The
    # current compiler found 35, 64.1667 and 93.3333 instead, and its own source
    # notes that the empty default context proves the legacy poses were never
    # core truth.
    for record in records:
        if record[9] != "true":
            continue
        target = int(record[12])
        for legacy in (523599, 872665, 1483530, 1570796):
            if abs(target - legacy) <= 1000:
                fail(f"{data_path}: auxiliary target {target} micro-rad matches the "
                     f"superseded "
                     f"hardcoded prerequisite {legacy} - parking poses come from the current "
                     f"Geometry V5 plan, never from the stale calibration block")

    # --- (6) POSITION_COMMAND keeps its own route --------------------------
    policy = by_name.get("ActuatorWritePolicy.cpp")
    if policy is not None:
        path, code = policy
        route = re.search(r"bool operationUsesAcceptedLimits\(ActuatorOperation operation\)"
                          r"\s*\{(.*?)\n\}", code, re.DOTALL)
        if not route:
            fail(f"{path}: could not locate operationUsesAcceptedLimits() to audit it")
        else:
            text = route.group(1)
            if "POSITION_COMMAND" not in text:
                fail(f"{path}: POSITION_COMMAND no longer takes the accepted-limits route")
            for other in ("CALIBRATION_CONTACT_PROBE", "DIRECTION_VERIFY",
                          "CALIBRATION_AUXILIARY_MOVE", "TORQUE_ENABLE"):
                if other in text:
                    fail(f"{path}: {other!r} was added to the accepted-limits route - the "
                         f"three authorisation routes must stay mutually exclusive")
        envelope = re.search(r"bool operationUsesBootstrapEnvelope\(ActuatorOperation "
                             r"operation\)\s*\{(.*?)\n\}", code, re.DOTALL)
        if envelope and "POSITION_COMMAND" in envelope.group(1):
            fail(f"{path}: POSITION_COMMAND can reach the bootstrap envelope - a geometry "
                 f"plan describes the MODEL and must never stand in for an accepted bound "
                 f"on the current machine")
        plan_route = re.search(r"bool operationUsesEndpointPlan\(ActuatorOperation "
                               r"operation\)\s*\{(.*?)\n\}", code, re.DOTALL)
        if plan_route and "POSITION_COMMAND" in plan_route.group(1):
            fail(f"{path}: POSITION_COMMAND can reach the endpoint-plan route")


def check_calibration_geometry_export(sketch_dir):
    """The generated profile must still be what the exporter produces from the
    canonical bundle. Catches a hand-patched table, a drifted URDF or mesh, and
    a compiler source edit - the exporter re-verifies every input hash."""
    repo_root = sketch_dir.parents[1]
    exporter = (repo_root / "06_Software/Matdog_Core/calibration/"
                            "matdog_calibration_geometry_export.py")
    if not exporter.exists():
        fail(f"{exporter}: the calibration geometry exporter was not found")
        return
    result = subprocess.run([sys.executable, str(exporter), "--check"],
                            capture_output=True, text=True)
    if result.returncode != 0:
        fail(f"{exporter}: the committed geometry profile does not match the canonical "
             f"Geometry Compiler V5 bundle (stdout={result.stdout!r} "
             f"stderr={result.stderr!r})")


def check_position_offset_boundary(files, sketch_dir):
    """PositionOffset: exactly one approved read accessor, and never a write.

    The old rule was "the register symbol must not appear at all". That was
    strong but too blunt: it also forbade READING the offset, so the firmware
    could not verify that every unit still holds PositionOffset = 0 - the one
    provisioning fact the 2026-08-27 reset document cares most about.

    The rule is now:

        read through exactly ServoBus::readPositionOffset()  = ALLOWED
        any PositionOffset write                             = FORBIDDEN
        CalibrationOfs                                       = FORBIDDEN

    Writing an offset to compensate a mechanical mounting error is named in
    MATDOG_JOINT_CALIBRATION.yaml's `forbidden:` list. Reading it is how we
    prove nobody did.
    """
    by_name = {path.name: (path, code) for path, code in files}

    accessor = "readPositionOffset"
    entry = by_name.get("ServoBus.cpp")
    if entry is None:
        fail(f"{sketch_dir / 'src' / 'servo' / 'ServoBus.cpp'}: not found")
        return
    bus_path, bus_code = entry

    # --- the symbol appears in exactly one file, and one function ----------
    for path, code in files:
        if "SMS_STS_OFS_L" not in code:
            continue
        if path.name != "ServoBus.cpp":
            fail(f"{path}: names SMS_STS_OFS_L - the PositionOffset register may only be "
                 f"touched by the single approved read accessor "
                 f"ServoBus::{accessor}(), so that every access to it is in one "
                 f"auditable place")

    body = re.search(r"bool ServoBus::" + accessor + r"\(int id, int16_t\* out\)\s*\{(.*?)\n\}",
                     bus_code, re.DOTALL)
    if not body:
        fail(f"{bus_path}: the approved accessor ServoBus::{accessor}(int, int16_t*) was "
             f"not found - PositionOffset must be reachable through exactly one function")
        return
    accessor_body = body.group(1)

    if "SMS_STS_OFS_L" not in accessor_body:
        fail(f"{bus_path}: {accessor}() does not name SMS_STS_OFS_L - if the register is "
             f"reached some other way (a raw 0x1F, an alias) the audit can no longer see "
             f"every access, which is the entire point of having one accessor")

    # --- it must be a READ, and only a read --------------------------------
    if not re.search(r"\breadWord\s*\(", accessor_body):
        fail(f"{bus_path}: {accessor}() does not use readWord() - PositionOffset is a "
             f"read-only accessor")
    for token in ("writeByte", "writeWord", "genWrite", "regWrite", "RegWrite",
                  "EnableTorque", "WritePos", "SyncWrite", "Action(", "unLockEprom",
                  "LockEprom"):
        if token in accessor_body:
            fail(f"{bus_path}: {accessor}() contains {token!r} - the PositionOffset "
                 f"accessor is READ-ONLY and no write primitive may appear in it")

    # --- no PositionOffset write anywhere, under any spelling --------------
    offset_write = re.compile(
        r"(writeByte|writeWord|genWrite|regWrite|RegWrite)\s*\([^;]*?"
        r"(SMS_STS_OFS_L|SMS_STS_OFS|0x1F|POSITION_OFFSET|PositionOffset)")
    for path, code in files:
        if offset_write.search(code):
            fail(f"{path}: a PositionOffset WRITE is expressible here - writing an offset "
                 f"to compensate mechanical mounting error is named in "
                 f"MATDOG_JOINT_CALIBRATION.yaml's forbidden: list and must stay "
                 f"unreachable")
        # 0x1F as a bare register address is how the accessor rule gets evaded.
        if path.name != "ServoBus.cpp" and re.search(r"(readByte|readWord)\s*\([^;]*0x1F", code):
            fail(f"{path}: reads register 0x1F directly - PositionOffset must go through "
                 f"ServoBus::{accessor}(), not a magic address that the audit cannot "
                 f"attribute")

    # --- the offset decoder must be two's complement -----------------------
    profile = by_name.get("ServoProfile.cpp")
    if profile is None:
        fail(f"{sketch_dir / 'src' / 'servo' / 'ServoProfile.cpp'}: not found")
    else:
        path, code = profile
        decoder = re.search(r"int16_t decodePositionOffset\(uint16_t raw\)\s*\{(.*?)\n\}",
                            code, re.DOTALL)
        if not decoder:
            fail(f"{path}: decodePositionOffset() not found")
        elif "static_cast<int16_t>" not in decoder.group(1):
            fail(f"{path}: decodePositionOffset() is not a two's-complement cast - the "
                 f"C018 stores the offset as int16 LE two's complement, and a "
                 f"sign-magnitude decode turns a negative offset into a positive one")


def check_servo_profile_contract(files, sketch_dir):
    """MATDOG_C018_V1 stays a generated, read-only contract.

      1. the register table is generated from the reviewed YAML, not retyped;
      2. the exporter still reproduces the committed header exactly;
      3. runtime RAM state never leaks into the persistent profile;
      4. nothing in the profile path can write.
    """
    by_name = {path.name: (path, code) for path, code in files}
    servo_dir = sketch_dir / "src" / "servo"

    for name in ("ServoProfile.h", "ServoProfile.cpp", "ServoProfileData.h"):
        if name not in by_name:
            fail(f"{servo_dir / name}: the MATDOG_C018_V1 profile contract was not found")
            return

    data_path, data_code = by_name["ServoProfileData.h"]
    raw = data_path.read_text(encoding="utf-8")
    for marker in ("GENERATED FILE - DO NOT EDIT BY HAND",
                   "matdog_servo_profile_export.py",
                   "MATDOG_ST3215_C018_V1.yaml"):
        if marker not in raw:
            fail(f"{data_path}: missing the generated-file marker {marker!r} - the twenty "
                 f"register values come from the reviewed YAML and are never hand-written")

    rows = re.findall(r"\{0x([0-9A-F]{2}), (\d), (\d+), \"(\w+)\"\}", data_code)
    if len(rows) != 20:
        fail(f"{data_path}: {len(rows)} persistent registers, expected the canonical 20")
    # Runtime RAM state is a different layer and must not appear as persistent.
    for _addr, _w, _v, name in rows:
        if name in ("TorqueLimit", "GoalSpeed", "Acc", "GoalPosition", "Acceleration"):
            fail(f"{data_path}: {name!r} is runtime RAM state and must never be part of "
                 f"the persistent profile - TorqueLimit/GoalSpeed/Acc are written per "
                 f"motion, not provisioned")

    for name in ("ServoProfile.h", "ServoProfile.cpp"):
        path, code = by_name[name]
        for forbidden in ("#include <Arduino.h>", "SMS_STS", "ServoBus", "Serial.",
                          "writeByte", "writeWord", "EnableTorque", "WritePos"):
            if forbidden in code:
                fail(f"{path}: references {forbidden!r} - the profile contract is pure "
                     f"data plus comparisons and must never reach a bus or write")

    # A missing read must never fold into a MATCH.
    path, code = by_name["ServoProfile.cpp"]
    fold = re.search(r"ProfileVerdict foldRegisterCheck\(.*?\n\}", code, re.DOTALL)
    if not fold:
        fail(f"{path}: foldRegisterCheck() not found")
    elif not re.search(r"case RegisterCheck::NO_ANSWER:\s*case RegisterCheck::NOT_READ:"
                       r"\s*return ProfileVerdict::INCOMPLETE;", fold.group(0)):
        fail(f"{path}: foldRegisterCheck() does not map NO_ANSWER/NOT_READ to INCOMPLETE - "
             f"an unread register would then be assumed to hold its expected value, and a "
             f"partially read unit could report MATCH")


def check_servo_profile_export(sketch_dir):
    """The committed profile table must still be what the exporter produces."""
    repo_root = sketch_dir.parents[1]
    exporter = repo_root / "06_Software/Matdog_Core/config/matdog_servo_profile_export.py"
    if not exporter.exists():
        fail(f"{exporter}: the MATDOG_C018_V1 profile exporter was not found")
        return
    result = subprocess.run([sys.executable, str(exporter), "--check"],
                            capture_output=True, text=True)
    if result.returncode != 0:
        fail(f"{exporter}: the committed profile table does not match "
             f"MATDOG_ST3215_C018_V1.yaml (stdout={result.stdout!r} "
             f"stderr={result.stderr!r})")


def check_h0_preflight_boundaries(files, sketch_dir):
    """The H0 preflight is a permanent READ-ONLY capability.

      1. no write primitive anywhere in the service;
      2. MAINTENANCE-gated at the command surface, like the census;
      3. expected_physical_unit is configuration and is never called observed;
      4. present_position is never presented as q0.
    """
    by_name = {path.name: (path, code) for path, code in files}
    servo_dir = sketch_dir / "src" / "servo"

    for name in ("ServoPreflight.h", "ServoPreflight.cpp"):
        if name not in by_name:
            fail(f"{servo_dir / name}: the H0 preflight service was not found")
            return
        path, code = by_name[name]
        for token in ("EnableTorque", "WritePos", "SyncWrite", "RegWrite", "writeByte",
                      "writeWord", "unLockEprom", "LockEprom", "Serial."):
            if token in code:
                fail(f"{path}: contains {token!r} - the preflight is strictly read-only "
                     f"and transport-independent")

    router = by_name.get("CommandRouter.cpp")
    if router is not None:
        path, code = router
        branch = re.search(r'upper == "@SERVO PREFLIGHT"\s*\)\s*\{(.*?)\}\s*else',
                           code, re.DOTALL)
        if not branch:
            fail(f"{path}: could not locate the @SERVO PREFLIGHT branch to audit it")
        elif "NOT_IN_MAINTENANCE_MODE" not in branch.group(1):
            fail(f"{path}: @SERVO PREFLIGHT is not MAINTENANCE-gated - it carries the same "
                 f"bounded per-tick blocking as the census and must take the same gate")

        printer = re.search(r"void CommandRouter::printServoPreflightResult\(\)\s*\{(.*?)\n\}",
                            code, re.DOTALL)
        if not printer:
            fail(f"{path}: printServoPreflightResult() not found")
        else:
            text = printer.group(1)
            if "expected_physical_unit" not in text:
                fail(f"{path}: the preflight report does not label the unit column as "
                     f"EXPECTED - a servo cannot report its unit label, so it must never "
                     f"be presented as observed hardware identity")
            if re.search(r"observed_physical_unit|physical_unit_observed", text):
                fail(f"{path}: the preflight report claims an OBSERVED physical unit - an "
                     f"ST3215 exposes no unit serial; that binding is held by labelling "
                     f"discipline, not by measurement")
            if "NOT q0" not in text:
                fail(f"{path}: the preflight report does not state that present_position "
                     f"is NOT q0 - a raw liveness tick must never be read as a "
                     f"calibration pose")


def check_evidence_geometry_binding(files, sketch_dir):
    """B1: calibration evidence is bound to the geometry it was measured under.

    A servo swap invalidates a JOINT; a URDF/mesh/profile change invalidates a
    MODEL. Those are different axes and both must bite:

      1. JointLimit and JointTransform each carry a geometry tag;
      2. admit() refuses a record that does not name its geometry;
      3. find() is geometry-scoped, and an unbound tag matches nothing;
      4. findAny() exists so a superseded record stays ON RECORD without ever
         becoming current evidence;
      5. the policy's current tag requires the provenance this build expects,
         not merely "some geometry is loaded".
    """
    by_name = {path.name: (path, code) for path, code in files}
    actuator_dir = sketch_dir / "src" / "actuator"

    policy_h = by_name.get("ActuatorWritePolicy.h")
    policy_cpp = by_name.get("ActuatorWritePolicy.cpp")
    profile_h = by_name.get("CalibrationGeometryProfile.h")
    if policy_h is None or policy_cpp is None or profile_h is None:
        fail(f"{actuator_dir}: the Safe Actuator Layer sources were not found")
        return

    # --- (1) both records carry the tag ------------------------------------
    for holder, name in ((policy_h, "JointLimit"), (profile_h, "JointTransform")):
        path, code = holder
        body = re.search(r"struct " + name + r"\s*\{(.*?)\n\};", code, re.DOTALL)
        if not body:
            fail(f"{path}: struct {name} not found")
            continue
        if "GeometryProvenanceTag geometry" not in body.group(1):
            fail(f"{path}: {name} carries no GeometryProvenanceTag - a record that cannot "
                 f"name the model it was measured under could never be invalidated when "
                 f"that model changes")
        if "boundToGeometry" not in code:
            fail(f"{path}: {name} has no boundToGeometry() - the geometry axis must stay "
                 f"separate from calibration provenance and from physical-unit identity")

    # --- (2)(3)(4) the tables ----------------------------------------------
    path, code = policy_cpp
    for table, record in (("ActuatorLimitTable", "limit"), ("JointTransformTable", "transform")):
        admit = re.search(r"bool " + table + r"::admit\(.*?\n\}", code, re.DOTALL)
        if not admit:
            fail(f"{path}: {table}::admit() not found")
        elif f"{record}.boundToGeometry()" not in admit.group(0):
            fail(f"{path}: {table}::admit() does not require boundToGeometry() - a record "
                 f"with no geometry would be stored and could never be invalidated")

        find = re.search(r"::find\(const JointIdentity& joint,\s*"
                         r"GeometryProvenanceTag geometry\) const\s*\{(.*?)\n\}",
                         code, re.DOTALL)
        if not find:
            fail(f"{path}: {table}::find() is not geometry-scoped - the decision path must "
                 f"not be able to look evidence up by identity alone")
    # findAny() exists to explain a refusal, never to produce the pointer a
    # decision is made from. Assigning from it is exactly how the geometry
    # scope gets bypassed while still looking careful.
    for m in re.finditer(r"=\s*(limits_|transforms_)\.findAny\(", code):
        fail(f"{path}: authorising evidence is assigned from {m.group(1)}findAny() - "
             f"findAny() ignores the geometry tag and may only be used as a predicate "
             f"when explaining REJECT_EVIDENCE_GEOMETRY_MISMATCH")

    if "findAny" not in code:
        fail(f"{path}: findAny() is gone - a superseded record must stay historically "
             f"registered even though it is never current evidence")
    # An unbound tag must match nothing, in both tables.
    if code.count("if (geometry == kNoGeometryProvenance) return nullptr;") < 2:
        fail(f"{path}: a geometry-scoped find() does not fail closed on "
             f"kNoGeometryProvenance - an unbound policy would then match stored evidence")

    # --- (5) the current tag is the EXPECTED model's ------------------------
    tag = re.search(r"GeometryProvenanceTag SafeActuatorPolicy::currentGeometryTag\(\) const"
                    r"\s*\{(.*?)\n\}", code, re.DOTALL)
    if not tag:
        fail(f"{path}: SafeActuatorPolicy::currentGeometryTag() not found")
    else:
        text = tag.group(1)
        if "provenanceMatches" not in text:
            fail(f"{path}: currentGeometryTag() does not check provenanceMatches() - "
                 f"'some geometry is loaded' is not 'the geometry this build expects'")
        if "kNoGeometryProvenance" not in text:
            fail(f"{path}: currentGeometryTag() cannot return kNoGeometryProvenance - an "
                 f"unbound or mismatched profile must match no stored evidence")

    # The decision path must never look evidence up without the tag.
    # One level of nesting, so currentGeometryTag()'s own parentheses do not
    # truncate the captured argument list.
    for call in re.finditer(
            r"(limits_|transforms_)\.find\(((?:[^()]|\([^()]*\))*)\)", code):
        if "currentGeometryTag()" not in call.group(2):
            fail(f"{path}: {call.group(0)} looks evidence up without the current geometry "
                 f"tag - identity alone is not enough to make a record current")


def check_direction_is_contractual(files, sketch_dir):
    """Joint direction is hardware-contract data, not a recalibration datum.

    The canonical URDF carries per-joint motorDirection and those directions
    were validated on real hardware. The 2026-08-27 reprovisioning changed the
    physical units, the PositionOffset baseline and the raw q0 installation -
    it did NOT change the servo model, the mounting orientation, the joint
    mechanical architecture, the URDF axes or motorDirection.

        q0              CURRENT INSTALLATION CALIBRATION DATA - measured
        motorDirection  CURRENT URDF / HARDWARE CONTRACT DATA - read

    So:

      1. JointTransform carries NO direction field - one source of truth;
      2. jointDirection() resolves it from the bound profile's URDF record;
      3. usableProvenance() does not require a measured direction;
      4. the optional DIRECTION_VERIFY diagnostic budget is consulted ONLY by
         the diagnostic path - never by calibration acceptance.
    """
    by_name = {path.name: (path, code) for path, code in files}
    profile_h = by_name.get("CalibrationGeometryProfile.h")
    profile_cpp = by_name.get("CalibrationGeometryProfile.cpp")
    policy_cpp = by_name.get("ActuatorWritePolicy.cpp")
    if profile_h is None or profile_cpp is None or policy_cpp is None:
        fail(f"{sketch_dir / 'src' / 'actuator'}: the Safe Actuator sources were not found")
        return

    # --- (1) direction is not stored as transform evidence -----------------
    path, code = profile_h
    body = re.search(r"struct JointTransform\s*\{(.*?)\n\};", code, re.DOTALL)
    if not body:
        fail(f"{path}: struct JointTransform not found")
    elif re.search(r"^\s*int8_t\s+direction\s*=", body.group(1), re.M):
        fail(f"{path}: JointTransform carries a `direction` field - direction is "
             f"hardware-contract data read from the URDF, not measured evidence. Storing a "
             f"copy creates a second source of truth that can silently disagree with the "
             f"URDF the geometry plan was compiled against")

    # --- (2) it is resolved from the profile -------------------------------
    path, code = profile_cpp
    resolver = re.search(r"int8_t jointDirection\(.*?\n\}", code, re.DOTALL)
    if not resolver:
        fail(f"{path}: jointDirection() not found - direction must be resolvable from the "
             f"bound profile, which is what ties it to the URDF provenance")
    else:
        text = resolver.group(0)
        if "urdf_motor_direction" not in text:
            fail(f"{path}: jointDirection() does not read urdf_motor_direction - the URDF "
                 f"is the only authority for a joint's direction")
        if "findJoint" not in text:
            fail(f"{path}: jointDirection() does not go through the bound profile - a "
                 f"direction read outside the profile escapes the geometry provenance tag, "
                 f"so a URDF change would not invalidate it")
        if "return 0" not in text:
            fail(f"{path}: jointDirection() cannot return 0 - an unknown joint or an "
                 f"unbound profile must fail closed rather than guess a sign")

    # --- (3) provenance does not demand a measured direction ---------------
    provenance = re.search(r"bool JointTransform::usableProvenance\(\) const\s*\{(.*?)\n\}",
                           code, re.DOTALL)
    if provenance and "direction" in provenance.group(1):
        fail(f"{path}: JointTransform::usableProvenance() still consults a direction - a "
             f"same-type servo replacement in the same mounting invalidates q0 only, and "
             f"must not be blocked waiting for a direction measurement")

    # --- (4) the diagnostic budget never gates calibration -----------------
    path, code = policy_cpp
    for m in re.finditer(r"(\w+)\s*\([^)]*\)\s*(?:const\s*)?\{", code):
        pass  # function boundaries are not reliable here; scope by name instead
    envelope = re.search(r"WriteDecision SafeActuatorPolicy::evaluateBootstrapEnvelope"
                         r"\(.*?\n\}", code, re.DOTALL)
    plan_route = re.search(r"WriteDecision SafeActuatorPolicy::evaluateEndpointPlan"
                           r"\(.*?\n\}", code, re.DOTALL)
    if plan_route is None:
        fail(f"{path}: evaluateEndpointPlan() not found")
    else:
        for token in ("direction_verify_tick_budget", "DIRECTION_VERIFY"):
            if token in plan_route.group(0):
                fail(f"{path}: the endpoint-plan route consults {token!r} - contact "
                     f"probing and auxiliary moves are calibration work and must never "
                     f"depend on an optional direction diagnostic")
    budget_uses = code.count("direction_verify_tick_budget")
    in_envelope = envelope.group(0).count("direction_verify_tick_budget") if envelope else 0
    if budget_uses != in_envelope:
        fail(f"{path}: direction_verify_tick_budget is read outside "
             f"evaluateBootstrapEnvelope() ({budget_uses} uses, {in_envelope} of them in "
             f"the diagnostic path) - it is an OPTIONAL diagnostic budget and must gate "
             f"nothing else")


def check_ota_boundaries(files, sketch_dir):
    """OTA-A permanent invariants.

    OTA is the one subsystem that can make a device unbootable, so the rules
    that keep it safe are enforced here rather than trusted to review:

      1. the decision layers stay host-linkable, so the offline suite drives
         the shipped state machine and every failure path is reachable;
      2. the ESP-IDF OTA API is confined to ONE translation unit;
      3. the boot target can be changed from exactly one validated state, and
         only through esp_ota_set_boot_partition in that one unit;
      4. the running image is never the write target;
      5. the first-boot confirmation is not called at startup;
      6. byte ingest is compiled OUT by default - OTA-A has no authentication,
         so a production image must not contain a reachable firmware writer.
    """
    by_name = {path.name: (path, code) for path, code in files}
    update_dir = sketch_dir / "src" / "update"

    # --- (1) host-linkable decision layers ---------------------------------
    for name in ("OtaPolicy.h", "OtaPolicy.cpp", "OtaBootGuard.h", "OtaBootGuard.cpp",
                 "Sha256.h", "Sha256.cpp"):
        entry = by_name.get(name)
        if entry is None:
            fail(f"{update_dir / name}: OTA-A decision unit not found")
            continue
        path, code = entry
        for forbidden in ("#include <Arduino.h>", "#include <esp_ota_ops.h>",
                          "#include <esp_partition.h>", "#include <WiFi.h>"):
            if forbidden in code:
                fail(f"{path}: contains {forbidden!r} - the OTA decision layers must stay "
                     f"free of the Arduino runtime and of ESP-IDF so "
                     f"scripts/tests/test_ota_policy.cpp drives the REAL state machine "
                     f"against a fake backend (OTA-A)")
        if "Serial." in code:
            fail(f"{path}: contains Serial output - the OTA state layer must stay "
                 f"transport-independent (ARCHITECTURE.md, telemetry snapshot model)")

    # --- (2) the ESP-IDF OTA API lives in exactly one unit -----------------
    ota_api = ("esp_ota_begin", "esp_ota_write", "esp_ota_end", "esp_ota_abort",
               "esp_ota_set_boot_partition", "esp_ota_get_next_update_partition",
               "esp_ota_mark_app_valid_cancel_rollback", "esp_ota_mark_app_invalid")
    for path, code in files:
        if path.name == "OtaEspBackend.cpp":
            continue
        hits = [sym for sym in ota_api if sym in code]
        if hits:
            fail(f"{path}: calls ESP-IDF OTA API {hits} outside "
                 f"update/OtaEspBackend.cpp - the write/boot-switch surface must stay in "
                 f"one auditable translation unit (OTA-A)")

    backend = by_name.get("OtaEspBackend.cpp")
    if backend is None:
        fail(f"{update_dir / 'OtaEspBackend.cpp'}: OTA ESP-IDF backend not found")
    else:
        path, code = backend
        # --- (4) the backend refuses to point boot at the running slot -----
        body = re.search(r"bool OtaEspBackend::setBootPartition\([^)]*\)\s*\{(.*?)\n\}",
                         code, re.DOTALL)
        if not body:
            fail(f"{path}: could not locate setBootPartition() to audit it")
        elif "esp_ota_get_running_partition" not in body.group(1):
            fail(f"{path}: setBootPartition() does not independently re-check the running "
                 f"partition - the last line of defence against pointing the boot target "
                 f"at the slot we are executing from (OTA-A)")
        # esp_ota_begin must not be handed an explicit image size: that erases
        # the whole range up front, seconds of blocking for a ~1 MB image.
        if "OTA_WITH_SEQUENTIAL_WRITES" not in code:
            fail(f"{path}: esp_ota_begin() is not using OTA_WITH_SEQUENTIAL_WRITES - an "
                 f"up-front full-range erase blocks the Controller loop for seconds "
                 f"(handoff section 18)")

    # --- (3) exactly one commit path, reachable from one state -------------
    policy = by_name.get("OtaPolicy.cpp")
    if policy is not None:
        path, code = policy
        commit = re.search(r"bool OtaPolicy::commitBootTarget\(\)\s*\{(.*?)\n\}",
                           code, re.DOTALL)
        if not commit:
            fail(f"{path}: could not locate commitBootTarget() to audit it")
        else:
            body = commit.group(1)
            if "OtaState::IDENTITY_VERIFIED" not in body:
                fail(f"{path}: commitBootTarget() is not gated on "
                     f"OtaState::IDENTITY_VERIFIED - the boot target must only change "
                     f"after the stream completed, the image passed the ESP-IDF check and "
                     f"the hash matched (OTA-A critical safety rule)")
        # setBootPartition must be called from that one method and nowhere else.
        calls = len(re.findall(r"setBootPartition\(", code))
        if calls != 1:
            fail(f"{path}: setBootPartition( is called {calls} time(s) - exactly one call "
                 f"site, inside commitBootTarget(), keeps the boot switch auditable")
        for required, why in (
            ("OtaFault::TARGET_IS_RUNNING", "the target must be explicitly checked against "
                                            "the running partition"),
            ("OtaFault::TARGET_NOT_OTA_SLOT", "the target subtype must be explicitly checked"),
            ("OtaFault::IMAGE_TOO_LARGE", "the image size must be explicitly bounded by the "
                                          "target partition"),
            ("OtaFault::RUNNING_IMAGE_UNCONFIRMED", "a PENDING_VERIFY running image must be "
                                                    "refused before esp_ota_begin sees it"),
        ):
            if required not in code:
                fail(f"{path}: {required} is not present - {why} (OTA-A, fail closed)")

    # --- (5) first-boot confirmation is earned, not granted at startup -----
    guard = by_name.get("OtaBootGuard.cpp")
    if guard is not None:
        path, code = guard
        if "markAppValid()" not in code:
            fail(f"{path}: the boot guard never confirms an image - a PENDING_VERIFY image "
                 f"would always be rolled back")
    ctl = by_name.get("Controller.cpp")
    if ctl is not None:
        path, code = ctl
        begin_body = re.search(r"void Controller::begin\(\)\s*\{(.*?)\n\}", code,
                               re.DOTALL)
        if begin_body and "markAppValid" in begin_body.group(1):
            fail(f"{path}: Controller::begin() confirms the OTA image - confirmation must "
                 f"be earned by running, not granted at startup, or the bootloader's "
                 f"rollback is thrown away for exactly the case it exists for (OTA-A)")

    # --- (6) ingest compiled out by default --------------------------------
    manager = by_name.get("OtaManager.h")
    if manager is None:
        fail(f"{update_dir / 'OtaManager.h'}: OTA manager not found")
    else:
        path, code = manager
        m = re.search(r"#define\s+MATDOG_OTA_INGEST_ENABLED\s+(\S+)", code)
        if not m:
            fail(f"{path}: could not locate the MATDOG_OTA_INGEST_ENABLED default")
        elif m.group(1).strip() != "0":
            fail(f"{path}: MATDOG_OTA_INGEST_ENABLED defaults to {m.group(1)!r}, expected 0 "
                 f"- OTA-A has no authentication, so a reachable firmware writer must not "
                 f"be compiled into an image by default (handoff section 17)")


def check_wifi_runtime_boundaries(files, sketch_dir):
    """W1 permanent invariants for the Wi-Fi runtime.

    The tripwire in check_no_network_to_servo_path already forbids the worst
    outcome (a network translation unit reaching a servo primitive). These
    checks defend the three other properties W1 actually rests on, so none
    of them can be lost to a later "small" edit:

      1. the decision logic stays host-linkable, so the offline suite
         exercises the shipped state machine rather than a copy;
      2. the Wi-Fi tick stays bounded, so it cannot starve the Controller
         loop (ARCHITECTURE.md, resource isolation);
      3. the passphrase stays in exactly one place and out of Git.
    """
    by_name = {path.name: (path, code) for path, code in files}
    network_dir = sketch_dir / "src" / "network"

    # --- (1) the policy layer stays host-linkable --------------------------
    for name in ("WifiPolicy.h", "WifiPolicy.cpp"):
        entry = by_name.get(name)
        if entry is None:
            fail(f"{network_dir / name}: W1 Wi-Fi policy unit not found - the Wi-Fi "
                 f"decision logic must live in a host-linkable translation unit")
            continue
        path, code = entry
        for forbidden in ("#include <Arduino.h>", "#include <WiFi.h>"):
            if forbidden in code:
                fail(f"{path}: contains {forbidden!r} - the Wi-Fi policy must stay free of "
                     f"the Arduino runtime and of the radio so scripts/tests/"
                     f"test_wifi_policy.cpp links the REAL state machine (W1, and the same "
                     f"contract as DalyProtocol/ServoPopulation)")
        if "Serial." in code:
            fail(f"{path}: contains Serial output - the Wi-Fi state layer must stay "
                 f"transport-independent so USB CDC and a future Web UI consume one "
                 f"structured snapshot (ARCHITECTURE.md, telemetry snapshot model)")

    policy_header = by_name.get("WifiPolicy.h")
    if policy_header is not None and "struct WifiStatus" not in policy_header[1]:
        fail(f"{policy_header[0]}: WifiStatus struct not found - the Wi-Fi layer must "
             f"produce structured state, not formatted text")

    # --- (2) the Wi-Fi tick stays bounded ----------------------------------
    manager = by_name.get("WifiManager.cpp")
    if manager is None:
        fail(f"{network_dir / 'WifiManager.cpp'}: W1 Wi-Fi radio owner not found")
    else:
        path, code = manager
        # Blocking calls that exist in esp32:esp32 3.3.11 and would stall
        # Controller::update(). WiFi.disconnect() is listed because its
        # default overload polls for up to 100 ms; disconnectAsync() does
        # not, and is what this module is required to use.
        for token, why in (
            ("waitForConnectResult", "blocks until the association resolves"),
            ("WiFi.disconnect(", "the blocking overload polls up to 100 ms - use "
                                 "disconnectAsync()"),
            ("WiFi.scanNetworks()", "the blocking scan form stops the loop for seconds"),
            ("WiFi.SSID()", "returns an Arduino String and would allocate on every poll"),
            ("delay(", "a delay in the network path is not a connection-management "
                       "strategy (handoff section 18)"),
        ):
            if token in code:
                fail(f"{path}: {token} is forbidden in the Wi-Fi runtime - {why}")

        # No unbounded iteration in the per-tick entry point. begin() may
        # loop (it copies the SSID once, bounded by the buffer); update()
        # may not.
        body = re.search(r"void WifiManager::update\(uint32_t now_ms\)\s*\{(.*?)\n\}",
                         code, re.DOTALL)
        if not body:
            fail(f"{path}: could not locate WifiManager::update() to audit its bounds")
        else:
            for token in ("while (", "while("):
                if token in body.group(1):
                    fail(f"{path}: WifiManager::update() contains {token!r} - the Wi-Fi "
                         f"tick must be a single bounded evaluation, never a wait loop")

        # --- snapshot self-consistency (W1 review findings) -------------
        # The adapter is not host-linkable (it owns the radio), so these two
        # invariants cannot be pinned by the offline suite. They were found
        # by review and are pinned here instead.
        #
        # 1. A command handler that changes state must republish the
        #    snapshot before returning. CommandRouter prints the snapshot in
        #    the same pass as the acknowledgement, so a stale one made
        #    @WIFI OFF answer "WIFI=OFF" and then print "enabled=YES".
        setter = re.search(r"bool WifiManager::setEnabled\([^)]*\)\s*\{(.*?)\n\}",
                           code, re.DOTALL)
        if not setter:
            fail(f"{path}: could not locate WifiManager::setEnabled() to audit it")
        elif "publishPolicyState(" not in setter.group(1) and \
             "refreshSnapshot(" not in setter.group(1):
            fail(f"{path}: WifiManager::setEnabled() does not republish the snapshot - the "
                 f"@WIFI reply would contradict itself, printing the previous tick's "
                 f"enabled/state alongside the new acknowledgement (W1 review)")

        # 2. Tearing the radio down invalidates the link state read at the
        #    top of the same tick. Without this the snapshot publishes
        #    state=INACTIVE together with connected=YES and a live IP/RSSI,
        #    and polls a radio that is going away.
        if body:
            stop_case = re.search(r"case WifiAction::STOP_RADIO:(.*?)break;", body.group(1),
                                  re.DOTALL)
            if not stop_case:
                fail(f"{path}: WifiManager::update() has no STOP_RADIO case to audit")
            elif not re.search(r"link_up\s*=\s*false", stop_case.group(1)):
                fail(f"{path}: the STOP_RADIO path does not invalidate link_up - the "
                     f"snapshot would report a torn-down radio as a live link (W1 review)")

        # The network layer may observe mode, never change it: authority is
        # not a network concept (handoff sections 9/19).
        if "setMode(" in code:
            fail(f"{path}: calls setMode( - a network task must never change "
                 f"OperatingMode; authority stays with the Controller")

    # --- (3) the passphrase: one use site, never committed -----------------
    creds = by_name.get("WifiCredentials.h")
    if creds is None:
        fail(f"{sketch_dir / 'src' / 'config' / 'WifiCredentials.h'}: Wi-Fi credential "
             f"resolver not found")
    else:
        path, code = creds
        if "kWifiCredentialsPresent" not in code:
            fail(f"{path}: kWifiCredentialsPresent not found - an absent SSID must be a "
                 f"compile-time fact so the radio is never started without credentials")
        if 'define MATDOG_WIFI_SSID ""' not in code:
            fail(f"{path}: the empty-SSID fallback is missing - a checkout with no "
                 f"credentials must still build and boot, with the radio never started")

    # kWifiPassword must be named in exactly one place besides its own
    # declaration: the single WiFi.begin() call. Anywhere else is a step
    # toward it reaching a log line, @STATUS or a web response.
    password_sites = []
    for path, code in files:
        if path.name == "WifiCredentials.h":
            continue
        if "kWifiPassword" in code:
            password_sites.append((str(path), code.count("kWifiPassword")))
    total = sum(n for _, n in password_sites)
    if total != 1 or (password_sites and pathlib.Path(password_sites[0][0]).name != "WifiManager.cpp"):
        fail(f"kWifiPassword is referenced {total} time(s) at {password_sites} - it must "
             f"appear exactly once, in network/WifiManager.cpp, passed straight to the "
             f"connect call. It must never be stored, returned or printed (W1)")

    policy_files = [c for n, (p, c) in by_name.items() if n in ("WifiPolicy.h", "WifiPolicy.cpp")]
    for code in policy_files:
        for token in ("password", "passphrase", "psk"):
            if token in code.lower():
                fail(f"src/network/WifiPolicy.*: mentions {token!r} - the observable Wi-Fi "
                     f"snapshot must have no field that could ever hold a secret (W1)")

    # The local credentials file must stay ignored by Git, and untracked.
    gitignore = sketch_dir / ".gitignore"
    local_rel = "src/config/WifiCredentials.local.h"
    if not gitignore.exists():
        fail(f"{gitignore}: not found - it must ignore {local_rel}")
    else:
        # Exact-line match, not a substring search. The surrounding comment
        # block names ".../WifiCredentials.local.h.example", which contains
        # the rule as a substring - a naive `in` test would keep passing
        # after the real rule line was deleted.
        rules = [ln.strip() for ln in gitignore.read_text(encoding="utf-8").splitlines()]
        rules = [ln for ln in rules if ln and not ln.startswith("#")]
        if local_rel not in rules:
            fail(f"{gitignore}: has no ignore rule for {local_rel} (active rules: {rules}) - "
                 f"removing that entry makes a real Wi-Fi passphrase committable (W1)")

    template = sketch_dir / "src" / "config" / "WifiCredentials.local.h.example"
    if not template.exists():
        fail(f"{template}: credential template not found - it is the documented way to "
             f"configure Wi-Fi without touching a tracked file")

    result = subprocess.run(["git", "-C", str(sketch_dir), "ls-files", "--error-unmatch",
                             local_rel],
                            capture_output=True, text=True)
    if result.returncode == 0:
        fail(f"{local_rel} is TRACKED by Git - a real Wi-Fi passphrase must never be "
             f"committed. Run: git rm --cached {local_rel}")


def check_host_tests(sketch_dir):
    """Runs the offline C++ census/profile suite, the same way the OTA
    parser's Python suite is already run from here: one gate command."""
    runner = sketch_dir / "scripts" / "tests" / "run_host_tests.sh"
    suite = sketch_dir / "scripts" / "tests" / "test_servo_population.cpp"
    daly_suite = sketch_dir / "scripts" / "tests" / "test_daly_protocol.cpp"
    wifi_suite = sketch_dir / "scripts" / "tests" / "test_wifi_policy.cpp"
    ota_suite = sketch_dir / "scripts" / "tests" / "test_ota_policy.cpp"
    authority_suite = sketch_dir / "scripts" / "tests" / "test_actuator_authority.cpp"
    policy_suite = sketch_dir / "scripts" / "tests" / "test_actuator_write_policy.cpp"
    geometry_suite = sketch_dir / "scripts" / "tests" / "test_calibration_geometry.cpp"
    profile_suite = sketch_dir / "scripts" / "tests" / "test_servo_profile.cpp"
    calibration_suites = [
        sketch_dir / "scripts" / "tests" / "test_calibration_domain.cpp",
        sketch_dir / "scripts" / "tests" / "test_calibration_manager.cpp",
    ]
    led_status_suite = sketch_dir / "scripts" / "tests" / "test_led_status_policy.cpp"
    actuator_runtime_suite = sketch_dir / "scripts" / "tests" / "test_actuator_runtime.cpp"
    if not suite.exists():
        fail(f"{suite}: G2 servo population/profile offline test suite not found")
        return
    if not daly_suite.exists():
        fail(f"{daly_suite}: DALY protocol/KEY probe offline test suite not found")
        return
    if not wifi_suite.exists():
        fail(f"{wifi_suite}: W1 Wi-Fi runtime policy offline test suite not found")
        return
    if not ota_suite.exists():
        fail(f"{ota_suite}: OTA-A policy/boot-guard/sha256 offline test suite not found")
        return
    if not authority_suite.exists():
        fail(f"{authority_suite}: ActuatorAuthority offline test suite not found")
        return
    if not policy_suite.exists():
        fail(f"{policy_suite}: Safe Actuator Layer write-policy offline test suite not found")
        return
    if not geometry_suite.exists():
        fail(f"{geometry_suite}: calibration bootstrap geometry offline test suite not found")
        return
    if not profile_suite.exists():
        fail(f"{profile_suite}: MATDOG_C018_V1 profile offline test suite not found")
        return
    for suite in calibration_suites:
        if not suite.exists():
            fail(f"{suite}: calibration offline test suite not found")
            return
    if not led_status_suite.exists():
        fail(f"{led_status_suite}: LED status policy offline test suite not found")
        return
    if not actuator_runtime_suite.exists():
        fail(f"{actuator_runtime_suite}: Safe Actuator runtime adapter offline test suite not found")
        return
    if not runner.exists():
        fail(f"{runner}: host test runner not found")
        return
    runner_text = strip_shell_comments(runner.read_text(encoding="utf-8"))
    for binary in ("test_servo_population", "test_daly_protocol", "test_wifi_policy",
                   "test_ota_policy", "test_actuator_authority", "test_actuator_write_policy",
                   "test_calibration_geometry", "test_servo_profile",
                   "test_calibration_domain", "test_calibration_manager",
                   "test_led_status_policy", "test_actuator_runtime"):
        if f'"$OUT/{binary}"' not in runner_text:
            fail(f"{runner}: does not run {binary} - every offline suite must gate")
    result = subprocess.run(["bash", str(runner)], capture_output=True, text=True)
    if result.returncode != 0:
        fail(f"{runner}: servo population/profile offline tests FAILED "
             f"(stdout={result.stdout!r} stderr={result.stderr!r})")


def check_daly_audit_mutation_suite(sketch_dir):
    """Proves the DALY write prohibition above actually fails on mutation."""
    suite = sketch_dir / "scripts" / "tests" / "test_static_audit_daly.py"
    if not suite.exists():
        fail(f"{suite}: DALY audit mutation suite not found")
        return
    result = subprocess.run([sys.executable, str(suite)], capture_output=True, text=True)
    if result.returncode != 0:
        fail(f"{suite}: DALY audit mutation tests FAILED "
             f"(stdout={result.stdout!r} stderr={result.stderr!r})")


def check_safe_actuator_audit_mutation_suite(sketch_dir):
    """Proves the Safe Actuator Layer boundaries above actually fail on
    mutation - purity, SAFE_OFF independence, the operation classes, limit
    provenance, and the pre-existing torque-on prohibition."""
    suite = sketch_dir / "scripts" / "tests" / "test_static_audit_safe_actuator.py"
    if not suite.exists():
        fail(f"{suite}: Safe Actuator Layer audit mutation suite not found")
        return
    result = subprocess.run([sys.executable, str(suite)], capture_output=True, text=True)
    if result.returncode != 0:
        fail(f"{suite}: Safe Actuator Layer audit mutation tests FAILED "
             f"(stdout={result.stdout!r} stderr={result.stderr!r})")


def strip_shell_comments(text):
    """Drops whole-line shell comments. The provenance checks below must
    match REAL invocations, not the prose describing them - an early
    version of this audit passed a mutation that deleted the manifest gate
    entirely, because the words 'build_manifest.py' still appeared in this
    script's own header comment."""
    return "\n".join(line for line in text.splitlines()
                     if not line.lstrip().startswith("#"))


def check_build_profile_provenance(sketch_dir):
    """G2 pre-G3 hardening (review Finding 1): the flashed image's hardware
    profile must be PROVEN, never assumed.

    build.sh can emit a USB_ONLY or a ROBOT_POWERED image from the same
    commit to the same path, so the commit/build-id gate cannot tell them
    apart. The manifest closes that hole; this check makes its removal or
    neutralization a build failure rather than a silent regression.
    """
    scripts_dir = sketch_dir / "scripts"
    logic = scripts_dir / "build_manifest.py"
    build_sh = scripts_dir / "build.sh"
    flash_sh = scripts_dir / "flash_app_only.sh"
    tests = scripts_dir / "tests" / "test_build_manifest.py"

    if not logic.exists():
        fail(f"{logic}: build manifest logic module not found - the flash path would "
             f"have no way to prove which hardware profile a binary came from")
        return
    logic_text = logic.read_text(encoding="utf-8")

    for token in ("KNOWN_PROFILES", "MANIFEST_VERSION", "verify_manifest",
                  "render_manifest", "parse_manifest",
                  "PROFILE_MISMATCH", "PROFILE_UNKNOWN", "MANIFEST_MISSING",
                  "FQBN_MISMATCH",
                  "BINARY_SHA256_MISMATCH", "BINARY_SIZE_MISMATCH",
                  "SOURCE_COMMIT_MISMATCH", "TREE_NOT_CLEAN"):
        if token not in logic_text:
            fail(f"{logic}: missing required manifest/provenance primitive {token!r}")

    # verify_manifest() must not acquire a permissive default for the
    # profile it compares against - a caller that forgot the argument must
    # be an error, never a silent "allow".
    if re.search(r"def verify_manifest\([^)]*requested_profile\s*=", logic_text, re.DOTALL):
        fail(f"{logic}: verify_manifest() gained a default for requested_profile - the "
             f"caller must always state the profile it intends to flash")
    if re.search(r'add_argument\("--requested-profile"[^)]*default=', logic_text):
        fail(f"{logic}: --requested-profile gained a default - flash_app_only.sh must "
             f"pass it explicitly")

    # Build-configuration provenance: the recorded FQBN must be COMPARED,
    # not merely recorded. The manifest carried FQBN from the start while
    # nothing checked it, which left the partition scheme / flash size /
    # PSRAM mode unverified at flash time.
    if re.search(r"def verify_manifest\([^)]*expected_fqbn\s*=", logic_text, re.DOTALL):
        fail(f"{logic}: verify_manifest() gained a default for expected_fqbn - the "
             f"caller must always state the FQBN it expects")
    if "expected_fqbn" not in logic_text:
        fail(f"{logic}: verify_manifest() no longer takes expected_fqbn - the recorded "
             f"FQBN would be unverified again")
    if re.search(r'add_argument\("--expected-fqbn"[^)]*default=', logic_text):
        fail(f"{logic}: --expected-fqbn gained a default - flash_app_only.sh must pass "
             f"its own pinned FQBN explicitly")
    if not re.search(r'manifest\["FQBN"\]\s*!=\s*expected_fqbn', logic_text):
        fail(f"{logic}: the FQBN equality comparison against expected_fqbn is missing - "
             f"recording the FQBN without comparing it proves nothing")

    if not build_sh.exists():
        fail(f"{build_sh}: build script not found")
        return
    build_text = strip_shell_comments(build_sh.read_text(encoding="utf-8"))
    if not re.search(r'build_manifest\.py"?\s+write', build_text):
        fail(f"{build_sh}: does not invoke `build_manifest.py write` - every build must "
             f"record the hardware profile and binary digest it produced")
    if "--profile" not in build_text:
        fail(f"{build_sh}: does not pass --profile to the manifest writer")
    if "--source-state" not in build_text:
        fail(f"{build_sh}: does not record the clean/dirty source state in the manifest")

    if not flash_sh.exists():
        fail(f"{flash_sh}: application-only flash script not found")
        return
    flash_text = strip_shell_comments(flash_sh.read_text(encoding="utf-8"))

    verify_match = re.search(r'build_manifest\.py"?\s+verify', flash_text)
    if not verify_match:
        fail(f"{flash_sh}: does not invoke `build_manifest.py verify` - a binary of "
             f"unknown hardware profile could be written to the device")
    if "--requested-profile" not in flash_text:
        fail(f"{flash_sh}: does not pass --requested-profile to the manifest verifier")
    if not re.search(r'--expected-fqbn\s+"\$FQBN"', flash_text):
        fail(f"{flash_sh}: does not pass --expected-fqbn \"$FQBN\" to the manifest "
             f"verifier - the recorded build FQBN would go unverified")
    # The parameter expansion, not just the name in prose: the operator
    # authorization input must actually be readable from the environment.
    if "MATDOG_FLASH_PROFILE:-" not in flash_text:
        fail(f"{flash_sh}: lost the MATDOG_FLASH_PROFILE operator authorization input "
             f"(expected a ${{MATDOG_FLASH_PROFILE:-...}} expansion)")

    write_match = re.search(r"write-flash", flash_text)

    # The gate must run BEFORE the device write, not after it.
    if verify_match is None or write_match is None or \
            verify_match.start() > write_match.start():
        fail(f"{flash_sh}: the build-manifest verification must appear before the "
             f"esptool write-flash invocation")

    # The verification must not be neutralized into a warning. Scans the
    # whole verify command (it spans continuation lines) for a swallowed
    # failure - `|| refuse ...` is correct, `|| true` / `|| :` is not.
    if verify_match is not None:
        lines = flash_text.splitlines()
        start = flash_text[:verify_match.start()].count("\n")
        block = []
        for line in lines[start:start + 12]:
            block.append(line)
            if not line.rstrip().endswith("\\"):
                break
        if re.search(r"\|\|\s*(true|:|echo|warn)\b", "\n".join(block)):
            fail(f"{flash_sh}: the manifest verification swallows its own failure "
                 f"('|| true'/'|| :'/'|| echo') - it must REFUSE, not warn")

    # Anti-weakening: every pre-existing flash gate must still be ENFORCED,
    # not merely mentioned. Counting token occurrences is not enough - each
    # of these constants also appears in its own refuse() message, so a
    # deleted comparison still leaves two mentions behind. These patterns
    # match the actual comparison that does the gating.
    for pattern, description in (
            (r'\[\s*"\$BACKUP_SIZE"\s*-eq\s*"\$EXPECTED_BACKUP_SIZE"\s*\]',
             "full-flash backup size comparison"),
            (r'\[\s*"\$BACKUP_SHA256"\s*=\s*"\$EXPECTED_BACKUP_SHA256"\s*\]',
             "full-flash backup digest comparison"),
            (r'\[\s*"\$DEVICE_MAC"\s*=\s*"\$EXPECTED_MAC"\s*\]',
             "device identity (MAC) comparison"),
            (r'\[\s*-n\s*"\$(APPLICATION_OFFSET|MAX_PARTITION_SIZE)"\s*\]',
             "verified application offset/size check"),
            (r'"\$APPLICATION_SIZE"\s*-le\s*"\$MAX_PARTITION_SIZE"',
             "application-fits-in-partition check"),
            (r'python3 "\$SCRIPT_DIR/verify_application_partition\.py"',
             "verified application partition gate"),
            (r"--sdkconfig", "rollback/anti-rollback state gate"),
            (r'python3 "\$SCRIPT_DIR/static_audit\.py"', "static safety audit gate"),
            # Anchored to the esptool invocation: 'verify-flash' also appears
            # in this script's own refuse() message, so a bare token match
            # would survive the command itself being deleted.
            (r'"\$ESPTOOL"[^\n]*verify-flash', "independent post-write verification")):
        if not re.search(pattern, flash_text):
            fail(f"{flash_sh}: lost the {description} - G2 hardening must not weaken "
                 f"any pre-existing application-only protection")

    # The operator must SEE the verified profile prominently before the
    # write. Anchored to the echo that actually prints the value, not to
    # any mention of the variable (the assignment and the post-write
    # summary both mention it and would otherwise satisfy a loose check).
    banner = None
    for idx, line in enumerate(flash_text.splitlines()):
        if "echo" in line and "HARDWARE PROFILE" in line and \
                "$VERIFIED_HARDWARE_PROFILE" in line:
            banner = flash_text.index(line)
            break
    if banner is None:
        fail(f"{flash_sh}: does not print the verified hardware profile prominently "
             f"before writing (expected an echo of $VERIFIED_HARDWARE_PROFILE)")
    elif write_match is not None and banner > write_match.start():
        fail(f"{flash_sh}: prints the verified hardware profile only after the write - "
             f"the operator must see it beforehand")

    if not tests.exists():
        fail(f"{tests}: build manifest offline test suite not found")
        return
    result = subprocess.run([sys.executable, str(tests)], capture_output=True, text=True)
    if result.returncode != 0:
        fail(f"{tests}: build manifest offline tests FAILED "
             f"(stdout={result.stdout!r} stderr={result.stderr!r})")

def check_unknown_detection_is_not_a_verdict(files):
    """G2 pre-G3 hardening (review Finding 2): classify() must keep
    DetectedState::UNKNOWN ('nothing has established anything') distinct
    from NO_RESPONSE ('we asked and it did not answer').

    Collapsing them again would re-break two things at once: the LED ring
    (non-probeable, therefore permanently UNKNOWN when powered) would make
    SystemHealth::READY unreachable, and the servo bus (REQUIRED but never
    probed at boot) would report FAULT on a healthy powered robot.
    """
    for path, code in files:
        if path.name != "Availability.cpp":
            continue
        m = re.search(r"Classification classify\(const AvailabilityStatus& s\)\s*\{(.*?)\n\}",
                      code, re.DOTALL)
        if not m:
            fail(f"{path}: classify() not found to audit its UNKNOWN handling")
            continue
        body = m.group(1)
        if "DetectedState::UNKNOWN" not in body:
            fail(f"{path}: classify() no longer distinguishes DetectedState::UNKNOWN from "
                 f"an observed absence - 'not probed' must not be turned into a verdict "
                 f"(G2 review Finding 2)")
        # An observed failure must still escalate: both escalation arms
        # must survive somewhere in the function.
        if "Classification::FAULT" not in body or "Classification::DEGRADED" not in body:
            fail(f"{path}: classify() lost its FAULT/DEGRADED escalation for an observed "
                 f"absence - the UNKNOWN fix must not mute real failures")

    # And the modules must not have been "fixed" by faking a physical
    # observation instead.
    for path, code in files:
        if path.name != "Availability.cpp":
            continue
        m = re.search(r"DetectedState detectedStateForLedRail\([^)]*\)\s*\{(.*?)\n\}",
                      code, re.DOTALL)
        if not m:
            fail(f"{path}: detectedStateForLedRail() not found")
            continue
        if "DetectedState::ONLINE" in m.group(1):
            fail(f"{path}: detectedStateForLedRail() reports ONLINE - a WS2812 chain "
                 f"cannot be interrogated, so claiming physical detection to make a "
                 f"status green is forbidden (G2 review Finding 2)")


def check_usb_cdc_tx_never_blocks(files):
    """G3.1: no USB CDC transmit condition may block Controller::update().

    With tx_timeout_ms = 0 every wait in HWCDC::write() (3.3.11) becomes an
    immediate drop. That guarantee holds only if the timeout is 0 before the
    first byte and nothing raises it again.
    """
    by_name = {path.name: (path, code) for path, code in files}

    cfg_path, cfg = by_name.get("BuildConfig.h", (None, ""))
    m = re.search(r"kUsbTxTimeoutMs\s*=\s*(\d+)\s*;", cfg)
    if not m or int(m.group(1)) != 0:
        fail(f"{cfg_path}: kUsbTxTimeoutMs must be exactly 0 - any non-zero value lets "
             f"HWCDC::write() wait on a host that is not reading (G3.1)")
    m = re.search(r"kUsbTxRingBytes\s*=\s*(\d+)\s*;", cfg)
    if not m or int(m.group(1)) < 3072:
        fail(f"{cfg_path}: kUsbTxRingBytes must be >= 3072 - the largest single loop-pass "
             f"burst is 2674 B (worst census + DALY KEY write ACK and COMPLETE + IMU + BMS; "
             f"2395 B before the KEY probe) and with timeout 0 anything beyond the ring is "
             f"dropped even while a host is reading (G3.1)")

    ctl_path, ctl = by_name.get("Controller.cpp", (None, ""))
    body = re.search(r"void Controller::begin\(\)\s*\{(.*?)\n\}", ctl, re.DOTALL)
    body = body.group(1) if body else ""
    ring = body.find("Serial.setTxBufferSize(build::kUsbTxRingBytes)")
    tout = body.find("Serial.setTxTimeoutMs(build::kUsbTxTimeoutMs)")
    begin = body.find("Serial.begin(")
    first_out = min([i for i in (body.find("Serial.print"), body.find("printBootBanner("))
                     if i != -1] or [len(body)])
    if -1 in (ring, tout, begin) or not (ring < begin and tout < begin < first_out):
        fail(f"{ctl_path}: Controller::begin() must set the TX ring and the 0 ms TX "
             f"timeout before Serial.begin(), and Serial.begin() before any output (G3.1)")
    for guard in ("ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT",
                  "ESP_ARDUINO_VERSION_VAL(3, 3, 11)"):
        if guard not in ctl:
            fail(f"{ctl_path}: compile-time guard {guard!r} missing - the non-blocking "
                 f"guarantee was audited against HWCDC in esp32:esp32 3.3.11 only")

    for token, why in (("setTxTimeoutMs(", "exactly one TX timeout, set in Controller::begin()"),
                       ("setTxBufferSize(", "exactly one TX ring size, set in Controller::begin()")):
        hits = [str(p) for p, code in files for _ in range(code.count(token))]
        if len(hits) != 1:
            fail(f"{token} appears {len(hits)} time(s) {hits} - {why} (G3.1)")
    for path, code in files:
        for token in ("Serial.flush(", "setDebugOutput(", "shouldPrintChipDebugReport"):
            if token in code:
                fail(f"{path}: {token} is forbidden - flush() with timeout 0 discards the TX "
                     f"ring (and waits otherwise); debug output adds a second per-character "
                     f"transmit path and begins Serial before setup() (G3.1)")


def main():
    files = [(p, strip_comments(p.read_text(encoding="utf-8"))) for p in iter_source_files()]

    if not files:
        fail(f"no source files found under {SKETCH_DIR}")

    check_forbidden_literals(files)
    check_torque_enable(files)
    check_servo_id_write(files)
    check_daly_write(files)
    check_bms_command_surface(files)
    check_daly_protocol_is_host_linkable(files)
    check_pin_collisions(files)
    check_uart_peripheral_separation(files)
    check_no_auto_scan_on_boot(files)
    check_servo_scan_bounded_incremental(files)
    check_servo_diagnostics_require_maintenance_mode(files)
    check_led_anti_back_power(files)
    check_led_status_boundaries(files)
    check_actuator_runtime_boundaries(files)
    check_app_only_script_never_targets_other_partitions(SKETCH_DIR)
    check_ota_partition_verifier_fail_closed(SKETCH_DIR)
    check_servo_timeout_not_global(files)
    check_servo_timeout_categories_finding1(files)
    check_safe_off_verifies_readback(files)
    check_hardware_profile_authority(files, SKETCH_DIR)
    check_servo_population_model(files, SKETCH_DIR)
    check_g2_state_is_transport_independent(files)
    check_no_startup_servo_traffic(files)
    check_no_network_to_servo_path(files)
    check_wifi_runtime_boundaries(files, SKETCH_DIR)
    check_ota_boundaries(files, SKETCH_DIR)
    check_actuator_authority(files, SKETCH_DIR)
    check_calibration_boundaries(files, SKETCH_DIR)
    check_safe_actuator_boundaries(files, SKETCH_DIR)
    check_calibration_geometry_boundaries(files, SKETCH_DIR)
    check_calibration_geometry_export(SKETCH_DIR)
    check_position_offset_boundary(files, SKETCH_DIR)
    check_servo_profile_contract(files, SKETCH_DIR)
    check_servo_profile_export(SKETCH_DIR)
    check_h0_preflight_boundaries(files, SKETCH_DIR)
    check_evidence_geometry_binding(files, SKETCH_DIR)
    check_direction_is_contractual(files, SKETCH_DIR)
    check_host_tests(SKETCH_DIR)
    check_daly_audit_mutation_suite(SKETCH_DIR)
    check_safe_actuator_audit_mutation_suite(SKETCH_DIR)
    check_build_profile_provenance(SKETCH_DIR)
    check_unknown_detection_is_not_a_verdict(files)
    check_usb_cdc_tx_never_blocks(files)

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
