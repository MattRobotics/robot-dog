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

    rows = re.findall(r'\{\s*(\d+),\s*"(\w+)",\s*CurrentConfig::(\w+)\s*\}', code)
    if len(rows) != 17:
        fail(f"{path}: canonical servo table has {len(rows)} entries, expected 17 "
             f"(MATDOG canonical allocation)")
    installed = [r for r in rows if r[2] == "INSTALLED"]
    absent = [r for r in rows if r[2] == "ABSENT_BY_DESIGN"]
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
    if not suite.exists():
        fail(f"{suite}: G2 servo population/profile offline test suite not found")
        return
    if not daly_suite.exists():
        fail(f"{daly_suite}: DALY protocol/KEY probe offline test suite not found")
        return
    if not wifi_suite.exists():
        fail(f"{wifi_suite}: W1 Wi-Fi runtime policy offline test suite not found")
        return
    if not runner.exists():
        fail(f"{runner}: host test runner not found")
        return
    runner_text = strip_shell_comments(runner.read_text(encoding="utf-8"))
    for binary in ("test_servo_population", "test_daly_protocol", "test_wifi_policy"):
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
    check_host_tests(SKETCH_DIR)
    check_daly_audit_mutation_suite(SKETCH_DIR)
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
