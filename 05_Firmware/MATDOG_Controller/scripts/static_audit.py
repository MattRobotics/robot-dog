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

CR3 uncertain-write safety (2026-09-28) addition: a failed/absent ACK on the
servo bus does not prove a write was not applied - the ST3215 applies a
register write before it transmits any reply, so a lost/garbled reply is
indistinguishable, at the ACK layer, from the command never having arrived at
all. enableTorqueOn()/writeGoalPosition() must therefore return the three-way
servo::ServoWriteVerifyResult, classified ONLY from an independent readback of
the register just written (TorqueEnable / the GoalPosition register itself,
never present_position, which lags behind a write by the joint's travel time)
- the write's own ACK/status may never be the verdict, the same rule Session
2.2 Finding D already established for safeOff(). The uncertain state must
survive unflattened up through actuator::BackendWriteOutcome and
actuator::ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF: nothing may collapse it
into either a "written" or a "rejected" outcome. Reading (never writing) the
GoalPosition register is confined to exactly one accessor, the same
read-only-through-one-accessor shape as PositionOffset.

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


def contains_ws(haystack, needle):
    """Substring containment tolerant of reformatting: clang-format may wrap
    a long boolean/comparison expression across lines, which turns the
    single run of whitespace an exact-token check expects into a newline
    plus indentation. Collapsing every run of whitespace in both operands to
    one space makes the check pin the SAME token semantically regardless of
    where the formatter chose to break the line - see CR3 continuation
    audit-anchor fix (2026-09-28)."""
    normalize = lambda s: re.sub(r"\s+", " ", s)
    return normalize(needle) in normalize(haystack)


def check_forbidden_literals(files):
    forbidden = [
        "CalibrationOfs",
        "runNormalizeMatdog",
        "NORMALIZE_MATDOG",
        "unLockEprom",
        "LockEprom",
        "sh2_saveDcdNow",
        "RegWritePosEx",
        "SyncWritePosEx",
        "WheelMode",
        # SMS_STS_GOAL_POSITION_L is NOT banned outright any more (CR3
        # uncertain-write safety), for exactly the same reason SMS_STS_OFS_L
        # was narrowed below: a total ban on the register would also have
        # blocked reading back what was just written, which is the only way
        # to verify a GoalPosition write actually landed. check_
        # goal_position_register_boundary() replaces the ban with a narrower
        # rule: one approved read site, inside writeGoalPosition() only, via
        # readWord() only, and no write ever targets it.
        "SMS_STS_GOAL_POSITION_H",
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
    """CR3-M3: exactly one reviewed torque-on primitive may exist, and only
    inside ServoBus::enableTorqueOn(). SAFE_OFF remains the separate torque-off
    path. No other source may call EnableTorque(..., nonzero).

    CR3 uncertain-write safety additionally requires that the verdict come
    ONLY from the independent TorqueEnable readback via
    servo::classifyServoWriteVerify() - never from EnableTorque()'s own
    return value or st_.Error, which Session 2.2 Finding D already proved
    cannot even observe failure (SCS::Ack() returns 0, not -1, on failure)."""
    pattern = re.compile(r"EnableTorque\([^,]+,\s*([^)]+)\)")
    torque_on = []
    for path, code in files:
        for match in pattern.finditer(code):
            arg = match.group(1).strip()
            if arg == "0":
                continue
            torque_on.append((path, arg, match.start()))
            if path.name != "ServoBus.cpp" or arg != "1":
                fail(f"{path}: unreviewed EnableTorque non-zero call {arg!r}; CR3 permits "
                     f"exactly ServoBus::enableTorqueOn(id) -> EnableTorque(..., 1)")
    if len(torque_on) != 1:
        fail(f"CR3 torque-on surface must contain exactly one reviewed non-zero "
             f"EnableTorque call, found {len(torque_on)}")
    else:
        path, _, pos = torque_on[0]
        code = next(c for p, c in files if p == path)
        body = re.search(
            r"ServoWriteVerifyResult ServoBus::enableTorqueOn\(int id\)\s*\{(.*?)\n\}",
            code, re.DOTALL)
        if not body or not (body.start() <= pos <= body.end()):
            fail(f"{path}: the one torque-on call is not inside "
                 f"ServoBus::enableTorqueOn(), or it no longer returns "
                 f"ServoWriteVerifyResult")
        else:
            text = body.group(1)
            if "SMS_STS_TORQUE_ENABLE" not in text or "readByte" not in text:
                fail(f"{path}: enableTorqueOn() must independently read back TorqueEnable")
            if "classifyServoWriteVerify" not in text:
                fail(f"{path}: enableTorqueOn() must classify strictly via "
                     f"classifyServoWriteVerify(readback, ...), not by hand")
            if re.search(r"\bst_\.Error\b", text) or re.search(r"==\s*1\s*&&", text):
                fail(f"{path}: enableTorqueOn() appears to condition its verdict on the "
                     f"write's own ACK/status again - the readback alone must decide "
                     f"(CR3 uncertain-write safety)")


def check_servo_motion_write_surface(files):
    """CR3-M3: one and only one GoalPosition primitive exists, in ServoBus.

    It must use unsigned-domain validation and the fixed conservative
    WritePosEx speed/acceleration constants. Direct writeWord/writeByte,
    RegWrite/SyncWrite and every additional WritePosEx call remain forbidden.

    CR3 uncertain-write safety additionally requires the verdict to come ONLY
    from independently reading the GoalPosition register back and comparing
    it to the commanded tick via servo::classifyServoWriteVerify() - never
    from WritePosEx()'s own ACK/status, for the same reason as the torque-on
    primitive: a lost/garbled ACK on this half-duplex bus does not prove the
    write was not applied.
    """
    hits = []
    for path, code in files:
        for m in re.finditer(r"\bWritePosEx\s*\(", code):
            hits.append((path, m.start()))
    if len(hits) != 1:
        fail(f"CR3 GoalPosition surface must contain exactly one WritePosEx call, found {len(hits)}")
        return
    path, pos = hits[0]
    if path.name != "ServoBus.cpp":
        fail(f"{path}: WritePosEx may exist only in ServoBus::writeGoalPosition()")
        return
    code = next(c for p, c in files if p == path)
    body = re.search(
        r"ServoWriteVerifyResult ServoBus::writeGoalPosition\(int id, uint16_t target_tick\)"
        r"\s*\{(.*?)\n\}", code, re.DOTALL)
    if not body or not (body.start() <= pos <= body.end()):
        fail(f"{path}: WritePosEx is not inside ServoBus::writeGoalPosition(), or it no "
             f"longer returns ServoWriteVerifyResult")
        return
    text = body.group(1)
    for required in ("target_tick >= 4096u", "kOperationalTimeoutMs",
                     "kBoundedWriteSpeed", "kBoundedWriteAcceleration",
                     "SMS_STS_GOAL_POSITION_L", "readWord", "classifyServoWriteVerify"):
        if required not in text:
            fail(f"{path}: writeGoalPosition() missing reviewed CR3 guard {required!r}")
    for forbidden in ("RegWrite", "SyncWrite", "writeByte(", "writeWord("):
        if forbidden in text:
            fail(f"{path}: writeGoalPosition() contains forbidden primitive {forbidden!r}")
    if re.search(r"\bst_\.Error\b", text):
        fail(f"{path}: writeGoalPosition() reads st_.Error again - the independent "
             f"GoalPosition readback alone must decide (CR3 uncertain-write safety)")


def check_goal_position_register_boundary(files, sketch_dir):
    """CR3 uncertain-write safety: SMS_STS_GOAL_POSITION_L may be READ, and
    only inside writeGoalPosition()'s own verification step - the exact same
    read-only-through-one-accessor shape check_position_offset_boundary()
    already enforces for PositionOffset."""
    by_name = {path.name: (path, code) for path, code in files}
    entry = by_name.get("ServoBus.cpp")
    if entry is None:
        fail(f"{sketch_dir / 'src' / 'servo' / 'ServoBus.cpp'}: not found")
        return
    bus_path, bus_code = entry

    for path, code in files:
        if "SMS_STS_GOAL_POSITION_L" not in code:
            continue
        if path.name != "ServoBus.cpp":
            fail(f"{path}: names SMS_STS_GOAL_POSITION_L - the GoalPosition register may "
                 f"only be touched by ServoBus::writeGoalPosition()'s own verification read, "
                 f"so every access to it is in one auditable place")

    body = re.search(
        r"ServoWriteVerifyResult ServoBus::writeGoalPosition\(int id, uint16_t target_tick\)"
        r"\s*\{(.*?)\n\}", bus_code, re.DOTALL)
    if not body:
        fail(f"{bus_path}: ServoBus::writeGoalPosition() not found")
        return
    text = body.group(1)
    if "readWord(static_cast<uint8_t>(id), SMS_STS_GOAL_POSITION_L)" not in \
            re.sub(r"\s+", " ", text):
        fail(f"{bus_path}: writeGoalPosition() does not read back "
             f"SMS_STS_GOAL_POSITION_L via readWord() - the write's own ACK is not proof "
             f"the command was applied")
    for forbidden in ("writeByte", "writeWord", "genWrite", "regWrite", "RegWrite"):
        if forbidden in text and "readWord" not in forbidden:
            fail(f"{bus_path}: writeGoalPosition()'s verification step contains write "
                 f"primitive {forbidden!r} where a read was expected")

    # No PositionOffset-style write ever targets this register under any
    # spelling, anywhere in the tree.
    goal_write = re.compile(
        r"(writeByte|writeWord|genWrite|regWrite|RegWrite)\s*\([^;]*?"
        r"SMS_STS_GOAL_POSITION")
    for path, code in files:
        if goal_write.search(code):
            fail(f"{path}: a direct GoalPosition register WRITE is expressible here - the "
                 f"one reviewed write path is WritePosEx() inside writeGoalPosition(), never "
                 f"a raw register write")


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
    # ControllerService.h (I6) is a reviewed exception: it forwards the same
    # already-computed KEY snapshot/status CommandRouter already read, for
    # the same diagnostic presentation, through the transport-neutral
    # telemetry layer - it does not add a second decision path.
    for p2, c2 in files:
        if "scripts" in p2.parts or p2.name in DALY_SOURCE_NAMES or \
                p2.name in ("CommandRouter.cpp", "CommandRouter.h", "ControllerService.h"):
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
    """Every WS2812 transport entry retains an effective USB_ONLY return."""
    ring = next(((p, c) for p, c in files if p.name == "LedRing.cpp"), None)
    if ring is None:
        fail("LedRing.cpp not found - cannot audit anti-back-power")
        return
    path, code = ring
    for name in ("begin", "off", "setSolid", "renderFrame", "startTest", "startSocTest", "update"):
        match = re.search(r"(?:bool|void) LedRing::" + name + r"\([^)]*\)\s*\{(.*?)\n\}",
                          code, re.DOTALL)
        body = match.group(1) if match else ""
        guard = re.search(r"if\s*\(!build::kLedRailPowered\)\s*(?:\{([^}]+)\}|(return[^;]*;))",
                          body, re.DOTALL)
        guarded = (guard.group(1) or guard.group(2)) if guard else ""
        if not guard or not re.search(r"\breturn(?:\s+(?:true|false))?\s*;", guarded):
            fail(f"{path}: {name}() lost its effective USB_ONLY transport guard")
            continue
        if re.search(r"pixels_\.", body[:guard.end()]):
            fail(f"{path}: {name}() drives WS2812 before the USB_ONLY return")
        if name == "begin" and not re.search(r"pinMode\(pins::kLedRingDin,\s*INPUT\)", guarded):
            fail(f"{path}: USB_ONLY begin() must retain GPIO47 INPUT")
    if re.search(r"pinMode\([^,]+,\s*OUTPUT\)", code):
        fail(f"{path}: direct GPIO output bypasses the guarded WS2812 transport")


def check_uncertain_write_propagation(files, sketch_dir):
    """CR3 uncertain-write safety: an unverifiable backend outcome must reach
    the caller as a DISTINCT result, never silently folded into "written" or
    "rejected" at any layer of the chain
    (ServoWriteVerifyResult -> BackendWriteOutcome -> ExecuteResult)."""
    by_name = {path.name: (path, code) for path, code in files}

    runtime = by_name.get("ActuatorRuntime.cpp")
    if runtime is None:
        fail(f"{sketch_dir / 'src' / 'actuator' / 'ActuatorRuntime.cpp'}: not found")
    else:
        path, code = runtime
        execute = re.search(r"ExecuteResult ActuatorRuntime::execute\(.*?\n\}", code,
                            re.DOTALL)
        if not execute:
            fail(f"{path}: ActuatorRuntime::execute() not found")
        else:
            text = execute.group(0)
            if "BackendWriteOutcome::UNCERTAIN" not in text or \
                    "ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF" not in text:
                fail(f"{path}: execute() does not map BackendWriteOutcome::UNCERTAIN to "
                     f"ExecuteResult::UNCERTAIN_REQUIRES_SAFE_OFF")

    backend = by_name.get("ServoBusActuatorBackend.h")
    if backend is None:
        fail(f"{sketch_dir / 'src' / 'servo' / 'ServoBusActuatorBackend.h'}: not found")
    else:
        path, code = backend
        if "ServoWriteVerifyResult::UNVERIFIED_NO_RESPONSE" not in code or \
                "BackendWriteOutcome::UNCERTAIN" not in code:
            fail(f"{path}: does not translate "
                 f"ServoWriteVerifyResult::UNVERIFIED_NO_RESPONSE to "
                 f"BackendWriteOutcome::UNCERTAIN - an unverifiable transport-level result "
                 f"must not be able to become a verified one on its way up")

    # The three-value shape itself must not quietly shrink back to a bool
    # anywhere in the chain.
    policy_h = by_name.get("ActuatorRuntime.h")
    if policy_h is not None:
        path, code = policy_h
        iface = re.search(r"class ActuatorBackend\s*\{(.*?)\n\};", code, re.DOTALL)
        if not iface or "BackendWriteOutcome enableTorque" not in iface.group(1) or \
                "BackendWriteOutcome writeGoalPosition" not in iface.group(1):
            fail(f"{path}: ActuatorBackend::enableTorque()/writeGoalPosition() must return "
                 f"BackendWriteOutcome, not bool - a bool cannot express 'uncertain'")


def check_actuator_runtime_boundaries(files):
    """I4: the Safe Actuator runtime adapter (src/actuator/ActuatorRuntime.*)
    must stay host-linkable exactly like ActuatorWritePolicy itself, and its
    mere existence must not make ordinary physical motion reachable. There is
    CR3-M3 adds exactly one production ServoBusActuatorBackend and two narrow
    ServoBus write primitives, but Controller remains null-wired until the
    separately reviewed activation gate.

    2026-09-25 objective change: Controller.{h,cpp} may now own an
    ActuatorRuntime instance as fail-closed status/lifecycle infrastructure
    (it must be wired with a null backend - see
    check_actuator_infrastructure_wired_fail_closed()). Every OTHER file
    stays excluded, in particular CommandRouter.cpp and ControllerService.h:
    neither may reference this class by name, which is what keeps "Controller
    owns one" from silently growing into "a command can reach one"."""
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

    allowed_dirs = {"actuator", "calibration", "tests"}
    allowed_names = {"ActuatorRuntime.h", "ActuatorRuntime.cpp", "Controller.h", "Controller.cpp",
                     "ServoBusActuatorBackend.h", "ServoBusActuatorBackend.cpp"}
    for path, code in files:
        if path.name in allowed_names:
            continue
        if path.parent.name in allowed_dirs:
            continue
        if re.search(r"\bActuatorRuntime\b", code):
            fail(f"{path}: references ActuatorRuntime - I4's runtime adapter has no production "
                 f"backend (ServoBus exposes no torque-on/GoalPosition write) and must not be "
                 f"constructed or referenced outside src/actuator/, src/calibration/, "
                 f"Controller.{{h,cpp}} (fail-closed status infrastructure only) or the offline "
                 f"test suite")


def check_calibration_execution_engine_boundaries(files):
    """I5: the Calibration Execution boundary must stay free of the LF V25
    18-phase sequence (V3 handoff Sec 15.11, binding: historical oracle
    only, never the production architecture) and must never name a
    torque-removal primitive - SAFE_OFF stays outside this layer exactly
    like it stays outside ActuatorWritePolicy and ActuatorRuntime."""
    names = {p.name for p, _ in files}
    for required in ("CalibrationExecutionEngine.h", "CalibrationExecutionEngine.cpp"):
        if required not in names:
            fail(f"{required}: Calibration Execution engine unit not found")

    for path, code in files:
        if path.name not in ("CalibrationExecutionEngine.h", "CalibrationExecutionEngine.cpp"):
            continue
        for token in ("#include <Arduino.h>", "Serial.", "millis(", "ServoBus",
                      "CalibrationPhase", "safeOff", "EnableTorque"):
            if token in code:
                fail(f"{path}: contains {token!r} - the Calibration Execution boundary must "
                     f"stay host-linkable, must never reference the historical LF V25 18-phase "
                     f"sequence as its architecture (V3 handoff Sec 15.11), and must never name "
                     f"a torque-removal primitive (SAFE_OFF stays outside this layer)")

    # Same guarantee I4 enforces for ActuatorRuntime, applied to this class
    # directly: #include hides a transitive ActuatorRuntime reference from a
    # textual scan of Controller.h/.cpp, so the engine itself needs its own
    # boundary check rather than relying on check_actuator_runtime_boundaries()
    # alone. 2026-09-25 objective change: Controller.{h,cpp} may now own an
    # instance as fail-closed status/lifecycle infrastructure (audited by
    # check_actuator_infrastructure_wired_fail_closed()); every other file
    # stays excluded, in particular CommandRouter.cpp and ControllerService.h.
    allowed_dirs = {"calibration", "tests"}
    allowed_names = {"CalibrationExecutionEngine.h", "CalibrationExecutionEngine.cpp",
                     "Controller.h", "Controller.cpp"}
    for path, code in files:
        if path.name in allowed_names:
            continue
        if path.parent.name in allowed_dirs:
            continue
        if re.search(r"\bCalibrationExecutionEngine\b", code):
            fail(f"{path}: references CalibrationExecutionEngine - I5's execution boundary has "
                 f"no production backend behind it and must not be constructed or referenced "
                 f"outside src/calibration/, Controller.{{h,cpp}} (fail-closed status "
                 f"infrastructure only) or the offline test suite")



def check_first_motion_command_wiring(files):
    """CR3 first physical-motion command surface.

    This is deliberately NOT a generic motion API. Exactly one command may
    arm FirstMotionExecutor: LF_UPPER, current semantic identity, bus 12,
    fixed +16 raw-tick excursion. Session start and permit grant are likewise
    exact commands. All physical write execution remains below Controller's
    reviewed FirstMotionExecutor path; CommandRouter itself may never perform
    ServoBus/ActuatorRuntime writes.
    """
    by_name = {path.name: (path, code) for path, code in files}

    router_item = by_name.get("CommandRouter.cpp")
    controller_item = by_name.get("Controller.cpp")
    if router_item is None or controller_item is None:
        fail("CommandRouter.cpp/Controller.cpp missing - cannot audit CR3 first-motion wiring")
        return

    router_path, router = router_item
    controller_path, controller = controller_item

    handle_match = re.search(
        r"void CommandRouter::handleLine\(String line\)\s*\{(.*?)\n\}",
        router, re.DOTALL)
    if not handle_match:
        fail(f"{router_path}: handleLine() not found for CR3 command audit")
        return
    handle = handle_match.group(1)

    def branch_for(command):
        marker = f'"{command}"'
        start = handle.find(marker)
        if start < 0:
            return ""
        end = handle.find("} else if", start + len(marker))
        if end < 0:
            end = len(handle)
        return handle[start:end]

    def branch_for_marker(marker):
        """Branch whose opening condition contains `marker` (whitespace-
        tolerant), up to the next '} else if'."""
        normalized = re.sub(r"\s+", " ", handle)
        needle = re.sub(r"\s+", " ", marker)
        start = normalized.find(needle)
        if start < 0:
            return ""
        end = normalized.find("} else if", start + len(needle))
        if end < 0:
            end = len(normalized)
        return normalized[start:end]

    # ------------------------------------------------------------------
    # Exact first-motion command: no parser, no arbitrary servo/delta.
    # ------------------------------------------------------------------
    motion_cmd = (
        "@CALIBRATION MOTION DIRECTION_VERIFY "
        "LF_UPPER +16 CONFIRM_FIRST_MOTION"
    )
    if handle.count(f'"{motion_cmd}"') != 1:
        fail(f"{router_path}: exact command {motion_cmd!r} must appear exactly once "
             f"in handleLine()")
    if 'upper.startsWith("@CALIBRATION MOTION DIRECTION_VERIFY")' in handle:
        fail(f"{router_path}: DIRECTION_VERIFY may not use startsWith(); "
             f"the first-motion command must remain exact")

    motion = branch_for(motion_cmd)
    if not motion:
        fail(f"{router_path}: exact first-motion command branch not found")
    else:
        required = (
            "constexpr uint8_t kFirstMotionBusId = 12;",
            "identity.leg != calibration::Leg::LF",
            "identity.joint != calibration::JointKind::UPPER",
            "modules_.geometry_profile->withinDirectionVerifyEnvelope(identity, 16)",
            "modules_.calibration->status().leg != calibration::Leg::LF",
            "REASON=ACTIVE_SESSION_IS_NOT_LF",
            "request.bus_id = kFirstMotionBusId;",
            "request.delta_ticks = 16;",
            "modules_.motion_permit->active()",
            "modules_.motion_authorization->operator_authorized",
            "modules_.motion_authorization->token.valid()",
            "modules_.motion_authorization->direction_verify_tick_budget != 16",
            "context.motion_permit_active = modules_.motion_permit->active()",
            "context.authority = modules_.authority->current()",
            "context.authority_generation = modules_.authority->generation()",
            "context.authority_inhibited = modules_.authority->inhibited()",
            "modules_.first_motion->start(request, context, millis())",
        )
        for token in required:
            if token not in motion:
                fail(f"{router_path}: first-motion branch missing pinned token {token!r}")

        for token in ("sscanf(", "strtol(", "atoi(", ".toInt(", ".substring("):
            if token in motion:
                fail(f"{router_path}: first-motion branch contains runtime parser {token!r}; "
                     f"servo/joint/delta must not become caller-selectable")

        for token in ("writeGoalPosition(", "enableTorqueOn(", "safeOff(",
                      "->plan(", "->commit(", "->execute("):
            if token in motion:
                fail(f"{router_path}: first-motion command handler contains direct "
                     f"write/transaction primitive {token!r}; it may only call "
                     f"FirstMotionExecutor::start()")

    start_calls = sum(
        code.count("modules_.first_motion->start(")
        for _, code in files
    )
    if start_calls != 1:
        fail(f"CR3 must contain exactly one production first_motion->start() call; "
             f"found {start_calls}")

    # ------------------------------------------------------------------
    # Exact permit command: shared live fact-builder, ROBOT_POWERED only.
    # ------------------------------------------------------------------
    permit_cmd = "@CALIBRATION MOTION PERMIT GRANT 16 CONFIRM_FIRST_MOTION"
    if handle.count(f'"{permit_cmd}"') != 1:
        fail(f"{router_path}: exact permit grant command must appear exactly once")

    permit = branch_for(permit_cmd)
    if not permit:
        fail(f"{router_path}: exact permit grant branch not found")
    else:
        required = (
            "modules_.motion_authorization->operator_authorized = true;",
            "modules_.motion_authorization->direction_verify_tick_budget = 16;",
            "build::kHardwareProfile == config::HardwareProfile::ROBOT_POWERED",
            "inputs.session_active = modules_.calibration->sessionLive()",
            "inputs.session_id = session.session_id",
            "inputs.current_population_pass =",
            "inputs.current_geometry_bound =",
            "inputs.promoted_transforms_complete =",
            "inputs.authority = modules_.authority->current()",
            "inputs.authority_generation = modules_.authority->generation()",
            "inputs.authority_inhibited = modules_.authority->inhibited()",
            "calibration::buildCalibrationMotionPermitFacts(inputs)",
            "modules_.motion_permit->grant(facts, &token)",
        )
        for token in required:
            if token not in permit:
                fail(f"{router_path}: permit grant branch missing required live gate {token!r}")

        for token in ("sscanf(", "strtol(", "atoi(", ".toInt(", ".substring("):
            if token in permit:
                fail(f"{router_path}: permit grant contains runtime parser {token!r}; "
                     f"the first-motion budget must remain fixed at 16 ticks")

    # ------------------------------------------------------------------
    # Session start must reuse current Q0 population evidence. It is the
    # four-leg command: the leg comes ONLY from the strict matchLegCommand()
    # table (see check_full_leg_calibration_wiring), never from a parser.
    # ------------------------------------------------------------------
    session_marker = ('matchLegCommand(upper, "@CALIBRATION SESSION START ", '
                      '" CONFIRM_CURRENT_Q0"')
    if handle.count('"@CALIBRATION SESSION START ') != 1 or \
            not contains_ws(handle, session_marker):
        fail(f"{router_path}: the session-start command must appear exactly once in "
             f"handleLine(), as the strict four-leg matchLegCommand() match")

    session = branch_for_marker(session_marker)
    if not session:
        fail(f"{router_path}: CR3 session-start branch not found")
    else:
        for token in (
            "Q0CaptureState::COMPLETE",
            "modules_.q0_capture->populationResult()",
            "modules_.actuator_policy->currentGeometryTag()",
            "modules_.actuator_policy->transforms().size()",
            "calibration::startCalibrationSessionFromQ0Evidence(",
            "command_leg",
            "modules_.calibration->status().leg != command_leg",
        ):
            if not contains_ws(session, token):
                fail(f"{router_path}: session-start branch missing required gate {token!r}")

        for token in ("EnableTorque", "writeGoalPosition", "safeOff(",
                      "->plan(", "->commit(", "->execute("):
            if token in session:
                fail(f"{router_path}: session start contains physical write primitive "
                     f"{token!r}")

    # The orchestrator itself must remain pure and cannot secretly grant
    # permits or touch the actuator transport.
    for name in ("CalibrationSessionOrchestrator.h",
                 "CalibrationSessionOrchestrator.cpp"):
        item = by_name.get(name)
        if item is None:
            fail(f"{name}: CR3 session orchestrator unit missing")
            continue
        path, code = item
        for token in ("ServoBus", "FirstMotionExecutor",
                      "CalibrationMotionPermit", "motion_permit",
                      "EnableTorque", "writeGoalPosition", "safeOff("):
            if token in code:
                fail(f"{path}: session orchestrator contains {token!r}; "
                     f"starting a session must never grant or execute motion")

    # ------------------------------------------------------------------
    # Per-tick first-motion continuation + independent SAFE_OFF.
    # ------------------------------------------------------------------
    update = re.search(
        r"void Controller::updateFirstMotion\(uint32_t now_ms\)\s*\{(.*?)\n\}",
        controller, re.DOTALL)
    if not update:
        fail(f"{controller_path}: updateFirstMotion() not found")
    else:
        body = update.group(1)
        required = (
            "context.motion_permit_active = motion_permit_.active()",
            "context.authority = authority_.current()",
            "context.authority_generation = authority_.generation()",
            "context.authority_inhibited = authority_.inhibited()",
            "first_motion_.update(context, now_ms",
            "calibration::FirstMotionState::SAFE_OFF_REQUIRED",
            "calibration::FirstMotionState::COMPLETE",
            "first_motion_safe_off_result_ != servo::SafeOffResult::VERIFIED_OFF",
            "servo_bus_.safeOff(first_motion_.busId())",
        )
        for token in required:
            if not contains_ws(body, token):
                fail(f"{controller_path}: updateFirstMotion() missing safety token {token!r}")

    # The command layer must never turn the final operational flag on.
    if re.search(r"hardware_motion_authorized\s*=\s*true", router):
        fail(f"{router_path}: command surface attempts to enable final global "
             f"hardware_motion_authorized; CR3 calibration permit must remain separate")


def check_full_leg_calibration_wiring(files):
    """Four-leg Full Calibration: command, Controller and finalizer wiring.

    One build must calibrate LF, RF, RH and LH in one session, so the leg has
    to be an input of exactly two strict commands and NOTHING else in the
    path may be typed per leg:
      - matchLegCommand() is a table of exactly the four legs and an equality
        test - no parser, no prefix match, no number;
      - the SESSION START / FULL LEG branches name the leg only as
        `command_leg`; bus ids, identities, MIN/MAX endpoints, the auxiliary
        decision and the re-approach point come from resolveFullLegPlan();
      - exactly one full_leg_calibration->start() exists, armed BEFORE
        full_leg_run->arm() can be reached;
      - Controller closes the evidence lifecycle from ONE pure finalizer, on
        every terminal executor, with the production parameters that are
        pinned UNAPPROVED (no approved stand/gait spec exists), so a hardware
        run can end at HARDWARE_CONTACT_CALIBRATED and never at
        FINAL_OPERATIONAL_ENVELOPE_ACCEPTED;
      - JointLimit admission stays confined to the policy units and that
        finalizer.
    """
    by_name = {path.name: (path, code) for path, code in files}
    needed = ("CommandRouter.cpp", "Controller.cpp", "FullLegCalibrationFinalizer.h",
              "FullLegCalibrationFinalizer.cpp", "FullLegCalibrationPlan.cpp",
              "FullLegCalibrationExecutor.h")
    missing = [n for n in needed if n not in by_name]
    if missing:
        fail(f"four-leg Full Calibration units missing: {missing}")
        return
    router_path, router = by_name["CommandRouter.cpp"]
    controller_path, controller = by_name["Controller.cpp"]
    fin_h_path, fin_h = by_name["FullLegCalibrationFinalizer.h"]
    fin_c_path, fin_c = by_name["FullLegCalibrationFinalizer.cpp"]
    plan_c_path, plan_c = by_name["FullLegCalibrationPlan.cpp"]

    normalize = lambda text: re.sub(r"\s+", " ", text)
    parsers = ("sscanf(", "strtol(", "atoi(", ".toInt(", ".substring(", ".indexOf(",
               ".startsWith(", ".endsWith(")
    writers = ("writeGoalPosition(", "enableTorqueOn(", "EnableTorque", "safeOff(",
               "->plan(", ".plan(", "->commit(", ".commit(", "->execute(", ".execute(")

    # ---- the strict four-token matcher ------------------------------------
    matcher = re.search(r"bool matchLegCommand\(.*?\n\}", router, re.DOTALL)
    if not matcher:
        fail(f"{router_path}: matchLegCommand() not found")
    else:
        body = matcher.group(0)
        entries = re.findall(r'\{"([A-Z]+)",\s*calibration::Leg::([A-Z]+)\}', body)
        if entries != [("LF", "LF"), ("RF", "RF"), ("RH", "RH"), ("LH", "LH")]:
            fail(f"{router_path}: matchLegCommand() table must be exactly "
                 f"LF/RF/RH/LH mapped to their own calibration::Leg, got {entries}")
        if "line == candidate" not in body:
            fail(f"{router_path}: matchLegCommand() must compare the whole line for "
                 f"equality")
        for token in parsers:
            if token in body:
                fail(f"{router_path}: matchLegCommand() contains {token!r}; the leg "
                     f"token must be an exact table match, never parsed")

    handle_match = re.search(
        r"void CommandRouter::handleLine\(String line\)\s*\{(.*?)\n\}", router, re.DOTALL)
    if not handle_match:
        fail(f"{router_path}: handleLine() not found for four-leg command audit")
        return
    handle = normalize(handle_match.group(1))

    if handle.count("matchLegCommand(") != 2:
        fail(f"{router_path}: handleLine() must call matchLegCommand() exactly twice "
             f"(SESSION START, FULL LEG); found {handle.count('matchLegCommand(')}")
    for prefix in ("@CALIBRATION FULL LEG ", "@CALIBRATION SESSION START "):
        if handle.count(f'"{prefix}"') != 1:
            fail(f"{router_path}: the {prefix.strip()!r} prefix must appear exactly once "
                 f"in handleLine()")
        if f'startsWith("{prefix}' in handle:
            fail(f"{router_path}: {prefix.strip()!r} may not use startsWith(); the leg "
                 f"token must stay a strict four-token match")

    def branch(marker):
        needle = normalize(marker)
        start = handle.find(needle)
        if start < 0:
            return ""
        end = handle.find("} else if", start + len(needle))
        return handle[start:end if end >= 0 else len(handle)]

    session = branch('matchLegCommand(upper, "@CALIBRATION SESSION START "')
    full = branch('matchLegCommand(upper, "@CALIBRATION FULL LEG "')
    if not session or not full:
        fail(f"{router_path}: SESSION START / FULL LEG four-leg branches not found")
        return

    # ---- SESSION START: cleanup gates between legs ------------------------
    for token in ("modules_.operating_mode->mode() != OperatingMode::MAINTENANCE",
                  "modules_.system_state->systemHealth() != SystemHealth::READY",
                  "motionExecutorBusy()",
                  "modules_.full_leg_run->armed",
                  "modules_.calibration->sessionLive()",
                  "modules_.motion_permit->active()",
                  "modules_.authority->current() != ActuatorAuthority::NONE",
                  "modules_.motion_authorization->revoke()",
                  "modules_.calibration->status().leg != command_leg",
                  "modules_.calibration->abortSession()"):
        if token not in session:
            fail(f"{router_path}: SESSION START branch missing between-leg gate {token!r}")

    # ---- FULL LEG ----------------------------------------------------------
    for token in ("modules_.calibration->status().state != calibration::SessionState::ACTIVE",
                  "modules_.calibration->status().leg != command_leg",
                  "modules_.motion_permit->active()",
                  "modules_.motion_authorization->operator_authorized",
                  "modules_.motion_authorization->token.valid()",
                  "modules_.first_motion->active() || modules_.full_leg_calibration->active()",
                  "modules_.full_leg_run->armed",
                  "calibration::resolveFullLegPlan(",
                  "actuator::geometry_data::kProvenance",
                  "modules_.actuator_policy->transforms(), command_leg, &plan",
                  "plan_status != calibration::FullLegPlanStatus::OK",
                  "context.session_active = modules_.calibration->sessionLive()",
                  "context.motion_permit_active = modules_.motion_permit->active()",
                  "context.authority = modules_.authority->current()",
                  "context.authority_generation = modules_.authority->generation()",
                  "context.authority_inhibited = modules_.authority->inhibited()",
                  "modules_.full_leg_calibration->start(plan.request, context, millis())",
                  "modules_.full_leg_run->arm(plan,"):
        if token not in full:
            fail(f"{router_path}: FULL LEG branch missing pinned token {token!r}")
    start_at = full.find("modules_.full_leg_calibration->start(")
    arm_at = full.find("modules_.full_leg_run->arm(")
    if start_at < 0 or arm_at < 0 or arm_at < start_at:
        fail(f"{router_path}: FULL LEG must arm the run record only AFTER the executor "
             f"accepted start()")

    for name, text in (("SESSION START", session), ("FULL LEG", full)):
        for token in parsers:
            if token in text:
                fail(f"{router_path}: {name} branch contains runtime parser {token!r}")
        for token in writers:
            if token in text:
                fail(f"{router_path}: {name} branch contains write/transaction "
                     f"primitive {token!r}; it may only call the executor's start()")
        if re.search(r"calibration::Leg::[A-Z]", text):
            fail(f"{router_path}: {name} branch names a leg literal; the leg may only "
                 f"be `command_leg`")
        if "-700000" in text or "findCanonical(" in text or "kFirstMotionBusId" in text:
            fail(f"{router_path}: {name} branch hard-codes a per-leg bus/backoff value; "
                 f"resolveFullLegPlan() is the only source")
        if re.search(r"(?<![\w.])(?:1[1-3]|2[1-3]|3[1-3]|4[1-3]|51)(?![\w.])", text):
            fail(f"{router_path}: {name} branch contains a literal servo bus id")
    if re.search(r"(?:request|plan)\.\w+\s*=[^=]", full):
        fail(f"{router_path}: FULL LEG branch assigns into the plan/request; the plan "
             f"must come whole from resolveFullLegPlan()")
    if "modules_.full_leg_calibration->start(" in session or \
            "modules_.first_motion->start(" in session:
        fail(f"{router_path}: SESSION START may not start an executor")

    starts = sum(code.count("modules_.full_leg_calibration->start(") for _, code in files)
    if starts != 1:
        fail(f"exactly one production full_leg_calibration->start() call is allowed; "
             f"found {starts}")

    # ---- the plan resolver: canonical -> identity -> Geometry V5 ----------
    for token in ("legServoAt(", "semanticIdentityFromCanonical(", "findJoint(",
                  "findEndpoint(", "->bus_id"):
        if token not in plan_c:
            fail(f"{plan_c_path}: resolveFullLegPlan() lost {token!r}; identity and bus "
                 f"must come from the canonical allocation cross-checked with Geometry V5")
    for token in ("Leg::LF", "Leg::RF", "Leg::RH", "Leg::LH", "-700000"):
        if token in plan_c:
            fail(f"{plan_c_path}: resolveFullLegPlan() names {token!r}; nothing in the "
                 f"plan may be special-cased per leg")

    # ---- Controller: parked context + finalization ------------------------
    permit_fn = re.search(r"void Controller::updateCalibrationMotionPermit\(\)\s*\{(.*?)\n\}",
                          controller, re.DOTALL)
    if not permit_fn:
        fail(f"{controller_path}: updateCalibrationMotionPermit() not found")
    else:
        body = permit_fn.group(1)
        for token in ("ctx.parked_leg = full_leg_calibration_.endpointLeg()",
                      "ctx.parked_joint = full_leg_calibration_.endpointJoint()",
                      "ctx.parked_side = calibration::ContactSide::MAX_SIDE",
                      "ctx.auxiliary_parked = full_leg_calibration_.auxiliaryParked()"):
            if not contains_ws(body, token):
                fail(f"{controller_path}: parked-endpoint context lost {token!r}")
        if re.search(r"parked_leg\s*=\s*calibration::Leg::", body):
            fail(f"{controller_path}: parked_leg is hard-coded; it must follow the "
                 f"running request's endpoint")

    step_fn = re.search(r"void Controller::updateFullLegCalibration\(uint32_t now_ms\)\s*\{(.*?)\n\}",
                        controller, re.DOTALL)
    if not step_fn:
        fail(f"{controller_path}: updateFullLegCalibration() not found")
    else:
        body = normalize(step_fn.group(1))
        safe_offs = re.findall(r"safeOff\(\s*([\w.]+\(\))\s*\)", body)
        if safe_offs != ["full_leg_calibration_.primaryBusId()",
                         "full_leg_calibration_.auxiliaryBusId()"]:
            fail(f"{controller_path}: updateFullLegCalibration() must call safeOff() "
                 f"exactly for primaryBusId() then auxiliaryBusId(), got {safe_offs}")
        for token in ("if (full_leg_calibration_.primarySafeOffPending()) {",
                      "if (full_leg_calibration_.auxiliarySafeOffPending()) {"):
            if token not in body:
                fail(f"{controller_path}: SAFE_OFF servicing lost its pending guard "
                     f"{token!r}; an unneeded auxiliary must never be sent SAFE_OFF")

    final_fn = re.search(r"void Controller::updateFullLegFinalization\(\)\s*\{(.*?)\n\}",
                         controller, re.DOTALL)
    if not final_fn:
        fail(f"{controller_path}: updateFullLegFinalization() not found")
    else:
        body = normalize(final_fn.group(1))
        for token in ("if (!full_leg_run_.armed) return;",
                      "FullLegCalibrationPhase::COMPLETE",
                      "FullLegCalibrationPhase::FAILED",
                      "context.manager = &calibration_",
                      "context.policy = &actuator_policy_",
                      "context.geometry = &geometry_profile_",
                      "context.expected_provenance = &actuator::geometry_data::kProvenance",
                      "context.permit = &motion_permit_",
                      "context.authorization = &motion_authorization_",
                      "context.arbiter = &authority_",
                      "context.parameters = calibration::productionEnvelopeParameters()",
                      "calibration::outcomeFromExecutor(",
                      "calibration::finalizeFullLeg(context, full_leg_run_.plan, outcome, &record)",
                      "full_leg_evidence_.put(record)",
                      "full_leg_run_.clear()"):
            if token not in body:
                fail(f"{controller_path}: updateFullLegFinalization() missing {token!r}")
        if body.find("finalizeFullLeg(") > body.find("full_leg_run_.clear()"):
            fail(f"{controller_path}: the run record may be cleared only after it was "
                 f"finalized")
        for token in writers + ("startSession(", ".activate(", "motion_permit_.grant(",
                                ".admit(", "admitOperationalLimit("):
            if token in body:
                fail(f"{controller_path}: updateFullLegFinalization() contains {token!r}; "
                     f"it only hands live collaborators to the pure finalizer")
    update_fn = re.search(r"\nvoid Controller::update\(uint32_t now_ms\)\s*\{(.*?)\n\}",
                          controller, re.DOTALL)
    if not update_fn or not re.search(
            r"updateFullLegCalibration\(now_ms\);.*?updateFullLegFinalization\(\);",
            update_fn.group(1), re.DOTALL):
        fail(f"{controller_path}: update() must call updateFullLegFinalization() right "
             f"after updateFullLegCalibration(now_ms)")

    # ---- finalizer: pure, ordered, unapproved -----------------------------
    if normalize(fin_h).count("constexpr bool kFullLegOperationalParametersApproved = false;") != 1:
        fail(f"{fin_h_path}: kFullLegOperationalParametersApproved must be declared "
             f"exactly once and be false - no approved stand/gait specification exists, "
             f"so the production envelope parameters are placeholders and a hardware run "
             f"ends at HARDWARE_CONTACT_CALIBRATED, never FINAL_OPERATIONAL_ENVELOPE_ACCEPTED")
    if "parameters.approved = kFullLegOperationalParametersApproved;" not in normalize(fin_c):
        fail(f"{fin_c_path}: productionEnvelopeParameters() must take `approved` from "
             f"kFullLegOperationalParametersApproved")
    if re.search(r"approved\s*=\s*true", fin_c) or re.search(r"approved\s*=\s*true", fin_h):
        fail(f"{fin_c_path}: envelope parameters must never be marked approved in code")
    for path, code in ((fin_h_path, fin_h), (fin_c_path, fin_c)):
        for token in ("ServoBus", "servo::", "Arduino.h", "Serial", "EnableTorque",
                      "writeGoalPosition(", "enableTorqueOn(", "safeOff(", "millis("):
            if token in code:
                fail(f"{path}: the finalizer contains {token!r}; it is a pure decision "
                     f"unit that never touches the bus")
    order = ("manager.recordContact(outcome.min_contact)",
             "manager.recordContact(outcome.max_contact)",
             "actuator::buildContactDerivedEnvelope(",
             "policy.validateOperationalLimit(",
             "policy.admitOperationalLimit(",
             "manager.noteExecutionPhase(CalibrationPhase::TORQUE_OFF)",
             "manager.completeSession()",
             "context.permit->revoke(",
             "context.authorization->revoke()")
    fin_norm = normalize(fin_c)
    last = -1
    for token in order:
        at = fin_norm.find(token)
        if at < 0:
            fail(f"{fin_c_path}: evidence lifecycle lost {token!r}")
            break
        if at < last:
            fail(f"{fin_c_path}: evidence lifecycle out of order at {token!r}: contacts "
                 f"-> envelopes -> limit validate -> admit -> completeSession -> "
                 f"permit/authorization revoke")
        last = at
    if fin_c.count("admitOperationalLimit(") != 1:
        fail(f"{fin_c_path}: the finalizer must have exactly one admitOperationalLimit() "
             f"call site")

    # ---- JointLimit admission confinement ---------------------------------
    allowed = {"ActuatorWritePolicy.h", "ActuatorWritePolicy.cpp",
               "FullLegCalibrationFinalizer.cpp"}
    for path, code in files:
        if "scripts" in path.parts:
            continue
        if "admitOperationalLimit(" in code and path.name not in allowed:
            fail(f"{path}: admitOperationalLimit() may only be called from the "
                 f"policy units and FullLegCalibrationFinalizer.cpp")


def check_service_readiness_is_host_linkable(files):
    """I6: the HostLink readiness classifier must stay pure and
    host-linkable, the same contract as every other decision core in this
    codebase - no module pointer, no hardware call, no transport."""
    names = {p.name for p, _ in files}
    for required in ("ServiceReadiness.h", "ServiceReadiness.cpp"):
        if required not in names:
            fail(f"{required}: HostLink readiness classifier unit not found")

    for path, code in files:
        if path.name not in ("ServiceReadiness.h", "ServiceReadiness.cpp"):
            continue
        for token in ("#include <Arduino.h>", "Serial.", "millis(", "ServoBus",
                      "CommandRouter", "ControllerService"):
            if token in code:
                fail(f"{path}: contains {token!r} - keep the HostLink readiness classifier "
                     f"pure and host-linkable; module/transport wiring belongs in "
                     f"ControllerService, not here")


def check_actuator_infrastructure_wired_fail_closed(files):
    """I4/I5/CR3 production composition.

    The backend, Geometry V5, execution engine and permit core are now REAL
    production objects. Fail-closed therefore no longer means "motion code is
    unreachable by absence". CR3 intentionally exposes exactly one reviewed
    activation chain:

      completed current Q0 evidence
        -> live CalibrationManager session
        -> explicit RAM-only motion permit
        -> exact LF_UPPER +16 DIRECTION_VERIFY first-motion request

    This function pins Controller-owned composition and the per-tick permit
    refresh. check_first_motion_command_wiring() separately pins the ONLY
    reviewed command-level exceptions. Controller itself must never implicitly
    grant a permit or start a session merely because prerequisites become
    healthy."""

    by_name = {path.name: (path, code) for path, code in files}
    controller_cpp = by_name.get("Controller.cpp")
    if controller_cpp is None:
        fail("Controller.cpp not found - cannot audit fail-closed actuator wiring")
        return
    path, code = controller_cpp

    # --- (1) the transaction methods remain unreachable from any command ---
    # unchanged from CR3-M3/M4: still the single strongest guarantee, kept
    # exactly as strict now that the infrastructure behind it is real.
    # Matches BOTH call syntaxes - modules_.actuator_policy is a pointer
    # (-> ), while a hypothetical value member would use '.' - a dot-only
    # check silently misses every real call site in this codebase, which is
    # exactly the gap a manual mutation check (temporarily injecting
    # modules_.actuator_policy->plan(...) into CommandRouter.cpp) caught
    # during the original I4/I5 Controller-wiring review.
    # CR3 continuation: FirstMotionExecutor::abort() and
    # FullLegCalibrationExecutor::abort() are the reviewed, session-scoped,
    # operator-facing de-escalation primitives the
    # @CALIBRATION MOTION|SESSION|FULL LEG ABORT command handlers call -
    # check_first_motion_command_wiring() separately pins their ONE
    # production start() call site each. This is deliberately NOT the same
    # door as SafeActuatorPolicy::abort(ActuatorTransaction*) /
    # ActuatorRuntime, which remains forbidden below: the receiver name is
    # checked, not just the method name, so only these two exact, already-
    # reviewed objects' abort() is exempt.
    allowed_abort_receivers = ("first_motion", "full_leg_calibration")
    forbidden_call = re.compile(r"(\w+)\s*(?:\.|->)\s*(plan|commit|execute|abort)\s*\(")
    for path2, code2 in files:
        if path2.name not in ("CommandRouter.cpp", "ControllerService.h", "Controller.cpp"):
            continue
        found = None
        for m in forbidden_call.finditer(code2):
            receiver, method = m.group(1), m.group(2)
            if method == "abort" and receiver in allowed_abort_receivers:
                continue
            found = m
            break
        if found:
            fail(f"{path2}: contains a call to {found.group(2)}() - CommandRouter/ControllerService/"
                 f"Controller may only read status from the Safe Actuator/Calibration "
                 f"Execution infrastructure, never plan, commit, execute or abort a "
                 f"transaction; that belongs to a future, separately reviewed activation "
                 f"gate")

    # --- the backend and geometry must be the REAL production ones --------
    m = re.search(r"actuator_runtime_\.begin\(([^)]*)\)", code)
    if not m or re.sub(r"\s+", "", m.group(1)) != "&actuator_policy_,&actuator_backend_":
        fail(f"{path}: actuator_runtime_.begin() must be given the real "
             f"&actuator_policy_, &actuator_backend_ - CR3-M5 production composition "
             f"requires a real backend to exist (guarantee (1) above is what keeps that safe, "
             f"not the backend's absence)")
    if code.count("actuator_backend_.begin(&servo_bus_)") != 1:
        fail(f"{path}: actuator_backend_ must be bound to the one Controller-owned "
             f"servo_bus_ exactly once")
    if code.count("geometry_profile_.bind(&actuator::geometry_data::kProvenance") != 1:
        fail(f"{path}: geometry_profile_ must be bound to the real generated Geometry V5 "
             f"data (actuator::geometry_data) exactly once")
    if code.count("actuator_policy_.bindGeometry(&geometry_profile_, "
                 "&actuator::geometry_data::kProvenance)") != 1:
        fail(f"{path}: actuator_policy_ must be bound to geometry_profile_/the real expected "
             f"provenance exactly once")
    if code.count("calibration_execution_.begin(&actuator_policy_, &actuator_runtime_, "
                 "&geometry_profile_,") != 1:
        fail(f"{path}: calibration_execution_ must be bound to the same real geometry_profile_")

    # --- Controller never implicitly authorises calibration motion ----------
    # Explicit grant/session start now exist ONLY in CommandRouter and are
    # pinned by check_first_motion_command_wiring(). The periodic Controller
    # lifecycle may check/revoke an existing permit, never create one.
    if re.search(r"motion_authorization_\.operator_authorized\s*=\s*true", code):
        fail(f"{path}: Controller must never set operator_authorized=true implicitly; "
             f"only the exact reviewed CommandRouter permit command may do so")
    if "motion_permit_.grant(" in code:
        fail(f"{path}: Controller must never call motion_permit_.grant(); "
             f"per-tick lifecycle may check/revoke only")

    for forbidden in ("calibration_.startSession(",
                      "calibration_.activate(",
                      "calibration_.submitPopulationEvidence("):
        if forbidden in code:
            fail(f"{path}: Controller contains {forbidden!r}; live-session start is "
                 f"allowed only through the reviewed Q0 session orchestrator command")

    # --- the permit refresh itself must be real, not a stub -----------------
    refresh = re.search(r"void Controller::updateCalibrationMotionPermit\(\)\s*\{(.*?)\n\}",
                        code, re.DOTALL)
    if not refresh:
        fail(f"{path}: updateCalibrationMotionPermit() not found")
    else:
        body = refresh.group(1)
        for required in (
            "inputs.operator_calibration_motion_authorized = motion_authorization_.operator_authorized",
            "inputs.session_active = calibration_.sessionLive()",
            "inputs.current_geometry_bound =",
            "actuator_policy_.currentGeometryTag() != actuator::kNoGeometryProvenance",
            "inputs.promoted_transforms_complete =",
            "actuator_policy_.transforms().size() == calibration::kLegServoSlotCount",
            "inputs.authority = authority_.current()",
            "inputs.authority_generation = authority_.generation()",
            "inputs.authority_inhibited = authority_.inhibited()",
            "calibration::buildCalibrationMotionPermitFacts(inputs)",
            "motion_permit_.check(facts, motion_authorization_.token)",
            "ctx.motion_permit_active = motion_permit_.active()",
            "actuator_policy_.setBootstrapContext(ctx)",
        ):
            if required not in body:
                fail(f"{path}: updateCalibrationMotionPermit() missing required live fact "
                     f"{required!r} - every fact must be read fresh from its real source, "
                     f"never hardcoded or cached")
        if re.search(r"(?:facts|inputs)\.\w+\s*=\s*true\b", body):
            fail(f"{path}: updateCalibrationMotionPermit() hardcodes a fact to true instead "
                 f"of reading it from live state")
        if "motion_permit_.grant(" in body:
            fail(f"{path}: updateCalibrationMotionPermit() must never grant() - it may only "
                 f"check() an already-granted permit; granting implicitly from facts turning "
                 f"healthy is exactly what CalibrationMotionPermit.h forbids")

    # updateCalibrationMotionPermit() must run unconditionally, every tick,
    # before command_router_.update() can act on what it just computed.
    update_fn = re.search(r"void Controller::update\(uint32_t now_ms\)\s*\{(.*?)\n\}",
                          code, re.DOTALL)
    if not update_fn:
        fail(f"{path}: Controller::update(uint32_t now_ms) not found")
    else:
        body = update_fn.group(1)
        permit_pos = body.find("updateCalibrationMotionPermit();")
        router_pos = body.find("command_router_.update(now_ms);")
        if permit_pos < 0 or router_pos < 0 or permit_pos > router_pos:
            fail(f"{path}: Controller::update() must call updateCalibrationMotionPermit() "
                 f"unconditionally, before command_router_.update(now_ms) - a command "
                 f"processed against a stale permit snapshot is exactly the hazard this "
                 f"orders against")


def check_led_status_boundaries(files):
    """LED V2: pure cached-fact policy, one presenter, one transport owner.

    Textual regression tripwires complement real linked policy/driver tests;
    scripts/tests is excluded because those tests deliberately call the API.
    """
    production = [(p, c) for p, c in files if "tests" not in p.parts]
    by_name = {p.name: (p, c) for p, c in production}
    for required in ("LedStatusPolicy.h", "LedStatusPolicy.cpp", "LedStatusManager.h",
                     "LedStatusManager.cpp", "LedRing.h", "LedRing.cpp"):
        if required not in by_name:
            fail(f"{required}: LED presentation unit not found")

    for path, code in production:
        if path.name in ("LedStatusPolicy.h", "LedStatusPolicy.cpp"):
            for token in ("Arduino.h", "Serial", "millis(", "LedRing", "DalyBms",
                          "ControllerService", "pinMode(", "digitalWrite(", "analogWrite("):
                if token in code:
                    fail(f"{path}: {token!r} breaks the pure LED policy boundary")
        if path.parent.name == "status":
            for token in ("DalyBms", "bms_uart_", "ServoBus", "EEPROM", "PowerStateMachine",
                          "EnableTorque", "GoalPosition", "requestKey", "requestDischarge",
                          "esp_ota_", "WiFi."):
                if token in code:
                    fail(f"{path}: {token!r} gives LED presentation a hardware/control side effect")
            if re.search(r"\b(?:delay|delayMicroseconds)\s*\(", code):
                fail(f"{path}: LED presentation/diagnostics must remain non-blocking")
        if path.name not in ("LedStatusManager.cpp", "LedRing.cpp", "LedRing.h") and \
                re.search(r"\b(?:setSolid|setFrame|renderFrame)\s*\(", code):
            fail(f"{path}: direct LED rendering bypasses the single presentation owner")
        if path.name not in ("LedRing.h", "LedRing.cpp") and \
                re.search(r"Adafruit_NeoPixel|pixels_\.|\b(?:rmtWrite|neopixelWrite)\s*\(", code):
            fail(f"{path}: WS2812 access belongs only to LedRing")
        # Reserve the fact without creating a current producer in any layer.
        if path.name not in ("LedStatusPolicy.h", "LedStatusPolicy.cpp") and \
                re.search(r"\bcharge_complete_verified\s*=",
                          re.sub(r'"(?:\\.|[^"\\])*"', '""', code)):
            fail(f"{path}: charge_complete_verified has no reviewed production producer")

    policy_h = by_name.get("LedStatusPolicy.h", (None, ""))[1]
    if len(re.findall(r"bool charge_complete_verified\s*=\s*false\s*;", policy_h)) != 2 or re.search(
            r"charge_complete_verified\s*\(\s*(?!false\b)", policy_h):
        fail("LedStatusPolicy.h: reserved verified-full fact must default false")

    # These facts have no battery-policy owner today. Pin their entire
    # production vocabulary, including read sites, so a new constructor,
    # alias, compound assignment, or policy-side threshold cannot quietly
    # become a producer. Only the two false defaults, exact passthrough,
    # selector read, and read-only @LED STATUS observation are permitted.
    for fact, state in (("battery_warning", "BATTERY_WARNING"),
                        ("battery_critical", "BATTERY_CRITICAL")):
        declaration = rf"\bbool\s+{fact}\s*=\s*false\s*;"
        for struct in ("LedStatusInputs", "LedStatusSnapshot"):
            body = re.search(rf"struct\s+{struct}\s*\{{(.*?)\n\}};", policy_h, re.DOTALL)
            if not body or len(re.findall(declaration, body.group(1))) != 1:
                fail(f"LedStatusPolicy.h: {struct}.{fact} must default false")
        for path, code in production:
            # Ignore format-string labels, but require their presence below.
            remaining = re.sub(r'"(?:\\.|[^"\\])*"', '""', code)
            if path.name == "LedStatusPolicy.h":
                remaining, count = re.subn(declaration, "", remaining)
                if count != 2:
                    fail(f"{path}: {fact} must have exactly two false defaults")
            elif path.name == "LedStatusPolicy.cpp":
                for allowed in (rf"\bfacts\.{fact}\s*=\s*in\.{fact}\s*;",
                                rf"\bif\s*\(facts\.{fact}\)\s*return\s+"
                                rf"LedPresentationState::{state}\s*;"):
                    remaining, count = re.subn(allowed, "", remaining)
                    if count != 1:
                        fail(f"{path}: {fact} requires one exact passthrough and selector read")
            elif path.name == "CommandRouter.cpp":
                remaining, count = re.subn(rf'\bled\.{fact}\s*\?\s*""\s*:\s*""',
                                          "", remaining)
                if count != 1 or f"{fact}=%s" not in code:
                    fail(f"{path}: @LED STATUS must observe reserved {fact}")
            if re.search(rf"\b{fact}\b", remaining):
                fail(f"{path}: {fact} has no reviewed production producer; "
                     "only false defaults, exact cached passthrough and presentation reads are allowed")

    manager_path, manager = by_name.get("LedStatusManager.cpp", (None, ""))
    if manager.count("ring_->setFrame(frame)") != 1:
        fail(f"{manager_path}: manager must have exactly one frame submission")
    policy_pos = manager.find("policy_.update(inputs, now_ms, LedRing::kMaxBrightness)")
    guard = re.search(r"if\s*\(ring_ == nullptr \|\| ring_->testRunning\(\)\)\s*return;", manager)
    frame_pos = manager.find("ring_->setFrame(frame)")
    if policy_pos < 0 or not guard or not (policy_pos < guard.start() < frame_pos):
        fail(f"{manager_path}: update cached facts, then yield to diagnostics, then render")

    ctl_path, ctl = by_name.get("Controller.cpp", (None, ""))
    # Preserve default construction at the sole production input owner.
    # Aggregate construction/assignment or an alias of the whole object
    # could otherwise turn on a reserved fact without naming the field.
    input_uses = ctl
    for allowed in (r"\bstatus::LedStatusInputs\s+led_inputs\s*;",
                    r"\bled_status_\.update\(led_now_ms,\s*led_inputs\);"):
        input_uses, count = re.subn(allowed, "", input_uses)
        if count != 1:
            fail(f"{ctl_path}: LED inputs must be default-constructed once and passed directly to the manager")
    if re.search(r"\bled_inputs\b(?!\s*\.)|\bLedStatusInputs\b", input_uses):
        fail(f"{ctl_path}: LED inputs must not gain aggregate initialization, replacement or aliases")
    if len(re.findall(r"\bdaly_\.update\(", ctl)) != 1:
        fail(f"{ctl_path}: DALY must keep its one existing scheduler update")
    if len(re.findall(r"\bled_status_\.update\(", ctl)) != 1:
        fail(f"{ctl_path}: LED manager must have one periodic caller")
    for call in re.findall(r"\bdaly_\.(\w+)\s*\(", ctl):
        if call not in {"begin", "update", "availability", "health", "sample", "lastCommResult"}:
            fail(f"{ctl_path}: DALY {call}() is outside cached LED inputs/existing scheduler")
    for required in ("led_inputs.sample_valid = battery.valid",
                     "led_inputs.daly_comm_ok = daly_.lastCommResult() == power::DalyCommResult::OK",
                     "const uint32_t led_now_ms = millis()",
                     "led_inputs.telemetry_age_ms = led_now_ms - battery.sampled_at_ms",
                     "led_inputs.soc_percent = battery.soc_percent",
                     'led_inputs.battery_charging = strcmp(battery.state_name, "CHARGING") == 0'):
        if required not in ctl:
            fail(f"{ctl_path}: missing reviewed cached telemetry input: {required}")
    alarm = re.search(r"led_inputs\.battery_alarm\s*=([^;]+);", ctl)
    if not alarm or any(f"battery.alarms[{i}] != 0" not in alarm.group(1) for i in range(4)):
        fail(f"{ctl_path}: charging fault must consume all four cached alarm words")

    protocol = by_name.get("DalyProtocol.h", (None, ""))[1]
    if not re.search(r"kDalyTelemetryFreshnessMs\s*=\s*5000\s*;", protocol) or not re.search(
            r"kDalyKeyWriteMaxTelemetryAgeMs\s*=\s*kDalyTelemetryFreshnessMs\s*;", protocol):
        fail("DalyProtocol.h: LED and existing KEY age contract must share reviewed 5000 ms bound")


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
                 "CalibrationManager.h", "CalibrationManager.cpp",
                 "CalibrationPopulationEvidence.h", "CalibrationPopulationEvidence.cpp",
                 "CalibrationQ0CaptureSession.h", "CalibrationQ0CaptureSession.cpp"):
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


def check_calibration_population_evidence(files, sketch_dir):
    """CR1 formal leg-population producer stays pure, derived and fail closed."""
    by_name = {path.name: (path, code) for path, code in files}
    header = by_name.get("CalibrationPopulationEvidence.h")
    source = by_name.get("CalibrationPopulationEvidence.cpp")
    if header is None or source is None:
        fail(f"{sketch_dir / 'src' / 'calibration'}: CR1 population evidence producer missing")
        return

    for path, code in (header, source):
        for forbidden in ("#include <Arduino.h>", "ServoBus", "ServoCensus", "startScan(",
                          "EnableTorque", "WritePos", "GoalPosition", "Serial."):
            if forbidden in code:
                fail(f"{path}: CR1 producer contains {forbidden!r} - it must consume existing "
                     f"structured evidence and never own transport or hardware")

    path, code = source
    for required in (
        "current_observation_bundle",
        "census.scan_lo <= servo::kCanonicalScanLo",
        "census.scan_hi >= servo::kCanonicalScanHi",
        "census.unexpected_id",
        "census.absent_by_design_present",
        "census.missing_id_count == census.missing_expected",
        "census.unexpected_id_count == census.unexpected_id",
        "census.absent_by_design_present_id_count ==",
        "preflight.pass_count != servo::kLegPreflightCount",
        "preflight.no_response_count != 0",
        "preflight.mismatch_count != 0",
        "preflight.incomplete_count != 0",
        "servo::isLegServo(census.missing_ids[i])",
        "record.result != servo::JointPreflightResult::PASS",
        "record.model != servo::profile_data::kInvariants.model_expected",
        "record.profile != servo::ProfileVerdict::MATCH",
        "record.position_offset != 0",
        "record.torque_enable != 0",
        "record.present_position < kRawTickMin",
        "record.present_position > kRawTickMax",
        "semanticIdentityFromCanonical",
        "setPhysicalUnit",
        "legSlotIndex",
        "populationIsCurrentPass",
    ):
        if required not in code:
            fail(f"{path}: CR1 producer lost required formal-population gate {required!r}")

    domain = by_name.get("CalibrationDomain.cpp")
    if domain is None or "evidence.unexpected_count != 0" not in domain[1]:
        fail(f"{sketch_dir / 'src' / 'calibration' / 'CalibrationDomain.cpp'}: "
             f"domain PASS no longer rejects anomalous population evidence")

    # CR2-B is now the ONE reviewed production freshness owner. No parser,
    # Controller body or other module may manufacture a current bundle directly.
    # Mask only CR1's own public declaration; an inline/header call anywhere
    # else must still be treated as a production call site.
    for path, code in files:
        if "scripts" in path.parts or path.name == "CalibrationPopulationEvidence.cpp":
            continue
        inspected = code
        if path.name == "CalibrationPopulationEvidence.h":
            inspected = re.sub(
                r"PopulationEvidenceBuildResult\s+buildCurrentLegPopulationEvidence\s*"
                r"\([^;]*\);",
                "",
                inspected,
                flags=re.DOTALL,
            )
        if "buildCurrentLegPopulationEvidence(" not in inspected:
            continue
        if path.name != "CalibrationQ0CaptureSession.cpp":
            fail(f"{path}: calls the CR1 producer outside the reviewed CR2-B same-session "
                 f"orchestrator; cached/independent diagnostics must never self-declare current")

def check_calibration_q0_capture_session(files, sketch_dir):
    """CR2-B same-session acquisition stays pure, current and candidate-only."""
    by_name = {path.name: (path, code) for path, code in files}
    header = by_name.get("CalibrationQ0CaptureSession.h")
    source = by_name.get("CalibrationQ0CaptureSession.cpp")
    if header is None or source is None:
        fail(f"{sketch_dir / 'src' / 'calibration'}: CR2-B q0 capture session missing")
        return

    for path, code in (header, source):
        for forbidden in ("#include <Arduino.h>", "ServoBus", "HardwareSerial",
                          "Serial.", "millis(", "CalibrationManager", "ActuatorAuthority",
                          "hardwareMotionAuthorized", "EnableTorque", "WritePos",
                          "GoalPosition", "SyncWrite", "RegWrite", "CalibrationOfs",
                          "PositionOffset", "EvidenceState::PROMOTED", ".admit("):
            if forbidden in code:
                fail(f"{path}: CR2-B contains {forbidden!r} - evidence acquisition must stay "
                     f"transport/authority blind and candidate-only")

    path, code = source
    for required in (
        "buildCurrentLegPopulationEvidence",
        "context.current_observation_bundle = true",
        "populationIsCurrentPass(population_.evidence)",
        "servo::legServoAt",
        "semanticIdentityFromCanonical",
        "profile_.findJoint(identity)",
        "geometry->bus_id != canonical->bus_id",
        "actuator::geometry_data::kProvenance",
        "actuator::buildQ0BootstrapCandidate",
        "Q0BootstrapStatus::CANDIDATE",
        "status_.next_joint_index++",
        "status_.next_joint_index >= kLegServoSlotCount",
        "status_.completed_sample_passes++",
        "status_.completed_sample_passes >= config_.samples_per_joint",
        "observation.torque_enable != 0",
        "observation.raw_tick < kRawTickMin",
        "observation.raw_tick > kRawTickMax",
    ):
        if required not in code:
            fail(f"{path}: CR2-B lost required same-session/q0 gate {required!r}")

    # The only legitimate producer of current-observation truth in CR2-B is
    # this session after it has itself sequenced census then preflight.
    if code.count("current_observation_bundle = true") != 1:
        fail(f"{path}: CR2-B must stamp current_observation_bundle exactly once, only after "
             f"its own census+preflight sequence")

def check_calibration_q0_production_wiring(files, sketch_dir):
    """CR2-B production wiring is read-only, single-owner and authority-free."""
    by_name = {path.name: (path, code) for path, code in files}
    controller_h = by_name.get("Controller.h")
    controller_cpp = by_name.get("Controller.cpp")
    router_h = by_name.get("CommandRouter.h")
    router_cpp = by_name.get("CommandRouter.cpp")
    service_h = by_name.get("ControllerService.h")
    if any(x is None for x in (controller_h, controller_cpp, router_h, router_cpp, service_h)):
        fail(f"{sketch_dir / 'src' / 'core'}: CR2-B production wiring source missing")
        return

    # Exactly one coordinator instance, owned by Controller. Tests may create
    # locals; production must not gain a second session state machine.
    owners = []
    for path, code in files:
        if "scripts" in path.parts:
            continue
        for m in re.finditer(r"CalibrationQ0CaptureSession\s+(\w+)\s*[;{]", code):
            owners.append((path.name, m.group(1)))
    if owners != [("Controller.h", "q0_capture_")]:
        fail(f"CR2-B coordinator instances={owners!r}; expected exactly Controller.h:q0_capture_")

    # Controller orchestration may start the existing census/preflight and
    # take one existing runtime READ. Nothing in this function may acquire
    # actuator authority, enter a motion session or write a servo.
    cpath, ccode = controller_cpp
    body = re.search(r"void Controller::updateQ0Capture\(\)\s*\{(.*?)\n\}",
                     ccode, re.DOTALL)
    if not body:
        fail(f"{cpath}: updateQ0Capture() not found")
    else:
        text = body.group(1)
        for required in (
            "OperatingMode::MAINTENANCE",
            "servo_census_.start()",
            "servo_preflight_.start()",
            "servo_bus_.readRuntimeState",
            "Q0CaptureFailure::MODE_NOT_MAINTENANCE",
            "Q0CaptureFailure::CENSUS_START_REFUSED",
            "Q0CaptureFailure::PREFLIGHT_START_REFUSED",
        ):
            if required not in text:
                fail(f"{cpath}: CR2-B Controller wiring lost {required!r}")
        for forbidden in (
            "safeOff(", "EnableTorque", "WritePos", "GoalPosition", "SyncWrite",
            "RegWrite", "PositionOffset", "CalibrationOfs", "authority_.request",
            "calibration_.startSession", "calibration_.activate", ".plan(",
            ".commit(", ".execute(",
        ):
            if forbidden in text:
                fail(f"{cpath}: updateQ0Capture() contains {forbidden!r} - q0 acquisition "
                     f"must remain read-only and authority-free")
        if text.count("servo_bus_.readRuntimeState(") != 1:
            fail(f"{cpath}: updateQ0Capture() must contain exactly one runtime-read call site")
        if re.search(r"\b(?:for|while)\s*\(", text):
            fail(f"{cpath}: updateQ0Capture() gained a loop - CR2-B must advance at most one "
                 f"runtime q0 observation per Controller tick")

    # Never automatic at boot. Controller may own/update it only after an
    # explicit command starts the transaction.
    begin = re.search(r"void Controller::begin\(\)\s*\{(.*?)\n\}", ccode, re.DOTALL)
    if begin and re.search(r"q0_capture_\s*\.\s*start\s*\(", begin.group(1)):
        fail(f"{cpath}: q0 capture starts from Controller::begin() - hardware diagnostics must "
             f"never auto-start at boot")

    rpath, rcode = router_cpp

    # CAPTURE: explicit physical/USB command, maintenance-only, powered-profile
    # only, exact pose confirmation, and refuses any competing servo diagnostic.
    capture = re.search(
        r'upper\.startsWith\("@CALIBRATION Q0 CAPTURE"\)\s*\)\s*\{(.*?)\}\s*else',
        rcode, re.DOTALL)
    if not capture:
        fail(f"{rpath}: @CALIBRATION Q0 CAPTURE branch not found")
    else:
        text = capture.group(1)
        for required in ("OperatingMode::MAINTENANCE", "build::kServoPowerAvailable",
                         "CONFIRM_Q0_POSE", "servoDiagnosticBusy()",
                         "modules_.q0_capture->start(config)"):
            if required not in text:
                fail(f"{rpath}: q0 capture command lost gate {required!r}")
        for forbidden in ("authority->request", "startSession(", "activate(", "safeOff(",
                          "EnableTorque", "GoalPosition", "WritePos", "PositionOffset",
                          "CalibrationOfs"):
            if forbidden in text:
                fail(f"{rpath}: q0 capture command contains {forbidden!r} - read-only capture "
                     f"must not acquire motion authority or write hardware")

    # STATUS must remain cached presentation only.
    printer = re.search(r"void CommandRouter::printCalibrationQ0Status\(\)\s*\{(.*?)\n\}",
                        rcode, re.DOTALL)
    if not printer:
        fail(f"{rpath}: printCalibrationQ0Status() not found")
    else:
        for forbidden in ("readRuntimeState", "servo_bus", "startScan(", "safeOff("):
            if forbidden in printer.group(1):
                fail(f"{rpath}: q0 STATUS contains {forbidden!r} - status is cached only")

    # CR2-B reserves ordinary servo diagnostics so no other read/scan is
    # interleaved into its evidence bundle. Inspect the actual parser branches,
    # not nearby comments/help text.
    diagnostic_branches = (
        ("@SERVO SCAN",
         r'upper\.startsWith\("@SERVO SCAN"\)\s*\)\s*\{(.*?)\}\s*else'),
        ("@SERVO CENSUS",
         r'upper == "@SERVO CENSUS"\s*\)\s*\{(.*?)\}\s*else'),
        ("@SERVO PREFLIGHT",
         r'upper == "@SERVO PREFLIGHT"\s*\)\s*\{(.*?)\}\s*else'),
        ("@SERVO READ",
         r'upper\.startsWith\("@SERVO READ"\)\s*\)\s*\{(.*?)\}\s*else'),
    )
    for command, pattern in diagnostic_branches:
        branch = re.search(pattern, rcode, re.DOTALL)
        if not branch:
            fail(f"{rpath}: could not locate {command} branch for CR2-B audit")
        elif "CALIBRATION_Q0_CAPTURE_ACTIVE" not in branch.group(1):
            fail(f"{rpath}: {command} is not refused while CR2-B owns servo diagnostics")

    # SAFE_OFF is the deliberate exception: a de-escalation must stay
    # reachable even during a read-only capture and must not consult it.
    safe = re.search(r'upper\.startsWith\("@SERVO SAFE_OFF"\)\s*\)\s*\{(.*?)\}\s*else',
                     rcode, re.DOTALL)
    if not safe:
        fail(f"{rpath}: @SERVO SAFE_OFF branch not found for CR2-B audit")
    else:
        for forbidden in ("q0_capture", "Q0Capture", "CALIBRATION_Q0"):
            if forbidden in safe.group(1):
                fail(f"{rpath}: SAFE_OFF consults CR2-B via {forbidden!r} - de-escalation "
                     f"must remain independent")


def check_calibration_q0_promotion_wiring(files, sketch_dir):
    """CR3-M5: @CALIBRATION Q0 PROMOTE is the one reviewed path that may call
    transforms().admit() in production. MAINTENANCE-gated, exact-confirmation-
    gated, refused while a capture/session/run/motion is live, and it promotes
    the CURRENT-BOOT capture (prepareFreshQ0Evidence over freshCapture()) - never
    the frozen CR2-C package, which stays a regression oracle. It acquires no
    authority, issues no bus transaction and writes no EEPROM. SESSION START
    refuses unless the table holds exactly that capture's q0."""
    by_name = {path.name: (path, code) for path, code in files}
    router = by_name.get("CommandRouter.cpp")
    if router is None:
        fail(f"{sketch_dir / 'src' / 'core' / 'CommandRouter.cpp'}: not found")
        return
    rpath, rcode = router

    branch = re.search(
        r'upper == "@CALIBRATION Q0 PROMOTE CONFIRM_CURRENT_INSTALLATION"\)\s*\{(.*?)'
        r'\}\s*else if\s*\(matchLegCommand\(upper,\s*"@CALIBRATION SESSION START "',
        rcode, re.DOTALL)
    if not branch:
        fail(f"{rpath}: @CALIBRATION Q0 PROMOTE branch not found (exact confirmation text "
             f"required, immediately followed by the @CALIBRATION SESSION START branch)")
    else:
        text = branch.group(1)
        for required in ("OperatingMode::MAINTENANCE", "prepareFreshQ0Evidence(",
                         "modules_.q0_capture->freshCapture()",
                         "modules_.q0_capture->active()", "motionExecutorBusy()",
                         "modules_.full_leg_run->armed", "modules_.calibration->sessionLive()",
                         "prepared.ready()", "transforms().admit("):
            if required not in text:
                fail(f"{rpath}: @CALIBRATION Q0 PROMOTE lost required gate {required!r}")
        for forbidden in ("prepareCurrentQ0Evidence(", "q0_evidence_data",
                          "CalibrationQ0EvidenceData", "authority->request",
                          "authority_->request", "startSession(",
                          "activate(", "safeOff(", "EnableTorque", "GoalPosition",
                          "WritePos", "PositionOffset", "CalibrationOfs",
                          ".plan(", ".commit(", ".execute("):
            if forbidden in text:
                fail(f"{rpath}: @CALIBRATION Q0 PROMOTE contains {forbidden!r} - transform "
                     f"promotion must not acquire authority, start a session or touch "
                     f"hardware")

    # transforms().admit(/limits().admit( may be called ONLY from this one
    # branch, anywhere in the production tree (tests are exempt - they drive
    # SafeActuatorPolicy directly, the same contract as every other suite).
    admit_call = re.compile(r"(?:transforms|limits)\(\)\.admit\(")
    for path, code in files:
        if "scripts" in path.parts and path.name != "CommandRouter.cpp":
            continue
        if path.name == "CommandRouter.cpp":
            remaining = code.replace(branch.group(1), "") if branch else code
            if admit_call.search(remaining):
                fail(f"{rpath}: transforms()/limits().admit( appears outside the one "
                     f"reviewed @CALIBRATION Q0 PROMOTE branch")
            continue
        if admit_call.search(code):
            fail(f"{path}: transforms()/limits().admit( is only permitted inside "
                 f"CommandRouter.cpp's @CALIBRATION Q0 PROMOTE branch")

    # The frozen CR2-C package is a regression oracle: nothing in the production
    # tree except the preparation unit that owns it may consume it.
    frozen_owner = {"CalibrationQ0EvidencePreparation.h", "CalibrationQ0EvidencePreparation.cpp",
                    "CalibrationQ0EvidenceData.h"}
    for path, code in files:
        if "scripts" in path.parts or path.name in frozen_owner:
            continue
        for token in ("prepareCurrentQ0Evidence(", "q0_evidence_data::"):
            if token in code:
                fail(f"{path}: {token!r} - the frozen CR2-C q0 package is a historical "
                     f"regression oracle and must not be consumed by production code; "
                     f"promote the current-boot capture with prepareFreshQ0Evidence()")

    # SESSION START may only run on the q0 that the current capture produced.
    handle = re.search(r'matchLegCommand\(upper,\s*"@CALIBRATION SESSION START "(.*?)'
                       r'\}\s*else if', rcode, re.DOTALL)
    if not handle:
        fail(f"{rpath}: SESSION START four-leg branch not found for the fresh-q0 gate")
    else:
        for required in ("freshQ0CaptureIsPromoted(", "modules_.q0_capture->freshCapture()",
                         "currentGeometryTag()", "CURRENT_BOOT_Q0_NOT_PROMOTED"):
            if required not in handle.group(1):
                fail(f"{rpath}: SESSION START lost the fresh-q0 promotion gate {required!r}")

    # The preparation unit stays pure: it reduces data already in RAM.
    prep = [(path, code) for path, code in files
            if path.name in ("CalibrationQ0EvidencePreparation.h",
                             "CalibrationQ0EvidencePreparation.cpp")]
    if len(prep) != 2:
        fail(f"{sketch_dir / 'src' / 'actuator'}: CalibrationQ0EvidencePreparation unit missing")
    for path, code in prep:
        for forbidden in ("#include <Arduino.h>", "ServoBus", "HardwareSerial", "Serial.",
                          "millis(", "readRuntimeState(", "EnableTorque", "WritePos",
                          "GoalPosition", "SyncWrite", "RegWrite", "CalibrationOfs",
                          "PositionOffset", "unLockEprom", "LockEprom", "EEPROM",
                          "Preferences", "nvs_", "CalibrationQ0CaptureSession",
                          "authority->request", ".admit("):
            if forbidden in code:
                fail(f"{path}: q0 preparation contains {forbidden!r} - it must stay a pure "
                     f"reducer with no transport, EEPROM, authority or table mutation")


def check_calibration_q0_bootstrap(files, sketch_dir):
    """CR2 read-only q0 candidate builder stays pure and non-operational."""
    by_name = {path.name: (path, code) for path, code in files}
    header = by_name.get("CalibrationQ0Bootstrap.h")
    source = by_name.get("CalibrationQ0Bootstrap.cpp")
    if header is None or source is None:
        fail(f"{sketch_dir / 'src' / 'actuator'}: CR2 q0 bootstrap unit missing")
        return

    for path, code in (header, source):
        for forbidden in ("#include <Arduino.h>", "#include \"../servo/ServoBus.h\"",
                          "HardwareSerial", "Serial.", "millis(", "readRuntimeState(",
                          "EnableTorque", "WritePos", "GoalPosition", "SyncWrite",
                          "RegWrite", "CalibrationOfs", "PositionOffset", "unLockEprom",
                          "LockEprom"):
            if forbidden in code:
                fail(f"{path}: CR2 q0 bootstrap contains {forbidden!r} - it must reduce "
                     f"already-read evidence and must never own transport, motion or EEPROM")

    path, code = source
    for required in (
        "populationIsCurrentPass(population)",
        "profile.provenanceMatches(expected_provenance)",
        "profile.provenanceTag() == kNoGeometryProvenance",
        "profile.findJoint(request.identity)",
        "joint->bus_id != request.bus_id",
        "request.nominal_zero_pose_confirmed",
        "!request.stability_budget_specified",
        "sample_count < kQ0BootstrapMinSamples",
        "sample_count > kQ0BootstrapMaxSamples",
        "!sample.read_ok",
        "sample.torque_enable != 0",
        "sample.raw_tick < kRawMin",
        "sample.raw_tick > kRawMax",
        "spread > request.max_stability_spread_ticks",
        "Q0Estimator::MANUAL_ZERO_POSE",
        "EvidenceState::CANDIDATE",
        "CalibrationOrigin::LIVE_SESSION",
        "accepted_by_gate = false",
    ):
        if required not in code:
            fail(f"{path}: CR2 lost required q0 bootstrap gate {required!r}")

    # Candidate construction must not be able to promote or install itself.
    for forbidden in ("EvidenceState::PROMOTED", "JointTransformTable", ".admit("):
        if forbidden in code:
            fail(f"{path}: CR2 q0 bootstrap contains {forbidden!r} - CR2 ends at CANDIDATE; "
                 f"CR3 owns acceptance/promotion and transform admission")

    # 2048 may be recorded as a diagnostic distance only, never used to decide
    # whether the captured q0 is acceptable.
    if re.search(r"if\s*\([^)]*kServoRawCenter", code):
        fail(f"{path}: CR2 gates candidate acceptance against raw centre 2048 - the bootstrap "
             f"contract requires measure first, derive the plausibility window later")

    # CR2-B is the sole reviewed production consumer. Hardware-facing
    # Controller/parser/servo code must never call the reducer directly.
    # Mask only CR2-A's public prototype, not arbitrary header code.
    for path, code in files:
        if "scripts" in path.parts or path.name == "CalibrationQ0Bootstrap.cpp":
            continue
        inspected = code
        if path.name == "CalibrationQ0Bootstrap.h":
            inspected = re.sub(
                r"Q0BootstrapCandidate\s+buildQ0BootstrapCandidate\s*"
                r"\([^;]*\);",
                "",
                inspected,
                flags=re.DOTALL,
            )
        if "buildQ0BootstrapCandidate(" not in inspected:
            continue
        if path.name != "CalibrationQ0CaptureSession.cpp":
            fail(f"{path}: calls CR2-A outside the reviewed CR2-B acquisition coordinator")

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


def check_calibration_readiness_contract(sketch_dir):
    """CR0 - calibration-readiness repository contract.

    This is deliberately a documentation/source-truth audit, not a motion
    implementation. It prevents the three ambiguities closed on 2026-09-27
    from silently returning while all existing actuator fail-closed checks
    continue to enforce zero production write reachability:

      1. motorDirection is current URDF/Geometry V5 contract data;
      2. Full Calibration does not require the 16 beyond-URDF hip/lower
         diagnostic contacts to become executable;
      3. future calibration-motion permission is distinct from final
         hardware_motion_authorized.

    Historical development logs and CHANGELOG are intentionally not scanned:
    they are records of what was believed at their date, not current contracts.
    """
    required_docs = {
        "CALIBRATION_READINESS.md": sketch_dir / "CALIBRATION_READINESS.md",
        "DEVELOPMENT_GATES.md": sketch_dir / "DEVELOPMENT_GATES.md",
        "CALIBRATION_BOOTSTRAP.md": sketch_dir / "CALIBRATION_BOOTSTRAP.md",
        "CALIBRATION_SOURCE_PRECEDENCE.md": sketch_dir / "CALIBRATION_SOURCE_PRECEDENCE.md",
    }
    docs = {}
    for name, path in required_docs.items():
        if not path.is_file():
            fail(f"{path}: CR0 current-contract document missing")
            continue
        docs[name] = path.read_text(encoding="utf-8")

    readiness = docs.get("CALIBRATION_READINESS.md", "")
    for token, why in (
        ("CR2-B SAME-SESSION READ-ONLY ORCHESTRATION: IMPLEMENTED / OFFLINE + BUILD VALIDATED",
         "same-session q0 orchestration passed offline/build validation but remains hardware-unvalidated"),
        ("@CALIBRATION Q0 CAPTURE <samples 3..32> <stability_ticks 0..2047> CONFIRM_Q0_POSE",
         "the read-only q0 capture command must remain explicit and pose-confirmed"),
        ("motorDirection  = current URDF / hardware-contract data",
         "production direction must remain bound to the current URDF/geometry contract"),
        ("8  upper-leg endpoints  EXECUTABLE_URDF_DOMAIN",
         "the executable V5 endpoint population must remain explicit"),
        ("16 hip/lower endpoints  DIAGNOSTIC_GEOMETRY_OUTSIDE_URDF_LIMITS",
         "the beyond-URDF diagnostic population must remain explicit"),
        ("CALIBRATION_MOTION_PERMIT", "the separate calibration-motion concept must stay named"),
        ("!= hardware_motion_authorized",
         "calibration motion must not collapse back into final operational authorization"),
        ("remains false throughout readiness work",
         "CR0 must not authorize normal motion as part of readiness"),
    ):
        if token not in readiness:
            fail(f"{required_docs['CALIBRATION_READINESS.md']}: missing CR0 invariant {token!r} - {why}")

    # CR2-B is now implemented. Current contracts must not regress to
    # describing the same-session Controller orchestration as future work.
    for stale in (
        "CR2-B SAME-SESSION READ-ONLY ORCHESTRATION: TO_IMPLEMENT",
        "CR2-B same-session read-only Controller orchestration remains TO_IMPLEMENT",
        "same-session Controller read orchestration remains **TO_IMPLEMENT**",
    ):
        for name in ("CALIBRATION_READINESS.md", "DEVELOPMENT_GATES.md",
                     "CALIBRATION_BOOTSTRAP.md"):
            if stale in docs.get(name, ""):
                fail(f"{required_docs[name]}: stale CR2-B readiness state {stale!r}")

    # Current-contract docs must not regress to the superseded pre-9aae03d
    # direction model. Exact historical logs are intentionally left untouched.
    gates = docs.get("DEVELOPMENT_GATES.md", "")
    for stale in (
        "direction witnesses",
        "direction measurement (",
        "accepted q0/direction transform",
    ):
        if stale in gates:
            fail(f"{required_docs['DEVELOPMENT_GATES.md']}: stale CR0 direction contract {stale!r}")

    bootstrap = docs.get("CALIBRATION_BOOTSTRAP.md", "")
    for stale in (
        "current direction verification            TO_IMPLEMENT",
        "current q0/direction",
    ):
        if stale in bootstrap:
            fail(f"{required_docs['CALIBRATION_BOOTSTRAP.md']}: stale CR0 direction prerequisite {stale!r}")
    for required in (
        "current motorDirection from URDF/V5       REUSED / CONTRACT DATA",
        "8 upper endpoints are `REQUIRED_FOR_FINAL_CALIBRATION`",
        "a 12-joint direction campaign",
    ):
        if required not in bootstrap:
            fail(f"{required_docs['CALIBRATION_BOOTSTRAP.md']}: missing CR0 bootstrap evidence {required!r}")

    precedence = docs.get("CALIBRATION_SOURCE_PRECEDENCE.md", "")
    if "`MEASURED_CANDIDATE` and `ACCEPTED` are **TO_IMPLEMENT**" in precedence:
        fail(f"{required_docs['CALIBRATION_SOURCE_PRECEDENCE.md']}: legacy DirectionEvidence "
             f"vocabulary is still presented as a production TO_IMPLEMENT prerequisite")
    for required in (
        "production no longer needs one",
        "current URDF/Geometry V5 contract",
        "check_direction_is_contractual()",
    ):
        if required not in precedence:
            fail(f"{required_docs['CALIBRATION_SOURCE_PRECEDENCE.md']}: missing CR0 source-precedence "
                 f"statement {required!r}")

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


def check_http_transport_boundaries(files, sketch_dir):
    """I7/I8 permanent invariants for the network transport (2026-09-25
    correction).

    HttpTransport is the CONTROL/AUTHORIZATION plane in front of the one
    existing OtaManager/OtaPolicy/OtaEspBackend writer, never a second
    writer of its own. These checks defend the properties that design rests
    on, so none of them can be lost to a later "small" edit:

      1. Hmac256/OtaSession stay pure and host-linkable, so the offline
         suites (test_hmac256.cpp, test_ota_session.cpp) drive the REAL
         authentication logic, not a copy;
      2. the ESP-IDF HTTP server API is confined to ONE translation unit,
         the same "one auditable unit" rule check_ota_boundaries already
         gives the ESP-IDF OTA API;
      3. the listening socket is never opened from Controller::begin() -
         it stays MAINTENANCE-gated, reachable only through @WEB SERVER;
      4. it never reboots on its own - a committed OTA image waits for a
         separate, explicit step, exactly like the reviewed manual
         application-only flash path;
      5. the shared secret follows the exact same one-place,
         never-committed discipline as the Wi-Fi passphrase (W1).
    """
    by_name = {path.name: (path, code) for path, code in files}
    update_dir = sketch_dir / "src" / "update"
    network_dir = sketch_dir / "src" / "network"

    # --- (1) Hmac256/OtaSession/HttpMailbox stay host-linkable --------------
    host_linkable_units = {
        "Hmac256.h": update_dir, "Hmac256.cpp": update_dir,
        "OtaSession.h": update_dir, "OtaSession.cpp": update_dir,
        # HttpMailbox.h alone: the mailbox correlation logic (I7/I8
        # hardening, 2026-09-25) is small enough to stay header-only, so
        # there is no matching .cpp to require here.
        "HttpMailbox.h": network_dir,
    }
    for name, unit_dir in host_linkable_units.items():
        entry = by_name.get(name)
        if entry is None:
            fail(f"{unit_dir / name}: I7/I8 authentication/session/mailbox unit not found")
            continue
        path, code = entry
        for forbidden in ("#include <Arduino.h>", "#include <esp_http_server.h>",
                          "#include <WiFi.h>", "Serial."):
            if forbidden in code:
                fail(f"{path}: contains {forbidden!r} - the OTA authentication/session/"
                     f"mailbox-correlation layers must stay free of the Arduino runtime and "
                     f"of the transport so scripts/tests/test_hmac256.cpp, "
                     f"test_ota_session.cpp and test_http_mailbox.cpp link the REAL logic "
                     f"(I7/I8, the same contract as WifiPolicy/OtaPolicy)")

    # --- (2) the ESP-IDF HTTP server API lives in exactly one unit ---------
    httpd_api_prefixes = ("httpd_start", "httpd_stop", "httpd_register_uri_handler",
                          "httpd_req_recv", "httpd_req_get_hdr_value_str",
                          "httpd_req_get_hdr_value_len", "httpd_resp_send")
    for path, code in files:
        if path.name == "HttpTransport.cpp":
            continue
        hits = [sym for sym in httpd_api_prefixes if sym in code]
        if hits:
            fail(f"{path}: calls ESP-IDF HTTP server API {hits} outside "
                 f"network/HttpTransport.cpp - the listening socket and every handler must "
                 f"stay in one auditable translation unit (I7/I8)")

    # --- (3) never started from Controller::begin() ------------------------
    ctl = by_name.get("Controller.cpp")
    if ctl is None:
        fail(f"{sketch_dir / 'src' / 'core' / 'Controller.cpp'}: not found - cannot audit "
             f"HTTP transport startup")
    else:
        path, code = ctl
        begin_body = re.search(r"void Controller::begin\(\)\s*\{(.*?)\n\}", code, re.DOTALL)
        if begin_body and "http_transport_.start()" in begin_body.group(1):
            fail(f"{path}: Controller::begin() starts the HTTP transport - the listening "
                 f"socket must stay off at boot, reachable only from the MAINTENANCE-gated "
                 f"@WEB SERVER START command (I7/I8)")

    # @WEB SERVER START must be gated the same way every other diagnostic
    # that can be triggered without hardware authorization already is.
    router = by_name.get("CommandRouter.cpp")
    if router is None:
        fail(f"{sketch_dir / 'src' / 'core' / 'CommandRouter.cpp'}: not found - cannot audit "
             f"the @WEB SERVER command gate")
    else:
        path, code = router
        if '"@WEB SERVER START"' not in code:
            fail(f"{path}: @WEB SERVER START command not found - the HTTP transport must be "
                 f"reachable from the command surface, not only from source")
        else:
            # Bounded to the WEB SERVER branch itself - the next "} else if
            # (upper ==" marks the start of an unrelated command's branch, so
            # this must not just search a fixed character window forward,
            # which previously kept matching the NEXT command's own
            # MAINTENANCE check instead of this one's (caught by manual
            # mutation: deleting this branch's gate still passed until this
            # was bounded to the branch).
            branch = re.search(
                r'"@WEB SERVER START".*?\{(.*?)(?=\}\s*else\s+if\s*\(upper\b)',
                code, re.DOTALL)
            if not branch or "OperatingMode::MAINTENANCE" not in branch.group(1):
                fail(f"{path}: @WEB SERVER START does not check "
                     f"OperatingMode::MAINTENANCE inside its own branch - starting the "
                     f"listening socket must stay MAINTENANCE-gated, the same trust "
                     f"boundary as the DALY KEY write and the servo scan/census/preflight "
                     f"commands")

    # --- (4) never reboots itself -------------------------------------------
    transport = by_name.get("HttpTransport.cpp")
    if transport is not None:
        path, code = transport
        if "esp_restart(" in code:
            fail(f"{path}: calls esp_restart() - a committed OTA image must wait for a "
                 f"separate, explicit reboot step, never one the network transport takes "
                 f"on its own (I7)")

        # --- (4b) bounded START -> STOP resource lifecycle (I7/I8 hardening,
        # 2026-09-25): every semaphore created must be matched by exactly one
        # delete, so a repeated start()/stop() cycle cannot leak handles.
        # Counting textual occurrences is a coarse proxy, but a genuine leak
        # (a fourth CreateBinary with no matching Delete, or vice versa) can
        # only make these counts diverge, never coincidentally match.
        creates = len(re.findall(r"xSemaphoreCreateBinary\s*\(", code))
        deletes = len(re.findall(r"vSemaphoreDelete\s*\(", code))
        if creates == 0:
            fail(f"{path}: no xSemaphoreCreateBinary() call found - expected the "
                 f"request/response/slot-free mailbox semaphores")
        elif creates != deletes:
            fail(f"{path}: {creates} xSemaphoreCreateBinary() call(s) but {deletes} "
                 f"vSemaphoreDelete() call(s) - every semaphore start() creates must be "
                 f"released by stop(), or a bounded START -> STOP -> START -> STOP cycle "
                 f"leaks a handle (I7/I8 hardening)")

        # stop() must be reachable from every partial-failure path in
        # start() - not just from an explicit @WEB SERVER STOP - or a
        # failed httpd_start()/CreateBinary() attempt leaks whatever it did
        # allocate. Counted within start()'s own body only.
        start_body = re.search(r"bool HttpTransport::start\(\)\s*\{(.*?)\n\}", code, re.DOTALL)
        if not start_body:
            fail(f"{path}: could not locate HttpTransport::start() to audit its cleanup")
        else:
            stop_calls_in_start = len(re.findall(r"\bstop\(\)", start_body.group(1)))
            if stop_calls_in_start < 2:
                fail(f"{path}: HttpTransport::start() calls stop() {stop_calls_in_start} "
                     f"time(s) - expected at least 2 (an upfront defensive call, plus at "
                     f"least one partial-failure cleanup path) - a failed start() attempt "
                     f"must release whatever it already allocated (I7/I8 hardening)")

    # --- (5) the OTA shared secret: one use site, never committed ----------
    creds = by_name.get("OtaCredentials.h")
    if creds is None:
        fail(f"{sketch_dir / 'src' / 'config' / 'OtaCredentials.h'}: OTA credential resolver "
             f"not found")
    else:
        path, code = creds
        if "kOtaSecretPresent" not in code:
            fail(f"{path}: kOtaSecretPresent not found - an absent secret must be a "
                 f"compile-time fact so OtaSession fails closed rather than authenticating "
                 f"against an empty key")
        if 'define MATDOG_OTA_SECRET ""' not in code:
            fail(f"{path}: the empty-secret fallback is missing - a checkout with no OTA "
                 f"secret configured must still build and boot, with every OTA request "
                 f"rejected")

    # Word-boundary match: kOtaSecretPresent is a separate, harmless
    # compile-time boolean and must not be counted as a use of the secret
    # itself. Unlike kWifiPassword (passed once to WiFi.begin()), the raw
    # secret legitimately appears twice at its one call site - once as bytes
    # and once via strlen() for its length - so the invariant enforced here
    # is "only at that one call site", not "exactly once textually".
    secret_re = re.compile(r"\bkOtaSecret\b")
    secret_sites = []
    for path, code in files:
        if path.name == "OtaCredentials.h":
            continue
        n = len(secret_re.findall(code))
        if n:
            secret_sites.append((str(path), n))
    other_files = [s for s in secret_sites if pathlib.Path(s[0]).name != "Controller.cpp"]
    if other_files:
        fail(f"kOtaSecret is referenced outside core/Controller.cpp: {other_files} - it must "
             f"appear only at the one HttpTransport::begin() call site. It must never be "
             f"stored, returned or printed (I7)")
    ctl_entry = by_name.get("Controller.cpp")
    if ctl_entry is not None:
        path, code = ctl_entry
        n = len(secret_re.findall(code))
        begin_call = re.search(r"http_transport_\.begin\(([^;]*)\)", code, re.DOTALL)
        if n == 0:
            fail(f"{path}: kOtaSecret is not referenced - HttpTransport::begin() must be "
                 f"given the configured secret (I7)")
        elif not begin_call or len(secret_re.findall(begin_call.group(1))) != n:
            fail(f"{path}: kOtaSecret is referenced {n} time(s) outside the "
                 f"http_transport_.begin() call - every use must be at that one call site, "
                 f"passed straight through, never stored, returned or printed (I7)")

    session_files = [(p, c) for n, (p, c) in by_name.items()
                     if n in ("OtaSession.h", "OtaSession.cpp")]
    for path, code in session_files:
        for token in ("secret", "password", "passphrase"):
            # "secret" itself is expected (it is the parameter name); this
            # instead looks for it ever being formatted, logged or exposed
            # through a status/printf-shaped call, which none of these files
            # should ever contain regardless.
            if "printf" in code.lower() and token in code.lower():
                fail(f"{path}: mentions {token!r} near a printf-shaped call - the OTA session "
                     f"layer must never format or expose the secret (I7)")

    gitignore = sketch_dir / ".gitignore"
    local_rel = "src/config/OtaCredentials.local.h"
    if not gitignore.exists():
        fail(f"{gitignore}: not found - it must ignore {local_rel}")
    else:
        rules = [ln.strip() for ln in gitignore.read_text(encoding="utf-8").splitlines()]
        rules = [ln for ln in rules if ln and not ln.startswith("#")]
        if local_rel not in rules:
            fail(f"{gitignore}: has no ignore rule for {local_rel} (active rules: {rules}) - "
                 f"removing that entry makes a real OTA secret committable (I7)")

    template = sketch_dir / "src" / "config" / "OtaCredentials.local.h.example"
    if not template.exists():
        fail(f"{template}: credential template not found - it is the documented way to "
             f"configure OTA authentication without touching a tracked file")

    result = subprocess.run(["git", "-C", str(sketch_dir), "ls-files", "--error-unmatch",
                             local_rel],
                            capture_output=True, text=True)
    if result.returncode == 0:
        fail(f"{local_rel} is TRACKED by Git - a real OTA secret must never be committed. "
             f"Run: git rm --cached {local_rel}")


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
        sketch_dir / "scripts" / "tests" / "test_calibration_population_evidence.cpp",
        sketch_dir / "scripts" / "tests" / "test_calibration_q0_bootstrap.cpp",
        sketch_dir / "scripts" / "tests" / "test_calibration_q0_capture_session.cpp",
        sketch_dir / "scripts" / "tests" / "test_calibration_manager.cpp",
        sketch_dir / "scripts" / "tests" / "test_calibration_session_orchestrator.cpp",
    ]
    led_status_suite = sketch_dir / "scripts" / "tests" / "test_led_status_policy.cpp"
    led_driver_suite = sketch_dir / "scripts" / "tests" / "test_led_ring_manager.cpp"
    actuator_runtime_suite = sketch_dir / "scripts" / "tests" / "test_actuator_runtime.cpp"
    calibration_execution_suite = (
        sketch_dir / "scripts" / "tests" / "test_calibration_execution_engine.cpp"
    )
    service_readiness_suite = sketch_dir / "scripts" / "tests" / "test_service_readiness.cpp"
    hmac256_suite = sketch_dir / "scripts" / "tests" / "test_hmac256.cpp"
    ota_session_suite = sketch_dir / "scripts" / "tests" / "test_ota_session.cpp"
    http_mailbox_suite = sketch_dir / "scripts" / "tests" / "test_http_mailbox.cpp"
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
    if not led_driver_suite.exists():
        fail(f"{led_driver_suite}: real LED driver/manager host suite not found")
        return
    if not actuator_runtime_suite.exists():
        fail(f"{actuator_runtime_suite}: Safe Actuator runtime adapter offline test suite not found")
        return
    if not calibration_execution_suite.exists():
        fail(f"{calibration_execution_suite}: Calibration Execution boundary offline test suite "
             f"not found")
        return
    if not service_readiness_suite.exists():
        fail(f"{service_readiness_suite}: HostLink readiness offline test suite not found")
        return
    if not hmac256_suite.exists():
        fail(f"{hmac256_suite}: HMAC-SHA256 offline test suite not found (I7)")
        return
    if not ota_session_suite.exists():
        fail(f"{ota_session_suite}: OTA transport session/authentication offline test suite "
             f"not found (I7)")
        return
    if not http_mailbox_suite.exists():
        fail(f"{http_mailbox_suite}: HTTP transport mailbox correlation offline test suite "
             f"not found (I7/I8)")
        return
    if not runner.exists():
        fail(f"{runner}: host test runner not found")
        return
    runner_text = strip_shell_comments(runner.read_text(encoding="utf-8"))
    for binary in ("test_servo_population", "test_daly_protocol", "test_wifi_policy",
                   "test_ota_policy", "test_actuator_authority", "test_actuator_write_policy",
                   "test_calibration_geometry", "test_servo_profile",
                   "test_calibration_domain", "test_calibration_population_evidence",
                   "test_calibration_q0_bootstrap", "test_calibration_q0_capture_session",
                   "test_calibration_manager", "test_calibration_session_orchestrator",
                   "test_led_status_policy", "test_actuator_runtime",
                   "test_calibration_execution_engine", "test_service_readiness",
                   "test_hmac256", "test_ota_session", "test_http_mailbox"):
        if f'"$OUT/{binary}"' not in runner_text:
            fail(f"{runner}: does not run {binary} - every offline suite must gate")
    for profile in ("USB_ONLY", "ROBOT_POWERED"):
        if f'"$OUT/test_led_ring_manager_{profile}"' not in runner_text:
            fail(f"{runner}: real LED driver/manager suite must run for {profile}")
    for source in ("LedStatusManager.cpp", "LedRing.cpp"):
        if source not in runner_text:
            fail(f"{runner}: LED integration suite must link real {source}")
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


def check_led_audit_mutation_suite(sketch_dir):
    """Proves LED tripwires and linked policy tests reject targeted mutations."""
    suite = sketch_dir / "scripts" / "tests" / "test_static_audit_led.py"
    if not suite.exists():
        fail(f"{suite}: LED mutation suite not found")
        return
    result = subprocess.run([sys.executable, str(suite)], capture_output=True, text=True)
    if result.returncode != 0:
        fail(f"{suite}: LED mutation tests FAILED "
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
                  "SOURCE_COMMIT_MISMATCH", "TREE_NOT_CLEAN",
                  "KNOWN_OTA_INGEST_VALUES", "OTA_INGEST_MISMATCH", "OTA_INGEST_UNKNOWN"):
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

    # Same reasoning, for the OTA-ingest authorization axis (I7 hardening,
    # 2026-09-25): a binary with the firmware-ingest writer compiled in must
    # never be flashed by an invocation that did not explicitly ask for it.
    if re.search(r"def verify_manifest\([^)]*requested_ota_ingest\s*=", logic_text, re.DOTALL):
        fail(f"{logic}: verify_manifest() gained a default for requested_ota_ingest - the "
             f"caller must always state the OTA-ingest state it intends to flash")
    if re.search(r'add_argument\("--requested-ota-ingest"[^)]*default=', logic_text):
        fail(f"{logic}: --requested-ota-ingest gained a default - flash_app_only.sh must "
             f"pass it explicitly")
    if not re.search(r'manifest_ota_ingest\s*!=\s*requested_ota_ingest', logic_text):
        fail(f"{logic}: the OTA_INGEST_ENABLED equality comparison against "
             f"requested_ota_ingest is missing - recording the ingest state without "
             f"comparing it proves nothing")

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
    if "--ota-ingest" not in build_text:
        fail(f"{build_sh}: does not pass --ota-ingest to the manifest writer - a binary "
             f"with the OTA firmware-ingest writer compiled in could be produced with no "
             f"record of that fact")
    if "MATDOG_OTA_INGEST_VALIDATION:-" not in build_text:
        fail(f"{build_sh}: lost the MATDOG_OTA_INGEST_VALIDATION override input (expected "
             f"a ${{MATDOG_OTA_INGEST_VALIDATION:-...}} expansion) - the ONE hardware-"
             f"validation candidate needs an explicit, loud way to compile ingest in "
             f"without editing the source default")
    if "-DMATDOG_OTA_INGEST_ENABLED=1" not in build_text:
        fail(f"{build_sh}: the OTA-ingest override no longer passes "
             f"-DMATDOG_OTA_INGEST_ENABLED=1 to the compiler - the override input would be "
             f"read but never reach the build")

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
    if "--requested-ota-ingest" not in flash_text:
        fail(f"{flash_sh}: does not pass --requested-ota-ingest to the manifest verifier - "
             f"an ingest-enabled binary could be flashed with no explicit authorization "
             f"check")
    if not re.search(r'--expected-fqbn\s+"\$FQBN"', flash_text):
        fail(f"{flash_sh}: does not pass --expected-fqbn \"$FQBN\" to the manifest "
             f"verifier - the recorded build FQBN would go unverified")
    # The parameter expansion, not just the name in prose: the operator
    # authorization input must actually be readable from the environment.
    if "MATDOG_FLASH_PROFILE:-" not in flash_text:
        fail(f"{flash_sh}: lost the MATDOG_FLASH_PROFILE operator authorization input "
             f"(expected a ${{MATDOG_FLASH_PROFILE:-...}} expansion)")
    if "MATDOG_FLASH_OTA_INGEST:-" not in flash_text:
        fail(f"{flash_sh}: lost the MATDOG_FLASH_OTA_INGEST operator authorization input "
             f"(expected a ${{MATDOG_FLASH_OTA_INGEST:-...}} expansion)")

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
            # 2026-09-25 recovery-backup hardening moved the literal
            # size/digest comparison into scripts/backup_gate_logic.py
            # (host-tested, mutation-verified) - the anti-weakening
            # property here is now "the real measured size/sha256 are
            # actually passed to that gate", not a bash [ ] comparison.
            (r'--actual-size\s+"\$BACKUP_SIZE"',
             "full-flash backup size passed to the backup gate"),
            (r'--actual-sha256\s+"\$BACKUP_SHA256"',
             "full-flash backup digest passed to the backup gate"),
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


def check_backup_gate_provenance(sketch_dir):
    """Recovery-backup hardening (2026-09-25): flash_app_only.sh's backup
    gate must PROVE a full-flash backup is authorized, never accept one by
    path or size alone.

    The historical default backup's hash is pinned once, reviewed, in
    scripts/backup_gate_logic.py. Any other ("custom") backup path is
    accepted only together with an explicitly authorized expected SHA256 -
    this check makes the pinned hash's exact value, and the requirement
    that a custom backup supply its own, both audit-enforced rather than
    trusted to review alone.
    """
    scripts_dir = sketch_dir / "scripts"
    logic = scripts_dir / "backup_gate_logic.py"
    flash_sh = scripts_dir / "flash_app_only.sh"
    tests = scripts_dir / "tests" / "test_backup_gate_logic.py"

    if not logic.exists():
        fail(f"{logic}: backup gate logic module not found - the flash path would have "
             f"no way to prove a full-flash backup is authorized")
        return
    logic_text = logic.read_text(encoding="utf-8")

    for token in ("DEFAULT_BACKUP_SHA256", "EXPECTED_BACKUP_SIZE", "verify_backup",
                  "NO_EXPECTED_HASH", "SHA256_MISMATCH", "SIZE_MISMATCH"):
        if token not in logic_text:
            fail(f"{logic}: missing required backup-gate primitive {token!r}")

    # The ONE historical backup's hash, reviewed once. A silent edit here
    # would let a different backup pass as "the" trusted default without
    # any of the explicit-authorization ceremony a custom backup requires.
    if 'DEFAULT_BACKUP_SHA256 = "5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32"' \
            not in logic_text:
        fail(f"{logic}: DEFAULT_BACKUP_SHA256 no longer matches the reviewed 2026-09-10 "
             f"historical backup hash - this constant must never change silently")

    if 'EXPECTED_BACKUP_SIZE = 16777216' not in logic_text:
        fail(f"{logic}: EXPECTED_BACKUP_SIZE is no longer the full 16 MiB flash size")

    # verify_backup() must not acquire a permissive default that would make
    # a custom backup's missing authorization silently pass.
    if re.search(r"def verify_backup\([^)]*custom_expected_sha256\s*=\s*(?!None)",
                logic_text, re.DOTALL):
        fail(f"{logic}: verify_backup() gained a non-None default for "
             f"custom_expected_sha256 - a custom backup with no stated hash must reach "
             f"the NO_EXPECTED_HASH refusal, never a permissive default")
    if not re.search(r"else:\s*\n\s*return Verdict\(False, Refusal\.NO_EXPECTED_HASH",
                     logic_text):
        fail(f"{logic}: the NO_EXPECTED_HASH refusal path is missing or was moved out of "
             f"the is_default_backup/custom_expected_sha256/manifest_sha256 chain - a "
             f"custom backup with none of the three must still be refused")

    if not flash_sh.exists():
        fail(f"{flash_sh}: application-only flash script not found")
        return
    flash_text = strip_shell_comments(flash_sh.read_text(encoding="utf-8"))

    verify_match = re.search(r"backup_gate_logic\.py", flash_text)
    if not verify_match:
        fail(f"{flash_sh}: does not invoke backup_gate_logic.py - a backup of unknown "
             f"provenance could be trusted for the pre-flash recovery gate")
    if "MATDOG_FLASH_BACKUP_SHA256:-" not in flash_text:
        fail(f"{flash_sh}: lost the MATDOG_FLASH_BACKUP_SHA256 operator authorization "
             f"input (expected a ${{MATDOG_FLASH_BACKUP_SHA256:-...}} expansion)")
    if "MATDOG_FLASH_BACKUP_MANIFEST:-" not in flash_text:
        fail(f"{flash_sh}: lost the MATDOG_FLASH_BACKUP_MANIFEST recovery-manifest input "
             f"(expected a ${{MATDOG_FLASH_BACKUP_MANIFEST:-...}} expansion)")

    write_match = re.search(r"write-flash", flash_text)
    if verify_match is not None and write_match is not None and \
            verify_match.start() > write_match.start():
        fail(f"{flash_sh}: the backup-gate verification must appear before the esptool "
             f"write-flash invocation")

    if verify_match is not None:
        lines = flash_text.splitlines()
        start = flash_text[:verify_match.start()].count("\n")
        block = []
        for line in lines[max(0, start - 2):start + 12]:
            block.append(line)
        if re.search(r"backup_gate_logic\.py.*?\)\"\s*\|\|\s*(true|:|echo|warn)\b",
                     "\n".join(block), re.DOTALL):
            fail(f"{flash_sh}: the backup-gate verification swallows its own failure "
                 f"('|| true'/'|| :'/'|| echo') - it must REFUSE, not warn")

    if not tests.exists():
        fail(f"{tests}: backup gate offline test suite not found")
        return
    result = subprocess.run([sys.executable, str(tests)], capture_output=True, text=True)
    if result.returncode != 0:
        fail(f"{tests}: backup gate offline tests FAILED "
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
    check_servo_motion_write_surface(files)
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
    check_calibration_execution_engine_boundaries(files)
    check_actuator_infrastructure_wired_fail_closed(files)
    check_first_motion_command_wiring(files)
    check_full_leg_calibration_wiring(files)
    check_service_readiness_is_host_linkable(files)
    check_app_only_script_never_targets_other_partitions(SKETCH_DIR)
    check_ota_partition_verifier_fail_closed(SKETCH_DIR)
    check_servo_timeout_not_global(files)
    check_servo_timeout_categories_finding1(files)
    check_safe_off_verifies_readback(files)
    check_goal_position_register_boundary(files, SKETCH_DIR)
    check_uncertain_write_propagation(files, SKETCH_DIR)
    check_hardware_profile_authority(files, SKETCH_DIR)
    check_servo_population_model(files, SKETCH_DIR)
    check_g2_state_is_transport_independent(files)
    check_no_startup_servo_traffic(files)
    check_no_network_to_servo_path(files)
    check_wifi_runtime_boundaries(files, SKETCH_DIR)
    check_ota_boundaries(files, SKETCH_DIR)
    check_http_transport_boundaries(files, SKETCH_DIR)
    check_actuator_authority(files, SKETCH_DIR)
    check_calibration_boundaries(files, SKETCH_DIR)
    check_safe_actuator_boundaries(files, SKETCH_DIR)
    check_calibration_geometry_boundaries(files, SKETCH_DIR)
    check_calibration_geometry_export(SKETCH_DIR)
    check_position_offset_boundary(files, SKETCH_DIR)
    check_servo_profile_contract(files, SKETCH_DIR)
    check_servo_profile_export(SKETCH_DIR)
    check_h0_preflight_boundaries(files, SKETCH_DIR)
    check_calibration_population_evidence(files, SKETCH_DIR)
    check_calibration_q0_capture_session(files, SKETCH_DIR)
    check_calibration_q0_production_wiring(files, SKETCH_DIR)
    check_calibration_q0_promotion_wiring(files, SKETCH_DIR)
    check_calibration_q0_bootstrap(files, SKETCH_DIR)
    check_evidence_geometry_binding(files, SKETCH_DIR)
    check_calibration_readiness_contract(SKETCH_DIR)
    check_direction_is_contractual(files, SKETCH_DIR)
    check_host_tests(SKETCH_DIR)
    check_daly_audit_mutation_suite(SKETCH_DIR)
    check_safe_actuator_audit_mutation_suite(SKETCH_DIR)
    check_led_audit_mutation_suite(SKETCH_DIR)
    check_build_profile_provenance(SKETCH_DIR)
    check_backup_gate_provenance(SKETCH_DIR)
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
