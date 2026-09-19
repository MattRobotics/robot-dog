#!/usr/bin/env python3
"""Mutation tests for the static audit's DALY write prohibition.

Loads the REAL scripts/static_audit.py, runs its DALY checks against the
current firmware sources (must pass), then against single, targeted
mutations of those sources (each must fail). A rule that a mutation slips
past is a rule that only looks enforced.

Two headline families:
  - the read-only probe: flipping the KEY read's function code from 0x03 to
    0x06, or swapping in a well-formed FC06 frame, must fail;
  - the ONE permitted write (FC06 0x0120 := 0x005A): changing its function
    (0x06 -> 0x10), register (0x0120 -> 0x0121/0x0122), value (0x005A ->
    0x00AA/0x0055), letting a caller supply any of them, adding a second
    write, or making requestDischargeOff() transmit must fail.

No hardware, no device I/O, no files written. Run directly or via
static_audit.py (check_daly_audit_mutation_suite).
"""
import importlib.util
import sys
from pathlib import Path

SCRIPTS_DIR = Path(__file__).resolve().parent.parent
SKETCH_DIR = SCRIPTS_DIR.parent


def load_audit():
    spec = importlib.util.spec_from_file_location("static_audit", SCRIPTS_DIR / "static_audit.py")
    module = importlib.util.module_from_spec(spec)
    saved = sys.argv
    sys.argv = [str(SCRIPTS_DIR / "static_audit.py"), str(SKETCH_DIR)]
    try:
        spec.loader.exec_module(module)
    finally:
        sys.argv = saved
    return module


audit = load_audit()
BASE = [(p, audit.strip_comments(p.read_text(encoding="utf-8"))) for p in audit.iter_source_files()]


def run_daly_checks(files):
    audit.failures.clear()
    audit.check_daly_write(files)
    audit.check_bms_command_surface(files)
    audit.check_daly_protocol_is_host_linkable(files)
    audit.check_uart_peripheral_separation(files)
    return list(audit.failures)


def mutate(filename, old, new):
    """Returns BASE with exactly one occurrence of `old` replaced in `filename`."""
    out = []
    hits = 0
    for path, code in BASE:
        if path.name == filename:
            hits += code.count(old)
            code = code.replace(old, new, 1)
        out.append((path, code))
    if hits != 1:
        raise AssertionError(f"mutation anchor {old!r} found {hits} times in {filename}")
    return out


def frame_literal(frame: bytes) -> str:
    return ", ".join(f"0x{b:02X}" for b in frame)


def with_crc(six: bytes) -> bytes:
    crc = audit.modbus_crc16(six)
    return six + bytes([crc & 0xFF, crc >> 8])


KEY_BYTES = "0x81, 0x03, 0x01, 0x00,"
TELEMETRY_BYTES = "0xD2, 0x03, 0x00, 0x00,"
KEY_ARRAY_BODY = "    0x81, 0x03, 0x01, 0x00,\n    0x00, 0x78, 0x5B, 0xD4,"
TRANSMIT = "bms_uart_.write(dalyRequestFrame(request), kDalyRequestLen);"
GUARDED_KEY_READ = ('if (modules_.operating_mode->mode() != OperatingMode::MAINTENANCE) {\n'
                    '      Serial.println("BMS_KEY_READ=BLOCKED");')
KEY_STATUS_FN = "void CommandRouter::printBmsKeyStatus() {"

# A well-formed Modbus FC06 frame writing KEY logic 0x005A to 0x0120: the
# exact future action that is NOT authorized.
KEY_WRITE_FRAME = with_crc(bytes([0x81, 0x06, 0x01, 0x20, 0x00, 0x5A]))
EXTRA_READ_FRAME = with_crc(bytes([0x81, 0x03, 0x01, 0x20, 0x00, 0x01]))

WRITE_BUILDER = ("constexpr DalyFrame dalyKeyLogicDischargeWriteFrame() {\n"
                 "  DalyFrame f = {{0x81, 0x06, 0x01, 0x20, 0x00, 0x5A, 0x00, 0x00}};")
WRITE_CONST = "inline constexpr DalyFrame kDalyKeyLogicDischargeWrite = dalyKeyLogicDischargeWriteFrame();"
DISCHARGE_OFF = "bool requestDischargeOff() { return false; }"
SET_GUARD = ('if (modules_.operating_mode->mode() != OperatingMode::MAINTENANCE) {\n'
             '      Serial.println("BMS_KEY_WRITE=REFUSED reason=NOT_IN_MAINTENANCE_MODE");')
SET_CALL = "modules_.daly->requestKeyLogicDischarge(modules_.operating_mode->mode());"
WRITE_STATUS_FN = "void CommandRouter::printBmsKeyWriteStatus() {"
WRITE_HELP = 'Serial.println("  @BMS KEY WRITE STATUS   (cached write result; no bus transaction)");'
WRITE_STATUS_BRANCH = '} else if (upper == "@BMS KEY WRITE STATUS") {'


def write_payload(old, new):
    return lambda: mutate("DalyProtocol.h", WRITE_BUILDER,
                          WRITE_BUILDER.replace(old, new))


MUTATIONS = [
    # ---- the ONE permitted write ------------------------------------------
    ("WRITE function 0x06 -> 0x10 (FC10)",
     write_payload("0x81, 0x06, 0x01, 0x20", "0x81, 0x10, 0x01, 0x20")),
    ("WRITE register 0x0120 -> 0x0121 (charge MOS control)",
     write_payload("0x01, 0x20, 0x00, 0x5A", "0x01, 0x21, 0x00, 0x5A")),
    ("WRITE register 0x0120 -> 0x0122 (discharge MOS control)",
     write_payload("0x01, 0x20, 0x00, 0x5A", "0x01, 0x22, 0x00, 0x5A")),
    ("WRITE value 0x005A -> 0x00AA",
     write_payload("0x00, 0x5A, 0x00, 0x00", "0x00, 0xAA, 0x00, 0x00")),
    ("WRITE value 0x005A -> 0x0055 (an unreviewed rollback)",
     write_payload("0x00, 0x5A, 0x00, 0x00", "0x00, 0x55, 0x00, 0x00")),
    ("WRITE address 0x81 -> 0xD2",
     write_payload("{{0x81, 0x06", "{{0xD2, 0x06")),
    ("WRITE target supplied by a builder argument",
     lambda: mutate("DalyProtocol.h", WRITE_BUILDER,
                    WRITE_BUILDER.replace("dalyKeyLogicDischargeWriteFrame()",
                                          "dalyKeyLogicDischargeWriteFrame(uint16_t value)")
                                 .replace("0x00, 0x5A, 0x00, 0x00",
                                          "0x00, static_cast<uint8_t>(value), 0x00, 0x00"))),
    ("WRITE value supplied through the DalyBms API",
     lambda: mutate("DalyBms.h", "DalyKeyWriteGate requestKeyLogicDischarge(core::OperatingMode mode);",
                    "DalyKeyWriteGate requestKeyLogicDischarge(core::OperatingMode mode, uint16_t raw);")),
    ("WRITE requested with a constant mode instead of the live one",
     lambda: mutate("CommandRouter.cpp", SET_CALL,
                    "modules_.daly->requestKeyLogicDischarge(OperatingMode::MAINTENANCE);")),
    ("WRITE CRC recipe altered",
     lambda: mutate("DalyProtocol.h", "crc16Modbus(f.bytes, kDalyRequestLen - 2)",
                    "crc16Modbus(f.bytes, kDalyRequestLen - 3)")),
    ("second write frame constant",
     lambda: mutate("DalyProtocol.h", WRITE_CONST,
                    WRITE_CONST + "\ninline constexpr DalyFrame kDalyKeyLogicSpare = "
                                  "dalyKeyLogicDischargeWriteFrame();")),
    ("second write builder (rollback 0x0055)",
     lambda: mutate("DalyProtocol.h", WRITE_CONST,
                    "constexpr DalyFrame dalyKeyLogicRollbackFrame() {\n  DalyFrame f = {{0x81, 0x06, "
                    "0x01, 0x20, 0x00, 0x55, 0x00, 0x00}};\n  return f;\n}\n" + WRITE_CONST)),
    ("FC10 frame array added",
     lambda: mutate("DalyProtocol.h", WRITE_CONST,
                    WRITE_CONST + "\ninline constexpr uint8_t kDalyMultiWrite[kDalyRequestLen] = "
                                  "{0x81, 0x10, 0x01, 0x20, 0x00, 0x01, 0x00, 0x00};")),
    ("write frame sent directly from DalyBms",
     lambda: mutate("DalyBms.cpp", TRANSMIT,
                    "bms_uart_.write(kDalyKeyLogicDischargeWrite.bytes, kDalyRequestLen);")),
    ("selector maps the KEY read to the write frame",
     lambda: mutate("DalyProtocol.h", "? kDalyKeyConfigRequest",
                    "? kDalyKeyLogicDischargeWrite.bytes")),
    ("requestDischargeOff() requests the write",
     lambda: mutate("DalyBms.h", DISCHARGE_OFF,
                    "bool requestDischargeOff() { return requestKeyLogicDischarge(core::"
                    "OperatingMode::MAINTENANCE).decision == DalyKeyWriteDecision::START; }")),
    ("requestDischargeOff() transmits a frame",
     lambda: mutate("DalyBms.h", DISCHARGE_OFF,
                    "bool requestDischargeOff() { bms_uart_.write(dalyRequestFrame(DalyRequest::"
                    "KEY_LOGIC_DISCHARGE_WRITE), kDalyRequestLen); return false; }")),
    ("@BMS KEY SET DISCHARGE CONFIRM without the MAINTENANCE gate",
     lambda: mutate("CommandRouter.cpp", SET_GUARD,
                    SET_GUARD.replace("modules_.operating_mode->mode() != OperatingMode::MAINTENANCE",
                                      "false"))),
    ("shorter SET alias",
     lambda: mutate("CommandRouter.cpp", WRITE_STATUS_BRANCH,
                    '} else if (upper == "@BMS KEY SET DISCHARGE") {\n  ' + WRITE_STATUS_BRANCH)),
    ("@BMS KEY WRITE STATUS starts a transaction",
     lambda: mutate("CommandRouter.cpp", WRITE_STATUS_FN,
                    WRITE_STATUS_FN + "\n  " + SET_CALL)),
    ("generic @BMS WRITE text",
     lambda: mutate("CommandRouter.cpp", WRITE_HELP,
                    WRITE_HELP + '\n  Serial.println("  @BMS WRITE <reg> <value>");')),
    ("DALY write state persisted",
     lambda: mutate("DalyBms.cpp", TRANSMIT, TRANSMIT + "\n  Preferences prefs;")),
    ("write requested from the Controller",
     lambda: mutate("Controller.cpp", "  daly_.update(now_ms);",
                    "  daly_.update(now_ms);\n  daly_.requestKeyLogicDischarge("
                    "operating_mode_.mode());")),
    # ---- the read-only probe and transport ------------------------------------
    ("KEY request function 0x03 -> 0x06",
     lambda: mutate("DalyProtocol.h", KEY_BYTES, "0x81, 0x06, 0x01, 0x00,")),
    ("KEY request function 0x03 -> 0x10",
     lambda: mutate("DalyProtocol.h", KEY_BYTES, "0x81, 0x10, 0x01, 0x00,")),
    ("KEY request replaced by a CRC-valid FC06 write of 0x5A to 0x0120",
     lambda: mutate("DalyProtocol.h", KEY_ARRAY_BODY, "    " + frame_literal(KEY_WRITE_FRAME) + ",")),
    ("telemetry request function 0x03 -> 0x06",
     lambda: mutate("DalyProtocol.h", TELEMETRY_BYTES, "0xD2, 0x06, 0x00, 0x00,")),
    ("third (read) frame added",
     lambda: mutate("DalyProtocol.h", "constexpr const uint8_t* dalyRequestFrame(",
                    "inline constexpr uint8_t kDalyExtraRequest[kDalyRequestLen] = {"
                    + frame_literal(EXTRA_READ_FRAME) + "};\n"
                    "constexpr const uint8_t* dalyRequestFrame(")),
    ("second bms_uart_.write() call",
     lambda: mutate("DalyBms.cpp", TRANSMIT, TRANSMIT + "\n  " + TRANSMIT)),
    ("transmit of a caller/raw buffer",
     lambda: mutate("DalyBms.cpp", TRANSMIT, "bms_uart_.write(rx_buf_, kDalyRequestLen);")),
    ("print on the DALY UART",
     lambda: mutate("DalyBms.cpp", TRANSMIT, TRANSMIT + '\n  bms_uart_.print("x");')),
    ("DALY UART handed out as a stream",
     lambda: mutate("DalyBms.cpp", TRANSMIT, TRANSMIT + "\n  sendVia(bms_uart_);")),
    ("frame selector takes a buffer",
     lambda: mutate("DalyProtocol.h", "dalyRequestFrame(DalyRequest request) {\n  return",
                    "dalyRequestFrame(DalyRequest request, const uint8_t* custom) {\n  return "
                    "custom ? custom :")),
    ("frame selector returns an unlisted frame",
     lambda: mutate("DalyProtocol.h", ": kDalyTelemetryRequest;", ": kDalyWriteRequest;")),
    ("requestDischargeOff() implemented",
     lambda: mutate("DalyBms.h", "bool requestDischargeOff() { return false; }",
                    "bool requestDischargeOff() { return true; }")),
    ("requestDischargeOff() called",
     lambda: mutate("DalyBms.cpp", TRANSMIT, TRANSMIT + "\n  requestDischargeOff();")),
    ("@BMS KEY SET command added",
     lambda: mutate("CommandRouter.cpp", '} else if (upper == "@BMS KEY STATUS") {',
                    '} else if (upper == "@BMS KEY SET") {\n  } else if (upper == "@BMS KEY STATUS") {')),
    ("@BMS command parsing arguments",
     lambda: mutate("CommandRouter.cpp", '} else if (upper == "@BMS KEY STATUS") {',
                    '} else if (upper.startsWith("@BMS REG")) {\n  } else if (upper == "@BMS KEY STATUS") {')),
    ("@BMS KEY READ without the MAINTENANCE gate",
     lambda: mutate("CommandRouter.cpp", GUARDED_KEY_READ,
                    GUARDED_KEY_READ.replace(
                        "modules_.operating_mode->mode() != OperatingMode::MAINTENANCE", "false"))),
    ("@BMS KEY STATUS starts a bus transaction",
     lambda: mutate("CommandRouter.cpp", KEY_STATUS_FN,
                    KEY_STATUS_FN + "\n  modules_.daly->requestKeyConfigRead();")),
    ("KEY probe consumed by the power-state machine",
     lambda: mutate("PowerState.cpp", "void PowerStateMachine::requestShutdown() {",
                    "static const int kLeak = sizeof(power::DalyKeyConfigSnapshot);\n"
                    "void PowerStateMachine::requestShutdown() {")),
    ("second route to UART2",
     lambda: mutate("Controller.cpp", "  daly_.update(now_ms);",
                    "  daly_.update(now_ms);\n  Serial2.write(0x06);")),
    ("extra HardwareSerial owner",
     lambda: mutate("DalyBms.h", "HardwareSerial bms_uart_{2};",
                    "HardwareSerial bms_uart_{2};\n  HardwareSerial spare_uart_{2};")),
    ("Modbus write literal in DALY source",
     lambda: mutate("DalyBms.cpp", TRANSMIT, TRANSMIT + "\n  const uint8_t fc = 0x06;")),
    ("protocol unit gains an Arduino dependency",
     lambda: mutate("DalyProtocol.cpp", '#include "DalyProtocol.h"',
                    '#include "DalyProtocol.h"\n#include <Arduino.h>')),
]


def main():
    passed = 0
    failed = []

    baseline = run_daly_checks(BASE)
    if baseline:
        failed.append(("unmutated sources", baseline))
    else:
        passed += 1
        print("  PASS  unmutated firmware satisfies the DALY rules")

    for name, build in MUTATIONS:
        findings = run_daly_checks(build())
        if findings:
            passed += 1
            first = findings[0].replace(str(SKETCH_DIR) + "/", "")
            print(f"  PASS  caught: {name}  ({len(findings)} finding(s); first: {first[:100]})")
        else:
            failed.append((name, "mutation NOT detected"))

    # The whitelist is itself verified: admitting a write frame to it (and
    # to the source, consistently) must still fail.
    saved = dict(audit.DALY_WHITELISTED_READ_FRAMES)
    try:
        audit.DALY_WHITELISTED_READ_FRAMES["kDalyKeyConfigRequest"] = KEY_WRITE_FRAME
        findings = run_daly_checks(
            mutate("DalyProtocol.h", KEY_ARRAY_BODY, "    " + frame_literal(KEY_WRITE_FRAME) + ","))
    finally:
        audit.DALY_WHITELISTED_READ_FRAMES.clear()
        audit.DALY_WHITELISTED_READ_FRAMES.update(saved)
    if any("whitelist itself" in f for f in findings):
        passed += 1
        print("  PASS  caught: a write frame admitted to the audit whitelist")
    else:
        failed.append(("write frame admitted to the whitelist", findings or "NOT detected"))

    audit.failures.clear()
    print(f"cases_run={1 + len(MUTATIONS) + 1} passed={passed} failed={len(failed)}")
    if failed:
        for name, detail in failed:
            print(f"  FAIL  {name}: {detail}")
        print("DALY_AUDIT_MUTATION_TESTS = FAIL")
        return 1
    print("DALY_AUDIT_MUTATION_TESTS = PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
