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

Usage: python3 static_audit.py [sketch_dir]
Exit code 0 = PASS, 1 = FAIL.
"""
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


def check_host_tests(sketch_dir):
    """Runs the offline C++ census/profile suite, the same way the OTA
    parser's Python suite is already run from here: one gate command."""
    runner = sketch_dir / "scripts" / "tests" / "run_host_tests.sh"
    suite = sketch_dir / "scripts" / "tests" / "test_servo_population.cpp"
    if not suite.exists():
        fail(f"{suite}: G2 servo population/profile offline test suite not found")
        return
    if not runner.exists():
        fail(f"{runner}: host test runner not found")
        return
    result = subprocess.run(["bash", str(runner)], capture_output=True, text=True)
    if result.returncode != 0:
        fail(f"{runner}: servo population/profile offline tests FAILED "
             f"(stdout={result.stdout!r} stderr={result.stderr!r})")


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
    check_host_tests(SKETCH_DIR)

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
