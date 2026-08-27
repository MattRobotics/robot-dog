#!/usr/bin/env python3
"""
MATDOG ST-3215-C018 Characterize V1 — host runner.

Companion to:
    characterization/matdog_st3215_characterize_v1.ino
Design:
    characterization/CHARACTERIZATION_DESIGN.md

WHAT THIS IS
    A measurement harness for exactly three unresolved provisioner constants:

        PRIME_MAX_DELTA_TICKS
        COLD_ABSENCE_DEBOUNCE_MS
        COLD_RETURN_STABLE_SAMPLES

    It measures. It does not choose. Every report it writes states
    DEFERRED_TO_REVIEW for all three, and running it does not clear the
    provisioner's hardware-freeze block.

WHAT THIS IS NOT
    Not a provisioner. It cannot write EEPROM, cannot write an ID, cannot write
    PositionOffset, cannot centre anything, cannot command an arbitrary
    position, cannot enable torque, cannot reset a servo and cannot broadcast.
    The firmware's entire write capability is three RAM operations and the host
    cannot compose any of them.

WHAT THIS PROCESS OWNS
    Evidence and ordering. The 71 as-found bytes returned by @ARM must be
    durable on disk (exclusive create, write, flush, os.fsync, length verified,
    SHA256 recorded) BEFORE @PRIME can be transmitted. That is not an assertion
    made after the fact: the transport gate refuses to send @PRIME until the
    artifact is on disk.

USAGE
    python3 matdog_st3215_characterize_v1.py --self-test
    python3 matdog_st3215_characterize_v1.py --static-audit
    python3 matdog_st3215_characterize_v1.py --command-surface
    python3 matdog_st3215_characterize_v1.py --characterize --unit NEW01
"""

import argparse
import datetime
import hashlib
import json
import os
import re
import sys

TOOL_NAME = "matdog_st3215_characterize_v1"
TOOL_VERSION = "1.0.0"
REPORT_SCHEMA = "MATDOG_ST3215_CHARACTERIZE_V1"

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
FIRMWARE_SOURCE_PATH = os.path.join(PROJECT_DIR, TOOL_NAME + ".ino")
PROVISIONER_DIR = os.path.dirname(PROJECT_DIR)
PROVISIONER_SOURCE_PATH = os.path.join(
    PROVISIONER_DIR, "matdog_st3215_provisioner_v1.ino"
)

FIRMWARE_FQBN = (
    "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,"
    "CPUFreq=240,FlashMode=qio,FlashSize=16M,"
    "PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi"
)

DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Espressif_USB_JTAG_serial_debug_unit_"
    "14:C1:9F:22:75:94-if00"
)

DEFAULT_SESSION_ROOT = os.path.join(PROJECT_DIR, "sessions")


class SafetyViolation(RuntimeError):
    """A forbidden operation was attempted. Always fatal."""


class CharacterizationError(RuntimeError):
    """A precondition or verification failed. Always fail-closed."""


# --------------------------------------------------------------------------
# 71-byte state map, 0x00 .. 0x46 inclusive
# --------------------------------------------------------------------------

SNAPSHOT_LEN = 71

ADDR_MODEL = 0x03            # NOT 0x00 — see provisioner design section 5
ADDR_ID = 0x05
ADDR_BAUD = 0x06
ADDR_RESPONSE_STATUS = 0x08
ADDR_POSITION_OFFSET = 0x1F
ADDR_TORQUE_ENABLE = 0x28
ADDR_TORQUE_LIMIT = 0x30
ADDR_LOCK = 0x37
ADDR_PRESENT_POSITION = 0x38
ADDR_PRESENT_VOLTAGE = 0x3E
ADDR_PRESENT_TEMPERATURE = 0x3F
ADDR_STATUS = 0x40

EXPECTED_MODEL = 777
EXPECTED_RESPONSE_STATUS = 1
EXPECTED_BAUD = 0

ENCODER_COUNTS = 4096
HALF_TURN = 2048
POSITION_OFFSET_MIN = -2048
POSITION_OFFSET_MAX = 2047

VOLTAGE_MIN = 40
VOLTAGE_MAX = 140
THERMAL_LIMIT_C = 70

# --------------------------------------------------------------------------
# The exact final prime parameters under characterization
# --------------------------------------------------------------------------

CHAR_TORQUE_LIMIT = 300
CHAR_SPEED = 365
CHAR_ACC = 50
PRIME_REPETITIONS = 5

# The complete write capability of the firmware: (addr, width, value).
WRITE_ALLOWLIST = (
    (ADDR_TORQUE_ENABLE, 1, 0),
    (ADDR_TORQUE_LIMIT, 2, CHAR_TORQUE_LIMIT),
)

# Firmware stage names, mirrored for the exhaustive allowlist test.
STAGE_NONE = "STAGE_NONE"
STAGE_PRIME = "STAGE_PRIME"
STAGE_RECOVERY = "STAGE_RECOVERY"
STAGES = (STAGE_NONE, STAGE_PRIME, STAGE_RECOVERY)

TRACE_STABLE_TARGET = 20     # data-collection endpoint, NOT a threshold

# The three values this tool exists to inform and refuses to choose.
DEFERRED_CONSTANTS = (
    "PRIME_MAX_DELTA_TICKS",
    "COLD_ABSENCE_DEBOUNCE_MS",
    "COLD_RETURN_STABLE_SAMPLES",
)

# --------------------------------------------------------------------------
# Evidence artifacts
# --------------------------------------------------------------------------

ARTIFACTS = {
    "BEFORE": "01_before_state71.bin",
    "AFTER_PRIME": "02_after_prime_state71.bin",
    "COLD1": "03_after_cold_cycle1_state71.bin",
    "COLD2": "04_after_cold_cycle2_state71.bin",
}

CONSOLE_LOG_NAME = "session_console.log"
REPORT_NAME = "characterization_report.json"
SHA256SUMS_NAME = "SHA256SUMS"

ARM_TIMEOUT_S = 120.0        # a full 0..253 scan costs ~25 s
PRIME_TIMEOUT_S = 180.0
TRACE_TIMEOUT_S = 480.0      # includes the operator power-cycle window


# --------------------------------------------------------------------------
# Command surface — the single choke point
# --------------------------------------------------------------------------

ALLOWED_COMMAND_PATTERNS = (
    re.compile(r"^@ARM$"),
    re.compile(r"^@PRIME (?:[0-9A-F]{8})$"),
    re.compile(r"^@TRACE (?:[0-9A-F]{8})$"),
    re.compile(r"^@HELP$"),
)

# Commands that can cause the firmware to write anything at all.
WRITE_CAPABLE_PREFIXES = ("@PRIME",)

FORBIDDEN_COMMAND_TOKENS = (
    "@BEGIN", "@EXECUTE", "@NORMALIZE_MATDOG", "@QC_FAST", "@READ",
    "@DUMP_RAW", "@BENCH", "@CAPTURE", "@RAMTEST", "@PING", "@SCAN",
    "@SNAPSHOT71", "@SAFE_OFF", "@WRITE", "@SETID", "@GOAL", "@TORQUE",
    "@TORQUE_ON", "@RESET", "@CALIBRATE", "@CENTER", "@UNLOCK", "@OFFSET",
)


def assert_command_allowed(command):
    """Sole gate through which any serial command must pass."""
    if not isinstance(command, str):
        raise SafetyViolation("command must be str")
    if command != command.strip():
        raise SafetyViolation("command has leading/trailing whitespace")
    if "\n" in command or "\r" in command:
        raise SafetyViolation("command contains embedded newline")
    for pattern in ALLOWED_COMMAND_PATTERNS:
        if pattern.match(command):
            return command
    raise SafetyViolation("command not in allowed surface: %r" % (command,))


def command_surface_audit():
    return {
        "allowed_patterns": [p.pattern for p in ALLOWED_COMMAND_PATTERNS],
        "allowed_examples": ["@ARM", "@PRIME 1A2B3C4D", "@TRACE 1A2B3C4D",
                             "@HELP"],
        "write_capable_commands": list(WRITE_CAPABLE_PREFIXES),
        "forbidden_tokens_rejected": list(FORBIDDEN_COMMAND_TOKENS),
        "generic_write": "NOT IMPLEMENTED",
        "arbitrary_address_value": "NOT IMPLEMENTED",
        "arbitrary_goal_position": "NOT IMPLEMENTED",
        "arbitrary_id": "NOT IMPLEMENTED",
        "eeprom_write": "NOT IMPLEMENTED",
        "eeprom_unlock": "NOT IMPLEMENTED",
        "id_write": "NOT IMPLEMENTED",
        "position_offset_write": "NOT IMPLEMENTED",
        "mid_position_calibration": "NOT IMPLEMENTED",
        "factory_reset": "NOT IMPLEMENTED",
        "broadcast_write": "NOT IMPLEMENTED",
        "host_exposed_torque_on": "NOT IMPLEMENTED",
        "centering_move": "NOT IMPLEMENTED",
        "arm_performs_servo_writes": "NO",
        "firmware_write_allowlist": [
            {"address": "0x%02X" % a, "width": w, "value": v}
            for a, w, v in WRITE_ALLOWLIST
        ],
        "motion_allowlist": {
            "primitive": "WritePosEx",
            "position": "CURRENT_POSITION_READ_BY_FIRMWARE",
            "speed": CHAR_SPEED,
            "acc": CHAR_ACC,
        },
        "constants_selected_by_this_tool": [],
        "constants_deferred_to_review": list(DEFERRED_CONSTANTS),
    }


# --------------------------------------------------------------------------
# Primitives
# --------------------------------------------------------------------------

def sha256_bytes(data):
    return hashlib.sha256(data).hexdigest()


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def utc_now_iso():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


def floor_mod(value, modulus):
    """Explicit floor modulo returning 0..modulus-1. Mirrors the firmware."""
    if modulus <= 0:
        raise CharacterizationError("floor_mod modulus must be positive")
    result = value % modulus
    if result < 0:
        result += modulus
    return result


def physical_raw(displayed, signed_offset):
    return floor_mod(displayed + signed_offset, ENCODER_COUNTS)


def circular_delta(a, b):
    """Smallest signed distance on the 4096 circle.

    Safe across the 0/4095 wrap in both directions. The exact half turn
    resolves negative (-2048), never +2048. Mirrors the firmware helper.
    """
    return floor_mod(a - b + HALF_TURN, ENCODER_COUNTS) - HALF_TURN


def validate_position_domain(value, what="position"):
    if not isinstance(value, int) or isinstance(value, bool):
        raise CharacterizationError("%s must be int, got %r" % (what, value))
    if value < 0 or value > 4095:
        raise CharacterizationError(
            "%s %d outside the 0..4095 domain" % (what, value)
        )
    return value


def u16le(raw, addr):
    return raw[addr] | (raw[addr + 1] << 8)


def i16le_twos_complement(raw, addr):
    """PositionOffset only. Two's complement, little endian."""
    value = u16le(raw, addr)
    return value - 0x10000 if value & 0x8000 else value


def parse_raw71_hex(hex_text):
    text = hex_text.strip()
    if len(text) != SNAPSHOT_LEN * 2:
        raise CharacterizationError(
            "RAW71_HEX must be %d hex chars, got %d"
            % (SNAPSHOT_LEN * 2, len(text))
        )
    if not re.fullmatch(r"[0-9A-Fa-f]+", text):
        raise CharacterizationError("RAW71_HEX contains non-hex characters")
    raw = bytes.fromhex(text)
    if len(raw) != SNAPSHOT_LEN:
        raise CharacterizationError("decoded snapshot is not 71 bytes")
    return raw


def decode_snapshot(raw):
    if len(raw) != SNAPSHOT_LEN:
        raise CharacterizationError("snapshot must be exactly 71 bytes")
    offset = i16le_twos_complement(raw, ADDR_POSITION_OFFSET)
    present = u16le(raw, ADDR_PRESENT_POSITION)
    return {
        "model": u16le(raw, ADDR_MODEL),
        "id": raw[ADDR_ID],
        "baud": raw[ADDR_BAUD],
        "response_status": raw[ADDR_RESPONSE_STATUS],
        "position_offset": offset,
        "torque_enable": raw[ADDR_TORQUE_ENABLE],
        "torque_limit": u16le(raw, ADDR_TORQUE_LIMIT),
        "lock": raw[ADDR_LOCK],
        "present_position": present,
        "physical_raw": physical_raw(present, offset),
        "voltage": raw[ADDR_PRESENT_VOLTAGE],
        "temperature": raw[ADDR_PRESENT_TEMPERATURE],
        "status": raw[ADDR_STATUS],
        "sha256": sha256_bytes(raw),
    }


# --------------------------------------------------------------------------
# Discovery gate — recomputed by the host from the BEFORE bytes
#
# The firmware decides and the host must agree. PositionOffset is checked for a
# sane domain only: the unit under characterization has NOT been provisioned,
# so a nonzero offset is expected and is never a failure here.
# --------------------------------------------------------------------------

def evaluate_discovery_gate(responders, ping_status, model_status, scan_model,
                            raw):
    decoded = decode_snapshot(raw)
    checks = [
        ("responders", responders == 1, responders, "1"),
        ("ping_status", ping_status == 0, ping_status, "0x00"),
        ("model_status", model_status == 0, model_status, "0x00"),
        ("scan_model", scan_model == EXPECTED_MODEL, scan_model, "777"),
        ("model_0x03", decoded["model"] == EXPECTED_MODEL, decoded["model"],
         "777"),
        ("baud", decoded["baud"] == EXPECTED_BAUD, decoded["baud"], "0"),
        ("response_status",
         decoded["response_status"] == EXPECTED_RESPONSE_STATUS,
         decoded["response_status"], "1"),
        ("position_offset_domain",
         POSITION_OFFSET_MIN <= decoded["position_offset"] <=
         POSITION_OFFSET_MAX,
         decoded["position_offset"], "-2048..2047"),
        ("present_position_domain",
         0 <= decoded["present_position"] <= 4095,
         decoded["present_position"], "0..4095"),
        ("lock_domain", decoded["lock"] in (0, 1), decoded["lock"], "0 or 1"),
        ("torque_off", decoded["torque_enable"] == 0, decoded["torque_enable"],
         "0"),
        ("voltage", VOLTAGE_MIN <= decoded["voltage"] <= VOLTAGE_MAX,
         decoded["voltage"], "40..140"),
        ("temperature", decoded["temperature"] <= THERMAL_LIMIT_C,
         decoded["temperature"], "<=70"),
        ("status", decoded["status"] == 0, decoded["status"], "0x00"),
    ]
    failures = [
        {"gate": name, "actual": actual, "expected": expected}
        for name, ok, actual, expected in checks if not ok
    ]
    return {
        "decoded": decoded,
        "checks": [
            {"gate": n, "ok": ok, "actual": a, "expected": e}
            for n, ok, a, e in checks
        ],
        "failures": failures,
        "pass": not failures,
    }


# --------------------------------------------------------------------------
# Evidence persistence
# --------------------------------------------------------------------------

def persist_raw71(raw, path):
    """Write exactly 71 raw bytes. Fail-closed and exclusive.

    open(path, "xb") never overwrites existing evidence; the bytes are flushed
    and os.fsync'd before this returns; the on-disk size is verified.
    """
    if not isinstance(raw, (bytes, bytearray)):
        raise CharacterizationError(
            "raw snapshot must be bytes, got %s" % type(raw).__name__
        )
    if len(raw) != SNAPSHOT_LEN:
        raise CharacterizationError(
            "refusing to persist %d bytes, expected exactly %d"
            % (len(raw), SNAPSHOT_LEN)
        )
    payload = bytes(raw)
    try:
        handle = open(path, "xb")
    except FileExistsError:
        raise CharacterizationError(
            "raw artifact already exists, refusing to overwrite evidence: %s"
            % path
        )
    try:
        written = handle.write(payload)
        handle.flush()
        os.fsync(handle.fileno())
    finally:
        handle.close()

    on_disk = os.path.getsize(path)
    if written != SNAPSHOT_LEN or on_disk != SNAPSHOT_LEN:
        raise CharacterizationError(
            "raw artifact %s is %d bytes on disk (wrote %s), expected %d"
            % (path, on_disk, written, SNAPSHOT_LEN)
        )
    return {
        "path": path,
        "bytes": SNAPSHOT_LEN,
        "sha256": sha256_bytes(payload),
        "persisted_utc": utc_now_iso(),
        "exclusive_create": True,
        "flushed_and_fsynced": True,
        "persisted_before_decode": True,
    }


def artifact_path(session_dir, slot):
    if slot not in ARTIFACTS:
        raise CharacterizationError(
            "unknown artifact slot %r; expected one of %s"
            % (slot, ", ".join(sorted(ARTIFACTS)))
        )
    return os.path.join(session_dir, ARTIFACTS[slot])


def capture_snapshot(slot, raw_hex, session_dir, on_persisted=None):
    """One snapshot, in this order and no other:

        1. parse RAW71_HEX   -> exactly 71 bytes, or fail closed
        2. persist_raw71()   -> write + flush + os.fsync to its own .bin
        3. decode_snapshot() -> only once the bytes are durable
    """
    raw = parse_raw71_hex(raw_hex)
    artifact = persist_raw71(raw, artifact_path(session_dir, slot))
    if on_persisted is not None:
        on_persisted(slot, artifact)
    decoded = decode_snapshot(raw)
    decoded["raw_artifact"] = artifact
    decoded["slot"] = slot
    return raw, decoded


def create_session_dir(root, label, stamp=None):
    """Exclusive creation. A pre-existing directory fails closed."""
    if stamp is None:
        stamp = datetime.datetime.now(
            datetime.timezone.utc
        ).strftime("%Y%m%d_%H%M%SZ")
    session_dir = os.path.join(root, "%s__%s" % (label, stamp))
    os.makedirs(root, exist_ok=True)
    try:
        os.makedirs(session_dir, exist_ok=False)
    except FileExistsError:
        raise CharacterizationError(
            "session directory already exists, refusing to reuse: %s"
            % session_dir
        )
    return session_dir


# --------------------------------------------------------------------------
# EVIDENCE-BEFORE-WRITE GATE
#
# Not an after-the-fact assertion. Every transport consults this object before
# a command leaves the host, so @PRIME is unsendable until the 71 as-found
# bytes are durable on disk.
# --------------------------------------------------------------------------

class EvidenceGate:
    def __init__(self):
        self.ledger = []
        self.persisted = {}

    def note_persisted(self, slot, artifact):
        self.persisted[slot] = artifact
        self.ledger.append({"event": "persist", "slot": slot,
                            "sha256": artifact["sha256"]})

    def note_sent(self, command):
        self.ledger.append({"event": "command", "command": command})

    def before_is_durable(self):
        artifact = self.persisted.get("BEFORE")
        if artifact is None:
            return False
        if artifact.get("bytes") != SNAPSHOT_LEN:
            return False
        if not artifact.get("exclusive_create"):
            return False
        if not artifact.get("flushed_and_fsynced"):
            return False
        if len(artifact.get("sha256", "")) != 64:
            return False
        path = artifact.get("path")
        if not path or not os.path.isfile(path):
            return False
        if os.path.getsize(path) != SNAPSHOT_LEN:
            return False
        return sha256_file(path) == artifact["sha256"]

    def check(self, command):
        if command.startswith(WRITE_CAPABLE_PREFIXES):
            if not self.before_is_durable():
                raise SafetyViolation(
                    "refusing to transmit %r: the 71-byte BEFORE evidence is "
                    "not durable on disk" % command
                )
        return command

    def ordering_report(self):
        first_write = None
        before_persist = None
        for index, entry in enumerate(self.ledger):
            if (first_write is None and entry["event"] == "command"
                    and entry["command"].startswith(WRITE_CAPABLE_PREFIXES)):
                first_write = index
            if before_persist is None and entry.get("slot") == "BEFORE":
                before_persist = index
        return {
            "ledger": list(self.ledger),
            "before_persist_index": before_persist,
            "first_write_capable_command_index": first_write,
            "before_persisted": before_persist is not None,
            "evidence_precedes_write": (
                before_persist is not None
                and (first_write is None or before_persist < first_write)
            ),
        }


# --------------------------------------------------------------------------
# Static audit of the firmware source
# --------------------------------------------------------------------------

WRITE_PRIMITIVES = (
    "writeByte", "writeWord", "EnableTorque", "WritePosEx", "unLockEprom",
    "LockEprom", "CalibrationOfs", "genWrite", "regWrite", "syncWrite",
    "RegWriteAction", "WheelMode", "WriteSpe", "SyncWritePosEx",
)

# (primitive, enclosing function) -> why it is allowed to exist.
APPROVED_WRITE_SITES = {
    ("writeByte", "charTorqueOffCommand"):
        "the only byte write; gated by charWriteAllowed, value 0 only",
    ("writeWord", "charWriteWordVerified"):
        "the only word write; gated by charWriteAllowed, TorqueLimit 300 only",
    ("WritePosEx", "charPrimeWritePosEx"):
        "the only motion primitive; position is read by the primitive itself, "
        "speed and acc pinned to the canonical prime constants",
}

# Primitives that must have ZERO call sites anywhere in the firmware.
FORBIDDEN_PRIMITIVES = (
    "EnableTorque", "unLockEprom", "LockEprom", "CalibrationOfs", "genWrite",
    "regWrite", "syncWrite", "RegWriteAction", "WheelMode", "WriteSpe",
    "SyncWritePosEx",
)

FORBIDDEN_SOURCE_PATTERNS = (
    (r"\bCalibrationOfs\s*\(", "mid-position calibration call"),
    (r"\bfactoryReset\b", "factory reset"),
    (r"\bFACTORY_RESET\s*\(", "factory reset call"),
    (r"\bsyncWrite\s*\(", "broadcast/sync write"),
    (r"\bSyncWritePosEx\s*\(", "broadcast/sync position write"),
    (r"\bregWrite\s*\(", "asynchronous register write"),
    (r"\bRegWriteAction\s*\(", "asynchronous write action"),
    (r"\bgenWrite\s*\(", "generic write"),
    (r"\bunLockEprom\s*\(", "EEPROM unlock"),
    (r"\bLockEprom\s*\(", "EEPROM lock"),
    (r"\bEnableTorque\s*\(", "explicit torque enable"),
    (r"0xfe\s*,", "broadcast ID 0xFE as an argument"),
    (r"\(s16\)\s*-", "signed negative GoalPosition"),
)

# Registers that must never appear inside the write allowlist function.
ALLOWLIST_FORBIDDEN_REGISTERS = (
    "REG_ID", "REG_LOCK", "REG_POSITION_OFFSET", "REG_BAUD",
    "REG_RESPONSE_STATUS", "REG_MODEL", "REG_PRESENT_POSITION",
)


def read_firmware_source():
    if not os.path.isfile(FIRMWARE_SOURCE_PATH):
        raise CharacterizationError(
            "firmware source missing: %s" % FIRMWARE_SOURCE_PATH
        )
    with open(FIRMWARE_SOURCE_PATH, encoding="utf-8") as handle:
        return handle.read()


def is_comment(line):
    return line.lstrip().startswith(("//", "*", "/*"))


def firmware_constant(source, name):
    match = re.search(
        r"static constexpr\s+[A-Za-z0-9_]+\s+%s\s*=\s*([^;]+);"
        % re.escape(name), source
    )
    if not match:
        raise CharacterizationError(
            "firmware constant %s not found" % name
        )
    return match.group(1).strip()


def firmware_constant_int(source, name):
    text = firmware_constant(source, name)
    match = re.match(r"^-?\d+", text)
    if not match:
        raise CharacterizationError(
            "firmware constant %s is not an integer literal: %r" % (name, text)
        )
    return int(match.group(0))


def firmware_function_body(source, name):
    """Source text of one top-level firmware function, for targeted audits."""
    match = re.search(
        r"^[A-Za-z_][^\n]*?\b%s\s*\([^\n]*\{\s*$" % re.escape(name),
        source, re.M,
    )
    if not match:
        raise CharacterizationError("firmware function %s not found" % name)
    lines = source[match.start():].splitlines()
    body = [lines[0]]
    for line in lines[1:]:
        body.append(line)
        if line == "}":
            return "\n".join(body)
    raise CharacterizationError("firmware function %s is unterminated" % name)


def normalize_body(text):
    """Comment-stripped, whitespace-collapsed form, for a freeze digest."""
    out = []
    for line in text.splitlines():
        line = re.sub(r"//.*$", "", line)
        line = line.strip()
        if line:
            out.append(line)
    return re.sub(r"\s+", " ", " ".join(out))


def _enclosing_function(lines, index):
    """Nearest preceding top-level function definition."""
    pattern = re.compile(r"^[A-Za-z_].*?\b([A-Za-z_][A-Za-z0-9_]*)\s*\(")
    for i in range(index, -1, -1):
        line = lines[i]
        if line.startswith(" ") or line.startswith("\t"):
            continue
        match = pattern.match(line)
        if match and "(" in line:
            if match.group(1) not in ("if", "for", "while", "switch", "return"):
                return match.group(1)
    return None


def audit_write_allowlist(source):
    """The firmware's entire write capability, proven from its own source."""
    body = firmware_function_body(source, "charWriteAllowed")
    registers = set(re.findall(r"\bREG_[A-Z_]+\b", body))
    stripped = normalize_body(body)
    return {
        "body_normalized_sha256": sha256_bytes(stripped.encode("utf-8")),
        "registers_referenced": sorted(registers),
        "checks": {
            "only_two_registers":
                registers == {"REG_TORQUE_ENABLE", "REG_TORQUE_LIMIT"},
            "no_forbidden_register": not any(
                reg in registers for reg in ALLOWLIST_FORBIDDEN_REGISTERS
            ),
            "torque_value_zero_only":
                "if (value != 0) return false;" in body,
            "torque_width_one_only": "if (width != 1) return false;" in body,
            "torque_limit_value_300_only":
                "if (value != CHAR_TORQUE_LIMIT) return false;" in body,
            "torque_limit_width_two_only":
                "if (width != 2) return false;" in body,
            "default_deny": stripped.endswith("return false; }"),
            "no_torque_on_literal": not re.search(r"value\s*==\s*1\b", body),
            "no_calibration_literal": not re.search(r"value\s*==\s*128\b",
                                                    body),
        },
    }


def audit_motion_primitive(source):
    """WritePosEx can only ever mean 'stay exactly where you already are'."""
    body = firmware_function_body(source, "charPrimeWritePosEx")
    signature = body.splitlines()[0]
    calls = re.findall(r"st\.WritePosEx\(([^;]*?)\);", source)
    args = [re.sub(r"\s+", " ", c).strip() for c in calls]
    return {
        "signature": signature.strip(),
        "call_sites": args,
        "checks": {
            "exactly_one_call_site": len(args) == 1,
            "call_uses_current_position":
                args == ["id, (s16)present, CHAR_SPEED, CHAR_ACC"],
            "position_read_inside_primitive":
                "const int present = st.ReadPos(id);" in body,
            "signature_has_no_position_parameter":
                re.search(r"charPrimeWritePosEx\(uint8_t id, PrimeTiming &t\)",
                          signature) is not None,
            "domain_checked_before_command":
                "present < 0 || present > 4095" in body,
            "speed_is_the_canonical_constant": "CHAR_SPEED" in body,
            "acc_is_the_canonical_constant": "CHAR_ACC" in body,
        },
    }


def audit_failure_convergence(source):
    """Every prime failure path must end at a verified Torque OFF."""
    fault = firmware_function_body(source, "enterFault")
    prime = firmware_function_body(source, "runPrime")
    return {
        "checks": {
            "fault_commands_torque_off":
                fault.count("charTorqueOffVerified(id,") >= 2,
            "fault_reports_power_removal":
                'Serial.println("REMOVE_SERVO_POWER_NOW");' in fault,
            "fault_declares_no_eeprom_writes":
                'Serial.println("EEPROM_WRITES=NONE");' in fault,
            "fault_never_moves": "WritePosEx" not in fault,
            "fault_sets_recovery_stage": "stage = STAGE_RECOVERY;" in fault,
            "repetition_failure_enters_fault":
                re.search(
                    r"if \(!runPrimeRepetition\(id, i, reason\)\) \{"
                    r"[^}]*?enterFault\(id, reason\);", prime, re.S,
                ) is not None,
            "post_prime_torque_off_verified":
                'charTorqueOffVerified(id, "POST_PRIME")' in prime,
        },
    }


def static_audit(source=None):
    """Enumerate every firmware write primitive and where it may be reached."""
    if source is None:
        source = read_firmware_source()
    lines = source.splitlines()

    sites = []
    for index, line in enumerate(lines):
        if is_comment(line):
            continue
        for primitive in WRITE_PRIMITIVES:
            if re.search(r"\bst\.%s\s*\(" % primitive, line):
                function = _enclosing_function(lines, index)
                sites.append({
                    "primitive": primitive,
                    "line": index + 1,
                    "function": function,
                    "approved": (primitive, function) in APPROVED_WRITE_SITES,
                    "why": APPROVED_WRITE_SITES.get((primitive, function)),
                })
    unapproved = [s for s in sites if not s["approved"]]

    forbidden = []
    for pattern, description in FORBIDDEN_SOURCE_PATTERNS:
        for index, line in enumerate(lines):
            if is_comment(line):
                continue
            if re.search(pattern, line):
                forbidden.append({
                    "pattern": pattern,
                    "description": description,
                    "line": index + 1,
                    "text": line.strip(),
                })

    allowlist = audit_write_allowlist(source)
    motion = audit_motion_primitive(source)
    convergence = audit_failure_convergence(source)

    word_write_calls = re.findall(
        r"charWriteWordVerified\(id, ([A-Z_]+),\s*([A-Za-z0-9_]+),", source
    )

    checks = {
        "every_write_site_approved": not unapproved,
        "no_forbidden_patterns": not forbidden,
        "forbidden_primitives_have_zero_call_sites": not any(
            s["primitive"] in FORBIDDEN_PRIMITIVES for s in sites
        ),
        "torque_limit_is_the_only_word_write":
            word_write_calls == [("REG_TORQUE_LIMIT", "CHAR_TORQUE_LIMIT")],
        "torque_limit_constant_is_300":
            firmware_constant_int(source, "CHAR_TORQUE_LIMIT") == 300,
        "speed_constant_is_365":
            firmware_constant_int(source, "CHAR_SPEED") == 365,
        "acc_constant_is_50":
            firmware_constant_int(source, "CHAR_ACC") == 50,
        "repetitions_is_5":
            firmware_constant_int(source, "PRIME_REPETITIONS") == 5,
        "tool_selects_no_constants":
            firmware_constant(source, "CHARACTERIZER_SELECTS_CONSTANTS")
            == "false",
        "arm_declares_no_writes":
            'Serial.println("EEPROM_WRITES=NONE");' in source,
        "no_provisioner_allocation_table": "PhysicalUnit" not in source,
        "no_canonical_profile_table": "ProfileField" not in source,
        "deferred_markers_present": all(
            "%s_SELECTION=DEFERRED_TO_REVIEW" % name in source
            or "%s=DEFERRED_TO_REVIEW" % name in source
            for name in DEFERRED_CONSTANTS
        ),
        "freeze_block_untouched":
            "PROVISIONER_HARDWARE_FREEZE=UNCHANGED_BLOCKED" in source,
    }
    checks.update(
        {"allowlist_" + k: v for k, v in allowlist["checks"].items()}
    )
    checks.update({"motion_" + k: v for k, v in motion["checks"].items()})
    checks.update(
        {"convergence_" + k: v for k, v in convergence["checks"].items()}
    )

    return {
        "firmware_source": FIRMWARE_SOURCE_PATH,
        "firmware_source_sha256": sha256_file(FIRMWARE_SOURCE_PATH),
        "write_sites": sites,
        "unapproved_write_sites": unapproved,
        "forbidden_pattern_hits": forbidden,
        "write_allowlist": allowlist,
        "motion_primitive": motion,
        "failure_convergence": convergence,
        "primitives_with_zero_call_sites": sorted(
            p for p in WRITE_PRIMITIVES
            if not any(s["primitive"] == p for s in sites)
        ),
        "checks": checks,
        "ok": all(checks.values()),
    }


# --------------------------------------------------------------------------
# Host mirror of the firmware write allowlist
#
# Exhaustively exercised by the self-test over every address in the 71-byte
# map, both widths and a spread of values, in every stage.
# --------------------------------------------------------------------------

def char_write_allowed(addr, width, value, stage):
    if addr == ADDR_TORQUE_ENABLE:
        if width != 1:
            return False
        if value != 0:
            return False
        return stage in (STAGE_PRIME, STAGE_RECOVERY)
    if addr == ADDR_TORQUE_LIMIT:
        if width != 2:
            return False
        if value != CHAR_TORQUE_LIMIT:
            return False
        return stage == STAGE_PRIME
    return False


# --------------------------------------------------------------------------
# Transcript parsing
# --------------------------------------------------------------------------

RE_RAW71 = re.compile(r"^RAW71_HEX=([0-9A-Fa-f]+)$")
RE_SNAPSHOT_BEGIN = re.compile(r"^SNAPSHOT_BEGIN SLOT=([A-Z0-9_]+) ")
RE_SNAPSHOT_END = re.compile(r"^SNAPSHOT_END SLOT=([A-Z0-9_]+)$")


def parse_fields(line):
    """KEY=VALUE pairs from one firmware line, ignoring the leading tag."""
    fields = {}
    for key, value in re.findall(r"\b([A-Z0-9_]+)=(\S*)", line):
        fields[key] = value
    return fields


def field_int(fields, key):
    if key not in fields:
        raise CharacterizationError("missing field %s" % key)
    text = fields[key]
    try:
        return int(text, 16) if text.lower().startswith("0x") else int(text)
    except ValueError:
        raise CharacterizationError("field %s is not an integer: %r"
                                    % (key, text))


def extract_snapshots(lines):
    """RAW71_HEX by slot, with the begin/end framing verified."""
    snapshots = {}
    slot = None
    raw_hex = None
    for line in lines:
        begin = RE_SNAPSHOT_BEGIN.match(line)
        if begin:
            if slot is not None:
                raise CharacterizationError(
                    "nested SNAPSHOT_BEGIN for slot %s" % begin.group(1)
                )
            slot = begin.group(1)
            raw_hex = None
            continue
        if slot is not None:
            payload = RE_RAW71.match(line)
            if payload:
                if raw_hex is not None:
                    raise CharacterizationError(
                        "two RAW71_HEX lines in slot %s" % slot
                    )
                raw_hex = payload.group(1)
                continue
            end = RE_SNAPSHOT_END.match(line)
            if end:
                if end.group(1) != slot:
                    raise CharacterizationError(
                        "SNAPSHOT_END slot %s does not match begin %s"
                        % (end.group(1), slot)
                    )
                if raw_hex is None:
                    raise CharacterizationError(
                        "slot %s ended with no RAW71_HEX" % slot
                    )
                if slot in snapshots:
                    raise CharacterizationError(
                        "duplicate snapshot slot %s" % slot
                    )
                snapshots[slot] = raw_hex
                slot = None
    if slot is not None:
        raise CharacterizationError("slot %s never ended" % slot)
    return snapshots


def parse_arm(lines):
    """The @ARM reply, independently re-gated by the host."""
    result = {
        "discovered_id": None,
        "responders": 0,
        "found": [],
        "token": None,
        "before_digest": None,
        "firmware_gate_pass": False,
        "arm_result": None,
        "declared_no_writes": False,
        "declared_no_motion": False,
    }
    for line in lines:
        if line.startswith("FOUND "):
            f = parse_fields(line)
            result["found"].append({
                "id": field_int(f, "ID"),
                "model": field_int(f, "MODEL"),
                "ping_status": field_int(f, "PING_STATUS"),
                "model_status": field_int(f, "MODEL_STATUS"),
            })
        elif line.startswith("SCAN_RESULT "):
            result["responders"] = field_int(parse_fields(line), "FOUND")
        elif line.startswith("ARM_DISCOVERED_ID="):
            result["discovered_id"] = int(line.split("=", 1)[1])
        elif line.startswith("SESSION_TOKEN="):
            result["token"] = line.split("=", 1)[1]
        elif line.startswith("BEFORE_DIGEST="):
            result["before_digest"] = line.split("=", 1)[1]
        elif line == "GATE_RESULT PASS":
            result["firmware_gate_pass"] = True
        elif line == "EEPROM_WRITES=NONE":
            result["declared_no_writes"] = True
        elif line == "MOTION=NONE":
            result["declared_no_motion"] = True
        elif line.startswith("ARM_RESULT "):
            result["arm_result"] = line.split(" ", 1)[1]
    result["snapshots"] = extract_snapshots(lines)
    return result


def parse_write_records(lines):
    """Every WRITE line the firmware emitted, for the allowlist cross-check."""
    records = []
    for line in lines:
        if not line.startswith("WRITE "):
            continue
        f = parse_fields(line)
        records.append({
            "address": field_int(f, "ADDR"),
            "name": f.get("NAME"),
            "width": field_int(f, "WIDTH"),
            "expect": field_int(f, "EXPECT"),
            "ack": field_int(f, "ACK"),
            "status": field_int(f, "STATUS"),
            "readback": field_int(f, "READBACK"),
            "read_status": field_int(f, "READ_STATUS"),
            "result": f.get("RESULT"),
        })
    return records


def verify_write_records(records):
    """No write outside the two-entry allowlist may appear in a transcript."""
    violations = []
    for record in records:
        key = (record["address"], record["width"], record["expect"])
        if key not in WRITE_ALLOWLIST:
            violations.append(record)
        elif record["result"] == "OK" and record["readback"] != record["expect"]:
            violations.append(record)
    return violations


def parse_prime(lines):
    """Characterization A, re-derived from the transcript."""
    reps = {}

    def rep(index):
        return reps.setdefault(index, {"index": index})

    for line in lines:
        if line.startswith("PRIME_P0 "):
            f = parse_fields(line)
            rep(field_int(f, "INDEX"))["p0"] = field_int(f, "P0")
        elif line.startswith("PRIME_CMD "):
            f = parse_fields(line)
            r = rep(field_int(f, "INDEX"))
            r["goal"] = field_int(f, "GOAL")
            r["speed"] = field_int(f, "SPEED")
            r["acc"] = field_int(f, "ACC")
            r["cmd_ack"] = field_int(f, "ACK")
            r["cmd_status"] = field_int(f, "STATUS")
        elif line.startswith("PRIME_OFF "):
            f = parse_fields(line)
            r = rep(field_int(f, "INDEX"))
            r["off_ack"] = field_int(f, "ACK")
            r["off_status"] = field_int(f, "STATUS")
            r["off_readback"] = field_int(f, "READBACK")
            r["off_read_status"] = field_int(f, "READ_STATUS")
            r["off_result"] = f.get("RESULT")
        elif line.startswith("PRIME_TIMING "):
            f = parse_fields(line)
            r = rep(field_int(f, "INDEX"))
            r["read_us"] = field_int(f, "READ_US")
            r["writeposex_us"] = field_int(f, "WRITEPOSEX_US")
            r["exposure_us"] = field_int(f, "EXPOSURE_US")
            r["total_us"] = field_int(f, "TOTAL_US")
        elif line.startswith("PRIME_DELTA "):
            f = parse_fields(line)
            r = rep(field_int(f, "INDEX"))
            r["p_first"] = field_int(f, "P_FIRST")
            r["p_settled"] = field_int(f, "P_SETTLED")
            r["delta_immediate"] = field_int(f, "DELTA_IMMEDIATE")
            r["delta_ticks"] = field_int(f, "DELTA_TICKS")
            r["peak_abs_delta"] = field_int(f, "PEAK_ABS_DELTA")
            r["min_delta"] = field_int(f, "MIN_DELTA")
            r["max_delta"] = field_int(f, "MAX_DELTA")
        elif line.startswith("PRIME_TELEMETRY "):
            f = parse_fields(line)
            r = rep(field_int(f, "INDEX"))
            for key in ("SAMPLES", "ELAPSED_US", "SETTLED_US",
                        "PEAK_ABS_SPEED", "PEAK_ABS_LOAD", "PEAK_ABS_CURRENT",
                        "CURRENT", "VOLTAGE", "TEMPERATURE", "MIN_VOLTAGE",
                        "MAX_VOLTAGE", "MAX_TEMPERATURE", "STATUS"):
                r[key.lower()] = field_int(f, key)
            r["settled"] = f.get("SETTLED") == "YES"
        elif line.startswith("PRIME_REP_RESULT "):
            f = parse_fields(line)
            r = rep(field_int(f, "INDEX"))
            r["result"] = f.get("RESULT")
            r["reason"] = f.get("REASON")

    ordered = [reps[k] for k in sorted(reps)]
    return {
        "repetitions": ordered,
        "prime_result": next(
            (l.split(" ", 1)[1] for l in lines if l.startswith("PRIME_RESULT ")),
            None,
        ),
        "selection_deferred":
            "PRIME_MAX_DELTA_TICKS_SELECTION=DEFERRED_TO_REVIEW" in lines,
        "snapshots": extract_snapshots(lines),
    }


def verify_prime(parsed):
    """Independent re-derivation of every prime claim the firmware made."""
    findings = []
    reps = parsed["repetitions"]

    if len(reps) != PRIME_REPETITIONS:
        findings.append(
            "expected %d repetitions, transcript has %d"
            % (PRIME_REPETITIONS, len(reps))
        )

    for r in reps:
        index = r.get("index")
        for key in ("p0", "goal", "speed", "acc", "p_settled", "p_first",
                    "delta_ticks", "delta_immediate", "peak_abs_delta"):
            if key not in r:
                findings.append("rep %s missing %s" % (index, key))
        if findings and any(f.startswith("rep %s missing" % index)
                            for f in findings):
            continue

        validate_position_domain(r["p0"], "rep %s P0" % index)
        validate_position_domain(r["p_settled"], "rep %s P_SETTLED" % index)
        validate_position_domain(r["p_first"], "rep %s P_FIRST" % index)

        if r["goal"] != r["p0"]:
            findings.append(
                "rep %s commanded %d but PresentPosition was %d"
                % (index, r["goal"], r["p0"])
            )
        if r["speed"] != CHAR_SPEED:
            findings.append("rep %s speed %s, expected %d"
                            % (index, r["speed"], CHAR_SPEED))
        if r["acc"] != CHAR_ACC:
            findings.append("rep %s acc %s, expected %d"
                            % (index, r["acc"], CHAR_ACC))
        if r.get("cmd_ack") != 1 or r.get("cmd_status") != 0:
            findings.append("rep %s prime write not acknowledged cleanly"
                            % index)
        if r.get("off_result") != "CONFIRMED" or r.get("off_readback") != 0:
            findings.append("rep %s torque OFF not confirmed" % index)
        if r.get("off_ack") != 1 or r.get("off_status") != 0:
            findings.append("rep %s torque OFF write not clean" % index)

        expected = circular_delta(r["p_settled"], r["p0"])
        if r["delta_ticks"] != expected:
            findings.append(
                "rep %s DELTA_TICKS %d, host recomputes %d"
                % (index, r["delta_ticks"], expected)
            )
        expected_immediate = circular_delta(r["p_first"], r["p0"])
        if r["delta_immediate"] != expected_immediate:
            findings.append(
                "rep %s DELTA_IMMEDIATE %d, host recomputes %d"
                % (index, r["delta_immediate"], expected_immediate)
            )
        if abs(r["delta_ticks"]) > r["peak_abs_delta"]:
            findings.append(
                "rep %s settled delta %d exceeds observed peak %d"
                % (index, r["delta_ticks"], r["peak_abs_delta"])
            )
        if r.get("status", 0) != 0:
            findings.append("rep %s status byte 0x%02X"
                            % (index, r.get("status")))
        if r.get("result") != "OK":
            findings.append("rep %s result %s (%s)"
                            % (index, r.get("result"), r.get("reason")))

    deltas = [r["delta_ticks"] for r in reps if "delta_ticks" in r]
    peaks = [r["peak_abs_delta"] for r in reps if "peak_abs_delta" in r]
    exposures = [r["exposure_us"] for r in reps if "exposure_us" in r]

    summary = {
        "repetitions": len(reps),
        "observed_delta_ticks": deltas,
        "delta_min": min(deltas) if deltas else None,
        "delta_max": max(deltas) if deltas else None,
        "delta_abs_max": max((abs(d) for d in deltas), default=None),
        "observed_peak_abs_delta": peaks,
        "peak_abs_delta_max": max(peaks) if peaks else None,
        "exposure_us": exposures,
        "exposure_us_min": min(exposures) if exposures else None,
        "exposure_us_max": max(exposures) if exposures else None,
        "prime_parameters": {
            "torque_limit": CHAR_TORQUE_LIMIT,
            "speed": CHAR_SPEED,
            "acc": CHAR_ACC,
        },
        "PRIME_MAX_DELTA_TICKS": "DEFERRED_TO_REVIEW",
    }
    if not parsed.get("selection_deferred"):
        findings.append("firmware did not declare the selection deferred")

    return {"findings": findings, "ok": not findings, "summary": summary}


# --------------------------------------------------------------------------
# Characterization B — independent re-derivation of the power-cycle trace
#
# The firmware emits both a per-poll record and its own episode table. The host
# rebuilds the episode table from the per-poll records alone and the two must
# agree. Nothing here classifies an absence: every episode is reported with its
# measured duration and a human picks the thresholds afterwards.
# --------------------------------------------------------------------------

def parse_trace_polls(lines, trace_index):
    polls = []
    for line in lines:
        if not line.startswith("TRACE "):
            continue
        f = parse_fields(line)
        if field_int(f, "TRACE") != trace_index:
            continue
        polls.append({
            "n": field_int(f, "N"),
            "t_ms": field_int(f, "T_MS"),
            "dur_us": field_int(f, "DUR_US"),
            "ping": field_int(f, "PING"),
            "ping_status": field_int(f, "PSTAT"),
            "model": field_int(f, "MODEL"),
            "model_status": field_int(f, "MSTAT"),
            "clean": field_int(f, "CLEAN") == 1,
            "ping_streak": field_int(f, "PSTREAK"),
            "clean_streak": field_int(f, "CSTREAK"),
        })
    return polls


def parse_trace_episodes(lines, trace_index):
    episodes = []
    for line in lines:
        if not line.startswith("TRACE_EPISODE "):
            continue
        f = parse_fields(line)
        if field_int(f, "TRACE") != trace_index:
            continue
        episodes.append({
            "index": field_int(f, "INDEX"),
            "last_present_t_ms": field_int(f, "LAST_PRESENT_T_MS"),
            "first_absent_t_ms": field_int(f, "FIRST_ABSENT_T_MS"),
            "first_return_t_ms": field_int(f, "FIRST_RETURN_T_MS"),
            "absent_polls": field_int(f, "ABSENT_POLLS"),
            "duration_ms": field_int(f, "DURATION_MS"),
            "gap_ms": field_int(f, "GAP_MS"),
            "ended": field_int(f, "ENDED") == 1,
            "stable": field_int(f, "STABLE") == 1,
            "return_to_stable_ms": field_int(f, "RETURN_TO_STABLE_MS"),
            "return_to_stable_polls": field_int(f, "RETURN_TO_STABLE_POLLS"),
            "unclean_after_return": field_int(f, "UNCLEAN_AFTER_RETURN"),
            "streak_breaks": field_int(f, "STREAK_BREAKS"),
        })
    return episodes


def derive_trace_episodes(polls, servo_id, seed_last_present_ms,
                          stable_target=TRACE_STABLE_TARGET):
    """Rebuild the absence episodes from the per-poll records alone.

    seed_last_present_ms is the last present timestamp before the per-poll log
    starts. The firmware only logs per-poll records once it is observing, and
    the first such record is already the first absent poll, so that one value
    cannot be derived from the log and is taken from the firmware.
    """
    episodes = []
    in_absence = False
    active = None
    last_present = seed_last_present_ms
    clean_streak = 0
    polls_since_return = 0

    for poll in polls:
        t = poll["t_ms"]
        ping_ok = poll["ping"] == servo_id and poll["ping_status"] == 0
        clean = ping_ok and poll["model"] == EXPECTED_MODEL \
            and poll["model_status"] == 0

        if not ping_ok:
            clean_streak = 0
            if not in_absence:
                in_absence = True
                active = {
                    "index": len(episodes) + 1,
                    "last_present_t_ms": last_present,
                    "first_absent_t_ms": t,
                    "first_return_t_ms": 0,
                    "absent_polls": 1,
                    "duration_ms": 0,
                    "gap_ms": 0,
                    "ended": False,
                    "stable": False,
                    "return_to_stable_ms": 0,
                    "return_to_stable_polls": 0,
                    "unclean_after_return": 0,
                    "streak_breaks": 0,
                }
                episodes.append(active)
            else:
                active["absent_polls"] += 1
            continue

        last_present = t
        if in_absence:
            in_absence = False
            polls_since_return = 0
            active["first_return_t_ms"] = t
            active["duration_ms"] = t - active["first_absent_t_ms"]
            active["gap_ms"] = t - active["last_present_t_ms"]
            active["ended"] = True

        if active is not None and active["ended"]:
            polls_since_return += 1

        if clean:
            clean_streak += 1
        else:
            if (clean_streak > 0 and active is not None and active["ended"]
                    and not active["stable"]):
                active["streak_breaks"] += 1
            clean_streak = 0
            if active is not None and active["ended"] and not active["stable"]:
                active["unclean_after_return"] += 1

        if (active is not None and active["ended"] and not active["stable"]
                and clean_streak >= stable_target):
            active["stable"] = True
            active["return_to_stable_ms"] = t - active["first_return_t_ms"]
            active["return_to_stable_polls"] = polls_since_return

    return episodes


EPISODE_COMPARED_FIELDS = (
    "index", "first_absent_t_ms", "first_return_t_ms", "absent_polls",
    "duration_ms", "gap_ms", "ended", "stable", "return_to_stable_ms",
    "return_to_stable_polls", "unclean_after_return", "streak_breaks",
)


def compare_trace_episodes(derived, firmware):
    findings = []
    if len(derived) != len(firmware):
        findings.append(
            "host derived %d episodes, firmware reported %d"
            % (len(derived), len(firmware))
        )
    for host_ep, fw_ep in zip(derived, firmware):
        for key in EPISODE_COMPARED_FIELDS:
            if host_ep[key] != fw_ep[key]:
                findings.append(
                    "episode %s field %s: host %r, firmware %r"
                    % (fw_ep["index"], key, host_ep[key], fw_ep[key])
                )
    return findings


def verify_trace(lines, trace_index, servo_id):
    polls = parse_trace_polls(lines, trace_index)
    firmware_episodes = parse_trace_episodes(lines, trace_index)

    findings = []
    if not polls:
        findings.append("trace %d has no per-poll records" % trace_index)
    if not firmware_episodes:
        findings.append("trace %d has no episode table" % trace_index)

    seed = firmware_episodes[0]["last_present_t_ms"] if firmware_episodes else 0
    derived = derive_trace_episodes(polls, servo_id, seed)
    findings.extend(compare_trace_episodes(derived, firmware_episodes))

    if polls and firmware_episodes:
        if polls[0]["t_ms"] != firmware_episodes[0]["first_absent_t_ms"]:
            findings.append(
                "first logged poll t=%d is not the first absent poll t=%d"
                % (polls[0]["t_ms"], firmware_episodes[0]["first_absent_t_ms"])
            )

    summary_line = next(
        (l for l in lines
         if l.startswith("TRACE_SUMMARY ")
         and field_int(parse_fields(l), "TRACE") == trace_index),
        None,
    )
    summary = parse_fields(summary_line) if summary_line else {}

    ended = [e for e in derived if e["ended"]]
    stable = [e for e in ended if e["stable"]]

    deferred = (
        "COLD_ABSENCE_DEBOUNCE_MS_SELECTION=DEFERRED_TO_REVIEW" in lines
        and "COLD_RETURN_STABLE_SAMPLES_SELECTION=DEFERRED_TO_REVIEW" in lines
    )
    if not deferred:
        findings.append("trace %d did not declare the selections deferred"
                        % trace_index)

    return {
        "trace_index": trace_index,
        "polls": len(polls),
        "episodes": derived,
        "firmware_episodes": firmware_episodes,
        "episode_1_gap_ms_seeded_from_firmware": bool(firmware_episodes),
        "absence_durations_ms": [e["duration_ms"] for e in ended],
        "longest_absence_ms": max((e["duration_ms"] for e in ended),
                                  default=None),
        "shortest_absence_ms": min((e["duration_ms"] for e in ended),
                                   default=None),
        "return_to_stable_ms": [e["return_to_stable_ms"] for e in stable],
        "return_to_stable_polls": [e["return_to_stable_polls"]
                                   for e in stable],
        "unclean_after_return": [e["unclean_after_return"] for e in ended],
        "stable_target_samples": TRACE_STABLE_TARGET,
        "termination": summary.get("TERMINATION"),
        "usable": bool(stable),
        "findings": findings,
        "ok": not findings,
        "COLD_ABSENCE_DEBOUNCE_MS": "DEFERRED_TO_REVIEW",
        "COLD_RETURN_STABLE_SAMPLES": "DEFERRED_TO_REVIEW",
    }


# --------------------------------------------------------------------------
# Physical label validation — cross-checked against the frozen provisioner
# --------------------------------------------------------------------------

def provisioner_allocation():
    """Read-only parse of the provisioner's allocation table."""
    if not os.path.isfile(PROVISIONER_SOURCE_PATH):
        raise CharacterizationError(
            "provisioner source missing: %s" % PROVISIONER_SOURCE_PATH
        )
    with open(PROVISIONER_SOURCE_PATH, encoding="utf-8") as handle:
        source = handle.read()
    block = re.search(r"static const PhysicalUnit UNITS\[\] = \{(.*?)\n\};",
                      source, re.S)
    if not block:
        raise CharacterizationError("provisioner UNITS table not found")
    entries = re.findall(
        r"\{\"([A-Z0-9]+)\",\s*(\d+),\s*\"([A-Z_]+)\",\s*(\d+)\}",
        block.group(1),
    )
    return {label: (int(target), joint, int(cold))
            for label, target, joint, cold in entries}


def validate_physical_label(label):
    if not isinstance(label, str) or not re.fullmatch(r"[A-Z0-9]{1,15}",
                                                      label or ""):
        raise CharacterizationError("invalid physical label: %r" % (label,))
    allocation = provisioner_allocation()
    if label not in allocation:
        raise CharacterizationError(
            "%s is not a MATDOG physical label" % label
        )
    return label


# --------------------------------------------------------------------------
# Transports
# --------------------------------------------------------------------------

class SerialTransport:
    """The only object in this file that can reach a serial port."""

    def __init__(self, port, console_log, gate):
        import serial   # noqa: PLC0415 - deliberate lazy import
        import time     # noqa: PLC0415

        self._time = time
        self._log = console_log
        self._gate = gate

        ser = serial.Serial()
        ser.port = port
        ser.baudrate = 115200
        ser.timeout = 3
        ser.write_timeout = 3
        ser.dtr = False
        ser.rts = False
        ser.open()

        # Opening may reset USB-CDC once. Acceptable before the session starts.
        time.sleep(2.2)
        ser.reset_input_buffer()
        self._ser = ser

    def send(self, command):
        checked = self._gate.check(assert_command_allowed(command))
        self._gate.note_sent(checked)
        self._record(">>> " + checked)
        self._ser.write((checked + "\n").encode("ascii"))
        self._ser.flush()
        return checked

    def read_until(self, terminators, timeout):
        deadline = self._time.monotonic() + timeout
        lines = []
        while self._time.monotonic() < deadline:
            raw = self._ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            self._record(line)
            lines.append(line)
            if any(line.startswith(term) for term in terminators):
                return lines
        raise CharacterizationError(
            "timeout after %.1fs waiting for %s" % (timeout, terminators)
        )

    def _record(self, line):
        print(line, flush=True)
        if self._log is not None:
            self._log.write(line + "\n")
            self._log.flush()

    def close(self):
        self._ser.close()


class ScriptedTransport:
    """Offline test double. Opens nothing and owns no port."""

    def __init__(self, script, gate, console_log=None):
        self.script = {k: list(v) for k, v in script.items()}
        self.sent = []
        self._gate = gate
        self._log = console_log

    def send(self, command):
        checked = self._gate.check(assert_command_allowed(command))
        self._gate.note_sent(checked)
        self.sent.append(checked)
        return checked

    def read_until(self, terminators, timeout):
        if not self.sent:
            raise CharacterizationError("read_until before any command")
        command = self.sent[-1]
        replies = self.script.get(command)
        if not replies:
            raise CharacterizationError(
                "scripted transport has no reply for %r" % command
            )
        lines = replies.pop(0)
        if self._log is not None:
            for line in lines:
                self._log.write(line + "\n")
        for index, line in enumerate(lines):
            if any(line.startswith(term) for term in terminators):
                return lines[:index + 1]
        raise CharacterizationError(
            "scripted reply never reached %s" % (terminators,)
        )

    def close(self):
        pass


# --------------------------------------------------------------------------
# Session
# --------------------------------------------------------------------------

def build_record(label, port, session_dir, audit):
    return {
        "schema": REPORT_SCHEMA,
        "tool": TOOL_NAME,
        "tool_version": TOOL_VERSION,
        "runner_sha256": sha256_file(os.path.abspath(__file__)),
        "firmware_source_sha256": audit["firmware_source_sha256"],
        "firmware_fqbn": FIRMWARE_FQBN,
        "unit": label,
        "port": port,
        "session_dir": session_dir,
        "started_utc": utc_now_iso(),
        "command_surface": command_surface_audit(),
        "static_audit_ok": audit["ok"],
        "static_audit_checks": audit["checks"],
        "constants_selected_by_this_tool": [],
        "constants_deferred_to_review": {
            name: "DEFERRED_TO_REVIEW" for name in DEFERRED_CONSTANTS
        },
        "provisioner_hardware_freeze": "UNCHANGED_BLOCKED",
        "artifacts": {},
        "phases": {},
        "verdict": "INCOMPLETE",
        "reason": None,
    }


def write_report(record, session_dir):
    path = os.path.join(session_dir, REPORT_NAME)
    record["finished_utc"] = utc_now_iso()
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(record, handle, indent=2, sort_keys=True)
        handle.flush()
        os.fsync(handle.fileno())
    return path


def write_sha256sums(session_dir):
    path = os.path.join(session_dir, SHA256SUMS_NAME)
    names = sorted(
        n for n in os.listdir(session_dir) if n != SHA256SUMS_NAME
    )
    with open(path, "w", encoding="utf-8") as handle:
        for name in names:
            full = os.path.join(session_dir, name)
            if os.path.isfile(full):
                handle.write("%s  %s\n" % (sha256_file(full), name))
        handle.flush()
        os.fsync(handle.fileno())
    return path


def run_arm_phase(transport, record, session_dir, gate):
    transport.send("@ARM")
    lines = transport.read_until(("ARM_RESULT",), ARM_TIMEOUT_S)
    parsed = parse_arm(lines)

    if parsed["arm_result"] != "PASS":
        raise CharacterizationError("ARM failed: %s" % parsed["arm_result"])
    if "BEFORE" not in parsed["snapshots"]:
        raise CharacterizationError("ARM returned no BEFORE snapshot")

    # Persist FIRST. decode() runs only after the bytes are durable.
    raw, decoded = capture_snapshot(
        "BEFORE", parsed["snapshots"]["BEFORE"], session_dir,
        on_persisted=gate.note_persisted,
    )
    record["artifacts"]["BEFORE"] = decoded["raw_artifact"]

    if parsed["responders"] != 1:
        raise CharacterizationError(
            "expected exactly one responder, firmware reported %d"
            % parsed["responders"]
        )
    scan = parsed["found"][0]
    host_gate = evaluate_discovery_gate(
        parsed["responders"], scan["ping_status"], scan["model_status"],
        scan["model"], raw,
    )
    if not host_gate["pass"]:
        raise CharacterizationError(
            "host discovery gate failed: %s" % host_gate["failures"]
        )
    if not parsed["firmware_gate_pass"]:
        raise CharacterizationError("firmware gate did not pass")
    if decoded["id"] != parsed["discovered_id"]:
        raise CharacterizationError("ID register disagrees with the scan")
    if not (parsed["declared_no_writes"] and parsed["declared_no_motion"]):
        raise CharacterizationError("ARM did not declare zero writes/motion")

    record["phases"]["arm"] = {
        "discovered_id": parsed["discovered_id"],
        "responders": parsed["responders"],
        "token_bound": bool(parsed["token"]),
        "before": {k: v for k, v in decoded.items() if k != "raw_artifact"},
        "host_gate": host_gate["checks"],
    }
    return parsed


def run_prime_phase(transport, record, session_dir, gate, token):
    transport.send("@PRIME " + token)
    lines = transport.read_until(("PRIME_RESULT",), PRIME_TIMEOUT_S)

    writes = parse_write_records(lines)
    violations = verify_write_records(writes)
    if violations:
        raise SafetyViolation(
            "firmware emitted writes outside the allowlist: %s" % violations
        )

    parsed = parse_prime(lines)
    if parsed["prime_result"] != "PASS":
        raise CharacterizationError(
            "prime characterization failed: see FAULT lines"
        )

    verified = verify_prime(parsed)
    if not verified["ok"]:
        raise CharacterizationError(
            "host disagrees with the firmware prime record: %s"
            % verified["findings"]
        )

    if "AFTER_PRIME" in parsed["snapshots"]:
        _raw, decoded = capture_snapshot(
            "AFTER_PRIME", parsed["snapshots"]["AFTER_PRIME"], session_dir,
            on_persisted=gate.note_persisted,
        )
        record["artifacts"]["AFTER_PRIME"] = decoded["raw_artifact"]
        if decoded["torque_enable"] != 0:
            raise SafetyViolation("torque is not off after prime")

    record["phases"]["prime"] = {
        "repetitions": parsed["repetitions"],
        "summary": verified["summary"],
        "write_records": writes,
        "writes_within_allowlist": True,
    }
    return verified


def run_trace_phase(transport, record, session_dir, gate, token, trace_index,
                    servo_id, announce=True):
    if announce:
        print()
        print("=" * 70)
        print(" POWER CYCLE %d — switch the servo rail OFF, then ON."
              % trace_index)
        print(" Leave the ESP32 USB cable connected. No typed confirmation.")
        print("=" * 70)
        print()

    transport.send("@TRACE " + token)
    lines = transport.read_until(("TRACE_RESULT",), TRACE_TIMEOUT_S)

    writes = parse_write_records(lines)
    if writes:
        raise SafetyViolation(
            "the trace phase is read-only but emitted writes: %s" % writes
        )

    verified = verify_trace(lines, trace_index, servo_id)
    if not verified["ok"]:
        raise CharacterizationError(
            "host disagrees with the firmware trace record: %s"
            % verified["findings"]
        )

    snapshots = extract_snapshots(lines)
    slot = "COLD1" if trace_index == 1 else "COLD2"
    if slot in snapshots:
        _raw, decoded = capture_snapshot(
            slot, snapshots[slot], session_dir,
            on_persisted=gate.note_persisted,
        )
        record["artifacts"][slot] = decoded["raw_artifact"]
        # Observed, never asserted: the unit is not provisioned and its RAM
        # TorqueLimit is expected to have reverted across the power cycle.
        verified["cold_observed"] = {
            k: v for k, v in decoded.items() if k != "raw_artifact"
        }

    record["phases"].setdefault("traces", []).append(verified)
    return verified


def run_characterization(label, port, session_root, traces):
    validate_physical_label(label)
    audit = static_audit()
    if not audit["ok"]:
        raise SafetyViolation(
            "firmware static audit failed: %s"
            % [k for k, v in audit["checks"].items() if not v]
        )

    session_dir = create_session_dir(session_root, label)
    gate = EvidenceGate()
    record = build_record(label, port, session_dir, audit)

    console_path = os.path.join(session_dir, CONSOLE_LOG_NAME)
    console_log = open(console_path, "x", encoding="utf-8")
    transport = None
    try:
        transport = SerialTransport(port, console_log, gate)
        arm = run_arm_phase(transport, record, session_dir, gate)
        token = arm["token"]
        servo_id = arm["discovered_id"]

        run_prime_phase(transport, record, session_dir, gate, token)

        for index in range(1, traces + 1):
            run_trace_phase(transport, record, session_dir, gate, token,
                            index, servo_id)

        record["verdict"] = "PASS"
    except (CharacterizationError, SafetyViolation) as exc:
        record["verdict"] = "FAIL"
        record["reason"] = "%s: %s" % (type(exc).__name__, exc)
        raise
    finally:
        if transport is not None:
            transport.close()
        record["evidence_ordering"] = gate.ordering_report()
        if not record["evidence_ordering"]["evidence_precedes_write"] \
                and record["verdict"] == "PASS":
            record["verdict"] = "FAIL"
            record["reason"] = "evidence ordering could not be proven"
        console_log.close()
        write_report(record, session_dir)
        write_sha256sums(session_dir)
        print("\nEVIDENCE: %s" % session_dir)

    return record


# --------------------------------------------------------------------------
# Offline self-test
# --------------------------------------------------------------------------

# Real ST-3215-C018 hardware bytes: physical unit NEW01, source ID 1, captured
# 2026-08-26 by the frozen read-only survey. PositionOffset +85,
# PresentPosition 254, TorqueLimit 1000, Lock 1, TorqueEnable 0.
GOLDEN_NEW01_HEX = (
    "030A000903010000010000FF0F468C28E8030C2C2F2020001000010136010155"
    "000014C8500AC8C80000000000000000E803000000000001FE0000000000701F"
    "000000FE000000"
)


class SelfTest:
    def __init__(self):
        self.passed = 0
        self.failed = []

    def check(self, name, condition, detail=""):
        if condition:
            self.passed += 1
            print("  PASS  %s" % name)
        else:
            self.failed.append(name)
            print("  FAIL  %s %s" % (name, detail))

    def expect_raises(self, name, exc_types, fn, *args, **kwargs):
        try:
            fn(*args, **kwargs)
        except exc_types:
            self.passed += 1
            print("  PASS  %s" % name)
            return
        except Exception as exc:                        # noqa: BLE001
            self.failed.append(name)
            print("  FAIL  %s (wrong exception %r)" % (name, exc))
            return
        self.failed.append(name)
        print("  FAIL  %s (no exception raised)" % name)


def _frame_with(base, **fields):
    """Build a synthetic 71-byte frame from the golden one.

    Key form: b_<hexaddr> for a byte, w_<hexaddr> for a little-endian word.
    """
    frame = bytearray(base)
    for key, value in fields.items():
        kind, addr_text = key.split("_", 1)
        addr = int(addr_text, 16)
        if kind == "w":
            frame[addr] = value & 0xFF
            frame[addr + 1] = (value >> 8) & 0xFF
        else:
            frame[addr] = value & 0xFF
    return bytes(frame)


def _arm_transcript(source_id=1, token="1A2B3C4D", raw_hex=GOLDEN_NEW01_HEX,
                    responders=1, model=777, ping_status=0, model_status=0):
    lines = [
        "SCOPE_BEGIN",
        "  SELECTS_CONSTANTS=NO",
        "SCOPE_END",
        "SCAN_BEGIN RANGE=0..253",
    ]
    for i in range(responders):
        lines.append(
            "FOUND ID=%d MODEL=%d PING_STATUS=0x%02X MODEL_STATUS=0x%02X"
            % (source_id + i, model, ping_status, model_status)
        )
    lines += [
        "SCAN_RESULT FOUND=%d" % responders,
        "SCAN_COMPLETE",
        "ARM_DISCOVERED_ID=%d" % source_id,
        "SNAPSHOT_BEGIN SLOT=BEFORE ID=%d START=0x00 LEN=71" % source_id,
        "RAW71_HEX=" + raw_hex,
        "SNAPSHOT_RAW_COMPLETE SLOT=BEFORE",
        "SNAPSHOT_END SLOT=BEFORE",
        "GATE_RESULT PASS",
        "BEFORE_DIGEST=DEADBEEF",
        "PRIME_REPETITIONS=5",
        "SESSION_TOKEN=%s" % token,
        "EEPROM_WRITES=NONE",
        "MOTION=NONE",
        "ARM_RESULT PASS",
        "WAIT_PRIME",
    ]
    return lines


def _prime_rep_lines(index, p0=254, p_first=None, p_settled=None,
                     speed=CHAR_SPEED, acc=CHAR_ACC, goal=None,
                     peak=None, status=0, result="OK"):
    p_first = p0 if p_first is None else p_first
    p_settled = p0 if p_settled is None else p_settled
    goal = p0 if goal is None else goal
    delta = circular_delta(p_settled, p0)
    delta_immediate = circular_delta(p_first, p0)
    peak = max(abs(delta), abs(delta_immediate)) if peak is None else peak
    return [
        "PRIME_REP_BEGIN INDEX=%d OF=5" % index,
        "WRITE ADDR=0x28 NAME=TorqueEnable WIDTH=1 EXPECT=0 ACK=1 STATUS=0x00 "
        "READBACK=0 READ_STATUS=0x00 RESULT=OK",
        "TORQUE_OFF CONTEXT=PRE_PRIME RESULT=CONFIRMED",
        "WRITE ADDR=0x30 NAME=TorqueLimit WIDTH=2 EXPECT=300 ACK=1 "
        "STATUS=0x00 READBACK=300 READ_STATUS=0x00 RESULT=OK",
        "PRIME_P0 INDEX=%d P0=%d" % (index, p0),
        "PRIME_CMD INDEX=%d GOAL=%d SPEED=%d ACC=%d ACK=1 STATUS=0x00"
        % (index, goal, speed, acc),
        "PRIME_OFF INDEX=%d ACK=1 STATUS=0x00 READBACK=0 READ_STATUS=0x00 "
        "RESULT=CONFIRMED" % index,
        "PRIME_TIMING INDEX=%d READ_US=430 WRITEPOSEX_US=812 EXPOSURE_US=340 "
        "TOTAL_US=1152" % index,
        "PRIME_DELTA INDEX=%d P0=%d P_FIRST=%d P_SETTLED=%d "
        "DELTA_IMMEDIATE=%d DELTA_TICKS=%d PEAK_ABS_DELTA=%d MIN_DELTA=%d "
        "MAX_DELTA=%d"
        % (index, p0, p_first, p_settled, delta_immediate, delta, peak,
           min(0, delta, delta_immediate), max(0, delta, delta_immediate)),
        "PRIME_TELEMETRY INDEX=%d SAMPLES=165 ELAPSED_US=330000 SETTLED=YES "
        "SETTLED_US=330000 PEAK_ABS_SPEED=0 PEAK_ABS_LOAD=0 "
        "PEAK_ABS_CURRENT=0 CURRENT=0 VOLTAGE=120 TEMPERATURE=32 "
        "MIN_VOLTAGE=119 MAX_VOLTAGE=120 MAX_TEMPERATURE=32 STATUS=0x%02X"
        % (index, status),
        "PRIME_REP_RESULT INDEX=%d RESULT=%s" % (index, result),
    ]


def _prime_transcript(reps=None, raw_hex=GOLDEN_NEW01_HEX, result="PASS"):
    lines = []
    for index in range(1, PRIME_REPETITIONS + 1):
        override = (reps or {}).get(index, {})
        lines.extend(_prime_rep_lines(index, **override))
    lines += [
        "WRITE ADDR=0x28 NAME=TorqueEnable WIDTH=1 EXPECT=0 ACK=1 STATUS=0x00 "
        "READBACK=0 READ_STATUS=0x00 RESULT=OK",
        "TORQUE_OFF CONTEXT=POST_PRIME RESULT=CONFIRMED",
        "SNAPSHOT_BEGIN SLOT=AFTER_PRIME ID=1 START=0x00 LEN=71",
        "RAW71_HEX=" + raw_hex,
        "SNAPSHOT_RAW_COMPLETE SLOT=AFTER_PRIME",
        "SNAPSHOT_END SLOT=AFTER_PRIME",
        "PRIME_MAX_DELTA_TICKS_SELECTION=DEFERRED_TO_REVIEW",
        "EEPROM_WRITES=NONE",
        "PRIME_RESULT %s" % result,
        "WAIT_TRACE",
    ]
    return lines


def _poll(trace, n, t_ms, present, clean=True, servo_id=1, ep=0,
          pstreak=0, cstreak=0):
    ping = servo_id if present else -1
    pstat = 0x00 if present else 0xFF
    model = (777 if clean else 12345) if present else -1
    mstat = 0x00 if present else 0xFF
    return ("TRACE TRACE=%d N=%d T_MS=%d DUR_US=%d PING=%d PSTAT=0x%02X "
            "MODEL=%d MSTAT=0x%02X CLEAN=%d PSTREAK=%d CSTREAK=%d EP=%d"
            % (trace, n, t_ms, 100000 if not present else 500, ping, pstat,
               model, mstat, 1 if (present and clean) else 0, pstreak,
               cstreak, ep))


def _trace_transcript(trace=1, servo_id=1, last_present_ms=1000,
                      absent_polls=14, unclean_returns=0, stable_target=20,
                      raw_hex=GOLDEN_NEW01_HEX, extra_glitch=False):
    """A synthetic power-cycle trace with exact, hand-computable timing."""
    lines = ["TRACE_MODE=READ_ONLY",
             "TRACE_PHASE TRACE=%d PHASE=OBSERVE T_MS=1120" % trace]
    polls = []
    n = 0
    t = 1120
    episodes = []

    def absence(start_t, count, last_present, ep_index):
        nonlocal n, t
        first_absent = start_t
        for i in range(count):
            n += 1
            polls.append(_poll(trace, n, start_t + i * 120, False,
                               servo_id=servo_id, ep=ep_index))
        t = start_t + count * 120
        return {"first_absent": first_absent, "last_present": last_present}

    if extra_glitch:
        # One isolated failed poll, then an immediate clean return. This is
        # exactly the case a debounce has to survive, so the tool records it
        # as its own episode rather than deciding it is noise.
        info = absence(t, 1, last_present_ms, 1)
        episodes.append((info, 1))
        present_start = t
        for i in range(5):
            n += 1
            polls.append(_poll(trace, n, present_start + i * 21, True,
                               servo_id=servo_id, ep=1))
        t = present_start + 5 * 21
        last_present_ms = present_start + 4 * 21
        ep_index = 2
    else:
        ep_index = 1

    info = absence(t, absent_polls, last_present_ms, ep_index)
    episodes.append((info, ep_index))

    return_t = t
    clean_index = 0
    for i in range(unclean_returns):
        n += 1
        polls.append(_poll(trace, n, return_t + i * 21, True, clean=False,
                           servo_id=servo_id, ep=ep_index))
    for i in range(stable_target + 5):
        n += 1
        polls.append(_poll(trace, n,
                           return_t + (unclean_returns + i) * 21, True,
                           servo_id=servo_id, ep=ep_index))
        clean_index += 1
    lines.extend(polls)

    # Episode table, computed the same way the firmware computes it.
    derived = derive_trace_episodes(
        parse_trace_polls(polls, trace), servo_id,
        episodes[0][0]["last_present"], stable_target,
    )
    for ep in derived:
        lines.append(
            "TRACE_EPISODE TRACE=%d INDEX=%d LAST_PRESENT_T_MS=%d "
            "FIRST_ABSENT_T_MS=%d FIRST_RETURN_T_MS=%d ABSENT_POLLS=%d "
            "DURATION_MS=%d GAP_MS=%d ENDED=%d STABLE=%d "
            "RETURN_TO_STABLE_MS=%d RETURN_TO_STABLE_POLLS=%d "
            "UNCLEAN_AFTER_RETURN=%d STREAK_BREAKS=%d"
            % (trace, ep["index"], ep["last_present_t_ms"],
               ep["first_absent_t_ms"], ep["first_return_t_ms"],
               ep["absent_polls"], ep["duration_ms"], ep["gap_ms"],
               1 if ep["ended"] else 0, 1 if ep["stable"] else 0,
               ep["return_to_stable_ms"], ep["return_to_stable_polls"],
               ep["unclean_after_return"], ep["streak_breaks"])
        )
    lines += [
        "TRACE_SUMMARY TRACE=%d TERMINATION=STABLE_AFTER_ABSENCE POLLS=%d "
        "CLEAN=1 FAIL=1 EPISODES=%d ENDED=%d STABLE=1 OVERFLOW=0 "
        "LONGEST_ABSENCE_MS=%d SHORTEST_ABSENCE_MS=%d ARM_ELAPSED_MS=120 "
        "OBSERVE_ELAPSED_MS=1000"
        % (trace, n, len(derived), len(derived),
           max(e["duration_ms"] for e in derived),
           min(e["duration_ms"] for e in derived)),
        "TRACE_RESOLUTION TRACE=%d PRESENT_MS=20 ABSENT_MS=100 "
        "BASIS=SCSERIAL_IOTIMEOUT" % trace,
        "COLD_ABSENCE_DEBOUNCE_MS_SELECTION=DEFERRED_TO_REVIEW",
        "COLD_RETURN_STABLE_SAMPLES_SELECTION=DEFERRED_TO_REVIEW",
        "SNAPSHOT_BEGIN SLOT=COLD%d ID=%d START=0x00 LEN=71"
        % (trace, servo_id),
        "RAW71_HEX=" + raw_hex,
        "SNAPSHOT_RAW_COMPLETE SLOT=COLD%d" % trace,
        "SNAPSHOT_END SLOT=COLD%d" % trace,
        "EEPROM_WRITES=NONE",
        "TRACE_RESULT INDEX=%d RESULT=PASS REASON=STABLE_AFTER_ABSENCE"
        % trace,
    ]
    return lines


def _fail_transcript():
    """A prime that moved too far, converging to Torque OFF."""
    return _prime_rep_lines(1) + [
        "PRIME_REP_BEGIN INDEX=2 OF=5",
        "PRIME_P0 INDEX=2 P0=254",
        "PRIME_CMD INDEX=2 GOAL=254 SPEED=365 ACC=50 ACK=1 STATUS=0x00",
        "PRIME_OFF INDEX=2 ACK=1 STATUS=0x00 READBACK=0 READ_STATUS=0x00 "
        "RESULT=CONFIRMED",
        "PRIME_REP_RESULT INDEX=2 RESULT=FAIL REASON=GROSS_MOTION_ABORT",
        "FAULT REASON=GROSS_MOTION_ABORT",
        "WRITE ADDR=0x28 NAME=TorqueEnable WIDTH=1 EXPECT=0 ACK=1 STATUS=0x00 "
        "READBACK=0 READ_STATUS=0x00 RESULT=OK",
        "TORQUE_OFF CONTEXT=RECOVERY RESULT=CONFIRMED",
        "RECOVERY TORQUE_OFF=OK",
        "EEPROM_WRITES=NONE",
        "REMOVE_SERVO_POWER_NOW",
        "PRIME_RESULT FAIL",
    ]


def run_self_test():
    import tempfile                                     # noqa: PLC0415

    print("=" * 70)
    print(" MATDOG ST3215 CHARACTERIZE V1 — OFFLINE SELF-TEST")
    print("=" * 70)
    print("No serial port is opened. No hardware is contacted.")
    print()

    test = SelfTest()
    golden = parse_raw71_hex(GOLDEN_NEW01_HEX)
    decoded = decode_snapshot(golden)
    source = read_firmware_source()
    audit = static_audit(source)

    # ---------------------------------------------------------------- 1
    print("[1] Golden NEW01 hardware vector")
    test.check("71 bytes", len(golden) == SNAPSHOT_LEN, len(golden))
    test.check("model 777 at 0x03", decoded["model"] == 777, decoded["model"])
    test.check("id 1", decoded["id"] == 1, decoded["id"])
    test.check("baud 0", decoded["baud"] == 0, decoded["baud"])
    test.check("ResponseStatus 1", decoded["response_status"] == 1,
               decoded["response_status"])
    test.check("PositionOffset +85", decoded["position_offset"] == 85,
               decoded["position_offset"])
    test.check("PresentPosition 254", decoded["present_position"] == 254,
               decoded["present_position"])
    test.check("TorqueEnable 0", decoded["torque_enable"] == 0)
    test.check("TorqueLimit 1000", decoded["torque_limit"] == 1000)
    test.check("Lock 1", decoded["lock"] == 1)
    test.check("physical raw 339", decoded["physical_raw"] == 339,
               decoded["physical_raw"])

    # ---------------------------------------------------------------- 2
    print("[2] Modular position distance around the 0/4095 wrap")
    test.check("delta(0,4095) == +1", circular_delta(0, 4095) == 1)
    test.check("delta(4095,0) == -1", circular_delta(4095, 0) == -1)
    test.check("delta(3,4093) == +6", circular_delta(3, 4093) == 6)
    test.check("delta(4093,3) == -6", circular_delta(4093, 3) == -6)
    test.check("delta(x,x) == 0", all(circular_delta(x, x) == 0
                                      for x in range(0, 4096, 7)))
    test.check("half turn resolves negative",
               circular_delta(2048, 0) == -2048)
    test.check("delta never leaves -2048..2047",
               all(-2048 <= circular_delta(a, b) <= 2047
                   for a in range(0, 4096, 337) for b in range(0, 4096, 251)))
    test.check("delta is antisymmetric off the half turn",
               all(circular_delta(a, b) == -circular_delta(b, a)
                   for a in range(0, 4096, 337) for b in range(0, 4096, 251)
                   if abs(circular_delta(a, b)) != 2048))
    test.check("floor_mod never negative",
               all(floor_mod(v, 4096) >= 0 for v in range(-9000, 9000, 97)))

    # ---------------------------------------------------------------- 3
    print("[3] Position domain 0..4095")
    test.check("0 accepted", validate_position_domain(0) == 0)
    test.check("4095 accepted", validate_position_domain(4095) == 4095)
    test.expect_raises("-1 rejected", CharacterizationError,
                       validate_position_domain, -1)
    test.expect_raises("4096 rejected", CharacterizationError,
                       validate_position_domain, 4096)
    test.expect_raises("non-int rejected", CharacterizationError,
                       validate_position_domain, 100.5)

    # ---------------------------------------------------------------- 4
    print("[4] Discovery gate — exactly one responder / 777 / ResponseStatus")
    ok = evaluate_discovery_gate(1, 0, 0, 777, golden)
    test.check("golden NEW01 passes", ok["pass"], ok["failures"])
    test.check("two responders fail",
               not evaluate_discovery_gate(2, 0, 0, 777, golden)["pass"])
    test.check("zero responders fail",
               not evaluate_discovery_gate(0, 0, 0, 777, golden)["pass"])
    test.check("dirty ping status fails",
               not evaluate_discovery_gate(1, 0x20, 0, 777, golden)["pass"])
    test.check("dirty model status fails",
               not evaluate_discovery_gate(1, 0, 0x20, 777, golden)["pass"])
    test.check("scan model 999 fails",
               not evaluate_discovery_gate(1, 0, 0, 999, golden)["pass"])
    test.check(
        "model word 999 at 0x03 fails",
        not evaluate_discovery_gate(
            1, 0, 0, 777, _frame_with(golden, w_03=999))["pass"]
    )
    test.check(
        "ResponseStatus 0 fails",
        not evaluate_discovery_gate(
            1, 0, 0, 777, _frame_with(golden, b_08=0))["pass"]
    )
    test.check(
        "ResponseStatus 2 fails",
        not evaluate_discovery_gate(
            1, 0, 0, 777, _frame_with(golden, b_08=2))["pass"]
    )
    test.check(
        "baud 1 fails",
        not evaluate_discovery_gate(
            1, 0, 0, 777, _frame_with(golden, b_06=1))["pass"]
    )
    test.check(
        "torque already on fails",
        not evaluate_discovery_gate(
            1, 0, 0, 777, _frame_with(golden, b_28=1))["pass"]
    )
    test.check(
        "status byte set fails",
        not evaluate_discovery_gate(
            1, 0, 0, 777, _frame_with(golden, b_40=0x20))["pass"]
    )
    test.check(
        "corrupt PositionOffset 0x8000 fails",
        not evaluate_discovery_gate(
            1, 0, 0, 777, _frame_with(golden, w_1f=0x8000))["pass"]
    )
    test.check(
        "nonzero PositionOffset is NOT a failure (unit is unprovisioned)",
        evaluate_discovery_gate(
            1, 0, 0, 777, _frame_with(golden, w_1f=1234))["pass"]
    )
    test.check(
        "EEPROM Lock 1 is NOT a failure",
        evaluate_discovery_gate(
            1, 0, 0, 777, _frame_with(golden, b_37=1))["pass"]
    )
    test.check(
        "undervoltage fails",
        not evaluate_discovery_gate(
            1, 0, 0, 777, _frame_with(golden, b_3e=30))["pass"]
    )
    test.check(
        "overtemperature fails",
        not evaluate_discovery_gate(
            1, 0, 0, 777, _frame_with(golden, b_3f=80))["pass"]
    )

    # ---------------------------------------------------------------- 5
    print("[5] Write allowlist — only TorqueEnable=0 and TorqueLimit=300")
    allowed = set()
    for addr in range(0x00, 0x47):
        for width in (1, 2):
            for value in (0, 1, 2, 16, 32, 128, 255, 300, 301, 1000, 2048,
                          4095, 65535):
                for stage in STAGES:
                    if char_write_allowed(addr, width, value, stage):
                        allowed.add((addr, width, value, stage))
    test.check(
        "exactly three (addr,width,value,stage) combinations are writable",
        allowed == {
            (ADDR_TORQUE_ENABLE, 1, 0, STAGE_PRIME),
            (ADDR_TORQUE_ENABLE, 1, 0, STAGE_RECOVERY),
            (ADDR_TORQUE_LIMIT, 2, CHAR_TORQUE_LIMIT, STAGE_PRIME),
        },
        sorted(allowed),
    )
    test.check("only Torque OFF: value 1 refused in every stage",
               not any(char_write_allowed(ADDR_TORQUE_ENABLE, 1, 1, s)
                       for s in STAGES))
    test.check("mid-position calibration value 128 at 0x28 refused",
               not any(char_write_allowed(ADDR_TORQUE_ENABLE, 1, 128, s)
                       for s in STAGES))
    test.check("only TorqueLimit=300: 1000 refused",
               not any(char_write_allowed(ADDR_TORQUE_LIMIT, 2, 1000, s)
                       for s in STAGES))
    test.check("TorqueLimit refused outside the prime stage",
               not char_write_allowed(ADDR_TORQUE_LIMIT, 2, 300, STAGE_NONE)
               and not char_write_allowed(ADDR_TORQUE_LIMIT, 2, 300,
                                          STAGE_RECOVERY))
    test.check("no ID write", not any(
        char_write_allowed(ADDR_ID, w, v, s)
        for w in (1, 2) for v in range(0, 256) for s in STAGES))
    test.check("no PositionOffset write", not any(
        char_write_allowed(ADDR_POSITION_OFFSET, w, v, s)
        for w in (1, 2) for v in (0, 85, 1963, 65535) for s in STAGES))
    test.check("no EEPROM Lock write", not any(
        char_write_allowed(ADDR_LOCK, w, v, s)
        for w in (1, 2) for v in (0, 1) for s in STAGES))
    test.check("no baud write", not any(
        char_write_allowed(ADDR_BAUD, w, v, s)
        for w in (1, 2) for v in range(0, 8) for s in STAGES))

    # ---------------------------------------------------------------- 6
    print("[6] Static audit of the firmware source")
    for name, value in sorted(audit["checks"].items()):
        test.check("firmware %s" % name, value)
    test.check("no unapproved write sites", not audit["unapproved_write_sites"],
               audit["unapproved_write_sites"])
    test.check("no forbidden source patterns",
               not audit["forbidden_pattern_hits"],
               audit["forbidden_pattern_hits"])
    for primitive in FORBIDDEN_PRIMITIVES:
        test.check(
            "no call site for %s" % primitive,
            primitive in audit["primitives_with_zero_call_sites"],
        )
    test.check("exactly three write call sites",
               len(audit["write_sites"]) == 3, audit["write_sites"])

    # ---------------------------------------------------------------- 7
    print("[7] Motion primitive: WritePosEx always uses CURRENT position")
    motion = audit["motion_primitive"]
    test.check("exactly one WritePosEx call site in the firmware",
               motion["checks"]["exactly_one_call_site"])
    test.check("its position argument is the freshly read PresentPosition",
               motion["checks"]["call_uses_current_position"],
               motion["call_sites"])
    test.check("the primitive has no position parameter to pass",
               motion["checks"]["signature_has_no_position_parameter"],
               motion["signature"])
    test.check("speed exactly 365",
               audit["checks"]["speed_constant_is_365"])
    test.check("acc exactly 50", audit["checks"]["acc_constant_is_50"])
    test.check("torque limit exactly 300",
               audit["checks"]["torque_limit_constant_is_300"])

    # ---------------------------------------------------------------- 8
    print("[8] Command surface")
    for command in ("@ARM", "@HELP", "@PRIME 1A2B3C4D", "@TRACE DEADBEEF"):
        test.check("accepts %s" % command,
                   assert_command_allowed(command) == command)
    for command in FORBIDDEN_COMMAND_TOKENS:
        test.expect_raises("rejects %s" % command, SafetyViolation,
                           assert_command_allowed, command)
    for command in ("@PRIME", "@PRIME 1a2b3c4d", "@PRIME 1A2B3C4",
                    "@PRIME 1A2B3C4D5", "@ARM NEW01", "@ARM ",
                    "@PRIME 1A2B3C4D\n@RESET", " @ARM", "@HELP ",
                    "@TRACE ZZZZZZZZ", "@EXECUTE 1A2B3C4D"):
        test.expect_raises("rejects %r" % command, SafetyViolation,
                           assert_command_allowed, command)

    # ---------------------------------------------------------------- 9
    print("[9] 71-byte evidence persistence")
    with tempfile.TemporaryDirectory() as tmp:
        path = os.path.join(tmp, "before.bin")
        artifact = persist_raw71(golden, path)
        test.check("exactly 71 bytes on disk",
                   os.path.getsize(path) == SNAPSHOT_LEN)
        test.check("fsync recorded", artifact["flushed_and_fsynced"])
        test.check("exclusive create recorded", artifact["exclusive_create"])
        test.check("sha256 matches the file",
                   artifact["sha256"] == sha256_file(path))
        test.expect_raises("refuses to overwrite evidence",
                           CharacterizationError, persist_raw71, golden, path)
        test.expect_raises("refuses 70 bytes", CharacterizationError,
                           persist_raw71, golden[:70],
                           os.path.join(tmp, "short.bin"))
        test.expect_raises("refuses 72 bytes", CharacterizationError,
                           persist_raw71, golden + b"\x00",
                           os.path.join(tmp, "long.bin"))
        test.expect_raises("refuses str payload", CharacterizationError,
                           persist_raw71, "not bytes",
                           os.path.join(tmp, "str.bin"))
        first = create_session_dir(tmp, "NEW01", "STAMP")
        test.check("session dir created exclusively", os.path.isdir(first))
        test.expect_raises("session dir refuses reuse", CharacterizationError,
                           create_session_dir, tmp, "NEW01", "STAMP")
    test.expect_raises("short RAW71_HEX rejected", CharacterizationError,
                       parse_raw71_hex, GOLDEN_NEW01_HEX[:-2])
    test.expect_raises("non-hex RAW71_HEX rejected", CharacterizationError,
                       parse_raw71_hex, "ZZ" + GOLDEN_NEW01_HEX[2:])

    # --------------------------------------------------------------- 10
    print("[10] Evidence-before-write ordering")
    with tempfile.TemporaryDirectory() as tmp:
        gate = EvidenceGate()
        test.expect_raises(
            "@PRIME refused before any evidence exists", SafetyViolation,
            gate.check, "@PRIME 1A2B3C4D",
        )
        session = create_session_dir(tmp, "NEW01")
        capture_snapshot("BEFORE", GOLDEN_NEW01_HEX, session,
                         on_persisted=gate.note_persisted)
        test.check("@PRIME allowed once BEFORE is durable",
                   gate.check("@PRIME 1A2B3C4D") == "@PRIME 1A2B3C4D")
        test.check("BEFORE really is on disk with 71 bytes",
                   gate.before_is_durable())

        tampered = EvidenceGate()
        tampered.persisted["BEFORE"] = {
            "path": os.path.join(session, ARTIFACTS["BEFORE"]),
            "bytes": SNAPSHOT_LEN, "exclusive_create": True,
            "flushed_and_fsynced": True, "sha256": "0" * 64,
        }
        test.expect_raises(
            "@PRIME refused when the on-disk digest disagrees",
            SafetyViolation, tampered.check, "@PRIME 1A2B3C4D",
        )

    with tempfile.TemporaryDirectory() as tmp:
        gate = EvidenceGate()
        session = create_session_dir(tmp, "NEW01")
        record = {"artifacts": {}, "phases": {}}
        transport = ScriptedTransport(
            {"@ARM": [_arm_transcript()],
             "@PRIME 1A2B3C4D": [_prime_transcript()]}, gate,
        )
        arm = run_arm_phase(transport, record, session, gate)
        test.check("ARM phase persisted BEFORE first",
                   gate.ordering_report()["before_persisted"])
        run_prime_phase(transport, record, session, gate, arm["token"])
        report = gate.ordering_report()
        test.check("evidence strictly precedes the first write command",
                   report["evidence_precedes_write"])
        test.check("BEFORE persist is ledger entry before @PRIME",
                   report["before_persist_index"]
                   < report["first_write_capable_command_index"])
        test.check("commands sent were exactly @ARM then @PRIME",
                   transport.sent == ["@ARM", "@PRIME 1A2B3C4D"],
                   transport.sent)

    with tempfile.TemporaryDirectory() as tmp:
        gate = EvidenceGate()
        session = create_session_dir(tmp, "NEW01")
        record = {"artifacts": {}, "phases": {}}
        transport = ScriptedTransport(
            {"@ARM": [_arm_transcript(responders=2)]}, gate,
        )
        test.expect_raises(
            "two responders abort ARM before any @PRIME",
            CharacterizationError, run_arm_phase, transport, record, session,
            gate,
        )
        test.check("no write-capable command was ever sent",
                   not any(c.startswith("@PRIME") for c in transport.sent))

    # --------------------------------------------------------------- 11
    print("[11] Prime transcript verification")
    good = verify_prime(parse_prime(_prime_transcript()))
    test.check("clean 5-rep transcript verifies", good["ok"], good["findings"])
    test.check("5 repetitions recorded", good["summary"]["repetitions"] == 5)
    test.check("observed deltas reported",
               good["summary"]["observed_delta_ticks"] == [0, 0, 0, 0, 0])
    test.check("threshold explicitly deferred",
               good["summary"]["PRIME_MAX_DELTA_TICKS"] == "DEFERRED_TO_REVIEW")

    bad_speed = verify_prime(parse_prime(
        _prime_transcript({3: {"speed": 300}})))
    test.check("speed 300 is rejected", not bad_speed["ok"])
    bad_acc = verify_prime(parse_prime(_prime_transcript({2: {"acc": 20}})))
    test.check("acc 20 is rejected", not bad_acc["ok"])
    bad_goal = verify_prime(parse_prime(
        _prime_transcript({1: {"goal": 1963}})))
    test.check("a goal that is not the current position is rejected",
               not bad_goal["ok"], bad_goal["findings"])
    bad_status = verify_prime(parse_prime(
        _prime_transcript({4: {"status": 0x20}})))
    test.check("a nonzero status byte is rejected", not bad_status["ok"])

    wrapped = verify_prime(parse_prime(
        _prime_transcript({1: {"p0": 4095, "p_settled": 1}})))
    test.check("wrap-crossing delta is recomputed as +2, not -4094",
               wrapped["summary"]["observed_delta_ticks"][0] == 2,
               wrapped["summary"]["observed_delta_ticks"])
    test.check("wrap-crossing transcript still verifies", wrapped["ok"],
               wrapped["findings"])

    tampered_lines = [
        l.replace("DELTA_TICKS=0", "DELTA_TICKS=7")
        if l.startswith("PRIME_DELTA INDEX=2") else l
        for l in _prime_transcript()
    ]
    test.check("a firmware delta that disagrees with the host is rejected",
               not verify_prime(parse_prime(tampered_lines))["ok"])

    short = verify_prime(parse_prime(
        [l for l in _prime_transcript() if "INDEX=5" not in l]))
    test.check("4 repetitions instead of 5 is rejected", not short["ok"])

    # --------------------------------------------------------------- 12
    print("[12] Write records stay inside the allowlist")
    records = parse_write_records(_prime_transcript())
    test.check("all recorded writes are in the allowlist",
               not verify_write_records(records), records[:2])
    test.check("only 0x28 and 0x30 appear",
               {r["address"] for r in records} == {0x28, 0x30})
    test.check("torque writes are always value 0",
               all(r["expect"] == 0 for r in records if r["address"] == 0x28))
    test.check("torque limit writes are always 300",
               all(r["expect"] == 300 for r in records if r["address"] == 0x30))
    for rogue in (
        "WRITE ADDR=0x05 NAME=ID WIDTH=1 EXPECT=23 ACK=1 STATUS=0x00 "
        "READBACK=23 READ_STATUS=0x00 RESULT=OK",
        "WRITE ADDR=0x1F NAME=PositionOffset WIDTH=2 EXPECT=0 ACK=1 "
        "STATUS=0x00 READBACK=0 READ_STATUS=0x00 RESULT=OK",
        "WRITE ADDR=0x37 NAME=Lock WIDTH=1 EXPECT=0 ACK=1 STATUS=0x00 "
        "READBACK=0 READ_STATUS=0x00 RESULT=OK",
        "WRITE ADDR=0x28 NAME=TorqueEnable WIDTH=1 EXPECT=1 ACK=1 "
        "STATUS=0x00 READBACK=1 READ_STATUS=0x00 RESULT=OK",
        "WRITE ADDR=0x28 NAME=TorqueEnable WIDTH=1 EXPECT=128 ACK=1 "
        "STATUS=0x00 READBACK=128 READ_STATUS=0x00 RESULT=OK",
        "WRITE ADDR=0x30 NAME=TorqueLimit WIDTH=2 EXPECT=1000 ACK=1 "
        "STATUS=0x00 READBACK=1000 READ_STATUS=0x00 RESULT=OK",
    ):
        name = rogue.split("NAME=")[1].split(" ")[0]
        expect = rogue.split("EXPECT=")[1].split(" ")[0]
        test.check(
            "rejects a %s=%s write record" % (name, expect),
            bool(verify_write_records(parse_write_records([rogue]))),
        )

    with tempfile.TemporaryDirectory() as tmp:
        gate = EvidenceGate()
        session = create_session_dir(tmp, "NEW01")
        record = {"artifacts": {}, "phases": {}}
        rogue_prime = _prime_transcript()
        rogue_prime.insert(
            1,
            "WRITE ADDR=0x1F NAME=PositionOffset WIDTH=2 EXPECT=0 ACK=1 "
            "STATUS=0x00 READBACK=0 READ_STATUS=0x00 RESULT=OK",
        )
        transport = ScriptedTransport(
            {"@ARM": [_arm_transcript()],
             "@PRIME 1A2B3C4D": [rogue_prime]}, gate,
        )
        arm = run_arm_phase(transport, record, session, gate)
        test.expect_raises(
            "a PositionOffset write in the transcript aborts the session",
            SafetyViolation, run_prime_phase, transport, record, session, gate,
            arm["token"],
        )

    # --------------------------------------------------------------- 13
    print("[13] Failure path converges to Torque OFF")
    fail_lines = _fail_transcript()
    test.check("fault is declared",
               any(l.startswith("FAULT REASON=") for l in fail_lines))
    test.check("recovery commands torque off",
               "RECOVERY TORQUE_OFF=OK" in fail_lines)
    test.check("power removal is requested",
               "REMOVE_SERVO_POWER_NOW" in fail_lines)
    test.check("no EEPROM writes on the failure path",
               "EEPROM_WRITES=NONE" in fail_lines)
    last_write = parse_write_records(fail_lines)[-1]
    test.check("the last write on the failure path is TorqueEnable=0",
               last_write["address"] == ADDR_TORQUE_ENABLE
               and last_write["expect"] == 0, last_write)
    test.check("no motion command follows the fault",
               not any(l.startswith("PRIME_CMD")
                       for l in fail_lines[fail_lines.index(
                           "FAULT REASON=GROSS_MOTION_ABORT"):]))
    with tempfile.TemporaryDirectory() as tmp:
        gate = EvidenceGate()
        session = create_session_dir(tmp, "NEW01")
        record = {"artifacts": {}, "phases": {}}
        transport = ScriptedTransport(
            {"@ARM": [_arm_transcript()],
             "@PRIME 1A2B3C4D": [fail_lines]}, gate,
        )
        arm = run_arm_phase(transport, record, session, gate)
        test.expect_raises(
            "the host fails closed on a FAIL prime", CharacterizationError,
            run_prime_phase, transport, record, session, gate, arm["token"],
        )

    # --------------------------------------------------------------- 14
    print("[14] Power-cycle trace timestamp calculations")
    lines = _trace_transcript()
    trace = verify_trace(lines, 1, 1)
    test.check("clean single-absence trace verifies", trace["ok"],
               trace["findings"])
    test.check("one absence episode", len(trace["episodes"]) == 1)
    episode = trace["episodes"][0]
    test.check("first absent poll at t=1120",
               episode["first_absent_t_ms"] == 1120, episode)
    test.check("14 absent polls recorded", episode["absent_polls"] == 14)
    test.check("absence duration is 14 polls x 120 ms = 1680 ms",
               episode["duration_ms"] == 1680, episode["duration_ms"])
    test.check("gap from last present response is 1800 ms",
               episode["gap_ms"] == 1120 + 14 * 120 - 1000,
               episode["gap_ms"])
    test.check("return to 20 clean responses took 19 x 21 = 399 ms",
               episode["return_to_stable_ms"] == 399,
               episode["return_to_stable_ms"])
    test.check("20 polls consumed reaching the stable endpoint",
               episode["return_to_stable_polls"] == 20,
               episode["return_to_stable_polls"])
    test.check("stability endpoint reached", episode["stable"])
    test.check("host and firmware episode tables agree",
               not compare_trace_episodes(trace["episodes"],
                                          trace["firmware_episodes"]))
    test.check("longest absence reported", trace["longest_absence_ms"] == 1680)
    test.check("both cold constants deferred",
               trace["COLD_ABSENCE_DEBOUNCE_MS"] == "DEFERRED_TO_REVIEW"
               and trace["COLD_RETURN_STABLE_SAMPLES"] == "DEFERRED_TO_REVIEW")
    test.check("the trace never classifies an absence",
               "classified" not in json.dumps(trace).lower())

    flap = verify_trace(_trace_transcript(unclean_returns=3), 1, 1)
    test.check("a flapping return still verifies", flap["ok"], flap["findings"])
    test.check("3 unclean responses after return are counted",
               flap["episodes"][0]["unclean_after_return"] == 3,
               flap["episodes"][0])
    test.check("the stable window starts only after the flapping stops",
               flap["episodes"][0]["return_to_stable_ms"] == (3 + 19) * 21,
               flap["episodes"][0]["return_to_stable_ms"])

    glitch = verify_trace(_trace_transcript(extra_glitch=True), 1, 1)
    test.check("a single-poll dropout is recorded as its own episode",
               len(glitch["episodes"]) == 2, glitch["episodes"])
    test.check("the glitch and the real absence are both timed",
               glitch["absence_durations_ms"][0] == 120
               and glitch["absence_durations_ms"][1] == 1680,
               glitch["absence_durations_ms"])
    test.check("the tool reports both and picks neither",
               glitch["COLD_ABSENCE_DEBOUNCE_MS"] == "DEFERRED_TO_REVIEW")

    tampered = [
        l.replace("DURATION_MS=1680", "DURATION_MS=1200")
        if l.startswith("TRACE_EPISODE") else l for l in _trace_transcript()
    ]
    test.check("a firmware duration that disagrees with the polls is rejected",
               not verify_trace(tampered, 1, 1)["ok"])

    no_polls = [l for l in _trace_transcript() if not l.startswith("TRACE T")]
    test.check("a trace with no per-poll records is rejected",
               not verify_trace(no_polls, 1, 1)["ok"])

    two = _trace_transcript(trace=2)
    test.check("a second trace parses independently",
               verify_trace(two, 2, 1)["ok"])
    test.check("trace 1 records are not mixed into trace 2",
               parse_trace_polls(two, 1) == [])

    # --------------------------------------------------------------- 15
    print("[15] Trace phase is read-only")
    with tempfile.TemporaryDirectory() as tmp:
        gate = EvidenceGate()
        session = create_session_dir(tmp, "NEW01")
        record = {"artifacts": {}, "phases": {}}
        transport = ScriptedTransport(
            {"@ARM": [_arm_transcript()],
             "@PRIME 1A2B3C4D": [_prime_transcript()],
             "@TRACE 1A2B3C4D": [_trace_transcript()]}, gate,
        )
        arm = run_arm_phase(transport, record, session, gate)
        run_prime_phase(transport, record, session, gate, arm["token"])
        run_trace_phase(transport, record, session, gate, arm["token"], 1, 1,
                        announce=False)
        test.check("cold snapshot persisted", "COLD1" in record["artifacts"])
        test.check("trace recorded in the report",
                   len(record["phases"]["traces"]) == 1)

        rogue_trace = _trace_transcript()
        rogue_trace.insert(
            1,
            "WRITE ADDR=0x28 NAME=TorqueEnable WIDTH=1 EXPECT=0 ACK=1 "
            "STATUS=0x00 READBACK=0 READ_STATUS=0x00 RESULT=OK",
        )
        transport.script["@TRACE 1A2B3C4D"] = [rogue_trace]
        test.expect_raises(
            "any write during a trace aborts the session", SafetyViolation,
            run_trace_phase, transport, record, session, gate, arm["token"],
            2, 1, False,
        )

    # --------------------------------------------------------------- 16
    print("[16] Snapshot framing")
    test.check("BEFORE extracted",
               extract_snapshots(_arm_transcript())["BEFORE"]
               == GOLDEN_NEW01_HEX)
    test.expect_raises(
        "unterminated snapshot rejected", CharacterizationError,
        extract_snapshots,
        ["SNAPSHOT_BEGIN SLOT=BEFORE ID=1 START=0x00 LEN=71",
         "RAW71_HEX=" + GOLDEN_NEW01_HEX],
    )
    test.expect_raises(
        "mismatched slot rejected", CharacterizationError, extract_snapshots,
        ["SNAPSHOT_BEGIN SLOT=BEFORE ID=1 START=0x00 LEN=71",
         "RAW71_HEX=" + GOLDEN_NEW01_HEX, "SNAPSHOT_END SLOT=COLD1"],
    )
    test.expect_raises(
        "snapshot with no payload rejected", CharacterizationError,
        extract_snapshots,
        ["SNAPSHOT_BEGIN SLOT=BEFORE ID=1 START=0x00 LEN=71",
         "SNAPSHOT_END SLOT=BEFORE"],
    )

    # --------------------------------------------------------------- 17
    print("[17] Physical label and scope")
    test.check("NEW01 is a known MATDOG label",
               validate_physical_label("NEW01") == "NEW01")
    test.expect_raises("unknown label rejected", CharacterizationError,
                       validate_physical_label, "NOPE9")
    test.expect_raises("lowercase label rejected", CharacterizationError,
                       validate_physical_label, "new01")
    surface = command_surface_audit()
    test.check("this tool selects no constants",
               surface["constants_selected_by_this_tool"] == [])
    test.check("all three constants are deferred",
               set(surface["constants_deferred_to_review"])
               == set(DEFERRED_CONSTANTS))
    test.check("the runner never names an EEPROM write capability",
               surface["eeprom_write"] == "NOT IMPLEMENTED"
               and surface["eeprom_unlock"] == "NOT IMPLEMENTED")

    print()
    print("=" * 70)
    print(" PASSED: %d    FAILED: %d" % (test.passed, len(test.failed)))
    if test.failed:
        for name in test.failed:
            print("   FAILED: %s" % name)
    print(" SERIAL_OPENED=NO  UPLOAD_PERFORMED=NO  HARDWARE_TOUCHED=NO")
    print("=" * 70)
    return 0 if not test.failed else 1


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(
        prog=TOOL_NAME,
        description="MATDOG ST-3215-C018 characterization harness "
                    "(prime delta + power-cycle timing). Not a provisioner.",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--characterize", action="store_true",
                      help="HARDWARE MODE. Requires --unit.")
    mode.add_argument("--self-test", action="store_true",
                      help="run the offline self-test; opens no serial port")
    mode.add_argument("--command-surface", action="store_true",
                      help="print the allowed command surface; opens no port")
    mode.add_argument("--static-audit", action="store_true",
                      help="audit every firmware write primitive; no port")

    parser.add_argument("--unit", metavar="LABEL",
                        help="physical label, e.g. NEW01")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--session-root", default=DEFAULT_SESSION_ROOT)
    parser.add_argument("--traces", type=int, default=1, choices=(1, 2),
                        help="power-cycle traces to record (default 1)")

    args = parser.parse_args(argv)

    if args.self_test:
        return run_self_test()

    if args.command_surface:
        print(json.dumps(command_surface_audit(), indent=2, sort_keys=True))
        return 0

    if args.static_audit:
        audit = static_audit()
        print(json.dumps(audit, indent=2, sort_keys=True))
        return 0 if audit["ok"] else 1

    if args.characterize:
        if not args.unit:
            print("ERROR: --characterize requires --unit", file=sys.stderr)
            return 2
        try:
            record = run_characterization(
                args.unit, args.port, args.session_root, args.traces
            )
        except (CharacterizationError, SafetyViolation) as exc:
            print("CHARACTERIZATION FAILED: %s" % exc, file=sys.stderr)
            return 1
        return 0 if record["verdict"] == "PASS" else 1

    # Fail-closed default: no mode selected means no action, no port.
    parser.print_usage(sys.stderr)
    print("ERROR: no mode selected. Nothing was done and no port was opened.",
          file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
