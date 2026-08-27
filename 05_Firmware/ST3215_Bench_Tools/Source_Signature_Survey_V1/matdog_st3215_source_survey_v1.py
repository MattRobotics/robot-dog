#!/usr/bin/env python3
"""
MATDOG ST3215 Read-Only Source Signature Survey V1 — host runner.

Companion to the FROZEN firmware:
    matdog_st3215_source_survey_v1/matdog_st3215_source_survey_v1.ino
    SHA256 f37fa85a11e333634c18b4abbd991e24797b1891ec5d2e5432b1e4787e5a0c86

    build_freeze_20260826/matdog_st3215_source_survey_v1.ino.bin
    SHA256 89ce2a5178b663d7a19b01275ad65662856ce7b7bc114198b5a3e535464f6a12

PURPOSE
    Acquire, per physical servo unit, an auditable as-found record:
    a full 71-byte state snapshot BEFORE the single permitted safety write
    (TorqueEnable=0), and two snapshots AFTER it.

    This tool ACQUIRES FACTS. It does not classify source profiles.
    profile_classification is always PENDING_OFFLINE_REVIEW.

SAFETY MODEL — fail-closed
    * Exactly three commands may ever reach the firmware:
          @SCAN 0 253
          @SNAPSHOT71 <id>
          @SAFE_OFF <id>
      Enforced at a single choke point (send_command / assert_command_allowed).
      Everything else raises SafetyViolation before any byte is written.
    * No position command, no torque ON, no EEPROM unlock/write, no ID change,
      no CalibrationOfs, no broadcast write exists anywhere in this file.
    * Hardware mode requires an exact confirmation string and is never entered
      by default. Running with no arguments opens no port and exits non-zero.
    * Every 71-byte frame is written to its own .bin artifact (exclusive
      create, flush, os.fsync) BEFORE it is decoded, and the PRE artifact
      is durable on disk BEFORE @SAFE_OFF is transmitted. The raw bytes
      are the evidence; the decoded JSON is only a view of them.
    * Any exception - including pyserial errors, OSError and filesystem
      failures - stamps verdict=FAIL on the record before propagating.
      The serial port is always closed and no emergency command is ever
      sent from the failure path.

TRANSPORT
    Reuses the verified pattern of matdog_qc_runner.py
    (SHA256 a143958034be46c9a36ce89fca2fb27fab2371e1af730abe2e3eaf265bc1245e):
    pyserial, 115200, DTR=False, RTS=False, open, sleep 2.2 s,
    reset_input_buffer, ASCII command + newline, line-based reads.

    matdog_qc_unit.sh is deliberately NOT reused: its EXIT SAFE_OFF is
    incompatible with the SNAPSHOT71 gate of this firmware.
"""

import argparse
import datetime
import hashlib
import json
import os
import re
import sys
import tempfile

TOOL_NAME = "matdog_st3215_source_survey_v1"
TOOL_VERSION = "1.0.0"
REPORT_SCHEMA = "MATDOG_ST3215_SOURCE_SURVEY_V1"

# The survey acquires facts only. Source-profile classification is an
# OFFLINE activity performed later against the whole corpus. These two
# values are the ONLY verdict vocabulary this tool may emit.
SURVEY_RESULT_CAPTURED = "SOURCE_SIGNATURE_CAPTURED"
PROFILE_CLASSIFICATION_PENDING = "PENDING_OFFLINE_REVIEW"

# --------------------------------------------------------------------------
# Frozen firmware provenance
# --------------------------------------------------------------------------

ESP32_DIR = os.path.dirname(os.path.abspath(__file__))
FIRMWARE_DIR = os.path.join(ESP32_DIR, "matdog_st3215_source_survey_v1")

FIRMWARE_SOURCE_PATH = os.path.join(
    FIRMWARE_DIR, "matdog_st3215_source_survey_v1.ino"
)
FIRMWARE_BINARY_PATH = os.path.join(
    FIRMWARE_DIR,
    "build_freeze_20260826",
    "matdog_st3215_source_survey_v1.ino.bin",
)

FIRMWARE_SOURCE_SHA256 = (
    "f37fa85a11e333634c18b4abbd991e24797b1891ec5d2e5432b1e4787e5a0c86"
)
FIRMWARE_BINARY_SHA256 = (
    "89ce2a5178b663d7a19b01275ad65662856ce7b7bc114198b5a3e535464f6a12"
)

QC_RUNNER_REFERENCE_SHA256 = (
    "a143958034be46c9a36ce89fca2fb27fab2371e1af730abe2e3eaf265bc1245e"
)

DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Espressif_USB_JTAG_serial_debug_unit_"
    "14:C1:9F:22:75:94-if00"
)

DEFAULT_REPORT_DIR = os.path.join(ESP32_DIR, "source_survey_v1")

# --------------------------------------------------------------------------
# Physical identity -> future target ID
#
# Physical identity is permanent and independent of BOTH the current source ID
# and the future target ID. A unit is NEVER classified as unknown merely
# because its present source ID differs from its future target ID: these
# servos have not been recoded yet.
# --------------------------------------------------------------------------

PHYSICAL_UNITS = {
    "M22": 13, "ELR01": 12, "M33": 11,
    "NEW01": 23, "ELR03": 22, "NEW03": 21,
    "NEW06": 33, "ELR02": 32, "NEW05": 31,
    "M43": 43, "M42": 42, "M41": 41,
    "M31": 51, "M11": 52, "NEW04": 53, "NEW02": 54, "ELR04": 55,
}

CONFIRMATION_PREFIX = "SOURCE_SURVEY_"

# --------------------------------------------------------------------------
# 71-byte state map: 0x00 .. 0x46 inclusive
# --------------------------------------------------------------------------

SNAPSHOT_START = 0x00
SNAPSHOT_LEN = 71
SNAPSHOT_LAST = 0x46

ADDR_MODEL = 0x03
ADDR_ID = 0x05
ADDR_RESPONSE_STATUS = 0x08
ADDR_POSITION_OFFSET = 0x1F
ADDR_TORQUE_ENABLE = 0x28
ADDR_PRESENT_POSITION = 0x38

EXPECTED_MODEL = 777
EXPECTED_RESPONSE_STATUS = 1

# Comparison windows.
EEPROM_RANGE = (0x00, 0x27)      # invariant across the whole session
RAM_INVARIANT_RANGE = (0x29, 0x37)  # invariant; excludes TorqueEnable 0x28
TELEMETRY_RANGE = (0x38, 0x46)   # permitted to vary freely

POSITION_OFFSET_MIN = -2048
POSITION_OFFSET_MAX = 2047
ENCODER_COUNTS = 4096

SCAN_TIMEOUT_S = 120.0   # @SCAN 0 253 costs ~100 ms per absent ID (~25 s)
SNAPSHOT_TIMEOUT_S = 20.0
SAFE_OFF_TIMEOUT_S = 20.0


class SafetyViolation(RuntimeError):
    """A forbidden operation was attempted. Always fatal."""


class SurveyError(RuntimeError):
    """A survey precondition or verification failed. Always fail-closed."""


# --------------------------------------------------------------------------
# Command surface — the single choke point
# --------------------------------------------------------------------------

ALLOWED_COMMAND_PATTERNS = (
    re.compile(r"^@SCAN 0 253$"),
    re.compile(r"^@SNAPSHOT71 (?:[0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-3])$"),
    re.compile(r"^@SAFE_OFF (?:[0-9]|[1-9][0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-3])$"),
)

FORBIDDEN_COMMAND_TOKENS = (
    "@NORMALIZE_MATDOG", "@QC_FAST", "@READ", "@DUMP_RAW", "@BENCH",
    "@CAPTURE", "@RAMTEST", "@PING", "@HELP",
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
        "allowed_examples": ["@SCAN 0 253", "@SNAPSHOT71 13", "@SAFE_OFF 13"],
        "forbidden_tokens_rejected": list(FORBIDDEN_COMMAND_TOKENS),
        "position_commands": "NOT IMPLEMENTED",
        "torque_on": "NOT IMPLEMENTED",
        "eeprom_unlock_or_write": "NOT IMPLEMENTED",
        "id_change": "NOT IMPLEMENTED",
        "calibration_ofs": "NOT IMPLEMENTED",
        "broadcast_writes": "NOT IMPLEMENTED",
        "only_permitted_write": "TorqueEnable=0 via @SAFE_OFF (firmware-gated)",
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


def u16le(raw, addr):
    return raw[addr] | (raw[addr + 1] << 8)


def i16le_twos_complement(raw, addr):
    """PositionOffset ONLY. Two's complement, little endian.

    Deliberately distinct from sign_magnitude(): conflating the two is the
    single most dangerous decoding error in this register map.
    """
    value = u16le(raw, addr)
    return value - 0x10000 if value & 0x8000 else value


def sign_magnitude(value, sign_bit):
    """Present speed / load / current ONLY. Feetech sign-magnitude.

    NEVER use for PositionOffset (0x1F), which is two's complement.
    """
    mask = 1 << sign_bit
    if value & mask:
        return -(value & ~mask)
    return value


def validate_physical_label(label):
    if not isinstance(label, str) or label not in PHYSICAL_UNITS:
        raise SurveyError(
            "unknown physical label %r; expected one of %s"
            % (label, ", ".join(sorted(PHYSICAL_UNITS)))
        )
    return label


def expected_confirmation(label):
    return CONFIRMATION_PREFIX + validate_physical_label(label)


def verify_confirmation(label, supplied):
    """Exact match only. Any mismatch fails closed."""
    expected = expected_confirmation(label)
    if not isinstance(supplied, str) or supplied != expected:
        raise SurveyError(
            "confirmation mismatch: expected %r, got %r" % (expected, supplied)
        )
    return True


def verify_frozen_firmware():
    results = {}
    for name, path, expected in (
        ("source", FIRMWARE_SOURCE_PATH, FIRMWARE_SOURCE_SHA256),
        ("binary", FIRMWARE_BINARY_PATH, FIRMWARE_BINARY_SHA256),
    ):
        if not os.path.isfile(path):
            raise SurveyError("frozen firmware missing: %s" % path)
        actual = sha256_file(path)
        results[name] = {
            "path": path, "expected_sha256": expected,
            "actual_sha256": actual, "match": actual == expected,
        }
        if actual != expected:
            raise SurveyError(
                "frozen firmware %s SHA256 mismatch: expected %s got %s"
                % (name, expected, actual)
            )
    return results


# --------------------------------------------------------------------------
# 71-byte frame handling
# --------------------------------------------------------------------------

def parse_raw71_hex(hex_text):
    """Strict. Rejects anything that is not exactly 71 bytes of hex."""
    if not isinstance(hex_text, str):
        raise SurveyError("RAW71_HEX payload is not a string")
    text = hex_text.strip()
    if len(text) != SNAPSHOT_LEN * 2:
        raise SurveyError(
            "RAW71_HEX length %d, expected %d hex chars (%d bytes)"
            % (len(text), SNAPSHOT_LEN * 2, SNAPSHOT_LEN)
        )
    if not re.fullmatch(r"[0-9A-Fa-f]+", text):
        raise SurveyError("RAW71_HEX contains non-hexadecimal characters")
    raw = bytes.fromhex(text)
    if len(raw) != SNAPSHOT_LEN:
        raise SurveyError("decoded frame is %d bytes, expected %d"
                          % (len(raw), SNAPSHOT_LEN))
    return raw


def decode_snapshot(raw):
    """Decode a validated 71-byte frame into EEPROM / RAM / telemetry."""
    if len(raw) != SNAPSHOT_LEN:
        raise SurveyError("decode_snapshot requires exactly %d bytes"
                          % SNAPSHOT_LEN)

    offset_signed = i16le_twos_complement(raw, ADDR_POSITION_OFFSET)
    offset_raw16 = u16le(raw, ADDR_POSITION_OFFSET)
    present_raw16 = u16le(raw, ADDR_PRESENT_POSITION)

    present_out_of_range = present_raw16 > (ENCODER_COUNTS - 1)
    present_effective = present_raw16 & 0x0FFF
    offset_out_of_range = not (
        POSITION_OFFSET_MIN <= offset_signed <= POSITION_OFFSET_MAX
    )

    physical_raw = (present_effective + offset_signed) % ENCODER_COUNTS

    identity = {
        "model_0x03": u16le(raw, ADDR_MODEL),
        "id_register_0x05": raw[ADDR_ID],
        "baud_register_0x06": raw[0x06],
        "return_delay_0x07": raw[0x07],
        "response_status_0x08": raw[ADDR_RESPONSE_STATUS],
    }

    eeprom = {
        "min_angle_0x09": u16le(raw, 0x09),
        "max_angle_0x0B": u16le(raw, 0x0B),
        "max_temperature_0x0D": raw[0x0D],
        "max_voltage_0x0E": raw[0x0E],
        "min_voltage_0x0F": raw[0x0F],
        "max_torque_0x10": u16le(raw, 0x10),
        "unload_condition_0x12": raw[0x12],
        "led_alarm_0x13": raw[0x13],
        "p_0x15": raw[0x15],
        "d_0x16": raw[0x16],
        "i_0x17": raw[0x17],
        "min_startup_force_0x18": u16le(raw, 0x18),
        "cw_dead_0x1A": raw[0x1A],
        "ccw_dead_0x1B": raw[0x1B],
        "protection_current_0x1C": u16le(raw, 0x1C),
        "angular_resolution_0x1E": raw[0x1E],
        "position_offset_0x1F_raw16": offset_raw16,
        "position_offset_0x1F_signed": offset_signed,
        "mode_0x21": raw[0x21],
        "protection_torque_0x22": raw[0x22],
        "protection_time_0x23": raw[0x23],
        "overload_torque_0x24": raw[0x24],
        "speed_closed_loop_p_0x25": raw[0x25],
        "over_current_protection_time_0x26": raw[0x26],
        "velocity_closed_loop_i_0x27": raw[0x27],
    }

    ram = {
        "torque_enable_0x28": raw[ADDR_TORQUE_ENABLE],
        "acc_0x29": raw[0x29],
        "goal_position_0x2A": u16le(raw, 0x2A),
        "goal_time_0x2C": u16le(raw, 0x2C),
        "goal_speed_0x2E": u16le(raw, 0x2E),
        "torque_limit_0x30": u16le(raw, 0x30),
        "lock_0x37": raw[0x37],
    }

    telemetry = {
        "present_position_0x38_raw16": present_raw16,
        "present_speed_0x3A_signed": sign_magnitude(u16le(raw, 0x3A), 15),
        "present_load_0x3C_signed": sign_magnitude(u16le(raw, 0x3C), 10),
        "present_voltage_0x3E": raw[0x3E],
        "present_temperature_0x3F": raw[0x3F],
        "status_0x40": raw[0x40],
        "moving_0x42": raw[0x42],
        "present_current_0x45_signed": sign_magnitude(u16le(raw, 0x45), 15),
    }

    return {
        "raw_hex": raw.hex().upper(),
        "raw_sha256": sha256_bytes(raw),
        "raw_length": len(raw),
        "identity": identity,
        "eeprom": eeprom,
        "ram": ram,
        "telemetry": telemetry,
        "physical_raw": physical_raw,
        "physical_raw_formula": "(present_position + signed_position_offset) % 4096",
        "anomalies": {
            "position_offset_out_of_range": offset_out_of_range,
            "present_position_out_of_range": present_out_of_range,
        },
    }


def compare_ranges(raw_a, raw_b, window):
    """Byte-level diff over an inclusive address window."""
    lo, hi = window
    diffs = []
    for addr in range(lo, hi + 1):
        if raw_a[addr] != raw_b[addr]:
            diffs.append({
                "address": "0x%02X" % addr,
                "a": raw_a[addr],
                "b": raw_b[addr],
            })
    return diffs


def compare_snapshots(pre, post1, post2):
    """Fail-closed comparison across the three frames."""
    checks = {}

    for label, a, b in (
        ("pre_vs_post1", pre, post1),
        ("pre_vs_post2", pre, post2),
        ("post1_vs_post2", post1, post2),
    ):
        checks[label] = {
            "eeprom_diffs": compare_ranges(a, b, EEPROM_RANGE),
            "ram_invariant_diffs": compare_ranges(a, b, RAM_INVARIANT_RANGE),
            "telemetry_diffs": compare_ranges(a, b, TELEMETRY_RANGE),
            "torque_enable_a": a[ADDR_TORQUE_ENABLE],
            "torque_enable_b": b[ADDR_TORQUE_ENABLE],
        }

    eeprom_stable = all(not c["eeprom_diffs"] for c in checks.values())
    ram_stable = all(not c["ram_invariant_diffs"] for c in checks.values())

    # TorqueEnable may change only PRE -> POST (the single permitted write).
    # It must be 0 in both POST frames and must not differ between them.
    torque_post_ok = (
        post1[ADDR_TORQUE_ENABLE] == 0 and post2[ADDR_TORQUE_ENABLE] == 0
    )

    checks["summary"] = {
        "eeprom_invariant_ok": eeprom_stable,
        "ram_invariant_ok": ram_stable,
        "torque_enable_post_ok": torque_post_ok,
        "torque_enable_pre": pre[ADDR_TORQUE_ENABLE],
        "telemetry_variation_permitted": True,
        "all_ok": eeprom_stable and ram_stable and torque_post_ok,
    }
    return checks


# --------------------------------------------------------------------------
# Firmware response parsing
# --------------------------------------------------------------------------

RE_SCAN_FOUND = re.compile(
    r"^FOUND ID=(\d+) MODEL=(-?\d+) "
    r"PING_STATUS=0x([0-9A-Fa-f]{2}) MODEL_STATUS=0x([0-9A-Fa-f]{2})$"
)
RE_SCAN_RESULT = re.compile(r"^SCAN_RESULT FOUND=(\d+)$")
RE_RAW71 = re.compile(r"^RAW71_HEX=([0-9A-Fa-f]+)$")
RE_KV = re.compile(r"^([A-Z0-9_]+)=(-?(?:0x)?[0-9A-Fa-f]+)$")


def parse_scan(lines):
    responders = []
    reported = None
    complete = False
    for line in lines:
        match = RE_SCAN_FOUND.match(line)
        if match:
            responders.append({
                "id": int(match.group(1)),
                "model": int(match.group(2)),
                "ping_status": int(match.group(3), 16),
                "model_status": int(match.group(4), 16),
            })
            continue
        match = RE_SCAN_RESULT.match(line)
        if match:
            reported = int(match.group(1))
            continue
        if line == "SCAN_COMPLETE":
            complete = True
    if not complete:
        raise SurveyError("SCAN did not complete")
    if reported is None:
        raise SurveyError("SCAN_RESULT missing")
    if reported != len(responders):
        raise SurveyError(
            "SCAN_RESULT FOUND=%d disagrees with %d FOUND lines"
            % (reported, len(responders))
        )
    return responders


def select_single_responder(responders):
    """Gate between @SCAN and every later command. Fail-closed.

    Requires exactly one responder, model 777, and BOTH bus statuses clean:
    ping_status == 0 and model_status == 0. A nonzero status means the ping
    or the model read was not clean, so no @SNAPSHOT71 and no @SAFE_OFF may
    follow.
    """
    if len(responders) != 1:
        raise SurveyError(
            "expected exactly one responder, found %d" % len(responders)
        )
    responder = responders[0]
    if responder.get("model") != EXPECTED_MODEL:
        raise SurveyError(
            "scan model %s, expected %d"
            % (responder.get("model"), EXPECTED_MODEL)
        )
    for key in ("ping_status", "model_status"):
        if key not in responder:
            raise SurveyError("scan responder is missing %s" % key)
        status = responder[key]
        if status != 0:
            raise SurveyError(
                "scan %s=0x%02X, expected 0x00; no SNAPSHOT71 and no "
                "SAFE_OFF will be sent" % (key, status)
            )
    return responder


def parse_snapshot(lines):
    for line in lines:
        if line.startswith("SNAPSHOT71_ABORT"):
            raise SurveyError("firmware aborted snapshot: %s" % line)
    raw_hex = None
    fields = {}
    passed = False
    for line in lines:
        match = RE_RAW71.match(line)
        if match:
            if raw_hex is not None:
                raise SurveyError("multiple RAW71_HEX lines in one response")
            raw_hex = match.group(1)
            continue
        if line == "SNAPSHOT71_RESULT PASS":
            passed = True
            continue
        match = RE_KV.match(line)
        if match:
            fields[match.group(1)] = match.group(2)
    if raw_hex is None:
        raise SurveyError("RAW71_HEX missing from snapshot response")
    if not passed:
        raise SurveyError("SNAPSHOT71_RESULT PASS missing")
    return parse_raw71_hex(raw_hex), fields


def parse_safe_off(lines):
    for line in lines:
        if line.startswith("SAFE_OFF_ABORT"):
            raise SurveyError("firmware aborted SAFE_OFF: %s" % line)
    values = {}
    for line in lines:
        for key in ("SAFE_OFF_WRITE_RESULT", "SAFE_OFF_WRITE_STATUS",
                    "SAFE_OFF_READBACK", "SAFE_OFF_READ_STATUS"):
            prefix = key + "="
            if line.startswith(prefix):
                token = line[len(prefix):]
                values[key] = int(token, 16) if token.startswith("0x") \
                    else int(token)
    if "CUT_SERVO_POWER_NOW" in lines:
        raise SurveyError("firmware emitted CUT_SERVO_POWER_NOW")
    if "SAFE_OFF_RESULT PASS" not in lines:
        raise SurveyError("SAFE_OFF_RESULT PASS missing")
    required = {
        "SAFE_OFF_WRITE_RESULT": 1,
        "SAFE_OFF_WRITE_STATUS": 0,
        "SAFE_OFF_READBACK": 0,
        "SAFE_OFF_READ_STATUS": 0,
    }
    for key, expected in required.items():
        if key not in values:
            raise SurveyError("%s missing from SAFE_OFF response" % key)
        if values[key] != expected:
            raise SurveyError(
                "%s=%s, required %s" % (key, values[key], expected)
            )
    return values


# --------------------------------------------------------------------------
# Raw evidence persistence — 71 bytes on disk BEFORE any decoding
#
# The raw frame is the evidence; the decoded JSON is only a view of it.
# Every frame is therefore written to its own .bin artifact — exclusively
# created, flushed and os.fsync'd — before decode_snapshot() is allowed to
# run. For the PRE frame this also means the as-found bytes are durable on
# disk before @SAFE_OFF, the single permitted write, is transmitted.
# --------------------------------------------------------------------------

RAW_ARTIFACT_BASENAMES = {
    "PRE": "01_pre_safeoff_state71.bin",
    "POST1": "02_post_safeoff_state71.bin",
    "POST2": "03_repeat_state71.bin",
}


def raw_artifact_path(report_dir, label, stamp, slot):
    """Path of the .bin evidence artifact for one snapshot slot."""
    if slot not in RAW_ARTIFACT_BASENAMES:
        raise SurveyError(
            "unknown raw artifact slot %r; expected one of %s"
            % (slot, ", ".join(sorted(RAW_ARTIFACT_BASENAMES)))
        )
    return os.path.join(
        report_dir,
        "%s__%s__%s" % (label, stamp, RAW_ARTIFACT_BASENAMES[slot]),
    )


def persist_raw71(raw, path):
    """Write exactly 71 raw bytes to `path`. Fail-closed and exclusive.

    open(path, "xb") never overwrites existing evidence; the bytes are
    written, flushed and os.fsync'd before this returns; the resulting
    on-disk size is verified. Returns the artifact descriptor that the
    JSON record carries.
    """
    if not isinstance(raw, (bytes, bytearray)):
        raise SurveyError(
            "raw snapshot must be bytes, got %s" % type(raw).__name__
        )
    if len(raw) != SNAPSHOT_LEN:
        raise SurveyError(
            "refusing to persist %d bytes, expected exactly %d"
            % (len(raw), SNAPSHOT_LEN)
        )
    payload = bytes(raw)
    try:
        handle = open(path, "xb")
    except FileExistsError:
        raise SurveyError(
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
        raise SurveyError(
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


def capture_snapshot(lines, artifact_path, on_persisted=None):
    """One snapshot, in this order and no other:

        1. parse_snapshot()   -> exactly 71 bytes, or fail closed
        2. persist_raw71()    -> write + flush + os.fsync to its own .bin
        3. decode_snapshot()  -> only once the bytes are durable

    `on_persisted` is called with the artifact descriptor between steps 2
    and 3, so the .bin path is registered in the JSON record before any
    decoding takes place. If the process dies between 2 and 3 the raw
    evidence still exists on disk.
    """
    raw, fields = parse_snapshot(lines)
    artifact = persist_raw71(raw, artifact_path)
    if on_persisted is not None:
        on_persisted(artifact)
    decoded = decode_snapshot(raw)
    decoded["firmware_reported"] = fields
    decoded["raw_artifact"] = artifact
    return raw, decoded


# --------------------------------------------------------------------------
# Serial transport — reused pattern from matdog_qc_runner.py
# Imported lazily so that offline modes cannot touch a serial port.
# --------------------------------------------------------------------------

def open_port(path):
    import serial  # noqa: PLC0415 - deliberate lazy import

    ser = serial.Serial()
    ser.port = path
    ser.baudrate = 115200
    ser.timeout = 3
    ser.write_timeout = 3

    # Request inactive handshake lines before opening.
    ser.dtr = False
    ser.rts = False
    ser.open()

    # Opening may reset USB-CDC once. That is acceptable BEFORE the survey.
    import time  # noqa: PLC0415
    time.sleep(2.2)
    ser.reset_input_buffer()
    return ser


def send_command(ser, command):
    """The ONLY function in this file that writes to a serial port."""
    checked = assert_command_allowed(command)
    ser.write((checked + "\n").encode("ascii"))
    ser.flush()
    return checked


def read_until(ser, terminators, timeout, echo=True):
    import time  # noqa: PLC0415

    deadline = time.monotonic() + timeout
    lines = []
    while time.monotonic() < deadline:
        raw = ser.readline()
        if not raw:
            continue
        line = raw.decode("utf-8", errors="replace").strip()
        if not line:
            continue
        if echo:
            print(line, flush=True)
        lines.append(line)
        if any(term in line for term in terminators):
            return lines
    raise SurveyError(
        "timeout after %.1fs waiting for %s" % (timeout, terminators)
    )


# --------------------------------------------------------------------------
# Hardware survey — implemented, NOT executed by this task
# --------------------------------------------------------------------------

def build_record_skeleton(label, confirmation, port, firmware):
    """Construct the audit record. Shared by run_survey and the self-test."""
    return {
        "schema": REPORT_SCHEMA,
        "tool": TOOL_NAME,
        "tool_version": TOOL_VERSION,
        "runner_sha256": sha256_file(os.path.abspath(__file__)),
        "started_utc": utc_now_iso(),
        "physical_label": label,
        "future_target_id": PHYSICAL_UNITS[label],
        "confirmation_string": confirmation,
        "port": port,
        "frozen_firmware": firmware,
        "qc_runner_transport_reference_sha256": QC_RUNNER_REFERENCE_SHA256,
        "command_surface": command_surface_audit(),
        "commands_sent": [],
        "raw_artifacts": {},
        "survey_result": None,
        "profile_classification": PROFILE_CLASSIFICATION_PENDING,
        "verdict": "INCOMPLETE",
        "reason": None,
    }


def guard_session(record, session, persist, close_port):
    """Run one survey session under three unconditional guarantees.

    1. ANY exception — SurveyError, SafetyViolation, a pyserial error,
       OSError, a filesystem failure, anything else — stamps
       verdict="FAIL" plus a reason naming the exception type on the
       record, and is then re-raised. Nothing is ever swallowed.
    2. The record is persisted in the finally block if it still can be.
       A failure to persist is reported but never masks the original
       exception.
    3. close_port() always runs, so the serial port is always closed.

    No emergency command is sent from here. The firmware SNAPSHOT71 gate
    means an implicit SAFE_OFF would be a new, unaudited write, so this
    guard performs no serial I/O of its own.
    """
    try:
        return session()
    except Exception as exc:  # noqa: BLE001 - stamped, then re-raised
        record["verdict"] = "FAIL"
        record["reason"] = "%s: %s" % (type(exc).__name__, exc)
        raise
    finally:
        record["finished_utc"] = utc_now_iso()
        try:
            persist()
        except Exception as persist_exc:  # noqa: BLE001 - must not mask
            print("WARNING: record could not be persisted: %s" % persist_exc,
                  file=sys.stderr)
        finally:
            close_port()


def run_survey(label, confirmation, port, report_dir):
    """Single serial session for one physical unit. Fail-closed throughout."""
    validate_physical_label(label)
    verify_confirmation(label, confirmation)
    firmware = verify_frozen_firmware()

    os.makedirs(report_dir, exist_ok=True)
    stamp = datetime.datetime.now(
        datetime.timezone.utc
    ).strftime("%Y%m%d_%H%M%SZ")
    report_path = os.path.join(report_dir, "%s__%s.json" % (label, stamp))

    record = build_record_skeleton(label, confirmation, port, firmware)

    def persist():
        tmp = report_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as handle:
            json.dump(record, handle, indent=2, sort_keys=True)
            handle.write("\n")
        os.replace(tmp, report_path)

    def path_for(slot):
        return raw_artifact_path(report_dir, label, stamp, slot)

    def register(slot):
        """Register the .bin path in the JSON as soon as the bytes are
        durable — still before decode_snapshot() runs for that frame."""
        def on_persisted(artifact):
            record["raw_artifacts"][slot] = artifact
            persist()
        return on_persisted

    holder = {"ser": None}

    def close_port():
        ser = holder["ser"]
        if ser is None:
            return
        try:
            ser.close()
        except Exception as exc:  # noqa: BLE001 - close must not mask
            print("WARNING: serial close failed: %s" % exc, file=sys.stderr)

    def session():
        ser = open_port(port)
        holder["ser"] = ser

        # --- SCAN -----------------------------------------------------
        record["commands_sent"].append(send_command(ser, "@SCAN 0 253"))
        scan_lines = read_until(ser, ["SCAN_COMPLETE"], SCAN_TIMEOUT_S)
        responders = parse_scan(scan_lines)
        record["scan"] = {"responders": responders}
        persist()

        # Exactly one responder, model 777, ping_status 0, model_status 0.
        # Any nonzero status stops the session here: no SNAPSHOT71 and no
        # SAFE_OFF is transmitted.
        responder = select_single_responder(responders)
        source_id = responder["id"]
        record["discovered_source_id"] = source_id
        persist()

        # --- PRE snapshot: as-found evidence --------------------------
        record["commands_sent"].append(
            send_command(ser, "@SNAPSHOT71 %d" % source_id)
        )
        pre_raw, pre_decoded = capture_snapshot(
            read_until(ser,
                       ["SNAPSHOT71_END", "SNAPSHOT71_ABORT"],
                       SNAPSHOT_TIMEOUT_S),
            path_for("PRE"),
            register("PRE"),
        )
        record["pre_snapshot"] = pre_decoded

        # The 71 as-found bytes are already fsync'd on disk and their .bin
        # path is already in the JSON, BEFORE @SAFE_OFF is transmitted.
        persist()

        identity = record["pre_snapshot"]["identity"]
        if identity["model_0x03"] != EXPECTED_MODEL:
            raise SurveyError("PRE model %s, expected %d"
                              % (identity["model_0x03"], EXPECTED_MODEL))
        if identity["id_register_0x05"] != source_id:
            raise SurveyError(
                "PRE ID register %s disagrees with discovered ID %d"
                % (identity["id_register_0x05"], source_id)
            )
        if identity["response_status_0x08"] != EXPECTED_RESPONSE_STATUS:
            raise SurveyError(
                "PRE ResponseStatus %s, expected %d"
                % (identity["response_status_0x08"],
                   EXPECTED_RESPONSE_STATUS)
            )

        # --- SAFE_OFF: the single permitted write ---------------------
        record["commands_sent"].append(
            send_command(ser, "@SAFE_OFF %d" % source_id)
        )
        record["safe_off"] = parse_safe_off(
            read_until(ser,
                       ["SAFE_OFF_RESULT PASS", "CUT_SERVO_POWER_NOW",
                        "SAFE_OFF_ABORT"],
                       SAFE_OFF_TIMEOUT_S)
        )
        persist()

        # --- POST snapshots -------------------------------------------
        posts = []
        for index in (1, 2):
            record["commands_sent"].append(
                send_command(ser, "@SNAPSHOT71 %d" % source_id)
            )
            slot = "POST%d" % index
            raw, decoded = capture_snapshot(
                read_until(ser,
                           ["SNAPSHOT71_END", "SNAPSHOT71_ABORT"],
                           SNAPSHOT_TIMEOUT_S),
                path_for(slot),
                register(slot),
            )
            record["post_snapshot_%d" % index] = decoded
            posts.append(raw)
            persist()

        for index, raw in enumerate(posts, start=1):
            if raw[ADDR_TORQUE_ENABLE] != 0:
                raise SurveyError(
                    "POST%d TorqueEnable=%d, expected 0"
                    % (index, raw[ADDR_TORQUE_ENABLE])
                )

        comparison = compare_snapshots(pre_raw, posts[0], posts[1])
        record["comparison"] = comparison
        persist()

        if not comparison["summary"]["all_ok"]:
            raise SurveyError("snapshot invariants violated; see comparison")

        record["survey_result"] = SURVEY_RESULT_CAPTURED
        record["verdict"] = "PASS"
        record["reason"] = "read-only survey complete; single permitted write verified"
        return record

    # Any exception below stamps verdict=FAIL on the record and is re-raised;
    # the port is always closed; no emergency command is ever sent.
    guard_session(record, session, persist, close_port)

    print("REPORT: %s" % report_path)
    return record


# --------------------------------------------------------------------------
# Offline self-test
# --------------------------------------------------------------------------

# Real ST-3215-C018 hardware bytes, unit M13, captured 2026-08-14 in
# 09_Logs/Calibration/Digital_Zero/..._st3215_pre_recenter_backup.json.
# Chosen because it exercises a NEGATIVE PositionOffset (0xFE1A) and is
# ground truth rather than a synthetic vector.
GOLDEN_M13_HEX = (
    "030A0009030D0000010000FF0F468C28F4010C2C2F10000010000101F401011AFE"
    "0014C8190AC8C80000000000000000F401000000000001CF07000000006E260000"
    "00CF070000"
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
        except Exception as exc:  # noqa: BLE001
            self.failed.append(name)
            print("  FAIL  %s (wrong exception %r)" % (name, exc))
            return
        self.failed.append(name)
        print("  FAIL  %s (no exception raised)" % name)


def run_self_test():
    print("=" * 66)
    print(" MATDOG ST3215 SOURCE SURVEY V1 — OFFLINE SELF-TEST")
    print("=" * 66)
    print("No serial port is opened. No hardware is contacted.")
    print()

    test = SelfTest()
    golden = parse_raw71_hex(GOLDEN_M13_HEX)
    decoded = decode_snapshot(golden)

    print("[1] Golden frame decode (real M13 hardware bytes)")
    test.check("frame is 71 bytes", len(golden) == SNAPSHOT_LEN)
    test.check("model 777 at 0x03",
               decoded["identity"]["model_0x03"] == 777,
               decoded["identity"]["model_0x03"])
    test.check("ID register 13", decoded["identity"]["id_register_0x05"] == 13)
    test.check("ResponseStatus 1",
               decoded["identity"]["response_status_0x08"] == 1)
    test.check("PositionOffset 0xFE1A -> -486",
               decoded["eeprom"]["position_offset_0x1F_signed"] == -486,
               decoded["eeprom"]["position_offset_0x1F_signed"])
    test.check("PresentPosition 1999",
               decoded["telemetry"]["present_position_0x38_raw16"] == 1999)
    test.check("physical_raw 1513", decoded["physical_raw"] == 1513,
               decoded["physical_raw"])
    test.check("TorqueEnable 0", decoded["ram"]["torque_enable_0x28"] == 0)
    test.check("Lock 1", decoded["ram"]["lock_0x37"] == 1)
    test.check("no anomalies", not any(decoded["anomalies"].values()))

    print()
    print("[2] 71-byte length and format validation")
    test.expect_raises("70-byte frame rejected", SurveyError,
                       parse_raw71_hex, GOLDEN_M13_HEX[:-2])
    test.expect_raises("72-byte frame rejected", SurveyError,
                       parse_raw71_hex, GOLDEN_M13_HEX + "00")
    test.expect_raises("odd hex length rejected", SurveyError,
                       parse_raw71_hex, GOLDEN_M13_HEX[:-1])
    test.expect_raises("non-hex rejected", SurveyError,
                       parse_raw71_hex, "ZZ" + GOLDEN_M13_HEX[2:])
    test.expect_raises("empty rejected", SurveyError, parse_raw71_hex, "")
    test.expect_raises("non-string rejected", SurveyError,
                       parse_raw71_hex, None)
    test.check("map covers exactly 71 addresses",
               (EEPROM_RANGE[1] - EEPROM_RANGE[0] + 1)
               + 1
               + (RAM_INVARIANT_RANGE[1] - RAM_INVARIANT_RANGE[0] + 1)
               + (TELEMETRY_RANGE[1] - TELEMETRY_RANGE[0] + 1)
               == SNAPSHOT_LEN)
    test.check("last address is 0x46", TELEMETRY_RANGE[1] == SNAPSHOT_LAST)

    print()
    print("[3] PositionOffset signedness (two's complement, little endian)")
    for hex_pair, expected in (
        ("1AFE", -486), ("0000", 0), ("7F00", 127),
        ("FF07", 2047), ("00F8", -2048), ("FFFF", -1),
    ):
        frame = bytearray(golden)
        frame[0x1F:0x21] = bytes.fromhex(hex_pair)
        value = i16le_twos_complement(bytes(frame), ADDR_POSITION_OFFSET)
        test.check("offset %s -> %d" % (hex_pair, expected),
                   value == expected, value)

    frame = bytearray(golden)
    frame[0x1F:0x21] = bytes.fromhex("0080")  # 0x8000 = -32768
    out_of_range = decode_snapshot(bytes(frame))
    test.check("0x8000 flagged out of range",
               out_of_range["anomalies"]["position_offset_out_of_range"])

    print()
    print("[4] Sign-convention separation (offset vs telemetry)")
    same_bytes = 0xFE1A
    twos = same_bytes - 0x10000
    sm15 = sign_magnitude(same_bytes, 15)
    test.check("two's complement 0xFE1A -> -486", twos == -486, twos)
    test.check("sign-magnitude 0xFE1A -> -32282", sm15 == -32282, sm15)
    test.check("conventions are NOT interchangeable", twos != sm15)
    test.check("present_load uses bit10",
               sign_magnitude(0x0400 | 5, 10) == -5)

    print()
    print("[5] EEPROM difference detection")
    mutated = bytearray(golden)
    mutated[0x15] = (mutated[0x15] + 1) & 0xFF  # P coefficient
    diffs = compare_ranges(golden, bytes(mutated), EEPROM_RANGE)
    test.check("EEPROM 0x15 change detected", len(diffs) == 1, diffs)
    test.check("diff reports address 0x15",
               diffs and diffs[0]["address"] == "0x15")
    cmp_bad = compare_snapshots(golden, bytes(mutated), bytes(mutated))
    test.check("EEPROM mutation fails invariant",
               not cmp_bad["summary"]["eeprom_invariant_ok"])
    test.check("EEPROM mutation fails overall",
               not cmp_bad["summary"]["all_ok"])

    print()
    print("[6] Non-torque RAM difference detection")
    ram_mutated = bytearray(golden)
    ram_mutated[0x30] = (ram_mutated[0x30] + 1) & 0xFF  # TorqueLimit low
    cmp_ram = compare_snapshots(golden, bytes(ram_mutated), bytes(ram_mutated))
    test.check("RAM 0x30 change detected",
               not cmp_ram["summary"]["ram_invariant_ok"])
    test.check("RAM mutation fails overall",
               not cmp_ram["summary"]["all_ok"])

    print()
    print("[7] Telemetry-only difference accepted")
    tele = bytearray(golden)
    for addr in range(TELEMETRY_RANGE[0], TELEMETRY_RANGE[1] + 1):
        tele[addr] = (tele[addr] + 7) & 0xFF
    cmp_tele = compare_snapshots(golden, bytes(tele), bytes(tele))
    test.check("telemetry drift leaves EEPROM ok",
               cmp_tele["summary"]["eeprom_invariant_ok"])
    test.check("telemetry drift leaves RAM ok",
               cmp_tele["summary"]["ram_invariant_ok"])
    test.check("telemetry drift accepted overall",
               cmp_tele["summary"]["all_ok"])
    test.check("telemetry diffs are reported, not hidden",
               len(cmp_tele["pre_vs_post1"]["telemetry_diffs"]) == 15)

    print()
    print("[8] TorqueEnable transition rules")
    pre_on = bytearray(golden)
    pre_on[ADDR_TORQUE_ENABLE] = 1
    cmp_ok = compare_snapshots(bytes(pre_on), golden, golden)
    test.check("PRE torque 1 -> POST torque 0 accepted",
               cmp_ok["summary"]["all_ok"])
    test.check("PRE torque recorded",
               cmp_ok["summary"]["torque_enable_pre"] == 1)
    post_on = bytearray(golden)
    post_on[ADDR_TORQUE_ENABLE] = 1
    cmp_bad_t = compare_snapshots(golden, golden, bytes(post_on))
    test.check("POST torque 1 rejected", not cmp_bad_t["summary"]["all_ok"])
    test.check("POST torque flag false",
               not cmp_bad_t["summary"]["torque_enable_post_ok"])

    print()
    print("[9] Physical label validation and allocation")
    test.check("17 physical units", len(PHYSICAL_UNITS) == 17)
    test.check("17 unique target IDs",
               len(set(PHYSICAL_UNITS.values())) == 17)
    test.check("M22 -> 13", PHYSICAL_UNITS["M22"] == 13)
    test.check("ELR04 -> 55", PHYSICAL_UNITS["ELR04"] == 55)
    test.check("NEW05 -> 31", PHYSICAL_UNITS["NEW05"] == 31)
    test.check("M11 -> 52", PHYSICAL_UNITS["M11"] == 52)
    test.expect_raises("unknown label rejected", SurveyError,
                       validate_physical_label, "M99")
    test.expect_raises("empty label rejected", SurveyError,
                       validate_physical_label, "")
    test.expect_raises("lowercase label rejected", SurveyError,
                       validate_physical_label, "m22")
    test.check("source ID != target ID is NOT an error by itself",
               PHYSICAL_UNITS["M22"] != 22)

    print()
    print("[10] Hardware confirmation fails closed")
    test.check("expected string for M22",
               expected_confirmation("M22") == "SOURCE_SURVEY_M22")
    test.check("exact match accepted",
               verify_confirmation("M22", "SOURCE_SURVEY_M22"))
    test.expect_raises("wrong label rejected", SurveyError,
                       verify_confirmation, "M22", "SOURCE_SURVEY_M33")
    test.expect_raises("case mismatch rejected", SurveyError,
                       verify_confirmation, "M22", "source_survey_m22")
    test.expect_raises("missing prefix rejected", SurveyError,
                       verify_confirmation, "M22", "M22")
    test.expect_raises("empty confirmation rejected", SurveyError,
                       verify_confirmation, "M22", "")
    test.expect_raises("None confirmation rejected", SurveyError,
                       verify_confirmation, "M22", None)
    test.expect_raises("whitespace-padded rejected", SurveyError,
                       verify_confirmation, "M22", " SOURCE_SURVEY_M22 ")

    print()
    print("[11] Command surface is closed")
    for allowed in ("@SCAN 0 253", "@SNAPSHOT71 0", "@SNAPSHOT71 13",
                    "@SNAPSHOT71 253", "@SAFE_OFF 13", "@SAFE_OFF 253"):
        test.check("allowed: %s" % allowed,
                   assert_command_allowed(allowed) == allowed)
    for forbidden in ("@NORMALIZE_MATDOG 13", "@QC_FAST 13", "@READ 13",
                      "@DUMP_RAW 13", "@BENCH 13 1000", "@CAPTURE 13 100 50",
                      "@RAMTEST 13", "@PING 13", "@HELP",
                      "@SCAN 0 254", "@SCAN 1 253", "@SCAN 0 100",
                      "@SNAPSHOT71 254", "@SAFE_OFF 254",
                      "@SNAPSHOT71 -1", "@SAFE_OFF 999", "@SNAPSHOT71",
                      "@SNAPSHOT71 13 13", "", "SCAN 0 253",
                      "@SNAPSHOT71 13\n@SAFE_OFF 13", " @SAFE_OFF 13"):
        test.expect_raises("rejected: %r" % forbidden, SafetyViolation,
                           assert_command_allowed, forbidden)
    test.expect_raises("non-string rejected", SafetyViolation,
                       assert_command_allowed, 13)

    print()
    print("[12] Firmware response parsing")
    test.expect_raises("snapshot abort fails closed", SurveyError,
                       parse_snapshot,
                       ["SNAPSHOT71_ABORT PING=-1 STATUS=0x00"])
    test.expect_raises("snapshot without PASS fails", SurveyError,
                       parse_snapshot, ["RAW71_HEX=" + GOLDEN_M13_HEX])
    test.expect_raises("snapshot without RAW fails", SurveyError,
                       parse_snapshot, ["SNAPSHOT71_RESULT PASS"])
    ok_raw, _ = parse_snapshot([
        "SNAPSHOT71_BEGIN ID=13 START=0x00 LEN=71 STATUS=0x00",
        "RAW71_HEX=" + GOLDEN_M13_HEX,
        "SNAPSHOT71_RAW_COMPLETE",
        "MODEL_0x03=777",
        "SNAPSHOT71_RESULT PASS",
        "SNAPSHOT71_END",
    ])
    test.check("valid snapshot parsed", ok_raw == golden)
    test.expect_raises("safe_off gate abort fails closed", SurveyError,
                       parse_safe_off,
                       ["SAFE_OFF_ABORT: SNAPSHOT71_GATE_REQUIRED"])
    test.expect_raises("safe_off write_result 0 fails", SurveyError,
                       parse_safe_off,
                       ["SAFE_OFF_WRITE_RESULT=0", "SAFE_OFF_WRITE_STATUS=0x00",
                        "SAFE_OFF_READBACK=0", "SAFE_OFF_READ_STATUS=0x00",
                        "SAFE_OFF_RESULT PASS"])
    test.expect_raises("safe_off readback 1 fails", SurveyError,
                       parse_safe_off,
                       ["SAFE_OFF_WRITE_RESULT=1", "SAFE_OFF_WRITE_STATUS=0x00",
                        "SAFE_OFF_READBACK=1", "SAFE_OFF_READ_STATUS=0x00",
                        "SAFE_OFF_RESULT PASS"])
    test.expect_raises("CUT_SERVO_POWER_NOW fails closed", SurveyError,
                       parse_safe_off,
                       ["SAFE_OFF_WRITE_RESULT=1", "SAFE_OFF_WRITE_STATUS=0x00",
                        "SAFE_OFF_READBACK=0", "SAFE_OFF_READ_STATUS=0x00",
                        "SAFE_OFF_RESULT FAIL", "CUT_SERVO_POWER_NOW"])
    test.check("valid safe_off parsed", parse_safe_off([
        "SAFE_OFF_WRITE_RESULT=1", "SAFE_OFF_WRITE_STATUS=0x00",
        "SAFE_OFF_READBACK=0", "SAFE_OFF_READ_STATUS=0x00",
        "SAFE_OFF_RESULT PASS",
    ])["SAFE_OFF_WRITE_RESULT"] == 1)
    test.expect_raises("scan without SCAN_COMPLETE fails", SurveyError,
                       parse_scan, ["SCAN_RESULT FOUND=1"])
    test.expect_raises("scan count disagreement fails", SurveyError,
                       parse_scan,
                       ["FOUND ID=13 MODEL=777 PING_STATUS=0x00 "
                        "MODEL_STATUS=0x00",
                        "SCAN_RESULT FOUND=2", "SCAN_COMPLETE"])
    one = parse_scan(["FOUND ID=13 MODEL=777 PING_STATUS=0x00 "
                      "MODEL_STATUS=0x00",
                      "SCAN_RESULT FOUND=1", "SCAN_COMPLETE"])
    test.check("single responder parsed", len(one) == 1 and one[0]["id"] == 13)
    two = parse_scan(["FOUND ID=1 MODEL=777 PING_STATUS=0x00 MODEL_STATUS=0x00",
                      "FOUND ID=13 MODEL=777 PING_STATUS=0x00 MODEL_STATUS=0x00",
                      "SCAN_RESULT FOUND=2", "SCAN_COMPLETE"])
    test.check("two responders detected (survey must reject)", len(two) == 2)

    print()
    print("[13] Frozen firmware SHA256 verification")
    try:
        firmware = verify_frozen_firmware()
        test.check("firmware source SHA256 matches",
                   firmware["source"]["match"])
        test.check("firmware binary SHA256 matches",
                   firmware["binary"]["match"])
    except SurveyError as exc:
        test.check("frozen firmware verification", False, str(exc))

    print()
    print("[14] Classification policy — facts only, no invented signatures")
    test.check("survey result vocabulary",
               SURVEY_RESULT_CAPTURED == "SOURCE_SIGNATURE_CAPTURED")
    test.check("classification vocabulary",
               PROFILE_CLASSIFICATION_PENDING == "PENDING_OFFLINE_REVIEW")

    skeleton = build_record_skeleton(
        "M22", "SOURCE_SURVEY_M22", "/dev/null", {"source": {}, "binary": {}})
    test.check("record starts PENDING_OFFLINE_REVIEW",
               skeleton["profile_classification"]
               == PROFILE_CLASSIFICATION_PENDING)
    test.check("record starts with no survey_result",
               skeleton["survey_result"] is None)
    test.check("record starts INCOMPLETE",
               skeleton["verdict"] == "INCOMPLETE")
    test.check("record starts with no raw artifacts",
               skeleton["raw_artifacts"] == {})
    test.check("record binds physical identity",
               skeleton["physical_label"] == "M22")
    test.check("record binds future target independently of source ID",
               skeleton["future_target_id"] == 13)
    test.check("record carries no source-ID-derived classification",
               "source_profile" not in skeleton)
    test.check("record self-identifies by runner SHA256",
               len(skeleton["runner_sha256"]) == 64)

    module = sys.modules[__name__]
    classifiers = [
        name for name in dir(module)
        if "classif" in name.lower() and callable(getattr(module, name))
    ]
    test.check("module exposes no classifier function",
               not classifiers, classifiers)

    print()
    print("[15] RAW 71-byte evidence artifacts (exclusive create + fsync)")
    stamp = "20260826_000000Z"
    with tempfile.TemporaryDirectory(prefix="matdog_survey_raw_") as tmpdir:
        pre_path = raw_artifact_path(tmpdir, "M22", stamp, "PRE")
        post1_path = raw_artifact_path(tmpdir, "M22", stamp, "POST1")
        post2_path = raw_artifact_path(tmpdir, "M22", stamp, "POST2")
        test.check("three distinct artifact paths",
                   len({pre_path, post1_path, post2_path}) == 3)
        test.check("PRE artifact basename",
                   os.path.basename(pre_path).endswith(
                       "01_pre_safeoff_state71.bin"))
        test.check("POST1 artifact basename",
                   os.path.basename(post1_path).endswith(
                       "02_post_safeoff_state71.bin"))
        test.check("POST2 artifact basename",
                   os.path.basename(post2_path).endswith(
                       "03_repeat_state71.bin"))
        test.expect_raises("unknown artifact slot rejected", SurveyError,
                           raw_artifact_path, tmpdir, "M22", stamp, "PRE2")

        artifact = persist_raw71(golden, pre_path)
        test.check("artifact file created", os.path.isfile(pre_path))
        test.check("artifact is exactly 71 bytes on disk",
                   os.path.getsize(pre_path) == SNAPSHOT_LEN,
                   os.path.getsize(pre_path))
        with open(pre_path, "rb") as handle:
            on_disk = handle.read()
        test.check("artifact bytes identical to the frame", on_disk == golden)
        test.check("artifact sha256 matches frame and file",
                   artifact["sha256"] == sha256_bytes(golden)
                   == sha256_file(pre_path))
        test.check("artifact records its own path",
                   artifact["path"] == pre_path)
        test.check("artifact records the byte count",
                   artifact["bytes"] == SNAPSHOT_LEN)
        test.check("artifact records flush + fsync",
                   artifact["flushed_and_fsynced"] is True)
        test.check("artifact records exclusive create",
                   artifact["exclusive_create"] is True)

        # Evidence is never overwritten.
        test.expect_raises("existing artifact fails closed", SurveyError,
                           persist_raw71, golden, pre_path)
        test.check("rejected rewrite left the artifact untouched",
                   sha256_file(pre_path) == sha256_bytes(golden))
        mutated_frame = bytearray(golden)
        mutated_frame[0x15] = (mutated_frame[0x15] + 1) & 0xFF
        test.expect_raises("collision with different bytes fails closed",
                           SurveyError, persist_raw71,
                           bytes(mutated_frame), pre_path)
        test.check("collision did not alter the stored evidence",
                   sha256_file(pre_path) == sha256_bytes(golden))

        short_path = os.path.join(tmpdir, "short.bin")
        test.expect_raises("70-byte frame refused", SurveyError,
                           persist_raw71, golden[:-1], short_path)
        test.check("refused short frame created no file",
                   not os.path.exists(short_path))
        long_path = os.path.join(tmpdir, "long.bin")
        test.expect_raises("72-byte frame refused", SurveyError,
                           persist_raw71, golden + b"\x00", long_path)
        test.check("refused long frame created no file",
                   not os.path.exists(long_path))
        str_path = os.path.join(tmpdir, "str.bin")
        test.expect_raises("non-bytes frame refused", SurveyError,
                           persist_raw71, GOLDEN_M13_HEX, str_path)
        test.check("refused non-bytes created no file",
                   not os.path.exists(str_path))

    print()
    print("[16] RAW bytes reach disk BEFORE decode_snapshot() runs")
    snapshot_lines = [
        "SNAPSHOT71_BEGIN ID=13 START=0x00 LEN=71 STATUS=0x00",
        "RAW71_HEX=" + GOLDEN_M13_HEX,
        "SNAPSHOT71_RAW_COMPLETE",
        "MODEL_0x03=777",
        "SNAPSHOT71_RESULT PASS",
        "SNAPSHOT71_END",
    ]
    with tempfile.TemporaryDirectory(prefix="matdog_survey_order_") as tmpdir:
        order_path = raw_artifact_path(tmpdir, "M22", stamp, "PRE")
        seen = {}
        registered = []
        this_module = sys.modules[__name__]
        real_decode = this_module.decode_snapshot

        def probing_decode(raw):
            """Observe the on-disk state at the instant decoding begins."""
            exists = os.path.isfile(order_path)
            seen["exists"] = exists
            seen["size"] = os.path.getsize(order_path) if exists else -1
            seen["sha"] = sha256_file(order_path) if exists else None
            seen["registered_paths"] = [item["path"] for item in registered]
            return real_decode(raw)

        this_module.decode_snapshot = probing_decode
        try:
            raw_back, decoded_back = capture_snapshot(
                snapshot_lines, order_path, registered.append)
        finally:
            this_module.decode_snapshot = real_decode

        test.check("decode_snapshot restored",
                   this_module.decode_snapshot is real_decode)
        test.check("frame parsed to the golden 71 bytes", raw_back == golden)
        test.check("artifact already existed when decode started",
                   seen.get("exists") is True)
        test.check("artifact was already 71 bytes when decode started",
                   seen.get("size") == SNAPSHOT_LEN, seen.get("size"))
        test.check("on-disk bytes were already final when decode started",
                   seen.get("sha") == sha256_bytes(golden))
        test.check("path was registered before decode started",
                   seen.get("registered_paths") == [order_path],
                   seen.get("registered_paths"))
        test.check("exactly one registration", len(registered) == 1)
        test.check("decoded record carries raw_hex",
                   decoded_back["raw_hex"] == GOLDEN_M13_HEX.upper())
        test.check("decoded record carries raw_sha256",
                   decoded_back["raw_sha256"] == sha256_bytes(golden))
        test.check("decoded record carries decoded data",
                   decoded_back["identity"]["model_0x03"] == 777
                   and decoded_back["physical_raw"] == 1513)
        test.check("decoded record carries the .bin path",
                   decoded_back["raw_artifact"]["path"] == order_path)
        test.check("decoded record keeps firmware_reported",
                   isinstance(decoded_back["firmware_reported"], dict))

        abort_path = os.path.join(tmpdir, "abort.bin")
        test.expect_raises("aborted snapshot never persists", SurveyError,
                           capture_snapshot,
                           ["SNAPSHOT71_ABORT PING=-1 STATUS=0x00"],
                           abort_path)
        test.check("no artifact written for an aborted snapshot",
                   not os.path.exists(abort_path))

    print()
    print("[17] SCAN status gate — any nonzero status fails closed")
    clean = {"id": 13, "model": 777, "ping_status": 0, "model_status": 0}
    test.check("clean single responder accepted",
               select_single_responder([dict(clean)])["id"] == 13)
    test.expect_raises("zero responders rejected", SurveyError,
                       select_single_responder, [])
    test.expect_raises("two responders rejected", SurveyError,
                       select_single_responder, [dict(clean), dict(clean)])
    test.expect_raises("model != 777 rejected", SurveyError,
                       select_single_responder, [dict(clean, model=1)])
    test.expect_raises("model -1 rejected", SurveyError,
                       select_single_responder, [dict(clean, model=-1)])
    for status in (1, 2, 0x80, 0xFF):
        test.expect_raises("ping_status 0x%02X rejected" % status,
                           SurveyError, select_single_responder,
                           [dict(clean, ping_status=status)])
        test.expect_raises("model_status 0x%02X rejected" % status,
                           SurveyError, select_single_responder,
                           [dict(clean, model_status=status)])
    test.expect_raises("both statuses nonzero rejected", SurveyError,
                       select_single_responder,
                       [dict(clean, ping_status=1, model_status=1)])
    test.expect_raises("missing ping_status rejected", SurveyError,
                       select_single_responder,
                       [{"id": 13, "model": 777, "model_status": 0}])
    test.expect_raises("missing model_status rejected", SurveyError,
                       select_single_responder,
                       [{"id": 13, "model": 777, "ping_status": 0}])
    test.expect_raises("nonzero PING_STATUS rejected end to end", SurveyError,
                       select_single_responder,
                       parse_scan(["FOUND ID=13 MODEL=777 PING_STATUS=0x01 "
                                   "MODEL_STATUS=0x00",
                                   "SCAN_RESULT FOUND=1", "SCAN_COMPLETE"]))
    test.expect_raises("nonzero MODEL_STATUS rejected end to end", SurveyError,
                       select_single_responder,
                       parse_scan(["FOUND ID=13 MODEL=777 PING_STATUS=0x00 "
                                   "MODEL_STATUS=0x80",
                                   "SCAN_RESULT FOUND=1", "SCAN_COMPLETE"]))
    test.check("clean responder accepted end to end",
               select_single_responder(
                   parse_scan(["FOUND ID=13 MODEL=777 PING_STATUS=0x00 "
                               "MODEL_STATUS=0x00",
                               "SCAN_RESULT FOUND=1",
                               "SCAN_COMPLETE"]))["id"] == 13)

    print()
    print("[18] Any exception leaves a FAIL record; port always closed")

    class FakeSerialException(Exception):
        """Stands in for serial.SerialException, never imported here."""

    def guard_record():
        return build_record_skeleton(
            "M22", "SOURCE_SURVEY_M22", "/dev/null",
            {"source": {}, "binary": {}})

    for name, exc in (
        ("SurveyError", SurveyError("precondition failed")),
        ("SafetyViolation", SafetyViolation("forbidden command")),
        ("FakeSerialException", FakeSerialException("port disappeared")),
        ("OSError", OSError("filesystem failure")),
        ("ValueError", ValueError("unexpected internal error")),
    ):
        guarded = guard_record()
        counters = {"persist": 0, "close": 0}

        def raiser(exc=exc):
            raise exc

        def counting_persist(counters=counters):
            counters["persist"] += 1

        def counting_close(counters=counters):
            counters["close"] += 1

        test.expect_raises("%s re-raised by the guard" % name, type(exc),
                           guard_session, guarded, raiser,
                           counting_persist, counting_close)
        test.check("%s -> verdict FAIL" % name, guarded["verdict"] == "FAIL")
        test.check("%s -> reason names the exception type" % name,
                   guarded["reason"].startswith(name + ": "),
                   guarded["reason"])
        test.check("%s -> record persisted in finally" % name,
                   counters["persist"] == 1)
        test.check("%s -> port closed in finally" % name,
                   counters["close"] == 1)
        test.check("%s -> guard sent no extra command" % name,
                   guarded["commands_sent"] == [])
        test.check("%s -> no survey_result claimed" % name,
                   guarded["survey_result"] is None)
        test.check("%s -> classification still pending" % name,
                   guarded["profile_classification"]
                   == PROFILE_CLASSIFICATION_PENDING)
        test.check("%s -> finished_utc stamped" % name,
                   "finished_utc" in guarded)

    broken = guard_record()
    broken_counters = {"close": 0}

    def failing_persist():
        raise OSError("report volume is full")

    def broken_close():
        broken_counters["close"] += 1

    def raise_runtime():
        raise RuntimeError("original failure")

    test.expect_raises("original exception survives a persist failure",
                       RuntimeError, guard_session, broken, raise_runtime,
                       failing_persist, broken_close)
    test.check("record still stamped FAIL when persist fails",
               broken["verdict"] == "FAIL")
    test.check("reason preserved when persist fails",
               broken["reason"] == "RuntimeError: original failure")
    test.check("port still closed when persist fails",
               broken_counters["close"] == 1)

    happy = guard_record()
    happy_counters = {"persist": 0, "close": 0}

    def happy_persist():
        happy_counters["persist"] += 1

    def happy_close():
        happy_counters["close"] += 1

    happy_result = guard_session(happy, lambda: "session-value",
                                 happy_persist, happy_close)
    test.check("guard returns the session value",
               happy_result == "session-value")
    test.check("guard leaves a successful record unstamped",
               happy["verdict"] == "INCOMPLETE")
    test.check("guard persists on success too",
               happy_counters["persist"] == 1)
    test.check("guard closes the port on success",
               happy_counters["close"] == 1)

    print()
    print("=" * 66)
    total = test.passed + len(test.failed)
    if test.failed:
        print(" SELF-TEST RESULT: FAIL  (%d/%d passed)"
              % (test.passed, total))
        for name in test.failed:
            print("   failed: %s" % name)
        print("=" * 66)
        return 1
    print(" SELF-TEST RESULT: PASS  (%d/%d checks)" % (test.passed, total))
    print("=" * 66)
    return 0


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(
        prog=TOOL_NAME,
        description="MATDOG ST3215 Read-Only Source Signature Survey V1",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--self-test", action="store_true",
                      help="run the offline self-test; opens no serial port")
    mode.add_argument("--verify-firmware", action="store_true",
                      help="verify frozen firmware SHA256; opens no port")
    mode.add_argument("--command-surface", action="store_true",
                      help="print the allowed command surface; opens no port")
    mode.add_argument("--survey", action="store_true",
                      help="HARDWARE MODE. Requires --label and --confirm.")

    parser.add_argument("--label", help="physical unit label, e.g. M22")
    parser.add_argument("--confirm",
                        help="confirmation string SOURCE_SURVEY_<LABEL>")
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--report-dir", default=DEFAULT_REPORT_DIR)

    args = parser.parse_args(argv)

    if args.self_test:
        return run_self_test()

    if args.verify_firmware:
        print(json.dumps(verify_frozen_firmware(), indent=2, sort_keys=True))
        return 0

    if args.command_surface:
        print(json.dumps(command_surface_audit(), indent=2, sort_keys=True))
        return 0

    if args.survey:
        if not args.label or not args.confirm:
            print("ERROR: --survey requires --label and --confirm",
                  file=sys.stderr)
            return 2
        try:
            record = run_survey(args.label, args.confirm,
                                args.port, args.report_dir)
        except (SurveyError, SafetyViolation) as exc:
            print("SURVEY FAILED: %s" % exc, file=sys.stderr)
            return 1
        return 0 if record["verdict"] == "PASS" else 1

    # Fail-closed default: no mode selected means no action, no port.
    parser.print_usage(sys.stderr)
    print("ERROR: no mode selected. Nothing was done and no port was opened.",
          file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
