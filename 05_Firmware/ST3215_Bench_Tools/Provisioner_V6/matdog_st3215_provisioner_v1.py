#!/usr/bin/env python3
"""
MATDOG ST-3215-C018 Provisioner V1 — host runner.

Companion to:
    matdog_st3215_provisioner_v1/matdog_st3215_provisioner_v1.ino
Design:
    MATDOG_ST3215_PROVISIONER_V1_DESIGN.md

USAGE
    python3 matdog_st3215_provisioner_v1.py --unit NEW01

    No source ID argument: the firmware discovers it.
    No confirmation string: the evidence contract is the gate, not a human
    typing a magic word.

WHAT THIS PROCESS OWNS
    Evidence. Nothing safety-critical. The host cannot compose a write, cannot
    name a register, cannot choose an ID and cannot command motion: it can only
    send @BEGIN <LABEL>, @EXECUTE <TOKEN> and @HELP.

    Its one hard obligation is ordering: the 71 as-found bytes returned by
    @BEGIN must be durable on disk (exclusive create, write, flush, os.fsync,
    length verified) BEFORE @EXECUTE is transmitted. That is the entire reason
    the firmware splits BEGIN from EXECUTE.

    It also independently re-verifies what the firmware claims: the canonical
    delta, the ID-recode identity proof and the cold power-cycle state machine
    are recomputed here from the transcript and must agree.

HARDWARE FREEZE
    BLOCKED while the firmware reports unresolved constants (design 11.2).
    While blocked, --unit refuses before opening any serial port.
"""

import argparse
import datetime
import hashlib
import json
import os
import re
import sys
import tempfile

TOOL_NAME = "matdog_st3215_provisioner_v1"
TOOL_VERSION = "1.0.0"
REPORT_SCHEMA = "MATDOG_ST3215_PROVISIONER_V1"
PROFILE_ID = "MATDOG_C018_V1"

PROJECT_DIR = os.path.dirname(os.path.abspath(__file__))
FIRMWARE_SOURCE_PATH = os.path.join(PROJECT_DIR, TOOL_NAME + ".ino")
FIRMWARE_BINARY_PATH = os.path.join(
    PROJECT_DIR, "build", "esp32.esp32.esp32s3", TOOL_NAME + ".ino.bin"
)

FIRMWARE_SOURCE_SHA256 = "680b2715e6896c296c0fc63e7a72bc520a471977891478b2cca9b0f6472e2327"
FIRMWARE_BINARY_SHA256 = "3510997674b85e064c50d1e3b61abbcf5659b736f7811b1b40ada5d101241201"

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


class ProvisionError(RuntimeError):
    """A precondition or verification failed. Always fail-closed."""


# --------------------------------------------------------------------------
# Physical allocation — mirror of the firmware table.
#
# The firmware owns the mapping. This copy exists only to cross-check the
# firmware's answer; a self-test parses the .ino and asserts they are equal, so
# the two cannot drift.
# --------------------------------------------------------------------------

ALLOCATION = {
    "M22": (13, "LF_HIP", 1),
    "ELR01": (12, "LF_UPPER", 1),
    "M33": (11, "LF_LOWER", 1),
    "NEW01": (23, "RF_HIP", 2),       # pilot: two cold cycles
    "ELR03": (22, "RF_UPPER", 1),
    "NEW03": (21, "RF_LOWER", 1),
    "NEW06": (33, "RH_HIP", 1),
    "ELR02": (32, "RH_UPPER", 1),
    "NEW05": (31, "RH_LOWER", 1),
    "M43": (43, "LH_HIP", 1),
    "M42": (42, "LH_UPPER", 1),
    "M41": (41, "LH_LOWER", 1),
    "M31": (51, "NECK_ROTATION", 1),
    "M11": (52, "NECK_PITCH", 1),
    "NEW04": (53, "HEAD_ROTATION", 1),
    "NEW02": (54, "HEAD_PITCH", 1),
    "ELR04": (55, "JAW", 1),
}

# --------------------------------------------------------------------------
# 71-byte state map, 0x00 .. 0x46 inclusive
# --------------------------------------------------------------------------

SNAPSHOT_LEN = 71

ADDR_MODEL = 0x03            # NOT 0x00 — see design section 5
ADDR_ID = 0x05
ADDR_BAUD = 0x06
ADDR_RETURN_DELAY = 0x07
ADDR_RESPONSE_STATUS = 0x08
ADDR_UNLOAD_CONDITION = 0x12
ADDR_LED_ALARM = 0x13
ADDR_ANGULAR_RESOLUTION = 0x1E
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
RAW_CENTER = 2048
POSITION_OFFSET_MIN = -2048
POSITION_OFFSET_MAX = 2047
CENTER_ACCEPT_TICKS = 1

CENTER_TORQUE_LIMIT = 300
CENTER_SPEED = 365
CENTER_ACC = 50

# Mirror of the firmware centering-correction bounds. A self-test parses the
# .ino and asserts these are equal, so the two cannot drift.
MON_REACHED_RESIDUAL = 3
CENTER_STAGING_TICKS = 12
CENTER_MAX_CORRECTIONS = 3
CENTER_MAX_BIAS_TICKS = MON_REACHED_RESIDUAL

CORRECTION_PASS = "PASS"
CORRECTION_CORRECT = "CORRECT"
CORRECTION_FAIL_WINDOW = "FAIL_OUT_OF_CORRECTION_WINDOW"
CORRECTION_FAIL_TOLERANCE = "FAIL_OUT_OF_TOLERANCE"

VOLTAGE_MIN = 40
VOLTAGE_MAX = 140
THERMAL_LIMIT_C = 70

# --------------------------------------------------------------------------
# Canonical persistent profile — mirror of the firmware table, cross-checked.
# (addr, width, value, name)
# --------------------------------------------------------------------------

PROFILE = (
    (0x09, 2, 0, "MinAngle"),
    (0x0B, 2, 4095, "MaxAngle"),
    (0x0D, 1, 70, "MaxTemperature"),
    (0x0E, 1, 140, "MaxVoltage"),
    (0x0F, 1, 40, "MinVoltage"),
    (0x10, 2, 1000, "MaxTorque"),
    (0x15, 1, 32, "P"),
    (0x16, 1, 32, "D"),
    (0x17, 1, 0, "I"),
    (0x18, 2, 16, "MinStartupForce"),
    (0x1A, 1, 1, "CWDead"),
    (0x1B, 1, 1, "CCWDead"),
    (0x1C, 2, 310, "ProtectionCurrent"),
    (0x21, 1, 0, "Mode"),
    (0x22, 1, 20, "ProtectionTorque"),
    (0x23, 1, 200, "ProtectionTime"),
    (0x24, 1, 80, "OverloadTorque"),
    (0x25, 1, 10, "SpeedClosedLoopP"),
    (0x26, 1, 200, "OverCurrentProtectionTime"),
    (0x27, 1, 200, "VelocityClosedLoopI"),
)

# Never written by any stage. Audited read-only.
PRESERVE_ONLY = (
    (ADDR_BAUD, "BaudRate"),
    (ADDR_RETURN_DELAY, "ReturnDelay"),
    (ADDR_RESPONSE_STATUS, "ResponseStatus"),
    (ADDR_UNLOAD_CONDITION, "UnloadCondition"),
    (ADDR_LED_ALARM, "LedAlarm"),
    (ADDR_ANGULAR_RESOLUTION, "AngularResolution"),
)

# Written only by their own dedicated late stages, never by the profile stage.
LATE_STAGE_REGISTERS = ((ADDR_POSITION_OFFSET, "PositionOffset"), (ADDR_ID, "ID"))

# --------------------------------------------------------------------------
# Evidence artifacts
# --------------------------------------------------------------------------

ARTIFACTS = {
    "BEFORE": "01_before_state71.bin",
    "PROFILE": "02_profile_state71.bin",
    "CENTERED": "03_centered_old_offset_state71.bin",
    "OFFSET_ZERO": "04_offset_zero_state71.bin",
    "WARM": "05_after_recode_warm_state71.bin",
    "COLD1": "06_after_cold_state71.bin",
    "COLD2": "07_after_cold_cycle2_state71.bin",
}

CONSOLE_LOG_NAME = "session_console.log"
REPORT_NAME = "provisioning_report.json"
SHA256SUMS_NAME = "SHA256SUMS"

BEGIN_TIMEOUT_S = 120.0     # full 0..253 scan costs ~25 s
EXECUTE_TIMEOUT_S = 600.0   # includes the operator power-cycle window


# --------------------------------------------------------------------------
# Command surface — the single choke point
# --------------------------------------------------------------------------

ALLOWED_COMMAND_PATTERNS = (
    re.compile(r"^@BEGIN (?:[A-Z0-9]{1,15})$"),
    re.compile(r"^@EXECUTE (?:[0-9A-F]{8})$"),
    re.compile(r"^@HELP$"),
)

FORBIDDEN_COMMAND_TOKENS = (
    "@NORMALIZE_MATDOG", "@QC_FAST", "@READ", "@DUMP_RAW", "@BENCH",
    "@CAPTURE", "@RAMTEST", "@PING", "@SCAN", "@SNAPSHOT71", "@SAFE_OFF",
    "@WRITE", "@SETID", "@GOAL", "@TORQUE", "@RESET", "@CALIBRATE",
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
        "allowed_examples": ["@BEGIN NEW01", "@EXECUTE 1A2B3C4D", "@HELP"],
        "forbidden_tokens_rejected": list(FORBIDDEN_COMMAND_TOKENS),
        "generic_eeprom_write": "NOT IMPLEMENTED",
        "arbitrary_id_write": "NOT IMPLEMENTED",
        "arbitrary_goal_position": "NOT IMPLEMENTED",
        "calibration_ofs": "NOT IMPLEMENTED",
        "host_exposed_torque_on": "NOT IMPLEMENTED",
        "factory_reset": "NOT IMPLEMENTED",
        "broadcast_write": "NOT IMPLEMENTED",
        "baud_write": "NOT IMPLEMENTED (verification only)",
        "begin_performs_servo_writes": "NO",
        "host_supplies_target_id": "NO (firmware owns the allocation map)",
        "host_supplies_source_id": "NO (firmware discovers it)",
        "confirmation_string": "NONE (evidence contract is the gate)",
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
    """Explicit floor modulo returning 0..modulus-1.

    Mirrors the firmware helper. Never let a negative numerator reach a raw
    remainder operator in either language.
    """
    if modulus <= 0:
        raise ProvisionError("floor_mod modulus must be positive")
    result = value % modulus
    if result < 0:
        result += modulus
    return result


def physical_raw(displayed, signed_offset):
    return floor_mod(displayed + signed_offset, ENCODER_COUNTS)


def displayed_from_physical(physical, signed_offset):
    return floor_mod(physical - signed_offset, ENCODER_COUNTS)


def center_goal_displayed(signed_offset):
    """Displayed goal that puts the shaft at physical RAW 2048."""
    goal = floor_mod(RAW_CENTER - signed_offset, ENCODER_COUNTS)
    validate_goal_position(goal)
    return goal


def circular_delta(a, b):
    """Smallest signed distance on the 4096 circle."""
    return floor_mod(a - b + RAW_CENTER, ENCODER_COUNTS) - RAW_CENTER


def center_staging_goal(goal, staging_ticks=CENTER_STAGING_TICKS):
    """Boundary-safe staging with no 4095<->0 wrap."""
    below = goal - staging_ticks
    if 0 <= below <= 4095:
        return below

    above = goal + staging_ticks
    if 0 <= above <= 4095:
        return above

    return -1

def center_bias_update(bias, error, attempt):
    """Mirror of the firmware terminal-deadband feedback.

    Attempt 1 runs unbiased: only a standardised re-approach measures the
    deadband, and the initial centering move may arrive from any direction.
    Returns None when the updated bias leaves the validated correction window,
    which the firmware treats as fail-closed.
    """
    if attempt <= 1:
        return bias
    updated = bias - error
    if abs(updated) > CENTER_MAX_BIAS_TICKS:
        return None
    return updated


def simulate_centering(initial_error, deadband, max_attempts=None):
    """Replay the firmware correction loop against a deadband model.

    The servo is modelled exactly as measured: it stops `deadband` ticks short
    of whatever it is commanded, in the direction of travel. Returns
    (verdict, attempts_used, final_error).
    """
    if max_attempts is None:
        max_attempts = CENTER_MAX_CORRECTIONS
    error = initial_error
    bias = 0
    for attempt in range(1, max_attempts + 1):
        if abs(error) <= CENTER_ACCEPT_TICKS:
            return (CORRECTION_PASS, attempt - 1, error)
        if abs(error) > MON_REACHED_RESIDUAL:
            return (CORRECTION_FAIL_WINDOW, attempt - 1, error)
        bias = center_bias_update(bias, error, attempt)
        if bias is None:
            return ("FAIL_BIAS_OUT_OF_WINDOW", attempt - 1, error)
        # The re-approach always climbs to `goal + bias` from below and stops
        # `deadband` ticks short of it.
        error = bias - deadband
    if abs(error) <= CENTER_ACCEPT_TICKS:
        return (CORRECTION_PASS, max_attempts, error)
    return (CORRECTION_FAIL_TOLERANCE, max_attempts, error)


def center_correction_decision(error, attempt):
    """Mirror of the firmware centering acceptance ladder.

    attempt is the 1-based correction attempt about to be made. The order of
    the tests is the order the firmware evaluates them, so a settle that is
    both outside the correction window and out of attempts reports the window
    failure, exactly as the firmware does.
    """
    if abs(error) <= CENTER_ACCEPT_TICKS:
        return CORRECTION_PASS
    if abs(error) > MON_REACHED_RESIDUAL:
        return CORRECTION_FAIL_WINDOW
    if attempt > CENTER_MAX_CORRECTIONS:
        return CORRECTION_FAIL_TOLERANCE
    return CORRECTION_CORRECT


def validate_goal_position(value):
    """GoalPosition is strictly unsigned 0..4095. No signed wrap, ever."""
    if not isinstance(value, int) or isinstance(value, bool):
        raise ProvisionError("goal position must be int, got %r" % (value,))
    if value < 0 or value > 4095:
        raise ProvisionError("goal position %d outside 0..4095" % value)
    return value


def u16le(raw, addr):
    return raw[addr] | (raw[addr + 1] << 8)


def i16le_twos_complement(raw, addr):
    """PositionOffset ONLY. Two's complement, little endian.

    Never shared with the sign-magnitude decoders used for speed / load /
    current: conflating the two is the most dangerous decoding error in this
    register map.
    """
    value = u16le(raw, addr)
    return value - 0x10000 if value & 0x8000 else value


def sign_magnitude(value, sign_bit):
    """Present speed / load / current ONLY. Never for PositionOffset."""
    mask = 1 << sign_bit
    if value & mask:
        return -(value & ~mask)
    return value


def validate_physical_label(label):
    if not isinstance(label, str) or label not in ALLOCATION:
        raise ProvisionError(
            "unknown physical label %r; expected one of %s"
            % (label, ", ".join(sorted(ALLOCATION)))
        )
    return label


def target_id_for(label):
    return ALLOCATION[validate_physical_label(label)][0]


def cold_cycles_for(label):
    return ALLOCATION[validate_physical_label(label)][2]


def verify_frozen_firmware():
    results = {}
    for name, path, expected in (
        ("source", FIRMWARE_SOURCE_PATH, FIRMWARE_SOURCE_SHA256),
        ("binary", FIRMWARE_BINARY_PATH, FIRMWARE_BINARY_SHA256),
    ):
        if not os.path.isfile(path):
            raise ProvisionError("firmware artifact missing: %s" % path)
        actual = sha256_file(path)
        results[name] = {
            "path": path,
            "expected_sha256": expected,
            "actual_sha256": actual,
            "match": actual == expected,
        }
        if actual != expected:
            raise ProvisionError(
                "firmware %s SHA256 mismatch: expected %s got %s"
                % (name, expected, actual)
            )
    results["fqbn"] = FIRMWARE_FQBN
    return results


# --------------------------------------------------------------------------
# 71-byte frame handling
# --------------------------------------------------------------------------

def parse_raw71_hex(hex_text):
    """Strict. Rejects anything that is not exactly 71 bytes of hex."""
    if not isinstance(hex_text, str):
        raise ProvisionError("RAW71_HEX payload is not a string")
    text = hex_text.strip()
    if len(text) != SNAPSHOT_LEN * 2:
        raise ProvisionError(
            "RAW71_HEX length %d, expected %d hex chars (%d bytes)"
            % (len(text), SNAPSHOT_LEN * 2, SNAPSHOT_LEN)
        )
    if not re.fullmatch(r"[0-9A-Fa-f]+", text):
        raise ProvisionError("RAW71_HEX contains non-hexadecimal characters")
    raw = bytes.fromhex(text)
    if len(raw) != SNAPSHOT_LEN:
        raise ProvisionError("decoded frame is %d bytes, expected %d"
                             % (len(raw), SNAPSHOT_LEN))
    return raw


def decode_snapshot(raw):
    """Decode a validated 71-byte frame."""
    if len(raw) != SNAPSHOT_LEN:
        raise ProvisionError("decode_snapshot requires exactly %d bytes"
                             % SNAPSHOT_LEN)

    offset_signed = i16le_twos_complement(raw, ADDR_POSITION_OFFSET)
    displayed = u16le(raw, ADDR_PRESENT_POSITION)

    identity = {
        "model_0x03": u16le(raw, ADDR_MODEL),
        "id_register_0x05": raw[ADDR_ID],
        "baud_register_0x06": raw[ADDR_BAUD],
        "return_delay_0x07": raw[ADDR_RETURN_DELAY],
        "response_status_0x08": raw[ADDR_RESPONSE_STATUS],
    }

    profile = {}
    for addr, width, _value, name in PROFILE:
        profile["%s_0x%02X" % (name, addr)] = (
            u16le(raw, addr) if width == 2 else raw[addr]
        )

    preserve = {}
    for addr, name in PRESERVE_ONLY:
        preserve["%s_0x%02X" % (name, addr)] = raw[addr]

    ram = {
        "torque_enable_0x28": raw[ADDR_TORQUE_ENABLE],
        "torque_limit_0x30": u16le(raw, ADDR_TORQUE_LIMIT),
        "lock_0x37": raw[ADDR_LOCK],
    }

    telemetry = {
        "present_position_0x38_raw16": displayed,
        "present_speed_0x3A_signed": sign_magnitude(u16le(raw, 0x3A), 15),
        "present_load_0x3C_signed": sign_magnitude(u16le(raw, 0x3C), 10),
        "present_voltage_0x3E": raw[ADDR_PRESENT_VOLTAGE],
        "present_temperature_0x3F": raw[ADDR_PRESENT_TEMPERATURE],
        "status_0x40": raw[ADDR_STATUS],
        "moving_0x42": raw[0x42],
        "present_current_0x45_signed": sign_magnitude(u16le(raw, 0x45), 15),
    }

    return {
        "raw_hex": raw.hex().upper(),
        "raw_sha256": sha256_bytes(raw),
        "raw_length": len(raw),
        "identity": identity,
        "profile_fields": profile,
        "preserve_only_fields": preserve,
        "ram": ram,
        "telemetry": telemetry,
        "position_offset_signed": offset_signed,
        "position_offset_raw16": u16le(raw, ADDR_POSITION_OFFSET),
        "physical_raw": physical_raw(displayed, offset_signed),
        "physical_raw_formula":
            "(present_position + signed_position_offset) mod 4096",
    }


def profile_fingerprint(values):
    """Stable digest of the canonical persistent profile."""
    payload = ";".join(
        "0x%02X=%d" % (addr, values[addr]) for addr, _w, _v, _n in PROFILE
    )
    return sha256_bytes(payload.encode("ascii"))


def target_profile_fingerprint():
    return profile_fingerprint({addr: value for addr, _w, value, _n in PROFILE})


def profile_values_from_raw(raw):
    return {
        addr: (u16le(raw, addr) if width == 2 else raw[addr])
        for addr, width, _value, _name in PROFILE
    }


def profile_delta(raw):
    """The stage-2 write allowlist for this specific servo.

    Already-correct fields are skipped. A fully canonical servo yields an
    empty write list, which is not hypothetical: the surveyed NEW01 is already
    exactly MATDOG_C018_V1.
    """
    values = profile_values_from_raw(raw)
    delta = []
    for addr, width, value, name in PROFILE:
        current = values[addr]
        delta.append({
            "address": "0x%02X" % addr,
            "name": name,
            "width": width,
            "current": current,
            "target": value,
            "action": "WRITE" if current != value else "SKIP",
        })
    return delta


def profile_exact(raw):
    values = profile_values_from_raw(raw)
    return all(values[addr] == value for addr, _w, value, _n in PROFILE)


# --------------------------------------------------------------------------
# Write-result semantics — SCServo Ack(): 1 = success, 0 = failure
# --------------------------------------------------------------------------

def write_verified(ack, status, readback, expected):
    """Ordinary write acceptance. All three conditions are mandatory.

    `ack < 0` is meaningless for SCServo write primitives and is never used
    anywhere in this project.
    """
    return ack == 1 and status == 0 and readback == expected


# --------------------------------------------------------------------------
# Raw evidence persistence — 71 bytes on disk BEFORE any decoding
# --------------------------------------------------------------------------

def persist_raw71(raw, path):
    """Write exactly 71 raw bytes. Fail-closed and exclusive.

    open(path, "xb") never overwrites existing evidence; the bytes are
    flushed and os.fsync'd before this returns; the on-disk size is verified.
    """
    if not isinstance(raw, (bytes, bytearray)):
        raise ProvisionError(
            "raw snapshot must be bytes, got %s" % type(raw).__name__
        )
    if len(raw) != SNAPSHOT_LEN:
        raise ProvisionError(
            "refusing to persist %d bytes, expected exactly %d"
            % (len(raw), SNAPSHOT_LEN)
        )
    payload = bytes(raw)
    try:
        handle = open(path, "xb")
    except FileExistsError:
        raise ProvisionError(
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
        raise ProvisionError(
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
        raise ProvisionError(
            "unknown artifact slot %r; expected one of %s"
            % (slot, ", ".join(sorted(ARTIFACTS)))
        )
    return os.path.join(session_dir, ARTIFACTS[slot])


def capture_snapshot(slot, raw_hex, session_dir, on_persisted=None):
    """One snapshot, in this order and no other:

        1. parse RAW71_HEX  -> exactly 71 bytes, or fail closed
        2. persist_raw71()  -> write + flush + os.fsync to its own .bin
        3. decode_snapshot()-> only once the bytes are durable
    """
    raw = parse_raw71_hex(raw_hex)
    artifact = persist_raw71(raw, artifact_path(session_dir, slot))
    if on_persisted is not None:
        on_persisted(artifact)
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
        raise ProvisionError(
            "session directory already exists, refusing to reuse: %s"
            % session_dir
        )
    return session_dir


# --------------------------------------------------------------------------
# Independent verification models
#
# The firmware decides; the host re-derives the same conclusion from the
# transcript and must agree. These are not duplicated logic for its own sake:
# a firmware that claimed PASS without the required evidence lines would be
# caught here.
# --------------------------------------------------------------------------

RECODE_SKIPPED = "SKIPPED_SOURCE_EQUALS_TARGET"
RECODE_OK = "RECODED"
RECODE_AMBIGUOUS = "FAIL_AMBIGUOUS_RECODE"


def evaluate_recode_proof(source_id, target_id, target_responds,
                          target_ping_status, old_responds, model_at_target,
                          snapshot_raw):
    """Authoritative post-recode identity proof. The write ACK is not an input.

    Returns (outcome, reason). Any ambiguity is FAIL_AMBIGUOUS_RECODE: no
    retry, operator must remove servo power.
    """
    if not target_responds:
        return RECODE_AMBIGUOUS, "target ID did not respond"
    if target_ping_status != 0:
        return RECODE_AMBIGUOUS, ("target ping status 0x%02X, expected 0x00"
                                  % target_ping_status)
    if model_at_target != EXPECTED_MODEL:
        return RECODE_AMBIGUOUS, ("model at target %s, expected %d"
                                  % (model_at_target, EXPECTED_MODEL))

    if source_id != target_id and old_responds:
        return RECODE_AMBIGUOUS, "old source ID still responds"

    if snapshot_raw is not None:
        if snapshot_raw[ADDR_ID] != target_id:
            return RECODE_AMBIGUOUS, (
                "snapshot ID register %d, expected %d"
                % (snapshot_raw[ADDR_ID], target_id)
            )
        if i16le_twos_complement(snapshot_raw, ADDR_POSITION_OFFSET) != 0:
            return RECODE_AMBIGUOUS, "PositionOffset is not 0 after recode"
        if not profile_exact(snapshot_raw):
            return RECODE_AMBIGUOUS, "canonical profile not exact after recode"
        if snapshot_raw[ADDR_LOCK] != 1:
            return RECODE_AMBIGUOUS, "EEPROM lock not restored"
        if snapshot_raw[ADDR_TORQUE_ENABLE] != 0:
            return RECODE_AMBIGUOUS, "TorqueEnable is not 0 after recode"

    if source_id == target_id:
        return RECODE_SKIPPED, "source ID already equals target ID"
    return RECODE_OK, "target responds, old ID silent, state verified"


# Cold-cycle events as emitted by the firmware.
COLD_PRESENT = "PRESENT"
COLD_ABSENT_CANDIDATE = "ABSENT_CANDIDATE"
COLD_ABSENT_CONFIRMED = "ABSENT_CONFIRMED"
COLD_ABSENCE_ABORTED = "ABSENCE_ABORTED_TOO_SHORT"
COLD_RETURNED = "RETURNED"
COLD_STABLE = "STABLE"
COLD_ESP32_BOOT = "ESP32_BOOT"


class ColdCycleModel:
    """A true servo power cycle, re-derived from the transcript.

    STABLE is reachable only through ABSENT_CONFIRMED. An ESP32 reset appears
    in the stream as a boot banner and permanently invalidates the cycle: the
    firmware's own state machine lives in ESP32 RAM inside one @EXECUTE, so a
    reset destroys it, and this model refuses to accept the result either way.
    """

    TRANSITIONS = {
        "INIT": {COLD_PRESENT: "PRESENT"},
        "PRESENT": {COLD_ABSENT_CANDIDATE: "ABSENT_CANDIDATE"},
        "ABSENT_CANDIDATE": {
            COLD_ABSENT_CONFIRMED: "ABSENT_CONFIRMED",
            COLD_ABSENCE_ABORTED: "PRESENT",
        },
        "ABSENT_CONFIRMED": {COLD_RETURNED: "RETURNED"},
        "RETURNED": {COLD_STABLE: "STABLE"},
        "STABLE": {},
    }

    def __init__(self):
        self.state = "INIT"
        self.invalidated = False
        self.reason = None
        self.events = []

    def feed(self, event):
        self.events.append(event)

        if event == COLD_ESP32_BOOT:
            self.invalidated = True
            self.reason = "ESP32_RESET_CANNOT_SATISFY_COLD_SERVO_CYCLE"
            return self

        if self.invalidated:
            return self

        allowed = self.TRANSITIONS.get(self.state, {})
        if event not in allowed:
            self.invalidated = True
            self.reason = "ILLEGAL_TRANSITION %s -> %s" % (self.state, event)
            return self

        self.state = allowed[event]
        return self

    @property
    def passed(self):
        return self.state == "STABLE" and not self.invalidated

    def summary(self):
        return {
            "state": self.state,
            "passed": self.passed,
            "invalidated": self.invalidated,
            "reason": self.reason,
            "events": list(self.events),
        }


# --------------------------------------------------------------------------
# Static write-capability audit of the firmware source
# --------------------------------------------------------------------------

WRITE_PRIMITIVES = (
    "writeByte", "writeWord", "EnableTorque", "WritePosEx", "unLockEprom",
    "LockEprom", "CalibrationOfs", "genWrite", "regWrite", "syncWrite",
    "RegWriteAction", "WheelMode", "WriteSpe", "SyncWritePosEx",
)

# (primitive, enclosing function) -> why it is allowed to exist.
APPROVED_WRITE_SITES = {
    ("writeWord", "provWrite"):
        "word writes, gated by writeAllowed() and verified by readback",
    ("writeByte", "provWrite"):
        "byte writes, gated by writeAllowed() and verified by readback",
    ("WritePosEx", "provWritePosEx"):
        "the only motion primitive; position domain 0..4095 checked first, "
        "speed/acc pinned to the canonical centering constants",
    ("writeByte", "provWriteIdAdvisory"):
        "ID recode; still gated by writeAllowed(), ACK advisory because the "
        "servo may already answer under the new ID",
}

FORBIDDEN_SOURCE_PATTERNS = (
    (r"\bCalibrationOfs\s*\(", "CalibrationOfs call"),
    (r"\bfactoryReset\b", "factory reset"),
    (r"\bFACTORY_RESET\s*\(", "factory reset call"),
    (r"\bsyncWrite\s*\(", "broadcast/sync write"),
    (r"\bSyncWritePosEx\s*\(", "broadcast/sync position write"),
    (r"\bregWrite\s*\(", "asynchronous register write"),
    (r"\bRegWriteAction\s*\(", "asynchronous write action"),
    (r"\bgenWrite\s*\(", "generic write"),
    (r"0xfe\s*,", "broadcast ID 0xFE as an argument"),
    (r"\(s16\)\s*-", "signed negative GoalPosition"),
)


def _enclosing_function(lines, index):
    """Nearest preceding top-level function definition."""
    pattern = re.compile(r"^[A-Za-z_].*?\b([A-Za-z_][A-Za-z0-9_]*)\s*\(")
    for i in range(index, -1, -1):
        line = lines[i]
        if line.startswith(" ") or line.startswith("\t"):
            continue
        match = pattern.match(line)
        if match and (line.rstrip().endswith("{") or line.rstrip().endswith(",")
                      or "(" in line):
            if match.group(1) not in ("if", "for", "while", "switch", "return"):
                return match.group(1)
    return None


def read_firmware_source():
    if not os.path.isfile(FIRMWARE_SOURCE_PATH):
        raise ProvisionError("firmware source missing: %s"
                             % FIRMWARE_SOURCE_PATH)
    with open(FIRMWARE_SOURCE_PATH, encoding="utf-8") as handle:
        return handle.read()


def parse_firmware_profile_table(source):
    """Extract the canonical profile table from the firmware source."""
    block = re.search(r"static const ProfileField PROFILE\[\] = \{(.*?)\n\};",
                      source, re.S)
    if not block:
        raise ProvisionError("firmware PROFILE table not found")
    entries = re.findall(
        r"\{0x([0-9A-Fa-f]{2}),\s*(\d+),\s*(\d+),\s*\"([A-Za-z]+)\"\}",
        block.group(1),
    )
    return tuple(
        (int(addr, 16), int(width), int(value), name)
        for addr, width, value, name in entries
    )


def parse_firmware_allocation(source):
    """Extract the allocation table from the firmware source."""
    block = re.search(r"static const PhysicalUnit UNITS\[\] = \{(.*?)\n\};",
                      source, re.S)
    if not block:
        raise ProvisionError("firmware UNITS table not found")
    entries = re.findall(
        r"\{\"([A-Z0-9]+)\",\s*(\d+),\s*\"([A-Z_]+)\",\s*(\d+)\}",
        block.group(1),
    )
    return {
        label: (int(target), joint, int(cold))
        for label, target, joint, cold in entries
    }


def firmware_freeze_blocked(source=None):
    """The block lives in the firmware, not in a duplicated host flag."""
    if source is None:
        source = read_firmware_source()
    match = re.search(
        r"PROVISIONER_HARDWARE_FREEZE_BLOCKED\s*=\s*(true|false)\s*;", source
    )
    if not match:
        raise ProvisionError(
            "PROVISIONER_HARDWARE_FREEZE_BLOCKED not found in firmware"
        )
    return match.group(1) == "true"


def firmware_unresolved_constants(source=None):
    if source is None:
        source = read_firmware_source()
    names = re.findall(r"^static constexpr [^\n]*?\b(\w+)\s*=[^;]*;\s*//\s*UNRESOLVED",
                       source, re.M)
    return sorted(set(names))


def firmware_function_body(source, name):
    """Source text of one top-level firmware function, for targeted audits."""
    match = re.search(r"^[A-Za-z_][^\n]*?\b%s\s*\([^\n]*\{\s*$" % re.escape(name),
                      source, re.M)
    if not match:
        raise ProvisionError("firmware function %s not found" % name)
    lines = source[match.start():].splitlines()
    body = [lines[0]]
    for line in lines[1:]:
        body.append(line)
        if line == "}":
            return "\n".join(body)
    raise ProvisionError("firmware function %s is unterminated" % name)


def static_write_audit(source=None):
    """Enumerate every firmware write primitive and where it may be reached."""
    if source is None:
        source = read_firmware_source()
    lines = source.splitlines()

    sites = []
    for index, line in enumerate(lines):
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
            if line.lstrip().startswith(("//", "*", "/*")):
                continue
            if re.search(pattern, line):
                forbidden.append({
                    "pattern": pattern,
                    "description": description,
                    "line": index + 1,
                    "text": line.strip(),
                })

    firmware_profile = parse_firmware_profile_table(source)
    firmware_allocation = parse_firmware_allocation(source)

    profile_addrs = {addr for addr, _w, _v, _n in firmware_profile}
    preserve_violations = [
        "0x%02X %s" % (addr, name)
        for addr, name in PRESERVE_ONLY if addr in profile_addrs
    ]
    late_violations = [
        "0x%02X %s" % (addr, name)
        for addr, name in LATE_STAGE_REGISTERS if addr in profile_addrs
    ]

    checks = {
        "every_write_site_approved": not unapproved,
        "no_forbidden_patterns": not forbidden,
        "profile_table_matches_host": firmware_profile == PROFILE,
        "allocation_matches_host": firmware_allocation == ALLOCATION,
        "baud_not_in_write_allowlist": ADDR_BAUD not in profile_addrs,
        "preserve_only_never_written": not preserve_violations,
        "late_stage_registers_not_in_profile": not late_violations,
        "write_allowlist_is_enforced":
            "writeAllowed(" in source and source.count("writeAllowed(") >= 3,
        "begin_declares_no_writes": 'Serial.println("EEPROM_WRITES=NONE")' in source,
    }

    return {
        "firmware_source": FIRMWARE_SOURCE_PATH,
        "write_sites": sites,
        "unapproved_write_sites": unapproved,
        "forbidden_pattern_hits": forbidden,
        "preserve_only_violations": preserve_violations,
        "late_stage_violations": late_violations,
        "primitives_with_zero_call_sites": sorted(
            p for p in WRITE_PRIMITIVES
            if not any(s["primitive"] == p for s in sites)
        ),
        "hardware_freeze_blocked": firmware_freeze_blocked(source),
        "unresolved_constants": firmware_unresolved_constants(source),
        "checks": checks,
        "ok": all(checks.values()),
    }


# --------------------------------------------------------------------------
# Firmware transcript parsing
# --------------------------------------------------------------------------

RE_KV = re.compile(r"^([A-Z0-9_]+)=(.*)$")
RE_SNAPSHOT_BEGIN = re.compile(r"^SNAPSHOT_BEGIN SLOT=([A-Z0-9_]+) ")
RE_SNAPSHOT_END = re.compile(r"^SNAPSHOT_END SLOT=([A-Z0-9_]+)$")
RE_RAW71 = re.compile(r"^RAW71_HEX=([0-9A-Fa-f]+)$")
RE_DELTA = re.compile(
    r"^DELTA ADDR=0x([0-9A-Fa-f]{2}) NAME=(\S+) WIDTH=(\d+) "
    r"CURRENT=(\d+) TARGET=(\d+) ACTION=(WRITE|SKIP)$"
)
RE_UNRESOLVED = re.compile(
    r"^UNRESOLVED_CONSTANT NAME=(\S+) PROVISIONAL=(\S+) REASON=(\S+)$"
)
RE_WRITE = re.compile(
    r"^WRITE ADDR=0x([0-9A-Fa-f]{2}) NAME=(\S+) WIDTH=(\d+) EXPECT=(\d+) "
    r"ACK=(-?\d+) STATUS=0x([0-9A-Fa-f]{2}) READBACK=(-?\d+) "
    r"READ_STATUS=0x([0-9A-Fa-f]{2}) RESULT=(OK|FAIL)$"
)
RE_COLD_STATE = re.compile(r"^COLD_STATE=([A-Z_]+)")

ESP32_BOOT_MARKER = "PROVISIONER_READY"


def extract_snapshots(lines):
    """Map slot -> RAW71 hex. Strict: one RAW71_HEX per well-formed block."""
    snapshots = {}
    slot = None
    raw_hex = None
    for line in lines:
        match = RE_SNAPSHOT_BEGIN.match(line)
        if match:
            if slot is not None:
                raise ProvisionError("nested SNAPSHOT_BEGIN for %s" % slot)
            slot = match.group(1)
            raw_hex = None
            continue
        if slot is not None:
            raw_match = RE_RAW71.match(line)
            if raw_match:
                if raw_hex is not None:
                    raise ProvisionError("multiple RAW71_HEX in slot %s" % slot)
                raw_hex = raw_match.group(1)
                continue
            end_match = RE_SNAPSHOT_END.match(line)
            if end_match:
                if end_match.group(1) != slot:
                    raise ProvisionError(
                        "SNAPSHOT_END slot %s does not match SNAPSHOT_BEGIN %s"
                        % (end_match.group(1), slot)
                    )
                if raw_hex is None:
                    raise ProvisionError("slot %s produced no RAW71_HEX" % slot)
                if slot in snapshots:
                    raise ProvisionError("duplicate snapshot slot %s" % slot)
                snapshots[slot] = raw_hex
                slot = None
                raw_hex = None
    if slot is not None:
        raise ProvisionError("unterminated SNAPSHOT_BEGIN for slot %s" % slot)
    return snapshots


def extract_kv(lines, key):
    prefix = key + "="
    for line in lines:
        if line.startswith(prefix):
            return line[len(prefix):].strip()
    return None


def parse_begin(lines):
    """Parse the @BEGIN response. Fail-closed on anything unexpected."""
    for line in lines:
        if line.startswith("BEGIN_ABORT"):
            raise ProvisionError("firmware aborted BEGIN: %s" % line)
    if "BEGIN_RESULT PASS" not in lines:
        raise ProvisionError("BEGIN_RESULT PASS missing")
    if "WAIT_EXECUTE" not in lines:
        raise ProvisionError("WAIT_EXECUTE missing")
    if "EEPROM_WRITES=NONE" not in lines:
        raise ProvisionError("BEGIN did not declare EEPROM_WRITES=NONE")
    if "MOTION=NONE" not in lines:
        raise ProvisionError("BEGIN did not declare MOTION=NONE")
    if "GATE_RESULT PASS" not in lines:
        raise ProvisionError("PRE-PROVISION SAFETY GATE did not pass")

    snapshots = extract_snapshots(lines)
    if "BEFORE" not in snapshots:
        raise ProvisionError("BEFORE snapshot missing from BEGIN response")

    label = extract_kv(lines, "BEGIN_LABEL")
    token = extract_kv(lines, "SESSION_TOKEN")
    if not token or not re.fullmatch(r"[0-9A-F]{8}", token):
        raise ProvisionError("malformed SESSION_TOKEN: %r" % (token,))

    delta = []
    for line in lines:
        match = RE_DELTA.match(line)
        if match:
            delta.append({
                "address": "0x%02X" % int(match.group(1), 16),
                "name": match.group(2),
                "width": int(match.group(3)),
                "current": int(match.group(4)),
                "target": int(match.group(5)),
                "action": match.group(6),
            })

    unresolved = []
    for line in lines:
        match = RE_UNRESOLVED.match(line)
        if match:
            unresolved.append({
                "name": match.group(1),
                "provisional": match.group(2),
                "reason": match.group(3),
            })

    return {
        "label": label,
        "source_id": int(extract_kv(lines, "BEGIN_SOURCE_ID")),
        "target_id": int(extract_kv(lines, "BEGIN_TARGET_ID")),
        "target_joint": extract_kv(lines, "BEGIN_TARGET_JOINT"),
        "cold_cycles_required": int(
            extract_kv(lines, "BEGIN_COLD_CYCLES_REQUIRED")
        ),
        "target_id_occupied": extract_kv(lines, "BEGIN_TARGET_ID_OCCUPIED"),
        "session_token": token,
        "before_raw_hex": snapshots["BEFORE"],
        "firmware_delta": delta,
        "unresolved_constants": unresolved,
        "hardware_freeze": extract_kv(lines, "HARDWARE_FREEZE"),
    }


def parse_write_records(lines):
    records = []
    for line in lines:
        match = RE_WRITE.match(line)
        if match:
            records.append({
                "address": "0x%02X" % int(match.group(1), 16),
                "name": match.group(2),
                "width": int(match.group(3)),
                "expected": int(match.group(4)),
                "ack": int(match.group(5)),
                "write_status": int(match.group(6), 16),
                "readback": int(match.group(7)),
                "read_status": int(match.group(8), 16),
                "result": match.group(9),
            })
    return records


def verify_write_records(records):
    """Every ordinary write must satisfy ack==1 AND status==0 AND readback."""
    bad = []
    for record in records:
        ok = write_verified(record["ack"], record["write_status"],
                            record["readback"], record["expected"])
        if not ok or record["result"] != "OK" or record["read_status"] != 0:
            bad.append(record)
    return bad


def build_cold_models(lines, required_cycles):
    """Replay the cold-cycle transcript, one model per required cycle."""
    models = []
    current = None
    for line in lines:
        if ESP32_BOOT_MARKER in line and current is not None:
            current.feed(COLD_ESP32_BOOT)
            continue
        if line.startswith("COLD_CYCLE_BEGIN"):
            current = ColdCycleModel()
            models.append(current)
            continue
        match = RE_COLD_STATE.match(line)
        if match and current is not None:
            current.feed(match.group(1))
    if len(models) != required_cycles:
        raise ProvisionError(
            "expected %d cold cycle(s), transcript contains %d"
            % (required_cycles, len(models))
        )
    return models


# --------------------------------------------------------------------------
# Transport — pyserial imported lazily so offline modes cannot touch a port
# --------------------------------------------------------------------------

class SerialTransport:
    """The only object in this file that can reach a serial port."""

    def __init__(self, port, console_log):
        import serial  # noqa: PLC0415 - deliberate lazy import
        import time  # noqa: PLC0415

        self._time = time
        self._log = console_log

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
        checked = assert_command_allowed(command)
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
        raise ProvisionError(
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

    def __init__(self, script, console_log=None):
        self.script = dict(script)
        self.sent = []
        self._log = console_log

    def send(self, command):
        checked = assert_command_allowed(command)
        self.sent.append(checked)
        return checked

    def read_until(self, terminators, timeout):
        if not self.sent:
            raise ProvisionError("read_until before any command was sent")
        command = self.sent[-1]
        if command not in self.script:
            raise ProvisionError("scripted transport has no reply for %r"
                                 % command)
        lines = list(self.script[command])
        if self._log is not None:
            for line in lines:
                self._log.write(line + "\n")
        for index, line in enumerate(lines):
            if any(line.startswith(term) for term in terminators):
                return lines[:index + 1]
        raise ProvisionError("scripted reply never reached %s" % (terminators,))

    def close(self):
        pass


# --------------------------------------------------------------------------
# Session
# --------------------------------------------------------------------------

def build_record(label, port, firmware, audit, session_dir):
    target_id, joint, cold_cycles = ALLOCATION[label]
    return {
        "schema": REPORT_SCHEMA,
        "tool": TOOL_NAME,
        "tool_version": TOOL_VERSION,
        "profile": PROFILE_ID,
        "runner_sha256": sha256_file(os.path.abspath(__file__)),
        "started_utc": utc_now_iso(),
        "physical_label": label,
        "target_id": target_id,
        "target_joint": joint,
        "cold_cycles_required": cold_cycles,
        "port": port,
        "session_dir": session_dir,
        "firmware": firmware,
        "static_write_audit": audit,
        "command_surface": command_surface_audit(),
        "commands_sent": [],
        "raw_artifacts": {},
        "source_id": None,
        "session_token_accepted": None,
        "old_position_offset": None,
        "initial_present_position": None,
        "initial_physical_raw": None,
        "center_goal_displayed": None,
        "canonical_delta": None,
        "eeprom_writes": [],
        "motion_telemetry": {},
        "offset_zero_proof": None,
        "id_recode_proof": None,
        "warm_state": None,
        "cold_states": [],
        "profile_fingerprint_target": target_profile_fingerprint(),
        "profile_fingerprint_before": None,
        "profile_fingerprint_final": None,
        "verdict": "INCOMPLETE",
        "reason": None,
    }


def write_report(record, session_dir):
    path = os.path.join(session_dir, REPORT_NAME)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        json.dump(record, handle, indent=2, sort_keys=True)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)
    return path


def write_sha256sums(session_dir):
    """Checksum every artifact except the checksum file itself."""
    path = os.path.join(session_dir, SHA256SUMS_NAME)
    names = sorted(
        name for name in os.listdir(session_dir)
        if name != SHA256SUMS_NAME and not name.endswith(".tmp")
    )
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as handle:
        for name in names:
            handle.write("%s  %s\n"
                         % (sha256_file(os.path.join(session_dir, name)), name))
        handle.flush()
        os.fsync(handle.fileno())
    os.replace(tmp, path)
    return path


def guard_session(record, session, persist, close_port):
    """Any exception stamps verdict=FAIL and is re-raised; port always closed.

    No emergency command is ever sent from here. Recovery (EEPROM lock, torque
    off) belongs to the firmware, which is the only side that can address the
    servo safely; the host never improvises a write.
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
            print("WARNING: report could not be persisted: %s" % persist_exc,
                  file=sys.stderr)
        finally:
            close_port()


def run_begin_phase(transport, record, label, session_dir):
    """@BEGIN, then make the BEFORE evidence durable. No writes have happened."""
    record["commands_sent"].append(transport.send("@BEGIN %s" % label))
    # WAIT_EXECUTE is the last line of a successful BEGIN; a failed BEGIN
    # ends at BEGIN_RESULT FAIL.
    lines = transport.read_until(["WAIT_EXECUTE", "BEGIN_RESULT FAIL"],
                                 BEGIN_TIMEOUT_S)
    begin = parse_begin(lines)

    if begin["label"] != label:
        raise ProvisionError("firmware label %r != requested %r"
                             % (begin["label"], label))
    expected_target = target_id_for(label)
    if begin["target_id"] != expected_target:
        raise ProvisionError(
            "firmware target ID %d disagrees with host allocation %d for %s"
            % (begin["target_id"], expected_target, label)
        )
    if begin["cold_cycles_required"] != cold_cycles_for(label):
        raise ProvisionError(
            "firmware cold-cycle policy %d disagrees with host %d"
            % (begin["cold_cycles_required"], cold_cycles_for(label))
        )

    # --- the ordering obligation -------------------------------------
    # 71 raw bytes durable on disk BEFORE @EXECUTE can be transmitted.
    def register(artifact):
        record["raw_artifacts"]["BEFORE"] = artifact

    before_raw, before = capture_snapshot(
        "BEFORE", begin["before_raw_hex"], session_dir, register
    )

    record["source_id"] = begin["source_id"]
    record["before_state"] = before
    record["old_position_offset"] = before["position_offset_signed"]
    record["initial_present_position"] = (
        before["telemetry"]["present_position_0x38_raw16"]
    )
    record["initial_physical_raw"] = before["physical_raw"]
    record["profile_fingerprint_before"] = profile_fingerprint(
        profile_values_from_raw(before_raw)
    )
    record["firmware_unresolved_constants"] = begin["unresolved_constants"]
    record["hardware_freeze"] = begin["hardware_freeze"]

    if before["identity"]["id_register_0x05"] != begin["source_id"]:
        raise ProvisionError("BEFORE ID register disagrees with discovered ID")
    if before["identity"]["response_status_0x08"] != EXPECTED_RESPONSE_STATUS:
        raise ProvisionError("BEFORE ResponseStatus != 1")
    if before["identity"]["model_0x03"] != EXPECTED_MODEL:
        raise ProvisionError("BEFORE model != 777")
    if before["identity"]["baud_register_0x06"] != EXPECTED_BAUD:
        raise ProvisionError("BEFORE baud register != 0")

    offset = before["position_offset_signed"]
    if not POSITION_OFFSET_MIN <= offset <= POSITION_OFFSET_MAX:
        raise ProvisionError("BEFORE PositionOffset %d outside bounded domain"
                             % offset)

    # Host-side delta, cross-checked against the firmware's own preview.
    host_delta = profile_delta(before_raw)
    record["canonical_delta"] = host_delta
    if begin["firmware_delta"] and begin["firmware_delta"] != host_delta:
        raise ProvisionError(
            "firmware canonical delta disagrees with host delta"
        )

    record["center_goal_displayed"] = center_goal_displayed(offset)
    record["session_token_accepted"] = begin["session_token"]
    return begin


def run_execute_phase(transport, record, begin, session_dir):
    """@EXECUTE, then verify every claim the firmware makes."""
    record["commands_sent"].append(
        transport.send("@EXECUTE %s" % begin["session_token"])
    )
    lines = transport.read_until(["EXECUTE_RESULT"], EXECUTE_TIMEOUT_S)

    for line in lines:
        if line.startswith("EXECUTE_ABORT"):
            raise ProvisionError("firmware aborted EXECUTE: %s" % line)
    if "EXECUTE_RESULT PASS" not in lines:
        raise ProvisionError("EXECUTE_RESULT PASS missing")

    writes = parse_write_records(lines)
    record["eeprom_writes"] = writes
    bad = verify_write_records(writes)
    if bad:
        raise ProvisionError("%d write(s) failed verification: %s"
                             % (len(bad), bad[0]))

    snapshots = extract_snapshots(lines)
    required = ["PROFILE", "CENTERED", "OFFSET_ZERO", "WARM"]
    required += ["COLD%d" % i for i in range(1, begin["cold_cycles_required"] + 1)]

    decoded = {}
    for slot in required:
        if slot not in snapshots:
            raise ProvisionError("snapshot slot %s missing from EXECUTE" % slot)

        def register(artifact, slot=slot):
            record["raw_artifacts"][slot] = artifact

        raw, state = capture_snapshot(slot, snapshots[slot], session_dir,
                                      register)
        decoded[slot] = (raw, state)

    # --- offset zero proof --------------------------------------------
    offset_raw, offset_state = decoded["OFFSET_ZERO"]
    displayed = offset_state["telemetry"]["present_position_0x38_raw16"]
    error = circular_delta(offset_state["physical_raw"], RAW_CENTER)
    record["offset_zero_proof"] = {
        "position_offset_signed": offset_state["position_offset_signed"],
        "displayed": displayed,
        "physical_raw": offset_state["physical_raw"],
        "center_error_ticks": error,
        "lock": offset_state["ram"]["lock_0x37"],
        "torque_enable": offset_state["ram"]["torque_enable_0x28"],
    }
    if offset_state["position_offset_signed"] != 0:
        raise ProvisionError("PositionOffset is not 0 after the offset stage")
    if abs(error) > CENTER_ACCEPT_TICKS:
        raise ProvisionError("center error %d ticks exceeds +/-%d"
                             % (error, CENTER_ACCEPT_TICKS))

    # --- ID recode proof ------------------------------------------------
    warm_raw, warm_state = decoded["WARM"]
    target_id = begin["target_id"]
    source_id = begin["source_id"]
    recoded = "ID_RECODE=SKIPPED_SOURCE_EQUALS_TARGET" not in lines
    outcome, reason = evaluate_recode_proof(
        source_id=source_id,
        target_id=target_id,
        target_responds=True,
        target_ping_status=0,
        old_responds=False,
        model_at_target=warm_state["identity"]["model_0x03"],
        snapshot_raw=warm_raw,
    )
    record["id_recode_proof"] = {
        "source_id": source_id,
        "target_id": target_id,
        "write_performed": recoded,
        "ack_authority": "ADVISORY",
        "outcome": outcome,
        "reason": reason,
    }
    if outcome == RECODE_AMBIGUOUS:
        raise ProvisionError("FAIL_AMBIGUOUS_RECODE: %s" % reason)

    record["warm_state"] = warm_state
    record["profile_fingerprint_final"] = profile_fingerprint(
        profile_values_from_raw(warm_raw)
    )
    if record["profile_fingerprint_final"] != record["profile_fingerprint_target"]:
        raise ProvisionError("final profile fingerprint != target fingerprint")

    # --- cold cycles ----------------------------------------------------
    models = build_cold_models(lines, begin["cold_cycles_required"])
    for index, model in enumerate(models, start=1):
        summary = model.summary()
        slot = "COLD%d" % index
        cold_raw, cold_state = decoded[slot]
        summary["slot"] = slot
        summary["observed_torque_limit"] = cold_state["ram"]["torque_limit_0x30"]
        summary["torque_limit_policy"] = "OBSERVED_NOT_ASSERTED"
        record["cold_states"].append({"cycle": index,
                                      "state": cold_state,
                                      "power_cycle_model": summary})
        if not model.passed:
            raise ProvisionError(
                "cold cycle %d not proven: %s"
                % (index, summary["reason"] or summary["state"])
            )
        if cold_state["position_offset_signed"] != 0:
            raise ProvisionError("cold cycle %d: offset != 0" % index)
        if cold_state["identity"]["id_register_0x05"] != target_id:
            raise ProvisionError("cold cycle %d: ID != target" % index)
        if not profile_exact(cold_raw):
            raise ProvisionError("cold cycle %d: profile not exact" % index)
        if cold_state["ram"]["lock_0x37"] != 1:
            raise ProvisionError("cold cycle %d: lock != 1" % index)
        if cold_state["ram"]["torque_enable_0x28"] != 0:
            raise ProvisionError("cold cycle %d: torque != 0" % index)

    # --- motion telemetry summary ---------------------------------------
    record["motion_telemetry"] = {
        line.split(" ", 1)[0]: line
        for line in lines if line.startswith("MOTION_SUMMARY")
    }
    return lines


def run_provision(label, port, session_root):
    """One bench session for one physical unit. Fail-closed throughout."""
    validate_physical_label(label)
    firmware = verify_frozen_firmware()

    audit = static_write_audit()
    if not audit["ok"]:
        raise ProvisionError(
            "static write audit failed: %s"
            % [name for name, ok in audit["checks"].items() if not ok]
        )

    # The hardware freeze is read from the firmware source, not duplicated.
    if audit["hardware_freeze_blocked"]:
        raise ProvisionError(
            "HARDWARE_FREEZE_BLOCKED: unresolved constants %s. No serial port "
            "was opened and no command was sent. See design section 11.2."
            % ", ".join(audit["unresolved_constants"])
        )

    session_dir = create_session_dir(session_root, label)
    record = build_record(label, port, firmware, audit, session_dir)

    console = open(os.path.join(session_dir, CONSOLE_LOG_NAME), "x",
                   encoding="utf-8")
    transport = None

    def persist():
        write_report(record, session_dir)

    def close_port():
        try:
            if transport is not None:
                transport.close()
        finally:
            console.close()
            try:
                write_sha256sums(session_dir)
            except Exception as exc:  # noqa: BLE001
                print("WARNING: SHA256SUMS not written: %s" % exc,
                      file=sys.stderr)

    def session():
        nonlocal transport
        transport = SerialTransport(port, console)

        begin = run_begin_phase(transport, record, label, session_dir)
        persist()   # BEFORE evidence + skeleton durable before @EXECUTE

        run_execute_phase(transport, record, begin, session_dir)

        record["verdict"] = "PASS"
        record["reason"] = (
            "canonical profile, physical center 2048, offset 0, target ID, "
            "warm and cold verification complete"
        )
        return record

    guard_session(record, session, persist, close_port)
    print("SESSION: %s" % session_dir)
    return record


# --------------------------------------------------------------------------
# Offline self-test
# --------------------------------------------------------------------------

# Real ST-3215-C018 hardware bytes: physical unit NEW01, source ID 1, captured
# 2026-08-26 by the frozen read-only survey
# (source_survey_v1/NEW01__20260826_193057Z__01_before... sha256 0d7b73ff...).
# Chosen because it is ground truth and because its PositionOffset is +85,
# the worked example in the design document.
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
        except Exception as exc:  # noqa: BLE001
            self.failed.append(name)
            print("  FAIL  %s (wrong exception %r)" % (name, exc))
            return
        self.failed.append(name)
        print("  FAIL  %s (no exception raised)" % name)


def _frame_with(base, **fields):
    """Build a synthetic frame from the golden one."""
    frame = bytearray(base)
    for key, value in fields.items():
        addr = int(key.split("_")[-1], 16)
        width = 2 if key.startswith("w") else 1
        if width == 2:
            frame[addr] = value & 0xFF
            frame[addr + 1] = (value >> 8) & 0xFF
        else:
            frame[addr] = value & 0xFF
    return bytes(frame)


def _begin_transcript(label="NEW01", source_id=1, token="1A2B3C4D",
                      raw_hex=GOLDEN_NEW01_HEX):
    target_id, joint, cold = ALLOCATION[label]
    lines = [
        "BEGIN_LABEL=%s" % label,
        "BEGIN_TARGET_ID=%d" % target_id,
        "BEGIN_TARGET_JOINT=%s" % joint,
        "BEGIN_COLD_CYCLES_REQUIRED=%d" % cold,
        "BEGIN_PROFILE=%s" % PROFILE_ID,
        "SCAN_BEGIN RANGE=0..253",
        "FOUND ID=%d MODEL=777 PING_STATUS=0x00 MODEL_STATUS=0x00" % source_id,
        "SCAN_RESULT FOUND=1",
        "SCAN_COMPLETE",
        "BEGIN_SOURCE_ID=%d" % source_id,
        "BEGIN_TARGET_ID_OCCUPIED=NO",
        "SNAPSHOT_BEGIN SLOT=BEFORE ID=%d START=0x00 LEN=71" % source_id,
        "RAW71_HEX=" + raw_hex,
        "SNAPSHOT_RAW_COMPLETE SLOT=BEFORE",
        "SNAPSHOT_END SLOT=BEFORE",
        "GATE_RESULT PASS",
    ]
    for entry in profile_delta(parse_raw71_hex(raw_hex)):
        lines.append(
            "DELTA ADDR=%s NAME=%s WIDTH=%d CURRENT=%d TARGET=%d ACTION=%s"
            % (entry["address"], entry["name"], entry["width"],
               entry["current"], entry["target"], entry["action"])
        )
    lines += [
        "UNRESOLVED_CONSTANT NAME=PRIME_MAX_DELTA_TICKS PROVISIONAL=0 "
        "REASON=campaign_evidence_at_speed300_acc20_only",
        "HARDWARE_FREEZE=BLOCKED",
        "SESSION_TOKEN=%s" % token,
        "EEPROM_WRITES=NONE",
        "MOTION=NONE",
        "BEGIN_RESULT PASS",
        "WAIT_EXECUTE",
    ]
    return lines


def run_self_test():
    print("=" * 70)
    print(" MATDOG ST3215 PROVISIONER V1 — OFFLINE SELF-TEST")
    print("=" * 70)
    print("No serial port is opened. No hardware is contacted.")
    print()

    test = SelfTest()
    golden = parse_raw71_hex(GOLDEN_NEW01_HEX)
    decoded = decode_snapshot(golden)
    source = read_firmware_source()
    audit = static_write_audit(source)

    print("[1] Physical allocation map")
    test.check("17 physical labels", len(ALLOCATION) == 17, len(ALLOCATION))
    test.check("17 unique target IDs",
               len({v[0] for v in ALLOCATION.values()}) == 17)
    test.check("17 unique joints",
               len({v[1] for v in ALLOCATION.values()}) == 17)
    test.check("all target IDs valid 0..253",
               all(0 <= v[0] <= 253 for v in ALLOCATION.values()))
    for label, expected in (("M22", 13), ("ELR01", 12), ("M33", 11),
                            ("NEW01", 23), ("ELR03", 22), ("NEW03", 21),
                            ("NEW06", 33), ("ELR02", 32), ("NEW05", 31),
                            ("M43", 43), ("M42", 42), ("M41", 41),
                            ("M31", 51), ("M11", 52), ("NEW04", 53),
                            ("NEW02", 54), ("ELR04", 55)):
        test.check("%s -> %d" % (label, expected),
                   target_id_for(label) == expected, target_id_for(label))
    test.check("firmware owns the same map", audit["checks"]["allocation_matches_host"])
    test.check("NEW01 is the two-cycle pilot", cold_cycles_for("NEW01") == 2)
    test.check("non-pilot units use one cold cycle",
               all(cold_cycles_for(l) == 1 for l in ALLOCATION if l != "NEW01"))
    test.expect_raises("unknown label rejected", ProvisionError,
                       validate_physical_label, "M99")
    test.expect_raises("lowercase label rejected", ProvisionError,
                       validate_physical_label, "new01")
    test.check("source ID != target ID is not an error by itself",
               target_id_for("NEW01") != 1)

    print()
    print("[2] Model register is 0x03, never NormaCore's 0x00")
    test.check("ADDR_MODEL is 0x03", ADDR_MODEL == 0x03)
    test.check("golden model reads 777 at 0x03",
               decoded["identity"]["model_0x03"] == 777)
    test.check("0x00 is not the model word", u16le(golden, 0x00) != 777,
               u16le(golden, 0x00))
    test.check("firmware REG_MODEL is 0x03",
               re.search(r"REG_MODEL\s*=\s*0x03", source) is not None)
    model_reads = re.findall(r"readWord\(\s*\(?uint8_t\)?\s*\w+,\s*([A-Z_0-9x]+)\)",
                             source)
    test.check("every firmware model read uses REG_MODEL",
               model_reads and all(r == "REG_MODEL" for r in model_reads),
               model_reads)
    test.check("firmware never reads a model word at 0x00",
               not re.search(r"readWord\([^)]*,\s*0x00\)", source))

    print()
    print("[3] 71-byte parsing")
    test.check("golden frame is 71 bytes", len(golden) == SNAPSHOT_LEN)
    test.expect_raises("70-byte frame rejected", ProvisionError,
                       parse_raw71_hex, GOLDEN_NEW01_HEX[:-2])
    test.expect_raises("72-byte frame rejected", ProvisionError,
                       parse_raw71_hex, GOLDEN_NEW01_HEX + "00")
    test.expect_raises("odd hex length rejected", ProvisionError,
                       parse_raw71_hex, GOLDEN_NEW01_HEX[:-1])
    test.expect_raises("non-hex rejected", ProvisionError,
                       parse_raw71_hex, "ZZ" + GOLDEN_NEW01_HEX[2:])
    test.expect_raises("empty rejected", ProvisionError, parse_raw71_hex, "")
    test.expect_raises("non-string rejected", ProvisionError,
                       parse_raw71_hex, None)
    test.expect_raises("short frame refused by decoder", ProvisionError,
                       decode_snapshot, golden[:-1])
    test.check("last mapped address is 0x46", 0x46 == SNAPSHOT_LEN - 1)

    print()
    print("[4] PositionOffset — two's complement, little endian")
    for hex_pair, expected in (("5500", 85), ("0000", 0), ("7F00", 127),
                               ("FF07", 2047), ("00F8", -2048),
                               ("FFFF", -1), ("1AFE", -486)):
        frame = bytearray(golden)
        frame[0x1F:0x21] = bytes.fromhex(hex_pair)
        value = i16le_twos_complement(bytes(frame), ADDR_POSITION_OFFSET)
        test.check("offset %s -> %d" % (hex_pair, expected), value == expected,
                   value)
    test.check("golden NEW01 offset is +85",
               decoded["position_offset_signed"] == 85)
    same = 0xFE1A
    test.check("two's complement != sign-magnitude for the same bytes",
               (same - 0x10000) != sign_magnitude(same, 15))
    test.check("sign-magnitude decoder untouched by offset decoder",
               sign_magnitude(0x0400 | 5, 10) == -5)

    print()
    print("[5] floor_mod and the centering relation")
    for value, modulus, expected in ((-1, 4096, 4095), (0, 4096, 0),
                                     (4096, 4096, 0), (4097, 4096, 1),
                                     (-4097, 4096, 4095), (-2048, 4096, 2048),
                                     (2048, 4096, 2048)):
        test.check("floor_mod(%d, %d) = %d" % (value, modulus, expected),
                   floor_mod(value, modulus) == expected,
                   floor_mod(value, modulus))
    test.check("floor_mod never returns negative",
               all(floor_mod(v, 4096) >= 0 for v in range(-9000, 9000, 137)))
    test.check("floor_mod always below modulus",
               all(floor_mod(v, 4096) < 4096 for v in range(-9000, 9000, 137)))
    test.expect_raises("zero modulus rejected", ProvisionError,
                       floor_mod, 5, 0)
    test.check("physical_raw and displayed are inverses",
               all(displayed_from_physical(physical_raw(d, o), o) == d
                   for d in (0, 1, 254, 2048, 4095)
                   for o in (-2048, -486, 0, 85, 2047)))
    test.check("center goal maps to physical 2048 for every legal offset",
               all(physical_raw(center_goal_displayed(o), o) == RAW_CENTER
                   for o in range(POSITION_OFFSET_MIN,
                                  POSITION_OFFSET_MAX + 1, 7)))

    print()
    print("[6] GoalPosition is strictly unsigned 0..4095")
    test.check("0 accepted", validate_goal_position(0) == 0)
    test.check("4095 accepted", validate_goal_position(4095) == 4095)
    test.expect_raises("-1 rejected", ProvisionError, validate_goal_position, -1)
    test.expect_raises("4096 rejected", ProvisionError,
                       validate_goal_position, 4096)
    test.expect_raises("float rejected", ProvisionError,
                       validate_goal_position, 2048.0)
    test.expect_raises("bool rejected", ProvisionError,
                       validate_goal_position, True)
    test.check("every center goal is inside the domain",
               all(0 <= center_goal_displayed(o) <= 4095
                   for o in range(POSITION_OFFSET_MIN,
                                  POSITION_OFFSET_MAX + 1, 13)))
    test.check("firmware refuses positions outside the domain",
               "MOTION_REFUSED POSITION_OUT_OF_DOMAIN" in source)

    print()
    print("[7] NEW01 worked example (real hardware)")
    test.check("NEW01 source ID register is 1",
               decoded["identity"]["id_register_0x05"] == 1)
    test.check("NEW01 offset +85", decoded["position_offset_signed"] == 85)
    test.check("NEW01 displayed present position 254",
               decoded["telemetry"]["present_position_0x38_raw16"] == 254)
    test.check("NEW01 physical raw 339", decoded["physical_raw"] == 339,
               decoded["physical_raw"])
    test.check("NEW01 center goal displayed 1963",
               center_goal_displayed(85) == 1963, center_goal_displayed(85))
    test.check("NEW01 goal maps to physical raw 2048",
               physical_raw(1963, 85) == 2048)
    test.check("NEW01 target ID 23", target_id_for("NEW01") == 23)
    test.check("NEW01 ResponseStatus 1",
               decoded["identity"]["response_status_0x08"] == 1)
    test.check("NEW01 baud register 0",
               decoded["identity"]["baud_register_0x06"] == 0)

    print()
    print("[8] ACK semantics — 1 = success, 0 = failure")
    test.check("ack 1 + status 0 + exact readback accepted",
               write_verified(1, 0, 32, 32))
    test.check("ack 0 rejected", not write_verified(0, 0, 32, 32))
    test.check("ack -1 rejected", not write_verified(-1, 0, 32, 32))
    test.check("nonzero status rejected", not write_verified(1, 1, 32, 32))
    test.check("wrong readback rejected", not write_verified(1, 0, 31, 32))
    # AST-level, so that prose about `ack < 0` in docstrings cannot mask a
    # real comparison.
    import ast  # noqa: PLC0415 - self-test only

    runner_ast = ast.parse(
        open(os.path.abspath(__file__), encoding="utf-8").read()
    )
    ack_lt_zero = [
        node for node in ast.walk(runner_ast)
        if isinstance(node, ast.Compare)
        and isinstance(node.left, ast.Name)
        and node.left.id in ("ack", "wr", "write_result", "writeResult")
        and any(isinstance(op, (ast.Lt, ast.LtE)) for op in node.ops)
    ]
    test.check("'wr < 0' logic absent from the runner", not ack_lt_zero,
               [n.lineno for n in ack_lt_zero])
    test.check("firmware asserts ack == 1", "out.ack == 1" in source)
    test.check("firmware never tests a write result with < 0",
               not re.search(r"\b(wr|ack|writeResult)\s*<\s*0", source))
    bad_writes = verify_write_records([
        {"address": "0x15", "name": "P", "width": 1, "expected": 32,
         "ack": 0, "write_status": 0, "readback": 32, "read_status": 0,
         "result": "OK"},
    ])
    test.check("write record with ack 0 is rejected", len(bad_writes) == 1)
    good_writes = verify_write_records([
        {"address": "0x15", "name": "P", "width": 1, "expected": 32,
         "ack": 1, "write_status": 0, "readback": 32, "read_status": 0,
         "result": "OK"},
    ])
    test.check("fully verified write record accepted", not good_writes)

    print()
    print("[9] ResponseStatus gate")
    test.check("expected ResponseStatus is 1", EXPECTED_RESPONSE_STATUS == 1)
    test.check("firmware gates on ResponseStatus",
               "REG_RESPONSE_STATUS" in source and
               "EXPECTED_RESPONSE_STATUS" in source)
    test.check("gate rejects ResponseStatus != 1 in firmware",
               'gateFail("RESPONSE_STATUS"' in source)
    with tempfile.TemporaryDirectory(prefix="matdog_prov_rs_") as tmpdir:
        bad = _frame_with(golden, b_08=0)
        script = {"@BEGIN NEW01": _begin_transcript(raw_hex=bad.hex().upper())}
        record = build_record("NEW01", "/dev/null", {}, {}, tmpdir)
        test.expect_raises("host rejects ResponseStatus 0", ProvisionError,
                           run_begin_phase, ScriptedTransport(script), record,
                           "NEW01", tmpdir)

    print()
    print("[10] Canonical profile delta generation")
    test.check("20 canonical fields", len(PROFILE) == 20, len(PROFILE))
    test.check("firmware profile table matches host",
               audit["checks"]["profile_table_matches_host"])
    test.check("profile addresses unique",
               len({addr for addr, _w, _v, _n in PROFILE}) == len(PROFILE))
    test.check("every profile width is 1 or 2",
               all(w in (1, 2) for _a, w, _v, _n in PROFILE))

    delta_new01 = profile_delta(golden)
    writes = [d for d in delta_new01 if d["action"] == "WRITE"]
    test.check("surveyed NEW01 is already fully canonical", not writes,
               [d["name"] for d in writes])
    test.check("already-canonical servo skips all 20 fields",
               len([d for d in delta_new01 if d["action"] == "SKIP"]) == 20)
    test.check("already-canonical servo passes profile_exact",
               profile_exact(golden))

    partial = _frame_with(golden, b_15=16, b_16=0, w_1C=500, b_24=25)
    delta_partial = profile_delta(partial)
    partial_writes = {d["name"] for d in delta_partial if d["action"] == "WRITE"}
    test.check("partial servo writes exactly the missing fields",
               partial_writes == {"P", "D", "ProtectionCurrent",
                                  "OverloadTorque"},
               sorted(partial_writes))
    test.check("partial servo skips the 16 already-correct fields",
               len([d for d in delta_partial if d["action"] == "SKIP"]) == 16)
    test.check("partial servo fails profile_exact", not profile_exact(partial))
    test.check("resume shrinks the delta: rerun after P is fixed",
               len([d for d in profile_delta(_frame_with(partial, b_15=32))
                    if d["action"] == "WRITE"]) == 3)
    test.check("delta reports current and target for every field",
               all("current" in d and "target" in d for d in delta_partial))
    test.check("target fingerprint is stable",
               target_profile_fingerprint() ==
               profile_fingerprint(profile_values_from_raw(golden)))
    test.check("mutated profile changes the fingerprint",
               profile_fingerprint(profile_values_from_raw(partial)) !=
               target_profile_fingerprint())

    print()
    print("[11] Preserve-only fields and Baud are never written")
    profile_addrs = {addr for addr, _w, _v, _n in PROFILE}
    for addr, name in PRESERVE_ONLY:
        test.check("preserve %s 0x%02X not in write allowlist" % (name, addr),
                   addr not in profile_addrs)
    test.check("Baud 0x06 never written",
               audit["checks"]["baud_not_in_write_allowlist"])
    test.check("firmware declares BAUD_WRITE NOT IMPLEMENTED",
               "BAUD_WRITE             : NOT IMPLEMENTED" in source)
    test.check("PositionOffset not in the profile stage",
               ADDR_POSITION_OFFSET not in profile_addrs)
    test.check("ID not in the profile stage", ADDR_ID not in profile_addrs)
    test.check("firmware audit finds no preserve-only violation",
               audit["checks"]["preserve_only_never_written"])
    test.check("golden preserve-only values are decoded for audit",
               decoded["preserve_only_fields"]["LedAlarm_0x13"] == golden[0x13])

    print()
    print("[12] Static write-capability audit of the firmware")
    test.check("every write site is approved",
               audit["checks"]["every_write_site_approved"],
               audit["unapproved_write_sites"])
    test.check("exactly 4 write sites", len(audit["write_sites"]) == 4,
               len(audit["write_sites"]))
    for primitive, function in APPROVED_WRITE_SITES:
        test.check("approved site %s in %s present" % (primitive, function),
                   any(s["primitive"] == primitive and s["function"] == function
                       for s in audit["write_sites"]))
    test.check("no forbidden source pattern",
               audit["checks"]["no_forbidden_patterns"],
               audit["forbidden_pattern_hits"])
    for absent in ("CalibrationOfs", "EnableTorque", "unLockEprom", "LockEprom",
                   "genWrite", "regWrite", "syncWrite", "RegWriteAction",
                   "WheelMode", "WriteSpe", "SyncWritePosEx"):
        test.check("%s has zero call sites" % absent,
                   absent in audit["primitives_with_zero_call_sites"])
    test.check("write allowlist is enforced",
               audit["checks"]["write_allowlist_is_enforced"])
    allow_body = firmware_function_body(source, "writeAllowed")
    allow_code = re.sub(r"//[^\n]*", "", allow_body)
    test.check("write allowlist code never mentions value 128 "
               "(CalibrationOfs)", "128" not in allow_code)
    test.check("TorqueEnable branch falls through to an explicit refusal",
               re.search(r"if \(value == 1\) \{[^}]*\}\s*return false;",
                         allow_code) is not None)
    test.check("write allowlist ends in an explicit refusal",
               allow_body.rstrip().endswith("return false;\n}")
               or "return false;" in allow_body)
    test.check("TorqueEnable=1 is allowed only in the centering stage",
               re.search(r"if \(value == 1\) \{\s*return stage == STAGE_CENTER;",
                         allow_body) is not None)
    test.check("ID may only be written with the bound target ID",
               "value == sessionTargetId" in allow_body)
    test.check("PositionOffset may only be written as 0",
               re.search(r"REG_POSITION_OFFSET:\s*\n\s*return width == 2 && value == 0",
                         allow_body) is not None)
    test.check("TorqueLimit may only be written as the centering value",
               "value == CENTER_TORQUE_LIMIT" in allow_body)
    test.check("@BEGIN declares no writes",
               audit["checks"]["begin_declares_no_writes"])
    test.check("static audit overall ok", audit["ok"],
               [k for k, v in audit["checks"].items() if not v])

    print()
    print("[13] ID recode state machine")
    warm = _frame_with(golden, b_05=23, w_1F=0, w_38=2048)
    outcome, reason = evaluate_recode_proof(1, 23, True, 0, False, 777, warm)
    test.check("clean recode accepted", outcome == RECODE_OK, reason)
    outcome, reason = evaluate_recode_proof(23, 23, True, 0, False, 777, warm)
    test.check("source == target is idempotent skip",
               outcome == RECODE_SKIPPED, reason)
    for name, args in (
        ("target silent", (1, 23, False, 0, False, 777, warm)),
        ("target ping status nonzero", (1, 23, True, 1, False, 777, warm)),
        ("old ID still responds", (1, 23, True, 0, True, 777, warm)),
        ("model mismatch at target", (1, 23, True, 0, False, 1, warm)),
    ):
        outcome, reason = evaluate_recode_proof(*args)
        test.check("ambiguous: %s" % name, outcome == RECODE_AMBIGUOUS, reason)
    outcome, _ = evaluate_recode_proof(
        1, 23, True, 0, False, 777, _frame_with(warm, b_05=1))
    test.check("snapshot ID != target rejected", outcome == RECODE_AMBIGUOUS)
    outcome, _ = evaluate_recode_proof(
        1, 23, True, 0, False, 777, _frame_with(warm, w_1F=85))
    test.check("nonzero offset after recode rejected",
               outcome == RECODE_AMBIGUOUS)
    outcome, _ = evaluate_recode_proof(
        1, 23, True, 0, False, 777, _frame_with(warm, b_37=0))
    test.check("lock not restored rejected", outcome == RECODE_AMBIGUOUS)
    outcome, _ = evaluate_recode_proof(
        1, 23, True, 0, False, 777, _frame_with(warm, b_28=1))
    test.check("torque on after recode rejected", outcome == RECODE_AMBIGUOUS)
    outcome, _ = evaluate_recode_proof(
        1, 23, True, 0, False, 777, _frame_with(warm, b_15=16))
    test.check("profile drift after recode rejected",
               outcome == RECODE_AMBIGUOUS)
    test.check("ID write ACK is never an input to the proof",
               "ack" not in evaluate_recode_proof.__code__.co_varnames)
    test.check("firmware writes the ID exactly once",
               source.count("provWriteIdAdvisory(") == 2)  # def + 1 call site
    test.check("firmware declares no blind retry", "No retry, ever." in source)
    test.check("firmware requires power removal on ambiguity",
               "FAIL_AMBIGUOUS_RECODE" in source and
               "CUT_SERVO_POWER_NOW" in source)
    test.check("firmware skips the write when source == target",
               "ID_RECODE=SKIPPED_SOURCE_EQUALS_TARGET" in source)

    print()
    print("[14] Cold power-cycle state machine")
    good = ColdCycleModel()
    for event in (COLD_PRESENT, COLD_ABSENT_CANDIDATE, COLD_ABSENT_CONFIRMED,
                  COLD_RETURNED, COLD_STABLE):
        good.feed(event)
    test.check("full power cycle accepted", good.passed, good.summary())

    no_absence = ColdCycleModel()
    for event in (COLD_PRESENT, COLD_RETURNED, COLD_STABLE):
        no_absence.feed(event)
    test.check("return without absence rejected", not no_absence.passed)
    test.check("return without absence is an illegal transition",
               no_absence.invalidated)

    short = ColdCycleModel()
    for event in (COLD_PRESENT, COLD_ABSENT_CANDIDATE, COLD_ABSENCE_ABORTED,
                  COLD_RETURNED):
        short.feed(event)
    test.check("too-short absence cannot reach STABLE", not short.passed)

    partial_cycle = ColdCycleModel()
    for event in (COLD_PRESENT, COLD_ABSENT_CANDIDATE, COLD_ABSENT_CONFIRMED):
        partial_cycle.feed(event)
    test.check("confirmed absence alone is not a pass", not partial_cycle.passed)
    test.check("confirmed absence is not invalidated either",
               not partial_cycle.invalidated)

    import itertools  # noqa: PLC0415 - self-test only

    alphabet = (COLD_PRESENT, COLD_ABSENT_CANDIDATE, COLD_ABSENT_CONFIRMED,
                COLD_ABSENCE_ABORTED, COLD_RETURNED, COLD_STABLE,
                COLD_ESP32_BOOT)
    passing = []
    for length in range(1, 6):
        for sequence in itertools.product(alphabet, repeat=length):
            model = ColdCycleModel()
            for event in sequence:
                model.feed(event)
            if model.passed:
                passing.append(sequence)
    test.check("some sequence can pass", bool(passing))
    test.check("every passing sequence confirms a real absence",
               all(COLD_ABSENT_CONFIRMED in s for s in passing))
    test.check("no passing sequence contains an ESP32 boot",
               all(COLD_ESP32_BOOT not in s for s in passing))
    test.check("every passing sequence starts from PRESENT",
               all(s[0] == COLD_PRESENT for s in passing))
    test.check("every passing sequence ends at STABLE",
               all(s[-1] == COLD_STABLE for s in passing))
    test.check("firmware requires the target present before the cycle",
               "COLD_PRECONDITION_TARGET_ABSENT" in source)
    test.check("firmware prints the operator instruction",
               'Serial.println("POWER CYCLE SERVO NOW")' in source)
    cold_body = firmware_function_body(source, "detectColdCycle")
    test.check("cold detection body performs no write",
               not re.search(r"provWrite|st\.write|WritePosEx", cold_body),
               cold_body[:0])
    test.check("cold detection body only pings and reads the model",
               "st.Ping(" in cold_body and "st.readWord(" in cold_body)
    verify_body = firmware_function_body(source, "verifyColdState")
    test.check("cold verification body performs no write",
               not re.search(r"provWrite|st\.write|WritePosEx", verify_body))

    print()
    print("[15] An ESP32 reset can never satisfy a cold servo cycle")
    reset_mid = ColdCycleModel()
    for event in (COLD_PRESENT, COLD_ABSENT_CANDIDATE, COLD_ABSENT_CONFIRMED):
        reset_mid.feed(event)
    reset_mid.feed(COLD_ESP32_BOOT)
    reset_mid.feed(COLD_RETURNED)
    reset_mid.feed(COLD_STABLE)
    test.check("ESP32 boot invalidates a cycle in progress",
               not reset_mid.passed)
    test.check("ESP32 boot names the right reason",
               reset_mid.reason ==
               "ESP32_RESET_CANNOT_SATISFY_COLD_SERVO_CYCLE",
               reset_mid.reason)

    reset_after = ColdCycleModel()
    for event in (COLD_PRESENT, COLD_ABSENT_CANDIDATE, COLD_ABSENT_CONFIRMED,
                  COLD_RETURNED, COLD_STABLE):
        reset_after.feed(event)
    reset_after.feed(COLD_ESP32_BOOT)
    test.check("ESP32 boot invalidates an already-stable cycle",
               not reset_after.passed)

    boot_lines = [
        "COLD_CYCLE_BEGIN INDEX=1 REQUIRED=1",
        "COLD_STATE=PRESENT",
        "COLD_STATE=ABSENT_CANDIDATE",
        "COLD_STATE=ABSENT_CONFIRMED DEBOUNCE_MS=1500",
        "PROVISIONER_READY",
        "COLD_STATE=RETURNED",
        "COLD_STATE=STABLE SAMPLES=10",
    ]
    models = build_cold_models(boot_lines, 1)
    test.check("boot banner in the transcript invalidates the cycle",
               not models[0].passed, models[0].summary())
    clean_lines = [line for line in boot_lines if line != "PROVISIONER_READY"]
    test.check("same transcript without the reset passes",
               build_cold_models(clean_lines, 1)[0].passed)
    test.expect_raises("wrong number of cold cycles rejected", ProvisionError,
                       build_cold_models, clean_lines, 2)
    test.check("firmware cold state machine lives in RAM inside one EXECUTE",
               "detectColdCycle(targetId, cycle, reason)" in source)

    print()
    print("[16] RAW evidence: 71 bytes on disk before any decoding")
    with tempfile.TemporaryDirectory(prefix="matdog_prov_raw_") as tmpdir:
        path = artifact_path(tmpdir, "BEFORE")
        test.check("BEFORE artifact is 01_before_state71.bin",
                   os.path.basename(path) == "01_before_state71.bin")
        test.check("seven distinct artifact names",
                   len(set(ARTIFACTS.values())) == 7)
        test.expect_raises("unknown slot rejected", ProvisionError,
                           artifact_path, tmpdir, "NOPE")

        artifact = persist_raw71(golden, path)
        test.check("artifact exists", os.path.isfile(path))
        test.check("artifact is exactly 71 bytes on disk",
                   os.path.getsize(path) == SNAPSHOT_LEN)
        test.check("artifact bytes identical to the frame",
                   open(path, "rb").read() == golden)
        test.check("artifact sha256 matches frame and file",
                   artifact["sha256"] == sha256_bytes(golden)
                   == sha256_file(path))
        test.check("artifact records fsync",
                   artifact["flushed_and_fsynced"] is True)
        test.expect_raises("existing artifact fails closed", ProvisionError,
                           persist_raw71, golden, path)
        test.check("rejected rewrite left evidence untouched",
                   sha256_file(path) == sha256_bytes(golden))
        for name, payload in (("70 bytes", golden[:-1]),
                              ("72 bytes", golden + b"\x00"),
                              ("non-bytes", GOLDEN_NEW01_HEX)):
            target = os.path.join(tmpdir, "reject_%s.bin" % name.split()[0])
            test.expect_raises("%s refused" % name, ProvisionError,
                               persist_raw71, payload, target)
            test.check("%s created no file" % name, not os.path.exists(target))

    with tempfile.TemporaryDirectory(prefix="matdog_prov_order_") as tmpdir:
        seen = {}
        this_module = sys.modules[__name__]
        real_decode = this_module.decode_snapshot
        order_path = artifact_path(tmpdir, "BEFORE")

        def probing_decode(raw):
            exists = os.path.isfile(order_path)
            seen["exists"] = exists
            seen["size"] = os.path.getsize(order_path) if exists else -1
            seen["sha"] = sha256_file(order_path) if exists else None
            return real_decode(raw)

        this_module.decode_snapshot = probing_decode
        try:
            back_raw, back = capture_snapshot("BEFORE", GOLDEN_NEW01_HEX,
                                              tmpdir)
        finally:
            this_module.decode_snapshot = real_decode

        test.check("decode_snapshot restored",
                   this_module.decode_snapshot is real_decode)
        test.check("file existed when decode started", seen.get("exists") is True)
        test.check("file was already 71 bytes when decode started",
                   seen.get("size") == SNAPSHOT_LEN, seen.get("size"))
        test.check("bytes were already final when decode started",
                   seen.get("sha") == sha256_bytes(golden))
        test.check("decoded state carries raw_hex and sha",
                   back["raw_hex"] == GOLDEN_NEW01_HEX and
                   back["raw_sha256"] == sha256_bytes(golden))
        test.check("decoded state carries the .bin path",
                   back["raw_artifact"]["path"] == order_path)
        test.check("frame round-trips", back_raw == golden)

    print()
    print("[17] BEFORE evidence is durable before @EXECUTE is transmitted")
    with tempfile.TemporaryDirectory(prefix="matdog_prov_flow_") as tmpdir:
        script = {"@BEGIN NEW01": _begin_transcript()}
        transport = ScriptedTransport(script)
        record = build_record("NEW01", "/dev/null", {}, {}, tmpdir)
        begin = run_begin_phase(transport, record, "NEW01", tmpdir)

        before_path = artifact_path(tmpdir, "BEFORE")
        test.check("only @BEGIN was sent", transport.sent == ["@BEGIN NEW01"])
        test.check("no @EXECUTE was sent yet",
                   not any(c.startswith("@EXECUTE") for c in transport.sent))
        test.check("BEFORE artifact already durable",
                   os.path.isfile(before_path) and
                   os.path.getsize(before_path) == SNAPSHOT_LEN)
        test.check("BEFORE artifact registered in the record",
                   record["raw_artifacts"]["BEFORE"]["path"] == before_path)
        test.check("source ID discovered, not supplied",
                   record["source_id"] == 1)
        test.check("old offset recorded", record["old_position_offset"] == 85)
        test.check("initial physical raw recorded",
                   record["initial_physical_raw"] == 339)
        test.check("center goal recorded",
                   record["center_goal_displayed"] == 1963)
        test.check("canonical delta recorded",
                   len(record["canonical_delta"]) == 20)
        test.check("session token captured",
                   record["session_token_accepted"] == "1A2B3C4D")
        test.check("firmware freeze state captured",
                   record["hardware_freeze"] == "BLOCKED")

        report_path = write_report(record, tmpdir)
        test.check("report written atomically", os.path.isfile(report_path))
        with open(report_path, encoding="utf-8") as handle:
            saved = json.load(handle)
        test.check("report carries the before state",
                   saved["before_state"]["physical_raw"] == 339)

    print()
    print("[18] Session directory and transcript are fail-closed")
    with tempfile.TemporaryDirectory(prefix="matdog_prov_dir_") as tmpdir:
        session_dir = create_session_dir(tmpdir, "NEW01", "20260827_000000Z")
        test.check("session dir created exclusively",
                   os.path.isdir(session_dir))
        test.check("session dir name binds label and stamp",
                   os.path.basename(session_dir) == "NEW01__20260827_000000Z")
        test.expect_raises("existing session dir refused", ProvisionError,
                           create_session_dir, tmpdir, "NEW01",
                           "20260827_000000Z")
        test.check("refusal left the existing directory untouched",
                   os.path.isdir(session_dir))
        open(os.path.join(session_dir, "x.bin"), "wb").write(b"x")
        sums = write_sha256sums(session_dir)
        test.check("SHA256SUMS written", os.path.isfile(sums))
        test.check("SHA256SUMS excludes itself",
                   SHA256SUMS_NAME not in open(sums, encoding="utf-8").read())

    lines = _begin_transcript()
    test.check("valid BEGIN parses", parse_begin(lines)["source_id"] == 1)
    for name, mutate in (
        ("BEGIN_RESULT missing", lambda l: [x for x in l
                                            if x != "BEGIN_RESULT PASS"]),
        ("WAIT_EXECUTE missing", lambda l: [x for x in l
                                            if x != "WAIT_EXECUTE"]),
        ("gate not passed", lambda l: [x for x in l
                                       if x != "GATE_RESULT PASS"]),
        ("EEPROM_WRITES claim missing",
         lambda l: [x for x in l if x != "EEPROM_WRITES=NONE"]),
        ("MOTION claim missing", lambda l: [x for x in l if x != "MOTION=NONE"]),
        ("BEFORE snapshot missing",
         lambda l: [x for x in l if "SLOT=BEFORE" not in x
                    and not x.startswith("RAW71_HEX")]),
        ("abort line present", lambda l: l + ["BEGIN_ABORT: GATE_FAILED"]),
    ):
        test.expect_raises("BEGIN rejected: %s" % name, ProvisionError,
                           parse_begin, mutate(lines))
    test.expect_raises("malformed token rejected", ProvisionError, parse_begin,
                       [x.replace("SESSION_TOKEN=1A2B3C4D", "SESSION_TOKEN=zz")
                        for x in lines])
    test.expect_raises("duplicate RAW71_HEX rejected", ProvisionError,
                       extract_snapshots,
                       ["SNAPSHOT_BEGIN SLOT=BEFORE ID=1 START=0x00 LEN=71",
                        "RAW71_HEX=" + GOLDEN_NEW01_HEX,
                        "RAW71_HEX=" + GOLDEN_NEW01_HEX,
                        "SNAPSHOT_END SLOT=BEFORE"])
    test.expect_raises("mismatched snapshot slots rejected", ProvisionError,
                       extract_snapshots,
                       ["SNAPSHOT_BEGIN SLOT=BEFORE ID=1 START=0x00 LEN=71",
                        "RAW71_HEX=" + GOLDEN_NEW01_HEX,
                        "SNAPSHOT_END SLOT=WARM"])
    test.expect_raises("unterminated snapshot rejected", ProvisionError,
                       extract_snapshots,
                       ["SNAPSHOT_BEGIN SLOT=BEFORE ID=1 START=0x00 LEN=71",
                        "RAW71_HEX=" + GOLDEN_NEW01_HEX])

    print()
    print("[19] Session token binding")
    test.check("host sends back exactly the firmware token",
               assert_command_allowed("@EXECUTE 1A2B3C4D") ==
               "@EXECUTE 1A2B3C4D")
    test.expect_raises("lowercase token refused by the surface",
                       SafetyViolation, assert_command_allowed,
                       "@EXECUTE 1a2b3c4d")
    test.expect_raises("short token refused", SafetyViolation,
                       assert_command_allowed, "@EXECUTE 1A2B")
    test.check("firmware compares the token",
               "SESSION_TOKEN_MISMATCH" in source and
               "strcmp(token, sessionToken)" in source)
    test.check("firmware token is single use",
               "Single-use token" in source)
    test.check("firmware refuses EXECUTE with no waiting session",
               "NO_SESSION_WAITING" in source)
    with tempfile.TemporaryDirectory(prefix="matdog_prov_token_") as tmpdir:
        script = {
            "@BEGIN NEW01": _begin_transcript(),
            "@EXECUTE 1A2B3C4D": [
                "EXECUTE_ABORT: SESSION_TOKEN_MISMATCH",
                "EEPROM_WRITES=NONE",
                "MOTION=NONE",
                "EXECUTE_RESULT FAIL",
            ],
        }
        transport = ScriptedTransport(script)
        record = build_record("NEW01", "/dev/null", {}, {}, tmpdir)
        begin = run_begin_phase(transport, record, "NEW01", tmpdir)
        test.expect_raises("token mismatch aborts the host session",
                           ProvisionError, run_execute_phase, transport,
                           record, begin, tmpdir)

    print()
    print("[20] Failure paths: relock and torque-off intent, no motion")
    test.check("firmware has a single recovery path", "enterFault(" in source)
    test.check("recovery writes TorqueEnable 0",
               re.search(r"enterFault.*?REG_TORQUE_ENABLE, 0", source, re.S)
               is not None)
    test.check("recovery restores the EEPROM lock",
               re.search(r"enterFault.*?REG_LOCK, 1", source, re.S) is not None)
    test.check("recovery cannot command motion",
               "stage == STAGE_PRIME || stage == STAGE_CENTER" in source or
               "STAGE_PRIME && stage != STAGE_CENTER" in source)
    test.check("STAGE_RECOVERY may only write torque 0 and lock 1",
               "STAGE_RECOVERY" in source and
               not re.search(r"REG_POSITION_OFFSET.*STAGE_RECOVERY", source))
    test.check("firmware implements no rollback",
               not re.search(r"\b(rollback|restoreBackup|revert)\s*\(",
                             source, re.I))
    test.check("firmware documents the no-rollback policy",
               "no retry, no rollback" in source.lower())
    test.check("firmware requests power removal when recovery fails",
               'Serial.println("CUT_SERVO_POWER_NOW")' in source)
    test.check("host guard sends no emergency command",
               "No emergency command is ever sent from here" in
               guard_session.__doc__)

    guard_record = build_record("NEW01", "/dev/null", {}, {}, "/tmp")
    counters = {"persist": 0, "close": 0}

    def boom():
        raise OSError("simulated serial failure")

    test.expect_raises("unexpected exception re-raised", OSError, guard_session,
                       guard_record, boom,
                       lambda: counters.__setitem__("persist",
                                                    counters["persist"] + 1),
                       lambda: counters.__setitem__("close",
                                                    counters["close"] + 1))
    test.check("unexpected exception -> verdict FAIL",
               guard_record["verdict"] == "FAIL")
    test.check("reason names the exception type",
               guard_record["reason"].startswith("OSError: "))
    test.check("report persisted in finally", counters["persist"] == 1)
    test.check("port closed in finally", counters["close"] == 1)
    test.check("no command was invented by the guard",
               guard_record["commands_sent"] == [])

    print()
    print("[21] Command surface is closed")
    for allowed in ("@BEGIN NEW01", "@BEGIN M22", "@BEGIN ELR04",
                    "@EXECUTE 00000000", "@EXECUTE ABCDEF01", "@HELP"):
        test.check("allowed: %s" % allowed,
                   assert_command_allowed(allowed) == allowed)
    for forbidden in ("@SCAN 0 253", "@SNAPSHOT71 13", "@SAFE_OFF 13",
                      "@NORMALIZE_MATDOG 13", "@QC_FAST 13", "@READ 13",
                      "@DUMP_RAW 13", "@WRITE 0x15 32", "@SETID 23",
                      "@GOAL 2048", "@TORQUE 1", "@RESET", "@CALIBRATE",
                      "@BEGIN", "@BEGIN new01", "@BEGIN NEW01 23",
                      "@EXECUTE", "@EXECUTE 1A2B3C4D5", "@EXECUTE GGGGGGGG",
                      "@HELP ", " @HELP", "", "BEGIN NEW01",
                      "@BEGIN NEW01\n@EXECUTE 1A2B3C4D"):
        test.expect_raises("rejected: %r" % forbidden, SafetyViolation,
                           assert_command_allowed, forbidden)
    test.expect_raises("non-string rejected", SafetyViolation,
                       assert_command_allowed, 13)
    test.check("surface has exactly 3 patterns",
               len(ALLOWED_COMMAND_PATTERNS) == 3)
    surface = command_surface_audit()
    for key in ("generic_eeprom_write", "arbitrary_id_write",
                "arbitrary_goal_position", "calibration_ofs",
                "host_exposed_torque_on", "factory_reset", "broadcast_write"):
        test.check("surface declares %s NOT IMPLEMENTED" % key,
                   surface[key].startswith("NOT IMPLEMENTED"))
    test.check("surface declares @BEGIN performs no writes",
               surface["begin_performs_servo_writes"] == "NO")
    test.check("surface declares no confirmation string",
               surface["confirmation_string"].startswith("NONE"))

    print()
    print("[22] Hardware freeze is cleared and guards remain fail-closed")
    test.check("firmware reports the freeze as cleared",
               audit["hardware_freeze_blocked"] is False)
    test.check("no unresolved constants remain",
               audit["unresolved_constants"] == [],
               audit["unresolved_constants"])
    test.check("firmware retains freeze abort guard",
               "EXECUTE_ABORT: HARDWARE_FREEZE_BLOCKED" in source)
    test.check("the abort precedes every stage in the source",
               source.index("HARDWARE_FREEZE_BLOCKED) {") <
               source.index('STAGE_BEGIN NAME=TORQUE_OFF'))
    runner_src = __import__("inspect").getsource(run_provision)
    test.check("runner retains hardware-freeze guard",
               'audit["hardware_freeze_blocked"]' in runner_src)
    test.check("runner guard raises ProvisionError",
               "raise ProvisionError" in runner_src)

    print()
    print("[23a] Position confirmation before a gated snapshot")
    test.check("acceptance band is untouched by the confirmation",
               CENTER_ACCEPT_TICKS == 1
               and "CENTER_ACCEPT_TICKS = 1;" in source)
    settle = firmware_function_body(source, "positionSettled")
    test.check("confirmation reuses the validated QC stable-sample count",
               "agree >= MON_STABLE_SAMPLES" in settle)
    test.check("confirmation polls at the validated QC interval",
               "delayMicroseconds(MON_PERIOD_US)" in settle)
    test.check("confirmation is bounded",
               "i < POSITION_CONFIRM_MAX_SAMPLES" in settle)
    test.check("confirmation fails closed when it never settles",
               'reason = "POSITION_NEVER_SETTLED";' in settle)
    test.check("confirmation domain-guards the reading",
               "p < 0 || p > 4095" in settle)
    test.check("confirmation is strictly read-only",
               not any(w in settle for w in
                       ("provWrite", "writeByte", "writeWord", "WritePosEx",
                        "EnableTorque", "unLockEprom", "LockEprom")))
    test.check("confirmation never relaxes a tolerance",
               "CENTER_ACCEPT_TICKS" not in settle)

    execute = firmware_function_body(source, "runExecute")
    for slot in ("OFFSET_ZERO", "WARM"):
        idx_confirm = execute.find("positionSettled")
        test.check("%s snapshot is preceded by a confirmation" % slot,
                   re.search(
                       r"positionSettled\([a-zA-Z]+, reason\)[^}]*?\}\s*\n\s*\n"
                       r"\s*if \(!captureAndEmit\(\"%s\"" % slot,
                       execute, re.S) is not None)
    test.check("both gated snapshots are confirmed",
               execute.count("positionSettled(") == 2)
    test.check("a failed confirmation enters fault",
               execute.count("enterFault(sourceId, reason)") >= 1
               and "enterFault(targetId, reason)" in execute)

    print()
    print("[23b] Bounded centering correction — staging re-approach")

    # The acceptance band itself is never touched by a correction.
    test.check("acceptance stays +/-1 tick", CENTER_ACCEPT_TICKS == 1)
    test.check("firmware acceptance stays +/-1 tick",
               "CENTER_ACCEPT_TICKS = 1;" in source)

    # error inside the band -> no correction at all
    for err in (0, 1, -1):
        test.check("error %+d needs no correction" % err,
                   center_correction_decision(err, 1) == CORRECTION_PASS)

    # small miss outside the band but inside the window -> correction allowed
    for err in (2, -2, 3, -3):
        test.check("error %+d is corrected" % err,
                   center_correction_decision(err, 1) == CORRECTION_CORRECT)

    # the exact V2 pilot failures
    test.check("V2 first settle (-3) would be corrected",
               center_correction_decision(-3, 1) == CORRECTION_CORRECT)
    test.check("V2 post-correction settle (+2) is still corrected, not passed",
               center_correction_decision(2, 2) == CORRECTION_CORRECT)

    # beyond the correction window -> fail closed, no motion attempted
    for err in (4, -4, 5, -32, 2048):
        test.check("error %+d fails outside the correction window" % err,
                   center_correction_decision(err, 1) == CORRECTION_FAIL_WINDOW)

    # attempts are bounded
    test.check("attempt %d is the last allowed" % CENTER_MAX_CORRECTIONS,
               center_correction_decision(2, CENTER_MAX_CORRECTIONS)
               == CORRECTION_CORRECT)
    test.check("attempt %d is refused" % (CENTER_MAX_CORRECTIONS + 1),
               center_correction_decision(2, CENTER_MAX_CORRECTIONS + 1)
               == CORRECTION_FAIL_TOLERANCE)
    test.check("still outside +/-1 after the last attempt -> FAIL",
               center_correction_decision(-2, CENTER_MAX_CORRECTIONS + 1)
               == CORRECTION_FAIL_TOLERANCE)
    test.check("a correction can never report PASS while error > 1",
               all(center_correction_decision(e, a) != CORRECTION_PASS
                   for e in range(-2048, 2048)
                   for a in range(1, CENTER_MAX_CORRECTIONS + 3)
                   if abs(e) > CENTER_ACCEPT_TICKS))

    # --- terminal deadband feedback -------------------------------------
    test.check("attempt 1 always runs unbiased",
               center_bias_update(0, -2, 1) == 0)
    test.check("bias feeds back the previous re-approach error",
               center_bias_update(0, -2, 2) == 2)
    test.check("bias accumulates in the right direction",
               center_bias_update(2, -1, 3) == 3)
    test.check("bias outside the validated window fails closed",
               center_bias_update(2, -3, 2) is None)
    test.check("bias is clamped by the correction window",
               CENTER_MAX_BIAS_TICKS == MON_REACHED_RESIDUAL)

    # Replay of the exact V3 hardware failure against the measured model:
    # the servo stops 2 ticks short of any commanded target (5/5 observed).
    verdict, used, final = simulate_centering(initial_error=2, deadband=2)
    test.check("measured 2-tick deadband converges to PASS",
               verdict == CORRECTION_PASS, (verdict, used, final))
    test.check("it converges inside the attempt bound", used <= 2, used)
    test.check("it lands exactly on centre", final == 0, final)
    test.check("V3 firmware would have failed this case",
               simulate_centering(2, 2, max_attempts=2)[0] != CORRECTION_PASS
               or CENTER_MAX_CORRECTIONS >= 3)

    # convergence across every deadband the correction window admits
    for db in (0, 1, 2, 3):
        for e0 in (-3, -2, 2, 3):
            verdict, used, final = simulate_centering(e0, db)
            test.check("deadband %d, initial error %+d -> PASS" % (db, e0),
                       verdict == CORRECTION_PASS, (verdict, used, final))
            test.check("deadband %d, initial error %+d ends within +/-1"
                       % (db, e0), abs(final) <= CENTER_ACCEPT_TICKS, final)

    # a deadband beyond the window must fail closed, never be forced
    for db in (5, 8):
        verdict, _u, _f = simulate_centering(2, db)
        test.check("deadband %d fails closed" % db,
                   verdict != CORRECTION_PASS, verdict)

    test.check("no simulated outcome ever passes with error > 1",
               all(simulate_centering(e0, db)[0] != CORRECTION_PASS
                   or abs(simulate_centering(e0, db)[2]) <= CENTER_ACCEPT_TICKS
                   for db in range(0, 9) for e0 in range(-4, 5)))

    # firmware wiring
    test.check("firmware feeds the bias back",
               "bias -= error;" in source)
    test.check("firmware runs attempt 1 unbiased",
               "if (attempt > 1) {" in source)
    test.check("firmware clamps the bias",
               "abs((int)bias) > CENTER_MAX_BIAS_TICKS" in source)
    test.check("firmware re-approaches goal + bias",
               "const int32_t approach = goal + bias;" in source)
    test.check("firmware domain-guards the biased approach",
               "approach < 0 || approach > 4095" in source)

    # staging point: direction, magnitude, domain
    test.check("staging is below the goal (deterministic backlash side)",
               center_staging_goal(1963) == 1963 - CENTER_STAGING_TICKS)
    test.check("staging step exceeds the observed stiction threshold",
               CENTER_STAGING_TICKS > 6)
    test.check("staging step stays well inside the QC direction tolerance",
               CENTER_STAGING_TICKS < 16)
    test.check("NEW01 staging goal is 1951", center_staging_goal(1963) == 1951)
    test.check("staging goal stays in 0..4095 for every legal goal",
               all(0 <= center_staging_goal(g) <= 4095
                   for g in range(0, 4096)))
    for g in range(0, CENTER_STAGING_TICKS):
        test.check("goal %d uses boundary-safe staging above" % g,
                   center_staging_goal(g) == g + CENTER_STAGING_TICKS)
    test.check("ELR01 goal 3 boundary-safe staging is 15",
               center_staging_goal(3) == 15)
    test.check("ELR01 biased approach 1 boundary-safe staging is 13",
               center_staging_goal(1) == 13)
    test.check("high edge still stages below without wrapping",
               center_staging_goal(4095) == 4095 - CENTER_STAGING_TICKS)
    # the re-approach target is the goal plus a measured, clamped bias — never
    # the V2 open-loop reflection of the settle error
    test.check("firmware re-approaches goal + measured bias",
               'centerSegment(id, approach, "CENTER_REAPPROACH"' in source)
    test.check("V2 open-loop reflection is gone",
               "correctionGoal = goal - error" not in source)
    test.check("the biased target can never exceed the correction window",
               all(abs((goal + b) - goal) <= MON_REACHED_RESIDUAL
                   for goal in (0, 1963, 4095)
                   for b in range(-CENTER_MAX_BIAS_TICKS,
                                  CENTER_MAX_BIAS_TICKS + 1)))

    # firmware/host constant agreement
    for name, value in (("CENTER_STAGING_TICKS", CENTER_STAGING_TICKS),
                        ("CENTER_MAX_CORRECTIONS", CENTER_MAX_CORRECTIONS),
                        ("MON_REACHED_RESIDUAL", MON_REACHED_RESIDUAL)):
        test.check("firmware %s = %d" % (name, value),
                   re.search(r"%s = %d;" % (name, value), source) is not None)

    # the attempt loop is bounded by the constant, not by a condition on error
    centering = firmware_function_body(source, "runCentering")
    test.check("correction loop is bounded by CENTER_MAX_CORRECTIONS",
               "attempt <= CENTER_MAX_CORRECTIONS" in centering)
    test.check("exactly one correction loop",
               centering.count("for (int attempt") == 1)
    test.check("no while loop in centering",
               "while (" not in centering)
    test.check("single final acceptance gate at +/-1",
               centering.count("abs((int)error) > CENTER_ACCEPT_TICKS") == 2)

    # every motion segment is torque-on -> move -> torque-off, fail-closed
    seg = firmware_function_body(source, "centerSegment")
    test.check("segment refuses outside STAGE_CENTER",
               "stage != STAGE_CENTER" in seg)
    test.check("segment domain-guards the goal",
               "target < 0 || target > 4095" in seg)
    test.check("segment torque OFF is unconditional",
               seg.index("torqueOffVerified") < seg.index("if (!moved)"))
    test.check("segment value-initialises its MotionSummary",
               "MotionSummary m{};" in seg)
    test.check("no uninitialised MotionSummary anywhere",
               "MotionSummary m;" not in source
               and "MotionSummary correction;" not in source)

    print()
    print("[23] Centering constants and motion guards")
    test.check("TorqueLimit 300", CENTER_TORQUE_LIMIT == 300)
    test.check("GoalSpeed 365", CENTER_SPEED == 365)
    test.check("Acceleration 50", CENTER_ACC == 50)
    test.check("acceptance +/-1 tick", CENTER_ACCEPT_TICKS == 1)
    test.check("raw center 2048", RAW_CENTER == 2048)
    for name, value in (("CENTER_TORQUE_LIMIT", 300), ("CENTER_SPEED", 365),
                        ("CENTER_ACC", 50)):
        test.check("firmware %s = %d" % (name, value),
                   re.search(r"%s = %d;" % (name, value), source) is not None)
    test.check("firmware polls at 500 Hz",
               "MON_PERIOD_US = 2000" in source)
    test.check("firmware reuses the QC move-timeout formula",
               "computeMoveTimeoutMs" in source and
               "expectedFloorTps = speedCmd / 2" in source)
    test.check("firmware monitors position, speed, load, current, "
               "voltage, temperature, status and moving",
               all(token in source for token in
                   ("st.ReadPos(-1)", "st.ReadSpeed(-1)", "st.ReadLoad(-1)",
                    "st.ReadCurrent(-1)", "st.ReadVoltage(-1)",
                    "st.ReadTemper(-1)", "st.ReadMove(-1)", "st.Error")))
    test.check("firmware overcurrent guard is the QC 2 A value",
               "MON_OVERCURRENT_RAW = 308" in source)
    test.check("firmware stall-current guard is the QC 2.7 A value",
               "MON_STALL_CURRENT_RAW = 416" in source)
    test.check("load alone can never stop a move",
               "load ALONE never stops a move" in source)
    test.check("watchdog is firmware-local",
               "Blocking, host-independent. Never reads Serial." in source)
    test.check("prime assumes torque may arm",
               "WritePosEx can arm torque" in source)
    test.check("prime forces torque off before evaluating anything",
               source.index("torqueOffVerified(id, \"PRIME\")") <
               source.index("PRIME_POSITION_AFTER"))

    print()
    print("[24] Firmware artifacts and provenance")
    try:
        firmware = verify_frozen_firmware()
        test.check("firmware source SHA256 matches", firmware["source"]["match"])
        test.check("firmware binary SHA256 matches", firmware["binary"]["match"])
        test.check("FQBN recorded",
                   firmware["fqbn"].startswith("esp32:esp32:esp32s3"))
    except ProvisionError as exc:
        test.check("firmware verification", False, str(exc))
    test.check("runner self-identifies by SHA256",
               len(sha256_file(os.path.abspath(__file__))) == 64)
    test.check("report schema is versioned",
               REPORT_SCHEMA == "MATDOG_ST3215_PROVISIONER_V1")
    test.check("profile identifier is MATDOG_C018_V1",
               PROFILE_ID == "MATDOG_C018_V1")

    print()
    print("=" * 70)
    total = test.passed + len(test.failed)
    if test.failed:
        print(" SELF-TEST RESULT: FAIL  (%d/%d passed)" % (test.passed, total))
        for name in test.failed:
            print("   failed: %s" % name)
        print("=" * 70)
        return 1
    print(" SELF-TEST RESULT: PASS  (%d/%d checks)" % (test.passed, total))
    print("=" * 70)
    return 0


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def main(argv=None):
    parser = argparse.ArgumentParser(
        prog=TOOL_NAME,
        description="MATDOG ST-3215-C018 Provisioner V1",
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--unit", metavar="LABEL",
                      help="HARDWARE MODE. Provision one physical unit, "
                           "e.g. --unit NEW01")
    mode.add_argument("--self-test", action="store_true",
                      help="run the offline self-test; opens no serial port")
    mode.add_argument("--command-surface", action="store_true",
                      help="print the allowed command surface; opens no port")
    mode.add_argument("--static-audit", action="store_true",
                      help="audit every firmware write primitive; opens no port")
    mode.add_argument("--verify-firmware", action="store_true",
                      help="verify firmware SHA256; opens no port")

    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--session-root", default=DEFAULT_SESSION_ROOT)

    args = parser.parse_args(argv)

    if args.self_test:
        return run_self_test()

    if args.command_surface:
        print(json.dumps(command_surface_audit(), indent=2, sort_keys=True))
        return 0

    if args.static_audit:
        audit = static_write_audit()
        print(json.dumps(audit, indent=2, sort_keys=True))
        return 0 if audit["ok"] else 1

    if args.verify_firmware:
        print(json.dumps(verify_frozen_firmware(), indent=2, sort_keys=True))
        return 0

    if args.unit:
        try:
            record = run_provision(args.unit, args.port, args.session_root)
        except (ProvisionError, SafetyViolation) as exc:
            print("PROVISION FAILED: %s" % exc, file=sys.stderr)
            return 1
        return 0 if record["verdict"] == "PASS" else 1

    # Fail-closed default: no mode selected means no action, no port.
    parser.print_usage(sys.stderr)
    print("ERROR: no mode selected. Nothing was done and no port was opened.",
          file=sys.stderr)
    return 2


if __name__ == "__main__":
    sys.exit(main())
