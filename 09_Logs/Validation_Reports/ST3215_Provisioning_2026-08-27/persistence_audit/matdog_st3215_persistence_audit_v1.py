#!/usr/bin/env python3
"""
MATDOG ST-3215-C018 — POST-PROVISION PERSISTENCE AUDIT, strictly read-only.

Independently verifies that an already-provisioned unit still holds every
MATDOG parameter across a REAL servo power cycle.

WHAT THIS TOOL CAN SEND
    @BEGIN <LABEL>      the provisioner's audited zero-write command:
                        full 0..253 scan, 71-byte snapshot, gate. No writes.
    @HELP

    @EXECUTE is NOT in the allowlist and cannot be composed. There is no
    EEPROM write, no ID write, no PositionOffset write, no TorqueLimit write
    and no motion command anywhere in this file.

The verdict is computed from the RAW BYTES, never from a previous report.
"""

import argparse
import datetime
import hashlib
import json
import os
import re
import sys
import time

TOOL = "matdog_st3215_persistence_audit_v1"
SNAPSHOT_LEN = 71

PORT = ("/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_"
        "14:C1:9F:22:75:94-if00")

# The only strings this process may ever put on the wire.
ALLOWED = (re.compile(r"^@BEGIN [A-Z0-9]{1,15}$"), re.compile(r"^@HELP$"))
FORBIDDEN_SUBSTRING = "@EXECUTE"


class AuditError(RuntimeError):
    pass


def assert_command_allowed(cmd):
    if FORBIDDEN_SUBSTRING in cmd:
        raise AuditError("refusing to transmit a write-capable command: %r" % cmd)
    for pat in ALLOWED:
        if pat.match(cmd):
            return cmd
    raise AuditError("command not in the read-only surface: %r" % cmd)


# --------------------------------------------------------------------------
# Register map and expectations
# --------------------------------------------------------------------------

EEPROM_LAST = 0x27          # 0x00..0x27 persistent, 0x28..0x46 RAM/telemetry

def identity_map(target_id):
    return {
        "model_0x03": (0x03, 2, 777),
        "id_0x05": (0x05, 1, target_id),
        "baud_0x06": (0x06, 1, 0),
        "response_status_0x08": (0x08, 1, 1),
    }

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

ADDR_POSITION_OFFSET = 0x1F
ADDR_TORQUE_ENABLE = 0x28
ADDR_TORQUE_LIMIT = 0x30
ADDR_LOCK = 0x37
ADDR_PRESENT_POSITION = 0x38
ADDR_PRESENT_VOLTAGE = 0x3E
ADDR_PRESENT_TEMPERATURE = 0x3F
ADDR_STATUS = 0x40

RAW_CENTER = 2048
CENTER_TOLERANCE = 1


def u16(raw, a):
    return raw[a] | (raw[a + 1] << 8)


def i16(raw, a):
    v = u16(raw, a)
    return v - 0x10000 if v & 0x8000 else v


def field(raw, addr, width):
    return u16(raw, addr) if width == 2 else raw[addr]


def floor_mod(v, m):
    r = v % m
    return r + m if r < 0 else r


def sha256_bytes(b):
    return hashlib.sha256(b).hexdigest()


def utc():
    return datetime.datetime.now(datetime.timezone.utc).isoformat()


# --------------------------------------------------------------------------
# Transport — read-only by construction
# --------------------------------------------------------------------------

class ReadOnlyTransport:
    def __init__(self, port):
        import serial
        s = serial.Serial()
        s.port = port
        s.baudrate = 115200
        s.timeout = 2
        s.write_timeout = 3
        s.dtr = False
        s.rts = False
        s.open()
        time.sleep(2.5)
        s.reset_input_buffer()
        self._s = s
        self.sent = []

    def send(self, cmd):
        checked = assert_command_allowed(cmd)
        self.sent.append(checked)
        self._s.write((checked + "\n").encode("ascii"))
        self._s.flush()
        return checked

    def read_until(self, terms, timeout):
        deadline = time.monotonic() + timeout
        lines = []
        while time.monotonic() < deadline:
            raw = self._s.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", "replace").strip()
            if not line:
                continue
            lines.append(line)
            if any(line.startswith(t) for t in terms):
                return lines
        raise AuditError("timeout after %.0fs waiting for %s" % (timeout, terms))

    def close(self):
        self._s.close()


# --------------------------------------------------------------------------
# Capture
# --------------------------------------------------------------------------

def capture(label, outdir, slot):
    t = ReadOnlyTransport(PORT)
    try:
        t.send("@BEGIN " + label)
        lines = t.read_until(("BEGIN_RESULT",), 180.0)
    finally:
        t.close()

    # Nothing this tool sent could write. Prove it from the transcript too.
    if any("@EXECUTE" in c for c in t.sent):
        raise AuditError("a write-capable command was transmitted")
    writes = [l for l in lines if l.startswith("WRITE ")]
    if writes:
        raise AuditError("firmware reported writes during a read-only audit: %s"
                         % writes)

    found = []
    for line in lines:
        if line.startswith("FOUND "):
            f = dict(re.findall(r"\b([A-Z0-9_]+)=(\S+)", line))
            found.append({k.lower(): int(v, 16) if v.startswith("0x") else int(v)
                          for k, v in f.items()})
    responders = None
    for line in lines:
        m = re.match(r"^SCAN_RESULT FOUND=(\d+)$", line)
        if m:
            responders = int(m.group(1))

    raw_hex = None
    for line in lines:
        m = re.match(r"^RAW71_HEX=([0-9A-Fa-f]+)$", line)
        if m:
            raw_hex = m.group(1)

    os.makedirs(outdir, exist_ok=True)
    with open(os.path.join(outdir, "%s_transcript.log" % slot), "w") as h:
        h.write("\n".join(lines) + "\n")
        h.flush()
        os.fsync(h.fileno())

    raw = None
    if raw_hex:
        if len(raw_hex) != SNAPSHOT_LEN * 2:
            raise AuditError("RAW71_HEX is not 71 bytes")
        raw = bytes.fromhex(raw_hex)
        path = os.path.join(outdir, "%s_state71.bin" % slot)
        with open(path, "xb") as h:
            h.write(raw)
            h.flush()
            os.fsync(h.fileno())

    return {"slot": slot, "lines": lines, "found": found,
            "responders": responders, "raw": raw,
            "raw_sha256": sha256_bytes(raw) if raw else None,
            "captured_utc": utc()}


# --------------------------------------------------------------------------
# Decode and verify
# --------------------------------------------------------------------------

def decode(raw):
    offset = i16(raw, ADDR_POSITION_OFFSET)
    present = u16(raw, ADDR_PRESENT_POSITION)
    d = {
        "model_0x03": u16(raw, 0x03),
        "id_0x05": raw[0x05],
        "baud_0x06": raw[0x06],
        "response_status_0x08": raw[0x08],
        "position_offset_0x1F": offset,
        "torque_enable_0x28": raw[ADDR_TORQUE_ENABLE],
        "torque_limit_0x30": u16(raw, ADDR_TORQUE_LIMIT),
        "lock_0x37": raw[ADDR_LOCK],
        "present_position_0x38": present,
        "physical_raw": floor_mod(present + offset, 4096),
        "voltage_0x3E": raw[ADDR_PRESENT_VOLTAGE],
        "temperature_0x3F": raw[ADDR_PRESENT_TEMPERATURE],
        "status_0x40": raw[ADDR_STATUS],
    }
    for addr, width, _exp, name in PROFILE:
        d["profile_%s_0x%02X" % (name, addr)] = field(raw, addr, width)
    return d


def verify(raw, responders, found, phase, target_id, old_id):
    checks = []

    def chk(group, name, actual, expected, ok=None):
        checks.append({
            "group": group, "requirement": name,
            "expected": expected, "actual": actual,
            "pass": (actual == expected) if ok is None else bool(ok),
        })

    for name, (addr, width, exp) in identity_map(target_id).items():
        chk("IDENTITY", name, field(raw, addr, width), exp)
    chk("IDENTITY", "position_offset_0x1F", i16(raw, ADDR_POSITION_OFFSET), 0)

    chk("SAFETY", "torque_enable_0x28", raw[ADDR_TORQUE_ENABLE], 0)
    chk("SAFETY", "lock_0x37", raw[ADDR_LOCK], 1)
    chk("SAFETY", "status_0x40", raw[ADDR_STATUS], 0)
    if phase == "post_cold":
        chk("SAFETY", "torque_limit_0x30_after_reboot",
            u16(raw, ADDR_TORQUE_LIMIT), 1000)
    else:
        checks.append({
            "group": "SAFETY",
            "requirement": "torque_limit_0x30 (observed, not asserted pre-cold)",
            "expected": "OBSERVED", "actual": u16(raw, ADDR_TORQUE_LIMIT),
            "pass": True,
        })

    for addr, width, exp, name in PROFILE:
        chk("PROFILE", "0x%02X %s" % (addr, name), field(raw, addr, width), exp)

    present = u16(raw, ADDR_PRESENT_POSITION)
    offset = i16(raw, ADDR_POSITION_OFFSET)
    physical = floor_mod(present + offset, 4096)
    chk("CENTER", "physical_raw equals PresentPosition (offset is 0)",
        physical, present)
    err = floor_mod(physical - RAW_CENTER + 2048, 4096) - 2048
    checks.append({
        "group": "CENTER",
        "requirement": "physical_raw 2048 +/-%d" % CENTER_TOLERANCE,
        "expected": "2048 +/-%d" % CENTER_TOLERANCE,
        "actual": "%d (error %+d)" % (physical, err),
        "pass": abs(err) <= CENTER_TOLERANCE,
    })

    ids = sorted(f["id"] for f in found)
    m777 = sorted(f["id"] for f in found if f.get("model") == 777)
    chk("BUS", "exactly one responder on 0..253", responders, 1)
    chk("BUS", "the responder is ID %d" % target_id, ids, [target_id])
    chk("BUS", "exactly one model 777 on the bus", m777, [target_id])
    checks.append({
        "group": "BUS", "requirement": "old ID %d is silent" % old_id,
        "expected": "absent from scan", "actual": ids,
        "pass": old_id not in ids,
    })
    for f in found:
        chk("BUS", "ID %d ping status clean" % f["id"], f.get("ping_status"), 0)
        chk("BUS", "ID %d model status clean" % f["id"], f.get("model_status"), 0)

    return checks


def compare(pre, post):
    diffs = []
    for addr in range(SNAPSHOT_LEN):
        if pre[addr] != post[addr]:
            diffs.append({
                "address": "0x%02X" % addr,
                "region": "EEPROM_PERSISTENT" if addr <= EEPROM_LAST else "RAM_TELEMETRY",
                "pre": pre[addr], "post": post[addr],
            })
    persistent = [d for d in diffs if d["region"] == "EEPROM_PERSISTENT"]
    return diffs, persistent


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def main(argv=None):
    ap = argparse.ArgumentParser(prog=TOOL)
    ap.add_argument("--label", default="NEW01")
    ap.add_argument("--target-id", type=int, default=23)
    ap.add_argument("--old-id", type=int, default=1)
    ap.add_argument("--outdir", required=True)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--capture", metavar="SLOT")
    g.add_argument("--absence", action="store_true")
    g.add_argument("--report", action="store_true")
    a = ap.parse_args(argv)

    if a.absence:
        r = capture(a.label, a.outdir, "absence")
        present = r["responders"] not in (0, None)
        print(json.dumps({"responders": r["responders"],
                          "servo_present": present,
                          "absence_proven": not present}, indent=2))
        return 0 if not present else 1

    if a.capture:
        r = capture(a.label, a.outdir, a.capture)
        if r["raw"] is None:
            print("NO SNAPSHOT — responders=%s" % r["responders"])
            return 1
        print(json.dumps({"slot": r["slot"], "responders": r["responders"],
                          "found": r["found"], "sha256": r["raw_sha256"],
                          "decoded": decode(r["raw"])}, indent=2, sort_keys=True))
        return 0

    # --report
    pre = open(os.path.join(a.outdir, "pre_cold_state71.bin"), "rb").read()
    post = open(os.path.join(a.outdir, "post_cold_state71.bin"), "rb").read()
    ptx = open(os.path.join(a.outdir, "post_cold_transcript.log")).read().splitlines()
    found, responders = [], None
    for line in ptx:
        if line.startswith("FOUND "):
            f = dict(re.findall(r"\b([A-Z0-9_]+)=(\S+)", line))
            found.append({k.lower(): int(v, 16) if v.startswith("0x") else int(v)
                          for k, v in f.items()})
        m = re.match(r"^SCAN_RESULT FOUND=(\d+)$", line)
        if m:
            responders = int(m.group(1))

    checks = verify(post, responders, found, "post_cold",
                    a.target_id, a.old_id)
    diffs, persistent = compare(pre, post)
    checks.append({
        "group": "PERSISTENCE",
        "requirement": "every EEPROM byte 0x00..0x27 identical across the power cycle",
        "expected": "0 differences", "actual": "%d differences" % len(persistent),
        "pass": not persistent,
    })

    failed = [c for c in checks if not c["pass"]]
    report = {
        "tool": TOOL,
        "label": a.label,
        "target_id": a.target_id,
        "old_id_expected_silent": a.old_id,
        "generated_utc": utc(),
        "port": PORT,
        "commands_this_tool_can_send": [p.pattern for p in ALLOWED],
        "servo_writes_performed": "NONE",
        "raw_pre_cold_hex": pre.hex().upper(),
        "raw_post_cold_hex": post.hex().upper(),
        "raw_pre_cold_sha256": sha256_bytes(pre),
        "raw_post_cold_sha256": sha256_bytes(post),
        "decoded_pre_cold": decode(pre),
        "decoded_post_cold": decode(post),
        "byte_differences": diffs,
        "persistent_differences": persistent,
        "checks": checks,
        "checks_total": len(checks),
        "checks_failed": len(failed),
        "verdict": ("%s POST-PROVISION PERSISTENCE AUDIT: PASS" % a.label
                    if not failed else "BLOCK"),
    }
    path = os.path.join(a.outdir, "persistence_audit_report.json")
    body = json.dumps(report, indent=2, sort_keys=True)
    with open(path, "w") as h:
        h.write(body + "\n")
        h.flush()
        os.fsync(h.fileno())
    print(body)
    print("\nREPORT: %s" % path)
    print("REPORT_SHA256: %s" % sha256_bytes(open(path, "rb").read()))
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.exit(main())
