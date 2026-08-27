#!/usr/bin/env python3

import argparse
import re
import serial
import struct
import sys
import time
from datetime import datetime
from pathlib import Path

DEFAULT_PORT = (
    "/dev/serial/by-id/"
    "usb-Espressif_USB_JTAG_serial_debug_unit_"
    "14:C1:9F:22:75:94-if00"
)

def open_port(path):
    ser = serial.Serial()
    ser.port = path
    ser.baudrate = 115200
    ser.timeout = 3
    ser.write_timeout = 3

    # Request inactive handshake lines before opening.
    ser.dtr = False
    ser.rts = False
    ser.open()

    # Opening may reset USB-CDC once. That is acceptable BEFORE QC.
    time.sleep(2.2)
    ser.reset_input_buffer()
    return ser

def send(ser, text):
    ser.write((text + "\n").encode("ascii"))
    ser.flush()

def read_until(ser, terminators, timeout):
    deadline = time.monotonic() + timeout
    lines = []

    while time.monotonic() < deadline:
        raw = ser.readline()

        if not raw:
            continue

        line = raw.decode("utf-8", errors="replace").strip()

        if not line:
            continue

        print(line, flush=True)
        lines.append(line)

        if any(t in line for t in terminators):
            return lines

    raise TimeoutError(
        f"Timeout waiting for {terminators}"
    )

def do_scan(ser, lo, hi):
    send(ser, f"@SCAN {lo} {hi}")
    return read_until(
        ser,
        ["SCAN_COMPLETE"],
        timeout=max(20, (hi - lo + 1) * 0.15 + 5)
    )

def do_read(ser, sid):
    send(ser, f"@READ {sid}")
    return read_until(
        ser,
        ["READ_COMPLETE", "READ_ABORTED"],
        timeout=5
    )

def safe_off(ser, sid):
    send(ser, f"@SAFE_OFF {sid}")

    lines = read_until(
        ser,
        ["SAFE_OFF_RESULT"],
        timeout=5
    )

    if not any(
        "SAFE_OFF_RESULT" in x and "PASS" in x
        for x in lines
    ):
        raise RuntimeError(
            "SAFE_OFF failed — DO NOT RUN MOTION"
        )

def fnv1a32(data):
    h = 2166136261

    for b in data:
        h ^= b
        h = (h * 16777619) & 0xFFFFFFFF

    return h

def dump_raw(ser, sid, outdir):
    send(ser, f"@DUMP_RAW {sid}")

    begin = None
    deadline = time.monotonic() + 10

    while time.monotonic() < deadline:
        raw = ser.readline()

        if not raw:
            continue

        line = raw.decode(
            "ascii",
            errors="replace"
        ).strip()

        if line:
            print(line, flush=True)

        if line.startswith("RAW_DUMP_ABORT"):
            raise RuntimeError(line)

        if line.startswith("RAW_DUMP_BEGIN "):
            begin = line
            break

    if begin is None:
        raise RuntimeError(
            "RAW_DUMP_BEGIN not received"
        )

    m = re.search(
        r"payload=(\d+)\s+"
        r"header=(\d+)\s+"
        r"samples=(\d+)\s+"
        r"sample_size=(\d+)\s+"
        r"raw_fnv=0x([0-9A-Fa-f]+)",
        begin
    )

    if not m:
        raise RuntimeError(
            f"Malformed dump header: {begin}"
        )

    payload_bytes = int(m.group(1))
    header_bytes = int(m.group(2))
    sample_count = int(m.group(3))
    sample_size = int(m.group(4))
    expected_fnv = int(m.group(5), 16)

    payload = bytearray()

    while len(payload) < payload_bytes:
        chunk = ser.read(
            payload_bytes - len(payload)
        )

        if not chunk:
            raise TimeoutError(
                f"RAW timeout "
                f"{len(payload)}/{payload_bytes}"
            )

        payload.extend(chunk)

    HEADER_FMT = "<8s7I48s"
    expected_header_size = struct.calcsize(
        HEADER_FMT
    )

    if header_bytes != expected_header_size:
        raise RuntimeError(
            f"Header size mismatch: "
            f"ESP={header_bytes} "
            f"Python={expected_header_size}"
        )

    header = struct.unpack_from(
        HEADER_FMT,
        payload,
        0
    )

    (
        magic,
        version,
        servo_id,
        sample_hz,
        hdr_sample_size,
        hdr_sample_count,
        elapsed_us,
        result_code,
        abort_raw,
    ) = header

    if servo_id != sid:
        raise RuntimeError(
            f"Servo ID mismatch: {servo_id} != {sid}"
        )

    if hdr_sample_size != sample_size:
        raise RuntimeError(
            "Sample size mismatch"
        )

    if hdr_sample_count != sample_count:
        raise RuntimeError(
            "Sample count mismatch"
        )

    raw_samples = bytes(payload[header_bytes:])
    actual_fnv = fnv1a32(raw_samples)

    abort_reason = abort_raw.split(
        b"\0",
        1
    )[0].decode(
        "ascii",
        errors="replace"
    )

    stamp = datetime.now().strftime(
        "%Y%m%d_%H%M%S"
    )

    outdir.mkdir(
        parents=True,
        exist_ok=True
    )

    path = outdir / (
        f"qc_fast_id{sid}_{stamp}.bin"
    )

    path.write_bytes(payload)

    print()
    print("===== RAW SAVED =====")
    print("FILE          :", path)
    print("MAGIC         :", magic)
    print("SERVO ID      :", servo_id)
    print("SAMPLE HZ     :", sample_hz)
    print("SAMPLES       :", sample_count)
    print("SAMPLE SIZE   :", sample_size)
    print("ELAPSED US    :", elapsed_us)
    print("RESULT CODE   :", result_code)
    print("ABORT REASON  :", abort_reason)
    print(
        "FNV ESP32     :",
        f"0x{expected_fnv:08X}"
    )
    print(
        "FNV ASUS      :",
        f"0x{actual_fnv:08X}"
    )
    print(
        "CHECKSUM      :",
        "PASS"
        if actual_fnv == expected_fnv
        else "FAIL"
    )

    if actual_fnv != expected_fnv:
        raise RuntimeError(
            "RAW checksum mismatch"
        )

    return path

def do_qc(ser, sid, confirm, outdir):
    required = f"QC_FAST_ID_{sid}"

    if confirm != required:
        raise SystemExit(
            "Motion confirmation mismatch.\n"
            f"Required exactly: {required}"
        )

    print(
        f"Safety precheck: forcing torque OFF on ID {sid}"
    )

    safe_off(ser, sid)

    print()
    print(
        f"Starting authorized QC_FAST on ID {sid}"
    )

    send(ser, f"@QC_FAST {sid}")

    lines = read_until(
        ser,
        ["QC_FAST_COMPLETE"],
        timeout=190
    )

    if not any(
        "FINAL_TORQUE" in x and ": 0" in x
        for x in lines
    ):
        raise RuntimeError(
            "FINAL_TORQUE=0 not confirmed. "
            "CUT SERVO POWER."
        )

    raw_available = any(
        "RAW_AVAILABLE" in x and "YES" in x
        for x in lines
    )

    if not raw_available:
        raise RuntimeError(
            "QC ended without retained RAW dataset"
        )

    print()
    print(
        "Torque OFF confirmed; downloading RAW "
        "without reopening serial port..."
    )

    return dump_raw(
        ser,
        sid,
        outdir
    )

def main():
    ap = argparse.ArgumentParser()

    ap.add_argument(
        "--port",
        default=DEFAULT_PORT
    )

    mode = ap.add_mutually_exclusive_group(
        required=True
    )

    mode.add_argument(
        "--scan",
        nargs=2,
        type=int,
        metavar=("MIN", "MAX")
    )

    mode.add_argument(
        "--read",
        type=int,
        metavar="ID"
    )

    mode.add_argument(
        "--qc",
        type=int,
        metavar="ID"
    )

    ap.add_argument(
        "--confirm",
        default=""
    )

    ap.add_argument(
        "--outdir",
        type=Path,
        default=(
            Path.home()
            / "MATDOG/runtime/esp32/qc_raw"
        )
    )

    args = ap.parse_args()

    ser = open_port(args.port)

    try:
        if args.scan:
            lo, hi = args.scan

            if not (
                0 <= lo <= hi <= 253
            ):
                raise SystemExit(
                    "SCAN IDs must be 0..253"
                )

            do_scan(ser, lo, hi)

        elif args.read is not None:
            if not 0 <= args.read <= 253:
                raise SystemExit(
                    "ID must be 0..253"
                )

            do_read(ser, args.read)

        elif args.qc is not None:
            if not 0 <= args.qc <= 253:
                raise SystemExit(
                    "ID must be 0..253"
                )

            path = do_qc(
                ser,
                args.qc,
                args.confirm,
                args.outdir
            )

            print()
            print("QC + RAW transfer complete:")
            print(path)

    finally:
        ser.close()

if __name__ == "__main__":
    main()
