#!/usr/bin/env python3
"""Read-only verification of the ESP32-S3's currently active application
partition, used to derive a trustworthy write offset for
scripts/flash_app_only.sh.

This exists because MATDOG_Controller's partition scheme
(app3M_fat9M_16MB) is a dual-OTA-slot layout (app0/ota_0 and app1/ota_1),
not a single fixed application region. Writing "the application" correctly
means writing whichever slot the device's own otadata currently selects,
not assuming ota_0 just because that happened to be true in Session 1.

Method (matches the standard ESP-IDF OTA-select format, cross-checked
against `boot_app0.bin` from the installed esp32 Arduino core — see
SOURCE_PROVENANCE.md for the worked-out reference values):

  1. Read the partition table at 0x8000 (one 4KB sector) and parse every
     32-byte entry (magic 0xAA50) until the MD5-checksum entry (0xEBEB) or
     0xFF padding. Extract the app0/ota_0 and app1/ota_1 entries.
  2. Read otadata at 0xe000 (two 4KB sectors). Each sector's first 4 bytes
     are a uint32 ota_seq (0xFFFFFFFF = blank/erased); the "active" sector
     is the one with the higher seq (both blank => nothing has explicitly
     selected a slot yet, which this script treats as ambiguous and
     refuses, since Session 1 always wrote a seed that selects one).
  3. active_slot_index = (active_seq - 1) % ota_app_count.
  4. Emit the matching partition's offset/size as APPLICATION_OFFSET /
     APPLICATION_PARTITION_SIZE, plus the raw facts used to get there.

Never writes anything. Exits non-zero with a clear message on any
ambiguity — callers must not guess past that.
"""
import argparse
import struct
import subprocess
import sys
import tempfile
from pathlib import Path


PARTITION_ENTRY_MAGIC = b"\xaa\x50"
PARTITION_TABLE_END_MAGIC = b"\xeb\xeb"
PARTITION_TABLE_OFFSET = 0x8000
PARTITION_TABLE_SIZE = 0x1000
OTADATA_OFFSET = 0xE000
OTADATA_SIZE = 0x2000
OTADATA_SECTOR_SIZE = 0x1000

APP_TYPE = 0x00
OTA_SUBTYPE_BASE = 0x10  # ota_0 = 0x10, ota_1 = 0x11, ...


def esptool_read(esptool, chip, port, offset, length, out_path):
    result = subprocess.run(
        [esptool, "--chip", chip, "--port", port, "read-flash",
         hex(offset), hex(length), str(out_path)],
        capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(result.stdout)
        print(result.stderr, file=sys.stderr)
        raise SystemExit(f"esptool read-flash failed at offset {hex(offset)}")


def parse_partition_table(data: bytes):
    entries = []
    off = 0
    while off + 32 <= len(data):
        entry = data[off:off + 32]
        magic = entry[0:2]
        if magic == PARTITION_ENTRY_MAGIC:
            ptype, subtype = entry[2], entry[3]
            part_offset, part_size = struct.unpack_from("<II", entry, 4)
            label = entry[12:28].split(b"\x00")[0].decode(errors="replace")
            entries.append({
                "label": label, "type": ptype, "subtype": subtype,
                "offset": part_offset, "size": part_size,
            })
            off += 32
        elif magic == PARTITION_TABLE_END_MAGIC or entry == b"\xff" * 32:
            break
        else:
            raise SystemExit(f"unrecognized partition table entry at 0x{off:x}: {entry.hex()}")
    return entries


def parse_otadata(data: bytes):
    seqs = []
    for sector in range(len(data) // OTADATA_SECTOR_SIZE):
        off = sector * OTADATA_SECTOR_SIZE
        seq = struct.unpack_from("<I", data, off)[0]
        seqs.append(seq)
    return seqs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--chip", default="esp32s3")
    ap.add_argument("--esptool", required=True)
    args = ap.parse_args()

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        part_bin = tmp / "partitions.bin"
        ota_bin = tmp / "otadata.bin"

        esptool_read(args.esptool, args.chip, args.port,
                     PARTITION_TABLE_OFFSET, PARTITION_TABLE_SIZE, part_bin)
        esptool_read(args.esptool, args.chip, args.port,
                     OTADATA_OFFSET, OTADATA_SIZE, ota_bin)

        entries = parse_partition_table(part_bin.read_bytes())
        seqs = parse_otadata(ota_bin.read_bytes())

    ota_entries = {
        e["subtype"] - OTA_SUBTYPE_BASE: e
        for e in entries
        if e["type"] == APP_TYPE and e["subtype"] >= OTA_SUBTYPE_BASE
    }
    if not ota_entries:
        raise SystemExit("REFUSE: no ota_N app partitions found in the device partition table")

    ota_app_count = len(ota_entries)
    blank = 0xFFFFFFFF
    non_blank = [(i, s) for i, s in enumerate(seqs) if s != blank]

    if not non_blank:
        raise SystemExit(
            "REFUSE: otadata is fully blank/erased — no slot has ever been explicitly "
            "selected. Refusing to guess; this needs a normal full Arduino upload first "
            "(which legitimately initializes otadata), not an application-only write."
        )

    active_sector_index, active_seq = max(non_blank, key=lambda pair: pair[1])
    active_slot_index = (active_seq - 1) % ota_app_count

    if active_slot_index not in ota_entries:
        raise SystemExit(
            f"REFUSE: computed active slot index {active_slot_index} has no matching "
            f"ota_{active_slot_index} partition in the table — ambiguous, refusing to guess."
        )

    active = ota_entries[active_slot_index]

    print(f"OTADATA_SECTOR_SEQS={seqs}")
    print(f"OTADATA_ACTIVE_SECTOR={active_sector_index}")
    print(f"OTADATA_ACTIVE_SEQ={active_seq}")
    print(f"ACTIVE_OTA_SLOT_INDEX={active_slot_index}")
    print(f"ACTIVE_PARTITION_LABEL={active['label']}")
    print(f"APPLICATION_OFFSET=0x{active['offset']:06x}")
    print(f"APPLICATION_PARTITION_SIZE={active['size']}")

    for i in sorted(ota_entries):
        e = ota_entries[i]
        print(f"# known slot ota_{i}: label={e['label']} offset=0x{e['offset']:06x} "
              f"size={e['size']}", file=sys.stderr)


if __name__ == "__main__":
    main()
