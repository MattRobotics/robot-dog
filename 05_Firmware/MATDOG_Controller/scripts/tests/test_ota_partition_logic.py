#!/usr/bin/env python3
"""Offline tests for ota_partition_logic.py — no device I/O, no flash
writes, no hardware required. Run with:

    python3 -m unittest scripts.tests.test_ota_partition_logic -v

or directly:

    python3 scripts/tests/test_ota_partition_logic.py

Covers the scenarios required by the Session 2.1 hardening handoff:
current real device state, slot-0-only-valid, slot-1-only-valid, a CRC-
invalid entry (the exact case Session 2's parser could have mis-selected),
an unacceptable ota_state (INVALID/ABORTED), both entries blank, an
ambiguous CRC-invalid-on-both state, and a partition table missing the
computed slot.
"""
import struct
import sys
import unittest
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from ota_partition_logic import (  # noqa: E402
    OTA_STATE_ABORTED,
    OTA_STATE_INVALID,
    OtaAmbiguous,
    resolve_application_partition,
)

OTA_UNDEFINED = 0xFFFFFFFF
BLANK_SEQ = 0xFFFFFFFF
SECTOR_SIZE = 0x1000


def build_ota_sector(seq, state=OTA_UNDEFINED, crc=None, corrupt_crc=False):
    """Builds one 4KB otadata sector with a real esp_ota_select_entry_t at
    its start, padded to 4096 bytes with 0xFF like real flash."""
    seq_bytes = struct.pack("<I", seq)
    if crc is None:
        crc = zlib.crc32(seq_bytes, 0xFFFFFFFF) & 0xFFFFFFFF
    if corrupt_crc:
        crc ^= 0xDEADBEEF
    entry = seq_bytes + (b"\xff" * 20) + struct.pack("<I", state) + struct.pack("<I", crc)
    assert len(entry) == 32
    return entry.ljust(SECTOR_SIZE, b"\xff")


def build_blank_sector():
    return b"\xff" * SECTOR_SIZE


def build_partition_table(entries):
    """entries: list of (label, type, subtype, offset, size)."""
    data = b""
    for label, ptype, subtype, offset, size in entries:
        label_bytes = label.encode()[:16].ljust(16, b"\x00")
        entry = (
            b"\xaa\x50"
            + bytes([ptype, subtype])
            + struct.pack("<II", offset, size)
            + label_bytes
            + struct.pack("<I", 0)
        )
        assert len(entry) == 32
        data += entry
    data += b"\xeb\xeb" + b"\x00" * 30  # simplified MD5-marker end-of-table
    data = data.ljust(0x1000, b"\xff")
    return data


TWO_SLOT_TABLE = build_partition_table([
    ("nvs", 0x01, 0x02, 0x9000, 0x5000),
    ("otadata", 0x01, 0x00, 0xE000, 0x2000),
    ("app0", 0x00, 0x10, 0x10000, 0x300000),
    ("app1", 0x00, 0x11, 0x310000, 0x300000),
])


class TestOtaPartitionLogic(unittest.TestCase):
    def test_current_real_device_state(self):
        # Exact bytes/values captured from the real device this session:
        # OTADATA_SECTOR_SEQS=[1, 0], active=app0. Also matches the known
        # boot_app0.bin seed file byte-for-byte (see module docstring).
        otadata = build_ota_sector(1) + build_ota_sector(0)
        resolved = resolve_application_partition(TWO_SLOT_TABLE, otadata)
        self.assertEqual(resolved.label, "app0")
        self.assertEqual(resolved.offset, 0x10000)
        self.assertEqual(resolved.size, 0x300000)
        self.assertEqual(resolved.slot_index, 0)

    def test_slot_0_valid_slot_1_blank(self):
        otadata = build_ota_sector(1) + build_blank_sector()
        resolved = resolve_application_partition(TWO_SLOT_TABLE, otadata)
        self.assertEqual(resolved.label, "app0")
        self.assertEqual(resolved.slot_index, 0)

    def test_slot_1_valid_slot_0_blank(self):
        otadata = build_blank_sector() + build_ota_sector(2)  # (2-1)%2 == 1
        resolved = resolve_application_partition(TWO_SLOT_TABLE, otadata)
        self.assertEqual(resolved.label, "app1")
        self.assertEqual(resolved.slot_index, 1)

    def test_crc_invalid_entry_is_excluded_even_with_higher_seq(self):
        # The exact regression Session 2 did not guard against: a higher
        # raw ota_seq must NOT win if its CRC does not match. seq=9 (would
        # naively look "newer") has a corrupted CRC; seq=2 is the only
        # genuinely valid entry and must be the one selected.
        otadata = build_ota_sector(9, corrupt_crc=True) + build_ota_sector(2)
        resolved = resolve_application_partition(TWO_SLOT_TABLE, otadata)
        self.assertEqual(resolved.active_seq, 2)
        self.assertEqual(resolved.slot_index, (2 - 1) % 2)
        self.assertEqual(resolved.label, "app1")

    def test_unacceptable_ota_state_invalid_is_excluded(self):
        otadata = (
            build_ota_sector(7, state=OTA_STATE_INVALID)
            + build_ota_sector(8)
        )
        resolved = resolve_application_partition(TWO_SLOT_TABLE, otadata)
        self.assertEqual(resolved.active_seq, 8)
        self.assertEqual(resolved.label, "app1")

    def test_unacceptable_ota_state_aborted_is_excluded(self):
        otadata = (
            build_ota_sector(3)
            + build_ota_sector(4, state=OTA_STATE_ABORTED)
        )
        resolved = resolve_application_partition(TWO_SLOT_TABLE, otadata)
        self.assertEqual(resolved.active_seq, 3)
        self.assertEqual(resolved.label, "app0")

    def test_both_blank_refuses(self):
        otadata = build_blank_sector() + build_blank_sector()
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(TWO_SLOT_TABLE, otadata)
        self.assertIn("both otadata sectors are invalid", str(ctx.exception))

    def test_both_crc_invalid_is_ambiguous_refuses(self):
        # Both entries pass the coarse (blank/state) gate but neither's CRC
        # matches — the real bootloader has no deterministic winner here.
        otadata = (
            build_ota_sector(5, corrupt_crc=True)
            + build_ota_sector(6, corrupt_crc=True)
        )
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(TWO_SLOT_TABLE, otadata)
        self.assertIn("neither otadata sector passes", str(ctx.exception))

    def test_partition_table_missing_computed_slot_refuses(self):
        # Only ota_1 declared (no ota_0) — a valid single entry maps to
        # slot 0 via (seq-1) % app_count, but app_count == 1 here, so slot 0
        # is computed while only slot-index 1 exists in the table at all.
        one_slot_table = build_partition_table([
            ("nvs", 0x01, 0x02, 0x9000, 0x5000),
            ("otadata", 0x01, 0x00, 0xE000, 0x2000),
            ("app1", 0x00, 0x11, 0x310000, 0x300000),
        ])
        otadata = build_ota_sector(1) + build_blank_sector()
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(one_slot_table, otadata)
        self.assertIn("no matching ota_", str(ctx.exception))

    def test_no_ota_partitions_in_table_refuses(self):
        empty_table = build_partition_table([
            ("nvs", 0x01, 0x02, 0x9000, 0x5000),
        ])
        otadata = build_ota_sector(1) + build_blank_sector()
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(empty_table, otadata)
        self.assertIn("no ota_N app partitions", str(ctx.exception))


if __name__ == "__main__":
    unittest.main()
