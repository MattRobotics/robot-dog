#!/usr/bin/env python3
"""Offline tests for ota_partition_logic.py — no device I/O, no flash
writes, no hardware required. Run with:

    python3 -m unittest scripts.tests.test_ota_partition_logic -v

or directly:

    python3 scripts/tests/test_ota_partition_logic.py

Covers the scenarios required by the Session 2.1 hardening handoff (basic
selection: current real device state, slot-0/1-only-valid, a CRC-invalid
entry, an unacceptable ota_state, both entries blank, an ambiguous
CRC-invalid-on-both state, a partition table missing the computed slot)
plus the Session 2.2 findings:

  Finding A — OTA subtype recognition must use the real bitmask
  (subtype & 0xF0 == 0x10), not a `>= 0x10` threshold, so PART_SUBTYPE_TEST
  (0x20) and PART_SUBTYPE_TEE_0/1 (0x30/0x31) are never mistaken for OTA
  app slots.

  Finding B — when the real build has
  CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y, an otadata entry whose ota_state
  is NEW or PENDING_VERIFY must be refused (the bootloader can rewrite it
  on the very next boot); CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK=y must refuse
  unconditionally; both disabled must behave exactly like Session 2.1.

plus the Session 2.3 findings:

  Finding 2 — an sdkconfig symbol absent from the text entirely (neither
  its enabled nor its explicitly-disabled form found) must resolve to
  SdkconfigFlag.UNKNOWN, which resolve_application_partition() must REFUSE
  on, exactly like ENABLED — not silently treated as DISABLED.

  Finding 3 — OTA slot index sets must be exactly {0, ..., N-1}; any
  sparse set (a gap, or slots not starting at 0) must REFUSE.
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
    OTA_STATE_NEW,
    OTA_STATE_PENDING_VERIFY,
    OTA_STATE_VALID,
    OtaAmbiguous,
    SdkconfigFlag,
    ota_app_partitions,
    parse_partition_table,
    parse_sdkconfig_flag,
    parse_sdkconfig_ota_flags,
    resolve_application_partition,
)

OTA_UNDEFINED = 0xFFFFFFFF
BLANK_SEQ = 0xFFFFFFFF
SECTOR_SIZE = 0x1000

# Every call in this file is explicit about rollback/anti_rollback so
# nothing relies on a hidden default (there isn't one — both are
# required keyword-only parameters).
NO_ROLLBACK = dict(rollback=SdkconfigFlag.DISABLED, anti_rollback=SdkconfigFlag.DISABLED)


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
        resolved = resolve_application_partition(TWO_SLOT_TABLE, otadata, **NO_ROLLBACK)
        self.assertEqual(resolved.label, "app0")
        self.assertEqual(resolved.offset, 0x10000)
        self.assertEqual(resolved.size, 0x300000)
        self.assertEqual(resolved.slot_index, 0)

    def test_slot_0_valid_slot_1_blank(self):
        otadata = build_ota_sector(1) + build_blank_sector()
        resolved = resolve_application_partition(TWO_SLOT_TABLE, otadata, **NO_ROLLBACK)
        self.assertEqual(resolved.label, "app0")
        self.assertEqual(resolved.slot_index, 0)

    def test_slot_1_valid_slot_0_blank(self):
        otadata = build_blank_sector() + build_ota_sector(2)  # (2-1)%2 == 1
        resolved = resolve_application_partition(TWO_SLOT_TABLE, otadata, **NO_ROLLBACK)
        self.assertEqual(resolved.label, "app1")
        self.assertEqual(resolved.slot_index, 1)

    def test_crc_invalid_entry_is_excluded_even_with_higher_seq(self):
        # The exact regression Session 2 did not guard against: a higher
        # raw ota_seq must NOT win if its CRC does not match. seq=9 (would
        # naively look "newer") has a corrupted CRC; seq=2 is the only
        # genuinely valid entry and must be the one selected.
        otadata = build_ota_sector(9, corrupt_crc=True) + build_ota_sector(2)
        resolved = resolve_application_partition(TWO_SLOT_TABLE, otadata, **NO_ROLLBACK)
        self.assertEqual(resolved.active_seq, 2)
        self.assertEqual(resolved.slot_index, (2 - 1) % 2)
        self.assertEqual(resolved.label, "app1")

    def test_unacceptable_ota_state_invalid_is_excluded(self):
        otadata = (
            build_ota_sector(7, state=OTA_STATE_INVALID)
            + build_ota_sector(8)
        )
        resolved = resolve_application_partition(TWO_SLOT_TABLE, otadata, **NO_ROLLBACK)
        self.assertEqual(resolved.active_seq, 8)
        self.assertEqual(resolved.label, "app1")

    def test_unacceptable_ota_state_aborted_is_excluded(self):
        otadata = (
            build_ota_sector(3)
            + build_ota_sector(4, state=OTA_STATE_ABORTED)
        )
        resolved = resolve_application_partition(TWO_SLOT_TABLE, otadata, **NO_ROLLBACK)
        self.assertEqual(resolved.active_seq, 3)
        self.assertEqual(resolved.label, "app0")

    def test_both_blank_refuses(self):
        otadata = build_blank_sector() + build_blank_sector()
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(TWO_SLOT_TABLE, otadata, **NO_ROLLBACK)
        self.assertIn("both otadata sectors are invalid", str(ctx.exception))

    def test_both_crc_invalid_is_ambiguous_refuses(self):
        # Both entries pass the coarse (blank/state) gate but neither's CRC
        # matches — the real bootloader has no deterministic winner here.
        otadata = (
            build_ota_sector(5, corrupt_crc=True)
            + build_ota_sector(6, corrupt_crc=True)
        )
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(TWO_SLOT_TABLE, otadata, **NO_ROLLBACK)
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
            resolve_application_partition(one_slot_table, otadata, **NO_ROLLBACK)
        # ota_1 alone is itself non-contiguous (slot index {1} != {0}), so
        # this now refuses earlier, inside ota_app_partitions() — see
        # Finding 3. Either message is an acceptable REFUSE; assert on the
        # exception type only (already checked above) plus a substring that
        # is true for both possible reasons.
        self.assertTrue(
            "no matching ota_" in str(ctx.exception)
            or "not contiguous" in str(ctx.exception)
        )

    def test_no_ota_partitions_in_table_refuses(self):
        empty_table = build_partition_table([
            ("nvs", 0x01, 0x02, 0x9000, 0x5000),
        ])
        otadata = build_ota_sector(1) + build_blank_sector()
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(empty_table, otadata, **NO_ROLLBACK)
        self.assertIn("no ota_N app partitions", str(ctx.exception))


class TestOtaSubtypeRecognitionFindingA(unittest.TestCase):
    """PART_SUBTYPE_TEST (0x20) and PART_SUBTYPE_TEE_0/1 (0x30/0x31) are
    real, defined, non-OTA app subtypes in the exact same header. Session
    2.1's `subtype >= 0x10` filter wrongly matched them; the fix is the
    real ESP-IDF bitmask (subtype & 0xF0) == 0x10."""

    def test_ota_0_and_ota_1_recognized(self):
        entries = parse_partition_table(TWO_SLOT_TABLE)
        apps = ota_app_partitions(entries)
        self.assertEqual(set(apps.keys()), {0, 1})
        self.assertEqual(apps[0].label, "app0")
        self.assertEqual(apps[1].label, "app1")

    def test_test_subtype_does_not_count_as_a_third_ota_slot(self):
        table = build_partition_table([
            ("app0", 0x00, 0x10, 0x10000, 0x300000),
            ("app1", 0x00, 0x11, 0x310000, 0x300000),
            ("test", 0x00, 0x20, 0x610000, 0x100000),  # PART_SUBTYPE_TEST
        ])
        apps = ota_app_partitions(parse_partition_table(table))
        self.assertEqual(set(apps.keys()), {0, 1})

    def test_tee_subtypes_do_not_count_as_ota_slots(self):
        table = build_partition_table([
            ("app0", 0x00, 0x10, 0x10000, 0x300000),
            ("app1", 0x00, 0x11, 0x310000, 0x300000),
            ("tee_0", 0x00, 0x30, 0x610000, 0x40000),  # PART_SUBTYPE_TEE_0
            ("tee_1", 0x00, 0x31, 0x650000, 0x40000),  # PART_SUBTYPE_TEE_1
        ])
        apps = ota_app_partitions(parse_partition_table(table))
        self.assertEqual(set(apps.keys()), {0, 1})

    def test_test_subtype_alone_gives_app_count_ota_of_one_not_two(self):
        # Regression case for the exact bug: `subtype >= 0x10` would have
        # matched PART_SUBTYPE_TEST (0x20) as if it were ota_16.
        table = build_partition_table([
            ("app0", 0x00, 0x10, 0x10000, 0x300000),
            ("test", 0x00, 0x20, 0x610000, 0x100000),
        ])
        apps = ota_app_partitions(parse_partition_table(table))
        self.assertEqual(set(apps.keys()), {0})
        self.assertEqual(len(apps), 1)

    def test_test_and_tee_without_any_ota_slot_finds_none(self):
        table = build_partition_table([
            ("test", 0x00, 0x20, 0x610000, 0x100000),
            ("tee_0", 0x00, 0x30, 0x650000, 0x40000),
        ])
        apps = ota_app_partitions(parse_partition_table(table))
        self.assertEqual(apps, {})
        otadata = build_ota_sector(1) + build_blank_sector()
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(table, otadata, **NO_ROLLBACK)
        self.assertIn("no ota_N app partitions", str(ctx.exception))

    def test_duplicate_ota_slot_index_refuses(self):
        # Two entries both claiming subtype 0x10 (ota_0) is a corrupt/
        # ambiguous table, not something to silently pick one of.
        table = build_partition_table([
            ("app0a", 0x00, 0x10, 0x10000, 0x300000),
            ("app0b", 0x00, 0x10, 0x310000, 0x300000),
        ])
        with self.assertRaises(OtaAmbiguous) as ctx:
            ota_app_partitions(parse_partition_table(table))
        self.assertIn("duplicate OTA slot index", str(ctx.exception))


class TestOtaSlotContiguityFinding3(unittest.TestCase):
    """MATDOG has no use for a sparse OTA layout in V0.1 — every slot
    index set must be exactly {0, ..., N-1}."""

    def test_single_slot_0_accepted(self):
        table = build_partition_table([("app0", 0x00, 0x10, 0x10000, 0x300000)])
        apps = ota_app_partitions(parse_partition_table(table))
        self.assertEqual(set(apps.keys()), {0})

    def test_two_slots_0_1_accepted(self):
        apps = ota_app_partitions(parse_partition_table(TWO_SLOT_TABLE))
        self.assertEqual(set(apps.keys()), {0, 1})

    def test_three_slots_0_1_2_accepted(self):
        table = build_partition_table([
            ("app0", 0x00, 0x10, 0x10000, 0x300000),
            ("app1", 0x00, 0x11, 0x310000, 0x300000),
            ("app2", 0x00, 0x12, 0x610000, 0x300000),
        ])
        apps = ota_app_partitions(parse_partition_table(table))
        self.assertEqual(set(apps.keys()), {0, 1, 2})

    def test_slot_1_only_refuses(self):
        table = build_partition_table([("app1", 0x00, 0x11, 0x10000, 0x300000)])
        with self.assertRaises(OtaAmbiguous) as ctx:
            ota_app_partitions(parse_partition_table(table))
        self.assertIn("not contiguous", str(ctx.exception))

    def test_slots_0_2_gap_refuses(self):
        table = build_partition_table([
            ("app0", 0x00, 0x10, 0x10000, 0x300000),
            ("app2", 0x00, 0x12, 0x610000, 0x300000),
        ])
        with self.assertRaises(OtaAmbiguous) as ctx:
            ota_app_partitions(parse_partition_table(table))
        self.assertIn("not contiguous", str(ctx.exception))

    def test_slots_1_2_refuses(self):
        table = build_partition_table([
            ("app1", 0x00, 0x11, 0x10000, 0x300000),
            ("app2", 0x00, 0x12, 0x310000, 0x300000),
        ])
        with self.assertRaises(OtaAmbiguous) as ctx:
            ota_app_partitions(parse_partition_table(table))
        self.assertIn("not contiguous", str(ctx.exception))

    def test_slots_0_1_3_gap_refuses(self):
        table = build_partition_table([
            ("app0", 0x00, 0x10, 0x10000, 0x300000),
            ("app1", 0x00, 0x11, 0x310000, 0x300000),
            ("app3", 0x00, 0x13, 0x610000, 0x300000),
        ])
        with self.assertRaises(OtaAmbiguous) as ctx:
            ota_app_partitions(parse_partition_table(table))
        self.assertIn("not contiguous", str(ctx.exception))


class TestRollbackAwarenessFindingB(unittest.TestCase):
    """CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y (the real, confirmed setting
    of this project's build) means the bootloader can autonomously rewrite
    an otadata entry whose state is NEW or PENDING_VERIFY on the very next
    boot. A read-only snapshot cannot treat those as stable."""

    def test_rollback_disabled_new_state_is_fine(self):
        # Matches Session 2.1 behaviour exactly when rollback is off.
        otadata = build_ota_sector(1, state=OTA_STATE_NEW) + build_blank_sector()
        resolved = resolve_application_partition(
            TWO_SLOT_TABLE, otadata,
            rollback=SdkconfigFlag.DISABLED, anti_rollback=SdkconfigFlag.DISABLED)
        self.assertEqual(resolved.label, "app0")

    def test_rollback_enabled_new_state_refuses(self):
        otadata = build_ota_sector(1, state=OTA_STATE_NEW) + build_blank_sector()
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(
                TWO_SLOT_TABLE, otadata,
                rollback=SdkconfigFlag.ENABLED, anti_rollback=SdkconfigFlag.DISABLED)
        self.assertIn("ROLLBACK_ENABLE", str(ctx.exception))

    def test_rollback_enabled_pending_verify_state_refuses(self):
        otadata = build_ota_sector(1, state=OTA_STATE_PENDING_VERIFY) + build_blank_sector()
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(
                TWO_SLOT_TABLE, otadata,
                rollback=SdkconfigFlag.ENABLED, anti_rollback=SdkconfigFlag.DISABLED)
        self.assertIn("ROLLBACK_ENABLE", str(ctx.exception))

    def test_rollback_enabled_undefined_state_is_fine(self):
        # The real, current, repeatedly-reverified state of the actual
        # MATDOG device this session: ota_state=UNDEFINED on the active
        # entry. UNDEFINED is not NEW/PENDING_VERIFY, so it is stable even
        # with rollback enabled - the bootloader never autonomously
        # rewrites it. Matches the live verify_application_partition.py
        # run against the real hardware this session.
        otadata = build_ota_sector(1, state=OTA_UNDEFINED) + build_ota_sector(0, state=OTA_UNDEFINED)
        resolved = resolve_application_partition(
            TWO_SLOT_TABLE, otadata,
            rollback=SdkconfigFlag.ENABLED, anti_rollback=SdkconfigFlag.DISABLED)
        self.assertEqual(resolved.label, "app0")

    def test_rollback_enabled_valid_state_is_fine(self):
        # VALID is the "confirmed, bootloader will never rewrite it" state
        # - also stable regardless of rollback being enabled.
        otadata = build_ota_sector(1, state=OTA_STATE_VALID) + build_blank_sector()
        resolved = resolve_application_partition(
            TWO_SLOT_TABLE, otadata,
            rollback=SdkconfigFlag.ENABLED, anti_rollback=SdkconfigFlag.DISABLED)
        self.assertEqual(resolved.label, "app0")

    def test_anti_rollback_enabled_always_refuses_even_with_stable_state(self):
        # Anti-rollback introduces secure_version/eFuse semantics this tool
        # does not implement at all - refuse unconditionally, regardless of
        # how clean the otadata itself looks.
        otadata = build_ota_sector(1, state=OTA_STATE_VALID) + build_blank_sector()
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(
                TWO_SLOT_TABLE, otadata,
                rollback=SdkconfigFlag.ENABLED, anti_rollback=SdkconfigFlag.ENABLED)
        self.assertIn("ANTI_ROLLBACK", str(ctx.exception))

    def test_anti_rollback_enabled_refuses_even_with_rollback_disabled(self):
        otadata = build_ota_sector(1, state=OTA_STATE_VALID) + build_blank_sector()
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(
                TWO_SLOT_TABLE, otadata,
                rollback=SdkconfigFlag.DISABLED, anti_rollback=SdkconfigFlag.ENABLED)
        self.assertIn("ANTI_ROLLBACK", str(ctx.exception))


class TestSdkconfigUnknownFinding2(unittest.TestCase):
    """Session 2.3 Finding 2: a symbol absent from the sdkconfig text
    entirely is UNKNOWN, not DISABLED, and resolve_application_partition()
    must REFUSE on UNKNOWN exactly like it does on ENABLED."""

    STABLE_OTADATA = None  # set below; state=VALID, rollback-stable regardless

    @classmethod
    def setUpClass(cls):
        cls.STABLE_OTADATA = build_ota_sector(1, state=OTA_STATE_VALID) + build_blank_sector()

    def test_rollback_symbol_absent_is_unknown(self):
        flag = parse_sdkconfig_flag("CONFIG_SOMETHING_ELSE=y\n",
                                     "CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE")
        self.assertEqual(flag, SdkconfigFlag.UNKNOWN)

    def test_anti_rollback_symbol_absent_is_unknown(self):
        flag = parse_sdkconfig_flag("CONFIG_SOMETHING_ELSE=y\n",
                                     "CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK")
        self.assertEqual(flag, SdkconfigFlag.UNKNOWN)

    def test_both_symbols_absent_resolve_to_unknown(self):
        rollback, anti = parse_sdkconfig_ota_flags("CONFIG_SOMETHING_ELSE=y\n")
        self.assertEqual(rollback, SdkconfigFlag.UNKNOWN)
        self.assertEqual(anti, SdkconfigFlag.UNKNOWN)

    def test_rollback_unknown_refuses_even_with_stable_otadata(self):
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(
                TWO_SLOT_TABLE, self.STABLE_OTADATA,
                rollback=SdkconfigFlag.UNKNOWN, anti_rollback=SdkconfigFlag.DISABLED)
        self.assertIn("does not explicitly define", str(ctx.exception))
        self.assertIn("ROLLBACK_ENABLE", str(ctx.exception))

    def test_anti_rollback_unknown_refuses_even_with_stable_otadata(self):
        with self.assertRaises(OtaAmbiguous) as ctx:
            resolve_application_partition(
                TWO_SLOT_TABLE, self.STABLE_OTADATA,
                rollback=SdkconfigFlag.DISABLED, anti_rollback=SdkconfigFlag.UNKNOWN)
        self.assertIn("does not explicitly define", str(ctx.exception))
        self.assertIn("ANTI_ROLLBACK", str(ctx.exception))

    def test_both_unknown_refuses(self):
        with self.assertRaises(OtaAmbiguous):
            resolve_application_partition(
                TWO_SLOT_TABLE, self.STABLE_OTADATA,
                rollback=SdkconfigFlag.UNKNOWN, anti_rollback=SdkconfigFlag.UNKNOWN)


class TestParseSdkconfigOtaFlags(unittest.TestCase):
    def test_both_disabled(self):
        text = (
            "# CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE is not set\n"
            "# CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK is not set\n"
        )
        rollback, anti = parse_sdkconfig_ota_flags(text)
        self.assertEqual(rollback, SdkconfigFlag.DISABLED)
        self.assertEqual(anti, SdkconfigFlag.DISABLED)

    def test_rollback_enabled_anti_rollback_disabled(self):
        # The real, confirmed content of this project's actual build sdkconfig.
        text = (
            "CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y\n"
            "# CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK is not set\n"
        )
        rollback, anti = parse_sdkconfig_ota_flags(text)
        self.assertEqual(rollback, SdkconfigFlag.ENABLED)
        self.assertEqual(anti, SdkconfigFlag.DISABLED)

    def test_both_enabled(self):
        text = (
            "CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y\n"
            "CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK=y\n"
        )
        rollback, anti = parse_sdkconfig_ota_flags(text)
        self.assertEqual(rollback, SdkconfigFlag.ENABLED)
        self.assertEqual(anti, SdkconfigFlag.ENABLED)

    def test_flags_absent_entirely_is_unknown_not_disabled(self):
        # Session 2.3 Finding 2: this used to assert both were falsy
        # (silently treated as disabled). A missing symbol is not the same
        # fact as a confirmed-disabled one - it must be UNKNOWN.
        rollback, anti = parse_sdkconfig_ota_flags("CONFIG_SOMETHING_ELSE=y\n")
        self.assertEqual(rollback, SdkconfigFlag.UNKNOWN)
        self.assertEqual(anti, SdkconfigFlag.UNKNOWN)


if __name__ == "__main__":
    unittest.main()
