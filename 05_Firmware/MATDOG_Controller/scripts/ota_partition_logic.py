#!/usr/bin/env python3
"""Pure, offline-testable logic for selecting the ESP32-S3's currently
active OTA application partition from a read-only device dump.

SESSION 2.1 CORRECTION: Session 2's verify_application_partition.py picked
whichever otadata sector had the numerically higher `ota_seq` field among
any "non-blank" (seq != 0xFFFFFFFF) sectors, and mislabeled the struct's
`ota_state` field as `crc`. It never validated the real CRC field at all.
That is not what the installed bootloader actually does, and it could in
principle select a slot the real bootloader would refuse to boot (e.g. one
marked ESP_OTA_IMG_INVALID/ABORTED, or one whose CRC does not match).

This module replicates the REAL algorithm, read from the exact source of
the exact ESP-IDF version bundled with the installed arduino-esp32 3.3.11
core (ESP-IDF v5.5.5, commit b774170ff46 — see versions.txt in the esp32
core package):

  struct layout — components/bootloader_support/include/esp_flash_partitions.h
  (esp_ota_select_entry_t, 32 bytes):
      offset  0: ota_seq    uint32 LE
      offset  4: seq_label  uint8[20]           (unused here)
      offset 24: ota_state  uint32 LE            (esp_ota_img_states_t)
      offset 28: crc        uint32 LE            (CRC32 of the ota_seq field only)

  esp_ota_img_states_t — same header:
      0 NEW, 1 PENDING_VERIFY, 2 VALID, 3 INVALID, 4 ABORTED, 0xFFFFFFFF UNDEFINED

  validity / selection — components/bootloader_support/src/bootloader_common_loader.c
  (bootloader_common_ota_select_crc / _invalid / _valid / get_active_otadata /
  select_otadata) and bootloader_utility.c (the seq -> slot-index mapping):

      crc(entry)     = crc32(entry.ota_seq_bytes, init=0xFFFFFFFF)   [see below]
      invalid(entry) = entry.ota_seq == 0xFFFFFFFF
                       or entry.ota_state in {INVALID, ABORTED}
      valid(entry)   = not invalid(entry) and entry.crc == crc(entry)

      if both entries invalid (ignoring crc): NO deterministic selection —
        the real bootloader falls back to a factory partition (MATDOG's
        table has none) or a special first-boot ota_0 initialization path.
        This script treats that as ambiguous and refuses.
      elif both valid: pick the one with the larger ota_seq (tie -> index 0)
      elif exactly one valid: pick that one
      else (neither valid, i.e. both fail the CRC check despite passing the
        coarser invalid() gate): no deterministic selection -> refuse

      slot_index = (winning_entry.ota_seq - 1) % ota_app_count
        (bootloader_utility.c, get_selected_boot_partition /
         get_active_otadata_with_check_anti_rollback)

  CRC32 variant — verified empirically against the real, known-good
  boot_app0.bin seed file shipped with this exact core (which the installed
  bootloader accepts and boots from — see SOURCE_PROVENANCE.md): the stored
  crc field for both of its otadata sectors matches
  `zlib.crc32(ota_seq_bytes, 0xFFFFFFFF) & 0xFFFFFFFF` (Python's zlib.crc32
  called with a custom starting value, no additional inversion). This
  matches esp_rom_crc.h's documented usage pattern for esp_rom_crc32_le
  (raw accumulator; callers invert only when they want the fully-finalized
  standard CRC32, which bootloader_common_ota_select_crc does not do).

This module performs NO device I/O and writes nothing. See
scripts/verify_application_partition.py for the device-facing wrapper and
scripts/tests/test_ota_partition_logic.py for the offline test suite.
"""
import struct
import zlib
from dataclasses import dataclass, field
from typing import Dict, List, Optional

PARTITION_ENTRY_MAGIC = b"\xaa\x50"
PARTITION_TABLE_END_MAGIC = b"\xeb\xeb"

APP_TYPE = 0x00
OTA_SUBTYPE_BASE = 0x10  # ota_0 = 0x10, ota_1 = 0x11, ...

OTA_STATE_INVALID = 3
OTA_STATE_ABORTED = 4
BLANK_SEQ = 0xFFFFFFFF

OTADATA_SECTOR_SIZE = 0x1000
OTA_SELECT_ENTRY_SIZE = 32


class OtaAmbiguous(Exception):
    """Raised whenever the real bootloader's rule does not deterministically
    select a slot. Callers must REFUSE, not guess, on this."""


@dataclass
class PartitionEntry:
    label: str
    type: int
    subtype: int
    offset: int
    size: int


@dataclass
class OtaSelectEntry:
    ota_seq: int
    ota_state: int
    crc_stored: int
    sector_index: int  # 0 or 1 — which of the two otadata sectors this came from

    def crc_expected(self) -> int:
        seq_bytes = struct.pack("<I", self.ota_seq)
        return zlib.crc32(seq_bytes, 0xFFFFFFFF) & 0xFFFFFFFF

    def is_invalid(self) -> bool:
        """bootloader_common_ota_select_invalid — ignores CRC."""
        return self.ota_seq == BLANK_SEQ or self.ota_state in (OTA_STATE_INVALID, OTA_STATE_ABORTED)

    def is_valid(self) -> bool:
        """bootloader_common_ota_select_valid — full check including CRC."""
        return (not self.is_invalid()) and self.crc_stored == self.crc_expected()


def parse_partition_table(data: bytes) -> List[PartitionEntry]:
    entries = []
    off = 0
    while off + 32 <= len(data):
        entry = data[off:off + 32]
        magic = entry[0:2]
        if magic == PARTITION_ENTRY_MAGIC:
            ptype, subtype = entry[2], entry[3]
            part_offset, part_size = struct.unpack_from("<II", entry, 4)
            label = entry[12:28].split(b"\x00")[0].decode(errors="replace")
            entries.append(PartitionEntry(label, ptype, subtype, part_offset, part_size))
            off += 32
        elif magic == PARTITION_TABLE_END_MAGIC or entry == b"\xff" * 32:
            break
        else:
            raise OtaAmbiguous(f"unrecognized partition table entry at 0x{off:x}: {entry.hex()}")
    return entries


def parse_otadata(data: bytes) -> List[OtaSelectEntry]:
    """Parses both 4KB otadata sectors into their 32-byte select entries."""
    entries = []
    n_sectors = len(data) // OTADATA_SECTOR_SIZE
    for sector in range(n_sectors):
        off = sector * OTADATA_SECTOR_SIZE
        ota_seq = struct.unpack_from("<I", data, off)[0]
        ota_state = struct.unpack_from("<I", data, off + 24)[0]
        crc_stored = struct.unpack_from("<I", data, off + 28)[0]
        entries.append(OtaSelectEntry(ota_seq, ota_state, crc_stored, sector))
    return entries


def ota_app_partitions(entries: List[PartitionEntry]) -> Dict[int, PartitionEntry]:
    """Maps ota slot index (0, 1, ...) -> its PartitionEntry."""
    return {
        e.subtype - OTA_SUBTYPE_BASE: e
        for e in entries
        if e.type == APP_TYPE and e.subtype >= OTA_SUBTYPE_BASE
    }


def select_active_otadata(two_otadata: List[OtaSelectEntry]) -> OtaSelectEntry:
    """Replicates bootloader_common_select_otadata(..., max=True), preceded
    by the top-level get_selected_boot_partition() gate. Raises
    OtaAmbiguous whenever the real bootloader would not deterministically
    pick one of these two exact entries (falls through to a
    factory/first-boot path instead, which this fail-closed utility refuses
    to guess at)."""
    if len(two_otadata) != 2:
        raise OtaAmbiguous(f"expected exactly 2 otadata sectors, found {len(two_otadata)}")

    e0, e1 = two_otadata

    if e0.is_invalid() and e1.is_invalid():
        raise OtaAmbiguous(
            "both otadata sectors are invalid (blank ota_seq, or ota_state "
            "INVALID/ABORTED) — the real bootloader would fall back to a "
            "factory partition (this table has none) or a first-boot "
            "ota_0-initialization path, neither of which is a deterministic "
            "'the currently active app is X' answer. Needs a normal full "
            "Arduino upload first, not an application-only write."
        )

    valid0, valid1 = e0.is_valid(), e1.is_valid()

    if valid0 and valid1:
        return e0 if e0.ota_seq >= e1.ota_seq else e1  # tie -> index 0, matches real code
    if valid0:
        return e0
    if valid1:
        return e1

    raise OtaAmbiguous(
        "neither otadata sector passes the full validity check (CRC mismatch "
        "on both, despite passing the coarser blank/state check) — cannot "
        "determine the active slot deterministically. Refusing to guess."
    )


def resolve_application_partition(
    partition_table: bytes, otadata: bytes
) -> "ResolvedPartition":
    """End-to-end: parse both dumps and return the verified active
    application partition, or raise OtaAmbiguous."""
    entries = parse_partition_table(partition_table)
    ota_apps = ota_app_partitions(entries)
    if not ota_apps:
        raise OtaAmbiguous("no ota_N app partitions found in the partition table")

    otadata_entries = parse_otadata(otadata)
    active_entry = select_active_otadata(otadata_entries)

    slot_index = (active_entry.ota_seq - 1) % len(ota_apps)
    if slot_index not in ota_apps:
        raise OtaAmbiguous(
            f"computed active slot index {slot_index} has no matching ota_{slot_index} "
            f"partition in the table — ambiguous, refusing to guess."
        )

    partition = ota_apps[slot_index]
    return ResolvedPartition(
        slot_index=slot_index,
        label=partition.label,
        offset=partition.offset,
        size=partition.size,
        active_sector_index=active_entry.sector_index,
        active_seq=active_entry.ota_seq,
        all_seqs=[e.ota_seq for e in otadata_entries],
    )


@dataclass
class ResolvedPartition:
    slot_index: int
    label: str
    offset: int
    size: int
    active_sector_index: int
    active_seq: int
    all_seqs: List[int] = field(default_factory=list)
