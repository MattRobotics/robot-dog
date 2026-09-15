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

SESSION 2.2, FINDING A — OTA subtype recognition corrected. Session 2.1's
filter was `subtype >= 0x10`, which is too permissive: it would also match
PART_SUBTYPE_TEST (0x20) and PART_SUBTYPE_TEE_0/1 (0x30/0x31) — real,
non-OTA app subtypes defined in the SAME esp_flash_partitions.h header.
The real convention (matches ESP-IDF's own partition-table tooling) is a
bitmask, not a threshold: an app partition is an OTA slot iff
`(subtype & 0xF0) == PART_SUBTYPE_OTA_FLAG (0x10)`, and its slot index is
`subtype & PART_SUBTYPE_OTA_MASK (0x0F)`. This correctly matches ota_0..
ota_15 (0x10..0x1F) and correctly excludes TEST/TEE_0/TEE_1, which are
numerically >= 0x10 but not in the 0x10-0x1F band.

SESSION 2.2, FINDING B — rollback/anti-rollback awareness. The real,
installed MATDOG_Controller build has
`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y` (confirmed by reading the actual
build's sdkconfig, not assumed — see parse_sdkconfig_ota_flags() and
VALIDATION.md Session 2.2). With that enabled, bootloader_utility.c's
get_selected_boot_partition() autonomously WRITES to otadata during normal
boot: any sector currently ESP_OTA_IMG_PENDING_VERIFY is marked ABORTED
unconditionally at the start of every boot, and the winning sector is
marked PENDING_VERIFY if its prior state was NEW. That means an
ota_state of NEW or PENDING_VERIFY is not stable across a read -> a
following boot can change it before this tool's read-only snapshot is
still accurate. is_invalid()/is_valid() already correctly exclude
ABORTED (and INVALID), which are stable; NEW/PENDING_VERIFY need an
explicit additional refusal, gated on whether rollback is actually
enabled in the build that produced the binary being flashed.
CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK introduces secure_version/eFuse
comparison this tool does not implement at all — if that is ever enabled,
refuse unconditionally rather than approximate it.

This module performs NO device I/O and writes nothing. See
scripts/verify_application_partition.py for the device-facing wrapper and
scripts/tests/test_ota_partition_logic.py for the offline test suite.
"""
import re
import struct
import zlib
from dataclasses import dataclass, field
from typing import Dict, List, Optional

PARTITION_ENTRY_MAGIC = b"\xaa\x50"
PARTITION_TABLE_END_MAGIC = b"\xeb\xeb"

APP_TYPE = 0x00

# esp_flash_partitions.h — real bitmask convention, not a `>= 0x10` threshold
# (Session 2.1 used the latter, which wrongly matched TEST/TEE_0/TEE_1 too).
PART_SUBTYPE_OTA_FLAG = 0x10
PART_SUBTYPE_OTA_MASK = 0x0F
PART_SUBTYPE_TEST = 0x20
PART_SUBTYPE_TEE_0 = 0x30
PART_SUBTYPE_TEE_1 = 0x31

# esp_ota_img_states_t (esp_flash_partitions.h).
OTA_STATE_NEW = 0
OTA_STATE_PENDING_VERIFY = 1
OTA_STATE_VALID = 2
OTA_STATE_INVALID = 3
OTA_STATE_ABORTED = 4
OTA_STATE_UNDEFINED = 0xFFFFFFFF

BLANK_SEQ = 0xFFFFFFFF

OTADATA_SECTOR_SIZE = 0x1000
OTA_SELECT_ENTRY_SIZE = 32


class OtaAmbiguous(Exception):
    """Raised whenever the real bootloader's rule does not deterministically
    select a slot, or the build's rollback configuration is not one this
    read-only tool can safely reason about. Callers must REFUSE, not
    guess, on this."""


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

    def is_rollback_unstable(self) -> bool:
        """True for the two states get_selected_boot_partition() can
        autonomously rewrite on the very next boot when
        CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y (see Finding B in the
        module docstring). A read-only single-snapshot tool cannot safely
        treat these as a stable answer."""
        return self.ota_state in (OTA_STATE_NEW, OTA_STATE_PENDING_VERIFY)


def parse_sdkconfig_ota_flags(sdkconfig_text: str):
    """Parses the two facts Session 2.2 Finding B requires be read from the
    real build, not assumed. Arduino's sdkconfig uses the standard
    Kconfig text format: `CONFIG_X=y` when set, `# CONFIG_X is not set`
    (or the line simply absent) when not. Returns
    (rollback_enabled: bool, anti_rollback_enabled: bool)."""
    rollback_enabled = bool(re.search(
        r"^CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y\s*$", sdkconfig_text, re.MULTILINE))
    anti_rollback_enabled = bool(re.search(
        r"^CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK=y\s*$", sdkconfig_text, re.MULTILINE))
    return rollback_enabled, anti_rollback_enabled


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
    """Maps ota slot index (0..15) -> its PartitionEntry, using the real
    bitmask convention (Finding A): an app partition is an OTA slot iff
    (subtype & 0xF0) == PART_SUBTYPE_OTA_FLAG, never a `>= 0x10` threshold
    (which would also match PART_SUBTYPE_TEST=0x20, TEE_0=0x30, TEE_1=0x31).
    Raises OtaAmbiguous if two entries claim the same slot index — that is
    itself an ambiguous table, not something to silently pick one of."""
    result: Dict[int, PartitionEntry] = {}
    for e in entries:
        if e.type != APP_TYPE:
            continue
        if (e.subtype & 0xF0) != PART_SUBTYPE_OTA_FLAG:
            continue
        slot = e.subtype & PART_SUBTYPE_OTA_MASK
        if slot in result:
            raise OtaAmbiguous(
                f"duplicate OTA slot index {slot}: both {result[slot].label!r} "
                f"(subtype 0x{result[slot].subtype:02x}) and {e.label!r} "
                f"(subtype 0x{e.subtype:02x}) claim it — ambiguous partition table."
            )
        result[slot] = e
    return result


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
    partition_table: bytes,
    otadata: bytes,
    *,
    rollback_enabled: bool,
    anti_rollback_enabled: bool,
) -> "ResolvedPartition":
    """End-to-end: parse both dumps and return the verified active
    application partition, or raise OtaAmbiguous.

    rollback_enabled/anti_rollback_enabled are REQUIRED (no default) —
    Finding B: callers must read them from the real build's sdkconfig
    (parse_sdkconfig_ota_flags()) rather than let this function assume a
    value, since a future FQBN/sdkconfig change enabling either must not
    silently pass through unnoticed.
    """
    if anti_rollback_enabled:
        raise OtaAmbiguous(
            "CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK=y in the build's sdkconfig — this "
            "introduces secure_version/eFuse-gated slot selection this tool does not "
            "implement at all. Refusing rather than approximate it; see handoff "
            "Session 2.2 Finding B."
        )

    entries = parse_partition_table(partition_table)
    ota_apps = ota_app_partitions(entries)
    if not ota_apps:
        raise OtaAmbiguous("no ota_N app partitions found in the partition table")

    otadata_entries = parse_otadata(otadata)
    active_entry = select_active_otadata(otadata_entries)

    if rollback_enabled and active_entry.is_rollback_unstable():
        raise OtaAmbiguous(
            f"CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y in the build's sdkconfig and the "
            f"selected otadata entry has ota_state={active_entry.ota_state} "
            f"(NEW/PENDING_VERIFY) — the real bootloader can autonomously rewrite this "
            f"on the very next boot (mark PENDING_VERIFY, or abort it), so this "
            f"read-only snapshot cannot be trusted as stable. Refusing; see handoff "
            f"Session 2.2 Finding B."
        )

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
