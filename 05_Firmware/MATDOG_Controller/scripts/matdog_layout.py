#!/usr/bin/env python3
"""MATDOG flash layout V1 - the pinned contract and its fail-closed checks.

LAYOUT_ID MATDOG_16M_2x5M_NVS_V1 (ESP32-S3, 16 MiB):

    nvs         data/nvs      0x009000  0x005000   default NVS (Arduino core / Wi-Fi)
    otadata     data/ota      0x00E000  0x002000
    app0        app/ota_0     0x010000  0x500000   5 MiB
    app1        app/ota_1     0x510000  0x500000   5 MiB
    ffat        data/fat      0xA10000  0x5D0000
    matdog_nvs  data/nvs      0xFE0000  0x010000   persistent MATDOG data
    coredump    data/coredump 0xFF0000  0x010000

THE ORDER MATTERS. The Arduino core's initArduino() erases the FIRST
nvs-subtype partition it finds when the default NVS is unusable. "nvs" must
therefore precede "matdog_nvs", or a bad default NVS would wipe MATDOG data.

Authority: partitions.csv (sketch folder) is what the build compiles. This
module pins the SAME values and the SHA-256 of the binary table the real
toolchain produces from it, so a drift in either is caught offline. Pure
logic: no device I/O. The CLI at the bottom is a thin shell over it.

This module never writes, erases or contacts anything. The migration
contract at the end is DATA ONLY (what a future, explicitly authorized
migration procedure must satisfy); it implements no hardware operation.
"""
import argparse
import hashlib
import sys
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from ota_partition_logic import OtaAmbiguous, PartitionEntry, parse_partition_table  # noqa: E402

LAYOUT_ID = "MATDOG_16M_2x5M_NVS_V1"
# Compiled into the firmware (src/update/OtaLayoutContract.cpp) so the layout
# identity is a property of the binary itself, readable with `strings`.
LAYOUT_MARKER = "MATDOG_LAYOUT_ID=" + LAYOUT_ID

FLASH_SIZE_BYTES = 0x1000000
PARTITION_TABLE_OFFSET = 0x8000
PARTITION_TABLE_MAX_BYTES = 0xC00  # the table incl. MD5 entry is always padded to this
ERASE_SECTOR_BYTES = 0x1000

APP_SLOT_SIZE = 0x500000  # 5,242,880
APP_GROWTH_WARNING_BYTES = 4 * 1024 * 1024

TYPE_APP = 0x00
TYPE_DATA = 0x01
SUBTYPE_APP_OTA_0 = 0x10
SUBTYPE_APP_OTA_1 = 0x11
SUBTYPE_DATA_OTA = 0x00
SUBTYPE_DATA_NVS = 0x02
SUBTYPE_DATA_COREDUMP = 0x03
SUBTYPE_DATA_FAT = 0x81

DEFAULT_NVS_LABEL = "nvs"
MATDOG_NVS_LABEL = "matdog_nvs"

EXPECTED_PARTITIONS = (
    PartitionEntry("nvs", TYPE_DATA, SUBTYPE_DATA_NVS, 0x009000, 0x005000),
    PartitionEntry("otadata", TYPE_DATA, SUBTYPE_DATA_OTA, 0x00E000, 0x002000),
    PartitionEntry("app0", TYPE_APP, SUBTYPE_APP_OTA_0, 0x010000, 0x500000),
    PartitionEntry("app1", TYPE_APP, SUBTYPE_APP_OTA_1, 0x510000, 0x500000),
    PartitionEntry("ffat", TYPE_DATA, SUBTYPE_DATA_FAT, 0xA10000, 0x5D0000),
    PartitionEntry("matdog_nvs", TYPE_DATA, SUBTYPE_DATA_NVS, 0xFE0000, 0x010000),
    PartitionEntry("coredump", TYPE_DATA, SUBTYPE_DATA_COREDUMP, 0xFF0000, 0x010000),
)

# SHA-256 of the 3072-byte binary table gen_esp32part.py produces from
# partitions.csv (MD5 entry included). Confirmed against the real compile.
EXPECTED_TABLE_SHA256 = "8f756ecb719c4894b9c23c26bcc171e1d01ae8cda69882950944d5ce264946e7"

# The table of the app3M_fat9M_16MB scheme the device runs today (identical to
# the first 0xC00 bytes of the authorized 2026-09-29 full-flash backup).
# Used ONLY to give a precise refusal; it never authorizes anything.
LEGACY_TABLE_SHA256 = "ace02503447d0f470692e65fa76002f2d77a92dc81cd3813d8aa66718d716da9"
LEGACY_PARTITION_SCHEME = "app3M_fat9M_16MB"

PINNED_FQBN = (
    "esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,"
    "FlashMode=qio,FlashSize=16M,PartitionScheme=custom,DebugLevel=none,PSRAM=opi"
)
REQUIRED_PARTITION_SCHEME = "custom"


class LayoutRefusal(Exception):
    def __init__(self, code, detail=""):
        super().__init__(f"{code}: {detail}" if detail else code)
        self.code = code
        self.detail = detail


class Code:
    TABLE_EMPTY = "TABLE_EMPTY"
    TABLE_TRUNCATED = "TABLE_TRUNCATED"
    TABLE_UNPARSEABLE = "TABLE_UNPARSEABLE"
    OUT_OF_FLASH = "OUT_OF_FLASH"
    OVERLAP = "OVERLAP"
    MATDOG_NVS_MISSING = "MATDOG_NVS_MISSING"
    MATDOG_NVS_MOVED = "MATDOG_NVS_MOVED"
    NVS_ORDER = "NVS_ORDER"
    ENTRY_COUNT = "ENTRY_COUNT"
    PARTITION_MISMATCH = "PARTITION_MISMATCH"
    INSTALLED_LAYOUT_LEGACY = "INSTALLED_LAYOUT_LEGACY"
    TABLE_HASH_MISMATCH = "TABLE_HASH_MISMATCH"
    APP_EMPTY = "APP_EMPTY"
    APP_TOO_LARGE = "APP_TOO_LARGE"
    WRITE_UNALIGNED = "WRITE_UNALIGNED"
    WRITE_TOUCHES_MATDOG_NVS = "WRITE_TOUCHES_MATDOG_NVS"
    WRITE_OUTSIDE_APP = "WRITE_OUTSIDE_APP"
    WRITE_EXCEEDS_APP = "WRITE_EXCEEDS_APP"
    TARGET_NOT_APP_SLOT = "TARGET_NOT_APP_SLOT"
    PARTITION_SIZE_MISMATCH = "PARTITION_SIZE_MISMATCH"
    FQBN_NOT_CUSTOM_LAYOUT = "FQBN_NOT_CUSTOM_LAYOUT"
    FQBN_LEGACY_SCHEME = "FQBN_LEGACY_SCHEME"
    LAYOUT_ID_MISSING_FROM_BINARY = "LAYOUT_ID_MISSING_FROM_BINARY"
    MIGRATION_PRECONDITION = "MIGRATION_PRECONDITION"
    MIGRATION_FORBIDDEN_REGION = "MIGRATION_FORBIDDEN_REGION"


def _fmt(e):
    return f"{e.label}(type=0x{e.type:02x},sub=0x{e.subtype:02x},0x{e.offset:06x}+0x{e.size:x})"


def sha256_hex(data):
    return hashlib.sha256(data).hexdigest()


# --------------------------------------------------------------------------
# Partition table (entries)
# --------------------------------------------------------------------------

def check_entries(entries):
    """Raises LayoutRefusal unless `entries` (in table order) is EXACTLY the
    V1 layout. The specific diagnoses come first so the refusal names the
    real problem; the final positional comparison is the exact contract."""
    if not entries:
        raise LayoutRefusal(Code.TABLE_EMPTY, "no partition entries")

    for e in entries:
        if e.size == 0 or e.offset + e.size > FLASH_SIZE_BYTES:
            raise LayoutRefusal(Code.OUT_OF_FLASH, f"{_fmt(e)} leaves the 16 MiB flash")
    by_offset = sorted(entries, key=lambda e: e.offset)
    for a, b in zip(by_offset, by_offset[1:]):
        if a.offset + a.size > b.offset:
            raise LayoutRefusal(Code.OVERLAP, f"{_fmt(a)} overlaps {_fmt(b)}")

    matdog = [e for e in entries if e.label == MATDOG_NVS_LABEL]
    if not matdog:
        raise LayoutRefusal(Code.MATDOG_NVS_MISSING, "no 'matdog_nvs' partition")
    want = EXPECTED_PARTITIONS[5]
    if len(matdog) != 1 or (matdog[0].type, matdog[0].subtype, matdog[0].offset,
                            matdog[0].size) != (want.type, want.subtype, want.offset, want.size):
        raise LayoutRefusal(Code.MATDOG_NVS_MOVED,
                            f"{_fmt(matdog[0])} != expected {_fmt(want)}")

    nvs_order = [e.label for e in entries
                 if e.type == TYPE_DATA and e.subtype == SUBTYPE_DATA_NVS]
    if not nvs_order or nvs_order[0] != DEFAULT_NVS_LABEL:
        raise LayoutRefusal(Code.NVS_ORDER,
                            f"first nvs-subtype partition is {nvs_order[:1]}, must be "
                            f"'{DEFAULT_NVS_LABEL}': the core erases the first one on a bad "
                            f"default NVS")

    if len(entries) != len(EXPECTED_PARTITIONS):
        raise LayoutRefusal(Code.ENTRY_COUNT,
                            f"{len(entries)} entries, expected {len(EXPECTED_PARTITIONS)}")
    for got, want in zip(entries, EXPECTED_PARTITIONS):
        if (got.label, got.type, got.subtype, got.offset, got.size) != \
           (want.label, want.type, want.subtype, want.offset, want.size):
            raise LayoutRefusal(Code.PARTITION_MISMATCH,
                                f"{_fmt(got)} != expected {_fmt(want)}")


# --------------------------------------------------------------------------
# Binary partition table
# --------------------------------------------------------------------------

def canonical_table_bytes(raw):
    """The bytes that identify a table: the first 0xC00 (the table region is
    always padded to this). A device read of 0x1000 and a build artifact of
    0xC00 hash identically."""
    if len(raw) < PARTITION_TABLE_MAX_BYTES:
        raise LayoutRefusal(Code.TABLE_TRUNCATED,
                            f"{len(raw)} bytes < {PARTITION_TABLE_MAX_BYTES}")
    return bytes(raw[:PARTITION_TABLE_MAX_BYTES])


def table_sha256(raw):
    return sha256_hex(canonical_table_bytes(raw))


def check_table_bytes(raw):
    """Full check of a binary table (build artifact or device read). Returns
    (sha256, entries) or raises. The pinned hash makes the MD5 entry and the
    padding part of the contract too."""
    table = canonical_table_bytes(raw)
    digest = sha256_hex(table)
    try:
        entries = parse_partition_table(table)
    except OtaAmbiguous as exc:
        raise LayoutRefusal(Code.TABLE_UNPARSEABLE, str(exc))
    if digest == LEGACY_TABLE_SHA256:
        raise LayoutRefusal(Code.INSTALLED_LAYOUT_LEGACY,
                            f"partition table is the legacy {LEGACY_PARTITION_SCHEME} layout "
                            f"({digest}); it needs the separately authorized migration, never "
                            f"an application-only write")
    check_entries(entries)
    if digest != EXPECTED_TABLE_SHA256:
        raise LayoutRefusal(Code.TABLE_HASH_MISMATCH,
                            f"entries conform but table bytes {digest} != pinned "
                            f"{EXPECTED_TABLE_SHA256}")
    return digest, entries


# --------------------------------------------------------------------------
# Application size
# --------------------------------------------------------------------------

def check_app_size(size, slot_size=APP_SLOT_SIZE):
    """Returns 'OK' or 'GROWTH_WARNING'; raises on empty or too large."""
    if size <= 0:
        raise LayoutRefusal(Code.APP_EMPTY, "application image is empty")
    if size > slot_size:
        raise LayoutRefusal(Code.APP_TOO_LARGE,
                            f"{size} bytes > {slot_size} bytes (5 MiB slot): not an acceptable build")
    if size >= APP_GROWTH_WARNING_BYTES:
        return "GROWTH_WARNING"
    return "OK"


# --------------------------------------------------------------------------
# Write / erase range
# --------------------------------------------------------------------------

def _align_up(value, quantum):
    return (value + quantum - 1) // quantum * quantum


@dataclass(frozen=True)
class WritePlan:
    label: str
    start: int
    write_end: int   # exclusive, exact image end
    erase_end: int   # exclusive, sector-rounded: what esptool erases


def _intersects(a_start, a_end, b_start, b_end):
    return a_start < b_end and b_start < a_end


def check_write_range(start, length, entries=EXPECTED_PARTITIONS):
    """The range [start, start+length) that esptool would write, with its
    sector-rounded erase end, must lie entirely inside ONE app slot and must
    not touch MATDOG NVS. `entries` must already have passed check_entries."""
    if length <= 0:
        raise LayoutRefusal(Code.APP_EMPTY, "empty write")
    erase_end = _align_up(start + length, ERASE_SECTOR_BYTES)
    for e in entries:
        if e.label == MATDOG_NVS_LABEL and _intersects(start, erase_end, e.offset, e.offset + e.size):
            raise LayoutRefusal(Code.WRITE_TOUCHES_MATDOG_NVS,
                                f"write/erase [0x{start:x},0x{erase_end:x}) intersects "
                                f"{_fmt(e)}")
    if start % ERASE_SECTOR_BYTES:
        raise LayoutRefusal(Code.WRITE_UNALIGNED, f"start 0x{start:x} is not sector aligned")
    slot = next((e for e in entries if e.type == TYPE_APP and e.offset <= start < e.offset + e.size),
                None)
    if slot is None:
        raise LayoutRefusal(Code.WRITE_OUTSIDE_APP, f"0x{start:x} is not inside an app slot")
    if erase_end > slot.offset + slot.size:
        raise LayoutRefusal(Code.WRITE_EXCEEDS_APP,
                            f"write/erase end 0x{erase_end:x} passes the end of {slot.label} "
                            f"(0x{slot.offset + slot.size:x})")
    return WritePlan(slot.label, start, start + length, erase_end)


def check_app_only_write(offset, partition_size, image_size, entries=EXPECTED_PARTITIONS):
    """Everything about the target and the range, before any write."""
    if image_size <= 0:
        raise LayoutRefusal(Code.APP_EMPTY, "application image is empty")
    # MATDOG NVS first: naming it is the most important refusal.
    for e in entries:
        if e.label == MATDOG_NVS_LABEL and _intersects(
                offset, _align_up(offset + image_size, ERASE_SECTOR_BYTES),
                e.offset, e.offset + e.size):
            raise LayoutRefusal(Code.WRITE_TOUCHES_MATDOG_NVS,
                                f"write/erase from 0x{offset:x} intersects {_fmt(e)}")
    slot = next((e for e in entries if e.type == TYPE_APP and e.offset == offset), None)
    if slot is None:
        raise LayoutRefusal(Code.TARGET_NOT_APP_SLOT,
                            f"0x{offset:x} is not the start of an app slot of {LAYOUT_ID}")
    if partition_size != slot.size:
        raise LayoutRefusal(Code.PARTITION_SIZE_MISMATCH,
                            f"target {slot.label} size {partition_size} != layout {slot.size}")
    if image_size > slot.size:
        raise LayoutRefusal(Code.APP_TOO_LARGE,
                            f"image {image_size} bytes > {slot.label} {slot.size} bytes")
    return check_write_range(offset, image_size, entries)


# --------------------------------------------------------------------------
# FQBN / binary identity
# --------------------------------------------------------------------------

def partition_scheme_of(fqbn):
    for part in fqbn.split(":")[-1].split(","):
        key, _, value = part.partition("=")
        if key == "PartitionScheme":
            return value
    return None


def check_fqbn(fqbn):
    """The build must select the custom scheme (so partitions.csv in the
    sketch folder is the only source of the table). A legacy scheme name is
    refused with its own code."""
    scheme = partition_scheme_of(fqbn)
    if scheme == REQUIRED_PARTITION_SCHEME:
        return
    if scheme == LEGACY_PARTITION_SCHEME:
        raise LayoutRefusal(Code.FQBN_LEGACY_SCHEME,
                            f"FQBN selects the legacy {LEGACY_PARTITION_SCHEME} scheme")
    raise LayoutRefusal(Code.FQBN_NOT_CUSTOM_LAYOUT,
                        f"PartitionScheme={scheme!r}, must be {REQUIRED_PARTITION_SCHEME!r}")


def binary_embeds_layout_id(binary_bytes):
    return LAYOUT_MARKER.encode("ascii") in binary_bytes


# --------------------------------------------------------------------------
# Future migration contract (DATA ONLY - no hardware operation exists here)
# --------------------------------------------------------------------------

# Regions a future, explicitly authorized migration from the legacy layout is
# allowed to write. Everything else - and in particular the default NVS, the
# whole MATDOG NVS and any chip erase - is outside the contract.
MIGRATION_ALLOWED_WRITE_REGIONS = (
    ("partition_table", 0x008000, 0x009000),
    ("otadata", 0x00E000, 0x010000),
    ("app0", 0x010000, 0x510000),
)
MIGRATION_PROTECTED_REGIONS = (
    ("bootloader", 0x000000, 0x008000),
    ("default_nvs", 0x009000, 0x00E000),
    ("matdog_nvs", 0xFE0000, 0xFF0000),
)
MIGRATION_PRECONDITIONS = (
    "operator_authorization_for_this_session",
    "device_identity_verified",
    "fresh_full_16mib_backup_with_authorized_sha256",
    "installed_table_is_the_known_legacy_table",
    "target_table_sha256_equals_manifest_v2_and_pinned",
    "application_is_manifest_v2_for_this_layout_id",
    "default_nvs_content_backup_verified",
    "ffat_data_loss_accepted",
    "post_migration_boot_slot_is_app0",
)
MIGRATION_FORBIDDEN_OPERATIONS = ("erase-flash", "full-image-restore", "write-flash-of-merged-image")


def migration_unmet_preconditions(facts):
    """facts: mapping precondition-name -> bool. Returns the names that are
    not explicitly True (absent == unmet)."""
    return [name for name in MIGRATION_PRECONDITIONS if facts.get(name) is not True]


def check_migration_plan(regions, facts):
    """regions: iterable of (name, start, end) a migration wants to write.
    Raises unless every region sits inside an allowed region, touches no
    protected region, and every precondition holds."""
    unmet = migration_unmet_preconditions(facts)
    if unmet:
        raise LayoutRefusal(Code.MIGRATION_PRECONDITION, "unmet: " + ", ".join(unmet))
    for name, start, end in regions:
        for pname, ps, pe in MIGRATION_PROTECTED_REGIONS:
            if _intersects(start, end, ps, pe):
                raise LayoutRefusal(Code.MIGRATION_FORBIDDEN_REGION,
                                    f"{name} [0x{start:x},0x{end:x}) touches protected {pname}")
        if not any(s <= start and end <= e for _, s, e in MIGRATION_ALLOWED_WRITE_REGIONS):
            raise LayoutRefusal(Code.MIGRATION_FORBIDDEN_REGION,
                                f"{name} [0x{start:x},0x{end:x}) is not an allowed migration region")


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def _refuse(exc):
    print(f"REFUSED={exc.code}", file=sys.stderr)
    print(f"DETAIL={exc.detail}", file=sys.stderr)
    return 1


def _cmd_check_build(args):
    try:
        check_fqbn(args.fqbn)
        digest, _ = check_table_bytes(Path(args.partitions).read_bytes())
        size = Path(args.binary).stat().st_size
        status = check_app_size(size)
        if not binary_embeds_layout_id(Path(args.binary).read_bytes()):
            raise LayoutRefusal(Code.LAYOUT_ID_MISSING_FROM_BINARY,
                                f"'{LAYOUT_MARKER}' not found in {args.binary}")
    except LayoutRefusal as exc:
        return _refuse(exc)
    except OSError as exc:
        print(f"REFUSED=BUILD_ARTIFACT_UNREADABLE\nDETAIL={exc}", file=sys.stderr)
        return 1
    print(f"LAYOUT_ID={LAYOUT_ID}")
    print(f"PARTITION_TABLE_SHA256={digest}")
    print(f"APP_PARTITION_SIZE={APP_SLOT_SIZE}")
    print(f"APPLICATION_SIZE={size}")
    print(f"APPLICATION_SIZE_STATUS={status}")
    print(f"APPLICATION_SLOT_USED_PERCENT={100.0 * size / APP_SLOT_SIZE:.1f}")
    if status == "GROWTH_WARNING":
        print(f"WARNING: application is {size} bytes, >= {APP_GROWTH_WARNING_BYTES} "
              f"(4 MiB) of the {APP_SLOT_SIZE}-byte slot: growth warning", file=sys.stderr)
    return 0


def _cmd_check_write(args):
    try:
        plan = check_app_only_write(int(args.offset, 0), int(args.partition_size, 0),
                                    int(args.image_size, 0))
    except LayoutRefusal as exc:
        return _refuse(exc)
    print(f"LAYOUT_ID={LAYOUT_ID}")
    print(f"WRITE_TARGET_LABEL={plan.label}")
    print(f"WRITE_START=0x{plan.start:06x}")
    print(f"WRITE_END=0x{plan.write_end:06x}")
    print(f"ERASE_END=0x{plan.erase_end:06x}")
    return 0


def _cmd_check_fqbn(args):
    try:
        check_fqbn(args.fqbn)
    except LayoutRefusal as exc:
        return _refuse(exc)
    print(f"FQBN_PARTITION_SCHEME={partition_scheme_of(args.fqbn)}")
    return 0


def _cmd_contract(_args):
    print(f"LAYOUT_ID={LAYOUT_ID}")
    print(f"EXPECTED_TABLE_SHA256={EXPECTED_TABLE_SHA256}")
    for e in EXPECTED_PARTITIONS:
        print(f"PARTITION={_fmt(e)}")
    print(f"APP_SLOT_SIZE={APP_SLOT_SIZE}")
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)

    b = sub.add_parser("check-build", help="validate a finished build's layout artifacts")
    b.add_argument("--partitions", required=True, help="binary partition table the build produced")
    b.add_argument("--binary", required=True)
    b.add_argument("--fqbn", required=True)
    b.set_defaults(func=_cmd_check_build)

    w = sub.add_parser("check-write", help="validate the application-only write range")
    w.add_argument("--offset", required=True)
    w.add_argument("--partition-size", required=True)
    w.add_argument("--image-size", required=True)
    w.set_defaults(func=_cmd_check_write)

    f = sub.add_parser("check-fqbn", help="the FQBN must select the custom layout")
    f.add_argument("--fqbn", required=True)
    f.set_defaults(func=_cmd_check_fqbn)

    c = sub.add_parser("contract", help="print the pinned layout contract")
    c.set_defaults(func=_cmd_contract)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
