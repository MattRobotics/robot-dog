#!/usr/bin/env python3
"""M0 file-only checks. No serial/network/process API and no hardware actions.

The two commands inspect local files or create new local evidence files. A PASS
is a technical result, never an operator authorization or proof of acquisition
time/device identity. See FLASH_LAYOUT_M0_RUNBOOK.md for those separate gates.
Existing layout, Manifest V2, backup and OTA logic remain authoritative.
"""
import argparse
import hashlib
import json
import re
import sys
from pathlib import Path

import backup_gate_logic as backup_gate
import build_manifest as manifest_gate
import matdog_layout as layout
import ota_partition_logic as ota

APPROVED_COMMIT = "be0c12979e5b4b8ddc9dd21772d0d106e4358f4a"
PROFILES = ("USB_ONLY", "ROBOT_POWERED")
BOOTLOADER_SIZE = 19968
BOOTLOADER_SHA256 = "31b3c1be45dc5a76aa85c82540d6787b675e711eaf11021a5eea7e36f469c6de"
ARCHIVE_REGIONS = (
    ("bootloader_region", 0x0, 0x8000),
    ("partition_sector", 0x8000, 0x9000),
    ("default_nvs", 0x9000, 0xE000),
    ("otadata", 0xE000, 0x10000),
    ("legacy_app0", 0x10000, 0x310000),
    ("legacy_app1", 0x310000, 0x610000),
    ("legacy_ffat", 0x610000, 0xFF0000),
    ("destination_matdog_nvs", 0xFE0000, 0xFF0000),
    ("coredump", 0xFF0000, 0x1000000),
)


class Stop(ValueError):
    def __init__(self, code, detail=""):
        super().__init__(f"{code}: {detail}")
        self.code, self.detail = code, detail


def digest(data):
    return hashlib.sha256(data).hexdigest()


def read_file(path):
    try:
        return Path(path).read_bytes()
    except OSError as exc:
        raise Stop("FILE_UNREADABLE", str(exc)) from exc


def verified_backup(image_path, repeat_path, expected_sha256):
    """Distinct file names are necessary, not proof of independent device reads."""
    a, b = Path(image_path), Path(repeat_path)
    if not a.is_file() or not b.is_file():
        raise Stop("BACKUP_MISSING", "both complete acquisitions are required")
    if a.samefile(b):
        raise Stop("READ_INDEPENDENCE_MISSING", "same file/inode supplied twice")
    image, repeat = read_file(a), read_file(b)
    verdict = backup_gate.verify_backup(
        is_default_backup=False, actual_size=len(image), actual_sha256=digest(image),
        custom_expected_sha256=expected_sha256, manifest_sha256=None)
    if not verdict.ok:
        raise Stop(verdict.reason, verdict.detail)
    if len(repeat) != layout.FLASH_SIZE_BYTES or image != repeat:
        raise Stop("INDEPENDENT_READ_MISMATCH", "size or bytes differ; do not vote/merge reads")
    return image


def legacy_table(image):
    if len(image) != layout.FLASH_SIZE_BYTES:
        raise Stop("SIZE_MISMATCH", "a complete 16 MiB image is required")
    table = image[0x8000:0x8C00]
    if layout.table_sha256(table) != layout.LEGACY_TABLE_SHA256:
        raise Stop("INSTALLED_LAYOUT_UNEXPECTED", "exact pinned legacy table required")
    return table


def check_reclassified_regions(image):
    """No occupied legacy data may be hidden by the V1 geometry or sector erase.

    The legacy app0 replacement is intentional. Everything else that changes
    allocation must be erased already; the table's erased padding is checked
    separately because FLASH_BEGIN erases the whole 4 KiB sector.
    """
    legacy_table(image)
    for code, start, end in (
        ("TABLE_SECTOR_PADDING_OCCUPIED", 0x8C00, 0x9000),
        ("DESTINATION_NVS_NOT_EMPTY", 0xFE0000, 0xFF0000),
        ("NEW_APP1_NOT_EMPTY", 0x510000, 0xA10000),
        ("LEGACY_APP1_NOT_EMPTY", 0x310000, 0x610000),
        ("FFAT_NOT_EMPTY", 0x610000, 0xFF0000),
    ):
        if image[start:end] != b"\xff" * (end - start):
            raise Stop(code, f"[0x{start:x},0x{end:x}) contains non-FF bytes; no erase exception")


def inspect_legacy(image, *, sdkconfig, bootloader, running_offset, installed_build_id):
    table = legacy_table(image)
    if len(bootloader) != BOOTLOADER_SIZE or digest(bootloader) != BOOTLOADER_SHA256:
        raise Stop("BOOTLOADER_REFERENCE_UNEXPECTED", "approved core-generated reference required")
    if image[:BOOTLOADER_SIZE] != bootloader or image[BOOTLOADER_SIZE:0x8000] != b"\xff" * (0x8000 - BOOTLOADER_SIZE):
        raise Stop("BOOTLOADER_NOT_PROVEN", "existing bootloader region differs from the reference")
    rollback, anti = ota.parse_sdkconfig_ota_flags(sdkconfig)
    if rollback is not ota.SdkconfigFlag.ENABLED or anti is not ota.SdkconfigFlag.DISABLED:
        raise Stop("BOOTLOADER_CONFIG_UNEXPECTED", "require rollback enabled, anti-rollback explicitly off")
    entries = ota.parse_otadata(image[0xE000:0x10000])
    known_states = {ota.OTA_STATE_UNDEFINED, ota.OTA_STATE_VALID, ota.OTA_STATE_INVALID, ota.OTA_STATE_ABORTED}
    # Stronger than ordinary app-only selection: even a losing pending sector
    # would be rewritten by this bootloader at the next boot.
    if any(e.ota_state not in known_states for e in entries):
        raise Stop("OTADATA_UNSTABLE", "NEW/PENDING_VERIFY/unknown state in either sector")
    try:
        selected = ota.resolve_application_partition(
            table, image[0xE000:0x10000], rollback=rollback, anti_rollback=anti)
    except ota.OtaAmbiguous as exc:
        raise Stop("OTADATA_AMBIGUOUS", str(exc)) from exc
    if selected.slot_index != 0:
        raise Stop("ACTIVE_SLOT_UNEXPECTED", "this M0 sequence supports legacy app0 only")
    if running_offset != selected.offset:
        raise Stop("RUNNING_SLOT_MISMATCH", "runtime observation disagrees with selection; possible fallback")
    active = image[selected.offset:selected.offset + selected.size]
    if active[0] != 0xE9:
        raise Stop("INSTALLED_APP_HEADER_INVALID", "also validate checksum/hash using offline image-info")
    if not installed_build_id or installed_build_id.encode("ascii") + b"\0" not in active:
        raise Stop("INSTALLED_IDENTITY_NOT_FOUND", "runtime BUILD_ID must be found in the selected image")
    check_reclassified_regions(image)
    return {
        "legacy_table_sha256": digest(table), "selected_slot": selected.slot_index,
        "selected_offset": selected.offset, "installed_build_id_observed": installed_build_id,
        "otadata_sha256": digest(image[0xE000:0x10000]),
        "otadata_entries": [vars(e) | {"crc_valid": e.is_valid()} for e in entries],
        "otadata_action": "PRESERVE", "matdog_nvs_action": "PRESERVE_ERASED",
        "bootloader_action": "PRESERVE", "ffat_erased_bytes": 0x9E0000,
    }


def verified_artifact(binary_path, manifest_path, *, expected_profile="USB_ONLY"):
    if expected_profile not in PROFILES:
        raise Stop("PROFILE_UNSUPPORTED", "only explicitly named M0 profiles are accepted")
    binary = read_file(binary_path)
    table = read_file(manifest_gate.partition_table_path_for(binary_path))
    table_hash, _ = layout.check_table_bytes(table)
    if len(table) != 0xC00:
        raise Stop("TABLE_ARTIFACT_SIZE", "build artifact must be exactly 3072 bytes")
    m = manifest_gate.parse_manifest(Path(manifest_path).read_text(encoding="utf-8"))
    # File-only verification does not inspect Git. Step 3 of the runbook must
    # separately prove the approved source checkout is still clean; the CLEAN
    # input here does not grant that operator/provenance assertion by itself.
    verdict = manifest_gate.verify_manifest(
        m, head_commit=APPROVED_COMMIT, expected_fqbn=layout.PINNED_FQBN,
        tree_state="CLEAN", binary_exists=True, binary_size=len(binary),
        binary_sha256=digest(binary), requested_profile=expected_profile, requested_ota_ingest="0",
        build_partition_table_sha256=table_hash,
        binary_embeds_layout_id=layout.binary_embeds_layout_id(binary))
    if not verdict.ok:
        raise Stop(verdict.reason, verdict.detail)
    if m["APPLICATION_BINARY"] != Path(binary_path).name:
        raise Stop("BINARY_NAME_MISMATCH")
    if m["BUILD_ID"] != APPROVED_COMMIT[:12] or m["BUILD_ID"].encode("ascii") + b"\0" not in binary:
        raise Stop("BUILD_ID_NOT_PROVEN", "Manifest V2 alone does not check the embedded build id")
    if binary[0] != 0xE9:
        raise Stop("TARGET_APP_HEADER_INVALID")
    plan = layout.check_app_only_write(0x10000, layout.APP_SLOT_SIZE, len(binary))
    # This reviewed candidate must also fit the legacy app0 during staging.
    if plan.erase_end > 0x310000:
        raise Stop("STAGING_EXCEEDS_LEGACY_APP0", "requires a separately reviewed staging sequence")
    return binary, table, m, plan


def expected_snapshot(before, binary, table, *, stage):
    legacy_table(before)
    layout.check_table_bytes(table)
    plan = layout.check_app_only_write(0x10000, layout.APP_SLOT_SIZE, len(binary))
    if stage not in ("app", "table"):
        raise Stop("STAGE_UNKNOWN")
    result = bytearray(before)
    result[plan.start:plan.erase_end] = binary + b"\xff" * (plan.erase_end - plan.write_end)
    if stage == "table":
        result[0x8000:0x9000] = table[:0xC00] + b"\xff" * 0x400
    return bytes(result)


def check_snapshot(before, snapshot, binary, table, *, stage):
    expected = expected_snapshot(before, binary, table, stage=stage)
    if len(snapshot) != len(expected):
        raise Stop("SNAPSHOT_SIZE_MISMATCH")
    if snapshot != expected:
        first = next(i for i, (a, b) in enumerate(zip(snapshot, expected)) if a != b)
        raise Stop("SNAPSHOT_BYTE_MISMATCH", f"first mismatch at 0x{first:06x}")
    return digest(snapshot)


def new_output(path):
    out = Path(path)
    out.mkdir(parents=True, exist_ok=False)
    return out


def archive_backup(image, out):
    rows = []
    for name, start, end in ARCHIVE_REGIONS:
        raw = image[start:end]
        (out / (name + ".bin")).write_bytes(raw)
        rows.append({"file": name + ".bin", "start": start, "end_exclusive": end,
                     "bytes": len(raw), "sha256": digest(raw)})
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("archive", "plan"):
        p = sub.add_parser(name)
        p.add_argument("--backup", required=True)
        p.add_argument("--repeat", required=True)
        p.add_argument("--backup-sha256", required=True)
        p.add_argument("--out", required=True, help="must not exist; no evidence is overwritten")
    p = sub.choices["plan"]
    p.add_argument("--sdkconfig", required=True)
    p.add_argument("--bootloader", required=True)
    p.add_argument("--running-offset", required=True, type=lambda s: int(s, 0))
    p.add_argument("--installed-build-id", required=True)
    p.add_argument("--expected-mac", required=True)
    p.add_argument("--observed-mac", required=True)
    p.add_argument("--binary", required=True)
    p.add_argument("--manifest", required=True)
    p.add_argument("--profile", choices=PROFILES, default="USB_ONLY")
    p = sub.add_parser("check-snapshot")
    p.add_argument("--backup", required=True)
    p.add_argument("--backup-sha256", required=True)
    p.add_argument("--snapshot", required=True)
    p.add_argument("--binary", required=True)
    p.add_argument("--manifest", required=True)
    p.add_argument("--profile", choices=PROFILES, default="USB_ONLY")
    p.add_argument("--stage", choices=("app", "table"), required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "check-snapshot":
            before = read_file(args.backup)
            if digest(before) != args.backup_sha256.lower():
                raise Stop("SHA256_MISMATCH", "original backup changed")
            binary, table, _, _ = verified_artifact(args.binary, args.manifest, expected_profile=args.profile)
            sha = check_snapshot(before, read_file(args.snapshot), binary, table, stage=args.stage)
            print(f"SNAPSHOT_CHECK=PASS\nSNAPSHOT_SHA256={sha}\nAUTHORIZATION_GRANTED=NO")
            return 0
        image = verified_backup(args.backup, args.repeat, args.backup_sha256)
        legacy_table(image)
        report = {"backup_sha256": digest(image), "backup_bytes": len(image),
                  "hardware_io": False, "authorization_granted": False,
                  "operator_evidence_required": ["approved clean source/toolchain verification",
                                                 "fresh independent device reads", "device identity/security",
                                                 "runtime/image validation", "profile-specific power/peripheral safety", "session authorization"]}
        if args.command == "plan":
            if (not re.fullmatch(r"(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}", args.expected_mac)
                    or args.expected_mac.lower() != args.observed_mac.lower()):
                raise Stop("DEVICE_IDENTITY_MISMATCH")
            report.update(inspect_legacy(image, sdkconfig=Path(args.sdkconfig).read_text(encoding="utf-8"),
                                         bootloader=read_file(args.bootloader), running_offset=args.running_offset,
                                         installed_build_id=args.installed_build_id))
            binary, table, m, plan = verified_artifact(args.binary, args.manifest, expected_profile=args.profile)
            report.update({"observed_mac": args.observed_mac.lower(), "manifest": m,
                           "app_write": vars(plan), "table_write": {"start": 0x8000, "end": 0x8C00,
                                                                      "erase_end": 0x9000},
                           "expected_profile": args.profile,
                           "reclassified_legacy_regions": "ALL_FF",
                           "expected_app_stage_sha256": digest(expected_snapshot(image, binary, table, stage="app")),
                           "expected_final_sha256": digest(expected_snapshot(image, binary, table, stage="table"))})
        out = new_output(args.out)
        report["regions"] = archive_backup(image, out)
        if args.command == "plan":
            (out / "app0_sector_padded.bin").write_bytes(binary + b"\xff" * (plan.erase_end - plan.write_end))
            (out / "target_partition_sector.bin").write_bytes(table + b"\xff" * 0x400)
        (out / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"FILE_CHECKS=PASS\nREPORT={out / 'report.json'}\nAUTHORIZATION_GRANTED=NO")
        return 0
    except (Stop, layout.LayoutRefusal) as exc:
        print(f"STOP={exc.code}\nDETAIL={exc.detail}", file=sys.stderr)
    except (OSError, ValueError, UnicodeError) as exc:
        print(f"STOP=INPUT_UNREADABLE_OR_INVALID\nDETAIL={exc}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
