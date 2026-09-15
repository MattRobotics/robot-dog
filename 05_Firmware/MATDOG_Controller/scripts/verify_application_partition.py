#!/usr/bin/env python3
"""Read-only verification of the ESP32-S3's currently active application
partition, used to derive a trustworthy write offset for
scripts/flash_app_only.sh.

This exists because MATDOG_Controller's partition scheme
(app3M_fat9M_16MB) is a dual-OTA-slot layout (app0/ota_0 and app1/ota_1),
not a single fixed application region. Writing "the application" correctly
means writing whichever slot the device's own otadata currently selects.

All the actual selection logic — struct layout, CRC, validity, and the
seq -> slot mapping — lives in ota_partition_logic.py, verified against the
real installed ESP-IDF v5.5.5 source and empirically checked against the
known-good boot_app0.bin seed file (see that module's docstring and
scripts/tests/test_ota_partition_logic.py). This script is only the
device-I/O wrapper: read two flash regions, hand the bytes to the pure
logic, print the verified result or refuse.

--sdkconfig is REQUIRED (Session 2.2 Finding B): this script reads the
REAL sdkconfig produced by the build that made the binary being flashed
(not an assumption) to determine whether
CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE / CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK
are set, and passes that into ota_partition_logic's fail-closed checks. A
future FQBN/sdkconfig change that enables either cannot silently pass
through this tool.

Never writes anything. Exits non-zero with a clear message on any
ambiguity (OtaAmbiguous) — callers must not guess past that.
"""
import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

from ota_partition_logic import (
    OtaAmbiguous,
    parse_sdkconfig_ota_flags,
    resolve_application_partition,
)

PARTITION_TABLE_OFFSET = 0x8000
PARTITION_TABLE_SIZE = 0x1000
OTADATA_OFFSET = 0xE000
OTADATA_SIZE = 0x2000


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


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--chip", default="esp32s3")
    ap.add_argument("--esptool", required=True)
    ap.add_argument("--sdkconfig", required=True,
                     help="path to the sdkconfig produced by the build being flashed")
    args = ap.parse_args()

    sdkconfig_path = Path(args.sdkconfig)
    if not sdkconfig_path.is_file():
        raise SystemExit(f"REFUSE: sdkconfig not found: {sdkconfig_path}")
    rollback_enabled, anti_rollback_enabled = parse_sdkconfig_ota_flags(
        sdkconfig_path.read_text(encoding="utf-8"))
    print(f"CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE={'y' if rollback_enabled else 'n'}")
    print(f"CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK={'y' if anti_rollback_enabled else 'n'}")

    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        part_bin = tmp / "partitions.bin"
        ota_bin = tmp / "otadata.bin"

        esptool_read(args.esptool, args.chip, args.port,
                     PARTITION_TABLE_OFFSET, PARTITION_TABLE_SIZE, part_bin)
        esptool_read(args.esptool, args.chip, args.port,
                     OTADATA_OFFSET, OTADATA_SIZE, ota_bin)

        try:
            resolved = resolve_application_partition(
                part_bin.read_bytes(), ota_bin.read_bytes(),
                rollback_enabled=rollback_enabled,
                anti_rollback_enabled=anti_rollback_enabled,
            )
        except OtaAmbiguous as exc:
            raise SystemExit(f"REFUSE: {exc}")

    print(f"OTADATA_SECTOR_SEQS={resolved.all_seqs}")
    print(f"OTADATA_ACTIVE_SECTOR={resolved.active_sector_index}")
    print(f"OTADATA_ACTIVE_SEQ={resolved.active_seq}")
    print(f"ACTIVE_OTA_SLOT_INDEX={resolved.slot_index}")
    print(f"ACTIVE_PARTITION_LABEL={resolved.label}")
    print(f"APPLICATION_OFFSET=0x{resolved.offset:06x}")
    print(f"APPLICATION_PARTITION_SIZE={resolved.size}")


if __name__ == "__main__":
    main()
