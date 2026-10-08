#!/usr/bin/env bash
# REFUSING STUB (P2.3). There is no full-image upload any more.
#
# This script used to run `arduino-cli upload`, which rewrites the bootloader,
# the partition table, otadata AND the application every time. With the MATDOG
# V1 flash layout (partitions.csv, LAYOUT_ID MATDOG_16M_2x5M_NVS_V1) that is
# exactly what must never happen during ordinary maintenance: a full rewrite
# can put the old partition table back, or move/erase the persistent MATDOG
# NVS partition.
#
#   - ordinary updates:  scripts/flash_app_only.sh (application slot only, every
#     gate in that script applies)
#   - migration from the legacy app3M_fat9M_16MB layout: a SEPARATE procedure
#     that does not exist yet. It needs its own explicit operator authorization
#     for the session.
#
# CONTRACT a future migration procedure must satisfy (data, not code - the
# machine-readable form is MIGRATION_* in scripts/matdog_layout.py, tested by
# scripts/tests/test_matdog_layout.py):
#   preconditions, ALL required:
#     - explicit operator authorization for that session
#     - device identity verified
#     - a fresh full 16 MiB backup with an authorized SHA-256
#     - the installed table is the known legacy table
#     - the target table SHA-256 equals the Manifest V2 value and the pinned one
#     - the application is a Manifest V2 build for LAYOUT_ID MATDOG_16M_2x5M_NVS_V1
#     - the default NVS content is backed up and the backup verified
#     - the loss of the ffat data is accepted (its range moves)
#     - the boot slot after migration is app0
#   allowed writes: partition table (0x8000..0x9000), otadata (0xE000..0x10000),
#     app0 (0x10000..0x510000). Nothing else.
#   protected, never written or erased: bootloader (0x0..0x8000), default NVS
#     (0x9000..0xE000), MATDOG NVS (0xFE0000..0xFF0000).
#   forbidden operations: a whole-chip erase, a full-image restore, a write of a
#     merged image.
#
# This stub performs no hardware operation of any kind.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FQBN='esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=custom,DebugLevel=none,PSRAM=opi'

echo "REFUSE: scripts/upload.sh performs no upload." >&2
echo "  Full-image uploads are disabled for layout $(python3 "$SCRIPT_DIR/matdog_layout.py" contract | grep '^LAYOUT_ID=' | cut -d= -f2)." >&2
echo "  Ordinary update : scripts/flash_app_only.sh" >&2
echo "  Layout migration: separate, explicitly authorized procedure (not implemented)." >&2
echo "  fqbn            : $FQBN" >&2
exit 1
