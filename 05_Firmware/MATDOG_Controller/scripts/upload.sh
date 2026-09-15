#!/usr/bin/env bash
# FULL Arduino application upload (arduino-cli upload / esptool stub) for
# the ESP32-S3. This writes FOUR regions every time it runs: bootloader
# (0x0), partition table (0x8000), boot_app0/otadata (0xe000) AND the
# application partition (0x10000) — arduino-cli's default UploadMode
# always rewrites all four, not just the application.
#
# SESSION 1 CORRECTION (2026-09-15 Session 2 hardening): this script was
# previously documented here as a "normal application upload" that "does
# not touch the bootloader/partition table" and "only replaces the
# application partition". That was wrong — Session 1's own upload log shows
# writes to all four regions, and VALIDATION.md's Session 1 record has been
# corrected to say so plainly. It happened to be harmless because the
# bootloader/partition-table/boot_app0 bytes written were byte-identical to
# what was already on the device (verified by a read-only flash audit at
# the start of Session 2 — see VALIDATION.md), not because the write never
# happened.
#
# For a write that is actually limited to the application partition, use
# scripts/flash_app_only.sh instead — that is the script Session 2 uses.
# This script remains for the case that legitimately needs a full image
# (e.g. bring-up on a replacement/blank board, or after a deliberate
# partition-scheme change) and requires the same operator authorization any
# bootloader/partition-table write does; it is not part of the routine
# Session 2 flashing path.
#
# Gates that must already be true before running this (see VALIDATION.md):
#   - full 16 MiB backup verified (size + SHA256)
#   - compile PASS
#   - static safety audit PASS
#   - existing BNO085 viewer test suite PASS
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

ARDUINO="${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}"
FQBN='esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi'
PORT="${MATDOG_ESP32_PORT:-/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00}"

echo "== MATDOG Controller upload =="
echo "sketch : $SKETCH_DIR"
echo "port   : $PORT"
echo "fqbn   : $FQBN"
echo

"$ARDUINO" upload \
  --fqbn "$FQBN" \
  --port "$PORT" \
  --input-dir "$SKETCH_DIR/build/esp32.esp32.esp32s3" \
  "$SKETCH_DIR"
