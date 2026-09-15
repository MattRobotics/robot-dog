#!/usr/bin/env bash
# Uploads MATDOG Controller V0.1 to the ESP32-S3 over USB. This is a NORMAL
# Arduino application upload (arduino-cli upload) — it does not erase flash,
# does not touch the bootloader/partition table, and does not write NVS. It
# only replaces the application partition with the current build.
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
