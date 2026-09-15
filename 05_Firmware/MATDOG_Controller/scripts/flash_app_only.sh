#!/usr/bin/env bash
# Writes MATDOG Controller V0.1 to the ESP32-S3's currently active
# application (OTA) partition — and ONLY that partition. Session 2
# hardening: Session 1's scripts/upload.sh (arduino-cli upload) is a
# NORMAL Arduino upload, which also writes the bootloader, partition
# table and boot_app0/otadata every time. This script exists because that
# is not application-only, and a genuinely limited write needed its own
# implementation rather than a corrected comment on the old one.
#
# This script calls esptool write-flash exactly ONCE, with exactly one
# <offset> <file> pair: the application binary at the offset
# scripts/verify_application_partition.py derives (read-only) from the
# device's own partition table + otadata. It is therefore not possible for
# this script, as written, to address the bootloader (0x0), partition
# table (0x8000) or boot_app0/otadata (0xe000) — see also
# scripts/static_audit.py::check_app_only_script_never_targets_other_partitions,
# which fails the build if this file is ever edited to reference those
# artifact filenames again.
#
# scripts/upload.sh remains in the repository, with its documentation
# corrected, as the (still occasionally legitimate — e.g. bring-up on a
# replacement board) full-image path. It is NOT what this script runs and
# is NOT authorized by Session 2 for routine use.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$SKETCH_DIR/../.." && pwd)"
BUILD_DIR="$SKETCH_DIR/build/esp32.esp32.esp32s3"

ARDUINO="${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}"
ESPTOOL="${ESPTOOL_BIN:-$HOME/.arduino15/packages/esp32/tools/esptool_py/5.3.1/esptool}"
FQBN='esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi'
PORT="${MATDOG_ESP32_PORT:-/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00}"
EXPECTED_MAC="${MATDOG_ESP32_MAC:-14:c1:9f:22:75:94}"

BACKUP="${MATDOG_FLASH_BACKUP:-$HOME/MATDOG/backups/esp32/matdog_esp32s3_fullflash_2026-09-10.bin}"
EXPECTED_BACKUP_SIZE=16777216
EXPECTED_BACKUP_SHA256="5cbba0b9c5500d0c95247b9b7e7173a29f934b8b13f6800cc9f583374d67fd32"

APPLICATION_BINARY="$BUILD_DIR/MATDOG_Controller.ino.bin"

refuse() {
  echo "REFUSE: $1" >&2
  exit 1
}

echo "== MATDOG Controller — application-only flash =="
echo

# --- Gate: working tree clean, know the exact source commit ---------------
if [ -n "$(git -C "$REPO_ROOT" status --porcelain)" ]; then
  refuse "working tree is not clean — commit or stash before an application-only flash. \
This is what guarantees the binary we are about to write corresponds to a real, \
inspectable commit rather than an untracked mix of edits."
fi
SOURCE_COMMIT="$(git -C "$REPO_ROOT" rev-parse HEAD)"
SOURCE_COMMIT_SHORT="$(git -C "$REPO_ROOT" rev-parse --short=12 HEAD)"

# --- Gate: application binary present and matches this exact commit -------
[ -f "$APPLICATION_BINARY" ] || refuse "application binary not found: $APPLICATION_BINARY \
(run scripts/build.sh from this clean commit first)"


# Captured into a variable rather than piped straight into `grep -q`: under
# `pipefail`, grep -q's early exit on first match sends strings SIGPIPE,
# which pipefail then reports as a pipeline failure even though grep did
# match — this avoids that.
BINARY_STRINGS="$(strings "$APPLICATION_BINARY")"
if ! grep -q "$SOURCE_COMMIT_SHORT" <<< "$BINARY_STRINGS"; then
  refuse "application binary does not appear to embed build id '$SOURCE_COMMIT_SHORT' \
(current HEAD). It was likely built from a different commit or before the tree was \
cleaned — rebuild with scripts/build.sh from this exact clean commit."
fi

APPLICATION_SHA256="$(sha256sum "$APPLICATION_BINARY" | cut -d' ' -f1)"
APPLICATION_SIZE="$(stat -c%s "$APPLICATION_BINARY")"

# --- Gate: backup exists, correct size and hash ----------------------------
[ -f "$BACKUP" ] || refuse "full-flash backup not found: $BACKUP"
BACKUP_SIZE="$(stat -c%s "$BACKUP")"
[ "$BACKUP_SIZE" -eq "$EXPECTED_BACKUP_SIZE" ] || \
  refuse "backup size $BACKUP_SIZE != expected $EXPECTED_BACKUP_SIZE"
BACKUP_SHA256="$(sha256sum "$BACKUP" | cut -d' ' -f1)"
[ "$BACKUP_SHA256" = "$EXPECTED_BACKUP_SHA256" ] || \
  refuse "backup sha256 $BACKUP_SHA256 != expected $EXPECTED_BACKUP_SHA256"

# --- Gate: static safety audit PASS ----------------------------------------
if ! python3 "$SCRIPT_DIR/static_audit.py" "$SKETCH_DIR"; then
  refuse "static safety audit did not PASS"
fi

# --- Gate: serial device present and identity matches ----------------------
[ -e "$PORT" ] || refuse "expected serial device not present: $PORT"
DEVICE_MAC="$("$ESPTOOL" --chip esp32s3 --port "$PORT" read-mac 2>/dev/null \
  | grep -oE '([0-9a-f]{2}:){5}[0-9a-f]{2}' | head -1 || true)"
[ -n "$DEVICE_MAC" ] || refuse "could not read device MAC over $PORT"
[ "$DEVICE_MAC" = "$EXPECTED_MAC" ] || \
  refuse "device MAC $DEVICE_MAC != expected $EXPECTED_MAC — refusing to flash an \
unexpected board"

# --- Gate: verified application partition offset/size, read from the ------
# --- device's own partition table + otadata (never assumed) ---------------
PARTITION_INFO="$(python3 "$SCRIPT_DIR/verify_application_partition.py" \
  --port "$PORT" --esptool "$ESPTOOL")"
APPLICATION_OFFSET="$(echo "$PARTITION_INFO" | grep '^APPLICATION_OFFSET=' | cut -d= -f2)"
MAX_PARTITION_SIZE="$(echo "$PARTITION_INFO" | grep '^APPLICATION_PARTITION_SIZE=' | cut -d= -f2)"
ACTIVE_PARTITION_LABEL="$(echo "$PARTITION_INFO" | grep '^ACTIVE_PARTITION_LABEL=' | cut -d= -f2)"

[ -n "$APPLICATION_OFFSET" ] || refuse "could not determine a verified application offset"
[ -n "$MAX_PARTITION_SIZE" ] || refuse "could not determine the target partition size"
[ "$APPLICATION_SIZE" -le "$MAX_PARTITION_SIZE" ] || \
  refuse "application binary ($APPLICATION_SIZE bytes) exceeds target partition \
($MAX_PARTITION_SIZE bytes)"

# --- Print every required field before writing anything --------------------
echo "DEVICE                = $PORT (MAC $DEVICE_MAC)"
echo "APPLICATION_BINARY    = $APPLICATION_BINARY"
echo "APPLICATION_SHA256    = $APPLICATION_SHA256"
echo "APPLICATION_OFFSET    = $APPLICATION_OFFSET (partition '$ACTIVE_PARTITION_LABEL')"
echo "APPLICATION_SIZE      = $APPLICATION_SIZE"
echo "MAX_PARTITION_SIZE    = $MAX_PARTITION_SIZE"
echo "FQBN                  = $FQBN"
echo "SOURCE_COMMIT         = $SOURCE_COMMIT"
echo

# --- The one and only write: exactly one <offset> <file> pair -------------
echo "Writing application partition only (no bootloader/partition-table/boot_app0/NVS)..."
"$ESPTOOL" --chip esp32s3 --port "$PORT" write-flash \
  "$APPLICATION_OFFSET" "$APPLICATION_BINARY"

# --- Post-write read-back verification (read-only) --------------------------
echo
echo "Verifying written application partition by reading it back..."
READBACK="$(mktemp)"
trap 'rm -f "$READBACK"' EXIT
"$ESPTOOL" --chip esp32s3 --port "$PORT" read-flash \
  "$APPLICATION_OFFSET" "$APPLICATION_SIZE" "$READBACK" >/dev/null
READBACK_SHA256="$(sha256sum "$READBACK" | cut -d' ' -f1)"
if [ "$READBACK_SHA256" != "$APPLICATION_SHA256" ]; then
  refuse "post-write read-back sha256 $READBACK_SHA256 != written $APPLICATION_SHA256 — \
flash may be corrupted, investigate before trusting this device"
fi

echo "APPLICATION_ONLY_FLASH = PASS"
echo "READBACK_SHA256        = $READBACK_SHA256 (matches)"
