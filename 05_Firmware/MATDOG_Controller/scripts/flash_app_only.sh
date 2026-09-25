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
# SESSION 2.1: verify_application_partition.py's slot-selection logic was
# corrected to match the real installed bootloader algorithm exactly
# (esp_ota_select_entry_t CRC + ota_state validation, not just a raw
# ota_seq comparison) and now fails closed — REFUSE, not a guess — on any
# ambiguous/invalid/unverifiable OTA state. See scripts/ota_partition_logic.py
# and scripts/tests/test_ota_partition_logic.py.
#
# scripts/upload.sh remains in the repository, with its documentation
# corrected, as the (still occasionally legitimate — e.g. bring-up on a
# replacement board) full-image path. It is NOT what this script runs and
# is NOT authorized by Session 2 for routine use.
# G2 PRE-G3 HARDENING (review Finding 1): build.sh can produce either a
# USB_ONLY or a ROBOT_POWERED image from the same commit, at the same path.
# The commit/build-id gate below cannot tell them apart — the build id is
# identical for both. Every write is therefore now additionally gated on a
# build manifest (scripts/build_manifest.py) that binds the hardware profile
# and the binary's exact size/sha256 to the source commit, and on the
# operator naming the profile they intend to flash via MATDOG_FLASH_PROFILE.
# Fail-closed: a missing, stale, unparseable or mismatched manifest REFUSES.
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
BUILD_MANIFEST="$BUILD_DIR/matdog_build_manifest.txt"

# Backwards-safe default: an omitted MATDOG_FLASH_PROFILE means USB_ONLY,
# preserving the existing USB workflow with no new ceremony. Because the
# verifier requires manifest profile == requested profile, a ROBOT_POWERED
# manifest is then REFUSED unless the operator explicitly asks for it:
#
#   MATDOG_FLASH_PROFILE=ROBOT_POWERED scripts/flash_app_only.sh
#
# There is no path by which a powered image is written implicitly.
REQUESTED_FLASH_PROFILE="${MATDOG_FLASH_PROFILE:-USB_ONLY}"

# Same backwards-safe shape for the OTA-ingest authorization axis (I7
# hardening, 2026-09-25). An omitted MATDOG_FLASH_OTA_INGEST means 0: a
# manifest built with the firmware-ingest writer compiled in is then
# REFUSED unless the operator explicitly asks for it —
#
#   MATDOG_FLASH_OTA_INGEST=1 scripts/flash_app_only.sh
#
# There is no path by which an ingest-enabled image is written implicitly.
REQUESTED_FLASH_OTA_INGEST="${MATDOG_FLASH_OTA_INGEST:-0}"

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

# --- Gate: build manifest proves WHAT this binary actually is -------------
# Verifies, fail-closed: manifest exists and parses; its schema version is
# known; its source commit == HEAD; its FQBN == this script's own pinned
# $FQBN (the right source built with the wrong partition scheme / flash
# size / PSRAM mode is still the wrong artifact); the tree was clean at
# build time and is clean now; the binary exists and its size AND sha256
# equal the recorded ones; the manifest profile is recognized; and it
# equals the profile the operator asked to flash. Any failure REFUSES — see
# scripts/build_manifest.py and scripts/tests/test_build_manifest.py.
TREE_STATE="CLEAN"  # proven by the working-tree gate above
MANIFEST_INFO="$(python3 "$SCRIPT_DIR/build_manifest.py" verify \
  --manifest "$BUILD_MANIFEST" \
  --binary "$APPLICATION_BINARY" \
  --head "$SOURCE_COMMIT" \
  --expected-fqbn "$FQBN" \
  --tree-state "$TREE_STATE" \
  --requested-profile "$REQUESTED_FLASH_PROFILE" \
  --requested-ota-ingest "$REQUESTED_FLASH_OTA_INGEST")" || \
  refuse "build manifest verification failed (see REFUSED=... above) — the binary in \
$BUILD_DIR cannot be proven to be a $REQUESTED_FLASH_PROFILE build of $SOURCE_COMMIT with \
OTA_INGEST_ENABLED=$REQUESTED_FLASH_OTA_INGEST. Rebuild with the intended settings: \
MATDOG_PROFILE=$REQUESTED_FLASH_PROFILE MATDOG_OTA_INGEST_VALIDATION=$REQUESTED_FLASH_OTA_INGEST \
scripts/build.sh"

VERIFIED_HARDWARE_PROFILE="$(echo "$MANIFEST_INFO" | grep '^VERIFIED_HARDWARE_PROFILE=' | cut -d= -f2-)"
[ -n "$VERIFIED_HARDWARE_PROFILE" ] || \
  refuse "manifest verification produced no VERIFIED_HARDWARE_PROFILE"
VERIFIED_OTA_INGEST_ENABLED="$(echo "$MANIFEST_INFO" | grep '^VERIFIED_OTA_INGEST_ENABLED=' | cut -d= -f2-)"
[ -n "$VERIFIED_OTA_INGEST_ENABLED" ] || \
  refuse "manifest verification produced no VERIFIED_OTA_INGEST_ENABLED"
VERIFIED_FQBN="$(echo "$MANIFEST_INFO" | grep '^VERIFIED_FQBN=' | cut -d= -f2-)"
[ -n "$VERIFIED_FQBN" ] || refuse "manifest verification produced no VERIFIED_FQBN"

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
# --- device's own partition table + otadata (never assumed), AND the ------
# --- real rollback/anti-rollback config of the build being flashed --------
SDKCONFIG="$BUILD_DIR/sdkconfig"
[ -f "$SDKCONFIG" ] || refuse "sdkconfig not found: $SDKCONFIG (run scripts/build.sh first)"

PARTITION_INFO="$(python3 "$SCRIPT_DIR/verify_application_partition.py" \
  --port "$PORT" --esptool "$ESPTOOL" --sdkconfig "$SDKCONFIG")"
ROLLBACK_ENABLE="$(echo "$PARTITION_INFO" | grep '^CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=' | cut -d= -f2)"
ANTI_ROLLBACK="$(echo "$PARTITION_INFO" | grep '^CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK=' | cut -d= -f2)"
APPLICATION_OFFSET="$(echo "$PARTITION_INFO" | grep '^APPLICATION_OFFSET=' | cut -d= -f2)"
MAX_PARTITION_SIZE="$(echo "$PARTITION_INFO" | grep '^APPLICATION_PARTITION_SIZE=' | cut -d= -f2)"
ACTIVE_PARTITION_LABEL="$(echo "$PARTITION_INFO" | grep '^ACTIVE_PARTITION_LABEL=' | cut -d= -f2)"

[ -n "$APPLICATION_OFFSET" ] || refuse "could not determine a verified application offset"
[ -n "$MAX_PARTITION_SIZE" ] || refuse "could not determine the target partition size"
[ "$APPLICATION_SIZE" -le "$MAX_PARTITION_SIZE" ] || \
  refuse "application binary ($APPLICATION_SIZE bytes) exceeds target partition \
($MAX_PARTITION_SIZE bytes)"

# --- Print every required field before writing anything --------------------
echo "SDKCONFIG_ROLLBACK    = CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=$ROLLBACK_ENABLE"
echo "SDKCONFIG_ANTI_ROLLBACK = CONFIG_BOOTLOADER_APP_ANTI_ROLLBACK=$ANTI_ROLLBACK"
echo "DEVICE                = $PORT (MAC $DEVICE_MAC)"
echo "APPLICATION_BINARY    = $APPLICATION_BINARY"
echo "APPLICATION_SHA256    = $APPLICATION_SHA256"
echo "APPLICATION_OFFSET    = $APPLICATION_OFFSET (partition '$ACTIVE_PARTITION_LABEL')"
echo "APPLICATION_SIZE      = $APPLICATION_SIZE"
echo "MAX_PARTITION_SIZE    = $MAX_PARTITION_SIZE"
echo "FQBN                  = $FQBN (verified against the build manifest)"
echo "SOURCE_COMMIT         = $SOURCE_COMMIT"
echo "BUILD_MANIFEST        = $BUILD_MANIFEST"
echo
echo "############################################################"
echo "#  HARDWARE PROFILE   = $VERIFIED_HARDWARE_PROFILE"
echo "#  OTA_INGEST_ENABLED = $VERIFIED_OTA_INGEST_ENABLED"
echo "#  (both verified against the build manifest, not assumed)"
echo "############################################################"
echo

# --- The one and only write: exactly one <offset> <file> pair -------------
echo "Writing application partition only (no bootloader/partition-table/boot_app0/NVS)..."
"$ESPTOOL" --chip esp32s3 --port "$PORT" write-flash \
  "$APPLICATION_OFFSET" "$APPLICATION_BINARY"

# --- Post-write verification ------------------------------------------------
# esptool's dedicated verify-flash (device-side digest compare) rather than
# read-flash into a file: raw multi-packet read-flash proved unreliable
# immediately after a write+reset in this environment (intermittent
# "Packet content transfer stopped" around the same byte offset on repeat
# attempts, root-caused to a USB-Serial-JTAG re-enumeration timing quirk,
# not a flash content problem — write-flash's own post-write hash check
# had already passed). verify-flash reads the device again independently
# and reports a clear pass/fail without staging a local copy.
echo
echo "Verifying written application partition (esptool verify-flash)..."
sleep 2  # let the device finish its post-write reset/re-enumeration
if ! "$ESPTOOL" --chip esp32s3 --port "$PORT" verify-flash \
    "$APPLICATION_OFFSET" "$APPLICATION_BINARY"; then
  refuse "post-write verify-flash failed — flash may be corrupted, investigate before \
trusting this device"
fi

echo "APPLICATION_ONLY_FLASH = PASS"
echo "FLASHED_HARDWARE_PROFILE = $VERIFIED_HARDWARE_PROFILE"
echo "FLASHED_OTA_INGEST_ENABLED = $VERIFIED_OTA_INGEST_ENABLED"
