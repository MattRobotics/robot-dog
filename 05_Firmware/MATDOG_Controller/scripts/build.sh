#!/usr/bin/env bash
# Compiles MATDOG Controller V0.1 with the pinned FQBN. Does not touch
# hardware — no upload happens here (see scripts/upload.sh for that, and
# note it still requires the backup/static-audit gates to have passed).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
REPO_ROOT="$(cd "$SKETCH_DIR/../.." && pwd)"

ARDUINO="${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}"
FQBN='esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi'

GIT_SHA="nogit"
DIRTY_SUFFIX=""
if git -C "$REPO_ROOT" rev-parse --short=12 HEAD >/dev/null 2>&1; then
  GIT_SHA="$(git -C "$REPO_ROOT" rev-parse --short=12 HEAD)"
  if ! git -C "$REPO_ROOT" diff --quiet 2>/dev/null || ! git -C "$REPO_ROOT" diff --cached --quiet 2>/dev/null; then
    DIRTY_SUFFIX="-dirty"
  fi
fi
BUILD_ID="${GIT_SHA}${DIRTY_SUFFIX}"

# Hardware profile override. The SOURCE default is USB_ONLY and must stay
# USB_ONLY (scripts/static_audit.py enforces that — it is the G3
# authorization gate). A powered-validation session overrides it here, for
# one build, without editing the repository default:
#
#   MATDOG_PROFILE=ROBOT_POWERED scripts/build.sh
#
# Deliberately explicit and loud: the chosen profile is echoed below and
# ends up in the boot banner, so a ROBOT_POWERED image can never be
# produced or flashed silently.
PROFILE_FLAG=""
PROFILE_NAME="USB_ONLY (source default)"
case "${MATDOG_PROFILE:-}" in
  "")
    ;;
  USB_ONLY)
    PROFILE_FLAG=" -DMATDOG_ACTIVE_HARDWARE_PROFILE=::matdog::config::HardwareProfile::USB_ONLY"
    PROFILE_NAME="USB_ONLY (explicit)"
    ;;
  ROBOT_POWERED)
    PROFILE_FLAG=" -DMATDOG_ACTIVE_HARDWARE_PROFILE=::matdog::config::HardwareProfile::ROBOT_POWERED"
    PROFILE_NAME="ROBOT_POWERED (OVERRIDE — requires G3 authorization)"
    ;;
  *)
    echo "ERROR: MATDOG_PROFILE='${MATDOG_PROFILE}' is not a known profile" >&2
    echo "       valid values: USB_ONLY, ROBOT_POWERED" >&2
    exit 1
    ;;
esac

echo "== MATDOG Controller build =="
echo "sketch   : $SKETCH_DIR"
echo "fqbn     : $FQBN"
echo "build_id : $BUILD_ID"
echo "profile  : $PROFILE_NAME"
echo

"$ARDUINO" compile \
  --fqbn "$FQBN" \
  --build-property "compiler.cpp.extra_flags=-DMATDOG_BUILD_ID=\"${BUILD_ID}\"${PROFILE_FLAG}" \
  --warnings all \
  --export-binaries \
  "$SKETCH_DIR" \
  "$@"
