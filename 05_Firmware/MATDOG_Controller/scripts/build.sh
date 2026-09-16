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

SOURCE_COMMIT="nogit"
SOURCE_STATE="NO_GIT"
if git -C "$REPO_ROOT" rev-parse HEAD >/dev/null 2>&1; then
  SOURCE_COMMIT="$(git -C "$REPO_ROOT" rev-parse HEAD)"
  if [ -n "$(git -C "$REPO_ROOT" status --porcelain)" ]; then
    SOURCE_STATE="DIRTY"
  else
    SOURCE_STATE="CLEAN"
  fi
fi

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
PROFILE_ID="USB_ONLY"
case "${MATDOG_PROFILE:-}" in
  "")
    ;;
  USB_ONLY)
    PROFILE_FLAG=" -DMATDOG_ACTIVE_HARDWARE_PROFILE=::matdog::config::HardwareProfile::USB_ONLY"
    PROFILE_NAME="USB_ONLY (explicit)"
    PROFILE_ID="USB_ONLY"
    ;;
  ROBOT_POWERED)
    PROFILE_FLAG=" -DMATDOG_ACTIVE_HARDWARE_PROFILE=::matdog::config::HardwareProfile::ROBOT_POWERED"
    PROFILE_NAME="ROBOT_POWERED (OVERRIDE — requires G3 authorization)"
    PROFILE_ID="ROBOT_POWERED"
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

# --- Build manifest (G2 pre-G3 hardening, review Finding 1) ----------------
# Both profiles produce the same artifact pathname from the same commit, so
# the embedded build id alone cannot prove WHICH profile a binary came from.
# This binds profile + exact bytes + source state to the build, and
# scripts/flash_app_only.sh refuses to write anything it cannot verify
# against this file. The manifest lives inside the gitignored build output
# directory, adjacent to the binary — it is a build artifact, never
# committed.
BUILD_DIR="$SKETCH_DIR/build/esp32.esp32.esp32s3"
APPLICATION_BINARY="$BUILD_DIR/MATDOG_Controller.ino.bin"
MANIFEST="$BUILD_DIR/matdog_build_manifest.txt"

# Stale-manifest safety: if the compile produced no binary, make sure a
# previous build's manifest cannot be left behind to be verified against.
if [ ! -f "$APPLICATION_BINARY" ]; then
  rm -f "$MANIFEST"
  echo "ERROR: expected application binary not found after compile: $APPLICATION_BINARY" >&2
  exit 1
fi

python3 "$SCRIPT_DIR/build_manifest.py" write \
  --output "$MANIFEST" \
  --binary "$APPLICATION_BINARY" \
  --source-commit "$SOURCE_COMMIT" \
  --build-id "$BUILD_ID" \
  --source-state "$SOURCE_STATE" \
  --profile "$PROFILE_ID" \
  --fqbn "$FQBN"
