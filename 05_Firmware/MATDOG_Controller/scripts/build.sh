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

echo "== MATDOG Controller build =="
echo "sketch   : $SKETCH_DIR"
echo "fqbn     : $FQBN"
echo "build_id : $BUILD_ID"
echo

"$ARDUINO" compile \
  --fqbn "$FQBN" \
  --build-property "compiler.cpp.extra_flags=-DMATDOG_BUILD_ID=\"${BUILD_ID}\"" \
  --warnings all \
  --export-binaries \
  "$SKETCH_DIR" \
  "$@"
