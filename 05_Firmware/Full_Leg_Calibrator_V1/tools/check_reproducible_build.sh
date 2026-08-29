#!/usr/bin/env bash
#
# MATDOG Full Leg Calibrator V1 — deterministic build proof.
#
# Builds the same stage TWICE from a genuinely cold cache and asserts the two
# application binaries are byte-identical.
#
# Why this exists: the firmware used to print BUILD_DATE=__DATE__ and
# BUILD_TIME=__TIME__, which baked the compile instant into the image. Two clean
# builds of one commit produced different SHA256s, so the binary hash could not
# be used as a pre-flash integrity check, and a hash published for a commit was
# unattainable minutes later. Build metadata is now derived from the commit.
#
#   ./check_reproducible_build.sh          # H0, the flashed-by-default stage
#   ./check_reproducible_build.sh 6        # any stage
#
# --clean is mandatory: arduino-cli caches per-sketch objects, and a cached
# object would hide exactly the nondeterminism this check exists to catch.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(dirname "$HERE")/matdog_full_leg_calibrator_v1"
REPO_ROOT="$(git -C "$HERE" rev-parse --show-toplevel)"
ARDUINO_CLI="${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}"
FQBN="${FQBN:-esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi}"

STAGE="${1:-0}"
BOOTSTRAP=0
if [[ "$STAGE" -ge 3 ]]; then BOOTSTRAP=1; fi

GIT_SHA="$(git -C "$REPO_ROOT" rev-parse HEAD)"
if [[ -n "$(git -C "$REPO_ROOT" status --porcelain=v1)" ]]; then
  WORKTREE_DIRTY=1
else
  WORKTREE_DIRTY=0
fi
GIT_COMMIT_EPOCH="$(git -C "$REPO_ROOT" show -s --format=%ct HEAD)"

# Reproducible builds. GCC honours SOURCE_DATE_EPOCH for __DATE__/__TIME__, so
# exporting the COMMIT time makes every translation unit — including the Arduino
# ESP32 core, which embeds its own "Compile Date" string — expand those macros
# identically for a given commit. Without this, two clean builds of one commit
# differ and the binary SHA256 cannot serve as a pre-flash integrity check.
export SOURCE_DATE_EPOCH="${GIT_COMMIT_EPOCH}"

EXTRA="-DFLC_AUTHORIZED_STAGE=${STAGE} -DFLC_H3_BOOTSTRAP_APPROVED=${BOOTSTRAP} -DFLC_BUILD_GIT_SHA_TOKEN=${GIT_SHA} -DFLC_BUILD_WORKTREE_DIRTY=${WORKTREE_DIRTY} -DFLC_BUILD_SOURCE_EPOCH=${GIT_COMMIT_EPOCH}"
BIN="${SKETCH_DIR}/build/esp32.esp32.esp32s3/matdog_full_leg_calibrator_v1.ino.bin"

echo "=============================================="
echo " Deterministic build check — stage H${STAGE}"
echo "=============================================="
echo "git sha   : ${GIT_SHA}"
echo "src epoch : ${GIT_COMMIT_EPOCH}"
echo "dirty     : ${WORKTREE_DIRTY}"
echo

build_once() {
  rm -rf "${SKETCH_DIR}/build"
  "$ARDUINO_CLI" compile --clean \
    --fqbn "$FQBN" --warnings default --export-binaries \
    --build-property "compiler.cpp.extra_flags=${EXTRA}" \
    "$SKETCH_DIR" > /dev/null
  sha256sum "$BIN" | cut -d' ' -f1
}

echo "build 1 of 2 (cold cache) ..."
FIRST="$(build_once)"
echo "  $FIRST"
echo "build 2 of 2 (cold cache) ..."
SECOND="$(build_once)"
echo "  $SECOND"
echo

if [[ "$FIRST" == "$SECOND" ]]; then
  echo "REPRODUCIBLE_BUILD=PASS sha256=${FIRST}"
  exit 0
fi
echo "REPRODUCIBLE_BUILD=FAIL"
echo "  build 1: $FIRST"
echo "  build 2: $SECOND"
echo "The application binary still depends on something other than the source."
exit 1
