#!/usr/bin/env bash
#
# MATDOG Full Leg Calibrator V1 — build a stage-authorized firmware image.
#
# The ONLY supported way to produce a validation build. There is exactly one
# switch (flc_stage_config.h) and this script sets it explicitly, so a stage
# is never raised by editing sources in several places and forgetting one.
#
#   ./build_stage.sh                  # H0, bootstrap denied (default, safe)
#   ./build_stage.sh 3 --bootstrap    # H3 characterization, bootstrap approved
#   ./build_stage.sh 4                # H4 joint calibration
#   ./build_stage.sh 6 --bootstrap --upload
#
# Raising the stage does NOT bypass identity, fresh census, measured direction
# or any hard servo guard. Those are enforced independently of this flag.
#
# The bootstrap envelope additionally requires the operator to confirm it in the
# live session with `@APPROVE_BOOTSTRAP CONFIRM`. The build flag alone is not
# sufficient to move anything.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$(dirname "$HERE")/matdog_full_leg_calibrator_v1"
REPO_ROOT="$(git -C "$HERE" rev-parse --show-toplevel)"

ARDUINO_CLI="${ARDUINO_CLI:-$HOME/.local/bin/arduino-cli}"
PORT="${PORT:-/dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_14:C1:9F:22:75:94-if00}"
FQBN="${FQBN:-esp32:esp32:esp32s3:USBMode=hwcdc,CDCOnBoot=cdc,UploadMode=default,CPUFreq=240,FlashMode=qio,FlashSize=16M,PartitionScheme=app3M_fat9M_16MB,DebugLevel=none,PSRAM=opi}"

STAGE="${1:-0}"
shift || true

BOOTSTRAP=0
UPLOAD=0
for arg in "$@"; do
  case "$arg" in
    --bootstrap) BOOTSTRAP=1 ;;
    --upload) UPLOAD=1 ;;
    *) echo "unknown option: $arg" >&2; exit 2 ;;
  esac
done

if ! [[ "$STAGE" =~ ^[0-7]$ ]]; then
  echo "stage must be 0..7, got: $STAGE" >&2
  exit 2
fi

if [[ "$BOOTSTRAP" == "1" && "$STAGE" -lt 3 ]]; then
  echo "refusing: --bootstrap is meaningless below H3 (nothing may move)" >&2
  exit 2
fi

STAGE_NAMES=(H0_ESP32_ONLY H1_CENSUS_READONLY H2_MANUAL_Q0 H3_JOINT_CHARACTERIZE \
             H4_JOINT_CALIBRATE H5_LEG H6_FOUR_LEGS H7_FREEZE)

GIT_SHA="$(git -C "$REPO_ROOT" rev-parse HEAD)"
if [[ -n "$(git -C "$REPO_ROOT" status --porcelain=v1)" ]]; then
  WORKTREE_DIRTY=1
else
  WORKTREE_DIRTY=0
fi

EXTRA="-DFLC_AUTHORIZED_STAGE=${STAGE} -DFLC_H3_BOOTSTRAP_APPROVED=${BOOTSTRAP} -DFLC_BUILD_GIT_SHA_TOKEN=${GIT_SHA} -DFLC_BUILD_WORKTREE_DIRTY=${WORKTREE_DIRTY}"

echo "=============================================="
echo " MATDOG Full Leg Calibrator V1 — stage build"
echo "=============================================="
echo "stage      : H${STAGE} (${STAGE_NAMES[$STAGE]})"
echo "bootstrap  : $([[ "$BOOTSTRAP" == "1" ]] && echo "BUILD-APPROVED (session confirmation still required)" || echo "DENIED")"
echo "flags      : ${EXTRA}"
echo "git sha    : ${GIT_SHA}"
echo "dirty      : ${WORKTREE_DIRTY}"
echo "fqbn       : ${FQBN}"
echo

if [[ "$STAGE" -ge 3 ]]; then
  echo "*** This image can command servo motion at H${STAGE}."
  echo "*** Only flash it with the operator present and the robot supported."
  echo
fi

"$ARDUINO_CLI" compile \
  --fqbn "$FQBN" \
  --warnings default \
  --export-binaries \
  --build-property "compiler.cpp.extra_flags=${EXTRA}" \
  "$SKETCH_DIR"

BIN="${SKETCH_DIR}/build/esp32.esp32.esp32s3/matdog_full_leg_calibrator_v1.ino.bin"
if [[ -f "$BIN" ]]; then
  echo
  echo "binary sha256: $(sha256sum "$BIN" | cut -d' ' -f1)"
fi

if [[ "$UPLOAD" == "1" ]]; then
  echo
  echo "uploading to ${PORT}"
  "$ARDUINO_CLI" upload -p "$PORT" --fqbn "$FQBN" "$SKETCH_DIR"
  echo "verify with: @STATUS  (check AUTHORIZED_HARDWARE_STAGE and H3_BOOTSTRAP_*)"
fi
